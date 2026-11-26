#include "birdnet_format.h"
#include "birdnet_opus_codec.h"

#if defined(BIRDNET_DETECTION_AUDIO_CODEC) && BIRDNET_DETECTION_AUDIO_CODEC == 2

#include <algorithm>
#include <cstdio>
#include <cstring>
#include "opus.h"

namespace birdnet_opus_codec {
#if BIRDNET_VALIDATE_CODEC == 2
int benchmark_complexity = kComplexity;
#endif
namespace {

constexpr size_t kFrameSamples = 960;  // 20 ms at 48 kHz.
constexpr size_t kMaxPacketBytes = 1275;
constexpr size_t kPageBytes = 4096;
constexpr size_t kPageSegments = 64;
constexpr size_t kPacketsPerPage = 10;

#if defined(__arm__) || defined(__thumb__)
// CPU-only state: do not occupy the NPU/DMA-addressable SRAM banks.
__attribute__((section(".scratch_dtcm.opus")))
#endif
alignas(16) uint8_t g_encoder_state[kEncoderStateBytes];

void Put16(uint8_t* dst, uint16_t value) {
  dst[0] = static_cast<uint8_t>(value);
  dst[1] = static_cast<uint8_t>(value >> 8);
}
void Put32(uint8_t* dst, uint32_t value) {
  for (unsigned i = 0; i < 4; ++i) dst[i] = static_cast<uint8_t>(value >> (8 * i));
}
void Put64(uint8_t* dst, uint64_t value) {
  Put32(dst, static_cast<uint32_t>(value));
  Put32(dst + 4, static_cast<uint32_t>(value >> 32));
}

uint32_t Crc(uint32_t crc, const uint8_t* bytes, size_t count) {
  static constexpr uint32_t table[16] = {
      0x00000000, 0x04c11db7, 0x09823b6e, 0x0d4326d9,
      0x130476dc, 0x17c56b6b, 0x1a864db2, 0x1e475005,
      0x2608edb8, 0x22c9f00f, 0x2f8ad6d6, 0x2b4bcb61,
      0x350c9b64, 0x31cd86d3, 0x3c8ea00a, 0x384fbdbd};
  for (size_t i = 0; i < count; ++i) {
    crc ^= static_cast<uint32_t>(bytes[i]) << 24;
    crc = (crc << 4) ^ table[crc >> 28];
    crc = (crc << 4) ^ table[crc >> 28];
  }
  return crc;
}

class Pages {
 public:
  Pages(WriteFn write, void* context, uint32_t serial, EncodeStats* stats)
      : write_(write), context_(context), serial_(serial), stats_(stats) {}

  bool Packet(const uint8_t* bytes, size_t count, uint64_t granule,
              uint8_t flags = 0, bool flush = false) {
    const size_t segments = count / 255 + 1;  // Include zero lacing at exact multiples.
    if (count > sizeof(payload_) || segments > sizeof(lacing_)) return false;
    if (used_ + count > sizeof(payload_) || segments_ + segments > sizeof(lacing_)) {
      if (!Flush(0)) return false;
    }
    size_t remaining = count;
    while (remaining >= 255) {
      lacing_[segments_++] = 255;
      remaining -= 255;
    }
    lacing_[segments_++] = static_cast<uint8_t>(remaining);
    std::memcpy(payload_ + used_, bytes, count);
    used_ += count;
    granule_ = granule;
    ++packets_;
    return (!flush && packets_ < kPacketsPerPage) || Flush(flags);
  }

 private:
  bool Flush(uint8_t flags) {
    uint8_t header[27 + kPageSegments] = {};
    std::memcpy(header, "OggS", 4);
    header[5] = flags;
    Put64(header + 6, granule_);
    Put32(header + 14, serial_);
    Put32(header + 18, sequence_++);
    header[26] = static_cast<uint8_t>(segments_);
    std::memcpy(header + 27, lacing_, segments_);
    const size_t header_bytes = 27 + segments_;
    Put32(header + 22, Crc(Crc(0, header, header_bytes), payload_, used_));
    if (!write_(context_, header, header_bytes) || !write_(context_, payload_, used_)) {
      stats_->error = OPUS_INTERNAL_ERROR;
      return false;
    }
    stats_->file_bytes += static_cast<uint32_t>(header_bytes + used_);
    used_ = segments_ = packets_ = 0;
    return true;
  }

  WriteFn write_;
  void* context_;
  uint32_t serial_;
  EncodeStats* stats_;
  uint8_t payload_[kPageBytes];
  uint8_t lacing_[kPageSegments];
  size_t used_ = 0, segments_ = 0, packets_ = 0;
  uint64_t granule_ = 0;
  uint32_t sequence_ = 0;
};

bool WriteTags(Pages* pages, const Metadata& metadata) {
  uint8_t tags[1536] = {};
  std::memcpy(tags, "OpusTags", 8);
  static constexpr char vendor[] = "BirdNET WE2 / libopus 1.6.1";
  Put32(tags + 8, sizeof(vendor) - 1);
  std::memcpy(tags + 12, vendor, sizeof(vendor) - 1);
  size_t used = 12 + sizeof(vendor) - 1;
  Put32(tags + used, metadata.clip ? 8 : 4);
  used += 4;
  char timestamp[64], serial[48], deployment[96];
  birdnet_format::Snprintf(timestamp, sizeof(timestamp), "CAPTURE_END_MS=%llu",
                static_cast<unsigned long long>(metadata.capture_end_ms));
  birdnet_format::Snprintf(serial, sizeof(serial), "STREAM_SERIAL=%lu",
                static_cast<unsigned long>(metadata.stream_serial));
  birdnet_format::Snprintf(deployment, sizeof(deployment), "DEPLOYMENT_ID=%s", metadata.deployment_id);
  const char* comments[] = {timestamp, serial, deployment, "TIMEBASE=CAPTURE_RELATIVE_MS"};
  for (const char* comment : comments) {
    const size_t count = std::strlen(comment);
    if (used + 4 + count > sizeof(tags)) return false;
    Put32(tags + used, static_cast<uint32_t>(count));
    std::memcpy(tags + used + 4, comment, count);
    used += 4 + count;
  }
  if (metadata.clip) {
    const auto& clip = *metadata.clip;
    if (clip.count > birdnet_clip::kMaxDetections) return false;
    char session[40], threshold[40], overflow[40], detections[512];
    birdnet_format::Snprintf(session, sizeof(session), "SESSION_ID=%lu", static_cast<unsigned long>(clip.session));
    birdnet_format::Snprintf(threshold, sizeof(threshold), "THRESHOLD_Q1E4=%u", clip.threshold_q1e4);
    birdnet_format::Snprintf(overflow, sizeof(overflow), "DETECTIONS_TRUNCATED=%u", clip.overflow ? 1 : 0);
    size_t length = birdnet_format::Snprintf(detections, sizeof(detections), "DETECTIONS_INDEX_Q1E4=");
    for (size_t i = 0; i < clip.count; ++i) {
      const int n = birdnet_format::Snprintf(detections + length, sizeof(detections) - length, "%s%lu:%u",
                                i ? ";" : "", static_cast<unsigned long>(clip.species[i]),
                                clip.probability[i]);
      if (n < 0 || static_cast<size_t>(n) >= sizeof(detections) - length) return false;
      length += static_cast<size_t>(n);
    }
    for (const char* comment : {session, threshold, overflow, detections}) {
      const size_t count = std::strlen(comment);
      if (used + 4 + count > sizeof(tags)) return false;
      Put32(tags + used, count);
      std::memcpy(tags + used + 4, comment, count);
      used += 4 + count;
    }
  }
  return pages->Packet(tags, used, 0, 0, true);
}

}  // namespace

bool EncodeOggFrom48kRing(const int16_t* ring, size_t ring_size, size_t start,
                         size_t samples, const Metadata& metadata,
                         WriteFn write, void* context, EncodeStats* stats) {
  EncodeStats local{};
  if (!stats) stats = &local;
  *stats = {};
  if (!ring || !write || !samples || samples > 3u * kSampleRateHz ||
      samples > ring_size || start >= ring_size || !metadata.deployment_id ||
      std::strlen(metadata.deployment_id) > 64) {
    stats->error = OPUS_BAD_ARG;
    return false;
  }
  const int state_bytes = opus_encoder_get_size(1);
  if (state_bytes <= 0 || static_cast<size_t>(state_bytes) > sizeof(g_encoder_state)) {
    stats->error = OPUS_ALLOC_FAIL;
    return false;
  }
  stats->encoder_state_bytes = static_cast<uint32_t>(state_bytes);
  auto* encoder = reinterpret_cast<OpusEncoder*>(g_encoder_state);
  stats->error = opus_encoder_init(encoder, kSampleRateHz, 1, OPUS_APPLICATION_AUDIO);
  if (stats->error != OPUS_OK) return false;
  // Explicitly reproduce the low-complexity offline candidate, not VOIP/DTX.
  const int controls[] = {
      opus_encoder_ctl(encoder, OPUS_SET_BITRATE(kBitrate)),
#if BIRDNET_VALIDATE_CODEC == 2
      opus_encoder_ctl(encoder, OPUS_SET_COMPLEXITY(benchmark_complexity)),
#else
      opus_encoder_ctl(encoder, OPUS_SET_COMPLEXITY(kComplexity)),
#endif
      opus_encoder_ctl(encoder, OPUS_SET_VBR(1)),
      opus_encoder_ctl(encoder, OPUS_SET_VBR_CONSTRAINT(1)),
      opus_encoder_ctl(encoder, OPUS_SET_DTX(0)),
      opus_encoder_ctl(encoder, OPUS_SET_INBAND_FEC(0)),
      opus_encoder_ctl(encoder, OPUS_SET_PACKET_LOSS_PERC(0)),
      opus_encoder_ctl(encoder, OPUS_SET_LSB_DEPTH(16)),
  };
  for (int error : controls) {
    if (error != OPUS_OK) { stats->error = error; return false; }
  }
  opus_int32 lookahead = 0;
  stats->error = opus_encoder_ctl(encoder, OPUS_GET_LOOKAHEAD(&lookahead));
  if (stats->error != OPUS_OK || lookahead < 0 || lookahead > 65535) {
    stats->error = OPUS_INTERNAL_ERROR;
    return false;
  }
  stats->pre_skip = static_cast<uint16_t>(lookahead);
  Pages pages(write, context, metadata.stream_serial, stats);
  uint8_t head[19] = {};
  std::memcpy(head, "OpusHead", 8);
  head[8] = 1;
  head[9] = 1;
  Put16(head + 10, stats->pre_skip);
  Put32(head + 12, kSampleRateHz);
  if (!pages.Packet(head, sizeof(head), 0, 2, true) || !WriteTags(&pages, metadata)) return false;

  int16_t frame[kFrameSamples];
  uint8_t packet[kMaxPacketBytes];
  const size_t total_samples = samples + static_cast<size_t>(lookahead);
  for (size_t offset = 0; offset < total_samples; offset += kFrameSamples) {
    for (size_t i = 0; i < kFrameSamples; ++i) {
      const size_t position = offset + i;
      const size_t index = position < ring_size - start ? start + position : position - (ring_size - start);
      frame[i] = position < samples ? ring[index] : 0;
    }
    const int count = opus_encode(encoder, frame, kFrameSamples, packet, sizeof(packet));
    if (count < 0) { stats->error = count; return false; }
    if (count == 0) { stats->error = OPUS_INTERNAL_ERROR; return false; }
    const bool last = offset + kFrameSamples >= total_samples;
    // Granules count decoded samples before pre-skip. EOS trims padding so
    // every conforming reader returns exactly the input sample count.
    const uint64_t granule = last ? total_samples : offset + kFrameSamples;
    if (!pages.Packet(packet, count, granule, last ? 4 : 0, last)) return false;
    ++stats->packets;
  }
  stats->samples = static_cast<uint32_t>(samples);
  return true;
}

}  // namespace birdnet_opus_codec
#endif
