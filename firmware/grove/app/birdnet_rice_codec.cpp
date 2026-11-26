#include "birdnet_rice_codec.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

namespace birdnet_rice_codec {
namespace {

constexpr uint8_t kVersion = 1;
constexpr int kFirRadius = 31;
constexpr int16_t kFirCenterQ15 = 16384;
constexpr int16_t kFirPairQ15[16] = {
    -2, 7, -16, 32, -56, 92, -143, 214,
    -311, 443, -623, 877, -1261, 1917, -3373, 10395,
};
constexpr int kMaxRiceK = 15;
constexpr size_t kBitWriterBytes = 128;
constexpr uint32_t kMaxInputSamples = kInputSampleRateHz * 3u;
constexpr uint16_t kMaxBlockCount = static_cast<uint16_t>(
    ((kMaxInputSamples / 2u) + kBlockSamples - 1u) / kBlockSamples);

void PutLe16(uint8_t* out, uint16_t value) {
  out[0] = static_cast<uint8_t>(value);
  out[1] = static_cast<uint8_t>(value >> 8);
}

void PutLe32(uint8_t* out, uint32_t value) {
  out[0] = static_cast<uint8_t>(value);
  out[1] = static_cast<uint8_t>(value >> 8);
  out[2] = static_cast<uint8_t>(value >> 16);
  out[3] = static_cast<uint8_t>(value >> 24);
}

int16_t SaturateInt16(int32_t value) {
  if (value > std::numeric_limits<int16_t>::max()) {
    return std::numeric_limits<int16_t>::max();
  }
  if (value < std::numeric_limits<int16_t>::min()) {
    return std::numeric_limits<int16_t>::min();
  }
  return static_cast<int16_t>(value);
}

class RingReader {
 public:
  RingReader(const int16_t* ring, size_t ring_size, size_t start, size_t samples)
      : ring_(ring), ring_size_(ring_size), start_(start), samples_(samples) {}

  int16_t ReadClamped(int64_t chronological_index) const {
    if (chronological_index < 0) {
      chronological_index = 0;
    } else if (chronological_index >= static_cast<int64_t>(samples_)) {
      chronological_index = static_cast<int64_t>(samples_ - 1);
    }
    size_t ring_index = start_ + static_cast<size_t>(chronological_index);
    if (ring_index >= ring_size_) {
      ring_index -= ring_size_;
    }
    return ring_[ring_index];
  }

 private:
  const int16_t* ring_;
  size_t ring_size_;
  size_t start_;
  size_t samples_;
};

int16_t DecimateSample(const RingReader& input, uint32_t output_index) {
  const int64_t center = static_cast<int64_t>(output_index) * 2;
  // Coefficient L1 bound: 1,831,993,344, including full-scale PCM. Rounding
  // adds only 16,384, so every partial sum and its negation fit in int32.
  int32_t accumulator =
      static_cast<int32_t>(kFirCenterQ15) * input.ReadClamped(center);
  for (int pair = 0; pair < 16; ++pair) {
    const int offset = kFirRadius - pair * 2;
    const int32_t symmetric_samples =
        static_cast<int32_t>(input.ReadClamped(center - offset)) +
        static_cast<int32_t>(input.ReadClamped(center + offset));
    accumulator += static_cast<int32_t>(kFirPairQ15[pair]) * symmetric_samples;
  }
  const int32_t rounded =
      accumulator >= 0
          ? (accumulator + (1 << 14)) >> 15
          : -(((-accumulator) + (1 << 14)) >> 15);
  return SaturateInt16(static_cast<int32_t>(rounded));
}

int16_t Quantize(int16_t sample, uint16_t quant_step) {
  const int32_t value = sample;
  const int32_t magnitude = value < 0 ? -value : value;
  const int32_t units =
      (magnitude + static_cast<int32_t>(quant_step / 2u)) / quant_step;
  return static_cast<int16_t>(value < 0 ? -units : units);
}

void FillBlock(const RingReader& input,
               uint32_t output_start,
               uint16_t sample_count,
               uint16_t quant_step,
               int16_t* block) {
  for (uint16_t i = 0; i < sample_count; ++i) {
    block[i] = Quantize(DecimateSample(input, output_start + i), quant_step);
  }
}

int32_t ResidualAt(const int16_t* block, uint16_t index, uint8_t order) {
  const int32_t sample = block[index];
  if (order == 0) {
    return sample;
  }
  if (order == 1) {
    return sample - static_cast<int32_t>(block[index - 1]);
  }
  return sample - 2 * static_cast<int32_t>(block[index - 1]) +
         static_cast<int32_t>(block[index - 2]);
}

uint32_t ZigZag(int32_t value) {
  return value >= 0 ? static_cast<uint32_t>(value) * 2u
                    : static_cast<uint32_t>(-value) * 2u - 1u;
}

struct BlockMode {
  uint8_t order;
  uint8_t rice_k;
  uint32_t rice_bits;
};

BlockMode SelectMode(const int16_t* block, uint16_t sample_count) {
  uint32_t best_total_bits = std::numeric_limits<uint32_t>::max();
  BlockMode best{};
  const uint8_t max_order =
      static_cast<uint8_t>(std::min<uint16_t>(2, sample_count - 1));
  for (uint8_t order = 0; order <= max_order; ++order) {
    uint32_t rice_bits[kMaxRiceK + 1] = {};
    const uint16_t residual_count = sample_count - order;
    for (int k = 0; k <= kMaxRiceK; ++k) {
      rice_bits[k] = static_cast<uint32_t>(residual_count) * (k + 1u);
    }
    for (uint16_t i = order; i < sample_count; ++i) {
      uint32_t shifted = ZigZag(ResidualAt(block, i, order));
      for (int k = 0; shifted != 0 && k <= kMaxRiceK; ++k) {
        rice_bits[k] += shifted;
        shifted >>= 1;
      }
    }
    for (uint8_t k = 0; k <= kMaxRiceK; ++k) {
      const uint32_t total_bits =
          static_cast<uint32_t>(order) * 16u + rice_bits[k];
      if (total_bits < best_total_bits) {
        best_total_bits = total_bits;
        best = {order, k, rice_bits[k]};
      }
    }
  }
  return best;
}

uint32_t CountRiceBits(const int16_t* block,
                       uint16_t sample_count,
                       uint8_t order,
                       uint8_t rice_k) {
  uint32_t bits =
      static_cast<uint32_t>(sample_count - order) * (rice_k + 1u);
  for (uint16_t i = order; i < sample_count; ++i) {
    bits += ZigZag(ResidualAt(block, i, order)) >> rice_k;
  }
  return bits;
}

class BitWriter {
 public:
  BitWriter(WriteFn write, void* context)
      : write_(write), context_(context) {}

  bool WriteBit(uint32_t bit) {
    current_ |= static_cast<uint8_t>((bit & 1u) << used_);
    ++used_;
    ++bit_count_;
    if (used_ == 8) {
      return PushByte(current_);
    }
    return true;
  }

  bool WriteBits(uint32_t value, uint8_t count) {
    for (uint8_t bit = 0; bit < count; ++bit) {
      if (!WriteBit(value >> bit)) {
        return false;
      }
    }
    return true;
  }

  bool Finish() {
    if (used_ != 0 && !PushByte(current_)) {
      return false;
    }
    return Flush();
  }

  uint32_t bit_count() const { return bit_count_; }

 private:
  bool PushByte(uint8_t value) {
    buffer_[buffer_size_++] = value;
    current_ = 0;
    used_ = 0;
    return buffer_size_ != sizeof(buffer_) || Flush();
  }

  bool Flush() {
    if (buffer_size_ == 0) {
      return true;
    }
    if (!write_(context_, buffer_, buffer_size_)) {
      return false;
    }
    buffer_size_ = 0;
    return true;
  }

  WriteFn write_;
  void* context_;
  uint8_t buffer_[kBitWriterBytes] = {};
  size_t buffer_size_ = 0;
  uint8_t current_ = 0;
  uint8_t used_ = 0;
  uint32_t bit_count_ = 0;
};

bool WriteRicePayload(const int16_t* block,
                      uint16_t sample_count,
                      const BlockMode& mode,
                      WriteFn write,
                      void* write_context) {
  BitWriter writer(write, write_context);
  const uint32_t remainder_mask =
      mode.rice_k == 0 ? 0u : (1u << mode.rice_k) - 1u;
  for (uint16_t i = mode.order; i < sample_count; ++i) {
    const uint32_t mapped = ZigZag(ResidualAt(block, i, mode.order));
    const uint32_t quotient = mapped >> mode.rice_k;
    for (uint32_t zero = 0; zero < quotient; ++zero) {
      if (!writer.WriteBit(0)) {
        return false;
      }
    }
    if (!writer.WriteBit(1) ||
        (mode.rice_k != 0 &&
         !writer.WriteBits(mapped & remainder_mask, mode.rice_k))) {
      return false;
    }
  }
  return writer.Finish() && writer.bit_count() == mode.rice_bits;
}

void BuildFileHeader(uint8_t* header,
                     uint16_t quant_step,
                     uint32_t sample_count,
                     uint16_t block_count,
                     uint32_t payload_bytes) {
  std::memset(header, 0, kFileHeaderBytes);
  std::memcpy(header, "BRC1", 4);
  header[4] = kVersion;
  header[5] = 1;
  PutLe16(header + 6, quant_step);
  PutLe32(header + 8, kOutputSampleRateHz);
  PutLe32(header + 12, sample_count);
  PutLe16(header + 16, kBlockSamples);
  PutLe16(header + 18, block_count);
  PutLe32(header + 24, payload_bytes);
}

}  // namespace

bool EncodeBrcFrom48kRing(const int16_t* pcm_ring,
                          size_t ring_size,
                          size_t chronological_start,
                          size_t chronological_samples,
                          uint16_t quant_step,
                          WriteFn write,
                          void* write_context,
                          EncodeStats* stats,
                          SeekFn seek) {
  if (!pcm_ring || ring_size == 0 || chronological_samples < 2 ||
      chronological_samples > ring_size || chronological_start >= ring_size ||
      chronological_samples > kMaxInputSamples ||
      (chronological_samples & 1u) != 0 || quant_step == 0 || !write) {
    return false;
  }

  const uint32_t output_samples =
      static_cast<uint32_t>(chronological_samples / 2);
  const uint32_t block_count_u32 =
      (output_samples + kBlockSamples - 1) / kBlockSamples;
  if (block_count_u32 > std::numeric_limits<uint16_t>::max()) {
    return false;
  }
  const uint16_t block_count = static_cast<uint16_t>(block_count_u32);
  const RingReader input(
      pcm_ring, ring_size, chronological_start, chronological_samples);
  int16_t block[kBlockSamples];
  uint8_t block_modes[kMaxBlockCount];

  uint32_t payload_bytes = 0;
  uint16_t predictor_counts[3] = {};
  for (uint16_t block_index = 0; !seek && block_index < block_count; ++block_index) {
    const uint32_t output_start =
        static_cast<uint32_t>(block_index) * kBlockSamples;
    const uint16_t sample_count = static_cast<uint16_t>(
        std::min<uint32_t>(kBlockSamples, output_samples - output_start));
    FillBlock(input, output_start, sample_count, quant_step, block);
    const BlockMode mode = SelectMode(block, sample_count);
    payload_bytes += kBlockHeaderBytes + mode.order * 2u +
                     (mode.rice_bits + 7u) / 8u;
    block_modes[block_index] =
        static_cast<uint8_t>((mode.order << 4) | mode.rice_k);
    ++predictor_counts[mode.order];
  }

  uint8_t file_header[kFileHeaderBytes];
  BuildFileHeader(
      file_header, quant_step, output_samples, block_count, payload_bytes);
  if (!write(write_context, file_header, sizeof(file_header))) {
    return false;
  }

  for (uint16_t block_index = 0; block_index < block_count; ++block_index) {
    const uint32_t output_start =
        static_cast<uint32_t>(block_index) * kBlockSamples;
    const uint16_t sample_count = static_cast<uint16_t>(
        std::min<uint32_t>(kBlockSamples, output_samples - output_start));
    FillBlock(input, output_start, sample_count, quant_step, block);
    BlockMode mode{};
    if (seek) {
      mode = SelectMode(block, sample_count);
      payload_bytes += kBlockHeaderBytes + mode.order * 2u + (mode.rice_bits + 7u) / 8u;
      ++predictor_counts[mode.order];
    } else {
      const uint8_t packed_mode = block_modes[block_index];
      mode.order = packed_mode >> 4;
      mode.rice_k = packed_mode & 0x0fu;
      mode.rice_bits = CountRiceBits(block, sample_count, mode.order, mode.rice_k);
    }
    const uint16_t rice_bytes =
        static_cast<uint16_t>((mode.rice_bits + 7u) / 8u);
    const uint8_t valid_last =
        static_cast<uint8_t>(mode.rice_bits % 8u == 0 ? 8 : mode.rice_bits % 8u);

    uint8_t block_header[kBlockHeaderBytes] = {};
    PutLe16(block_header, sample_count);
    block_header[2] = mode.order;
    block_header[3] = mode.rice_k;
    block_header[4] = valid_last;
    PutLe16(block_header + 6, rice_bytes);
    if (!write(write_context, block_header, sizeof(block_header))) {
      return false;
    }
    uint8_t warmup[4];
    for (uint8_t i = 0; i < mode.order; ++i) {
      PutLe16(warmup + i * 2u, static_cast<uint16_t>(block[i]));
    }
    if (mode.order != 0 &&
        !write(write_context, warmup, mode.order * 2u)) {
      return false;
    }
    if (!WriteRicePayload(
            block, sample_count, mode, write, write_context)) {
      return false;
    }
  }

  if (seek) {
    // An interrupted one-pass file retains a zero payload length and must
    // not be added to AUDIOMAP. Final files are identical to the two-pass path.
    BuildFileHeader(file_header, quant_step, output_samples, block_count, payload_bytes);
    if (!seek(write_context, 0) || !write(write_context, file_header, sizeof(file_header)) ||
        !seek(write_context, kFileHeaderBytes + payload_bytes)) {
      return false;
    }
  }

  if (stats) {
    stats->input_samples = static_cast<uint32_t>(chronological_samples);
    stats->output_samples = output_samples;
    stats->block_count = block_count;
    stats->payload_bytes = payload_bytes;
    stats->file_bytes = kFileHeaderBytes + payload_bytes;
    stats->quant_step = quant_step;
    for (int order = 0; order < 3; ++order) {
      stats->predictor_counts[order] = predictor_counts[order];
    }
  }
  return true;
}

}  // namespace birdnet_rice_codec
