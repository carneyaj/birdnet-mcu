#pragma once

#include <cstddef>
#include <cstdint>
#include "birdnet_clip_metadata.h"

namespace birdnet_opus_codec {

constexpr uint32_t kSampleRateHz = 48000;
constexpr uint32_t kBitrate = 64000;
constexpr int kComplexity = 0;
#if BIRDNET_VALIDATE_CODEC == 2
extern int benchmark_complexity;
#endif
constexpr size_t kEncoderStateBytes = 32768;
using WriteFn = bool (*)(void* context, const uint8_t* data, size_t size);

struct Metadata {
  uint64_t capture_end_ms;
  uint32_t stream_serial;
  const char* deployment_id;
  const birdnet_clip::Metadata* clip = nullptr;
};

struct EncodeStats {
  uint32_t samples = 0;
  uint32_t file_bytes = 0;
  uint32_t packets = 0;
  uint32_t encoder_state_bytes = 0;
  uint16_t pre_skip = 0;
  int error = 0;
};

// Non-reentrant; owns one preallocated encoder, reset on every clip. Only the
// saved copy is encoded. No heap allocation, seeks, or full-clip output buffer.
bool EncodeOggFrom48kRing(const int16_t* ring, size_t ring_size, size_t start,
                         size_t samples, const Metadata& metadata,
                         WriteFn write, void* context, EncodeStats* stats);

}  // namespace birdnet_opus_codec
