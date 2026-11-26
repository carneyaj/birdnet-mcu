#pragma once

#include <cstddef>
#include <cstdint>

namespace birdnet_rice_codec {

constexpr uint32_t kInputSampleRateHz = 48000;
constexpr uint32_t kOutputSampleRateHz = 24000;
constexpr uint16_t kBlockSamples = 256;
constexpr uint32_t kFileHeaderBytes = 32;
constexpr uint32_t kBlockHeaderBytes = 8;

using WriteFn = bool (*)(void* context, const uint8_t* data, size_t size);
using SeekFn = bool (*)(void* context, uint32_t offset);

struct EncodeStats {
  uint32_t input_samples = 0;
  uint32_t output_samples = 0;
  uint32_t block_count = 0;
  uint32_t payload_bytes = 0;
  uint32_t file_bytes = 0;
  uint16_t quant_step = 0;
  uint16_t predictor_counts[3] = {};
};

// Encodes chronological_samples from a potentially wrapped 48 kHz PCM ring.
// BRC1 stores 24 kHz quantized PCM with a fixed predictor and Rice code per
// 256-sample block.
bool EncodeBrcFrom48kRing(const int16_t* pcm_ring,
                          size_t ring_size,
                          size_t chronological_start,
                          size_t chronological_samples,
                          uint16_t quant_step,
                          WriteFn write,
                          void* write_context,
                          EncodeStats* stats,
                          SeekFn seek = nullptr);

}  // namespace birdnet_rice_codec
