#pragma once

#include <cstddef>
#include <cstdint>

namespace birdnet_ima_adpcm {

constexpr uint32_t kInputSampleRateHz = 48000;
constexpr uint32_t kOutputSampleRateHz = 24000;
constexpr uint16_t kBlockAlign = 256;
constexpr uint16_t kSamplesPerBlock = 505;
constexpr uint32_t kWavHeaderBytes = 60;

using WriteFn = bool (*)(void* context, const uint8_t* data, size_t size);

struct EncodeStats {
  uint32_t input_samples = 0;
  uint32_t output_samples = 0;
  uint32_t block_count = 0;
  uint32_t data_bytes = 0;
  uint32_t file_bytes = 0;
};

// Encodes chronological_samples from a potentially wrapped PCM ring. The
// output is a standard mono Microsoft IMA ADPCM WAV at 24 kHz.
bool EncodeWavFrom48kRing(const int16_t* pcm_ring,
                          size_t ring_size,
                          size_t chronological_start,
                          size_t chronological_samples,
                          WriteFn write,
                          void* write_context,
                          EncodeStats* stats);

}  // namespace birdnet_ima_adpcm
