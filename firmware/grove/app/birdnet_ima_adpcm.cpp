#include "birdnet_ima_adpcm.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

namespace birdnet_ima_adpcm {
namespace {

constexpr int kFirRadius = 31;
constexpr int16_t kFirCenterQ15 = 16384;
// Symmetric outer-to-inner coefficients for a 63-tap, 48-to-24 kHz
// half-band decimator. The zero-valued taps are omitted.
constexpr int16_t kFirPairQ15[16] = {
    -2, 7, -16, 32, -56, 92, -143, 214,
    -311, 443, -623, 877, -1261, 1917, -3373, 10395,
};

constexpr int16_t kStepTable[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28,
    31, 34, 37, 41, 45, 50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
    130, 143, 157, 173, 190, 209, 230, 253, 279, 307, 337, 371, 408,
    449, 494, 544, 598, 658, 724, 796, 876, 963, 1060, 1166, 1282,
    1411, 1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024, 3327, 3660,
    4026, 4428, 4871, 5358, 5894, 6484, 7132, 7845, 8630, 9493, 10442,
    11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623,
    27086, 29794, 32767,
};

constexpr int8_t kIndexAdjust[16] = {
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8,
};

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
  int64_t accumulator =
      static_cast<int64_t>(kFirCenterQ15) * input.ReadClamped(center);
  for (int pair = 0; pair < 16; ++pair) {
    const int offset = kFirRadius - (pair * 2);
    const int32_t symmetric_samples =
        static_cast<int32_t>(input.ReadClamped(center - offset)) +
        static_cast<int32_t>(input.ReadClamped(center + offset));
    accumulator += static_cast<int64_t>(kFirPairQ15[pair]) * symmetric_samples;
  }
  const int64_t quantized =
      accumulator >= 0
          ? (accumulator + (1ll << 14)) >> 15
          : -(((-accumulator) + (1ll << 14)) >> 15);
  return SaturateInt16(static_cast<int32_t>(quantized));
}

uint8_t EncodeNibble(int16_t sample, int16_t* predictor, int* step_index) {
  const int step = kStepTable[*step_index];
  int difference = static_cast<int>(sample) - static_cast<int>(*predictor);
  uint8_t code = 0;
  if (difference < 0) {
    code = 8;
    difference = -difference;
  }

  int reconstructed_difference = step >> 3;
  if (difference >= step) {
    code |= 4;
    difference -= step;
    reconstructed_difference += step;
  }
  if (difference >= (step >> 1)) {
    code |= 2;
    difference -= step >> 1;
    reconstructed_difference += step >> 1;
  }
  if (difference >= (step >> 2)) {
    code |= 1;
    reconstructed_difference += step >> 2;
  }

  int next_predictor = static_cast<int>(*predictor);
  next_predictor += (code & 8) ? -reconstructed_difference : reconstructed_difference;
  *predictor = SaturateInt16(next_predictor);
  *step_index = std::clamp(*step_index + static_cast<int>(kIndexAdjust[code]), 0, 88);
  return code;
}

void BuildWavHeader(uint8_t* header,
                    uint32_t output_samples,
                    uint32_t data_bytes) {
  std::memset(header, 0, kWavHeaderBytes);
  std::memcpy(header + 0, "RIFF", 4);
  PutLe32(header + 4, (kWavHeaderBytes - 8) + data_bytes);
  std::memcpy(header + 8, "WAVE", 4);
  std::memcpy(header + 12, "fmt ", 4);
  PutLe32(header + 16, 20);
  PutLe16(header + 20, 0x0011);  // WAVE_FORMAT_IMA_ADPCM
  PutLe16(header + 22, 1);
  PutLe32(header + 24, kOutputSampleRateHz);
  PutLe32(header + 28,
          (kOutputSampleRateHz * static_cast<uint32_t>(kBlockAlign)) /
              static_cast<uint32_t>(kSamplesPerBlock));
  PutLe16(header + 32, kBlockAlign);
  PutLe16(header + 34, 4);
  PutLe16(header + 36, 2);
  PutLe16(header + 38, kSamplesPerBlock);
  std::memcpy(header + 40, "fact", 4);
  PutLe32(header + 44, 4);
  PutLe32(header + 48, output_samples);
  std::memcpy(header + 52, "data", 4);
  PutLe32(header + 56, data_bytes);
}

}  // namespace

bool EncodeWavFrom48kRing(const int16_t* pcm_ring,
                          size_t ring_size,
                          size_t chronological_start,
                          size_t chronological_samples,
                          WriteFn write,
                          void* write_context,
                          EncodeStats* stats) {
  if (!pcm_ring || ring_size == 0 || chronological_samples < 2 ||
      chronological_samples > ring_size || chronological_start >= ring_size ||
      (chronological_samples & 1u) != 0 || !write) {
    return false;
  }

  const uint32_t output_samples =
      static_cast<uint32_t>(chronological_samples / 2);
  const uint32_t block_count =
      (output_samples + kSamplesPerBlock - 1) / kSamplesPerBlock;
  const uint32_t data_bytes = block_count * static_cast<uint32_t>(kBlockAlign);

  uint8_t header[kWavHeaderBytes];
  BuildWavHeader(header, output_samples, data_bytes);
  if (!write(write_context, header, sizeof(header))) {
    return false;
  }

  const RingReader input(
      pcm_ring, ring_size, chronological_start, chronological_samples);
  uint8_t block[kBlockAlign];
  uint32_t output_index = 0;
  int step_index = 0;
  int16_t last_sample = DecimateSample(input, 0);

  for (uint32_t block_index = 0; block_index < block_count; ++block_index) {
    std::memset(block, 0, sizeof(block));
    last_sample =
        output_index < output_samples ? DecimateSample(input, output_index) : last_sample;
    ++output_index;

    int16_t predictor = last_sample;
    PutLe16(block, static_cast<uint16_t>(predictor));
    block[2] = static_cast<uint8_t>(step_index);
    block[3] = 0;

    for (uint32_t nibble_index = 0; nibble_index < kSamplesPerBlock - 1;
         ++nibble_index) {
      if (output_index < output_samples) {
        last_sample = DecimateSample(input, output_index);
        ++output_index;
      }
      const uint8_t nibble =
          EncodeNibble(last_sample, &predictor, &step_index);
      const size_t byte_index = 4 + (nibble_index / 2);
      if ((nibble_index & 1u) == 0) {
        block[byte_index] = nibble;
      } else {
        block[byte_index] |= static_cast<uint8_t>(nibble << 4);
      }
    }

    if (!write(write_context, block, sizeof(block))) {
      return false;
    }
  }

  if (stats) {
    stats->input_samples = static_cast<uint32_t>(chronological_samples);
    stats->output_samples = output_samples;
    stats->block_count = block_count;
    stats->data_bytes = data_bytes;
    stats->file_bytes = kWavHeaderBytes + data_bytes;
  }
  return true;
}

}  // namespace birdnet_ima_adpcm
