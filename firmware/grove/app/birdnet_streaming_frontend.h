#pragma once

#include <cstdint>
#include <cstring>

#ifndef BIRDNET_STREAMING_CACHE_FRAMES
#define BIRDNET_STREAMING_CACHE_FRAMES 340
#endif

namespace birdnet_streaming {
constexpr int kFrames = 511;
constexpr int kMels = 96;
constexpr int kCachedFrames = BIRDNET_STREAMING_CACHE_FRAMES;
static_assert(kCachedFrames > 0 && kCachedFrames <= kFrames, "Invalid streaming cache size");
constexpr uint32_t kMagic = 0x42534601u;

inline uint16_t Pack(float value) {
  const __fp16 half = static_cast<__fp16>(value);
  uint16_t bits;
  std::memcpy(&bits, &half, sizeof(bits));
  return bits;
}
inline float Unpack(uint16_t bits) {
  __fp16 half;
  std::memcpy(&half, &bits, sizeof(bits));
  return static_cast<float>(half);
}

struct Cache {
  uint16_t values[2][kCachedFrames][kMels];
  int16_t anchors[2][kCachedFrames];
  uint64_t tags[2][kCachedFrames];

  uint32_t Hash() const {
    uint32_t hash = 2166136261u;
    for (int ch = 0; ch < 2; ++ch) {
      for (int slot = 0; slot < kCachedFrames; ++slot) {
        for (uint16_t value : values[ch][slot]) hash = (hash ^ value) * 16777619u;
        hash = (hash ^ static_cast<uint16_t>(anchors[ch][slot])) * 16777619u;
        hash = (hash ^ static_cast<uint32_t>(tags[ch][slot])) * 16777619u;
        hash = (hash ^ static_cast<uint32_t>(tags[ch][slot] >> 32)) * 16777619u;
      }
    }
    return hash;
  }
  bool Load(int ch, uint64_t frame, float* values_out, int16_t* anchor) const {
    const int slot = frame % kCachedFrames;
    if (tags[ch][slot] != frame + 1) return false;
    for (int mel = 0; mel < kMels; ++mel) values_out[mel] = Unpack(values[ch][slot][mel]);
    *anchor = anchors[ch][slot];
    return true;
  }
  void Store(int ch, uint64_t frame, const float* input, int16_t anchor) {
    const int slot = frame % kCachedFrames;
    for (int mel = 0; mel < kMels; ++mel) values[ch][slot][mel] = Pack(input[mel]);
    anchors[ch][slot] = anchor;
    tags[ch][slot] = frame + 1;
  }
};

// State is in retained DTCM; bulk cache is separately retained SRAM. Verify
// the bulk data before reuse so lost retention cannot silently affect inference.
struct State {
  uint32_t magic;
  uint32_t hash;
  uint64_t end;
};
inline bool Begin(Cache& cache, State& state, uint64_t end, uint32_t step) {
  const bool reusable = state.magic == kMagic && end > state.end &&
      end - state.end == step && state.hash == cache.Hash();
  state.magic = 0;  // An interrupted fill must never be reused.
  if (!reusable) std::memset(&cache, 0, sizeof(cache));
  return reusable;
}
inline void Seal(const Cache& cache, State& state, uint64_t end) {
  state.hash = cache.Hash();
  state.end = end;
  state.magic = kMagic;
}
}  // namespace birdnet_streaming
