#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "config.h"

class AudioRingBuffer {
 public:
  AudioRingBuffer();

  // Pushes mono PCM samples into the ring, overwriting oldest samples when full.
  void Push(const int16_t* samples, size_t n);

  bool CopyLatestWindow(int16_t* out) const;
  bool CopyWindowEndingAt(int16_t* out, uint64_t end_sample_exclusive) const;

  size_t Size() const { return size_; }
  bool IsReady() const { return size_ >= cfg::kWindowSamples; }
  uint64_t TotalSamples() const { return total_samples_; }

 private:
  std::array<int16_t, cfg::kRingBufferSamples> data_{};
  size_t write_idx_ = 0;
  size_t size_ = 0;
  uint64_t total_samples_ = 0;
};
