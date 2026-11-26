#include "audio_ring_buffer.h"

AudioRingBuffer::AudioRingBuffer() = default;

void AudioRingBuffer::Push(const int16_t* samples, size_t n) {
  if (!samples) {
    return;
  }

  for (size_t i = 0; i < n; ++i) {
    data_[write_idx_] = samples[i];
    write_idx_ = (write_idx_ + 1) % data_.size();
    if (size_ < data_.size()) {
      ++size_;
    }
    ++total_samples_;
  }
}

bool AudioRingBuffer::CopyLatestWindow(int16_t* out) const {
  return CopyWindowEndingAt(out, total_samples_);
}

bool AudioRingBuffer::CopyWindowEndingAt(int16_t* out, uint64_t end_sample_exclusive) const {
  if (!IsReady()) {
    return false;
  }
  if (!out || end_sample_exclusive > total_samples_ || end_sample_exclusive < cfg::kWindowSamples) {
    return false;
  }

  const uint64_t earliest_sample = total_samples_ - size_;
  const uint64_t start_sample = end_sample_exclusive - cfg::kWindowSamples;
  if (start_sample < earliest_sample) {
    return false;
  }

  const size_t oldest_idx = (size_ == data_.size()) ? write_idx_ : 0;
  const uint64_t rel_start = start_sample - earliest_sample;
  for (size_t i = 0; i < static_cast<size_t>(cfg::kWindowSamples); ++i) {
    const size_t idx = (oldest_idx + static_cast<size_t>(rel_start) + i) % data_.size();
    out[i] = data_[idx];
  }
  return true;
}
