#pragma once
#include <cstdint>

constexpr uint16_t kDetectionThresholdQ1e4 = 2500;
constexpr float kDetectionThreshold = kDetectionThresholdQ1e4 / 10000.f;

void init_recorder();
// On acceptance the writer owns the detected PCM and returns a spare buffer.
const char* queue_recording(int16_t*& pcm, uint32_t window, int64_t ready_us,
                            int species, float probability, const char* label);
