#pragma once

#include <cstddef>
#include <cstdint>

#include "config.h"

struct InferenceResult {
  uint64_t timestamp_ms = 0;
  float inference_ms = 0.0f;
  float init_ms = 0.0f;
  float frontend_ms = 0.0f;
  float invoke_ms = 0.0f;
  float decode_ms = 0.0f;
  int pred_class = -1;
  float pred_score = 0.0f;
  bool detected = false;
  // Evaluated over every class, before top-K and overlap filtering.
  bool human_vocal_detected = false;
  int top_indices[cfg::kTopK]{};
  float top_scores[cfg::kTopK]{};
  int log_detection_count = 0;
  bool log_detection_overflow = false;
  int log_detection_indices[cfg::kSdLogMaxDetectionsPerWindow]{};
  float log_detection_scores[cfg::kSdLogMaxDetectionsPerWindow]{};
};

class TFLMInferencePipeline {
 public:
  TFLMInferencePipeline();
  bool Run(const int16_t* pcm_window, InferenceResult* out);
  bool RunFromRing(const int16_t* pcm_ring, size_t ring_size, size_t write_idx, InferenceResult* out);
  void PrepareForSleep();
  void SetCaptureWindow(uint64_t end_samples) { capture_end_samples_ = end_samples; }
  void ResetStreamingCache();
  void SetRecordingWindow(uint64_t end_samples, const uint64_t* previous_ends) {
    recording_end_samples_ = end_samples;
    recording_previous_ends_ = previous_ends;
  }

 private:
  bool Init();
  bool FillInputTensorFromBirdNetFrontend(const int16_t* pcm_window);
  bool FillInputTensorFromBirdNetFrontendRing(const int16_t* pcm_ring, size_t ring_size, size_t write_idx);
  bool ReadOutput(InferenceResult* out);

  bool initialized_ = false;
  bool init_ok_ = false;
  uint64_t recording_end_samples_ = 0;
  const uint64_t* recording_previous_ends_ = nullptr;
  uint64_t capture_end_samples_ = 0;
};
