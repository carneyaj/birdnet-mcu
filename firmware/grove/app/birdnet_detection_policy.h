#pragma once

#include <cstdint>
#include <cstring>

namespace birdnet_detection {

inline bool IsHumanVocal(const char* label) {
  return label && std::strcmp(label, "Human vocal_Human vocal") == 0;
}

template <typename Result>
bool CanSaveAmbient(bool inference_ok, const Result& prediction) {
  return inference_ok && !prediction.human_vocal_detected;
}

// Windows are half-open sample intervals. Only accepted recordings advance
// history; rejected overlaps must not indefinitely extend the exclusion.
inline bool CanRecord(uint64_t previous_end, uint64_t end, uint32_t window) {
  return window != 0 && end >= window &&
      (previous_end == 0 || (end >= previous_end && end - previous_end >= window));
}

// Decoder ranking for the OLED/UART remains unfiltered. Storage must instead
// use the best eligible class, including when the overall top class overlaps.
template <typename Result, typename CanTrigger>
Result ForRecording(const Result& prediction, CanTrigger can_trigger) {
  Result recording = prediction;
  recording.detected = false;
  recording.pred_class = -1;
  recording.pred_score = 0.0f;
  for (int i = 0; !recording.human_vocal_detected && i < recording.log_detection_count; ++i) {
    if (!can_trigger(recording.log_detection_indices[i])) continue;
    recording.detected = true;
    recording.pred_class = recording.log_detection_indices[i];
    recording.pred_score = recording.log_detection_scores[i];
    break;
  }
  return recording;
}

}  // namespace birdnet_detection
