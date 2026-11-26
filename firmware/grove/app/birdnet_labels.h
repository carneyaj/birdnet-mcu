#pragma once

namespace birdnet_labels {

constexpr int kCount = 1018;

extern const char* const kLabels[kCount];
const char* LabelForIndex(int index);
inline bool IsHumanVocal(int index) { return index >= 0 && index == 460; }
inline bool IsBird(int index) { return index >= 0 && index < kCount && index != 31  && index != 209  && index != 210  && index != 324  && index != 364  && index != 365  && index != 393  && index != 434  && index != 459  && index != 460  && index != 461  && index != 625  && index != 636  && index != 761  && index != 823  && index != 859  && index != 921  && index != 922; }
inline bool CanTriggerDetectionAudio(int index) { return index >= 0 && index < kCount && index != 364  && index != 365  && index != 393  && index != 434  && index != 459  && index != 460  && index != 461  && index != 625  && index != 761  && index != 859; }

}  // namespace birdnet_labels
