#pragma once

#include "birdnet_species_profile.h"

namespace birdnet_labels {

using birdnet_species::IsBird;
using birdnet_species::kCount;
const char* LabelForIndex(int index);
inline bool IsHumanVocal(int index) { return index == birdnet_species::kHumanVocal; }
inline bool CanTriggerDetectionAudio(int index) { return birdnet_species::CanTriggerAudio(index); }

}  // namespace birdnet_labels
