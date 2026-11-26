#include "birdnet_labels.h"

#include "birdnet_flash_labels.h"
#include "birdnet_model_contract.h"

const uint8_t* BirdnetLabelFlashBase();

namespace birdnet_labels {

static_assert(kCount == birdnet_contract::kClasses, "Label/model count mismatch");

const char* LabelForIndex(int index) {
  return birdnet_flash_labels::Lookup(BirdnetLabelFlashBase(), birdnet_contract::kLabelBlobBytes, kCount, index);
}

}  // namespace birdnet_labels
