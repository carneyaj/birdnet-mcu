#pragma once
#include <cstddef>
#include <cstdint>

// Stage embedding and classifier weights in PSRAM before constructing the interpreter.
bool prepare_packed_weights(const uint8_t* model, size_t bytes);
