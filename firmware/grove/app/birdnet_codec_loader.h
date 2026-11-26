#pragma once
#include <cstddef>
#include <cstdint>
#include "config.h"

namespace birdnet_codec_loader {
constexpr uintptr_t kFlashAddress = 0x3A900000u;
constexpr uint32_t kMagic = 0x424e4f50u;
struct Header {
  uint32_t magic, version, destination, bytes, fnv1a, reserved[3];
};
static_assert(sizeof(Header) == 32);
inline bool ValidHeader(const Header& h, uintptr_t destination, size_t size) {
  return h.magic == kMagic && h.version == 1 && h.destination == destination &&
         h.bytes == size && size > 0 && size <= 0x40000 &&
         destination == 0x340A0000u && destination + size <= 0x340E0000u &&
         h.reserved[0] == 0 && h.reserved[1] == 0 && h.reserved[2] == 0;
}
inline bool Prepare() { return true; }
}  // namespace birdnet_codec_loader
