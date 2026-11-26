#pragma once
#include <cstddef>
#include <cstdint>

namespace birdnet_flash_labels {
inline uint32_t Read32(const uint8_t* p) {
  return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
}

// Called once per cold boot. Never trust lengths or offsets from unverified flash.
inline bool Validate(const uint8_t* p, size_t bytes, uint32_t count, uint32_t expected) {
  if (!p || !count || count > 65535 || bytes < 16 + 4 * (size_t(count) + 1)) return false;
  if (Read32(p) != 0x314c4e42u || Read32(p + 4) != 1 ||
      Read32(p + 8) != count || Read32(p + 12) != bytes) return false;
  uint32_t hash = 2166136261u;
  for (size_t i = 0; i < bytes; ++i) hash = (hash ^ p[i]) * 16777619u;
  if (hash != expected) return false;
  uint32_t start = Read32(p + 16);
  if (start != 16 + 4 * (count + 1)) return false;
  for (uint32_t i = 0; i < count; ++i) {
    const uint32_t end = Read32(p + 20 + 4 * i);
    if (end <= start + 1 || end > bytes || p[end - 1] != 0) return false;
    for (uint32_t j = start; j + 1 < end; ++j) if (!p[j]) return false;
    start = end;
  }
  return start == bytes;
}

// Base is supplied only after validation and enabling XIP. Do not retain the
// returned pointer across sleep; all consumers format/copy it synchronously.
inline const char* Lookup(const uint8_t* p, size_t bytes, int count, int index) {
  if (!p || index < 0 || index >= count || bytes < 16 + 4 * (size_t(count) + 1)) return "unknown";
  const uint32_t start = Read32(p + 16 + 4 * index);
  const uint32_t end = Read32(p + 20 + 4 * index);
  if (start < 16 + 4 * (size_t(count) + 1) || end <= start || end > bytes || p[end - 1]) return "unknown";
  return reinterpret_cast<const char*>(p + start);
}
}  // namespace birdnet_flash_labels
