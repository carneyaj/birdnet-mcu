#include "birdnet_format.h"
#pragma once
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace birdnet_storage {
constexpr uint32_t kClipsPerDirectory = 512;
constexpr size_t kAudioPathBytes = 96;
constexpr size_t kSpeciesDirectoryBytes = 58; // 48 readable characters + '-' + hash + NUL.

// BNL1 names are packed at byte offsets in memory-mapped flash. Do not give
// FatFs/libc that pointer: memcpy may make an unaligned word load depending
// on the current offset in the FAT sector. Stage and CSV-escape in SRAM.
template<typename Write>
bool WriteQuotedLabel(const char* label, Write write) {
  alignas(4) char buffer[64];
  size_t used = 0;
  const auto put = [&](char c) {
    buffer[used++] = c;
    if (used != sizeof(buffer)) return true;
    used = 0;
    return write(buffer, sizeof(buffer));
  };
  if (!put('"')) return false;
  if (label) {
    const volatile char* p = label;  // XIP reads must stay byte-wide.
    while (const char c = *p++) {
      if (!put(c) || (c == '"' && !put('"'))) return false;
    }
  }
  return put('"') && (!used || write(buffer, used));
}

inline bool NameChar(char c) {
  return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
      (c >= '0' && c <= '9') || c == '_' || c == '-';
}

// A full-label hash disambiguates sanitized/truncated names and remains stable
// when the classifier head is reordered. Only ASCII is emitted to the card.
inline bool SpeciesDirectory(const char* label, char* out, size_t size) {
  if (!out || size < kSpeciesDirectoryBytes) return false;
  if (!label) { std::strcpy(out, "AMBIENT"); return true; }
  uint32_t hash = 2166136261u;
  for (const unsigned char* p = reinterpret_cast<const unsigned char*>(label); *p; ++p)
    hash = (hash ^ *p) * 16777619u;
  const char* common = std::strchr(label, '_');
  common = common ? common + 1 : label;
  size_t used = 0;
  for (; *common && used < 48; ++common) {
    const char c = NameChar(*common) ? *common : '_';
    if (c == '_' && (!used || out[used - 1] == '_')) continue;
    out[used++] = c;
  }
  while (used && out[used - 1] == '_') --used;
  if (!used) out[used++] = 'X';
  return birdnet_format::Snprintf(out + used, size - used, "-%08lX",
                       static_cast<unsigned long>(hash)) == 9;
}

inline bool SpeciesComponent(const char* name, size_t length) {
  if (length == 7 && !std::memcmp(name, "AMBIENT", 7)) return true;
  if (length < 10 || length >= kSpeciesDirectoryBytes || name[length - 9] != '-') return false;
  for (size_t i = 0; i < length - 9; ++i) if (!NameChar(name[i])) return false;
  for (size_t i = length - 8; i < length; ++i)
    if (!((name[i] >= '0' && name[i] <= '9') || (name[i] >= 'A' && name[i] <= 'F'))) return false;
  return true;
}

inline bool Bucket(const char* name, size_t length, uint32_t* number) {
  if (length != 8 || name[0] != 'D') return false;
  uint32_t n = 0;
  for (int i = 1; i < 8; ++i) {
    if (name[i] < '0' || name[i] > '9') return false;
    n = n * 10u + unsigned(name[i] - '0');
  }
  if (n > 9999999u / kClipsPerDirectory) return false;
  if (number) *number = n;
  return true;
}

inline bool AudioDirectory(const char* path) {
  if (!path) return false;
  if (!*path || !std::strcmp(path, "AUDIO")) return true;
  if (std::strncmp(path, "AUDIO/", 6)) return Bucket(path, std::strlen(path), nullptr);
  const char* name = path + 6;
  const char* slash = std::strchr(name, '/');
  if (!slash) return SpeciesComponent(name, std::strlen(name));
  return SpeciesComponent(name, slash - name) && Bucket(slash + 1, std::strlen(slash + 1), nullptr);
}

inline bool AudioPath(uint32_t sequence, const char* extension, char* path, size_t size,
                      const char* label = nullptr) {
  if (sequence > 9999999u || !path || size < kAudioPathBytes || !extension ||
      (std::strcmp(extension, "BRC") != 0 && std::strcmp(extension, "WAV") != 0 &&
       std::strcmp(extension, "OGG") != 0)) return false;
  char species[kSpeciesDirectoryBytes];
  if (!SpeciesDirectory(label, species, sizeof(species))) return false;
  const int n = birdnet_format::Snprintf(path, size, "AUDIO/%s/D%07lu/A%07lu.%s", species,
                       static_cast<unsigned long>(sequence / kClipsPerDirectory),
                       static_cast<unsigned long>(sequence), extension);
  return n > 0 && static_cast<size_t>(n) < size;
}

inline bool ParseSequence(const char* name, size_t length, uint32_t* sequence) {
  if (!name || !sequence) return false;
  if (length >= 6 && !std::memcmp(name, "AUDIO/", 6)) {
    const char* slash = static_cast<const char*>(std::memchr(name + 6, '/', length - 6));
    if (!slash || !SpeciesComponent(name + 6, slash - name - 6)) return false;
    length -= slash + 1 - name;
    name = slash + 1;
    if (length != 21) return false;
  }
  uint32_t directory = 0;
  if (length == 21 && name[0] == 'D' && name[8] == '/') {
    if (!Bucket(name, 8, &directory)) return false;
    name += 9;
  } else if (length != 12) {
    return false;
  }
  if (name[0] != 'A' || name[8] != '.' ||
      (std::memcmp(name + 9, "BRC", 3) && std::memcmp(name + 9, "WAV", 3) &&
       std::memcmp(name + 9, "OGG", 3))) return false;
  uint32_t value = 0;
  for (int i = 1; i < 8; ++i) {
    if (name[i] < '0' || name[i] > '9') return false;
    value = value * 10u + static_cast<uint32_t>(name[i] - '0');
  }
  if (length == 21 && directory != value / kClipsPerDirectory) return false;
  *sequence = value;
  return true;
}

inline uint32_t NextRetryDelay(uint32_t previous_ms) {
  return previous_ms == 0 ? 60000u : (previous_ms >= 450000u ? 900000u : previous_ms * 2u);
}
}  // namespace birdnet_storage
