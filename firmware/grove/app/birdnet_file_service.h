#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>
#include "birdnet_storage_policy.h"
#include "birdnet_clip_metadata.h"

namespace birdnet_files {
constexpr char kBeacon[] = "\nBNFS2 READY\n";
constexpr char kKnock[] = "BNFSOPEN";
constexpr size_t kPathBytes = birdnet_storage::kAudioPathBytes;
constexpr size_t kRequestBytes = 16 + kPathBytes + 4, kChunkBytes = 4096;
constexpr size_t kEntryBytes = kPathBytes + 8;
constexpr uint64_t kIdleMs = 5000, kSessionMs = 600000;
enum Op : uint8_t { List = 1, Next = 2, Open = 3, Read = 4, Quit = 5 };
enum Status : uint8_t { Ok = 0, Invalid = 1, IoError = 2, End = 3, Busy = 4 };

inline uint32_t Get32(const uint8_t* p) {
  return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
}
inline void Put32(uint8_t* p, uint32_t v) {
  for (unsigned i = 0; i < 4; ++i) p[i] = uint8_t(v >> (8 * i));
}
inline uint32_t CrcUpdate(uint32_t crc, const uint8_t* p, size_t size) {
  return birdnet_clip::Crc(crc, p, size);
}
inline bool Directory(const char* path) {
  return birdnet_storage::AudioDirectory(path);
}
inline bool File(const char* path) {
  uint32_t sequence = 0;
  if (std::strlen(path) == 12 && path[0] == 'M' && !std::strcmp(path + 8, ".TXT")) {
    for (int i = 1; i < 8; ++i) if (path[i] < '0' || path[i] > '9') return false;
    return true;
  }
  return birdnet_storage::ParseSequence(path, std::strlen(path), &sequence) ||
      !std::strcmp(path, "HEALTH.CSV") ||
      !std::strcmp(path, "BIRDLOG.CSV") || !std::strcmp(path, "ENVLOG.CSV") ||
      !std::strcmp(path, "AUDIOMAP.CSV") || !std::strcmp(path, "BOOTSEQ.BIN");
}
struct Request {
  uint16_t id;
  uint8_t op;
  uint32_t offset, length;
  char path[kPathBytes];
};
inline bool Decode(const uint8_t* p, Request* out) {
  if (std::memcmp(p, "BNQ2", 4) || p[7] >= kPathBytes ||
      (CrcUpdate(~0u, p, kRequestBytes - 4) ^ ~0u) != Get32(p + kRequestBytes - 4)) return false;
  for (unsigned i = 0; i < kPathBytes; ++i) {
    if (i < p[7] ? (p[16 + i] == 0) : (p[16 + i] != 0)) return false;
  }
  out->id = uint16_t(p[4] | uint16_t(p[5]) << 8);
  out->op = p[6];
  out->offset = Get32(p + 8);
  out->length = Get32(p + 12);
  std::memcpy(out->path, p + 16, sizeof(out->path));
  switch (out->op) {
    case List: return !out->offset && !out->length && Directory(out->path);
    case Open: return !out->offset && !out->length && File(out->path);
    case Read: return !p[7] && out->length && out->length <= kChunkBytes;
    case Next: case Quit: return !p[7] && !out->offset && !out->length;
    default: return false;
  }
}

// The platform owns UART, watchdog/capture servicing, and the exclusive SD mount.
struct Platform {
  int (*read)(uint8_t*, size_t); // nonblocking: 0=no data, negative=error
  bool (*write)(const uint8_t*, size_t);
  uint64_t (*millis)();
  bool (*progress)();
  bool (*mount)();
  void (*unmount)();
  uint32_t (*skipped_windows)();
};
void Run(const Platform& platform, const char* deployment_id);
}  // namespace birdnet_files
