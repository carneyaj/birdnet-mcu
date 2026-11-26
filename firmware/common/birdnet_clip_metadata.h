#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace birdnet_clip {
constexpr size_t kBytes = 256;
constexpr size_t kMaxDetections = 16;
inline void Put16(uint8_t* p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
inline void Put32(uint8_t* p, uint32_t v) {
  for (unsigned i = 0; i < 4; ++i) p[i] = v >> (8 * i);
}
inline uint32_t Get32(const uint8_t* p) {
  return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
}
// Incremental IEEE CRC32; initialize with ~0u and complement at completion.
inline uint32_t Crc(uint32_t crc, const uint8_t* p, size_t n) {
  static constexpr uint32_t lut[16] = {
    0,0x1db71064,0x3b6e20c8,0x26d930ac,0x76dc4190,0x6b6b51f4,0x4db26158,0x5005713c,
    0xedb88320,0xf00f9344,0xd6d6a3e8,0xcb61b38c,0x9b64c2b0,0x86d3d2d4,0xa00ae278,0xbdbdf21c};
  while (n--) { crc ^= *p++; crc = (crc >> 4) ^ lut[crc & 15]; crc = (crc >> 4) ^ lut[crc & 15]; }
  return crc;
}
struct Metadata {
  uint32_t session = 0, sequence = 0;
  uint64_t capture_end_ms = 0;
  uint16_t threshold_q1e4 = 0, count = 0;
  bool overflow = false;
  const char* deployment = nullptr;
  uint32_t species[kMaxDetections]{};
  uint16_t probability[kMaxDetections]{};
};
inline bool Pack(const Metadata& m, uint32_t audio_crc, uint32_t audio_bytes, uint8_t* out,
                 uint32_t header_crc = 0) {
  if (!out || !m.deployment || std::strlen(m.deployment) > 64 || m.count > kMaxDetections) return false;
  std::memset(out, 0, kBytes);
  std::memcpy(out, "BNMD", 4);
  Put16(out + 4, 1); Put16(out + 6, kBytes);
  Put32(out + 8, m.session); Put32(out + 12, m.sequence);
  Put32(out + 16, uint32_t(m.capture_end_ms)); Put32(out + 20, uint32_t(m.capture_end_ms >> 32));
  Put16(out + 24, m.threshold_q1e4); Put16(out + 26, m.count);
  Put32(out + 28, m.overflow ? 1 : 0); // Time is capture-relative, never wall clock.
  std::memcpy(out + 32, m.deployment, std::strlen(m.deployment));
  for (size_t i = 0; i < m.count; ++i) {
    Put32(out + 112 + i * 8, m.species[i]); Put16(out + 116 + i * 8, m.probability[i]);
  }
  Put32(out + 240, audio_crc); Put32(out + 244, audio_bytes);
  Put32(out + 248, header_crc);
  Put32(out + 252, ~Crc(~0u, out, 252));
  return true;
}
inline void SessionRecord(uint32_t session, uint8_t* out) {
  std::memcpy(out, "BNS1", 4); Put32(out + 4, session); Put32(out + 8, ~session);
  Put32(out + 12, ~Crc(~0u, out, 12));
}
inline bool ValidSession(const uint8_t* p) {
  return std::memcmp(p, "BNS1", 4) == 0 && Get32(p + 4) != 0 &&
         Get32(p + 8) == ~Get32(p + 4) && Get32(p + 12) == ~Crc(~0u, p, 12);
}
} // namespace birdnet_clip
