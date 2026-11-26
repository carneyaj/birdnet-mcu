#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>

#ifndef BIRDNET_ENABLE_FIELD_MONITORING
#define BIRDNET_ENABLE_FIELD_MONITORING 0
#endif
#ifndef BIRDNET_BATTERY_PAUSE_MV
#define BIRDNET_BATTERY_PAUSE_MV 3500
#endif
#ifndef BIRDNET_BATTERY_RESUME_MV
#define BIRDNET_BATTERY_RESUME_MV 3800
#endif

namespace birdnet_field {
constexpr uint64_t kHourMs = 3600000;
constexpr uint64_t kHealthMs = 300000;
constexpr uint64_t kAudioReserve = 64ull * 1024 * 1024;
constexpr uint64_t kLogReserve = 1024 * 1024;
constexpr uint64_t kClipBudget = 256 * 1024; // Includes allocation rounding and map row.
static_assert(BIRDNET_BATTERY_PAUSE_MV >= 3300 &&
              BIRDNET_BATTERY_RESUME_MV > BIRDNET_BATTERY_PAUSE_MV &&
              BIRDNET_BATTERY_RESUME_MV <= 4200, "Expected a conventional 1S Li-ion cell");

struct BatteryPolicy {
  uint64_t since;
  uint64_t last_read;
  bool pending;
  bool paused;

  bool Update(uint64_t now, bool valid, int mv) {
    const bool old = paused;
    // A missing gauge never initiates a pause or authorizes recovery. A long
    // measurement gap cannot count as sustained voltage above/below a limit.
    if (!valid || mv < 2500 || mv > 4500) { pending = false; return false; }
    if (!paused && mv <= 3300) { paused = true; pending = false; return true; }
    if (now < last_read || now - last_read > 20000) pending = false;
    last_read = now;
    const bool qualified = paused ? mv >= BIRDNET_BATTERY_RESUME_MV
                                  : mv <= BIRDNET_BATTERY_PAUSE_MV;
    if (!qualified) pending = false;
    else {
      if (!pending) { since = now; pending = true; }
      if (now - since >= (paused ? 120000u : 30000u)) {
        paused = !paused;
        pending = false;
      }
    }
    return old != paused;
  }
};

// Fixed-size, detection-independent sampling schedule. Mark attempts, not just
// successes, so a full/missing SD cannot cause an encode/retry on every window.
inline bool AmbientDue(uint64_t now, uint64_t* last_hour) {
  const uint64_t hour = now / kHourMs;
  if (!hour || hour <= *last_hour) return false;
  *last_hour = hour;
  return true;
}

struct SpeciesCount { int index; uint32_t count; };
template <size_t Classes> struct StartupCounts {
  uint32_t totals[Classes];

  void Add(int index) {
    if (index < 0 || size_t(index) >= Classes) return;
    auto& count = totals[index];
    if (count != UINT32_MAX) ++count;
  }
  template <typename IsBird> void Top(SpeciesCount* top,
                                     size_t size, IsBird is_bird) {
    for (size_t j = 0; j < size; ++j) top[j] = {-1, 0};
    for (size_t i = 0; i < Classes; ++i) {
      if (!is_bird(int(i))) continue;
      const uint32_t sum = totals[i];
      for (size_t j = 0; j < size; ++j) {
        if (sum <= top[j].count) continue;
        for (size_t k = size - 1; k > j; --k) top[k] = top[k - 1];
        top[j] = {int(i), sum};
        break;
      }
    }
  }
};
} // namespace birdnet_field
