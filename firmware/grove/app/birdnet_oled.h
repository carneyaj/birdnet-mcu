#pragma once
#include <cstdint>

namespace birdnet_oled {

using EnsureI2cReadyFn = bool (*)();

void SetEnsureI2cReadyFn(EnsureI2cReadyFn fn);
bool Init();
// Restore software state only; do not blank an active panel at a warm boot.
bool Resume(uint8_t address);
uint8_t Address();
bool ShowStatus(bool env_ok, int temp_c_q1e2, int rh_q1e2,
                bool battery_ok, int battery_percent_q1e2, int battery_mv = 0,
                bool paused = false, uint64_t uptime_seconds = 0);
bool ShowCounts(const char* const labels[3], const uint32_t counts[3],
                unsigned page, unsigned pages);
bool ShowDetection(const char* label,
                   float probability,
                   bool env_ok,
                   int temp_c_q1e2,
                   int rh_q1e2,
                   bool battery_ok = false,
                   int battery_percent_q1e2 = 0,
                   int battery_mv = 0);
bool PowerOff();

}  // namespace birdnet_oled
