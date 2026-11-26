#pragma once

#include <cstdint>

namespace birdnet_battery {

struct Reading {
  bool ok = false;
  int percent_q1e2 = 0;
  int millivolts = 0;
};

// The caller initializes I2C master 0. No reset/quick-start: ModelGauge must
// keep tracking the battery across the main core's three-second warm boots.
bool Read(Reading* out);

}  // namespace birdnet_battery
