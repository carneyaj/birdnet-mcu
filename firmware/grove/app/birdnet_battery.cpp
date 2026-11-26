#include "birdnet_battery.h"

extern "C" {
#include "hx_drv_iic.h"
}

namespace birdnet_battery {
namespace {
constexpr uint8_t kAddress = 0x36;

bool ReadRegister(uint8_t address, uint16_t* out) {
  uint8_t data[2]{};
  if (hx_drv_i2cm_write_restart_read(USE_DW_IIC_0, kAddress, &address, 1,
                                    data, sizeof(data)) != IIC_ERR_OK) {
    return false;
  }
  *out = (static_cast<uint16_t>(data[0]) << 8) | data[1];
  return true;
}
}  // namespace

bool Read(Reading* out) {
  if (!out) return false;
  *out = {};
  uint16_t version = 0, voltage = 0, soc = 0;
  // Match the version family as in Adafruit's driver; an unpowered gauge can
  // return 0xffff. Qwiic power alone does not power its battery-side circuitry.
  if (!ReadRegister(0x08, &version) || (version & 0xfff0u) != 0x0010u ||
      !ReadRegister(0x02, &voltage) || !ReadRegister(0x04, &soc) ||
      voltage == 0 || voltage == 0xffffu || soc == 0xffffu) {
    return false;
  }
  // MAX17048 VCELL: 78.125 uV/LSB; SOC: 1/256 percent/LSB, MSB first.
  out->millivolts = (static_cast<uint32_t>(voltage) * 5u + 32u) / 64u;
  const int percent = (static_cast<uint32_t>(soc) * 100u + 128u) / 256u;
  out->percent_q1e2 = percent > 10000 ? 10000 : percent;
  out->ok = true;
  return true;
}
}  // namespace birdnet_battery
