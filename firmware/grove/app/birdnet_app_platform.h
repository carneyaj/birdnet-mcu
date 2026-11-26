// Private application implementation. Included inside birdnet_we2.cpp
// after SDK headers; not a standalone public header.
#pragma once
bool SetModelXipEnabled(bool enable) {
  birdnet_log::Status("[birdnet] QSPI request enable=%d opened=%d xip=%d\r\n",
                enable ? 1 : 0,
                g_model_flash_opened ? 1 : 0,
                g_model_xip_enabled ? 1 : 0);
  if (!g_model_flash_opened) {
    birdnet_log::Status("[birdnet] QSPI open begin\r\n");
    if (hx_lib_spi_eeprom_open(USE_DW_SPI_MST_Q) != 0) {
      birdnet_log::Write("[birdnet] QSPI open failed\r\n");
      return false;
    }
    g_model_flash_opened = true;
    birdnet_log::Status("[birdnet] QSPI open ok\r\n");
  }

  if (g_model_xip_enabled == enable) {
    birdnet_log::Status("[birdnet] QSPI XIP already %s\r\n", enable ? "enabled" : "disabled");
    return true;
  }

  birdnet_log::Status("[birdnet] QSPI XIP set begin enable=%d\r\n", enable ? 1 : 0);
  if (hx_lib_spi_eeprom_enable_XIP(USE_DW_SPI_MST_Q, enable, FLASH_QUAD, true) != 0) {
    birdnet_log::Write("[birdnet] QSPI XIP %s failed\r\n", enable ? "enable" : "disable");
    return false;
  }

  g_model_xip_enabled = enable;
  birdnet_log::Status("[birdnet] QSPI XIP set ok enable=%d\r\n", enable ? 1 : 0);
  return true;
}

uint64_t PlatformMillis() {
  uint32_t tick = 0;
  uint32_t loop_cnt = 0;
  SystemGetTick(&tick, &loop_cnt);
  const uint64_t elapsed_ticks =
      static_cast<uint64_t>(loop_cnt) * static_cast<uint64_t>(SysTick_LOAD_RELOAD_Msk + 1u) +
      static_cast<uint64_t>(SysTick_LOAD_RELOAD_Msk + 1u - tick);
  return elapsed_ticks / (SystemCoreClock / 1000u);
}

