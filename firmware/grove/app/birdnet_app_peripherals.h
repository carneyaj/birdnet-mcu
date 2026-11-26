// Private application implementation. Included inside birdnet_we2.cpp
// after SDK headers; not a standalone public header.
#pragma once
GPIO_OUT_LEVEL_E DetectionLedLevel(bool on) {
  const bool drive_high = cfg::kDetectionLedActiveLow ? !on : on;
  return drive_high ? GPIO_OUT_HIGH : GPIO_OUT_LOW;
}

void InitDetectionLed() {
  if constexpr (!cfg::kEnableDetectionLed) {
    return;
  }
  hx_drv_scu_set_SEN_D2_pinmux(SCU_SEN_D2_PINMUX_GPIO20);
  const GPIO_OUT_LEVEL_E off_level = DetectionLedLevel(false);
  const GPIO_ERROR_E direction_err = hx_drv_gpio_set_output(GPIO20, off_level);
  const GPIO_ERROR_E level_err = hx_drv_gpio_set_out_value(GPIO20, off_level);
  g_detection_led_initialized =
      (direction_err == GPIO_NO_ERROR && level_err == GPIO_NO_ERROR);
}

void SetDetectionLed(bool on) {
  if constexpr (!cfg::kEnableDetectionLed) {
    (void)on;
    return;
  }
  // Reassert pinmux, direction, and the inactive level on every clear. GPIO20
  // retains its output state in audio-only sleep, so this also fails safe if a
  // previous detection was interrupted during the following warm boot.
  if (!g_detection_led_initialized || !on) {
    InitDetectionLed();
  }
  if (!g_detection_led_initialized) {
    return;
  }
  if (hx_drv_gpio_set_out_value(GPIO20, DetectionLedLevel(on)) != GPIO_NO_ERROR) {
    g_detection_led_initialized = false;
  }
}

void SetOledLease(bool on) {
#if BIRDNET_ENABLE_STATUS_BUTTON
  g_retained_runtime.oled_on = on;
  if (!on) g_retained_runtime.oled_page = 0;
  g_retained_runtime.oled_address = birdnet_oled::Address();
  g_retained_runtime.oled_command = birdnet_ui_command(g_retained_runtime.oled_command, on);
  hx_drv_mb_set_sharedata(BN_MB_UI_COMMAND, g_retained_runtime.oled_command);
  __DMB();
  hx_drv_mb_trigger_cm55s_irq(BN_MB_SLEEP_EVENT);
#else
  (void)on;
#endif
}

void InitDetectionOled() {
  if constexpr (!cfg::kEnableDetectionOled) {
    return;
  }
  const uint64_t capture_ms = g_retained_runtime.last_capture_end * 1000u / cfg::kSampleRateHz;
  if (capture_ms < g_retained_runtime.oled_retry_ms) return;
  g_detection_oled_available = cfg::kEnableStatusButton && g_retained_runtime.oled_on
      ? birdnet_oled::Resume(g_retained_runtime.oled_address) : birdnet_oled::Init();
  g_detection_oled_error_logged = false;
  if (!g_detection_oled_available) {
    g_retained_runtime.oled_retry_ms = capture_ms + 60000u;
    birdnet_log::Status("[birdnet] detection OLED unavailable at startup @0x%02x\r\n",
                  static_cast<unsigned int>(cfg::kDetectionOledI2cAddr));
  } else {
    g_retained_runtime.oled_retry_ms = 0;
  }
}

void SetDetectionOutputs(const InferenceResult* result, const Sht41Reading* sht41) {
  const bool detected = result && result->detected;
  SetDetectionLed(detected);
  if constexpr (cfg::kEnableLedDiagnostics) {
    birdnet_log::Write("[bntrace] led=%d oled=%d page=%d\r\n", int(detected),
        int(g_detection_oled_available), int(g_retained_runtime.oled_page));
  }

  if constexpr (!cfg::kEnableDetectionOled) {
    (void)sht41;
    return;
  }
  if (!g_detection_oled_available) {
    InitDetectionOled();
    if (!g_detection_oled_available) return;
  }

  if (!detected || !result || result->pred_class < 0) {
    // A negative result is not an instruction to truncate an active lease.
    // Explicit nullptr calls (fault/export) still force the panel off.
    if (cfg::kEnableStatusButton && result) return;
    SetOledLease(false);
    if (!birdnet_oled::PowerOff()) {
      g_detection_oled_available = false;
      g_retained_runtime.oled_retry_ms =
          g_audio_ring_total_samples * 1000u / cfg::kSampleRateHz + 60000u;
      if (!g_detection_oled_error_logged) {
        birdnet_log::Status("[birdnet] detection OLED unavailable at runtime\r\n");
        g_detection_oled_error_logged = true;
      }
    }
    return;
  }

  const bool env_ok = sht41 && sht41->ok;
  // A detection must not overwrite or extend a deliberately selected page.
  if (g_retained_runtime.oled_on && g_retained_runtime.oled_page != 0) return;
  const int temp_c_q1e2 = sht41 ? sht41->temp_c_q1e2 : 0;
  const int rh_q1e2 = sht41 ? sht41->rh_q1e2 : 0;
  if (!birdnet_oled::ShowDetection(
          birdnet_labels::LabelForIndex(result->pred_class),
          result->pred_score,
          env_ok,
          temp_c_q1e2,
          rh_q1e2,
          g_battery_reading.ok,
          g_battery_reading.percent_q1e2,
          g_battery_reading.millivolts)) {
    g_detection_oled_available = false;
    g_retained_runtime.oled_retry_ms =
        g_audio_ring_total_samples * 1000u / cfg::kSampleRateHz + 60000u;
    if (!g_detection_oled_error_logged) {
      birdnet_log::Status("[birdnet] detection OLED unavailable at runtime\r\n");
      g_detection_oled_error_logged = true;
    }
  } else {
    SetOledLease(true);
    if constexpr (cfg::kEnableLedDiagnostics) birdnet_log::Write("[bntrace] detection drawn\r\n");
    birdnet_log::Status("[birdnet] OLED detection displayed (height=%d, env=%d, battery=%d)\r\n",
                  cfg::kOledHeight, env_ok, g_battery_reading.ok);
  }
}

void app_i2cm_0_err_cb(void* status) {
  if (!status) {
    g_i2c_last_err_state = 0;
    return;
  }
  HX_DRV_DEV_IIC* iic_obj = static_cast<HX_DRV_DEV_IIC*>(status);
  HX_DRV_DEV_IIC_INFO* iic_info_ptr = &(iic_obj->iic_info);
  g_i2c_last_err_state = iic_info_ptr->err_state;
}

uint8_t Sht41Crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80u) ? static_cast<uint8_t>((crc << 1) ^ 0x31u)
                          : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

bool InitI2cMaster0() {
  if (g_i2c_initialized) {
    return true;
  }

  hx_drv_scu_set_PA2_pinmux(SCU_PA2_PINMUX_I2C_M_SCL, 1);
  hx_drv_scu_set_PA3_pinmux(SCU_PA3_PINMUX_I2C_M_SDA, 1);
  g_i2c_last_err_state = 0;
  if (hx_drv_i2cm_init(USE_DW_IIC_0, BASE_ADDR_APB_I2C_MST_0, DW_IIC_SPEED_FAST) != IIC_ERR_OK) {
    birdnet_log::Write("[birdnet] i2c master init failed\r\n");
    return false;
  }
  if (hx_drv_i2cm_set_err_cb(USE_DW_IIC_0, reinterpret_cast<void*>(app_i2cm_0_err_cb)) != IIC_ERR_OK) {
    birdnet_log::Write("[birdnet] i2c err cb init failed\r\n");
    return false;
  }

  g_i2c_initialized = true;
  birdnet_log::Status("[birdnet] I2C master 0 ready on Grove bus\r\n");
  return true;
}

bool InitSht41() {
  if (!cfg::kEnableSht41Sensor) {
    return true;
  }
  if (!InitI2cMaster0()) {
    return false;
  }
  birdnet_log::Status("[birdnet] SHT4x (SHT40/41/45) enabled @0x%02x\r\n", cfg::kSht41I2cAddr);
  return true;
}

bool g_sht_pending = false;
uint64_t g_sht_started_ms = 0;

bool StartSht41() {
  g_sht_pending = false;
  if (!cfg::kEnableSht41Sensor) {
    return false;
  }
  const uint64_t capture_ms = g_audio_ring_total_samples * 1000u / cfg::kSampleRateHz;
  if (capture_ms < g_retained_runtime.sht_retry_ms) return false;
  if (!InitI2cMaster0()) {
    return false;
  }

  uint8_t measure_cmd = 0xFD;
  uint8_t unused = 0;
  g_i2c_last_err_state = 0;

  const IIC_ERR_CODE_E write_err =
      hx_drv_i2cm_write_data(USE_DW_IIC_0, cfg::kSht41I2cAddr, &measure_cmd, 1, &unused, 0);
  if (write_err != IIC_ERR_OK) {
    g_retained_runtime.sht_retry_ms = capture_ms + 60000u;
    if (!g_retained_runtime.sht_error_logged) {
      birdnet_log::Write("[birdnet] SHT41 write failed err=%d state=%lu\r\n",
              static_cast<int>(write_err),
              static_cast<unsigned long>(g_i2c_last_err_state));
      g_retained_runtime.sht_error_logged = true;
    }
    return false;
  }
  g_sht_pending = true;
  g_sht_started_ms = PlatformMillis();
  return true;
}

bool ReadSht41(Sht41Reading* out) {
  if (!out) return false;
  *out = {};
  if (!g_sht_pending && !StartSht41()) return false;
  const uint64_t elapsed = PlatformMillis() - g_sht_started_ms;
  if (elapsed < static_cast<uint64_t>(cfg::kSht41MeasureDelayMs)) {
    hx_drv_timer_cm55x_delay_ms(cfg::kSht41MeasureDelayMs - elapsed, TIMER_STATE_DC);
  }
  g_sht_pending = false;
  uint8_t rx[6]{};

  const IIC_ERR_CODE_E read_err =
      hx_drv_i2cm_read_data(USE_DW_IIC_0, cfg::kSht41I2cAddr, rx, sizeof(rx));
  if (read_err != IIC_ERR_OK) {
    g_retained_runtime.sht_retry_ms = g_audio_ring_total_samples * 1000u / cfg::kSampleRateHz + 60000u;
    if (!g_retained_runtime.sht_error_logged) {
      birdnet_log::Write("[birdnet] SHT41 read failed err=%d state=%lu\r\n",
              static_cast<int>(read_err),
              static_cast<unsigned long>(g_i2c_last_err_state));
      g_retained_runtime.sht_error_logged = true;
    }
    return false;
  }

  if (Sht41Crc8(rx, 2) != rx[2] || Sht41Crc8(rx + 3, 2) != rx[5]) {
    g_retained_runtime.sht_retry_ms = g_audio_ring_total_samples * 1000u / cfg::kSampleRateHz + 60000u;
    if (!g_retained_runtime.sht_error_logged) {
      birdnet_log::Write("[birdnet] SHT41 CRC failed t_crc=0x%02x rh_crc=0x%02x\r\n",
              static_cast<unsigned int>(rx[2]),
              static_cast<unsigned int>(rx[5]));
      g_retained_runtime.sht_error_logged = true;
    }
    return false;
  }

  const uint16_t raw_temp = static_cast<uint16_t>((static_cast<uint16_t>(rx[0]) << 8) | rx[1]);
  const uint16_t raw_rh = static_cast<uint16_t>((static_cast<uint16_t>(rx[3]) << 8) | rx[4]);
  const float temp_c = -45.0f + (175.0f * static_cast<float>(raw_temp) / 65535.0f);
  float rh = -6.0f + (125.0f * static_cast<float>(raw_rh) / 65535.0f);
  if (rh < 0.0f) {
    rh = 0.0f;
  } else if (rh > 100.0f) {
    rh = 100.0f;
  }

  out->ok = true;
  out->temp_c_q1e2 = static_cast<int>(std::lround(temp_c * 100.0f));
  out->rh_q1e2 = static_cast<int>(std::lround(rh * 100.0f));
  g_retained_runtime.sht_error_logged = false;
  g_retained_runtime.sht_retry_ms = 0;
  return true;
}

void ReadBattery() {
  g_battery_reading = {};
  if constexpr (!cfg::kEnableMax17048) return;
  const uint64_t capture_ms = g_audio_ring_total_samples * 1000u / cfg::kSampleRateHz;
  if (capture_ms < g_retained_runtime.battery_retry_ms) return;
  if (!InitI2cMaster0() || !birdnet_battery::Read(&g_battery_reading)) {
    g_retained_runtime.battery_retry_ms = capture_ms + 60000u;
    birdnet_log::Status("[birdnet] MAX17048 unavailable (battery connected?); retry in 60s\r\n");
    return;
  }
  g_retained_runtime.battery_retry_ms = 0;
  birdnet_log::Status("[birdnet] MAX17048 mv=%d soc_q1e2=%d\r\n",
                g_battery_reading.millivolts, g_battery_reading.percent_q1e2);
}

#if BIRDNET_ENABLE_STATUS_BUTTON
void ConfigureStatusButton() {
  // CM55S owns the GPIO18 callback; do not service the shared IRQ on CM55M.
  NVIC_DisableIRQ(GPIO_GROUP_2_IRQn);
  hx_drv_scu_set_SEN_D0_pinmux(SCU_SEN_D0_PINMUX_GPIO18);
  hx_drv_gpio_set_input(GPIO18);
  hx_drv_gpio_set_int_type(GPIO18, GPIO_IRQ_TRIG_TYPE_EDGE_RISING);
  hx_drv_gpio_set_int_enable(GPIO18, 1);
}

uint64_t CaptureUptimeSeconds(uint32_t capture_low) {
  // Capture runs through CM55M deep sleep and low-battery inference pauses.
  // Extend the helper's UI counter relative to the retained 64-bit snapshot;
  // its low word wraps after ~25.5 hours. Ignore a slightly older UI update.
  uint64_t samples = g_retained_runtime.last_capture_end;
  const uint32_t advance = capture_low - static_cast<uint32_t>(samples);
  if (advance < 0x80000000u) samples += advance;
  // This release uses LSCREF / PDM_PCM_FREQ_48K: the physical rate is 46875 Hz.
  // Do not use the model's nominal 48000 Hz or the warm-boot-reset SysTick.
  static_assert(cfg::kSampleRateHz == 48000, "Revisit uptime for a different capture clock");
  return samples / 46875u;
}

void HandleStatusUi() {
  uint32_t sequence = 0, ack = 0, expired = 0, capture = 0, diagnostics = 0;
  hx_drv_mb_get_sharedata(BN_MB_BUTTON_SEQUENCE, &sequence);
  hx_drv_mb_get_sharedata(BN_MB_BUTTON_ACK, &ack);
  hx_drv_mb_get_sharedata(BN_MB_UI_EXPIRED, &expired);
  hx_drv_mb_get_sharedata(BN_MB_UI_CAPTURE_LOW, &capture);
  hx_drv_mb_get_sharedata(BN_MB_UI_DIAGNOSTICS, &diagnostics);
  if constexpr (cfg::kEnableStatusLogs) {
    uint8_t pin = 0;
    const int pin_error = hx_drv_gpio_get_in_value(GPIO18, &pin);
    birdnet_log::Status("[birdnet] UI input main=%u err=%d helper=0x%lx seq=%lu ack=%lu\r\n",
                  pin, pin_error, static_cast<unsigned long>(diagnostics),
                  static_cast<unsigned long>(sequence), static_cast<unsigned long>(ack));
  }
  if (sequence != ack) {
    hx_drv_mb_set_sharedata(BN_MB_BUTTON_ACK, sequence);
    // A deliberate button press also retries newly connected peripherals.
    g_retained_runtime.sht_retry_ms = 0;
    g_retained_runtime.battery_retry_ms = 0;
    g_retained_runtime.oled_retry_ms = 0;
    Sht41Reading env{};
    ReadSht41(&env);
    ReadBattery();
    if (!g_detection_oled_available) InitDetectionOled();
    bool shown = false;
#if BIRDNET_ENABLE_FIELD_MONITORING
    uint32_t presses = sequence - ack; // Preserve accepted taps, including sequence wrap.
    const bool browsing = g_retained_runtime.oled_on &&
        expired != g_retained_runtime.oled_command && g_retained_runtime.oled_page >= 1;
    if (!browsing) {
      g_retained_runtime.oled_page = 1;
      --presses; // First press wakes status; remaining presses advance the pages.
    }
    if (presses && g_detection_oled_available) shown = ShowFieldCounts(presses);
    if (g_retained_runtime.oled_page == 1)
#endif
    {
      g_retained_runtime.oled_page = 1;
      shown = g_detection_oled_available &&
          birdnet_oled::ShowStatus(env.ok, env.temp_c_q1e2, env.rh_q1e2,
              g_battery_reading.ok, g_battery_reading.percent_q1e2,
              g_battery_reading.millivolts
#if BIRDNET_ENABLE_FIELD_MONITORING
              , g_field.battery.paused
#else
              , false
#endif
              , CaptureUptimeSeconds(capture)
          );
    }
    SetOledLease(shown);
    if (!shown) {
      g_detection_oled_available = false;
      g_retained_runtime.oled_retry_ms =
          g_retained_runtime.last_capture_end * 1000u / cfg::kSampleRateHz + 60000u;
    }
    birdnet_log::Status("[birdnet] UI button seq=%lu shown=%d env=%d battery=%d capture_low=%lu pin=%lu\r\n",
                  static_cast<unsigned long>(sequence), shown, env.ok, g_battery_reading.ok,
                  static_cast<unsigned long>(capture), static_cast<unsigned long>(diagnostics));
  } else if (g_retained_runtime.oled_on && expired == g_retained_runtime.oled_command) {
    const bool off = birdnet_oled::PowerOff();
    SetOledLease(false);
    if (!off) {
      g_detection_oled_available = false;
      g_retained_runtime.oled_retry_ms =
          g_retained_runtime.last_capture_end * 1000u / cfg::kSampleRateHz + 60000u;
    }
    birdnet_log::Status("[birdnet] UI timeout off=%d capture_low=%lu\r\n",
                  off, static_cast<unsigned long>(capture));
  }
}
#endif

