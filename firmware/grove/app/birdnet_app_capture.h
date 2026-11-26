// Private application implementation. Included inside birdnet_we2.cpp
// after SDK headers; not a standalone public header.
#pragma once
uint32_t SaveAndDisableIrq() {
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

void RestoreIrq(uint32_t primask) {
  __set_PRIMASK(primask);
}

uint32_t TakeMailboxEvents() {
  const uint32_t primask = SaveAndDisableIrq();
  const uint32_t events = g_mb_event_bits;
  g_mb_event_bits = 0;
  RestoreIrq(primask);
  return events;
}

void PushAudioRingSamples(const int16_t* samples, int n) {
  if (!samples || n <= 0) {
    return;
  }
  for (int i = 0; i < n; ++i) {
    g_audio_ring[g_audio_ring_write_idx] = samples[i];
    ++g_audio_ring_write_idx;
    if (g_audio_ring_write_idx >= static_cast<size_t>(cfg::kWindowSamples)) {
      g_audio_ring_write_idx = 0;
    }
    if (g_audio_ring_fill < static_cast<size_t>(cfg::kWindowSamples)) {
      ++g_audio_ring_fill;
    }
    ++g_audio_ring_total_samples;
  }
}

bool AudioRingReady() {
  return g_audio_ring_fill >= static_cast<size_t>(cfg::kWindowSamples);
}

size_t CurrentWindowStartIdx() {
  return g_audio_ring_write_idx;
}

void app_pdm_dma_rx_cb(uint32_t /*int_status*/) {
  const int finished_idx = g_write_idx;
  g_write_idx ^= 1;
  hx_drv_pdm_dma_lli_transfer((void*)g_dma_buf[g_write_idx], kDmaBlocks, kDmaBlockBytes, 0);
  SCB_InvalidateDCache_by_Addr((uint32_t*)g_dma_buf[finished_idx], kDmaTotalBytes);

  if (g_chunk_queue_count >= static_cast<uint32_t>(kChunkQueueSlots)) {
    ++g_chunk_queue_overflow_count;
    return;
  }

  int16_t* dst = g_chunk_queue[g_chunk_queue_tail];
  if (kDecimation == 1) {
    std::memcpy(dst, g_dma_buf[finished_idx], static_cast<size_t>(kOutputChunkSamples) * sizeof(int16_t));
  } else {
    for (int i = 0; i < kOutputChunkSamples; ++i) {
      dst[i] = g_dma_buf[finished_idx][i * kDecimation];
    }
  }

  g_chunk_queue_tail = (g_chunk_queue_tail + 1) % static_cast<uint32_t>(kChunkQueueSlots);
  ++g_chunk_queue_count;
}

void app_vad_cb(uint32_t /*int_status*/) {
  ++g_vad_irq_count;
}

void app_mb_cb(uint32_t event) {
  if (event == 0) {
    return;
  }
  hx_drv_mb_clear_cm55m_irq(event);
  g_mb_event_bits |= event;
}

PDM_PCM_FREQ_E pdm_rate_from_cfg() {
  if (cfg::kSampleRateHz == 24000) return PDM_PCM_FREQ_48K;
  if (cfg::kSampleRateHz == 48000) return PDM_PCM_FREQ_48K;
  if (cfg::kSampleRateHz == 32000) return PDM_PCM_FREQ_32K;
  if (cfg::kSampleRateHz == 16000) return PDM_PCM_FREQ_16K;
  if (cfg::kSampleRateHz == 8000) return PDM_PCM_FREQ_8K;
  return PDM_PCM_FREQ_48K;
}

bool InitMicCapture() {
  if (cfg::kUseCm55sAudioHelper) {
    return true;
  }
  if (g_mic_initialized) {
    return true;
  }

  hx_drv_scu_set_PB9_pinmux(SCU_PB9_PINMUX_PDM_CLK_13, 1);
  hx_drv_scu_set_PB10_pinmux(SCU_PB10_PINMUX_PDM_DATA0_13, 1);

  g_pdm_dev_info.pdm_cfg.reg_addr = HIMAX_PDM_BASE_ADDR;
  g_pdm_dev_info.pdm_cfg.rx_fifo_threshold = 5;
  g_pdm_dev_info.pdm_cfg.oversampling_ratio = 8;
  g_pdm_dev_info.pdm_cfg.cic_stages = 0;
  g_pdm_dev_info.pdm_cfg.cic_delay = 0;
  g_pdm_dev_info.pdm_cfg.dc_removal = 6;
  g_pdm_dev_info.pdm_cfg.bit_range_shift = cfg::kPdmBitRangeShift;
  g_pdm_dev_info.pdm_cfg.data_in_0_en = PDM_DATA_IN_ENABLE;
  g_pdm_dev_info.pdm_cfg.data_in_1_en = PDM_DATA_IN_DISABLE;
  g_pdm_dev_info.pdm_cfg.data_in_2_en = PDM_DATA_IN_DISABLE;
  g_pdm_dev_info.pdm_cfg.data_in_3_en = PDM_DATA_IN_DISABLE;
  g_pdm_dev_info.pdm_cfg.capture_channel = PDM_CAPTURE_CHANNEL_LEFT_ONLY;
  g_pdm_dev_info.pdm_cfg.dma_ch = PDM_USE_DMA2_CH_0;
  g_pdm_dev_info.pdm_cfg.pdm_clk_src = PDM_CLKSRC_LSCREF;
  g_pdm_dev_info.pdm_cfg.sample_rate = pdm_rate_from_cfg();
  g_pdm_dev_info.pdm_rx_cb = app_pdm_dma_rx_cb;
  g_pdm_dev_info.pdm_err_cb = nullptr;
  g_pdm_dev_info.pdm_clip_cb = nullptr;

  if (hx_drv_pdm_init(&g_pdm_dev_info) != PDM_NO_ERROR) {
    birdnet_log::Write("[birdnet] hx_drv_pdm_init failed\r\n");
    return false;
  }

  if (cfg::kUseHardwareVad) {
    hx_drv_pdm_vad_output(PDM_DATA_OUT_TO_VAD_ENABLE, PDM_DATA_PATH_0_OUT_TO_VAD);
  }

  g_write_idx = 0;
  g_chunk_queue_head = 0;
  g_chunk_queue_tail = 0;
  g_chunk_queue_count = 0;
  g_chunk_queue_overflow_count = 0;
  g_logged_chunk_queue_overflow_count = 0;
  g_audio_ring_write_idx = 0;
  g_audio_ring_fill = 0;
  g_audio_ring_total_samples = 0;
  g_chunk_count = 0;
#if !BIRDNET_SKIP_PCM_ZERO
  std::memset(g_audio_ring, 0, sizeof(g_audio_ring));
#endif
  hx_drv_pdm_dma_lli_transfer((void*)g_dma_buf[g_write_idx], kDmaBlocks, kDmaBlockBytes, 0);
  g_mic_initialized = true;

  birdnet_log::Status("[birdnet] mic started sr=%dHz capture_sr=%dHz decim=%d bit_shift=%d\r\n",
                cfg::kSampleRateHz,
                kPdmCaptureRateHz,
                kDecimation,
                cfg::kPdmBitRangeShift);
  return true;
}

bool InitHardwareVad() {
  if (cfg::kUseCm55sAudioHelper) {
    return true;
  }
  if (!cfg::kUseHardwareVad) {
    return true;
  }
  if (g_vad_initialized) {
    return true;
  }

  VAD_CFG_T vad_cfg{};
  if (hx_drv_vad_get_defCfg(&vad_cfg) != VAD_NO_ERROR) {
    birdnet_log::Write("[birdnet] hx_drv_vad_get_defCfg failed\r\n");
    return false;
  }

  vad_cfg.vad_src = VAD_SRC_PDM;
  vad_cfg.vad_ch = VAD_PCM_DATA_L_CH;
  vad_cfg.pdm_data_path_out = PDM_DATA_PATH_0_OUT_TO_VAD;
  vad_cfg.input_gain = static_cast<uint8_t>(cfg::kVadInputGain);
  vad_cfg.speech_gain = static_cast<uint8_t>(cfg::kVadSpeechGain);
  vad_cfg.noise_gain = static_cast<uint8_t>(cfg::kVadNoiseGain);
  vad_cfg.k_noise = static_cast<uint16_t>(cfg::kVadKNoise);
  vad_cfg.high_pass = cfg::kVadBypassHighPass ? VAD_HIGH_PASS_FILTER_BYPASS
                                              : VAD_HIGH_PASS_FILTER_CUTOFF_1K;

  if (hx_drv_vad_init(HW_VAD_BASEADDR, &vad_cfg) != VAD_NO_ERROR) {
    birdnet_log::Write("[birdnet] hx_drv_vad_init failed\r\n");
    return false;
  }
  hx_drv_vad_set_gain(vad_cfg.input_gain, vad_cfg.speech_gain, vad_cfg.noise_gain);
  hx_drv_vad_set_datapath(vad_cfg.high_pass, vad_cfg.lut, vad_cfg.inverse);
  hx_drv_vad_set_k_noise(vad_cfg.k_noise);
  if (hx_drv_vad_register_cb(app_vad_cb) != VAD_NO_ERROR) {
    birdnet_log::Write("[birdnet] hx_drv_vad_register_cb failed\r\n");
    return false;
  }
  if (hx_drv_vad_set_enable(1) != VAD_NO_ERROR) {
    birdnet_log::Write("[birdnet] hx_drv_vad_set_enable failed\r\n");
    return false;
  }

  g_vad_initialized = true;
  birdnet_log::Status(
      "[birdnet] hw vad enabled input_gain=%d speech_gain=%d noise_gain=%d k_noise=%d high_pass=%d\r\n",
      cfg::kVadInputGain,
      cfg::kVadSpeechGain,
      cfg::kVadNoiseGain,
      cfg::kVadKNoise,
      cfg::kVadBypassHighPass ? 0 : 2);
  return true;
}


bool ParkGroveI2c() {
  // Release both lines before selecting GPIO; external pull-ups hold them high.
  // Never drive the Grove I2C lines low during sleep.
  if (g_i2c_initialized && hx_drv_i2cm_deinit(USE_DW_IIC_0) != IIC_ERR_OK) return false;
  g_i2c_initialized = false;
  if (hx_drv_gpio_init(GPIO_GROUP_4, HX_GPIO_GROUP_4_BASE) != GPIO_NO_ERROR ||
      hx_drv_gpio_set_input(SB_GPIO0) != GPIO_NO_ERROR ||
      hx_drv_gpio_set_input(SB_GPIO1) != GPIO_NO_ERROR ||
      hx_drv_scu_set_PA2_pinmux(SCU_PA2_PINMUX_SB_GPIO0, 1) != 0 ||
      hx_drv_scu_set_PA3_pinmux(SCU_PA3_PINMUX_SB_GPIO1, 1) != 0) return false;
  __DSB();
  return true;
}

bool EnterAudioOnlyPrerollingMode() {
  if (!cfg::kUseAudioOnlyPrerolling) {
    return false;
  }

  if (cfg::kGateXipBetweenWindows && !SetModelXipEnabled(false)) {
    birdnet_log::Write("[birdnet] XIP disable before preroll failed\r\n");
    return false;
  }

  ForceVideoPathOff();

  PM_PD_AUDPRE_CFG_T pm_cfg{};
  if constexpr (cfg::kPmuFastWakeClock) {
    pm_cfg.bootromspeed.bootromclkfreq = SCU_PLL_FREQ_ENABLE;
    pm_cfg.bootromspeed.pll_freq = cfg::kPmuFastWakePllHz;
    pm_cfg.bootromspeed.cm55m_div = SCU_HSCCLKDIV_1;
    pm_cfg.bootromspeed.cm55s_div = LscClockDivFromInt(cfg::kPmuWakeCm55sDiv);
  } else {
    pm_cfg.bootromspeed.bootromclkfreq = SCU_PLL_FREQ_DISABLE;
    pm_cfg.bootromspeed.pll_freq = 0;
    pm_cfg.bootromspeed.cm55m_div = SCU_HSCCLKDIV_1;
    pm_cfg.bootromspeed.cm55s_div = LscClockDivFromInt(cfg::kPmuWakeCm55sDiv);
  }
  pm_cfg.cm55s_reset = SWREG_AON_PMUWAKE_CM55S_RERESET_NO;
  pm_cfg.pmu_pad_pa01_mask = PM_IP_INT_RESERVED;
  pm_cfg.pmu_rtc_mask = PM_RTC_INT_MASK_RESERVED;
  pm_cfg.pmu_pad_pa23_mask = PM_IP_INT_RESERVED;
  pm_cfg.pmu_i2cw_mask = PM_IP_INT_RESERVED;
  pm_cfg.pmu_timer_mask = PM_TIMER_INT_MASK_TIMER_RESERVED;
  pm_cfg.pmu_cmp_mask = PM_IP_INT_RESERVED;
  pm_cfg.pmu_ts_mask = PM_IP_INT_RESERVED;
  pm_cfg.pmu_senint_mask = PM_IP_INT_RESERVED;
  pm_cfg.support_debugdump = 0;
  pm_cfg.tcm_retention = PM_MEM_RET_YES;
  for (uint32_t bank = 0; bank < 4; ++bank) {
    pm_cfg.hscsram_retention[bank] =
        (cfg::kPmuHscSramRetentionMask & (1u << bank)) ? PM_MEM_RET_YES : PM_MEM_RET_NO;
  }
  pm_cfg.lscsram_retention = PM_MEM_RET_YES;
  pm_cfg.skip_bootflow.sec_mem_flag = SWREG_AON_RETENTION;
  pm_cfg.skip_bootflow.first_bl_flag = SWREG_AON_RETENTION;
  pm_cfg.skip_bootflow.cm55m_s_app_flag = SWREG_AON_RETENTION;
  pm_cfg.skip_bootflow.cm55m_ns_app_flag = SWREG_AON_RETENTION;
  pm_cfg.skip_bootflow.cm55s_s_app_flag = SWREG_AON_RETENTION;
  pm_cfg.skip_bootflow.cm55s_ns_app_flag = SWREG_AON_RETENTION;
  pm_cfg.skip_bootflow.cm55m_model_flag = SWREG_AON_RETENTION;
  pm_cfg.skip_bootflow.cm55s_model_flag = SWREG_AON_RETENTION;
  pm_cfg.skip_bootflow.cm55m_appcfg_flag = SWREG_AON_RETENTION;
  pm_cfg.skip_bootflow.cm55s_appcfg_flag = SWREG_AON_RETENTION;
  pm_cfg.skip_bootflow.cm55m_s_app_rwdata_flag = SWREG_AON_RETENTION;
  pm_cfg.skip_bootflow.cm55m_ns_app_rwdata_flag = SWREG_AON_RETENTION;
  pm_cfg.skip_bootflow.cm55s_s_app_rwdata_flag = SWREG_AON_RETENTION;
  pm_cfg.skip_bootflow.cm55s_ns_app_rwdata_flag = SWREG_AON_RETENTION;
  pm_cfg.pmu_anti_mask = PM_IP_INT_MASK_ALL_UNMASK;
  pm_cfg.pmu_dcdc_outpin = PM_CFG_DCDC_MODE_OFF;

  if (hx_lib_pm_cfg_set(&pm_cfg, nullptr, PM_MODE_PS_AUD_ONLY_PREROLLING) != PM_NO_ERROR) {
    birdnet_log::Write("[birdnet] aud preroll cfg failed\r\n");
    return false;
  }

  SCU_LSC_CLK_CFG_T lsc_cfg{};
  SCU_PDHSC_HSCCLK_CFG_T hsc_cfg{};
  hsc_cfg.hscclk.hscclksrc = SCU_HSCCLKSRC_RC24M1M;
  hsc_cfg.hscclk.hscclkdiv = SCU_HSCCLKDIV_1;
  hsc_cfg.hscd12clksrc = SCU_HSCD12CLKSRC_HSC;
  hsc_cfg.i3chcdiv = SCU_HSCI3CHCLKDIV_1;
  hsc_cfg.sdiodiv = SCU_HSCSDIOCLKDIV_1;
  lsc_cfg.lscclksrc = SCU_LSCCLKSRC_RC24M1M;
  lsc_cfg.lscclkdiv = LscClockDivFromInt(cfg::kPmuIdleLscDiv);

  birdnet_log::Status("[birdnet] entering audio-only prerolling lsc_div=%d wake_s_div=%d byapp=%d hsc_ret=0x%lx\r\n",
                cfg::kPmuIdleLscDiv,
                cfg::kPmuWakeCm55sDiv,
                cfg::kPmuClockCtrlByApp ? 1 : 0,
                static_cast<unsigned long>(cfg::kPmuHscSramRetentionMask));
  if (!ParkGroveI2c()) { birdnet_log::Write("[bnio-error] park failed\r\n"); return false; }
  if (g_console_uart) g_console_uart->uart_control(UART_CMD_FLUSH_OUTPUT, nullptr);
  const PM_CLK_PARA_CTRL_E clock_ctrl = cfg::kPmuClockCtrlByApp
                                            ? PM_CLK_PARA_CTRL_BYAPP
                                            : PM_CLK_PARA_CTRL_BYPMLIB;
  if constexpr (cfg::kUseDualCoreFixedCadence) {
    hx_drv_mb_set_sharedata(BN_MB_SLEEP_READY, 1);
    __DMB();
    hx_drv_mb_trigger_cm55s_irq(BN_MB_SLEEP_EVENT);
  }
  // DC watchdog must not run across the PMU clock/power transition. Each warm
  // entry arms it again; the capture core remains responsible for waking us.
  hx_drv_watchdog_cm55m_stop();
  if (hx_lib_pm_trigger(hsc_cfg, lsc_cfg, clock_ctrl) != PM_NO_ERROR) {
    (void)ArmActiveWatchdog(9);
    if constexpr (cfg::kUseDualCoreFixedCadence) {
      hx_drv_mb_set_sharedata(BN_MB_SLEEP_READY, 0);
    }
    birdnet_log::Write("[birdnet] aud preroll trigger failed\r\n");
    return false;
  }
  return true;
}

int ReadMicSamples(int16_t* out_samples, int max_samples) {
  if (cfg::kUseDualCoreAudioPreroll) {
    if (!out_samples || max_samples <= 0 || g_mb_ready_bytes < sizeof(int16_t)) {
      return 0;
    }
    const int available_samples =
        static_cast<int>((g_mb_ready_bytes - g_mb_audio_read_offset) / sizeof(int16_t));
    if (available_samples <= 0) {
      g_mb_ready_bytes = 0;
      g_mb_audio_read_offset = 0;
      return 0;
    }
    const int to_copy = (max_samples < available_samples) ? max_samples : available_samples;
    std::memcpy(out_samples,
                g_mb_audio_buf + g_mb_audio_read_offset,
                static_cast<size_t>(to_copy) * sizeof(int16_t));
    g_mb_audio_read_offset += static_cast<uint32_t>(to_copy * sizeof(int16_t));
    if (g_mb_audio_read_offset >= g_mb_ready_bytes) {
      g_mb_ready_bytes = 0;
      g_mb_audio_read_offset = 0;
    }
    return to_copy;
  }
  if (!out_samples || max_samples < kOutputChunkSamples) {
    return 0;
  }

  uint32_t primask = SaveAndDisableIrq();
  if (g_chunk_queue_count == 0) {
    RestoreIrq(primask);
    return 0;
  }
  const uint32_t head = g_chunk_queue_head;
  std::memcpy(out_samples, g_chunk_queue[head], static_cast<size_t>(kOutputChunkSamples) * sizeof(int16_t));
  g_chunk_queue_head = (head + 1) % static_cast<uint32_t>(kChunkQueueSlots);
  --g_chunk_queue_count;
  RestoreIrq(primask);

  ++g_chunk_count;
  if (cfg::kEnableDiagnosticLogs && g_chunk_count <= 3) {
    birdnet_log::Write("[birdnet] dma chunk=%llu first=%d,%d,%d,%d\r\n",
            static_cast<unsigned long long>(g_chunk_count),
            static_cast<int>(out_samples[0]),
            static_cast<int>(out_samples[1]),
            static_cast<int>(out_samples[2]),
            static_cast<int>(out_samples[3]));
  }
  return kOutputChunkSamples;
}

void ResetMailboxWindowState() {
  g_mb_window_fill_samples = 0;
  g_mb_window_ready = false;
}

bool CopyFixedMailboxWindow() {
  const uint64_t copy_started_ms = PlatformMillis();
  BirdnetAudioDescriptor d{};
  uint32_t magic = 0, live = 0, sequence_after = 0, fault = 0;
  hx_drv_mb_get_sharedata(BN_MB_MAGIC, &magic);
  hx_drv_mb_get_sharedata(BN_MB_SEQUENCE, &d.sequence);
  __DMB();
  hx_drv_mb_get_sharedata(BN_MB_RING_BASE, &d.ring_base);
  hx_drv_mb_get_sharedata(BN_MB_RING_SAMPLES, &d.ring_samples);
  hx_drv_mb_get_sharedata(BN_MB_START, &d.start);
  hx_drv_mb_get_sharedata(BN_MB_SAMPLES, &d.samples);
  hx_drv_mb_get_sharedata(BN_MB_END_LOW, &d.end_low);
  hx_drv_mb_get_sharedata(BN_MB_END_HIGH, &d.end_high);
  hx_drv_mb_get_sharedata(BN_MB_CAPTURE_GENERATION, &d.capture_generation);
  hx_drv_mb_get_sharedata(BN_MB_LIVE_GENERATION, &live);
  hx_drv_mb_get_sharedata(BN_MB_FAULT, &fault);
  __DMB();
  hx_drv_mb_get_sharedata(BN_MB_SEQUENCE, &sequence_after);
  bool valid = magic == BN_MB_MAGIC_VALUE && fault == 0 &&
               d.sequence == sequence_after &&
               birdnet_audio_descriptor_valid(&d, cfg::kWindowSamples, live) &&
               birdnet_audio_end(&d) > g_retained_runtime.last_capture_end;
  uint32_t dma_ahead=UINT32_MAX;
  if (valid) {
    dma_ahead=birdnet_circular_ahead(d.ring_base,d.ring_samples,d.start,d.samples,
        *(volatile const uint32_t *)0x51800104u);
    valid=dma_ahead!=UINT32_MAX;
  }
  if (valid) {
    const auto* ring = reinterpret_cast<const int16_t*>(d.ring_base);
    const uint32_t first = birdnet_audio_first_samples(&d);
    SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t*>(d.ring_base),
                               d.ring_samples * sizeof(int16_t));
    std::memcpy(g_audio_ring, ring + d.start, first * sizeof(int16_t));
    std::memcpy(g_audio_ring + first, ring, (d.samples - first) * sizeof(int16_t));
    __DMB();
    hx_drv_mb_get_sharedata(BN_MB_LIVE_GENERATION, &live);
    hx_drv_mb_get_sharedata(BN_MB_SEQUENCE, &sequence_after);
    const uint32_t after=birdnet_circular_ahead(d.ring_base,d.ring_samples,d.start,d.samples,
        *(volatile const uint32_t *)0x51800104u);
    valid = live == d.capture_generation && sequence_after == d.sequence &&
        after!=UINT32_MAX && after>=dma_ahead && PlatformMillis()-copy_started_ms<200u;
  }
  hx_drv_mb_set_sharedata(BN_MB_ACK, d.sequence);
  __DMB();
  hx_drv_mb_trigger_cm55s_irq(BN_MB_ACK_EVENT);
  if (!valid) {
    ++g_retained_runtime.rejected_windows;
    birdnet_log::Status("[birdnet] rejected stale/invalid window seq=%lu generation=%lu/%lu\r\n",
                  static_cast<unsigned long>(d.sequence),
                  static_cast<unsigned long>(d.capture_generation),
                  static_cast<unsigned long>(live));
    return false;
  }
  const uint64_t end = birdnet_audio_end(&d);
  g_retained_runtime.skipped_windows += static_cast<uint32_t>(
      birdnet_audio_skipped_windows(g_retained_runtime.last_capture_end, end,
                                    cfg::kWindowSamples, cfg::kInferenceStepSamples));
  g_retained_runtime.last_capture_end = end;
  g_audio_ring_total_samples = end;
  g_mb_window_count = RetainedAdvanceFixedWindowCount();
  g_audio_ring_fill = g_mb_window_fill_samples = d.samples;
  g_audio_ring_write_idx = 0;
  g_mb_window_ready = true;
  birdnet_log::Status("[birdnet] capture end=%llu seq=%lu skipped=%lu rejected=%lu copy_ms=%lu\r\n",
                static_cast<unsigned long long>(end), static_cast<unsigned long>(d.sequence),
                static_cast<unsigned long>(g_retained_runtime.skipped_windows),
                static_cast<unsigned long>(g_retained_runtime.rejected_windows),
                static_cast<unsigned long>(PlatformMillis() - copy_started_ms));
  return true;
}

bool CopyMailboxBufferToWindow(uint8_t buf_no) {
  if (g_mb_window_ready || g_mb_window_fill_samples >= static_cast<uint32_t>(cfg::kWindowSamples)) {
    return false;
  }

  uint32_t copy_size = 0;
  int16_t* dst = g_audio_ring + g_mb_window_fill_samples;
  if (hx_lib_mb_copy_vad_data(buf_no,
                              reinterpret_cast<uint32_t>(dst),
                              &copy_size) != MBLIB_ERR_NO_ERROR) {
    birdnet_log::Write("[birdnet] mailbox audio copy failed buf=%u\r\n",
            static_cast<unsigned int>(buf_no));
    ResetMailboxWindowState();
    return false;
  }

  if ((copy_size & 1u) != 0u) {
    birdnet_log::Write("[birdnet] mailbox audio odd copy size=%lu\r\n",
            static_cast<unsigned long>(copy_size));
    ResetMailboxWindowState();
    return false;
  }

  const uint32_t copied_samples = copy_size / static_cast<uint32_t>(sizeof(int16_t));
  if (g_mb_window_fill_samples + copied_samples > static_cast<uint32_t>(cfg::kWindowSamples)) {
    birdnet_log::Write("[birdnet] mailbox audio overflow fill=%lu add=%lu window=%d\r\n",
            static_cast<unsigned long>(g_mb_window_fill_samples),
            static_cast<unsigned long>(copied_samples),
            cfg::kWindowSamples);
    ResetMailboxWindowState();
    return false;
  }

  g_mb_window_fill_samples += copied_samples;
  return true;
}

int RunMinimalDualCoreBringupDebug() {
  if (hx_lib_mb_cm55m_init(app_mb_cb) != MBLIB_ERR_NO_ERROR) {
    birdnet_log::Write("[birdnet] cm55m mailbox init failed; halting\r\n");
    while (1) {}
  }

  hx_drv_scu_set_cm55s_state(SCU_CM55S_STATE_RESET);
  hx_drv_scu_set_cm55s_state(SCU_CM55S_STATE_NORMAL);
  hx_drv_scu_set_CM55S_CPUWAIT(SCU_CM55_CPU_RUN);

  uint64_t last_ping_ms = 0;
  birdnet_log::Write("[birdnet] minimal dual-core bringup debug\r\n");
  while (1) {
    const uint64_t now = PlatformMillis();
    if (now - last_ping_ms >= 1000u) {
      uint32_t cm55s_mark = 0;
      SCU_CM55S_STATE_E cm55s_state = SCU_CM55S_STATE_RESET;
      hx_lib_mb_set_cm55m_dc_rdy();
      hx_drv_swreg_aon_get_appused1(&cm55s_mark);
      if (hx_drv_scu_get_cm55s_state(&cm55s_state) == SCU_NO_ERROR) {
        birdnet_log::Write("[birdnet] minimal wait state=%d ev=0x%08lx mark=0x%08lx\r\n",
                static_cast<int>(cm55s_state),
                static_cast<unsigned long>(g_mb_event_bits),
                static_cast<unsigned long>(cm55s_mark));
      } else {
        birdnet_log::Write("[birdnet] minimal wait state=<err> ev=0x%08lx mark=0x%08lx\r\n",
                static_cast<unsigned long>(g_mb_event_bits),
                static_cast<unsigned long>(cm55s_mark));
      }
      last_ping_ms = now;
    }

    if (g_mb_event_bits != 0) {
      const uint32_t events = TakeMailboxEvents();
      birdnet_log::Write("[birdnet] minimal events=0x%08lx\r\n",
              static_cast<unsigned long>(events));
    }
  }
}

bool WaitForCm55sReady(bool cold_boot) {
  birdnet_log::Status("[birdnet] cm55m mailbox init begin cold=%d\r\n", cold_boot ? 1 : 0);
  if (hx_lib_mb_cm55m_init(app_mb_cb) != MBLIB_ERR_NO_ERROR) {
    birdnet_log::Write("[birdnet] cm55m mailbox init failed; halting\r\n");
    return false;
  }
  birdnet_log::Status("[birdnet] cm55m mailbox init ok cold=%d\r\n", cold_boot ? 1 : 0);

  if (!cold_boot) {
    if constexpr (cfg::kUseDualCoreFixedCadence) {
      hx_drv_mb_set_sharedata(BN_MB_SLEEP_READY, 0);
    }
    birdnet_log::Status("[birdnet] resumed mailbox: advertise cm55m dc begin\r\n");
    hx_lib_mb_set_cm55m_dc_rdy();
    birdnet_log::Status("[birdnet] resumed dual-core mailbox path\r\n");
    return true;
  }

  if (cfg::kEnableStatusLogs) {
    uint32_t cm55s_mark = 0;
    hx_drv_swreg_aon_get_appused1(&cm55s_mark);
    birdnet_log::Write("[birdnet] cm55s mark before bringup=0x%08lx\r\n",
            static_cast<unsigned long>(cm55s_mark));
  }
  hx_drv_scu_set_cm55s_state(SCU_CM55S_STATE_RESET);
  hx_drv_scu_set_cm55s_state(SCU_CM55S_STATE_NORMAL);
  hx_drv_scu_set_CM55S_CPUWAIT(SCU_CM55_CPU_RUN);
  birdnet_log::Status("[birdnet] dual-core audio preroll requested\r\n");

  uint64_t last_ping_ms = 0;
  uint64_t next_state_log_ms = PlatformMillis();
  while (true) {
    const uint64_t now = PlatformMillis();
    if (now - last_ping_ms >= 500u) {
      hx_lib_mb_set_cm55m_dc_rdy();
      last_ping_ms = now;
    }

    if (g_mb_event_bits != 0) {
      const uint32_t events = TakeMailboxEvents();
      if (events & MBLIB_CM55BIG_EVT_CM55S_RDY) {
        if (cfg::kEnableStatusLogs) {
          birdnet_log::Write("[birdnet] cm55s ready\r\n");
        }
        return true;
      }
      if (cfg::kEnableStatusLogs) {
        birdnet_log::Write("[birdnet] pre-ready events=0x%08lx\r\n",
                static_cast<unsigned long>(events));
      }
    }

    if (cfg::kEnableStatusLogs && now >= next_state_log_ms) {
      SCU_CM55S_STATE_E cm55s_state = SCU_CM55S_STATE_RESET;
      uint32_t cm55s_mark = 0;
      hx_drv_swreg_aon_get_appused1(&cm55s_mark);
      if (hx_drv_scu_get_cm55s_state(&cm55s_state) == SCU_NO_ERROR) {
        birdnet_log::Write("[birdnet] waiting cm55s state=%d ev=0x%08lx\r\n",
                static_cast<int>(cm55s_state),
                static_cast<unsigned long>(g_mb_event_bits));
        birdnet_log::Write("[birdnet] cm55s mark=0x%08lx\r\n",
                static_cast<unsigned long>(cm55s_mark));
      } else {
        birdnet_log::Write("[birdnet] waiting cm55s state=<err> ev=0x%08lx\r\n",
                static_cast<unsigned long>(g_mb_event_bits));
      }
      next_state_log_ms = now + 1000u;
    }
  }
}

