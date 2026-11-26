#include "birdnet_format.h"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <limits>

#include "birdnet_labels.h"
#include "birdnet_audio_mailbox.h"
#include "birdnet_storage_policy.h"
#include "birdnet_detection_policy.h"
#include "birdnet_file_service.h"
#include "birdnet_buffered_writer.h"
#include "birdnet_ima_adpcm.h"
#include "birdnet_rice_codec.h"
#include "birdnet_opus_codec.h"
#include "birdnet_codec_loader.h"
#include "birdnet_model_contract.h"
#include "birdnet_flash_labels.h"
#include "birdnet_lora.h"
#include "birdnet_oled.h"
#include "birdnet_battery.h"
#include "config.h"
#include "inference_pipeline.h"
#include "birdnet_log.h"

extern "C" {
#include "WE2_device.h"
#include "WE2_core.h"
#include "board.h"
#include "console_io.h"
#include "hxmb.h"
#include "powermode.h"
#include "WE2_device_addr.h"
#include "hx_drv_gpio.h"
#include "hx_drv_iic.h"
#include "hx_drv_pdm_rx.h"
#include "hx_drv_pmu.h"
#include "hx_drv_scu.h"
#include "hx_drv_swreg_aon.h"
#include "hx_drv_uart.h"
#include "hx_drv_vad.h"
#include "hx_drv_watchdog.h"
#include "spi_eeprom_comm.h"
#include "timer_interface.h"
#include "xprintf.h"
#if BIRDNET_ENABLE_SD_DETECTION_LOGGING
#include "Driver_SPI.h"
#include "diskio.h"
#include "ff.h"
#endif
}

#if BIRDNET_ENABLE_SD_DETECTION_LOGGING
extern "C" ARM_DRIVER_SPI Driver_SPI0;

extern "C" void SSPI_CS_GPIO_Output_Level(bool setLevelHigh) {
  hx_drv_gpio_set_out_value(GPIO16, static_cast<GPIO_OUT_LEVEL_E>(setLevelHigh));
}

extern "C" void SSPI_CS_GPIO_Pinmux(bool setGpioFn) {
  if (setGpioFn) {
    hx_drv_scu_set_PB5_pinmux(SCU_PB5_PINMUX_GPIO16, 0);
  } else {
    hx_drv_scu_set_PB5_pinmux(SCU_PB5_PINMUX_SPI_M_CS_1, 0);
  }
}

extern "C" void SSPI_CS_GPIO_Dir(bool setDirOut) {
  if (setDirOut) {
    hx_drv_gpio_set_output(GPIO16, GPIO_OUT_HIGH);
  } else {
    hx_drv_gpio_set_input(GPIO16);
  }
}
#endif

#if BIRDNET_DETECTION_AUDIO_CODEC == 2 && BIRDNET_VERIFY_OPUS_ON_WAKE
extern "C" uint8_t __opus_text_start__[], __opus_text_end__[];
#endif

namespace {

// Private services retain internal linkage and are compiled together so the
// optimizer can eliminate legacy modes without a public cross-module ABI.
#include "birdnet_app_runtime.h"
#include "birdnet_app_platform.h"
#include "birdnet_app_peripherals.h"
#include "birdnet_app_capture.h"
#include "birdnet_app_storage.h"
#include "birdnet_app_export.h"
#include "birdnet_app_window.h"

}  // namespace

#if BIRDNET_LABELS_IN_FLASH
const uint8_t* BirdnetLabelFlashBase() {
  if (g_retained_runtime.labels_verified_fnv != birdnet_contract::kLabelBlobFnv ||
      !SetModelXipEnabled(true)) return nullptr;
  return reinterpret_cast<const uint8_t*>(cfg::kModelFlashAddr + birdnet_contract::kLabelBlobOffset);
}
#endif
extern "C" int app_main(void) {
  birdnet_log::SetSink([](const char* text, size_t bytes) {
    if (g_console_uart) g_console_uart->uart_write(text, static_cast<uint32_t>(bytes));
  });
#if BIRDNET_ENABLE_STATUS_BUTTON
  static_assert(cfg::kUseDualCoreFixedCadence && cfg::kUseAudioOnlyPrerolling,
                "Status button requires the paired fixed-cadence audio helper");
  NVIC_DisableIRQ(GPIO_GROUP_2_IRQn);
#endif
  uint8_t detection_led_before_clear = 0xffu;
  const bool detection_led_before_clear_ok =
      cfg::kEnableDetectionLed &&
      hx_drv_gpio_get_in_value(GPIO20, &detection_led_before_clear) ==
          GPIO_NO_ERROR;

  // Clear a detection indication before mailbox or peripheral initialization.
  // This keeps a retained high GPIO from surviving a failed warm-wake path.
  InitDetectionLed();
  uint8_t detection_led_after_clear = 0xffu;
  const bool detection_led_after_clear_ok =
      cfg::kEnableDetectionLed &&
      hx_drv_gpio_get_in_value(GPIO20, &detection_led_after_clear) ==
          GPIO_NO_ERROR;

  if (cfg::kMinimalDualCoreBringupDebug) {
    g_console_uart = hx_drv_uart_get_dev(USE_DW_UART_0);
    if (g_console_uart) {
      g_console_uart->uart_open(UART_BAUDRATE_921600);
    }
    console_setup(USE_DW_UART_0, UART_BAUDRATE_921600);
    xprintf_setup();
    MaybeMuteVendorXprintf();
    birdnet_log::Status("[birdnet] app start sr=%d window=%d infer_interval_ms=%d infer_step_samples=%d\r\n",
                  cfg::kSampleRateHz,
                  cfg::kWindowSamples,
                  cfg::kInferenceIntervalMs, cfg::kInferenceStepSamples);
    return RunMinimalDualCoreBringupDebug();
  }

  static TFLMInferencePipeline pipeline;
  static_assert(cfg::kInferenceStepSamples > 0,
                "Inference step must be positive");
  const bool defer_local_init = cfg::kUseCm55sAudioHelper && cfg::kDualCoreHandshakeDebug;
  auto finish_local_init = [&]() -> bool {
    birdnet_log::Status("[birdnet] local init: video off begin\r\n");
    ForceVideoPathOff();
    birdnet_log::Status("[birdnet] camera/mipi/isp/dp forced off\r\n");
    InitDetectionLed();
    birdnet_oled::SetEnsureI2cReadyFn(&InitI2cMaster0);
    InitDetectionOled();

    birdnet_log::Status("[birdnet] local init: sht41 begin\r\n");
    if (cfg::kEnableSht41Sensor && !InitSht41()) {
      birdnet_log::Status("[birdnet] SHT41 init unavailable at startup\r\n");
    }
    if constexpr (cfg::kEnableLoraDetectionTx) {
      (void)birdnet_lora::Init(&birdnet_log::Write);
      if (cfg::kLoraTestIntervalMs > 0) {
        (void)birdnet_lora::SendTestPacket(PlatformMillis());
      }
    }
    birdnet_log::Status("[birdnet] local init: audio path begin\r\n");

    if (!cfg::kUseCm55sAudioHelper) {
      if (!InitMicCapture()) {
        birdnet_log::Write("[birdnet] mic init failed; halting\r\n");
        return false;
      }
      if (!InitHardwareVad()) {
        birdnet_log::Write("[birdnet] hw vad init failed; halting\r\n");
        return false;
      }
    } else if (cfg::kEnableStatusLogs) {
      birdnet_log::Status("[birdnet] using CM55S audio mailbox path\r\n");
    }

    birdnet_log::Status("[birdnet] local init: xip begin\r\n");
    RestoreSdkFlashPinmux();
#if BIRDNET_ENABLE_STATUS_BUTTON
    ConfigureStatusButton();
#endif
    if (!SetModelXipEnabled(true)) {
      birdnet_log::Write("[birdnet] model flash init failed; halting\r\n");
      return false;
    }
    birdnet_log::Status("[birdnet] QSPI XIP enabled\r\n");
    RestoreHighSpeedClockAfterPmWakeup();
#if BIRDNET_LABELS_IN_FLASH
    if (g_retained_runtime.labels_verified_fnv != birdnet_contract::kLabelBlobFnv) {
      const auto* blob = reinterpret_cast<const uint8_t*>(cfg::kModelFlashAddr + birdnet_contract::kLabelBlobOffset);
      const bool ok = birdnet_flash_labels::Validate(blob, birdnet_contract::kLabelBlobBytes,
                                                    birdnet_labels::kCount, birdnet_contract::kLabelBlobFnv);
      birdnet_log::Write("[bnlabels] bytes=%lu ok=%u\r\n",
                        (unsigned long)birdnet_contract::kLabelBlobBytes, unsigned(ok));
      if (!ok) return false;
      g_retained_runtime.labels_verified_fnv = birdnet_contract::kLabelBlobFnv;
    }
#endif
    return true;
  };

  const bool resumed_from_preroll = cfg::kUseAudioOnlyPrerolling ? RestoreFromPmWakeup() : false;

  if (cfg::kEnableConsoleUart) {
    g_console_uart = hx_drv_uart_get_dev(USE_DW_UART_0);
    if (g_console_uart) {
      g_console_uart->uart_open(UART_BAUDRATE_921600);
    }
    console_setup(USE_DW_UART_0, UART_BAUDRATE_921600);
    xprintf_setup();
    MaybeMuteVendorXprintf();
  } else {
    g_console_uart = nullptr;
    MaybeMuteVendorXprintf();
    console_desetup(DW_UART_0_ID);
  }

  if (!resumed_from_preroll) {
    birdnet_log::Write("[bnrelease] id=%s model=%s classes=%d threshold_q1e4=%d\r\n",
            birdnet_contract::kId, birdnet_contract::kModelSha256,
            birdnet_contract::kClasses, BIRDNET_SD_LOG_THRESHOLD_Q1E4);
  }
  InitRetainedRuntimeState(resumed_from_preroll);
  SdLogInitRetainedState(resumed_from_preroll);
  if (!resumed_from_preroll) pipeline.ResetStreamingCache();
  if (!ArmActiveWatchdog(1)) {
    birdnet_log::Status("[birdnet] active watchdog unavailable\r\n");
  }
#if BIRDNET_DETECTION_AUDIO_CODEC == 2 && BIRDNET_VERIFY_OPUS_ON_WAKE
  const uint64_t opus_check_start = PlatformMillis();
  if (!VerifyOpusCode(resumed_from_preroll)) {
    g_retained_runtime.phase = 11;
    birdnet_log::Write("[birdnet] Opus SRAM code changed across sleep; awaiting watchdog reset\r\n");
    while (true) __WFI();
  }
  birdnet_log::Status("[birdnet] Opus SRAM code verified ms=%lu\r\n",
                static_cast<unsigned long>(PlatformMillis() - opus_check_start));
#endif

  if constexpr (cfg::kEnableLedDiagnostics) {
    birdnet_log::Write("[birdnet] led_boot before_clear=%d after_clear=%d\r\n",
            detection_led_before_clear_ok
                ? static_cast<int>(detection_led_before_clear)
                : -1,
            detection_led_after_clear_ok
                ? static_cast<int>(detection_led_after_clear)
                : -1);
  }

  birdnet_log::Status("[birdnet] app start sr=%d window=%d infer_interval_ms=%d infer_step_samples=%d\r\n",
                cfg::kSampleRateHz,
                cfg::kWindowSamples,
                cfg::kInferenceIntervalMs, cfg::kInferenceStepSamples);

  if (cfg::kRecoveryFirmware) {
    ForceVideoPathOff();
    birdnet_log::Write("[birdnet] recovery firmware running; model/audio disabled\r\n");
    while (1) {
      __WFI();
    }
  }

  bool local_init_done = false;
  bool cm55s_ready = false;
  if (cfg::kUseCm55sAudioHelper) {
    if (defer_local_init) {
      birdnet_log::Status("[birdnet] handshake-first debug mode\r\n");
    }
    cm55s_ready = WaitForCm55sReady(!resumed_from_preroll);
    if (!cm55s_ready) {
      while (1) {}
    }
    local_init_done = finish_local_init();
    if (!local_init_done) {
      while (1) {}
    }
    if (cfg::kUseAudioOnlyPrerolling && !resumed_from_preroll) {
      EnterAudioOnlyPrerollingMode();
    }
  } else {
    local_init_done = finish_local_init();
    if (!local_init_done) {
      while (1) {}
    }
  }

  uint64_t next_infer_end_sample = static_cast<uint64_t>(cfg::kWindowSamples);
  uint64_t active_until_ms = 0;
  uint64_t audio_only_preroll_deadline_ms =
      PlatformMillis() + static_cast<uint64_t>(cfg::kAudioOnlyPrerollIdleDelayMs);
  uint64_t next_cm55m_dc_advertise_ms = PlatformMillis() + 500u;
  uint64_t next_cm55s_state_log_ms = PlatformMillis();
  uint64_t next_lora_test_tx_ms = PlatformMillis() + static_cast<uint64_t>(cfg::kLoraTestIntervalMs);
  uint32_t activation_infer_count = 0;
  bool inference_active = !cfg::kUseHardwareVad;
#if BIRDNET_ENABLE_STATUS_BUTTON
  bool ui_wake_handled = false;
#endif

  while (1) {
    bool did_work = false;
    const uint64_t now = PlatformMillis();

    const uint32_t overflow_count = g_chunk_queue_overflow_count;
    if (overflow_count != g_logged_chunk_queue_overflow_count) {
      birdnet_log::Write("[birdnet] audio fifo overflow drops=%u\r\n",
              static_cast<unsigned int>(overflow_count));
      g_logged_chunk_queue_overflow_count = overflow_count;
    }

    if (cfg::kUseCm55sAudioHelper && g_mb_event_bits != 0) {
      const uint32_t events = TakeMailboxEvents();
      if constexpr (cfg::kUseDualCoreFixedCadence) {
        uint32_t capture_fault = 0;
        hx_drv_mb_get_sharedata(BN_MB_FAULT, &capture_fault);
        if (capture_fault != 0) {
          g_retained_runtime.phase = 10;
          SetDetectionOutputs(nullptr, nullptr);
          birdnet_log::Status("[birdnet] capture fault=%lu; awaiting watchdog reset\r\n",
                        static_cast<unsigned long>(capture_fault));
          while (true) { __WFI(); }
        }
        if (events & BN_MB_WAKE_EVENT) {
          hx_drv_mb_set_sharedata(BN_MB_SLEEP_READY, 0);
          hx_lib_mb_set_cm55m_dc_rdy();
        }
#if BIRDNET_ENABLE_STATUS_BUTTON
        if (events & BN_MB_UI_EVENT) {
          hx_drv_mb_set_sharedata(BN_MB_SLEEP_READY, BN_MB_ACTIVE);
          HandleStatusUi();
          ui_wake_handled = true;
        }
#endif
      }
      if (events & MBLIB_CM55BIG_EVT_CM55S_RDY) {
        cm55s_ready = true;
        if (!local_init_done) {
          local_init_done = finish_local_init();
          if (!local_init_done) {
            while (1) {}
          }
        }
        if (cfg::kUseAudioOnlyPrerolling) {
          EnterAudioOnlyPrerollingMode();
        }
        if (cfg::kEnableStatusLogs) {
          birdnet_log::Write("[birdnet] cm55s ready\r\n");
        }
      }
      if (events & (MBLIB_CM55BIG_EVT_VAD_BUF1_RDY | MBLIB_CM55BIG_EVT_VAD_BUF2_RDY)) {
        if (cfg::kUseDualCoreFixedCadence) {
          // Fixed-cadence images use the versioned descriptor, never SDK slots.
        } else {
          const uint8_t buf_no =
              (events & MBLIB_CM55BIG_EVT_VAD_BUF1_RDY) ? MBLIB_MUTEX_SHAREDATA_IDX_VADBUF1
                                                        : MBLIB_MUTEX_SHAREDATA_IDX_VADBUF2;
          uint32_t copy_size = 0;
          if (hx_lib_mb_copy_vad_data(buf_no,
                                      reinterpret_cast<uint32_t>(g_mb_audio_buf),
                                      &copy_size) == MBLIB_ERR_NO_ERROR &&
              copy_size <= sizeof(g_mb_audio_buf)) {
            g_mb_ready_bytes = copy_size;
            g_mb_audio_read_offset = 0;
            ++g_vad_irq_count;
          } else if (cfg::kEnableStatusLogs) {
            birdnet_log::Write("[birdnet] mailbox vad copy failed\r\n");
          }
        }
      }
      if (events & MBLIB_CM55BIG_EVT_CM55S_AUDIO_STOP) {
        if (cfg::kUseDualCoreFixedCadence) {
          if (!CopyFixedMailboxWindow()) {
            ResetMailboxWindowState();
            EnterAudioOnlyPrerollingMode();
          }
        } else if (cfg::kUseAudioOnlyPrerolling) {
          EnterAudioOnlyPrerollingMode();
        }
        if (cfg::kEnableStatusLogs) {
          birdnet_log::Write("[birdnet] cm55s audio stop; re-arming\r\n");
        }
      }
      did_work = true;
    }

    if (cfg::kUseCm55sAudioHelper && !cm55s_ready && now >= next_cm55m_dc_advertise_ms) {
      if (cfg::kEnableStatusLogs && now >= next_cm55s_state_log_ms) {
        SCU_CM55S_STATE_E cm55s_state = SCU_CM55S_STATE_RESET;
        if (hx_drv_scu_get_cm55s_state(&cm55s_state) == SCU_NO_ERROR) {
          uint32_t cm55s_mark = 0;
          hx_drv_swreg_aon_get_appused1(&cm55s_mark);
          birdnet_log::Write("[birdnet] waiting cm55s state=%d ev=0x%08lx\r\n",
                  static_cast<int>(cm55s_state),
                  static_cast<unsigned long>(g_mb_event_bits));
          birdnet_log::Write("[birdnet] cm55s mark=0x%08lx\r\n",
                  static_cast<unsigned long>(cm55s_mark));
        } else {
          birdnet_log::Write("[birdnet] waiting cm55s state=<err> ev=0x%08lx\r\n",
                  static_cast<unsigned long>(g_mb_event_bits));
        }
        next_cm55s_state_log_ms = now + 1000u;
      }
      hx_lib_mb_set_cm55m_dc_rdy();
      next_cm55m_dc_advertise_ms = now + 500u;
    }

    if (cfg::kUseCm55sAudioHelper && !local_init_done) {
      continue;
    }

    if constexpr (cfg::kEnableLoraDetectionTx) {
      if (cfg::kLoraTestIntervalMs > 0 && now >= next_lora_test_tx_ms) {
        (void)birdnet_lora::SendTestPacket(now);
        next_lora_test_tx_ms = now + static_cast<uint64_t>(cfg::kLoraTestIntervalMs);
        did_work = true;
      }
    }

    if (cfg::kUseDualCoreFixedCadence && g_mb_window_ready) {
      ProcessCapturedWindow(pipeline);
      did_work = true;
    }

    if constexpr (cfg::kUseDualCoreFixedCadence) {
#if BIRDNET_ENABLE_STATUS_BUTTON
      // The helper publishes any due audio BEFORE its UI event. Drain all
      // mailbox IRQs first, then return a UI-only wake directly to sleep.
      if (ui_wake_handled && !g_mb_window_ready && g_mb_event_bits == 0) {
        ui_wake_handled = false;
        EnterAudioOnlyPrerollingMode();
      }
#endif
      const uint32_t primask = SaveAndDisableIrq();
      if (g_mb_event_bits == 0) {
        __DSB();
        __WFI();
      }
      RestoreIrq(primask);
      continue;
    }

    if (cfg::kUseHardwareVad && g_vad_handled_count != g_vad_irq_count) {
      g_vad_handled_count = g_vad_irq_count;
      if ((g_last_vad_ms == 0) ||
          (now - g_last_vad_ms >= static_cast<uint64_t>(cfg::kVadRetriggerDebounceMs))) {
        g_last_vad_ms = now;
        active_until_ms = now + static_cast<uint64_t>(cfg::kVadActiveHoldMs);
        activation_infer_count = 0;
        if (!inference_active) {
          inference_active = true;
          if (cfg::kEnableStatusLogs) {
            birdnet_log::Write("[birdnet] active: vad trigger=%lu samples=%llu\r\n",
                    static_cast<unsigned long>(g_vad_handled_count),
                    static_cast<unsigned long long>(g_audio_ring_total_samples));
          }
        }
        audio_only_preroll_deadline_ms =
            active_until_ms + static_cast<uint64_t>(cfg::kAudioOnlyPrerollIdleDelayMs);
        if (AudioRingReady() && g_audio_ring_total_samples >= static_cast<uint64_t>(cfg::kWindowSamples)) {
          next_infer_end_sample = g_audio_ring_total_samples;
        } else {
          next_infer_end_sample = static_cast<uint64_t>(cfg::kWindowSamples);
        }
      }
    }

    if (cfg::kUseHardwareVad && inference_active && now >= active_until_ms) {
      inference_active = false;
      audio_only_preroll_deadline_ms =
          now + static_cast<uint64_t>(cfg::kAudioOnlyPrerollIdleDelayMs);
      if (cfg::kEnableStatusLogs) {
        birdnet_log::Write("[birdnet] idle: hold expired samples=%llu\r\n",
                static_cast<unsigned long long>(g_audio_ring_total_samples));
      }
    }

    if (!cfg::kUseDualCoreFixedCadence) {
      int16_t chunk[kOutputChunkSamples];
      const int n = ReadMicSamples(chunk, kOutputChunkSamples);
      if (n > 0) {
        PushAudioRingSamples(chunk, n);

        // In fixed-cadence mode, pace inferences by captured audio rather than
        // by "last result time + interval".
        if ((!cfg::kUseHardwareVad || inference_active) &&
            AudioRingReady() &&
            g_audio_ring_total_samples >= next_infer_end_sample) {
          const size_t window_start_idx = CurrentWindowStartIdx();
          InferenceResult result{};
          if (!SetModelXipEnabled(true)) {
            birdnet_log::Write("[birdnet] XIP enable before inference failed\r\n");
            while (1) {}
          }
#if BIRDNET_ENABLE_SD_DETECTION_LOGGING
          pipeline.SetRecordingWindow(g_audio_ring_total_samples,
                                      g_sd_log_state.last_detection_end_samples);
#endif
          if (pipeline.RunFromRing(g_audio_ring, cfg::kWindowSamples, window_start_idx, &result)) {
            const uint64_t audio_ts_ms = AudioEndTimestampMs();
            result.timestamp_ms = audio_ts_ms;
            Sht41Reading sht41{};
            ReadSht41(&sht41);
            ReadBattery();
            LogResult(result, sht41);
            SdLogCaptureDetections(result, audio_ts_ms);
            SdLogCaptureEnvironment(sht41, audio_ts_ms);
            const auto recording = birdnet_detection::ForRecording(result, birdnet_labels::CanTriggerDetectionAudio);
            (void)SdSaveDetectionAudio(recording,
                                       audio_ts_ms,
                                       g_audio_ring,
                                       cfg::kWindowSamples,
                                       window_start_idx,
                                       cfg::kWindowSamples);
            if constexpr (cfg::kEnableLoraDetectionTx) {
              (void)birdnet_lora::SendDetection(result);
            }
            SdLogFlush(audio_ts_ms, false);
            SdLogPowerDown();
            SetDetectionOutputs(&result, &sht41);
            if (cfg::kUseHardwareVad && cfg::kMaxInferencesPerActivation > 0) {
              ++activation_infer_count;
              if (activation_infer_count >= static_cast<uint32_t>(cfg::kMaxInferencesPerActivation)) {
                inference_active = false;
                audio_only_preroll_deadline_ms =
                    result.timestamp_ms + static_cast<uint64_t>(cfg::kAudioOnlyPrerollIdleDelayMs);
                if (cfg::kEnableStatusLogs) {
                  birdnet_log::Write("[birdnet] idle: sparse-limit reached samples=%llu\r\n",
                          static_cast<unsigned long long>(g_audio_ring_total_samples));
                }
              }
            }
          } else {
            SetDetectionOutputs(nullptr, nullptr);
            birdnet_log::Write("[birdnet] inference failed\r\n");
          }
          next_infer_end_sample += static_cast<uint64_t>(cfg::kInferenceStepSamples);
        }
        did_work = true;
      }
    }

    if (cfg::kUseAudioOnlyPrerolling &&
        cfg::kUseHardwareVad &&
        !inference_active &&
        now >= audio_only_preroll_deadline_ms) {
      if (EnterAudioOnlyPrerollingMode()) {
        audio_only_preroll_deadline_ms =
            now + static_cast<uint64_t>(cfg::kAudioOnlyPrerollIdleDelayMs);
      } else {
        audio_only_preroll_deadline_ms =
            now + static_cast<uint64_t>(cfg::kAudioOnlyPrerollIdleDelayMs);
      }
      did_work = true;
    }

    if (!did_work) {
      __WFI();
    }
  }
}
