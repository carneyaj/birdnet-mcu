// Private application implementation. Included inside birdnet_we2.cpp
// after SDK headers; not a standalone public header.
#pragma once
#define BIRDNET_AUDIO_SRAM0_BSS __attribute__((used, section(".bss.audio_sram0")))
#define BIRDNET_NOINIT __attribute__((used, section(".noinit")))
#if BIRDNET_DUAL_CORE_FIXED_CADENCE
#define BIRDNET_LEGACY_AUDIO_BSS
#else
#define BIRDNET_LEGACY_AUDIO_BSS BIRDNET_AUDIO_SRAM0_BSS
#endif

constexpr int kPdmCaptureRateHz = (cfg::kSampleRateHz == 24000) ? 48000 : cfg::kSampleRateHz;
static_assert((kPdmCaptureRateHz % cfg::kSampleRateHz) == 0, "PDM capture rate must divide to model sample rate");
constexpr int kDecimation = kPdmCaptureRateHz / cfg::kSampleRateHz;
static_assert(kDecimation >= 1, "Invalid audio decimation");
constexpr int kDmaBlocks = 4;
constexpr int kDmaBlockBytes = 1024;
constexpr int kDmaTotalBytes = kDmaBlocks * kDmaBlockBytes;
constexpr int kRawChunkSamples = kDmaTotalBytes / static_cast<int>(sizeof(int16_t));
constexpr int kOutputChunkSamples = kRawChunkSamples / kDecimation;
constexpr int kChunkQueueSlots = 32;

alignas(32) int16_t g_dma_buf[2][kRawChunkSamples] BIRDNET_LEGACY_AUDIO_BSS;
volatile int g_write_idx = 0;
alignas(32) int16_t g_chunk_queue[kChunkQueueSlots][kOutputChunkSamples] BIRDNET_LEGACY_AUDIO_BSS;
#if BIRDNET_SKIP_PCM_ZERO
// Every path waits for a complete capture or validates a complete mailbox copy.
alignas(32) int16_t g_audio_ring[cfg::kWindowSamples] __attribute__((section(".scratch_audio.pcm")));
#else
alignas(32) int16_t g_audio_ring[cfg::kWindowSamples] BIRDNET_AUDIO_SRAM0_BSS;
#endif
volatile uint32_t g_chunk_queue_head = 0;
volatile uint32_t g_chunk_queue_tail = 0;
volatile uint32_t g_chunk_queue_count = 0;
volatile uint32_t g_chunk_queue_overflow_count = 0;
uint32_t g_logged_chunk_queue_overflow_count = 0;
size_t g_audio_ring_write_idx = 0;
size_t g_audio_ring_fill = 0;
uint64_t g_audio_ring_total_samples = 0;
PDM_DEV_INFO g_pdm_dev_info{};
bool g_mic_initialized = false;
bool g_vad_initialized = false;
DEV_UART* g_console_uart = nullptr;
uint64_t g_chunk_count = 0;
volatile uint32_t g_vad_irq_count = 0;
uint32_t g_vad_handled_count = 0;
uint64_t g_last_vad_ms = 0;
bool g_i2c_initialized = false;

uint32_t g_i2c_last_err_state = 0;
bool g_video_path_forced_off = false;
bool g_detection_led_initialized = false;
bool g_detection_oled_available = false;
bool g_detection_oled_error_logged = false;
bool g_model_flash_opened = false;
bool g_model_xip_enabled = false;
volatile uint32_t g_mb_event_bits = 0;
volatile uint32_t g_mb_ready_bytes = 0;
uint32_t g_last_pmu_wakeup_event = PMU_WAKEUP_NONE;
uint32_t g_last_pmu_wakeup_event1 = PMU_WAKEUPEVENT1_NONE;
uint32_t g_last_pmu_pll_freq = 0;
uint32_t g_last_resume_hsc_freq = 0;
uint32_t g_last_resume_qspi_freq = 0;
uint32_t g_pm_resume_count = 0;
bool g_pm_hsc_restore_pending = false;
alignas(32) uint8_t g_mb_audio_buf[64 * 1024] BIRDNET_LEGACY_AUDIO_BSS;
uint32_t g_mb_audio_read_offset = 0;
uint32_t g_mb_window_fill_samples = 0;
bool g_mb_window_ready = false;
uint64_t g_mb_window_count = 0;

struct RetainedRuntimeState {
  uint32_t magic;
  uint32_t version;
  uint32_t sample_rate_hz;
  uint32_t inference_step_samples;
  uint64_t fixed_window_count;
  uint64_t last_capture_end;
  uint32_t rejected_windows;
  uint32_t skipped_windows;
  uint32_t phase;
  uint64_t sht_retry_ms;
  uint64_t oled_retry_ms;
  uint64_t battery_retry_ms;
  uint32_t oled_command;
  uint8_t oled_address;
  bool sht_error_logged;
  bool oled_on;
  uint8_t oled_page; // 0=detection, 1=button status, 2+=three-species count pages.
  uint32_t labels_verified_fnv;
};

constexpr uint32_t kRetainedRuntimeMagic = 0x424e5254u;  // BNRT
constexpr uint32_t kRetainedRuntimeVersion = 10;

RetainedRuntimeState g_retained_runtime BIRDNET_NOINIT;

#if BIRDNET_DETECTION_AUDIO_CODEC == 2 && BIRDNET_VERIFY_OPUS_ON_WAKE
struct OpusCodeProof { uint32_t magic, crc; };
OpusCodeProof g_opus_code_proof BIRDNET_NOINIT;
bool VerifyOpusCode(bool warm_resume) {
  const size_t length = reinterpret_cast<uintptr_t>(__opus_text_end__) -
                        reinterpret_cast<uintptr_t>(__opus_text_start__);
  const uint32_t crc = ~birdnet_clip::Crc(~0u, __opus_text_start__, length);
  const bool ok = !warm_resume ||
      (g_opus_code_proof.magic == 0x4f505553u && g_opus_code_proof.crc == crc);
  if (!warm_resume) g_opus_code_proof = {0x4f505553u, crc};
  return ok;
}
#endif

struct Sht41Reading {
  bool ok = false;
  int temp_c_q1e2 = 0;
  int rh_q1e2 = 0;
};

birdnet_battery::Reading g_battery_reading;

#if BIRDNET_ENABLE_FIELD_MONITORING
static_assert(BIRDNET_ENABLE_SD_DETECTION_LOGGING && BIRDNET_ENABLE_MAX17048 &&
              cfg::kUseDualCoreFixedCadence, "Field profile requires paired capture, SD and gauge support");
struct FieldState {
  birdnet_field::BatteryPolicy battery;
  birdnet_field::StartupCounts<cfg::kNumClasses> counts;
  uint64_t last_hour, health_period, last_success_end, monitored_samples;
  uint64_t free_bytes, pause_started_ms, paused_ms, audio_retry_ms;
  uint32_t inferences, inference_errors, sd_errors, audio_saved, ambient_saved;
  uint32_t audio_suppressed, ambient_missed, metadata_session;
  uint32_t audio_peak, audio_clipped;
  int battery_min_mv;
  bool space_known, audio_stopped, log_stopped, dirty;
};
FieldState g_field BIRDNET_NOINIT;

bool ShowFieldCounts(uint32_t presses) {
  constexpr unsigned kDisplayedSpecies = 15; // Five pages, independent of model size.
  // Rank every class into bounded button-only scratch; retain all class totals.
  birdnet_field::SpeciesCount ranked[kDisplayedSpecies]{};
  g_field.counts.Top(ranked, kDisplayedSpecies, [](int index) {
    return birdnet_labels::IsBird(index);
  });
  unsigned species = 0;
  while (species < kDisplayedSpecies && ranked[species].index >= 0) ++species;
  const unsigned pages = species ? (species + 2) / 3 : 1;
  unsigned current = g_retained_runtime.oled_page ? g_retained_runtime.oled_page - 1 : 0;
  // Defensively wrap stale retained page selections back to status.
  if (current > pages) current = pages;
  const unsigned next = (current + presses % (pages + 1)) % (pages + 1);
  g_retained_runtime.oled_page = static_cast<uint8_t>(next + 1);
  if (!next) return false; // Caller renders the status page after the last page.
  const unsigned offset = (next - 1) * 3;
  const char* labels[3]{};
  uint32_t counts[3]{};
  for (unsigned i = 0; i < 3 && offset + i < species; ++i) {
    labels[i] = birdnet_labels::LabelForIndex(ranked[offset + i].index);
    counts[i] = ranked[offset + i].count;
  }
  return birdnet_oled::ShowCounts(labels, counts, next, pages);
}
#endif

#if BIRDNET_ENABLE_SD_DETECTION_LOGGING
struct DetectionLogRecord {
  uint64_t timestamp_ms;
  uint16_t species_index;
  uint16_t probability_q1e4;
};

struct EnvLogRecord {
  uint64_t timestamp_ms;
  bool ok;
  int temp_c_q1e2;
  int rh_q1e2;
};

struct SdLogRetainedState {
  uint32_t magic;
  uint32_t version;
  uint32_t inference_step_samples;
  uint32_t flush_interval_ms;
  uint32_t detection_capacity;
  uint32_t env_capacity;
  DetectionLogRecord log_records[cfg::kSdLogBufferRecords];
  EnvLogRecord env_log_records[cfg::kSdEnvLogBufferRecords];
  EnvLogRecord latest_env_log;
  int log_record_count;
  int env_log_record_count;
  uint32_t has_latest_env_log;
  uint32_t dropped_records;
  uint32_t env_dropped_records;
  uint64_t last_flush_period;
  uint64_t last_env_queued_period;
  uint32_t next_audio_sequence;
  uint32_t audio_sequence_initialized;
  uint32_t session_id;
  uint64_t next_retry_ms;
  uint32_t retry_delay_ms;
  bool log_error_logged;
  bool audio_error_logged;
  uint64_t last_detection_end_samples[cfg::kNumClasses];
};

constexpr uint32_t kSdLogRetainedMagic = 0x53444c47u;  // SDLG
constexpr uint32_t kSdLogRetainedVersion = 8;

FATFS g_sd_log_fs{};
SdLogRetainedState g_sd_log_state BIRDNET_NOINIT;


bool g_sd_mounted = false;
bool g_sd_spi_active = false;
#endif

constexpr bool IsSaturatedSample(int16_t sample) {
  return sample == std::numeric_limits<int16_t>::min() ||
         sample == std::numeric_limits<int16_t>::max();
}

struct AudioWindowDiagnostics {
  int16_t min_sample = std::numeric_limits<int16_t>::max();
  int16_t max_sample = std::numeric_limits<int16_t>::min();
  int32_t mean_q1e2 = 0;
  uint32_t mean_abs = 0;
  uint32_t rms = 0;
  uint32_t mean_abs_delta = 0;
  uint32_t zero_samples = 0;
  uint32_t saturated_samples = 0;
  uint32_t hash = 2166136261u;
};

AudioWindowDiagnostics MeasureAudioWindow(const int16_t* samples, size_t count) {
  AudioWindowDiagnostics stats{};
  if (!samples || count == 0) {
    stats.min_sample = 0;
    stats.max_sample = 0;
    return stats;
  }

  int64_t sum = 0;
  uint64_t sum_abs = 0;
  uint64_t sum_squares = 0;
  uint64_t sum_abs_delta = 0;
  int16_t previous = samples[0];
  for (size_t i = 0; i < count; ++i) {
    const int16_t sample = samples[i];
    const int32_t value = static_cast<int32_t>(sample);
    stats.min_sample = std::min(stats.min_sample, sample);
    stats.max_sample = std::max(stats.max_sample, sample);
    sum += value;
    sum_abs += static_cast<uint32_t>(std::abs(value));
    sum_squares += static_cast<uint64_t>(static_cast<int64_t>(value) * value);
    if (i != 0) {
      sum_abs_delta +=
          static_cast<uint32_t>(std::abs(value - static_cast<int32_t>(previous)));
    }
    previous = sample;
    stats.zero_samples += (sample == 0) ? 1u : 0u;
    stats.saturated_samples += IsSaturatedSample(sample) ? 1u : 0u;
    stats.hash = (stats.hash ^ static_cast<uint16_t>(sample)) * 16777619u;
  }

  const uint64_t divisor = static_cast<uint64_t>(count);
  stats.mean_q1e2 = static_cast<int32_t>((sum * 100) / static_cast<int64_t>(count));
  stats.mean_abs = static_cast<uint32_t>(sum_abs / divisor);
  stats.rms = static_cast<uint32_t>(
      std::sqrt(static_cast<double>(sum_squares) / static_cast<double>(divisor)));
  stats.mean_abs_delta = count > 1
                             ? static_cast<uint32_t>(sum_abs_delta / (divisor - 1u))
                             : 0u;
  return stats;
}

SCU_LSCCLKDIV_E LscClockDivFromInt(int div) {
  switch (div) {
    case 1: return SCU_LSCCLKDIV_1;
    case 2: return SCU_LSCCLKDIV_2;
    case 3: return SCU_LSCCLKDIV_3;
    case 4: return SCU_LSCCLKDIV_4;
    case 5: return SCU_LSCCLKDIV_5;
    case 6: return SCU_LSCCLKDIV_6;
    case 7: return SCU_LSCCLKDIV_7;
    case 8: return SCU_LSCCLKDIV_8;
    case 9: return SCU_LSCCLKDIV_9;
    case 10: return SCU_LSCCLKDIV_10;
    case 11: return SCU_LSCCLKDIV_11;
    case 12: return SCU_LSCCLKDIV_12;
    case 13: return SCU_LSCCLKDIV_13;
    case 14: return SCU_LSCCLKDIV_14;
    case 15: return SCU_LSCCLKDIV_15;
    case 16: return SCU_LSCCLKDIV_16;
    default: return SCU_LSCCLKDIV_1;
  }
}




// Private diagnostic headers share the same bounded logger.
constexpr auto LogLine = &birdnet_log::Write;
#include "birdnet_model_integrity.h"

void InitRetainedRuntimeState(bool warm_resume) {
  const bool valid =
      warm_resume &&
      g_retained_runtime.magic == kRetainedRuntimeMagic &&
      g_retained_runtime.version == kRetainedRuntimeVersion &&
      g_retained_runtime.sample_rate_hz == static_cast<uint32_t>(cfg::kSampleRateHz) &&
      g_retained_runtime.inference_step_samples == static_cast<uint32_t>(cfg::kInferenceStepSamples);
  if (valid) {
    return;
  }

  if (g_retained_runtime.magic == kRetainedRuntimeMagic &&
      g_retained_runtime.version == kRetainedRuntimeVersion) {
    birdnet_log::Status("[birdnet] previous retained phase=%lu (reset cause not inferred)\r\n",
                  static_cast<unsigned long>(g_retained_runtime.phase));
  }
  std::memset(&g_retained_runtime, 0, sizeof(g_retained_runtime));
#if BIRDNET_ENABLE_FIELD_MONITORING
  std::memset(&g_field, 0, sizeof(g_field));
  g_field.dirty = true;
#endif
  g_retained_runtime.magic = kRetainedRuntimeMagic;
  g_retained_runtime.version = kRetainedRuntimeVersion;
  g_retained_runtime.sample_rate_hz = static_cast<uint32_t>(cfg::kSampleRateHz);
  g_retained_runtime.inference_step_samples = static_cast<uint32_t>(cfg::kInferenceStepSamples);
}

uint64_t RetainedAdvanceFixedWindowCount() {
  if (g_retained_runtime.magic != kRetainedRuntimeMagic ||
      g_retained_runtime.version != kRetainedRuntimeVersion ||
      g_retained_runtime.sample_rate_hz != static_cast<uint32_t>(cfg::kSampleRateHz) ||
      g_retained_runtime.inference_step_samples != static_cast<uint32_t>(cfg::kInferenceStepSamples)) {
    InitRetainedRuntimeState(false);
  }
  return ++g_retained_runtime.fixed_window_count;
}

#if BIRDNET_ENABLE_SD_DETECTION_LOGGING
void SdLogInitRetainedState(bool warm_resume) {
  const bool valid =
      warm_resume &&
      g_sd_log_state.magic == kSdLogRetainedMagic &&
      g_sd_log_state.version == kSdLogRetainedVersion &&
      g_sd_log_state.inference_step_samples == static_cast<uint32_t>(cfg::kInferenceStepSamples) &&
      g_sd_log_state.flush_interval_ms == static_cast<uint32_t>(cfg::kSdLogFlushIntervalMs) &&
      g_sd_log_state.detection_capacity == static_cast<uint32_t>(cfg::kSdLogBufferRecords) &&
      g_sd_log_state.env_capacity == static_cast<uint32_t>(cfg::kSdEnvLogBufferRecords);
  if (valid) {
    return;
  }

  std::memset(&g_sd_log_state, 0, sizeof(g_sd_log_state));
  g_sd_log_state.magic = kSdLogRetainedMagic;
  g_sd_log_state.version = kSdLogRetainedVersion;
  g_sd_log_state.inference_step_samples = static_cast<uint32_t>(cfg::kInferenceStepSamples);
  g_sd_log_state.flush_interval_ms = static_cast<uint32_t>(cfg::kSdLogFlushIntervalMs);
  g_sd_log_state.detection_capacity = static_cast<uint32_t>(cfg::kSdLogBufferRecords);
  g_sd_log_state.env_capacity = static_cast<uint32_t>(cfg::kSdEnvLogBufferRecords);
}
#else
void SdLogInitRetainedState(bool) {}
#endif

bool ArmActiveWatchdog(uint32_t phase) {
  if constexpr (!cfg::kUseDualCoreFixedCadence || cfg::kRecoveryFirmware) return true;
  g_retained_runtime.phase = phase;
  WATCHDOG_CFG_T watchdog{};
  watchdog.period = 15000;
  watchdog.ctrl = WATCHDOG_CTRL_CPU;
  watchdog.state = WATCHDOG_STATE_DC;
  watchdog.type = WATCHDOG_RESET;
  return hx_drv_watchdog_cm55m_start(&watchdog, nullptr) == WATCHDOG_NO_ERROR;
}

void MaybeMuteVendorXprintf() {
  if (cfg::kMuteVendorXprintf) {
    xprintf_desetup();
  }
}

bool RestoreFromPmWakeup() {
  uint32_t wakeup_event = PMU_WAKEUP_NONE;
  uint32_t wakeup_event1 = PMU_WAKEUPEVENT1_NONE;
  uint32_t pll_freq = 0;
  hx_drv_pmu_get_ctrl(PMU_pmu_wakeup_EVT, &wakeup_event);
  hx_drv_pmu_get_ctrl(PMU_pmu_wakeup_EVT1, &wakeup_event1);
  g_last_pmu_wakeup_event = wakeup_event;
  g_last_pmu_wakeup_event1 = wakeup_event1;
  if ((wakeup_event == PMU_WAKEUP_NONE) && (wakeup_event1 == PMU_WAKEUPEVENT1_NONE)) {
    return false;
  }
  hx_drv_swreg_aon_get_pllfreq(&pll_freq);
  hx_lib_pm_ctrl_fromPMUtoCPU(nullptr);

  uint32_t hsc_freq = 0;
  uint32_t qspi_freq = 0;
  hx_drv_scu_get_freq(SCU_CLK_FREQ_TYPE_HSC_CLK, &hsc_freq);
  hx_drv_scu_get_freq(SCU_CLK_FREQ_TYPE_HSC_QSPI_CLK, &qspi_freq);
  if (hsc_freq != 0) {
    SystemCoreClock = hsc_freq;
  }

  g_last_pmu_pll_freq = pll_freq;
  g_last_resume_hsc_freq = hsc_freq;
  g_last_resume_qspi_freq = qspi_freq;
  g_pm_hsc_restore_pending = true;
  ++g_pm_resume_count;
  return true;
}

void RestoreHighSpeedClockAfterPmWakeup() {
  if (!g_pm_hsc_restore_pending) {
    return;
  }
  uint32_t hsc_freq = 0;
  uint32_t qspi_freq = 0;

#ifndef CPU_24MHZ_VERSION
  if constexpr (cfg::kRestoreHscAfterPmWakeup) {
    // The Himax TFLM examples restore these HSC clock-source registers after
    // PMU wake, but only after QSPI XIP has been reopened. Leave this disabled
    // by default because it can invalidate the current XIP timing on this app.
    EPII_set_memory(0x56100030, 0x4037);
    EPII_set_memory(0x56100034, 0x0);
    EPII_set_memory(0x56100038, 0xc1b8);
    __DSB();
    __ISB();
  }
#endif

  hx_drv_scu_get_freq(SCU_CLK_FREQ_TYPE_HSC_CLK, &hsc_freq);
  hx_drv_scu_get_freq(SCU_CLK_FREQ_TYPE_HSC_QSPI_CLK, &qspi_freq);
  if (hsc_freq != 0) {
    // platform_driver_init runs before hx_lib_pm_ctrl_fromPMUtoCPU() on a
    // retention wake, so SystemCoreClock can still reflect the RC24 PM clock.
    // Update only the clock variable here; calling SystemCoreClockUpdate()
    // would reset the SysTick loop counter and make per-inference timing jump.
    SystemCoreClock = hsc_freq;
  }
  g_last_resume_hsc_freq = hsc_freq;
  g_last_resume_qspi_freq = qspi_freq;
  g_pm_hsc_restore_pending = false;
  birdnet_log::Status("[birdnet] pm hsc %s hsc=%lu qspi=%lu sys=%lu\r\n",
                cfg::kRestoreHscAfterPmWakeup ? "restored" : "checked",
                static_cast<unsigned long>(g_last_resume_hsc_freq),
                static_cast<unsigned long>(g_last_resume_qspi_freq),
                static_cast<unsigned long>(SystemCoreClock));
}

void RestoreSdkFlashPinmux() {
  SCU_PINMUX_CFG_T pinmux_cfg{};
  hx_drv_scu_get_all_pinmux_cfg(&pinmux_cfg);
  pinmux_cfg.pin_pb2 = SCU_PB2_PINMUX_SPI_M_DO_1;
  pinmux_cfg.pin_pb3 = SCU_PB3_PINMUX_SPI_M_DI_1;
  pinmux_cfg.pin_pb4 = SCU_PB4_PINMUX_SPI_M_SCLK_1;
#ifdef GROVE_VISION_AI_II
  pinmux_cfg.pin_pb11 = SCU_PB11_PINMUX_SPI_M_CS;
#else
  pinmux_cfg.pin_pb5 = SCU_PB5_PINMUX_SPI_M_CS_1;
#endif
  hx_drv_scu_set_all_pinmux_cfg(&pinmux_cfg, 1);
}

void ForceVideoPathOff() {
  if (g_video_path_forced_off) {
    return;
  }

  hx_lib_pm_mipi_txoff_lvofhv();
  hx_drv_scu_set_mclk_enable(0);
  hx_drv_scu_set_isp_write_en(0);
  hx_drv_scu_set_dp_wakeupctrl_sel(SCU_DP_WAKEUPCTRL_SEL_CPU);

  SCU_PDHSC_DPCLKEN_CFG_T pdhsc_dpclken{};
  hx_drv_scu_set_pdhsc_dpclken_cfg(pdhsc_dpclken);

  SCU_PDLSC_DPCLKEN_CFG_T pdlsc_dpclken{};
  hx_drv_scu_set_pdlsc_dpclken_cfg(pdlsc_dpclken);

  g_video_path_forced_off = true;
}

