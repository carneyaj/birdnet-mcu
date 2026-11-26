#pragma once
#include "birdnet_field_policy.h"

#include <cstdint>

#ifndef BIRDNET_ENABLE_SERIAL_FILES
#define BIRDNET_ENABLE_SERIAL_FILES 0
#endif

#ifndef BIRDNET_SKIP_PCM_ZERO
#define BIRDNET_SKIP_PCM_ZERO 0
#endif
#ifndef BIRDNET_VERIFY_OPUS_ON_WAKE
#define BIRDNET_VERIFY_OPUS_ON_WAKE 0
#endif
#ifndef BIRDNET_SD_AUDIO_BUFFER_BYTES
#define BIRDNET_SD_AUDIO_BUFFER_BYTES 4096
#endif
static_assert(BIRDNET_SD_AUDIO_BUFFER_BYTES >= 512 && BIRDNET_SD_AUDIO_BUFFER_BYTES <= 16384 &&
              BIRDNET_SD_AUDIO_BUFFER_BYTES % 512 == 0, "SD buffer must be 512..16384 whole-sector bytes");

namespace cfg {

#ifndef BIRDNET_DUAL_CORE_FIXED_CADENCE
#define BIRDNET_DUAL_CORE_FIXED_CADENCE 0
#endif

#ifndef BIRDNET_ENABLE_CONSOLE_UART
#define BIRDNET_ENABLE_CONSOLE_UART 0
#endif

#ifndef BIRDNET_ENABLE_STATUS_LOGS
#define BIRDNET_ENABLE_STATUS_LOGS 0
#endif

#ifndef BIRDNET_ENABLE_DIAGNOSTIC_LOGS
#define BIRDNET_ENABLE_DIAGNOSTIC_LOGS 0
#endif

#ifndef BIRDNET_ENABLE_LED_DIAGNOSTICS
#define BIRDNET_ENABLE_LED_DIAGNOSTICS 0
#endif

#ifndef BIRDNET_ENABLE_SD_DETECTION_LOGGING
#define BIRDNET_ENABLE_SD_DETECTION_LOGGING 0
#endif

#ifndef BIRDNET_ENABLE_DETECTION_AUDIO
#define BIRDNET_ENABLE_DETECTION_AUDIO 0
#endif

#ifndef BIRDNET_DETECTION_AUDIO_CODEC
#define BIRDNET_DETECTION_AUDIO_CODEC 0
#endif

#ifndef BIRDNET_DETECTION_AUDIO_RICE_QUANT_STEP
#define BIRDNET_DETECTION_AUDIO_RICE_QUANT_STEP 80
#endif

#ifndef BIRDNET_ENABLE_DETECTION_LED
#define BIRDNET_ENABLE_DETECTION_LED 1
#endif

#ifndef BIRDNET_DETECTION_LED_ACTIVE_LOW
#define BIRDNET_DETECTION_LED_ACTIVE_LOW 0
#endif

#ifndef BIRDNET_DETECTION_LED_RETENTION_SELF_TEST
#define BIRDNET_DETECTION_LED_RETENTION_SELF_TEST 0
#endif

#ifndef BIRDNET_ENABLE_DETECTION_OLED
#define BIRDNET_ENABLE_DETECTION_OLED 1
#endif

#ifndef BIRDNET_ENABLE_EXTENDED_CLASSES_673
#define BIRDNET_ENABLE_EXTENDED_CLASSES_673 0
#endif

#ifndef BIRDNET_NUM_CLASSES
#if (BIRDNET_ENABLE_EXTENDED_CLASSES_673 != 0)
#define BIRDNET_NUM_CLASSES 673
#else
#define BIRDNET_NUM_CLASSES 256
#endif
#endif

#ifndef BIRDNET_ENABLE_LORA_DETECTION_TX
#define BIRDNET_ENABLE_LORA_DETECTION_TX 0
#endif

#ifndef BIRDNET_LORA_FREQ_HZ
#define BIRDNET_LORA_FREQ_HZ 915000000u
#endif

#ifndef BIRDNET_LORA_TX_POWER_DBM
#define BIRDNET_LORA_TX_POWER_DBM 14
#endif

#ifndef BIRDNET_LORA_SYNC_WORD
#define BIRDNET_LORA_SYNC_WORD 0x12
#endif

#ifndef BIRDNET_LORA_NODE_ID
#define BIRDNET_LORA_NODE_ID 1
#endif

#ifndef BIRDNET_LORA_TEST_INTERVAL_MS
#define BIRDNET_LORA_TEST_INTERVAL_MS 0
#endif

#ifndef BIRDNET_DETECTION_OLED_I2C_ADDR
#define BIRDNET_DETECTION_OLED_I2C_ADDR 0x3C
#endif

#ifndef BIRDNET_DETECTION_OLED_FLIP_180
#define BIRDNET_DETECTION_OLED_FLIP_180 1
#endif

#ifndef BIRDNET_OLED_HEIGHT
#define BIRDNET_OLED_HEIGHT 64
#endif

#ifndef BIRDNET_ENABLE_STATUS_BUTTON
#define BIRDNET_ENABLE_STATUS_BUTTON 0
#endif

#ifndef BIRDNET_ENABLE_MAX17048
#define BIRDNET_ENABLE_MAX17048 0
#endif

#ifndef BIRDNET_SD_LOG_THRESHOLD_Q1E4
#define BIRDNET_SD_LOG_THRESHOLD_Q1E4 3000
#endif

// Bird-detection calibration must not silently raise the recording veto.
#ifndef BIRDNET_HUMAN_VOCAL_THRESHOLD_Q1E4
#define BIRDNET_HUMAN_VOCAL_THRESHOLD_Q1E4 3000
#endif

#ifndef BIRDNET_SD_LOG_BUFFER_RECORDS
#define BIRDNET_SD_LOG_BUFFER_RECORDS 384
#endif

#ifndef BIRDNET_SD_LOG_FLUSH_RECORDS
#define BIRDNET_SD_LOG_FLUSH_RECORDS 16
#endif

#ifndef BIRDNET_SD_LOG_FLUSH_INTERVAL_MS
#define BIRDNET_SD_LOG_FLUSH_INTERVAL_MS 60000
#endif

#ifndef BIRDNET_SD_LOG_MAX_DETECTIONS_PER_WINDOW
#define BIRDNET_SD_LOG_MAX_DETECTIONS_PER_WINDOW 16
#endif

#ifndef BIRDNET_SD_ENV_LOG_BUFFER_RECORDS
#define BIRDNET_SD_ENV_LOG_BUFFER_RECORDS 16
#endif

#ifndef BIRDNET_USE_AUDIO_ONLY_PREROLLING
#if BIRDNET_DUAL_CORE_FIXED_CADENCE
#define BIRDNET_USE_AUDIO_ONLY_PREROLLING 1
#else
#define BIRDNET_USE_AUDIO_ONLY_PREROLLING 0
#endif
#endif

#ifndef BIRDNET_GATE_XIP_BETWEEN_WINDOWS
#define BIRDNET_GATE_XIP_BETWEEN_WINDOWS 0
#endif

#ifndef BIRDNET_RESTORE_HSC_AFTER_PMU
#define BIRDNET_RESTORE_HSC_AFTER_PMU 0
#endif

#ifndef BIRDNET_PMU_FAST_WAKE_CLOCK
#define BIRDNET_PMU_FAST_WAKE_CLOCK 0
#endif

#ifndef BIRDNET_PMU_WAKE_CM55S_DIV
#if BIRDNET_PMU_FAST_WAKE_CLOCK
#define BIRDNET_PMU_WAKE_CM55S_DIV 4
#else
#define BIRDNET_PMU_WAKE_CM55S_DIV 1
#endif
#endif

#ifndef BIRDNET_PMU_IDLE_LSC_DIV
#define BIRDNET_PMU_IDLE_LSC_DIV 1
#endif

#ifndef BIRDNET_PMU_CLOCK_CTRL_BYAPP
#define BIRDNET_PMU_CLOCK_CTRL_BYAPP 0
#endif

#ifndef BIRDNET_PMU_HSC_SRAM_RETENTION_MASK
#define BIRDNET_PMU_HSC_SRAM_RETENTION_MASK 0xF
#endif

#ifndef BIRDNET_FRONTEND_STREAMING
#define BIRDNET_FRONTEND_STREAMING 0
#endif
#ifndef BIRDNET_STREAMING_VERIFY
#define BIRDNET_STREAMING_VERIFY 0
#endif

#ifndef BIRDNET_INFERENCE_INTERVAL_MS
#define BIRDNET_INFERENCE_INTERVAL_MS 3000
#endif
#ifndef BIRDNET_INFERENCE_STEP_SAMPLES
#define BIRDNET_INFERENCE_STEP_SAMPLES 0
#endif

#ifndef BIRDNET_SAMPLE_RATE_HZ
#define BIRDNET_SAMPLE_RATE_HZ 48000
#endif

constexpr int kSampleRateHz = BIRDNET_SAMPLE_RATE_HZ;
constexpr int kWindowSec = 3;
constexpr int kInferenceStepSamples = BIRDNET_INFERENCE_STEP_SAMPLES > 0
    ? BIRDNET_INFERENCE_STEP_SAMPLES : (kSampleRateHz * BIRDNET_INFERENCE_INTERVAL_MS) / 1000;
// Rounded milliseconds are for display only; scheduling/retention use samples.
constexpr int kInferenceIntervalMs = (1000LL * kInferenceStepSamples + kSampleRateHz / 2) / kSampleRateHz;
static_assert(kSampleRateHz == 48000 || kSampleRateHz == 32000 || kSampleRateHz == 24000 ||
                  kSampleRateHz == 16000 || kSampleRateHz == 8000,
              "Unsupported BirdNET sample rate");
static_assert(BIRDNET_INFERENCE_STEP_SAMPLES >= 0, "Invalid sample step override");
static_assert(BIRDNET_INFERENCE_STEP_SAMPLES > 0 ||
                  (kSampleRateHz * BIRDNET_INFERENCE_INTERVAL_MS) % 1000 == 0,
              "Inference interval must align to samples");
static_assert(kInferenceStepSamples > 0, "Inference step must be positive");

// Toggle this to true when you want live serial verification. Leave it false
// for the quiet low-power build.
constexpr bool kEnableConsoleUart = (BIRDNET_ENABLE_CONSOLE_UART != 0);
// Temporary recovery mode: boot a stable firmware without touching the
// external model slot or audio pipeline so a partially flashed model cannot
// crash the app at startup.
constexpr bool kRecoveryFirmware = false;
// Keep BirdNET's own UART output while muting vendor xprintf-based mailbox/VAD
// spam from prebuilt libraries.
constexpr bool kMuteVendorXprintf = true;
// Fixed-cadence capture mode: keep audio streaming on CM55M and infer on a
// regular 3-second cadence, without VAD gating.
constexpr bool kUseHardwareVad = false;
constexpr bool kUseDualCoreAudioPreroll = false;
// Experimental fixed-cadence two-core mode:
// - CM55S owns continuous PDM DMA
// - CM55M wakes only when CM55S ships a complete 3-second window
constexpr bool kUseDualCoreFixedCadence = (BIRDNET_DUAL_CORE_FIXED_CADENCE != 0);
constexpr bool kUseCm55sAudioHelper = kUseDualCoreAudioPreroll || kUseDualCoreFixedCadence;
constexpr bool kDualCoreHandshakeDebug = false;
constexpr bool kMinimalDualCoreBringupDebug = false;
// In the fixed-cadence dual-core build, put CM55M into the same audio-only PMU
// state between windows so the big core is not just spinning in __WFI().
constexpr bool kUseAudioOnlyPrerolling = (BIRDNET_USE_AUDIO_ONLY_PREROLLING != 0);
// The SDK TFLM examples write raw HSC clock-source registers after PMU wake.
// On this app that made post-wake QSPI XIP accesses hang, so keep it off unless
// explicitly testing the SDK clock-restore path.
constexpr bool kRestoreHscAfterPmWakeup = (BIRDNET_RESTORE_HSC_AFTER_PMU != 0);
// Use the PMU warmboot clock field to resume CM55M at the high-speed PLL
// instead of waking at RC96 and trying to patch HSC/QSPI registers afterward.
constexpr bool kPmuFastWakeClock = (BIRDNET_PMU_FAST_WAKE_CLOCK != 0);
constexpr int kPmuWakeCm55sDiv = BIRDNET_PMU_WAKE_CM55S_DIV;
constexpr int kPmuIdleLscDiv = BIRDNET_PMU_IDLE_LSC_DIV;
constexpr bool kPmuClockCtrlByApp = (BIRDNET_PMU_CLOCK_CTRL_BYAPP != 0);
constexpr uint32_t kPmuHscSramRetentionMask = BIRDNET_PMU_HSC_SRAM_RETENTION_MASK;
static_assert(kPmuWakeCm55sDiv >= 1 && kPmuWakeCm55sDiv <= 16, "CM55S wake divider must be 1..16");
static_assert(kPmuIdleLscDiv >= 1 && kPmuIdleLscDiv <= 16, "Idle LSC divider must be 1..16");
static_assert(kPmuHscSramRetentionMask <= 0xFu, "HSC SRAM retention mask must fit four banks");
constexpr uint32_t kPmuFastWakePllHz = 400000000u;
constexpr int kVadActiveHoldMs = 3500;
constexpr int kVadRetriggerDebounceMs = 250;
constexpr int kMaxInferencesPerActivation = 2;
constexpr int kAudioOnlyPrerollIdleDelayMs = 750;
constexpr int kVadInputGain = 4;
constexpr int kVadSpeechGain = 8;
constexpr int kVadNoiseGain = 8;
constexpr int kVadKNoise = 160;
constexpr bool kVadBypassHighPass = true;
// Keep this separate from kEnableConsoleUart so inference results can stay on
// serial without pulling in the extra boot/status strings in production.
constexpr bool kEnableStatusLogs = (BIRDNET_ENABLE_STATUS_LOGS != 0);
constexpr bool kEnableDiagnosticLogs = (BIRDNET_ENABLE_DIAGNOSTIC_LOGS != 0);
constexpr bool kEnableLedDiagnostics = (BIRDNET_ENABLE_LED_DIAGNOSTICS != 0);
constexpr bool kEnableSdDetectionLogging = (BIRDNET_ENABLE_SD_DETECTION_LOGGING != 0);
constexpr bool kEnableDetectionAudio = (BIRDNET_ENABLE_DETECTION_AUDIO != 0);
constexpr int kDetectionAudioCodec = BIRDNET_DETECTION_AUDIO_CODEC;
constexpr int kDetectionAudioRiceQuantStep =
    BIRDNET_DETECTION_AUDIO_RICE_QUANT_STEP;
constexpr bool kEnableDetectionLed = (BIRDNET_ENABLE_DETECTION_LED != 0);
constexpr bool kDetectionLedActiveLow = (BIRDNET_DETECTION_LED_ACTIVE_LOW != 0);
constexpr bool kDetectionLedRetentionSelfTest =
    (BIRDNET_DETECTION_LED_RETENTION_SELF_TEST != 0);
constexpr bool kEnableDetectionOled = (BIRDNET_ENABLE_DETECTION_OLED != 0);
constexpr bool kEnableLoraDetectionTx = (BIRDNET_ENABLE_LORA_DETECTION_TX != 0);
constexpr uint32_t kLoraFreqHz = BIRDNET_LORA_FREQ_HZ;
constexpr int kLoraTxPowerDbm = BIRDNET_LORA_TX_POWER_DBM;
constexpr uint8_t kLoraSyncWord = BIRDNET_LORA_SYNC_WORD;
constexpr uint8_t kLoraNodeId = BIRDNET_LORA_NODE_ID;
constexpr uint32_t kLoraTestIntervalMs = BIRDNET_LORA_TEST_INTERVAL_MS;
constexpr uint8_t kDetectionOledI2cAddr = BIRDNET_DETECTION_OLED_I2C_ADDR;
constexpr bool kDetectionOledFlip180 = (BIRDNET_DETECTION_OLED_FLIP_180 != 0);
constexpr int kOledHeight = BIRDNET_OLED_HEIGHT;
constexpr bool kEnableStatusButton = (BIRDNET_ENABLE_STATUS_BUTTON != 0);
static_assert(kOledHeight == 32 || kOledHeight == 64, "OLED must be 128x32 or 128x64");
constexpr bool kEnableMax17048 = (BIRDNET_ENABLE_MAX17048 != 0);
constexpr int kSdLogThresholdQ1e4 = BIRDNET_SD_LOG_THRESHOLD_Q1E4;
constexpr float kSdLogThreshold = static_cast<float>(kSdLogThresholdQ1e4) / 10000.0f;
constexpr float kHumanVocalThreshold =
    static_cast<float>(BIRDNET_HUMAN_VOCAL_THRESHOLD_Q1E4) / 10000.0f;
constexpr int kSdLogBufferRecords = BIRDNET_SD_LOG_BUFFER_RECORDS;
constexpr int kSdLogFlushRecords = BIRDNET_SD_LOG_FLUSH_RECORDS;
constexpr int kSdLogFlushIntervalMs = BIRDNET_SD_LOG_FLUSH_INTERVAL_MS;
constexpr int kSdLogMaxDetectionsPerWindow = BIRDNET_SD_LOG_MAX_DETECTIONS_PER_WINDOW;
constexpr int kSdEnvLogBufferRecords = BIRDNET_SD_ENV_LOG_BUFFER_RECORDS;
constexpr const char* kSdLogFileName = "BIRDLOG.CSV";
constexpr const char* kSdEnvLogFileName = "ENVLOG.CSV";
constexpr const char* kSdAudioMapFileName = "AUDIOMAP.CSV";
static_assert(!kEnableDetectionAudio || kEnableSdDetectionLogging,
              "Detection audio requires SD detection logging");
static_assert(!kEnableDetectionAudio || kSampleRateHz == 48000,
              "Detection audio currently requires 48 kHz capture");
static_assert(kDetectionAudioCodec >= 0 && kDetectionAudioCodec <= 2,
              "Detection audio codec must be 0 (IMA WAV), 1 (BRC1 Rice), or 2 (Ogg Opus)");
static_assert(kDetectionAudioRiceQuantStep >= 1 &&
                  kDetectionAudioRiceQuantStep <= 32768,
              "Detection audio Rice quantization step must be 1..32768");
static_assert(kSdLogThresholdQ1e4 >= 0 && kSdLogThresholdQ1e4 <= 10000,
              "SD log threshold must be in q1e4 probability units");
static_assert(BIRDNET_HUMAN_VOCAL_THRESHOLD_Q1E4 >= 0 &&
                  BIRDNET_HUMAN_VOCAL_THRESHOLD_Q1E4 <= 10000,
              "Human-vocal threshold must be in q1e4 probability units");
static_assert(kSdLogBufferRecords > 0, "SD log buffer must hold at least one record");
static_assert(kSdLogFlushRecords > 0, "SD log flush record count must be positive");
static_assert(kSdLogFlushIntervalMs > 0, "SD log flush interval must be positive");
static_assert(kSdLogMaxDetectionsPerWindow > 0,
              "SD log max detections per window must be positive");
static_assert(kSdEnvLogBufferRecords > 0, "SD env log buffer must hold at least one record");
static_assert(kSdLogBufferRecords >=
                  kSdLogMaxDetectionsPerWindow *
                      ((1LL * kSdLogFlushIntervalMs * kSampleRateHz + 1000LL * kInferenceStepSamples - 1) /
                       (1000LL * kInferenceStepSamples)),
              "SD detection buffer must hold one full flush interval");
static_assert(kLoraFreqHz >= 860000000u && kLoraFreqHz <= 930000000u,
              "LoRa frequency must match an 868/915 MHz SX1262 region");
static_assert(kLoraTxPowerDbm >= -9 && kLoraTxPowerDbm <= 22,
              "SX1262 LoRa TX power must be -9..22 dBm");
// The deployed BirdNET artifact is a pure Ethos-U model at a fixed flash
// address. Leave this false for the smallest runtime; turn it on only if you
// need to probe alternate addresses or run mixed-op TFLM models again.
constexpr bool kEnableMixedModelSupport = false;
// SHT40/SHT41/SHT45 share the SHT4x protocol at 0x44. Legacy names retained.
// Poll once per inference window on PA2/SCL and PA3/SDA (D5/D4).
constexpr bool kEnableSht41Sensor = true;
constexpr uint8_t kSht41I2cAddr = 0x44;
constexpr int kSht41MeasureDelayMs = 10;
// Leave external flash XIP state alone across fixed-cadence PMU entry for now.
// The verified stable audio-only PMU build keeps this off; re-test carefully
// before enabling XIP gating in the same build as PMU retention wake.
constexpr bool kGateXipBetweenWindows = (BIRDNET_GATE_XIP_BETWEEN_WINDOWS != 0);

constexpr int kWindowSamples = kSampleRateHz * kWindowSec;
constexpr int kRingSlackSamples = 8192;
constexpr int kRingBufferSamples = kWindowSamples + kRingSlackSamples;
constexpr int kPdmBitRangeShift = 5;

// FFT backend selection for the v2.4 BirdNET frontend.
// Branch 1 is the 2048-point path and Branch 2 is the 1024-point path.
// Keep the selection explicit per branch so experiments are reproducible.
#define BIRDNET_FRONTEND_BACKEND_KISS 0
#define BIRDNET_FRONTEND_BACKEND_CMSIS_RFFT_SCALAR 1
#define BIRDNET_FRONTEND_BACKEND_CMSIS_CFFT 2

// Current verified fast target:
// - accelerate only the heavier 2048-point branch
// - keep the 1024-point branch on the known-good KISS path
//
// Leave these overrideable so FFT experiments can be reproduced from build
// flags instead of by hand-editing this file.
#ifndef BIRDNET_FRONTEND_BRANCH1_BACKEND
#define BIRDNET_FRONTEND_BRANCH1_BACKEND BIRDNET_FRONTEND_BACKEND_CMSIS_RFFT_SCALAR
#endif
#ifndef BIRDNET_FRONTEND_BRANCH2_BACKEND
#define BIRDNET_FRONTEND_BRANCH2_BACKEND BIRDNET_FRONTEND_BACKEND_KISS
#endif
#ifndef BIRDNET_FRONTEND_BRANCH1_DECIMATE2
#define BIRDNET_FRONTEND_BRANCH1_DECIMATE2 0
#endif
#ifndef BIRDNET_FRONTEND_RFFT_USE_ARM_INIT
#define BIRDNET_FRONTEND_RFFT_USE_ARM_INIT 0
#endif
#ifndef BIRDNET_FRONTEND_BRANCH1_RFFT_USE_C_OVERRIDE
#define BIRDNET_FRONTEND_BRANCH1_RFFT_USE_C_OVERRIDE 0
#endif
#ifndef BIRDNET_FRONTEND_BRANCH2_RFFT_USE_C_OVERRIDE
#define BIRDNET_FRONTEND_BRANCH2_RFFT_USE_C_OVERRIDE 1
#endif
#ifndef BIRDNET_FRONTEND_CMSIS_CFFT_REPAIR_LAST_STAGE
#define BIRDNET_FRONTEND_CMSIS_CFFT_REPAIR_LAST_STAGE 1
#endif
#ifndef BIRDNET_FRONTEND_CMSIS_RFFT_USE_MVE_STAGE
#define BIRDNET_FRONTEND_CMSIS_RFFT_USE_MVE_STAGE 0
#endif
#ifndef BIRDNET_FRONTEND_MEL_USE_MVE_DOT
#define BIRDNET_FRONTEND_MEL_USE_MVE_DOT 0
#endif
#ifndef BIRDNET_FRONTEND_MEL_MVE_DOT_MIN_LEN
#define BIRDNET_FRONTEND_MEL_MVE_DOT_MIN_LEN 8
#endif
#ifndef BIRDNET_FRONTEND_FRAME_FILL_USE_MVE
#define BIRDNET_FRONTEND_FRAME_FILL_USE_MVE 0
#endif
#ifndef BIRDNET_FRONTEND_MINMAX_USE_MVE
#define BIRDNET_FRONTEND_MINMAX_USE_MVE 0
#endif
#ifndef BIRDNET_FRONTEND_RFFT_UNPACK_USE_MVE
#define BIRDNET_FRONTEND_RFFT_UNPACK_USE_MVE 0
#endif
#ifndef BIRDNET_FRONTEND_RFFT_DIRECT_MEL
#define BIRDNET_FRONTEND_RFFT_DIRECT_MEL 0
#endif
#ifndef BIRDNET_FRONTEND_PARTIAL_REAL
#define BIRDNET_FRONTEND_PARTIAL_REAL 0
#endif
#ifndef BIRDNET_FRONTEND_FOLDED_REORDER
#define BIRDNET_FRONTEND_FOLDED_REORDER 0
#endif
#define BIRDNET_USE_FOLDED_REORDER BIRDNET_FRONTEND_FOLDED_REORDER
#ifndef BIRDNET_FRONTEND_STAGE_PROFILING
#define BIRDNET_FRONTEND_STAGE_PROFILING 0
#endif
#ifndef BIRDNET_NPU_PMU_PROFILE_SET
#define BIRDNET_NPU_PMU_PROFILE_SET 0
#endif
#if (BIRDNET_NPU_PMU_PROFILE_SET != 0) && (BIRDNET_NPU_PMU_PROFILE_SET != 1)
#error "BIRDNET_NPU_PMU_PROFILE_SET must be 0 (activity) or 1 (stalls)"
#endif
#ifndef BIRDNET_NPU_GATE_DURING_FRONTEND
#define BIRDNET_NPU_GATE_DURING_FRONTEND 0
#endif
#ifndef BIRDNET_CM55M_INVOKE_CYCLE_PROFILING
#define BIRDNET_CM55M_INVOKE_CYCLE_PROFILING 0
#endif
#ifndef BIRDNET_FRONTEND_CMSIS_CFFT_VECTOR_TAIL
#define BIRDNET_FRONTEND_CMSIS_CFFT_VECTOR_TAIL 0
#endif
#ifndef BIRDNET_FRONTEND_FFT_SELF_TEST
#define BIRDNET_FRONTEND_FFT_SELF_TEST 0
#endif
#ifndef BIRDNET_FRONTEND_FFT_SELF_TEST_FATAL
#define BIRDNET_FRONTEND_FFT_SELF_TEST_FATAL 0
#endif
#ifndef BIRDNET_FRONTEND_FFT_SELF_TEST_DIRECT_RFFT
#define BIRDNET_FRONTEND_FFT_SELF_TEST_DIRECT_RFFT 0
#endif

#define BIRDNET_BRANCH1_USE_KISS (BIRDNET_FRONTEND_BRANCH1_BACKEND == BIRDNET_FRONTEND_BACKEND_KISS)
#define BIRDNET_BRANCH1_USE_CMSIS_RFFT_SCALAR \
  (BIRDNET_FRONTEND_BRANCH1_BACKEND == BIRDNET_FRONTEND_BACKEND_CMSIS_RFFT_SCALAR)
#define BIRDNET_BRANCH1_USE_CMSIS_CFFT \
  (BIRDNET_FRONTEND_BRANCH1_BACKEND == BIRDNET_FRONTEND_BACKEND_CMSIS_CFFT)

#define BIRDNET_BRANCH2_USE_KISS (BIRDNET_FRONTEND_BRANCH2_BACKEND == BIRDNET_FRONTEND_BACKEND_KISS)
#define BIRDNET_BRANCH2_USE_CMSIS_RFFT_SCALAR \
  (BIRDNET_FRONTEND_BRANCH2_BACKEND == BIRDNET_FRONTEND_BACKEND_CMSIS_RFFT_SCALAR)
#define BIRDNET_BRANCH2_USE_CMSIS_CFFT \
  (BIRDNET_FRONTEND_BRANCH2_BACKEND == BIRDNET_FRONTEND_BACKEND_CMSIS_CFFT)

#if (BIRDNET_FRONTEND_BRANCH1_DECIMATE2 != 0) && !BIRDNET_BRANCH1_USE_CMSIS_RFFT_SCALAR
#error "BIRDNET_FRONTEND_BRANCH1_DECIMATE2 requires branch 1 CMSIS RFFT backend"
#endif
#if (BIRDNET_FRONTEND_BRANCH1_DECIMATE2 < 0) || (BIRDNET_FRONTEND_BRANCH1_DECIMATE2 > 1)
#error "BIRDNET_FRONTEND_BRANCH1_DECIMATE2 must be 0 or 1"
#endif
#if BIRDNET_FRONTEND_MEL_MVE_DOT_MIN_LEN < 1
#error "BIRDNET_FRONTEND_MEL_MVE_DOT_MIN_LEN must be >= 1"
#endif

#define BIRDNET_BRANCH1_USE_DECIMATE2_RFFT \
  (BIRDNET_BRANCH1_USE_CMSIS_RFFT_SCALAR && (BIRDNET_FRONTEND_BRANCH1_DECIMATE2 != 0))
#define BIRDNET_BRANCH1_RFFT_USE_C_OVERRIDE \
  (BIRDNET_BRANCH1_USE_CMSIS_RFFT_SCALAR && (BIRDNET_FRONTEND_BRANCH1_RFFT_USE_C_OVERRIDE != 0))
#define BIRDNET_BRANCH2_RFFT_USE_C_OVERRIDE \
  (BIRDNET_BRANCH2_USE_CMSIS_RFFT_SCALAR && (BIRDNET_FRONTEND_BRANCH2_RFFT_USE_C_OVERRIDE != 0))

#define BIRDNET_ANY_USE_CMSIS_RFFT_SCALAR \
  (BIRDNET_BRANCH1_USE_CMSIS_RFFT_SCALAR || BIRDNET_BRANCH2_USE_CMSIS_RFFT_SCALAR)
#define BIRDNET_ANY_USE_CMSIS_CFFT \
  (BIRDNET_BRANCH1_USE_CMSIS_CFFT || BIRDNET_BRANCH2_USE_CMSIS_CFFT)
#if BIRDNET_FRONTEND_FOLDED_REORDER && (!BIRDNET_FRONTEND_PARTIAL_REAL || \
    !BIRDNET_FRONTEND_CMSIS_CFFT_REPAIR_LAST_STAGE || !BIRDNET_BRANCH1_USE_DECIMATE2_RFFT || \
    !BIRDNET_BRANCH2_USE_CMSIS_RFFT_SCALAR || BIRDNET_BRANCH2_RFFT_USE_C_OVERRIDE)
#error "Field folded FFT requires the repaired partial-real profile"
#endif
constexpr bool kUseCompressionLut = true;

constexpr int kSpecMels = 96;
constexpr int kSpecFrames = 511;
constexpr int kSpecChannels = 2;
constexpr int kInputElems = kSpecMels * kSpecFrames * kSpecChannels;

constexpr bool kEnableExtendedClasses673 = (BIRDNET_ENABLE_EXTENDED_CLASSES_673 != 0);
constexpr int kNumClasses = BIRDNET_NUM_CLASSES;
constexpr int kTopK = 5;
// Keep LED/OLED activation aligned with SD detection logging by default.
constexpr float kDetectThreshold = kSdLogThreshold;

constexpr uintptr_t kModelFlashAddr = 0x3A200000;
constexpr int kTensorArenaBytes = 1088 * 1024;
constexpr bool kUseEthosDelegateForTflm = true;

}  // namespace cfg

// Production memory policy; experimental/reference builds explicitly opt out.
#ifndef BIRDNET_COMPACT_FRONTEND
#define BIRDNET_COMPACT_FRONTEND 0
#endif
#ifndef BIRDNET_ENABLE_PREDICTION_LOGS
#define BIRDNET_ENABLE_PREDICTION_LOGS 0
#endif
#ifndef BIRDNET_SKIP_ARENA_ZERO
#define BIRDNET_SKIP_ARENA_ZERO 0
#endif
#ifndef BIRDNET_VALIDATE_MEMORY
#define BIRDNET_VALIDATE_MEMORY 0
#endif

#ifndef BIRDNET_OPUS_ON_DEMAND
#define BIRDNET_OPUS_ON_DEMAND 0
#endif

#ifndef BIRDNET_BANK0_OFF
#define BIRDNET_BANK0_OFF 0
#endif

#ifndef BIRDNET_VALIDATE_CODEC
#define BIRDNET_VALIDATE_CODEC 0
#endif

static_assert(!BIRDNET_OPUS_ON_DEMAND || (BIRDNET_DETECTION_AUDIO_CODEC == 2 &&
    BIRDNET_COMPACT_FRONTEND && !BIRDNET_FRONTEND_STREAMING && !BIRDNET_VERIFY_OPUS_ON_WAKE),
    "On-demand codec requires compact, non-streaming production with explicit load-before-call");
static_assert(!BIRDNET_BANK0_OFF || BIRDNET_OPUS_ON_DEMAND,
    "Bank 0 off requires on-demand code and bootloader reload policy");
