// Private inference implementation; included at its original namespace scope.
#pragma once
// The production branches both execute 1024-point RFFTs. Reference/FFT-test
// configurations keep their original workspace and independently allocated frames.
#if BIRDNET_COMPACT_FRONTEND
static_assert(BIRDNET_BRANCH1_USE_DECIMATE2_RFFT && BIRDNET_BRANCH2_USE_CMSIS_RFFT_SCALAR &&
              BIRDNET_FRONTEND_RFFT_DIRECT_MEL && !cfg::kEnableDiagnosticLogs &&
              !BIRDNET_FRONTEND_FFT_SELF_TEST && !BIRDNET_ANY_USE_CMSIS_CFFT &&
              !BIRDNET_BRANCH1_USE_KISS && !BIRDNET_BRANCH2_USE_KISS,
              "Compact workspace requires the production direct-mel RFFT paths");
constexpr int kWorkspaceFftSize = kBranch1Decimate2FftSize;
#else
constexpr int kWorkspaceFftSize = birdnet_frontend::kBranch1FftSize;
#endif

struct ModelRuntime {
  bool npu_init = false;
  bool frontend_init = false;
  struct ethosu_driver ethosu_drv{};

  const tflite::Model* model = nullptr;
  tflite::MicroInterpreter* interpreter = nullptr;
  TfLiteTensor* input = nullptr;
  TfLiteTensor* output = nullptr;
  float input_inv_scale = 0.0f;
  int input_zero_point = 0;
  int8_t* input_int8_data = nullptr;
  float* input_float_data = nullptr;
  float input_quant_mul[2]{};
  float input_quant_add[2]{};

#if BIRDNET_BRANCH1_USE_CMSIS_RFFT_SCALAR
  arm_rfft_fast_instance_f32 rfft_branch1{};
#endif
#if BIRDNET_BRANCH2_USE_CMSIS_RFFT_SCALAR
  arm_rfft_fast_instance_f32 rfft_branch2{};
#endif
#if BIRDNET_BRANCH1_USE_CMSIS_CFFT
  const arm_cfft_instance_f32* cfft_branch1 = nullptr;
#endif
#if BIRDNET_BRANCH2_USE_CMSIS_CFFT
  const arm_cfft_instance_f32* cfft_branch2 = nullptr;
#endif
#if BIRDNET_ANY_USE_CMSIS_RFFT_SCALAR
  alignas(32) float fft_packed[kWorkspaceFftSize]{};
#endif
#if BIRDNET_ANY_USE_CMSIS_CFFT
  alignas(32) float fft_work[2 * birdnet_frontend::kBranch1FftSize]{};
#endif
#if BIRDNET_BRANCH1_USE_KISS
  alignas(32) uint8_t kiss_branch1_cfg[21 * 1024]{};
  size_t kiss_branch1_cfg_len = 0;
#endif
#if BIRDNET_BRANCH2_USE_KISS
  alignas(32) uint8_t kiss_branch2_cfg[12 * 1024]{};
  size_t kiss_branch2_cfg_len = 0;
#endif

#if BIRDNET_COMPACT_FRONTEND
  alignas(32) float frame[kWorkspaceFftSize]{};
  float* Frame1() { return frame; }
  float* Frame2() { return frame; }
#else
  alignas(32) float frame_branch1[birdnet_frontend::kBranch1FftSize]{};
  alignas(32) float frame_branch2[birdnet_frontend::kBranch2FftSize]{};
  float* Frame1() { return frame_branch1; }
  float* Frame2() { return frame_branch2; }
#endif
  alignas(32) float window_scale[kWorkspaceFftSize]{};
  alignas(32) float window_bias[kWorkspaceFftSize]{};
#if BIRDNET_BRANCH1_USE_KISS
  alignas(32) birdnet_kiss_fft_cpx fft_out_branch1[birdnet_frontend::kBranch1Bins]{};
#endif
#if BIRDNET_BRANCH2_USE_KISS
  alignas(32) birdnet_kiss_fft_cpx fft_out_branch2[birdnet_frontend::kBranch2Bins]{};
#endif
#if BIRDNET_COMPACT_FRONTEND
  // No unpacked real-bin buffer exists. All consumers are discarded constexpr
  // branches; a non-direct/diagnostic build is rejected above, not undersized.
  static constexpr float* real_bins_branch1 = nullptr;
  static constexpr float* real_bins_branch2 = nullptr;
#else
  alignas(32) float real_bins_branch1[birdnet_frontend::kBranch1Bins]{};
  alignas(32) float real_bins_branch2[birdnet_frontend::kBranch2Bins]{};
#endif
};

alignas(32) uint8_t g_tensor_arena[cfg::kTensorArenaBytes] BIRDNET_TENSOR_ARENA_BSS;
#if BIRDNET_FRONTEND_STREAMING
// Scratch need not survive PMU sleep. Move it beside the arena to leave room
// for the retained cache, without shrinking the existing PCM/Opus/stack buffers.
ModelRuntime g_rt __attribute__((used, section(".bss.audio_sram1")));
birdnet_streaming::Cache g_frontend_cache __attribute__((used, aligned(32), section(".frontend_cache")));
birdnet_streaming::State g_frontend_state __attribute__((used, section(".noinit.frontend")));
static_assert(cfg::kSampleRateHz == 48000 && cfg::kInferenceStepSamples > 0 &&
              cfg::kInferenceStepSamples <= cfg::kWindowSamples && cfg::kUseDualCoreFixedCadence,
              "Streaming requires 48 kHz, overlapping fixed cadence");
static_assert(BIRDNET_BRANCH1_USE_DECIMATE2_RFFT && BIRDNET_BRANCH2_USE_CMSIS_RFFT_SCALAR &&
              BIRDNET_FRONTEND_CMSIS_RFFT_USE_MVE_STAGE, "Streaming requires optimized RFFT paths");
#else
ModelRuntime g_rt BIRDNET_AUDIO_SRAM0_BSS;
#endif
