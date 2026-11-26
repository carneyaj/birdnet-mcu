#include "birdnet_format.h"
#include "inference_pipeline.h"
#include "birdnet_log.h"
#include "birdnet_detection_policy.h"
#include "birdnet_streaming_frontend.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <cstring>

#include "birdnet_frontend_assets.h"
#include "birdnet_frontend_tables.h"
#include "birdnet_model_contract.h"
#include "birdnet_labels.h"

extern "C" {
#include "WE2_core.h"
#include "WE2_device.h"
#include "board.h"
#include "hx_drv_scu.h"
#include "hx_drv_uart.h"
#include "../../../library/inference/tflmtag2209_u55tag2205/third_party/ethos_u_core_driver/include/ethosu_driver.h"
#include "xprintf.h"
}

#include "arm_math.h"
#include "arm_common_tables.h"
#include "arm_mve_tables.h"
#include "../../../library/inference/tflmtag2209_u55tag2205/tensorflow/lite/c/common.h"
#include "../../../library/inference/tflmtag2209_u55tag2205/tensorflow/lite/core/api/error_reporter.h"
#include "../../../library/inference/tflmtag2209_u55tag2205/tensorflow/lite/micro/micro_error_reporter.h"
#include "../../../library/inference/tflmtag2209_u55tag2205/tensorflow/lite/micro/micro_interpreter.h"
#include "../../../library/inference/tflmtag2209_u55tag2205/tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "../../../library/inference/tflmtag2209_u55tag2205/tensorflow/lite/schema/schema_generated.h"

#if   BIRDNET_SKIP_IMMUTABLE_CACHE_WALK
extern "C" int __real_ethosu_invoke_v3(ethosu_driver*,const void*,int,const uint64_t*,
                                      const size_t*,int,void*);
extern "C" int __wrap_ethosu_invoke_v3(ethosu_driver* drv,const void* stream,int bytes,
    const uint64_t* bases,const size_t* sizes,int n,void* user) {
  constexpr uint64_t first=cfg::kModelFlashAddr, end=first+birdnet_contract::kModelBytes;
  size_t adjusted[8];
  if (bases && sizes && n>0 && n<=8 && bases[0]>=first && bases[0]<end &&
      sizes[0]<=end-bases[0]) {
    // In the pinned WE2 driver this length is used only for CPU cache maintenance.
    std::copy_n(sizes,n,adjusted);
    adjusted[0]=0;
    sizes=adjusted;
  }
  return __real_ethosu_invoke_v3(drv,stream,bytes,bases,sizes,n,user);
}
#endif
namespace {

#define BIRDNET_AUDIO_SRAM0_BSS __attribute__((used, section(".bss.audio_sram0")))
#define BIRDNET_TENSOR_ARENA_BSS __attribute__((used, section(".tensor_arena")))

extern "C" {
typedef struct {
  float r;
  float i;
} birdnet_kiss_fft_cpx;
size_t birdnet_kiss_fftr_init_for_nfft(int nfft, void* mem, size_t mem_bytes);
void birdnet_kiss_fftr_run(void* cfg, const float* timedata, birdnet_kiss_fft_cpx* freqdata);
arm_status birdnet_arm_rfft_fast_init_f32(arm_rfft_fast_instance_f32* S, uint16_t fftLen);
void birdnet_arm_rfft_fast_f32(const arm_rfft_fast_instance_f32* S,
                               float32_t* p,
                               float32_t* pOut,
                               uint8_t ifftFlag);
arm_status birdnet_arm_cfft_init_f32(arm_cfft_instance_f32* S, uint16_t fftLen);
void birdnet_arm_cfft_f32(const arm_cfft_instance_f32* S,
                          float32_t* p1,
                          uint8_t ifftFlag,
                          uint8_t bitReverseFlag);
void birdnet_arm_cfft_f32_mve_repaired(const arm_cfft_instance_f32* S,
                                       float32_t* p1,
                                       uint8_t ifftFlag,
                                       uint8_t bitReverseFlag);
void stage_rfft_f32(const arm_rfft_fast_instance_f32* S, float32_t* p, float32_t* pOut);
}

#ifdef TRUSTZONE_SEC
constexpr uintptr_t kU55Base = BASE_ADDR_APB_U55_CTRL_ALIAS;
#else
#ifndef TRUSTZONE
constexpr uintptr_t kU55Base = BASE_ADDR_APB_U55_CTRL_ALIAS;
#else
constexpr uintptr_t kU55Base = BASE_ADDR_APB_U55_CTRL;
#endif
#endif

constexpr float kPi = 3.14159265358979323846f;
constexpr int kPowMantissaLutSize = 256;
constexpr int kPowExp2Min = -64;
constexpr int kPowExp2Max = 64;
constexpr int kPowExp2LutSize = kPowExp2Max - kPowExp2Min + 1;
constexpr int kBranch1Decimate2Factor = 2;
constexpr int kBranch1Decimate2FftSize =
    birdnet_frontend::kBranch1FftSize / kBranch1Decimate2Factor;
constexpr int kBranch1Decimate2Bins = (kBranch1Decimate2FftSize / 2) + 1;


#if BIRDNET_BRANCH1_USE_DECIMATE2_RFFT
static_assert((birdnet_frontend::kBranch1FftSize % kBranch1Decimate2Factor) == 0,
              "Branch 1 FFT size must be divisible by the decimation factor");
static_assert((birdnet_frontend::kBranch1Hop % kBranch1Decimate2Factor) == 0,
              "Branch 1 hop must be divisible by the decimation factor");
static_assert(kBranch1Decimate2FftSize == birdnet_frontend::kBranch2FftSize,
              "Branch 1 decimate-by-2 path assumes a 1024-point FFT");
#endif

#include "birdnet_frontend_workspace.h"

uintptr_t g_model_flash_addr = cfg::kModelFlashAddr;
constexpr bool kUseDirectRfftMel =
    (BIRDNET_FRONTEND_RFFT_DIRECT_MEL != 0) && !cfg::kEnableDiagnosticLogs;

#include "birdnet_frontend_math.h"

uint64_t PlatformMillis() {
  uint32_t tick = 0;
  uint32_t loop_cnt = 0;
  SystemGetTick(&tick, &loop_cnt);
  const uint64_t elapsed_ticks =
      static_cast<uint64_t>(loop_cnt) * static_cast<uint64_t>(SysTick_LOAD_RELOAD_Msk + 1u) +
      static_cast<uint64_t>(SysTick_LOAD_RELOAD_Msk + 1u - tick);
  return elapsed_ticks / (SystemCoreClock / 1000u);
}

void ArmNpuIrqHandler() {
  ethosu_irq_handler(&g_rt.ethosu_drv);
}

void ArmNpuIrqInit() {
  const IRQn_Type irqnum = static_cast<IRQn_Type>(U55_IRQn);
  EPII_NVIC_SetVector(irqnum, reinterpret_cast<uint32_t>(ArmNpuIrqHandler));
  NVIC_EnableIRQ(irqnum);
}

bool ArmNpuInit(bool security_enable, bool privilege_enable) {
  if (g_rt.npu_init) {
    birdnet_log::Status("[birdnet] Ethos-U55 already initialised\r\n");
    return true;
  }

  SCU_U55_STATE_E u55_state = SCU_U55_STATE_RESET;
  if (hx_drv_scu_get_u55_state(&u55_state) == SCU_NO_ERROR) {
    birdnet_log::Status("[birdnet] U55 state before init=%d\r\n", static_cast<int>(u55_state));
  } else {
    birdnet_log::Status("[birdnet] U55 state before init=<err>\r\n");
  }
  hx_drv_scu_set_u55_state(SCU_U55_STATE_RESET);
  hx_drv_scu_set_u55_state(SCU_U55_STATE_NORMAL);
  if (hx_drv_scu_get_u55_state(&u55_state) == SCU_NO_ERROR) {
    birdnet_log::Status("[birdnet] U55 state after reset=%d\r\n", static_cast<int>(u55_state));
  }

  ArmNpuIrqInit();

#if TFLM2209_U55TAG2205
  const void* ethosu_base_address = reinterpret_cast<const void*>(kU55Base);
#else
  void* const ethosu_base_address = reinterpret_cast<void*>(kU55Base);
#endif

  const int err = ethosu_init(
      &g_rt.ethosu_drv,
      ethosu_base_address,
      nullptr,
      0,
      security_enable,
      privilege_enable);
  if (err != 0) {
    birdnet_log::Write("[birdnet] failed to initialise Ethos-U device: %d\r\n", err);
    return false;
  }

  g_rt.npu_init = true;
  birdnet_log::Status("[birdnet] Ethos-U55 initialised\r\n");
  return true;
}

void ArmNpuDeinitForSleep() {
  if (!g_rt.npu_init) {
    return;
  }
  birdnet_log::Status("[birdnet] Ethos-U55 deinit for sleep\r\n");
  ethosu_deinit(&g_rt.ethosu_drv);
  std::memset(&g_rt.ethosu_drv, 0, sizeof(g_rt.ethosu_drv));
  g_rt.npu_init = false;
}


bool HasTfl3Header(uintptr_t addr) {
  const uint8_t* p = reinterpret_cast<const uint8_t*>(addr);
  return (p[4] == 'T' && p[5] == 'F' && p[6] == 'L' && p[7] == '3');
}

tflite::BuiltinOperator GetBuiltinOp(const tflite::OperatorCode* opcode) {
  if (!opcode) {
    return tflite::BuiltinOperator_CUSTOM;
  }
  const auto builtin = opcode->builtin_code();
  if (builtin == tflite::BuiltinOperator_PLACEHOLDER_FOR_GREATER_OP_CODES) {
    return static_cast<tflite::BuiltinOperator>(opcode->deprecated_builtin_code());
  }
  return builtin;
}

bool IsEthosOpcode(const tflite::OperatorCode* opcode) {
  if (!opcode || GetBuiltinOp(opcode) != tflite::BuiltinOperator_CUSTOM) {
    return false;
  }
  const auto* custom = opcode->custom_code();
  return custom && std::strcmp(custom->c_str(), "ethos-u") == 0;
}

bool IsPureEthosModel(const tflite::Model* model) {
  if (!model || !model->operator_codes() || !model->subgraphs() || model->subgraphs()->size() != 1) {
    return false;
  }
  const auto* subgraph = model->subgraphs()->Get(0);
  if (!subgraph || !subgraph->operators() || subgraph->operators()->size() != 1) {
    return false;
  }
  const auto* opcodes = model->operator_codes();
  if (opcodes->size() != 1) {
    return false;
  }
  return IsEthosOpcode(opcodes->Get(0));
}

void LogTensorShape(const char* name, TfLiteTensor* t) {
  if constexpr (!cfg::kEnableStatusLogs) {
    (void)name;
    (void)t;
    return;
  }
  if (!t || !t->dims) {
    return;
  }
  birdnet_log::Write("[birdnet] %s type=%d dims=", name, static_cast<int>(t->type));
  for (int i = 0; i < t->dims->size; ++i) {
    birdnet_log::Write("%d", t->dims->data[i]);
    if (i + 1 < t->dims->size) {
      birdnet_log::Write("x");
    }
  }
  birdnet_log::Write(" bytes=%d zp=%d scale_q1e6=%d\r\n",
          static_cast<int>(t->bytes),
          static_cast<int>(t->params.zero_point),
          static_cast<int>(t->params.scale * 1000000.0f));
}

#if BIRDNET_ANY_USE_CMSIS_RFFT_SCALAR
bool UseStaticRfftInstance(arm_rfft_fast_instance_f32* out, int fft_size) {
  if (!out) {
    return false;
  }
  switch (fft_size) {
    case 512:
      *out = kCmsisRfftLen512;
      return true;
    case 1024:
      *out = kCmsisRfftLen1024;
      return true;
    case 2048:
      *out = kCmsisRfftLen2048;
      return true;
    default:
      return false;
  }
}
#endif

#if BIRDNET_ANY_USE_CMSIS_CFFT
const arm_cfft_instance_f32* StaticCfftInstance(int fft_size) {
  switch (fft_size) {
    case 256:
      return &kCmsisCfftLen256;
    case 512:
      return &kCmsisCfftLen512;
    case 1024:
      return &kCmsisCfftLen1024;
    case 2048:
      return &kCmsisCfftLen2048;
    default:
      return nullptr;
  }
}
#endif

bool InitFrontend() {
  if (g_rt.frontend_init) {
    return true;
  }

  static_assert(sizeof(birdnet_tables::kWindow1) == birdnet_frontend::kBranch1FftSize * sizeof(float));
  static_assert(sizeof(birdnet_tables::kWindow2) == birdnet_frontend::kBranch2FftSize * sizeof(float));

#if BIRDNET_BRANCH1_USE_DECIMATE2_RFFT
  for (int mel_idx = 0; mel_idx < cfg::kSpecMels; ++mel_idx) {
    const int end_bin = static_cast<int>(birdnet_frontend::kBranch1MelStarts[mel_idx]) +
                        static_cast<int>(birdnet_frontend::kBranch1MelLengths[mel_idx]);
    if (end_bin > kBranch1Decimate2Bins) {
      birdnet_log::Write("[birdnet] branch0 decimate2 mel bin overflow mel=%d end=%d bins=%d\r\n",
              mel_idx,
              end_bin,
              kBranch1Decimate2Bins);
      return false;
    }
  }
#if BIRDNET_FRONTEND_CMSIS_RFFT_USE_MVE_STAGE
  if (!UseStaticRfftInstance(&g_rt.rfft_branch1, kBranch1Decimate2FftSize)) {
    birdnet_log::Write("[birdnet] branch0 decimate2 static RFFT init failed\r\n");
    return false;
  }
#else
  if (birdnet_arm_rfft_fast_init_f32(&g_rt.rfft_branch1, kBranch1Decimate2FftSize) !=
      ARM_MATH_SUCCESS) {
    birdnet_log::Write("[birdnet] branch0 decimate2 C override RFFT init failed\r\n");
    return false;
  }
#endif
#elif BIRDNET_BRANCH1_USE_CMSIS_RFFT_SCALAR
#if BIRDNET_BRANCH1_RFFT_USE_C_OVERRIDE
  if (birdnet_arm_rfft_fast_init_f32(&g_rt.rfft_branch1, birdnet_frontend::kBranch1FftSize) !=
      ARM_MATH_SUCCESS) {
    birdnet_log::Write("[birdnet] branch0 C override RFFT init failed\r\n");
    return false;
  }
#elif BIRDNET_FRONTEND_RFFT_USE_ARM_INIT
  if (arm_rfft_fast_init_f32(&g_rt.rfft_branch1, birdnet_frontend::kBranch1FftSize) !=
      ARM_MATH_SUCCESS) {
    birdnet_log::Write("[birdnet] branch0 ARM RFFT init failed\r\n");
    return false;
  }
#else
  if (!UseStaticRfftInstance(&g_rt.rfft_branch1, birdnet_frontend::kBranch1FftSize)) {
    birdnet_log::Write("[birdnet] branch0 static RFFT unsupported size=%d\r\n",
            birdnet_frontend::kBranch1FftSize);
    return false;
  }
#endif
#elif BIRDNET_BRANCH1_USE_CMSIS_CFFT
  g_rt.cfft_branch1 = StaticCfftInstance(birdnet_frontend::kBranch1FftSize);
  if (!g_rt.cfft_branch1) {
    birdnet_log::Write("[birdnet] branch0 static CFFT unsupported size=%d\r\n",
            birdnet_frontend::kBranch1FftSize);
    return false;
  }
#elif BIRDNET_BRANCH1_USE_KISS
  g_rt.kiss_branch1_cfg_len =
      birdnet_kiss_fftr_init_for_nfft(
          birdnet_frontend::kBranch1FftSize, g_rt.kiss_branch1_cfg, sizeof(g_rt.kiss_branch1_cfg));
  if (g_rt.kiss_branch1_cfg_len == 0 || g_rt.kiss_branch1_cfg_len > sizeof(g_rt.kiss_branch1_cfg)) {
    birdnet_log::Write("[birdnet] branch0 KISS FFT cfg too large need=%u have=%u\r\n",
            static_cast<unsigned int>(g_rt.kiss_branch1_cfg_len),
            static_cast<unsigned int>(sizeof(g_rt.kiss_branch1_cfg)));
    return false;
  }

#endif

#if BIRDNET_BRANCH2_USE_CMSIS_RFFT_SCALAR
#if BIRDNET_BRANCH2_RFFT_USE_C_OVERRIDE
  if (birdnet_arm_rfft_fast_init_f32(&g_rt.rfft_branch2, birdnet_frontend::kBranch2FftSize) !=
      ARM_MATH_SUCCESS) {
    birdnet_log::Write("[birdnet] branch2 C override RFFT init failed\r\n");
    return false;
  }
#elif BIRDNET_FRONTEND_RFFT_USE_ARM_INIT
  if (arm_rfft_fast_init_f32(&g_rt.rfft_branch2, birdnet_frontend::kBranch2FftSize) !=
      ARM_MATH_SUCCESS) {
    birdnet_log::Write("[birdnet] branch2 ARM RFFT init failed\r\n");
    return false;
  }
#else
  if (!UseStaticRfftInstance(&g_rt.rfft_branch2, birdnet_frontend::kBranch2FftSize)) {
    birdnet_log::Write("[birdnet] branch2 static RFFT unsupported size=%d\r\n",
            birdnet_frontend::kBranch2FftSize);
    return false;
  }
#endif
#elif BIRDNET_BRANCH2_USE_CMSIS_CFFT
  g_rt.cfft_branch2 = StaticCfftInstance(birdnet_frontend::kBranch2FftSize);
  if (!g_rt.cfft_branch2) {
    birdnet_log::Write("[birdnet] branch2 static CFFT unsupported size=%d\r\n",
            birdnet_frontend::kBranch2FftSize);
    return false;
  }
#elif BIRDNET_BRANCH2_USE_KISS
  g_rt.kiss_branch2_cfg_len =
      birdnet_kiss_fftr_init_for_nfft(
          birdnet_frontend::kBranch2FftSize, g_rt.kiss_branch2_cfg, sizeof(g_rt.kiss_branch2_cfg));
  if (g_rt.kiss_branch2_cfg_len == 0 || g_rt.kiss_branch2_cfg_len > sizeof(g_rt.kiss_branch2_cfg)) {
    birdnet_log::Write("[birdnet] branch2 KISS FFT cfg too large need=%u have=%u\r\n",
            static_cast<unsigned int>(g_rt.kiss_branch2_cfg_len),
            static_cast<unsigned int>(sizeof(g_rt.kiss_branch2_cfg)));
    return false;
  }
#endif

  if (cfg::kEnableDiagnosticLogs) {
    birdnet_log::Write("[birdnet] fft b0=%d b1=%d b0_decim2=%d arm_init=%d c_override=(%d,%d) params b0=(%d,%d,%d) b1=(%d,%d,%d) q1e3\r\n",
            BIRDNET_FRONTEND_BRANCH1_BACKEND,
            BIRDNET_FRONTEND_BRANCH2_BACKEND,
            BIRDNET_FRONTEND_BRANCH1_DECIMATE2,
            BIRDNET_FRONTEND_RFFT_USE_ARM_INIT,
            BIRDNET_FRONTEND_BRANCH1_RFFT_USE_C_OVERRIDE,
            BIRDNET_FRONTEND_BRANCH2_RFFT_USE_C_OVERRIDE,
            static_cast<int>(birdnet_frontend::kBranchPowerExponent[0] * 1000.0f),
            static_cast<int>(birdnet_frontend::kBatchNormMul[0] * 1000.0f),
            static_cast<int>(birdnet_frontend::kBatchNormAdd[0] * 1000.0f),
            static_cast<int>(birdnet_frontend::kBranchPowerExponent[1] * 1000.0f),
            static_cast<int>(birdnet_frontend::kBatchNormMul[1] * 1000.0f),
            static_cast<int>(birdnet_frontend::kBatchNormAdd[1] * 1000.0f));
  }

  g_rt.frontend_init = true;
  return true;
}

bool InputIsExpected(TfLiteTensor* input) {
  if (!input || !input->dims || input->dims->size != 4) {
    return false;
  }
  return input->dims->data[0] == 1 &&
         input->dims->data[1] == cfg::kSpecMels &&
         input->dims->data[2] == cfg::kSpecFrames &&
         input->dims->data[3] == cfg::kSpecChannels;
}

inline __attribute__((always_inline)) bool StoreCompressedFeatureAt(
    int flat_index,
    float compressed,
    int channel,
    float bn_mul,
    float bn_add) {
  if (g_rt.input_int8_data) {
    const int q = static_cast<int>(
        std::nearbyint(compressed * g_rt.input_quant_mul[channel] + g_rt.input_quant_add[channel]));
    g_rt.input_int8_data[flat_index] = static_cast<int8_t>(Clamp(q, -128, 127));
    return true;
  }
  if (g_rt.input_float_data) {
    g_rt.input_float_data[flat_index] = compressed * bn_mul + bn_add;
    return true;
  }
  birdnet_log::Write("[birdnet] unsupported input tensor type %d\r\n", static_cast<int>(g_rt.input ? g_rt.input->type : -1));
  return false;
}

void LogInputChannelStats() {
  if constexpr (!cfg::kEnableDiagnosticLogs) {
    return;
  }
  if (!g_rt.input || g_rt.input->type != kTfLiteInt8 || !g_rt.input->data.int8) {
    return;
  }

  int ch_min[cfg::kSpecChannels];
  int ch_max[cfg::kSpecChannels];
  for (int ch = 0; ch < cfg::kSpecChannels; ++ch) {
    ch_min[ch] = 127;
    ch_max[ch] = -128;
  }

  for (int mel_idx = 0; mel_idx < cfg::kSpecMels; ++mel_idx) {
    for (int frame_idx = 0; frame_idx < cfg::kSpecFrames; ++frame_idx) {
      const int base = ((mel_idx * cfg::kSpecFrames) + frame_idx) * cfg::kSpecChannels;
      for (int ch = 0; ch < cfg::kSpecChannels; ++ch) {
        const int q = static_cast<int>(g_rt.input->data.int8[base + ch]);
        ch_min[ch] = std::min(ch_min[ch], q);
        ch_max[ch] = std::max(ch_max[ch], q);
      }
    }
  }

  birdnet_log::Write("[birdnet] input ch0=[%d,%d] ch1=[%d,%d]\r\n",
          ch_min[0],
          ch_max[0],
          ch_min[1],
          ch_max[1]);
}

#if BIRDNET_FIELD_MEL_CODES
#include "birdnet_mel_codes.h"
#endif
#include "birdnet_frontend_channels.h"

void InsertTopK(int cls, float score, InferenceResult* out) {
  for (int rank = 0; rank < cfg::kTopK; ++rank) {
    if (score <= out->top_scores[rank]) {
      continue;
    }
    for (int move = cfg::kTopK - 1; move > rank; --move) {
      out->top_scores[move] = out->top_scores[move - 1];
      out->top_indices[move] = out->top_indices[move - 1];
    }
    out->top_scores[rank] = score;
    out->top_indices[rank] = cls;
    return;
  }
}

void InsertLogDetection(int cls, float score, InferenceResult* out) {
  if (!out || score < cfg::kSdLogThreshold) {
    return;
  }

  const int max_detections = cfg::kSdLogMaxDetectionsPerWindow;
  int insert_rank = 0;
  while (insert_rank < out->log_detection_count &&
         score <= out->log_detection_scores[insert_rank]) {
    ++insert_rank;
  }

  if (out->log_detection_count >= max_detections) {
    out->log_detection_overflow = true;
    if (insert_rank >= max_detections) {
      return;
    }
  } else {
    ++out->log_detection_count;
  }

  const int last = out->log_detection_count - 1;
  for (int move = last; move > insert_rank; --move) {
    out->log_detection_scores[move] = out->log_detection_scores[move - 1];
    out->log_detection_indices[move] = out->log_detection_indices[move - 1];
  }
  out->log_detection_scores[insert_rank] = score;
  out->log_detection_indices[insert_rank] = cls;
}

class UartErrorReporter : public tflite::ErrorReporter {
 public:
  int Report(const char* format, va_list args) override {
    char buf[256];
    const int n = birdnet_format::Vsnprintf(buf, sizeof(buf), format, args);
    if (n > 0) {
      birdnet_log::Write("[tflm] %s\r\n", buf);
    }
    return n;
  }
};

}  // namespace

TFLMInferencePipeline::TFLMInferencePipeline() = default;

void TFLMInferencePipeline::PrepareForSleep() {
  ArmNpuDeinitForSleep();
  initialized_ = false;
  init_ok_ = false;
}

void TFLMInferencePipeline::ResetStreamingCache() {
#if BIRDNET_FRONTEND_STREAMING
  g_frontend_state.magic = 0;
#endif
}

bool TFLMInferencePipeline::Init() {
  if (initialized_) {
    birdnet_log::Status("[birdnet] pipeline init cached ok=%d\r\n", init_ok_ ? 1 : 0);
    return init_ok_;
  }
  initialized_ = true;

  birdnet_log::Status("[birdnet] pipeline init begin\r\n");
  if (!InitFrontend()) {
    birdnet_log::Write("[birdnet] frontend init failed\r\n");
    init_ok_ = false;
    return false;
  }
  birdnet_log::Status("[birdnet] frontend init ok\r\n");


  if (cfg::kUseEthosDelegateForTflm && !ArmNpuInit(true, true)) {
    init_ok_ = false;
    return false;
  }

  g_rt.model = nullptr;
  constexpr uintptr_t main_model = cfg::kModelFlashAddr;
  birdnet_log::Status("[birdnet] model probe begin base=0x%08x mixed=%d\r\n",
                static_cast<unsigned int>(cfg::kModelFlashAddr),
                cfg::kEnableMixedModelSupport ? 1 : 0);
  if constexpr (cfg::kEnableMixedModelSupport) {
    constexpr uintptr_t kAddrCandidates[] = {
        cfg::kModelFlashAddr,
        cfg::kModelFlashAddr + 0x00100000u,
        0x3A200000u,
        0x3A300000u,
    };

    for (uintptr_t addr : kAddrCandidates) {
      if (!HasTfl3Header(addr)) {
        continue;
      }
      const tflite::Model* candidate = tflite::GetModel(reinterpret_cast<const void*>(addr));
      if (candidate && candidate->version() == TFLITE_SCHEMA_VERSION) {
        g_rt.model = candidate;
        g_model_flash_addr = addr;
        break;
      }
    }
  } else if (HasTfl3Header(main_model)) {
    birdnet_log::Status("[birdnet] model header ok @0x%08x\r\n",
                  static_cast<unsigned int>(cfg::kModelFlashAddr));
    const tflite::Model* candidate =
        tflite::GetModel(reinterpret_cast<const void*>(main_model));
    if (candidate && candidate->version() == TFLITE_SCHEMA_VERSION) {
      g_rt.model = candidate;
      g_model_flash_addr = main_model;
      birdnet_log::Status("[birdnet] model schema ok version=%d\r\n", candidate->version());
    } else {
      birdnet_log::Status("[birdnet] model schema bad candidate=%p\r\n", candidate);
    }
  } else {
    birdnet_log::Status("[birdnet] model header missing @0x%08x\r\n",
                  static_cast<unsigned int>(cfg::kModelFlashAddr));
  }
  if (!g_rt.model) {
    birdnet_log::Write("[birdnet] no valid model at candidate addresses\r\n");
    init_ok_ = false;
    return false;
  }

  static UartErrorReporter uart_error_reporter;
  if constexpr (!cfg::kEnableMixedModelSupport) {
    birdnet_log::Status("[birdnet] resolver/interpreter begin pure ethos\r\n");
    static tflite::MicroMutableOpResolver<1> resolver;
    static bool resolver_init = false;
    if (!resolver_init) {
      resolver.AddEthosU();
      resolver_init = true;
    }
    static tflite::MicroInterpreter static_interpreter(
        g_rt.model,
        resolver,
        g_tensor_arena,
        cfg::kTensorArenaBytes,
        &uart_error_reporter);
    g_rt.interpreter = &static_interpreter;
    birdnet_log::Status("[birdnet] resolver/interpreter ok pure ethos\r\n");
  } else {
    const bool pure_ethos_model = cfg::kUseEthosDelegateForTflm && IsPureEthosModel(g_rt.model);

    if (pure_ethos_model) {
      birdnet_log::Status("[birdnet] resolver/interpreter begin mixed-pure ethos\r\n");
      static tflite::MicroMutableOpResolver<1> resolver;
      static bool resolver_init = false;
      if (!resolver_init) {
        resolver.AddEthosU();
        resolver_init = true;
      }
      static tflite::MicroInterpreter static_interpreter(
          g_rt.model,
          resolver,
          g_tensor_arena,
          cfg::kTensorArenaBytes,
          &uart_error_reporter);
      g_rt.interpreter = &static_interpreter;
      birdnet_log::Status("[birdnet] resolver/interpreter ok mixed-pure ethos\r\n");
    } else {
      birdnet_log::Status("[birdnet] resolver/interpreter begin mixed cpu/ethos\r\n");
      static tflite::MicroMutableOpResolver<16> resolver;
      static bool resolver_init = false;
      if (!resolver_init) {
        if (cfg::kUseEthosDelegateForTflm) {
          resolver.AddEthosU();
        }
        resolver.AddReshape();
        resolver.AddQuantize();
        resolver.AddDequantize();
        resolver.AddConv2D();
        resolver.AddDepthwiseConv2D();
        resolver.AddFullyConnected();
        resolver.AddAveragePool2D();
        resolver.AddAdd();
        resolver.AddMul();
        resolver.AddPad();
        resolver.AddMean();
        resolver.AddLogistic();
        resolver.AddHardSwish();
        resolver.AddConcatenation();
        resolver_init = true;
      }
      static tflite::MicroInterpreter static_interpreter(
          g_rt.model,
          resolver,
          g_tensor_arena,
          cfg::kTensorArenaBytes,
          &uart_error_reporter);
      g_rt.interpreter = &static_interpreter;
      birdnet_log::Status("[birdnet] resolver/interpreter ok mixed cpu/ethos\r\n");
    }
  }

  birdnet_log::Status("[birdnet] AllocateTensors begin\r\n");
  if (g_rt.interpreter->AllocateTensors() != kTfLiteOk) {
    birdnet_log::Write("[birdnet] AllocateTensors failed\r\n");
    init_ok_ = false;
    return false;
  }
  birdnet_log::Status("[birdnet] AllocateTensors ok\r\n");

  g_rt.input = g_rt.interpreter->input(0);
  g_rt.output = g_rt.interpreter->output(0);
  g_rt.input_inv_scale = 0.0f;
  g_rt.input_zero_point = 0;
  g_rt.input_int8_data = nullptr;
  g_rt.input_float_data = nullptr;
  for (int channel = 0; channel < 2; ++channel) {
    g_rt.input_quant_mul[channel] = 0.0f;
    g_rt.input_quant_add[channel] = 0.0f;
  }

  birdnet_log::Status("[birdnet] model init ok @0x%08x arena_used=%u\r\n",
                static_cast<unsigned int>(g_model_flash_addr),
                static_cast<unsigned int>(g_rt.interpreter->arena_used_bytes()));
  LogTensorShape("input", g_rt.input);
  LogTensorShape("output", g_rt.output);

  if (!InputIsExpected(g_rt.input)) {
    birdnet_log::Write("[birdnet] unexpected input tensor shape\r\n");
    init_ok_ = false;
    return false;
  }

  static_assert(birdnet_contract::kClasses == cfg::kNumClasses, "Model/label class count mismatch");
  constexpr auto expected_classes = cfg::kNumClasses;
  constexpr auto expected_scale = birdnet_contract::kOutputScale;
  constexpr auto expected_zero = birdnet_contract::kOutputZero;
  if (!g_rt.output || !g_rt.output->dims || g_rt.output->dims->size != 2 ||
      g_rt.output->dims->data[0] != 1 || g_rt.output->dims->data[1] != expected_classes ||
      g_rt.input->type != kTfLiteInt8 || g_rt.output->type != kTfLiteInt8 ||
      g_rt.input->params.scale != birdnet_contract::kInputScale ||
      g_rt.input->params.zero_point != birdnet_contract::kInputZero ||
      g_rt.output->params.scale != expected_scale ||
      g_rt.output->params.zero_point != expected_zero) {
    birdnet_log::Write("[birdnet] model does not match compiled deployment contract\r\n");
    init_ok_ = false;
    return false;
  }
  birdnet_log::Status("[birdnet] deployment=%s\r\n", birdnet_contract::kId);

  if (g_rt.input->type == kTfLiteInt8) {
    if (g_rt.input->params.scale <= 0.0f) {
      birdnet_log::Write("[birdnet] invalid input scale\r\n");
      init_ok_ = false;
      return false;
    }
    g_rt.input_inv_scale = 1.0f / g_rt.input->params.scale;
    g_rt.input_zero_point = g_rt.input->params.zero_point;
    g_rt.input_int8_data = g_rt.input->data.int8;
    for (int channel = 0; channel < 2; ++channel) {
      g_rt.input_quant_mul[channel] =
          birdnet_frontend::kBatchNormMul[channel] * g_rt.input_inv_scale;
      g_rt.input_quant_add[channel] =
          birdnet_frontend::kBatchNormAdd[channel] * g_rt.input_inv_scale +
          static_cast<float>(g_rt.input_zero_point);
    }
  } else if (g_rt.input->type == kTfLiteFloat32) {
    g_rt.input_float_data = g_rt.input->data.f;
  }

  init_ok_ = (g_rt.input != nullptr && g_rt.output != nullptr);
  if (init_ok_ && cfg::kUseEthosDelegateForTflm && BIRDNET_NPU_GATE_DURING_FRONTEND) {
    // Device init leaves the U55 powered until a CMD write. A driver soft reset
    // with no outstanding power request writes CMD with Q-gating enabled.
    if (!ethosu_soft_reset(&g_rt.ethosu_drv)) {
      birdnet_log::Write("[birdnet] U55 frontend Q-gating failed\r\n");
      init_ok_ = false;
    } else {
      birdnet_log::Status("[birdnet] U55 frontend Q-gating enabled\r\n");
    }
  }
  return init_ok_;
}

#include "birdnet_frontend_input.h"

template<int LookupMode>
static bool DecodeScores(const TfLiteTensor* tensor, int classes, const uint64_t* recording_previous_ends_,
                         uint64_t recording_end_samples_, InferenceResult* out, const float* cached=nullptr) {
  for (int i = 0; i < cfg::kTopK; ++i) {
    out->top_indices[i] = -1;
    out->top_scores[i] = -1.0f;
  }
  out->log_detection_count = 0;
  out->log_detection_overflow = false;
  out->human_vocal_detected = false;
  for (int i = 0; i < cfg::kSdLogMaxDetectionsPerWindow; ++i) {
    out->log_detection_indices[i] = -1;
    out->log_detection_scores[i] = -1.0f;
  }

  static bool s_logged_output_once = false;
  for (int i = 0; i < classes; ++i) {
    float score;
      float logit;
      if (tensor->type == kTfLiteInt8) {
        logit = (int(tensor->data.int8[i]) - tensor->params.zero_point) * tensor->params.scale;
      } else if (tensor->type == kTfLiteUInt8) {
        logit = (int(tensor->data.uint8[i]) - tensor->params.zero_point) * tensor->params.scale;
      } else if (tensor->type == kTfLiteFloat32) {
        logit = tensor->data.f[i];
      } else {
        birdnet_log::Write("[birdnet] unsupported output type %d\r\n", static_cast<int>(tensor->type));
        return false;
      }
      score = Sigmoid(logit);
    if (score >= cfg::kHumanVocalThreshold &&
        birdnet_labels::IsHumanVocal(i)) {
      out->human_vocal_detected = true;
    }
    InsertTopK(i, score, out);
    if constexpr (cfg::kEnableSdDetectionLogging) {
      // Filter before the 16-record limit so suppressed high-scoring species
      // cannot crowd a different, eligible species out of the recording.
      if (!recording_previous_ends_ ||
          birdnet_detection::CanRecord(recording_previous_ends_[i],
                                      recording_end_samples_, cfg::kWindowSamples)) {
        InsertLogDetection(i, score, out);
      }
    }
  }

  out->pred_class = out->top_indices[0];
  out->pred_score = out->top_scores[0];
  out->detected = out->pred_score >= cfg::kDetectThreshold;

  if (!s_logged_output_once && tensor->type == kTfLiteInt8) {
    if constexpr (cfg::kEnableStatusLogs) {
      birdnet_log::Write("[birdnet] output raw first8=");
      for (int i = 0; i < 8 && i < cfg::kNumClasses; ++i) {
        birdnet_log::Write("%d", static_cast<int>(tensor->data.int8[i]));
        if (i + 1 < 8 && i + 1 < cfg::kNumClasses) {
          birdnet_log::Write(",");
        }
      }
      birdnet_log::Write("\r\n");
    }
    s_logged_output_once = true;
  }

  return true;
}

bool TFLMInferencePipeline::ReadOutput(InferenceResult* out) {
  if (!out || !g_rt.output || !g_rt.output->dims || g_rt.output->dims->size < 2) return false;
  const int classes = g_rt.output->dims->data[g_rt.output->dims->size - 1];
  if (classes != cfg::kNumClasses) {
    birdnet_log::Write("[birdnet] output classes %d != expected %d\r\n", classes, cfg::kNumClasses);
    return false;
  }
  return DecodeScores<false>(g_rt.output, cfg::kNumClasses, recording_previous_ends_, recording_end_samples_, out);
}

bool TFLMInferencePipeline::Run(const int16_t* pcm_window, InferenceResult* out) {
  if (!out) {
    return false;
  }
  const uint64_t init_start = PlatformMillis();
  if (!Init()) {
    return false;
  }

  const uint64_t t0 = PlatformMillis();
  out->init_ms = static_cast<float>(t0 - init_start);
  birdnet_log::Status("[birdnet] frontend fill begin\r\n");
  if (!FillInputTensorFromBirdNetFrontend(pcm_window)) {
    return false;
  }
  const uint64_t t_frontend = PlatformMillis();
  birdnet_log::Status("[birdnet] frontend fill ok ms=%lu\r\n",
                static_cast<unsigned long>(t_frontend - t0));
  birdnet_log::Status("[birdnet] invoke begin\r\n");
  const TfLiteStatus invoke_status = g_rt.interpreter->Invoke();
  if (invoke_status != kTfLiteOk) {
    birdnet_log::Write("[birdnet] Invoke failed\r\n");
    return false;
  }
  const uint64_t t_invoke = PlatformMillis();
  birdnet_log::Status("[birdnet] invoke ok ms=%lu\r\n",
                static_cast<unsigned long>(t_invoke - t_frontend));
  if (!ReadOutput(out)) {
    return false;
  }
  const uint64_t t_done = PlatformMillis();
  out->frontend_ms = static_cast<float>(t_frontend - t0);
  out->invoke_ms = static_cast<float>(t_invoke - t_frontend);
  out->decode_ms = static_cast<float>(t_done - t_invoke);
  out->inference_ms = static_cast<float>(t_done - t0);
  return true;
}

bool TFLMInferencePipeline::RunFromRing(
    const int16_t* pcm_ring,
    size_t ring_size,
    size_t write_idx,
    InferenceResult* out) {
  if (!out) {
    return false;
  }
  if (!Init()) {
    return false;
  }

  const uint64_t t0 = PlatformMillis();
  birdnet_log::Status("[birdnet] frontend fill ring begin\r\n");
  if (!FillInputTensorFromBirdNetFrontendRing(pcm_ring, ring_size, write_idx)) {
    return false;
  }
  const uint64_t t_frontend = PlatformMillis();
  birdnet_log::Status("[birdnet] frontend fill ring ok ms=%lu\r\n",
                static_cast<unsigned long>(t_frontend - t0));
  birdnet_log::Status("[birdnet] invoke begin\r\n");
  const TfLiteStatus invoke_status = g_rt.interpreter->Invoke();
  if (invoke_status != kTfLiteOk) {
    birdnet_log::Write("[birdnet] Invoke failed\r\n");
    return false;
  }
  const uint64_t t_invoke = PlatformMillis();
  birdnet_log::Status("[birdnet] invoke ok ms=%lu\r\n",
                static_cast<unsigned long>(t_invoke - t_frontend));
  if (!ReadOutput(out)) {
    return false;
  }
  const uint64_t t_done = PlatformMillis();
  out->frontend_ms = static_cast<float>(t_frontend - t0);
  out->invoke_ms = static_cast<float>(t_invoke - t_frontend);
  out->decode_ms = static_cast<float>(t_done - t_invoke);
  out->inference_ms = static_cast<float>(t_done - t0);
  return true;
}
