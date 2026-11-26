#ifdef ARM_MATH_MVEI
#undef ARM_MATH_MVEI
#endif
#ifdef ARM_MATH_MVEF
#undef ARM_MATH_MVEF
#endif
#ifdef ARM_MATH_HELIUM
#undef ARM_MATH_HELIUM
#endif

/*
 * CMSIS-DSP re-enables the MVE paths from compiler feature macros in
 * arm_math_types.h. Force the non-MVE implementations for this override unit
 * so the local FFT path cannot silently fall back to the vendor Helium code.
 */
#ifndef ARM_MATH_AUTOVECTORIZE
#define ARM_MATH_AUTOVECTORIZE
#endif

#include "arm_math.h"
#include "arm_const_structs.h"
#include "arm_common_tables.h"

#define arm_radix8_butterfly_f32 birdnet_arm_radix8_butterfly_f32
#define arm_bitreversal_64 birdnet_arm_bitreversal_64
#define arm_bitreversal_32 birdnet_arm_bitreversal_32
#define arm_bitreversal_16 birdnet_arm_bitreversal_16
#define arm_cfft_f32 birdnet_arm_cfft_f32
#define arm_rfft_fast_f32 birdnet_arm_rfft_fast_f32
#define stage_rfft_f32 birdnet_stage_rfft_f32
#define merge_rfft_f32 birdnet_merge_rfft_f32

#include "../../../library/cmsis_dsp/Source/TransformFunctions/arm_cfft_radix8_f32.c"
#include "../../../library/cmsis_dsp/Source/TransformFunctions/arm_bitreversal2.c"
#include "../../../library/cmsis_dsp/Source/TransformFunctions/arm_cfft_f32.c"
#include "../../../library/cmsis_dsp/Source/TransformFunctions/arm_rfft_fast_f32.c"

arm_status birdnet_arm_cfft_init_f32(arm_cfft_instance_f32* S, uint16_t fftLen) {
  if (S == NULL) {
    return ARM_MATH_ARGUMENT_ERROR;
  }

  switch (fftLen) {
    case 256U:
      S->fftLen = 256U;
      S->pTwiddle = twiddleCoef_256;
      S->pBitRevTable = armBitRevIndexTable256;
      S->bitRevLength = ARMBITREVINDEXTABLE_256_TABLE_LENGTH;
      return ARM_MATH_SUCCESS;
    case 512U:
      S->fftLen = 512U;
      S->pTwiddle = twiddleCoef_512;
      S->pBitRevTable = armBitRevIndexTable512;
      S->bitRevLength = ARMBITREVINDEXTABLE_512_TABLE_LENGTH;
      return ARM_MATH_SUCCESS;
    case 1024U:
      S->fftLen = 1024U;
      S->pTwiddle = twiddleCoef_1024;
      S->pBitRevTable = armBitRevIndexTable1024;
      S->bitRevLength = ARMBITREVINDEXTABLE_1024_TABLE_LENGTH;
      return ARM_MATH_SUCCESS;
    case 2048U:
      S->fftLen = 2048U;
      S->pTwiddle = twiddleCoef_2048;
      S->pBitRevTable = armBitRevIndexTable2048;
      S->bitRevLength = ARMBITREVINDEXTABLE_2048_TABLE_LENGTH;
      return ARM_MATH_SUCCESS;
    default:
      return ARM_MATH_ARGUMENT_ERROR;
  }
}

arm_status birdnet_arm_rfft_fast_init_f32(arm_rfft_fast_instance_f32* S, uint16_t fftLen) {
  if (S == NULL) {
    return ARM_MATH_ARGUMENT_ERROR;
  }

  arm_status status = ARM_MATH_ARGUMENT_ERROR;
  switch (fftLen) {
    case 512U:
      status = birdnet_arm_cfft_init_f32(&(S->Sint), 256U);
      if (status != ARM_MATH_SUCCESS) {
        return status;
      }
      S->fftLenRFFT = fftLen;
      S->pTwiddleRFFT = (const float32_t*)twiddleCoef_rfft_512;
      return ARM_MATH_SUCCESS;
    case 1024U:
      status = birdnet_arm_cfft_init_f32(&(S->Sint), 512U);
      if (status != ARM_MATH_SUCCESS) {
        return status;
      }
      S->fftLenRFFT = fftLen;
      S->pTwiddleRFFT = (const float32_t*)twiddleCoef_rfft_1024;
      return ARM_MATH_SUCCESS;
    case 2048U:
      status = birdnet_arm_cfft_init_f32(&(S->Sint), 1024U);
      if (status != ARM_MATH_SUCCESS) {
        return status;
      }
      S->fftLenRFFT = fftLen;
      S->pTwiddleRFFT = (const float32_t*)twiddleCoef_rfft_2048;
      return ARM_MATH_SUCCESS;
    default:
      return ARM_MATH_ARGUMENT_ERROR;
  }
}
