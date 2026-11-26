#include "arm_math.h"
#include "arm_common_tables.h"

#ifndef BIRDNET_FRONTEND_CMSIS_CFFT_VECTOR_TAIL
#define BIRDNET_FRONTEND_CMSIS_CFFT_VECTOR_TAIL 0
#endif

#if defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE)

#include "arm_helium_utils.h"
#include "arm_vec_fft.h"
#include "arm_mve_tables.h"

static void birdnet_scalar_final_radix4_forward_f32(float32_t *data, uint32_t fftLen)
{
    for (uint32_t i = 0; i < fftLen; i += 4U)
    {
        float32_t *a = data + (2U * i);
        float32_t *b = a + 2U;
        float32_t *c = a + 4U;
        float32_t *d = a + 6U;

        const float32_t ar = a[0];
        const float32_t ai = a[1];
        const float32_t br = b[0];
        const float32_t bi = b[1];
        const float32_t cr = c[0];
        const float32_t ci = c[1];
        const float32_t dr = d[0];
        const float32_t di = d[1];

        const float32_t sum0_r = ar + cr;
        const float32_t sum0_i = ai + ci;
        const float32_t diff0_r = ar - cr;
        const float32_t diff0_i = ai - ci;
        const float32_t sum1_r = br + dr;
        const float32_t sum1_i = bi + di;
        const float32_t diff1_r = br - dr;
        const float32_t diff1_i = bi - di;

        a[0] = sum0_r + sum1_r;
        a[1] = sum0_i + sum1_i;
        b[0] = sum0_r - sum1_r;
        b[1] = sum0_i - sum1_i;
        c[0] = diff0_r + diff1_i;
        c[1] = diff0_i - diff1_r;
        d[0] = diff0_r - diff1_i;
        d[1] = diff0_i + diff1_r;
    }
}

#if BIRDNET_FRONTEND_CMSIS_CFFT_VECTOR_TAIL
static void birdnet_mve_final_radix4_forward_f32(float32_t *data, uint32_t fftLen)
{
    static const uint32_t lane_offsets[4] = {0U, 4U, 32U, 36U};
    uint32x4_t addresses = vld1q_u32(lane_offsets) + (uint32_t)data;

    /* Each vector holds the real/imaginary pairs for two butterflies. */
    for (uint32_t i = 0; i < fftLen; i += 8U)
    {
        const f32x4_t vecA = vldrwq_gather_base_f32(addresses, 0);
        const f32x4_t vecB = vldrwq_gather_base_f32(addresses, 8);
        const f32x4_t vecC = vldrwq_gather_base_f32(addresses, 16);
        const f32x4_t vecD = vldrwq_gather_base_f32(addresses, 24);

        const f32x4_t vecSum0 = vecA + vecC;
        const f32x4_t vecDiff0 = vecA - vecC;
        const f32x4_t vecSum1 = vecB + vecD;
        const f32x4_t vecDiff1 = vecB - vecD;

        vstrwq_scatter_base_f32(addresses, 0, vecSum0 + vecSum1);
        vstrwq_scatter_base_f32(addresses, 8, vecSum0 - vecSum1);
        vstrwq_scatter_base_f32(addresses, 16, MVE_CMPLX_SUB_A_ixB(vecDiff0, vecDiff1));
        vstrwq_scatter_base_f32(addresses, 24, MVE_CMPLX_ADD_A_ixB(vecDiff0, vecDiff1));
        addresses += 64U;
    }
}
#endif

static void birdnet_scalar_bitreversal_32(uint32_t *pSrc,
                                          uint16_t bitRevLen,
                                          const uint16_t *pBitRevTab)
{
    for (uint16_t i = 0; i < bitRevLen; i += 2U)
    {
        const uint32_t a = pBitRevTab[i] >> 2;
        const uint32_t b = pBitRevTab[i + 1U] >> 2;

        uint32_t tmp = pSrc[a];
        pSrc[a] = pSrc[b];
        pSrc[b] = tmp;

        tmp = pSrc[a + 1U];
        pSrc[a + 1U] = pSrc[b + 1U];
        pSrc[b + 1U] = tmp;
    }
}

static void birdnet_radix4_butterfly_f32_mve_main_only(const arm_cfft_instance_f32 *S,
                                                       float32_t *pSrc,
                                                       uint32_t fftLen)
{
    f32x4_t vecTmp0, vecTmp1;
    f32x4_t vecSum0, vecDiff0, vecSum1, vecDiff1;
    f32x4_t vecA, vecB, vecC, vecD;
    uint32_t blkCnt;
    uint32_t n1, n2;
    uint32_t stage = 0;
    int32_t iter = 1;

    n2 = fftLen;
    n1 = n2;
    n2 >>= 2U;
    for (uint32_t k = fftLen / 4U; k > 1U; k >>= 2U)
    {
        const float32_t *p_rearranged_twiddle_tab_stride1 =
            &S->rearranged_twiddle_stride1[S->rearranged_twiddle_tab_stride1_arr[stage]];
        const float32_t *p_rearranged_twiddle_tab_stride2 =
            &S->rearranged_twiddle_stride2[S->rearranged_twiddle_tab_stride2_arr[stage]];
        const float32_t *p_rearranged_twiddle_tab_stride3 =
            &S->rearranged_twiddle_stride3[S->rearranged_twiddle_tab_stride3_arr[stage]];

        float32_t *pBase = pSrc;
        for (int32_t i = 0; i < iter; i++)
        {
            float32_t *inA = pBase;
            float32_t *inB = inA + n2 * CMPLX_DIM;
            float32_t *inC = inB + n2 * CMPLX_DIM;
            float32_t *inD = inC + n2 * CMPLX_DIM;
            const float32_t *pW1 = p_rearranged_twiddle_tab_stride1;
            const float32_t *pW2 = p_rearranged_twiddle_tab_stride2;
            const float32_t *pW3 = p_rearranged_twiddle_tab_stride3;
            f32x4_t vecW;

            blkCnt = n2 / 2U;
            vecA = vldrwq_f32(inA);
            vecC = vldrwq_f32(inC);
            while (blkCnt > 0U)
            {
                vecB = vldrwq_f32(inB);
                vecD = vldrwq_f32(inD);

                vecSum0 = vecA + vecC;
                vecDiff0 = vecA - vecC;

                vecSum1 = vecB + vecD;
                vecDiff1 = vecB - vecD;

                vecTmp0 = vecSum0 + vecSum1;
                vst1q(inA, vecTmp0);
                inA += 4;

                vecTmp0 = vecSum0 - vecSum1;
                vecW = vld1q(pW2);
                pW2 += 4;
                vecTmp1 = MVE_CMPLX_MULT_FLT_Conj_AxB(vecW, vecTmp0);
                vst1q(inB, vecTmp1);
                inB += 4;

                vecTmp0 = MVE_CMPLX_SUB_A_ixB(vecDiff0, vecDiff1);
                vecW = vld1q(pW1);
                pW1 += 4;
                vecTmp1 = MVE_CMPLX_MULT_FLT_Conj_AxB(vecW, vecTmp0);
                vst1q(inC, vecTmp1);
                inC += 4;

                vecTmp0 = MVE_CMPLX_ADD_A_ixB(vecDiff0, vecDiff1);
                vecW = vld1q(pW3);
                pW3 += 4;
                vecTmp1 = MVE_CMPLX_MULT_FLT_Conj_AxB(vecW, vecTmp0);
                vst1q(inD, vecTmp1);
                inD += 4;

                vecA = vldrwq_f32(inA);
                vecC = vldrwq_f32(inC);

                blkCnt--;
            }
            pBase += CMPLX_DIM * n1;
        }
        n1 = n2;
        n2 >>= 2U;
        iter <<= 2;
        stage++;
    }

#if BIRDNET_FRONTEND_CMSIS_CFFT_VECTOR_TAIL
    birdnet_mve_final_radix4_forward_f32(pSrc, fftLen);
#else
    birdnet_scalar_final_radix4_forward_f32(pSrc, fftLen);
#endif
}

static void birdnet_cfft_radix4by2_f32_mve_repaired(const arm_cfft_instance_f32 *S,
                                                    float32_t *pSrc,
                                                    uint32_t fftLen)
{
    const float32_t *pCoefVec = S->pTwiddle;
    float32_t *pIn0 = pSrc;
    float32_t *pIn1 = pSrc + fftLen;
    const uint32_t n2 = fftLen >> 1U;
    uint32_t blkCnt = n2 / 2U;

    while (blkCnt > 0U)
    {
        const f32x4_t vecIn0 = *(f32x4_t *)pIn0;
        const f32x4_t vecIn1 = *(f32x4_t *)pIn1;
        const f32x4_t vecTw = vld1q(pCoefVec);
        pCoefVec += 4;

        const f32x4_t vecSum = vecIn0 + vecIn1;
        const f32x4_t vecDiff = vecIn0 - vecIn1;
        const f32x4_t vecCmplxTmp = MVE_CMPLX_MULT_FLT_Conj_AxB(vecTw, vecDiff);

        vst1q(pIn0, vecSum);
        pIn0 += 4;
        vst1q(pIn1, vecCmplxTmp);
        pIn1 += 4;

        blkCnt--;
    }

    birdnet_radix4_butterfly_f32_mve_main_only(S, pSrc, n2);
    birdnet_radix4_butterfly_f32_mve_main_only(S, pSrc + fftLen, n2);
}

void birdnet_arm_cfft_f32_mve_repaired(const arm_cfft_instance_f32 *S,
                                       float32_t *pSrc,
                                       uint8_t ifftFlag,
                                       uint8_t bitReverseFlag)
{
    const uint32_t fftLen = S->fftLen;

    if (ifftFlag != 0U)
    {
        arm_cfft_f32(S, pSrc, ifftFlag, bitReverseFlag);
        return;
    }

    switch (fftLen)
    {
    case 16:
    case 64:
    case 256:
    case 1024:
    case 4096:
        birdnet_radix4_butterfly_f32_mve_main_only(S, pSrc, fftLen);
        break;

    case 32:
    case 128:
    case 512:
    case 2048:
        birdnet_cfft_radix4by2_f32_mve_repaired(S, pSrc, fftLen);
        break;

    default:
        arm_cfft_f32(S, pSrc, ifftFlag, bitReverseFlag);
        return;
    }

    if (bitReverseFlag != 0U)
    {
        birdnet_scalar_bitreversal_32((uint32_t *)pSrc, S->bitRevLength, S->pBitRevTable);
    }
}

#else

void birdnet_arm_cfft_f32_mve_repaired(const arm_cfft_instance_f32 *S,
                                       float32_t *pSrc,
                                       uint8_t ifftFlag,
                                       uint8_t bitReverseFlag)
{
    arm_cfft_f32(S, pSrc, ifftFlag, bitReverseFlag);
}

#endif
