// Private inference implementation; included at its original namespace scope.
#pragma once
#if BIRDNET_ANY_USE_CMSIS_RFFT_SCALAR || BIRDNET_ANY_USE_CMSIS_CFFT
#if defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE)
// The SDK build trims out the prebuilt arm_cfft_sR_f32_len* descriptors on MVE,
// but it still ships the underlying twiddle and bit-reversal tables we need.
constexpr arm_cfft_instance_f32 kCmsisCfftLen256 = {
    256,
    twiddleCoef_256,
    armBitRevIndexTable_fixed_256,
    ARMBITREVINDEXTABLE_FIXED_256_TABLE_LENGTH,
    rearranged_twiddle_tab_stride1_arr_256_f32,
    rearranged_twiddle_tab_stride2_arr_256_f32,
    rearranged_twiddle_tab_stride3_arr_256_f32,
    rearranged_twiddle_stride1_256_f32,
    rearranged_twiddle_stride2_256_f32,
    rearranged_twiddle_stride3_256_f32,
};

constexpr arm_cfft_instance_f32 kCmsisCfftLen512 = {
    512,
    twiddleCoef_512,
    armBitRevIndexTable_fixed_512,
    ARMBITREVINDEXTABLE_FIXED_512_TABLE_LENGTH,
    rearranged_twiddle_tab_stride1_arr_256_f32,
    rearranged_twiddle_tab_stride2_arr_256_f32,
    rearranged_twiddle_tab_stride3_arr_256_f32,
    rearranged_twiddle_stride1_256_f32,
    rearranged_twiddle_stride2_256_f32,
    rearranged_twiddle_stride3_256_f32,
};

constexpr arm_cfft_instance_f32 kCmsisCfftLen1024 = {
    1024,
    twiddleCoef_1024,
    armBitRevIndexTable_fixed_1024,
    ARMBITREVINDEXTABLE_FIXED_1024_TABLE_LENGTH,
    rearranged_twiddle_tab_stride1_arr_1024_f32,
    rearranged_twiddle_tab_stride2_arr_1024_f32,
    rearranged_twiddle_tab_stride3_arr_1024_f32,
    rearranged_twiddle_stride1_1024_f32,
    rearranged_twiddle_stride2_1024_f32,
    rearranged_twiddle_stride3_1024_f32,
};

// CMSIS arm_cfft_init_f32() uses the 1024 rearranged MVE tables for 2048-point
// FFTs with twidCoefModifier=2, so mirror that layout here.
constexpr arm_cfft_instance_f32 kCmsisCfftLen2048 = {
    2048,
    twiddleCoef_2048,
    armBitRevIndexTable_fixed_2048,
    ARMBITREVINDEXTABLE_FIXED_2048_TABLE_LENGTH,
    rearranged_twiddle_tab_stride1_arr_1024_f32,
    rearranged_twiddle_tab_stride2_arr_1024_f32,
    rearranged_twiddle_tab_stride3_arr_1024_f32,
    rearranged_twiddle_stride1_1024_f32,
    rearranged_twiddle_stride2_1024_f32,
    rearranged_twiddle_stride3_1024_f32,
};

constexpr arm_rfft_fast_instance_f32 kCmsisRfftLen1024 = {
    kCmsisCfftLen512,
    1024U,
    twiddleCoef_rfft_1024,
};

constexpr arm_rfft_fast_instance_f32 kCmsisRfftLen512 = {
    kCmsisCfftLen256,
    512U,
    twiddleCoef_rfft_512,
};

constexpr arm_rfft_fast_instance_f32 kCmsisRfftLen2048 = {
    kCmsisCfftLen1024,
    2048U,
    twiddleCoef_rfft_2048,
};
#else
#error "CMSIS FFT frontends require ARM_MATH_MVEF on this target"
#endif
#endif

#if BIRDNET_ANY_USE_CMSIS_RFFT_SCALAR || BIRDNET_ANY_USE_CMSIS_CFFT
void RunScalarFinalRadix4Forward(float* data, int fft_len) {
  for (int i = 0; i < fft_len; i += 4) {
    float* a = data + (2 * i);
    float* b = a + 2;
    float* c = a + 4;
    float* d = a + 6;

    const float ar = a[0];
    const float ai = a[1];
    const float br = b[0];
    const float bi = b[1];
    const float cr = c[0];
    const float ci = c[1];
    const float dr = d[0];
    const float di = d[1];

    const float sum0_r = ar + cr;
    const float sum0_i = ai + ci;
    const float diff0_r = ar - cr;
    const float diff0_i = ai - ci;
    const float sum1_r = br + dr;
    const float sum1_i = bi + di;
    const float diff1_r = br - dr;
    const float diff1_i = bi - di;

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

void RunScalarBitReverseComplex(float* data, uint16_t bit_rev_len, const uint16_t* bit_rev_table) {
  for (uint16_t i = 0; i < bit_rev_len; i += 2) {
    const uint16_t a = bit_rev_table[i] >> 2;
    const uint16_t b = bit_rev_table[i + 1] >> 2;
    std::swap(data[a], data[b]);
    std::swap(data[a + 1], data[b + 1]);
  }
}

void RunCmsisCfftForwardFixed(
    const arm_cfft_instance_f32* cfft,
    float* data,
    uint8_t bit_reverse_flag) {
#if BIRDNET_FRONTEND_CMSIS_CFFT_REPAIR_LAST_STAGE
  birdnet_arm_cfft_f32_mve_repaired(cfft, data, 0, bit_reverse_flag);
#else
  arm_cfft_f32(cfft, data, 0, bit_reverse_flag);
#endif
}
#endif



template <typename T>
inline T Clamp(T v, T lo, T hi) {
  return std::min(hi, std::max(lo, v));
}

inline __attribute__((always_inline)) float SparseDotProductScalar(
    const float* __restrict values,
    const float* __restrict weights,
    uint16_t length) {
  switch (length) {
    case 0:
      return 0.0f;
    case 1:
      return values[0] * weights[0];
    case 2:
      return (values[0] * weights[0]) + (values[1] * weights[1]);
    case 3:
      return (values[0] * weights[0]) + (values[1] * weights[1]) + (values[2] * weights[2]);
    case 4:
      return (values[0] * weights[0]) + (values[1] * weights[1]) + (values[2] * weights[2]) +
             (values[3] * weights[3]);
    case 5:
      return (values[0] * weights[0]) + (values[1] * weights[1]) + (values[2] * weights[2]) +
             (values[3] * weights[3]) + (values[4] * weights[4]);
    case 6:
      return (values[0] * weights[0]) + (values[1] * weights[1]) + (values[2] * weights[2]) +
             (values[3] * weights[3]) + (values[4] * weights[4]) + (values[5] * weights[5]);
    case 7:
      return (values[0] * weights[0]) + (values[1] * weights[1]) + (values[2] * weights[2]) +
             (values[3] * weights[3]) + (values[4] * weights[4]) + (values[5] * weights[5]) +
             (values[6] * weights[6]);
    case 8:
      return (values[0] * weights[0]) + (values[1] * weights[1]) + (values[2] * weights[2]) +
             (values[3] * weights[3]) + (values[4] * weights[4]) + (values[5] * weights[5]) +
             (values[6] * weights[6]) + (values[7] * weights[7]);
    default:
      break;
  }

  float s0 = 0.0f;
  float s1 = 0.0f;
  float s2 = 0.0f;
  float s3 = 0.0f;
  uint16_t i = 0;
  for (; static_cast<uint16_t>(i + 3) < length; i += 4) {
    s0 += values[i] * weights[i];
    s1 += values[i + 1] * weights[i + 1];
    s2 += values[i + 2] * weights[i + 2];
    s3 += values[i + 3] * weights[i + 3];
  }
  float sum = (s0 + s1) + (s2 + s3);
  for (; i < length; ++i) {
    sum += values[i] * weights[i];
  }
  return sum;
}

#if BIRDNET_FRONTEND_MEL_USE_MVE_DOT && \
    (defined(ARM_MATH_MVEF) || defined(ARM_MATH_HELIUM)) && \
    !defined(ARM_MATH_AUTOVECTORIZE)
inline __attribute__((always_inline)) float SparseDotProductMve(
    const float* __restrict values,
    const float* __restrict weights,
    uint16_t length) {
  f32x4_t acc = vdupq_n_f32(0.0f);
  uint16_t i = 0;
  for (; static_cast<uint16_t>(i + 3) < length; i += 4) {
    acc = vfmaq_f32(acc, vldrwq_f32(values + i), vldrwq_f32(weights + i));
  }
  float sum = vgetq_lane(acc, 0) + vgetq_lane(acc, 1) +
              vgetq_lane(acc, 2) + vgetq_lane(acc, 3);
  for (; i < length; ++i) {
    sum += values[i] * weights[i];
  }
  return sum;
}
#endif

inline __attribute__((always_inline)) float SparseDotProduct(
    const float* __restrict values,
    const float* __restrict weights,
    uint16_t length) {
#if BIRDNET_FRONTEND_MEL_USE_MVE_DOT && \
    (defined(ARM_MATH_MVEF) || defined(ARM_MATH_HELIUM)) && \
    !defined(ARM_MATH_AUTOVECTORIZE)
  if (length >= BIRDNET_FRONTEND_MEL_MVE_DOT_MIN_LEN) {
    return SparseDotProductMve(values, weights, length);
  }
#endif
  return SparseDotProductScalar(values, weights, length);
}

inline __attribute__((always_inline)) float SparseDotProductPackedRfftRealFast(
    const float* __restrict packed_out,
    uint16_t start_bin,
    const float* __restrict weights,
    uint16_t length) {
  const float* __restrict values = packed_out + (2u * start_bin);
  switch (length) {
    case 0:
      return 0.0f;
    case 1:
      return values[0] * weights[0];
    case 2:
      return (values[0] * weights[0]) + (values[2] * weights[1]);
    case 3:
      return (values[0] * weights[0]) + (values[2] * weights[1]) + (values[4] * weights[2]);
    case 4:
      return (values[0] * weights[0]) + (values[2] * weights[1]) + (values[4] * weights[2]) +
             (values[6] * weights[3]);
    case 5:
      return (values[0] * weights[0]) + (values[2] * weights[1]) + (values[4] * weights[2]) +
             (values[6] * weights[3]) + (values[8] * weights[4]);
    case 6:
      return (values[0] * weights[0]) + (values[2] * weights[1]) + (values[4] * weights[2]) +
             (values[6] * weights[3]) + (values[8] * weights[4]) + (values[10] * weights[5]);
    case 7:
      return (values[0] * weights[0]) + (values[2] * weights[1]) + (values[4] * weights[2]) +
             (values[6] * weights[3]) + (values[8] * weights[4]) + (values[10] * weights[5]) +
             (values[12] * weights[6]);
    case 8:
      return (values[0] * weights[0]) + (values[2] * weights[1]) + (values[4] * weights[2]) +
             (values[6] * weights[3]) + (values[8] * weights[4]) + (values[10] * weights[5]) +
             (values[12] * weights[6]) + (values[14] * weights[7]);
    default:
      break;
  }

  float s0 = 0.0f;
  float s1 = 0.0f;
  float s2 = 0.0f;
  float s3 = 0.0f;
  uint16_t i = 0;
  for (; static_cast<uint16_t>(i + 3) < length; i += 4) {
    const uint16_t base = static_cast<uint16_t>(2u * i);
    s0 += values[base] * weights[i];
    s1 += values[base + 2] * weights[i + 1];
    s2 += values[base + 4] * weights[i + 2];
    s3 += values[base + 6] * weights[i + 3];
  }
  float sum = (s0 + s1) + (s2 + s3);
  for (; i < length; ++i) {
    sum += values[2u * i] * weights[i];
  }
  return sum;
}

inline __attribute__((always_inline)) float PackedRfftRealValue(
    const float* __restrict packed_out,
    int last_bin,
    int bin) {
  if (bin == 0) {
    return packed_out[0];
  }
  if (bin == last_bin) {
    return packed_out[1];
  }
  return packed_out[2 * bin];
}

inline __attribute__((always_inline)) float SparseDotProductPackedRfftReal(
    const float* __restrict packed_out,
    int fft_size,
    uint16_t start_bin,
    const float* __restrict weights,
    uint16_t length,
    float real_scale = 1.0f, bool dense = false) {
  if (dense) {
    if (!length) return 0.0f;
    return SparseDotProductScalar(packed_out + start_bin, weights, length) * real_scale;
  }
  if (length == 0) {
    return 0.0f;
  }
  const int last_bin = fft_size / 2;
  const int end_bin = static_cast<int>(start_bin) + static_cast<int>(length);
  if (start_bin > 0 && end_bin <= last_bin) {
    return SparseDotProductPackedRfftRealFast(packed_out, start_bin, weights, length) * real_scale;
  }

  float sum = 0.0f;
  for (uint16_t i = 0; i < length; ++i) {
    sum += PackedRfftRealValue(packed_out, last_bin, static_cast<int>(start_bin) + i) * weights[i];
  }
  return sum * real_scale;
}

inline __attribute__((always_inline)) bool DecomposePositiveNormalFloat(
    float value,
    int* exp2,
    float* mantissa_lut_pos) {
  uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  const uint32_t raw_exp = (bits >> 23) & 0xffu;
  if (raw_exp == 0 || raw_exp == 0xffu) {
    return false;
  }

  *exp2 = static_cast<int>(raw_exp) - 126;
  const uint32_t fraction = bits & 0x007fffffu;
  *mantissa_lut_pos =
      static_cast<float>(fraction) *
      (static_cast<float>(kPowMantissaLutSize) / 8388608.0f);
  return true;
}

float CompressMelEnergy(float mel_sum, int channel, float power_exponent) {
  const float energy = mel_sum * mel_sum;
  if (energy <= 0.0f) {
    return 0.0f;
  }
  if constexpr (!cfg::kUseCompressionLut) {
    return std::pow(energy, power_exponent);
  }

  int exp2 = 0;
  float pos = 0.0f;
  if (!DecomposePositiveNormalFloat(energy, &exp2, &pos)) {
    const float mantissa = std::frexp(energy, &exp2);
    pos = (mantissa - 0.5f) * (2.0f * static_cast<float>(kPowMantissaLutSize));
  }
  const int idx = Clamp(static_cast<int>(pos), 0, kPowMantissaLutSize - 1);
  const float frac = pos - static_cast<float>(idx);
  const float lut0 = (channel == 0 ? birdnet_tables::kMantissa0 : birdnet_tables::kMantissa1)[idx];
  const float lut1 = (channel == 0 ? birdnet_tables::kMantissa0 : birdnet_tables::kMantissa1)[idx + 1];
  const float mantissa_pow = lut0 + frac * (lut1 - lut0);

  float exp2_pow = 0.0f;
  if (exp2 >= kPowExp2Min && exp2 <= kPowExp2Max) {
    exp2_pow = (channel == 0 ? birdnet_tables::kExp0 : birdnet_tables::kExp1)[exp2 - kPowExp2Min];
  } else {
    exp2_pow = std::exp2(power_exponent * static_cast<float>(exp2));
  }
  return mantissa_pow * exp2_pow;
}

float Sigmoid(float x) {
  if (x >= 0.0f) {
    const float z = std::exp(-x);
    return 1.0f / (1.0f + z);
  }
  const float z = std::exp(x);
  return z / (1.0f + z);
}

void PrepareWindowAffine(
    int fft_size,
    const float* __restrict window,
    int16_t min_sample,
    float inv_range,
    float* __restrict window_scale,
    float* __restrict window_bias) {
  const float offset = -static_cast<float>(min_sample) * inv_range - 1.0f;
  for (int i = 0; i < fft_size; ++i) {
    const float w = window[i];
    window_scale[i] = w * inv_range;
    window_bias[i] = w * offset;
  }
}

#if BIRDNET_FRONTEND_FRAME_FILL_USE_MVE && \
    (defined(ARM_MATH_MVEF) || defined(ARM_MATH_HELIUM)) && \
    !defined(ARM_MATH_AUTOVECTORIZE)
inline __attribute__((always_inline)) void FillFrameContiguousMve(
    const int16_t* __restrict src,
    int count,
    const float* __restrict window_scale,
    const float* __restrict window_bias,
    float* __restrict frame) {
  int i = 0;
  for (; i + 7 < count; i += 8) {
    const f32x4_t samples_lo = vcvtq_f32_s32(vldrhq_s32(src + i));
    const f32x4_t samples_hi = vcvtq_f32_s32(vldrhq_s32(src + i + 4));
    const f32x4_t out_lo =
        vfmaq_f32(vldrwq_f32(window_bias + i), samples_lo, vldrwq_f32(window_scale + i));
    const f32x4_t out_hi =
        vfmaq_f32(vldrwq_f32(window_bias + i + 4), samples_hi, vldrwq_f32(window_scale + i + 4));
    vstrwq_f32(frame + i, out_lo);
    vstrwq_f32(frame + i + 4, out_hi);
  }
  for (; i < count; ++i) {
    frame[i] = static_cast<float>(src[i]) * window_scale[i] + window_bias[i];
  }
}
#endif

inline __attribute__((always_inline)) void FillFrameContiguous(
    const int16_t* __restrict src,
    int count,
    const float* __restrict window_scale,
    const float* __restrict window_bias,
    float* __restrict frame) {
#if BIRDNET_FRONTEND_FRAME_FILL_USE_MVE && \
    (defined(ARM_MATH_MVEF) || defined(ARM_MATH_HELIUM)) && \
    !defined(ARM_MATH_AUTOVECTORIZE)
  FillFrameContiguousMve(src, count, window_scale, window_bias, frame);
#else
  for (int i = 0; i < count; ++i) {
    frame[i] = static_cast<float>(src[i]) * window_scale[i] + window_bias[i];
  }
#endif
}

inline __attribute__((always_inline)) void FillFrameFromPcm(
    const int16_t* __restrict pcm,
    int start,
    int fft_size,
    const float* __restrict window_scale,
    const float* __restrict window_bias,
    float* __restrict frame) {
  FillFrameContiguous(pcm + start, fft_size, window_scale, window_bias, frame);
}

inline __attribute__((always_inline)) void UpdateMinMaxSpan(
    const int16_t* __restrict samples,
    int count,
    int16_t* min_sample,
    int16_t* max_sample) {
  int16_t mn = *min_sample;
  int16_t mx = *max_sample;
#if BIRDNET_FRONTEND_MINMAX_USE_MVE && \
    (defined(ARM_MATH_MVEI) || defined(ARM_MATH_HELIUM) || defined(ARM_MATH_MVEF)) && \
    !defined(ARM_MATH_AUTOVECTORIZE)
  int i = 0;
  int16x8_t vec_min = vdupq_n_s16(mn);
  int16x8_t vec_max = vdupq_n_s16(mx);
  for (; i + 7 < count; i += 8) {
    const int16x8_t values = vldrhq_s16(samples + i);
    vec_min = vminq_s16(vec_min, values);
    vec_max = vmaxq_s16(vec_max, values);
  }
  mn = vminvq_s16(mn, vec_min);
  mx = vmaxvq_s16(mx, vec_max);
  for (; i < count; ++i) {
    const int16_t sample = samples[i];
    mn = std::min(mn, sample);
    mx = std::max(mx, sample);
  }
#else
  for (int i = 0; i < count; ++i) {
    const int16_t sample = samples[i];
    mn = std::min(mn, sample);
    mx = std::max(mx, sample);
  }
#endif
  *min_sample = mn;
  *max_sample = mx;
}

void FindMinMaxPcm(
    const int16_t* __restrict pcm,
    int count,
    int16_t* min_sample,
    int16_t* max_sample) {
  int16_t mn = pcm[0];
  int16_t mx = pcm[0];
  UpdateMinMaxSpan(pcm + 1, count - 1, &mn, &mx);
  *min_sample = mn;
  *max_sample = mx;
}

void FindMinMaxRing(
    const int16_t* __restrict pcm_ring,
    size_t ring_size,
    size_t ring_idx,
    int count,
    int16_t* min_sample,
    int16_t* max_sample) {
  size_t idx = ring_idx % ring_size;
  int16_t mn = pcm_ring[idx];
  int16_t mx = mn;
  const int first_count = std::min(count, static_cast<int>(ring_size - idx));
  UpdateMinMaxSpan(pcm_ring + idx, first_count, &mn, &mx);
  const int remaining = count - first_count;
  if (remaining > 0) {
    UpdateMinMaxSpan(pcm_ring, remaining, &mn, &mx);
  }
  *min_sample = mn;
  *max_sample = mx;
}

inline __attribute__((always_inline)) void FillFrameFromRing(
    const int16_t* __restrict pcm_ring,
    size_t ring_size,
    size_t ring_idx,
    int fft_size,
    const float* __restrict window_scale,
    const float* __restrict window_bias,
    float* __restrict frame) {
  const size_t idx = ring_idx % ring_size;
  if (idx + static_cast<size_t>(fft_size) <= ring_size) {
    FillFrameFromPcm(
        pcm_ring,
        static_cast<int>(idx),
        fft_size,
        window_scale,
        window_bias,
        frame);
    return;
  }

  const int first_count = static_cast<int>(ring_size - idx);
  FillFrameContiguous(pcm_ring + idx, first_count, window_scale, window_bias, frame);
  FillFrameContiguous(
      pcm_ring,
      fft_size - first_count,
      window_scale + first_count,
      window_bias + first_count,
      frame + first_count);
}

inline __attribute__((always_inline)) void FillFrameDecimate2Contiguous(
    const int16_t* __restrict src,
    int fft_size,
    const float* __restrict window_scale,
    const float* __restrict window_bias,
    float* __restrict frame) {
#if BIRDNET_FRONTEND_FRAME_FILL_USE_MVE && \
    (defined(ARM_MATH_MVEF) || defined(ARM_MATH_HELIUM)) && \
    !defined(ARM_MATH_AUTOVECTORIZE)
  const uint32x4_t offsets = {0, 2, 4, 6};
  int i = 0;
  for (; i + 7 < fft_size; i += 8) {
    const f32x4_t samples_lo = vcvtq_f32_s32(
        vldrhq_gather_shifted_offset_s32(src + (i * kBranch1Decimate2Factor), offsets));
    const f32x4_t samples_hi = vcvtq_f32_s32(vldrhq_gather_shifted_offset_s32(
        src + ((i + 4) * kBranch1Decimate2Factor), offsets));
    vstrwq_f32(
        frame + i,
        vfmaq_f32(
            vldrwq_f32(window_bias + i), samples_lo, vldrwq_f32(window_scale + i)));
    vstrwq_f32(
        frame + i + 4,
        vfmaq_f32(
            vldrwq_f32(window_bias + i + 4),
            samples_hi,
            vldrwq_f32(window_scale + i + 4)));
  }
  for (; i < fft_size; ++i) {
    frame[i] = static_cast<float>(src[i * kBranch1Decimate2Factor]) * window_scale[i] + window_bias[i];
  }
#else
  for (int i = 0; i < fft_size; ++i) {
    frame[i] = static_cast<float>(src[i * kBranch1Decimate2Factor]) * window_scale[i] +
               window_bias[i];
  }
#endif
}

inline __attribute__((always_inline)) void FillFrameDecimate2FromPcm(
    const int16_t* __restrict pcm,
    int start,
    int fft_size,
    const float* __restrict window_scale,
    const float* __restrict window_bias,
    float* __restrict frame) {
  FillFrameDecimate2Contiguous(
      pcm + start, fft_size, window_scale, window_bias, frame);
}

inline __attribute__((always_inline)) void FillFrameDecimate2FromRing(
    const int16_t* __restrict pcm_ring,
    size_t ring_size,
    size_t ring_idx,
    int fft_size,
    const float* __restrict window_scale,
    const float* __restrict window_bias,
    float* __restrict frame) {
  const size_t idx = ring_idx % ring_size;
  const size_t final_idx = idx + static_cast<size_t>((fft_size - 1) * kBranch1Decimate2Factor);
  if (final_idx < ring_size) {
    FillFrameDecimate2Contiguous(
        pcm_ring + idx, fft_size, window_scale, window_bias, frame);
    return;
  }

  for (int i = 0; i < fft_size; ++i) {
    const size_t sample_idx =
        (idx + static_cast<size_t>(i * kBranch1Decimate2Factor)) % ring_size;
    frame[i] = static_cast<float>(pcm_ring[sample_idx]) * window_scale[i] + window_bias[i];
  }
}

