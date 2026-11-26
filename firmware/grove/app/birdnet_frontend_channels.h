// Private inference implementation; included at its original namespace scope.
#pragma once
#if BIRDNET_BRANCH1_USE_KISS || BIRDNET_BRANCH2_USE_KISS
bool FillBranchChannelWithKiss(
    const int16_t* pcm_window,
    int16_t min_sample,
    float inv_range,
    int channel,
    int fft_size,
    int hop,
    const float* window,
    void* fft_cfg,
    float* frame,
    birdnet_kiss_fft_cpx* fft_out,
    float* real_bins,
    const uint16_t* mel_starts,
    const uint16_t* mel_lengths,
    const uint16_t* mel_offsets,
    const float* packed_weights,
    float power_exponent,
    float bn_mul,
    float bn_add) {
  float batchnorm_min = 0.0f;
  float batchnorm_max = 0.0f;
  bool have_batchnorm = false;
  int nonfinite_real = 0;
  int nonfinite_mel = 0;
  int nonfinite_batch = 0;
  static bool s_logged_bad_bins[2] = {false, false};
  const int bins = (fft_size / 2) + 1;

  PrepareWindowAffine(fft_size, window, min_sample, inv_range, g_rt.window_scale, g_rt.window_bias);
  for (int frame_idx = 0; frame_idx < cfg::kSpecFrames; ++frame_idx) {
    const int start = frame_idx * hop;
    FillFrameFromPcm(pcm_window, start, fft_size, g_rt.window_scale, g_rt.window_bias, frame);

    birdnet_kiss_fftr_run(fft_cfg, frame, fft_out);
    if constexpr (cfg::kEnableDiagnosticLogs) {
      int logged_bins = 0;
      for (int k = 0; k < bins; ++k) {
        real_bins[k] = fft_out[k].r;
        if (!std::isfinite(real_bins[k])) {
          ++nonfinite_real;
          if (!s_logged_bad_bins[channel] && frame_idx == 0 && logged_bins < 16) {
            birdnet_log::Write("[birdnet] branch%d bad_bin[%d]=%d\r\n", channel, logged_bins, k);
            ++logged_bins;
          }
          real_bins[k] = 0.0f;
        }
      }
      if (!s_logged_bad_bins[channel] && frame_idx == 0 && logged_bins > 0) {
        s_logged_bad_bins[channel] = true;
      }
    } else {
      for (int k = 0; k < bins; ++k) {
        real_bins[k] = fft_out[k].r;
      }
    }

    int flat_index = (((cfg::kSpecMels - 1) * cfg::kSpecFrames) + frame_idx) * cfg::kSpecChannels + channel;
    for (int mel_idx = 0; mel_idx < cfg::kSpecMels; ++mel_idx) {
      const uint16_t length = mel_lengths[mel_idx];
      const float mel_sum =
          (length > 0) ? SparseDotProduct(real_bins + mel_starts[mel_idx], packed_weights + mel_offsets[mel_idx], length)
                       : 0.0f;
      if constexpr (cfg::kEnableDiagnosticLogs) {
        nonfinite_mel += !std::isfinite(mel_sum);
      }
      const float compressed = CompressMelEnergy(mel_sum, channel, power_exponent);
      if constexpr (cfg::kEnableDiagnosticLogs) {
        const float batchnorm = compressed * bn_mul + bn_add;
        nonfinite_batch += !std::isfinite(batchnorm);
        if (!have_batchnorm) {
          batchnorm_min = batchnorm;
          batchnorm_max = batchnorm;
          have_batchnorm = true;
        } else {
          batchnorm_min = std::min(batchnorm_min, batchnorm);
          batchnorm_max = std::max(batchnorm_max, batchnorm);
        }
      }
      if (!StoreCompressedFeatureAt(flat_index, compressed, channel, bn_mul, bn_add)) {
        return false;
      }
      flat_index -= (cfg::kSpecFrames * cfg::kSpecChannels);
    }
  }

  if (cfg::kEnableDiagnosticLogs && have_batchnorm) {
    birdnet_log::Write("[birdnet] branch%d f=[%d,%d] q1e3 nonfinite_real=%d nonfinite_mel=%d nonfinite_bn=%d\r\n",
            channel,
            static_cast<int>(batchnorm_min * 1000.0f),
            static_cast<int>(batchnorm_max * 1000.0f),
            nonfinite_real,
            nonfinite_mel,
            nonfinite_batch);
  }
  return true;
}
#endif

#if BIRDNET_ANY_USE_CMSIS_RFFT_SCALAR
void RunScalarStageRfft(
    const arm_rfft_fast_instance_f32* rfft,
    float* cfft_out,
    float* packed_out) {
  int32_t k = rfft->Sint.fftLen - 1;
  const float32_t* coeff = rfft->pTwiddleRFFT;
  float32_t* pA = cfft_out;
  float32_t* pB = cfft_out;

  float xBR = pB[0];
  float xBI = pB[1];
  float xAR = pA[0];
  float xAI = pA[1];

  float twR = *coeff++;
  float twI = *coeff++;

  float t1a = xBR + xAR;
  float t1b = xBI + xAI;

  *packed_out++ = 0.5f * (t1a + t1b);
  *packed_out++ = 0.5f * (t1a - t1b);

  pB = cfft_out + 2 * k;
  pA += 2;

  while (k > 0) {
    xBI = pB[1];
    xBR = pB[0];
    xAR = pA[0];
    xAI = pA[1];

    twR = *coeff++;
    twI = *coeff++;

    t1a = xBR - xAR;
    t1b = xBI + xAI;

    const float p0 = twR * t1a;
    const float p1 = twI * t1a;
    const float p2 = twR * t1b;
    const float p3 = twI * t1b;

    *packed_out++ = 0.5f * (xAR + xBR + p0 + p3);
    *packed_out++ = 0.5f * (xAI - xBI + p1 - p2);

    pA += 2;
    pB -= 2;
    --k;
  }
}

#if BIRDNET_FRONTEND_RFFT_UNPACK_USE_MVE && \
    (defined(ARM_MATH_MVEF) || defined(ARM_MATH_HELIUM)) && \
    !defined(ARM_MATH_AUTOVECTORIZE)
alignas(16) constexpr uint32_t kPackedRfftRealOffsets4[4] = {2, 4, 6, 8};

inline __attribute__((always_inline)) void StorePackedRfftRealBinsMve(
    const float* __restrict packed_out,
    int fft_size,
    float* __restrict real_bins) {
  const int last_bin = fft_size / 2;
  real_bins[0] = packed_out[0];
  uint32x4_t offsets = vldrwq_u32(kPackedRfftRealOffsets4);
  const uint32x4_t step = vdupq_n_u32(8);
  int k = 1;
  for (; k + 3 < last_bin; k += 4) {
    const f32x4_t values = vldrwq_gather_shifted_offset_f32(packed_out, offsets);
    vstrwq_f32(real_bins + k, values);
    offsets = vaddq_u32(offsets, step);
  }
  for (; k < last_bin; ++k) {
    real_bins[k] = packed_out[2 * k];
  }
  real_bins[last_bin] = packed_out[1];
}
#endif

inline __attribute__((always_inline)) void StorePackedRfftRealBins(
    const float* __restrict packed_out,
    int fft_size,
    float* __restrict real_bins,
    int* nonfinite_real,
    float real_scale = 1.0f) {
  const int last_bin = fft_size / 2;
  if constexpr (cfg::kEnableDiagnosticLogs) {
    const auto store_bin = [&](int index, float value) {
      const float scaled = value * real_scale;
      if (std::isfinite(scaled)) {
        real_bins[index] = scaled;
      } else {
        real_bins[index] = 0.0f;
        ++(*nonfinite_real);
      }
    };

    store_bin(0, packed_out[0]);
    for (int k = 1; k < last_bin; ++k) {
      store_bin(k, packed_out[2 * k]);
    }
    store_bin(last_bin, packed_out[1]);
    return;
  }

  (void)nonfinite_real;
#if BIRDNET_FRONTEND_RFFT_UNPACK_USE_MVE && \
    (defined(ARM_MATH_MVEF) || defined(ARM_MATH_HELIUM)) && \
    !defined(ARM_MATH_AUTOVECTORIZE)
  if (real_scale == 1.0f) {
    StorePackedRfftRealBinsMve(packed_out, fft_size, real_bins);
    return;
  }
#endif
  real_bins[0] = packed_out[0] * real_scale;
  for (int k = 1; k < last_bin; ++k) {
    real_bins[k] = packed_out[2 * k] * real_scale;
  }
  real_bins[last_bin] = packed_out[1] * real_scale;
}

void RunCmsisScalarRfftRealBins(
    const arm_rfft_fast_instance_f32* rfft,
    int fft_size,
    float* frame,
    float* packed_out,
    float* real_bins,
    int* nonfinite_real) {
  RunCmsisCfftForwardFixed(&(rfft->Sint), frame, 1);
  RunScalarStageRfft(rfft, frame, packed_out);

  StorePackedRfftRealBins(packed_out, fft_size, real_bins, nonfinite_real);
}

void RunCmsisScalarRfftPacked(
    const arm_rfft_fast_instance_f32* rfft,
    float* frame,
    float* packed_out) {
  RunCmsisCfftForwardFixed(&(rfft->Sint), frame, 1);
  RunScalarStageRfft(rfft, frame, packed_out);
}

void RunCmsisDirectRfftRealBins(
    const arm_rfft_fast_instance_f32* rfft,
    int fft_size,
    float* frame,
    float* packed_out,
    float* real_bins,
    int* nonfinite_real) {
  arm_rfft_fast_f32(rfft, frame, packed_out, 0);

  StorePackedRfftRealBins(packed_out, fft_size, real_bins, nonfinite_real);
}

void RunCmsisDirectRfftPacked(
    const arm_rfft_fast_instance_f32* rfft,
    float* frame,
    float* packed_out) {
  arm_rfft_fast_f32(rfft, frame, packed_out, 0);
}

void RunCmsisMveStageRfftRealBins(
    const arm_rfft_fast_instance_f32* rfft,
    int fft_size,
    float* frame,
    float* packed_out,
    float* real_bins,
    int* nonfinite_real) {
  RunCmsisCfftForwardFixed(&(rfft->Sint), frame, 1);
  stage_rfft_f32(rfft, frame, packed_out);

  StorePackedRfftRealBins(packed_out, fft_size, real_bins, nonfinite_real);
}

#if BIRDNET_FRONTEND_PARTIAL_REAL && (BIRDNET_SAMPLE_RATE_HZ != 48000 || BIRDNET_FRONTEND_STREAMING || !BIRDNET_FRONTEND_RFFT_DIRECT_MEL || !BIRDNET_FRONTEND_CMSIS_RFFT_USE_MVE_STAGE)
#error "Partial real split requires the validated 48 kHz nonstreaming MVE direct-mel profile"
#endif
#if BIRDNET_FRONTEND_PARTIAL_REAL
struct MelSplitRange { unsigned first, end; };

MelSplitRange MelRange(const uint16_t* starts, const uint16_t* lengths) {
  MelSplitRange r{0xffff, 0};
  for (int m = 0; m < cfg::kSpecMels; ++m) {
    if (!lengths[m]) continue;
    r.first = std::min(r.first, unsigned(starts[m]));
    r.end = std::max(r.end, unsigned(starts[m]) + lengths[m]);
  }
  return r;
}

#if BIRDNET_USE_FOLDED_REORDER
struct ReversedOffsets512 {
  uint16_t words[512]{};
  constexpr ReversedOffsets512() {
    for (unsigned i = 0; i < 512; ++i) {
      unsigned v = 0;
      for (unsigned b = 0; b < 9; ++b) v = 2*v + ((i >> b) & 1);
      words[i] = 2*v;
    }
  }
};
alignas(16) constexpr ReversedOffsets512 kRev512{};
#endif


template<bool Verify, bool Dense = false, bool Fold = false>
void PartialRealSplit(const arm_rfft_fast_instance_f32* rfft, const float* frame,
                      float* packed, MelSplitRange range) {
  const uint32x4_t reverse = {6, 4, 2, 0};
  const uint32x4_t forward = {0, 2, 4, 6};
  for (unsigned k = range.first; k < range.end; k += 4) {
    float32x4x2_t a;
    float32x4_t br, bi;
#if BIRDNET_USE_FOLDED_REORDER
    if constexpr (Fold) {
      const auto fwd = vldrhq_u32(kRev512.words + k);
      // rev9(512 - j) == 511 XOR rev9(j - 1), for interior bins j.
      const auto rev = veorq_u32(vldrhq_u32(kRev512.words + k - 1), vdupq_n_u32(1022));
      a.val[0] = vldrwq_gather_shifted_offset_f32(frame, fwd);
      a.val[1] = vldrwq_gather_shifted_offset_f32(frame + 1, fwd);
      br = vldrwq_gather_shifted_offset_f32(frame, rev);
      bi = vnegq_f32(vldrwq_gather_shifted_offset_f32(frame + 1, rev));
    } else
#endif
    {
      a = vld2q_f32(frame + 2*k);
      const float* b = frame + 2*(rfft->Sint.fftLen - k - 3);
      br = vldrwq_gather_shifted_offset_f32(b, reverse);
      bi = vnegq_f32(vldrwq_gather_shifted_offset_f32(b + 1, reverse));
    }
    const auto tw = vld2q_f32(rfft->pTwiddleRFFT + 2*k);
    // Preserve the stock MVE split's rounded multiply followed by fused subtract.
    auto real = vmulq_f32(tw.val[0], vsubq_f32(br, a.val[0]));
    real = vfmsq_f32(real, tw.val[1], vsubq_f32(bi, a.val[1]));
    real = vmulq_n_f32(vaddq_f32(real, vaddq_f32(a.val[0], br)), 0.5f);
    if constexpr (Dense) vstrwq_f32(packed + k, real);
    else vstrwq_scatter_shifted_offset_f32(packed + 2*k, forward, real);
  }
}
#endif

// True only when the output contains contiguous interior real bins.
bool RunCmsisMveStageRfftPacked(
    const arm_rfft_fast_instance_f32* rfft,
    float* frame,
    float* packed_out) {
#if !BIRDNET_USE_FOLDED_REORDER
  RunCmsisCfftForwardFixed(&(rfft->Sint), frame, 1);
#endif
#if BIRDNET_FRONTEND_PARTIAL_REAL
  static const auto r1 = MelRange(birdnet_frontend::kBranch1MelStarts, birdnet_frontend::kBranch1MelLengths);
  static const auto r2 = MelRange(birdnet_frontend::kBranch2MelStarts, birdnet_frontend::kBranch2MelLengths);
  const auto r = rfft == &g_rt.rfft_branch1 ? r1 : r2;
  constexpr int mode = 1;
  // Only the interior vector path is specialized. Leave DC, Nyquist and
  // the CMSIS scalar tail to the stock implementation if filters reach them.
#if BIRDNET_USE_FOLDED_REORDER
  constexpr bool fold = true;
  if (mode && fold && rfft->Sint.fftLen == 512 &&
      rfft->Sint.pBitRevTable == armBitRevIndexTable_fixed_512 &&
      r.first > 0 && r.first < r.end && r.end + 6 <= 512) {
    RunCmsisCfftForwardFixed(&rfft->Sint, frame, 0);
    PartialRealSplit<false, false, true>(rfft, frame, packed_out, r);
    return false; // Bin addressing at the mel consumer is still interleaved.
  }
  RunCmsisCfftForwardFixed(&rfft->Sint, frame, 1);
#endif
  if (mode && r.first > 0 && r.first < r.end && r.end + 6 <= rfft->Sint.fftLen) {
    if (mode == 2) {
      stage_rfft_f32(rfft, frame, packed_out);
      PartialRealSplit<true>(rfft, frame, packed_out, r);
    } else {
      PartialRealSplit<false>(rfft, frame, packed_out, r);
    }
    return false;
  }
#endif
  stage_rfft_f32(rfft, frame, packed_out);
  return false;
}

void RunCmsisOverrideRfftRealBins(
    const arm_rfft_fast_instance_f32* rfft,
    int fft_size,
    float* frame,
    float* packed_out,
    float* real_bins,
    int* nonfinite_real,
    float real_scale = 1.0f) {
  birdnet_arm_rfft_fast_f32(rfft, frame, packed_out, 0);

  StorePackedRfftRealBins(packed_out, fft_size, real_bins, nonfinite_real, real_scale);
}

void RunCmsisOverrideRfftPacked(
    const arm_rfft_fast_instance_f32* rfft,
    float* frame,
    float* packed_out) {
  birdnet_arm_rfft_fast_f32(rfft, frame, packed_out, 0);
}

#if BIRDNET_FRONTEND_STREAMING
void StreamingMel(int channel, float* frame, float* mel) {
  const auto* rfft = channel == 0 ? &g_rt.rfft_branch1 : &g_rt.rfft_branch2;
  RunCmsisMveStageRfftPacked(rfft, frame, g_rt.fft_packed);
  const auto* starts = channel == 0 ? birdnet_frontend::kBranch1MelStarts : birdnet_frontend::kBranch2MelStarts;
  const auto* lengths = channel == 0 ? birdnet_frontend::kBranch1MelLengths : birdnet_frontend::kBranch2MelLengths;
  const auto* offsets = channel == 0 ? birdnet_frontend::kBranch1MelOffsets : birdnet_frontend::kBranch2MelOffsets;
  const auto* weights = channel == 0 ? birdnet_frontend::kBranch1MelWeights : birdnet_frontend::kBranch2MelWeights;
  for (int m = 0; m < cfg::kSpecMels; ++m) {
    mel[m] = SparseDotProductPackedRfftReal(g_rt.fft_packed, 1024, starts[m],
        weights + offsets[m], lengths[m], channel == 0 ? 2.0f : 1.0f);
  }
}

void FillStreamingFrame(const int16_t* pcm, int stride, int count, int16_t anchor, float* frame) {
  int i = 0;
#if BIRDNET_FRONTEND_FRAME_FILL_USE_MVE && \
    (defined(ARM_MATH_MVEF) || defined(ARM_MATH_HELIUM)) && !defined(ARM_MATH_AUTOVECTORIZE)
  const uint32x4_t offsets = {0, static_cast<uint32_t>(stride),
      static_cast<uint32_t>(2 * stride), static_cast<uint32_t>(3 * stride)};
  for (; i + 3 < count; i += 4) {
    const auto samples = vsubq_n_s32(vldrhq_gather_shifted_offset_s32(pcm + i * stride, offsets), anchor);
    vstrwq_f32(frame + i, vmulq_f32(vcvtq_f32_s32(samples), vldrwq_f32(g_rt.window_scale + i)));
  }
#endif
  for (; i < count; ++i) frame[i] = (static_cast<int>(pcm[i * stride]) - anchor) * g_rt.window_scale[i];
  for (; i < 1024; ++i) frame[i] = 0.0f;
}

bool FillStreamingFrontend(const int16_t* pcm, uint64_t end) {
  using namespace birdnet_streaming;
  if (end < cfg::kWindowSamples) return false;
  const bool retained = Begin(g_frontend_cache, g_frontend_state, end, cfg::kInferenceStepSamples);
  int16_t low = 0, high = 0;
  FindMinMaxPcm(pcm, cfg::kWindowSamples, &low, &high);
  const float inverse = 2.0f / (static_cast<float>(static_cast<int>(high) - low) + 1e-6f);
  const uint64_t window_start = end - cfg::kWindowSamples;
  constexpr bool aligned_step = cfg::kInferenceStepSamples % birdnet_frontend::kBranch1Hop == 0 &&
      cfg::kInferenceStepSamples % birdnet_frontend::kBranch2Hop == 0;
  for (int i = 0; i < 1024; ++i) g_rt.window_scale[i] = birdnet_tables::kWindow2[i] / 32768.0f;
  unsigned reused = 0, computed = 0;
  for (int ch = 0; ch < 2; ++ch) {
    const int stride = ch == 0 ? 2 : 1;
    const int hop = ch == 0 ? birdnet_frontend::kBranch1Hop : birdnet_frontend::kBranch2Hop;
    // With an aligned step, preserve the original window-relative FFT grid.
    // The capture DMA may start the first window at a nonzero phase. Floor
    // IDs still advance by exactly step/hop, and Begin rejects any phase gap.
    const uint64_t first = (window_start + (aligned_step ? 0 : hop - 1)) / hop;
    float raw[kMels], dc[kMels];
    std::memcpy(g_rt.Frame1(), birdnet_tables::kWindow2, 1024 * sizeof(float));
    StreamingMel(ch, g_rt.Frame1(), dc);
    for (int f = 0; f < kFrames; ++f) {
      const uint64_t id = first + f;
      const int start = aligned_step ? f * hop : static_cast<int>(id * hop - window_start);
      const int count = std::min(1024, (cfg::kWindowSamples - start + stride - 1) / stride);
      const bool complete = count == 1024;
      int16_t anchor = 0;
      if (complete && g_frontend_cache.Load(ch, id, raw, &anchor)) {
        ++reused;
      } else {
        // Subtract a frame-local PCM anchor before FP16 storage. Reapplying
        // (anchor - window_min) later avoids cancellation on quiet/DC audio.
        anchor = pcm[start];
        if (!complete) {
          for (int i = 0; i < 1024; ++i) g_rt.Frame1()[i] = i < count ? birdnet_tables::kWindow2[i] : 0.0f;
          StreamingMel(ch, g_rt.Frame1(), dc);
        }
        FillStreamingFrame(pcm + start, stride, count, anchor, g_rt.Frame1());
        StreamingMel(ch, g_rt.Frame1(), raw);
        for (float& value : raw) value = Unpack(Pack(value));
        ++computed;
        // Early writes would evict later frames before they are consumed.
        // Partial final frames must be recomputed once future PCM arrives.
        if (complete && f >= kFrames - kCachedFrames) g_frontend_cache.Store(ch, id, raw, anchor);
      }
      const float bias = static_cast<float>(static_cast<int>(anchor) - low) * inverse - 1.0f;
      int flat = (((cfg::kSpecMels - 1) * cfg::kSpecFrames) + f) * cfg::kSpecChannels + ch;
      for (int m = 0; m < kMels; ++m) {
        const float sum = raw[m] * (32768.0f * inverse) + dc[m] * bias;
        if (!std::isfinite(sum)) return false;
        const float compressed = CompressMelEnergy(sum, ch, birdnet_frontend::kBranchPowerExponent[ch]);
        if (!StoreCompressedFeatureAt(flat, compressed, ch,
              birdnet_frontend::kBatchNormMul[ch], birdnet_frontend::kBatchNormAdd[ch])) return false;
        flat -= cfg::kSpecFrames * cfg::kSpecChannels;
      }
    }
  }
  Seal(g_frontend_cache, g_frontend_state, end);
  SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t*>(&g_frontend_cache), sizeof(g_frontend_cache));
  __DSB();
  birdnet_log::Status("[birdnet] streaming retained=%d reused=%u computed=%u end=%llu hash=%08lx\r\n",
      retained, reused, computed, static_cast<unsigned long long>(end),
      static_cast<unsigned long>(g_frontend_state.hash));
  return true;
}
#endif

bool FillBranchChannelWithCmsisScalarRfft(
    const int16_t* pcm_window,
    int16_t min_sample,
    float inv_range,
    int channel,
    int fft_size,
    int hop,
    const float* window,
    const arm_rfft_fast_instance_f32* rfft,
    bool use_c_override,
    float* frame,
    float* packed_out,
    float* real_bins,
    const uint16_t* mel_starts,
    const uint16_t* mel_lengths,
    const uint16_t* mel_offsets,
    const float* packed_weights,
    float power_exponent,
    float bn_mul,
    float bn_add) {
#if BIRDNET_FIELD_MEL_CODES
  const bool fast_q=birdnet_quant::enabled() && g_rt.input_int8_data && !cfg::kEnableDiagnosticLogs;
  if (fast_q) birdnet_quant::prepare(channel);
#endif
  float batchnorm_min = 0.0f;
  float batchnorm_max = 0.0f;
  bool have_batchnorm = false;
  int nonfinite_real = 0;
  int nonfinite_mel = 0;
  int nonfinite_batch = 0;

  PrepareWindowAffine(fft_size, window, min_sample, inv_range, g_rt.window_scale, g_rt.window_bias);
  for (int frame_idx = 0; frame_idx < cfg::kSpecFrames; ++frame_idx) {
    const int start = frame_idx * hop;
    FillFrameFromPcm(pcm_window, start, fft_size, g_rt.window_scale, g_rt.window_bias, frame);

    bool dense = false;
    if constexpr (kUseDirectRfftMel) {
      if (use_c_override) {
        RunCmsisOverrideRfftPacked(rfft, frame, packed_out);
      } else {
#if BIRDNET_FRONTEND_CMSIS_RFFT_USE_MVE_STAGE
        dense = RunCmsisMveStageRfftPacked(rfft, frame, packed_out);
#else
        RunCmsisScalarRfftPacked(rfft, frame, packed_out);
#endif
      }
    } else {
      if (use_c_override) {
        RunCmsisOverrideRfftRealBins(rfft, fft_size, frame, packed_out, real_bins, &nonfinite_real);
      } else {
#if BIRDNET_FRONTEND_CMSIS_RFFT_USE_MVE_STAGE
        RunCmsisMveStageRfftRealBins(rfft, fft_size, frame, packed_out, real_bins, &nonfinite_real);
#else
        RunCmsisScalarRfftRealBins(rfft, fft_size, frame, packed_out, real_bins, &nonfinite_real);
#endif
      }
    }

    int flat_index = (((cfg::kSpecMels - 1) * cfg::kSpecFrames) + frame_idx) * cfg::kSpecChannels + channel;
    for (int mel_idx = 0; mel_idx < cfg::kSpecMels; ++mel_idx) {
      const uint16_t length = mel_lengths[mel_idx];
      const float mel_sum = kUseDirectRfftMel
                                ? SparseDotProductPackedRfftReal(
                                      packed_out,
                                      fft_size,
                                      mel_starts[mel_idx],
                                      packed_weights + mel_offsets[mel_idx],
                                      length, 1.0f, dense)
                                : ((length > 0)
                                       ? SparseDotProduct(
                                             real_bins + mel_starts[mel_idx],
                                             packed_weights + mel_offsets[mel_idx],
                                             length)
                                       : 0.0f);
      if constexpr (cfg::kEnableDiagnosticLogs) {
        nonfinite_mel += !std::isfinite(mel_sum);
      }
#if BIRDNET_FIELD_MEL_CODES
      if (fast_q && birdnet_quant::store(flat_index,mel_sum,channel)) {
        flat_index -= cfg::kSpecFrames * cfg::kSpecChannels;
        continue;
      }
#endif
      const float compressed = CompressMelEnergy(mel_sum, channel, power_exponent);
      if constexpr (cfg::kEnableDiagnosticLogs) {
        const float batchnorm = compressed * bn_mul + bn_add;
        nonfinite_batch += !std::isfinite(batchnorm);
        if (!have_batchnorm) {
          batchnorm_min = batchnorm;
          batchnorm_max = batchnorm;
          have_batchnorm = true;
        } else {
          batchnorm_min = std::min(batchnorm_min, batchnorm);
          batchnorm_max = std::max(batchnorm_max, batchnorm);
        }
      }
      if (!StoreCompressedFeatureAt(flat_index, compressed, channel, bn_mul, bn_add)) {
        return false;
      }
      flat_index -= (cfg::kSpecFrames * cfg::kSpecChannels);
    }
  }


  if (cfg::kEnableDiagnosticLogs && have_batchnorm) {
    birdnet_log::Write("[birdnet] branch%d f=[%d,%d] q1e3 nonfinite_real=%d nonfinite_mel=%d nonfinite_bn=%d\r\n",
            channel,
            static_cast<int>(batchnorm_min * 1000.0f),
            static_cast<int>(batchnorm_max * 1000.0f),
            nonfinite_real,
            nonfinite_mel,
            nonfinite_batch);
  }
  return true;
}

#if defined(__GNUC__)
__attribute__((optimize("O2"), noinline))
#endif
bool FillBranchChannelFromRingWithCmsisScalarRfft(
    const int16_t* pcm_ring,
    size_t ring_size,
    size_t write_idx,
    int16_t min_sample,
    float inv_range,
    int channel,
    int fft_size,
    int hop,
    const float* window,
    const arm_rfft_fast_instance_f32* rfft,
    bool use_c_override,
    float* frame,
    float* packed_out,
    float* real_bins,
    const uint16_t* mel_starts,
    const uint16_t* mel_lengths,
    const uint16_t* mel_offsets,
    const float* packed_weights,
    float power_exponent,
    float bn_mul,
    float bn_add) {
  if (!pcm_ring || ring_size == 0) {
    return false;
  }

#if BIRDNET_FIELD_MEL_CODES
  const bool fast_q=birdnet_quant::enabled() && g_rt.input_int8_data && !cfg::kEnableDiagnosticLogs;
  if (fast_q) birdnet_quant::prepare(channel);
#endif
  float batchnorm_min = 0.0f;
  float batchnorm_max = 0.0f;
  bool have_batchnorm = false;
  int nonfinite_real = 0;
  int nonfinite_mel = 0;
  int nonfinite_batch = 0;

  PrepareWindowAffine(fft_size, window, min_sample, inv_range, g_rt.window_scale, g_rt.window_bias);
  for (int frame_idx = 0; frame_idx < cfg::kSpecFrames; ++frame_idx) {
    size_t ring_idx = (write_idx + static_cast<size_t>(frame_idx * hop)) % ring_size;
    FillFrameFromRing(pcm_ring, ring_size, ring_idx, fft_size, g_rt.window_scale, g_rt.window_bias, frame);

    bool dense = false;
    if constexpr (kUseDirectRfftMel) {
      if (use_c_override) {
        RunCmsisOverrideRfftPacked(rfft, frame, packed_out);
      } else {
#if BIRDNET_FRONTEND_CMSIS_RFFT_USE_MVE_STAGE
        dense = RunCmsisMveStageRfftPacked(rfft, frame, packed_out);
#else
        RunCmsisScalarRfftPacked(rfft, frame, packed_out);
#endif
      }
    } else {
      if (use_c_override) {
        RunCmsisOverrideRfftRealBins(rfft, fft_size, frame, packed_out, real_bins, &nonfinite_real);
      } else {
#if BIRDNET_FRONTEND_CMSIS_RFFT_USE_MVE_STAGE
        RunCmsisMveStageRfftRealBins(rfft, fft_size, frame, packed_out, real_bins, &nonfinite_real);
#else
        RunCmsisScalarRfftRealBins(rfft, fft_size, frame, packed_out, real_bins, &nonfinite_real);
#endif
      }
    }

    int flat_index = (((cfg::kSpecMels - 1) * cfg::kSpecFrames) + frame_idx) * cfg::kSpecChannels + channel;
    for (int mel_idx = 0; mel_idx < cfg::kSpecMels; ++mel_idx) {
      const uint16_t length = mel_lengths[mel_idx];
      const float mel_sum = kUseDirectRfftMel
                                ? SparseDotProductPackedRfftReal(
                                      packed_out,
                                      fft_size,
                                      mel_starts[mel_idx],
                                      packed_weights + mel_offsets[mel_idx],
                                      length, 1.0f, dense)
                                : ((length > 0)
                                       ? SparseDotProduct(
                                             real_bins + mel_starts[mel_idx],
                                             packed_weights + mel_offsets[mel_idx],
                                             length)
                                       : 0.0f);
      if constexpr (cfg::kEnableDiagnosticLogs) {
        nonfinite_mel += !std::isfinite(mel_sum);
      }
#if BIRDNET_FIELD_MEL_CODES
      if (fast_q && birdnet_quant::store(flat_index,mel_sum,channel)) {
        flat_index -= cfg::kSpecFrames * cfg::kSpecChannels;
        continue;
      }
#endif
      const float compressed = CompressMelEnergy(mel_sum, channel, power_exponent);
      if constexpr (cfg::kEnableDiagnosticLogs) {
        const float batchnorm = compressed * bn_mul + bn_add;
        nonfinite_batch += !std::isfinite(batchnorm);
        if (!have_batchnorm) {
          batchnorm_min = batchnorm;
          batchnorm_max = batchnorm;
          have_batchnorm = true;
        } else {
          batchnorm_min = std::min(batchnorm_min, batchnorm);
          batchnorm_max = std::max(batchnorm_max, batchnorm);
        }
      }
      if (!StoreCompressedFeatureAt(flat_index, compressed, channel, bn_mul, bn_add)) {
        return false;
      }
      flat_index -= (cfg::kSpecFrames * cfg::kSpecChannels);
    }
  }


  if (cfg::kEnableDiagnosticLogs && have_batchnorm) {
    birdnet_log::Write("[birdnet] branch%d f=[%d,%d] q1e3 nonfinite_real=%d nonfinite_mel=%d nonfinite_bn=%d\r\n",
            channel,
            static_cast<int>(batchnorm_min * 1000.0f),
            static_cast<int>(batchnorm_max * 1000.0f),
            nonfinite_real,
            nonfinite_mel,
            nonfinite_batch);
  }
  return true;
}

#if BIRDNET_BRANCH1_USE_DECIMATE2_RFFT
bool FillBranch1ChannelWithDecimate2Rfft(
    const int16_t* pcm_window,
    int16_t min_sample,
    float inv_range) {
#if BIRDNET_FIELD_MEL_CODES
  const bool fast_q=birdnet_quant::enabled() && g_rt.input_int8_data && !cfg::kEnableDiagnosticLogs;
  if (fast_q) birdnet_quant::prepare(0);
#endif
  float batchnorm_min = 0.0f;
  float batchnorm_max = 0.0f;
  bool have_batchnorm = false;
  int nonfinite_real = 0;
  int nonfinite_mel = 0;
  int nonfinite_batch = 0;

  // The 1024-point Hann is exactly the 2048-point branch-1 Hann sampled at
  // even indices, so reuse the branch-2 window table without adding SRAM.
  PrepareWindowAffine(
      kBranch1Decimate2FftSize,
      birdnet_tables::kWindow2,
      min_sample,
      inv_range,
      g_rt.window_scale,
      g_rt.window_bias);
  for (int frame_idx = 0; frame_idx < cfg::kSpecFrames; ++frame_idx) {
    const int start = frame_idx * birdnet_frontend::kBranch1Hop;
    FillFrameDecimate2FromPcm(
        pcm_window,
        start,
        kBranch1Decimate2FftSize,
        g_rt.window_scale,
        g_rt.window_bias,
        g_rt.Frame1());

    bool dense = false;
    if constexpr (kUseDirectRfftMel) {
#if BIRDNET_FRONTEND_CMSIS_RFFT_USE_MVE_STAGE
      dense = RunCmsisMveStageRfftPacked(&g_rt.rfft_branch1, g_rt.Frame1(), g_rt.fft_packed);
#else
      RunCmsisOverrideRfftPacked(&g_rt.rfft_branch1, g_rt.Frame1(), g_rt.fft_packed);
#endif
    } else {
#if BIRDNET_FRONTEND_CMSIS_RFFT_USE_MVE_STAGE
      RunCmsisMveStageRfftRealBins(
          &g_rt.rfft_branch1,
          kBranch1Decimate2FftSize,
          g_rt.Frame1(),
          g_rt.fft_packed,
          g_rt.real_bins_branch1,
          &nonfinite_real);
      for (int bin = 0; bin < kBranch1Decimate2Bins; ++bin) {
        g_rt.real_bins_branch1[bin] *= static_cast<float>(kBranch1Decimate2Factor);
      }
#else
      RunCmsisOverrideRfftRealBins(
          &g_rt.rfft_branch1,
          kBranch1Decimate2FftSize,
          g_rt.Frame1(),
          g_rt.fft_packed,
          g_rt.real_bins_branch1,
          &nonfinite_real,
          static_cast<float>(kBranch1Decimate2Factor));
#endif
    }

    int flat_index = (((cfg::kSpecMels - 1) * cfg::kSpecFrames) + frame_idx) * cfg::kSpecChannels;
    for (int mel_idx = 0; mel_idx < cfg::kSpecMels; ++mel_idx) {
      const uint16_t length = birdnet_frontend::kBranch1MelLengths[mel_idx];
      const float mel_sum =
          kUseDirectRfftMel
              ? SparseDotProductPackedRfftReal(
                    g_rt.fft_packed,
                    kBranch1Decimate2FftSize,
                    birdnet_frontend::kBranch1MelStarts[mel_idx],
                    birdnet_frontend::kBranch1MelWeights + birdnet_frontend::kBranch1MelOffsets[mel_idx],
                    length,
                    static_cast<float>(kBranch1Decimate2Factor), dense)
              : ((length > 0)
                     ? SparseDotProduct(
                           g_rt.real_bins_branch1 + birdnet_frontend::kBranch1MelStarts[mel_idx],
                           birdnet_frontend::kBranch1MelWeights + birdnet_frontend::kBranch1MelOffsets[mel_idx],
                           length)
                     : 0.0f);
      if constexpr (cfg::kEnableDiagnosticLogs) {
        nonfinite_mel += !std::isfinite(mel_sum);
      }
#if BIRDNET_FIELD_MEL_CODES
      if (fast_q && birdnet_quant::store(flat_index,mel_sum,0)) {
        flat_index -= cfg::kSpecFrames * cfg::kSpecChannels;
        continue;
      }
#endif
      const float compressed =
          CompressMelEnergy(mel_sum, 0, birdnet_frontend::kBranchPowerExponent[0]);
      if constexpr (cfg::kEnableDiagnosticLogs) {
        const float batchnorm =
            compressed * birdnet_frontend::kBatchNormMul[0] + birdnet_frontend::kBatchNormAdd[0];
        nonfinite_batch += !std::isfinite(batchnorm);
        if (!have_batchnorm) {
          batchnorm_min = batchnorm;
          batchnorm_max = batchnorm;
          have_batchnorm = true;
        } else {
          batchnorm_min = std::min(batchnorm_min, batchnorm);
          batchnorm_max = std::max(batchnorm_max, batchnorm);
        }
      }
      if (!StoreCompressedFeatureAt(
              flat_index,
              compressed,
              0,
              birdnet_frontend::kBatchNormMul[0],
              birdnet_frontend::kBatchNormAdd[0])) {
        return false;
      }
      flat_index -= (cfg::kSpecFrames * cfg::kSpecChannels);
    }
  }


  if (cfg::kEnableDiagnosticLogs && have_batchnorm) {
    birdnet_log::Write("[birdnet] branch0 decim2 f=[%d,%d] q1e3 nonfinite_real=%d nonfinite_mel=%d nonfinite_bn=%d\r\n",
            static_cast<int>(batchnorm_min * 1000.0f),
            static_cast<int>(batchnorm_max * 1000.0f),
            nonfinite_real,
            nonfinite_mel,
            nonfinite_batch);
  }
  return true;
}

#if defined(__GNUC__)
__attribute__((optimize("O2"), noinline))
#endif
bool FillBranch1ChannelFromRingWithDecimate2Rfft(
    const int16_t* pcm_ring,
    size_t ring_size,
    size_t write_idx,
    int16_t min_sample,
    float inv_range) {
  if (!pcm_ring || ring_size == 0) {
    return false;
  }

#if BIRDNET_FIELD_MEL_CODES
  const bool fast_q=birdnet_quant::enabled() && g_rt.input_int8_data && !cfg::kEnableDiagnosticLogs;
  if (fast_q) birdnet_quant::prepare(0);
#endif
  float batchnorm_min = 0.0f;
  float batchnorm_max = 0.0f;
  bool have_batchnorm = false;
  int nonfinite_real = 0;
  int nonfinite_mel = 0;
  int nonfinite_batch = 0;

  PrepareWindowAffine(
      kBranch1Decimate2FftSize,
      birdnet_tables::kWindow2,
      min_sample,
      inv_range,
      g_rt.window_scale,
      g_rt.window_bias);
  for (int frame_idx = 0; frame_idx < cfg::kSpecFrames; ++frame_idx) {
    const size_t ring_idx =
        (write_idx + static_cast<size_t>(frame_idx * birdnet_frontend::kBranch1Hop)) % ring_size;
    FillFrameDecimate2FromRing(
        pcm_ring,
        ring_size,
        ring_idx,
        kBranch1Decimate2FftSize,
        g_rt.window_scale,
        g_rt.window_bias,
        g_rt.Frame1());

    bool dense = false;
    if constexpr (kUseDirectRfftMel) {
#if BIRDNET_FRONTEND_CMSIS_RFFT_USE_MVE_STAGE
      dense = RunCmsisMveStageRfftPacked(&g_rt.rfft_branch1, g_rt.Frame1(), g_rt.fft_packed);
#else
      RunCmsisOverrideRfftPacked(&g_rt.rfft_branch1, g_rt.Frame1(), g_rt.fft_packed);
#endif
    } else {
#if BIRDNET_FRONTEND_CMSIS_RFFT_USE_MVE_STAGE
      RunCmsisMveStageRfftRealBins(
          &g_rt.rfft_branch1,
          kBranch1Decimate2FftSize,
          g_rt.Frame1(),
          g_rt.fft_packed,
          g_rt.real_bins_branch1,
          &nonfinite_real);
      for (int bin = 0; bin < kBranch1Decimate2Bins; ++bin) {
        g_rt.real_bins_branch1[bin] *= static_cast<float>(kBranch1Decimate2Factor);
      }
#else
      RunCmsisOverrideRfftRealBins(
          &g_rt.rfft_branch1,
          kBranch1Decimate2FftSize,
          g_rt.Frame1(),
          g_rt.fft_packed,
          g_rt.real_bins_branch1,
          &nonfinite_real,
          static_cast<float>(kBranch1Decimate2Factor));
#endif
    }

    int flat_index = (((cfg::kSpecMels - 1) * cfg::kSpecFrames) + frame_idx) * cfg::kSpecChannels;
    for (int mel_idx = 0; mel_idx < cfg::kSpecMels; ++mel_idx) {
      const uint16_t length = birdnet_frontend::kBranch1MelLengths[mel_idx];
      const float mel_sum =
          kUseDirectRfftMel
              ? SparseDotProductPackedRfftReal(
                    g_rt.fft_packed,
                    kBranch1Decimate2FftSize,
                    birdnet_frontend::kBranch1MelStarts[mel_idx],
                    birdnet_frontend::kBranch1MelWeights + birdnet_frontend::kBranch1MelOffsets[mel_idx],
                    length,
                    static_cast<float>(kBranch1Decimate2Factor), dense)
              : ((length > 0)
                     ? SparseDotProduct(
                           g_rt.real_bins_branch1 + birdnet_frontend::kBranch1MelStarts[mel_idx],
                           birdnet_frontend::kBranch1MelWeights + birdnet_frontend::kBranch1MelOffsets[mel_idx],
                           length)
                     : 0.0f);
      if constexpr (cfg::kEnableDiagnosticLogs) {
        nonfinite_mel += !std::isfinite(mel_sum);
      }
#if BIRDNET_FIELD_MEL_CODES
      if (fast_q && birdnet_quant::store(flat_index,mel_sum,0)) {
        flat_index -= cfg::kSpecFrames * cfg::kSpecChannels;
        continue;
      }
#endif
      const float compressed =
          CompressMelEnergy(mel_sum, 0, birdnet_frontend::kBranchPowerExponent[0]);
      if constexpr (cfg::kEnableDiagnosticLogs) {
        const float batchnorm =
            compressed * birdnet_frontend::kBatchNormMul[0] + birdnet_frontend::kBatchNormAdd[0];
        nonfinite_batch += !std::isfinite(batchnorm);
        if (!have_batchnorm) {
          batchnorm_min = batchnorm;
          batchnorm_max = batchnorm;
          have_batchnorm = true;
        } else {
          batchnorm_min = std::min(batchnorm_min, batchnorm);
          batchnorm_max = std::max(batchnorm_max, batchnorm);
        }
      }
      if (!StoreCompressedFeatureAt(
              flat_index,
              compressed,
              0,
              birdnet_frontend::kBatchNormMul[0],
              birdnet_frontend::kBatchNormAdd[0])) {
        return false;
      }
      flat_index -= (cfg::kSpecFrames * cfg::kSpecChannels);
    }
  }


  if (cfg::kEnableDiagnosticLogs && have_batchnorm) {
    birdnet_log::Write("[birdnet] branch0 decim2 f=[%d,%d] q1e3 nonfinite_real=%d nonfinite_mel=%d nonfinite_bn=%d\r\n",
            static_cast<int>(batchnorm_min * 1000.0f),
            static_cast<int>(batchnorm_max * 1000.0f),
            nonfinite_real,
            nonfinite_mel,
            nonfinite_batch);
  }
  return true;
}
#endif

#endif

#if BIRDNET_ANY_USE_CMSIS_CFFT
void RunCmsisCfftRealBins(
    const arm_cfft_instance_f32* cfft,
    int fft_size,
    const float* frame,
    float* fft_work,
    float* real_bins,
    int* nonfinite_real) {
  for (int i = 0; i < fft_size; ++i) {
    fft_work[2 * i] = frame[i];
    fft_work[(2 * i) + 1] = 0.0f;
  }

  RunCmsisCfftForwardFixed(cfft, fft_work, 1);

  const int bins = (fft_size / 2) + 1;
  if constexpr (cfg::kEnableDiagnosticLogs) {
    for (int k = 0; k < bins; ++k) {
      const float value = fft_work[2 * k];
      if (std::isfinite(value)) {
        real_bins[k] = value;
      } else {
        real_bins[k] = 0.0f;
        ++(*nonfinite_real);
      }
    }
  } else {
    for (int k = 0; k < bins; ++k) {
      real_bins[k] = fft_work[2 * k];
    }
  }
}

bool FillBranchChannelWithCmsisCfft(
    const int16_t* pcm_window,
    int16_t min_sample,
    float inv_range,
    int channel,
    int fft_size,
    int hop,
    const float* window,
    const arm_cfft_instance_f32* cfft,
    float* frame,
    float* fft_work,
    float* real_bins,
    const uint16_t* mel_starts,
    const uint16_t* mel_lengths,
    const uint16_t* mel_offsets,
    const float* packed_weights,
    float power_exponent,
    float bn_mul,
    float bn_add) {
  float batchnorm_min = 0.0f;
  float batchnorm_max = 0.0f;
  bool have_batchnorm = false;
  int nonfinite_real = 0;
  int nonfinite_mel = 0;
  int nonfinite_batch = 0;

  PrepareWindowAffine(fft_size, window, min_sample, inv_range, g_rt.window_scale, g_rt.window_bias);
  for (int frame_idx = 0; frame_idx < cfg::kSpecFrames; ++frame_idx) {
    const int start = frame_idx * hop;
    FillFrameFromPcm(pcm_window, start, fft_size, g_rt.window_scale, g_rt.window_bias, frame);

    RunCmsisCfftRealBins(cfft, fft_size, frame, fft_work, real_bins, &nonfinite_real);

    int flat_index = (((cfg::kSpecMels - 1) * cfg::kSpecFrames) + frame_idx) * cfg::kSpecChannels + channel;
    for (int mel_idx = 0; mel_idx < cfg::kSpecMels; ++mel_idx) {
      const uint16_t length = mel_lengths[mel_idx];
      const float mel_sum =
          (length > 0) ? SparseDotProduct(real_bins + mel_starts[mel_idx], packed_weights + mel_offsets[mel_idx], length)
                       : 0.0f;
      if constexpr (cfg::kEnableDiagnosticLogs) {
        nonfinite_mel += !std::isfinite(mel_sum);
      }
      const float compressed = CompressMelEnergy(mel_sum, channel, power_exponent);
      if constexpr (cfg::kEnableDiagnosticLogs) {
        const float batchnorm = compressed * bn_mul + bn_add;
        nonfinite_batch += !std::isfinite(batchnorm);
        if (!have_batchnorm) {
          batchnorm_min = batchnorm;
          batchnorm_max = batchnorm;
          have_batchnorm = true;
        } else {
          batchnorm_min = std::min(batchnorm_min, batchnorm);
          batchnorm_max = std::max(batchnorm_max, batchnorm);
        }
      }
      if (!StoreCompressedFeatureAt(flat_index, compressed, channel, bn_mul, bn_add)) {
        return false;
      }
      flat_index -= (cfg::kSpecFrames * cfg::kSpecChannels);
    }
  }

  if (cfg::kEnableDiagnosticLogs && have_batchnorm) {
    birdnet_log::Write("[birdnet] branch%d f=[%d,%d] q1e3 nonfinite_real=%d nonfinite_mel=%d nonfinite_bn=%d\r\n",
            channel,
            static_cast<int>(batchnorm_min * 1000.0f),
            static_cast<int>(batchnorm_max * 1000.0f),
            nonfinite_real,
            nonfinite_mel,
            nonfinite_batch);
  }
  return true;
}

#if defined(__GNUC__)
__attribute__((optimize("O2"), noinline))
#endif
bool FillBranchChannelFromRingWithCmsisCfft(
    const int16_t* pcm_ring,
    size_t ring_size,
    size_t write_idx,
    int16_t min_sample,
    float inv_range,
    int channel,
    int fft_size,
    int hop,
    const float* window,
    const arm_cfft_instance_f32* cfft,
    float* frame,
    float* fft_work,
    float* real_bins,
    const uint16_t* mel_starts,
    const uint16_t* mel_lengths,
    const uint16_t* mel_offsets,
    const float* packed_weights,
    float power_exponent,
    float bn_mul,
    float bn_add) {
  if (!pcm_ring || ring_size == 0) {
    return false;
  }

  float batchnorm_min = 0.0f;
  float batchnorm_max = 0.0f;
  bool have_batchnorm = false;
  int nonfinite_real = 0;
  int nonfinite_mel = 0;
  int nonfinite_batch = 0;

  PrepareWindowAffine(fft_size, window, min_sample, inv_range, g_rt.window_scale, g_rt.window_bias);
  for (int frame_idx = 0; frame_idx < cfg::kSpecFrames; ++frame_idx) {
    size_t ring_idx = (write_idx + static_cast<size_t>(frame_idx * hop)) % ring_size;
    FillFrameFromRing(pcm_ring, ring_size, ring_idx, fft_size, g_rt.window_scale, g_rt.window_bias, frame);

    RunCmsisCfftRealBins(cfft, fft_size, frame, fft_work, real_bins, &nonfinite_real);

    int flat_index = (((cfg::kSpecMels - 1) * cfg::kSpecFrames) + frame_idx) * cfg::kSpecChannels + channel;
    for (int mel_idx = 0; mel_idx < cfg::kSpecMels; ++mel_idx) {
      const uint16_t length = mel_lengths[mel_idx];
      const float mel_sum =
          (length > 0) ? SparseDotProduct(real_bins + mel_starts[mel_idx], packed_weights + mel_offsets[mel_idx], length)
                       : 0.0f;
      if constexpr (cfg::kEnableDiagnosticLogs) {
        nonfinite_mel += !std::isfinite(mel_sum);
      }
      const float compressed = CompressMelEnergy(mel_sum, channel, power_exponent);
      if constexpr (cfg::kEnableDiagnosticLogs) {
        const float batchnorm = compressed * bn_mul + bn_add;
        nonfinite_batch += !std::isfinite(batchnorm);
        if (!have_batchnorm) {
          batchnorm_min = batchnorm;
          batchnorm_max = batchnorm;
          have_batchnorm = true;
        } else {
          batchnorm_min = std::min(batchnorm_min, batchnorm);
          batchnorm_max = std::max(batchnorm_max, batchnorm);
        }
      }
      if (!StoreCompressedFeatureAt(flat_index, compressed, channel, bn_mul, bn_add)) {
        return false;
      }
      flat_index -= (cfg::kSpecFrames * cfg::kSpecChannels);
    }
  }

  if (cfg::kEnableDiagnosticLogs && have_batchnorm) {
    birdnet_log::Write("[birdnet] branch%d f=[%d,%d] q1e3 nonfinite_real=%d nonfinite_mel=%d nonfinite_bn=%d\r\n",
            channel,
            static_cast<int>(batchnorm_min * 1000.0f),
            static_cast<int>(batchnorm_max * 1000.0f),
            nonfinite_real,
            nonfinite_mel,
            nonfinite_batch);
  }
  return true;
}
#endif

#if BIRDNET_BRANCH1_USE_KISS || BIRDNET_BRANCH2_USE_KISS
#if defined(__GNUC__)
__attribute__((optimize("O2"), noinline))
#endif
bool FillBranchChannelFromRingWithKiss(
    const int16_t* pcm_ring,
    size_t ring_size,
    size_t write_idx,
    int16_t min_sample,
    float inv_range,
    int channel,
    int fft_size,
    int hop,
    const float* window,
    void* fft_cfg,
    float* frame,
    birdnet_kiss_fft_cpx* fft_out,
    float* real_bins,
    const uint16_t* mel_starts,
    const uint16_t* mel_lengths,
    const uint16_t* mel_offsets,
    const float* packed_weights,
    float power_exponent,
    float bn_mul,
    float bn_add) {
  if (!pcm_ring || ring_size == 0) {
    return false;
  }

  float batchnorm_min = 0.0f;
  float batchnorm_max = 0.0f;
  bool have_batchnorm = false;
  int nonfinite_real = 0;
  int nonfinite_mel = 0;
  int nonfinite_batch = 0;
  static bool s_logged_bad_bins[2] = {false, false};
  const int bins = (fft_size / 2) + 1;

  PrepareWindowAffine(fft_size, window, min_sample, inv_range, g_rt.window_scale, g_rt.window_bias);
  for (int frame_idx = 0; frame_idx < cfg::kSpecFrames; ++frame_idx) {
    size_t ring_idx = (write_idx + static_cast<size_t>(frame_idx * hop)) % ring_size;
    FillFrameFromRing(pcm_ring, ring_size, ring_idx, fft_size, g_rt.window_scale, g_rt.window_bias, frame);

    birdnet_kiss_fftr_run(fft_cfg, frame, fft_out);
    if constexpr (cfg::kEnableDiagnosticLogs) {
      int logged_bins = 0;
      for (int k = 0; k < bins; ++k) {
        real_bins[k] = fft_out[k].r;
        if (!std::isfinite(real_bins[k])) {
          ++nonfinite_real;
          if (!s_logged_bad_bins[channel] && frame_idx == 0 && logged_bins < 16) {
            birdnet_log::Write("[birdnet] branch%d bad_bin[%d]=%d\r\n", channel, logged_bins, k);
            ++logged_bins;
          }
          real_bins[k] = 0.0f;
        }
      }
      if (!s_logged_bad_bins[channel] && frame_idx == 0 && logged_bins > 0) {
        s_logged_bad_bins[channel] = true;
      }
    } else {
      for (int k = 0; k < bins; ++k) {
        real_bins[k] = fft_out[k].r;
      }
    }

    int flat_index = (((cfg::kSpecMels - 1) * cfg::kSpecFrames) + frame_idx) * cfg::kSpecChannels + channel;
    for (int mel_idx = 0; mel_idx < cfg::kSpecMels; ++mel_idx) {
      const uint16_t length = mel_lengths[mel_idx];
      const float mel_sum =
          (length > 0) ? SparseDotProduct(real_bins + mel_starts[mel_idx], packed_weights + mel_offsets[mel_idx], length)
                       : 0.0f;
      if constexpr (cfg::kEnableDiagnosticLogs) {
        nonfinite_mel += !std::isfinite(mel_sum);
      }
      const float compressed = CompressMelEnergy(mel_sum, channel, power_exponent);
      if constexpr (cfg::kEnableDiagnosticLogs) {
        const float batchnorm = compressed * bn_mul + bn_add;
        nonfinite_batch += !std::isfinite(batchnorm);
        if (!have_batchnorm) {
          batchnorm_min = batchnorm;
          batchnorm_max = batchnorm;
          have_batchnorm = true;
        } else {
          batchnorm_min = std::min(batchnorm_min, batchnorm);
          batchnorm_max = std::max(batchnorm_max, batchnorm);
        }
      }
      if (!StoreCompressedFeatureAt(flat_index, compressed, channel, bn_mul, bn_add)) {
        return false;
      }
      flat_index -= (cfg::kSpecFrames * cfg::kSpecChannels);
    }
  }

  if (cfg::kEnableDiagnosticLogs && have_batchnorm) {
    birdnet_log::Write("[birdnet] branch%d f=[%d,%d] q1e3 nonfinite_real=%d nonfinite_mel=%d nonfinite_bn=%d\r\n",
            channel,
            static_cast<int>(batchnorm_min * 1000.0f),
            static_cast<int>(batchnorm_max * 1000.0f),
            nonfinite_real,
            nonfinite_mel,
            nonfinite_batch);
  }
  return true;
}
#endif

