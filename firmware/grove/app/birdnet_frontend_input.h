// Private inference implementation; included at its original namespace scope.
#pragma once
bool TFLMInferencePipeline::FillInputTensorFromBirdNetFrontend(const int16_t* pcm_window) {
  if (!pcm_window || !g_rt.input) {
    return false;
  }
#if BIRDNET_FRONTEND_STREAMING
  const uint64_t streaming_start = PlatformMillis();
  if (!FillStreamingFrontend(pcm_window, capture_end_samples_)) return false;
  (void)streaming_start;
  return true;
#endif

  int16_t min_sample = 0;
  int16_t max_sample = 0;
  FindMinMaxPcm(pcm_window, cfg::kWindowSamples, &min_sample, &max_sample);
  const float range = static_cast<float>(static_cast<int>(max_sample) - static_cast<int>(min_sample));
  const float inv_range = 2.0f / (range + 1e-6f);

  static bool s_logged_input_once = false;
  if (!s_logged_input_once) {
    birdnet_log::Status("[birdnet] frontend min=%d max=%d range_q1e3=%d\r\n",
                  static_cast<int>(min_sample),
                  static_cast<int>(max_sample),
                  static_cast<int>(range * 1000.0f));
  }

#if BIRDNET_BRANCH1_USE_DECIMATE2_RFFT
  if (!FillBranch1ChannelWithDecimate2Rfft(
          pcm_window,
          min_sample,
          inv_range)) {
    return false;
  }
#elif BIRDNET_BRANCH1_USE_CMSIS_RFFT_SCALAR
  if (!FillBranchChannelWithCmsisScalarRfft(
          pcm_window,
          min_sample,
          inv_range,
          0,
          birdnet_frontend::kBranch1FftSize,
          birdnet_frontend::kBranch1Hop,
          birdnet_tables::kWindow1,
          &g_rt.rfft_branch1,
          BIRDNET_BRANCH1_RFFT_USE_C_OVERRIDE,
          g_rt.Frame1(),
          g_rt.fft_packed,
          g_rt.real_bins_branch1,
          birdnet_frontend::kBranch1MelStarts,
          birdnet_frontend::kBranch1MelLengths,
          birdnet_frontend::kBranch1MelOffsets,
          birdnet_frontend::kBranch1MelWeights,
          birdnet_frontend::kBranchPowerExponent[0],
          birdnet_frontend::kBatchNormMul[0],
          birdnet_frontend::kBatchNormAdd[0])) {
    return false;
  }
#elif BIRDNET_BRANCH1_USE_CMSIS_CFFT
  if (!FillBranchChannelWithCmsisCfft(
          pcm_window,
          min_sample,
          inv_range,
          0,
          birdnet_frontend::kBranch1FftSize,
          birdnet_frontend::kBranch1Hop,
          birdnet_tables::kWindow1,
          g_rt.cfft_branch1,
          g_rt.Frame1(),
          g_rt.fft_work,
          g_rt.real_bins_branch1,
          birdnet_frontend::kBranch1MelStarts,
          birdnet_frontend::kBranch1MelLengths,
          birdnet_frontend::kBranch1MelOffsets,
          birdnet_frontend::kBranch1MelWeights,
          birdnet_frontend::kBranchPowerExponent[0],
          birdnet_frontend::kBatchNormMul[0],
          birdnet_frontend::kBatchNormAdd[0])) {
    return false;
  }
#else
  if (!FillBranchChannelWithKiss(
          pcm_window,
          min_sample,
          inv_range,
          0,
          birdnet_frontend::kBranch1FftSize,
          birdnet_frontend::kBranch1Hop,
          birdnet_tables::kWindow1,
          g_rt.kiss_branch1_cfg,
          g_rt.Frame1(),
          g_rt.fft_out_branch1,
          g_rt.real_bins_branch1,
          birdnet_frontend::kBranch1MelStarts,
          birdnet_frontend::kBranch1MelLengths,
          birdnet_frontend::kBranch1MelOffsets,
          birdnet_frontend::kBranch1MelWeights,
          birdnet_frontend::kBranchPowerExponent[0],
          birdnet_frontend::kBatchNormMul[0],
          birdnet_frontend::kBatchNormAdd[0])) {
    return false;
  }
#endif

#if BIRDNET_BRANCH2_USE_CMSIS_RFFT_SCALAR
  if (!FillBranchChannelWithCmsisScalarRfft(
          pcm_window,
          min_sample,
          inv_range,
          1,
          birdnet_frontend::kBranch2FftSize,
          birdnet_frontend::kBranch2Hop,
          birdnet_tables::kWindow2,
          &g_rt.rfft_branch2,
          BIRDNET_BRANCH2_RFFT_USE_C_OVERRIDE,
          g_rt.Frame2(),
          g_rt.fft_packed,
          g_rt.real_bins_branch2,
          birdnet_frontend::kBranch2MelStarts,
          birdnet_frontend::kBranch2MelLengths,
          birdnet_frontend::kBranch2MelOffsets,
          birdnet_frontend::kBranch2MelWeights,
          birdnet_frontend::kBranchPowerExponent[1],
          birdnet_frontend::kBatchNormMul[1],
          birdnet_frontend::kBatchNormAdd[1])) {
    return false;
  }
#elif BIRDNET_BRANCH2_USE_CMSIS_CFFT
  if (!FillBranchChannelWithCmsisCfft(
          pcm_window,
          min_sample,
          inv_range,
          1,
          birdnet_frontend::kBranch2FftSize,
          birdnet_frontend::kBranch2Hop,
          birdnet_tables::kWindow2,
          g_rt.cfft_branch2,
          g_rt.Frame2(),
          g_rt.fft_work,
          g_rt.real_bins_branch2,
          birdnet_frontend::kBranch2MelStarts,
          birdnet_frontend::kBranch2MelLengths,
          birdnet_frontend::kBranch2MelOffsets,
          birdnet_frontend::kBranch2MelWeights,
          birdnet_frontend::kBranchPowerExponent[1],
          birdnet_frontend::kBatchNormMul[1],
          birdnet_frontend::kBatchNormAdd[1])) {
    return false;
  }
#else
  if (!FillBranchChannelWithKiss(
          pcm_window,
          min_sample,
          inv_range,
          1,
          birdnet_frontend::kBranch2FftSize,
          birdnet_frontend::kBranch2Hop,
          birdnet_tables::kWindow2,
          g_rt.kiss_branch2_cfg,
          g_rt.Frame2(),
          g_rt.fft_out_branch2,
          g_rt.real_bins_branch2,
          birdnet_frontend::kBranch2MelStarts,
          birdnet_frontend::kBranch2MelLengths,
          birdnet_frontend::kBranch2MelOffsets,
          birdnet_frontend::kBranch2MelWeights,
          birdnet_frontend::kBranchPowerExponent[1],
          birdnet_frontend::kBatchNormMul[1],
          birdnet_frontend::kBatchNormAdd[1])) {
    return false;
  }
#endif

  if (!s_logged_input_once && g_rt.input->type == kTfLiteInt8) {
    int qmin = 127;
    int qmax = -128;
    for (int i = 0; i < static_cast<int>(g_rt.input->bytes); ++i) {
      const int q = static_cast<int>(g_rt.input->data.int8[i]);
      qmin = std::min(qmin, q);
      qmax = std::max(qmax, q);
    }
    birdnet_log::Status("[birdnet] input qmin=%d qmax=%d\r\n", qmin, qmax);
    s_logged_input_once = true;
  }
  if (cfg::kEnableDiagnosticLogs) {
    LogInputChannelStats();
  }

  return true;
}

bool TFLMInferencePipeline::FillInputTensorFromBirdNetFrontendRing(
    const int16_t* pcm_ring,
    size_t ring_size,
    size_t write_idx) {
  if (!pcm_ring || !g_rt.input || ring_size < static_cast<size_t>(cfg::kWindowSamples)) {
    return false;
  }

  int16_t min_sample = 0;
  int16_t max_sample = 0;
  FindMinMaxRing(pcm_ring, ring_size, write_idx, cfg::kWindowSamples, &min_sample, &max_sample);

  const float range = static_cast<float>(static_cast<int>(max_sample) - static_cast<int>(min_sample));
  const float inv_range = 2.0f / (range + 1e-6f);

  static bool s_logged_input_once = false;
  if (!s_logged_input_once) {
    birdnet_log::Status("[birdnet] frontend ring write_idx=%u ring_size=%u min=%d max=%d range_q1e3=%d\r\n",
                  static_cast<unsigned int>(write_idx),
                  static_cast<unsigned int>(ring_size),
                  static_cast<int>(min_sample),
                  static_cast<int>(max_sample),
                  static_cast<int>(range * 1000.0f));
  }

#if BIRDNET_BRANCH1_USE_DECIMATE2_RFFT
  if (!FillBranch1ChannelFromRingWithDecimate2Rfft(
          pcm_ring,
          ring_size,
          write_idx,
          min_sample,
          inv_range)) {
    return false;
  }
#elif BIRDNET_BRANCH1_USE_CMSIS_RFFT_SCALAR
  if (!FillBranchChannelFromRingWithCmsisScalarRfft(
          pcm_ring,
          ring_size,
          write_idx,
          min_sample,
          inv_range,
          0,
          birdnet_frontend::kBranch1FftSize,
          birdnet_frontend::kBranch1Hop,
          birdnet_tables::kWindow1,
          &g_rt.rfft_branch1,
          BIRDNET_BRANCH1_RFFT_USE_C_OVERRIDE,
          g_rt.Frame1(),
          g_rt.fft_packed,
          g_rt.real_bins_branch1,
          birdnet_frontend::kBranch1MelStarts,
          birdnet_frontend::kBranch1MelLengths,
          birdnet_frontend::kBranch1MelOffsets,
          birdnet_frontend::kBranch1MelWeights,
          birdnet_frontend::kBranchPowerExponent[0],
          birdnet_frontend::kBatchNormMul[0],
          birdnet_frontend::kBatchNormAdd[0])) {
    return false;
  }
#elif BIRDNET_BRANCH1_USE_CMSIS_CFFT
  if (!FillBranchChannelFromRingWithCmsisCfft(
          pcm_ring,
          ring_size,
          write_idx,
          min_sample,
          inv_range,
          0,
          birdnet_frontend::kBranch1FftSize,
          birdnet_frontend::kBranch1Hop,
          birdnet_tables::kWindow1,
          g_rt.cfft_branch1,
          g_rt.Frame1(),
          g_rt.fft_work,
          g_rt.real_bins_branch1,
          birdnet_frontend::kBranch1MelStarts,
          birdnet_frontend::kBranch1MelLengths,
          birdnet_frontend::kBranch1MelOffsets,
          birdnet_frontend::kBranch1MelWeights,
          birdnet_frontend::kBranchPowerExponent[0],
          birdnet_frontend::kBatchNormMul[0],
          birdnet_frontend::kBatchNormAdd[0])) {
    return false;
  }
#else
  if (!FillBranchChannelFromRingWithKiss(
          pcm_ring,
          ring_size,
          write_idx,
          min_sample,
          inv_range,
          0,
          birdnet_frontend::kBranch1FftSize,
          birdnet_frontend::kBranch1Hop,
          birdnet_tables::kWindow1,
          g_rt.kiss_branch1_cfg,
          g_rt.Frame1(),
          g_rt.fft_out_branch1,
          g_rt.real_bins_branch1,
          birdnet_frontend::kBranch1MelStarts,
          birdnet_frontend::kBranch1MelLengths,
          birdnet_frontend::kBranch1MelOffsets,
          birdnet_frontend::kBranch1MelWeights,
          birdnet_frontend::kBranchPowerExponent[0],
          birdnet_frontend::kBatchNormMul[0],
          birdnet_frontend::kBatchNormAdd[0])) {
    return false;
  }
#endif

#if BIRDNET_BRANCH2_USE_CMSIS_RFFT_SCALAR
  if (!FillBranchChannelFromRingWithCmsisScalarRfft(
          pcm_ring,
          ring_size,
          write_idx,
          min_sample,
          inv_range,
          1,
          birdnet_frontend::kBranch2FftSize,
          birdnet_frontend::kBranch2Hop,
          birdnet_tables::kWindow2,
          &g_rt.rfft_branch2,
          BIRDNET_BRANCH2_RFFT_USE_C_OVERRIDE,
          g_rt.Frame2(),
          g_rt.fft_packed,
          g_rt.real_bins_branch2,
          birdnet_frontend::kBranch2MelStarts,
          birdnet_frontend::kBranch2MelLengths,
          birdnet_frontend::kBranch2MelOffsets,
          birdnet_frontend::kBranch2MelWeights,
          birdnet_frontend::kBranchPowerExponent[1],
          birdnet_frontend::kBatchNormMul[1],
          birdnet_frontend::kBatchNormAdd[1])) {
    return false;
  }
#elif BIRDNET_BRANCH2_USE_CMSIS_CFFT
  if (!FillBranchChannelFromRingWithCmsisCfft(
          pcm_ring,
          ring_size,
          write_idx,
          min_sample,
          inv_range,
          1,
          birdnet_frontend::kBranch2FftSize,
          birdnet_frontend::kBranch2Hop,
          birdnet_tables::kWindow2,
          g_rt.cfft_branch2,
          g_rt.Frame2(),
          g_rt.fft_work,
          g_rt.real_bins_branch2,
          birdnet_frontend::kBranch2MelStarts,
          birdnet_frontend::kBranch2MelLengths,
          birdnet_frontend::kBranch2MelOffsets,
          birdnet_frontend::kBranch2MelWeights,
          birdnet_frontend::kBranchPowerExponent[1],
          birdnet_frontend::kBatchNormMul[1],
          birdnet_frontend::kBatchNormAdd[1])) {
    return false;
  }
#else
  if (!FillBranchChannelFromRingWithKiss(
          pcm_ring,
          ring_size,
          write_idx,
          min_sample,
          inv_range,
          1,
          birdnet_frontend::kBranch2FftSize,
          birdnet_frontend::kBranch2Hop,
          birdnet_tables::kWindow2,
          g_rt.kiss_branch2_cfg,
          g_rt.Frame2(),
          g_rt.fft_out_branch2,
          g_rt.real_bins_branch2,
          birdnet_frontend::kBranch2MelStarts,
          birdnet_frontend::kBranch2MelLengths,
          birdnet_frontend::kBranch2MelOffsets,
          birdnet_frontend::kBranch2MelWeights,
          birdnet_frontend::kBranchPowerExponent[1],
          birdnet_frontend::kBatchNormMul[1],
          birdnet_frontend::kBatchNormAdd[1])) {
    return false;
  }
#endif

  if (!s_logged_input_once && g_rt.input->type == kTfLiteInt8) {
    int qmin = 127;
    int qmax = -128;
    for (int i = 0; i < static_cast<int>(g_rt.input->bytes); ++i) {
      const int q = static_cast<int>(g_rt.input->data.int8[i]);
      qmin = std::min(qmin, q);
      qmax = std::max(qmax, q);
    }
    birdnet_log::Status("[birdnet] input qmin=%d qmax=%d\r\n", qmin, qmax);
    s_logged_input_once = true;
  }
  if (cfg::kEnableDiagnosticLogs) {
    LogInputChannelStats();
  }

  return true;
}

