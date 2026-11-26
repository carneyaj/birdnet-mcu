// Private application implementation. Included inside birdnet_we2.cpp
// after SDK headers; not a standalone public header.
#pragma once
void LogResult(const InferenceResult& result, const Sht41Reading& sht41) {
  if constexpr (!BIRDNET_ENABLE_PREDICTION_LOGS) return;
  birdnet_log::Status("[birdnet] model_init_ms=%lu\r\n", static_cast<unsigned long>(result.init_ms));
  birdnet_log::Write("[birdnet] ts_ms=%llu infer_ms_q1e2=%d front_ms_q1e2=%d invoke_ms_q1e2=%d decode_ms_q1e2=%d pred=%d score_q1e4=%d detected=%d sht_ok=%d temp_c_q1e2=%d rh_q1e2=%d label=%s\r\n",
          static_cast<unsigned long long>(result.timestamp_ms),
          static_cast<int>(result.inference_ms * 100.0f),
          static_cast<int>(result.frontend_ms * 100.0f),
          static_cast<int>(result.invoke_ms * 100.0f),
          static_cast<int>(result.decode_ms * 100.0f),
          result.pred_class,
          static_cast<int>(result.pred_score * 10000.0f),
          result.detected ? 1 : 0,
          sht41.ok ? 1 : 0,
          sht41.temp_c_q1e2,
          sht41.rh_q1e2,
          birdnet_labels::LabelForIndex(result.pred_class));
  for (int rank = 0; rank < cfg::kTopK; ++rank) {
    const int cls = result.top_indices[rank];
    if (cls < 0) {
      continue;
    }
    birdnet_log::Write("[birdnet] top%d idx=%d score_q1e4=%d label=%s\r\n",
            rank + 1,
            cls,
            static_cast<int>(result.top_scores[rank] * 10000.0f),
            birdnet_labels::LabelForIndex(cls));
  }
}

// Validated immutable snapshot -> inference -> policy/storage/UI -> sleep.
// No peripheral may retain a pointer to the snapshot after this returns.
void ProcessCapturedWindow(TFLMInferencePipeline& pipeline) {
  const uint64_t processing_started_ms = PlatformMillis();
  FileAdvertise();
  #if BIRDNET_ENABLE_FIELD_MONITORING
  const uint64_t field_now = AudioEndTimestampMs();
  ReadBattery();
  if (g_battery_reading.ok && (g_field.battery_min_mv == 0 ||
      g_battery_reading.millivolts < g_field.battery_min_mv))
    g_field.battery_min_mv = g_battery_reading.millivolts;
  const bool power_changed = g_field.battery.Update(
      field_now, g_battery_reading.ok, g_battery_reading.millivolts);
  hx_drv_mb_set_sharedata(BN_MB_POWER_SAVE, g_field.battery.paused ? 1 : 0);
  if (power_changed) {
    g_field.dirty = true;
    pipeline.ResetStreamingCache();
    if (g_field.battery.paused) {
      g_field.pause_started_ms = field_now;
      SetDetectionOutputs(nullptr, nullptr);
      SdLogFlush(field_now, true);
    } else {
      g_field.paused_ms += field_now - g_field.pause_started_ms;
    }
  }
  if (g_field.battery.paused) {
    if (birdnet_field::AmbientDue(field_now, &g_field.last_hour)) ++g_field.ambient_missed;
    FieldHealth(field_now);
    SdLogPowerDown();
    ResetMailboxWindowState();
    // Keep the verified capture/progress watchdog alive. No classifier,
    // frontend, encoder or routine environmental SD writes in standby.
    FilePollAndClose();
    EnterAudioOnlyPrerollingMode();
    return;
  }
  #endif
  InferenceResult result{};
  AudioWindowDiagnostics audio_stats{};
  if constexpr (cfg::kEnableLedDiagnostics) {
    audio_stats = MeasureAudioWindow(g_audio_ring, cfg::kWindowSamples);
  }
  if (!SetModelXipEnabled(true)) {
    birdnet_log::Write("[birdnet] XIP enable before inference failed\r\n");
    while (1) {}
  }
  if (g_retained_runtime.fixed_window_count == 1u && !VerifyInstalledModel()) {
    birdnet_log::Write("[birdnet] model readback mismatch; awaiting watchdog reset\r\n");
    while (true) __WFI();
  }
  (void)StartSht41();
  g_retained_runtime.phase = 2;
  pipeline.SetCaptureWindow(g_audio_ring_total_samples);
  #if BIRDNET_ENABLE_SD_DETECTION_LOGGING
  pipeline.SetRecordingWindow(g_audio_ring_total_samples,
                              g_sd_log_state.last_detection_end_samples);
  #endif
  Sht41Reading sht41{};
  const bool inference_ok = pipeline.Run(g_audio_ring, &result);
  // The result owns its scores. Release the NPU before any blocking storage work.
  pipeline.PrepareForSleep();
  if (inference_ok) {
    (void)ArmActiveWatchdog(3);
    if constexpr (cfg::kEnableLedDiagnostics) {
      birdnet_log::Write("[bntrace] result window=%lu cls=%d q1e4=%d hit=%d vocal=%d\r\n",
          (unsigned long)g_retained_runtime.fixed_window_count, result.pred_class,
          int(result.pred_score * 10000.0f), int(result.detected), int(result.human_vocal_detected));
    }
    const uint64_t audio_ts_ms = AudioEndTimestampMs();
    result.timestamp_ms = audio_ts_ms;
    ReadSht41(&sht41);
  #if !BIRDNET_ENABLE_FIELD_MONITORING
    ReadBattery();
  #else
    ++g_field.inferences;
    const uint64_t end = g_audio_ring_total_samples;
    const uint64_t covered = g_field.last_success_end && end > g_field.last_success_end
        ? std::min(uint64_t(cfg::kWindowSamples), end - g_field.last_success_end)
        : uint64_t(cfg::kWindowSamples);
    g_field.monitored_samples += covered;
    g_field.last_success_end = end;
  #endif
    g_retained_runtime.phase = 4;
    LogResult(result, sht41);
    if constexpr (cfg::kDetectionLedRetentionSelfTest) {
      const bool self_test_on = (g_mb_window_count == 1u);
      SetDetectionLed(self_test_on);
      birdnet_log::Status("[birdnet] detection LED retention self-test window=%llu on=%d\r\n",
                    static_cast<unsigned long long>(g_mb_window_count),
                    self_test_on ? 1 : 0);
    }
    SdLogCaptureDetections(result, audio_ts_ms);
    SdLogCaptureEnvironment(sht41, audio_ts_ms);
    if constexpr (cfg::kEnableLedDiagnostics) birdnet_log::Write("[bntrace] queued\r\n");
    const auto recording = birdnet_detection::ForRecording(result, birdnet_labels::CanTriggerDetectionAudio);
    (void)SdSaveDetectionAudio(recording,
                               audio_ts_ms,
                               g_audio_ring,
                               cfg::kWindowSamples,
                               0,
                               cfg::kWindowSamples);
    if constexpr (cfg::kEnableLoraDetectionTx) {
      (void)birdnet_lora::SendDetection(result);
    }
    SdLogFlush(audio_ts_ms, false);
    if constexpr (cfg::kEnableLedDiagnostics) birdnet_log::Write("[bntrace] flush done\r\n");
  #if !BIRDNET_ENABLE_FIELD_MONITORING
    SdLogPowerDown();
    g_retained_runtime.phase = 5;
    // Assert retained user-visible outputs only after operations that can
    // block on external peripherals, so a storage fault cannot strand the
    // detection LED high indefinitely.
    SetDetectionOutputs(&result, &sht41);
  #endif
    if constexpr (cfg::kEnableLedDiagnostics) {
      uint8_t led_level = 0xffu;
      const bool led_read_ok =
          hx_drv_gpio_get_in_value(GPIO20, &led_level) == GPIO_NO_ERROR;
      birdnet_log::Write("[birdnet] audio_diag window=%llu fill=%lu min=%d max=%d mean_q1e2=%ld "
              "mean_abs=%lu rms=%lu delta=%lu zero=%lu sat=%lu hash=%08lx "
              "samples=%d,%d,%d,%d led_cmd=%d led_pin=%d\r\n",
              static_cast<unsigned long long>(g_mb_window_count),
              static_cast<unsigned long>(g_mb_window_fill_samples),
              static_cast<int>(audio_stats.min_sample),
              static_cast<int>(audio_stats.max_sample),
              static_cast<long>(audio_stats.mean_q1e2),
              static_cast<unsigned long>(audio_stats.mean_abs),
              static_cast<unsigned long>(audio_stats.rms),
              static_cast<unsigned long>(audio_stats.mean_abs_delta),
              static_cast<unsigned long>(audio_stats.zero_samples),
              static_cast<unsigned long>(audio_stats.saturated_samples),
              static_cast<unsigned long>(audio_stats.hash),
              static_cast<int>(g_audio_ring[0]),
              static_cast<int>(g_audio_ring[cfg::kWindowSamples / 3]),
              static_cast<int>(g_audio_ring[(cfg::kWindowSamples * 2) / 3]),
              static_cast<int>(g_audio_ring[cfg::kWindowSamples - 1]),
              result.detected ? 1 : 0,
              led_read_ok ? static_cast<int>(led_level) : -1);
    }
  } else {
  #if BIRDNET_ENABLE_FIELD_MONITORING
    ++g_field.inference_errors;
  #endif
    SetDetectionOutputs(nullptr, nullptr);
    birdnet_log::Write("[birdnet] inference failed\r\n");
  }
  #if BIRDNET_ENABLE_FIELD_MONITORING
  // Hourly scheduling is independent of detection, but audio must pass the veto.
  if (birdnet_field::AmbientDue(field_now, &g_field.last_hour)) {
    InferenceResult ambient{};
    ambient.detected = true;
    ambient.pred_class = -1;
    // No unclassified fallback recording: an inference failure cannot
    // establish that this hourly window passed the human-vocal veto.
    if (birdnet_detection::CanSaveAmbient(inference_ok, result) &&
        SdSaveDetectionAudio(ambient, field_now, g_audio_ring, cfg::kWindowSamples, 0, cfg::kWindowSamples))
      ++g_field.ambient_saved;
    else ++g_field.ambient_missed;
  }
  if (field_now >= g_sd_log_state.next_retry_ms &&
      (g_field.dirty || field_now / birdnet_field::kHealthMs > g_field.health_period)) {
    const auto stats = MeasureAudioWindow(g_audio_ring, cfg::kWindowSamples);
    g_field.audio_peak = std::max(std::abs(int(stats.min_sample)), std::abs(int(stats.max_sample)));
    g_field.audio_clipped = stats.saturated_samples;
    FieldHealth(field_now);
  }
  SdLogPowerDown();
  g_retained_runtime.phase = 5;
  if constexpr (cfg::kEnableLedDiagnostics) birdnet_log::Write("[bntrace] outputs begin\r\n");
  if (inference_ok) SetDetectionOutputs(&result, &sht41);
  if constexpr (cfg::kEnableLedDiagnostics) birdnet_log::Write("[bntrace] outputs done\r\n");
  #endif
  #if BIRDNET_ENABLE_FIELD_MONITORING
  // One compact health line per minute of captured audio, plus first window.
  // Retained counters detect warm-wake failures without per-window tracing.
  if (g_retained_runtime.fixed_window_count == 1u ||
      g_retained_runtime.fixed_window_count % 20u == 0u) {
    birdnet_log::Write("[bnhealth] end=%llu ok=%lu errors=%lu skipped=%lu rejected=%lu frontend_ms=%lu invoke_ms=%lu\r\n",
            (unsigned long long)g_audio_ring_total_samples,
            (unsigned long)g_field.inferences, (unsigned long)g_field.inference_errors,
            (unsigned long)g_retained_runtime.skipped_windows,
            (unsigned long)g_retained_runtime.rejected_windows,
            (unsigned long)result.frontend_ms, (unsigned long)result.invoke_ms);
  }
  #endif
  ResetMailboxWindowState();
  FilePollAndClose();
  birdnet_log::Status("[birdnet] processing_to_sleep_ms=%lu\r\n",
                static_cast<unsigned long>(PlatformMillis() - processing_started_ms));
  g_retained_runtime.phase = 6;
  if (cfg::kUseAudioOnlyPrerolling) {
    EnterAudioOnlyPrerollingMode();
  }
}
