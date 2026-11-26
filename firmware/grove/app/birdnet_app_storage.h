// Private application implementation. Included inside birdnet_we2.cpp
// after SDK headers; not a standalone public header.
#pragma once
#if BIRDNET_ENABLE_SD_DETECTION_LOGGING
void SdLogPinmuxSpi() {
  hx_drv_scu_set_PB2_pinmux(SCU_PB2_PINMUX_SPI_M_DO_1, 1);
  hx_drv_scu_set_PB3_pinmux(SCU_PB3_PINMUX_SPI_M_DI_1, 1);
  hx_drv_scu_set_PB4_pinmux(SCU_PB4_PINMUX_SPI_M_SCLK_1, 1);
  hx_drv_scu_set_PB5_pinmux(SCU_PB5_PINMUX_SPI_M_CS_1, 1);
}

bool SdLogPrepareSpi() {
  SdLogPinmuxSpi();
  g_sd_spi_active = true;
  return true;
}

void SdLogPowerDown() {
  if (!g_sd_spi_active) return;
  const DRESULT res = disk_ioctl(0, CTRL_POWER_OFF, nullptr);
  if (res != RES_OK && cfg::kEnableStatusLogs) {
    birdnet_log::Write("[birdnet] sd power off failed res=%d\r\n", static_cast<int>(res));
  }
  f_unmount("");
  // CMSIS PowerControl(OFF) only disables this SDK's SPI peripheral. Close its
  // open handle too, so an export remount in the same wake can reopen it.
  (void)Driver_SPI0.Uninitialize();
  hx_drv_scu_set_PB5_pinmux(SCU_PB5_PINMUX_GPIO16, 0);
  hx_drv_gpio_set_output(GPIO16, GPIO_OUT_HIGH);
  g_sd_mounted = false;
  g_sd_spi_active = false;
}

void SdLogFailed(uint64_t now_ms) {
#if BIRDNET_ENABLE_FIELD_MONITORING
  ++g_field.sd_errors;
#endif
  g_sd_log_state.session_id = 0; // Recover a fresh durable session after storage faults.
  g_sd_log_state.retry_delay_ms = birdnet_storage::NextRetryDelay(g_sd_log_state.retry_delay_ms);
  g_sd_log_state.next_retry_ms = now_ms + g_sd_log_state.retry_delay_ms;
}

bool SdLogMount(uint64_t now_ms) {
  if (now_ms < g_sd_log_state.next_retry_ms) return false;
  if (g_sd_mounted) return true;
  if (!SdLogPrepareSpi() || f_mount(&g_sd_log_fs, "", 1) != FR_OK) {
    SdLogFailed(now_ms);
    if (!g_sd_log_state.log_error_logged) {
      birdnet_log::Write("[birdnet] SD unavailable; buffered records retained, retry_ms=%lu\r\n",
              static_cast<unsigned long>(g_sd_log_state.retry_delay_ms));
      g_sd_log_state.log_error_logged = true;
    }
    SdLogPowerDown();
    return false;
  }
  if (g_sd_log_state.log_error_logged) birdnet_log::Write("[birdnet] SD available again\r\n");
  g_sd_log_state.log_error_logged = false;
  g_sd_mounted = true;
  return true;
}

// Card-local session IDs survive battery loss. Never use an ID before sync.
// A malformed committed record fails closed rather than reusing an old ID.
bool SdLogEnsureSession() {
  if (g_sd_log_state.session_id) return true;
  FIL file{};
  if (f_open(&file, "BOOTSEQ.BIN", FA_OPEN_ALWAYS | FA_READ | FA_WRITE) != FR_OK) return false;
  const FSIZE_t end = f_size(&file) / 16u * 16u;
  uint8_t record[16]{};
  UINT count = 0;
  uint32_t previous = 0;
  bool ok = true;
  if (end) {
    ok = f_lseek(&file, end - 16) == FR_OK &&
         f_read(&file, record, sizeof(record), &count) == FR_OK && count == sizeof(record) &&
         birdnet_clip::ValidSession(record);
    if (ok) previous = birdnet_clip::Get32(record + 4);
  }
  ok = ok && previous != UINT32_MAX;
  if (ok) {
    birdnet_clip::SessionRecord(previous + 1, record);
    ok = f_lseek(&file, end) == FR_OK &&
         f_write(&file, record, sizeof(record), &count) == FR_OK && count == sizeof(record) &&
         f_sync(&file) == FR_OK;
  }
  if (f_close(&file) != FR_OK) ok = false;
  if (ok) g_sd_log_state.session_id = previous + 1;
  return ok;
}

uint16_t ProbabilityToQ1e4(float probability) {
  if (probability <= 0.0f) {
    return 0;
  }
  if (probability >= 1.0f) {
    return 10000;
  }
  return static_cast<uint16_t>(std::lround(probability * 10000.0f));
}

uint64_t AudioEndTimestampMs() {
  return (g_audio_ring_total_samples * 1000ull) / static_cast<uint64_t>(cfg::kSampleRateHz);
}

void SdLogAppendRecord(const DetectionLogRecord& record) {
  if (g_sd_log_state.log_record_count >= cfg::kSdLogBufferRecords) {
    std::memmove(&g_sd_log_state.log_records[0],
                 &g_sd_log_state.log_records[1],
                 static_cast<size_t>(cfg::kSdLogBufferRecords - 1) *
                     sizeof(g_sd_log_state.log_records[0]));
    g_sd_log_state.log_record_count = cfg::kSdLogBufferRecords - 1;
    ++g_sd_log_state.dropped_records;
  }
  g_sd_log_state.log_records[g_sd_log_state.log_record_count++] = record;
}

void SdLogAppendEnvRecord(const EnvLogRecord& record) {
  if (g_sd_log_state.env_log_record_count >= cfg::kSdEnvLogBufferRecords) {
    std::memmove(&g_sd_log_state.env_log_records[0],
                 &g_sd_log_state.env_log_records[1],
                 static_cast<size_t>(cfg::kSdEnvLogBufferRecords - 1) *
                     sizeof(g_sd_log_state.env_log_records[0]));
    g_sd_log_state.env_log_record_count = cfg::kSdEnvLogBufferRecords - 1;
    ++g_sd_log_state.env_dropped_records;
  }
  g_sd_log_state.env_log_records[g_sd_log_state.env_log_record_count++] = record;
}

void SdLogCaptureDetections(const InferenceResult& result, uint64_t timestamp_ms) {
  if constexpr (!cfg::kEnableSdDetectionLogging) {
    (void)result;
    (void)timestamp_ms;
    return;
  }

  for (int i = 0; i < result.log_detection_count; ++i) {
    const int cls = result.log_detection_indices[i];
    if (cls < 0 || cls >= cfg::kNumClasses) {
      continue;
    }
    DetectionLogRecord record{};
    record.timestamp_ms = timestamp_ms;
    record.species_index = static_cast<uint16_t>(cls);
    record.probability_q1e4 = ProbabilityToQ1e4(result.log_detection_scores[i]);
    SdLogAppendRecord(record);
#if BIRDNET_ENABLE_FIELD_MONITORING
    g_field.counts.Add(cls);
#endif
    // This CSV entry is now accepted into retained storage. SD failures retain
    // the queued row and use the existing bounded retry/drop policy.
    g_sd_log_state.last_detection_end_samples[cls] = g_audio_ring_total_samples;
  }

  if (result.log_detection_overflow && cfg::kEnableStatusLogs) {
    birdnet_log::Write("[birdnet] sd log window clipped at %d detections\r\n",
            cfg::kSdLogMaxDetectionsPerWindow);
  }
}

void SdLogCaptureEnvironment(const Sht41Reading& sht41, uint64_t timestamp_ms) {
  if constexpr (!cfg::kEnableSdDetectionLogging) {
    (void)sht41;
    (void)timestamp_ms;
    return;
  }

  g_sd_log_state.latest_env_log.timestamp_ms = timestamp_ms;
  g_sd_log_state.latest_env_log.ok = sht41.ok;
  g_sd_log_state.latest_env_log.temp_c_q1e2 = sht41.temp_c_q1e2;
  g_sd_log_state.latest_env_log.rh_q1e2 = sht41.rh_q1e2;
  g_sd_log_state.has_latest_env_log = 1;
}

bool SdLogWriteAll(FIL* file, const void* data, UINT len) {
  UINT written = 0;
  const FRESULT res = f_write(file, data, len, &written);
  return res == FR_OK && written == len;
}

bool SdLogWriteCsvQuoted(FIL* file, const char* text) {
  return birdnet_storage::WriteQuotedLabel(text, [file](const char* bytes, size_t n) {
    return SdLogWriteAll(file, bytes, static_cast<UINT>(n));
  });
}

bool SdLogOpenAppend(FIL* file, const char* filename, const char* header) {
  if constexpr (cfg::kEnableLedDiagnostics) birdnet_log::Write("[bntrace] append open %s\r\n", filename);
  FRESULT res = f_open(file, filename, FA_OPEN_ALWAYS | FA_WRITE);
  if (res != FR_OK) {
    return false;
  }

  const FSIZE_t file_size = f_size(file);
  if constexpr (cfg::kEnableLedDiagnostics) birdnet_log::Write("[bntrace] append seek %lu\r\n", (unsigned long)file_size);
  if (f_lseek(file, file_size) != FR_OK) {
    f_close(file);
    return false;
  }
  if (file_size == 0 && header) {
    if (!SdLogWriteAll(file, header, static_cast<UINT>(std::strlen(header)))) {
      f_close(file);
      return false;
    }
  }
  return true;
}

void FormatSignedQ1e2(char* out, size_t out_len, int value_q1e2) {
  const bool neg = value_q1e2 < 0;
  const unsigned abs_value = static_cast<unsigned>(neg ? -value_q1e2 : value_q1e2);
  birdnet_format::Snprintf(out,
                out_len,
                "%s%u.%02u",
                neg ? "-" : "",
                abs_value / 100u,
                abs_value % 100u);
}

void FormatUnsignedQ1e2(char* out, size_t out_len, int value_q1e2) {
  const unsigned value = value_q1e2 <= 0 ? 0u : static_cast<unsigned>(value_q1e2);
  birdnet_format::Snprintf(out, out_len, "%u.%02u", value / 100u, value % 100u);
}

bool FormatTimestampSeconds(char* out, size_t out_len, uint64_t timestamp_ms) {
  const int n = birdnet_format::Snprintf(out,
                              out_len,
                              "%llu.%03u",
                              static_cast<unsigned long long>(timestamp_ms / 1000u),
                              static_cast<unsigned int>(timestamp_ms % 1000u));
  return n > 0 && n < static_cast<int>(out_len);
}

bool ParseAudioSequence(const char* filename, uint32_t* sequence) {
  return filename && birdnet_storage::ParseSequence(filename, std::strlen(filename), sequence);
}

bool SdAudioLoadNextSequence() {
  uint32_t next_sequence = 0;
  FIL map{};
  FRESULT res = f_open(&map, cfg::kSdAudioMapFileName, FA_READ);
  if (res == FR_NO_FILE) {
    g_sd_log_state.next_audio_sequence = 0;
    g_sd_log_state.audio_sequence_initialized = 1;
    return true;
  }
  if (res != FR_OK) {
    return false;
  }

  constexpr UINT kTailBytes = 512;
  char tail[kTailBytes];
  const FSIZE_t size = f_size(&map);
  const FSIZE_t offset = size > kTailBytes ? size - kTailBytes : 0;
  UINT bytes_read = 0;
  if (f_lseek(&map, offset) != FR_OK ||
      f_read(&map, tail, static_cast<UINT>(size - offset), &bytes_read) != FR_OK) {
    f_close(&map);
    return false;
  }
  if (f_close(&map) != FR_OK) {
    return false;
  }

  // Search backward for the filename at the start of the newest complete map
  // row. Collision retries below also cover an audio file whose map row was
  // interrupted by power loss.
  if (bytes_read >= 13) {
    for (int i = static_cast<int>(bytes_read) - 13; i >= 0; --i) {
      const bool at_line_start =
          (offset == 0 && i == 0) || (i > 0 && tail[i - 1] == '\n');
      uint32_t sequence = 0;
      const char* comma = at_line_start
          ? static_cast<const char*>(std::memchr(tail + i, ',', bytes_read - i)) : nullptr;
      const bool complete = comma && std::memchr(comma, '\n', tail + bytes_read - comma);
      if (complete && birdnet_storage::ParseSequence(tail + i, comma - (tail + i), &sequence)) {
        next_sequence = sequence + 1u;
        break;
      }
    }
  }

  g_sd_log_state.next_audio_sequence = next_sequence;
  g_sd_log_state.audio_sequence_initialized = 1;
  return true;
}

bool SdAudioOpenNew(FIL* file,
                    char* filename,
                    size_t filename_len,
                    const char* extension,
                    const char* label) {
  if (!file || !filename || filename_len < birdnet_storage::kAudioPathBytes || !extension ||
      std::strlen(extension) != 3) {
    return false;
  }

  if (!g_sd_log_state.audio_sequence_initialized &&
      !SdAudioLoadNextSequence()) {
    return false;
  }

  for (int attempt = 0; attempt < 1024; ++attempt) {
    if (!g_sd_log_state.audio_sequence_initialized &&
        !SdAudioLoadNextSequence()) {
      return false;
    }
    if (g_sd_log_state.next_audio_sequence > 9999999u) {
      return false;
    }

    const uint32_t sequence = g_sd_log_state.next_audio_sequence;
    if (!birdnet_storage::AudioPath(sequence, extension, filename, filename_len, label)) {
      return false;
    }
    FRESULT res = f_open(file, filename, FA_CREATE_NEW | FA_WRITE);
    if (res == FR_NO_PATH) {
      // Normally one lookup suffices. Only touch parent directories when a new
      // species/bucket is encountered; no repeated mkdir calls per clip.
      char directory[birdnet_storage::kAudioPathBytes];
      std::strcpy(directory, filename);
      for (char* p = directory; *p; ++p) {
        if (*p != '/') continue;
        *p = '\0';
        const FRESULT made = f_mkdir(directory);
        *p = '/';
        if (made != FR_OK && made != FR_EXIST) return false;
      }
      res = f_open(file, filename, FA_CREATE_NEW | FA_WRITE);
    }
    if (res == FR_OK) {
      g_sd_log_state.next_audio_sequence = sequence + 1u;
      return true;
    }
    if (res != FR_EXIST) {
      return false;
    }
    g_sd_log_state.next_audio_sequence = sequence + 1u;
  }
  return false;
}

// One file writer at a time; avoid expanding the Opus call stack.
alignas(32) uint8_t g_sd_audio_buffer[BIRDNET_SD_AUDIO_BUFFER_BYTES]
    __attribute__((section(".scratch_dtcm.sd")));
using SdAudioWriteContext = birdnet_storage::BufferedWriter;
bool SdFileWrite(void* context, const uint8_t* data, size_t size) {
  return SdLogWriteAll(static_cast<FIL*>(context), data, static_cast<UINT>(size));
}
bool SdFileSeek(void* context, uint32_t offset) {
  return f_lseek(static_cast<FIL*>(context), offset) == FR_OK;
}
bool SdAudioFlush(SdAudioWriteContext* writer) { return writer->Flush(); }
bool SdAudioWrite(void* context, const uint8_t* data, size_t size) {
  return context && static_cast<SdAudioWriteContext*>(context)->Write(data, size);
}
bool SdAudioSeek(void* context, uint32_t offset) {
  return context && static_cast<SdAudioWriteContext*>(context)->Seek(offset);
}

struct SdAudioEncodedStats {
  const char* codec;
  uint32_t sample_rate_hz;
  uint32_t samples;
  uint32_t file_bytes;
};

bool SdAudioAppendMap(const char* filename,
                      uint64_t timestamp_ms,
                      const InferenceResult& result,
                      const SdAudioEncodedStats& stats) {
  FIL map{};
  static constexpr char kHeader[] =
      "filename,audio_end_timestamp_s,species_index,species_name,probability,"
      "codec,sample_rate_hz,samples,file_bytes\r\n";
  if (!SdLogOpenAppend(&map, cfg::kSdAudioMapFileName, kHeader)) {
    return false;
  }

  char timestamp_s[32];
  char line[192];
  bool ok = FormatTimestampSeconds(timestamp_s, sizeof(timestamp_s), timestamp_ms);
  const uint16_t probability_q1e4 = ProbabilityToQ1e4(result.pred_score);
  if (ok) {
    const int n = birdnet_format::Snprintf(line,
                                sizeof(line),
                                "%s,%s,%d,",
                                filename,
                                timestamp_s,
                                result.pred_class);
    ok = n > 0 && n < static_cast<int>(sizeof(line)) &&
         SdLogWriteAll(&map, line, static_cast<UINT>(n));
  }
  if (ok) {
    ok = SdLogWriteCsvQuoted(
        &map, result.pred_class < 0 ? "Ambient_hourly" : birdnet_labels::LabelForIndex(result.pred_class));
  }
  if (ok) {
    const int n = birdnet_format::Snprintf(
        line,
        sizeof(line),
        ",%u.%04u,%s,%lu,%lu,%lu\r\n",
        static_cast<unsigned int>(probability_q1e4 / 10000u),
        static_cast<unsigned int>(probability_q1e4 % 10000u),
        stats.codec,
        static_cast<unsigned long>(stats.sample_rate_hz),
        static_cast<unsigned long>(stats.samples),
        static_cast<unsigned long>(stats.file_bytes));
    ok = n > 0 && n < static_cast<int>(sizeof(line)) &&
         SdLogWriteAll(&map, line, static_cast<UINT>(n));
  }
  if (ok && f_sync(&map) != FR_OK) {
    ok = false;
  }
  if (f_close(&map) != FR_OK) {
    ok = false;
  }
  return ok;
}

#if BIRDNET_ENABLE_FIELD_MONITORING
void FieldCheckSpace() {
  // Use the mounted FAT32 FSInfo count. f_getfree() would scan the entire FAT
  // when this is unknown, potentially blocking capture/watchdogs for seconds.
  // Fail closed for audio on unknown space; repair/reformat the card on a host.
  const auto& fs = g_sd_log_fs;
  g_field.space_known = fs.n_fatent >= 2 && fs.free_clst <= fs.n_fatent - 2;
  g_field.free_bytes = g_field.space_known ? uint64_t(fs.free_clst) * fs.csize * 512u : 0;
  g_field.audio_stopped = !g_field.space_known ||
      g_field.free_bytes < birdnet_field::kAudioReserve + birdnet_field::kClipBudget;
  g_field.log_stopped = g_field.space_known && g_field.free_bytes < birdnet_field::kLogReserve;
}

bool FieldMetadata() {
  if (!g_sd_log_state.session_id || g_sd_log_state.session_id > 9999999) return false;
  if (g_field.metadata_session == g_sd_log_state.session_id) return true;
  char path[16], line[320];
  birdnet_format::Snprintf(path, sizeof(path), "M%07lu.TXT", static_cast<unsigned long>(g_sd_log_state.session_id));
  FIL file{};
  if (f_open(&file, path, FA_CREATE_NEW | FA_WRITE) != FR_OK) return false;
  int n = birdnet_format::Snprintf(line, sizeof(line),
      "format=birdnet-field-1\r\nsession=%lu\r\ndeployment=%s\r\n"
      "clock=capture_relative_nominal_not_UTC\r\nlocation=see_label_manifest\r\n"
      "sample_rate=%d\r\nwindow_samples=%d\r\nstep_samples=%d\r\nthreshold_q1e4=%d\r\n",
      static_cast<unsigned long>(g_sd_log_state.session_id), birdnet_contract::kId,
      cfg::kSampleRateHz, cfg::kWindowSamples, cfg::kInferenceStepSamples, cfg::kSdLogThresholdQ1e4);
  bool ok = n > 0 && n < int(sizeof(line)) && SdLogWriteAll(&file, line, UINT(n));
  n = birdnet_format::Snprintf(line, sizeof(line), "model_sha256=%s\r\nlabels_sha256=%s\r\n"
      "pdm_bit_range_shift=5\r\npdm_dc_removal=6\r\nhuman_vocal_threshold_q1e4=%d\r\n",
      birdnet_contract::kModelSha256, birdnet_contract::kLabelsSha256, BIRDNET_HUMAN_VOCAL_THRESHOLD_Q1E4);
  ok = ok && n > 0 && n < int(sizeof(line)) && SdLogWriteAll(&file, line, UINT(n));
  n = birdnet_format::Snprintf(line, sizeof(line),
      "codec=%d\r\nambient_seconds=3600\r\nhealth_seconds=300\r\n"
      "pause_mv=%d\r\nresume_mv=%d\r\naudio_reserve_mib=64\r\n"
      "full_policy=stop_audio_never_delete\r\ncounts=uint32_since_boot_top15_display\r\n"
      "ambient=map_species_minus1_no_detections_in_clip_metadata\r\n",
      cfg::kDetectionAudioCodec, BIRDNET_BATTERY_PAUSE_MV, BIRDNET_BATTERY_RESUME_MV);
  ok = ok && n > 0 && n < int(sizeof(line)) && SdLogWriteAll(&file, line, UINT(n));
  constexpr char policy[] =
      "audio_layout=AUDIO/common_name-label_hash/Dxxxxxxx/Axxxxxxx.ext\r\n"
      "audio_owner=highest_scoring_nonoverlapping_detection\r\n"
      "detection_audio_triggers=birds_and_nonhuman_mammals_only\r\n"
      "human_vocal=CSV_only_veto_all_audio_at_human_vocal_threshold\r\n"
      "ambient_requires=successful_inference_without_human_vocal\r\nlabels:\r\n";
  ok = ok && SdLogWriteAll(&file, policy, sizeof(policy) - 1);
  for (int i = 0; ok && i < cfg::kNumClasses; ++i) {
    char folder[birdnet_storage::kSpeciesDirectoryBytes];
    const char* label = birdnet_labels::LabelForIndex(i);
    if (!birdnet_storage::SpeciesDirectory(label, folder, sizeof(folder))) { ok = false; break; }
    n = birdnet_format::Snprintf(line, sizeof(line), "%d=%s\r\nfolder.%d=AUDIO/%s\r\n", i, label, i, folder);
    ok = n > 0 && n < int(sizeof(line)) && SdLogWriteAll(&file, line, UINT(n));
  }
  ok = ok && SdLogWriteAll(&file, "complete=1\r\n", 12) && f_sync(&file) == FR_OK;
  if (f_close(&file) != FR_OK) ok = false;
  if (ok) g_field.metadata_session = g_sd_log_state.session_id;
  return ok;
}

bool FieldHealth(uint64_t now) {
  const uint64_t period = now / birdnet_field::kHealthMs;
  if (!g_field.dirty && period <= g_field.health_period) return true;
  if (!SdLogMount(now)) return false;
  FieldCheckSpace();
  // Leave the final MiB untouched rather than repeatedly writing a full card.
  if (g_field.log_stopped) { g_field.health_period = period; g_field.dirty = false; return true; }
  if (!SdLogEnsureSession() || !FieldMetadata()) { SdLogFailed(now); return false; }
  FIL file{};
  constexpr char header[] = "session,elapsed_s,paused,battery_mv,battery_pct_q100,min_mv,"
      "monitored_s,inferences,infer_errors,skipped,rejected,csv_dropped,sd_errors,"
      "free_kib,audio_stopped,clips,ambient,ambient_missed,audio_suppressed,peak,clipped,paused_s\r\n";
  if (!SdLogOpenAppend(&file, "HEALTH.CSV", header)) { SdLogFailed(now); return false; }
  char line[384];
  const uint64_t pause_ms = g_field.paused_ms +
      (g_field.battery.paused ? now - g_field.pause_started_ms : 0);
  const int n = birdnet_format::Snprintf(line, sizeof(line),
      "%lu,%llu,%u,%d,%d,%d,%llu,%lu,%lu,%lu,%lu,%lu,%lu,%lld,%u,%lu,%lu,%lu,%lu,%lu,%lu,%llu\r\n",
      static_cast<unsigned long>(g_sd_log_state.session_id), static_cast<unsigned long long>(now / 1000),
      g_field.battery.paused, g_battery_reading.ok ? g_battery_reading.millivolts : -1,
      g_battery_reading.ok ? g_battery_reading.percent_q1e2 : -1, g_field.battery_min_mv,
      static_cast<unsigned long long>(g_field.monitored_samples / cfg::kSampleRateHz),
      static_cast<unsigned long>(g_field.inferences), static_cast<unsigned long>(g_field.inference_errors),
      static_cast<unsigned long>(g_retained_runtime.skipped_windows),
      static_cast<unsigned long>(g_retained_runtime.rejected_windows),
      static_cast<unsigned long>(g_sd_log_state.dropped_records + g_sd_log_state.env_dropped_records),
      static_cast<unsigned long>(g_field.sd_errors),
      g_field.space_known ? static_cast<long long>(g_field.free_bytes / 1024) : -1ll,
      g_field.audio_stopped, static_cast<unsigned long>(g_field.audio_saved),
      static_cast<unsigned long>(g_field.ambient_saved), static_cast<unsigned long>(g_field.ambient_missed),
      static_cast<unsigned long>(g_field.audio_suppressed), static_cast<unsigned long>(g_field.audio_peak),
      static_cast<unsigned long>(g_field.audio_clipped), static_cast<unsigned long long>(pause_ms / 1000));
  bool ok = n > 0 && n < int(sizeof(line)) && SdLogWriteAll(&file, line, UINT(n)) && f_sync(&file) == FR_OK;
  if (f_close(&file) != FR_OK) ok = false;
  if (ok) { g_field.health_period = period; g_field.dirty = false; g_field.battery_min_mv = 0; }
  else SdLogFailed(now);
  return ok;
}
#endif

bool SdSaveDetectionAudio(const InferenceResult& result,
                          uint64_t timestamp_ms,
                          const int16_t* pcm_ring,
                          size_t ring_size,
                          size_t chronological_start,
                          size_t chronological_samples) {
  if constexpr (!cfg::kEnableDetectionAudio) {
    (void)result;
    (void)timestamp_ms;
    (void)pcm_ring;
    (void)ring_size;
    (void)chronological_start;
    (void)chronological_samples;
    return true;
  }
  if (result.human_vocal_detected) {
#if BIRDNET_ENABLE_FIELD_MONITORING
    ++g_field.audio_suppressed;
#endif
    return false; // No SD mount or encoder work, even for a mixed-species clip.
  }
  if (!result.detected) {
    return true;
  }

  if constexpr (cfg::kEnableLedDiagnostics) {
    birdnet_log::Write("[birdnet] detection audio stage=begin\r\n");
  }
  if (timestamp_ms < g_sd_log_state.next_retry_ms) return false;
#if BIRDNET_ENABLE_FIELD_MONITORING
  if (g_field.battery.paused || (g_field.audio_stopped && timestamp_ms < g_field.audio_retry_ms)) {
    ++g_field.audio_suppressed;
    return false;
  }
#endif
  bool ok = SdLogMount(timestamp_ms);
#if BIRDNET_ENABLE_FIELD_MONITORING
  if (ok) {
    FieldCheckSpace();
    if (g_field.audio_stopped || g_field.battery.paused) {
      ++g_field.audio_suppressed;
      g_field.audio_retry_ms = timestamp_ms + birdnet_field::kHealthMs;
      return false;
    }
  }
#endif
  ok = ok && SdLogEnsureSession();
#if BIRDNET_ENABLE_FIELD_MONITORING
  if (ok) ok = FieldMetadata();
#endif
  if constexpr (cfg::kEnableLedDiagnostics) {
    birdnet_log::Write("[birdnet] detection audio stage=spi-ready ok=%d\r\n", ok ? 1 : 0);
  }
  FRESULT mount_res = FR_NOT_READY;
  if (ok) {
    if constexpr (cfg::kEnableLedDiagnostics) {
      birdnet_log::Write("[birdnet] detection audio stage=mount-begin\r\n");
    }
    mount_res = FR_OK;
    if constexpr (cfg::kEnableLedDiagnostics) {
      birdnet_log::Write("[birdnet] detection audio stage=mount-end res=%d\r\n",
              static_cast<int>(mount_res));
    }
  }

  FIL audio{};
  char filename[birdnet_storage::kAudioPathBytes] = {};
  bool audio_open = false;
  SdAudioEncodedStats stats{};
  uint32_t encode_write_ms = 0;
  if (ok) {
    if constexpr (cfg::kEnableLedDiagnostics) {
      birdnet_log::Write("[birdnet] detection audio stage=open-begin next=%lu initialized=%d\r\n",
              static_cast<unsigned long>(g_sd_log_state.next_audio_sequence),
              g_sd_log_state.audio_sequence_initialized ? 1 : 0);
    }
    audio_open = SdAudioOpenNew(
        &audio,
        filename,
        sizeof(filename),
        cfg::kDetectionAudioCodec == 2 ? "OGG" :
        cfg::kDetectionAudioCodec == 1 ? "BRC" : "WAV",
        result.pred_class < 0 ? nullptr : birdnet_labels::LabelForIndex(result.pred_class));
    ok = audio_open;
    if constexpr (cfg::kEnableLedDiagnostics) {
      birdnet_log::Write("[birdnet] detection audio stage=open-end ok=%d file=%s\r\n",
              ok ? 1 : 0,
              filename[0] ? filename : "<none>");
    }
  }
  if (ok) {
    SdAudioWriteContext writer{&audio, SdFileWrite, SdFileSeek, g_sd_audio_buffer,
                              sizeof(g_sd_audio_buffer),
                              cfg::kDetectionAudioCodec == 1 ? 32u :
                              cfg::kDetectionAudioCodec == 0 ? 60u : 0u};
    birdnet_clip::Metadata clip_metadata{};
    clip_metadata.session = g_sd_log_state.session_id;
    clip_metadata.sequence = g_sd_log_state.next_audio_sequence - 1u;
    clip_metadata.capture_end_ms = timestamp_ms;
    clip_metadata.threshold_q1e4 = BIRDNET_SD_LOG_THRESHOLD_Q1E4;
    clip_metadata.deployment = birdnet_contract::kId;
    clip_metadata.count = static_cast<uint16_t>(std::min(
        std::max(result.log_detection_count, 0), static_cast<int>(birdnet_clip::kMaxDetections)));
    clip_metadata.overflow = result.log_detection_overflow ||
                            result.log_detection_count > static_cast<int>(clip_metadata.count);
    for (size_t i = 0; i < clip_metadata.count; ++i) {
      clip_metadata.species[i] = result.log_detection_indices[i];
      clip_metadata.probability[i] = ProbabilityToQ1e4(result.log_detection_scores[i]);
    }
    const uint64_t encode_write_start_ms = PlatformMillis();
    if constexpr (cfg::kEnableLedDiagnostics) {
      birdnet_log::Write("[birdnet] detection audio stage=encode-begin\r\n");
    }
    if constexpr (cfg::kDetectionAudioCodec == 1) {
      birdnet_rice_codec::EncodeStats rice_stats{};
      ok = birdnet_rice_codec::EncodeBrcFrom48kRing(
          pcm_ring,
          ring_size,
          chronological_start,
          chronological_samples,
          static_cast<uint16_t>(cfg::kDetectionAudioRiceQuantStep),
          SdAudioWrite,
          &writer,
          &rice_stats,
          SdAudioSeek);
      stats = {
          "brc2_rice",
          birdnet_rice_codec::kOutputSampleRateHz,
          rice_stats.output_samples,
          rice_stats.file_bytes,
      };
    } else if constexpr (cfg::kDetectionAudioCodec == 2) {
      birdnet_opus_codec::EncodeStats opus_stats{};
      const birdnet_opus_codec::Metadata metadata{
          timestamp_ms, clip_metadata.sequence, birdnet_contract::kId, &clip_metadata};
      ok = birdnet_codec_loader::Prepare() && birdnet_opus_codec::EncodeOggFrom48kRing(
          pcm_ring, ring_size, chronological_start, chronological_samples,
          metadata, SdAudioWrite, &writer, &opus_stats);
      stats = {"ogg_opus", birdnet_opus_codec::kSampleRateHz,
               opus_stats.samples, opus_stats.file_bytes};
      if constexpr (cfg::kEnableLedDiagnostics) {
        birdnet_log::Write("[birdnet] opus error=%d state_bytes=%lu packets=%lu\r\n",
                opus_stats.error,
                static_cast<unsigned long>(opus_stats.encoder_state_bytes),
                static_cast<unsigned long>(opus_stats.packets));
      }
    } else {
      birdnet_ima_adpcm::EncodeStats ima_stats{};
      ok = birdnet_ima_adpcm::EncodeWavFrom48kRing(
          pcm_ring,
          ring_size,
          chronological_start,
          chronological_samples,
          SdAudioWrite,
          &writer,
          &ima_stats);
      stats = {
          "ima_adpcm",
          birdnet_ima_adpcm::kOutputSampleRateHz,
          ima_stats.output_samples,
          ima_stats.file_bytes,
      };
    }
    if constexpr (cfg::kDetectionAudioCodec != 2) {
      if (ok) {
        ok = writer.FinishMetadata(cfg::kDetectionAudioCodec == 1, stats.file_bytes, clip_metadata);
        if (ok) stats.file_bytes += birdnet_clip::kBytes + (cfg::kDetectionAudioCodec == 0 ? 8u : 0u);
      }
    }
    ok = ok && SdAudioFlush(&writer);
    encode_write_ms =
        static_cast<uint32_t>(PlatformMillis() - encode_write_start_ms);
    if constexpr (cfg::kEnableLedDiagnostics) {
      birdnet_log::Write("[birdnet] detection audio stage=encode-end ok=%d bytes=%lu ms=%lu\r\n",
              ok ? 1 : 0,
              static_cast<unsigned long>(stats.file_bytes),
              static_cast<unsigned long>(encode_write_ms));
    }
  }
  if constexpr (cfg::kEnableLedDiagnostics) {
    birdnet_log::Write("[birdnet] detection audio stage=sync-begin ok=%d\r\n", ok ? 1 : 0);
  }
  if (ok && f_sync(&audio) != FR_OK) {
    ok = false;
  }
  if constexpr (cfg::kEnableLedDiagnostics) {
    birdnet_log::Write("[birdnet] detection audio stage=sync-end ok=%d\r\n", ok ? 1 : 0);
  }
  if (audio_open && f_close(&audio) != FR_OK) {
    ok = false;
  }
  if constexpr (cfg::kEnableLedDiagnostics) {
    birdnet_log::Write("[birdnet] detection audio stage=close-end ok=%d\r\n", ok ? 1 : 0);
  }
  if (ok) {
    if constexpr (cfg::kEnableLedDiagnostics) {
      birdnet_log::Write("[birdnet] detection audio stage=map-begin\r\n");
    }
    ok = SdAudioAppendMap(filename, timestamp_ms, result, stats);
    if constexpr (cfg::kEnableLedDiagnostics) {
      birdnet_log::Write("[birdnet] detection audio stage=map-end ok=%d\r\n", ok ? 1 : 0);
    }
  }
  // Keep the mounted session for a due CSV flush in this same wake.
  if constexpr (cfg::kEnableLedDiagnostics) {
    birdnet_log::Write("[birdnet] detection audio stage=session-kept-for-csv\r\n");
  }

  if (!ok) {
    if (timestamp_ms >= g_sd_log_state.next_retry_ms) SdLogFailed(timestamp_ms);
    if (!g_sd_log_state.audio_error_logged) {
      birdnet_log::Write("[birdnet] detection audio save failed mount=%d file=%s\r\n",
              static_cast<int>(mount_res),
              filename[0] ? filename : "<none>");
      g_sd_log_state.audio_error_logged = true;
    }
    return false;
  }

  if (cfg::kEnableStatusLogs) {
    birdnet_log::Write("[birdnet] detection audio saved file=%s bytes=%lu encode_write_ms=%lu\r\n",
            filename,
            static_cast<unsigned long>(stats.file_bytes),
            static_cast<unsigned long>(encode_write_ms));
  }
  g_sd_log_state.audio_error_logged = false;
#if BIRDNET_ENABLE_FIELD_MONITORING
  ++g_field.audio_saved;
#endif
  g_sd_log_state.retry_delay_ms = 0;
  return true;
}

bool SdLogAppendDetectionsFile() {
  if (g_sd_log_state.log_record_count <= 0) {
    return true;
  }

  FIL file{};
  static constexpr char kHeader[] = "timestamp_s,species_index,species_name,probability\r\n";
  if (!SdLogOpenAppend(&file, cfg::kSdLogFileName, kHeader)) {
    return false;
  }

  bool ok = true;
  char line[64];
  char timestamp_s[32];
  for (int i = 0; ok && i < g_sd_log_state.log_record_count; ++i) {
    const DetectionLogRecord& record = g_sd_log_state.log_records[i];
    if constexpr (cfg::kEnableLedDiagnostics) birdnet_log::Write("[bntrace] row %d/%d class=%u\r\n", i,
        int(g_sd_log_state.log_record_count), unsigned(record.species_index));
    const unsigned prob_whole = record.probability_q1e4 / 10000u;
    const unsigned prob_frac = record.probability_q1e4 % 10000u;
    if (!FormatTimestampSeconds(timestamp_s, sizeof(timestamp_s), record.timestamp_ms)) {
      ok = false;
      break;
    }
    const int n = birdnet_format::Snprintf(line,
                                sizeof(line),
                                "%s,%u,",
                                timestamp_s,
                                static_cast<unsigned int>(record.species_index));
    if (n <= 0 || n >= static_cast<int>(sizeof(line))) {
      ok = false;
      break;
    }
    ok = SdLogWriteAll(&file, line, static_cast<UINT>(n));
    if (!ok) {
      break;
    }
    ok = SdLogWriteCsvQuoted(&file, birdnet_labels::LabelForIndex(record.species_index));
    if constexpr (cfg::kEnableLedDiagnostics) birdnet_log::Write("[bntrace] quoted ok=%d\r\n", int(ok));
    if (!ok) {
      break;
    }
    const int suffix_len = birdnet_format::Snprintf(line,
                                         sizeof(line),
                                         ",%u.%04u\r\n",
                                         prob_whole,
                                         prob_frac);
    if (suffix_len <= 0 || suffix_len >= static_cast<int>(sizeof(line))) {
      ok = false;
      break;
    }
    ok = SdLogWriteAll(&file, line, static_cast<UINT>(suffix_len));
  }

  if (ok && f_sync(&file) != FR_OK) {
    ok = false;
  }
  if (f_close(&file) != FR_OK) ok = false;
  return ok;
}

bool SdLogAppendEnvFile() {
  if (g_sd_log_state.env_log_record_count <= 0) {
    return true;
  }

  FIL file{};
  static constexpr char kHeader[] = "timestamp_s,temp_c,rh_percent,ok\r\n";
  if (!SdLogOpenAppend(&file, cfg::kSdEnvLogFileName, kHeader)) {
    return false;
  }

  bool ok = true;
  char line[80];
  char timestamp_s[32];
  char temp[16];
  char rh[16];
  for (int i = 0; ok && i < g_sd_log_state.env_log_record_count; ++i) {
    const EnvLogRecord& record = g_sd_log_state.env_log_records[i];
    if (!FormatTimestampSeconds(timestamp_s, sizeof(timestamp_s), record.timestamp_ms)) {
      ok = false;
      break;
    }
    int n = 0;
    if (record.ok) {
      FormatSignedQ1e2(temp, sizeof(temp), record.temp_c_q1e2);
      FormatUnsignedQ1e2(rh, sizeof(rh), record.rh_q1e2);
      n = birdnet_format::Snprintf(line,
                        sizeof(line),
                        "%s,%s,%s,1\r\n",
                        timestamp_s,
                        temp,
                        rh);
    } else {
      n = birdnet_format::Snprintf(line,
                        sizeof(line),
                        "%s,,,0\r\n",
                        timestamp_s);
    }
    if (n <= 0 || n >= static_cast<int>(sizeof(line))) {
      ok = false;
      break;
    }
    ok = SdLogWriteAll(&file, line, static_cast<UINT>(n));
  }

  if (ok && f_sync(&file) != FR_OK) {
    ok = false;
  }
  if (f_close(&file) != FR_OK) ok = false;
  return ok;
}

void SdLogQueueMinuteEnvironment(uint64_t now_ms, uint64_t period) {
  if (!g_sd_log_state.has_latest_env_log ||
      period == 0 ||
      period <= g_sd_log_state.last_env_queued_period) {
    return;
  }

  EnvLogRecord record = g_sd_log_state.latest_env_log;
  record.timestamp_ms = now_ms;
  SdLogAppendEnvRecord(record);
  g_sd_log_state.last_env_queued_period = period;
}

bool SdLogFlush(uint64_t now_ms, bool force) {
  if constexpr (!cfg::kEnableSdDetectionLogging) {
    (void)now_ms;
    (void)force;
    return true;
  }

  const uint64_t interval_ms = static_cast<uint64_t>(cfg::kSdLogFlushIntervalMs);
  const uint64_t period = now_ms / interval_ms;
  if (!force && (period == 0 || period <= g_sd_log_state.last_flush_period)) {
    return true;
  }

  SdLogQueueMinuteEnvironment(now_ms, period);
  if (g_sd_log_state.log_record_count <= 0 && g_sd_log_state.env_log_record_count <= 0) {
    g_sd_log_state.last_flush_period = period;
    return true;
  }

  if (!SdLogMount(now_ms)) {
    SdLogPowerDown();
    return false;
  }

  const int detection_count = g_sd_log_state.log_record_count;
  const int env_count = g_sd_log_state.env_log_record_count;
  if constexpr (cfg::kEnableLedDiagnostics) birdnet_log::Write("[bntrace] flush mounted det=%d env=%d\r\n", detection_count, env_count);
#if BIRDNET_ENABLE_FIELD_MONITORING
  FieldCheckSpace();
  if (g_field.log_stopped) {
    g_sd_log_state.dropped_records += detection_count;
    g_sd_log_state.env_dropped_records += env_count;
    g_sd_log_state.log_record_count = g_sd_log_state.env_log_record_count = 0;
    g_sd_log_state.last_flush_period = period;
    return true;
  }
  if (!SdLogEnsureSession() || !FieldMetadata()) { SdLogFailed(now_ms); return false; }
#endif
  const bool detections_ok = SdLogAppendDetectionsFile();
  if constexpr (cfg::kEnableLedDiagnostics) birdnet_log::Write("[bntrace] detection CSV done ok=%d\r\n", int(detections_ok));
  const bool env_ok = SdLogAppendEnvFile();

  if (detections_ok) {
    g_sd_log_state.log_record_count = 0;
  }
  if (env_ok) {
    g_sd_log_state.env_log_record_count = 0;
  }

  if (!detections_ok || !env_ok) {
    SdLogFailed(now_ms);
    if (!g_sd_log_state.log_error_logged) {
      birdnet_log::Write("[birdnet] sd log write failed det_buffered=%d env_buffered=%d\r\n",
              detection_count,
              env_count);
      g_sd_log_state.log_error_logged = true;
    }
    return false;
  }

  g_sd_log_state.last_flush_period = period;
  g_sd_log_state.retry_delay_ms = 0;
  if (cfg::kEnableStatusLogs) {
    birdnet_log::Write("[birdnet] sd log flushed det=%d env=%d det_dropped=%lu env_dropped=%lu\r\n",
            detection_count,
            env_count,
            static_cast<unsigned long>(g_sd_log_state.dropped_records),
            static_cast<unsigned long>(g_sd_log_state.env_dropped_records));
  }
  g_sd_log_state.log_error_logged = false;
  return true;
}
#else
uint64_t AudioEndTimestampMs() {
  return (g_audio_ring_total_samples * 1000ull) / static_cast<uint64_t>(cfg::kSampleRateHz);
}

void SdLogCaptureDetections(const InferenceResult&, uint64_t) {}

void SdLogCaptureEnvironment(const Sht41Reading&, uint64_t) {}

bool SdSaveDetectionAudio(const InferenceResult&,
                          uint64_t,
                          const int16_t*,
                          size_t,
                          size_t,
                          size_t) {
  return true;
}

bool SdLogFlush(uint64_t, bool) {
  return true;
}
void SdLogPowerDown() {}
#endif

