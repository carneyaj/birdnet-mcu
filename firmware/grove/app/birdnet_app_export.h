// Private application implementation. Included inside birdnet_we2.cpp
// after SDK headers; not a standalone public header.
#pragma once
#if BIRDNET_ENABLE_SERIAL_FILES
static_assert(cfg::kUseDualCoreFixedCadence && cfg::kUseAudioOnlyPrerolling &&
              BIRDNET_ENABLE_SD_DETECTION_LOGGING && cfg::kMuteVendorXprintf,
              "Serial export needs fixed cadence, audio-only sleep, SD and muted vendor printf");
DEV_UART* g_files_uart = nullptr;
uint64_t g_files_last_service = 0;

int FileUartRead(uint8_t* data, size_t size) {
  int32_t available = 0;
  if (!g_files_uart || g_files_uart->uart_control(UART_CMD_GET_RXAVAIL, &available) != 0)
    return -1;
  if (available <= 0) return 0;
  return g_files_uart->uart_read(data, std::min(size, size_t(available)));
}
bool FileUartWrite(const uint8_t* data, size_t size) {
  return g_files_uart && g_files_uart->uart_write(data, size) == int32_t(size);
}
bool FileCaptureProgress() {
  const uint64_t now = PlatformMillis();
  if (now - g_files_last_service < 100) return true;
  g_files_last_service = now;
  uint32_t fault = 0, magic = 0, sequence = 0, after = 0, ack = 0, low = 0, high = 0;
  hx_drv_mb_get_sharedata(BN_MB_MAGIC, &magic);
  hx_drv_mb_get_sharedata(BN_MB_FAULT, &fault);
  if (magic != BN_MB_MAGIC_VALUE || fault) return false;
  hx_drv_mb_get_sharedata(BN_MB_SEQUENCE, &sequence);
  __DMB();
  hx_drv_mb_get_sharedata(BN_MB_END_LOW, &low);
  hx_drv_mb_get_sharedata(BN_MB_END_HIGH, &high);
  hx_drv_mb_get_sharedata(BN_MB_ACK, &ack);
  __DMB();
  hx_drv_mb_get_sharedata(BN_MB_SEQUENCE, &after);
  if (sequence && !(sequence & 1u) && sequence == after && sequence != ack) {
    const uint64_t end = (uint64_t(high) << 32) | low;
    if (end <= g_retained_runtime.last_capture_end) return false;
    // These descriptors are deliberately discarded, not classified. Advance
    // the capture timeline and counters honestly; never invent watchdog ACKs.
    g_retained_runtime.skipped_windows += static_cast<uint32_t>(
        (end - g_retained_runtime.last_capture_end) / cfg::kInferenceStepSamples);
    g_retained_runtime.last_capture_end = end;
    g_audio_ring_total_samples = end;
    hx_drv_mb_set_sharedata(BN_MB_ACK, sequence);
    __DMB();
    hx_drv_mb_trigger_cm55s_irq(BN_MB_ACK_EVENT);
  }
  // A stalled helper is still covered by its independent progress watchdog.
  return hx_drv_watchdog_cm55m_update(15000) == WATCHDOG_NO_ERROR;
}
bool FileMount() {
  SdLogFlush(AudioEndTimestampMs(), true);
  return SdLogMount(AudioEndTimestampMs());
}
uint32_t FileSkippedWindows() { return g_retained_runtime.skipped_windows; }
bool FileCaptureMode(uint32_t mode) {
  hx_drv_mb_set_sharedata(BN_MB_SLEEP_READY, mode);
  __DMB();
  hx_drv_mb_trigger_cm55s_irq(BN_MB_ACK_EVENT);
  const uint64_t start = PlatformMillis();
  uint32_t acknowledged = UINT32_MAX;
  while (PlatformMillis() - start < 1500) {
    hx_drv_mb_get_sharedata(BN_MB_MODE_ACK, &acknowledged);
    if (acknowledged == mode) return true;
    if (!FileCaptureProgress()) break;
  }
  return false;
}
void FileAdvertise() {
  g_files_uart = hx_drv_uart_get_dev(USE_DW_UART_0);
  if (!g_files_uart) return;
  (void)g_files_uart->uart_open(UART_BAUDRATE_921600);
  // Reopening an already-open SDK UART does not necessarily change its baud.
  if (g_files_uart->uart_control(UART_CMD_SET_BAUD, reinterpret_cast<void*>(921600u)) != 0) {
    g_files_uart = nullptr;
    return;
  }
  uint8_t discard[16];
  for (unsigned i = 0; i < 4 && FileUartRead(discard, sizeof(discard)) > 0; ++i) {}
  (void)FileUartWrite(reinterpret_cast<const uint8_t*>(birdnet_files::kBeacon),
                      sizeof(birdnet_files::kBeacon) - 1);
}
void FilePollAndClose() {
  uint8_t knock[8];
  if (FileUartRead(knock, sizeof(knock)) == sizeof(knock) &&
      !std::memcmp(knock, birdnet_files::kKnock, sizeof(knock))) {
    SetDetectionOutputs(nullptr, nullptr);
    DEV_UART* saved_console = g_console_uart;
    g_console_uart = nullptr;
    g_files_last_service = 0;
    if (!FileCaptureMode(BN_MB_MAINTENANCE)) {
      while (true) __WFI(); // Leave watchdogs armed on a broken capture handshake.
    }
    const birdnet_files::Platform platform = {
        FileUartRead, FileUartWrite, PlatformMillis, FileCaptureProgress, FileMount,
        SdLogPowerDown, FileSkippedWindows};
    birdnet_files::Run(platform, birdnet_contract::kId);
    if (!FileCaptureMode(BN_MB_ACTIVE)) {
      while (true) __WFI();
    }
    // Drain any final published snapshot. The next normal wake obtains fresh PCM.
    g_files_last_service = 0;
    (void)FileCaptureProgress();
    (void)TakeMailboxEvents();
    ResetMailboxWindowState();
    g_console_uart = saved_console;
  }
  if (g_files_uart) (void)g_files_uart->uart_control(UART_CMD_FLUSH_OUTPUT, nullptr);
  if (!cfg::kEnableConsoleUart) console_desetup(DW_UART_0_ID);
  g_files_uart = nullptr;
}
#else
void FileAdvertise() {}
void FilePollAndClose() {}
#endif

