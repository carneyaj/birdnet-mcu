#include "birdnet_format.h"
#include "birdnet_lora.h"

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstdint>
#include <cstring>

#include "config.h"

extern "C" {
#include "Driver_SPI.h"
#include "WE2_core.h"
#include "board.h"
#include "hx_drv_gpio.h"
#include "hx_drv_scu.h"
}

#if BIRDNET_ENABLE_LORA_DETECTION_TX
extern "C" ARM_DRIVER_SPI Driver_SPI0;
#endif

namespace birdnet_lora {
namespace {

#if BIRDNET_ENABLE_LORA_DETECTION_TX
constexpr uint8_t kCmdSetStandby = 0x80;
constexpr uint8_t kCmdSetPacketType = 0x8A;
constexpr uint8_t kCmdSetRfFrequency = 0x86;
constexpr uint8_t kCmdSetTxParams = 0x8E;
constexpr uint8_t kCmdSetDioIrqParams = 0x08;
constexpr uint8_t kCmdGetIrqStatus = 0x12;
constexpr uint8_t kCmdClearIrqStatus = 0x02;
constexpr uint8_t kCmdSetBufferBaseAddress = 0x8F;
constexpr uint8_t kCmdWriteBuffer = 0x0E;
constexpr uint8_t kCmdWriteRegister = 0x0D;
constexpr uint8_t kCmdSetModulationParams = 0x8B;
constexpr uint8_t kCmdSetPacketParams = 0x8C;
constexpr uint8_t kCmdSetDio2AsRfSwitchCtrl = 0x9D;
constexpr uint8_t kCmdSetDio3AsTcxoCtrl = 0x97;
constexpr uint8_t kCmdSetPaConfig = 0x95;
constexpr uint8_t kCmdSetTx = 0x83;

constexpr uint16_t kIrqTxDone = 0x0001;
constexpr uint16_t kIrqTimeout = 0x0200;
constexpr uint16_t kAllIrqs = 0x03FF;
constexpr uint32_t kSpiHz = 1000000u;
constexpr uint32_t kBusyTimeoutMs = 40;
constexpr uint32_t kTxTimeoutMs = 1500;
constexpr uint8_t kPayloadMagic0 = 'B';
constexpr uint8_t kPayloadMagic1 = 'N';
constexpr uint8_t kPayloadVersion = 1;

bool g_initialized = false;
LogFn g_log = nullptr;
uint16_t g_sequence = 0;

void MaybeLog(const char* fmt, ...) {
  if (!g_log || !fmt) {
    return;
  }
  char buf[160];
  va_list ap;
  va_start(ap, fmt);
  birdnet_format::Vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  g_log("%s", buf);
}

void DelayMs(uint32_t ms) {
  board_delay_ms(ms);
}

uint64_t Millis() {
  uint32_t tick = 0;
  uint32_t loop_cnt = 0;
  SystemGetTick(&tick, &loop_cnt);
  const uint64_t elapsed_ticks =
      static_cast<uint64_t>(loop_cnt) * static_cast<uint64_t>(SysTick_LOAD_RELOAD_Msk + 1u) +
      static_cast<uint64_t>(SysTick_LOAD_RELOAD_Msk + 1u - tick);
  return elapsed_ticks / (SystemCoreClock / 1000u);
}

void Cs(bool active) {
  hx_drv_gpio_set_out_value(GPIO16, active ? GPIO_OUT_LOW : GPIO_OUT_HIGH);
}

void RfSwitchRx(bool rx) {
  hx_drv_gpio_set_out_value(GPIO24, rx ? GPIO_OUT_HIGH : GPIO_OUT_LOW);
}

bool BusyHigh() {
  uint8_t value = 0;
  if (hx_drv_gpio_get_in_value(GPIO22, &value) != GPIO_NO_ERROR) {
    return false;
  }
  return value != 0;
}

bool WaitBusyLow(uint32_t timeout_ms) {
  const uint64_t deadline = Millis() + timeout_ms;
  while (BusyHigh()) {
    if (Millis() >= deadline) {
      return false;
    }
  }
  return true;
}

bool WaitSpiIdle(uint32_t timeout_ms) {
  const uint64_t deadline = Millis() + timeout_ms;
  while (Driver_SPI0.GetStatus().busy) {
    if (Millis() >= deadline) {
      return false;
    }
  }
  return true;
}

bool SpiTransfer(const uint8_t* tx, uint8_t* rx, uint32_t len) {
  if (!WaitBusyLow(kBusyTimeoutMs)) {
    return false;
  }
  Cs(true);
  const int32_t ret = Driver_SPI0.Transfer(tx, rx, len);
  const bool ok = ret == ARM_DRIVER_OK && WaitSpiIdle(kBusyTimeoutMs);
  Cs(false);
  return ok && WaitBusyLow(kBusyTimeoutMs);
}

bool WriteCommand(uint8_t op, const uint8_t* data, uint32_t len) {
  uint8_t tx[16];
  if (len + 1u > sizeof(tx)) {
    return false;
  }
  tx[0] = op;
  if (len > 0) {
    std::memcpy(&tx[1], data, len);
  }
  uint8_t rx[sizeof(tx)]{};
  return SpiTransfer(tx, rx, len + 1u);
}

bool ReadCommand(uint8_t op, uint8_t* data, uint32_t len) {
  uint8_t tx[8]{};
  uint8_t rx[8]{};
  if (len + 2u > sizeof(tx)) {
    return false;
  }
  tx[0] = op;
  if (!SpiTransfer(tx, rx, len + 2u)) {
    return false;
  }
  std::memcpy(data, &rx[2], len);
  return true;
}

bool WriteBuffer(uint8_t offset, const uint8_t* data, uint32_t len) {
  uint8_t tx[96];
  if (len + 2u > sizeof(tx)) {
    return false;
  }
  tx[0] = kCmdWriteBuffer;
  tx[1] = offset;
  std::memcpy(&tx[2], data, len);
  uint8_t rx[sizeof(tx)]{};
  return SpiTransfer(tx, rx, len + 2u);
}

bool WriteRegister(uint16_t addr, const uint8_t* data, uint32_t len) {
  uint8_t tx[16];
  if (len + 3u > sizeof(tx)) {
    return false;
  }
  tx[0] = kCmdWriteRegister;
  tx[1] = static_cast<uint8_t>(addr >> 8);
  tx[2] = static_cast<uint8_t>(addr);
  if (len > 0) {
    std::memcpy(&tx[3], data, len);
  }
  uint8_t rx[sizeof(tx)]{};
  return SpiTransfer(tx, rx, len + 3u);
}

uint32_t RfFrequencyReg(uint32_t freq_hz) {
  return static_cast<uint32_t>((static_cast<uint64_t>(freq_hz) << 25) / 32000000ull);
}

uint8_t RampCode() {
  return 0x04;  // 200 us
}

uint8_t ClampedPower() {
  return static_cast<uint8_t>(std::max(-9, std::min(22, cfg::kLoraTxPowerDbm)));
}

bool ConfigureRadio() {
  uint8_t standby[] = {0x00};
  MaybeLog("[birdnet] lora cfg standby\r\n");
  if (!WriteCommand(kCmdSetStandby, standby, sizeof(standby))) return false;

  MaybeLog("[birdnet] lora cfg tcxo\r\n");
  uint8_t tcxo[] = {0x02, 0x00, 0x01, 0x40};  // 1.8 V, about 5 ms.
  (void)WriteCommand(kCmdSetDio3AsTcxoCtrl, tcxo, sizeof(tcxo));

  MaybeLog("[birdnet] lora cfg packet type\r\n");
  uint8_t packet_type[] = {0x01};  // LoRa
  if (!WriteCommand(kCmdSetPacketType, packet_type, sizeof(packet_type))) return false;

  MaybeLog("[birdnet] lora cfg sync word\r\n");
  uint8_t sync_word[] = {cfg::kLoraSyncWord, 0x44};
  if (!WriteRegister(0x0740, sync_word, sizeof(sync_word))) return false;

  MaybeLog("[birdnet] lora cfg frequency\r\n");
  const uint32_t frf = RfFrequencyReg(cfg::kLoraFreqHz);
  uint8_t freq[] = {
      static_cast<uint8_t>(frf >> 24),
      static_cast<uint8_t>(frf >> 16),
      static_cast<uint8_t>(frf >> 8),
      static_cast<uint8_t>(frf),
  };
  if (!WriteCommand(kCmdSetRfFrequency, freq, sizeof(freq))) return false;

  MaybeLog("[birdnet] lora cfg rf switch\r\n");
  uint8_t dio2_rf[] = {0x01};
  (void)WriteCommand(kCmdSetDio2AsRfSwitchCtrl, dio2_rf, sizeof(dio2_rf));

  MaybeLog("[birdnet] lora cfg pa\r\n");
  uint8_t pa[] = {0x04, 0x07, 0x00, 0x01};
  if (!WriteCommand(kCmdSetPaConfig, pa, sizeof(pa))) return false;

  MaybeLog("[birdnet] lora cfg tx params\r\n");
  uint8_t tx_params[] = {ClampedPower(), RampCode()};
  if (!WriteCommand(kCmdSetTxParams, tx_params, sizeof(tx_params))) return false;

  MaybeLog("[birdnet] lora cfg modulation\r\n");
  uint8_t mod[] = {0x07, 0x04, 0x01, 0x00};  // SF7, 125 kHz, CR 4/5, LDRO off.
  if (!WriteCommand(kCmdSetModulationParams, mod, sizeof(mod))) return false;

  MaybeLog("[birdnet] lora cfg packet params\r\n");
  uint8_t pkt[] = {0x00, 0x08, 0x00, 0x00, 0x01, 0x00};  // Explicit header, CRC on.
  if (!WriteCommand(kCmdSetPacketParams, pkt, sizeof(pkt))) return false;

  MaybeLog("[birdnet] lora cfg buffer base\r\n");
  uint8_t base[] = {0x00, 0x00};
  if (!WriteCommand(kCmdSetBufferBaseAddress, base, sizeof(base))) return false;

  MaybeLog("[birdnet] lora cfg irq\r\n");
  uint8_t irq[] = {
      static_cast<uint8_t>(kAllIrqs >> 8),
      static_cast<uint8_t>(kAllIrqs),
      static_cast<uint8_t>((kIrqTxDone | kIrqTimeout) >> 8),
      static_cast<uint8_t>(kIrqTxDone | kIrqTimeout),
      0x00,
      0x00,
      0x00,
      0x00,
  };
  if (!WriteCommand(kCmdSetDioIrqParams, irq, sizeof(irq))) return false;

  MaybeLog("[birdnet] lora cfg clear irq\r\n");
  uint8_t clear[] = {static_cast<uint8_t>(kAllIrqs >> 8), static_cast<uint8_t>(kAllIrqs)};
  return WriteCommand(kCmdClearIrqStatus, clear, sizeof(clear));
}

void ConfigurePins() {
  hx_drv_scu_set_PB2_pinmux(SCU_PB2_PINMUX_SPI_M_DO_1, 1);
  hx_drv_scu_set_PB3_pinmux(SCU_PB3_PINMUX_SPI_M_DI_1, 1);
  hx_drv_scu_set_PB4_pinmux(SCU_PB4_PINMUX_SPI_M_SCLK_1, 1);
  hx_drv_scu_set_PB5_pinmux(SCU_PB5_PINMUX_GPIO16, 1);
  hx_drv_scu_set_SEN_D3_pinmux(SCU_SEN_D3_PINMUX_GPIO21);
  hx_drv_scu_set_SEN_D4_pinmux(SCU_SEN_D4_PINMUX_GPIO22);
  hx_drv_scu_set_SEN_D5_pinmux(SCU_SEN_D5_PINMUX_GPIO23);
  hx_drv_scu_set_SEN_D6_pinmux(SCU_SEN_D6_PINMUX_GPIO24);
  hx_drv_gpio_set_output(GPIO16, GPIO_OUT_HIGH);
  hx_drv_gpio_set_output(GPIO21, GPIO_OUT_HIGH);
  hx_drv_gpio_set_input(GPIO22);
  hx_drv_gpio_set_input(GPIO23);
  hx_drv_gpio_set_output(GPIO24, GPIO_OUT_LOW);
}

void ResetRadio() {
  hx_drv_gpio_set_out_value(GPIO21, GPIO_OUT_LOW);
  DelayMs(2);
  hx_drv_gpio_set_out_value(GPIO21, GPIO_OUT_HIGH);
  DelayMs(10);
  (void)WaitBusyLow(100);
}

uint16_t ScoreQ1e4(float score) {
  if (score <= 0.0f) return 0;
  if (score >= 1.0f) return 10000;
  return static_cast<uint16_t>(score * 10000.0f + 0.5f);
}

uint16_t ReadIrqStatus() {
  uint8_t irq[2]{};
  if (!ReadCommand(kCmdGetIrqStatus, irq, sizeof(irq))) {
    return 0;
  }
  return (static_cast<uint16_t>(irq[0]) << 8) | irq[1];
}

bool SendPayload(const uint8_t* payload, uint8_t payload_len) {
  uint8_t pkt[] = {0x00, 0x08, 0x00, payload_len, 0x01, 0x00};
  uint8_t clear[] = {static_cast<uint8_t>(kAllIrqs >> 8), static_cast<uint8_t>(kAllIrqs)};
  uint8_t timeout[] = {0x00, 0x7D, 0x00};  // 500 ms in 15.625 us units.

  RfSwitchRx(false);
  if (!WriteCommand(kCmdClearIrqStatus, clear, sizeof(clear)) ||
      !WriteCommand(kCmdSetPacketParams, pkt, sizeof(pkt)) ||
      !WriteBuffer(0x00, payload, payload_len) ||
      !WriteCommand(kCmdSetTx, timeout, sizeof(timeout))) {
    return false;
  }

  const uint64_t deadline = Millis() + kTxTimeoutMs;
  while (Millis() < deadline) {
    const uint16_t irq = ReadIrqStatus();
    if (irq & kIrqTxDone) {
      (void)WriteCommand(kCmdClearIrqStatus, clear, sizeof(clear));
      return true;
    }
    if (irq & kIrqTimeout) {
      (void)WriteCommand(kCmdClearIrqStatus, clear, sizeof(clear));
      return false;
    }
  }
  (void)WriteCommand(kCmdClearIrqStatus, clear, sizeof(clear));
  return false;
}
#endif

}  // namespace

bool Init(LogFn log_fn) {
#if BIRDNET_ENABLE_LORA_DETECTION_TX
  g_log = log_fn;
  MaybeLog("[birdnet] lora init begin\r\n");
  ConfigurePins();
  MaybeLog("[birdnet] lora pins configured\r\n");
  ResetRadio();
  MaybeLog("[birdnet] lora reset done busy=%u\r\n", BusyHigh() ? 1u : 0u);
  if (Driver_SPI0.Initialize(nullptr) != ARM_DRIVER_OK ||
      Driver_SPI0.PowerControl(ARM_POWER_FULL) != ARM_DRIVER_OK ||
      Driver_SPI0.Control(ARM_SPI_MODE_MASTER | ARM_SPI_CPOL0_CPHA0 |
                              ARM_SPI_DATA_BITS(8) | ARM_SPI_MSB_LSB |
                              ARM_SPI_SS_MASTER_SW,
                          kSpiHz) != ARM_DRIVER_OK ||
      Driver_SPI0.Control(ARM_SPI_SET_DEFAULT_TX_VALUE, 0x00) != ARM_DRIVER_OK) {
    MaybeLog("[birdnet] lora spi init failed\r\n");
    return false;
  }
  g_initialized = ConfigureRadio();
  MaybeLog("[birdnet] lora tx %s freq=%lu power=%d\r\n",
           g_initialized ? "ready" : "unavailable",
           static_cast<unsigned long>(cfg::kLoraFreqHz),
           cfg::kLoraTxPowerDbm);
  return g_initialized;
#else
  (void)log_fn;
  return false;
#endif
}

bool SendDetection(const InferenceResult& result) {
#if BIRDNET_ENABLE_LORA_DETECTION_TX
  if (!g_initialized || !result.detected || result.pred_class < 0) {
    return false;
  }

  uint8_t payload[22]{};
  const uint16_t seq = ++g_sequence;
  const uint32_t ts_s = static_cast<uint32_t>(result.timestamp_ms / 1000ull);
  const uint16_t score = ScoreQ1e4(result.pred_score);
  payload[0] = kPayloadMagic0;
  payload[1] = kPayloadMagic1;
  payload[2] = kPayloadVersion;
  payload[3] = cfg::kLoraNodeId;
  payload[4] = static_cast<uint8_t>(seq >> 8);
  payload[5] = static_cast<uint8_t>(seq);
  payload[6] = static_cast<uint8_t>(ts_s >> 24);
  payload[7] = static_cast<uint8_t>(ts_s >> 16);
  payload[8] = static_cast<uint8_t>(ts_s >> 8);
  payload[9] = static_cast<uint8_t>(ts_s);
  payload[10] = static_cast<uint8_t>(result.pred_class >> 8);
  payload[11] = static_cast<uint8_t>(result.pred_class);
  payload[12] = static_cast<uint8_t>(score >> 8);
  payload[13] = static_cast<uint8_t>(score);
  payload[14] = static_cast<uint8_t>(result.log_detection_count);

  const bool ok = SendPayload(payload, sizeof(payload));
  if (!ok) {
    MaybeLog("[birdnet] lora tx failed seq=%u idx=%d score_q1e4=%u\r\n",
             static_cast<unsigned>(seq),
             result.pred_class,
             static_cast<unsigned>(score));
  } else if (cfg::kEnableStatusLogs) {
    MaybeLog("[birdnet] lora tx ok seq=%u idx=%d score_q1e4=%u\r\n",
             static_cast<unsigned>(seq),
             result.pred_class,
             static_cast<unsigned>(score));
  }
  return ok;
#else
  (void)result;
  return false;
#endif
}

bool SendTestPacket(uint64_t timestamp_ms) {
#if BIRDNET_ENABLE_LORA_DETECTION_TX
  InferenceResult result{};
  result.timestamp_ms = timestamp_ms;
  result.pred_class = 0;
  result.pred_score = 1.0f;
  result.detected = true;
  result.log_detection_count = 1;
  const bool ok = SendDetection(result);
  MaybeLog("[birdnet] lora test tx %s\r\n", ok ? "ok" : "failed");
  return ok;
#else
  (void)timestamp_ms;
  return false;
#endif
}

}  // namespace birdnet_lora
