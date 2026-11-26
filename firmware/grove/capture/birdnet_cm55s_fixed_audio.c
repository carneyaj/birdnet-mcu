#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "WE2_core.h"
#include "WE2_device.h"
#include "WE2_device_addr.h"
#include "board.h"
#include "console_io.h"
#include "hx_drv_dmac.h"
#include "hx_drv_gpio.h"
#include "hx_drv_mb.h"
#include "hx_drv_pdm_rx.h"
#include "hx_drv_pmu.h"
#include "hx_drv_scu.h"
#include "hx_drv_swreg_aon.h"
#include "hx_drv_swreg_lsc.h"
#include "hx_drv_uart.h"
#include "hxmb.h"
#include "trustzone_cfg.h"
#include "xprintf.h"
#include "birdnet_audio_mailbox.h"
#ifndef BIRDNET_ENABLE_STATUS_BUTTON
#define BIRDNET_ENABLE_STATUS_BUTTON 0
#endif
#if BIRDNET_CAPTURE_WATCHDOG
#include "hx_drv_watchdog.h"
/* Opt-in until PMU clock-domain/reset coverage is fault-injection tested. */
#define CAPTURE_WATCHDOG_MS 30000u
static BirdnetProgressWatchdog g_watchdog_progress;
#endif

#ifndef BIRDNET_CM55S_HELPER_LOGS
#define BIRDNET_CM55S_HELPER_LOGS 0
#endif

#ifndef BIRDNET_CM55S_PMU_WAKE_HANDSHAKE
#define BIRDNET_CM55S_PMU_WAKE_HANDSHAKE 0
#endif

#ifndef BIRDNET_CM55S_DISABLE_CONSOLE_UART
#define BIRDNET_CM55S_DISABLE_CONSOLE_UART 0
#endif

#ifndef BIRDNET_CM55S_MINIMAL_BOARD_INIT
#define BIRDNET_CM55S_MINIMAL_BOARD_INIT 0
#endif

#if BIRDNET_CM55S_HELPER_LOGS
#define CM55S_LOG(...) xprintf(__VA_ARGS__)
#else
#define CM55S_LOG(...) ((void)0)
#endif

#define PDM_CAPTURE_SAMPLE_RATE_HZ 48000
#ifndef BIRDNET_CM55S_OUTPUT_SAMPLE_RATE_HZ
#define BIRDNET_CM55S_OUTPUT_SAMPLE_RATE_HZ 48000
#endif
#define SAMPLE_RATE_HZ BIRDNET_CM55S_OUTPUT_SAMPLE_RATE_HZ
#define WINDOW_SEC 3
#ifndef BIRDNET_INFERENCE_INTERVAL_MS
#define BIRDNET_INFERENCE_INTERVAL_MS 3000
#endif
#if (SAMPLE_RATE_HZ <= 0)
#error "Invalid CM55S output sample rate"
#endif
#if ((PDM_CAPTURE_SAMPLE_RATE_HZ % SAMPLE_RATE_HZ) != 0)
#error "PDM capture sample rate must divide evenly to CM55S output sample rate"
#endif
#define PDM_DECIMATION (PDM_CAPTURE_SAMPLE_RATE_HZ / SAMPLE_RATE_HZ)
#define WINDOW_SAMPLES (SAMPLE_RATE_HZ * WINDOW_SEC)
#ifndef BIRDNET_INFERENCE_STEP_SAMPLES
#define BIRDNET_INFERENCE_STEP_SAMPLES 0
#endif
#if BIRDNET_INFERENCE_STEP_SAMPLES > 0
#define WINDOW_STEP_SAMPLES BIRDNET_INFERENCE_STEP_SAMPLES
#else
#define WINDOW_STEP_SAMPLES ((SAMPLE_RATE_HZ * BIRDNET_INFERENCE_INTERVAL_MS) / 1000)

#if ((SAMPLE_RATE_HZ * BIRDNET_INFERENCE_INTERVAL_MS) % 1000) != 0
#error "Inference interval must align to whole samples"
#endif
#endif

#if (WINDOW_STEP_SAMPLES <= 0) || (BIRDNET_INFERENCE_STEP_SAMPLES < 0)
#error "Inference sample step must be positive"
#endif
/*
 * DMA writes directly into the audio ring. The SDK limits each LLI block to
 * less than 8192 bytes; this pinned driver allows at most 64 LLI blocks. Defaults keep the
 * verified 0.375 s cadence; larger block counts reduce CM55S wakeups.
 */
#ifndef BIRDNET_CM55S_DMA_BLOCKS
#define BIRDNET_CM55S_DMA_BLOCKS 5
#endif

#ifndef BIRDNET_CM55S_DMA_BLOCK_BYTES
#define BIRDNET_CM55S_DMA_BLOCK_BYTES 7200
#endif

#define DMA_BLOCKS BIRDNET_CM55S_DMA_BLOCKS
#define DMA_BLOCK_BYTES BIRDNET_CM55S_DMA_BLOCK_BYTES
#define PCM_SAMPLE_BYTES 2
#define DMA_TOTAL_BYTES (DMA_BLOCKS * DMA_BLOCK_BYTES)
#define DMA_RAW_CHUNK_SAMPLES (DMA_TOTAL_BYTES / PCM_SAMPLE_BYTES)
#define DMA_CHUNK_SAMPLES (DMA_RAW_CHUNK_SAMPLES / PDM_DECIMATION)
#if ((WINDOW_STEP_SAMPLES % DMA_CHUNK_SAMPLES) != 0)
#error "Inference interval must align to the capture DMA chunk"
#endif
/* Round UP, retaining a whole spare DMA chunk so an in-flight write cannot
 * overwrite any published window, even when three seconds is not a multiple
 * of the capture chunk. The first snapshot waits for the rounded-up fill. */
#define RING_SAMPLES (((WINDOW_SAMPLES + DMA_CHUNK_SAMPLES - 1) / DMA_CHUNK_SAMPLES + 1) * DMA_CHUNK_SAMPLES)
#define RING_SLACK_SAMPLES (RING_SAMPLES - WINDOW_SAMPLES)

#if (DMA_BLOCKS < 1)
#error "PDM DMA LLI block count must be positive"
#endif

#if (DMA_BLOCKS > 64)
#error "PDM DMA LLI block count must be <= 64"
#endif

#if (DMA_BLOCK_BYTES >= 8192)
#error "PDM DMA LLI block size must be less than 8192 bytes"
#endif

#if ((DMA_TOTAL_BYTES % 32) != 0)
#error "PDM DMA total byte count must remain cache-line aligned"
#endif

#if ((DMA_TOTAL_BYTES % PCM_SAMPLE_BYTES) != 0)
#error "PDM DMA total byte count must contain whole int16 samples"
#endif

#if ((DMA_RAW_CHUNK_SAMPLES % PDM_DECIMATION) != 0)
#error "PDM DMA chunk must decimate to whole output samples"
#endif

#if ((RING_SAMPLES % DMA_CHUNK_SAMPLES) != 0)
#error "Audio ring must be an integer number of DMA chunks"
#endif

#define CM55S_MARK_BOOT 0x53550021u
#define CM55S_MARK_MB_INIT 0x53550022u
#define CM55S_MARK_CM55M_DC 0x53550023u
#define CM55S_MARK_RDY_SENT 0x53550024u
#define CM55S_MARK_SNAPSHOT 0x53550025u
#define CM55S_MARK_SNAPSHOT_ERR 0x53550026u

static volatile uint32_t g_mb_event_bits = 0;
static volatile uint32_t g_dma_start_idx = 0;
static volatile uint32_t g_dma_buf_idx = 0;
static volatile uint32_t g_ring_write_idx = 0;
static volatile uint32_t g_ring_fill = 0;
static volatile uint64_t g_total_samples = 0;
static uint64_t g_next_snapshot_sample =
    WINDOW_STEP_SAMPLES > WINDOW_SAMPLES ? WINDOW_STEP_SAMPLES : WINDOW_SAMPLES;
static bool g_cm55m_dc_ready = false;
static bool g_cm55s_ready_sent = false;
static bool g_snapshot_wake_pending = false;
static uint32_t g_capture_generation = 0;
static uint32_t g_descriptor_sequence = 0;
static uint64_t g_last_wake_sample = 0;

static volatile uint32_t g_circular_irq_count;
static volatile bool g_capture_fault = false;
static bool g_fault_wake_sent = false;
#if BIRDNET_ENABLE_STATUS_BUTTON
static BirdnetUiState g_ui;
static volatile bool g_button_edge = false;
static uint32_t g_button_sequence = 0;
static bool g_ui_event_sent = false;
static uint32_t g_button_init_errors = 0;
static uint64_t g_button_wake_after = 0;
static bool g_button_wake_blocked = false;
static bool g_button_release_seen = false;

static void button_cb(uint8_t group, uint8_t index) {
  (void)group;
  (void)index;
  g_button_edge = true;
}

static void init_status_button(void) {
  g_button_init_errors |= hx_drv_gpio_init(GPIO_GROUP_2, HX_GPIO_GROUP_2_BASE) != GPIO_NO_ERROR;
  g_button_init_errors |= (hx_drv_scu_set_SEN_D0_pinmux(SCU_SEN_D0_PINMUX_GPIO18) != 0) << 1;
  g_button_init_errors |= (hx_drv_gpio_set_input(GPIO18) != GPIO_NO_ERROR) << 2;
  g_button_init_errors |= (hx_drv_gpio_cb_register(GPIO18, button_cb) != GPIO_NO_ERROR) << 3;
  g_button_init_errors |= (hx_drv_gpio_set_int_type(GPIO18, GPIO_IRQ_TRIG_TYPE_EDGE_RISING) != GPIO_NO_ERROR) << 4;
  hx_drv_gpio_clr_int_status(GPIO18);
  g_button_init_errors |= (hx_drv_gpio_set_int_enable(GPIO18, 1) != GPIO_NO_ERROR) << 5;
  g_ui.button_armed = true;
}

static bool service_ui(uint64_t samples) {
  uint32_t command = 0, ack = 0;
  hx_drv_mb_get_sharedata(BN_MB_UI_COMMAND, &command);
  const uint32_t previous_expired = g_ui.expired;
  birdnet_ui_update(&g_ui, command, samples, SAMPLE_RATE_HZ, DMA_CHUNK_SAMPLES);
  hx_drv_mb_set_sharedata(BN_MB_UI_COMMAND_ACK, command);
  hx_drv_mb_set_sharedata(BN_MB_UI_EXPIRED, g_ui.expired);
  hx_drv_mb_set_sharedata(BN_MB_UI_CAPTURE_LOW, (uint32_t)samples);

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  // GPIO18 can latch an edge even when its IRQ is not routed to CM55S.
  // Consume it on an existing DMA/mailbox wake; no polling timer is needed.
  const bool pending = hx_drv_gpio_get_int_status(GPIO18);
  const bool edge = g_button_edge || pending;
  if (pending) hx_drv_gpio_clr_int_status(GPIO18);
  g_button_edge = false;
  __set_PRIMASK(primask);
  uint8_t high = 0;
  const bool read_ok = hx_drv_gpio_get_in_value(GPIO18, &high) == GPIO_NO_ERROR;
  // BOOT1 also asserts BOOT_OPT. Do not restart CM55M through the bootloader
  // until a captured press has been released for an existing DMA interval.
  if (edge || !read_ok || high) {
    g_button_wake_blocked = true;
    g_button_release_seen = false;
  } else if (g_button_wake_blocked) {
    if (!g_button_release_seen) {
      g_button_release_seen = true;
      g_button_wake_after = samples + 4800u; // >100 ms BOOT strap release guard
    } else if (samples >= g_button_wake_after) {
      g_button_wake_blocked = false;
    }
  }
  if (birdnet_ui_button(&g_ui, edge || (read_ok && high), !read_ok || high,
                        samples, DMA_CHUNK_SAMPLES)) {
    ++g_button_sequence;
    hx_drv_mb_set_sharedata(BN_MB_BUTTON_SEQUENCE, g_button_sequence);
    g_ui_event_sent = false;
  }
  const bool gpio_clock_on=(*(volatile const uint32_t *)0x56100044u&(1u<<11))!=0;
  hx_drv_mb_set_sharedata(BN_MB_UI_DIAGNOSTICS,
      (g_button_init_errors << 24) | (g_button_sequence << 8) |
      (read_ok ? high : 0x80u) | (NVIC_GetEnableIRQ(GPIO_GROUP_2_IRQn) ? 2u : 0u) |
      (hx_drv_gpio_get_int_status(GPIO18) ? 4u : 0u) | (gpio_clock_on ? 8u : 0u) |
      (g_ui.button_armed ? 0x10u : 0u) | (g_button_wake_blocked ? 0x20u : 0u));
  if (g_ui.expired != previous_expired) g_ui_event_sent = false;
  hx_drv_mb_get_sharedata(BN_MB_BUTTON_ACK, &ack);
  return ack != g_button_sequence || g_ui.expired != 0;
}

static bool status_button_allows_wake(void) {
  uint8_t high = 0;
  // Recheck at the wake boundary: a press may arrive after service_ui().
  return !g_button_wake_blocked && !g_button_edge &&
      !hx_drv_gpio_get_int_status(GPIO18) &&
      hx_drv_gpio_get_in_value(GPIO18, &high) == GPIO_NO_ERROR && !high;
}
#endif

// Helper UART output is already disabled; main retains all live diagnostics.
void __wrap_xprintf(const char *format,...) {(void)format;}
static PDM_DEV_INFO g_pdm_dev_info;

#include "birdnet_capture_power.h"


static struct __attribute__((aligned(32))) {
  BirdnetClockTrace trace;
  int16_t ring[RING_SAMPLES] __attribute__((aligned(32)));
} g_audio_storage;
#define g_audio_ring (g_audio_storage.ring)
typedef char BirdnetClockRingOffset[
    (__builtin_offsetof(__typeof__(g_audio_storage), ring) == 160) ? 1 : -1];

static inline void capture_sleep_clocks(uint64_t samples, uint32_t ready) { (void)samples; (void)ready; }

#if (PDM_DECIMATION != 1)
static int16_t g_dma_buf[2][DMA_RAW_CHUNK_SAMPLES] __attribute__((aligned(32)));
#endif

void __real_board_init(void);

void __wrap_board_init(void) {
#if BIRDNET_CM55S_MINIMAL_BOARD_INIT
  uint32_t freq = 0;

#if defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
  TZ_Set_ALL_Secure();
#endif
  hx_drv_scu_init();
  hx_drv_scu_get_freq(SCU_CLK_FREQ_TYPE_HSC_CLK, &freq);
  SystemCoreClockUpdate(freq);
  hx_drv_dmac_init(USE_HX_DMAC_2, DMAC2_BASE);
  hx_drv_mb_init(MB_CORE_CM55M, HX_MAILBOX_INT_BASE);
#else
  __real_board_init();
#endif
}

static void mb_cb(uint32_t event) {
  if (event == 0) {
    return;
  }
  hx_drv_mb_clear_cm55s_irq(event);
  g_mb_event_bits |= event;
}

static void publish_dma_chunk(uint32_t next_write_idx) {
  g_ring_write_idx = next_write_idx;
  if (g_ring_fill < RING_SAMPLES) {
    const uint32_t new_fill = g_ring_fill + WINDOW_STEP_SAMPLES;
    g_ring_fill = (new_fill < RING_SAMPLES) ? new_fill : RING_SAMPLES;
  }
  g_total_samples += WINDOW_STEP_SAMPLES;
  /* Invalidate outstanding descriptors BEFORE DMA starts reusing the ring. */
  hx_drv_mb_set_sharedata(BN_MB_LIVE_GENERATION, ++g_capture_generation);
  __DMB();
}

static void app_pdm_error_cb(uint32_t int_status) {
  (void)int_status;
  g_capture_fault = true;
  hx_drv_mb_set_sharedata(BN_MB_FAULT, 2);
  hx_drv_mb_set_sharedata(BN_MB_LIVE_GENERATION, ++g_capture_generation);
  __DMB();
}


/* Exact SDK exports this table as global DMAC_MEM_PERI_LLI_T[64]. Its normal
 * infinite flag builds a closed hardware LLI chain with no terminal IRQs.
 * Enable a TC IRQ on each sixth node, keeping the existing 24000-sample
 * publication granularity. Hardware continues across every IRQ; no re-arm. */
extern DMAC_MEM_PERI_LLI_T pdm_lli_table[];
enum { BN_CIRCULAR_BLOCKS=RING_SAMPLES*PCM_SAMPLE_BYTES/DMA_BLOCK_BYTES };
typedef char CircularGeometry[(PDM_DECIMATION==1 && BN_CIRCULAR_BLOCKS==42 &&
    DMA_BLOCKS==6 && DMA_BLOCK_BYTES==8000 && sizeof(DMAC_MEM_PERI_LLI_T)==28) ? 1 : -1];

static unsigned g_circular_next_node=35;
static void circular_dma_irq(void) {
  restore_sleep_divider();
  const uint32_t errors=*(volatile const uint32_t *)0x5180000cu;
  const uint32_t complete=*(volatile const uint32_t *)0x51800004u;
  if (errors || (complete & ~1u)) {
    *(volatile uint32_t *)0x51800010u=errors;
    *(volatile uint32_t *)0x51800008u=complete;
    app_pdm_error_cb(errors|0x100u); return;
  }
  if (!(complete&1u)) return;
  *(volatile uint32_t *)0x51800008u=1u;
  __DSB();
  const uint32_t finished=g_dma_start_idx;
  uint32_t next=finished+WINDOW_STEP_SAMPLES;
  if (next>=RING_SAMPLES) next-=RING_SAMPLES;
  const uint32_t dest=*(volatile const uint32_t *)0x51800104u;
  // A missing IRQ or an unexpected channel position is a capture fault.
  const uint32_t ahead=birdnet_circular_ahead((uint32_t)g_audio_ring,RING_SAMPLES,
      (next+RING_SAMPLES-WINDOW_SAMPLES)%RING_SAMPLES,WINDOW_SAMPLES,dest);
  if (ahead>=DMA_BLOCK_BYTES/PCM_SAMPLE_BYTES) {
    app_pdm_error_cb(0x200u); return;
  }

  const unsigned next_node=(g_circular_next_node+WINDOW_STEP_SAMPLES/(DMA_BLOCK_BYTES/PCM_SAMPLE_BYTES))%BN_CIRCULAR_BLOCKS;
  pdm_lli_table[g_circular_next_node].raw_lli.transfer_ctrl&=~(1u<<31);
  pdm_lli_table[next_node].raw_lli.transfer_ctrl|=(1u<<31);
  g_circular_next_node=next_node;
  hx_CleanDCache_by_Addr(pdm_lli_table,BN_CIRCULAR_BLOCKS*sizeof(*pdm_lli_table));
  __DSB();
  g_dma_start_idx=next;
  ++g_circular_irq_count;
  publish_dma_chunk(next);
}

static bool __attribute__((optimize("Os"), noinline)) init_circular_dma(void) {
  if (hx_drv_pdm_dma_lli_transfer(g_audio_ring,BN_CIRCULAR_BLOCKS,DMA_BLOCK_BYTES,1)!=PDM_NO_ERROR) return false;
  // Complete setup well before the first 85 ms hardware block is captured.
  for (unsigned i=0;i<BN_CIRCULAR_BLOCKS;++i) {
    DMAC_RAWLLI_T *node=&pdm_lli_table[i].raw_lli;
    if (node->dst_addr!=(uint32_t)&g_audio_ring[i*DMA_BLOCK_BYTES/PCM_SAMPLE_BYTES] ||
        (node->next_lli&~3u)!=(uint32_t)&pdm_lli_table[(i+1u)%BN_CIRCULAR_BLOCKS]) return false;
    node->transfer_ctrl=(node->transfer_ctrl&~(1u<<31))|((i==g_circular_next_node)?(1u<<31):0u);
  }
  hx_CleanDCache_by_Addr(pdm_lli_table,BN_CIRCULAR_BLOCKS*sizeof(*pdm_lli_table));
  EPII_NVIC_SetVector(DMAC2_DMACINTR_IRQn,(uint32_t)circular_dma_irq);
  EPII_NVIC_SetVector(DMAC2_DMACINTTC_IRQn,(uint32_t)circular_dma_irq);
  EPII_NVIC_SetVector(DMAC2_DMACINTERR_IRQn,(uint32_t)circular_dma_irq);
  // The SDK's infinite-loop mode normally has no completion callback. Keep
  // both channel interrupt sources unmasked for our explicit TC checkpoints.
  *(volatile uint32_t *)0x51800110u |= (1u<<14)|(1u<<15);
  __DSB(); __ISB();
  return true;
}

static bool init_mailbox(void) {
  const MBLIB_ERR_E err = hx_lib_mb_cm55s_init(mb_cb);
  if (err != MBLIB_ERR_NO_ERROR) {
    return false;
  }
  hx_drv_swreg_aon_set_appused1(CM55S_MARK_MB_INIT);
  for (uint8_t slot = BN_MB_MAGIC; slot <= BN_MB_LAST_SLOT; ++slot) {
    hx_drv_mb_set_sharedata(slot, 0);
  }
  hx_drv_mb_set_sharedata(BN_MB_MAGIC, BN_MB_MAGIC_VALUE);
  return true;
}

static bool init_mic_capture(void) {
  memset(&g_pdm_dev_info, 0, sizeof(g_pdm_dev_info));

  hx_drv_scu_set_PB9_pinmux(SCU_PB9_PINMUX_PDM_CLK_13, 1);
  hx_drv_scu_set_PB10_pinmux(SCU_PB10_PINMUX_PDM_DATA0_13, 1);

  g_pdm_dev_info.pdm_cfg.reg_addr = HIMAX_PDM_BASE_ADDR;
  g_pdm_dev_info.pdm_cfg.rx_fifo_threshold = 5;
  g_pdm_dev_info.pdm_cfg.oversampling_ratio = 8;
  g_pdm_dev_info.pdm_cfg.cic_stages = 0;
  g_pdm_dev_info.pdm_cfg.cic_delay = 0;
  g_pdm_dev_info.pdm_cfg.dc_removal = 6;
  g_pdm_dev_info.pdm_cfg.bit_range_shift = 5;
  g_pdm_dev_info.pdm_cfg.data_in_0_en = PDM_DATA_IN_ENABLE;
  g_pdm_dev_info.pdm_cfg.data_in_1_en = PDM_DATA_IN_DISABLE;
  g_pdm_dev_info.pdm_cfg.data_in_2_en = PDM_DATA_IN_DISABLE;
  g_pdm_dev_info.pdm_cfg.data_in_3_en = PDM_DATA_IN_DISABLE;
  g_pdm_dev_info.pdm_cfg.capture_channel = PDM_CAPTURE_CHANNEL_LEFT_ONLY;
  g_pdm_dev_info.pdm_cfg.dma_ch = PDM_USE_DMA2_CH_0;
  g_pdm_dev_info.pdm_cfg.pdm_clk_src = PDM_CLKSRC_LSCREF;
  g_pdm_dev_info.pdm_cfg.sample_rate = PDM_PCM_FREQ_48K;
  g_pdm_dev_info.pdm_rx_cb = 0; // owned circular DMA2 IRQ
  g_pdm_dev_info.pdm_err_cb = app_pdm_error_cb;
  g_pdm_dev_info.pdm_clip_cb = 0;

  if (hx_drv_pdm_init(&g_pdm_dev_info) != PDM_NO_ERROR) {
    return false;
  }

  g_dma_start_idx = 0;
  g_dma_buf_idx = 0;
  g_ring_write_idx = 0;
  g_ring_fill = 0;
  g_total_samples = 0;
  memset(g_audio_ring, 0, sizeof(g_audio_ring));
  hx_CleanDCache_by_Addr((volatile void *)g_audio_ring, (int32_t)sizeof(g_audio_ring));

  if (!init_circular_dma()) return false;
  return true;
}

#if BIRDNET_CM55S_PMU_WAKE_HANDSHAKE
static void signal_cm55m_preroll_wakeup(void) {
  /*
   * In PM_MODE_PS_AUD_ONLY_PREROLLING, mailbox traffic alone is not enough to
   * wake CM55M reliably on this board. Pulse the explicit PS_SCL wake path
   * before sending the retained audio snapshot.
   */
  (void)hx_drv_pmu_set_ctrl(PMU_REG_FORCE_EVENT_NSTATE, PMU_PS_STATE_DC);
  (void)hx_drv_pmu_set_ctrl(PMU_REG_FORCE_EVENT_trig, 1);
  __DSB();
  __ISB();
  (void)hx_drv_pmu_set_ctrl(PMU_REG_FORCE_EVENT_trig, 0);
  (void)hx_drv_swreg_lsc_set_sw_wakeupevent(
      (SW_FORCE_WAKEUPEVENT_E)(PMU_SW_WAKEUP_SCL_AUDONLY_SW_TRIG |
                               PMU_SW_WAKEUP_SCL_SENEXTINT));
}
#endif

static bool send_latest_window_snapshot(void) {
  uint32_t ack = 0;
  hx_drv_mb_get_sharedata(BN_MB_ACK, &ack);
  if (ack != g_descriptor_sequence) return false;
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (g_ring_fill < WINDOW_SAMPLES || !g_cm55m_dc_ready || g_capture_fault) {
    __set_PRIMASK(primask);
    return false;
  }
  const uint32_t window_start = (g_ring_write_idx + RING_SAMPLES - WINDOW_SAMPLES) % RING_SAMPLES;
  const uint64_t end = g_total_samples;
  g_descriptor_sequence += 2u;
  if (g_descriptor_sequence == 0) g_descriptor_sequence = 2;
  hx_drv_mb_set_sharedata(BN_MB_SEQUENCE, g_descriptor_sequence | 1u);
  __DMB();
  hx_drv_mb_set_sharedata(BN_MB_RING_BASE, (uint32_t)g_audio_ring);
  hx_drv_mb_set_sharedata(BN_MB_RING_SAMPLES, RING_SAMPLES);
  hx_drv_mb_set_sharedata(BN_MB_START, window_start);
  hx_drv_mb_set_sharedata(BN_MB_SAMPLES, WINDOW_SAMPLES);
  hx_drv_mb_set_sharedata(BN_MB_END_LOW, (uint32_t)end);
  hx_drv_mb_set_sharedata(BN_MB_END_HIGH, (uint32_t)(end >> 32));
  hx_drv_mb_set_sharedata(BN_MB_CAPTURE_GENERATION, g_capture_generation);
  __DMB();
  hx_drv_mb_set_sharedata(BN_MB_SEQUENCE, g_descriptor_sequence);
  uint32_t power_save = 0;
  hx_drv_mb_get_sharedata(BN_MB_POWER_SAVE, &power_save);
  // Ten seconds leaves margin inside the unchanged 30 s progress watchdog.
  // PDM remains running: this is recoverable standby, not battery isolation.
  g_next_snapshot_sample = birdnet_audio_next_deadline(
      end, power_save ? 10u * SAMPLE_RATE_HZ : WINDOW_STEP_SAMPLES);
  __set_PRIMASK(primask);
  hx_drv_swreg_aon_set_appused1(CM55S_MARK_SNAPSHOT);
  hx_drv_mb_trigger_cm55m_irq(BN_MB_WINDOW_EVENT);
  return true;
}

static bool request_snapshot_wakeup(void) {
#if BIRDNET_ENABLE_STATUS_BUTTON
  if (!status_button_allows_wake()) return false;
  g_ui_event_sent = false;
#endif
  restore_sleep_divider();
  restore_unused_sleep_clocks();
  hx_drv_mb_trigger_cm55m_irq(BN_MB_WAKE_EVENT);
#if BIRDNET_CM55S_PMU_WAKE_HANDSHAKE
  g_cm55m_dc_ready = false;
  g_snapshot_wake_pending = true;
  signal_cm55m_preroll_wakeup();
#else
  g_snapshot_wake_pending = false;
#endif
  return true;
}

static void handle_mailbox_events(void) {
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const uint32_t events = g_mb_event_bits;
  g_mb_event_bits = 0;
  __set_PRIMASK(primask);
  if (events & MBLIB_CM55BIG_EVT_CM55M_DC) {
    g_cm55m_dc_ready = true;
    hx_drv_swreg_aon_set_appused1(CM55S_MARK_CM55M_DC);
  }
}


__attribute__((target("general-regs-only"), optimize("Os"))) int app_main(void) {
  capture_power_init();
  hx_drv_swreg_aon_set_appused1(CM55S_MARK_BOOT);
#if BIRDNET_CAPTURE_WATCHDOG
  WATCHDOG_CFG_T supervisor = {CAPTURE_WATCHDOG_MS, WATCHDOG_CTRL_CPU,
                              WATCHDOG_STATE_PMU, WATCHDOG_RESET};
  if (hx_drv_watchdog_cm55s_start(&supervisor, NULL) != WATCHDOG_NO_ERROR) {
    g_capture_fault = true;
  }
#endif

#if BIRDNET_CM55S_HELPER_LOGS
  xprintf_setup();
  CM55S_LOG("[cm55s] fixed audio helper boot\r\n");
#elif !BIRDNET_CM55S_MINIMAL_BOARD_INIT
  xprintf_desetup();
#if BIRDNET_CM55S_DISABLE_CONSOLE_UART
  console_desetup(DW_UART_0_ID);
#endif
#endif

  if (!init_mailbox()) {
    while (1) {}
  }
  if (g_capture_fault) hx_drv_mb_set_sharedata(BN_MB_FAULT, 4);
#if BIRDNET_ENABLE_STATUS_BUTTON
  init_status_button();
#endif
  if (!init_mic_capture()) {
    g_capture_fault = true;
    hx_drv_mb_set_sharedata(BN_MB_FAULT, 3);
  }

  scalar_epu_guard();

  while (1) {
    uint64_t total_samples;

    handle_mailbox_events();

    if (g_capture_fault && !g_fault_wake_sent) {
      /* Wake the watchdog-protected core instead of silently ceasing capture. */
      if (request_snapshot_wakeup()) {
        g_fault_wake_sent = true;
        hx_drv_mb_trigger_cm55m_irq(BN_MB_FAULT_EVENT);
      }
    }

    if (g_cm55m_dc_ready && !g_cm55s_ready_sent && !g_capture_fault) {
      hx_lib_mb_set_cm55s_rdy();
      g_cm55s_ready_sent = true;
      hx_drv_swreg_aon_set_appused1(CM55S_MARK_RDY_SENT);
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    total_samples = g_total_samples;
    __set_PRIMASK(primask);
#if BIRDNET_CAPTURE_WATCHDOG
    uint32_t ack = 0;
    hx_drv_mb_get_sharedata(BN_MB_ACK, &ack);
    if (birdnet_watchdog_progress(&g_watchdog_progress, total_samples, ack, g_capture_fault) &&
        hx_drv_watchdog_cm55s_update(CAPTURE_WATCHDOG_MS) != WATCHDOG_NO_ERROR) {
      g_capture_fault = true;
      hx_drv_mb_set_sharedata(BN_MB_FAULT, 5);
    }
#endif
    uint32_t sleep_ready = 0;
    hx_drv_mb_get_sharedata(BN_MB_SLEEP_READY, &sleep_ready);
    capture_power_service(sleep_ready);
    capture_sleep_clocks(total_samples, sleep_ready);
    bool ui_pending = false;
#if BIRDNET_ENABLE_STATUS_BUTTON
    // Button/UI time follows the live hardware head even between sparse IRQs.
    const uint32_t ui_mask=__get_PRIMASK(); __disable_irq();
    uint64_t ui_samples=g_total_samples;
    const uint32_t ui_ahead=birdnet_circular_ahead((uint32_t)g_audio_ring,RING_SAMPLES,
        g_dma_start_idx,0,*(volatile const uint32_t *)0x51800104u);
    if (ui_ahead<WINDOW_STEP_SAMPLES) ui_samples+=ui_ahead;
    __set_PRIMASK(ui_mask);
    ui_pending = service_ui(ui_samples);
#endif
    if (sleep_ready == BN_MB_MAINTENANCE && g_cm55m_dc_ready && !g_capture_fault) {
      if (total_samples >= g_next_snapshot_sample) (void)send_latest_window_snapshot();
    } else if (g_snapshot_wake_pending && g_cm55m_dc_ready && !g_capture_fault) {
      // A UI-only wake must NOT publish overlapping audio or shift cadence.
      if (total_samples < g_next_snapshot_sample || send_latest_window_snapshot()) {
        g_snapshot_wake_pending = false;
#if BIRDNET_ENABLE_STATUS_BUTTON
        hx_drv_mb_trigger_cm55m_irq(BN_MB_UI_EVENT);
        g_ui_event_sent = true;
#endif
      }
    } else if (g_snapshot_wake_pending &&
               total_samples - g_last_wake_sample >= SAMPLE_RATE_HZ) {
      /* Retry a lost wake/ready handshake once per captured second. */
      g_last_wake_sample = total_samples;
      request_snapshot_wakeup();
    } else if (!g_snapshot_wake_pending &&
               g_cm55m_dc_ready &&
               sleep_ready == BN_MB_SLEEP && !g_capture_fault &&
               (total_samples >= g_next_snapshot_sample || ui_pending)) {
      g_last_wake_sample = total_samples;
      request_snapshot_wakeup();
#if !BIRDNET_CM55S_PMU_WAKE_HANDSHAKE
      (void)send_latest_window_snapshot();
#endif
    }
#if BIRDNET_ENABLE_STATUS_BUTTON
    if (ui_pending && !g_ui_event_sent && g_cm55m_dc_ready &&
        sleep_ready != BN_MB_SLEEP && !g_snapshot_wake_pending) {
      hx_drv_mb_trigger_cm55m_irq(BN_MB_UI_EVENT);
      g_ui_event_sent = true;
    }
#endif
    // Echo only AFTER publication so the main core can drain the last export
    // descriptor before sleeping, without racing another maintenance snapshot.
    __DMB();
    hx_drv_mb_set_sharedata(BN_MB_MODE_ACK, sleep_ready);
    /* An IRQ arriving after the work check stays pending while PRIMASK is
     * set, so WFI cannot lose it. Never mask interrupts across actual work. */
    primask = __get_PRIMASK();
    __disable_irq();
    if (g_mb_event_bits == 0 && g_total_samples == total_samples
#if BIRDNET_ENABLE_STATUS_BUTTON
        && !g_button_edge && !g_button_wake_blocked
#endif
    ) {
      capture_power_wait();
    }
    __set_PRIMASK(primask);
  }

  return 0;
}
