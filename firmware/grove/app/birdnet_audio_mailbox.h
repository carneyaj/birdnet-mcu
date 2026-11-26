#ifndef BIRDNET_AUDIO_MAILBOX_H
#define BIRDNET_AUDIO_MAILBOX_H

#include <stdbool.h>
#include <stdint.h>


/* Private to the paired fixed-cadence firmware. Slots 0/1 remain SDK-owned.
 * The descriptor uses peripheral registers, not cacheable shared SRAM.
 * CM55S owns the audio descriptor; publication is a seqlock. CM55M writes
 * ACK, SLEEP_READY, UI_COMMAND and BUTTON_ACK. Other UI slots are CM55S-owned.
 */
enum {
  BN_MB_MAGIC = 2,
  BN_MB_SEQUENCE,
  BN_MB_RING_BASE,
  BN_MB_RING_SAMPLES,
  BN_MB_START,
  BN_MB_SAMPLES,
  BN_MB_END_LOW,
  BN_MB_END_HIGH,
  BN_MB_CAPTURE_GENERATION,
  BN_MB_LIVE_GENERATION,
  BN_MB_ACK,
  BN_MB_SLEEP_READY,
  BN_MB_FAULT,
  BN_MB_MODE_ACK,
  BN_MB_UI_COMMAND,
  BN_MB_UI_COMMAND_ACK,
  BN_MB_BUTTON_SEQUENCE,
  BN_MB_BUTTON_ACK,
  BN_MB_UI_EXPIRED,
  BN_MB_UI_CAPTURE_LOW,
  BN_MB_UI_DIAGNOSTICS,
  BN_MB_POWER_SAVE,
  BN_MB_LAST_SLOT = BN_MB_POWER_SAVE,
  BN_MB_MAGIC_VALUE = 0x424e4300u,
  BN_MB_WINDOW_EVENT = 0x10u,
  BN_MB_ACK_EVENT = 0x20u,
  BN_MB_SLEEP_EVENT = 0x40u,
  BN_MB_WAKE_EVENT = 0x80u,
  BN_MB_FAULT_EVENT = 0x100u,
  BN_MB_UI_EVENT = 0x200u
};
/* SDK mailbox driver accepts indices 0..23, verified in the linked driver. */
typedef char BirdnetMailboxSlotLimit[(BN_MB_LAST_SLOT <= 23) ? 1 : -1];

/* A single atomic command carries a generation plus its on/off bit. The
 * capture-core deadline survives main-core warm boots. Rounding to the NEXT
 * DMA boundary guarantees at least five seconds after the panel turns on. */
typedef struct {
  uint32_t command;
  uint32_t expired;
  uint64_t deadline;
  uint64_t button_rearm;
  bool button_armed;
} BirdnetUiState;

static inline uint32_t birdnet_ui_command(uint32_t previous, bool on) {
  return ((previous + 2u) & ~1u) | (on ? 1u : 0u);
}

static inline void birdnet_ui_update(BirdnetUiState *s, uint32_t command,
                                     uint64_t samples, uint32_t sample_rate,
                                     uint32_t chunk_samples) {
  if (command != s->command) {
    s->command = command;
    s->expired = 0;
    s->deadline = (command & 1u) ? samples + 5ull * sample_rate + chunk_samples : 0;
  }
  if (s->deadline && samples >= s->deadline) {
    s->expired = s->command;
    s->deadline = 0;
  }
}

static inline bool birdnet_ui_button(BirdnetUiState *s, bool edge, bool high,
                                     uint64_t samples, uint32_t chunk_samples) {
  if (s->button_armed && edge) {
    s->button_armed = false;
    s->button_rearm = samples + chunk_samples;
    return true;
  }
  if (!high && samples >= s->button_rearm) s->button_armed = true;
  return false;
}

/* SLEEP_READY=2 keeps CM55M awake for bounded file export. Publish capture
 * descriptors without a PMU wake pulse; CM55M acknowledges discarded windows.
 * Both cores must be built from the same protocol header. */
enum { BN_MB_ACTIVE = 0, BN_MB_SLEEP = 1, BN_MB_MAINTENANCE = 2 };

typedef struct {
  uint32_t sequence;
  uint32_t ring_base;
  uint32_t ring_samples;
  uint32_t start;
  uint32_t samples;
  uint32_t end_low;
  uint32_t end_high;
  uint32_t capture_generation;
} BirdnetAudioDescriptor;

typedef struct {
  uint64_t capture_end;
  uint32_t ack;
} BirdnetProgressWatchdog;

/* Feed only after BOTH cores made progress, never for unrelated interrupts.
 * The hardware countdown, not sample count, detects a stopped DMA clock. */
static inline bool birdnet_watchdog_progress(BirdnetProgressWatchdog *state,
                                            uint64_t capture_end, uint32_t ack,
                                            bool fault) {
  if (fault || capture_end <= state->capture_end || !ack || ack == state->ack) return false;
  state->capture_end = capture_end;
  state->ack = ack;
  return true;
}

static inline uint64_t birdnet_audio_end(const BirdnetAudioDescriptor *d) {
  return ((uint64_t)d->end_high << 32) | d->end_low;
}

static inline bool birdnet_audio_descriptor_valid(const BirdnetAudioDescriptor *d,
                                                 uint32_t expected_samples,
                                                 uint32_t live_generation) {
  const uint64_t end_address = (uint64_t)d->ring_base + (uint64_t)d->ring_samples * 2u;
  return d->sequence != 0 && !(d->sequence & 1u) &&
         d->samples == expected_samples && d->ring_samples > d->samples &&
         d->start < d->ring_samples && (d->ring_base & 31u) == 0 &&
         d->ring_base >= 0x36000000u && end_address <= 0x36060000ull &&
         birdnet_audio_end(d) >= d->samples &&
         live_generation == d->capture_generation;
}

static inline uint32_t birdnet_audio_first_samples(const BirdnetAudioDescriptor *d) {
  const uint32_t remaining = d->ring_samples - d->start;
  return remaining < d->samples ? remaining : d->samples;
}

static inline uint64_t birdnet_audio_next_deadline(uint64_t end, uint32_t step) {
  /* After a late snapshot, do not compress the next interval to "catch up":
   * preserve the configured step (and overlap). Re-anchor to capture end. */
  return end + step;
}

static inline uint64_t birdnet_audio_skipped_windows(uint64_t previous_end,
    uint64_t end, uint32_t window, uint32_t step) {
  if (!step || end < previous_end) return 0;
  /* Cold start needs a complete window, not three one-second invocations. */
  const uint64_t expected = previous_end ? previous_end + step
      : (window > step ? window : step);
  return end > expected ? (end - expected) / step : 0;
}

/* Paired clock diagnostics. SDK owns PM entry and warm-wake clocks.
 * Profile source/divider and gate masks apply only after observed PS_SCL.
 * PDM reference stays unchanged; preserve DMA2, SRAM, buses and GPIO.
 * The helper trace occupies cache lines separate from the live DMA ring.
 * Sparse profiles observe the short entry transition with a bounded poll.
 */
enum { BN_CLOCK_MAGIC = 0x424e4302u, BN_CLOCK_WORDS = 22 };
enum { BN_UNUSED_GATE_A = 0xfd80u, BN_UNUSED_GATE_B = 0x773fu };
enum { BN_SLEEP_DIV_MASK = 0x07f7u, BN_SLEEP_TARGET_BITS = 0x0171u };
typedef struct __attribute__((aligned(32))) {
  uint32_t magic, sequence, window_sequence, capture_low, capture_high;
  uint32_t state_before, state_after, dbsel;
  uint32_t regs[BN_CLOCK_WORDS];
  uint32_t io[8];
} BirdnetClockTrace;
typedef char BirdnetClockTraceSize[(sizeof(BirdnetClockTrace) == 160) ? 1 : -1];
static const uint16_t bn_clock_offsets[BN_CLOCK_WORDS] = { 0x8, 0x10, 0x14, 0x18, 0x1c, 0x20, 0x24, 0x28, 0x30, 0x38, 0x40, 0x44, 0x48, 0x700, 0x704, 0x710, 0x714, 0x718, 0x71c, 0x720, 0x724, 0x728 };

/* The DMA destination is the next sample to write. A copied descriptor must
 * remain wholly behind it, within the spare ring span, before AND after copy.
 * Reject malformed, odd or out-of-ring addresses; allow the transient one-past
 * ring destination at the hardware wrap boundary. */
static __attribute__((noinline, unused)) uint32_t birdnet_circular_ahead(uint32_t base, uint32_t ring,
    uint32_t start, uint32_t samples, uint32_t dest) {
  if (!ring || samples>=ring || start>=ring || dest<base || ((dest-base)&1u)) return UINT32_MAX;
  uint32_t head=(dest-base)/2u;
  if (head>ring) return UINT32_MAX;
  if (head==ring) head=0;
  const uint32_t end=(start+samples)%ring;
  const uint32_t ahead=(head+ring-end)%ring;
  return ahead<ring-samples ? ahead : UINT32_MAX;
}

#endif
