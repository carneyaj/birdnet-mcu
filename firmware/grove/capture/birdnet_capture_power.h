// Private CM55S power policy. Included after capture state declarations.
// Preserves test79 clock ownership, SDK transition order, cache and scalar guards.
#pragma once
static uint32_t g_initial_cpd_request, g_initial_cache_controls;
static uint32_t g_scalar_guard_installed;
static uint32_t scalar_epu_proof(void);
static inline uint32_t bn_clock_state(void) {
  return (*(volatile const uint32_t *)(HX_PMU_BASE + 0xe8u) >> 16) & 0xffu;
}
/* The sleep mailbox arrives just BEFORE main enters PMU SCL. With three-second
 * DMA checkpoints, waiting for the next DMA IRQ would leave fast clocks on for
 * almost the whole sleep. Observe that short transition once, with interrupts
 * enabled and a bounded budget, then apply the owned gates/divider immediately.
 * A timeout is visible in the sleep trace and fails USB qualification. */
static inline uint32_t bn_clock_state(void);
static bool g_sleep_entry_observed;
static uint32_t g_sleep_entry_success, g_sleep_entry_timeout;
static void observe_sleep_entry(uint32_t ready) {
  if (ready!=BN_MB_SLEEP || g_snapshot_wake_pending) {
    g_sleep_entry_observed=false; return;
  }
  if (g_sleep_entry_observed) return;
  g_sleep_entry_observed=true;
  for (unsigned budget=0;budget<50000u;++budget) {
    if (bn_clock_state()==PMU_PS_STATE_SCL) { ++g_sleep_entry_success; return; }
    uint32_t current=0;
    hx_drv_mb_get_sharedata(BN_MB_SLEEP_READY,&current);
    if (current!=BN_MB_SLEEP || g_snapshot_wake_pending) return;
  }
  ++g_sleep_entry_timeout;
}

static uint32_t g_deep_wfi_count, g_last_deep_scr;
/* Change only source/divider fields after the SDK has entered verified SCL.
 * Restore before the helper wakes main, never over a completed SDK PLL wake. */
static bool g_sleep_divider_active;
static uint32_t g_sleep_saved_divider;
static inline uint32_t sleep_divider_read(void) {
  return *(volatile const uint32_t *)0x56100008u;
}
static bool __attribute__((target("general-regs-only"), optimize("Os"), noinline))
sleep_divider_write(uint32_t expected, uint32_t value) {
  // Exact SDK non-PLL switching order: low selectors, upper selectors, divider.
  // The low selectors alone do not reproduce hx_drv_scu_set_pdlsc_lscclk_cfg.
  // Only RC96M48M <-> XTAL24M is supported here; PLL transitions stay with SDK.
  static const uint32_t stages[3]={7u,0x700u,0xf0u};
  for(unsigned i=0;i<3;++i) {
    const uint32_t current=sleep_divider_read();
    if(bn_clock_state()!=PMU_PS_STATE_SCL ||
        (current&BN_SLEEP_DIV_MASK)!=(expected&BN_SLEEP_DIV_MASK)) return false;
    const uint32_t next=(current&~stages[i])|(value&stages[i]);
    *(volatile uint32_t *)0x56100008u=next;
    __DSB(); __ISB();
    expected=next;
  }
  return bn_clock_state()==PMU_PS_STATE_SCL;
}
static void restore_sleep_divider_locked(void) {
  if (!g_sleep_divider_active) return;
  const uint32_t current=sleep_divider_read();
  if (bn_clock_state()==PMU_PS_STATE_SCL &&
      (current & BN_SLEEP_DIV_MASK)==BN_SLEEP_TARGET_BITS) {
    (void)sleep_divider_write(current, (current & ~BN_SLEEP_DIV_MASK)|g_sleep_saved_divider);
    __DSB(); __ISB();
  }
  g_sleep_divider_active=false;
}
static void restore_sleep_divider(void) {
  const uint32_t saved=__get_PRIMASK(); __disable_irq();
  restore_sleep_divider_locked(); __set_PRIMASK(saved);
}
static void service_sleep_divider_locked(uint32_t ready) {
  if (ready!=BN_MB_SLEEP || g_snapshot_wake_pending || bn_clock_state()!=PMU_PS_STATE_SCL) {
    restore_sleep_divider(); return;
  }
  const uint32_t current=sleep_divider_read();
  if (!g_sleep_divider_active) {
    if ((current & BN_SLEEP_DIV_MASK)!=0x404u) return;
    g_sleep_saved_divider=current & BN_SLEEP_DIV_MASK;
    g_sleep_divider_active=true;
  }
  const uint32_t owned=current & BN_SLEEP_DIV_MASK;
  if (owned!=0x404u && owned!=BN_SLEEP_TARGET_BITS) {
    g_sleep_divider_active=false; return;
  }
  if (owned!=BN_SLEEP_TARGET_BITS) {
    if (!sleep_divider_write(current, (current & ~BN_SLEEP_DIV_MASK)|BN_SLEEP_TARGET_BITS)) g_sleep_divider_active=false;
    __DSB(); __ISB();
  }
  if (bn_clock_state()!=PMU_PS_STATE_SCL) restore_sleep_divider();
}
static void service_sleep_divider(uint32_t ready) {
  const uint32_t saved=__get_PRIMASK(); __disable_irq();
  service_sleep_divider_locked(ready); __set_PRIMASK(saved);
}

/* Exact SDK SCU gate layout: only profile-owned enable bits may change.
 * Keep PDM, DMA2, SRAM/buses, GPIO, mailbox and watchdog clocks.
 * Sources/dividers and SDK PM policy are untouched. Save only owned bits;
 * restoring the whole register could overwrite unrelated warm-boot changes.
 */
static bool g_unused_sleep_gates_active;
static uint32_t g_unused_sleep_saved_a, g_unused_sleep_saved_b;
static inline uint32_t unused_gate_read(unsigned word) {
  return *(volatile const uint32_t *)(0x56100040u + word * 4u);
}
static inline void unused_gate_write(unsigned word, uint32_t value) {
  *(volatile uint32_t *)(0x56100040u + word * 4u) = value;
}
static void restore_unused_sleep_clocks(void) {
  if (!g_unused_sleep_gates_active) return;
  unused_gate_write(0, (unused_gate_read(0) & ~BN_UNUSED_GATE_A) | g_unused_sleep_saved_a);
  unused_gate_write(1, (unused_gate_read(1) & ~BN_UNUSED_GATE_B) | g_unused_sleep_saved_b);
  __DSB();
  g_unused_sleep_gates_active = false;
}
static void service_unused_sleep_clocks(uint32_t sleep_ready) {
  if (sleep_ready != BN_MB_SLEEP || g_snapshot_wake_pending ||
      bn_clock_state() != PMU_PS_STATE_SCL) {
    restore_unused_sleep_clocks();
    return;
  }
  const uint32_t a = unused_gate_read(0), b = unused_gate_read(1);
  if (!g_unused_sleep_gates_active) {
    g_unused_sleep_saved_a = a & BN_UNUSED_GATE_A;
    g_unused_sleep_saved_b = b & BN_UNUSED_GATE_B;
    g_unused_sleep_gates_active = true;
  }
  if (a & BN_UNUSED_GATE_A) unused_gate_write(0, a & ~BN_UNUSED_GATE_A);
  if (b & BN_UNUSED_GATE_B) unused_gate_write(1, b & ~BN_UNUSED_GATE_B);
  __DSB();
  // Cover an external PMU wake racing these bounded writes. Normal helper
  // wakes restore explicitly BEFORE the mailbox IRQ or PMU wake pulse.
  if (bn_clock_state() != PMU_PS_STATE_SCL) restore_unused_sleep_clocks();
}

// SDK initialization may use MVE. Once initialization is complete, this
// nonreturning task and its interrupt paths must not use FP/MVE again.
// Disabling CP10/CP11 makes accidental use fault instead of silently losing state.
static void __attribute__((target("general-regs-only"), optimize("Os"), noinline)) scalar_epu_guard(void) {
  const uint32_t primask=__get_PRIMASK();
  __disable_irq();
  if (!(FPU->FPCCR&1u)) {
    // No outstanding lazy stack frame may refer to the discarded context.
    __set_CONTROL(__get_CONTROL()&~12u); __ISB();
    SCB->CPACR &= ~0x00f00000u;
    ICB->CPPWR |= 0x00500000u; // Matching CP10/CP11 state-unneeded bits.
    __DSB(); __ISB();
    g_scalar_guard_installed=1;
  } else {
    g_capture_fault=true;
  }
  __set_PRIMASK(primask);
}

static uint32_t __attribute__((target("general-regs-only"), optimize("Os"), noinline)) scalar_epu_proof(void) {
  return ((SCB->CPACR&0x00f00000u)==0) |
      (((__get_CONTROL()&12u)==0)<<1) |
      (((ICB->CPPWR&0x00500000u)==0x00500000u)<<2) |
      (((FPU->FPCCR&1u)==0)<<3) | (g_scalar_guard_installed<<4);
}


static void capture_power_init(void) {
  // Read/write private power controls only at startup, before any sleep bus
  // gates. Preserve EPU policy; request core retention and allow cache OFF.
  // Clock enable stays on below, allowing the processor to execute WFI.
  PWRMODCTL->CPDLPSTATE =
      (PWRMODCTL->CPDLPSTATE & ~(PWRMODCTL_CPDLPSTATE_CLPSTATE_Msk |
                                PWRMODCTL_CPDLPSTATE_RLPSTATE_Msk)) | 0x302u;
  g_initial_cpd_request = PWRMODCTL->CPDLPSTATE;
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  __DSB(); __ISB();

  // Arm M55 TRM 6.5.2 / 7.8: stop allocation, clean dirty data, then
  // clear MSCR active hints. SDK hx_enable_* performs those operations.
  // No interrupt may modify cached data while the clean is in progress.
  const uint32_t cache_init_primask = __get_PRIMASK();
  __disable_irq();
  hx_enable_DCache(0);
  hx_enable_ICache(0);
  __DSB(); __ISB();
  // Read private controls at startup, before audio/SCL. Keep the trace/ring
  // layout unchanged. CPWRDN is an observation, not proof of board rail OFF.
  g_initial_cache_controls =
      ((MEMSYSCTL->MSCR >> 12) & 3u) |
      (((SCB->CCR >> 16) & 3u) << 2) |
      (((MEMSYSCTL->MSCR >> 17) & 1u) << 4);
  __set_PRIMASK(cache_init_primask);

}

static void capture_power_service(uint32_t ready) {
  observe_sleep_entry(ready);
  service_unused_sleep_clocks(ready);
  service_sleep_divider(ready);
}

static inline void capture_power_wait(void) {
      const uint32_t saved_scr=SCB->SCR;
      if (bn_clock_state()==PMU_PS_STATE_SCL) {
        SCB->SCR=saved_scr|SCB_SCR_SLEEPDEEP_Msk;
        g_last_deep_scr=SCB->SCR;
        ++g_deep_wfi_count;
      }
      __DSB();
      __WFI();
      SCB->SCR=saved_scr;
      __DSB(); __ISB();
}
