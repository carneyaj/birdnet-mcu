#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "WE2_core.h"

/*
 * Audio-only PMU warm boots retain enough C library state that repeated
 * Ethos-U driver mallocs exhaust the small newlib heap after about 83 wake
 * cycles. Keep the Ethos-U driver's tiny bare-metal allocations off the heap.
 */

void* __real_malloc(size_t size);
void __real_free(void* ptr);

enum {
  kSmallSlotCount = 8,
  kSmallSlotSize = 64,
};

static uint8_t g_small_slot_used[kSmallSlotCount];
static uint8_t g_small_slots[kSmallSlotCount][kSmallSlotSize] __attribute__((aligned(8)));

static int PoolIndexForPtr(void* ptr) {
  uintptr_t addr = (uintptr_t)ptr;
  uintptr_t base = (uintptr_t)&g_small_slots[0][0];
  uintptr_t end = (uintptr_t)&g_small_slots[kSmallSlotCount - 1][kSmallSlotSize];
  if (addr < base || addr >= end) {
    return -1;
  }
  uintptr_t offset = addr - base;
  if ((offset % kSmallSlotSize) != 0) {
    return -1;
  }
  return (int)(offset / kSmallSlotSize);
}

void* __wrap_malloc(size_t size) {
  if (size > 0 && size <= kSmallSlotSize) {
    for (int i = 0; i < kSmallSlotCount; ++i) {
      if (!g_small_slot_used[i]) {
        g_small_slot_used[i] = 1;
        memset(g_small_slots[i], 0, kSmallSlotSize);
        return g_small_slots[i];
      }
    }
  }
  return __real_malloc(size);
}

void __wrap_free(void* ptr) {
  if (ptr == NULL) {
    return;
  }
  int index = PoolIndexForPtr(ptr);
  if (index >= 0) {
    memset(g_small_slots[index], 0, kSmallSlotSize);
    g_small_slot_used[index] = 0;
    return;
  }
  __real_free(ptr);
}

typedef struct {
  volatile uint8_t count;
} BirdnetEthosuSemaphore;

static BirdnetEthosuSemaphore g_ethosu_global_sem;
static BirdnetEthosuSemaphore g_ethosu_driver_sem;
static uint8_t g_ethosu_global_sem_allocated;
static uint8_t g_ethosu_driver_sem_allocated;
static uint8_t g_ethosu_mutex_storage;

void* ethosu_mutex_create(void) {
  return &g_ethosu_mutex_storage;
}

void ethosu_mutex_destroy(void* mutex) {
  (void)mutex;
}

int ethosu_mutex_lock(void* mutex) {
  (void)mutex;
  return 0;
}

int ethosu_mutex_unlock(void* mutex) {
  (void)mutex;
  return 0;
}

void* ethosu_semaphore_create(void) {
  BirdnetEthosuSemaphore* sem = NULL;
  if (!g_ethosu_global_sem_allocated) {
    sem = &g_ethosu_global_sem;
    g_ethosu_global_sem_allocated = 1;
  } else {
    sem = &g_ethosu_driver_sem;
    g_ethosu_driver_sem_allocated = 1;
  }
  sem->count = 0;
  return sem;
}

void ethosu_semaphore_destroy(void* sem) {
  if (sem == &g_ethosu_driver_sem) {
    g_ethosu_driver_sem.count = 0;
    g_ethosu_driver_sem_allocated = 0;
  }
}

int ethosu_semaphore_take(void* sem) {
  BirdnetEthosuSemaphore* s = (BirdnetEthosuSemaphore*)sem;
  while (s->count == 0) {
    __WFE();
  }
  s->count = 0;
  return 0;
}

int ethosu_semaphore_give(void* sem) {
  BirdnetEthosuSemaphore* s = (BirdnetEthosuSemaphore*)sem;
  s->count = 1;
  __SEV();
  return 0;
}
