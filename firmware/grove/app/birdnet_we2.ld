#include "WE2_device_addr.h"
#ifndef BIRDNET_PMU_HSC_SRAM_RETENTION_MASK
#define BIRDNET_PMU_HSC_SRAM_RETENTION_MASK 0xF
#endif
#ifndef BIRDNET_OPUS_ON_DEMAND
#define BIRDNET_OPUS_ON_DEMAND 0
#endif
#ifndef BIRDNET_BANK0_OFF
#define BIRDNET_BANK0_OFF 0
#endif
MEMORY
{
  CM55M_S_APP_ROM (rx) : ORIGIN = 0x10000000, LENGTH = 0x40000
  CM55M_S_APP_DATA (rwx) : ORIGIN = 0x30000000, LENGTH = 0x40000
  /* Keep CM55M out of the low secure alias space used by the boot chain and CM55S bring-up. */
#if BIRDNET_OPUS_ON_DEMAND
  CM55M_S_SRAM0 (rwx) : ORIGIN = 0x3404D000, LENGTH = 0x53000
  CM55M_S_CODEC (rwx) : ORIGIN = 0x340A0000, LENGTH = 0x40000
#else
  CM55M_S_SRAM0 (rwx) : ORIGIN = 0x3404D000, LENGTH = 0x93000
#endif
  CM55M_S_SRAM1 (rwx) : ORIGIN = 0x340E0000, LENGTH = 0x120000
}

__HEAP_SIZE = 0x7000;
__STACK_SIZE = 0x11000;

ENTRY(Reset_Handler)

SECTIONS
{
    .table : ALIGN(4)
    {
        FILL(0xff)
        __vectors_start__ = ABSOLUTE(.) ;
        KEEP(*(.vectors))
        *(.after_vectors*)

        . = ALIGN(32);
        __privileged_functions_start__ = .;
        *(privileged_functions)
        *(privileged_functions*)
        . = ALIGN(32);
        __privileged_functions_end__ = (. - 1);

        . = ALIGN(32);
        __syscalls_flash_start__ = .;
        *(freertos_system_calls)
        *(freertos_system_calls*)
        . = ALIGN(32);
        __syscalls_flash_end__ = (. - 1);
        __unprivileged_flash_start__ = .;
    } > CM55M_S_APP_ROM

    /* Keep the reusable frontend in the lowest app SRAM, outside zero/copy
       tables and below 0x34080000. Check actual PMU retention on hardware. */
    .frontend_cache (NOLOAD) : ALIGN(32)
    {
      KEEP(*(.frontend_cache))
      . = ALIGN(32);
    } > CM55M_S_SRAM0
    ASSERT(ADDR(.frontend_cache) + SIZEOF(.frontend_cache) <= 0x34080000,
           "Frontend cache exceeds low retention region")
    ASSERT(SIZEOF(.frontend_cache) == 0 || (BIRDNET_PMU_HSC_SRAM_RETENTION_MASK & 1),
           "Frontend cache requires SRAM bank 0 retention")

#if BIRDNET_DETECTION_AUDIO_CODEC == 2
    .audio0 (NOLOAD) : ALIGN(32)
    {
      __audio0_start__ = .;
      KEEP(*(.bss.audio_sram0))
      KEEP(*(.bss.audio_sram0.*))
      __audio0_end__ = .;
    } > CM55M_S_SRAM0

    /* Bootloader-loaded code, NOT part of the zeroed/retained audio buffers.
       The 256 KiB ITCM cannot also hold libopus. Keep existing buffers first. */
    .opus_text : ALIGN(32)
    {
      __opus_text_start__ = .;
      *libopus.a:*(.text* .rodata*)
      *birdnet_opus_codec.o(.text* .rodata*)
#if BIRDNET_OPUS_ON_DEMAND
    } > CM55M_S_CODEC
    ASSERT(ADDR(.opus_text) >= 0x340A0000 && ADDR(.opus_text) + SIZEOF(.opus_text) <= 0x340E0000,
           "On-demand Opus must fit the separate executable scratch region")
#else
    } > CM55M_S_SRAM0
    ASSERT((BIRDNET_PMU_HSC_SRAM_RETENTION_MASK & 1) &&
           (ADDR(.opus_text) + SIZEOF(.opus_text) <= 0x34080000 ||
            (BIRDNET_PMU_HSC_SRAM_RETENTION_MASK & 2)),
           "Opus code crosses a non-retained SRAM bank")
#endif
#endif

#if BIRDNET_DETECTION_AUDIO_CODEC == 2
    /* Include the linker's late branch veneers in the copied/checked payload. */
    __opus_text_end__ = ADDR(.opus_text) + SIZEOF(.opus_text);
#endif
#if BIRDNET_BANK0_OFF
    ASSERT(ADDR(.scratch_audio) <= 0x3407FFE0 &&
           ADDR(.scratch_audio) + SIZEOF(.scratch_audio) >= 0x34080000,
           "Bank-0 proof marker must be within disposable PCM snapshot")
#endif
    ASSERT(BIRDNET_PMU_HSC_SRAM_RETENTION_MASK != 0 || BIRDNET_BANK0_OFF,
           "Disabling bank 0 requires the explicit bootloader reload experiment")

    .text : ALIGN(4)
    {
       *(.text*)
       KEEP(*freertos*/tasks.o(.rodata*))
       . = ALIGN(4);
       KEEP(*(.init))
       KEEP(*(.fini));

       *crtbegin.o(.ctors)
       *crtbegin?.o(.ctors)
       *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
       *(SORT(.ctors.*))
       *(.ctors)

       *crtbegin.o(.dtors)
       *crtbegin?.o(.dtors)
       *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
       *(SORT(.dtors.*))
       *(.dtors)
       . = ALIGN(4);

       KEEP(*(.eh_frame*))
    } > CM55M_S_APP_ROM

#if BIRDNET_DETECTION_AUDIO_CODEC != 2
    .audio0 (NOLOAD) : ALIGN(32)
    {
      __audio0_start__ = .;
      KEEP(*(.bss.audio_sram0))
      KEEP(*(.bss.audio_sram0.*))
      __audio0_end__ = .;
    } > CM55M_S_SRAM0
#endif

#if BIRDNET_SKIP_ARENA_ZERO
    /* TFLM initializes its control allocations; the frontend and Ethos-U
       overwrite their tensors. Qualified separately with whole-arena poison. */
    .scratch_tensor (NOLOAD) : ALIGN(32)
    {
      KEEP(*(.tensor_arena))
      KEEP(*(.tensor_arena.*))
    } > CM55M_S_SRAM1
#endif
    .audio1 (NOLOAD) : ALIGN(32)
    {
      __audio1_start__ = .;
      KEEP(*(.bss.audio_sram1))
      KEEP(*(.bss.audio_sram1.*))
      KEEP(*(.tensor_arena))
      KEEP(*(.tensor_arena.*))
      __audio1_end__ = .;
    } > CM55M_S_SRAM1

    .rodata : ALIGN(4)
    {
        __rodata_start = .;
        KEEP(*(.birdnet_contract))
        KEEP(*(.rodata.pad))
        *(.rodata .rodata.* .constdata .constdata.*)
        __rodata_end = .;
    } > CM55M_S_APP_ROM

    .ARM.extab : ALIGN(4)
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > CM55M_S_APP_ROM

    .ARM.exidx : ALIGN(4)
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > CM55M_S_APP_ROM

    .copy.table :
    {
      . = ALIGN(4);
      __copy_table_start__ = .;
      LONG(LOADADDR(.data));
      LONG(ADDR(.data));
      LONG(SIZEOF(.data)/4);
      __copy_table_end__ = .;
    } > CM55M_S_APP_ROM

    .zero.table :
    {
      . = ALIGN(4);
      __zero_table_start__ = .;
      LONG(ADDR(.audio0));
      LONG(SIZEOF(.audio0)/4);
      LONG(ADDR(.audio1));
      LONG(SIZEOF(.audio1)/4);
      LONG(ADDR(.bss));
      LONG(SIZEOF(.bss)/4);
      __zero_table_end__ = .;
    } > CM55M_S_APP_ROM

    . = ALIGN(32);
    __unprivileged_flash_end__ = (. - 1);

    .data : ALIGN(4)
    {
       FILL(0xff)
       __data_start__ = .;
       . = ALIGN(32);
       __privileged_sram_start__ = .;
       *(privileged_data)
       *(privileged_data*)
       . = ALIGN(32);
       __privileged_sram_end__ = (. - 1);
       *(vtable)
       *(.data)
       *(.data.*)
       . = ALIGN(4);
       PROVIDE_HIDDEN (__preinit_array_start = .);
       KEEP(*(.preinit_array))
       PROVIDE_HIDDEN (__preinit_array_end = .);

       . = ALIGN(4);
       PROVIDE_HIDDEN (__init_array_start = .);
       KEEP(*(SORT(.init_array.*)))
       KEEP(*(.init_array))
       PROVIDE_HIDDEN (__init_array_end = .);

       . = ALIGN(4);
       PROVIDE_HIDDEN (__fini_array_start = .);
       KEEP(*(SORT(.fini_array.*)))
       KEEP(*(.fini_array))
       PROVIDE_HIDDEN (__fini_array_end = .);

       KEEP(*(.jcr*))
       . = ALIGN(4);
       __data_end__ = .;
    } > CM55M_S_APP_DATA

    .bss :
    {
      . = ALIGN(4);
      __bss_start__ = .;
      *(.bss)
      *(.bss.*)
      *(COMMON)
      . = ALIGN(4);
      __bss_end__ = .;
    } > CM55M_S_APP_DATA

    .noinit (NOLOAD): ALIGN(4)
    {
        _noinit = .;
        PROVIDE(__start_noinit_RAM = .);
        PROVIDE(__start_noinit_SRAM = .);
        *(.noinit*)
        . = ALIGN(4);
        _end_noinit = .;
        PROVIDE(__end_noinit_RAM = .);
        PROVIDE(__end_noinit_SRAM = .);
    } > CM55M_S_APP_DATA

    /* Explicitly initialized on every use (Opus initializes its complete state,
       and the SD writer only consumes its filled prefix). Not retained state. */
    .scratch_dtcm (NOLOAD): ALIGN(32)
    {
        *(.scratch_dtcm*)
    } > CM55M_S_APP_DATA

    /* PCM is consumed only after a complete, validated snapshot/capture. */
    .scratch_audio (NOLOAD): ALIGN(32)
    {
        *(.scratch_audio*)
    } > CM55M_S_SRAM0

    .heap (COPY) :
    {
      . = ALIGN(8);
      __HeapBase = .;
      PROVIDE(__HeapBase = .);
      end = __HeapBase;
      . = . + __HEAP_SIZE;
      . = ALIGN(8);
      __HeapLimit = .;
      PROVIDE(__HeapLimit = .);
    } > CM55M_S_APP_DATA

    .stack (ORIGIN(CM55M_S_APP_DATA) + LENGTH(CM55M_S_APP_DATA) - __STACK_SIZE) (COPY) :
    {
      . = ALIGN(8);
      __StackLimit = .;
      PROVIDE(__StackLimit = .);
      . = . + __STACK_SIZE;
      . = ALIGN(8);
      __StackTop = .;
      PROVIDE(__StackTop = .);
    } > CM55M_S_APP_DATA

    PROVIDE(__stack = __StackTop);

    ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed with stack")

    _image_start = LOADADDR(.text);
    _image_end = LOADADDR(.data) + SIZEOF(.data);
    _image_size = _image_end - _image_start;
}
