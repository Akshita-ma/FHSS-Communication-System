/*
 * Linker script for STM32F103C8T6 Blue Pill (Node A)
 * RAM: 20 KB @ 0x20000000, Stack Top: 0x20005000
 * Flash: 64 KB @ 0x08000000
 */

ENTRY(Reset_Handler)

_estack = 0x20005000;    /* End of 20KB RAM */

MEMORY
{
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 64K
    RAM   (rw) : ORIGIN = 0x20000000, LENGTH = 20K
}

SECTIONS
{
    /* The vector table goes first into FLASH */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > FLASH

    /* Program code and constants */
    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.rodata)
        *(.rodata*)
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)
        . = ALIGN(4);
        _etext = .;
    } > FLASH

    /* Initialized data placed in RAM, loaded from FLASH */
    _sidata = LOADADDR(.data);

    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FLASH

    /* Uninitialized data in RAM */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > RAM

    /* Discard unused sections */
    /DISCARD/ :
    {
        *(.comment)
        *(.ARM.attributes)
    }
}
