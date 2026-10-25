/*
 * Linker script for STM32F401 / STM32F411 Black Pill (Node B)
 * RAM: 64 KB (compatible with both chips), Stack Top: 0x20010000
 * Flash: 256 KB @ 0x08000000
 */

ENTRY(Reset_Handler)

_estack = 0x20010000;    /* End of 64KB RAM */

MEMORY
{
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 256K
    RAM   (rw) : ORIGIN = 0x20000000, LENGTH = 64K
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
