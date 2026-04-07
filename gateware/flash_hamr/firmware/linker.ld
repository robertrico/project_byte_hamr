/* linker.ld — PicoRV32 memory layout for Flash Hamr
 *
 * IMEM: 32 KB at 0x00000000 (instruction BRAM, initialized via $readmemh)
 * DMEM: 16 KB at 0x10000000 (data BRAM, stack + heap + FatFS work area)
 */

ENTRY(_start)

MEMORY
{
    IMEM (rx)  : ORIGIN = 0x00000000, LENGTH = 32K
    DMEM (rwx) : ORIGIN = 0x10000000, LENGTH = 16K
}

SECTIONS
{
    /* Code and read-only data in IMEM */
    .init : {
        KEEP(*(.init))
    } > IMEM

    .text : {
        *(.text .text.*)
    } > IMEM

    .rodata : {
        *(.rodata .rodata.*)
    } > IMEM

    /* Initialized data: stored in IMEM (LMA), copied to DMEM at startup */
    _data_lma = LOADADDR(.data);
    .data : {
        _data_start = .;
        *(.data .data.*)
        _data_end = .;
    } > DMEM AT > IMEM

    /* Uninitialized data in DMEM — zeroed by crt0 */
    .bss (NOLOAD) : {
        _bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        _bss_end = .;
    } > DMEM

    /* Stack at top of DMEM */
    _stack_top = ORIGIN(DMEM) + LENGTH(DMEM);

    /* Discard unwanted sections */
    /DISCARD/ : {
        *(.comment)
        *(.note.*)
        *(.eh_frame*)
    }
}
