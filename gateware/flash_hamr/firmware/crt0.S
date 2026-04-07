# crt0.S — PicoRV32 startup code
#
# - Sets stack pointer to top of DMEM
# - Zeroes BSS
# - Calls main()
# - Loops forever if main returns

.section .init, "ax"
.global _start

_start:
    # Set stack pointer to top of DMEM (0x10000000 + 16KB)
    lui   sp, %hi(_stack_top)
    addi  sp, sp, %lo(_stack_top)

    # Zero BSS section
    la    a0, _bss_start
    la    a1, _bss_end
bss_loop:
    bge   a0, a1, bss_done
    sw    zero, 0(a0)
    addi  a0, a0, 4
    j     bss_loop
bss_done:

    # Copy .data from ROM (IMEM) to RAM (DMEM) if needed
    la    a0, _data_lma       # source (in IMEM)
    la    a1, _data_start     # dest (in DMEM)
    la    a2, _data_end
data_loop:
    bge   a1, a2, data_done
    lw    a3, 0(a0)
    sw    a3, 0(a1)
    addi  a0, a0, 4
    addi  a1, a1, 4
    j     data_loop
data_done:

    # Call main
    jal   ra, main

    # If main returns, spin forever
halt:
    j     halt
