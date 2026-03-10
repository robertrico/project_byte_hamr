    .syntax unified
    .cpu cortex-m0plus
    .thumb

@ ============================================================
@ sp_asm.S — Timing-critical assembly helpers for PicoPort
@ ============================================================
@
@ Only functions where cycle-level timing matters live here.
@ Everything else is in C (sp_cmd.c, sp_encode.c, sp_decode_pkt.c).

    .section .text

@ ============================================================
@ Constants
@ ============================================================
.equ SIO_BASE,          0xD0000000
.equ SIO_GPIO_IN,       0x04
.equ SIO_GPIO_OE_SET,   0x24
.equ SIO_GPIO_OE_CLR,   0x28

.equ PIN_ACK,           9       @ GP9 — open-drain ACK output
.equ PIN_REQ,           5       @ GP5 — PH0/REQ input
.equ PIN_PHI_BASE,      3       @ GP3-6 — phase pins

.equ PIO_FSTAT,         0x004
.equ PIO_RXF0,          0x020

.equ IDLE_WORDS,        8       @ consecutive no-edge words = end-of-packet
                                @ 8 × 32 × 480ns ≈ 123µs of silence
.equ SAMPLE_BUF_SIZE,   2048

@ ============================================================
@ ack_assert — Drive ACK LOW (set GP9 as output, value=0)
@ ============================================================
    .global ack_assert
    .type ack_assert, %function
    .thumb_func
ack_assert:
    ldr     r0, =SIO_BASE
    movs    r1, #1
    lsls    r1, r1, #PIN_ACK
    str     r1, [r0, #SIO_GPIO_OE_SET]
    bx      lr

@ ============================================================
@ ack_deassert — Release ACK to hi-Z (set GP9 as input)
@ ============================================================
    .global ack_deassert
    .type ack_deassert, %function
    .thumb_func
ack_deassert:
    ldr     r0, =SIO_BASE
    movs    r1, #1
    lsls    r1, r1, #PIN_ACK
    str     r1, [r0, #SIO_GPIO_OE_CLR]
    bx      lr

@ ============================================================
@ read_phases — Read GP3-6, unscramble to PH3:PH2:PH1:PH0
@ ============================================================
@ Returns: r0 = logical phases (0x00-0x0F)
    .global read_phases
    .type read_phases, %function
    .thumb_func
read_phases:
    ldr     r0, =SIO_BASE
    ldr     r0, [r0, #SIO_GPIO_IN]
    lsrs    r0, r0, #PIN_PHI_BASE
    movs    r1, #0x0F
    ands    r0, r0, r1
    @ Raw bit order: [PH1, PH0, PH3, PH2] → logical [PH3, PH2, PH1, PH0]
    ldr     r1, =phase_lut
    ldrb    r0, [r1, r0]
    bx      lr

    .ltorg

@ ============================================================
@ req_wait_rise — Wait for REQ (GP5) HIGH, with timeout
@ ============================================================
@ r0 = timeout in microseconds (approximate)
@ Returns: r0 = 0 on success, 1 on timeout
    .global req_wait_rise
    .type req_wait_rise, %function
    .thumb_func
req_wait_rise:
    push    {r4, r5, lr}
    lsls    r4, r0, #2             @ rough loop count (~4 cycles/iter)
    ldr     r5, =SIO_BASE
rwr_loop:
    ldr     r0, [r5, #SIO_GPIO_IN]
    movs    r1, #1
    lsls    r1, r1, #PIN_REQ
    tst     r0, r1
    bne     rwr_ok
    subs    r4, r4, #1
    bne     rwr_loop
    movs    r0, #1                  @ timeout
    pop     {r4, r5, pc}
rwr_ok:
    movs    r0, #0
    pop     {r4, r5, pc}

@ ============================================================
@ req_wait_fall — Wait for REQ (GP5) LOW, with timeout
@ ============================================================
@ r0 = timeout in microseconds (approximate)
@ Returns: r0 = 0 on success, 1 on timeout
    .global req_wait_fall
    .type req_wait_fall, %function
    .thumb_func
req_wait_fall:
    push    {r4, r5, lr}
    lsls    r4, r0, #2
    ldr     r5, =SIO_BASE
rwf_loop:
    ldr     r0, [r5, #SIO_GPIO_IN]
    movs    r1, #1
    lsls    r1, r1, #PIN_REQ
    tst     r0, r1
    beq     rwf_ok
    subs    r4, r4, #1
    bne     rwf_loop
    movs    r0, #1                  @ timeout
    pop     {r4, r5, pc}
rwf_ok:
    movs    r0, #0
    pop     {r4, r5, pc}

    .ltorg

@ ============================================================
@ capture_samples — Collect raw PIO RX samples until idle
@ ============================================================
@ Called from C. Uses g_pio_rx_hw and sample_buf from C globals.
@ Returns: r0 = sample count (0 = nothing captured)
@
@ Detects end-of-packet by content: IDLE_WORDS consecutive
@ "no-edge" words (all-1s = 0xFFFFFFFF or all-0s = 0x00000000).
@ WRDATA can idle at either level after last FM toggle.

    .global capture_samples
    .type capture_samples, %function
    .thumb_func
capture_samples:
    push    {r4-r7, lr}

    ldr     r0, =g_pio_rx_hw
    ldr     r4, [r0]                @ r4 = PIO RX base (pio_hw_t *)
    movs    r5, #0                  @ sample count
    ldr     r6, =sample_buf
    movs    r7, #0                  @ consecutive idle word count

cap_poll:
    @ Wait for FIFO data (bit 8 = RXEMPTY for SM0)
    ldr     r0, [r4, #PIO_FSTAT]
    movs    r1, #1
    lsls    r1, r1, #8
    tst     r0, r1
    bne     cap_poll

    @ Read sample word
    ldr     r0, [r4, #PIO_RXF0]

    @ Check if no-edge word: all-0s or all-1s
    @ Used ONLY for end-of-packet detection — word is still stored.
    cmp     r0, #0
    beq     cap_idle                @ all-0s = pin stuck LOW
    adds    r1, r0, #1
    beq     cap_idle                @ all-1s (0xFFFFFFFF + 1 = 0) = pin stuck HIGH
    movs    r7, #0                  @ has edges — reset idle counter
    b       cap_store

cap_idle:
    adds    r7, r7, #1
    cmp     r7, #IDLE_WORDS
    bge     cap_done                @ enough consecutive idle → packet over

cap_store:
    @ Store ALL words (idle + active) to preserve timing for fm_decode
    ldr     r1, =SAMPLE_BUF_SIZE
    cmp     r5, r1
    bge     cap_done
    lsls    r1, r5, #2
    str     r0, [r6, r1]
    adds    r5, r5, #1
    b       cap_poll

cap_done:
    mov     r0, r5                  @ return sample count
    pop     {r4-r7, pc}

    .ltorg

@ ============================================================
@ Read-only data
@ ============================================================
    .section .rodata

    @ Phase unscramble LUT: raw GP[6:3] → logical PH3:PH2:PH1:PH0
    @ Raw: bit3=GP6=PH1, bit2=GP5=PH0, bit1=GP4=PH3, bit0=GP3=PH2
    .align 2
    .global phase_lut
phase_lut:
    .byte 0x00, 0x04, 0x08, 0x0C, 0x01, 0x05, 0x09, 0x0D
    .byte 0x02, 0x06, 0x0A, 0x0E, 0x03, 0x07, 0x0B, 0x0F

@ ============================================================
@ BSS — shared buffers (accessed from C via sp_proto.h externs)
@ ============================================================
    .section .bss
    .align 4

    .global sample_buf
sample_buf:
    .space  SAMPLE_BUF_SIZE * 4

    .global pkt_bytes
pkt_bytes:
    .space  64                      @ PKT_MAX

    .global tx_buf
tx_buf:
    .space  700                     @ TX_BUF_MAX

    @ PIO base address — written by C, read by capture_samples
    .global g_pio_rx_hw
g_pio_rx_hw:
    .space  4
