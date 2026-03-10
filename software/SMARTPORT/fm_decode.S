    .syntax unified
    .cpu cortex-m0plus
    .thumb

    .section .text

@ ============================================================
@ fm_decode — Decode oversampled FM pin data into packet bytes
@ ============================================================
@
@ C prototype:
@   int fm_decode(const uint32_t *samples, int count,
@                 uint8_t *pkt_out, int max_pkt);
@
@ Input: raw pin samples at ~8x oversampling (480ns/sample).
@ One bit cell ≈ 8.33 samples. An edge = transition in samples.
@
@ Algorithm:
@   1. XOR consecutive samples to find edges
@   2. Count samples between edges
@   3. Each edge produces: floor(gap / 8) zero bits, then one 1 bit
@      (because each edge IS a 1-bit; silence between = 0-bits)
@   4. After enough 1-bits, hunt for C3 to establish framing
@   5. Accumulate into bytes, autopush to output
@
@ This re-syncs on every edge, immune to crystal drift.
@
@ Stack frame:
@   [sp, #0]  = prev_sample (0 or 1)
@   [sp, #4]  = state (0=HUNT_SYNC, 1=HUNT_C3, 2=FRAMED)
@   [sp, #8]  = sliding / byte_acc
@   [sp, #12] = byte_bits
@   [sp, #16] = zero_run
@   [sp, #20] = ones_run
@   [sp, #24] = pkt_start
@   [sp, #28] = max_pkt
@   [sp, #32] = pkt_out current
@   [sp, #36] = gap_counter (samples since last edge)
#define OFF_PREV     0
#define OFF_STATE    4
#define OFF_ACC      8
#define OFF_BBITS    12
#define OFF_ZRUN     16
#define OFF_ONES     20
#define OFF_PSTART   24
#define OFF_MAX      28
#define OFF_PKT      32
#define OFF_GAP      36
#define FRAME_SIZE   40

@ Threshold: bit cell ~8.33 samples. Use 5 as half-cell threshold.
@ gap < 5  → noise, ignore (shouldn't happen with clean signal)
@ 5 ≤ gap < 13 → one 1-bit (edge within one cell)
@ 13 ≤ gap < 21 → one 0-bit then one 1-bit
@ 21 ≤ gap < 29 → two 0-bits then one 1-bit
@ General: zero_count = (gap - 5) / 8, then one 1-bit
@ Simplified: zero_count = (gap + 3) / 8 - 1
@   gap=8  → (11)/8-1 = 0 zeros + 1 one  ✓
@   gap=16 → (19)/8-1 = 1 zero + 1 one   ✓
@   gap=24 → (27)/8-1 = 2 zeros + 1 one  ✓
@ Correct formula: cells = gap >> 3, zeros = cells - 1 (min 0)
@ The edge itself occupies one cell, so subtract 1.
@   gap=8  → cells=1 → 0 zeros, 1 one ✓
@   gap=9  → cells=1 → 0 zeros, 1 one ✓
@   gap=17 → cells=2 → 1 zero, 1 one  ✓
@   gap=26 → cells=3 → 2 zeros, 1 one ✓
@   gap=62 → cells=7 → 6 zeros, 1 one ✓  ($81 = 10000001)

    .global fm_decode
    .type fm_decode, %function
    .thumb_func
fm_decode:
    push    {r4-r7, lr}
    sub     sp, sp, #FRAME_SIZE

    movs    r4, #1
    str     r4, [sp, #OFF_PREV]     @ wrdata idles HIGH — start at 1
    movs    r4, #0
    str     r4, [sp, #OFF_STATE]
    str     r4, [sp, #OFF_ACC]
    str     r4, [sp, #OFF_BBITS]
    str     r4, [sp, #OFF_ZRUN]
    str     r4, [sp, #OFF_ONES]
    str     r2, [sp, #OFF_PSTART]
    str     r3, [sp, #OFF_MAX]
    str     r2, [sp, #OFF_PKT]
    str     r4, [sp, #OFF_GAP]

    @ r0 = samples ptr, r1 = word count

word_loop:
    cmp     r1, #0
    ble     done
    ldr     r4, [sp, #OFF_ZRUN]
    cmp     r4, #4
    bge     done

    ldm     r0!, {r4}               @ load sample word
    subs    r1, r1, #1
    movs    r5, #31                 @ bit position

sample_loop:
    cmp     r5, #0
    blt     word_loop
    ldr     r6, [sp, #OFF_ZRUN]
    cmp     r6, #4
    bge     done

    @ Extract sample bit
    movs    r6, r4
    lsrs    r6, r6, r5
    movs    r7, #1
    ands    r6, r6, r7              @ r6 = current sample

    @ Edge detect
    ldr     r7, [sp, #OFF_PREV]
    str     r6, [sp, #OFF_PREV]
    eors    r7, r7, r6              @ r7 = edge (0 or 1)

    subs    r5, r5, #1

    @ Increment gap counter
    ldr     r6, [sp, #OFF_GAP]
    adds    r6, r6, #1
    str     r6, [sp, #OFF_GAP]

    cmp     r7, #0
    beq     sample_loop             @ no edge, keep counting

    @ --- Edge detected ---
    @ gap = samples since last edge (in r6)
    @ Noise filter: reject glitch edges (gap < 6) but ONLY after
    @ framing is established.  During sync hunting (state 0,1),
    @ every edge must be seen or we may miss the $FF/$C3 preamble.
    @ Min real gap ~8.33 samples (one bit cell at 480ns/sample).
    @ Threshold 6 catches most noise while preserving real edges.
    ldr     r7, [sp, #OFF_STATE]
    cmp     r7, #2
    bne     edge_accept             @ not framed yet — accept all edges
    cmp     r6, #6
    blt     sample_loop             @ framed + short gap = noise, skip

edge_accept:
    @ Valid edge — emit floor(gap/8) zero bits, then one 1 bit
    @ Reset gap counter
    movs    r7, #0
    str     r7, [sp, #OFF_GAP]

    @ r6 = gap. cells = gap >> 3. zeros = cells - 1 (min 0).
    @ One cell (~8 samples)   → 0 zeros + 1 one
    @ Two cells (~16 samples) → 1 zero  + 1 one
    @ Seven cells (~58)       → 6 zeros + 1 one
    lsrs    r6, r6, #3             @ r6 = total cells
    subs    r6, r6, #1             @ r6 = zero count (cells - 1)
    ble     emit_one               @ 0 or fewer → just emit the 1-bit

emit_zeros:
    movs    r7, #0                  @ edge bit = 0
    bl      push_bit
    subs    r6, r6, #1
    bne     emit_zeros

emit_one:
    movs    r7, #1                  @ edge bit = 1
    bl      push_bit
    b       sample_loop

@ ============================================================
@ push_bit — Process one decoded FM bit (in r7: 0 or 1)
@ ============================================================
push_bit:
    push    {lr}
    ldr     r3, [sp, #OFF_STATE + 4]   @ +4 for extra push

    cmp     r3, #0
    beq     pb_hunt_sync
    cmp     r3, #1
    beq     pb_hunt_c3
    b       pb_framed

pb_hunt_sync:
    cmp     r7, #0
    beq     pb_sync_reset
    ldr     r3, [sp, #OFF_ONES + 4]
    adds    r3, r3, #1
    str     r3, [sp, #OFF_ONES + 4]
    cmp     r3, #8
    blt     pb_done
    movs    r3, #1
    str     r3, [sp, #OFF_STATE + 4]
    movs    r3, #0xFF
    str     r3, [sp, #OFF_ACC + 4]
    b       pb_done
pb_sync_reset:
    movs    r3, #0
    str     r3, [sp, #OFF_ONES + 4]
    b       pb_done

pb_hunt_c3:
    ldr     r3, [sp, #OFF_ACC + 4]
    lsls    r3, r3, #1
    orrs    r3, r3, r7
    movs    r2, #0xFF
    ands    r3, r3, r2
    str     r3, [sp, #OFF_ACC + 4]
    cmp     r3, #0xC3
    bne     pb_done
    @ Found C3
    movs    r3, #2
    str     r3, [sp, #OFF_STATE + 4]
    movs    r3, #0
    str     r3, [sp, #OFF_ACC + 4]
    str     r3, [sp, #OFF_BBITS + 4]
    str     r3, [sp, #OFF_ZRUN + 4]
    @ Store C3
    ldr     r3, [sp, #OFF_MAX + 4]
    cmp     r3, #0
    ble     pb_done
    ldr     r2, [sp, #OFF_PKT + 4]
    movs    r3, #0xC3
    strb    r3, [r2, #0]
    adds    r2, r2, #1
    str     r2, [sp, #OFF_PKT + 4]
    ldr     r3, [sp, #OFF_MAX + 4]
    subs    r3, r3, #1
    str     r3, [sp, #OFF_MAX + 4]
    b       pb_done

pb_framed:
    ldr     r3, [sp, #OFF_ACC + 4]
    lsls    r3, r3, #1
    orrs    r3, r3, r7
    movs    r2, #0xFF
    ands    r3, r3, r2
    str     r3, [sp, #OFF_ACC + 4]
    ldr     r2, [sp, #OFF_BBITS + 4]
    adds    r2, r2, #1
    str     r2, [sp, #OFF_BBITS + 4]
    cmp     r2, #8
    bne     pb_done

    @ Full byte in r3
    ldr     r2, [sp, #OFF_MAX + 4]
    cmp     r2, #0
    ble     pb_no_store
    ldr     r2, [sp, #OFF_PKT + 4]
    strb    r3, [r2, #0]
    adds    r2, r2, #1
    str     r2, [sp, #OFF_PKT + 4]
    ldr     r2, [sp, #OFF_MAX + 4]
    subs    r2, r2, #1
    str     r2, [sp, #OFF_MAX + 4]
pb_no_store:
    cmp     r3, #0
    bne     pb_not_zero
    ldr     r2, [sp, #OFF_ZRUN + 4]
    adds    r2, r2, #1
    str     r2, [sp, #OFF_ZRUN + 4]
    b       pb_byte_reset
pb_not_zero:
    movs    r2, #0
    str     r2, [sp, #OFF_ZRUN + 4]
pb_byte_reset:
    movs    r3, #0
    str     r3, [sp, #OFF_ACC + 4]
    str     r3, [sp, #OFF_BBITS + 4]

pb_done:
    pop     {pc}

done:
    ldr     r0, [sp, #OFF_PKT]
    ldr     r1, [sp, #OFF_PSTART]
    subs    r0, r0, r1
    add     sp, sp, #FRAME_SIZE
    pop     {r4-r7, pc}
