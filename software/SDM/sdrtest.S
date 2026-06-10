* sdrtest.S - SDRAM read-window proof skill. ORG $0300, ends JMP DONE ($1006).
* Reads 8 bytes from SDRAM bank0 $0080..$0087 via the auto-incrementing READ
* window ($E004-$E008), sums them (single-byte accumulate, carry discarded),
* and writes the sum to SDRAM bank0 $0090 via the WRITE window ($E000-$E003).
*
* READ window semantics (gateware coproc.v):
*   STA RTRIG -> reads SDRAM[{rbank,raddr}] (CPU stalls until done), latches the
*               byte into the read register, then auto-increments the 24-bit ptr.
*   LDA RDATA -> returns the latched byte (zero-latency).
* The data written by STA RTRIG is ignored by the gateware (the write only
* triggers), so the value left in A is harmless.
*
* ZP scratch: SUM/CNT use the slot-0 window ($80/$81). The C4 kernel owns
* $C0-$F3 (TCB/BUDGET/TMP*) and never touches $80-$8F; the free-running timer
* ISR preserves task context via the TCB, so these survive preemption. This
* skill is dispatched as slot 0, so $80/$81 are exactly its own window.
 TYP $06
 DSK sdrtest.bin
 ORG $0300

RADDRLO = $E004
RADDRHI = $E005
RBANK   = $E006
RTRIG   = $E007
RDATA   = $E008
SADDRLO = $E000
SADDRHI = $E001
SBANK   = $E002
SDATA   = $E003
DONE    = $1006

SUM     = $80
CNT     = $81

SDRT
* --- read pointer = bank0, addr $0080 ---
 LDA #$80
 STA RADDRLO
 LDA #$00
 STA RADDRHI
 LDA #$00
 STA RBANK
* --- init accumulator + 8-byte counter ---
 LDA #$00
 STA SUM
 LDA #$08
 STA CNT
LOOP
 STA RTRIG          ; trigger read (data ignored) -> stall, latch, auto-inc ptr
 LDA RDATA          ; fetch the latched byte
 CLC
 ADC SUM
 STA SUM
 DEC CNT
 BNE LOOP
* --- write sum to bank0 $0090 ---
 LDA #$90
 STA SADDRLO
 LDA #$00
 STA SADDRHI
 LDA #$00
 STA SBANK
 LDA SUM
 STA SDATA
 JMP DONE
