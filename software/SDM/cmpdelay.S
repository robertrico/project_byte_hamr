* cmpdelay.S - C4 reentrant compute skill WITH a real, settable wall-clock delay.
* Same mailbox ABI + reentrancy as cmpskill (ORG $0300), but instead of a short
* spin it runs a nested busy-delay scaled by arg0 (~13 ms per arg0 unit @ 25 MHz)
* so the call takes meaningful time you can SEE. Result = arg0+1 to mailbox+16.
*
* Slot-relative ZP window: SLOTZP = $80 + CUR*16 (16 bytes, non-overlapping).
*   SLOTZP+0/+1  mailbox pointer lo/hi  (adjusted per access)
*   SLOTZP+2     original arg0          (preserved for result = arg0+1)
*   SLOTZP+3     outer delay countdown  (= arg0, counted to zero by delay)
*   SLOTZP+4     mid delay countdown    (256 per outer iteration)
* Y register = innermost loop counter (kernel saves/restores Y across preempt).
*
* Addressing: X = SLOTZP base ($80/$90/$A0/$B0) held throughout.
*   ZP,X   -> the slot's private window.   (zp,X) -> deref pointer @ SLOTZP+0/+1.
* Two concurrent instances in different slots use different X / different ZP
* windows -> fully non-overlapping.  The delay loop touches ONLY Y, $03,X, $04,X
* and A -> it never clobbers X (the held SLOTZP base) nor the mailbox pointer.
*
* ZP slot windows (no overlap):
*   Slot 0: $80..$84  Slot 1: $90..$94  Slot 2: $A0..$A4  Slot 3: $B0..$B4
* (all below kernel BUDGET @ $C0 and TCB @ $D0)
 TYP $06
 DSK cmpdelay.bin
 ORG $0300

CUR    = $EC
DONEV  = $1006

CMPDL
* --- X = SLOTZP base = $80 + CUR*16 ---
 LDA CUR        ; slot number 0-3
 ASL
 ASL
 ASL
 ASL            ; CUR*16  (max $30, no carry)
 CLC
 ADC #$80       ; + $80 -> $80/$90/$A0/$B0
 TAX            ; X = SLOTZP base for this slot

* --- store mailbox base pointer at SLOTZP+0/+1: lo=CUR*32+$80, hi=$0F ---
 LDA CUR
 ASL
 ASL
 ASL
 ASL
 ASL            ; CUR*32  (max $60, no carry)
 CLC
 ADC #$80       ; lo = $80/$A0/$C0/$E0
 STA $00,X      ; SLOTZP+0 = mailbox lo
 LDA #$0F
 STA $01,X      ; SLOTZP+1 = mailbox hi ($0F)

* --- advance pointer to +2, read arg0, save copies ---
 INC $00,X      ; ptr lo: mbox+1
 INC $00,X      ; ptr lo: mbox+2  (arg0 field)
 LDA ($00,X)    ; read arg0  [ZP ptr addr = ($00+X)&$FF = SLOTZP base]
 STA $02,X      ; SLOTZP+2 = original arg0 (preserved for result)
 STA $03,X      ; SLOTZP+3 = outer delay countdown

* --- nested busy-delay: arg0 outer x 256 mid x 256 inner ---
 LDA $03,X      ; arg0=0 edge case: skip delay entirely
 BEQ WRDONE
DOUT
 LDA #$00
 STA $04,X      ; mid = 256
DMID
 LDY #$00       ; inner = 256
DINN
 DEY
 BNE DINN       ; inner: 256 * ~5 cyc
 DEC $04,X
 BNE DMID       ; mid: 256 * inner
 DEC $03,X
 BNE DOUT       ; outer: arg0 * mid   (X untouched throughout)

* --- write result = arg0+1 to mailbox+16 ---
WRDONE
 LDA $00,X      ; current ptr lo = mbox+2
 CLC
 ADC #$0E       ; +14 -> mbox+16  (max $E2+$0E=$F0, no carry)
 STA $00,X      ; ptr -> mailbox+16 (hi still $0F)
 LDA $02,X      ; original arg0
 CLC
 ADC #$01       ; arg0 + 1
 STA ($00,X)    ; write result to mailbox+16
 JMP DONEV      ; kernel DONE (not RTS)
