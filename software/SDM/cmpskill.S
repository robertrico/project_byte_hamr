* cmpskill.S - C4 reentrant compute skill on the slot mailbox ABI. ORG $0300.
* Reads mailbox+2 (arg0), busy-spins arg0 outer loops x 256 inner iterations,
* writes result=arg0+1 to mailbox+16, then JMP DONE.
*
* Slot-relative ZP window: SLOTZP = $80 + CUR*16 (16 bytes, non-overlapping).
*   SLOTZP+0/+1  mailbox pointer lo/hi  (adjusted per access)
*   SLOTZP+2     loop countdown         (= arg0, counted to zero by spin)
*   SLOTZP+3     original arg0          (saved before spin for result computation)
*
* Addressing: X = SLOTZP base ($80/$90/$A0/$B0) throughout.
*   ZP,X     (STA $00,X etc.)   -> addresses the slot's private window directly.
*   (zp,X)   (LDA ($00,X) etc.) -> dereferences through pointer at (SLOTZP+0/+1).
*   Two concurrent instances in different slots use different X values and
*   different ZP windows -> completely non-overlapping, no cross-slot collision.
*
* ZP slot windows (no overlap):
*   Slot 0: $80..$83  Slot 1: $90..$93  Slot 2: $A0..$A3  Slot 3: $B0..$B3
* (all below kernel BUDGET @ $C0 and TCB @ $D0)
 TYP $06
 DSK cmpskill.bin
 ORG $0300

CUR    = $EC
DONEV  = $1006

CMPSK
* --- X = SLOTZP base = $80 + CUR*16 ---
 LDA CUR        ; slot number 0-3
 ASL
 ASL
 ASL
 ASL            ; CUR*16  (max $30 for CUR=3, no carry)
 CLC
 ADC #$80       ; + $80 -> $80/$90/$A0/$B0
 TAX            ; X = SLOTZP base for this slot

* --- store mailbox base pointer at SLOTZP+0/+1: lo=CUR*32+$80, hi=$0F ---
 LDA CUR
 ASL
 ASL
 ASL
 ASL
 ASL            ; CUR*32  (max $60 for CUR=3, no carry)
 CLC
 ADC #$80       ; lo = $80/$A0/$C0/$E0  (no carry: max $60+$80=$E0)
 STA $00,X      ; SLOTZP+0 = mailbox lo byte
 LDA #$0F
 STA $01,X      ; SLOTZP+1 = mailbox hi byte ($0F)

* --- advance pointer to +2, read arg0, save copy before countdown ---
 INC $00,X      ; pointer lo: mbox+1
 INC $00,X      ; pointer lo: mbox+2  (arg0 field)
 LDA ($00,X)    ; read arg0  [ZP ptr addr = ($00+X)&$FF = SLOTZP base]
 STA $02,X      ; SLOTZP+2 = outer-loop countdown (decremented to 0)
 STA $03,X      ; SLOTZP+3 = original arg0 (preserved for result)

* --- busy-spin: arg0 outer iterations x 256 inner SBC steps ---
 LDA $02,X      ; handle arg0=0 edge case: skip spin entirely
 BEQ WRDONE
OUTER
 LDA #$00       ; inner countdown: $00 -> 256 executions of the loop body
INNER
 SEC
 SBC #$01       ; $00->$FF->..->$01->$00
 BNE INNER      ; loop until A wraps to zero (256 iterations)
 DEC $02,X      ; outer counter--
 BNE OUTER      ; while outer != 0

* --- write result = arg0+1 to mailbox+16 ---
WRDONE
 LDA $00,X      ; current pointer lo = mbox+2 (set at entry to INNER block)
 CLC
 ADC #$0E       ; +14 -> mbox+16  (max: $E2+$0E=$F0, no carry)
 STA $00,X      ; pointer -> mailbox+16  (hi still $0F)
 LDA $03,X      ; original arg0
 CLC
 ADC #$01       ; arg0 + 1
 STA ($00,X)    ; write result to mailbox+16
 JMP DONEV      ; kernel DONE (not RTS)
