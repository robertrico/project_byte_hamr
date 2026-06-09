* CPDEMO.S - C4 async skill demo. Registers cmpskill at $0300 (skill 0),
* fires two concurrent calls (arg0=8, arg0=20), prints '.' dots while both
* run in parallel on the coproc, then collects and prints hex results.
* Expected output: dots followed by CR then "0915" (8+1=09, 20+1=15).
*
* Async proof: the Apple II prints dots WHILE the coproc computes.
*
* All STA $C0Cx are NON-INDEXED (no indexed STA to $C0Cx ever).
 TYP $06
 DSK CPDEMO
 ORG $6000

COUT   = $FDED
CROUT  = $FD8E
PRBYTE = $FDDA
SRCL   = $06
SRCH   = $07

 JMP MAIN
 PUT CPLIB

LADLO DS 1
LADHI DS 1
LEN   DS 1

* Load LEN bytes from (SRCL),Y to coproc BRAM starting at LADHI:LADLO.
* Mirrors CPRACE3's LOADBLK exactly; uses CP_LADDRLO/HI/CP_WDATA from CPLIB.
LOADBLK
 LDA LADLO
 STA CP_LADDRLO
 LDA LADHI
 STA CP_LADDRHI
 LDY #0
:lp LDA (SRCL),Y
 STA CP_WDATA          ; non-indexed -> one write, gateware auto-incs laddr
 INY
 CPY LEN
 BNE :lp
 RTS

MAIN
 JSR REGSKILL          ; load cmpskill -> coproc $0300; TABLE[0]=$0300
* --- fire call A: skill 0, arg0=8, budget=0 (no limit) ---
 LDA #$00              ; skill_id = 0
 LDX #$08              ; arg0 = 8
 LDY #$00              ; budget = 0
 JSR CP_CALL
 STA HA                ; save handle A
* --- fire call B: skill 0, arg0=20, budget=0 ---
 LDA #$00
 LDX #$14              ; arg0 = 20 = $14
 LDY #$00
 JSR CP_CALL
 STA HB                ; save handle B
* --- work loop: print dots while both calls are pending ---
WORK
 LDA #$AE              ; '.' (high-bit ASCII)
 JSR COUT
 LDA HA
 JSR CP_POLL
 BEQ WORK              ; A not done ($00) -> keep printing
 LDA HB
 JSR CP_POLL
 BEQ WORK              ; B not done ($00) -> keep printing
* --- both done; collect and display results ---
 JSR CROUT
 LDA HA
 JSR CP_RESULT         ; frees slot A; returns result in A
 JSR PRBYTE            ; print hex: expect 09  (8+1)
 LDA HB
 JSR CP_RESULT         ; frees slot B; returns result in A
 JSR PRBYTE            ; print hex: expect 15  (20+1)
 JSR CROUT
 RTS

HA DS 1
HB DS 1

* Register cmpskill:
*   1. load 68 cmpskill bytes -> coproc BRAM $0300 (via load port)
*   2. write TABLE entry: coproc $0200/$0201 = lo/hi = $00/$03 -> $0300
* Pattern mirrors CPRACE3's skill-load + TABLE-write idiom exactly.
REGSKILL
 LDA #$00
 STA LADLO
 LDA #$03
 STA LADHI
 LDA #68               ; cmpskill.bin = 68 bytes
 STA LEN
 LDA #<CSKILL
 STA SRCL
 LDA #>CSKILL
 STA SRCH
 JSR LOADBLK
* TABLE entry 0 (skill 0 -> $0300): write lo=$00, hi=$03 at coproc $0200
 LDA #$00
 STA CP_LADDRLO
 LDA #$02
 STA CP_LADDRHI
 LDA #$00
 STA CP_WDATA          ; TABLE[0] lo = $00
 LDA #$03
 STA CP_WDATA          ; TABLE[0] hi = $03 => skill 0 entry = $0300
 RTS

* cmpskill.bin (68 bytes) verbatim, ORG $0300 - loaded into coproc BRAM by REGSKILL.
CSKILL
 DFB $A5,$EC,$0A,$0A,$0A,$0A,$18,$69,$80,$AA,$A5,$EC,$0A,$0A,$0A,$0A
 DFB $0A,$18,$69,$80,$95,$00,$A9,$0F,$95,$01,$F6,$00,$F6,$00,$A1,$00
 DFB $95,$02,$95,$03,$B5,$02,$F0,$0B,$A9,$00,$38,$E9,$01,$D0,$FB,$D6
 DFB $02,$D0,$F5,$B5,$00,$18,$69,$0E,$95,$00,$B5,$03,$18,$69,$01,$81
 DFB $00,$4C,$06,$10
CSKEND
