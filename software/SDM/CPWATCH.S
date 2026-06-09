* CPWATCH.S - C4 OBSERVABLE concurrency proof. ORG $6000.
* Registers cmpdelay as skill 0, then fires TWO async calls:
*   A = SLOW  (arg0=$14=20  -> ~260 ms)   fired FIRST,  handle in HA
*   B = FAST  (arg0=$04= 4  -> ~ 53 ms)   fired SECOND, handle in HB
* While both run on the coproc, the //e spins a watch loop, counts its OWN
* iterations in a 16-bit CTR, and streams a '.' every 64 loops (visible activity).
* It polls B and A each pass and, the instant one completes, prints
*   "B=" result " @" CTR     /     "A=" result " @" CTR
*
* THE PROOF (expected screen):
*   WATCH
*   .........B=05 @01A4
*   ...............A=15 @06F2
*   - B (fired SECOND, faster) reports BEFORE A  -> out-of-order completion
*     = the two skills ran CONCURRENTLY on the coproc (serial would finish A first).
*   - @01A4 / @06F2 large + nonzero  -> the //e was NOT blocked while the coproc
*     computed (async). B's loop count < A's because B finished sooner.
*   - B=05 (4+1), A=15 (20+1) = results correct.
*
* All STA $C0Cx are NON-INDEXED (no indexed STA to $C0Cx ever).
 TYP $06
 DSK CPWATCH
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
LOADBLK
 LDA LADLO
 STA CP_LADDRLO
 LDA LADHI
 STA CP_LADDRHI
 LDY #0
:lp LDA (SRCL),Y
 STA CP_WDATA          ; non-indexed -> gateware auto-incs laddr
 INY
 CPY LEN
 BNE :lp
 RTS

MAIN
 JSR REGSKILL          ; load cmpdelay -> coproc $0300; TABLE[0]=$0300

* --- fire A = SLOW: skill 0, arg0=$14 (20), budget 0 ---
 LDA #$00
 LDX #$14
 LDY #$00
 JSR CP_CALL
 STA HA

* --- fire B = FAST: skill 0, arg0=$04 (4), budget 0 ---
 LDA #$00
 LDX #$04
 LDY #$00
 JSR CP_CALL
 STA HB

* --- init watch-loop state ---
 LDA #$00
 STA CTR
 STA CTR+1
 STA REPA
 STA REPB

* --- header line: "WATCH" CR ---
 LDX #$00
:hdr LDA HDR,X
 BEQ :hdrd
 JSR COUT
 INX
 BNE :hdr
:hdrd
 JSR CROUT

* ============================ WATCH LOOP ============================
WLOOP
 INC CTR
 BNE :nc
 INC CTR+1
:nc
* print '.' every 64 loops -> visible //e streaming
 LDA CTR
 AND #$3F
 BNE :poll
 LDA #$AE              ; '.'
 JSR COUT

:poll
* --- check B (fast) first ---
 LDA REPB
 BNE :ckA
 LDA HB
 JSR CP_POLL
 BEQ :ckA              ; B not done
 LDA #$01
 STA REPB             ; mark B reported
 JSR CROUT
 LDA #$C2              ; 'B'
 JSR COUT
 LDA #$BD              ; '='
 JSR COUT
 LDA HB
 JSR CP_RESULT
 JSR PRBYTE           ; B result (expect 05)
 JSR PRSPAT           ; " @"
 LDA CTR+1
 JSR PRBYTE           ; loop count hi
 LDA CTR
 JSR PRBYTE           ; loop count lo

:ckA
* --- check A (slow) ---
 LDA REPA
 BNE :chkdone
 LDA HA
 JSR CP_POLL
 BEQ :chkdone          ; A not done
 LDA #$01
 STA REPA             ; mark A reported
 JSR CROUT
 LDA #$C1              ; 'A'
 JSR COUT
 LDA #$BD              ; '='
 JSR COUT
 LDA HA
 JSR CP_RESULT
 JSR PRBYTE           ; A result (expect 15)
 JSR PRSPAT           ; " @"
 LDA CTR+1
 JSR PRBYTE
 LDA CTR
 JSR PRBYTE

:chkdone
 LDA REPA
 AND REPB
 BNE WDONE             ; both reported ($01 AND $01 = $01) -> exit
 JMP WLOOP

WDONE
 JSR CROUT
 RTS

* --- print " @" ---
PRSPAT
 LDA #$A0              ; ' '
 JSR COUT
 LDA #$C0              ; '@'
 JSR COUT
 RTS

HDR  ASC "WATCH"
     DFB $00

HA   DS 1
HB   DS 1
CTR  DS 2
REPA DS 1
REPB DS 1

* Register cmpdelay:
*   1. load 74 cmpdelay bytes -> coproc BRAM $0300 (via load port)
*   2. write TABLE entry: coproc $0200/$0201 = lo/hi = $00/$03 -> $0300
REGSKILL
 LDA #$00
 STA LADLO
 LDA #$03
 STA LADHI
 LDA #74               ; cmpdelay.bin = 74 bytes
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

* cmpdelay.bin (74 bytes) verbatim, ORG $0300 - loaded into coproc BRAM by REGSKILL.
CSKILL
 DFB $A5,$EC,$0A,$0A,$0A,$0A,$18,$69,$80,$AA,$A5,$EC,$0A,$0A,$0A,$0A
 DFB $0A,$18,$69,$80,$95,$00,$A9,$0F,$95,$01,$F6,$00,$F6,$00,$A1,$00
 DFB $95,$02,$95,$03,$B5,$03,$F0,$11,$A9,$00,$95,$04,$A0,$00,$88,$D0
 DFB $FD,$D6,$04,$D0,$F7,$D6,$03,$D0,$EF,$B5,$00,$18,$69,$0E,$95,$00
 DFB $B5,$02,$18,$69,$01,$81,$00,$4C,$06,$10
CSKEND
