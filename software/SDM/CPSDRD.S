* CPSDRD.S - Seed SDRAM bank0 $0080..$0087 via host SDRAMLIB, fire sdrtest
* skill (coproc reads+sums those 8 bytes), then host reads result from SDRAM
* bank0 $0090 and prints it.  Expected output: "SDRD=64"
*
* Sum: $11+$22+$33+$44+$55+$66+$77+$88 = $0264 -> low byte $64.
*
* All STA $C0Cx are NON-INDEXED (no indexed STA to $C0Cx ever).
 TYP $06
 DSK CPSDRD
 ORG $6000

COUT   = $FDED
CROUT  = $FD8E
PRBYTE = $FDDA
SRCL   = $06            ; ZP lo ptr for LOADBLK indirect read
SRCH   = $07            ; ZP hi ptr for LOADBLK indirect read

 JMP MAIN
 PUT CPLIB
 PUT SDRAMLIB

LADLO DS 1
LADHI DS 1
LEN   DS 1

* Load LEN bytes from (SRCL),Y to coproc BRAM starting at LADHI:LADLO.
* Mirrors CPDEMO's LOADBLK exactly.
LOADBLK
 LDA LADLO
 STA CP_LADDRLO
 LDA LADHI
 STA CP_LADDRHI
 LDY #0
:lp LDA (SRCL),Y
 STA CP_WDATA           ; non-indexed -> one write; gateware auto-incs laddr
 INY
 CPY LEN
 BNE :lp
 RTS

MAIN
* --- init SDRAM driver ---
 JSR SDM_READY
 BCC :rdyok
 BRK                    ; timeout; halt so the user can see something went wrong
:rdyok

* --- seed bank0 $0080=$11 via SDM_WRITE ---
 LDA #$00
 STA SDM_BANK
 STA SDM_BANK+1
 LDA #$80
 STA SDM_ADDR
 LDA #$00
 STA SDM_ADDR+1
 LDA #$11
 STA SDM_VAL
 JSR SDM_WRITE          ; sets bank+addr in hardware, writes $11, auto-incs ptr

* --- seed $0081..$0087 = $22..$88 via SDM_WRNEXT (relies on hardware auto-inc) ---
 LDA #$22
 STA SDM_VAL
 JSR SDM_WRNEXT
 LDA #$33
 STA SDM_VAL
 JSR SDM_WRNEXT
 LDA #$44
 STA SDM_VAL
 JSR SDM_WRNEXT
 LDA #$55
 STA SDM_VAL
 JSR SDM_WRNEXT
 LDA #$66
 STA SDM_VAL
 JSR SDM_WRNEXT
 LDA #$77
 STA SDM_VAL
 JSR SDM_WRNEXT
 LDA #$88
 STA SDM_VAL
 JSR SDM_WRNEXT

* --- zero result cell bank0 $0090 so a stale value can't false-pass ---
 LDA #$90
 STA SDM_ADDR
 LDA #$00
 STA SDM_ADDR+1
 LDA #$00
 STA SDM_VAL
 JSR SDM_WRITE          ; bank still 0; writes $00 to $0090

* --- register sdrtest as skill 0 ---
 JSR REGSKILL

* --- run skill 0: CP_CALL(skill=0, arg0=0, budget=0) -> handle in A ---
 LDA #$00               ; skill_id = 0
 LDX #$00               ; arg0 = 0 (sdrtest ignores arg0)
 LDY #$00               ; budget = 0 (no watchdog)
 JSR CP_CALL
 STA H

* --- wait for skill completion ---
 LDA H
 JSR CP_WAIT

* --- free the slot (real result is in SDRAM $0090, not the mailbox) ---
 LDA H
 JSR CP_RESULT          ; returns mailbox result byte in A; we ignore it

* --- read result: SDM_BANK=0, SDM_ADDR=$0090 still set from zero-write above ---
 JSR SDM_READ           ; re-arms bank+addr from SDM_BANK/SDM_ADDR vars -> SDM_VAL

* --- print "SDRD=" + result byte + CR ---
 LDA #$D3               ; 'S' high-bit
 JSR COUT
 LDA #$C4               ; 'D' high-bit
 JSR COUT
 LDA #$D2               ; 'R' high-bit
 JSR COUT
 LDA #$C4               ; 'D' high-bit
 JSR COUT
 LDA #$BD               ; '=' high-bit
 JSR COUT
 LDA SDM_VAL
 JSR PRBYTE             ; prints 2 hex digits; expect "64"
 JSR CROUT
 RTS

H DS 1

* Register sdrtest as skill 0:
*   1. load 61 sdrtest bytes -> coproc BRAM $0300 (via CP load port)
*   2. write TABLE entry: coproc $0200/$0201 = lo/hi = $00/$03 -> $0300
* Pattern mirrors CPDEMO's REGSKILL exactly.
REGSKILL
 LDA #$00
 STA LADLO
 LDA #$03
 STA LADHI
 LDA #61               ; sdrtest.bin = 61 bytes
 STA LEN
 LDA #<CSKILL
 STA SRCL
 LDA #>CSKILL
 STA SRCH
 JSR LOADBLK
* TABLE entry 0 (skill 0 -> $0300): write lo=$00 hi=$03 at coproc $0200
 LDA #$00
 STA CP_LADDRLO
 LDA #$02
 STA CP_LADDRHI
 LDA #$00
 STA CP_WDATA           ; TABLE[0] lo = $00
 LDA #$03
 STA CP_WDATA           ; TABLE[0] hi = $03 => skill 0 entry = $0300
 RTS

* sdrtest.bin (61 bytes) verbatim, ORG $0300 - loaded into coproc BRAM by REGSKILL.
CSKILL
 DFB $A9,$80,$8D,$04,$E0,$A9,$00,$8D,$05,$E0,$A9,$00,$8D,$06,$E0,$A9
 DFB $00,$85,$80,$A9,$08,$85,$81,$8D,$07,$E0,$AD,$08,$E0,$18,$65,$80
 DFB $85,$80,$C6,$81,$D0,$F1,$A9,$90,$8D,$00,$E0,$A9,$00,$8D,$01,$E0
 DFB $A9,$00,$8D,$02,$E0,$A5,$80,$8D,$03,$E0,$4C,$06,$10
CSKEND
