* =============================================================================
* SDMTEST.S - project_obscurus SDRAM bank test (Merlin32)
* =============================================================================
* BRUN at $2000. PUTs SDRAMLIB. Two-phase write-all/read-all over offsets
* $00-$09 of all 1024 banks. expected = lo(bank) EOR hi(bank) EOR off EOR $5A.
* Output: banner, '.' per 64 banks, first 16 mismatches, then PASS / nnnn FAILURES.
* NO CARD + quit if the card isn't ready (SDM_READY timeout).
* =============================================================================
 TYP $06
 DSK SDMTEST
 ORG $2000

 JMP MAIN
 PUT SDRAMLIB ; lib code + its DS vars live here

COUT = $FDED
CROUT = $FD8E
PRBYTE = $FDDA
STRPTR = $06 ; test-local ZP for string printing (app, not lib)

* ---- test state ----
CURBANK DS 2 ; 0..1023
FAILS DS 2 ; 16-bit mismatch counter
PRINTED DS 1 ; mismatch lines printed (cap 16)
GOT DS 1 ; byte read back

* =============================================================================
MAIN LDA #<MBANNER
 STA STRPTR
 LDA #>MBANNER
 STA STRPTR+1
 JSR PRSTR
 JSR CROUT

 JSR SDM_READY
 BCC :ready
 LDA #<MNOCARD
 STA STRPTR
 LDA #>MNOCARD
 STA STRPTR+1
 JSR PRSTR
 JSR CROUT
 RTS ; back to BASIC
:ready
 LDA #0
 STA FAILS
 STA FAILS+1
 STA PRINTED

* ---------- PHASE 1: write all ----------
 LDA #0
 STA CURBANK
 STA CURBANK+1
P1 JSR DOTCHK
 LDA CURBANK
 STA SDM_BANK
 LDA CURBANK+1
 STA SDM_BANK+1
 JSR SDM_SETBANK
 LDA #0
 STA SDM_ADDR
 STA SDM_ADDR+1
 JSR SDM_SETADDR
 LDX #0
:wl JSR CALCEXP ; A = expected for offset X
 STA SDM_VAL
 JSR SDM_WRNEXT
 INX
 CPX #10
 BNE :wl
 JSR BANKINC
 BCC P1 ; carry clear = more banks

* ---------- PHASE 2: read all ----------
 LDA #0
 STA CURBANK
 STA CURBANK+1
P2 JSR DOTCHK
 LDA CURBANK
 STA SDM_BANK
 LDA CURBANK+1
 STA SDM_BANK+1
 JSR SDM_SETBANK
 LDA #0
 STA SDM_ADDR
 STA SDM_ADDR+1
 JSR SDM_SETADDR
 LDX #0
:rl JSR SDM_RDNEXT ; A = byte read
 STA GOT
 JSR CALCEXP ; A = expected
 CMP GOT
 BEQ :ok
 JSR ONFAIL
:ok INX
 CPX #10
 BNE :rl
 JSR BANKINC
 BCC P2

* ---------- result ----------
 JSR CROUT
 LDA FAILS
 ORA FAILS+1
 BNE :fail
 LDA #<MPASS
 STA STRPTR
 LDA #>MPASS
 STA STRPTR+1
 JSR PRSTR
 JSR CROUT
 RTS
:fail LDA FAILS+1
 JSR PRBYTE
 LDA FAILS
 JSR PRBYTE
 LDA #<MFAIL
 STA STRPTR
 LDA #>MFAIL
 STA STRPTR+1
 JSR PRSTR
 JSR CROUT
 RTS

* ---- expected = X(offset) EOR lo(bank) EOR hi(bank) EOR $5A ; clobbers A ----
CALCEXP TXA
 EOR CURBANK
 EOR CURBANK+1
 EOR #$5A
 RTS

* ---- CURBANK++ ; returns C=0 if more (CURBANK<1024), C=1 if done ----
BANKINC INC CURBANK
 BNE :nc
 INC CURBANK+1
:nc LDA CURBANK+1
 CMP #$04 ; 1024 = $0400
 RTS ; C set when hi>=4 -> done

* ---- print '.' when (CURBANK & $3F)==0 ----
DOTCHK LDA CURBANK
 AND #$3F
 BNE :skip
 LDA #"."
 JSR COUT
:skip RTS

* ---- on mismatch: FAILS++ ; print line if under cap (16) ----
ONFAIL INC FAILS
 BNE :nf
 INC FAILS+1
:nf LDA PRINTED
 CMP #16
 BCS :done ; already printed 16 -> suppress
 INC PRINTED
 LDA CURBANK+1
 JSR PRBYTE
 LDA CURBANK
 JSR PRBYTE
 LDA #"/"
 JSR COUT
 TXA ; offset
 JSR PRBYTE
 LDA #" "
 JSR COUT
 JSR CALCEXP ; expected
 JSR PRBYTE
 LDA #" "
 JSR COUT
 LDA GOT
 JSR PRBYTE
 JSR CROUT
:done RTS

* ---- print high-bit-set, $00-terminated string at (STRPTR) ----
PRSTR LDY #0
:l LDA (STRPTR),Y
 BEQ :x
 JSR COUT
 INY
 BNE :l
:x RTS

MBANNER ASC "SDM BANK TEST"
 DFB $00
MNOCARD ASC "NO CARD"
 DFB $00
MPASS ASC "PASS"
 DFB $00
MFAIL ASC " FAILURES"
 DFB $00
