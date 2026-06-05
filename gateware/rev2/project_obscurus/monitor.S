* =============================================================================
* monitor.S - project_obscurus SDRAM monitor (Merlin32), ORG $C800
* =============================================================================
* Commands (all numbers hex):
*   R aaaa        read one byte:  prints "aaaa: bb"
*   W aaaa bb     write one byte
*   B bbb         set bank ($000-$3FF)
*   D aaaa        dump 16 bytes (hex + ASCII)
*   Q             return to BASIC (RTS on PR#'s JSR return addr)
*
* Scratch lives in the $0300 page; ZP $06/$07 used only as the PRINTSTR
* indirect pointer (indirect addressing forces ZP). GETLN returns chars with
* the high bit set; Merlin32 "X" char literals are ALSO high-bit-set, so input
* is compared directly (NO masking) - they match in high-bit space. (Only the
* dump's ASCII column masks, because that operates on raw SDRAM data bytes.)
* =============================================================================
 TYP $06
 DSK monitor.bin
 ORG $C800

* ---- register port ----
RADDRLO = $C0C0
RADDRHI = $C0C1
RBANKLO = $C0C2
RBANKHI = $C0C3
RTRIG = $C0C4
RSTATUS = $C0C5
RDATA = $C0C6
DISARM = $C420 ; slot ROM disarm routine (always-live)

* ---- $0300 scratch ----
ADDR = $0300 ; 2 bytes
DATB = $0302
BANK = $0303 ; 2 bytes
DBUF = $0305 ; 16-byte dump buffer ($0305-$0314)
TBANK = $0315 ; 2 bytes: sweep bank counter
TFAIL = $0317 ; 2 bytes: failure counter
TGOT = $0319 ; 1 byte: last byte read
STRPTR = $06 ; ZP indirect ptr (PRINTSTR only)

* ---- monitor ROM entries ----
GETLN = $FD6A
COUT = $FDED
CROUT = $FD8E
PRBYTE = $FDDA
PROMPTC = $33
INBUF = $0200

* =============================================================================
MONITOR JSR READYW ; wait SDRAM ready (BIT/BVC)
 LDA #<BANNER
 STA STRPTR
 LDA #>BANNER
 STA STRPTR+1
 JSR PRINTSTR
 JSR CROUT
 LDA #0
 STA BANK
 STA BANK+1

PROMPT LDA #"*"
 ORA #$80
 STA PROMPTC
 JSR GETLN ; line -> INBUF, terminated $8D
 LDY #0
 JSR SKIPSP
 LDA INBUF,Y
 INY
 CMP #"R"
 BNE :nR
 JMP DOREAD
:nR CMP #"W"
 BNE :nW
 JMP DOWRITE
:nW CMP #"B"
 BNE :nB
 JMP DOBANK
:nB CMP #"D"
 BNE :nD
 JMP DODUMP
:nD CMP #"T"
 BNE :nT
 JMP DOTEST
:nT CMP #"Q"
 BNE :nErr
 JMP DISARM ; $C420: disarm $C800 + RTS to BASIC
:nErr JSR PRERR
 JMP PROMPT

* ---- handlers ----
DOREAD JSR SKIPSP
 JSR PH16
 JSR RDBYTE
 PHA
 JSR PRADDR
 PLA
 JSR PRBYTE
 JSR CROUT
 JMP PROMPT

DOWRITE JSR SKIPSP
 JSR PH16
 JSR SKIPSP
 JSR PH8
 LDA DATB
 JSR WRBYTE
 JSR CROUT
 JMP PROMPT

DOBANK JSR SKIPSP
 JSR PH16
 LDA ADDR
 STA BANK
 LDA ADDR+1
 AND #$03
 STA BANK+1
 JMP PROMPT

DODUMP JSR SKIPSP
 JSR PH16
 JSR SETADDR ; first access sets addr; then auto-inc
 LDX #0
:rdl STA RTRIG
 JSR POLLB
 LDA RDATA
 STA DBUF,X
 INX
 CPX #16
 BNE :rdl
 JSR PRADDR
 LDX #0
:hex LDA DBUF,X
 JSR PRBYTE
 LDA #" "
 ORA #$80
 JSR COUT
 INX
 CPX #16
 BNE :hex
 LDA #" "
 ORA #$80
 JSR COUT
 LDX #0
:asc LDA DBUF,X
 AND #$7F
 CMP #$20
 BCC :dot
 CMP #$7F
 BCS :dot
 BCC :put
:dot LDA #"."
:put ORA #$80
 JSR COUT
 INX
 CPX #16
 BNE :asc
 JSR CROUT
 JMP PROMPT

* ---- T: 1024-bank two-phase sweep ----
DOTEST LDA #0
 STA TFAIL
 STA TFAIL+1
 LDA #0
 STA TBANK
 STA TBANK+1
:p1 LDA TBANK
 STA BANK
 LDA TBANK+1
 STA BANK+1
 LDA #0
 STA ADDR
 STA ADDR+1
 JSR SETADDR
 LDX #0
:w1 TXA
 EOR TBANK
 EOR TBANK+1
 EOR #$5A
 STA RDATA ; A=expected -> DATA write strobe (auto-inc)
 JSR POLLB
 INX
 CPX #10
 BNE :w1
 INC TBANK
 BNE :n1
 INC TBANK+1
:n1 LDA TBANK+1
 CMP #$04 ; 1024 = $0400
 BCC :p1
 LDA #0
 STA TBANK
 STA TBANK+1
:p2 LDA TBANK
 STA BANK
 LDA TBANK+1
 STA BANK+1
 LDA #0
 STA ADDR
 STA ADDR+1
 JSR SETADDR
 LDX #0
:r1 STA RTRIG ; trigger read (A don't-care, auto-inc)
 JSR POLLB
 LDA RDATA
 STA TGOT
 TXA
 EOR TBANK
 EOR TBANK+1
 EOR #$5A
 CMP TGOT
 BEQ :rok
 INC TFAIL
 BNE :rok
 INC TFAIL+1
:rok INX
 CPX #10
 BNE :r1
 INC TBANK
 BNE :n2
 INC TBANK+1
:n2 LDA TBANK+1
 CMP #$04
 BCC :p2
 JSR CROUT
 LDA TFAIL
 ORA TFAIL+1
 BNE :tfail
 LDA #<MTPASS
 STA STRPTR
 LDA #>MTPASS
 STA STRPTR+1
 JSR PRINTSTR
 JSR CROUT
 JMP PROMPT
:tfail LDA TFAIL+1
 JSR PRBYTE
 LDA TFAIL
 JSR PRBYTE
 LDA #<MTFAIL
 STA STRPTR
 LDA #>MTFAIL
 STA STRPTR+1
 JSR PRINTSTR
 JSR CROUT
 JMP PROMPT

* ---- SDRAM access helpers ----
SETADDR LDA ADDR
 STA RADDRLO
 LDA ADDR+1
 STA RADDRHI
 LDA BANK
 STA RBANKLO
 LDA BANK+1
 STA RBANKHI
 RTS

POLLB LDA RSTATUS
 BMI POLLB ; bit7=busy -> N -> loop
 RTS

READYW BIT RSTATUS
 BVC READYW ; bit6=ready -> V; wait until set
 RTS

RDBYTE JSR SETADDR
 STA RTRIG ; any value triggers a read
 JSR POLLB
 LDA RDATA
 RTS

WRBYTE PHA
 JSR SETADDR
 PLA
 STA RDATA ; latch + trigger write
 JSR POLLB
 RTS

* ---- output helpers ----
PRADDR LDA ADDR+1
 JSR PRBYTE
 LDA ADDR
 JSR PRBYTE
 LDA #":"
 ORA #$80
 JSR COUT
 LDA #" "
 ORA #$80
 JSR COUT
 RTS

PRINTSTR LDY #0
:l LDA (STRPTR),Y
 BEQ :x
 ORA #$80
 JSR COUT
 INY
 BNE :l
:x RTS

PRERR LDA #"?"
 ORA #$80
 JSR COUT
 JSR CROUT
 RTS

* ---- input helpers ----
SKIPSP LDA INBUF,Y
 CMP #" "
 BNE :done
 INY
 BNE SKIPSP
:done RTS

* PH16: parse hex into ADDR (16-bit). Stops at first non-hex char.
PH16 LDA #0
 STA ADDR
 STA ADDR+1
:l LDA INBUF,Y
 JSR HEXNIB
 BCC :done
 PHA
 ASL ADDR
 ROL ADDR+1
 ASL ADDR
 ROL ADDR+1
 ASL ADDR
 ROL ADDR+1
 ASL ADDR
 ROL ADDR+1
 PLA
 ORA ADDR
 STA ADDR
 INY
 BNE :l
:done RTS

* PH8: parse up to 2 hex into DATB.
PH8 LDA #0
 STA DATB
:l LDA INBUF,Y
 JSR HEXNIB
 BCC :done
 ASL DATB
 ASL DATB
 ASL DATB
 ASL DATB
 ORA DATB
 STA DATB
 INY
 BNE :l
:done RTS

* HEXNIB: A=ascii char (no high bit). Returns C=1 + A=nibble if hex, else C=0.
HEXNIB CMP #"0"
 BCC :bad
 CMP #"9"+1
 BCC :dig
 CMP #"A"
 BCC :bad
 CMP #"G"
 BCS :bad
 SEC
 SBC #"A"
 CLC
 ADC #10
 SEC
 RTS
:dig SEC
 SBC #"0"
 SEC
 RTS
:bad CLC
 RTS

BANNER ASC "OBSCURUS SDRAM MONITOR"
 DFB $00
MTPASS ASC "TEST PASS"
 DFB $00
MTFAIL ASC " TEST FAIL"
 DFB $00
