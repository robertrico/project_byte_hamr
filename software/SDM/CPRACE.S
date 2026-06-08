* CPRACE.S - C2 host loader. Registers TWO instances of racetask ($0300) into
* the coproc, runs the cooperative-scheduler race with budgets NPARAM[0]=10 /
* NPARAM[1]=20, prints the finish order (bank0 $0061/$0062), then re-arms with
* swapped budgets (30/20) to show the flip.
*
* COUNT-LAST: load racetask code + both TABLE entries + NPARAM while COUNT=0,
* then write CP_COUNT=2 LAST. Re-arm requires the kernel parked (both tasks
* finished) FIRST - re-arming early HANGS the kernel, hence the ample WAITRACE.
*
* All STA CPWDATA / CPCOUNT are NON-INDEXED.
 TYP $06
 DSK CPRACE
 ORG $6000

CPLADDRLO = $C0C9
CPLADDRHI = $C0CA
CPWDATA   = $C0CB
CPCOUNT   = $C0CD
PRBYTE    = $FDDA
CROUT     = $FD8E
SRCL = $06
SRCH = $07

 JMP MAIN
 PUT SDRAMLIB

LADLO DS 1
LADHI DS 1
LEN   DS 1

* load LEN bytes from (SRCL),Y to coproc BRAM starting at LADHI:LADLO
LOADBLK
 LDA LADLO
 STA CPLADDRLO
 LDA LADHI
 STA CPLADDRHI
 LDY #0
:lp LDA (SRCL),Y
 STA CPWDATA          ; non-indexed -> one write + gateware auto-inc
 INY
 CPY LEN
 BNE :lp
 RTS

* ~16M coproc-cycle host spin - both tasks finish + kernel parks
WAITRACE
 LDX #$00
:w1 LDY #$00
:w2 INY
 BNE :w2
 INX
 BNE :w1
 RTS

* read bank0 $0061 -> print, then $0062 -> print, CR
SHOWRESULTS
 LDA #$00
 STA SDM_BANK
 STA SDM_BANK+1
 LDA #$61
 STA SDM_ADDR
 LDA #$00
 STA SDM_ADDR+1
 JSR SDM_READ
 LDA SDM_VAL
 JSR PRBYTE
 LDA #$00
 STA SDM_BANK
 STA SDM_BANK+1
 LDA #$62
 STA SDM_ADDR
 LDA #$00
 STA SDM_ADDR+1
 JSR SDM_READ
 LDA SDM_VAL
 JSR PRBYTE
 JSR CROUT
 RTS

MAIN JSR SDM_READY     ; card present + SDRAM ready
 BCC :ok
 RTS                   ; timeout -> back to BASIC
:ok
* --- load racetask code -> coproc $0300 ---
 LDA #$00
 STA LADLO
 LDA #$03
 STA LADHI
 LDA #45
 STA LEN
 LDA #<RTASK
 STA SRCL
 LDA #>RTASK
 STA SRCH
 JSR LOADBLK
* --- TABLE entry0 = $0300 @ coproc $0200, entry1 = $0300 @ $0202 ---
 LDA #$00
 STA CPLADDRLO
 LDA #$02
 STA CPLADDRHI
 LDA #$00
 STA CPWDATA          ; entry0 lo
 LDA #$03
 STA CPWDATA          ; entry0 hi -> $0300
 LDA #$00
 STA CPWDATA          ; entry1 lo
 LDA #$03
 STA CPWDATA          ; entry1 hi -> $0300
* --- NPARAM[0]=10  NPARAM[1]=20 @ coproc $00E8 ---
 LDA #$E8
 STA CPLADDRLO
 LDA #$00
 STA CPLADDRHI
 LDA #10
 STA CPWDATA          ; NPARAM[0]
 LDA #20
 STA CPWDATA          ; NPARAM[1]
* --- COUNT = 2 LAST (release) ---
 LDA #$02
 STA CPCOUNT
 JSR WAITRACE
 JSR SHOWRESULTS
* --- re-arm with swapped budgets: NPARAM[0]=30 NPARAM[1]=20 ---
 LDA #$00
 STA CPCOUNT          ; park kernel before reloading params
 LDA #$E8
 STA CPLADDRLO
 LDA #$00
 STA CPLADDRHI
 LDA #30
 STA CPWDATA          ; NPARAM[0]
 LDA #20
 STA CPWDATA          ; NPARAM[1]
 LDA #$02
 STA CPCOUNT
 JSR WAITRACE
 JSR SHOWRESULTS
 RTS

* racetask.bin (45 bytes), ORG $0300 - loaded verbatim into coproc BRAM
RTASK  DFB $A4,$EC,$B6,$E8,$CA,$20,$03,$10,$D0,$FA,$A5,$ED,$18,$69,$01,$85
       DFB $ED,$48,$A5,$EE,$18,$65,$EC,$8D,$00,$E0,$A5,$EF,$69,$00,$8D,$01
       DFB $E0,$A9,$00,$8D,$02,$E0,$68,$8D,$03,$E0,$20,$06,$10
RTASKEND
