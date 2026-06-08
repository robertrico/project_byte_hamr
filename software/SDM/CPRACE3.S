* CPRACE3.S - C3 host loader. Registers TWO instances of racetask3 ($0300) into
* the coproc, runs the PREEMPTIVE-scheduler race with budgets NPARAM[0]=50 /
* NPARAM[1]=200, prints the finish order (bank0 $0061/$0062 -> expect 01 02),
* then re-arms with swapped budgets (250/200) to show the flip (-> expect 02 01).
*
* racetask3 does NOT yield - the kernel's timer tick slices it. The host just
* sets COUNT to arm; the kernel arms the timer itself in BOOTSTRAP.
*
* COUNT-LAST: load racetask3 code + both TABLE entries + NPARAM while COUNT=0,
* then write CP_COUNT=2 LAST. Re-arm requires the kernel parked (both tasks
* finished) FIRST - re-arming early HANGS the kernel, hence the ample WAITRACE.
*
* All STA CPWDATA / CPCOUNT are NON-INDEXED.
 TYP $06
 DSK CPRACE3
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

* host spin - both tasks finish + kernel parks. The no-yield inner-delay tasks
* run long, so this is wider than CPRACE.S's: full 16-bit X*Y*Z triple loop.
WAITRACE
 LDA #$04
 STA :ctr
:w0 LDX #$00
:w1 LDY #$00
:w2 INY
 BNE :w2
 INX
 BNE :w1
 DEC :ctr
 BNE :w0
 RTS
:ctr DS 1

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
* --- load racetask3 code -> coproc $0300 ---
 LDA #$00
 STA LADLO
 LDA #$03
 STA LADHI
 LDA #49
 STA LEN
 LDA #<RT3
 STA SRCL
 LDA #>RT3
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
* --- NPARAM[0]=50  NPARAM[1]=200 @ coproc $00E8 ---
 LDA #$E8
 STA CPLADDRLO
 LDA #$00
 STA CPLADDRHI
 LDA #50
 STA CPWDATA          ; NPARAM[0]
 LDA #200
 STA CPWDATA          ; NPARAM[1]
* --- COUNT = 2 LAST (arm) ---
 LDA #$02
 STA CPCOUNT
 JSR WAITRACE
 JSR SHOWRESULTS      ; expect 01 02
* --- re-arm with swapped budgets: NPARAM[0]=250 NPARAM[1]=200 ---
 LDA #$00
 STA CPCOUNT          ; park kernel before reloading params
 LDA #$E8
 STA CPLADDRLO
 LDA #$00
 STA CPLADDRHI
 LDA #250
 STA CPWDATA          ; NPARAM[0]
 LDA #200
 STA CPWDATA          ; NPARAM[1]
 LDA #$02
 STA CPCOUNT
 JSR WAITRACE
 JSR SHOWRESULTS      ; expect 02 01
 RTS

* racetask3.bin (49 bytes), ORG $0300 - loaded verbatim into coproc BRAM
RT3    DFB $A4,$EC,$B6,$E8,$A9,$80,$38,$E9,$01,$D0,$FC,$CA,$D0,$F6,$78,$A5
       DFB $ED,$18,$69,$01,$85,$ED,$48,$98,$18,$65,$EE,$8D,$00,$E0,$A5,$EF
       DFB $69,$00,$8D,$01,$E0,$A9,$00,$8D,$02,$E0,$68,$8D,$03,$E0,$4C,$06
       DFB $10
RT3END
