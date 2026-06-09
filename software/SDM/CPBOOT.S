* CPBOOT.S - C-flash host BOOT. Run AFTER a power-cycle. The gateware
* auto-restores the registry (service code + TABLE) from SPI flash into coproc
* BRAM at boot. CPBOOT does NOT load racetask3 and does NOT write the TABLE -
* the whole point is that they came from FLASH. It only: confirms restore_valid
* (CP_SAVE bit0 @ $C0CF), sets the per-task budgets NPARAM @ coproc $00E8
* (50/200, matching CPRACE3), zeroes the result cells, then GOs (CP_COUNT=2)
* and prints the finish order (bank0 $0061/$0062 -> expect 01 02).
*
* NPARAM is NOT persisted by the registry, so the host re-supplies it here.
* CP_SVCNT ($C0CE) currently reads 0 in gateware - we hardcode COUNT=2 instead.
*
* All STA CPWDATA / CPCOUNT are NON-INDEXED.
 TYP $06
 DSK CPBOOT
 ORG $6000

CPLADDRLO = $C0C9
CPLADDRHI = $C0CA
CPWDATA   = $C0CB
CPCOUNT   = $C0CD
CPSAVE    = $C0CF
PRBYTE    = $FDDA
CROUT     = $FD8E
COUT      = $FDED
SRCL = $06
SRCH = $07

 JMP MAIN
 PUT SDRAMLIB

* print $00-terminated high-bit msg at (SRCL),Y
PRMSG
 LDY #0
:lp LDA (SRCL),Y
 BEQ :done
 JSR COUT
 INY
 BNE :lp
:done RTS

* host spin - both tasks finish + kernel parks (copied from CPRACE3.S)
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

* zero SDRAM result cells bank0 $0061/$0062 before arming the race
ZEROCELLS
 LDA #$00
 STA SDM_BANK
 STA SDM_BANK+1
 LDA #$61
 STA SDM_ADDR
 LDA #$00
 STA SDM_ADDR+1
 STA SDM_VAL
 JSR SDM_WRITE
 LDA #$62
 STA SDM_ADDR
 LDA #$00
 STA SDM_VAL
 JSR SDM_WRITE
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
* --- confirm registry was auto-restored from flash (restore_valid bit0) ---
 LDA CPSAVE
 AND #$01
 BNE :restored
 LDA #<MNOREG
 STA SRCL
 LDA #>MNOREG
 STA SRCH
 JSR PRMSG
 JSR CROUT
 RTS
:restored
* --- NPARAM[0]=50  NPARAM[1]=200 @ coproc $00E8 (not persisted; re-supply) ---
 LDA #$E8
 STA CPLADDRLO
 LDA #$00
 STA CPLADDRHI
 LDA #50
 STA CPWDATA          ; NPARAM[0]
 LDA #200
 STA CPWDATA          ; NPARAM[1]
* --- zero result cells, then GO (COUNT=2, hardcoded - $C0CE reads 0) ---
 JSR ZEROCELLS
 LDA #$02
 STA CPCOUNT
 JSR WAITRACE
 JSR SHOWRESULTS      ; expect 01 02
 RTS

MNOREG ASC "NO REGISTRY"
 DFB $00
