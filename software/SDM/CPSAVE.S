* CPSAVE.S - C-flash host SAVE. Registers ONE service (racetask3 @ $0300) into
* the coproc registry exactly like CPRACE3.S (LOADBLK + TABLE entries), then
* triggers SAVE-to-flash (CP_SAVE=$5A @ $C0CF) and polls bit7 (save_busy) until
* it clears. Does NOT issue a GO (no CP_COUNT write) - CPSAVE only registers +
* persists. After a power-cycle the gateware auto-restores the registry; run
* CPBOOT to confirm + execute the service from flash.
*
* All STA CPWDATA are NON-INDEXED (STA abs,X dummy-read double-bumps the port).
 TYP $06
 DSK CPSAVE
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

* print $00-terminated high-bit msg at (SRCL),Y
PRMSG
 LDY #0
:lp LDA (SRCL),Y
 BEQ :done
 JSR COUT
 INY
 BNE :lp
:done RTS

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
* --- trigger SAVE-to-flash ---
 LDA #$5A
 STA CPSAVE
* --- poll bit7 (save_busy) until clear ---
:wait LDA CPSAVE
 AND #$80
 BNE :wait
* --- print "SAVED" + CR ---
 LDA #<MSAVED
 STA SRCL
 LDA #>MSAVED
 STA SRCH
 JSR PRMSG
 JSR CROUT
 RTS

MSAVED ASC "SAVED"
 DFB $00

* racetask3.bin (49 bytes), ORG $0300 - loaded verbatim into coproc BRAM
RT3    DFB $A4,$EC,$B6,$E8,$A9,$80,$38,$E9,$01,$D0,$FC,$CA,$D0,$F6,$78,$A5
       DFB $ED,$18,$69,$01,$85,$ED,$48,$98,$18,$65,$EE,$8D,$00,$E0,$A5,$EF
       DFB $69,$00,$8D,$01,$E0,$A9,$00,$8D,$02,$E0,$68,$8D,$03,$E0,$4C,$06
       DFB $10
RT3END
