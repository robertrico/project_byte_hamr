* =============================================================================
* slot_rom.S - project_obscurus $C400 slot ROM (Merlin32)
* =============================================================================
* PR#4 -> JSR $C400 -> ENTRY: ARM $C800 ($C0C7<-$AA), restore CSW=COUT1, JMP $C800.
* DISARM (fixed $C420) lives in the ALWAYS-LIVE slot ROM (gated only by
* nI_O_SELECT, never rom_armed) so it keeps executing after it disarms $C800.
* The monitor's Q JMPs here: disarm ($C0C8<-$AA), then RTS to BASIC over PR#'s
* JSR return address.
* =============================================================================
 TYP $06
 DSK slot_rom.bin
 ORG $C400

CSWL = $36
CSWH = $37
COUT1 = $FDF0
MONITOR = $C800
ROMARM = $C0C7
ROMDIS = $C0C8
MAGIC = $AA

ENTRY LDA #MAGIC
 STA ROMARM ; arm $C800
 SEI ; no IRQ during the armed session (IRQ+armed=$C800 collision)
 LDA #<COUT1
 STA CSWL
 LDA #>COUT1
 STA CSWH
 JMP MONITOR

 DS $C420-* ; pad so DISARM lands at fixed $C420
DISARM LDA #MAGIC
 STA ROMDIS ; disarm $C800 (runs from always-live slot ROM)
 CLI ; re-enable IRQ now that we are off the $C800 bus
 RTS ; back to BASIC
