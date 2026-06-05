* =============================================================================
* SDRAMLIB.S - project_obscurus SDRAM register-port driver (Merlin32)
* =============================================================================
* PUT-include library. NO ORG, NO zero page, NO PR#4 / $C800 / $C400 ROM.
* All access is plain absolute STA/LDA to $C0Cx (single nDEVICE_SELECT pulse;
* never indexed, to avoid the STA abs,X dummy-read double-count).
*
* Public params (set, then JSR an entry point):
*   SDM_BANK  (2 bytes)  10-bit bank (high 6 bits ignored by hardware)
*   SDM_ADDR  (2 bytes)  16-bit offset within bank
*   SDM_VAL   (1 byte)   value to write / last value read
*
* Entry points:
*   SDM_READY   bounded wait for ready. RETURNS C=0 ready, C=1 timeout.
*   SDM_SETBANK push SDM_BANK -> bank registers
*   SDM_SETADDR push SDM_ADDR -> addr registers
*   SDM_WRITE   setbank+setaddr, write SDM_VAL, wait busy clear
*   SDM_READ    setbank+setaddr, trigger read, wait, load -> A and SDM_VAL
*   SDM_WRNEXT  write SDM_VAL at hardware auto-inc addr (no setaddr), wait
*   SDM_RDNEXT  trigger read at auto-inc addr, wait, load -> A and SDM_VAL
* WRNEXT/RDNEXT require a prior SETADDR (or WRITE/READ) to set the start; the
* hardware then advances by one per completed access. No bank carry on wrap.
*
* Slot configurable by the single SLOT equate.
* =============================================================================

SLOT = 4
SDMBASE = SLOT*16+$C080 ; $C0C0 MERLIN EVALS LEFT-TO-RIGHT NO PRECEDENCE
R_ADRLO = SDMBASE+0
R_ADRHI = SDMBASE+1
R_BNKLO = SDMBASE+2
R_BNKHI = SDMBASE+3
R_TRIG = SDMBASE+4
R_STAT = SDMBASE+5
R_DATA = SDMBASE+6
R_ARM = SDMBASE+7 ; $C0C7 - write $AA to arm $C800 ROM
R_DISARM = SDMBASE+8 ; $C0C8 - write $AA to disarm

SDM_BANK DS 2
SDM_ADDR DS 2
SDM_VAL DS 1

* ---- bounded ready wait: C=0 ready, C=1 timeout (~65536 polls) ----
SDM_READY LDX #0
 LDY #0
:lp BIT R_STAT
 BVS :rdy ; bit6 (ready) -> V set
 INY
 BNE :lp
 INX
 BNE :lp
 SEC ; timed out
 RTS
:rdy CLC
 RTS

SDM_SETBANK
 LDA SDM_BANK
 STA R_BNKLO
 LDA SDM_BANK+1
 STA R_BNKHI
 RTS

SDM_SETADDR
 LDA SDM_ADDR
 STA R_ADRLO
 LDA SDM_ADDR+1
 STA R_ADRHI
 RTS

* ---- wait busy clear (bit7). ops always complete; no timeout ----
SDM_POLL LDA R_STAT
 BMI SDM_POLL ; bit7 (busy) -> N set -> loop
 RTS

SDM_WRITE JSR SDM_SETBANK
 JSR SDM_SETADDR
 LDA SDM_VAL
 STA R_DATA ; latch + trigger write
 JMP SDM_POLL ; tail call (RTS from POLL)

SDM_READ JSR SDM_SETBANK
 JSR SDM_SETADDR
 STA R_TRIG ; any write triggers a read (value ignored)
 JSR SDM_POLL
 LDA R_DATA
 STA SDM_VAL
 RTS

SDM_WRNEXT
 LDA SDM_VAL
 STA R_DATA
 JMP SDM_POLL

SDM_RDNEXT
 STA R_TRIG
 JSR SDM_POLL
 LDA R_DATA
 STA SDM_VAL
 RTS

* ---- $C800 expansion-ROM arm/disarm. ONLY needed to run ROM-resident code;
*      SDRAM data access (SDM_READ/WRITE/etc.) NEVER needs this. ----
SDM_ROMON LDA #$AA
 STA R_ARM
 RTS
SDM_ROMOFF
 LDA #$AA
 STA R_DISARM
 RTS
