# Farm + Market Event Game Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

> **Status:** Tasks 1-10 executed on branch farm-game (M1 sim-verified 38 PASS / 0 FAIL). M2-M5 are bench milestones — pending hardware session.

**Goal:** SDRAM event ring + command mailbox protocols on the C4 coproc, proven by a playable Lo-Res farm/market game (spec: `docs/superpowers/specs/2026-06-10-farm-market-event-game-design.md`).

**Architecture:** One resident coproc task (GAMETASK, skill id 2, ORG $0600, budget=0 forever loop) is the sole writer of game bank 32: it consumes a command mailbox, ticks crop growth and a price drift on divider counters, and publishes EV_RIPE/EV_PRICE to a 64-record ring. The //e FARM bin is a viewport: GR mixed-mode grid, event drain, periodic grid re-stream, keyboard commands.

**Tech Stack:** Merlin32 6502 assembly (single-space format, ASCII, ≤50-char lines), iverilog -g2005 sim via `project_obscurus_tb.v`, GNU Make.

**No gateware changes.** All work is in `software/SDM/`, the Makefile, and the existing testbench.

---

## Plan-time facts (verified against source, 2026-06-10)

- Skill table = coproc BRAM `$0200 + id*2` (lo,hi). Conway uses id 0 (LIFE8 @$0300) and id 1 (LIFE1 @$0303). **GAMETASK = skill id 2 → table bytes at $0204/$0205.** LIFE8GR blob is 680 bytes from $0300 (last byte $05A7), so ORG $0600 is clear; mailboxes start $0F80.
- `CP_CALL` (CPLIB.S) picks the lowest **clear** bit of `CP_ACTIVE` — it can never overwrite a running slot. Cold-start-over-slow-task worst case = duplicate task in a second slot (two writers). Mitigated by the 3 s STATUS probe; accepted residual risk per spec.
- Coproc SDRAM ports: `SADDRLO/HI $E000/1, SBANKR $E002, SDATA $E003` (write: set address **per byte** — LIFE8GR WRITEROW precedent), `RADDRLO/HI $E004/5, RBANKR $E006, RTRIG $E007, RDATA $E008` (read: address **auto-increments** after each RTRIG — LIFE8GR READROW precedent).
- Host side: `SDRAMLIB.S` (`SDM_BANK/ADDR/VAL` params; `SDM_READY/SETBANK/SETADDR/WRITE/READ/WRNEXT/RDNEXT`), `CPLIB.S` (`CP_CALL` A=skill X=arg0 Y=budget → handle; `CP_POLL/WAIT/RESULT`), CP load ports `$C0C9/A/B`.
- tb (`project_obscurus_tb.v`, 1120 lines) already provides: `wr_reg/rd_reg`, `sdram_write/read(bank,addr,d)`, `load_byte` (CP_WDATA autoinc), `cp_read`, `stage_mbox(slot,skill,budget,arg0)`, `ring(slot)`, `collect(slot)`, `wait_done`. Skill images load via `$readmemh` into a `reg [7:0] img[]` then stream through `load_byte`.
- Blob auto-gen pattern (Makefile lines 503-518): sed LSIM toggle → Merlin32 → `od|awk` DFB include with `CSKILL/CSKEND/CSKLEN` labels.
- `make sim` builds `$(DESIGN)_tb` from all `*_tb.v` + copies all `$(DESIGN_DIR)/*.mem` into the build dir. iverilog **-g2005 only** (-g2009/-g2012 hang on this design).
- Merlin gotchas: `"X"` literals assemble high-bit set; `DO/FIN` conditional works (LIFE8GR LSIM); local labels `:name` OK (SDRAMLIB/CPLIB use them).

## File structure

```
software/SDM/FARMEQU.S     shared equates only (no code) — PUT by all below
software/SDM/EVLIB.S       PUTEV coproc-side event publish (PUT-include, no ORG)
software/SDM/FARMTASK.S    GAMETASK coproc blob, ORG $0600 (PUTs FARMEQU, EVLIB)
software/SDM/FARMTASKB.S   AUTO-GENERATED DFB include (never hand-edited, git-ignored OK to commit)
software/SDM/FARM.S        //e game, ORG $6000 (PUTs FARMEQU, SDRAMLIB, CPLIB, FARMTASKB)
gateware/rev2/project_obscurus/farmtask.mem   AUTO-GENERATED sim blob (tiny dividers)
gateware/rev2/project_obscurus/project_obscurus_tb.v   MODIFY: append farm protocol tests
Makefile                   MODIFY: farmtask/farmtaskb/farm/farmsim targets + sdmdisk entries
```

---

### Task 1: FARMEQU.S — shared equates

**Files:**
- Create: `software/SDM/FARMEQU.S`

- [x] **Step 1: Write the file**

```
* FARMEQU.S - FARM GAME SHARED EQUATES
* PUT-INCLUDE. EQUATES ONLY. NO CODE.
* GBANK LAYOUT (SDRAM BANK 32):
*  $0000 SIG(2) $0002 SEQCTR $0003 HEAD
*  $0100 RING 64x4 (PAGE-ALIGNED: REC ADDR
*   HI CONST $01, LO = IDX*4 - NO CARRY)
*  $0200 MAILBOX(6) $0210 MARKET(7)
*  $0300 GRID 20x20
GBANK = 32
FSIG0 = $0000
FSIG1 = $0001
FSEQC = $0002
FHEAD = $0003
FRING = $0100
FMFLAG = $0200
FMOP = $0201
FMA0 = $0202
FMA1 = $0203
FMA2 = $0204
FMRES = $0205
FPRICEL = $0210
FPRICEH = $0211
FSUPPLY = $0212
FCASHL = $0213
FCASHH = $0214
FSEEDS = $0215
FCROPS = $0216
FGRID = $0300
* GRID GEOMETRY
GCOLS = 20
GROWSN = 20
GPLOTS = 400
* PLOT STAGES: 0 EMPTY 1-5 GROW 6 RIPE
STRIPE = 6
* SIGNATURE BYTES "FM"
SIGF = $46
SIGM = $4D
* OPS
OPSTAT = $00
OPPLANT = $01
OPHARV = $02
OPSELL = $03
OPBUY = $04
* RESULTS
ROK = $01
RERROCC = $E1
RERRRIPE = $E2
RERRSEED = $E3
RERRCASH = $E4
RERRCROP = $E5
RERRBAD = $E6
RERRFULL = $E7
* EVENTS
EVRIPE = $01
EVPRICE = $02
* ECONOMY
PBASE = 10
PFLOOR = 2
SEEDCOST = 3
CASH0 = 100
SEEDS0 = 5
* DIVIDERS (24-BIT DOWN-COUNTERS, LO/MID/HI)
* HW DEFAULTS ~SECONDS-SCALE, TUNED IN M5.
* FSIM=1 -> TINY (SIM BUILD SED-TOGGLES FSIM)
FSIM = 0
 DO FSIM
* sim: one grow tick = 128 passes. NOT smaller:
* a tb mailbox round-trip is ~10+ passes, and
* the error-path tests (occupied/unripe) must
* run before plot (3,3) ripens (5 ticks).
GROWD0 = $80
GROWD1 = $00
GROWD2 = $00
MKTD0 = $40
MKTD1 = $00
MKTD2 = $00
 FIN
 DO 1-FSIM
GROWD0 = $00
GROWD1 = $00
GROWD2 = $30
MKTD0 = $00
MKTD1 = $80
MKTD2 = $00
 FIN
* SUPPLY DECAY: 1 EVERY DECAYDIV MKT TICKS
DECAYDIV = 4
* COPROC SDRAM PORTS
SADDRLO = $E000
SADDRHI = $E001
SBANKR = $E002
SDATA = $E003
RADDRLO = $E004
RADDRHI = $E005
RBANKR = $E006
RTRIG = $E007
RDATA = $E008
```

Notes for the implementer:
- `FSIM` mirrors the LIFE8GR `LSIM` pattern: committed source has `FSIM = 0` (hardware dividers); the sim build seds `FSIM = 0` → `FSIM = 1`.
- `GROWSN` not `GROWS` — LIFE8GR already claims `GROWS`; if both blobs were ever assembled into one listing the collision would be silent.
- SDRAM port equates live here (not duplicated in FARMTASK.S) because EVLIB.S needs them too. LIFEMAPGR.S also defines them; FARMTASK must NOT also PUT LIFEMAPGR (duplicate-symbol error).

- [x] **Step 2: Assemble-check it parses (PUT into a throwaway shell)**

```bash
cd software/SDM && printf ' TYP $06\n DSK eqtest.bin\n ORG $0600\n PUT FARMEQU\n NOP\n' > EQTEST.S && Merlin32 -V . EQTEST.S && rm -f EQTEST.S eqtest.bin eqtest_Output.txt
```

(Use the exact Merlin32 invocation from the Makefile: `$(MERLIN32) $(MERLIN_LIB)` — check `grep MERLIN32 Makefile` for the binary path/flags and substitute.)
Expected: assembles with no errors.

- [x] **Step 3: Commit**

```bash
git add software/SDM/FARMEQU.S
git commit -m "feat(farm): FARMEQU shared equates - GBANK map, ops, events, dividers"
```

---

### Task 2: EVLIB.S — PUTEV event publish

**Files:**
- Create: `software/SDM/EVLIB.S`

PUTEV contract (spec-normative order): write 4 record bytes (SEQ=SEQCTR), then SEQCTR++, then HEAD++ last. Caller sets `EVTYPE/EVP0/EVP1`. Uses absolute scratch only (no ZP — kernel owns it). Each port burst SEI/CLI-bracketed. Write port needs the address re-set per byte; the ring is page-aligned at $0100, so the record burst keeps SADDRHI constant at >FRING and runs an 8-bit lo byte (`HEAD*4` max 252 + 3 = 255 — no carry possible by construction).

- [x] **Step 1: Write the file**

```
* EVLIB.S - COPROC EVENT RING PUBLISH
* PUT-INCLUDE. NO ORG. NEEDS FARMEQU.
* CALLER SETS EVTYPE EVP0 EVP1, JSR PUTEV.
* ORDER (NORMATIVE): RECORD, SEQCTR++, HEAD++.
* ABS SCRATCH $0B00+ (NO ZP - KERNEL OWNS ZP)
EVTYPE = $0B00
EVP0 = $0B01
EVP1 = $0B02
EVHEAD = $0B03
EVSEQ = $0B04
EVRLO = $0B05

PUTEV
* read SEQCTR and HEAD (one burst)
 SEI
 LDA #FSEQC
 STA RADDRLO
 LDA #$00
 STA RADDRHI
 LDA #GBANK
 STA RBANKR
 STA RTRIG
 LDA RDATA
 STA EVSEQ
 STA RTRIG
 LDA RDATA
 STA EVHEAD
 CLI
* record lo = HEAD*4 (ring page-aligned at
* $0100: hi const, max lo 63*4=252, no carry)
 LDA EVHEAD
 ASL
 ASL
 STA EVRLO
* write record: SEQ TYPE P0 P1 (hi = >FRING)
 SEI
 LDA #GBANK
 STA SBANKR
 LDA #>FRING
 STA SADDRHI
 LDA EVRLO
 STA SADDRLO
 LDA EVSEQ
 STA SDATA
 INC EVRLO
 LDA EVRLO
 STA SADDRLO
 LDA EVTYPE
 STA SDATA
 INC EVRLO
 LDA EVRLO
 STA SADDRLO
 LDA EVP0
 STA SDATA
 INC EVRLO
 LDA EVRLO
 STA SADDRLO
 LDA EVP1
 STA SDATA
* SEQCTR++ THEN HEAD++ (LAST) - PAGE 0 AGAIN
 LDA #$00
 STA SADDRHI
 LDA #FSEQC
 STA SADDRLO
 LDA EVSEQ
 CLC
 ADC #1
 STA SDATA
 LDA #FHEAD
 STA SADDRLO
 LDA EVHEAD
 CLC
 ADC #1
 AND #$3F
 STA SDATA
 CLI
 RTS
```

Note: the record burst runs with SADDRHI = >FRING ($01) and only SADDRLO re-set per byte (page-aligned ring, no carry possible); the SEQCTR/HEAD writes switch SADDRHI back to 0. The whole record+SEQCTR+HEAD burst sits in ONE SEI/CLI bracket deliberately: it is 6 port writes (~30 µs), and publishing atomically means a preempting task can never observe HEAD published without SEQCTR (keeps the resync pair-read hole to the documented benign case).

- [x] **Step 2: Commit**

```bash
git add software/SDM/EVLIB.S
git commit -m "feat(farm): EVLIB - PUTEV ring publish, normative record/SEQCTR/HEAD order"
```

(Assembles only as part of FARMTASK — next task verifies.)

---

### Task 3: FARMTASK.S — GAMETASK blob (full)

**Files:**
- Create: `software/SDM/FARMTASK.S`

Single forever-loop task. Every pass: mailbox; growth divider; market divider. All scratch absolute at $0B10+ (EVLIB owns $0B00-$0B05; LIFE8GR owns $0C00-$0D67 — stay below $0C00).

- [x] **Step 1: Write the file**

```
* FARMTASK.S - FARM GAME COPROC TASK
* SKILL ID 2, ORG $0600, BUDGET=0 FOREVER.
* SOLE WRITER OF GBANK. SEI/CLI PER PORT BURST.
* EVERY PASS: MAILBOX. DIVIDED: GROW, MARKET.
 TYP $06
 DSK FARMTASK.bin
 ORG $0600
 PUT FARMEQU
* ABS SCRATCH (EVLIB USES $0B00-$0B05)
GROWC0 = $0B10
GROWC1 = $0B11
GROWC2 = $0B12
MKTC0 = $0B13
MKTC1 = $0B14
MKTC2 = $0B15
DECAYC = $0B16
TOP = $0B17
TA0 = $0B18
TA1 = $0B19
TA2 = $0B1A
TRES = $0B1B
PLOTV = $0B1C
PADRL = $0B1D
PADRH = $0B1E
PX = $0B1F
PY = $0B20
QTY = $0B21
PRC = $0B22
SUP = $0B23
CSHL = $0B24
CSHH = $0B25
SEED = $0B26
CROP = $0B27
TGT = $0B28
M20L = $0B29
M20H = $0B2A

 JMP GAME

* === MAIN FOREVER LOOP ===
GAME
 JSR RELOADG
 JSR RELOADM
 LDA #0
 STA DECAYC
GLOOP
 JSR DOMBOX
* growth 24-bit down-counter
 LDA GROWC0
 SEC
 SBC #1
 STA GROWC0
 LDA GROWC1
 SBC #0
 STA GROWC1
 LDA GROWC2
 SBC #0
 STA GROWC2
 ORA GROWC1
 ORA GROWC0
 BNE GNOG
 JSR DOGROW
 JSR RELOADG
GNOG
* market 24-bit down-counter
 LDA MKTC0
 SEC
 SBC #1
 STA MKTC0
 LDA MKTC1
 SBC #0
 STA MKTC1
 LDA MKTC2
 SBC #0
 STA MKTC2
 ORA MKTC1
 ORA MKTC0
 BNE GNOM
 JSR DOMKT
 JSR RELOADM
GNOM
 JMP GLOOP

RELOADG
 LDA #GROWD0
 STA GROWC0
 LDA #GROWD1
 STA GROWC1
 LDA #GROWD2
 STA GROWC2
 RTS
RELOADM
 LDA #MKTD0
 STA MKTC0
 LDA #MKTD1
 STA MKTC1
 LDA #MKTD2
 STA MKTC2
 RTS

* === PORT HELPERS ===
* RDB: read GBANK byte at addr A(lo) Y(hi) -> A
RDB
 SEI
 STA RADDRLO
 STY RADDRHI
 LDA #GBANK
 STA RBANKR
 STA RTRIG
 LDA RDATA
 CLI
 RTS
* WRB: write X to GBANK addr A(lo) Y(hi)
WRB
 SEI
 STA SADDRLO
 STY SADDRHI
 LDA #GBANK
 STA SBANKR
 TXA
 STA SDATA
 CLI
 RTS

* === DOMBOX: consume one command if flagged ===
DOMBOX
 LDA #<FMFLAG
 LDY #>FMFLAG
 JSR RDB
 BNE MBGO
 RTS
MBGO
* read OP A0 A1 A2 (auto-inc after FLAG read?
* no: separate reads, addresses explicit)
 LDA #<FMOP
 LDY #>FMOP
 JSR RDB
 STA TOP
 LDA #<FMA0
 LDY #>FMA0
 JSR RDB
 STA TA0
 LDA #<FMA1
 LDY #>FMA1
 JSR RDB
 STA TA1
 LDA #<FMA2
 LDY #>FMA2
 JSR RDB
 STA TA2
* dispatch
 LDA TOP
 BEQ CSTAT
 CMP #OPPLANT
 BEQ CPLANT
 CMP #OPHARV
 BEQ CHARV
 CMP #OPSELL
 BEQ JSELL
 CMP #OPBUY
 BEQ JBUY
 LDA #RERRBAD
 JMP MBFIN
JSELL
 JMP CSELL
JBUY
 JMP CBUY
CSTAT
 LDA #ROK
 JMP MBFIN

* --- PLANT TA0=x TA1=y ---
CPLANT
 JSR PLOTADR
 JSR RDPLOT
 BEQ PLOK
 LDA #RERROCC
 JMP MBFIN
PLOK
 LDA #<FSEEDS
 LDY #>FSEEDS
 JSR RDB
 BNE PLSEED
 LDA #RERRSEED
 JMP MBFIN
PLSEED
 SEC
 SBC #1
 TAX
 LDA #<FSEEDS
 LDY #>FSEEDS
 JSR WRB
 LDX #1
 JSR WRPLOT
 LDA #ROK
 JMP MBFIN

* --- HARVEST TA0=x TA1=y ---
CHARV
 JSR PLOTADR
 JSR RDPLOT
 CMP #STRIPE
 BEQ HVOK
 LDA #RERRRIPE
 JMP MBFIN
HVOK
 LDA #<FCROPS
 LDY #>FCROPS
 JSR RDB
 CMP #$FF
 BNE HVROOM
 LDA #RERRFULL
 JMP MBFIN
HVROOM
 CLC
 ADC #1
 TAX
 LDA #<FCROPS
 LDY #>FCROPS
 JSR WRB
 LDX #0
 JSR WRPLOT
 LDA #ROK
 JMP MBFIN

* --- SELL TA0=qty (qty=0 -> ERR_BAD:
* DEC-first loops would run 256x) ---
CSELL
 LDA TA0
 BNE CSELQ
 LDA #RERRBAD
 JMP MBFIN
CSELQ
 LDA #<FCROPS
 LDY #>FCROPS
 JSR RDB
 CMP TA0
 BCS SLOK
 LDA #RERRCROP
 JMP MBFIN
SLOK
 SEC
 SBC TA0
 TAX
 LDA #<FCROPS
 LDY #>FCROPS
 JSR WRB
* cash += qty*price (loop), clamp $FFFF
 LDA #<FPRICEL
 LDY #>FPRICEL
 JSR RDB
 STA PRC
 LDA #<FCASHL
 LDY #>FCASHL
 JSR RDB
 STA CSHL
 LDA #<FCASHH
 LDY #>FCASHH
 JSR RDB
 STA CSHH
 LDA TA0
 STA QTY
SLMUL
 LDA CSHL
 CLC
 ADC PRC
 STA CSHL
 LDA CSHH
 ADC #0
 STA CSHH
 BCC SLNC
 LDA #$FF
 STA CSHL
 STA CSHH
SLNC
 DEC QTY
 BNE SLMUL
 LDX CSHL
 LDA #<FCASHL
 LDY #>FCASHL
 JSR WRB
 LDX CSHH
 LDA #<FCASHH
 LDY #>FCASHH
 JSR WRB
* supply += qty clamp 255
 LDA #<FSUPPLY
 LDY #>FSUPPLY
 JSR RDB
 CLC
 ADC TA0
 BCC SPNC
 LDA #$FF
SPNC
 TAX
 LDA #<FSUPPLY
 LDY #>FSUPPLY
 JSR WRB
 LDA #ROK
 JMP MBFIN

* --- BUYSEED TA0=qty (qty=0 -> ERR_BAD) ---
CBUY
 LDA TA0
 BNE CBUYQ
 LDA #RERRBAD
 JMP MBFIN
CBUYQ
* room: SEEDS+qty must not exceed 255
 LDA #<FSEEDS
 LDY #>FSEEDS
 JSR RDB
 STA SEED
 CLC
 ADC TA0
 BCC BYROOM
 LDA #RERRFULL
 JMP MBFIN
BYROOM
* cost = qty*SEEDCOST (loop) -> M20L/H
 LDA #0
 STA M20L
 STA M20H
 LDA TA0
 STA QTY
BYCST
 LDA M20L
 CLC
 ADC #SEEDCOST
 STA M20L
 LDA M20H
 ADC #0
 STA M20H
 DEC QTY
 BNE BYCST
* cash >= cost?
 LDA #<FCASHL
 LDY #>FCASHL
 JSR RDB
 STA CSHL
 LDA #<FCASHH
 LDY #>FCASHH
 JSR RDB
 STA CSHH
 CMP M20H
 BCC BYPOOR
 BNE BYRICH
 LDA CSHL
 CMP M20L
 BCC BYPOOR
BYRICH
* full 16-bit subtract into scratch FIRST -
* never carry a borrow across a JSR
 LDA CSHL
 SEC
 SBC M20L
 STA CSHL
 LDA CSHH
 SBC M20H
 STA CSHH
 LDX CSHL
 LDA #<FCASHL
 LDY #>FCASHL
 JSR WRB
 LDX CSHH
 LDA #<FCASHH
 LDY #>FCASHH
 JSR WRB
 LDA SEED
 CLC
 ADC TA0
 TAX
 LDA #<FSEEDS
 LDY #>FSEEDS
 JSR WRB
 LDA #ROK
 JMP MBFIN
BYPOOR
 LDA #RERRCASH
 JMP MBFIN

* --- MBFIN: A=result. RESULT then FLAG=0 ---
MBFIN
 TAX
 LDA #<FMRES
 LDY #>FMRES
 JSR WRB
 LDX #0
 LDA #<FMFLAG
 LDY #>FMFLAG
 JSR WRB
 RTS

* === PLOTADR: PADR = FGRID + TA1*20 + TA0 ===
* y*20 = y*16 + y*4, computed 16-BIT (19*16=304
* overflows 8 bits - corner test (19,19) guards)
PLOTADR
 LDA TA1
 STA M20L
 LDA #0
 STA M20H
 ASL M20L
 ROL M20H
 ASL M20L
 ROL M20H
* M20 = y*4, save copy in PADR
 LDA M20L
 STA PADRL
 LDA M20H
 STA PADRH
 ASL M20L
 ROL M20H
 ASL M20L
 ROL M20H
* M20 = y*16; add saved y*4
 LDA M20L
 CLC
 ADC PADRL
 STA M20L
 LDA M20H
 ADC PADRH
 STA M20H
* + x
 LDA M20L
 CLC
 ADC TA0
 STA PADRL
 LDA M20H
 ADC #0
 STA PADRH
* + FGRID base
 LDA PADRL
 CLC
 ADC #<FGRID
 STA PADRL
 LDA PADRH
 ADC #>FGRID
 STA PADRH
 RTS
RDPLOT
 LDA PADRL
 LDY PADRH
 JSR RDB
 RTS
WRPLOT
 LDA PADRL
 LDY PADRH
 JSR WRB
 RTS

* === DOGROW: advance all 400 plots ===
* walk PADR from FGRID, PX/PY track coords
DOGROW
 LDA #<FGRID
 STA PADRL
 LDA #>FGRID
 STA PADRH
 LDA #0
 STA PX
 STA PY
GRLP
 LDA PADRL
 LDY PADRH
 JSR RDB
 STA PLOTV
 BEQ GRNEXT
 CMP #STRIPE
 BEQ GRNEXT
 CLC
 ADC #1
 STA PLOTV
 TAX
 LDA PADRL
 LDY PADRH
 JSR WRB
 LDA PLOTV
 CMP #STRIPE
 BNE GRNEXT
* publish EV_RIPE x,y
 LDA #EVRIPE
 STA EVTYPE
 LDA PX
 STA EVP0
 LDA PY
 STA EVP1
 JSR PUTEV
GRNEXT
 INC PADRL
 BNE GRNC
 INC PADRH
GRNC
 INC PX
 LDA PX
 CMP #GCOLS
 BNE GRLP
 LDA #0
 STA PX
 INC PY
 LDA PY
 CMP #GROWSN
 BNE GRLP
 RTS

* === DOMKT: price step + supply decay ===
DOMKT
 LDA #<FSUPPLY
 LDY #>FSUPPLY
 JSR RDB
 STA SUP
 LDA #<FPRICEL
 LDY #>FPRICEL
 JSR RDB
 STA PRC
* TGT = BASE - SUP/2, compare-first 8-bit
 LDA SUP
 LSR
 CMP #PBASE-PFLOOR
 BCC TGCALC
 LDA #PFLOOR
 STA TGT
 JMP TGDONE
TGCALC
 STA TGT
 LDA #PBASE
 SEC
 SBC TGT
 STA TGT
TGDONE
* step PRICE 1 toward TGT
 LDA PRC
 CMP TGT
 BEQ MKDECAY
 BCC MKUP
 DEC PRC
 JMP MKWR
MKUP
 INC PRC
MKWR
 LDX PRC
 LDA #<FPRICEL
 LDY #>FPRICEL
 JSR WRB
* publish EV_PRICE (lo, hi=0)
 LDA #EVPRICE
 STA EVTYPE
 LDA PRC
 STA EVP0
 LDA #0
 STA EVP1
 JSR PUTEV
MKDECAY
 INC DECAYC
 LDA DECAYC
 CMP #DECAYDIV
 BNE MKDONE
 LDA #0
 STA DECAYC
 LDA SUP
 BEQ MKDONE
 SEC
 SBC #1
 TAX
 LDA #<FSUPPLY
 LDY #>FSUPPLY
 JSR WRB
MKDONE
 RTS

 PUT EVLIB
```

**Implementer notes:**
- PLOTADR is 16-bit on purpose (19*16=304 overflows 8 bits). The sim test in Task 5 asserts plot (3,3) → GBANK $0300+63 = $033F and plot (19,19) → $0300+399 = $048F; both must hold.
- Branch-range check: the dispatch `BEQ/CMP` chain reaches across the PLANT/HARVEST bodies — `JSELL/JBUY` trampolines are there because CSELL/CBUY are >127 bytes away. If Merlin32 reports branch out of range anywhere else, insert the same `Jxxx JMP target` trampoline pattern.
- Blob size estimate ~900 bytes → $0600-$09xx, well under the $0F80 mailboxes and clear of $0B00 scratch.

- [x] **Step 2: Assemble**

```bash
cd software/SDM && Merlin32 -V . FARMTASK.S
```
Expected: `FARMTASK.bin` produced, no errors. Check size: `wc -c FARMTASK.bin` — expect 700-1100 bytes; MUST be < 2432 ($0600+size ≤ $0F80).

- [x] **Step 3: Commit**

```bash
git add software/SDM/FARMTASK.S
git commit -m "feat(farm): FARMTASK - mailbox dispatch, growth scan, market drift, forever loop"
```

---

### Task 4: Makefile targets

**Files:**
- Modify: `Makefile` (after the `life8gr:` target block, ~line 528, and the `sdmdisk` block ~line 558)

- [x] **Step 1: Add build rules**

```make
# FARMTASK sim blob (FSIM=1 tiny dividers) -> farmtask.mem for the tb.
# Committed FARMTASK.S keeps FSIM=0 (hardware dividers); sim variant is
# generated, never hand-edited. Pattern mirrors LIFE8GRHW (lines 503-516).
FARMTASKSIM_BIN := $(SDM_DIR)/FARMTASKSIM.bin
FARMTASK_MEM := $(GATEWARE_DIR)/project_obscurus/farmtask.mem

$(FARMTASKSIM_BIN): $(SDM_DIR)/FARMTASK.S $(SDM_DIR)/FARMEQU.S $(SDM_DIR)/EVLIB.S
	sed -e 's/^ DSK FARMTASK.bin/ DSK FARMTASKSIM.bin/' $(SDM_DIR)/FARMTASK.S > $(SDM_DIR)/FARMTASKSIM.S
	sed -e 's/^FSIM = 0/FSIM = 1/' $(SDM_DIR)/FARMEQU.S > $(SDM_DIR)/FARMEQUS.S
	cd $(SDM_DIR) && sed -e 's/ PUT FARMEQU$$/ PUT FARMEQUS/' FARMTASKSIM.S > FARMTASKSIM.tmp && mv FARMTASKSIM.tmp FARMTASKSIM.S && $(MERLIN32) $(MERLIN_LIB) FARMTASKSIM.S

$(FARMTASK_MEM): $(FARMTASKSIM_BIN)
	python3 -c "b=open('$(FARMTASKSIM_BIN)','rb').read(); open('$(FARMTASK_MEM)','w').write('\n'.join('%02x'%x for x in b)+'\n')"

farmtasksim: $(FARMTASK_MEM)

# stale-artifact guard (the hamr_rom.mem lesson): editing FARMTASK.S must
# rebuild farmtask.mem before any project_obscurus sim run.
ifeq ($(DESIGN),project_obscurus)
$(SIM_OUT): $(FARMTASK_MEM)
endif

# FARMTASK HW blob (FSIM=0) -> DFB include for FARM.S
FARMTASK_BIN := $(SDM_DIR)/FARMTASK.bin
FARMTASKB_S := $(SDM_DIR)/FARMTASKB.S

$(FARMTASK_BIN): $(SDM_DIR)/FARMTASK.S $(SDM_DIR)/FARMEQU.S $(SDM_DIR)/EVLIB.S
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) FARMTASK.S

$(FARMTASKB_S): $(FARMTASK_BIN)
	{ echo 'FSKILL'; od -An -tx1 -v $< | awk '{for(i=1;i<=NF;i++)printf " DFB $$%s\n",toupper($$i)}'; echo 'FSKEND'; echo 'FSKLEN = FSKEND-FSKILL'; } > $@

farm: $(FARMTASKB_S)
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) FARM.S
```

Also add `farmtasksim farm` to the `.PHONY` list (line 43) and these two lines inside the `sdmdisk` recipe after the GRVERSE import (~line 576):

```make
sdmdisk: ... farm     # append 'farm' to the prerequisite list
	$(AC_CLASSIC) -p $(SDM_PO) FARM BIN 0x6000 < $(SDM_DIR)/FARM.bin
```

- [x] **Step 2: Verify the sim-blob rule works (FARM.S doesn't exist yet — only run farmtasksim)**

```bash
make farmtasksim && head -3 gateware/rev2/project_obscurus/farmtask.mem && wc -l gateware/rev2/project_obscurus/farmtask.mem
```
Expected: hex lines; line count = FARMTASKSIM.bin byte count. The `make sim` MEM_FILES wildcard picks farmtask.mem up automatically (Makefile line 127/349).

- [x] **Step 3: Commit**

```bash
git add Makefile
git commit -m "build(farm): farmtasksim mem + FARMTASKB DFB autogen + farm/sdmdisk targets"
```

---

### Task 5: tb — register, spawn, STATUS, PLANT/HARVEST, growth, EV_RIPE (M1 a,b,e + errors)

**Files:**
- Modify: `gateware/rev2/project_obscurus/project_obscurus_tb.v` (append after the GR-oracle section; new helper tasks near the other task definitions ~line 300; test calls before the final PASS/FAIL summary in the `initial` block)

- [x] **Step 1: Add farm image + helper tasks**

Near the LIFE8GR image declarations (~line 99):

```verilog
    // ---- FARM game protocol (skill 2 @ $0600, GBANK=32) ----
    localparam integer FARMLEN = 1100;   // ADJUST to actual FARMTASKSIM.bin size
    reg [7:0] farmimg [0:2047];
    initial $readmemh("farmtask.mem", farmimg);
```

Near the other tasks (~line 300):

```verilog
    // load FARMTASK blob -> coproc BRAM $0600, table[2]=$0600
    task farm_load;
        integer i;
    begin
        wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h06);      // CP_LADDR = $0600
        for (i=0; i<FARMLEN; i=i+1) load_byte(farmimg[i]);
        wr_reg(4'h9, 8'h04); wr_reg(4'hA, 8'h02);      // table[2] @ $0204
        load_byte(8'h00); load_byte(8'h06);            // vector = $0600
    end endtask

    // init GBANK cold-start state. SIG written here too (tb rig writes it
    // up front; FARM.S cold start writes SIG last - the liveness ordering
    // matters on hardware, not in this single-threaded rig). SIG also serves
    // as the lap-test regression net: ring page-wrap bugs land on $0000-$0003.
    task farm_init;
        integer i;
    begin
        sdram_write(10'd32, 16'h0000, 8'h46);          // SIG 'F'
        sdram_write(10'd32, 16'h0001, 8'h4D);          // SIG 'M'
        sdram_write(10'd32, 16'h0002, 8'h00);          // SEQCTR
        sdram_write(10'd32, 16'h0003, 8'h00);          // HEAD
        sdram_write(10'd32, 16'h0200, 8'h00);          // MFLAG
        sdram_write(10'd32, 16'h0210, 8'd10);          // PRICEL=BASE
        sdram_write(10'd32, 16'h0211, 8'h00);
        sdram_write(10'd32, 16'h0212, 8'h00);          // SUPPLY
        sdram_write(10'd32, 16'h0213, 8'd100);         // CASHL
        sdram_write(10'd32, 16'h0214, 8'h00);
        sdram_write(10'd32, 16'h0215, 8'd5);           // SEEDS
        sdram_write(10'd32, 16'h0216, 8'h00);          // CROPS
        for (i=0; i<400; i=i+1) sdram_write(10'd32, 16'h0300+i, 8'h00);
    end endtask

    // send one command, poll FLAG clear, return RESULT. ok=0 on timeout.
    task farm_cmd(input [7:0] op, input [7:0] a0, input [7:0] a1,
                  input [7:0] a2, output [7:0] res, output ok);
        integer t; reg [7:0] f;
    begin
        sdram_write(10'd32, 16'h0201, op);
        sdram_write(10'd32, 16'h0202, a0);
        sdram_write(10'd32, 16'h0203, a1);
        sdram_write(10'd32, 16'h0204, a2);
        sdram_write(10'd32, 16'h0200, 8'h01);          // FLAG last
        ok = 0; res = 8'hFF;
        for (t=0; t<200000 && !ok; t=t+1) begin
            sdram_read(10'd32, 16'h0200, f);
            if (f == 8'h00) ok = 1;
        end
        if (ok) sdram_read(10'd32, 16'h0205, res);
    end endtask

    // reader-rule drain: from farm_tail/farm_seq, dispatch into ev arrays
    reg [7:0] farm_tail = 0, farm_seq = 0;
    integer farm_nev = 0;
    reg [7:0] ev_type [0:255]; reg [7:0] ev_p0 [0:255]; reg [7:0] ev_p1 [0:255];
    integer farm_resyncs = 0;
    task farm_drain;
        reg [7:0] h, s, s2, rs, rt, rp0, rp1; integer guard;
    begin
        sdram_read(10'd32, 16'h0003, h);
        guard = 0;
        while (farm_tail !== h && guard < 128) begin
            sdram_read(10'd32, 16'h0100 + farm_tail*4 + 0, rs);
            sdram_read(10'd32, 16'h0100 + farm_tail*4 + 1, rt);
            sdram_read(10'd32, 16'h0100 + farm_tail*4 + 2, rp0);
            sdram_read(10'd32, 16'h0100 + farm_tail*4 + 3, rp1);
            if (rs !== farm_seq) begin
                // overrun -> resync: stable (SEQCTR,HEAD) pair
                farm_resyncs = farm_resyncs + 1;
                s2 = 8'hFF;
                while (s2 !== s) begin
                    sdram_read(10'd32, 16'h0002, s);
                    sdram_read(10'd32, 16'h0003, h);
                    sdram_read(10'd32, 16'h0002, s2);
                end
                farm_tail = h; farm_seq = s;
            end else begin
                ev_type[farm_nev]=rt; ev_p0[farm_nev]=rp0; ev_p1[farm_nev]=rp1;
                farm_nev = farm_nev + 1;
                farm_tail = (farm_tail + 1) & 8'h3F;
                farm_seq = farm_seq + 1;
            end
            guard = guard + 1;
        end
    end endtask
```

- [x] **Step 2: Add the M1 test sequence in the `initial` block (before the final summary)**

```verilog
        // ===== FARM: event ring + mailbox protocol (skill 2, GBANK 32) =====
        $display("--- FARM protocol tests ---");
        farm_init;
        farm_load;
        stage_mbox(2'd0, 8'd2, 8'd0, 8'd0);   // skill 2, budget 0 forever
        ring(2'd0);
        // (a)-(e): named block - V2005 needs names for local declarations
        begin : farm_m1
        reg [7:0] r; reg ok;
        // (a) STATUS probe
        farm_cmd(8'h00, 8'h00, 8'h00, 8'h00, r, ok);
        if (!ok || r!==8'h01) begin errors=errors+1; $display("FAIL farm STATUS r=%h ok=%b", r, ok); end
        else $display("PASS farm STATUS");
        // (b) PLANT 3,3 -> plot $033F = 1, SEEDS 4
        farm_cmd(8'h01, 8'd3, 8'd3, 8'h00, r, ok);
        if (!ok || r!==8'h01) begin errors=errors+1; $display("FAIL farm PLANT r=%h", r); end
        sdram_read(10'd32, 16'h033F, r);
        if (r!==8'h01) begin errors=errors+1; $display("FAIL plot(3,3)=%h want 01", r); end
        sdram_read(10'd32, 16'h0215, r);
        if (r!==8'h04) begin errors=errors+1; $display("FAIL SEEDS=%h want 04", r); end
        // PLANT corner 19,19 -> $048F (PLOTADR 16-bit math)
        farm_cmd(8'h01, 8'd19, 8'd19, 8'h00, r, ok);
        sdram_read(10'd32, 16'h048F, r);
        if (r!==8'h01) begin errors=errors+1; $display("FAIL plot(19,19)=%h", r); end
        // (d) errors: PLANT occupied, HARVEST unripe
        // (must run before 5 grow ticks elapse - see FSIM divider note)
        farm_cmd(8'h01, 8'd3, 8'd3, 8'h00, r, ok);
        if (r!==8'hE1) begin errors=errors+1; $display("FAIL occupied r=%h want E1", r); end
        farm_cmd(8'h02, 8'd3, 8'd3, 8'h00, r, ok);
        if (r!==8'hE2) begin errors=errors+1; $display("FAIL unripe r=%h want E2", r); end
        // (d cont.) SELL qty=0 / BUYSEED qty=0 -> ERR_BAD
        farm_cmd(8'h03, 8'd0, 8'h00, 8'h00, r, ok);
        if (r!==8'hE6) begin errors=errors+1; $display("FAIL sell-0 r=%h want E6", r); end
        farm_cmd(8'h04, 8'd0, 8'h00, 8'h00, r, ok);
        if (r!==8'hE6) begin errors=errors+1; $display("FAIL buy-0 r=%h want E6", r); end
        // (b cont.) wait on (19,19) - the LAST plot planted; watching (3,3)
        // races a grow tick landing between the two PLANT commands
        begin : farm_ripen
        integer t; reg [7:0] pv;
        pv = 0; t = 0;
        while (pv !== 8'h06 && t < 400) begin
            repeat (10000) @(posedge clk100);
            sdram_read(10'd32, 16'h048F, pv); t = t + 1;
        end
        if (pv!==8'h06) begin errors=errors+1; $display("FAIL plot never ripened"); end
        end
        farm_drain;
        if (farm_nev < 2) begin errors=errors+1; $display("FAIL want 2 EV_RIPE got %0d", farm_nev); end
        else begin
            if (ev_type[0]!==8'h01 || ev_p0[0]!==8'd3 || ev_p1[0]!==8'd3)
                begin errors=errors+1; $display("FAIL EV_RIPE[0] %h %d,%d", ev_type[0], ev_p0[0], ev_p1[0]); end
            else $display("PASS EV_RIPE 3,3 then %0d,%0d", ev_p0[1], ev_p1[1]);
        end
        // (c) HARVEST -> CROPS=1, plot 0
        farm_cmd(8'h02, 8'd3, 8'd3, 8'h00, r, ok);
        if (r!==8'h01) begin errors=errors+1; $display("FAIL HARVEST r=%h", r); end
        sdram_read(10'd32, 16'h0216, r);
        if (r!==8'h01) begin errors=errors+1; $display("FAIL CROPS=%h want 01", r); end
        end
```

Notes: `clk100` is the tb's existing clock net name — verify (`grep "posedge clk" project_obscurus_tb.v | head -3`) and match. The ripen wait loop bounds total sim time; with FSIM dividers (GROWD=$80 = 128 passes/tick) 5 stages arrive well inside the 400 × 10k-cycle budget. Do not shrink GROWD below ~$80: the occupied/unripe error tests must complete before (3,3) ripens. Two tb hygiene points: (1) all local declarations need **named** begin blocks (`begin : farm_m1` — V2005 rule, existing tb style); (2) tb `sdram_write` pokes to live game state deliberately violate the single-writer invariant — fine in the rig, never copy the pattern into //e code. Before `stage_mbox(0, ...)`, verify slot 0 is actually free: `rd_reg(4'h1, r)` and assert `r[0]==0` (the prior multiverse tests end with their task halted, but make it explicit).

- [x] **Step 3: Set FARMLEN to the real blob size**

```bash
make farmtasksim && wc -l gateware/rev2/project_obscurus/farmtask.mem
```
Put that number in `FARMLEN`.

- [x] **Step 4: Run sim, expect the farm tests to PASS (and all pre-existing tests still green)**

```bash
make DESIGN=project_obscurus REV=rev2 sim 2>&1 | tail -30
```
Expected: `PASS farm STATUS`, `PASS EV_RIPE 3,3 ...`, no new FAILs, existing `PASS` lines intact.

- [x] **Step 5: Commit**

```bash
git add gateware/rev2/project_obscurus/project_obscurus_tb.v
git commit -m "test(farm): M1 a-e - spawn, STATUS, PLANT/HARVEST, growth walk, EV_RIPE, error paths"
```

---

### Task 6: tb — SELL/BUYSEED economy + clamps (M1 c cont.)

**Files:**
- Modify: `gateware/rev2/project_obscurus/project_obscurus_tb.v` (continue the farm test block)

- [x] **Step 1: Append economy assertions**

```verilog
        begin : farm_econ
        reg [7:0] r, r2; reg ok;
        // SELL 1 @ price 10 -> CASH 110, SUPPLY 1, CROPS 0
        farm_cmd(8'h03, 8'd1, 8'h00, 8'h00, r, ok);
        if (r!==8'h01) begin errors=errors+1; $display("FAIL SELL r=%h", r); end
        sdram_read(10'd32, 16'h0213, r); sdram_read(10'd32, 16'h0214, r2);
        if ({r2,r}!==16'd110) begin errors=errors+1; $display("FAIL CASH=%d want 110", {r2,r}); end
        sdram_read(10'd32, 16'h0212, r);
        if (r!==8'h01) begin errors=errors+1; $display("FAIL SUPPLY=%h", r); end
        // SELL with no crops -> E5
        farm_cmd(8'h03, 8'd1, 8'h00, 8'h00, r, ok);
        if (r!==8'hE5) begin errors=errors+1; $display("FAIL no-crops r=%h want E5", r); end
        // BUYSEED 2 @ cost 3 -> CASH 104, SEEDS 5
        farm_cmd(8'h04, 8'd2, 8'h00, 8'h00, r, ok);
        if (r!==8'h01) begin errors=errors+1; $display("FAIL BUYSEED r=%h", r); end
        sdram_read(10'd32, 16'h0213, r);
        if (r!==8'd104) begin errors=errors+1; $display("FAIL CASH=%d want 104", r); end
        sdram_read(10'd32, 16'h0215, r);
        if (r!==8'd5) begin errors=errors+1; $display("FAIL SEEDS=%d want 5", r); end
        // BUYSEED overflow guard: force SEEDS=254, buy 5 -> E7, SEEDS unchanged
        // (tb-only direct poke: rig deliberately bypasses single-writer rule)
        sdram_write(10'd32, 16'h0215, 8'd254);
        farm_cmd(8'h04, 8'd5, 8'h00, 8'h00, r, ok);
        if (r!==8'hE7) begin errors=errors+1; $display("FAIL seed-full r=%h want E7", r); end
        sdram_read(10'd32, 16'h0215, r);
        if (r!==8'd254) begin errors=errors+1; $display("FAIL SEEDS clobbered=%d", r); end
        sdram_write(10'd32, 16'h0215, 8'd5);   // restore
        // EV_PRICE drift: force SUPPLY=10 -> TGT=5. Supply decays while the
        // price walks (DECAYDIV=4), so the target RISES under it and the
        // price recovers - assert the MINIMUM seen, not the endpoint.
        sdram_write(10'd32, 16'h0212, 8'd10);
        begin : farm_pwalk
        integer t; reg [7:0] pv, pmin;
        pmin = 8'd255;
        for (t=0; t<200; t=t+1) begin
            repeat (10000) @(posedge clk100);
            sdram_read(10'd32, 16'h0210, pv);
            if (pv < pmin) pmin = pv;
        end
        if (pmin > 8'd7) begin errors=errors+1; $display("FAIL price min=%d want <=7", pmin); end
        end
        farm_drain;
        begin : farm_evcount
        integer i; integer sawprice;
        sawprice = 0;
        for (i=0; i<farm_nev; i=i+1) if (ev_type[i]===8'h02) sawprice = sawprice + 1;
        if (sawprice < 3) begin errors=errors+1; $display("FAIL want >=3 EV_PRICE got %0d", sawprice); end
        else $display("PASS economy: sell/buy/clamps + %0d EV_PRICE", sawprice);
        end
        end
```

The min-tracking assertion is deliberate: with DECAYDIV=4 the live target rises as supply drains (price parks around 6-7 then climbs home to 10), so any endpoint assertion is wrong by construction. `pmin <= 7` proves downward drift; `sawprice >= 3` proves the events flowed.

- [x] **Step 2: Run sim**

```bash
make DESIGN=project_obscurus REV=rev2 sim 2>&1 | tail -20
```
Expected: `PASS economy: ...`, zero new FAILs.

- [x] **Step 3: Commit**

```bash
git add gateware/rev2/project_obscurus/project_obscurus_tb.v
git commit -m "test(farm): M1 economy - SELL/BUYSEED, ERR_FULL clamp, price walk, EV_PRICE"
```

---

### Task 7: tb — lap recovery (M1 f)

**Files:**
- Modify: `gateware/rev2/project_obscurus/project_obscurus_tb.v`

The reader's cursor (`farm_tail/farm_seq`) is deliberately NOT updated while the sim generates >64 events; the next `farm_drain` must detect SEQ mismatch, resync (counted in `farm_resyncs`), and drain clean afterward.

- [x] **Step 1: Append lap test**

```verilog
        // ===== lap recovery: >64 events with stale reader cursor =====
        begin : farm_lap
        reg [7:0] r, r2; reg ok; integer i, baseresync; integer t;
        baseresync = farm_resyncs;
        // plant 70 plots (rows 5..8, cols 0..19 = 80 available; use 70)
        sdram_write(10'd32, 16'h0215, 8'd255);          // plenty of seeds (tb poke)
        for (i=0; i<70; i=i+1)
            farm_cmd(8'h01, i%20, 8'd5 + i/20, 8'h00, r, ok);
        // wait until the LAST-planted plot (9,8) ripens -> 70 EV_RIPE, ring lapped
        t = 0; r = 0;
        while (r !== 8'h06 && t < 600) begin
            repeat (10000) @(posedge clk100);
            sdram_read(10'd32, 16'h0300 + 8*20 + 9, r);  // plot (9,8) = $03A9
            t = t + 1;
        end
        if (r!==8'h06) begin errors=errors+1; $display("FAIL lap: plots never ripened"); end
        farm_drain;
        if (farm_resyncs < baseresync + 1)
            begin errors=errors+1; $display("FAIL lap: resyncs=%0d want >=%0d", farm_resyncs, baseresync+1); end
        // SIG regression net: a ring page-wrap bug writes records over
        // $0000-$0003 - SIG must survive 70+ publishes including indexes 60-63
        sdram_read(10'd32, 16'h0000, r); sdram_read(10'd32, 16'h0001, r2);
        if (r!==8'h46 || r2!==8'h4D)
            begin errors=errors+1; $display("FAIL lap: SIG destroyed %h %h (ring wrapped into page 0)", r, r2); end
        // post-resync: cursor must be live -> one more event drains clean
        sdram_write(10'd32, 16'h0212, 8'd20);            // kick price -> EV_PRICE soon
        begin : farm_postsync
        integer n0; n0 = farm_nev;
        t = 0;
        while (farm_nev == n0 && t < 200) begin
            repeat (10000) @(posedge clk100);
            farm_drain; t = t + 1;
        end
        if (farm_nev == n0)
            begin errors=errors+1; $display("FAIL lap: post-resync drain dirty"); end
        else $display("PASS lap recovery: resync + SIG intact + clean drain");
        end
        end
```

Caveat for the implementer: EV_PRICE events fire during the ripen wait too (market keeps ticking) — only adds to the >64 total, helps the lap. The resync assertion is `>=` from the start (a drain landing mid-publish can legitimately resync twice). The SIG check is the regression net for ring address-wrap bugs: tb arithmetic is full-width and would otherwise read correct addresses while 8-bit coproc/host math corrupts page 0.

- [x] **Step 2: Run sim — full suite**

```bash
make DESIGN=project_obscurus REV=rev2 sim 2>&1 | tail -15
```
Expected: `PASS lap recovery ...`, all prior PASS intact. **This is M1 complete.**

- [x] **Step 3: Commit**

```bash
git add gateware/rev2/project_obscurus/project_obscurus_tb.v
git commit -m "test(farm): M1 f - ring lap detection + resync recovery (only overrun-path coverage)"
```

---

### Task 8: FARM.S part 1 — skeleton, screen, render, HUD

**Files:**
- Create: `software/SDM/FARM.S`

This task builds everything except startup/main-loop logic, ending in an assemble check. ZP usage: $06-$0D transient pointers (GRVERSE precedent, non-ProDOS). Shadow grid + state live inline (DS). Text rows 20-23 via the same 24-entry line table.

- [x] **Step 1: Write FARM.S (part 1 content)**

```
* FARM.S - FARM/MARKET GAME, //e HOST SIDE
* ORG $6000. BRUN FARM.
* GR MIXED MODE: 20x20 PLOTS AS 2x2 LO-RES
* BLOCKS (ROWS 0-39), HUD TEXT ROWS 20-23.
* COPROC GAMETASK (SKILL 2) IS WORLD AUTHORITY.
 TYP $06
 DSK FARM.bin
 ORG $6000

 JMP MAIN
 PUT FARMEQU
 PUT SDRAMLIB
 PUT CPLIB

* --- SOFT SWITCHES ---
S80VOFF = $C00C
DHIROFF = $C05F
S80SOFF = $C000
PAGE2OF = $C054
GRAPH = $C050
LORES = $C056
MIXED = $C053
TEXTSW = $C051
KBD = $C000
KBDSTR = $C010

* --- ZP (NON-PRODOS TRANSIENT) ---
LINEP = $06
SHADP = $08
MSGPTR = $0A
SRC = $0C

* --- STATE ---
TAIL DS 1
EXPSEQ DS 1
CURX DS 1
CURY DS 1
CASHL DS 1
CASHH DS 1
SEEDS DS 1
CROPS DS 1
PRICE DS 1
LASTPR DS 1
RESCNT DS 1
TMO DS 2
T16 DS 2
GRLO DS 24
GRHI DS 24
SHADOW DS 400
RSTRCT DS 2
PRCOL DS 1
CNT DS 2
EVT DS 1
EVA DS 1
EVB DS 1
PLOTC DS 1
DECV DS 5
DAX DS 1
DAY DS 1
SHADV DS 1

* === GRON: UNDO ALL, LO-RES MIXED ===
GRON
 STA S80VOFF
 STA DHIROFF
 STA S80SOFF
 STA PAGE2OF
 STA GRAPH
 STA LORES
 STA MIXED
 RTS

* === GRINIT: 24-ENTRY LINE TABLE ===
* (identical to GRVERSE.S GRINIT - copy verbatim,
*  lines 81-117 of GRVERSE.S)

* === GRCLR: clear GR rows 0-19 (40 bytes each),
* space-fill text rows 20-23. line table only -
* NEVER blanket $400-$7FF (screen holes).
GRCLR
 LDX #0
GCROW
 LDA GRLO,X
 STA LINEP
 LDA GRHI,X
 STA LINEP+1
 CPX #20
 BCS GCTXT
 LDA #0
 JMP GCFILL
GCTXT
 LDA #$A0
GCFILL
 LDY #0
GCCOL
 STA (LINEP),Y
 INY
 CPY #40
 BNE GCCOL
 INX
 CPX #24
 BNE GCROW
 RTS

* === STAGE COLOR TABLE ===
STAGEC
 DFB $00,$08,$04,$0C,$0E,$09,$0D

* === PLOTDRAW: plot CURX-free (X=px Y=py),
* A=stage -> solid 2x2 block, text line py,
* cols 2*px and 2*px+1, byte = color*$11
PLOTDRAW
 STX PLOTC
 TAX
 LDA STAGEC,X
 STA T16
 ASL
 ASL
 ASL
 ASL
 ORA T16
 STA T16
 LDA GRLO,Y
 STA LINEP
 LDA GRHI,Y
 STA LINEP+1
 LDA PLOTC
 ASL
 TAY
 LDA T16
 STA (LINEP),Y
 INY
 STA (LINEP),Y
 RTS

* === CURSDRAW: XOR #$FF both bytes at CURX,CURY ===
CURSDRAW
 LDY CURY
 LDA GRLO,Y
 STA LINEP
 LDA GRHI,Y
 STA LINEP+1
 LDA CURX
 ASL
 TAY
 LDA (LINEP),Y
 EOR #$FF
 STA (LINEP),Y
 INY
 LDA (LINEP),Y
 EOR #$FF
 STA (LINEP),Y
 RTS

* === PRSTR: 0-term hi-ASCII string (MSGPTR)
* to text row Y at column PRCOL ===
PRSTR
 LDA GRLO,Y
 STA LINEP
 LDA GRHI,Y
 STA LINEP+1
 LDA LINEP
 CLC
 ADC PRCOL
 STA LINEP
 LDA LINEP+1
 ADC #0
 STA LINEP+1
 LDY #0
PRLP
 LDA (MSGPTR),Y
 BEQ PRDONE
 STA (LINEP),Y
 INY
 JMP PRLP
PRDONE
 RTS

* === PRDEC: 16-bit T16 -> 5 digits DECV,
* print at (X=col, Y=textrow) ===
PRDEC
 TYA
 PHA
 TXA
 PHA
 LDX #0
PDLP
 LDA #0
 STA DECV,X
PDSUB
 LDA T16
 SEC
 SBC DECTL,X
 PHA
 LDA T16+1
 SBC DECTH,X
 BCC PDNXT
 STA T16+1
 PLA
 STA T16
 INC DECV,X
 JMP PDSUB
PDNXT
 PLA
 INX
 CPX #5
 BNE PDLP
 PLA
 TAX
 PLA
 TAY
 LDA GRLO,Y
 STA LINEP
 LDA GRHI,Y
 STA LINEP+1
 TXA
 TAY
 LDX #0
PDPRT
 LDA DECV,X
 ORA #$B0
 STA (LINEP),Y
 INY
 INX
 CPX #5
 BNE PDPRT
 RTS
DECTL
 DFB <10000,<1000,<100,<10,<1
DECTH
 DFB >10000,>1000,>100,>10,>1
```

**Implementer notes:**
- `PRSTR` callers set MSGPTR (ZP $0A) + PRCOL, Y=text row. Caveat: PRSTR's `(LINEP),Y` reuses Y as the string index after the row lookup — the version above is correct because LINEP is pre-offset by PRCOL and Y restarts at 0 for both pointers (string and screen advance together).
- `PLOTDRAW` inputs: X=plot-x (0-19), Y=plot-y (0-19, also the text row), A=stage. Y survives the STAGEC lookup since the lookup uses X. Keep register discipline exactly as shown.
- Strings (hi-ASCII, 0-terminated), defined at end of file:
  ```
  SCASH ASC "CASH "
   DFB 0
  SSEED ASC " SEED "
   DFB 0
  SCROP ASC " CROP "
   DFB 0
  SPRICE ASC "PRICE "
   DFB 0
  SHELP ASC "ARROWS P H S B Q"
   DFB 0
  SDEAD ASC "WORLD DEAD - C COLDSTART"
   DFB 0
  SNORESP ASC "COPROC NOT RESPONDING"
   DFB 0
  SRIPE ASC "RIPE!"
   DFB 0
  SOK ASC "OK"
   DFB 0
  SERR ASC "ERR "
   DFB 0
  ```
  Merlin `ASC "..."` emits high-bit-set ASCII (project gotcha: that is what the text page wants — do NOT mask).

- [x] **Step 2: Add HUD update routine**

```
* === HUDDRAW: rows 20-22 ===
HUDDRAW
* row 20: CASH nnnnn SEED nnn CROP nnn
 LDA #<SCASH
 STA MSGPTR
 LDA #>SCASH
 STA MSGPTR+1
 LDA #0
 STA PRCOL
 LDY #20
 JSR PRSTR
 LDA CASHL
 STA T16
 LDA CASHH
 STA T16+1
 LDX #5
 LDY #20
 JSR PRDEC
* SEED label col 11, 3-digit value col 17
 LDA #<SSEED
 STA MSGPTR
 LDA #>SSEED
 STA MSGPTR+1
 LDA #11
 STA PRCOL
 LDY #20
 JSR PRSTR
 LDA SEEDS
 STA T16
 LDA #0
 STA T16+1
 LDX #17
 LDY #20
 JSR PRDEC3
* CROP label col 21, 3-digit value col 27
 LDA #<SCROP
 STA MSGPTR
 LDA #>SCROP
 STA MSGPTR+1
 LDA #21
 STA PRCOL
 LDY #20
 JSR PRSTR
 LDA CROPS
 STA T16
 LDA #0
 STA T16+1
 LDX #27
 LDY #20
 JSR PRDEC3
* row 21: PRICE nnnnn + trend
 LDA #<SPRICE
 STA MSGPTR
 LDA #>SPRICE
 STA MSGPTR+1
 LDA #0
 STA PRCOL
 LDY #21
 JSR PRSTR
 LDA PRICE
 STA T16
 LDA #0
 STA T16+1
 LDX #6
 LDY #21
 JSR PRDEC
* trend arrow col 12: price vs LASTPR
 LDY #21
 LDA GRLO,Y
 STA LINEP
 LDA GRHI,Y
 STA LINEP+1
 LDY #12
 LDA PRICE
 CMP LASTPR
 BEQ TRFLAT
 BCC TRDN
 LDA #$AB ; '+'
 JMP TRPUT
TRDN
 LDA #$AD ; '-'
 JMP TRPUT
TRFLAT
 LDA #$BD ; '='
TRPUT
 STA (LINEP),Y
 RTS
```

`PRDEC3` is PRDEC printing only digits 2-4 (3 chars, enough for 8-bit values). Factor the digit computation out so both share it:

```
* === DIGITS: T16 -> DECV[0..4] (digit calc
* portion of PRDEC, unchanged, ends in RTS) ===
* === PRDEC: JSR DIGITS, print DECV 0..4 at
* col X, text row Y (print portion as written)
* === PRDEC3: JSR DIGITS, print DECV 2..4 ===
PRDEC3
 TYA
 PHA
 TXA
 PHA
 JSR DIGITS
 PLA
 TAX
 PLA
 TAY
 LDA GRLO,Y
 STA LINEP
 LDA GRHI,Y
 STA LINEP+1
 TXA
 TAY
 LDX #2
PD3P
 LDA DECV,X
 ORA #$B0
 STA (LINEP),Y
 INY
 INX
 CPX #5
 BNE PD3P
 RTS
```

(Restructure PRDEC accordingly: its digit loop becomes `DIGITS`, its print loop stays in PRDEC after a `JSR DIGITS` — same save/restore-X,Y discipline as PRDEC3 above.)

- [x] **Step 3: Assemble check (MAIN stub)**

Temporarily add `MAIN RTS` at the end, then:
```bash
cd software/SDM && Merlin32 -V . FARM.S
```
Expected: clean assemble. (FARMTASKB PUT comes in part 2 — not yet referenced.)

- [x] **Step 4: Commit**

```bash
git add software/SDM/FARM.S
git commit -m "feat(farm): FARM.S part 1 - GR mixed init, line table, plot/cursor render, HUD, decimal print"
```

---

### Task 9: FARM.S part 2 — startup, protocol, main loop

**Files:**
- Modify: `software/SDM/FARM.S` (replace MAIN stub; append protocol routines; append `PUT FARMTASKB` as the LAST line)

- [x] **Step 1: SDM access helpers (host side)**

```
* === FRD: read GBANK byte, A=lo Y=hi -> SDM_VAL ===
FRD
 STA SDM_ADDR
 STY SDM_ADDR+1
 LDA #GBANK
 STA SDM_BANK
 LDA #0
 STA SDM_BANK+1
 JSR SDM_READ
 RTS
* === FWR: write SDM_VAL to GBANK, A=lo Y=hi ===
FWR
 STA SDM_ADDR
 STY SDM_ADDR+1
 LDA #GBANK
 STA SDM_BANK
 LDA #0
 STA SDM_BANK+1
 JSR SDM_WRITE
 RTS
```

- [x] **Step 2: Command send with timeout**

```
* === SENDCMD: TOP/TA0/TA1/TA2 staged by caller
* in CMDOP/CMDA0/CMDA1/CMDA2. RETURNS A=RESULT,
* C=1 TIMEOUT. PRE: WAIT FLAG=0 (POST-TIMEOUT
* OWNERSHIP RULE). ~1S TIMEOUT = $0000 WRAP OF
* 16-BIT COUNTER OF FLAG POLLS.
CMDOP DS 1
CMDA0 DS 1
CMDA1 DS 1
CMDA2 DS 1
SENDCMD
* never write while FLAG=1
 LDA #0
 STA TMO
 STA TMO+1
SCWAIT0
 LDA #<FMFLAG
 LDY #>FMFLAG
 JSR FRD
 LDA SDM_VAL
 BEQ SCWRITE
 INC TMO
 BNE SCWAIT0
 INC TMO+1
 BNE SCWAIT0
 SEC
 RTS
SCWRITE
 LDA CMDOP
 STA SDM_VAL
 LDA #<FMOP
 LDY #>FMOP
 JSR FWR
 LDA CMDA0
 STA SDM_VAL
 LDA #<FMA0
 LDY #>FMA0
 JSR FWR
 LDA CMDA1
 STA SDM_VAL
 LDA #<FMA1
 LDY #>FMA1
 JSR FWR
 LDA CMDA2
 STA SDM_VAL
 LDA #<FMA2
 LDY #>FMA2
 JSR FWR
 LDA #1
 STA SDM_VAL
 LDA #<FMFLAG
 LDY #>FMFLAG
 JSR FWR
* poll clear
 LDA #0
 STA TMO
 STA TMO+1
SCPOLL
 LDA #<FMFLAG
 LDY #>FMFLAG
 JSR FRD
 LDA SDM_VAL
 BEQ SCRES
 INC TMO
 BNE SCPOLL
 INC TMO+1
 BNE SCPOLL
 SEC
 RTS
SCRES
 LDA #<FMRES
 LDY #>FMRES
 JSR FRD
 LDA SDM_VAL
 CLC
 RTS
```

(Timeout: 65536 polls, each poll = a full FRD (SETBANK+SETADDR+trigger+busy-poll) — realistically 1-3 s, calibrate in M5; the comment "~1 s" is an estimate, not a spec. The probe loops the whole SENDCMD 3× → up to ~10 s worst-case dead-world screen; acceptable for the rare path, note it on the message line by printing SDEAD only after the loop.)

- [x] **Step 3: Market re-read, event drain, resync**

```
* === RDMKT: refresh CASH/SEEDS/CROPS/PRICE
* (16-bit field: re-read and compare BOTH
* bytes - L-only guard misses the coproc
* writing L then H between host reads) ===
RDMKT
RMAGN
 LDA #<FCASHL
 LDY #>FCASHL
 JSR FRD
 LDA SDM_VAL
 STA CASHL
 LDA #<FCASHH
 LDY #>FCASHH
 JSR FRD
 LDA SDM_VAL
 STA CASHH
 LDA #<FCASHL
 LDY #>FCASHL
 JSR FRD
 LDA SDM_VAL
 CMP CASHL
 BNE RMAGN
 LDA #<FCASHH
 LDY #>FCASHH
 JSR FRD
 LDA SDM_VAL
 CMP CASHH
 BNE RMAGN
 LDA #<FPRICEL
 LDY #>FPRICEL
 JSR FRD
 LDA SDM_VAL
 STA PRICE
 LDA #<FSEEDS
 LDY #>FSEEDS
 JSR FRD
 LDA SDM_VAL
 STA SEEDS
 LDA #<FCROPS
 LDY #>FCROPS
 JSR FRD
 LDA SDM_VAL
 STA CROPS
 RTS

* === DRAIN: consume ring TAIL..HEAD ===
RECLO DS 1
RSEQ DS 1
DRAIN
 LDA #<FHEAD
 LDY #>FHEAD
 JSR FRD
 LDA SDM_VAL
 CMP TAIL
 BEQ DRDONE
* record lo = TAIL*4, hi = >FRING (ring is
* page-aligned at $0100 - no carry ever)
 LDA TAIL
 ASL
 ASL
 STA RECLO
 LDY #>FRING
 LDA RECLO
 JSR FRD
 LDA SDM_VAL
 STA RSEQ
 LDA RECLO
 CLC
 ADC #1
 LDY #>FRING
 JSR FRD
 LDA SDM_VAL
 STA EVT
 LDA RECLO
 CLC
 ADC #2
 LDY #>FRING
 JSR FRD
 LDA SDM_VAL
 STA EVA
 LDA RECLO
 CLC
 ADC #3
 LDY #>FRING
 JSR FRD
 LDA SDM_VAL
 STA EVB
* seqlock tear guard: re-read SEQ byte
 LDA RECLO
 LDY #>FRING
 JSR FRD
 LDA SDM_VAL
 CMP RSEQ
 BNE DRLAP
 LDA RSEQ
 CMP EXPSEQ
 BNE DRLAP
 JSR EVDISP
 INC EXPSEQ
 LDA TAIL
 CLC
 ADC #1
 AND #$3F
 STA TAIL
 JMP DRAIN
DRLAP
 JSR RESYNC
DRDONE
 RTS

* === EVDISP: dispatch EVT/EVA/EVB ===
EVDISP
 LDA EVT
 CMP #EVRIPE
 BNE EVD2
 LDX EVA
 LDY EVB
 LDA #STRIPE
 JSR PLOTDRAW
 LDA #STRIPE
 JSR SHADSET
* ripe plot under cursor? re-apply XOR
 LDA EVA
 CMP CURX
 BNE EVRMSG
 LDA EVB
 CMP CURY
 BNE EVRMSG
 JSR CURSDRAW
EVRMSG
 LDA #<SRIPE
 STA MSGPTR
 LDA #>SRIPE
 STA MSGPTR+1
 LDA #0
 STA PRCOL
 LDY #22
 JSR PRSTR
 RTS
EVD2
 CMP #EVPRICE
 BNE EVDONE
 LDA PRICE
 STA LASTPR
 LDA EVA
 STA PRICE
 JSR HUDDRAW
EVDONE
 RTS

* === SHADSET: SHADOW[EVB*20+EVA] = A ===
* same 16-bit y*16+y*4 shape as coproc PLOTADR
SHADSET
 STA SHADV
 LDA EVB
 STA T16
 LDA #0
 STA T16+1
 ASL T16
 ROL T16+1
 ASL T16
 ROL T16+1
 LDA T16
 STA SHADP
 LDA T16+1
 STA SHADP+1
 ASL T16
 ROL T16+1
 ASL T16
 ROL T16+1
 LDA T16
 CLC
 ADC SHADP
 STA T16
 LDA T16+1
 ADC SHADP+1
 STA T16+1
 LDA T16
 CLC
 ADC EVA
 STA T16
 LDA T16+1
 ADC #0
 STA T16+1
 LDA T16
 CLC
 ADC #<SHADOW
 STA SHADP
 LDA T16+1
 ADC #>SHADOW
 STA SHADP+1
 LDY #0
 LDA SHADV
 STA (SHADP),Y
 RTS

* === RESYNC: stable (SEQCTR,HEAD) pair, full
* state re-read + redraw ===
RESYNC
RSAGN
 LDA #<FSEQC
 LDY #>FSEQC
 JSR FRD
 LDA SDM_VAL
 STA EXPSEQ
 LDA #<FHEAD
 LDY #>FHEAD
 JSR FRD
 LDA SDM_VAL
 STA TAIL
 LDA #<FSEQC
 LDY #>FSEQC
 JSR FRD
 LDA SDM_VAL
 CMP EXPSEQ
 BNE RSAGN
 JSR RDMKT
 JSR RDGRID
 JSR DRAWALL
 JSR HUDDRAW
 RTS

* === RDGRID: stream 400 bytes -> SHADOW ===
RDGRID
 LDA #GBANK
 STA SDM_BANK
 LDA #0
 STA SDM_BANK+1
 JSR SDM_SETBANK
 LDA #<FGRID
 STA SDM_ADDR
 LDA #>FGRID
 STA SDM_ADDR+1
 JSR SDM_SETADDR
 LDA #<SHADOW
 STA SHADP
 LDA #>SHADOW
 STA SHADP+1
 LDA #<400
 STA CNT
 LDA #>400
 STA CNT+1
 LDY #0
RGLP
 JSR SDM_RDNEXT
 LDA SDM_VAL
 STA (SHADP),Y
 INY
 BNE RGDEC
 INC SHADP+1
RGDEC
 LDA CNT
 SEC
 SBC #1
 STA CNT
 LDA CNT+1
 SBC #0
 STA CNT+1
 ORA CNT
 BNE RGLP
 RTS
```

**Implementer notes:**
- Add `CNT DS 2` to the state block. `SHADP`/`MSGPTR` must be ZP ($08/$0A as declared) — indirect addressing requires it.
- DRAIN includes the seqlock tear guard (SEQ byte re-read) from the spec reader rule — keep it.
- `DRAWALL`: walk SHADOW 0..399 (row y, col x), `JSR PLOTDRAW` per plot, then `JSR CURSDRAW` to re-apply cursor.
- `RESTREAM` (periodic): same as RDGRID but compare each byte to SHADOW first; on diff, store + repaint that plot (and if it is the cursor plot, re-apply CURSDRAW after). Trigger: 16-bit RSTRCT counter incremented each main-loop pass; on wrap (~2-3 s of polling) do one re-stream. Pin exact reload constant on bench (M5).

- [x] **Step 4: Startup + cold start + probe**

```
* === MAIN ===
MAIN
 JSR SDM_READY
 BCC MRDY
 RTS
MRDY
 JSR GRINIT
 JSR GRON
 JSR GRCLR
* SIG present?
 LDA #<FSIG0
 LDY #>FSIG0
 JSR FRD
 LDA SDM_VAL
 CMP #SIGF
 BNE COLDST
 LDA #<FSIG1
 LDY #>FSIG1
 JSR FRD
 LDA SDM_VAL
 CMP #SIGM
 BNE COLDST
* probe: STATUS with 3x timeout
 LDX #3
PRLOOP
 LDA #OPSTAT
 STA CMDOP
 JSR SENDCMD
 BCC PROBED
 DEX
 BNE PRLOOP
* dead world
 JSR MSGDEAD
PRKEY
 LDA KBD
 BPL PRKEY
 STA KBDSTR
 CMP #$C3 ; 'C'
 BEQ COLDST
 JMP QUIT
PROBED
 CMP #ROK
 BNE COLDST
 JSR RESYNC
 JMP MLOOP

COLDST
* init GBANK (SIG LAST - after CP_CALL)
 LDA #0
 STA SDM_VAL
 LDA #<FSEQC
 LDY #>FSEQC
 JSR FWR
 LDA #<FHEAD
 LDY #>FHEAD
 JSR FWR
 LDA #<FMFLAG
 LDY #>FMFLAG
 JSR FWR
 LDA #<FPRICEH
 LDY #>FPRICEH
 JSR FWR
 LDA #<FSUPPLY
 LDY #>FSUPPLY
 JSR FWR
 LDA #<FCASHH
 LDY #>FCASHH
 JSR FWR
 LDA #<FCROPS
 LDY #>FCROPS
 JSR FWR
 LDA #PBASE
 STA SDM_VAL
 LDA #<FPRICEL
 LDY #>FPRICEL
 JSR FWR
 LDA #CASH0
 STA SDM_VAL
 LDA #<FCASHL
 LDY #>FCASHL
 JSR FWR
 LDA #SEEDS0
 STA SDM_VAL
 LDA #<FSEEDS
 LDY #>FSEEDS
 JSR FWR
* zero grid: 400 x WRNEXT
 LDA #GBANK
 STA SDM_BANK
 LDA #0
 STA SDM_BANK+1
 JSR SDM_SETBANK
 LDA #<FGRID
 STA SDM_ADDR
 LDA #>FGRID
 STA SDM_ADDR+1
 JSR SDM_SETADDR
 LDA #0
 STA SDM_VAL
 LDA #<400
 STA CNT
 LDA #>400
 STA CNT+1
CZLP
 JSR SDM_WRNEXT
 LDA CNT
 SEC
 SBC #1
 STA CNT
 LDA CNT+1
 SBC #0
 STA CNT+1
 ORA CNT
 BNE CZLP
* load blob -> coproc $0600 (GRVERSE REGSKILL
* pattern, FSKILL/FSKLEN, CP_LADLO/HI=$00/$06)
 JSR LOADBLOB
* table[2] @ $0204 = $00,$06
* (CPLIB names: CP_LADDRLO/CP_LADDRHI/CP_WDATA)
 LDA #$04
 STA CP_LADDRLO
 LDA #$02
 STA CP_LADDRHI
 LDA #$00
 STA CP_WDATA
 LDA #$06
 STA CP_WDATA
* spawn: CP_CALL A=skill X=arg0 Y=budget
 LDA #2
 LDX #0
 LDY #0
 JSR CP_CALL
 CMP #$FF
 BNE SPAWNOK
 JSR MSGDEAD
 JMP QUIT
SPAWNOK
* SIG last
 LDA #SIGF
 STA SDM_VAL
 LDA #<FSIG0
 LDY #>FSIG0
 JSR FWR
 LDA #SIGM
 STA SDM_VAL
 LDA #<FSIG1
 LDY #>FSIG1
 JSR FWR
 JSR RESYNC
 JMP MLOOP
```

`LOADBLOB` full code is in Step 6 (GRVERSE REGSKILL shape; SRC pointer at ZP $0C; CP load-port names come from CPLIB — `CP_LADDRLO/CP_LADDRHI/CP_WDATA`, do NOT redefine GRVERSE's `CP_LADLO` variants).

- [x] **Step 5: Main loop + keys + quit**

```
* === MLOOP ===
MLOOP
 JSR DRAIN
* periodic re-stream
 INC RSTRCT
 BNE MNOST
 INC RSTRCT+1
 BNE MNOST
 JSR RESTREAM
MNOST
 LDA KBD
 BPL MLOOP
 STA KBDSTR
 CMP #$88 ; left
 BNE K2
 JSR CURSDRAW
 DEC CURX
 BPL KDONE
 LDA #19
 STA CURX
 JMP KDONE
K2
 CMP #$95 ; right
 BNE K3
 JSR CURSDRAW
 INC CURX
 LDA CURX
 CMP #20
 BNE KDONE
 LDA #0
 STA CURX
 JMP KDONE
K3
 CMP #$8B ; up
 BNE K4
 JSR CURSDRAW
 DEC CURY
 BPL KDONE
 LDA #19
 STA CURY
 JMP KDONE
K4
 CMP #$8A ; down
 BNE K5
 JSR CURSDRAW
 INC CURY
 LDA CURY
 CMP #20
 BNE KDONE
 LDA #0
 STA CURY
 JMP KDONE
K5
 CMP #$D0 ; P
 BNE K6
 LDA #OPPLANT
 JSR DOCMDXY
 JMP MLOOP
K6
 CMP #$C8 ; H
 BNE K7
 LDA #OPHARV
 JSR DOCMDXY
 JMP MLOOP
K7
 CMP #$D3 ; S
 BNE K8
 LDA #OPSELL
 JSR DOCMD1
 JMP MLOOP
K8
 CMP #$C2 ; B
 BNE K9
 LDA #OPBUY
 JSR DOCMD1
 JMP MLOOP
K9
 CMP #$D1 ; Q
 BNE KNONE
QUIT
 STA TEXTSW
 RTS
KNONE
 JMP MLOOP
KDONE
* arrow handlers: CURSDRAW (XOR undo at old
* pos) BEFORE mutate, CURSDRAW (apply) here
 JSR CURSDRAW
 JMP MLOOP
```

Cursor discipline: XOR is its own inverse — CURSDRAW before the mutate undoes the old cursor, CURSDRAW at KDONE applies the new. Startup applies it once after DRAWALL. Last line of file: ` PUT FARMTASKB`.

- [x] **Step 6: Remaining routines (full code — no improvisation)**

```
* === DOCMDXY: A=op, args = cursor pos ===
DOCMDXY
 STA CMDOP
 LDA CURX
 STA CMDA0
 LDA CURY
 STA CMDA1
 LDA #0
 STA CMDA2
 JMP DOCMD
* === DOCMD1: A=op, qty=1 ===
DOCMD1
 STA CMDOP
 LDA #1
 STA CMDA0
 LDA #0
 STA CMDA1
 STA CMDA2
DOCMD
 JSR SENDCMD
 BCC DCRES
 LDA #<SNORESP
 STA MSGPTR
 LDA #>SNORESP
 STA MSGPTR+1
 JMP DCMSG
DCRES
 CMP #ROK
 BEQ DCOK
 LDA #<SERR
 STA MSGPTR
 LDA #>SERR
 STA MSGPTR+1
DCMSG
 LDA #0
 STA PRCOL
 LDY #22
 JSR PRSTR
 RTS
DCOK
 LDA CMDOP
 CMP #OPPLANT
 BNE DCH
 LDA #1
 JSR DCPAINT
 JMP DCMKT
DCH
 CMP #OPHARV
 BNE DCMKT
 LDA #0
 JSR DCPAINT
DCMKT
 JSR RDMKT
 JSR HUDDRAW
 LDA #<SOK
 STA MSGPTR
 LDA #>SOK
 STA MSGPTR+1
 LDA #0
 STA PRCOL
 LDY #22
 JSR PRSTR
 RTS
* === DCPAINT: A=stage -> plot+shadow at
* cursor, re-apply cursor XOR on top ===
DCPAINT
 PHA
 LDA CURX
 STA EVA
 LDA CURY
 STA EVB
 PLA
 PHA
 JSR SHADSET
 PLA
 LDX CURX
 LDY CURY
 JSR PLOTDRAW
 JSR CURSDRAW
 RTS

* === MSGDEAD ===
MSGDEAD
 LDA #<SDEAD
 STA MSGPTR
 LDA #>SDEAD
 STA MSGPTR+1
 LDA #0
 STA PRCOL
 LDY #22
 JSR PRSTR
 RTS

* === DRAWALL: repaint grid from SHADOW,
* help row, cursor ===
DRAWALL
 LDA #<SHADOW
 STA SHADP
 LDA #>SHADOW
 STA SHADP+1
 LDA #0
 STA DAY
DAROW
 LDA #0
 STA DAX
DACOL
 LDY #0
 LDA (SHADP),Y
 LDX DAX
 LDY DAY
 JSR PLOTDRAW
 INC SHADP
 BNE DANC
 INC SHADP+1
DANC
 INC DAX
 LDA DAX
 CMP #GCOLS
 BNE DACOL
 INC DAY
 LDA DAY
 CMP #GROWSN
 BNE DAROW
 LDA #<SHELP
 STA MSGPTR
 LDA #>SHELP
 STA MSGPTR+1
 LDA #0
 STA PRCOL
 LDY #23
 JSR PRSTR
 JSR CURSDRAW
 RTS

* === RESTREAM: re-read grid, repaint diffs,
* preserve cursor XOR ===
RESTREAM
 LDA #GBANK
 STA SDM_BANK
 LDA #0
 STA SDM_BANK+1
 JSR SDM_SETBANK
 LDA #<FGRID
 STA SDM_ADDR
 LDA #>FGRID
 STA SDM_ADDR+1
 JSR SDM_SETADDR
 LDA #<SHADOW
 STA SHADP
 LDA #>SHADOW
 STA SHADP+1
 LDA #0
 STA DAY
RSROW2
 LDA #0
 STA DAX
RSCOL2
 JSR SDM_RDNEXT
 LDY #0
 LDA (SHADP),Y
 CMP SDM_VAL
 BEQ RSSAME
 LDA SDM_VAL
 STA (SHADP),Y
 LDX DAX
 LDY DAY
 JSR PLOTDRAW
 LDA DAX
 CMP CURX
 BNE RSSAME
 LDA DAY
 CMP CURY
 BNE RSSAME
 JSR CURSDRAW
RSSAME
 INC SHADP
 BNE RSNC
 INC SHADP+1
RSNC
 INC DAX
 LDA DAX
 CMP #GCOLS
 BNE RSCOL2
 INC DAY
 LDA DAY
 CMP #GROWSN
 BNE RSROW2
 RTS

* === LOADBLOB: FSKILL -> coproc $0600 ===
LOADBLOB
 LDA #$00
 STA CP_LADDRLO
 LDA #$06
 STA CP_LADDRHI
 LDA #<FSKILL
 STA SRC
 LDA #>FSKILL
 STA SRC+1
 LDA #<FSKLEN
 STA CNT
 LDA #>FSKLEN
 STA CNT+1
 LDY #0
LBLP
 LDA (SRC),Y
 STA CP_WDATA
 INC SRC
 BNE LBDEC
 INC SRC+1
LBDEC
 LDA CNT
 SEC
 SBC #1
 STA CNT
 LDA CNT+1
 SBC #0
 STA CNT+1
 ORA CNT
 BNE LBLP
 RTS
```

Notes:
- RESTREAM's `JSR PLOTDRAW` between `SDM_RDNEXT` calls is safe: PLOTDRAW/CURSDRAW touch only the text page, never the $C0Cx SDM registers, so the read stream's auto-increment is undisturbed. `SDM_RDNEXT` clobbers only A and SDM_VAL (Y preserved).
- In RESTREAM, `LDA SDM_VAL` must be reloaded before `LDX/LDY` for PLOTDRAW (A carries the stage) — order shown is correct as written: A is loaded from SDM_VAL, then X/Y loads don't touch A.
- Zombie-slot caveat (accepted v1, document in code header): if the dead-world `C` cold-start path runs while the old GAMETASK is merely slow, the old slot stays CP_ACTIVE (leaks — 4 zombie cold starts exhaust slots and CP_CALL returns $FF, handled by the MSGDEAD path) and LOADBLOB overwrites $0600 under a possibly-executing task. The 3 s probe makes this remote; spec already accepts the duplicate-writer residual.

- [x] **Step 7: Build + size check**

```bash
make farm && wc -c software/SDM/FARM.bin
```
Expected: clean build. FARM.bin = code + 400-byte shadow + ~1 KB blob — expect 3-4.5 KB, ORG $6000 → ends ≤ $7200, fine under BASIC.SYSTEM at $9600.

- [x] **Step 8: Commit**

```bash
git add software/SDM/FARM.S Makefile
git commit -m "feat(farm): FARM.S part 2 - cold start/probe/resync, drain, mailbox send, main loop"
```

---

### Task 10: Disk + bench bring-up notes (M2-M5)

**Files:**
- Modify: `Makefile` (sdmdisk additions from Task 4 if not already done)
- Modify: `docs/superpowers/specs/2026-06-10-farm-market-event-game-design.md` (pin skill id 2 + CP_CALL finding)

- [x] **Step 1: Build disk**

```bash
make sdmdisk
```
Expected: SDMTEST.po now contains FARM (BIN $6000) alongside GRVERSE/MVERSE.

- [x] **Step 2: Update spec plan-time pins**

In the spec, change "default **skill id 3**" → "**skill id 2** (Conway uses 0/1 — verified)" and add to the CP_CALL plan-time-verify sentence: "Verified: CP_CALL scans CP_ACTIVE for the lowest clear bit and cannot overwrite a running slot; the residual risk is a duplicate spawn in a second slot."

- [x] **Step 3: Commit**

```bash
git add Makefile docs/superpowers/specs/2026-06-10-farm-market-event-game-design.md
git commit -m "build(farm): FARM on sdmdisk; spec: pin skill id 2 + CP_CALL slot-reuse finding"
```

- [x] **Step 4: Bench milestones (manual, user flashes — NEVER prog-flash without permission)**

- **M2 (protocol on silicon):** boot SDMTEST.po, `BRUN FARM` once to cold-start, `Q` out. Enter the $C800 monitor; watch GBANK 32 offset $0003 (HEAD) advance as EV_PRICE fires (force: from monitor, write SUPPLY=$0212 high → price walks). Hand-poke a STATUS command: write $0201=$00, $0200=$01, watch $0200 clear and $0205=$01.
- **M3 (game):** `BRUN FARM` — plant/harvest/sell with keys, HUD live, ripe plots repaint within one re-stream period.
- **M4 (re-entry):** `Q` to BASIC, wait 2+ min, `BRUN FARM` → probe OK, world advanced, no thrash (message row should NOT flash constant resync). Stay out ~30 min (>64 events) → re-entry still clean.
- **M5 (tuning):** adjust `GROWD*/MKTD*/DECAYDIV` + `RSTRCT` reload + economy constants in FARMEQU.S, rebuild, re-test. Target: seed-to-ripe 2-3 min, price visibly drifting within ~10 s of a dump.

---

## Self-review (done at plan time)

- **Spec coverage:** memory map (T1), PUTEV order (T2), GAMETASK ops/growth/market/SEI (T3), build+blob autogen (T4), M1 a-e (T5), economy+clamps (T6), M1 f lap (T7), //e render/HUD/GR-mixed (T8), startup/probe/SIG-last/resync/drain/timeout-ownership/re-stream/cursor (T9), disk+M2-M5+spec pins (T10). Out-of-scope items untouched. ✓
- **Known deliberate deviations from pure TDD:** sim tests land after each coproc feature batch (blob must exist for `$readmemh`); //e-side code is bench-verified (M3/M4), not sim-tested — the protocol layer it depends on is sim-covered.
- **Type consistency:** GBANK=32 / skill 2 / $0600 / mailbox offsets consistent across FARMEQU, tb tasks, FARM.S. `farm_drain` tb task implements the same reader rules as FARM.S DRAIN/RESYNC. Result codes match FARMEQU in tb hex literals. ✓
- **Open risks for the executor:** (1) Merlin branch ranges in FARMTASK dispatch — trampolines provided, add more if needed; (2) tb `clk100` net name and `FARMLEN` must be checked against the real files; (3) PLOTADR 16-bit version is normative (corner test 19,19 catches the 8-bit bug); (4) loop-pass timing unknown until bench — HW divider defaults are placeholders for M5.
