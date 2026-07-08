# Economy Phase 2A — Purchaser/Vendor Trade Engine Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add the purchaser/vendor trading engine — NPC price tables with supply/demand fluctuation, three new mailbox trade ops (OPTSELL/OPTBUY/WOPTSELL), per-tick mean-revert drift, cold-seeding, and on-hardware FARM_TEST coverage — with no UI (the TRADE screen is the follow-on Plan 2B).

**Architecture:** Approach A (extend the existing tasks, single-writer-per-bank). Crop-trading NPCs (GROCER, TRADER-crops, SEED-SHOP, BULK-VENDOR, EXOTIC-VENDOR) live in bank 32, owned by FARMTASK. Goods-trading NPCs (BAKER, TRADER-goods) live in bank 33, owned by WORKTASK. Each NPC holds a **current price** per item in SDRAM; a parallel **base** region in SDRAM is the mean-revert target. **DEVIATION FROM SPEC §2 (user-approved 2026-06-18):** base prices live in SDRAM (single authority `NPCBASE.S`, PUT-included by the //e seeder and the test seeder; the blobs read base from SDRAM), NOT as duplicate blob DFB tables — this removes the multi-copy sync hazard that Phase 1's RECTAB/RVALUE retune exposed. Selling to a purchaser drops its price (saturation); buying from a vendor raises its price (demand); each market tick every NPC price steps ±1 toward base. CVER bumps 5→6 to cold-seed the new regions.

**Tech Stack:** Merlin32 6502 assembly (single-space source, labels col 0, one leading space for unlabeled lines, ASCII, ≤50 char lines). `make farm` / `make farmtest` / `make sdmdisk`. Verified on hardware via `BRUN FARM_TEST` (blob/economy fixes iterate by disk rebuild — NO re-flash; gateware/kernel unchanged).

**Spec:** `docs/superpowers/specs/2026-06-18-economy-rebalance-vendors-design.md` §2 (engine), §3 (migration), §4 (testing). The TRADE UI (§2 "Trade UI") is Plan 2B, written after 2A is bench-green.

**Test architecture (do not violate):** the Verilog tb tests ONLY gateware. ALL 6502 software is tested by FARM_TEST on real hardware. Embed FSIM=0 production blobs. Force preconditions by writing SDRAM directly (don't wait on natural ticks) — except the ONE allowed "see a click" wait for the drift test. No hollow tests.

---

## NPC model (reference for all tasks)

**Crop-side NPCs (bank 32, FARMTASK), `NCNPC = 5`:**

| idx | name | role | trades | base prices (wheat,carrot,berry,pumpkin) |
|---|---|---|---|---|
| 0 `NPGROC` | GROCER | purchaser | buys crops | 8,14,20,32 |
| 1 `NPTRDC` | TRADER-crops | purchaser | buys rare crops (premium) | 4,8,40,64 |
| 2 `NPSEED` | SEED-SHOP | vendor | sells seeds (baseline) | 2,4,7,12 |
| 3 `NPBULK` | BULK-VENDOR | vendor | sells common seeds cheap | 1,3,7,12 |
| 4 `NPEXOT` | EXOTIC-VENDOR | vendor | sells rare seeds | 3,6,5,9 |

Purchasers are NPC `0..NCBUY-1` (`NCBUY = 2`); vendors are `NCBUY..NCNPC-1`. `OPTSELL` only accepts purchasers; `OPTBUY` only accepts vendors.

**Goods-side NPCs (bank 33, WORKTASK), `NGNPC = 2`:**

| idx | name | role | trades | base prices (recipes 0..11) |
|---|---|---|---|---|
| 0 `NPBAKE` | BAKER | purchaser | buys goods, premium on common | 22,30,38,50,62,72,72,80,118,124,124,130 |
| 1 `NPTRDG` | TRADER-goods | purchaser | buys high-rarity goods | 18,25,32,43,54,64,64,72,135,145,145,160 |

Both goods-NPCs are purchasers; `WOPTSELL` accepts NPC `0..NGNPC-1`.

**SDRAM layout (MAX-sized; live counts NCROPS=4, NRECIP=12 bound loops):**

- Bank 32 current prices `FNPC = $0B00`, stride 32/NPC (16 crops × 2 bytes lo/hi, lo used). NPC n crop c at `$0B00 + n*32 + c*2`. 5×32 = 160 → `$0B00-$0B9F`.
- Bank 32 base prices `FNPCB = $0C00`, same stride. NPC n crop c at `$0C00 + n*32 + c*2`. → `$0C00-$0C9F`.
- Bank 33 current prices `WNPC = $0500`, stride 128/NPC (64 recipes × 2). NPC n good g at `$0500 + n*128 + g*2`. 2×128 = 256 → `$0500-$05FF`.
- Bank 33 base prices `WNPCB = $0600`, same stride. → `$0600-$06FF`.

All prices are 8-bit (stored in the lo byte; hi byte reserved 0, like `RPRICEH`). `n*32 + c*2 < 256` and `n*128 + g*2 < 256`, so the offset is a single byte and the hi byte is the region page constant.

---

## File Structure

| File | Role | Change |
|---|---|---|
| `software/SDM/FARMEQU.S` | shared equates (FARMTASK + FARM.S + FARMTEST) | add bank-32 + bank-33(WB) NPC equates, op codes, NPC constants; CVERNUM 5→6 |
| `software/SDM/WORKEQU.S` | WORKTASK equates | add bank-33 NPC equates (bare names), `WOPTSELL`, `PFLOOR` |
| `software/SDM/NPCBASE.S` | **new** — shared NPC base-price tables (single authority) | PUT-included by FARM.S + FARMTEST.S |
| `software/SDM/FARMTASK.S` | crop-task blob | OPTSELL/OPTBUY handlers + dispatch + `NPCDFT` drift + loop wiring |
| `software/SDM/WORKTASK.S` | goods-task blob | WOPTSELL handler + dispatch + `WNPDFT` drift + loop wiring |
| `software/SDM/FARM.S` | //e host | `SEEDNPC`/`SEEDNPG` seed routines + COLDST/SEED33 wiring + PUT NPCBASE |
| `software/SDM/FARMTEST.S` | //e self-test | PUT NPCBASE; SETUP seeds NPC regions; new trade-op + drift test cases |

WORKTASK reads NPC base from SDRAM at runtime — no blob base table. The crop/goods *current* and *base* SDRAM regions are written by the //e (sole seeder) at cold-seed; the owning task is the sole writer thereafter.

---

### Task 1: NPC equates + CVER bump

The equates must be added to BOTH files: FARMEQU.S carries the bank-32 names (FARMTASK) and the `WB`-prefixed bank-33 names (FARM.S host + FARMTEST); WORKEQU.S carries the bare bank-33 names (WORKTASK). Keep the two bank-33 definitions in sync (same addresses, different symbol names — this is the existing convention).

**Files:**
- Modify: `software/SDM/FARMEQU.S`
- Modify: `software/SDM/WORKEQU.S`

- [ ] **Step 1: Add bank-32 NPC equates to FARMEQU.S**

In `software/SDM/FARMEQU.S`, immediately after the `FGRID = $0300` line (the last bank-32 address equate, ~line 32), add:
```
* NPC TRADING (crop-side, bank 32)
FNPC = $0B00
FNPCB = $0C00
NCNPC = 5
NCBUY = 2
NPGROC = 0
NPTRDC = 1
NPSEED = 2
NPBULK = 3
NPEXOT = 4
OPTSELL = $08
OPTBUY = $09
```

- [ ] **Step 2: Add bank-33 NPC equates (WB names) to FARMEQU.S**

In `software/SDM/FARMEQU.S`, after the `WBRECIP = $0300` line (~line 142, the last bank-33 address), add:
```
* NPC TRADING (goods-side, bank 33)
WBNPC = $0500
WBNPCB = $0600
NGNPC = 2
NPBAKE = 0
NPTRDG = 1
WOPTSELL = $06
```

- [ ] **Step 3: Bump CVERNUM 5 → 6 in FARMEQU.S**

In `software/SDM/FARMEQU.S`, change:
```
CVERNUM = 5
```
to:
```
CVERNUM = 6
```
(Forces the farm/workshop CVER gate to fail on a warm world → full cold-seed → the new NPC regions get seeded. FARM_TEST is always-cold and unaffected.)

- [ ] **Step 4: Add bank-33 NPC equates (bare names) + PFLOOR to WORKEQU.S**

In `software/SDM/WORKEQU.S`, after the `WRECIP = $0300` line (~line 32), add:
```
* NPC TRADING (goods-side, bank 33)
WNPC = $0500
WNPCB = $0600
NGNPC = 2
WOPTSELL = $06
PFLOOR = 2
```
(WORKTASK has no `PFLOOR` of its own; the drift/drop clamps need it. FARMTASK already gets `PFLOOR = 2` from FARMEQU.S line 70.)

- [ ] **Step 5: Verify both files assemble cleanly via a dependent build**

Run:
```bash
cd /Users/hambook/Development/project_byte_hamr
make farmtask 2>&1 | grep -iE "Creating Object|Error" | tail -2
```
Expected: `Creating Object file ... FARMTASK.bin` (FARMTASK includes FARMEQU.S; a clean assemble proves the new bank-32 equates parse). WORKEQU.S is validated by Task 4's build.

- [ ] **Step 6: Commit**

```bash
cd /Users/hambook/Development/project_byte_hamr
git add software/SDM/FARMEQU.S software/SDM/WORKEQU.S
git commit -m "feat(econ): phase 2a step 1 - NPC trading equates + CVER 5->6

Bank-32 crop-NPC region (FNPC \$0B00 / FNPCB \$0C00, 5 NPCs) and bank-33
goods-NPC region (WBNPC/WNPC \$0500 / \$0600, 2 NPCs); OPTSELL/OPTBUY/WOPTSELL
op codes; NCNPC/NCBUY/NGNPC counts; PFLOOR added to WORKEQU. CVERNUM 5->6
cold-seeds the new regions on a warm world.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 2: FARMTASK — OPTSELL + OPTBUY handlers

Append the two handlers at the end of the executable code (before the `GROWM`/`BASEP`/`SEEDC` DFB block at ~line 874) so no existing branch distance grows. Wire them into the dispatch chain with `J*` trampolines (handlers are out of `BEQ` range). Reuse existing scratch: `TA0`=npc, `TA1`=crop, `TA2`=qty (set by `DOMBOX`); `CROPI`=offset; `PRC`=price; `CSHL`/`CSHH`/`QTY`=cash math; `PADRL`/`SEED`/`M20L`/`M20H`=inventory/cost (as in CSELL/CBUY).

**Files:**
- Modify: `software/SDM/FARMTASK.S` (dispatch chain ~line 184; trampolines ~line 196; new handlers before line 874)

- [ ] **Step 1: Extend the dispatch chain**

In `software/SDM/FARMTASK.S`, find the dispatch tail (the `CMP #OPSPEND` / `BEQ JSPEND` pair followed by the `LDA #RERRBAD` default):
```
 CMP #OPSPEND
 BEQ JSPEND
 LDA #RERRBAD
 JMP MBFIN
```
Change it to insert the two new ops before the default:
```
 CMP #OPSPEND
 BEQ JSPEND
 CMP #OPTSELL
 BEQ JOPTSL
 CMP #OPTBUY
 BEQ JOPTBY
 LDA #RERRBAD
 JMP MBFIN
```

- [ ] **Step 2: Add the two trampolines**

In `software/SDM/FARMTASK.S`, find the trampoline block (the `JSELL`/`JBUY`/`JWDRAW`/`JADDC`/`JSPEND` `JMP` lines just below the dispatch). After `JSPEND`'s `JMP CSPEND` line, add:
```
JOPTSL
 JMP COPTSELL
JOPTBY
 JMP COPTBUY
```

- [ ] **Step 3: Add the OPTSELL handler**

In `software/SDM/FARMTASK.S`, immediately before the `* per-crop profiles (blob-resident: hot loop)` comment + `GROWM DFB ...` block (~line 874), add:
```
* --- OPTSELL TA0=npc TA1=crop TA2=qty ---
* sell crop to a purchaser at its current price,
* credit cash, drop the NPC price (saturation)
COPTSELL
 LDA TA0
 CMP #NCBUY
 BCC OSNOK
 LDA #RERRBAD
 JMP MBFIN
OSNOK
 LDA TA1
 CMP #NCROPS
 BCC OSCOK
 LDA #RERRBAD
 JMP MBFIN
OSCOK
 LDA TA2
 BNE OSQOK
 LDA #RERRBAD
 JMP MBFIN
OSQOK
* CROPI = npc*32 + crop*2
 LDA TA0
 ASL
 ASL
 ASL
 ASL
 ASL
 STA CROPI
 LDA TA1
 ASL
 CLC
 ADC CROPI
 STA CROPI
* inventory FCROPS[crop] >= qty?
 LDA TA1
 CLC
 ADC #<FCROPS
 STA PADRL
 LDA PADRL
 LDY #>FCROPS
 JSR RDB
 CMP TA2
 BCS OSINV
 LDA #RERRCROP
 JMP MBFIN
OSINV
 SEC
 SBC TA2
 TAX
 LDA PADRL
 LDY #>FCROPS
 JSR WRB
* price = current NPC price ($0B00+CROPI)
 LDA CROPI
 LDY #>FNPC
 JSR RDB
 STA PRC
* cash += qty*price clamp $FFFF
 LDA #<FCASHL
 LDY #>FCASHL
 JSR RDB
 STA CSHL
 LDA #<FCASHH
 LDY #>FCASHH
 JSR RDB
 STA CSHH
 LDA TA2
 STA QTY
OSMUL
 LDA CSHL
 CLC
 ADC PRC
 STA CSHL
 LDA CSHH
 ADC #0
 STA CSHH
 BCC OSNC
 LDA #$FF
 STA CSHL
 STA CSHH
OSNC
 DEC QTY
 BNE OSMUL
 JSR WRCASH
* drop price = max(PFLOOR, price-qty)
 LDA PRC
 SEC
 SBC TA2
 BCC OSFL
 CMP #PFLOOR
 BCS OSWR
OSFL
 LDA #PFLOOR
OSWR
 TAX
 LDA CROPI
 LDY #>FNPC
 JSR WRB
 LDA #ROK
 JMP MBFIN
```

- [ ] **Step 4: Add the OPTBUY handler**

In `software/SDM/FARMTASK.S`, immediately after the OPTSELL handler (still before the `GROWM` DFB block), add:
```
* --- OPTBUY TA0=npc TA1=crop TA2=qty ---
* buy seeds from a vendor at its price, debit
* cash, raise the NPC price (demand)
COPTBUY
 LDA TA0
 CMP #NCBUY
 BCS OBNOK
 LDA #RERRBAD
 JMP MBFIN
OBNOK
 LDA TA0
 CMP #NCNPC
 BCC OBN2
 LDA #RERRBAD
 JMP MBFIN
OBN2
 LDA TA1
 CMP #NCROPS
 BCC OBCOK
 LDA #RERRBAD
 JMP MBFIN
OBCOK
 LDA TA2
 BNE OBQOK
 LDA #RERRBAD
 JMP MBFIN
OBQOK
* CROPI = npc*32 + crop*2
 LDA TA0
 ASL
 ASL
 ASL
 ASL
 ASL
 STA CROPI
 LDA TA1
 ASL
 CLC
 ADC CROPI
 STA CROPI
* seeds[crop] + qty <= 255?
 LDA TA1
 CLC
 ADC #<FSEEDS
 STA PADRL
 LDA PADRL
 LDY #>FSEEDS
 JSR RDB
 STA SEED
 CLC
 ADC TA2
 BCC OBROOM
 LDA #RERRFULL
 JMP MBFIN
OBROOM
* price = current NPC price
 LDA CROPI
 LDY #>FNPC
 JSR RDB
 STA PRC
* cost = qty*price -> M20L/M20H
 LDA #0
 STA M20L
 STA M20H
 LDA TA2
 STA QTY
OBCST
 LDA M20L
 CLC
 ADC PRC
 STA M20L
 LDA M20H
 ADC #0
 STA M20H
 DEC QTY
 BNE OBCST
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
 BCC OBPOOR
 BNE OBRICH
 LDA CSHL
 CMP M20L
 BCC OBPOOR
OBRICH
 LDA CSHL
 SEC
 SBC M20L
 STA CSHL
 LDA CSHH
 SBC M20H
 STA CSHH
 JSR WRCASH
* seeds[crop] += qty
 LDA SEED
 CLC
 ADC TA2
 TAX
 LDA PADRL
 LDY #>FSEEDS
 JSR WRB
* raise price = min(255, price+qty)
 LDA PRC
 CLC
 ADC TA2
 BCC OBWR
 LDA #$FF
OBWR
 TAX
 LDA CROPI
 LDY #>FNPC
 JSR WRB
 LDA #ROK
 JMP MBFIN
OBPOOR
 LDA #RERRCASH
 JMP MBFIN
```

- [ ] **Step 4b: Verify FARMTASK assembles + under the size cap**

Run:
```bash
cd /Users/hambook/Development/project_byte_hamr
make farmtask 2>&1 | grep -iE "Creating Object|Error|exceeds" | tail -3
wc -c < software/SDM/FARMTASK.bin
```
Expected: `Creating Object file ... FARMTASK.bin`, no error, size well under 8192 (was 1840; +~250 B).

- [ ] **Step 5: Commit**

```bash
cd /Users/hambook/Development/project_byte_hamr
git add software/SDM/FARMTASK.S
git commit -m "feat(econ): phase 2a - FARMTASK OPTSELL/OPTBUY trade ops

OPTSELL: sell crop to purchaser (npc<NCBUY) at its SDRAM price, credit
cash, drop price by qty (floor PFLOOR). OPTBUY: buy seeds from vendor
(npc>=NCBUY) at its price, debit cash, raise price by qty (clamp 255).
Dispatch + trampolines wired; handlers appended (no branch-distance growth).

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 3: FARMTASK — NPC crop-price drift (`NPCDFT`)

A per-tick mean-revert: every crop-NPC price steps ±1 toward its SDRAM base. Called once per market tick, right after `DOMKT` (which already publishes the base-market `EVPRICE` events the //e listens on — NPC drift piggybacks that tick, no new event type).

**Files:**
- Modify: `software/SDM/FARMTASK.S` (new routine before `GROWM` block; one call in `GLOOP`)

- [ ] **Step 1: Add the `NPCDFT` routine**

In `software/SDM/FARMTASK.S`, after the OPTBUY handler (before the `GROWM` DFB block), add:
```
* === NPCDFT: NPC crop prices revert to base ===
* every NPC*crop: step current +/-1 toward base
NPCDFT
 LDA #0
 STA SUP
NDNPC
 LDA #0
 STA CROPI
NDCROP
* M20L = npc*32 + crop*2
 LDA SUP
 ASL
 ASL
 ASL
 ASL
 ASL
 STA M20L
 LDA CROPI
 ASL
 CLC
 ADC M20L
 STA M20L
 LDA M20L
 LDY #>FNPC
 JSR RDB
 STA PRC
 LDA M20L
 LDY #>FNPCB
 JSR RDB
 STA TGT
 LDA PRC
 CMP TGT
 BEQ NDNX
 BCC NDUP
 DEC PRC
 JMP NDWR
NDUP
 INC PRC
NDWR
 LDX PRC
 LDA M20L
 LDY #>FNPC
 JSR WRB
NDNX
 INC CROPI
 LDA CROPI
 CMP #NCROPS
 BNE NDCROP
 INC SUP
 LDA SUP
 CMP #NCNPC
 BNE NDNPC
 RTS
```
(`SUP` = npc index, `CROPI` = crop index, `M20L` = offset, `PRC`/`TGT` = current/base — all DOMKT scratch, free here since NPCDFT runs right after DOMKT, never concurrently.)

- [ ] **Step 2: Wire `NPCDFT` into the market tick**

In `software/SDM/FARMTASK.S`, find the tick-fire pair in `GLOOP`:
```
 JSR DOMKT
 JSR RELOADM
```
Change it to:
```
 JSR DOMKT
 JSR NPCDFT
 JSR RELOADM
```

- [ ] **Step 3: Verify assemble**

Run:
```bash
cd /Users/hambook/Development/project_byte_hamr
make farmtask 2>&1 | grep -iE "Creating Object|Error|exceeds" | tail -2
```
Expected: clean assemble, no error.

- [ ] **Step 4: Commit**

```bash
cd /Users/hambook/Development/project_byte_hamr
git add software/SDM/FARMTASK.S
git commit -m "feat(econ): phase 2a - FARMTASK NPC crop-price drift

NPCDFT steps every crop-NPC price +/-1 toward its SDRAM base each market
tick (called after DOMKT). Mean-reverts the supply/demand impact from
OPTSELL/OPTBUY back to base. No new event - piggybacks DOMKT's EVPRICE tick.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 4: WORKTASK — WOPTSELL handler

Sell a good to a goods-purchaser. Mirrors the existing `WSELL`: returns the per-unit NPC price in `WRES1` (the //e multiplies by qty and credits cash host-side; bank 33 has no cash field), debits the goods inventory, drops the NPC price. Append after the existing code, dispatch via a `J*` trampoline. WORKTASK has near-limit branches — do NOT splice into existing routines.

**Files:**
- Modify: `software/SDM/WORKTASK.S` (dispatch ~line 207; trampoline ~line 217; handler appended after the last routine, before any trailing DFB/`PUT`)

- [ ] **Step 1: Extend the dispatch chain**

In `software/SDM/WORKTASK.S`, find:
```
 CMP #WOPLEARN
 BEQ WJLEARN
 LDA #RERRBAD
 JMP WMFIN
```
Change to:
```
 CMP #WOPLEARN
 BEQ WJLEARN
 CMP #WOPTSELL
 BEQ WJTSL
 LDA #RERRBAD
 JMP WMFIN
```

- [ ] **Step 2: Add the trampoline**

In `software/SDM/WORKTASK.S`, find the trampoline block (`WJCRAFT`/`WJSELL`/`WJMODE`/`WJLEARN` `JMP` lines). After `WJLEARN`'s `JMP WCLEARN`, add:
```
WJTSL
 JMP WOPTSLH
```

- [ ] **Step 3: Add the WOPTSELL handler**

In `software/SDM/WORKTASK.S`, find the `FAILBASE DFB 48,80,112,144` line (the blob's only DFB table, ~line 602). Immediately BEFORE the `* known-craft failChance base per rarity /256` comment that precedes it, add:
```
* --- WOPTSELL TI0=npc TI1=good TI2=qty ---
* sell good to a goods-purchaser; per-unit NPC
* price -> WRES1 (//e does qty*price + cash);
* debit goods, drop the NPC price (saturation)
WOPTSLH
 LDA TI0
 CMP #NGNPC
 BCS WTSBAD
 LDA TI1
 CMP #NRECIP
 BCS WTSBAD
 LDA TI2
 BEQ WTSBAD
* goods[good] >= qty?
 LDA TI1
 CLC
 ADC #<WGOODS
 STA RADL
 LDA #0
 ADC #>WGOODS
 STA RADH
 LDA RADL
 LDY RADH
 JSR RDB
 CMP TI2
 BCS WTSOK
 LDA #RERRCROP
 JMP WMFIN
WTSOK
 SEC
 SBC TI2
 TAX
 LDA RADL
 LDY RADH
 JSR WRB
* RADL = npc*128 + good*2 (hi = >WNPC)
 LDA #0
 STA RADL
 LDA TI0
 BEQ WTSN0
 LDA #128
 STA RADL
WTSN0
 LDA TI1
 ASL
 CLC
 ADC RADL
 STA RADL
* per-unit price = WNPC[off]
 LDA RADL
 LDY #>WNPC
 JSR RDB
 STA MVAL
* WRES1 = per-unit price
 LDX MVAL
 LDA #<WRES1
 LDY #>WRES1
 JSR WRB
* drop price = max(PFLOOR, price-qty)
 LDA MVAL
 SEC
 SBC TI2
 BCC WTSFL
 CMP #PFLOOR
 BCS WTSDW
WTSFL
 LDA #PFLOOR
WTSDW
 TAX
 LDA RADL
 LDY #>WNPC
 JSR WRB
 LDA #ROK
 JMP WMFIN
WTSBAD
 LDA #RERRBAD
 JMP WMFIN
```

- [ ] **Step 4: Verify WORKTASK assembles + under cap (validates WORKEQU.S Task 1 edits)**

Run:
```bash
cd /Users/hambook/Development/project_byte_hamr
make worktask 2>&1 | grep -iE "Creating Object|Error|exceeds" | tail -3
wc -c < software/SDM/WORKTASK.bin
```
Expected: `Creating Object file ... WORKTASK.bin`, no error, size under 8192 (was 1225).

- [ ] **Step 5: Commit**

```bash
cd /Users/hambook/Development/project_byte_hamr
git add software/SDM/WORKTASK.S
git commit -m "feat(econ): phase 2a - WORKTASK WOPTSELL trade op

Sell good to a goods-purchaser (npc<NGNPC): debit goods, return per-unit
NPC price in WRES1 (//e multiplies + credits cash, like WSELL), drop the
NPC price by qty (floor PFLOOR). Appended + trampoline-dispatched to avoid
growing the near-limit branches.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 5: WORKTASK — NPC goods-price drift (`WNPDFT`)

Per-tick mean-revert for goods-NPC prices, called next to `WTICK` on the station-tick counter.

**Files:**
- Modify: `software/SDM/WORKTASK.S` (new routine appended; one call in `WLOOP`)

- [ ] **Step 1: Add the `WNPDFT` routine**

In `software/SDM/WORKTASK.S`, immediately after the WOPTSELL handler (still before the `FAILBASE` comment/DFB), add:
```
* === WNPDFT: goods-NPC prices revert to base ===
WNPDFT
 LDA #0
 STA STI
WNNPC
 LDA #0
 STA MTMP
WNGOOD
* RADL = npc*128 + good*2
 LDA #0
 STA RADL
 LDA STI
 BEQ WNN0
 LDA #128
 STA RADL
WNN0
 LDA MTMP
 ASL
 CLC
 ADC RADL
 STA RADL
 LDA RADL
 LDY #>WNPC
 JSR RDB
 STA MVAL
 LDA RADL
 LDY #>WNPCB
 JSR RDB
 STA EVP1
 LDA MVAL
 CMP EVP1
 BEQ WNNX
 BCC WNUP
 DEC MVAL
 JMP WNWR
WNUP
 INC MVAL
WNWR
 LDX MVAL
 LDA RADL
 LDY #>WNPC
 JSR WRB
WNNX
 INC MTMP
 LDA MTMP
 CMP #NRECIP
 BNE WNGOOD
 INC STI
 LDA STI
 CMP #NGNPC
 BNE WNNPC
 RTS
```
(`STI` = npc, `MTMP` = good, `RADL` = offset, `MVAL` = current, `EVP1` = base temp. `WNPDFT` runs after `WTICK` returns — no `PUTEV` here, so reusing the `EVP1` event-scratch byte is safe.)

- [ ] **Step 2: Wire `WNPDFT` into the station tick**

In `software/SDM/WORKTASK.S`, find in `WLOOP`:
```
 JSR WTICK
 JSR WRELOAD
```
Change to:
```
 JSR WTICK
 JSR WNPDFT
 JSR WRELOAD
```

- [ ] **Step 3: Verify assemble**

Run:
```bash
cd /Users/hambook/Development/project_byte_hamr
make worktask 2>&1 | grep -iE "Creating Object|Error|exceeds" | tail -2
```
Expected: clean assemble.

- [ ] **Step 4: Commit**

```bash
cd /Users/hambook/Development/project_byte_hamr
git add software/SDM/WORKTASK.S
git commit -m "feat(econ): phase 2a - WORKTASK NPC goods-price drift

WNPDFT steps every goods-NPC price +/-1 toward its SDRAM base each station
tick (called after WTICK). Mean-reverts WOPTSELL's saturation drop.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 6: Shared NPC base table + //e cold-seed

Create the single-authority base-price table file and the //e routines that stream it into both SDRAM regions (current = base = the table values) at cold-seed. Crop NPCs seed in `COLDST` (bank 32); goods NPCs seed in `SEED33` (bank 33).

**Files:**
- Create: `software/SDM/NPCBASE.S`
- Modify: `software/SDM/FARM.S` (PUT NPCBASE; `SEEDNPC`/`SEEDNPG` routines + ZP vars; calls in COLDST + SEED33)

- [ ] **Step 1: Create `software/SDM/NPCBASE.S`**

Create the file with exactly:
```
* NPC base price tables - SINGLE AUTHORITY
* PUT by FARM.S (game seeder) + FARMTEST.S (test
* seeder). The task blobs read base from SDRAM;
* this is the only place the numbers live.
* crop NPCs: 5 x NCROPS, [npc*NCROPS + crop]
* (wheat carrot berry pumpkin)
NPCBASE
 DFB 8,14,20,32
 DFB 4,8,40,64
 DFB 2,4,7,12
 DFB 1,3,7,12
 DFB 3,6,5,9
* goods NPCs: 2 x NRECIP, [npc*NRECIP + recipe]
NPGBASE
 DFB 22,30,38,50,62,72,72,80,118,124,124,130
 DFB 18,25,32,43,54,64,64,72,135,145,145,160
```

- [ ] **Step 2: Add ZP scratch vars for the seed loops in FARM.S**

In `software/SDM/FARM.S`, find the block of state-var equates near the top (where `PREVSCR`, `INVPAGE`, `INVCUR`, `INVOFF`, `INVSELL` are declared, ~lines 98-102). After the last one, add three new bytes (pick the next free ZP addresses in that contiguous block — read the existing declarations to find the next address; they are sequential `XX = $nn` lines):
```
NIDX = $<next>
NPI = $<next+1>
NCI = $<next+2>
```
(Replace `$<next>` with the actual next free addresses after `INVSELL`. `NTMP` already exists and is reused as the offset scratch.)

- [ ] **Step 3: Add `SEEDNPC` + `SEEDNPG` to FARM.S**

In `software/SDM/FARM.S`, after the `SEED33` routine (it ends with `RTS` ~line 2398), add both routines:
```
* seed crop-NPC tables (bank 32): current+base =
* NPCBASE, NPC n crop c at \$0B00/\$0C00 + n*32+c*2
SEEDNPC
 LDA #0
 STA NIDX
 STA NPI
SNNP
 LDA #0
 STA NCI
SNCR
 LDA NPI
 ASL
 ASL
 ASL
 ASL
 ASL
 STA NTMP
 LDA NCI
 ASL
 CLC
 ADC NTMP
 STA NTMP
 LDX NIDX
 LDA NPCBASE,X
 STA SDM_VAL
 LDA NTMP
 LDY #>FNPC
 JSR FWR
 LDX NIDX
 LDA NPCBASE,X
 STA SDM_VAL
 LDA NTMP
 LDY #>FNPCB
 JSR FWR
 INC NIDX
 INC NCI
 LDA NCI
 CMP #NCROPS
 BNE SNCR
 INC NPI
 LDA NPI
 CMP #NCNPC
 BNE SNNP
 RTS
* seed goods-NPC tables (bank 33): current+base =
* NPGBASE, NPC n good g at \$0500/\$0600 + n*128+g*2
SEEDNPG
 LDA #0
 STA NIDX
 STA NPI
SGNP
 LDA #0
 STA NCI
SGGD
 LDA #0
 STA NTMP
 LDA NPI
 BEQ SGN0
 LDA #128
 STA NTMP
SGN0
 LDA NCI
 ASL
 CLC
 ADC NTMP
 STA NTMP
 LDX NIDX
 LDA NPGBASE,X
 STA SDM_VAL
 LDA NTMP
 LDY #>WBNPC
 JSR WFWR
 LDX NIDX
 LDA NPGBASE,X
 STA SDM_VAL
 LDA NTMP
 LDY #>WBNPCB
 JSR WFWR
 INC NIDX
 INC NCI
 LDA NCI
 CMP #NRECIP
 BNE SGGD
 INC NPI
 LDA NPI
 CMP #NGNPC
 BNE SGNP
 RTS
```

- [ ] **Step 4: Call `SEEDNPC` from COLDST**

In `software/SDM/FARM.S`, find in `COLDST` the line `JSR SEED33` (~line 1476). Immediately BEFORE it, add:
```
 JSR SEEDNPC
```
(Crop NPC seeding runs in the bank-32 cold-seed phase, after the market base prices, before the bank-33 seed + task spawn — preserving the strict data-first ordering.)

- [ ] **Step 5: Call `SEEDNPG` from SEED33**

In `software/SDM/FARM.S`, find the end of `SEED33` (the RECTAB stream loop, then `RTS` ~line 2398). Immediately BEFORE the final `RTS`, add:
```
 JSR SEEDNPG
```

- [ ] **Step 6: PUT NPCBASE.S into FARM.S**

In `software/SDM/FARM.S`, find where the existing tables/blobs are PUT or where the source ends. Add (near the other `PUT` includes, e.g. alongside `PUT FARMEQU`):
```
 PUT NPCBASE
```
(Verify placement: `NPCBASE`/`NPGBASE` labels must be reachable by `SEEDNPC`/`SEEDNPG` absolute-indexed reads — i.e. resident in the FARM.S binary. A `PUT` at any top-level point includes the DFB data in the assembly.)

- [ ] **Step 7: Build FARM.S, confirm clean + NPCBASE resolved**

Run:
```bash
cd /Users/hambook/Development/project_byte_hamr
make farm 2>&1 | grep -iE "Creating Object file.*FARM.bin|Error|Unknown label" | tail -3
```
Expected: `Creating Object file ... FARM.bin`, no `Unknown label NPCBASE/NPGBASE`, no error.

- [ ] **Step 8: Add NPCBASE.S to the FARM.bin + FARMTEST.bin make dependencies**

In `/Users/hambook/Development/project_byte_hamr/Makefile`, find the `$(FARMTEST_BIN):` prerequisite list (it names `FARMTEST.S FARMEQU.S SDRAMLIB.S CPLIB.S` plus the blobs) and the `farm` target's FARM.bin rule. Add `$(SDM_DIR)/NPCBASE.S` to BOTH prerequisite lists so a base-table edit forces a rebuild (avoids the documented stale-include hazard). Example for the farmtest bin line:
```
$(FARMTEST_BIN): $(FARMTASKB_S) $(WORKTASKB_S) $(SDM_DIR)/FARMTEST.S $(SDM_DIR)/FARMEQU.S $(SDM_DIR)/NPCBASE.S $(SDM_DIR)/SDRAMLIB.S $(SDM_DIR)/CPLIB.S
```
(Find the actual FARM.bin rule — it depends on FARM.S; append `$(SDM_DIR)/NPCBASE.S` there too.)

- [ ] **Step 9: Commit**

```bash
cd /Users/hambook/Development/project_byte_hamr
git add software/SDM/NPCBASE.S software/SDM/FARM.S Makefile
git commit -m "feat(econ): phase 2a - shared NPC base table + //e cold-seed

NPCBASE.S = single authority for NPC base prices (PUT by FARM.S + test).
SEEDNPC streams crop-NPC bases into bank-32 \$0B00(current)/\$0C00(base) in
COLDST; SEEDNPG streams goods-NPC bases into bank-33 \$0500/\$0600 in SEED33.
Task blobs read base from SDRAM - no duplicate tables. Make deps updated.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 7: FARMTEST — seed NPC regions + farm trade-op tests

The test's `SETUP` zero-fills bank-32 `$0000-$0AFF` and bank-33 `$0200-$027F` — neither covers the new NPC regions, so SETUP must seed them (reusing the shared `NPCBASE.S` via PUT). Then add deterministic OPTSELL/OPTBUY cases (force inventory, cash, and a known NPC price; drive one op; assert cash delta, inventory delta, and price moved the right way).

**Files:**
- Modify: `software/SDM/FARMTEST.S` (PUT NPCBASE; SETUP seed loops; new label strings; new test cases)

- [ ] **Step 1: PUT NPCBASE.S into FARMTEST.S**

In `software/SDM/FARMTEST.S`, near the other includes (`PUT FARMEQU` ~line 14), add:
```
 PUT NPCBASE
```

- [ ] **Step 2: Seed the NPC regions in SETUP**

In `software/SDM/FARMTEST.S`, find the end of `SETUP` — after the RECTAB stream (~line 727) and the `WBSKILL = $FF` write (~line 733), before the drain-var init / spawn (~line 734). Insert a call to a new combined seeder:
```
 JSR TSEEDNPC
```
Then add the `TSEEDNPC` routine itself just after `SETUP`'s `RTS` (a self-contained copy of the FARM.S seed logic, since FARMTEST is a separate binary; reads the PUT'd `NPCBASE`/`NPGBASE`). Use the test's own free ZP — reuse `CNT` (2 bytes) and `RECLO` for the npc/crop/idx counters, or declare `TNIDX`/`TNPI`/`TNCI` near the other test ZP (lines 17-59). Add the declarations (next free ZP after `LAPN`):
```
TNIDX = $<next>
TNPI = $<next+1>
TNCI = $<next+2>
TNOFF = $<next+3>
```
and the routine:
```
* seed both NPC regions for the test fixture
TSEEDNPC
 LDA #0
 STA TNIDX
 STA TNPI
TSNP
 LDA #0
 STA TNCI
TSCR
 LDA TNPI
 ASL
 ASL
 ASL
 ASL
 ASL
 STA TNOFF
 LDA TNCI
 ASL
 CLC
 ADC TNOFF
 STA TNOFF
 LDX TNIDX
 LDA NPCBASE,X
 STA SDM_VAL
 LDA TNOFF
 LDY #>FNPC
 JSR FWR
 LDX TNIDX
 LDA NPCBASE,X
 STA SDM_VAL
 LDA TNOFF
 LDY #>FNPCB
 JSR FWR
 INC TNIDX
 INC TNCI
 LDA TNCI
 CMP #NCROPS
 BNE TSCR
 INC TNPI
 LDA TNPI
 CMP #NCNPC
 BNE TSNP
* goods NPCs
 LDA #0
 STA TNIDX
 STA TNPI
TGNP
 LDA #0
 STA TNCI
TGGD
 LDA #0
 STA TNOFF
 LDA TNPI
 BEQ TGN0
 LDA #128
 STA TNOFF
TGN0
 LDA TNCI
 ASL
 CLC
 ADC TNOFF
 STA TNOFF
 LDX TNIDX
 LDA NPGBASE,X
 STA SDM_VAL
 LDA TNOFF
 LDY #>WBNPC
 JSR WFWR
 LDX TNIDX
 LDA NPGBASE,X
 STA SDM_VAL
 LDA TNOFF
 LDY #>WBNPCB
 JSR WFWR
 INC TNIDX
 INC TNCI
 LDA TNCI
 CMP #NRECIP
 BNE TGGD
 INC TNPI
 LDA TNPI
 CMP #NGNPC
 BNE TGNP
 RTS
```

- [ ] **Step 3: Add the test-label strings**

In `software/SDM/FARMTEST.S`, in the label-string block (the `ASC "..." / DFB 0` definitions ~lines 871-974), add:
```
LLOSE ASC "OPTSELL CASH"
 DFB 0
LLOSI ASC "OPTSELL INV"
 DFB 0
LLOSP ASC "OPTSELL PRICE DROP"
 DFB 0
LLOSB ASC "OPTSELL BAD NPC"
 DFB 0
LLOBC ASC "OPTBUY CASH"
 DFB 0
LLOBS ASC "OPTBUY SEEDS"
 DFB 0
LLOBP ASC "OPTBUY PRICE RISE"
 DFB 0
LLOBB ASC "OPTBUY BAD NPC"
 DFB 0
```

- [ ] **Step 4: Add the OPTSELL + OPTBUY test cases**

In `software/SDM/FARMTEST.S`, find a clean point after the existing farm-op tests (after the SELL/BUY cases, before the workshop block — match the style of TEST blocks). Add:
```
* TEST OS1: OPTSELL berry x3 to TRADER-crops(1).
* Force crops[2]=5, cash=100, NPC1 berry price=40
* ($0B00+1*32+2*2=$0B24). Expect ROK, cash=220
* (100+3*40), crops[2]=2, price 40-3=37.
 LDA #5
 STA SDM_VAL
 LDA #<FCROPS+2
 LDY #>FCROPS
 JSR FWR
 LDA #100
 STA SDM_VAL
 LDA #<FCASHL
 LDY #>FCASHL
 JSR FWR
 LDA #0
 STA SDM_VAL
 LDA #<FCASHH
 LDY #>FCASHH
 JSR FWR
 LDA #40
 STA SDM_VAL
 LDA #$24
 LDY #$0B
 JSR FWR
 LDA #OPTSELL
 STA CMDOP
 LDA #NPTRDC
 STA CMDA0
 LDA #2
 STA CMDA1
 LDA #3
 STA CMDA2
 LDA #ROK
 STA TEXP
 LDA #<LLOSE
 STA MSGPTR
 LDA #>LLOSE
 STA MSGPTR+1
 JSR SENDCMD
 JSR DOTEST
* cash low == 220
 LDA #220
 STA TEXP
 LDA #<LLOSE
 STA MSGPTR
 LDA #>LLOSE
 STA MSGPTR+1
 LDA #<FCASHL
 LDY #>FCASHL
 JSR FRDCHK
* crops[2] == 2
 LDA #2
 STA TEXP
 LDA #<LLOSI
 STA MSGPTR
 LDA #>LLOSI
 STA MSGPTR+1
 LDA #<FCROPS+2
 LDY #>FCROPS
 JSR FRDCHK
* NPC1 berry price == 37
 LDA #37
 STA TEXP
 LDA #<LLOSP
 STA MSGPTR
 LDA #>LLOSP
 STA MSGPTR+1
 LDA #$24
 LDY #$0B
 JSR FRDCHK
* TEST OS2: OPTSELL to a vendor (npc=NPSEED) ->
* RERRBAD (not a purchaser)
 LDA #OPTSELL
 STA CMDOP
 LDA #NPSEED
 STA CMDA0
 LDA #0
 STA CMDA1
 LDA #1
 STA CMDA2
 LDA #RERRBAD
 STA TEXP
 LDA #<LLOSB
 STA MSGPTR
 LDA #>LLOSB
 STA MSGPTR+1
 JSR SENDCMD
 JSR DOTEST
* TEST OB1: OPTBUY wheat x5 from SEED-SHOP(2).
* Force cash=100, NPC2 wheat price=2 ($0B40=
* $0B00+2*32+0). Expect ROK, cash=90 (100-5*2),
* seeds[0]=base+5, price 2+5=7.
 LDA #100
 STA SDM_VAL
 LDA #<FCASHL
 LDY #>FCASHL
 JSR FWR
 LDA #0
 STA SDM_VAL
 LDA #<FCASHH
 LDY #>FCASHH
 JSR FWR
 LDA #0
 STA SDM_VAL
 LDA #<FSEEDS
 LDY #>FSEEDS
 JSR FWR
 LDA #2
 STA SDM_VAL
 LDA #$40
 LDY #$0B
 JSR FWR
 LDA #OPTBUY
 STA CMDOP
 LDA #NPSEED
 STA CMDA0
 LDA #0
 STA CMDA1
 LDA #5
 STA CMDA2
 LDA #ROK
 STA TEXP
 LDA #<LLOBC
 STA MSGPTR
 LDA #>LLOBC
 STA MSGPTR+1
 JSR SENDCMD
 JSR DOTEST
* cash low == 90
 LDA #90
 STA TEXP
 LDA #<LLOBC
 STA MSGPTR
 LDA #>LLOBC
 STA MSGPTR+1
 LDA #<FCASHL
 LDY #>FCASHL
 JSR FRDCHK
* seeds[0] == 5
 LDA #5
 STA TEXP
 LDA #<LLOBS
 STA MSGPTR
 LDA #>LLOBS
 STA MSGPTR+1
 LDA #<FSEEDS
 LDY #>FSEEDS
 JSR FRDCHK
* NPC2 wheat price == 7
 LDA #7
 STA TEXP
 LDA #<LLOBP
 STA MSGPTR
 LDA #>LLOBP
 STA MSGPTR+1
 LDA #$40
 LDY #$0B
 JSR FRDCHK
* TEST OB2: OPTBUY from a purchaser (npc=NPGROC)
* -> RERRBAD (not a vendor)
 LDA #OPTBUY
 STA CMDOP
 LDA #NPGROC
 STA CMDA0
 LDA #0
 STA CMDA1
 LDA #1
 STA CMDA2
 LDA #RERRBAD
 STA TEXP
 LDA #<LLOBB
 STA MSGPTR
 LDA #>LLOBB
 STA MSGPTR+1
 JSR SENDCMD
 JSR DOTEST
```

- [ ] **Step 5: Build farmtest, confirm clean**

Run:
```bash
cd /Users/hambook/Development/project_byte_hamr
make farmtest 2>&1 | grep -iE "Creating Object file.*FARMTEST|Error|Unknown label" | tail -3
```
Expected: `Creating Object file ... FARMTEST.bin`, no error.

- [ ] **Step 6: Commit**

```bash
cd /Users/hambook/Development/project_byte_hamr
git add software/SDM/FARMTEST.S
git commit -m "test(econ): phase 2a - FARMTEST seeds NPC regions + farm trade tests

SETUP seeds bank-32/33 NPC current+base from the shared NPCBASE (PUT).
OPTSELL: assert cash credit, inventory debit, NPC price drop, bad-NPC
reject. OPTBUY: assert cash debit, seed credit, NPC price rise, bad-NPC
reject. All forced/deterministic - no tick waits.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 8: FARMTEST — workshop trade-op test + drift "see a click"

WOPTSELL test (force goods + known price, sell, assert per-unit price in `WRES1` + inventory + price drop), and the ONE allowed tick-wait test: force a crop-NPC price off-base, wait one market tick, assert it stepped toward base.

**Files:**
- Modify: `software/SDM/FARMTEST.S` (label strings; WOPTSELL case; drift case)

- [ ] **Step 1: Add label strings**

In `software/SDM/FARMTEST.S` label block, add:
```
LLWTE ASC "WOPTSELL"
 DFB 0
LLWTV ASC "WOPTSELL PRICE"
 DFB 0
LLWTG ASC "WOPTSELL GOODS"
 DFB 0
LLWTP ASC "WOPTSELL DROP"
 DFB 0
LLNDR ASC "NPC DRIFT TICK"
 DFB 0
```

- [ ] **Step 2: Add the WOPTSELL test case**

In `software/SDM/FARMTEST.S`, after the existing WSELL test block (the W5/W6 cases ~line 1415), add:
```
* TEST WT1: WOPTSELL good0 x3 to BAKER(0). Force
* goods[0]=5, BAKER good0 price=22 ($0500). Expect
* ROK, WRES1=22, goods[0]=2, price 22-3=19.
 LDA #5
 STA SDM_VAL
 LDA #<WBGOODS
 LDY #>WBGOODS
 JSR WFWR
 LDA #22
 STA SDM_VAL
 LDA #$00
 LDY #$05
 JSR WFWR
 LDA #WOPTSELL
 STA CMDOP2
 LDA #NPBAKE
 STA CMDA02
 LDA #0
 STA CMDA12
 LDA #3
 STA CMDA22
 LDA #0
 STA CMDA32
 LDA #ROK
 STA TEXP
 LDA #<LLWTE
 STA MSGPTR
 LDA #>LLWTE
 STA MSGPTR+1
 JSR WSENDCMD
 JSR DOTEST
* WRES1V == 22 (per-unit price)
 LDA #<LLWTV
 STA MSGPTR
 LDA #>LLWTV
 STA MSGPTR+1
 LDA WRES1V
 CMP #22
 BNE WTVBF
 CLC
 LDA #1
 STA TEXP
 LDA #1
 JSR DOTEST
 JMP WTVBD
WTVBF
 SEC
 JSR DOTEST
WTVBD
* goods[0] == 2
 LDA #2
 STA TEXP
 LDA #<LLWTG
 STA MSGPTR
 LDA #>LLWTG
 STA MSGPTR+1
 LDA #<WBGOODS
 LDY #>WBGOODS
 JSR WRDCHK
* BAKER good0 price == 19
 LDA #19
 STA TEXP
 LDA #<LLWTP
 STA MSGPTR
 LDA #>LLWTP
 STA MSGPTR+1
 LDA #$00
 LDY #$05
 JSR WRDCHK
```

- [ ] **Step 3: Add the drift "see a click" test**

In `software/SDM/FARMTEST.S`, after the WOPTSELL case (this is the one allowed tick-wait — place it near the existing EV_PRICE / "see a click" tests so it shares that wait window). Add:
```
* TEST ND1: NPC drift. GROCER(0) wheat base=8.
* Force its current price to 20 ($0B00), then wait
* one market tick; NPCDFT must step it DOWN toward
* base. Poll until <20 (one click is enough), wide
* timeout. PASS if it dropped, FAIL on timeout.
 LDA #20
 STA SDM_VAL
 LDA #$00
 LDY #$0B
 JSR FWR
 LDA #0
 STA POLLC
 STA POLLC+1
NDRLP
 LDA #$00
 LDY #$0B
 JSR FRD
 LDA SDM_VAL
 CMP #20
 BCC NDROK
 INC POLLC
 BNE NDRLP
 INC POLLC+1
 LDA POLLC+1
 CMP #$80
 BNE NDRLP
* timeout -> FAIL
 SEC
 LDA #<LLNDR
 STA MSGPTR
 LDA #>LLNDR
 STA MSGPTR+1
 JSR DOTEST
 JMP NDRDN
NDROK
 LDA #1
 STA TEXP
 LDA #<LLNDR
 STA MSGPTR
 LDA #>LLNDR
 STA MSGPTR+1
 CLC
 LDA #1
 JSR DOTEST
NDRDN
```
(The wide 16-bit `POLLC` timeout (`$8000` iterations) only matters on the miss path — one market tick on hardware drops 20→19, well inside it. `POLLC` is the existing poll-timeout ZP pair. `DOTEST` with `A=1`/`TEXP=1`/`C=0` = PASS; `C=1` = FAIL.)

- [ ] **Step 4: Build farmtest, confirm clean**

Run:
```bash
cd /Users/hambook/Development/project_byte_hamr
make farmtest 2>&1 | grep -iE "Creating Object file.*FARMTEST|Error|Unknown label" | tail -3
```
Expected: clean assemble.

- [ ] **Step 5: Commit**

```bash
cd /Users/hambook/Development/project_byte_hamr
git add software/SDM/FARMTEST.S
git commit -m "test(econ): phase 2a - WOPTSELL test + NPC drift see-a-click

WOPTSELL: force goods + known BAKER price, sell, assert per-unit price in
WRES1, goods debit, price drop. Drift: force GROCER wheat off-base, wait
one market tick (the one allowed tick-wait), assert it stepped toward base.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 9: Build disk + verify artifacts + bench validation

**Files:** none (build + verify + on-hardware run).

- [ ] **Step 1: Full clean rebuild of the farm artifacts + embed==bin guard**

Run:
```bash
cd /Users/hambook/Development/project_byte_hamr
make farm farmtest 2>&1 | grep -iE "Creating Object|Error|exceeds|Unknown" | tail -6
cd software/SDM
echo "FARMTASK embed/bin: $(grep -c DFB FARMTASKB.S) / $(wc -c <FARMTASK.bin)"
echo "WORKTASK embed/bin: $(grep -c DFB WORKTASKB.S) / $(wc -c <WORKTASK.bin)"
```
Expected: all assemble clean; `grep -c DFB FARMTASKB.S` == `wc -c FARMTASK.bin` and same for WORKTASK (the embedded blobs match the freshly-built bins — the documented stale-embed hazard). FARMTASK/WORKTASK both well under 8192.

- [ ] **Step 2: Build the disk**

Run:
```bash
cd /Users/hambook/Development/project_byte_hamr
make sdmdisk 2>&1 | tail -5
```
Expected: `software/SDM/SDMTEST.po` built with FARM + FARM_TEST + CONWAY_TEST. No MAXLEN/guard trips.

- [ ] **Step 3: Burn + run FARM_TEST (user, on the //e)**

Burn `SDMTEST.po`, cold-reset, `BRUN FARM_TEST`.
Expected: full suite green, including the new cases — `OPTSELL CASH/INV/PRICE DROP`, `OPTSELL BAD NPC`, `OPTBUY CASH/SEEDS/PRICE RISE`, `OPTBUY BAD NPC`, `WOPTSELL`/`WOPTSELL PRICE/GOODS/DROP`, and `NPC DRIFT TICK`. The drift test takes up to one market-tick (~seconds) to go green — that is the single allowed tick-wait.

- [ ] **Step 4: If any red — iterate on disk only (NO re-flash)**

A failing trade-op or drift case is a blob/test fix: edit the relevant `.S`, `make farm farmtest && make sdmdisk`, re-burn, re-run. The gateware/kernel are unchanged, so never re-flash. Common suspects: an offset miscompute (`npc*32+crop*2` / `npc*128+good*2`), a dispatch CMP omitted, a SETUP seed missed (NPC region uninitialized), or a clamp/floor off-by-one.

---

## Self-Review

**Spec coverage (§2 engine + §3 migration + §4 testing):**
- NPCs (3 purchasers + 3 vendors): crop-side GROCER/TRADER-crops/SEED-SHOP/BULK/EXOTIC (Task 1 constants, Task 6 NPCBASE); goods-side BAKER/TRADER-goods (Task 1, Task 6). TRADER "spans both banks" — realized as TRADER-crops (bank 32, NPC 1) + TRADER-goods (bank 33, NPC 1); the UI (2B) presents them as one NPC. ✓
- Supply/demand fluctuation (sell drops, buy raises, per-tick mean-revert): OPTSELL drop (Task 2), OPTBUY raise (Task 2), WOPTSELL drop (Task 4), NPCDFT/WNPDFT mean-revert (Tasks 3,5). ✓
- Approach A (extend existing tasks, single-writer-per-bank): crop NPCs in FARMTASK/bank32, goods NPCs in WORKTASK/bank33. ✓
- SDRAM layout (MAX-sized, live counts bound loops): `$0B00`/`$0C00` bank 32, `$0500`/`$0600` bank 33; loops bound by NCROPS=4 / NRECIP=12 (Task 1, layout table). ✓ — **deviation:** base in SDRAM not blob (user-approved; noted in Architecture).
- Trade ops OPTSELL/OPTBUY/WOPTSELL with the documented validation (RERRBAD/RERRCROP/RERRCASH/RERRFULL): Tasks 2,4. ✓
- Migration CVER 5→6 cold-seeds both banks (init current=base): Task 1 Step 3 + Task 6 seed. ✓
- Testing all in FARM_TEST, forced/deterministic, drift = one allowed tick-wait: Tasks 7,8. ✓
- Trade UI (§2) — **out of scope for 2A; Plan 2B** (stated in Goal/Spec). ✓

**Placeholder scan:** the only intentional `$<next>` placeholders are the ZP-address allocations (Task 6 Step 2, Task 7 Step 2) — these MUST be resolved by reading the actual next-free ZP in each file at implementation time (they cannot be hard-coded blind without risking a collision). Every routine, op, table value, and test assertion is literal. No TODO/TBD.

**Type/value consistency:** op codes `OPTSELL=$08`/`OPTBUY=$09`/`WOPTSELL=$06` defined once (Task 1) and referenced by dispatch (Tasks 2,4) + tests (Tasks 7,8). NPC indices `NPGROC=0`...`NPEXOT=4`/`NPBAKE=0`/`NPTRDG=1` consistent across handlers, seed, tests. Addresses: `$0B00+1*32+2*2=$0B24` (OS1), `$0B00+2*32+0=$0B40` (OB1), `$0500+0=$0500` (WT1), `$0B00` (drift) — all match the layout table. Base values in NPCBASE.S match the NPC-model tables and the test seeds (single authority — SETUP reads NPCBASE, so test seeds can't drift from the game seeds). Cash math: OS1 100+3×40=220, OB1 100−5×2=90 — match assertions. Price moves: OS1 40−3=37, OB1 2+5=7, WT1 22−3=19, drift 20→<20 — match.

**Note for Plan 2B (TRADE UI):** add the `T` key (`$D4`) to `KFARM` → `TRENTER` (new `SCREEN=4`); paint NPC rows + prices read from `$0B00`/`$0500` (paged like inventory: crop-purchasers / seed-vendors / goods-purchasers); pick NPC+item+qty → `SENDCMD OPTSELL/OPTBUY` or `WSENDCMD WOPTSELL`; repaint on the `EVPRICE` event (DRAIN/EVDISP already fires it each market tick — NPC drift piggybacks it). No new CVER bump (2A already seeded the regions).
