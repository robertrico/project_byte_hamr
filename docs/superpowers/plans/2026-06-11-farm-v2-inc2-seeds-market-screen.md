# Farm v2 Increment 2: Seeds + Screen Manager + Market Screen — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Four crop varieties with distinct cost/growth/price profiles, a full-text MARKET screen for buying/selling (with quantity prompts), and the //e screen-manager pattern that increment 3's workshop screen will reuse.

**Architecture:** Coproc-first TDD: the farm bank moves to the v2 map (per-crop market array, relocated cash/inventory arrays), the plot byte becomes `crop*8 + stage` (crop 0 = wheat keeps v1 byte values, so most tb asserts survive), growth speed becomes a per-crop mask over a tick counter, DOMKT loops 4 crops and EVPRICE carries (crop, price). Then the //e: state arrays + crop-aware rendering on the farm screen, then a TEXT-mode market screen behind a tiny screen manager (M enters, ESC returns), with the inc-1 qty prompt generalized to buy/sell.

**Tech Stack:** Merlin32 6502 (single-space format, ASCII, ≤50-char lines), Icarus Verilog tb (-g2005), `make farmtasksim` / `make farm` / `make sdmdisk`, gates: `+farmonly` (~2 min) and full suite (~9 min).

**Spec:** `docs/superpowers/specs/2026-06-11-farm-v2-sidework-design.md`. Read the
"Per-game coproc RAM ownership", "Data placement rules", and "Seeds" sections first.

**Deploy reminder (tell the user):** blob changes — bench needs Ctrl-Reset or power-cycle so the respawn path reloads it. World resets too: the bank map moved, old worlds are garbage (cold start required; SIG check makes this automatic only on power-cycle — after a soft reset tell the user to press C for COLDSTART if the world looks wrong).

---

## Locked design decisions (do not re-litigate in tasks)

**Crops** (id, name, seed cost, growth mask, price base):

| id | name   | SEEDC | GROWM | BASEP | role |
|----|--------|-------|-------|-------|------|
| 0  | WHEAT  | 2     | $00 (every tick)  | 8  | volume staple |
| 1  | CARROT | 4     | $01 (every 2nd)   | 14 | balanced |
| 2  | BERRY  | 7     | $03 (every 4th)   | 24 | margin |
| 3  | PUMPKN | 12    | $07 (every 8th)   | 40 | slow jackpot |

- Profiles live as **blob tables** (12 bytes): growth masks are the per-plot hot
  loop, the spec's data-placement rule explicitly allows blob residency for
  hot-loop constants. SDRAM-content + CVER machinery lands in increment 3 with
  the recipe table. State (prices, supplies, inventories) stays in SDRAM.
- Plot byte = `crop*8 + stage`; stage = byte AND 7 (0 empty [byte must be $00],
  1-5 grow, 6 ripe, 7 reserved dead); crop = byte>>3. Harvest/clear writes $00.
- Growth: GTICK counter increments once per DOGROW pass; crop c advances only
  when `(GTICK AND GROWM[c]) == 0`. Hardware GROWD halves ($30→$18 hi byte) so
  wheat keeps ~30 s/stage and pumpkin lands ~4 min/stage (20 min to ripe — the
  long-tail crop; M5 retune still owns final numbers at increment 3).
- EVPRICE payload: **P0 = crop id, P1 = price lo** (prices capped <256 by
  BASEP ≤ 40). Emitted only when a price actually steps (v1 behavior kept).
- Mailbox (spec order): OPPLANT(x, y, crop) TA0/TA1/TA2; OPSELL(crop, qty)
  TA0/TA1; OPBUY(crop, qty) TA0/TA1. crop > 3 → RERRBAD. qty = 0 → RERRBAD.
- Farm bank v2 map (replaces $0210-$0216 scalars):

| Addr        | Content                                    |
|-------------|--------------------------------------------|
| $0210+c*3   | crop c: PRICE lo, PRICE hi, SUPPLY         |
| $0220-$0221 | FCASH lo/hi                                |
| $0222+c     | FSEEDS[c]                                  |
| $0226+c     | FCROPS[c]                                  |

- //e farm screen rows: 20 = `SEED <name> nn` (selected crop) cols 0-28 + CASH
  col 29+; 21 = messages (CLRMSG/prompts/OK/ERR move here); 22 = blank (news,
  inc 4); 23 = legend `PLANT HARVEST MARKET QUIT` + `1-4 SEED` (inverse keys).
- Crop selection: keys 1-4 on EITHER screen set SELCROP. P plants SELCROP.
  Market screen marks SELCROP's row with `*`; B/S there prompt qty for SELCROP.
- Market screen = full TEXT mode (STA $C051 on enter; STA $C050 on exit — LORES/
  MIXED soft-switch latches persist, GRON already set them). Full repaint both
  directions; farm repaint = existing DRAWALL/HUDDRAW/CURSDRAW.
- Ripe GR color per crop (PLOTDRAW takes the raw plot byte now): growing stages
  keep the v1 STAGEC progression; ripe = crop color: wheat $D yellow, carrot $9
  orange, berry $3 purple, pumpkin $C light green. ($1 magenta reserved dead.)
- COLDSTART inventory: cash 100, seeds 5/0/0/0 (only wheat — buying variety is
  the market screen's first job), crops 0, prices = BASEP, supplies 0.
- FARMTASK_MAXLEN raises 1536 → 1792 ($0600-$0CFF; reclaims the Conway scratch
  page per the spec's per-game RAM ownership rule). Workshop scratch later goes
  to $0D00 — nothing to do for that here.

## File map

- `software/SDM/FARMEQU.S` — bank map equates v2, crop count, EVPRICE comment.
- `software/SDM/FARMTASK.S` — encoding, tables, GTICK, 4-crop DOMKT, crop-arg ops.
- `Makefile` — FARMTASK_MAXLEN 1792 (two size-guard messages mention $0C00 → $0D00).
- `gateware/rev2/project_obscurus/project_obscurus_tb.v` — farm_init + asserts v2.
- `software/SDM/FARM.S` — state arrays, RDMKT, PLOTDRAW, EVDISP, HUD v2, screen
  manager, market screen, generalized qty prompt.
- `software/SDM/HANDOFF_FARM_V2.md` — status append (Task 6).

---

### Task 1: tb v2 farm section (red)

**Files:**
- Modify: `gateware/rev2/project_obscurus/project_obscurus_tb.v` (farm_init ~line 342; farm_m1/farm_econ/lap/reset asserts ~lines 1240-1470)

The current farm tb section asserts the v1 bank map and 1-arg sell/buy. Rewrite
to v2. Crop 0 keeps v1 plot byte values, so grid asserts mostly survive; the
moved scalars and new op signatures do not.

- [ ] **Step 1: Replace farm_init**

Replace the body of `task farm_init;` (keep SIG/SEQCTR/HEAD/MFLAG lines) so the
market/inventory init becomes:

```verilog
    task farm_init;
        integer i;
    begin
        sdram_write(10'd32, 16'h0000, 8'h46);          // SIG 'F'
        sdram_write(10'd32, 16'h0001, 8'h4D);          // SIG 'M'
        sdram_write(10'd32, 16'h0002, 8'h00);          // SEQCTR
        sdram_write(10'd32, 16'h0003, 8'h00);          // HEAD
        sdram_write(10'd32, 16'h0200, 8'h00);          // MFLAG
        // v2 market array: 4 x (PRICE lo, PRICE hi, SUPPLY) at $0210+c*3
        sdram_write(10'd32, 16'h0210, 8'd8);  sdram_write(10'd32, 16'h0211, 8'h00); sdram_write(10'd32, 16'h0212, 8'h00);
        sdram_write(10'd32, 16'h0213, 8'd14); sdram_write(10'd32, 16'h0214, 8'h00); sdram_write(10'd32, 16'h0215, 8'h00);
        sdram_write(10'd32, 16'h0216, 8'd24); sdram_write(10'd32, 16'h0217, 8'h00); sdram_write(10'd32, 16'h0218, 8'h00);
        sdram_write(10'd32, 16'h0219, 8'd40); sdram_write(10'd32, 16'h021A, 8'h00); sdram_write(10'd32, 16'h021B, 8'h00);
        sdram_write(10'd32, 16'h0220, 8'd100);         // CASH lo
        sdram_write(10'd32, 16'h0221, 8'h00);          // CASH hi
        sdram_write(10'd32, 16'h0222, 8'd5);           // SEEDS wheat
        sdram_write(10'd32, 16'h0223, 8'd3);           // SEEDS carrot
        sdram_write(10'd32, 16'h0224, 8'd2);           // SEEDS berry
        sdram_write(10'd32, 16'h0225, 8'd1);           // SEEDS pumpkin
        sdram_write(10'd32, 16'h0226, 8'h00);          // CROPS wheat
        sdram_write(10'd32, 16'h0227, 8'h00);
        sdram_write(10'd32, 16'h0228, 8'h00);
        sdram_write(10'd32, 16'h0229, 8'h00);
        for (i=0; i<400; i=i+1) sdram_write(10'd32, 16'h0300+i, 8'h00);
    end endtask
```

- [ ] **Step 2: Update farm_m1 asserts**

In `begin : farm_m1`, apply these exact changes:

(a) PLANT gains the crop arg (crop 0 here — byte values unchanged):
```verilog
        // (b) PLANT 3,3 crop0 -> plot $033F = 1, wheat SEEDS 4 @ $0222
        farm_cmd(8'h01, 8'd3, 8'd3, 8'h00, r, ok);
        if (!ok || r!==8'h01) begin errors=errors+1; $display("FAIL farm PLANT r=%h", r); end
        sdram_read(10'd32, 16'h033F, r);
        if (r!==8'h01) begin errors=errors+1; $display("FAIL plot(3,3)=%h want 01", r); end
        sdram_read(10'd32, 16'h0222, r);
        if (r!==8'h04) begin errors=errors+1; $display("FAIL SEEDS=%h want 04", r); end
```
(the existing `16'h0215` SEEDS read becomes `16'h0222` as shown; the corner
PLANT at 19,19 keeps `8'h00` as its third arg already — verify it reads back
`8'h01` at $048F unchanged).

(b) After the corner plant, add a pumpkin plant + crop-arg validation:
```verilog
        // v2: PLANT 5,5 crop3 (pumpkin) -> plot = 3*8+1 = $19, SEEDS[3] 0
        farm_cmd(8'h01, 8'd5, 8'd5, 8'd3, r, ok);
        if (!ok || r!==8'h01) begin errors=errors+1; $display("FAIL plant pumpkin r=%h", r); end
        sdram_read(10'd32, 16'h0369, r);   // $0300 + 5*20 + 5 = $0369
        if (r!==8'h19) begin errors=errors+1; $display("FAIL plot(5,5)=%h want 19", r); end
        sdram_read(10'd32, 16'h0225, r);
        if (r!==8'h00) begin errors=errors+1; $display("FAIL SEEDS[3]=%h want 00", r); end
        // crop>3 rejected
        farm_cmd(8'h01, 8'd7, 8'd7, 8'd4, r, ok);
        if (r!==8'hE6) begin errors=errors+1; $display("FAIL plant crop4 r=%h want E6", r); end
```

(c) SELL/BUY zero-qty asserts become (crop, qty) order:
```verilog
        // (d cont.) SELL qty=0 / BUYSEED qty=0 -> ERR_BAD (args: crop, qty)
        farm_cmd(8'h03, 8'd0, 8'd0, 8'h00, r, ok);
        if (r!==8'hE6) begin errors=errors+1; $display("FAIL sell-0 r=%h want E6", r); end
        farm_cmd(8'h04, 8'd0, 8'd0, 8'h00, r, ok);
        if (r!==8'hE6) begin errors=errors+1; $display("FAIL buy-0 r=%h want E6", r); end
```

(d) After the ripen wait on (19,19) (unchanged — wheat, value 6), add the
divergent-growth assert:
```verilog
        // v2 divergence: pumpkin (mask $07) must NOT be ripe when wheat is;
        // it advances at most stage 2 by now
        sdram_read(10'd32, 16'h0369, r);
        if (r < 8'h19 || r > 8'h1B) begin errors=errors+1;
            $display("FAIL pumpkin diverge plot=%h want 19-1B", r); end
        else $display("PASS seeds diverge: wheat ripe, pumpkin=%h", r);
```

(e) Harvest crop count read moves $0216 → $0226 (wheat):
```verilog
        sdram_read(10'd32, 16'h0226, r);
        if (r > 8'h03) begin errors=errors+1; $display("FAIL CROPS=%h want 0-3 (LFSR yield)", r); end
```

- [ ] **Step 3: Update farm_econ asserts**

In `begin : farm_econ` replace address/arg uses, preserving the yield-aware
logic. Full replacement of the changed lines:

```verilog
        sdram_read(10'd32, 16'h0226, r); nc = r;
        if (nc > 3) begin errors=errors+1; $display("FAIL pre-sell CROPS=%0d want 0-3", nc); end
        want = 100;
        if (nc > 0) begin
            farm_cmd(8'h03, 8'd0, nc[7:0], 8'h00, r, ok);   // SELL crop0 qty nc
            if (r!==8'h01) begin errors=errors+1; $display("FAIL SELL r=%h", r); end
            want = 100 + nc*8;                              // wheat base price 8
            sdram_read(10'd32, 16'h0220, r); sdram_read(10'd32, 16'h0221, r2);
            if ({r2,r}!==want[15:0]) begin errors=errors+1; $display("FAIL CASH=%d want %0d", {r2,r}, want); end
            sdram_read(10'd32, 16'h0212, r);                // wheat SUPPLY
            if (r!==nc[7:0]) begin errors=errors+1; $display("FAIL SUPPLY=%h want %0d", r, nc); end
        end else $display("note: death roll at m1 harvest, sell leg skipped");
        // SELL with no crops -> E5
        farm_cmd(8'h03, 8'd0, 8'd1, 8'h00, r, ok);
        if (r!==8'hE5) begin errors=errors+1; $display("FAIL no-crops r=%h want E5", r); end
        // BUYSEED carrot x2 @ cost 4 -> CASH want-8, SEEDS[1] 5
        farm_cmd(8'h04, 8'd1, 8'd2, 8'h00, r, ok);
        if (r!==8'h01) begin errors=errors+1; $display("FAIL BUYSEED r=%h", r); end
        want = want - 8;
        sdram_read(10'd32, 16'h0220, r);
        if (r!==want[7:0]) begin errors=errors+1; $display("FAIL post-buy CASH=%h want %0d", r, want[7:0]); end
        sdram_read(10'd32, 16'h0223, r);
        if (r!==8'h05) begin errors=errors+1; $display("FAIL SEEDS[1]=%h want 05", r); end
```

Then in the price-walk block that follows (it sells repeatedly and watches
EV_PRICE): every `farm_cmd(8'h03, X, ...)` gains crop-first form
`farm_cmd(8'h03, 8'd0, X, ...)`, and the EV_PRICE payload assert changes —
events now carry (crop, price): where the drain loop checks `ev_type[i]==8'h02`,
assert `ev_p0[i]===8'h00` (crop 0) and treat `ev_p1[i]` as the price byte
(previously `ev_p0` was the price). Read the existing block and swap p0/p1
roles accordingly, adding the crop check.

- [ ] **Step 4: lap + reset phases**

These plant/harvest crop 0 only — plot byte values unchanged. Verify by grep
that any FSEEDS/FCROPS/FCASH/FPRICE addresses in those phases move to the v2
map ($0222/$0226/$0220-1/$0210), and any `farm_cmd(8'h03/8'h04, qty, ...)`
single-arg forms gain the leading crop-0 arg. The reset phase's grid assert
(`$03A9 == 8'h06`, ripe wheat) is correct as-is.

- [ ] **Step 5: Run, verify red**

```bash
cd /Users/hambook/Development/project_byte_hamr
make DESIGN=project_obscurus REV=rev2 sim PLUSARGS=+farmonly
```
Expected: multiple `FAIL` lines in farm phases (old blob writes old map; new
asserts read new map). farm_beat and pre-farm boot must still pass. If the sim
crashes outright (vvp error), fix the verilog; red means failing asserts, not
broken testbench.

- [ ] **Step 6: Commit**

```bash
git add gateware/rev2/project_obscurus/project_obscurus_tb.v
git commit -m "test(farm): v2 seeds tb - new bank map, crop args, EVPRICE(crop,price) (red)"
```

---

### Task 2: FARMTASK v2 (green)

**Files:**
- Modify: `software/SDM/FARMEQU.S`
- Modify: `software/SDM/FARMTASK.S`
- Modify: `Makefile` (FARMTASK_MAXLEN := 1536 → 1792, ~line 543, and the two
  guard messages "code crosses \$0C00" → "\$0D00"; also the comment block at
  ~line 540)

- [ ] **Step 1: FARMEQU.S v2 map**

Replace the market/inventory equates (lines from `FPRICEL = $0210` through
`FCROPS = $0216`) and the layout header comment lines 3-8. New content:

```
* GBANK LAYOUT (SDRAM BANK 32) v2:
*  $0000 SIG(2) $0002 SEQCTR $0003 HEAD
*  $0004 HBEAT
*  $0100 RING 64x4 (PAGE-ALIGNED: REC ADDR
*   HI CONST $01, LO = IDX*4 - NO CARRY)
*  $0200 MAILBOX(6)
*  $0210+c*3 MARKET: PRICEL PRICEH SUPPLY
*  $0220 CASH(2) $0222 SEEDS(4) $0226 CROPS(4)
*  $0300 GRID 20x20 (BYTE = CROP*8 + STAGE)
```
and equates:
```
FMKT = $0210
FCASHL = $0220
FCASHH = $0221
FSEEDS = $0222
FCROPS = $0226
```
(delete FPRICEL/FPRICEH/FSUPPLY). Also: change `* PLOT STAGES: 0 EMPTY 1-5 GROW
6 RIPE` comment to `* PLOT BYTE = CROP*8+STAGE: 0 EMPTY 1-5`/`* GROW 6 RIPE (7
RESERVED DEAD)`, add `NCROPS = 4`, and change the hardware divider
`GROWD2 = $30` to `GROWD2 = $18` (halved; per-crop masks stretch the slow
crops). EVPRICE comment: add `* EVPRICE: P0=CROP P1=PRICE LO`.

- [ ] **Step 2: FARMTASK.S scratch + tables**

Add scratch after `HBV = $0E2D`:
```
GTICK = $0E2E
CROPI = $0E2F
MKAL = $0E30
MKAH = $0E31
```
Add tables at the end of the file (before any final include, after DOMKT's
last routine):
```
* per-crop profiles (blob-resident: hot loop)
GROWM DFB $00,$01,$03,$07
BASEP DFB 8,14,24,40
SEEDC DFB 2,4,7,12
```
Delete the old `SEEDCOST = 3` and `PBASE = 10` uses (PFLOOR stays; see DOMKT).
In FARMEQU.S keep `PFLOOR = 2`, delete `PBASE` and `SEEDCOST`, keep `CASH0/SEEDS0`.

- [ ] **Step 3: MKADR helper + GAME init**

Add helper (near PLOTADR):
```
* === MKADR: A=crop -> MKAL/H = FMKT+crop*3 ===
MKADR
 STA CROPI
 ASL
 CLC
 ADC CROPI
 CLC
 ADC #<FMKT
 STA MKAL
 LDA #0
 ADC #>FMKT
 STA MKAH
 RTS
```
In GAME init (after the LFSR seed), add `LDA #0` / `STA GTICK`.

- [ ] **Step 4: CPLANT with crop arg**

Replace CPLANT:
```
* --- PLANT TA0=x TA1=y TA2=crop ---
CPLANT
 LDA TA2
 CMP #NCROPS
 BCC PLCROK
 LDA #RERRBAD
 JMP MBFIN
PLCROK
 JSR PLOTADR
 JSR RDPLOT
 BEQ PLOK
 LDA #RERROCC
 JMP MBFIN
PLOK
 LDA TA2
 CLC
 ADC #<FSEEDS
 STA PLOTV
 LDA #0
 ADC #>FSEEDS
 STA PX
 LDA PLOTV
 LDY PX
 JSR RDB
 BNE PLSEED
 LDA #RERRSEED
 JMP MBFIN
PLSEED
 SEC
 SBC #1
 TAX
 LDA PLOTV
 LDY PX
 JSR WRB
* plot byte = crop*8 + 1
 LDA TA2
 ASL
 ASL
 ASL
 CLC
 ADC #1
 TAX
 JSR WRPLOT
 LDA #ROK
 JMP MBFIN
```
(PLOTV/PX reused as addr scratch here — they are free until DOGROW runs, and
DOMBOX completes before DOGROW within a pass.)

- [ ] **Step 5: CHARV crop-aware**

In CHARV, the ripe test and the crop-credit change. Replace the head:
```
* --- HARVEST TA0=x TA1=y ---
CHARV
 JSR PLOTADR
 JSR RDPLOT
 STA PLOTV
 AND #7
 CMP #STRIPE
 BEQ HVOK
 LDA #RERRRIPE
 JMP MBFIN
HVOK
* crop id -> CROPI, FCROPS+crop -> M20L/H
 LDA PLOTV
 LSR
 LSR
 LSR
 STA CROPI
 CLC
 ADC #<FCROPS
 STA M20L
 LDA #0
 ADC #>FCROPS
 STA M20H
 LDA M20L
 LDY M20H
 JSR RDB
 CMP #$FF
 BNE HVROOM
 LDA #RERRFULL
 JMP MBFIN
```
and in the yield-credit code after HVYSET, replace both `LDA #<FCROPS` /
`LDY #>FCROPS` pairs with `LDA M20L` / `LDY M20H` (read and write the per-crop
slot). HVCLR (write 0 to plot) is unchanged.

- [ ] **Step 6: CSELL(crop, qty)**

Replace CSELL head and price/supply addressing:
```
* --- SELL TA0=crop TA1=qty ---
CSELL
 LDA TA0
 CMP #NCROPS
 BCC SELCROK
 LDA #RERRBAD
 JMP MBFIN
SELCROK
 LDA TA1
 BNE CSELQ
 LDA #RERRBAD
 JMP MBFIN
CSELQ
 LDA TA0
 CLC
 ADC #<FCROPS
 STA M20L
 LDA #0
 ADC #>FCROPS
 STA M20H
 LDA M20L
 LDY M20H
 JSR RDB
 CMP TA1
 BCS SLOK
 LDA #RERRCROP
 JMP MBFIN
SLOK
 SEC
 SBC TA1
 TAX
 LDA M20L
 LDY M20H
 JSR WRB
* cash += qty*price[crop] (loop), clamp $FFFF
 LDA TA0
 JSR MKADR
 LDA MKAL
 LDY MKAH
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
 LDA TA1
 STA QTY
```
The SLMUL accumulate/clamp loop and cash write-back are unchanged. The supply
update replaces FSUPPLY addressing:
```
* supply[crop] += qty clamp 255 (MKAL+2)
 LDA MKAL
 CLC
 ADC #2
 STA MKAL
 LDA MKAL
 LDY MKAH
 JSR RDB
 CLC
 ADC TA1
 BCC SPNC
 LDA #$FF
SPNC
 TAX
 LDA MKAL
 LDY MKAH
 JSR WRB
 LDA #ROK
 JMP MBFIN
```

- [ ] **Step 7: CBUY(crop, qty)**

Replace CBUY head + cost loop constant + seeds addressing:
```
* --- BUYSEED TA0=crop TA1=qty ---
CBUY
 LDA TA0
 CMP #NCROPS
 BCC BYCROK
 LDA #RERRBAD
 JMP MBFIN
BYCROK
 LDA TA1
 BNE CBUYQ
 LDA #RERRBAD
 JMP MBFIN
CBUYQ
* seeds slot addr -> PADRL/H (free here)
 LDA TA0
 CLC
 ADC #<FSEEDS
 STA PADRL
 LDA #0
 ADC #>FSEEDS
 STA PADRH
 LDA PADRL
 LDY PADRH
 JSR RDB
 STA SEED
 CLC
 ADC TA1
 BCC BYROOM
 LDA #RERRFULL
 JMP MBFIN
BYROOM
* cost = qty * SEEDC[crop] -> M20L/H
 LDX TA0
 LDA SEEDC,X
 STA PRC
 LDA #0
 STA M20L
 STA M20H
 LDA TA1
 STA QTY
BYCST
 LDA M20L
 CLC
 ADC PRC
 STA M20L
 LDA M20H
 ADC #0
 STA M20H
 DEC QTY
 BNE BYCST
```
The cash compare/subtract block is unchanged. The final seeds write becomes:
```
 LDA SEED
 CLC
 ADC TA1
 TAX
 LDA PADRL
 LDY PADRH
 JSR WRB
 LDA #ROK
 JMP MBFIN
```

- [ ] **Step 8: DOGROW with masks**

Replace the per-plot advance logic:
```
* === DOGROW: advance plots due this tick ===
* GTICK++ each call; crop advances when
* (GTICK AND GROWM[crop]) == 0
DOGROW
 INC GTICK
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
 AND #7
 CMP #STRIPE
 BCS GRNEXT
* crop due this tick?
 LDA PLOTV
 LSR
 LSR
 LSR
 TAX
 LDA GROWM,X
 AND GTICK
 BNE GRNEXT
 LDA PLOTV
 CLC
 ADC #1
 STA PLOTV
 TAX
 LDA PADRL
 LDY PADRH
 JSR WRB
 LDA PLOTV
 AND #7
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
```
(loop tail INC PADRL/PX/PY unchanged.) Note `BCS GRNEXT` also skips stage 7
(reserved dead) for free.

- [ ] **Step 9: DOMKT 4-crop loop**

Replace DOMKT entirely:
```
* === DOMKT: per-crop price step + decay ===
DOMKT
 LDA #0
 STA CROPI
MKCROP
 LDA CROPI
 JSR MKADR
* read SUPPLY (MKAL+2) then PRICE (MKAL)
 LDA MKAL
 CLC
 ADC #2
 LDY MKAH
 JSR RDB
 STA SUP
 LDA MKAL
 LDY MKAH
 JSR RDB
 STA PRC
* TGT = BASEP[c] - SUP/2, floor PFLOOR
 LDX CROPI
 LDA BASEP,X
 STA TGT
 LDA SUP
 LSR
 STA PX
 LDA TGT
 SEC
 SBC PX
 BCC MKFLOOR
 CMP #PFLOOR
 BCS MKTGT
MKFLOOR
 LDA #PFLOOR
MKTGT
 STA TGT
* step PRICE 1 toward TGT, event on change
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
 LDA MKAL
 LDY MKAH
 JSR WRB
* publish EV_PRICE crop, price lo
 LDA #EVPRICE
 STA EVTYPE
 LDA CROPI
 STA EVP0
 LDA PRC
 STA EVP1
 JSR PUTEV
MKDECAY
 INC CROPI
 LDA CROPI
 CMP #NCROPS
 BNE MKCROP
* supply decay: every DECAYDIV ticks, all crops
 INC DECAYC
 LDA DECAYC
 CMP #DECAYDIV
 BNE MKDONE
 LDA #0
 STA DECAYC
 STA CROPI
MKDCRP
 LDA CROPI
 JSR MKADR
 LDA MKAL
 CLC
 ADC #2
 STA MKAL
 LDA MKAL
 LDY MKAH
 JSR RDB
 BEQ MKDNXT
 SEC
 SBC #1
 TAX
 LDA MKAL
 LDY MKAH
 JSR WRB
MKDNXT
 INC CROPI
 LDA CROPI
 CMP #NCROPS
 BNE MKDCRP
MKDONE
 RTS
```
Note `MKADR` clobbers CROPI? No — it STOREs A into CROPI (same value the caller
passed from CROPI). Safe. PX reused as halved-supply temp inside DOMKT (DOGROW
not active concurrently — single task, sequential calls).

- [ ] **Step 10: Makefile cap**

`FARMTASK_MAXLEN := 1536` → `1792`. Update both guard echo strings and the
comment block above (now: code must end below $0D00 = 1792 bytes; $0D00+ is
workshop scratch per spec).

- [ ] **Step 11: Build + farmonly green**

```bash
make farmtasksim
make DESIGN=project_obscurus REV=rev2 sim PLUSARGS=+farmonly
```
Expected: blob well under 1792 (report the size); ALL farm phases PASS
including the new diverge assert. Iterate here until green — this is the
core of the increment.

- [ ] **Step 12: Commit**

```bash
git add software/SDM/FARMEQU.S software/SDM/FARMTASK.S Makefile gateware/rev2/project_obscurus/farmtask.mem
git commit -m "feat(farm): seeds v2 - 4 crops, mask growth, per-crop market, crop-arg ops"
```

---

### Task 3: Full-suite gate

- [ ] **Step 1:**
```bash
make DESIGN=project_obscurus REV=rev2 sim
```
(~9 min, timeout 600000.) Expected: zero FAIL across all phases. The farm
phases are the only ones touching bank 32 — any non-farm failure means a real
regression: STOP and report.

- [ ] **Step 2: Commit nothing (gate only).** If red, fix before proceeding.

---

### Task 4: FARM.S state layer + crop-aware farm screen

**Files:**
- Modify: `software/SDM/FARM.S`

Farm screen becomes fully 4-crop (select with 1-4, plant selected, per-crop
ripe colors, HUD v2 rows) while still single-screen — the market screen lands
in Task 5. After this task the game must assemble and be bench-playable.

- [ ] **Step 1: State variables**

Replace `SEEDS DS 1` / `CROPS DS 1` / `PRICE DS 1` / `LASTPR DS 1` with:
```
SEED4 DS 4
CROP4 DS 4
PRICE4 DS 4
LASTP4 DS 4
SELCROP DS 1
SCREEN DS 1
```
Grep every use of the old names and fix per the steps below (SEEDS/CROPS/PRICE/
LASTPR appear in RDMKT, HUDDRAW, EVDISP, DCOK/DCMKT region).

- [ ] **Step 2: RDMKT v2**

Replace RDMKT body: cash tear-guard loop reads $0220/$0221 (FCASHL/FCASHH — the
equates moved in Task 2, FARM.S PUTs FARMEQU so names are already correct);
then read 4 prices and inventories (c*3 computed as c+c<<1):
```
 LDX #0
RMPRC
 TXA
 STA NTMP
 ASL
 CLC
 ADC NTMP
 CLC
 ADC #<FMKT
 LDY #>FMKT
 JSR FRD
 LDA SDM_VAL
 STA PRICE4,X
 INX
 CPX #4
 BNE RMPRC
 LDX #0
RMSEED
 TXA
 CLC
 ADC #<FSEEDS
 LDY #>FSEEDS
 JSR FRD
 LDA SDM_VAL
 STA SEED4,X
 INX
 CPX #4
 BNE RMSEED
 LDX #0
RMCROP
 TXA
 CLC
 ADC #<FCROPS
 LDY #>FCROPS
 JSR FRD
 LDA SDM_VAL
 STA CROP4,X
 INX
 CPX #4
 BNE RMCROP
 RTS
```
(FRD takes addr lo in A, hi in Y; `>FMKT`/`>FSEEDS`/`>FCROPS` are all $02 and
the lo-byte adds never carry: $10+9, $22+3, $26+3.) Delete the old
single-PRICE/SEEDS/CROPS reads. Also copy PRICE4 into LASTP4 at the end of
RDMKT (fresh resync = flat trend):
```
 LDX #0
RMLP4
 LDA PRICE4,X
 STA LASTP4,X
 INX
 CPX #4
 BNE RMLP4
 RTS
```

- [ ] **Step 3: PLOTDRAW takes the raw plot byte**

Replace the STAGEC lookup head of PLOTDRAW:
```
* crop ripe colors: wheat carrot berry pumpkin
CROPC DFB $0D,$09,$03,$0C
* === PLOTDRAW: X=px Y=py A=plot byte ->
* 2x2 block; stage<6 STAGEC, ripe CROPC[crop]
PLOTDRAW
 STX PLOTC
 STA NTMP
 AND #7
 CMP #STRIPE
 BEQ PDRIPE
 TAX
 LDA STAGEC,X
 JMP PDCOL
PDRIPE
 LDA NTMP
 LSR
 LSR
 LSR
 TAX
 LDA CROPC,X
PDCOL
 STA T16
```
(the rest — nibble doubling and screen write — unchanged.) STAGEC stays 7
entries; entry 6 is now unused by PLOTDRAW but harmless. ALL existing callers
already pass the raw shadow/plot byte except the DCPAINT/EVDISP paths that pass
literal stages — fix those in Step 5.

- [ ] **Step 4: HUD v2 (rows 20/21/23)**

- CLRMSG: row constant 20 → 21 (`LDX #20` becomes `LDX #21`; comment: row 21
  cols 0-28; CASH still owns row 20 cols 29+ — clearing row 21 fully is fine,
  change the loop bound to clear all 40 cols: `CPY #40`... read the loop; it
  clears cols 0-28 via `CPY #29`; make it `CPY #40`).
- Every `LDY #20` before a `JSR PRSTR` that prints a MESSAGE (SRIPE in EVDISP,
  SQTY in SELLQTY, SNORESP/SERR/SOK/SLOST/SDEAD/SRESP in the DOCMD result and
  probe paths, the yield digit `(LINEP),Y` writes in DCMKT) becomes `LDY #21`.
  The CASH block in HUDDRAW keeps row 20.
- HUDDRAW v2: delete the BUY/SELL/trend row-21 block and the SEED/CROP row-22
  block. New row 20 cols 0-28: `SEED <name> nn` for SELCROP:
```
* row 20: SEED label col 0, name col 5,
* count col 12; CASH col 29 (unchanged)
 LDA #<SSEED
 STA MSGPTR
 LDA #>SSEED
 STA MSGPTR+1
 LDA #0
 STA PRCOL
 LDY #20
 JSR PRSTR
 LDA SELCROP
 ASL
 ASL
 ASL
 TAX
 LDA #<SNAMES
 CLC
 ADC NMOFF
 STA MSGPTR
 LDA #>SNAMES
 ADC #0
 STA MSGPTR+1
```
Simpler and preferred — name table as 4 separate 0-terminated strings with an
address table:
```
SNAMW ASC "WHEAT"
 DFB 0
SNAMC ASC "CARROT"
 DFB 0
SNAMB ASC "BERRY"
 DFB 0
SNAMP ASC "PUMPKN"
 DFB 0
SNAML DFB <SNAMW,<SNAMC,<SNAMB,<SNAMP
SNAMH DFB >SNAMW,>SNAMC,>SNAMB,>SNAMP
```
then HUDDRAW row 20:
```
 LDX SELCROP
 LDA SNAML,X
 STA MSGPTR
 LDA SNAMH,X
 STA MSGPTR+1
 LDA #5
 STA PRCOL
 LDY #20
 JSR PRSTR
 LDX SELCROP
 LDA SEED4,X
 STA T16
 LDA #0
 STA T16+1
 LDX #12
 LDY #20
 JSR PRDEC
```
(keep the existing CASH label/value block, row 20 cols 29/34, as-is). Clear
row 20 cols 0-28 first (the old CLRMSG loop logic, inline or as a new CLRROW20
helper copied from CLRMSG with `LDX #20` + `CPY #29`).
- Legend SHELP v2:
```
SHELP DFB $10
 ASC "LANT "
 DFB $08
 ASC "ARVEST "
 DFB $0D
 ASC "ARKET "
 DFB $11
 ASC "UIT "
 ASC "1-4 SEED"
 DFB 0
```

- [ ] **Step 5: EVDISP + DCPAINT crop-aware**

- EVRIPE path: the plot's crop is in SHADOW. Before `LDA #STRIPE` /
  `JSR PLOTDRAW` + `JSR SHADSET`, compute the ripe byte:
```
* ripe byte = (shadow AND $F8) OR STRIPE
 JSR SHADGET
 AND #$F8
 ORA #STRIPE
 PHA
 LDX EVA
 LDY EVB
 PLA
 PHA
 JSR PLOTDRAW
 PLA
 JSR SHADSET
```
  Add SHADGET — copy of SHADSET's address math that ENDS with
  `LDA (SHADP),Y` instead of the store (factor the address computation into a
  shared SHADADR subroutine ending after the SHADP setup; SHADSET = JSR SHADADR
  + store; SHADGET = JSR SHADADR + load).
- EVPRICE path: EVA=crop, EVB=price:
```
EVD2
 CMP #EVPRICE
 BNE EVDONE
 LDX EVA
 LDA PRICE4,X
 STA LASTP4,X
 LDA EVB
 STA PRICE4,X
EVDONE
 RTS
```
  (no farm-screen redraw needed — price is not on the farm HUD anymore; Task 5
  adds the market-screen hook here.)
- DCPAINT callers (DCOK paths after PLANT) pass literal stage 1; ripe paint is
  the EVRIPE path. PLANT paint becomes the crop-aware byte: where DCOK/DCMKT
  calls `LDA #1` / `JSR DCPAINT`, use:
```
 LDA SELCROP
 ASL
 ASL
 ASL
 CLC
 ADC #1
 JSR DCPAINT
```
  DCPAINT itself is unchanged (it passes A through to SHADSET+PLOTDRAW, both
  now byte-aware).

- [ ] **Step 6: PLANT sends crop; key handler 1-4**

- DOCMDXY (used by P/H): PLANT needs CMDA2=SELCROP but HARVEST needs CMDA2=0.
  Change K5 (P) to stage CMDA2 after DOCMDXY's zero:
  simplest — add a parallel entry:
```
* === DOCMDXYC: like DOCMDXY + A2=SELCROP ===
DOCMDXYC
 STA CMDOP
 LDA CURX
 STA CMDA0
 LDA CURY
 STA CMDA1
 LDA SELCROP
 STA CMDA2
 JMP DOCMD
```
  K5 calls `LDA #OPPLANT` / `JSR DOCMDXYC`. K6 (H) keeps DOCMDXY.
- Key handler: remove K7 (S) and K8 (B) blocks entirely (sell/buy move to the
  market screen in Task 5; for THIS task S/B simply fall through to KNONE).
  Add 1-4 selection before K9 (Q):
```
K8
 CMP #$B1 ; '1'
 BCC K9
 CMP #$B5 ; > '4'
 BCS K9
 SEC
 SBC #$B1
 STA SELCROP
 JSR HUDDRAW
 JMP MLOOP
```
  (keys arrive hi-bit set: '1'=$B1..'4'=$B4.)
- SELLQTY routine: keep it (Task 5 generalizes it); it is simply unreachable
  this task. COLDST/MAIN init: `LDA #0` / `STA SELCROP` / `STA SCREEN` near the
  existing var init.

- [ ] **Step 7: COLDST v2 init**

Replace the GBANK init writes for the old scalar block with the v2 map:
zeros to $0211,$0212 (wheat hi/supply), $0214,$0215, $0217,$0218, $021A,$021B,
$0221 (cash hi), $0223,$0224,$0225 (seeds 1-3), $0226-$0229 (crops);
values: $0210=8, $0213=14, $0216=24, $0219=40 (BASEP), $0220=CASH0 (100),
$0222=SEEDS0 (5). Use the existing FWR pattern; the zero-writes can share the
`LDA #0 / STA SDM_VAL` prefix with a run of FWR calls. Full code:
```
* v2 market/inventory init (SIG LAST as before)
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
 LDA #$11
 LDY #$02
 JSR FWR
 LDA #$12
 LDY #$02
 JSR FWR
 LDA #$14
 LDY #$02
 JSR FWR
 LDA #$15
 LDY #$02
 JSR FWR
 LDA #$17
 LDY #$02
 JSR FWR
 LDA #$18
 LDY #$02
 JSR FWR
 LDA #$1A
 LDY #$02
 JSR FWR
 LDA #$1B
 LDY #$02
 JSR FWR
 LDA #<FCASHH
 LDY #>FCASHH
 JSR FWR
 LDA #$23
 LDY #$02
 JSR FWR
 LDA #$24
 LDY #$02
 JSR FWR
 LDA #$25
 LDY #$02
 JSR FWR
 LDA #$26
 LDY #$02
 JSR FWR
 LDA #$27
 LDY #$02
 JSR FWR
 LDA #$28
 LDY #$02
 JSR FWR
 LDA #$29
 LDY #$02
 JSR FWR
 LDA #8
 STA SDM_VAL
 LDA #$10
 LDY #$02
 JSR FWR
 LDA #14
 STA SDM_VAL
 LDA #$13
 LDY #$02
 JSR FWR
 LDA #24
 STA SDM_VAL
 LDA #$16
 LDY #$02
 JSR FWR
 LDA #40
 STA SDM_VAL
 LDA #$19
 LDY #$02
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
```
(grid zero loop + SPAWNT + SIG-last unchanged.)

- [ ] **Step 8: Assemble + commit**

```bash
make farm
```
Expected: clean. Then:
```bash
git add software/SDM/FARM.S software/SDM/FARMTASKB.S
git commit -m "feat(farm): //e 4-crop farm screen - select 1-4, crop colors, HUD v2 rows"
```

---

### Task 5: Screen manager + MARKET screen

**Files:**
- Modify: `software/SDM/FARM.S`

- [ ] **Step 1: Generalize the qty prompt**

Rename SELLQTY → QTYPROMPT, parameterized by op. Caller sets CMDOP before the
JSR; the prompt fills CMDA0 (crop = SELCROP) and CMDA1 (qty):
- Change the routine label and header comment.
- Prompt string: replace SQTY with two strings + a pointer set by the caller:
```
SQTYS ASC "SELL QTY:"
 DFB 0
SQTYB ASC "BUY QTY:"
 DFB 0
```
- The routine's PRSTR block prints via MSGPTR which the CALLER now sets (move
  the `LDA #<SQTY...STA MSGPTR+1` lines out to the two call sites). SQGO
  becomes:
```
SQGO
 LDA QVAL
 BEQ SQCAN
 STA CMDA1
 LDA SELCROP
 STA CMDA0
 LDA #0
 STA CMDA2
 JMP DOCMD
```
  (CMDOP set by caller before JSR.) Echo column and the rest unchanged, except
  every `LDY #20` in the routine is `LDY #21` already from Task 4 Step 4.

- [ ] **Step 2: Screen manager + market keys**

Main loop dispatch: MLOOP currently polls KBD and runs farm keys. Insert a
screen check at key dispatch:
```
MNOST
 LDA KBD
 BPL MLOOP
 STA KBDSTR
 LDX SCREEN
 BEQ KFARM
 JMP MKKEY
KFARM
```
(KFARM = the existing arrow/key chain start.) Farm keys: add M:
```
KM
 CMP #$CD ; M
 BNE K9
 JSR MKENTER
 JMP MLOOP
```
(insert between the 1-4 handler and K9/Q.)

Market key handler:
```
* === MKKEY: market screen keys ===
MKKEY
 CMP #$9B ; ESC
 BNE MK1
 JSR MKEXIT
 JMP MLOOP
MK1
 CMP #$B1 ; '1'-'4' select
 BCC MK2
 CMP #$B5
 BCS MK2
 SEC
 SBC #$B1
 STA SELCROP
 JSR MKDRAW
 JMP MLOOP
MK2
 CMP #$C2 ; B buy
 BNE MK3
 LDA #OPBUY
 STA CMDOP
 LDA #<SQTYB
 STA MSGPTR
 LDA #>SQTYB
 STA MSGPTR+1
 JSR QTYPROMPT
 JSR RDMKT
 JSR MKDRAW
 JMP MLOOP
MK3
 CMP #$D3 ; S sell
 BNE MKNONE
 LDA #OPSELL
 STA CMDOP
 LDA #<SQTYS
 STA MSGPTR
 LDA #>SQTYS
 STA MSGPTR+1
 JSR QTYPROMPT
 JSR RDMKT
 JSR MKDRAW
 JMP MLOOP
MKNONE
 JMP MLOOP
```

- [ ] **Step 3: MKENTER / MKEXIT / MKDRAW**

```
* === MKENTER: -> text mode market screen ===
MKENTER
 LDA #1
 STA SCREEN
 STA TEXTSW
 JSR MKDRAW
 RTS
* === MKEXIT: -> GR farm screen ===
MKEXIT
 LDA #0
 STA SCREEN
 STA GRAPH
 JSR DRAWALL
 JSR HUDDRAW
 JSR CURSDRAW
 RTS
* === MKCLR: blank text rows 0-19 ===
MKCLR
 LDY #0
MKCROW
 TYA
 PHA
 LDA GRLO,Y
 STA LINEP
 LDA GRHI,Y
 STA LINEP+1
 LDA #$A0
 LDY #0
MKCCOL
 STA (LINEP),Y
 INY
 CPY #40
 BNE MKCCOL
 PLA
 TAY
 INY
 CPY #20
 BNE MKCROW
 RTS
* === MKDRAW: full market repaint ===
* row 1 title; rows 4,7,10,13 = crops:
* sel star, key digit, name, BUY c, SELL p,
* trend, SD n, CR n; row 20 cash via HUDDRAW
MKDRAW
 JSR MKCLR
 LDA #<SMKT
 STA MSGPTR
 LDA #>SMKT
 STA MSGPTR+1
 LDA #15
 STA PRCOL
 LDY #1
 JSR PRSTR
 LDA #0
 STA NTMP
MKROW
* Y row = 4 + crop*3
 LDA NTMP
 ASL
 CLC
 ADC NTMP
 CLC
 ADC #4
 TAY
 STY EVT
* selection star col 0
 LDA GRLO,Y
 STA LINEP
 LDA GRHI,Y
 STA LINEP+1
 LDX NTMP
 CPX SELCROP
 BNE MKNOSEL
 LDA #$AA ; '*'
 LDY #0
 STA (LINEP),Y
MKNOSEL
* key digit col 1
 LDA NTMP
 CLC
 ADC #$B1
 LDY #1
 STA (LINEP),Y
* name col 3
 LDX NTMP
 LDA SNAML,X
 STA MSGPTR
 LDA SNAMH,X
 STA MSGPTR+1
 LDA #3
 STA PRCOL
 LDY EVT
 JSR PRSTR
* BUY label col 10 + cost col 14
 LDA #<SBUY
 STA MSGPTR
 LDA #>SBUY
 STA MSGPTR+1
 LDA #10
 STA PRCOL
 LDY EVT
 JSR PRSTR
 LDX NTMP
 LDA SEEDCT,X
 STA T16
 LDA #0
 STA T16+1
 LDX #14
 LDY EVT
 JSR PRDEC
* SELL label col 18 + price col 23 + trend 27
 LDA #<SSELL
 STA MSGPTR
 LDA #>SSELL
 STA MSGPTR+1
 LDA #18
 STA PRCOL
 LDY EVT
 JSR PRSTR
 LDX NTMP
 LDA PRICE4,X
 STA T16
 LDA #0
 STA T16+1
 LDX #23
 LDY EVT
 JSR PRDEC
 LDY EVT
 LDA GRLO,Y
 STA LINEP
 LDA GRHI,Y
 STA LINEP+1
 LDX NTMP
 LDA PRICE4,X
 CMP LASTP4,X
 BEQ MKTFL
 BCC MKTDN
 LDA #$AB ; '+'
 JMP MKTPUT
MKTDN
 LDA #$AD ; '-'
 JMP MKTPUT
MKTFL
 LDA #$BD ; '='
MKTPUT
 LDY #27
 STA (LINEP),Y
* SD col 29 val 32, CR col 34 val 37
 LDA #<SSD
 STA MSGPTR
 LDA #>SSD
 STA MSGPTR+1
 LDA #29
 STA PRCOL
 LDY EVT
 JSR PRSTR
 LDX NTMP
 LDA SEED4,X
 STA T16
 LDA #0
 STA T16+1
 LDX #32
 LDY EVT
 JSR PRDEC
 LDA #<SCR
 STA MSGPTR
 LDA #>SCR
 STA MSGPTR+1
 LDA #34
 STA PRCOL
 LDY EVT
 JSR PRSTR
 LDX NTMP
 LDA CROP4,X
 STA T16
 LDA #0
 STA T16+1
 LDX #37
 LDY EVT
 JSR PRDEC
 INC NTMP
 LDA NTMP
 CMP #4
 BNE MKJROW
* legend row 23 + cash row 20
 LDA #<SMHELP
 STA MSGPTR
 LDA #>SMHELP
 STA MSGPTR+1
 LDA #0
 STA PRCOL
 LDY #23
 JSR PRSTR
 JSR HUDDRAW
 RTS
MKJROW
 JMP MKROW
```
New data (next to the other strings):
```
SMKT ASC "MARKET"
 DFB 0
SSD ASC "SD"
 DFB 0
SCR ASC "CR"
 DFB 0
SEEDCT DFB 2,4,7,12
SMHELP DFB $02
 ASC "UY "
 DFB $13
 ASC "ELL "
 ASC "1-4 SEED ESC FARM"
 DFB 0
```
(HUDDRAW prints `SEED <name>` at row 20 cols 0-28 — on the market screen that
is redundant but harmless and keeps one code path; it also prints CASH, which
is wanted.)

- [ ] **Step 4: Event hook on market screen**

In EVDISP's EVPRICE arm (Task 4 Step 5), after updating PRICE4/LASTP4, redraw
when the market screen is up:
```
 LDA SCREEN
 BEQ EVDONE
 JSR MKDRAW
```
And the EVRIPE arm's grid/cursor painting must be skipped while in text mode
(writes would land on text rows): guard the EVRIPE paint block with
`LDA SCREEN` / `BNE EVRMSG` placed right after the `CMP #EVRIPE / BNE EVD2`
dispatch — SHADOW is still updated (move the SHADGET/SHADSET portion BEFORE
the guard so state stays correct; only PLOTDRAW/CURSDRAW are skipped; MKEXIT's
DRAWALL repaints from SHADOW). The "RIPE!" message prints either way (row 21
exists on both screens).

- [ ] **Step 5: Assemble, commit**

```bash
make farm
```
Expected: clean.
```bash
git add software/SDM/FARM.S software/SDM/FARMTASKB.S
git commit -m "feat(farm): market screen - text mode, buy/sell qty prompts, M/ESC nav"
```

---

### Task 6: Disk + gate + handoff

- [ ] **Step 1:**
```bash
make farm && make sdmdisk
```
Expected: SDMTEST.po rebuilt; FARM size grows ~600-900 B (report it; ceiling
$9600 is far away).

- [ ] **Step 2: Full suite once more**
```bash
make DESIGN=project_obscurus REV=rev2 sim
```
Expected: zero FAIL (//e changes don't touch sim, this is the regression
gate for the whole increment).

- [ ] **Step 3: Handoff append**

In `software/SDM/HANDOFF_FARM_V2.md` "Where we are", after the inc-1 bullet:
```
- **v2 increment 2 SHIPPED (branch farm-v2)**: 4 crops (WHEAT/CARROT/BERRY/
  PUMPKN, plot byte = crop*8+stage, mask growth off GTICK, per-crop market
  $0210+c*3, cash $0220, SEEDS[4] $0222, CROPS[4] $0226, EVPRICE=(crop,price),
  OPPLANT/+crop OPSELL/OPBUY=(crop,qty), FARMTASK_MAXLEN now 1792) + //e
  screen manager (M=market text screen, ESC=farm, 1-4 select seed, B/S qty
  prompts on market; farm rows: 20 seed+cash, 21 messages, 23 legend).
  COLD START REQUIRED after deploy (bank map moved). Next: increment 3 =
  WORKSHOP task (read spec quiesce invariant + EVLIB scratch param + CP_FSTAT
  restore_done rule FIRST).
```

- [ ] **Step 4: Commit**
```bash
git add software/SDM/HANDOFF_FARM_V2.md
git commit -m "docs(farm): handoff - inc 2 seeds + market screen shipped"
```

- [ ] **Step 5: Bench checklist (tell the user)**

1. **Power-cycle** (bank map moved — old world is garbage; soft reset preserves
   the stale world, so full cold start it is). Boot SDMTEST.po, BRUN FARM,
   press C if offered COLDSTART.
2. Farm: 1-4 changes the SEED row-20 name; P plants the selected crop; plant
   one of each — four different growth speeds visible; ripe plots show four
   different colors (wheat yellow, carrot orange, berry purple, pumpkin green).
3. M → market screen: 4 rows w/ prices 8/14/24/40, trends, SD/CR counts,
   `*` on selected row; 1-4 moves the star.
4. B → `BUY QTY:` 2 → seeds +2, cash -2×cost, row updates. S with crops →
   `SELL QTY:` works; S with none → ERR E5.
5. ESC → farm repaints clean (grid + cursor + HUD); M → market again — prices
   drift live while you watch (EVPRICE redraws).
6. Q quits from farm; re-enter FARM → world resyncs.
7. `/obs-screenshot` both screens.

---

## Self-review notes (planning time)

- Spec coverage: seeds encoding ✓ (T2), profiles-as-data ✓ (blob tables, spec
  hot-loop allowance, CVER consciously deferred to inc 3 — recorded in Locked
  Decisions), per-crop market/drift ✓ (T2 S9), mailbox crop args ✓ (T2 S4-7),
  saturation ✓ (carried v1 clamps), EVPRICE redefinition ✓ (T2 S9 + T1 asserts
  + T4/T5 //e), 2-digit modal prompt ✓ (T5 S1, modal loop unchanged), screen
  manager + market screen + keys ✓ (T5), farm rows assignment ✓ (T4 S4),
  S removed from farm ✓ (T4 S6), render colors ✓ (T4 S3), cap raise ✓ (T2 S10),
  re-entry resync unchanged (RESYNC calls RDMKT/RDGRID/DRAWALL — all updated
  underneath) ✓. World events, workshop, CVER, news row content = later
  increments by design.
- Known judgment calls for the implementer: exact insertion points use labels
  (file drifted +178 B since the line numbers in old notes); MKDRAW column
  layout may need ±1-col nudges on bench — cosmetic, implementer freedom.
- Type consistency: SELCROP/SCREEN/PRICE4/LASTP4/SEED4/CROP4/QTYPROMPT/SQTYB/
  SQTYS/MKDRAW/MKENTER/MKEXIT/MKCLR/MKKEY names used consistently across T4/T5;
  FMKT/FCASHL/FCASHH/FSEEDS/FCROPS across T1/T2/T4.
- tb risk: the price-walk block edit (T1 S3 tail) is described, not fully
  reproduced — implementer must read the existing block; flagged as the one
  read-and-adapt step. If it proves gnarly, acceptable fallback: assert only
  that EV_PRICE events with p0==0 arrive and prices move toward floor.
