# Goods Inventory + Pantry Elimination Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Finished crafts auto-store as goods (no manual collect); crafting pulls straight from harvested crops (pantry/deposit deleted); a new paged, scrolling INVENTORY screen shows seeds/crops/goods and sells crops (market price) and goods (fixed value).

**Architecture:** WORKTASK loses the pantry (ingredient checks + WCONSUM + deposit op) and gains a GOODS array; WTICK auto-stores on cook-done and frees the station; the collect op becomes WOPSELL. The //e debits crops from the farm before each craft (debit-first), and the new INVENTORY screen is a paged scrolling viewport that scales to any item count. CVER bumps to 3 with a selective migration that preserves discovered recipes + skill.

**Tech Stack:** Merlin32 6502 (single-space format, ASCII, ≤50-char lines), iverilog -g2005 tb, `make worktask`/`farm`/`worktasksim`/`sdmdisk`, gates `+farmonly` (~2-4 min) + full suite (~10 min).

**Spec:** `docs/superpowers/specs/2026-06-16-farm-v2-goods-inventory-design.md`. READ it first.

**CRITICAL build lesson (cost a bench session):** the hardware blobs go stale silently — `make` skips recompiling a .bin if git touched its mtime, and the SIM blobs build on a separate path so sim-green ≠ hardware-current. After ANY WORKTASK.S/FARMTASK.S change: force-rebuild (`rm software/SDM/{WORKTASK,FARMTASK,FARM}.bin {WORKTASKB,FARMTASKB}.S` then `make worktask && make farm`), and verify `grep -c DFB WORKTASKB.S` == `wc -c WORKTASK.bin` AND the .bin size reflects the source change. See `feedback_farmtaskb_stale_embed.md`.

**Deploy reminder (tell the user):** ctrl-reset + BRUN. Farm world (bank 32) survives; workshop (bank 33) migrates — keeps discovered recipes + skill, gains an empty GOODS inventory.

---

## Locked facts (verified against shipped code, 2026-06-16)

- WORKEQU ops: WOPSTAT=0 WOPDEP=1 WOPCRAFT=2 WOPCOLL=3 WOPMODE=4 WOPLEARN=5. NSLOTS=2 NRECIP=12. WSTA=$0220 (2×4 stations), WRECIP=$0300. WPANT=$0210 (retiring). DISC=$0215/16, SKILL=$0214, MODE=$0217.
- Recipe entry = `I0,I1,I2,I3,TIME,VALUE,RARITY,pad` (8 B); VALUE at offset +5. Confirmed in RECTAB (BREAD 28 … FEAST 127). The old WCOLL read +5.
- WORKTASK.S WTICK (~line 87): on a STATE==1 station, decrement timer at STAL+2; at 0 sets STATE=2 + WEVDONE(station, recipe). STADR gives STAL=<WSTA+STI*4. Station bytes: STATE(+0), RECIPE(+1), TIMER(+2), pad(+3).
- WCOLL (~line 290): checks STATE==2, reads recipe at +1, value at +5, BOOM ×2 clamp $FF, returns RES1, sets station idle. **Becomes WSELL.**
- WCRAFT (~line 359): free-station check (RERRFULL); WNEED ×4 → NEED0-3; pantry-cover check WCPCK (RERRCROP); WCSCAN table match; WCHIT (discovery/known roll); WCONSUM on every consume path. **Remove WNEED/NEED/WCPCK/WCONSUM; keep free-station + scan + rolls + cook.**
- WCDEP (~line 216) = deposit handler; WJDEP dispatch (~line 189). **Remove both.**
- FARMTASK: OPWITHDRAW=5, OPADDC=6 exist. No FARMTASK change this increment.
- //e FARM.S: WKCRAFT (~3188) sorts MIXBUF + sends OPCRAFT; WKKEY (~3140) has D (WKK3→WKDEPOS) and C (WKK4→WKCOLLECT); MLOOP dispatch (~1512) routes SCREEN 0→farm, 2→WKKEY, else MKKEY; WSYNC (~2325) reads bank-33 mirrors; CVERNUM (FARMEQU)=3 ALREADY?? — verify, recipe-shop set it to 2; this increment sets 3. WSENDCMD stages CMDOP2/CMDA02-32 → A=result + WRES1V. CROP4/SEED4/PRICE4 mirrors exist (RDMKT). MKDRAW/WKDRAW/WKDRAW_STA are the rendering patterns to mirror.

---

## File map

| File | Change |
|---|---|
| `software/SDM/WORKEQU.S` | WGOODS=$0228 equate; WOPSELL alias note |
| `software/SDM/WORKTASK.S` | WTICK auto-store+idle; WCOLL→WSELL; strip pantry from WCRAFT; remove WCDEP |
| `software/SDM/FARMEQU.S` | CVERNUM 2→3; WBGOODS=$0228 (//e read addr) |
| `software/SDM/FARM.S` | MIGRATE33 + CHKWK split; WSYNC GOODS mirror; WKCRAFT crop-debit; workshop cleanup; INVENTORY screen + I key + MLOOP dispatch + PREVSCR |
| `gateware/rev2/project_obscurus/project_obscurus_tb.v` | WTICK auto-store, WOPSELL, migration, e2e sale, stale-op phases |

---

### Task 1: tb — auto-store, WOPSELL, migration, e2e sale (red)

**Files:** Modify `gateware/rev2/project_obscurus/project_obscurus_tb.v`

- [ ] **Step 1: wk_init CVER→3 + GOODS/pantry zero.** In `wk_init`, change the CVER write to `8'h03` and add zeroing for GOODS + retired pantry:
```verilog
        sdram_write(10'd33, 16'h0005, 8'h03);          // CVER v3
        for (i=0; i<12; i=i+1) sdram_write(10'd33, 16'h0228+i, 8'h00); // GOODS
        for (i=0; i<4;  i=i+1) sdram_write(10'd33, 16'h0210+i, 8'h00); // retired pantry
```

- [ ] **Step 2: auto-store phase.** Add inside `begin : wk_m1` after the existing craft asserts:
```verilog
        // ===== craft auto-stores a good + frees station =====
        begin : wk_autostore
        reg [7:0] r, r1; reg ok; integer w; reg [7:0] st, g;
        // own BREAD (idx0, combo 0,0) at high skill so the known-roll cooks
        sdram_write(10'd33, 16'h0215, 8'h01);   // DISC bit0 (BREAD owned)
        sdram_write(10'd33, 16'h0214, 8'hFF);   // skill FF -> low fail
        sdram_write(10'd33, 16'h0228, 8'h00);   // GOODS[0]=0
        wk_cmd(8'h02, 8'd0, 8'd0, 8'hFF, 8'hFF, r, r1, ok);  // craft BREAD
        if (!ok || r!==8'h01) begin errors=errors+1; $display("FAIL autostore craft r=%h", r); end
        // wait the station to finish (FSIM tiny divider)
        st = 8'h01; w = 0;
        while (st !== 8'h00 && w < 400) begin
            repeat (4000) @(posedge clk100);
            sdram_read(10'd33, 16'h0220, st); w = w + 1;   // station0 STATE
        end
        if (st!==8'h00) begin errors=errors+1; $display("FAIL station not idle after cook st=%h", st); end
        sdram_read(10'd33, 16'h0228, g);                   // GOODS[0]
        if (g!==8'h01) begin errors=errors+1; $display("FAIL GOODS[0]=%h want 01", g); end
        else $display("PASS craft auto-stores good + frees station");
        end
```
(Note: the station never enters STATE 2 now — it goes 1→0 with GOODS++ on done. The poll waits for STATE==0.)

- [ ] **Step 3: WOPSELL phase + e2e sale.** After wk_autostore:
```verilog
        // ===== WOPSELL: sell goods at value, e2e to farm cash =====
        begin : wk_sell
        reg [7:0] r, r1; reg ok; reg [7:0] c0l, c0h;
        sdram_write(10'd33, 16'h0228, 8'd5);    // GOODS[0]=5 (BREAD value 28)
        sdram_write(10'd33, 16'h0217, 8'h00);   // MODE: no BOOM
        // sell 2 BREAD -> value 56, GOODS[0]=3
        wk_cmd(8'h03, 8'd0, 8'd2, 8'h00, 8'h00, r, r1, ok);  // WOPSELL(prod0, qty2)
        if (!ok || r!==8'h01) begin errors=errors+1; $display("FAIL wopsell r=%h", r); end
        if (r1!==8'd56) begin errors=errors+1; $display("FAIL sell value=%d want 56", r1); end
        sdram_read(10'd33, 16'h0228, r);
        if (r!==8'd3) begin errors=errors+1; $display("FAIL GOODS[0]=%d want 3", r); end
        // oversell -> RERRCROP, goods unchanged
        wk_cmd(8'h03, 8'd0, 8'd9, 8'h00, 8'h00, r, r1, ok);
        if (r!==8'hE5) begin errors=errors+1; $display("FAIL oversell r=%h want E5", r); end
        sdram_read(10'd33, 16'h0228, r);
        if (r!==8'd3) begin errors=errors+1; $display("FAIL oversell mutated goods=%d", r); end
        // BOOM doubles: MODE bit0, sell 1 -> value 56
        sdram_write(10'd33, 16'h0217, 8'h01);
        wk_cmd(8'h03, 8'd0, 8'd1, 8'h00, 8'h00, r, r1, ok);
        if (r1!==8'd56) begin errors=errors+1; $display("FAIL boom sell=%d want 56", r1); end
        // e2e cash credit: poke farm cash, OPADDC the value, assert
        sdram_read(10'd32, 16'h0220, c0l); sdram_read(10'd32, 16'h0221, c0h);
        farm_cmd(8'h06, 8'd56, 8'd0, 8'h00, r, ok);   // OPADDC 56
        sdram_read(10'd32, 16'h0220, r); sdram_read(10'd32, 16'h0221, r1);
        if ({r1,r} !== {c0h,c0l} + 16'd56) begin errors=errors+1; $display("FAIL e2e cash"); end
        else $display("PASS WOPSELL value + oversell + BOOM + e2e cash");
        end
```

- [ ] **Step 4: stale-op safety + migration phases.** Add a wk_migrate block (after wk_sell): seed DISC+SKILL nonzero, CVER=2, GOODS garbage; the //e migration path is FARM.S (not tb-reachable directly), so instead assert at the BLOB level that the old deposit op (WOPDEP=1) no longer mutates pantry/goods (it should now return RERRBAD since WCDEP is removed):
```verilog
        // ===== stale-op: WOPDEP removed -> RERRBAD, no mutation =====
        begin : wk_staleop
        reg [7:0] r, r1; reg ok;
        sdram_write(10'd33, 16'h0210, 8'h00);
        wk_cmd(8'h01, 8'd0, 8'd5, 8'h00, 8'h00, r, r1, ok);  // old WOPDEP
        if (r!==8'hE6) begin errors=errors+1; $display("FAIL stale WOPDEP r=%h want E6", r); end
        sdram_read(10'd33, 16'h0210, r);
        if (r!==8'h00) begin errors=errors+1; $display("FAIL stale WOPDEP mutated %h", r); end
        else $display("PASS WOPDEP removed (RERRBAD, no-op)");
        end
```
(Migration is //e-side, exercised at bench; tb covers the blob ops. Note this in the report.)

- [ ] **Step 5: run red.** `cd /Users/hambook/Development/project_byte_hamr && make DESIGN=project_obscurus REV=rev2 sim PLUSARGS=+farmonly` (timeout 500000). New asserts FAIL (WTICK still sets STATE=2 not GOODS; WOPSELL is still old WOPCOLL collect semantics; WOPDEP still works). Compile clean. Existing recipe-shop asserts may shift (autostore changes station behavior) — if a PRIOR assert breaks because it expected STATE==2/collect, note it; Task 2 will reconcile.

- [ ] **Step 6: commit.**
```bash
git add gateware/rev2/project_obscurus/project_obscurus_tb.v
git commit -m "test(farm): goods-inventory tb - auto-store, WOPSELL, e2e sale, stale-op (red)"
```

---

### Task 2: WORKTASK — auto-store, WOPSELL, strip pantry (green)

**Files:** Modify `software/SDM/WORKEQU.S`, `software/SDM/WORKTASK.S`

- [ ] **Step 1: WORKEQU.** After `WSTA = $0220` add:
```
WGOODS = $0228
```
Add a comment by the ops block: `* WOPCOLL slot ($03) now = SELL goods`.

- [ ] **Step 2: WTICK auto-store + idle.** Replace the WTICK done-handler (the `* done: state=2 ...` block through the PUTEV) with:
```
* done: GOODS[recipe]++ (clamp), station IDLE,
* WEVDONE(station, recipe) for the //e notice
 LDA STAL
 CLC
 ADC #1
 LDY #>WSTA
 JSR RDB
 STA EVP1
 STA MTMP
* GOODS[recipe]++ clamp 255
 LDA MTMP
 CLC
 ADC #<WGOODS
 STA RADL
 LDA #0
 ADC #>WGOODS
 STA RADH
 LDA RADL
 LDY RADH
 JSR RDB
 CMP #$FF
 BEQ WTKGFULL
 CLC
 ADC #1
WTKGFULL
 TAX
 LDA RADL
 LDY RADH
 JSR WRB
* station -> IDLE
 LDX #0
 LDA STAL
 LDY #>WSTA
 JSR WRB
* WEVDONE(station, recipe)
 LDA #WEVDONE
 STA EVTYPE
 LDA STI
 STA EVP0
 JSR PUTEV
```
(EVP1 was already set to recipe; MTMP holds recipe for the GOODS index. RADL/RADH used for the GOODS address — confirm not live across PUTEV; PUTEV uses EVTYPE/EVP0/EVP1 + EVLIB scratch, not RADL.)

- [ ] **Step 3: WCOLL → WSELL.** Replace the entire WCOLL handler with a sell handler (TI0=product, TI1=qty):
```
* --- SELL TI0=product TI1=qty ---
* goods>=qty -> dec, value=qty*VALUE (BOOM x2
* clamp $FFFF) in RES/RES1. //e credits cash.
WSELL
 LDA TI0
 CMP #NRECIP
 BCS WSLBAD
 LDA TI1
 BEQ WSLBAD
* goods slot addr -> RADL/H
 LDA TI0
 CLC
 ADC #<WGOODS
 STA RADL
 LDA #0
 ADC #>WGOODS
 STA RADH
 LDA RADL
 LDY RADH
 JSR RDB
 CMP TI1
 BCS WSLOK
 LDA #RERRCROP
 JMP WMFIN
WSLOK
 SEC
 SBC TI1
 TAX
 LDA RADL
 LDY RADH
 JSR WRB
* value = qty * RECIPE[product].VALUE (16-bit)
 LDA TI0
 JSR RECADR
 LDA RADL
 CLC
 ADC #5
 STA RADL
 LDA RADL
 LDY RADH
 JSR RDB
 STA MVAL
 LDA #0
 STA MVAL+1
 LDA TI1
 STA STI
WSLMUL
 LDA MVAL
 CLC
 ADC MVAL
 STA MVAL
 LDA MVAL+1
 ADC MVAL+1
 STA MVAL+1
 DEC STI
 BNE WSLMUL
```
WAIT — that doubles per loop (wrong). Correct multiply value×qty by repeated ADD of VALUE qty times:
```
WSLOK
 SEC
 SBC TI1
 TAX
 LDA RADL
 LDY RADH
 JSR WRB
* VALUE -> PRC
 LDA TI0
 JSR RECADR
 LDA RADL
 CLC
 ADC #5
 STA RADL
 LDA RADL
 LDY RADH
 JSR RDB
 STA PRC
* MVAL(16) = 0; add PRC qty times
 LDA #0
 STA MVAL
 STA MVAL+1
 LDA TI1
 STA STI
WSLMUL
 LDA MVAL
 CLC
 ADC PRC
 STA MVAL
 LDA MVAL+1
 ADC #0
 STA MVAL+1
 DEC STI
 BNE WSLMUL
* BOOM? MVAL *= 2 clamp $FFFF
 LDA #<WMODE
 LDY #>WMODE
 JSR RDB
 AND #1
 BEQ WSLNB
 ASL MVAL
 ROL MVAL+1
 BCC WSLNB
 LDA #$FF
 STA MVAL
 STA MVAL+1
WSLNB
* RES=MVAL lo, RES1=MVAL hi
 LDX MVAL
 LDA #<WRES
 LDY #>WRES
 JSR WRB
 LDX MVAL+1
 LDA #<WRES1
 LDY #>WRES1
 JSR WRB
 LDA #ROK
 JMP WMFIN
WSLBAD
 LDA #RERRBAD
 JMP WMFIN
```
NOTE: this needs scratch `MVAL+1`, `PRC`, `STI` — `PRC` and `STI` exist in WORKTASK scratch? Check the WORKTASK scratch equates; if PRC/MVAL+1 absent, add scratch bytes (there's room in the $0D page). The implementer must verify/add scratch vars and confirm MVAL is 2 bytes. The tb expects RES1 (the value) == 56 for 2×BREAD(28) — that's the LOW byte; values ≤ a few hundred fit, so for the tb's small values RES1 carries the whole value when <256. **Re-examine:** the tb reads `r1` = WRES1 (byte at $0207) and expects 56. But my code puts value LO in WRES ($0206) and HI in WRES1 ($0207). For value 56, lo=56 hi=0 → WRES1=0, not 56. **Conflict with the tb.** Resolve: the old WCOLL returned the value in WRES1 (single byte). Keep that contract for ≤255 values: put the 16-bit value's LOW byte in WRES1 (matching tb + old collect), and document the ≤255 assumption, OR make the //e read both WRES(lo)/WRES1(hi). **Decision: //e reads WRES=lo, WRES1=hi (16-bit), and the tb Step 3 must assert WRES (lo)=56.** Fix the tb in Task 1 Step 3 to read $0206 for the value low byte. Flag this to reconcile: the value is 16-bit (WRES lo / WRES1 hi); update the tb assert to read the low byte. (The implementer reconciles tb+blob so both agree; the 16-bit return is correct for large stacks.)

- [ ] **Step 4: dispatch — WOPSELL replaces WOPCOLL, remove WOPDEP.** In WMGO: keep `CMP #WOPCOLL / BEQ WJCOLL` but rename the trampoline target; remove the `CMP #WOPDEP / BEQ WJDEP` line and the `WJDEP / JMP WCDEP` trampoline. Point WJCOLL→WSELL:
```
WJCOLL
 JMP WSELL
```
Delete WJDEP + WCDEP entirely.

- [ ] **Step 5: strip pantry from WCRAFT.** In WCRAFT: delete the `WCRSL` per-crop need block (NEED0-3 init + 4×WNEED), delete the `WCPCK` pantry-cover loop + `WCPOOR`, and delete all `JSR WCONSUM` calls (no-match RUIN path, the known-fail path, discovery-fail path, and the cook path WCGO). Keep: free-station check (RERRFULL), WCSCAN match, WCHIT/WKNOWN rolls, and the station-start in WCGO. Also delete the now-unused `WNEED`/`WNDONE` and `WCONSUM` routines. The cook path WCGO must NOT consume — it goes straight to setting STATE=1/recipe/timer.

- [ ] **Step 6: build + farmonly green.**
```bash
rm -f software/SDM/WORKTASK.bin software/SDM/WORKTASKB.S
make worktasksim
make DESIGN=project_obscurus REV=rev2 sim PLUSARGS=+farmonly   # timeout 500000
echo "blob: $(wc -c < software/SDM/WORKTASKSIM.bin)"
```
All wk_* phases green (auto-store, WOPSELL, stale-op). Recipe-shop discovery/known-fail asserts still green (rolls untouched). Report blob size vs 4096.

- [ ] **Step 7: commit.**
```bash
git add software/SDM/WORKEQU.S software/SDM/WORKTASK.S gateware/rev2/project_obscurus/worktask.mem gateware/rev2/project_obscurus/project_obscurus_tb.v
git commit -m "feat(farm): WORKTASK goods - auto-store on cook, WOPSELL, pantry removed"
```

---

### Task 3: full-suite gate

- [ ] `make DESIGN=project_obscurus REV=rev2 sim` (~10 min, timeout 700000). Zero FAIL. Non-farm failure = regression: STOP.

---

### Task 4: FARM.S — CVER migration + GOODS mirror

**Files:** Modify `software/SDM/FARMEQU.S`, `software/SDM/FARM.S`

- [ ] **Step 1: equates.** FARMEQU.S: `CVERNUM = 2` → `CVERNUM = 3`. Add `WBGOODS = $0228` (//e read address for bank-33 goods).

- [ ] **Step 2: new vars.** FARM.S: add `WGOODS DS 12` (goods mirror), `PREVSCR DS 1`, `INVPAGE DS 1` (0=seeds 1=crops 2=goods), `INVCUR DS 1` (cursor row within page), `INVOFF DS 1` (scroll window offset), `INVSELL DS 1` (selected absolute index for sell).

- [ ] **Step 3: WSYNC reads GOODS.** In WSYNC, after the DISC read block, add a 12-byte GOODS read into WGOODS:
```
* goods 12 bytes -> WGOODS
 LDX #0
WSYNG
 TXA
 CLC
 ADC #<WBGOODS
 LDY #>WBGOODS
 JSR WFRD
 LDA SDM_VAL
 STA WGOODS,X
 INX
 CPX #12
 BNE WSYNG
```
(WBGOODS=$0228, +11=$0233, no page cross; WFRD reads bank 33.)

- [ ] **Step 4: split CHKWK into cold-seed vs migrate.** Find CHKWK (the bank-33 SIG/CVER check that calls SEED33 on mismatch). Restructure: if SIG invalid → SEED33 (full, as today). If SIG valid AND CVER≠CVERNUM → JSR MIGRATE33 (new). Add MIGRATE33:
```
* === MIGRATE33: valid SIG, CVER bumped.
* zero GOODS + retired pantry, set CVER,
* PRESERVE DISC/SKILL/MODE/STATIONS/recipes ===
MIGRATE33
 LDA #0
 STA SDM_VAL
* zero retired pantry $0210-13
 LDX #0
MIG_PANT
 TXA
 CLC
 ADC #$10
 LDY #$02
 JSR WFWR
 INX
 CPX #4
 BNE MIG_PANT
* zero GOODS $0228-33
 LDX #0
MIG_GOOD
 TXA
 CLC
 ADC #<WBGOODS
 LDY #>WBGOODS
 JSR WFWR
 INX
 CPX #12
 BNE MIG_GOOD
* CVER = CVERNUM
 LDA #CVERNUM
 STA SDM_VAL
 LDA #$05
 LDY #$02
 JSR WFWR
 RTS
```
(WFWR writes SDM_VAL to bank-33 addr A/Y. Confirm WFWR's contract by reading it; it should mirror FWR but bank 33. `#$05`=$0005 CVER, `#$10`=$0210, etc. — low bytes, hi $02.)

- [ ] **Step 5: SEED33 must also init GOODS.** In SEED33 (cold start), add zeroing of GOODS[12] ($0228-33) and set CVER=3 (CVERNUM). Confirm SEED33 already writes CVER (recipe-shop set it to CVERNUM=2; now 3 via the equate — no code change if it uses `#CVERNUM`). Add the GOODS-zero loop (same as MIG_GOOD body).

- [ ] **Step 6: assemble + commit.**
```bash
rm -f software/SDM/FARM.bin
make farm
git add software/SDM/FARMEQU.S software/SDM/FARM.S
git commit -m "feat(farm): bank-33 CVER=3 selective migration + GOODS mirror (preserve recipes/skill)"
```

---

### Task 5: FARM.S — craft pulls from crops; workshop cleanup

**Files:** Modify `software/SDM/FARM.S`

- [ ] **Step 1: WKCRAFT debits crops before OPCRAFT.** After the MIXBUF sort+pad (at WKISDN, before staging OPCRAFT), insert a crop-need check + debit. Compute per-crop needs from MIXBUF (counts of crop ids 0-3, ignoring $FF), check CROP4 mirror covers each, OPWITHDRAW each:
```
WKISDN
* per-crop needs from MIXBUF (0-3; $FF pad)
 LDA #0
 STA NEEDW
 STA NEEDC
 STA NEEDB
 STA NEEDP
 LDX #0
WKNDLP
 LDA MIXBUF,X
 CMP #4
 BCS WKNDNX
 TAY
 INC NEEDW,Y
WKNDNX
 INX
 CPX #4
 BNE WKNDLP
* check CROP4 mirror covers needs
 LDX #0
WKCKLP
 LDA NEEDW,X
 BEQ WKCKNX
 CMP CROP4,X
 BEQ WKCKNX
 BCS WKNOCROP
WKCKNX
 INX
 CPX #4
 BNE WKCKLP
* debit each needed crop from farm
 LDX #0
WKDBLP
 LDA NEEDW,X
 BEQ WKDBNX
 STX EVT
 STA CMDA1
 STX CMDA0
 LDA #OPWDRAW
 STA CMDOP
 JSR SENDCMD
 LDX EVT
WKDBNX
 INX
 CPX #4
 BNE WKDBLP
* fall through to existing OPCRAFT staging
```
Add `WKNOCROP` (message NEED CROPS, clear MIX, redraw):
```
WKNOCROP
 LDA #0
 STA MIXN
 JSR CLRMSG
 LDA #<SNEEDS
 STA MSGPTR
 LDA #>SNEEDS
 STA MSGPTR+1
 LDA #0
 STA PRCOL
 LDY #21
 JSR PRSTR
 JSR WKDRAW
 RTS
```
New vars: `NEEDW DS 1 / NEEDC DS 1 / NEEDB DS 1 / NEEDP DS 1` (4 contiguous, indexed as NEEDW,X). `SNEEDS` string exists (recipe-shop "NEED CROPS"? verify — if it's "NEED SEEDS" reuse/rename; add `SNEEDC2 ASC "NEED CROPS"` if needed). The existing OPCRAFT staging (LDA #WOPCRAFT ... JSR WSENDCMD) follows unchanged. NOTE: `CMP CROP4,X / BEQ / BCS WKNOCROP` — need == is ok (have exactly enough); BCS on need>crop. Verify the compare polarity (need > have → fail): `LDA need / CMP have` → carry set if need≥have; need==have ok, need>have fail. Use `BEQ ok / BCS fail` after CMP. (The code above does BEQ WKCKNX then BCS WKNOCROP — correct.)

- [ ] **Step 2: remove D and C from WKKEY.** Delete WKK3 (D→WKDEPOS) and WKK4 (C→WKCOLLECT) branches. WKK2 (RETURN→craft) now falls to WKKNONE for any other key. Delete the WKDEPOS, WKDEPOK, WKCOLLECT, WKCOLL_OK routines entirely (and WKCOLL_TE etc.). Remove the `CMP #OPWDRAW / JMP WKDEPOK` hook in DCOK (deposit no longer routes through DOCMD — WKCRAFT calls SENDCMD directly).

- [ ] **Step 3: workshop screen cleanup.** WKDRAW_STA: remove the STATE==2 (DONE PRESS C) rendering — stations are only 0 (IDLE) or 1 (COOKING name) now. Remove the pantry strip from WKDRAW (the PANTRY row) and its WPANT4 mirror reads (delete WPANT4 var + its WSYNC read). Legend string → `1-4 MIX RTN CRAFT I INV ESC`.

- [ ] **Step 4: add I-key to farm/market/workshop.** Farm key chain (KFARM area), MKKEY, and WKKEY each get an `I` ($C9) handler that sets PREVSCR=current screen and JSR INVENTER. E.g. in WKKEY before WKKNONE:
```
 CMP #$C9 ; I -> inventory
 BNE <next>
 LDA #2
 STA PREVSCR
 JSR INVENTER
 JMP MLOOP
```
(Farm sets PREVSCR=0, market=1, workshop=2.)

- [ ] **Step 5: assemble + commit.**
```bash
rm -f software/SDM/FARM.bin
make farm
git commit -am "feat(farm): craft debits crops directly (pantry gone); workshop drops D/C/pantry"
```

---

### Task 6: FARM.S — paged scrolling INVENTORY screen

**Files:** Modify `software/SDM/FARM.S`

- [ ] **Step 1: screen constants + strings.** Add `VROWS = 16` equate. Strings: `SINVT ASC "INVENTORY"`, page names `SPSEED ASC "SEEDS"` / `SPCROP ASC "CROPS"` / `SPGOOD ASC "GOODS"`, `SINVLEG ASC "UP/DN TAB-PAGE S-SELL ESC"`, plus the up/down hint chars. Crop names exist (SNAML/SNAMH); goods names exist (RNAML/RNAMH).

- [ ] **Step 2: INVENTER / INVEXIT.**
```
* === INVENTER: -> inventory screen ===
INVENTER
 LDA #3
 STA SCREEN
 STA TEXTSW
 LDA #2
 STA INVPAGE      ; default to GOODS page
 LDA #0
 STA INVCUR
 STA INVOFF
 JSR RDMKT
 JSR WSYNC
 JSR INVDRAW
 RTS
* === INVEXIT: -> PREVSCR ===
INVEXIT
 LDA PREVSCR
 STA SCREEN
 CMP #0
 BNE INVX1
 STA GRAPH        ; farm = GR
 JSR DRAWALL
 JSR HUDDRAW
 RTS
INVX1
 STA TEXTSW       ; market/workshop = text
 CMP #1
 BNE INVX2
 JSR MKDRAW
 RTS
INVX2
 JSR WKDRAW
 RTS
```

- [ ] **Step 3: INVDRAW (paged + scrolling).** Renders header + the current page's VROWS window. Page item count: SEEDS=4, CROPS=4, GOODS=12 (use a helper INVCNT returning A=count for INVPAGE). Each visible row index = INVOFF + i; render name + count (+ price for crops/goods); mark cursor row `>`; show `^`/`v` if INVOFF>0 / INVOFF+VROWS<count. Full code:
```
INVDRAW
 JSR MKCLR             ; blank rows 0-19 (reuse)
* row 0: title + page name + cash
 LDA #<SINVT
 STA MSGPTR
 LDA #>SINVT
 STA MSGPTR+1
 LDA #0
 STA PRCOL
 LDY #0
 JSR PRSTR
 JSR INVPGNAME        ; prints page name at col 12 row 0
 JSR CLRROW20
 JSR HUDCASH
* viewport rows 2..2+VROWS-1
 JSR INVCNT           ; A = item count for page -> NTMP
 STA NTMP
 LDA #0
 STA EVT              ; EVT = visible row i (0..VROWS-1)
INVDLP
 LDA EVT
 CMP #VROWS
 BCS INVDDN
* abs index = INVOFF + i
 CLC
 ADC INVOFF
 CMP NTMP             ; past end?
 BCS INVDNX2
 STA EVA              ; EVA = abs item index
* screen row = 2 + i
 LDA EVT
 CLC
 ADC #2
 STA EVB              ; EVB = screen row
* cursor mark '>' at col 0 if EVA==INVCUR+INVOFF
 LDA INVOFF
 CLC
 ADC INVCUR
 CMP EVA
 BNE INVDNOC
 LDY EVB
 LDA GRLO,Y
 STA LINEP
 LDA GRHI,Y
 STA LINEP+1
 LDA #$BE             ; '>'
 LDY #0
 STA (LINEP),Y
INVDNOC
 JSR INVROW           ; render item EVA on row EVB (name+count+price)
INVDNX2
 INC EVT
 JMP INVDLP
INVDDN
* up/down more-hints
 JSR INVHINTS
* row 23 legend
 LDA #<SINVLEG
 STA MSGPTR
 LDA #>SINVLEG
 STA MSGPTR+1
 LDA #0
 STA PRCOL
 LDY #23
 JSR PRSTR
 RTS
```
Helpers the implementer writes (mirror existing render code):
- `INVCNT`: A = 4 (page 0/1) or 12 (page 2).
- `INVPGNAME`: prints SPSEED/SPCROP/SPGOOD at row 0 col 12 by INVPAGE.
- `INVROW` (EVA=abs index, EVB=row): page 0 → crop name (SNAML/SNAMH,EVA) + SEED4[EVA]; page 1 → crop name + CROP4[EVA] + PRICE4[EVA] (PRDEC3 at a price column); page 2 → good name (RNAML/RNAMH,EVA) + WGOODS[EVA] + RECIPE value (the //e has no value table — read from... use a //e RVALUE table = the RPRICEL/2, OR add a small RVALUE DFB 28,36,... mirroring RECTAB+5). **Add `RVALUE DFB 28,36,48,56,76,80,80,92,100,105,110,127`** for the goods price column. Counts via PRDEC3.
- `INVHINTS`: if INVOFF>0 print `^` at row 1 col 0; if INVOFF+VROWS<count print `v` at row 22 col 0.

- [ ] **Step 4: INVKEY (cursor/scroll/page/sell).**
```
INVKEY
 CMP #$9B ; ESC
 BNE INVK1
 JSR INVEXIT
 JMP MLOOP
INVK1
 CMP #$8B ; up
 BNE INVK2
 JSR INVUP
 JSR INVDRAW
 JMP MLOOP
INVK2
 CMP #$8A ; down
 BNE INVK3
 JSR INVDOWN
 JSR INVDRAW
 JMP MLOOP
INVK3
 CMP #$89 ; TAB -> next page
 BNE INVK4
 INC INVPAGE
 LDA INVPAGE
 CMP #3
 BNE INVKPG
 LDA #0
 STA INVPAGE
INVKPG
 LDA #0
 STA INVCUR
 STA INVOFF
 JSR INVDRAW
 JMP MLOOP
INVK4
 CMP #$D3 ; S -> sell (crops/goods pages only)
 BNE INVKNONE
 LDA INVPAGE
 BEQ INVKNONE     ; seeds page: inert
 JSR INVSELLIT
 JMP MLOOP
INVKNONE
 JMP MLOOP
```
- `INVUP`: if INVCUR>0 dec INVCUR; else if INVOFF>0 dec INVOFF. `INVDOWN`: abs=INVOFF+INVCUR; if abs+1<count: if INVCUR<VROWS-1 inc INVCUR else inc INVOFF. (Standard cursor-scroll clamp against INVCNT.)
- `INVSELLIT`: abs index = INVOFF+INVCUR. Set QPCROP/selected = abs. `S` → QTYPROMPT path: stage CMDOP per page (CROPS → OPSELL, GOODS → WOPSELL via WSENDCMD). Reuse QTYPROMPT for the qty entry; on confirm, page 1 → `OPSELL(abs, qty)` (SENDCMD, farm credits cash); page 2 → `WOPSELL(abs, qty)` (WSENDCMD) → take WRES/WRES1 (16-bit value) → `OPADDC(lo,hi)` (SENDCMD). Then message `SOLD <name> +nnn`, JSR RDMKT + WSYNC + INVDRAW.
  - QTYPROMPT currently JMPs to DOCMD (farm mailbox). For GOODS sell the op goes to the WORKSHOP mailbox (WSENDCMD), so the goods path can't reuse QTYPROMPT's DOCMD tail directly. **Implement a small inventory-local qty entry** (mirror QTYPROMPT's modal digit loop, ~20 lines) that returns the qty in A/var, then branch to the correct op by page. Don't force QTYPROMPT's farm-mailbox tail onto the goods path.

- [ ] **Step 5: MLOOP dispatch.** In MLOOP screen dispatch add SCREEN==3 → INVKEY:
```
 LDX SCREEN
 BEQ KFARM
 CPX #2
 BEQ JWKKEY
 CPX #3
 BEQ JINVKEY
 JMP MKKEY
JINVKEY
 JMP INVKEY
JWKKEY
 JMP WKKEY
```

- [ ] **Step 6: assemble + commit.**
```bash
rm -f software/SDM/FARM.bin
make farm
git commit -am "feat(farm): paged scrolling INVENTORY screen - seeds/crops/goods, sell"
```
Watch branch ranges across the new screen (invert+JMP per the DCBAD precedent). Self-review: INVUP/INVDOWN clamp against INVCNT; sell routes by page; PREVSCR restore.

---

### Task 7: disk + gate + handoff + bench

- [ ] **Step 1: force-clean rebuild (stale-artifact guard).**
```bash
cd /Users/hambook/Development/project_byte_hamr
rm -f software/SDM/{WORKTASK,FARMTASK,FARM}.bin software/SDM/{WORKTASKB,FARMTASKB}.S
make worktask && make farm && make sdmdisk
echo "embeds: WT=$(wc -c < software/SDM/WORKTASK.bin)/$(grep -c DFB software/SDM/WORKTASKB.S) FT=$(wc -c < software/SDM/FARMTASK.bin)/$(grep -c DFB software/SDM/FARMTASKB.S)"
```
Both embed pairs must match. Report WORKTASK.bin size (must reflect the pantry-removal + GOODS/WOPSELL — i.e. differ from the prior 1402).

- [ ] **Step 2: full suite.** `make DESIGN=project_obscurus REV=rev2 sim` — zero FAIL.

- [ ] **Step 3: handoff.** Append to `software/SDM/HANDOFF_FARM_V2.md` "Where we are":
```
- **v2 goods-inventory SHIPPED (branch farm-v2)**: pantry/deposit DELETED
  - craft pulls crops straight from farm (//e OPWITHDRAW debit-first, then
  OPCRAFT cooks w/o consuming). Finished crafts auto-store GOODS[12]
  (bank 33 $0228), station auto-idles, WEVDONE still fires. WOPCOLL slot
  -> WOPSELL(product,qty) @ value (BOOM x2). New paged scrolling INVENTORY
  screen (key I, PREVSCR return): SEEDS(ro)/CROPS(sell @price)/GOODS(sell
  @value), TAB pages, up/dn scrolls VROWS=16 window - N-scalable. CVER=3
  SELECTIVE migration (MIGRATE33): preserves DISC/SKILL/recipes, zeros
  GOODS+retired pantry. Deploy = ctrl-reset + BRUN. Next: increment 4
  world events (FARMTASK headroom relocation still pending).
```

- [ ] **Step 4: commit handoff + artifacts.**
```bash
git add software/SDM/HANDOFF_FARM_V2.md software/SDM/*.bin software/SDM/*B.S gateware/rev2/project_obscurus/*.mem
git commit -m "docs(farm): handoff - goods inventory shipped"
```

- [ ] **Step 5: bench checklist (tell user).**
  1. Ctrl-Reset + BRUN. Workshop keeps your recipes + skill (migration); GOODS empty; farm crops survive.
  2. Workshop: no D/C keys, no pantry strip. MIX 1-4 + RETURN — crafts now pull straight from your harvested crops (NEED CROPS if short). Station shows COOKING then goes IDLE on its own (no collect).
  3. `I` → INVENTORY. TAB cycles SEEDS/CROPS/GOODS. Up/Down moves the cursor (scrolls if list > screen).
  4. GOODS page after a craft finishes → the product count went up. Cursor it, `S`, qty → cash up (`SOLD <name> +nnn`).
  5. CROPS page → `S` sells crops at market price.
  6. ESC returns to whatever screen you came from (farm/market/workshop).
  7. `/obs-screenshot` the inventory pages.

---

## Self-review notes

- Spec coverage: GOODS array + auto-store ✓ (T2 S2), station auto-idle ✓, WEVDONE kept ✓, WOPSELL @value+BOOM ✓ (T2 S3), pantry/deposit/WCONSUM removed ✓ (T2 S4-5), craft-from-crops debit-first ✓ (T5 S1), no-refund-on-fail ✓ (WCONSUM removal means crops debited by //e stay spent), INVENTORY paged/scrolling/N-scalable ✓ (T6), 3 sections w/ seeds read-only ✓, sell routing crop=price/good=value ✓ (T6 S4), PREVSCR ✓ (T6 S2), CVER=3 selective migration preserve DISC/SKILL/recipes ✓ (T4 S4), cold vs migrate split ✓, RECIPE.VALUE pre-existing ✓ (RVALUE table mirrors RECTAB+5), tb e2e sale + migration-note + stale-op ✓ (T1), measured blob size ✓ (T7 S1).
- Riskiest: T2 S3 WOPSELL 16-bit value vs tb byte assert (reconcile: //e reads WRES lo + WRES1 hi; tb asserts the low byte) — flagged inline. T6 S4 goods-sell can't reuse QTYPROMPT's farm-mailbox tail — inventory-local qty entry required (flagged). T5 S1 need/debit polarity (flagged). Branch ranges across T6 (invert+JMP).
- Migration is //e-side (FARM.S), not sim-reachable; tb covers blob ops, migration verified at bench (noted T1 S4).
- Type consistency: WGOODS/WBGOODS/WSELL/WOPSELL/MIGRATE33/INVENTER/INVEXIT/INVDRAW/INVKEY/INVCNT/INVROW/INVUP/INVDOWN/INVSELLIT/INVPAGE/INVCUR/INVOFF/PREVSCR/NEEDW-P/RVALUE used consistently.
- Plan-time: pick exact columns in INVROW/INVDRAW against 40-col rows; confirm VROWS=16 leaves room (header 0-1, viewport 2-17, hint 22, cash 20, legend 23).
