# Recipe Shop (Crafting Economy v2) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace blind discovery-only crafting with a two-road economy — buy recipes up a price ladder (market R key) OR gamble-discover them, with known recipes still rolling a skill-eased failure chance — plus two parked UX fixes (deposit crop-picker, farm-screen workshop status widget).

**Architecture:** Coproc-first TDD. WORKTASK gains a known-recipe failure roll (pinned to the bit-set branch, NOT the shared WCGO cook entry) + WOPLEARN op. FARMTASK gains OPSPEND (after a 37 B addressing reclaim — blob is at 1772/1792). Then the //e: refund-on-reject buy flow, market recipe row, RFAIL message, deposit picker, farm widget. CVER bumps to 2 (forces bank-33 re-seed on deploy).

**Tech Stack:** Merlin32 6502 (single-space format, ASCII, ≤50-char lines), iverilog -g2005 tb, `make worktasksim`/`farmtasksim`/`worktask`/`farm`/`sdmdisk`, gates `+farmonly` (~2-4 min) + full suite (~10 min).

**Spec:** `docs/superpowers/specs/2026-06-12-farm-v2-recipe-shop-design.md` — READ "Design: two roads", "WORKTASK changes" (esp. WCGO mandate + LFSR polarity), "FARMTASK changes", "//e (FARM.S)", "Testbench". Base recipe data in `docs/superpowers/specs/2026-06-11-farm-v2-sidework-design.md`.

**Deploy reminder (tell the user):** blob + CVER change → Ctrl-Reset + BRUN. CVER bump re-seeds bank 33: **pantry, skill, owned recipes all reset to zero.** Farm world (bank 32) survives. Crafting starts fresh.

---

## Locked facts (verified against shipped code, 2026-06-12)

- WORKTASK.S WCHIT (~line 417): `JSR WDBIT / AND MTMP / BNE WCGO` is the bit-set (known) branch; the discovery roll follows on fall-through; `WCDISC` (success) does `JMP WCGO` (~line 480); `WCGO` (~line 485) is the shared cook tail. **The known-fail roll goes on the BNE-WCGO branch target (new WKNOWN label), never at WCGO.**
- WORKEQU ops: WOPSTAT=0 WOPDEP=1 WOPCRAFT=2 WOPCOLL=3 WOPMODE=4 → **WOPLEARN=5** next free. RRUIN=$E8 exists; add **RFAIL=$E9**.
- WORKTASK dispatch chain at WMGO (~line 177): `CMP #WOPMODE / BEQ WJMODE / LDA #RERRBAD` — insert WOPLEARN compare + WJLEARN trampoline before the RERRBAD fallthrough.
- FARMTASK dispatch (~line 171): ends `CMP #OPADDC / BEQ JADDC / LDA #RERRBAD`. OPS: OPWDRAW=5 OPADDC=6 → **OPSPEND=7**. RERRCASH=$E4 already in FARMEQU (line 55).
- FARMTASK addr-hi reclaim sites (constant $02): lines 211-212, 259-260, 337-338, 432-433 (`LDA #0 / ADC #>FSEEDS|FCROPS`). FSEEDS=$0222 FCROPS=$0226 NCROPS=4 → max +3 = $0225/$0229, no page cross.
- FARMTASK.bin = **1772 / 1792 = 20 B headroom**. Reclaim (~37 B) is a prerequisite for OPSPEND (~50 B).
- FARM.S: WSKILLV (line 74), WDISCV (2 bytes, line 75) — WSYNC-populated mirrors. WSENDCMD (743) stages CMDOP2/CMDA02-32, returns A=result + WRES1V. QTYPROMPT SQGO (1775) hardcodes `LDA SELCROP / STA CMDA0` — change to a QPCROP var. MKKEY chain MK1..MK4 (1550-1597): MK4 = W key; **R ($D2) is unbound** — new MK5. WKCRAFT result dispatch (3160) branches ROK/RRUIN/RERRCROP/RERRFULL — add RFAIL. CVERNUM (FARMEQU 114) = 1 → 2.
- Recipe values (RECTAB, base spec): 28,36,48,56,76,80,80,92,100,105,110,127. Rarities: 0,0,0,1,1,1,1,1,2,2,2,3. Prices = 2×value: 56,72,96,112,152,160,160,184,200,210,220,254.

---

## File map

| File | Change |
|---|---|
| `software/SDM/WORKEQU.S` | RFAIL=$E9, WOPLEARN=5, FAILFLOOR=8 equates |
| `software/SDM/WORKTASK.S` | known-fail roll (WKNOWN), FAILBASE table, WOPLEARN op |
| `software/SDM/FARMEQU.S` | OPSPEND=7, CVERNUM=2 |
| `software/SDM/FARMTASK.S` | addr-hi reclaim, OPSPEND op |
| `software/SDM/FARM.S` | RPRICE table, market recipe row + R handler (refund-on-reject), RFAIL msg, deposit crop-picker (QPCROP), farm workshop widget |
| `gateware/rev2/project_obscurus/project_obscurus_tb.v` | WOPLEARN, OPSPEND, known-fail-curve, refund phases |

---

### Task 1: tb — WOPLEARN, OPSPEND, known-fail curve (red)

**Files:**
- Modify: `gateware/rev2/project_obscurus/project_obscurus_tb.v`

- [ ] **Step 1: bump wk_init CVER to 2.** In `wk_init`, change the CVER write from `8'h01` to `8'h02`:
```verilog
        sdram_write(10'd33, 16'h0005, 8'h02);          // CVER
```

- [ ] **Step 2: add a wk_learn phase.** Insert inside `begin : wk_m1` after the existing roll/rarity asserts, before the block's `end`:
```verilog
        // ===== recipe shop: WOPLEARN =====
        begin : wk_learn
        reg [7:0] r, r1; reg ok;
        // clear DISC, learn BREAD (idx 0) -> bit 0 set
        sdram_write(10'd33, 16'h0215, 8'h00);
        sdram_write(10'd33, 16'h0216, 8'h00);
        wk_cmd(8'h05, 8'd0, 8'h00, 8'h00, 8'h00, r, r1, ok);   // WOPLEARN 0
        if (!ok || r!==8'h01) begin errors=errors+1; $display("FAIL learn r=%h", r); end
        sdram_read(10'd33, 16'h0215, r);
        if (r[0]!==1'b1) begin errors=errors+1; $display("FAIL learn bit=%h", r); end
        // double-learn -> RERRBAD
        wk_cmd(8'h05, 8'd0, 8'h00, 8'h00, 8'h00, r, r1, ok);
        if (r!==8'hE6) begin errors=errors+1; $display("FAIL double-learn r=%h want E6", r); end
        else $display("PASS WOPLEARN + double-learn guard");
        // learn idx 9 (PIE, >7 -> hi byte)
        wk_cmd(8'h05, 8'd9, 8'h00, 8'h00, 8'h00, r, r1, ok);
        sdram_read(10'd33, 16'h0216, r);
        if (r[1]!==1'b1) begin errors=errors+1; $display("FAIL learn hi bit=%h", r); end
        else $display("PASS WOPLEARN hi byte (idx 9)");
        // out-of-range idx -> RERRBAD
        wk_cmd(8'h05, 8'd12, 8'h00, 8'h00, 8'h00, r, r1, ok);
        if (r!==8'hE6) begin errors=errors+1; $display("FAIL learn-12 r=%h want E6", r); end
        end
```

- [ ] **Step 3: add a wk_known_fail phase** (distribution test, not pin-to-observed). After wk_learn:
```verilog
        // ===== known-recipe failure curve =====
        begin : wk_kfail
        reg [7:0] r, r1; reg ok; integer a, nok, nfail;
        // own BREAD (idx0, combo 0,0), skill 0 -> FAILBASE[0]=48/256 ~19%
        sdram_write(10'd33, 16'h0215, 8'h01);   // DISC bit 0
        sdram_write(10'd33, 16'h0216, 8'h00);
        sdram_write(10'd33, 16'h0214, 8'h00);   // skill 0
        nok = 0; nfail = 0;
        for (a = 0; a < 30; a = a + 1) begin
            wk_cmd(8'h01, 8'd0, 8'd2, 8'h00, 8'h00, r, r1, ok); // deposit 2 wheat
            sdram_read(10'd33, 16'h0210, r);                    // pantry wheat
            wk_cmd(8'h02, 8'd0, 8'd0, 8'hFF, 8'hFF, r, r1, ok); // craft BREAD
            if (r === 8'h01) nok = nok + 1;
            else if (r === 8'hE9) nfail = nfail + 1;
            else begin errors=errors+1; $display("FAIL kfail unexpected r=%h", r); end
            sdram_read(10'd33, 16'h0210, r1);                   // pantry after
            if (r1 !== (r - r)) ;                                // (placeholder no-op)
            // pantry must drop by 2 every attempt (consumed either way)
            // collect/clear any cooking station so next attempt has a free slot
            begin : kf_clear
            integer w; reg [7:0] st;
            st = 8'h01; w = 0;
            while (st === 8'h01 && w < 200) begin
                repeat (4000) @(posedge clk100);
                sdram_read(10'd33, 16'h0220, st); w = w + 1;
            end
            wk_cmd(8'h03, 8'd0, 8'h00, 8'h00, 8'h00, r, r1, ok); // collect sta0 if done
            end
        end
        if (nok == 0 || nfail == 0) begin errors=errors+1;
            $display("FAIL kfail no spread: ok=%0d fail=%0d", nok, nfail); end
        else $display("PASS known-fail curve ok=%0d fail=%0d (both occur)", nok, nfail);
        end
```
(The `(r - r)` line is a deliberate no-op placeholder; the implementer should DELETE it and instead capture pantry-before into a separate reg and assert pantry-after == before−2. Write that assert properly — it is the "consumed either way" check. The phase as given is the spec's distribution intent; tighten it.)

- [ ] **Step 4: add OPSPEND to wk_m1's bus section.** Near the existing OPADDCASH assert:
```verilog
        // OPSPEND: debit farm cash, insufficient -> RERRCASH
        sdram_read(10'd32, 16'h0220, h1); sdram_read(10'd32, 16'h0221, h2);
        farm_cmd(8'h07, 8'd50, 8'd0, 8'h00, r, ok);   // spend 50
        if (!ok || r!==8'h01) begin errors=errors+1; $display("FAIL spend r=%h", r); end
        sdram_read(10'd32, 16'h0220, r); sdram_read(10'd32, 16'h0221, r1);
        if ({r1,r} !== {h2,h1} - 16'd50) begin errors=errors+1;
            $display("FAIL post-spend cash %h%h", r1, r); end
        // overspend: ask for 60000 -> RERRCASH, cash unchanged
        sdram_read(10'd32, 16'h0220, h1); sdram_read(10'd32, 16'h0221, h2);
        farm_cmd(8'h07, 8'h60, 8'hEA, 8'h00, r, ok);  // 60000
        if (r!==8'hE4) begin errors=errors+1; $display("FAIL overspend r=%h want E4", r); end
        sdram_read(10'd32, 16'h0220, r); sdram_read(10'd32, 16'h0221, r1);
        if ({r1,r} !== {h2,h1}) begin errors=errors+1; $display("FAIL overspend mutated cash"); end
        else $display("PASS OPSPEND + insufficient guard");
```
(Note: ensure farm cash is > 50 at this point — earlier phases credited 160 via OPADDCASH; if a prior phase left it low, poke `sdram_write(10'd32,16'h0220,8'd200)` first. Verify against the running cash by reading it; report what you did.)

- [ ] **Step 5: run red.**
```bash
cd /Users/hambook/Development/project_byte_hamr
make DESIGN=project_obscurus REV=rev2 sim PLUSARGS=+farmonly
```
Expected: existing phases PASS; new asserts FAIL (WOPLEARN timeout/RERRBAD-missing, OPSPEND op-7 returns E6, kfail never sees $E9). Compile clean. (CVER=2 in wk_init won't break anything — the blob doesn't read CVER.)

- [ ] **Step 6: commit.**
```bash
git add gateware/rev2/project_obscurus/project_obscurus_tb.v
git commit -m "test(farm): recipe-shop tb - WOPLEARN, OPSPEND, known-fail curve (red)"
```

---

### Task 2: WORKTASK known-fail roll + WOPLEARN (green, half)

**Files:**
- Modify: `software/SDM/WORKEQU.S`, `software/SDM/WORKTASK.S`

- [ ] **Step 1: WORKEQU equates.** After `RRUIN = $E8` add:
```
RFAIL = $E9
```
In the ops block after `WOPMODE = $04` add:
```
WOPLEARN = $05
```
Add near the discovery constants:
```
FAILFLOOR = 8
```

- [ ] **Step 2: WOPLEARN dispatch.** In WMGO (~line 183), change:
```
 CMP #WOPMODE
 BEQ WJMODE
 LDA #RERRBAD
 JMP WMFIN
```
to:
```
 CMP #WOPMODE
 BEQ WJMODE
 CMP #WOPLEARN
 BEQ WJLEARN
 LDA #RERRBAD
 JMP WMFIN
```
and add the trampoline next to the others:
```
WJLEARN
 JMP WCLEARN
```

- [ ] **Step 3: WCLEARN handler.** Add near WCMODE (TI0 = recipe idx):
```
* --- LEARN TI0=recipe idx ---
* idx<NRECIP and DISC bit clear -> set bit, ROK
* else RERRBAD (double-learn / bad idx). //e
* gates cash+skill; task only guards dup.
WCLEARN
 LDA TI0
 CMP #NRECIP
 BCC WLOK
 LDA #RERRBAD
 JMP WMFIN
WLOK
 LDA TI0
 STA RIDX
 JSR WDBIT
 AND MTMP
 BEQ WLSET
 LDA #RERRBAD
 JMP WMFIN
WLSET
 JSR WDBIT
 ORA MTMP
 TAX
 LDA RIDX
 CMP #8
 BCS WLHI
 LDA #<WDISCL
 LDY #>WDISCL
 JSR WRB
 LDA #ROK
 JMP WMFIN
WLHI
 LDA #<WDISCH
 LDY #>WDISCH
 JSR WRB
 LDA #ROK
 JMP WMFIN
```
(WDBIT reads RIDX, sets MTMP = mask, returns the current disc byte in A — same pattern WCDISC uses. Verify by reading WDBIT.)

- [ ] **Step 4: known-fail roll on the bit-set branch (THE mandate).** In WCHIT, change:
```
WCHIT
* discovered? DISC bit RIDX
 JSR WDBIT
 AND MTMP
 BNE WCGO
```
to:
```
WCHIT
* discovered? DISC bit RIDX
 JSR WDBIT
 AND MTMP
 BNE WKNOWN
```
Then add WKNOWN immediately BEFORE the discovery-roll code (between the `BNE` target and the `* roll:` comment), implementing the known-recipe failure roll:
```
* known recipe: failure roll. failChance =
* FAILBASE[rarity] - SKILL/2, floor FAILFLOOR.
* LFSR < failChance = FAIL (note: OPPOSITE
* polarity to the discovery roll below, which
* is LFSR < thresh = SUCCESS).
WKNOWN
* rarity at entry+6 -> X
 LDA RIDX
 JSR RECADR
 LDA RADL
 CLC
 ADC #6
 STA RADL
 LDA RADL
 LDY RADH
 JSR RDB
 TAX
 LDA FAILBASE,X
 STA MVAL
* MVAL -= SKILL/2
 LDA #<WSKILL
 LDY #>WSKILL
 JSR RDB
 LSR
 STA MTMP
 LDA MVAL
 SEC
 SBC MTMP
 BCC WKNFL
 CMP #FAILFLOOR
 BCS WKNRL
WKNFL
 LDA #FAILFLOOR
WKNRL
 STA MVAL
 LDA LFSRL
 CMP MVAL
 BCS WCGO        ; LFSR >= failChance -> SUCCESS, cook
* failed: consume + skill++ + RFAIL
 JSR WCONSUM
 JSR WSKUP
 LDA #RFAIL
 JMP WMFIN
```
NOTE: WKNOWN reuses MVAL/MTMP and clobbers RADL — confirm none are live across into WCGO (WCGO recomputes everything it needs from RIDX/STAL). The discovery code below (`* roll:`) is now reached only on fall-through from the `AND MTMP` being zero — i.e. bit clear — UNCHANGED.

- [ ] **Step 5: FAILBASE table.** Add next to the other blob tables (GROWM-equivalent area, after the recipe-scan code):
```
* known-craft failChance base per rarity /256
FAILBASE DFB 48,80,112,144
```

- [ ] **Step 6: build + partial green.**
```bash
make worktasksim
make DESIGN=project_obscurus REV=rev2 sim PLUSARGS=+farmonly
```
Expected: wk_learn + wk_known_fail PASS; OPSPEND asserts still RED (Task 3). Discovery phases still PASS (bit-clear path untouched). Report blob size vs 4096.

- [ ] **Step 7: commit.**
```bash
git add software/SDM/WORKEQU.S software/SDM/WORKTASK.S gateware/rev2/project_obscurus/worktask.mem
git commit -m "feat(farm): known-recipe fail roll (WKNOWN, not shared WCGO) + WOPLEARN"
```

---

### Task 3: FARMTASK reclaim + OPSPEND (green, rest)

**Files:**
- Modify: `software/SDM/FARMEQU.S`, `software/SDM/FARMTASK.S`

- [ ] **Step 1: addr-hi reclaim (prerequisite).** At each of the 4 sites (lines ~211-212, 259-260, 337-338, 432-433), the pattern is:
```
 LDA #0
 ADC #>FSEEDS      (or #>FCROPS)
 STA <somevar>
 ...
 LDY <somevar>
```
Read each site to find the exact `STA <var>` / `LDY <var>` pair it feeds, then replace the 3-instruction hi-byte computation with nothing and change the `LDY <var>` to `LDY #>FSEEDS` (or `#>FCROPS`) immediate. Add the comment `; hi const $02 - breaks if FSEEDS/FCROPS cross a page` at each. CAUTION: only remove the hi-byte computation; the LO-byte add (`CLC / ADC #<FSEEDS / STA <lovar>`) stays and feeds the A register for RDB/WRB. Do ONE site, `make farmtasksim`, confirm farmonly still fully green, then the next — these are surgical.

- [ ] **Step 2: verify reclaim saved space.**
```bash
make farmtasksim
wc -c software/SDM/FARMTASK.bin
```
Report the new size (was 1772; expect ~1735). farmonly must stay green (no behavior change).

- [ ] **Step 3: OPSPEND equate.** FARMEQU.S ops block after `OPADDC = $06`:
```
OPSPEND = $07
```

- [ ] **Step 4: OPSPEND dispatch.** FARMTASK DOMBOX (~line 177), change:
```
 CMP #OPADDC
 BEQ JADDC
 LDA #RERRBAD
```
to:
```
 CMP #OPADDC
 BEQ JADDC
 CMP #OPSPEND
 BEQ JSPEND
 LDA #RERRBAD
```
add trampoline:
```
JSPEND
 JMP CSPEND
```

- [ ] **Step 5: CSPEND handler.** Add near CADDC (TA0=lo, TA1=hi; cash>=amt → subtract, ROK; else RERRCASH). Uses the CBUY 16-bit-compare precedent:
```
* --- SPEND TA0=lo TA1=hi, RERRCASH if poor ---
CSPEND
 LDA #<FCASHL
 LDY #>FCASHL
 JSR RDB
 STA CSHL
 LDA #<FCASHH
 LDY #>FCASHH
 JSR RDB
 STA CSHH
* cash >= amount?
 LDA CSHH
 CMP TA1
 BCC SPPOOR
 BNE SPRICH
 LDA CSHL
 CMP TA0
 BCC SPPOOR
SPRICH
 LDA CSHL
 SEC
 SBC TA0
 STA CSHL
 LDA CSHH
 SBC TA1
 STA CSHH
 LDX CSHL
 LDA #<FCASHL
 LDY #>FCASHL
 JSR WRB
 LDX CSHH
 LDA #<FCASHH
 LDY #>FCASHH
 JSR WRB
 LDA #ROK
 JMP MBFIN
SPPOOR
 LDA #RERRCASH
 JMP MBFIN
```

- [ ] **Step 6: full farmonly green.**
```bash
make farmtasksim
make DESIGN=project_obscurus REV=rev2 sim PLUSARGS=+farmonly
```
Expected: ALL phases green — wk_learn, wk_known_fail, OPSPEND, plus everything prior. Report FARMTASK.bin size (must be < 1792). If over, the reclaim has more sites (grep `ADC #>` in FARMTASK.S).

- [ ] **Step 7: commit.**
```bash
git add software/SDM/FARMEQU.S software/SDM/FARMTASK.S gateware/rev2/project_obscurus/farmtask.mem
git commit -m "feat(farm): OPSPEND + addr-hi reclaim (37 B for the new op)"
```

---

### Task 4: full-suite gate

- [ ] `make DESIGN=project_obscurus REV=rev2 sim` (~10 min, timeout 700000). Zero FAIL. Non-farm failure = regression: STOP.

---

### Task 5: FARM.S — buy flow, market recipe row, RFAIL message

**Files:**
- Modify: `software/SDM/FARMEQU.S` (CVERNUM), `software/SDM/FARM.S`

Assemble (`make worktask && make farm`) after each step.

- [ ] **Step 1: CVERNUM bump.** FARMEQU.S: `CVERNUM = 1` → `CVERNUM = 2`.

- [ ] **Step 2: RPRICE table + new vars + strings.** FARM.S — vars near the others:
```
SHOPIDX DS 1
```
Data block near RECTAB:
```
* recipe prices = 2 x value, 16-bit lo/hi
RPRICEL DFB 56,72,96,112,152,160,160,184
 DFB 200,210,220,254
RPRICEH DFB 0,0,0,0,0,0,0,0
 DFB 0,0,0,0
```
Strings near the other workshop strings:
```
SRECIPE ASC "RECIPE "
 DFB 0
SBUYP ASC "BUY "
 DFB 0
SNEEDC ASC "NEED CASH"
 DFB 0
SALLOWN ASC "RECIPES: ALL OWNED"
 DFB 0
SLEARN ASC "LEARNED "
 DFB 0
SCFAIL ASC "CRAFT FAILED"
 DFB 0
```

- [ ] **Step 3: lowest-unowned helper.** Add a routine returning the lowest recipe idx whose DISC bit is clear in WDISCV, or $FF if all owned:
```
* === SHOPLOW: A = lowest unowned recipe idx
* (DISC bit clear in WDISCV), or $FF if none ===
SHOPLOW
 LDX #0
SHLP
 TXA
 JSR DISCSET
 BEQ SHFOUND
 INX
 CPX #12
 BNE SHLP
 LDA #$FF
 RTS
SHFOUND
 TXA
 RTS
* === DISCSET: A=idx -> Z=0 if owned, Z=1 if
* not. mask = 1<<(idx&7) vs WDISCV[idx>=8] ===
DISCSET
 STA NTMP
 AND #7
 TAX
 LDA #1
DSSHL
 CPX #0
 BEQ DSMK
 ASL
 DEX
 JMP DSSHL
DSMK
 LDX NTMP
 CPX #8
 BCS DSHI
 AND WDISCV
 RTS
DSHI
 AND WDISCV+1
 RTS
```
(DISCSET returns Z reflecting the masked bit: BNE = owned. SHOPLOW wants the FIRST clear, so `BEQ SHFOUND` on not-owned. Verify the WKDR_SHL render code uses the identical bit math — reuse the pattern, don't diverge.)

- [ ] **Step 4: market recipe row.** In MKDRAW, after the 4 crop rows and before the legend, draw the recipe row at row 16 (verify it's free in MKDRAW's layout):
```
* row 16: recipe shop line
 JSR WSYNC_STA
 JSR SHOPLOW
 STA SHOPIDX
 CMP #$FF
 BNE MKRSHOW
* all owned
 LDA #<SALLOWN
 STA MSGPTR
 LDA #>SALLOWN
 STA MSGPTR+1
 LDA #0
 STA PRCOL
 LDY #16
 JSR PRSTR
 JMP MKRDONE
MKRSHOW
 LDA #<SRECIPE
 STA MSGPTR
 LDA #>SRECIPE
 STA MSGPTR+1
 LDA #0
 STA PRCOL
 LDY #16
 JSR PRSTR
* recipe name at col 7
 LDX SHOPIDX
 LDA RNAML,X
 STA MSGPTR
 LDA RNAMH,X
 STA MSGPTR+1
 LDA #7
 STA PRCOL
 LDY #16
 JSR PRSTR
* price at col 14: "BUY nnn" if affordable
* else "NEED CASH"
 LDX SHOPIDX
 LDA RPRICEL,X
 STA T16
 LDA RPRICEH,X
 STA T16+1
 JSR CANAFFORD
 BCC MKRPOOR
 LDA #<SBUYP
 STA MSGPTR
 LDA #>SBUYP
 STA MSGPTR+1
 LDA #14
 STA PRCOL
 LDY #16
 JSR PRSTR
 LDX SHOPIDX
 LDA RPRICEL,X
 STA T16
 LDA RPRICEH,X
 STA T16+1
 LDX #18
 LDY #16
 JSR PRDEC3
 JMP MKRDONE
MKRPOOR
 LDA #<SNEEDC
 STA MSGPTR
 LDA #>SNEEDC
 STA MSGPTR+1
 LDA #14
 STA PRCOL
 LDY #16
 JSR PRSTR
MKRDONE
```
Add CANAFFORD (compares 16-bit T16 vs CASHL/CASHH already in FARM.S RAM from RDMKT):
```
* === CANAFFORD: C=1 if CASH >= T16 ===
CANAFFORD
 LDA CASHH
 CMP T16+1
 BCC CAFNO
 BNE CAFYES
 LDA CASHL
 CMP T16
 BCC CAFNO
CAFYES
 SEC
 RTS
CAFNO
 CLC
 RTS
```
(WSYNC_STA: confirm this is the lite re-sync that refreshes WDISCV/WSKILLV without full ring work — grep; if it doesn't exist use WSYNC. The render needs a fresh WDISCV mirror.)

- [ ] **Step 5: R key handler (refund-on-reject).** In MKKEY, add MK5 (R = $D2) before MKNONE:
```
MK4
 CMP #$D7 ; W -> workshop
 BNE MK5
 JSR WKENTER
 JMP MLOOP
MK5
 CMP #$D2 ; R buy recipe
 BNE MKNONE
 JSR SHOPBUY
 JSR RDMKT
 JSR MKDRAW
 JMP MLOOP
```
(MK4 currently ends `BNE MKNONE`; re-point it to MK5.) SHOPBUY:
```
* === SHOPBUY: buy lowest unowned recipe.
* debit-first, refund if WOPLEARN rejects ===
SHOPBUY
 JSR WSYNC_STA
 JSR SHOPLOW
 STA SHOPIDX
 CMP #$FF
 BNE SHB1
 RTS                ; all owned, no-op
SHB1
* affordable?
 LDX SHOPIDX
 LDA RPRICEL,X
 STA T16
 LDA RPRICEH,X
 STA T16+1
 JSR CANAFFORD
 BCS SHB2
 JSR CLRMSG
 LDA #<SNEEDC
 STA MSGPTR
 LDA #>SNEEDC
 STA MSGPTR+1
 LDA #0
 STA PRCOL
 LDY #21
 JSR PRSTR
 RTS
SHB2
* leg 1: farm OPSPEND(price)
 LDA #OPSPEND
 STA CMDOP
 LDX SHOPIDX
 LDA RPRICEL,X
 STA CMDA0
 LDA RPRICEH,X
 STA CMDA1
 LDA #0
 STA CMDA2
 JSR SENDCMD
 BCS SHB_TO
 CMP #ROK
 BNE SHB_POOR
* leg 2: workshop WOPLEARN(idx)
 LDA #WOPLEARN
 STA CMDOP2
 LDA SHOPIDX
 STA CMDA02
 LDA #0
 STA CMDA12
 STA CMDA22
 STA CMDA32
 JSR WSENDCMD
 BCS SHB_REFUND
 CMP #ROK
 BNE SHB_REFUND
* success: LEARNED <name>
 JSR CLRMSG
 LDA #<SLEARN
 STA MSGPTR
 LDA #>SLEARN
 STA MSGPTR+1
 LDA #0
 STA PRCOL
 LDY #21
 JSR PRSTR
 LDX SHOPIDX
 LDA RNAML,X
 STA MSGPTR
 LDA RNAMH,X
 STA MSGPTR+1
 LDA #8
 STA PRCOL
 LDY #21
 JSR PRSTR
 JSR WSYNC_STA
 RTS
SHB_REFUND
* WOPLEARN rejected after pay -> refund
 LDA #OPADDC
 STA CMDOP
 LDX SHOPIDX
 LDA RPRICEL,X
 STA CMDA0
 LDA RPRICEH,X
 STA CMDA1
 LDA #0
 STA CMDA2
 JSR SENDCMD
 JSR WSYNC_STA
 RTS
SHB_POOR
 JSR CLRMSG
 LDA #<SNEEDC
 STA MSGPTR
 LDA #>SNEEDC
 STA MSGPTR+1
 LDA #0
 STA PRCOL
 LDY #21
 JSR PRSTR
 RTS
SHB_TO
 JSR CLRMSG
 LDA #<SNORESP
 STA MSGPTR
 LDA #>SNORESP
 STA MSGPTR+1
 LDA #0
 STA PRCOL
 LDY #21
 JSR PRSTR
 RTS
```
(SENDCMD is the FARM-mailbox sender; it uses CMDOP/CMDA0-2. WSENDCMD is the workshop sender; CMDOP2/CMDA02-32. Confirm both contracts by re-reading; SHB_REFUND uses SENDCMD/OPADDC — the credit leg. Watch branch ranges across this long routine — invert+JMP if Merlin complains.)

- [ ] **Step 6: RFAIL message in WKCRAFT dispatch.** In WKCRAFT result branches (~3163), add an RFAIL case. Change:
```
 CMP #RRUIN
 BEQ WKCRAFT_RU
```
to also handle RFAIL:
```
 CMP #RRUIN
 BEQ WKCRAFT_RU
 CMP #RFAIL
 BEQ WKCRAFT_CF
```
add the branch (near WKCRAFT_RU):
```
WKCRAFT_CF
 JSR CLRMSG
 LDA #<SCFAIL
 STA MSGPTR
 LDA #>SCFAIL
 STA MSGPTR+1
 LDA #0
 STA PRCOL
 LDY #21
 JSR PRSTR
 JMP WKCRAFT_UPD
```
(FARMEQU must expose RFAIL=$E9 to FARM.S — it's in WORKEQU, but FARM.S PUTs FARMEQU not WORKEQU. Add `RFAIL = $E9` to FARMEQU.S too, with a comment that it mirrors WORKEQU.)

- [ ] **Step 7: assemble + commit.**
```bash
make worktask && make farm
git commit -am "feat(farm): recipe shop - market R buys lowest, refund-on-reject, CRAFT FAILED msg"
```

---

### Task 6: deposit crop-picker (parked polish)

**Files:**
- Modify: `software/SDM/FARM.S`

The D-key deposit currently uses SELCROP (set on other screens) — hidden state. Add an explicit `DEP 1-4?` prompt.

- [ ] **Step 1: QPCROP var + QTYPROMPT param.** Add var:
```
QPCROP DS 1
```
In QTYPROMPT SQGO (~1775), change:
```
 LDA SELCROP
 STA CMDA0
```
to:
```
 LDA QPCROP
 STA CMDA0
```
Update the existing callers to set QPCROP=SELCROP before JSR QTYPROMPT: market MK2 (buy) and MK3 (sell) handlers — add `LDA SELCROP / STA QPCROP` right before each `JSR QTYPROMPT`. (Grep all QTYPROMPT call sites; there are 3 — market buy, market sell, workshop deposit.)

- [ ] **Step 2: deposit crop-pick prompt.** Rewrite WKDEPOS (~3231):
```
* === WKDEPOS: D key - pick crop then qty ===
WKDEPOS
 JSR CLRMSG
 LDA #<SDEPC
 STA MSGPTR
 LDA #>SDEPC
 STA MSGPTR+1
 LDA #0
 STA PRCOL
 LDY #21
 JSR PRSTR
WKDPK
 LDA KBD
 BPL WKDPK
 STA KBDSTR
 CMP #$9B ; ESC cancel
 BEQ WKDPX
 CMP #$B1
 BCC WKDPK
 CMP #$B5
 BCS WKDPK
 SEC
 SBC #$B1
 STA QPCROP
* echo crop letter then run qty prompt
 LDA #OPWDRAW
 STA CMDOP
 LDA #<SQTYWD
 STA MSGPTR
 LDA #>SQTYWD
 STA MSGPTR+1
 JSR QTYPROMPT
 RTS
WKDPX
 JSR CLRMSG
 RTS
```
String:
```
SDEPC ASC "DEP 1-4? (CROP)"
 DFB 0
```

- [ ] **Step 3: WKDEPOK uses QPCROP.** WKDEPOK currently reads CMDA0 (which QTYPROMPT set from QPCROP) — verify it stages CMDA02 from CMDA0; since QTYPROMPT now sets CMDA0=QPCROP, WKDEPOK is correct unchanged. Confirm by re-reading WKDEPOK.

- [ ] **Step 4: assemble + commit.**
```bash
make farm
git commit -am "feat(farm): deposit crop-picker - DEP 1-4? prompt, no hidden SELCROP"
```

---

### Task 7: farm-screen workshop widget (parked polish)

**Files:**
- Modify: `software/SDM/FARM.S`

Show station status on the farm screen so the player knows crafts are cooking without switching.

- [ ] **Step 1: widget in HUDDRAW.** After the SEED block (row 20, before/after CASH — find free cols 22-27; SEED name ends ~col 18, CASH starts col 29), add a 5-char workshop indicator drawn from STREC station states:
```
* row 20 cols 23-26: workshop widget W:xx
* x per station: . idle, * cooking, ! done
 LDY #20
 LDA GRLO,Y
 STA LINEP
 LDA GRHI,Y
 STA LINEP+1
 LDA #$D7 ; 'W'
 LDY #23
 STA (LINEP),Y
 LDA #$BA ; ':'
 LDY #24
 STA (LINEP),Y
* station 0 glyph col 25
 LDX #0
 JSR WGGLYPH
 LDY #25
 STA (LINEP),Y
* station 1 glyph col 26
 LDX #4
 JSR WGGLYPH
 LDY #26
 STA (LINEP),Y
```
Add WGGLYPH (X = STREC offset of station's STATE byte → A = glyph):
```
* === WGGLYPH: X=STREC state offset -> A glyph
* 0 idle '.', 1 cooking '*', 2 done '!'(inv) ===
WGGLYPH
 LDA STREC,X
 CMP #1
 BEQ WGGCK
 CMP #2
 BEQ WGGDN
 LDA #$AE ; '.'
 RTS
WGGCK
 LDA #$AA ; '*'
 RTS
WGGDN
 LDA #$A1 ; '!' (normal video; use $21 for inverse if desired)
 RTS
```
(STREC layout: station 0 STATE at offset 0, station 1 STATE at offset 4 — verify against WSYNC_STA's STREC fill. HUDDRAW must run after WSYNC so STREC is current — confirm the farm-screen draw path calls WSYNC; if not, the widget shows stale state until an event. Acceptable, but note it.)

- [ ] **Step 2: refresh widget on WEVDONE.** In DRAIN2/WEVDISP (the bank-33 ring handler), the WEVDONE case already re-reads STREC + redraws on SCREEN==2. Add: when SCREEN==0 (farm), call HUDDRAW (or a lighter widget-only redraw) so the `!` appears live. Find WEVDISP's WEVDONE branch and add the SCREEN==0 → HUDDRAW path. Optional BELL: `JSR $FBDD` (BELL) once when a station transitions to done — nice "dinner ready" cue from the field.

- [ ] **Step 3: assemble + commit.**
```bash
make farm
git commit -am "feat(farm): farm-screen workshop widget - station glyphs + done bell"
```

---

### Task 8: disk + gate + handoff

- [ ] **Step 1:** `make worktask && make farm && make sdmdisk`. Confirm FARM grew, .po catalog shows it.
- [ ] **Step 2:** Full suite `make DESIGN=project_obscurus REV=rev2 sim` — zero FAIL.
- [ ] **Step 3:** Handoff append (after inc-3 bullet):
```
- **v2 recipe-shop SHIPPED (branch farm-v2)**: dual-road crafting -
  market R buys lowest-unowned recipe (price=2xvalue, debit-first +
  refund-on-WOPLEARN-reject); discovery gamble unchanged; KNOWN recipes
  now roll a fail curve (FAILBASE[r] - skill/2 floor 8, RFAIL=$E9 'CRAFT
  FAILED', pinned to WKNOWN branch NOT shared WCGO). New ops OPSPEND=7
  (farm, after 37 B addr-hi reclaim), WOPLEARN=5 (workshop). CVER=2
  (re-seeds bank 33 on deploy - pantry/skill/recipes reset). Plus polish:
  deposit DEP 1-4? crop-picker (QPCROP), farm-screen workshop widget
  (W:.. station glyphs + done bell). Deploy: ctrl-reset + BRUN, farm
  world survives. Next: increment 4 = world events (FARMTASK headroom
  ~57 B after reclaim).
```
- [ ] **Step 4:** Commit handoff.
- [ ] **Step 5: bench checklist (tell user):**
  1. Ctrl-Reset + BRUN. Workshop resets (CVER=2): skill 0, no recipes, empty pantry. Farm crops survive.
  2. Harvest + sell to build cash. M → market → recipe row shows `RECIPE BREAD BUY 56`.
  3. R → `LEARNED BREAD` (if cash≥56); book now shows BREAD; row advances to SALAD.
  4. Can't afford → `NEED CASH`. All owned → `RECIPES: ALL OWNED`.
  5. W → D → `DEP 1-4?` → pick crop → qty → pantry up (no more hidden-SELCROP error).
  6. Craft owned BREAD repeatedly at low skill → mix of `COOKING` and `CRAFT FAILED` (distinct from discovery `RUINED!`); skill climbs, failures shrink.
  7. Mix an unowned combo → discovery gamble: `RUINED!` or learn-free-cook.
  8. Farm screen row 20 shows `W:.*` widget; hear the bell when a craft finishes while you farm.
  9. `/obs-screenshot` market recipe row + farm widget.

---

## Self-review notes

- Spec coverage: shop ladder + R + cash gate ✓ (T5 SHOPBUY/SHOPLOW), no skill gate ✓ (none added), price=2×value ✓ (RPRICE), refund-on-reject ✓ (T5 SHB_REFUND), display=PRICE ✓ (BUY nnn), discovery unchanged ✓ (bit-clear path untouched, T2), known-fail roll ✓ (T2 WKNOWN — pinned off WCGO per mandate), WCGO mandate ✓ (Step 4 explicit), LFSR polarity comment ✓ (T2 inline), FAILBASE/FLOOR data ✓, WOPLEARN double-learn guard ✓ (T2 WCLEARN), OPSPEND+RERRCASH ✓ (T3), 37 B reclaim as prereq ✓ (T3 S1), CVER=2 destructive ✓ (T5 S1 + handoff), RFAIL message ✓ (T5 S6), testbench distribution-not-pinned ✓ (T1 S3), parked polish ✓ (T6 picker, T7 widget).
- Riskiest: T2 Step 4 (WKNOWN insertion — the mandate; reused MVAL/MTMP/RADL must not leak into WCGO), T5 Step 5 (long SHOPBUY, branch ranges), T1 Step 3 (the placeholder no-op line MUST be replaced with a real pantry-consumed assert).
- Type consistency: SHOPLOW/DISCSET/CANAFFORD/SHOPBUY/SHOPIDX/QPCROP/WGGLYPH/WKNOWN/WCLEARN/RFAIL/OPSPEND/WOPLEARN/FAILBASE/RPRICEL/RPRICEH used consistently. WSENDCMD/SENDCMD contracts (CMDOP2 vs CMDOP) kept distinct.
- Verify-at-exec: WSYNC_STA existence (T5 S4 note — fall back to WSYNC); MKDRAW row 16 free; STREC station-state offsets (0 and 4); HUDDRAW-after-WSYNC for live widget.
