# Farm v2 Increment 3: WORKSHOP Task + Crafting Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Second live coproc task (WORKSHOP, skill 3, SDRAM bank 33) running Morrowind-style crafting — hidden recipe book, discovery RNG gated by a craft-skill stat, timed stations that keep cooking while you farm — plus the //e workshop screen.

**Architecture:** Coproc-first TDD. First a pure refactor (EVLIB scratch parameterization + PORTLIB extraction, proven byte-identical). Then WORKTASK.S (skill 3, ORG $0300, 768 B cap) against new tb phases: deposit → craft → RUINED → forced discovery → cook → WEVDONE → collect, dual-task liveness, dual-task reset recovery. FARMTASK gains the two //e-bus ops (OPWITHDRAW, OPADDCASH). Then the //e: bank-33 seeding + CVER, SPAWNW, workshop screen behind the existing screen manager, crafting UX. The //e mediates ALL cross-bank transfers (debit first, credit second — loss never dup).

**Tech Stack:** Merlin32 6502 (single-space format, ASCII, ≤50-char lines), iverilog -g2005 tb, `make` targets (new: worktasksim analogous to farmtasksim), gates `+farmonly` → rename-scope note below, full suite ~9 min.

**Spec:** `docs/superpowers/specs/2026-06-11-farm-v2-sidework-design.md`. MANDATORY pre-reading: "Host BRAM-write quiesce invariant" (incl. CP_FSTAT restore_done rule), "EVLIB scratch parameterization", "Single-writer + //e-as-bus", "WORKSHOP task" + "Crafting / recipe model" sections.

**Deploy (tell the user):** Ctrl-Reset then BRUN new FARM. Farm world (bank 32) SURVIVES — re-entry sees valid farm SIG, seeds only bank 33 (invalid SIG there), respawns both tasks. No power cycle needed.

---

## Locked design decisions

### Hardware/kernel constraints honored
- WORKSHOP ORG $0300, cap 768 B ($0300-$05FF; FARMTASK starts $0600). Makefile WORKTASK_MAXLEN := 768 with size guard (mirror the FARMTASK rules).
- Host BRAM loads ONLY at entry/recovery when no task runs; after any reset poll CP_FSTAT ($C0CF on //e; tb reg 4'hF) bit 1 restore_done first. No single-task respawn — recovery reloads BOTH blobs, respawns BOTH.
- One-task-dead-while-other-busy (CP_ACTIVE nonzero) on //e = existing dead-menu path; message gains "OR CTRL-RESET". The //e cannot soft-reset the kernel; that's acceptable (rare path).
- Workshop scratch page $0D00 (spec): EVLIB scratch $0D00-$0D05, task scratch $0D10+. Farm keeps $0E00-$0E3F.
- Slot use: farm = slot 0 / skill 2 / TABLE[2]=$0600; workshop = slot 1 / skill 3 / TABLE[3]=$0300 ($0206-7).

### Bank 33 layout (SIG "WK")
| Addr | Content |
|---|---|
| $0000-1 | SIG $57,$4B |
| $0002 / $0003 | SEQCTR / HEAD |
| $0004 | WHBEAT heartbeat |
| $0005 | CVER content version (=1 this increment, //e-seeded) |
| $0100-$01FF | RING 64×4 (EVLIB verbatim) |
| $0200-$0207 | MAILBOX: FLAG OP I0 I1 I2 I3 RES RES1 |
| $0210-$0213 | PANTRY[4] (per-crop ingredient counts) |
| $0214 | SKILL (craft skill, clamps $FF) |
| $0215-$0216 | DISC bitmap (16 bits, 12 recipes) |
| $0217 | MODE (bit 0 = BOOM: collect value ×2) |
| $0220-$0227 | 2 stations × 4 B: STATE(0 idle/1 cooking/2 done), RECIPE, TIMER, pad |
| $0300-$035F | recipe table, //e-seeded: 12 × 8 B |

Recipe entry (8 B): I0 I1 I2 I3 (crop ids sorted ascending, $FF pad), TIME (station ticks), VALUE, RARITY, pad. Product id = table index.

### Recipe book (W=0 C=1 B=2 P=3; values ≤127 so BOOM ×2 fits a byte)
| idx | name | combo | TIME | VALUE | RARITY |
|---|---|---|---|---|---|
| 0 | BREAD | 0,0 | 4 | 28 | 0 |
| 1 | SALAD | 0,1 | 2 | 36 | 0 |
| 2 | SOUP | 1,1 | 4 | 48 | 0 |
| 3 | MUFFIN | 0,2 | 4 | 56 | 1 |
| 4 | CAKE | 0,0,2 | 8 | 76 | 1 |
| 5 | JAM | 2,2 | 4 | 80 | 1 |
| 6 | ROAST | 0,3 | 8 | 80 | 1 |
| 7 | STEW | 1,3 | 8 | 92 | 1 |
| 8 | CHOWDER | 1,1,3 | 12 | 100 | 2 |
| 9 | PIE | 0,2,3 | 12 | 105 | 2 |
| 10 | WINE | 2,2,2 | 12 | 110 | 2 |
| 11 | FEAST | 0,1,2,3 | 20 | 127 | 3 |

Names live ONLY in FARM.S (display); the blob never needs them.

### Crafting semantics (WORKTASK)
- OPCRAFT(I0-I3): //e sends ingredients SORTED ascending, $FF-padded (//e sorts — saves blob bytes). Flow: find free station (else RERRFULL $E7) → count needed per crop, verify pantry (else RERRCROP $E5) → scan table for combo match:
  - no match: consume ingredients, SKILL++ (clamp $FF), result RRUIN = $E8 ("RUINED!").
  - match + discovered bit set: consume, station ← (cooking, idx, TIME), ROK.
  - match + undiscovered: roll LFSR byte < threshold, threshold = 40 + SKILL/2 − RARITY×16 (8-bit, clamp ≥0 — high rarity impossible at low skill, intended). Pass: set DISC bit, consume, cook, ROK. Fail: consume, SKILL++, RRUIN.
  - SKILL++ on every attempt (match or dud), success or fail, clamp $FF.
- Station tick: STAD divider (24-bit, FSIM-switched like MKTD; hardware ~5 s/tick, sim tiny). Each expiry decrements every cooking station's TIMER; 0 → STATE=2 (done), PUTEV WEVDONE(station, recipe).
- OPCOLLECT(I0=station): STATE==2 → RES1 = VALUE (×2 clamped $FF if MODE bit 0), STATE=0, ROK. Else RERRBAD. The //e reads the station record from the bank for the recipe id BEFORE collecting (//e-as-bus precedent — no extra result byte needed).
- OPDEPOSIT(I0=crop, I1=qty): crop<4, qty>0, PANTRY[crop]+=qty clamp $FF, ROK.
- OPMODE(I0=flags): store to MODE ($0217). ROK.
- OPSTAT (op 0): ROK (the //e probe uses it like the farm's).
- Heartbeat: WHBEAT++ every loop pass (same WRB pattern as FARMTASK).
- LFSR: own 16-bit Galois (NEXTRND pattern), seed $7C,$5A; stepped every loop pass.
- Result codes reuse FARMEQU values; new RRUIN = $E8 (define in WORKEQU and FARMEQU comment block).

### FARMTASK additions (//e bus ops)
- OPWITHDRAW = op 5: TA0=crop (<4 else RERRBAD), TA1=qty (>0; FCROPS[crop] ≥ qty else RERRCROP) → subtract, ROK.
- OPADDCASH = op 6: cash += TA0 (lo) + TA1<<8, clamp $FFFF, ROK.
- Headroom: 175 B free; these need ~90 B. If the guard trips, apply the reviewer-verified reclaim: per-crop addr hi bytes are constant $02 — replace `LDA #0 / ADC #>FSEEDS / STA tmp` + `LDY tmp` patterns with direct `LDY #>FSEEDS` immediates in CPLANT/CBUY/CHARV/CSELL (~37 B).

### //e additions
- SCREEN ids: 0 farm, 1 market, 2 workshop. W key ($D7) from farm AND market → workshop; ESC behavior on workshop: clears MIX if nonempty, else returns to farm.
- Workshop screen (text): row 0 title `WORKSHOP  SKILL nnn`; row 2 pantry `PANTRY W nnn C nnn B nnn P nnn`; rows 4-5 stations (`1 IDLE` / `1 COOKING PIE` / `1 DONE PIE - PRESS C1`); rows 7-18 recipe book, 2 columns × 6 rows: known = `BREAD WW 28`, unknown = `?????`; row 19 `MIX: W C _ _`; row 21 messages/prompts; row 23 legend `1-4 MIX RTN CRAFT D DEP C1 C2 ESC`.
- Craft UX: keys 1-4 push crop letters into a 4-slot MIX buffer (display row 19); RETURN sorts ($FF-pads) and sends OPCRAFT, clears MIX, refreshes; ESC clears MIX first.
- D: `DEP QTY:` prompt (QTYPROMPT reuse, CMDOP path is farm OPWITHDRAW) for SELCROP → on ROK, workshop OPDEPOSIT same crop/qty (debit first, credit second). On deposit failure after withdraw succeeds, crops are lost (spec-accepted; show ERR).
- C then 1 or 2: read station record from bank 33; if done: OPCOLLECT(st) → on ROK take RES1 → farm OPADDCASH(RES1, 0) → message `SOLD <name> +nnn`.
- WEVDONE handling: SCREEN==2 → redraw stations+book region; else row-21 message `CRAFT DONE!`.
- Second ring reader: bank-33 TAIL2/EXPSEQ2 mirror of DRAIN with bank-33 reads; drained in MLOOP right after the farm DRAIN, all screens.
- Bank-33 access helpers: FARM.S FRD/FWR hardwire GBANK(=32); add WFRD/WFWR (bank 33) — copies with the bank constant changed (FARMEQU gains `WBANK = 33`).
- COLDST: seeds bank 33 (zeros $0002-$0227 fields, recipe table 12×8 from RECTAB data block, CVER=1) and bank 32 as today; loads BOTH blobs; spawns slot 0 + slot 1; writes BOTH SIGs LAST (33 then 32).
- Re-entry: farm probe as today PLUS workshop probe (SIG + OPSTAT). All alive → RESYNC + workshop ring sync (TAIL2/EXPSEQ2 stable-pair read) + workshop state re-read. Dead + CP_ACTIVE==0 → restore-wait (CP_FSTAT bit 1 poll, ~30 s timeout via 16-bit counter — NOT a ZP counter) → reload+respawn BOTH; seed bank 33 first if its SIG invalid; never touch bank-32 world when its SIG valid. CVER mismatch (bank 33 $0005 ≠ FARM's CVERNUM=1) with valid SIG → same quiesce path, re-seed 33.
- SPAWNW: mirror of SPAWNT (read SPAWNT in CPLIB/FARM.S first): CP_LADDR=$0300, stream WORKTASKB bytes, TABLE[3]=$00,$03 at $0206, kernel mailbox slot 1 (skill 3, budget 0), CP_RING slot 1.

### Naming
New files: `software/SDM/WORKEQU.S`, `software/SDM/WORKTASK.S`, `software/SDM/PORTLIB.S`. Makefile targets `worktask`, `worktasksim` mirroring farm; sim mem `gateware/rev2/project_obscurus/worktask.mem`; //e embed `WORKTASKB.S` (sed-generated like FARMTASKB).

---

### Task 1: PORTLIB + EVLIB scratch refactor (byte-identical proof)

**Files:**
- Create: `software/SDM/PORTLIB.S`
- Modify: `software/SDM/EVLIB.S`, `software/SDM/FARMEQU.S`, `software/SDM/FARMTASK.S`

- [ ] **Step 1:** Save baseline: `cp gateware/rev2/project_obscurus/farmtask.mem /tmp/ftm_before.mem`

- [ ] **Step 2:** Move EVLIB scratch equates to the includer. DELETE from `EVLIB.S` lines 6-11 (`EVTYPE = $0E00` … `EVRLO = $0E05`) and change its header comment line 5 to:
```
* ABS SCRATCH + GBANK/FSEQC/FHEAD/FRING COME
* FROM THE INCLUDER'S EQU FILE (NO ZP)
```
ADD to `FARMEQU.S` (after the divider block, before the SDRAM-port equates):
```
* EVLIB SCRATCH (FARM TASK INSTANCE)
EVTYPE = $0E00
EVP0 = $0E01
EVP1 = $0E02
EVHEAD = $0E03
EVSEQ = $0E04
EVRLO = $0E05
```

- [ ] **Step 3:** Extract RDB/WRB into `PORTLIB.S`:
```
* PORTLIB.S - COPROC SDRAM BYTE HELPERS
* PUT-INCLUDE. NO ORG. BANK = INCLUDER'S
* GBANK SYMBOL. SEI/CLI PER BURST (WINDOW
* CONVENTION - SEE SPEC).
* RDB: read GBANK byte addr A(lo) Y(hi) -> A
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
```
In `FARMTASK.S` replace the RDB/WRB routine bodies (keep their position — the `* === PORT HELPERS ===` block) with:
```
* === PORT HELPERS (PORTLIB) ===
 PUT PORTLIB
```

- [ ] **Step 4:** Rebuild + prove identical:
```bash
cd /Users/hambook/Development/project_byte_hamr && make farmtasksim
cmp /tmp/ftm_before.mem gateware/rev2/project_obscurus/farmtask.mem
```
cmp MUST be silent. If it differs, the PUT landed at a different offset — fix placement until identical. Also `make farm` must assemble clean (FARM.S includes FARMEQU; new equates harmless).

- [ ] **Step 5:** Commit:
```bash
git add software/SDM/PORTLIB.S software/SDM/EVLIB.S software/SDM/FARMEQU.S software/SDM/FARMTASK.S
git commit -m "refactor(farm): PORTLIB extract + EVLIB scratch to includer - byte-identical"
```

---

### Task 2: tb workshop phases (red)

**Files:**
- Modify: `gateware/rev2/project_obscurus/project_obscurus_tb.v`

- [ ] **Step 1:** Add image + length scan for the workshop blob next to the farm ones (~line 107):
```verilog
    reg [7:0] wkimg [0:1023];
    initial $readmemh("worktask.mem", wkimg);
    integer WKLEN;
    initial begin
        #1; WKLEN = 0;
        begin : wklen_scan
            integer wi;
            for (wi = 0; wi < 1024; wi = wi + 1)
                if (wkimg[wi] !== 8'hxx) WKLEN = wi + 1;
        end
    end
```
(worktask.mem won't exist until Task 3 — create a 1-byte placeholder so compilation works: `printf '00\n' > gateware/rev2/project_obscurus/worktask.mem` and note it gets overwritten by the build.)

- [ ] **Step 2:** Helpers (next to farm_load/farm_cmd):
```verilog
    task wk_load;
        integer i;
    begin
        wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h03);      // CP_LADDR = $0300
        for (i=0; i<WKLEN; i=i+1) load_byte(wkimg[i]);
        wr_reg(4'h9, 8'h06); wr_reg(4'hA, 8'h02);      // TABLE[3] @ $0206
        load_byte(8'h00); load_byte(8'h03);            // vector = $0300
    end endtask

    // recipe entry poke: 8 bytes at $0300+idx*8 in bank 33
    task wk_recipe(input [7:0] idx, input [7:0] i0, input [7:0] i1,
                   input [7:0] i2, input [7:0] i3, input [7:0] tm,
                   input [7:0] val, input [7:0] rar);
        reg [15:0] a;
    begin
        a = 16'h0300 + idx*8;
        sdram_write(10'd33, a+0, i0); sdram_write(10'd33, a+1, i1);
        sdram_write(10'd33, a+2, i2); sdram_write(10'd33, a+3, i3);
        sdram_write(10'd33, a+4, tm); sdram_write(10'd33, a+5, val);
        sdram_write(10'd33, a+6, rar); sdram_write(10'd33, a+7, 8'h00);
    end endtask

    task wk_init;       // bank 33 cold seed (mirrors //e COLDST)
        integer i;
    begin
        sdram_write(10'd33, 16'h0000, 8'h57);          // SIG 'W'
        sdram_write(10'd33, 16'h0001, 8'h4B);          // SIG 'K'
        sdram_write(10'd33, 16'h0002, 8'h00);          // SEQCTR
        sdram_write(10'd33, 16'h0003, 8'h00);          // HEAD
        sdram_write(10'd33, 16'h0004, 8'h00);          // WHBEAT
        sdram_write(10'd33, 16'h0005, 8'h01);          // CVER
        sdram_write(10'd33, 16'h0200, 8'h00);          // FLAG
        for (i=0; i<4;  i=i+1) sdram_write(10'd33, 16'h0210+i, 8'h00); // pantry
        sdram_write(10'd33, 16'h0214, 8'h00);          // SKILL
        sdram_write(10'd33, 16'h0215, 8'h00);          // DISC lo
        sdram_write(10'd33, 16'h0216, 8'h00);          // DISC hi
        sdram_write(10'd33, 16'h0217, 8'h00);          // MODE
        for (i=0; i<8;  i=i+1) sdram_write(10'd33, 16'h0220+i, 8'h00); // stations
        // 3 recipes are enough for the tb (full book is //e content):
        wk_recipe(8'd0, 8'd0, 8'd0, 8'hFF, 8'hFF, 8'd4, 8'd28, 8'd0); // BREAD
        wk_recipe(8'd5, 8'd2, 8'd2, 8'hFF, 8'hFF, 8'd4, 8'd80, 8'd1); // JAM
        wk_recipe(8'd11, 8'd0, 8'd1, 8'd2, 8'd3, 8'd20, 8'd127, 8'd3); // FEAST
        // remaining entries = $FF terminator pattern
        for (i=1; i<5;  i=i+1) wk_recipe(i[7:0], 8'hFE, 8'hFE, 8'hFE, 8'hFE, 8'd1, 8'd0, 8'd0);
        for (i=6; i<11; i=i+1) wk_recipe(i[7:0], 8'hFE, 8'hFE, 8'hFE, 8'hFE, 8'd1, 8'd0, 8'd0);
    end endtask

    task wk_cmd(input [7:0] op, input [7:0] a0, input [7:0] a1,
                input [7:0] a2, input [7:0] a3,
                output [7:0] res, output [7:0] res1, output ok);
        integer t; reg [7:0] f;
    begin
        sdram_write(10'd33, 16'h0201, op);
        sdram_write(10'd33, 16'h0202, a0);
        sdram_write(10'd33, 16'h0203, a1);
        sdram_write(10'd33, 16'h0204, a2);
        sdram_write(10'd33, 16'h0205, a3);
        sdram_write(10'd33, 16'h0200, 8'h01);          // FLAG last
        ok = 0; res = 8'hFF; res1 = 8'hFF;
        for (t=0; t<5000 && !ok; t=t+1) begin
            sdram_read(10'd33, 16'h0200, f);
            if (f == 8'h00) ok = 1;
        end
        if (ok) begin
            sdram_read(10'd33, 16'h0206, res);
            sdram_read(10'd33, 16'h0207, res1);
        end else $display("wk_cmd timeout op=%02X", op);
    end endtask
```
(Dud-combo scan note: unused entries use $FE ingredients — never matches a real combo since crops are 0-3/$FF.)

- [ ] **Step 3:** Workshop phase block. Insert AFTER the existing farm soft-reset survival block (before the final `if (errors==0)`):
```verilog
        // ===== WORKSHOP: 2nd task, craft pipeline =====
        begin : wk_m1
        reg [7:0] r, r1; reg ok; reg [7:0] h1, h2; integer t;
        $display("--- WORKSHOP task tests ---");
        // (the farm reset phase just respawned farm into slot 0)
        wk_init;
        wk_load;
        stage_mbox(2'd1, 8'd3, 8'd0, 8'd0);   // slot 1, skill 3
        ring(2'd1);
        wk_cmd(8'h00, 8'h00, 8'h00, 8'h00, 8'h00, r, r1, ok);   // OPSTAT
        if (!ok || r!==8'h01) begin errors=errors+1; $display("FAIL wk STATUS r=%h ok=%b", r, ok); end
        else $display("PASS wk STATUS (dual task live)");
        // dual heartbeats advance
        sdram_read(10'd33, 16'h0004, h1);
        sdram_read(10'd32, 16'h0004, r);
        begin : wkbeat
        reg beat0, beat1;
        beat0 = 0; beat1 = 0;
        for (t = 0; t < 5 && !(beat0 && beat1); t = t + 1) begin
            repeat (200000) @(posedge clk100);
            sdram_read(10'd33, 16'h0004, h2);
            if (h2 !== h1) beat1 = 1;
            sdram_read(10'd32, 16'h0004, r1);
            if (r1 !== r) beat0 = 1;
        end
        if (!beat1) begin errors=errors+1; $display("FAIL wk heartbeat stuck"); end
        if (!beat0) begin errors=errors+1; $display("FAIL farm heartbeat stuck w/ 2 tasks"); end
        if (beat0 && beat1) $display("PASS dual heartbeats");
        end
        // deposit via //e-bus pattern: poke farm crops, withdraw, deposit
        sdram_write(10'd32, 16'h0226, 8'd10);            // wheat crops = 10 (rig poke)
        farm_cmd(8'h05, 8'd0, 8'd4, 8'h00, r, ok);       // OPWITHDRAW wheat 4
        if (!ok || r!==8'h01) begin errors=errors+1; $display("FAIL withdraw r=%h", r); end
        sdram_read(10'd32, 16'h0226, r);
        if (r!==8'd6) begin errors=errors+1; $display("FAIL crops=%h want 06", r); end
        wk_cmd(8'h01, 8'd0, 8'd4, 8'h00, 8'h00, r, r1, ok); // OPDEPOSIT wheat 4
        if (!ok || r!==8'h01) begin errors=errors+1; $display("FAIL deposit r=%h", r); end
        sdram_read(10'd33, 16'h0210, r);
        if (r!==8'd4) begin errors=errors+1; $display("FAIL pantry=%h want 04", r); end
        else $display("PASS withdraw->deposit bus transfer");
        // dud combo: (0,0,0,FF) not in table -> RUINED, consumes 3, skill 1
        wk_cmd(8'h02, 8'd0, 8'd0, 8'd0, 8'hFF, r, r1, ok);
        if (!ok || r!==8'hE8) begin errors=errors+1; $display("FAIL dud r=%h want E8", r); end
        sdram_read(10'd33, 16'h0210, r);
        if (r!==8'd1) begin errors=errors+1; $display("FAIL pantry post-dud=%h want 01", r); end
        sdram_read(10'd33, 16'h0214, r);
        if (r!==8'd1) begin errors=errors+1; $display("FAIL skill=%h want 01", r); end
        else $display("PASS dud combo ruined + skill up");
        // refill pantry, force discovery (SKILL=$FF -> threshold maxed)
        wk_cmd(8'h01, 8'd0, 8'd3, 8'h00, 8'h00, r, r1, ok);  // deposit 3 more wheat (rig: no farm debit needed for unit test)
        sdram_write(10'd33, 16'h0214, 8'hFF);                // rig: maxed skill
        wk_cmd(8'h02, 8'd0, 8'd0, 8'hFF, 8'hFF, r, r1, ok);  // BREAD
        if (!ok || r!==8'h01) begin errors=errors+1; $display("FAIL craft r=%h", r); end
        sdram_read(10'd33, 16'h0220, r);                     // station0 STATE
        if (r!==8'h01) begin errors=errors+1; $display("FAIL station state=%h want 01", r); end
        sdram_read(10'd33, 16'h0215, r);                     // DISC lo bit 0
        if (r[0]!==1'b1) begin errors=errors+1; $display("FAIL disc bit=%h", r); end
        else $display("PASS discovery + cooking");
        // wait for WEVDONE (FSIM station divider is tiny)
        begin : wkdone
        reg [7:0] st; integer w;
        st = 8'h01; w = 0;
        while (st !== 8'h02 && w < 200) begin
            repeat (10000) @(posedge clk100);
            sdram_read(10'd33, 16'h0220, st); w = w + 1;
        end
        if (st!==8'h02) begin errors=errors+1; $display("FAIL station never done"); end
        end
        // drain wk ring: expect WEVDONE type 5, p0=station0, p1=recipe0
        begin : wkring
        reg [7:0] h, rs, rt, rp0, rp1; integer g;
        sdram_read(10'd33, 16'h0003, h);
        if (h == 8'h00) begin errors=errors+1; $display("FAIL wk ring empty"); end
        else begin
            sdram_read(10'd33, 16'h0100, rs);
            sdram_read(10'd33, 16'h0101, rt);
            sdram_read(10'd33, 16'h0102, rp0);
            sdram_read(10'd33, 16'h0103, rp1);
            if (rt!==8'h05 || rp0!==8'h00 || rp1!==8'h00) begin errors=errors+1;
                $display("FAIL WEVDONE %h %h %h", rt, rp0, rp1); end
            else $display("PASS WEVDONE(station0, BREAD)");
        end
        end
        // collect: value 28; then BOOM mode doubles JAM (80 -> 160)
        wk_cmd(8'h03, 8'd0, 8'h00, 8'h00, 8'h00, r, r1, ok);
        if (!ok || r!==8'h01 || r1!==8'd28) begin errors=errors+1;
            $display("FAIL collect r=%h r1=%h want 01,1C", r, r1); end
        else $display("PASS collect BREAD value 28");
        wk_cmd(8'h04, 8'h01, 8'h00, 8'h00, 8'h00, r, r1, ok);  // OPMODE BOOM
        wk_cmd(8'h01, 8'd2, 8'd2, 8'h00, 8'h00, r, r1, ok);    // deposit 2 berries
        wk_cmd(8'h02, 8'd2, 8'd2, 8'hFF, 8'hFF, r, r1, ok);    // JAM (skill FF)
        if (!ok || r!==8'h01) begin errors=errors+1; $display("FAIL jam craft r=%h", r); end
        begin : wkdone2
        reg [7:0] st; integer w;
        st = 8'h01; w = 0;
        while (st !== 8'h02 && w < 200) begin
            repeat (10000) @(posedge clk100);
            sdram_read(10'd33, 16'h0220, st); w = w + 1;
        end
        end
        wk_cmd(8'h03, 8'd0, 8'h00, 8'h00, 8'h00, r, r1, ok);
        if (!ok || r1!==8'd160) begin errors=errors+1; $display("FAIL boom collect r1=%h want A0", r1); end
        else $display("PASS BOOM collect 160");
        // OPADDCASH lands on the farm side (//e bus credit leg)
        sdram_read(10'd32, 16'h0220, h1); sdram_read(10'd32, 16'h0221, h2);
        farm_cmd(8'h06, 8'd160, 8'd0, 8'h00, r, ok);
        if (!ok || r!==8'h01) begin errors=errors+1; $display("FAIL addcash r=%h", r); end
        sdram_read(10'd32, 16'h0220, r); sdram_read(10'd32, 16'h0221, r1);
        if ({r1,r} !== {h2,h1} + 16'd160) begin errors=errors+1;
            $display("FAIL cash %h%h want +160", r1, r); end
        else $display("PASS OPADDCASH credited");
        end

        // ===== WORKSHOP+FARM dual reset recovery =====
        begin : wk_reset
        reg [7:0] r, r1; reg ok;
        $display("--- DUAL-TASK reset recovery ---");
        nRES_READ = 1'b0; #1000; nRES_READ = 1'b1;
        wait (dut.ready);
        repeat (2000) @(posedge clk100);
        begin : wk_rstwait
        integer t; reg [7:0] fs;
        fs = 8'h00; t = 0;
        while (!fs[1] && t < 20000) begin
            rd_reg(4'hF, fs); t = t + 1;
        end
        if (!fs[1]) begin errors=errors+1; $display("FAIL wk reset: restore never done"); end
        end
        // both worlds survive
        sdram_read(10'd33, 16'h0000, r); sdram_read(10'd33, 16'h0001, r1);
        if (r!==8'h57 || r1!==8'h4B) begin errors=errors+1; $display("FAIL wk SIG lost"); end
        sdram_read(10'd33, 16'h0214, r);
        if (r!==8'hFF) begin errors=errors+1; $display("FAIL skill lost %h", r); end
        // reload + respawn BOTH (quiesce path order)
        farm_load;
        wk_load;
        stage_mbox(2'd0, 8'd2, 8'd0, 8'd0);
        ring(2'd0);
        stage_mbox(2'd1, 8'd3, 8'd0, 8'd0);
        ring(2'd1);
        farm_cmd(8'h00, 8'h00, 8'h00, 8'h00, r, ok);
        if (!ok || r!==8'h01) begin errors=errors+1; $display("FAIL dual reset: farm respawn"); end
        wk_cmd(8'h00, 8'h00, 8'h00, 8'h00, 8'h00, r, r1, ok);
        if (!ok || r!==8'h01) begin errors=errors+1; $display("FAIL dual reset: wk respawn"); end
        else $display("PASS dual-task reset recovery");
        end
```

- [ ] **Step 4:** Run red:
```bash
make DESIGN=project_obscurus REV=rev2 sim PLUSARGS=+farmonly
```
Expected: existing farm phases still PASS; every new wk phase FAILs (wk_cmd timeouts — no blob). Compile must be clean. (+farmonly covers the farm section including these new blocks — they live inside it.)

- [ ] **Step 5:** Commit:
```bash
git add gateware/rev2/project_obscurus/project_obscurus_tb.v gateware/rev2/project_obscurus/worktask.mem
git commit -m "test(farm): workshop tb - craft pipeline, dual heartbeat, dual reset (red)"
```

---

### Task 3: WORKEQU.S + WORKTASK.S (green)

**Files:**
- Create: `software/SDM/WORKEQU.S`, `software/SDM/WORKTASK.S`
- Modify: `Makefile` (worktask/worktasksim targets, WORKTASK_MAXLEN := 768, WORKTASKB sed rule)

- [ ] **Step 1:** `WORKEQU.S`:
```
* WORKEQU.S - WORKSHOP SHARED EQUATES
* PUT-INCLUDE. EQUATES ONLY. NO CODE.
* BANK 33 LAYOUT:
*  $0000 SIG(2) $0002 SEQCTR $0003 HEAD
*  $0004 HBEAT $0005 CVER
*  $0100 RING 64x4 PAGE-ALIGNED
*  $0200 MAILBOX(8) FLAG OP I0-I3 RES RES1
*  $0210 PANTRY(4) $0214 SKILL $0215 DISC(2)
*  $0217 MODE $0220 STATIONS 2x4
*  $0300 RECIPES 12x8 (E-SEEDED)
GBANK = 33
FSIG0 = $0000
FSIG1 = $0001
FSEQC = $0002
FHEAD = $0003
FHBEAT = $0004
FCVER = $0005
FRING = $0100
WFLAG = $0200
WOP = $0201
WI0 = $0202
WI1 = $0203
WI2 = $0204
WI3 = $0205
WRES = $0206
WRES1 = $0207
WPANT = $0210
WSKILL = $0214
WDISCL = $0215
WDISCH = $0216
WMODE = $0217
WSTA = $0220
WRECIP = $0300
* SIGNATURE "WK"
SIGW = $57
SIGK = $4B
* OPS
WOPSTAT = $00
WOPDEP = $01
WOPCRAFT = $02
WOPCOLL = $03
WOPMODE = $04
* RESULTS (SHARED CODES + RUINED)
ROK = $01
RERRBAD = $E6
RERRCROP = $E5
RERRFULL = $E7
RRUIN = $E8
* EVENTS
WEVDONE = $05
* DISCOVERY
DISCBASE = 40
NRECIP = 12
NSLOTS = 2
* STATION TICK DIVIDER (24-BIT)
FSIM = 0
 DO FSIM
STAD0 = $40
STAD1 = $00
STAD2 = $00
 FIN
 DO 1-FSIM
STAD0 = $00
STAD1 = $40
STAD2 = $01
 FIN
* EVLIB SCRATCH (WORKSHOP INSTANCE, $0D PAGE)
EVTYPE = $0D00
EVP0 = $0D01
EVP1 = $0D02
EVHEAD = $0D03
EVSEQ = $0D04
EVRLO = $0D05
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

- [ ] **Step 2:** `WORKTASK.S` — complete source:
```
* WORKTASK.S - WORKSHOP COPROC TASK
* SKILL ID 3, ORG $0300, BUDGET=0 FOREVER.
* SOLE WRITER OF BANK 33. CAP 768 ($0600).
 TYP $06
 DSK WORKTASK.bin
 ORG $0300
 PUT WORKEQU
* ABS SCRATCH $0D10+ (EVLIB OWNS $0D00-05)
STAC0 = $0D10
STAC1 = $0D11
STAC2 = $0D12
TOP = $0D13
TI0 = $0D14
TI1 = $0D15
TI2 = $0D16
TI3 = $0D17
HBV = $0D18
LFSRL = $0D19
LFSRH = $0D1A
NEED0 = $0D1B
NEED1 = $0D1C
NEED2 = $0D1D
NEED3 = $0D1E
RIDX = $0D1F
RADL = $0D20
RADH = $0D21
MTMP = $0D22
MVAL = $0D23
STI = $0D24
STAL = $0D25

 JMP WGAME

WGAME
 JSR WRELOAD
 LDA #$7C
 STA LFSRL
 LDA #$5A
 STA LFSRH
WLOOP
 JSR WMBOX
 JSR WNEXTR
* heartbeat liveness pulse
 INC HBV
 LDX HBV
 LDA #<FHBEAT
 LDY #>FHBEAT
 JSR WRB
* station tick 24-bit down-counter
 LDA STAC0
 SEC
 SBC #1
 STA STAC0
 LDA STAC1
 SBC #0
 STA STAC1
 LDA STAC2
 SBC #0
 STA STAC2
 ORA STAC1
 ORA STAC0
 BNE WLOOP
 JSR WTICK
 JSR WRELOAD
 JMP WLOOP

WRELOAD
 LDA #STAD0
 STA STAC0
 LDA #STAD1
 STA STAC1
 LDA #STAD2
 STA STAC2
 RTS

WNEXTR
 LSR LFSRH
 ROR LFSRL
 BCC WNRNO
 LDA LFSRH
 EOR #$B4
 STA LFSRH
WNRNO
 RTS

* === WTICK: each cooking station timer-- ===
WTICK
 LDA #0
 STA STI
WTKLP
 JSR STADR
 LDA STAL
 LDY #>WSTA
 JSR RDB
 CMP #1
 BNE WTKNX
* timer at STAL+2
 LDA STAL
 CLC
 ADC #2
 LDY #>WSTA
 JSR RDB
 SEC
 SBC #1
 TAX
 PHA
 LDA STAL
 CLC
 ADC #2
 LDY #>WSTA
 JSR WRB
 PLA
 BNE WTKNX
* done: state=2, WEVDONE(station, recipe)
 LDX #2
 LDA STAL
 LDY #>WSTA
 JSR WRB
 LDA STAL
 CLC
 ADC #1
 LDY #>WSTA
 JSR RDB
 STA EVP1
 LDA #WEVDONE
 STA EVTYPE
 LDA STI
 STA EVP0
 JSR PUTEV
WTKNX
 INC STI
 LDA STI
 CMP #NSLOTS
 BNE WTKLP
 RTS

* === STADR: STAL = <WSTA + STI*4 ===
STADR
 LDA STI
 ASL
 ASL
 CLC
 ADC #<WSTA
 STA STAL
 RTS

* === WMBOX: consume one command ===
WMBOX
 LDA #<WFLAG
 LDY #>WFLAG
 JSR RDB
 BNE WMGO
 RTS
WMGO
 LDA #<WOP
 LDY #>WOP
 JSR RDB
 STA TOP
 LDA #<WI0
 LDY #>WI0
 JSR RDB
 STA TI0
 LDA #<WI1
 LDY #>WI1
 JSR RDB
 STA TI1
 LDA #<WI2
 LDY #>WI2
 JSR RDB
 STA TI2
 LDA #<WI3
 LDY #>WI3
 JSR RDB
 STA TI3
 LDA TOP
 BEQ WCSTAT
 CMP #WOPDEP
 BEQ WJDEP
 CMP #WOPCRAFT
 BEQ WJCRAFT
 CMP #WOPCOLL
 BEQ WJCOLL
 CMP #WOPMODE
 BEQ WJMODE
 LDA #RERRBAD
 JMP WMFIN
WJDEP
 JMP WCDEP
WJCRAFT
 JMP WCRAFT
WJCOLL
 JMP WCOLL
WJMODE
 JMP WCMODE
WCSTAT
 LDA #ROK
 JMP WMFIN

* === WMFIN: A=result code -> RES, FLAG=0 ===
WMFIN
 TAX
 LDA #<WRES
 LDY #>WRES
 JSR WRB
 LDX #0
 LDA #<WFLAG
 LDY #>WFLAG
 JSR WRB
 RTS

* --- DEPOSIT TI0=crop TI1=qty ---
WCDEP
 LDA TI0
 CMP #4
 BCS WDBAD
 LDA TI1
 BEQ WDBAD
 LDA TI0
 CLC
 ADC #<WPANT
 STA MTMP
 LDA MTMP
 LDY #>WPANT
 JSR RDB
 CLC
 ADC TI1
 BCC WDNC
 LDA #$FF
WDNC
 TAX
 LDA MTMP
 LDY #>WPANT
 JSR WRB
 LDA #ROK
 JMP WMFIN
WDBAD
 LDA #RERRBAD
 JMP WMFIN

* --- MODE TI0=flags ---
WCMODE
 LDX TI0
 LDA #<WMODE
 LDY #>WMODE
 JSR WRB
 LDA #ROK
 JMP WMFIN

* --- COLLECT TI0=station ---
WCOLL
 LDA TI0
 CMP #NSLOTS
 BCS WCLBAD
 STA STI
 JSR STADR
 LDA STAL
 LDY #>WSTA
 JSR RDB
 CMP #2
 BNE WCLBAD
* value = RECIPES[recipe].VALUE (+5)
 LDA STAL
 CLC
 ADC #1
 LDY #>WSTA
 JSR RDB
 JSR RECADR
 LDA RADL
 CLC
 ADC #5
 STA RADL
 LDA RADL
 LDY RADH
 JSR RDB
 STA MVAL
* BOOM? value *2 clamp $FF
 LDA #<WMODE
 LDY #>WMODE
 JSR RDB
 AND #1
 BEQ WCLNB
 LDA MVAL
 ASL
 BCC WCLST
 LDA #$FF
WCLST
 STA MVAL
WCLNB
 LDX MVAL
 LDA #<WRES1
 LDY #>WRES1
 JSR WRB
* station idle
 LDX #0
 LDA STAL
 LDY #>WSTA
 JSR WRB
 LDA #ROK
 JMP WMFIN
WCLBAD
 LDA #RERRBAD
 JMP WMFIN

* === RECADR: A=recipe idx -> RADL/H ===
RECADR
 ASL
 ASL
 ASL
 CLC
 ADC #<WRECIP
 STA RADL
 LDA #0
 ADC #>WRECIP
 STA RADH
 RTS

* --- CRAFT TI0-3 sorted asc, $FF pad ---
WCRAFT
* free station? prefer 0
 LDA #0
 STA STI
 JSR STADR
 LDA STAL
 LDY #>WSTA
 JSR RDB
 BEQ WCRSL
 LDA #1
 STA STI
 JSR STADR
 LDA STAL
 LDY #>WSTA
 JSR RDB
 BEQ WCRSL
 LDA #RERRFULL
 JMP WMFIN
WCRSL
* per-crop need counts
 LDA #0
 STA NEED0
 STA NEED1
 STA NEED2
 STA NEED3
 LDX TI0
 JSR WNEED
 LDX TI1
 JSR WNEED
 LDX TI2
 JSR WNEED
 LDX TI3
 JSR WNEED
* pantry covers needs?
 LDX #0
WCPCK
 TXA
 CLC
 ADC #<WPANT
 LDY #>WPANT
 JSR RDB
 CMP NEED0,X
 BCC WCPOOR
 INX
 CPX #4
 BNE WCPCK
 JMP WCSCAN
WCPOOR
 LDA #RERRCROP
 JMP WMFIN
* table scan for combo
WCSCAN
 LDA #0
 STA RIDX
WCSLP
 LDA RIDX
 JSR RECADR
 LDA RADL
 LDY RADH
 JSR RDB
 CMP TI0
 BNE WCSNX
 JSR RNEXT
 CMP TI1
 BNE WCSNX
 JSR RNEXT
 CMP TI2
 BNE WCSNX
 JSR RNEXT
 CMP TI3
 BNE WCSNX
 JMP WCHIT
WCSNX
 INC RIDX
 LDA RIDX
 CMP #NRECIP
 BNE WCSLP
* no match: dud -> consume + skill++ + RUIN
 JSR WCONSUM
 JSR WSKUP
 LDA #RRUIN
 JMP WMFIN
* helper: next recipe byte (RADL pre-inc)
RNEXT
 INC RADL
 LDA RADL
 LDY RADH
 JSR RDB
 RTS
WNEED
 CPX #4
 BCS WNDONE
 INC NEED0,X
WNDONE
 RTS

WCHIT
* discovered? DISC bit RIDX
 JSR WDBIT
 AND MTMP
 BNE WCGO
* roll: LFSR < 40 + SKILL/2 - RARITY*16
 LDA #<WSKILL
 LDY #>WSKILL
 JSR RDB
 LSR
 CLC
 ADC #DISCBASE
 BCS WCTHI
 STA MVAL
* rarity at entry+6
 LDA RIDX
 JSR RECADR
 LDA RADL
 CLC
 ADC #6
 STA RADL
 LDA RADL
 LDY RADH
 JSR RDB
 ASL
 ASL
 ASL
 ASL
 STA MTMP
 LDA MVAL
 SEC
 SBC MTMP
 BCS WCTOK
 LDA #0
WCTOK
 STA MVAL
 JMP WCROLL
WCTHI
 LDA #$FF
 STA MVAL
WCROLL
 LDA LFSRL
 CMP MVAL
 BCC WCDISC
* failed roll: consume + skill++ + RUIN
 JSR WCONSUM
 JSR WSKUP
 LDA #RRUIN
 JMP WMFIN
WCDISC
* set DISC bit
 JSR WDBIT
 ORA MTMP
 TAX
 LDA RIDX
 CMP #8
 BCS WCDH
 LDA #<WDISCL
 LDY #>WDISCL
 JSR WRB
 JMP WCGO
WCDH
 LDA #<WDISCH
 LDY #>WDISCH
 JSR WRB
WCGO
* consume + skill++ + start station
 JSR WCONSUM
 JSR WSKUP
* station <- cooking, recipe, TIME
 LDX #1
 LDA STAL
 LDY #>WSTA
 JSR WRB
 LDX RIDX
 LDA STAL
 CLC
 ADC #1
 LDY #>WSTA
 JSR WRB
* TIME at entry+4
 LDA RIDX
 JSR RECADR
 LDA RADL
 CLC
 ADC #4
 STA RADL
 LDA RADL
 LDY RADH
 JSR RDB
 TAX
 LDA STAL
 CLC
 ADC #2
 LDY #>WSTA
 JSR WRB
 LDA #ROK
 JMP WMFIN

* === WDBIT: A=disc byte, MTMP=bit mask ===
* (reads DISCL or DISCH per RIDX, computes
* mask = 1 << (RIDX AND 7))
WDBIT
 LDA RIDX
 AND #7
 TAX
 LDA #1
WDBSH
 CPX #0
 BEQ WDBRD
 ASL
 DEX
 JMP WDBSH
WDBRD
 STA MTMP
 LDA RIDX
 CMP #8
 BCS WDBH
 LDA #<WDISCL
 LDY #>WDISCL
 JSR RDB
 RTS
WDBH
 LDA #<WDISCH
 LDY #>WDISCH
 JSR RDB
 RTS

* === WCONSUM: pantry -= NEED0..3 ===
WCONSUM
 LDX #0
WCONLP
 TXA
 PHA
 TXA
 CLC
 ADC #<WPANT
 STA MTMP
 LDA MTMP
 LDY #>WPANT
 JSR RDB
 PLA
 TAX
 PHA
 TYA
 PHA
 SEC
 SBC NEED0,X
 TAX
 LDA MTMP
 LDY #>WPANT
 JSR WRB
 PLA
 TAY
 PLA
 TAX
 INX
 CPX #4
 BNE WCONLP
 RTS

* === WSKUP: SKILL++ clamp $FF ===
WSKUP
 LDA #<WSKILL
 LDY #>WSKILL
 JSR RDB
 CMP #$FF
 BEQ WSKDN
 CLC
 ADC #1
 TAX
 LDA #<WSKILL
 LDY #>WSKILL
 JSR WRB
WSKDN
 RTS

 PUT PORTLIB
 PUT EVLIB
```
NOTE for the implementer: the WCONSUM register juggling above is the planner's
sketch and is the most likely place for a register bug — rewrite it cleanly if
needed (semantics: for X=0..3, pantry[X] -= NEED0+X; pantry pre-verified ≥
need). Verify every RDB/WRB call's A/Y addressing (RDB clobbers A; X survives;
Y is the hi byte input). The tb is the truth.

- [ ] **Step 3:** Makefile: clone the farmtask rules:
- `WORKTASK_MAXLEN := 768`, guard message "code crosses \$0600 (FARMTASK)".
- `worktasksim`: sed FSIM toggle (mirror farmtasksim's sed on WORKEQU.S → WORKEQUS.S and WORKTASK.S → WORKTASKSIM.S), assemble, size-check, `rom2mem`-equivalent to `gateware/rev2/project_obscurus/worktask.mem` (copy the farmtask.mem rule).
- `worktask`: hardware blob + WORKTASKB.S sed rule (mirror FARMTASKB).
- `farm` target: add WORKTASKB.S dependency (FARM.S will PUT it in Task 6 — until then it's an unused-but-built artifact; acceptable).
- sim target's mem deps: ensure worktask.mem is in MEM_FILES (check how farmtask.mem gets into the copy list — likely wildcard; verify).

- [ ] **Step 4:** Green loop:
```bash
make worktasksim
make DESIGN=project_obscurus REV=rev2 sim PLUSARGS=+farmonly
```
Iterate until ALL phases (farm + workshop + dual reset) PASS, zero errors. Report blob size vs 768.

- [ ] **Step 5:** Commit:
```bash
git add software/SDM/WORKEQU.S software/SDM/WORKTASK.S Makefile gateware/rev2/project_obscurus/worktask.mem
git commit -m "feat(farm): WORKTASK - 2nd coproc task, recipes, discovery, stations"
```

---

### Task 4: FARMTASK //e-bus ops (OPWITHDRAW, OPADDCASH)

**Files:**
- Modify: `software/SDM/FARMEQU.S`, `software/SDM/FARMTASK.S`

(The tb from Task 2 already exercises these — they are red until now if Task 3 ran first; order Tasks 3/4 either way, both must be green before Task 5.)

- [ ] **Step 1:** FARMEQU.S ops block gains:
```
OPWDRAW = $05
OPADDC = $06
```

- [ ] **Step 2:** FARMTASK.S DOMBOX dispatch gains two compares (mind branch ranges — JjMP trampolines like JSELL/JBUY):
```
 CMP #OPWDRAW
 BEQ JWDRAW
 CMP #OPADDC
 BEQ JADDC
```
with
```
JWDRAW
 JMP CWDRAW
JADDC
 JMP CADDC
```
and handlers (place near CSELL):
```
* --- WITHDRAW TA0=crop TA1=qty ---
CWDRAW
 LDA TA0
 CMP #NCROPS
 BCC WDCROK
 LDA #RERRBAD
 JMP MBFIN
WDCROK
 LDA TA1
 BEQ WDBADQ
 LDA TA0
 CLC
 ADC #<FCROPS
 STA M20L
 LDA M20L
 LDY #>FCROPS
 JSR RDB
 CMP TA1
 BCC WDPOOR
 SEC
 SBC TA1
 TAX
 LDA M20L
 LDY #>FCROPS
 JSR WRB
 LDA #ROK
 JMP MBFIN
WDPOOR
 LDA #RERRCROP
 JMP MBFIN
WDBADQ
 LDA #RERRBAD
 JMP MBFIN

* --- ADDCASH TA0=lo TA1=hi, clamp $FFFF ---
CADDC
 LDA #<FCASHL
 LDY #>FCASHL
 JSR RDB
 STA CSHL
 LDA #<FCASHH
 LDY #>FCASHH
 JSR RDB
 STA CSHH
 LDA CSHL
 CLC
 ADC TA0
 STA CSHL
 LDA CSHH
 ADC TA1
 STA CSHH
 BCC ACNC
 LDA #$FF
 STA CSHL
 STA CSHH
ACNC
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
```

- [ ] **Step 3:** Size check + green:
```bash
make farmtasksim
make DESIGN=project_obscurus REV=rev2 sim PLUSARGS=+farmonly
```
If FARMTASK_MAXLEN trips, apply the 37 B hi-byte reclaim (Locked decisions) and retry. All phases green.

- [ ] **Step 4:** Commit:
```bash
git add software/SDM/FARMEQU.S software/SDM/FARMTASK.S gateware/rev2/project_obscurus/farmtask.mem
git commit -m "feat(farm): OPWITHDRAW + OPADDCASH - the //e-bus legs"
```

---

### Task 5: Full-suite gate

- [ ] `make DESIGN=project_obscurus REV=rev2 sim` (~9-10 min, timeout 700000). Zero FAIL. Non-farm failure = regression: STOP.

---

### Task 6: FARM.S — bank-33 seeding, SPAWNW, workshop screen, craft UX

**Files:**
- Modify: `software/SDM/FARM.S`

This is the big //e task. Sub-steps in order; assemble (`make farm`) after EVERY step — branch-range errors surface early. The file will pass ~8 KB; ceiling $9600 is fine.

- [ ] **Step 1: Equates + state.** FARMEQU.S gains `WBANK = 33` and `CVERNUM = 1`. FARM.S new vars (next to the others): `TAIL2 DS 1`, `EXPSEQ2 DS 1`, `WPANT4 DS 4`, `WSKILLV DS 1`, `WDISCV DS 2`, `MIXBUF DS 4`, `MIXN DS 1`, `STREC DS 8` (2×4 station cache). New helpers WFRD/WFWR — copies of FRD/FWR with `LDA #WBANK` instead of GBANK (read FRD/FWR first; they stage SDM_BANK/SDM_ADDR then SDM_READ/SDM_WRITE).

- [ ] **Step 2: Recipe content.** Data blocks:
```
* recipe table image, 12 x 8, seeded to
* bank 33 $0300 at cold start. MUST MATCH
* WORKTASK scan format. values <128 (BOOM x2)
RECTAB
 DFB 0,0,$FF,$FF,4,28,0,0
 DFB 0,1,$FF,$FF,2,36,0,0
 DFB 1,1,$FF,$FF,4,48,0,0
 DFB 0,2,$FF,$FF,4,56,1,0
 DFB 0,0,2,$FF,8,76,1,0
 DFB 2,2,$FF,$FF,4,80,1,0
 DFB 0,3,$FF,$FF,8,80,1,0
 DFB 1,3,$FF,$FF,8,92,1,0
 DFB 1,1,3,$FF,12,100,2,0
 DFB 0,2,3,$FF,12,105,2,0
 DFB 2,2,2,$FF,12,110,2,0
 DFB 0,1,2,3,20,127,3,0
* product names, 7 chars each 0-term packed
RNAMES ASC "BREAD"
 DFB 0
RNAME1 ASC "SALAD"
 DFB 0
RNAME2 ASC "SOUP"
 DFB 0
RNAME3 ASC "MUFFIN"
 DFB 0
RNAME4 ASC "CAKE"
 DFB 0
RNAME5 ASC "JAM"
 DFB 0
RNAME6 ASC "ROAST"
 DFB 0
RNAME7 ASC "STEW"
 DFB 0
RNAME8 ASC "CHOWDER"
 DFB 0
RNAME9 ASC "PIE"
 DFB 0
RNAMEA ASC "WINE"
 DFB 0
RNAMEB ASC "FEAST"
 DFB 0
RNAML DFB <RNAMES,<RNAME1,<RNAME2,<RNAME3
 DFB <RNAME4,<RNAME5,<RNAME6,<RNAME7
 DFB <RNAME8,<RNAME9,<RNAMEA,<RNAMEB
RNAMH DFB >RNAMES,>RNAME1,>RNAME2,>RNAME3
 DFB >RNAME4,>RNAME5,>RNAME6,>RNAME7
 DFB >RNAME8,>RNAME9,>RNAMEA,>RNAMEB
```

- [ ] **Step 3: SEED33 + SPAWNW.** `SEED33`: via WFWR write zeros to $0002-$0005 fields then CVER=1 to $0005, FLAG 0, pantry/skill/disc/mode/stations zero ($0210-$0227), then stream RECTAB's 96 bytes to $0300+ (SDM_SETBANK WBANK + SDM_SETADDR $0300 + 96 × SDM_WRNEXT from a (ptr),Y loop), then SIG LAST: $57→$0000, $4B→$0001. `SPAWNW`: read SPAWNT first and mirror it exactly with CP_LADDR=$0300, WORKTASKB byte stream (add ` PUT WORKTASKB` next to the FARMTASKB PUT at end of file), TABLE write $0206/$0207 = $00/$03, kernel-mailbox stage for slot 1 skill 3, CP_RING slot 1, same carry-error contract.

- [ ] **Step 4: Restore-wait + entry flow.** New `RSTWAIT`: poll $C0CF (slot ROM base + $F — read how FARM.S/CPLIB address the card registers; CP_FSTAT is reg $F) until bit 1 set, 16-bit STACK-based or DS-var counter timeout (~30 s; NEVER ZP). Entry/probe flow changes in MAIN/PRLOOP:
  - Cold start (farm SIG invalid): RSTWAIT → existing bank-32 init → SEED33 → SPAWNT → SPAWNW → SIGs → RESYNC + WSYNC.
  - Warm, both probes OK (farm OPSTAT + workshop OPSTAT via wk mailbox — write a `WSENDCMD` mirroring SENDCMD with WFRD/WFWR and the 8-byte mailbox incl. I2/I3 zeroing and RES1 read into `WRES1V DS 1`): RESYNC + WSYNC, play.
  - Probe dead + CP_ACTIVE==0: RSTWAIT → check bank-33 SIG/CVER: bad → SEED33; → SPAWNT + SPAWNW (BOTH, always) → RESYNC + WSYNC → "TASKS RESPAWNED".
  - Probe dead + CP_ACTIVE busy: existing dead menu; change SDEAD text to `WORLD DEAD - C COLDSTART/CTRL-RESET`.
  - CVER mismatch with valid SIGs (new FARM binary, old content): treat as dead path (CP_ACTIVE==0 after ctrl-reset) → SEED33 re-seed (pantry/skill wiped — acceptable for content upgrades; world bank 32 untouched).

- [ ] **Step 5: WSYNC + workshop ring drain.** `WSYNC`: stable (SEQCTR,HEAD) pair from bank 33 (mirror RESYNC's RSAGN loop with WFRD) → EXPSEQ2/TAIL2; read pantry → WPANT4, skill → WSKILLV, disc → WDISCV, stations → STREC (8 bytes). `DRAIN2`: mirror of DRAIN against bank 33 (WFRD, TAIL2/EXPSEQ2, RECLO2/RSEQ2 DS vars) dispatching to `WEVDISP`: WEVDONE → update STREC state from bank (re-read 8 station bytes), if SCREEN==2 redraw stations region else CLRMSG + print `SCRAFTD` ("CRAFT DONE!") row 21. MLOOP calls `JSR DRAIN2` right after `JSR DRAIN`. DRAIN2's lap path → WSYNC (not RESYNC).

- [ ] **Step 6: Workshop screen.** SCREEN=2. Key W ($D7) handler on BOTH farm chain (next to KM) and MKKEY → `WKENTER` (SCREEN=2, TEXTSW, CLRMSG, hint `1-4 MIX RTN CRAFT D DEP C ESC` row 21, `JSR WKDRAW`). `WKDRAW`: MKCLR reuse (rows 0-19) + row-23 clear; row 0 title `WORKSHOP SKILL` + PRDEC3 WSKILLV; row 2 `PANTRY W` PRDEC3 WPANT4 ×4 at cols 10/17/24/31 with C/B/P letters; rows 4-5 stations from STREC: state 0 `n IDLE`, 1 `n COOKING <name>`, 2 `n DONE <name> PRESS C n`; rows 7-18 recipe book two columns (col 0 and col 20), entries by idx: known (WDISCV bit set — same bit math as blob: idx<8 → lo byte) → name + value PRDEC3; unknown → `?????`; row 19 `MIX:` + MIXBUF letters (W/C/B/P per crop id) or `_`. `WKEXIT` mirrors MKEXIT (no extra CURSDRAW — DRAWALL applies cursor; copy the comment).
  Workshop keys (`WKKEY` dispatched from MLOOP like MKKEY): ESC → MIXN!=0 ? clear MIX + WKDRAW : WKEXIT. 1-4 → if MIXN<4 append crop id, WKDRAW row 19 region (full WKDRAW acceptable). RETURN → if MIXN==0 ignore; else insertion-sort MIXBUF ascending with $FF padding to 4, copy into a `WSENDCMD` craft call (OP=WOPCRAFT $02, I0-I3), clear MIXN, on result: ROK → `COOKING...`, $E8 → `RUINED!`, $E5 → `NEED CROPS`, $E7 → `STATIONS FULL`; then WSYNC-lite (re-read pantry/skill/disc/stations) + WKDRAW. D → QTYPROMPT reuse with farm OPWDRAW staging (CMDOP=OPWDRAW, CMDA0=SELCROP, CMDA1=qty via the prompt — NOTE: QTYPROMPT JMPs to DOCMD which targets the FARM mailbox; that is exactly right for the debit leg), then on ROK send workshop WOPDEP (same crop/qty) via WSENDCMD; deposit failure after withdraw = show ERR (spec-accepted loss). Then re-read + WKDRAW. C → next key 1/2 → station idx; STREC state must be 2 else `NOT DONE`; read STREC recipe id; WSENDCMD WOPCOLL → on ROK take WRES1V → farm DOCMD OPADDC (CMDA0=WRES1V, CMDA1=0) → message `SOLD <name>` + value; re-read + WKDRAW.

- [ ] **Step 7: Heartbeat-aware probe (optional but cheap).** The existing probe uses farm OPSTAT only; workshop probe = WSENDCMD OPSTAT. Skip heartbeat reads (OPSTAT suffices; heartbeats remain a monitor diagnostic).

- [ ] **Step 8: Assemble + commit.**
```bash
make farm
git add software/SDM/FARM.S software/SDM/FARMEQU.S
git commit -m "feat(farm): workshop screen - craft UX, deposits, collect, dual respawn"
```

---

### Task 7: Disk + gate + handoff

- [ ] **Step 1:** `make worktask && make farm && make sdmdisk` — confirm WORKTASKB.S regenerated and FARM grew (~8 KB); .po catalog shows it.
- [ ] **Step 2:** Full suite once more, zero FAIL.
- [ ] **Step 3:** Handoff bullet (after inc-2): increment 3 SHIPPED — WORKTASK skill 3 / bank 33 / 768-cap blob (report size), 12 recipes (values ≤127), discovery threshold 40+skill/2−rarity×16, stations 2, WEVDONE=5, RRUIN=$E8, OPWDRAW=5/OPADDC=6 farm-side, PORTLIB/EVLIB refactor byte-identical, CVER=1, deploy = ctrl-reset + BRUN (farm world survives; bank 33 fresh-seeded). Next: increment 4 = world events.
- [ ] **Step 4:** Commit handoff + tracked artifacts.
- [ ] **Step 5:** Bench checklist (tell the user):
  1. Ctrl-Reset, BRUN FARM — expect `TASKS RESPAWNED`-class recovery with your crops intact, workshop fresh.
  2. W → workshop: skill 0, pantry zeros, 12 `?????` book entries, 2 IDLE stations.
  3. Harvest some wheat → W → D → deposit 4 → pantry W=4.
  4. MIX 1,1 RETURN (BREAD attempt at skill 0: rarity 0 → threshold 40/256 ≈ 16% — expect RUINED a few times, skill ticking up; each attempt eats 2 wheat).
  5. First success: `COOKING`, station 1 busy; go farm, come back — `DONE`; C 1 → `SOLD BREAD +28`, cash up.
  6. Book now shows BREAD. Try a wrong combo (1,3 = wheat+pumpkin... that's ROAST, actually real!) — try 1,1,1 (W,W,W — dud) → RUINED always, skill up.
  7. Ctrl-Reset mid-cook → re-enter → station still cooking (world survival incl. crafts).
  8. `/obs-screenshot` the book and stations.

---

## Self-review notes (planning time)

- Spec coverage: WORKSHOP task/bank/mailbox/ops ✓ (T3), EVLIB scratch param ✓ (T1, byte-identical gate), PORTLIB ✓ (T1), recipe SDRAM table + //e seeding ✓ (T2 rig + T6 real), discovery RNG + skill clamp + rarity floor ✓ (T3), 2 stations + timed cook + WEVDONE ✓ (T3), OPMODE/BOOM ✓ (T3+tb), //e-bus debit-first ✓ (tb order + T6 D-key order), CVER ✓ (T2/T6), quiesce respawn BOTH + restore_done wait ✓ (tb wk_reset + T6 entry flow), heartbeat ✓, workshop screen + craft UX ✓ (T6), //e sorts ingredients ✓ (T6 RETURN handler).
- Known riskiest code: WCONSUM register juggling (flagged inline for the implementer to rewrite if buggy), WDBIT shift loop, and the T6 entry-flow rewiring (most intricate //e change so far — implementer must read MAIN/PRLOOP/PROBED/COLDST in full before editing).
- tb rig honesty: deposit-without-debit in one spot is labeled "rig"; the bus-order test happens explicitly earlier.
- worktask.mem placeholder before Task 3 exists is called out (1-byte file).
- Names/types consistent: WSENDCMD/WFRD/WFWR/WSYNC/DRAIN2/WKDRAW/WKENTER/WKEXIT/WKKEY/SEED33/SPAWNW/RSTWAIT/MIXBUF/MIXN/STREC/WPANT4/WSKILLV/WDISCV/WRES1V; blob: WGAME/WLOOP/WMBOX/WMFIN/WCDEP/WCRAFT/WCOLL/WCMODE/WTICK/STADR/RECADR/WDBIT/WCONSUM/WSKUP.
