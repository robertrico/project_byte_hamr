# Farm v2 Increment 1: Sell-Qty Prompt + Heartbeat — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ship the two smallest v2 wins: a modal 2-digit sell-quantity prompt on the farm HUD (replaces S-mash-1x), and a heartbeat byte each task increments so a wedged task is diagnosable.

**Architecture:** The coproc blob (FARMTASK) already implements SELL with a quantity argument (`CSELL`, TA0=qty, cash clamp $FFFF, supply clamp $FF) — verified, and the sim econ phase already exercises qty>1. So sell-qty is a pure //e change in FARM.S: the S key opens a modal prompt, digits accumulate into a qty byte, RETURN sends the existing OPSELL with CMDA0=qty, ESC/zero cancels. Heartbeat is the only blob change: FARMTASK increments farm-bank byte $0004 (FHBEAT) every main-loop pass, TDD'd via a new testbench phase.

**Tech Stack:** Merlin32 6502 (single-space format, ASCII, ≤50-char lines), Icarus Verilog testbench (`-g2005`), `make farmtasksim` / `make farm` / `make sdmdisk`, sim gate `make DESIGN=project_obscurus REV=rev2 sim PLUSARGS=+farmonly` (~90 s).

**Spec:** `docs/superpowers/specs/2026-06-11-farm-v2-sidework-design.md`

**Deploy reminder (tell the user, every time):** Task 2 changes the blob. Re-entry does NOT reload it — bench deploy needs a soft reset (Ctrl-Reset; respawn reloads blob from the FARM binary) or power-cycle.

---

## Working rules for every task

- Merlin source format: SINGLE SPACES between fields (no tabs, no alignment), labels in column 0, one leading space for unlabeled lines, ASCII only, lines ≤50 chars. Match the existing FARM.S/FARMTASK.S style exactly.
- Never touch ProDOS ZP. FARM.S uses MSGPTR=$0A etc. — already safe; add no new ZP.
- All line numbers below were verified against working tree @ cf4a672; re-grep the anchor text if a line has drifted.
- Do NOT run `prog-flash` or any flash target. Build + sim only; bench is the user's.

---

### Task 1: Failing testbench phase for the heartbeat

**Files:**
- Modify: `gateware/rev2/project_obscurus/project_obscurus_tb.v` (insert before the line `// ===== FARM economy: SELL/BUYSEED + clamps + price walk (M1 c) =====`, ~line 1318)

- [ ] **Step 1: Insert the heartbeat phase block**

Anchor: find the `end` that closes the `begin : farm_m1` named block, immediately before the `// ===== FARM economy:` comment. Insert between them:

```verilog
        // ===== FARM heartbeat: FHBEAT ($0004) advances while task lives =====
        // read-twice-with-delay (spec: byte wraps every 256 passes, so two
        // close reads can alias equal on a live task -> retry a few samples)
        begin : farm_beat
        reg [7:0] h1, h2; integer t; reg beat;
        beat = 0;
        sdram_read(10'd32, 16'h0004, h1);
        for (t = 0; t < 5 && !beat; t = t + 1) begin
            repeat (200000) @(posedge clk100);
            sdram_read(10'd32, 16'h0004, h2);
            if (h2 !== h1) beat = 1;
        end
        if (!beat) begin errors=errors+1;
            $display("FAIL farm heartbeat stuck h=%h", h1); end
        else $display("PASS farm heartbeat %h -> %h", h1, h2);
        end
```

Note: `farm_init` never writes $0004, so before the implementation lands both reads return the same value (x or stale) and the phase fails deterministically.

- [ ] **Step 2: Run the sim, verify it FAILS**

```bash
cd /Users/hambook/Development/project_byte_hamr
make DESIGN=project_obscurus REV=rev2 sim PLUSARGS=+farmonly
```

Expected: `FAIL farm heartbeat stuck` in the output, nonzero error count. All pre-existing farm phases must still PASS — if anything else fails, stop and investigate before proceeding.

- [ ] **Step 3: Commit the failing test**

```bash
git add gateware/rev2/project_obscurus/project_obscurus_tb.v
git commit -m "test(farm): heartbeat tb phase - FHBEAT must advance (red)"
```

---

### Task 2: Heartbeat in FARMTASK (make the test pass)

**Files:**
- Modify: `software/SDM/FARMEQU.S` (after `FHEAD = $0003`, line 13)
- Modify: `software/SDM/FARMTASK.S` (scratch list ~line 36, GLOOP ~line 54)
- Modify: `software/SDM/FARMTASKSIM.S` only if it does not `PUT FARMTASK` (check first — if it is a standalone copy, mirror the same edits there; the sim blob is built from FARMTASKSIM)

- [ ] **Step 1: Add the equate to FARMEQU.S**

After the line `FHEAD = $0003` add:

```
FHBEAT = $0004
```

- [ ] **Step 2: Add the scratch byte to FARMTASK.S**

After the line `LFSRH = $0E2C` add:

```
HBV = $0E2D
```

(Spec note: FARMTASK scratch officially grows to $0E2D; still well clear of mailboxes at $0F80.)

- [ ] **Step 3: Add the increment to the main loop**

In `GLOOP`, immediately after `JSR DOMBOX`, insert:

```
 INC HBV
 LDX HBV
 LDA #<FHBEAT
 LDY #>FHBEAT
 JSR WRB
```

(WRB writes X to GBANK addr A=lo Y=hi inside its own SEI/CLI burst — the window-access convention holds. HBV may start stale after respawn; irrelevant, only *advancement* matters.)

- [ ] **Step 4: Check FARMTASKSIM.S**

```bash
grep -n "PUT FARMTASK\|ORG\|FSIM" software/SDM/FARMTASKSIM.S
```

If FARMTASKSIM.S is a thin wrapper that `PUT`s FARMTASK, nothing more to do. If it's a standalone source, apply Steps 2-3 to it identically.

- [ ] **Step 5: Rebuild the sim blob, verify the size guard**

```bash
make farmtasksim
```

Expected: success; the Makefile FARMTASK_MAXLEN=1536 check passes (current blob 1299 B + ~13 B ≪ 1536). If the guard trips, something is very wrong — stop.

- [ ] **Step 6: Run the sim, verify heartbeat PASSES**

```bash
make DESIGN=project_obscurus REV=rev2 sim PLUSARGS=+farmonly
```

Expected: `PASS farm heartbeat xx -> yy`, zero errors overall (all farm phases green).

- [ ] **Step 7: Commit**

```bash
git add software/SDM/FARMEQU.S software/SDM/FARMTASK.S software/SDM/FARMTASKSIM.S gateware/rev2/project_obscurus/farmtask.mem
git commit -m "feat(farm): heartbeat - FARMTASK increments FHBEAT (\$0004) every pass"
```

(Stage whichever generated artifacts the build refreshed — the stale-artifact lesson: verify farmtask.mem actually changed: `git status` must show it.)

---

### Task 3: SELLQTY modal prompt in FARM.S

**Files:**
- Modify: `software/SDM/FARM.S` — four edits: vars (~line 478), string (~line 444), K7 handler (~line 1026), new routine after NIBA (~line 1115)

No sim coverage exists for the //e binary; this task is assemble-verified (`make farm`) and bench-verified (Task 4). The coproc side (CSELL qty, clamps, error results E5/E6) is already sim-covered by the econ phase.

- [ ] **Step 1: Add prompt variables**

After `ERRCODE DS 1` (follows `CMDA2 DS 1`) add:

```
QVAL DS 1
QDIG DS 1
```

- [ ] **Step 2: Add the prompt string**

After the `SERR ASC "ERR "` / `DFB 0` pair add:

```
SQTY ASC "SELL QTY:"
 DFB 0
```

- [ ] **Step 3: Rewire the S key**

Replace (in the K7 handler):

```
K7
 CMP #$D3 ; S
 BNE K8
 LDA #OPSELL
 JSR DOCMD1
 JMP MLOOP
```

with:

```
K7
 CMP #$D3 ; S
 BNE K8
 JSR SELLQTY
 JMP MLOOP
```

- [ ] **Step 4: Add the SELLQTY routine**

Insert after the end of the NIBA routine (the `RTS` following the `NIBDONE` label region — the last code of the DOCMD error path):

```
* === SELLQTY: modal 2-digit sell prompt ===
* digits echo col 10+, RETURN sends OPSELL
* qty, ESC or qty 0 cancels. modal: no DRAIN
* while open (ring holds 64, prompt is short)
SELLQTY
 JSR CLRMSG
 LDA #<SQTY
 STA MSGPTR
 LDA #>SQTY
 STA MSGPTR+1
 LDA #0
 STA PRCOL
 LDY #20
 JSR PRSTR
 LDA #0
 STA QVAL
 STA QDIG
SQKEY
 LDA KBD
 BPL SQKEY
 STA KBDSTR
 CMP #$9B ; ESC
 BEQ SQCAN
 CMP #$8D ; RETURN
 BEQ SQGO
 CMP #$B0 ; below '0'
 BCC SQKEY
 CMP #$BA ; above '9'
 BCS SQKEY
 LDX QDIG
 CPX #2
 BCS SQKEY
* echo digit at row 20 col 10+QDIG
 PHA
 LDY #20
 LDA GRLO,Y
 STA LINEP
 LDA GRHI,Y
 STA LINEP+1
 LDA LINEP
 CLC
 ADC #10
 STA LINEP
 LDA LINEP+1
 ADC #0
 STA LINEP+1
 PLA
 LDY QDIG
 STA (LINEP),Y
* QVAL = QVAL*10 + digit
 AND #$0F
 PHA
 LDA QVAL
 ASL
 STA QVAL
 ASL
 ASL
 CLC
 ADC QVAL
 STA QVAL
 PLA
 CLC
 ADC QVAL
 STA QVAL
 INC QDIG
 JMP SQKEY
SQGO
 LDA QVAL
 BEQ SQCAN
 STA CMDA0
 LDA #0
 STA CMDA1
 STA CMDA2
 LDA #OPSELL
 STA CMDOP
 JMP DOCMD
SQCAN
 JMP CLRMSG
```

Why this works with existing code: `PRSTR` prints 0-term hi-ASCII at text row Y / column PRCOL; `GRLO/GRHI` are the per-row base tables (CLRMSG and PRSTR both index them with row 20-23); `DOCMD` runs SENDCMD and prints OK/ERR to row 20 itself (its CLRMSG wipes the prompt remnants), then RTS — the `JMP DOCMD` tail returns to K7's `JSR SELLQTY` frame. The x10 sequence: ASL→2v stored, ASL ASL→8v in A, +2v=10v, +digit. Max input 99, no overflow. CSELL rejects qty>crops with E5 (shows as `ERR E5`) — no //e-side inventory check needed.

- [ ] **Step 5: Assemble**

```bash
make farm
```

Expected: clean assemble, no size errors. Confirm `software/SDM/FARM.bin` mtime changed.

- [ ] **Step 6: Commit**

```bash
git add software/SDM/FARM.S software/SDM/FARM.bin
git commit -m "feat(farm): sell-qty - S opens modal 2-digit prompt, OPSELL qty"
```

---

### Task 4: Disk build + bench checklist (user runs the //e)

**Files:**
- None modified; builds `SDMTEST.po` via `make sdmdisk`.

- [ ] **Step 1: Build the disk**

```bash
make farm && make sdmdisk
```

Expected: bootable .po containing the new FARM (with embedded new blob via `PUT FARMTASKB` — confirm FARMTASKB regenerated: `git status` shows it changed after `make farm`).

- [ ] **Step 2: Full sim suite (pre-merge gate)**

```bash
make DESIGN=project_obscurus REV=rev2 sim
```

Expected: all phases PASS (~9 min). Run once before declaring the increment done.

- [ ] **Step 3: Hand to user for bench — checklist**

Tell the user: **blob changed — soft reset (Ctrl-Reset) or power-cycle so the respawn path reloads it.** Then:

1. Boot SDMTEST.po, BRUN FARM. Plant + ripen + harvest a few crops.
2. `S` → row 20 shows `SELL QTY:` — type `3`, RETURN → `OK`, CASH jumps by 3x price, market SELL row updates.
3. `S` → ESC → prompt clears, nothing sold.
4. `S` → `0` RETURN → cancels (no E6 roundtrip).
5. `S` → `99` RETURN with fewer crops → `ERR E5`, nothing sold.
6. Heartbeat: quit to the SDM monitor, read bank 32 addr $0004 twice a second apart — values differ. (Read-only — no monitor pokes while the task lives.)
7. `/obs-screenshot` to capture the prompt and the post-sell HUD.

- [ ] **Step 4: Update handoff + commit**

Append to `software/SDM/HANDOFF_FARM_V2.md` under "Where we are": increment 1 (sell-qty prompt + heartbeat) shipped, spec at `docs/superpowers/specs/2026-06-11-farm-v2-sidework-design.md`, next = increment 2 (screen manager + market screen + seeds).

```bash
git add software/SDM/HANDOFF_FARM_V2.md
git commit -m "docs(farm): handoff - inc 1 sell-qty + heartbeat shipped"
```

---

## Self-review notes (done at planning time)

- Spec coverage for increment 1: sell-qty prompt ✓ (Task 3), heartbeat ✓ (Tasks 1-2), modal rule ✓ (no DRAIN in SQKEY loop), 2-digit/1-99 ✓, heartbeat read-twice ✓ (tb retry loop + bench step 6). CVER/quiesce/seeds are later increments by design.
- CSELL qty + clamps verified present in FARMTASK.S @ cf4a672 (lines 287-360); tb econ phase sells qty=nc — no new coproc sell code needed.
- Type/name consistency: QVAL/QDIG/SQTY/SELLQTY defined once, used once; FHBEAT/HBV match spec names.
- Open verify-at-execution item: whether FARMTASKSIM.S mirrors or includes FARMTASK.S (Task 2 Step 4 resolves it either way).
