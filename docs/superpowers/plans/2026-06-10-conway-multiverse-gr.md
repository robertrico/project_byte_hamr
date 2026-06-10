# Conway's Multiverse — Lo-Res (GR) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development. Steps use checkbox (`- [ ]`) syntax.

**Goal:** The watchable companion to the DHGR multiverse — 8 live Conway Life universes at Lo-Res 40×48, ~56× faster (near-real-time, fat visible blocks), channel-surf `0`-`7`. Ships beside MVERSE.

**Architecture:** Copy/alter the bench-proven DHGR multiverse. Coproc `LIFE8GR` = `LIFE8` with FOUR alters (8 cells/byte, GR equates, `GWIDTH-1` torus wrap, `MUL80→MUL5` stride) — same TICK1. //e `GRVERSE` = `MVERSE` re-targeted: GR display (undo-DHGR + lo-res enable, line-table clear), **GEN-snapshot-retry** read, **bit→nibble expand** to the GR text-page block format. No gateware change.

**Tech Stack:** Merlin 6502 — **Merlin-on-//e format** (single spaces, labels col 0, one leading space unlabeled, ASCII, ≤50-char lines, left-to-right expressions — [[feedback_merlin_source_format]]; match `software/SDM/SDRAMLIB.S`). Built via Merlin32. iverilog `-g2005` for the tick sim. Branch `conway-multiverse-gr`. Spec: `docs/superpowers/specs/2026-06-10-conway-multiverse-gr-design.md`.

**Reuse:** `software/SDM/LIFE8.S` + `MVERSE.S` (copy + alter), `SDRAMLIB.S`. Coproc read window `$E004-08` / write `$E000-03` (unchanged).

---

## Fixed layout

**Grid:** 40×48, **8 cells/byte, bit 0 = leftmost**, `ROWBYTES=5` (40=5×8), `GROWS=48`, `GWIDTH=40`. Buffer = 5×48 = **240 B**; double-buffered. `cell(r,x)` = bit `(x&7)` of `grid[r*5 + (x>>3)]`.

**SDRAM map:** universe u = bank `UBASE+u`, buffer A `$0000`, buffer B `$0400` (240 B each, `$0400` clear of A). Metadata bank `MBANK`: `FRONT[u]`@`$0010+u`, `GEN[u]`@`$0020+u`. (Separate `BRUN` from MVERSE — bank reuse fine; GRVERSE reseeds on launch.)

**GR display:** text page `$400-$7FF`, 40 bytes/row × 24 text-rows. Byte = 2 vertical blocks: **low nibble (0-3) = TOP block, high nibble (4-7) = BOTTOM**; `$0`=dead, `$F`=live. Text-row Y holds block-rows `2Y` (top) + `2Y+1` (bottom). Line base: `textbase(Y) = $400 + (Y&7)*$80 + (Y>>3)*$28` (24 entries).

---

## File Structure
- `software/SDM/LIFEMAPGR.S` — GR equates (ROWBYTES=5, GROWS=48, GWIDTH=40, banks).
- `software/SDM/LIFE8GR.S` — copy LIFE8.S + the FOUR alters. The coproc Life skill.
- `software/SDM/GRVERSE.S` — copy MVERSE.S, GR-retargeted. The //e app.
- `software/SDM/LIFE8GR.DFB.S` — AUTO-GENERATED (Makefile) DFB include of the LIFE8GR LSIM=0 bytes; GRVERSE PUTs it (G3 — no hand-pasted blob).
- `gateware/rev2/project_obscurus/project_obscurus_tb.v` — add the GR-dim TICK1 oracle (keep the DHGR one — both regressions).
- `Makefile` — `life8gr`/`grverse` targets, the DFB-gen rule, sdmdisk pack.

---

## Task 1: LIFEMAPGR.S equates

**Files:** Create `software/SDM/LIFEMAPGR.S`.

- [ ] **Step 1: Write it** (Merlin-//e format; match LIFEMAP.S cadence — READ `software/SDM/LIFEMAP.S` first):
```
* LIFEMAPGR.S - SHARED GR MULTIVERSE EQUATES
* PUT INTO LIFE8GR AND GRVERSE
UBASE = 16
MBANK = 24
BUFA = $0000
BUFB = $0400
ROWBYTES = 5
GWIDTH = 40
NROWS = 48
NUNIV = 8
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
(GR: ROWBYTES=5, GWIDTH=40, NROWS=48. `ROWBYTES*GROWS = 5*48 = 240` per buffer; A@$0000, B@$0400.)

- [ ] **Step 2: Format check** — `grep -cP '\t' software/SDM/LIFEMAPGR.S` (0), `grep -cP '[^\x00-\x7F]'` (0).
- [ ] **Step 3: Commit** — `git add software/SDM/LIFEMAPGR.S && git commit -m "feat(multiverse-gr): GR shared equates (LIFEMAPGR)` + trailer `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`.

---

## Task 2: LIFE8GR.S — the FOUR alters + GR-dim sim oracle (THE hard task)

**Files:** Create `software/SDM/LIFE8GR.S` (copy LIFE8.S); extend `gateware/rev2/project_obscurus/project_obscurus_tb.v`.

TDD: re-parameterize the DHGR TICK1 oracle for GR dims, make LIFE8GR pass it. The four alters are the only changes vs LIFE8 — the TICK1 algorithm is otherwise byte-identical.

### The FOUR alters (vs LIFE8.S)
1. **`PUT LIFEMAPGR`** (not LIFEMAP) — gets ROWBYTES=5, GROWS via NROWS=48, GWIDTH=40.
2. **Bit loop 0-7, not 0-6.** In `DOCOLSUM` (the per-byte bit extraction) and `DOROWOUT` (the pack): DHGR's `LDX #7` (7 bits/byte) → `LDX #8` (8 bits/byte). (The last-byte guards `CPX #1`/`CPX #2` are X-countdown-relative — leave them; they survive.)
3. **Torus-wrap column: `COLSUM+559` → `COLSUM+GWIDTH-1`.** LIFE8.S:339 has a hardcoded `LDA COLSUM+559` (DHGR's col 560-1). Replace with `LDA COLSUM+GWIDTH-1` (=COLSUM+39). Also confirm `DOROWOUT`'s right-edge wrap uses `GWIDTH-1`/`GWIDTH` (col 39's right neighbor = col 0), not a 560-relative literal.
4. **`MUL80` → `MUL5`.** LIFE8.S:271-296's `MUL80` (hand-rolled ×80 shift-add, computes row r's byte offset `r*ROWBYTES`) → **`MUL5`** (`PROD = A*4 + A`: `ASL ASL` then add original). Used by `READROW`/`WRITEROW`. This is NOT equate-driven — it's separate code. (Implement: save A, `ASL A / ASL A` (×4), `CLC / ADC saved` (×5); store to PROD lo/hi.)

### Step 1: re-parameterize the TICK1 oracle in `project_obscurus_tb.v` (the load-bearing test)
Add a SECOND multiverse oracle block (keep the DHGR one — both regressions). Same harness as the DHGR test (load via `load_byte`→coproc $0300, TABLE[skill]=$0300, `stage_mbox(slot,skill,budget 0,0)`, `ring`, sync on `GEN[u]` since LIFE8GR never DONEs — use the LIFE1-style $0303 single-universe oracle entry that DONEs). Re-parameterized for GR: `ROWBYTES=5`, GWIDTH=40, an `LSIM` small `GROWS` (e.g. 8), KEEP 5-byte rows / 8-cells-per-byte. Seed (via `sdram_write` into bank UBASE buffer A):
- **Blinker** (3 horizontal cells), interior → after 1 tick vertical, after 2 horizontal. Simplest oracle.
- **Glider** crossing an **8-cell byte boundary** (place it spanning byte 0/1, cols 6-9) → after 4 ticks +1/+1; assert its new cells (catches the 8/byte math).
- **Torus seam**: a pattern straddling col 39↔0 → assert the GWIDTH-1 wrap.
Hand-compute the expected generations, hardcode asserts. Print `PASS multiverse-GR TICK1 (...)`.
Register LIFE8GR as a NEW skill id in the tb (don't clobber the DHGR LIFE8 skill-0 registration — use skill id 2 or a separate tb block + its own load).
Run (expect FAIL — no LIFE8GR): `cd /Users/hambook/Development/project_byte_hamr && make sim DESIGN=project_obscurus REV=rev2 2>&1 | grep -iE "multiverse-GR|FAIL"`.

### Step 2: Implement LIFE8GR.S (copy LIFE8.S, apply the FOUR alters)
```bash
cp software/SDM/LIFE8.S software/SDM/LIFE8GR.S
# then edit: PUT LIFEMAPGR, LDX #7->#8 (both DOCOLSUM + DOROWOUT bit loops),
#            COLSUM+559 -> COLSUM+GWIDTH-1, MUL80 -> MUL5.
```
Keep the LSIM toggle + the LIFE1 ($0303) single-universe oracle entry (for the sim) + the LIFE8 ($0300) forever-loop entry. Merlin-//e format throughout.

### Step 3: Run — expect PASS
```bash
make sim DESIGN=project_obscurus REV=rev2 2>&1 | grep -iE "multiverse-GR|multiverse TICK1|round-robin|PASS|FAIL"
```
Expected: `PASS multiverse-GR TICK1` + the DHGR `multiverse TICK1`/`round-robin` still PASS + all prior, 0 errors. Format: `grep -cP '\t' software/SDM/LIFE8GR.S`==0, ASCII==0.

### Step 4: Commit
```bash
git add software/SDM/LIFE8GR.S gateware/rev2/project_obscurus/project_obscurus_tb.v Makefile
git commit -m "feat(multiverse-gr): LIFE8GR - LIFE8 + 4 alters (8/byte, GR equates, GWIDTH-1 wrap, MUL5), GR-dim sim oracle

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: GRVERSE.S — GR display + snapshot-retry + bit→nibble render

**Files:** Create `software/SDM/GRVERSE.S` (copy MVERSE.S, replace the DHGR render half).

### Step 1: GR init (`GRON`) — undo DHGR mode, then enable lo-res (G4)
```
GRON
 STA $C00C        ; 80VID OFF
 STA $C05F        ; DHIRES OFF
 STA $C000        ; 80STORE OFF (a WRITE to $C000)
 STA $C054        ; PAGE2 OFF
 STA $C050        ; GRAPHICS
 STA $C056        ; LORES
 STA $C052        ; FULL SCREEN
 RTS
```
(Undo first — MVERSE may have run this session, leaving 80VID/80STORE/HIRES/DHIRES on.)

### Step 2: GR clear (`GRCLR`) — via the line table, NOT a blanket fill (G5)
Build/use the 24-entry text-line table (`textbase(Y) = $400 + (Y&7)*$80 + (Y>>3)*$28`). `GRCLR`: for Y=0..23, write `$00` to `textbase(Y)+0..39`. Do NOT memset `$400-$7FF` (the screen holes `$478-$47F` etc. are firmware scratch).

### Step 3: `RENDER(CHAN)` — GEN-snapshot-retry + bit→nibble expand
```
RENDER
* snapshot-retry (G2):
RETRY
 read GEN[CHAN] -> GENB                ; SDM_READ MBANK,$0020+CHAN
 read FRONT[CHAN] -> live base         ; SDM_READ MBANK,$0010+CHAN
 SDM_SETBANK UBASE+CHAN ; SDM_SETADDR livebase ; 240x SDM_RDNEXT -> GBUF (240-B //e buffer)
 read GEN[CHAN] -> GENA
 if GENB != GENA goto RETRY            ; a tick flipped mid-snapshot -> retry (<=2 tries)
* expand GBUF -> text page:
 for Y = 0..23:
   for X = 0..39:
     top = bit(X&7) of GBUF[(2*Y)*5 + (X>>3)]      ; row 2Y  (use MUL5 for row*5)
     bot = bit(X&7) of GBUF[(2*Y+1)*5 + (X>>3)]    ; row 2Y+1
     byte = (top ? $0F : 0) | (bot ? $F0 : 0)
     store byte to textbase(Y) + X
 RTS
```
- GBUF = a 240-byte //e buffer (DS 240). Row r of the grid is at `GBUF + r*5` (**MUL5** — G1 //e side).
- The 240 contiguous `SDM_RDNEXT` reads work because grid rows are contiguous (stride=ROWBYTES=5).
- bit0=leftmost; top→low nibble, bottom→high nibble; `$0`/`$F`.

### Step 4: assemble-check (the render is //e/bench-verified — no GR sim)
Add a minimal MAIN (GRON, GRCLR, CHAN=0, RENDER, RTS) so it assembles; Task 4 replaces MAIN with the full orchestration. `make grverse 2>&1 | tail -3` (add the target); format `grep -cP '\t'`==0, ASCII==0. Note byte length.

### Step 5: Commit
```bash
git add software/SDM/GRVERSE.S Makefile
git commit -m "feat(multiverse-gr): GRVERSE render - GR init (undo-DHGR+lores), line-table clear, snapshot-retry, bit->nibble expand

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: GRVERSE orchestration — auto-gen blob, GR seeds, GO, surf

**Files:** Modify `software/SDM/GRVERSE.S`; add the DFB-gen Makefile rule.

### Step 1: auto-generate the LIFE8GR blob (G3 — no hand-paste)
Add a Makefile rule: build `LIFE8GR` at **LSIM=0** (192... no, GR NROWS=48 — the HW build), `xxd`/awk the `.bin` into `software/SDM/LIFE8GR.DFB.S` as `CSKILL DFB $..,$..` lines + a `CSKEND` label. GRVERSE `PUT`s `LIFE8GR.DFB` in its REGSKILL. (Verify the rule emits the LSIM=0 bytes, not the sim LSIM build — set LSIM=0 for the embed build, then restore for sim, OR build a separate LSIM=0 artifact.)
```make
software/SDM/LIFE8GR.DFB.S: software/SDM/LIFE8GR.bin
	xxd -i < $< | awk '...emit " DFB $hh,$hh,..." lines, label CSKILL/CSKEND...' > $@
```
(Exact awk: emit lines like ` DFB $A9,$2F,...` ≤ ~50 chars, ASCII. Lead with `CSKILL` label, end `CSKEND`.)

### Step 2: REGSKILL — load LIFE8GR to coproc $0300 + TABLE[0]
Mirror MVERSE/CPDEMO LOADBLK: CP_LADDR=$0300, stream `CSKILL..CSKEND` bytes (from the PUT'd DFB) via CP_WDATA; TABLE[0]=$0300.

### Step 3: SEED — 8 GR-recomputed patterns (G6)
Recompute ALL seed offsets for **5-byte rows + 8-bit packing**; fill counts **$3C00 → 240**. Patterns scaled to 40×48: gliders (u0-u1), blinkers/oscillators (u2), r-pentomino centered (u3), LFSR soup (u4-u7). Per universe: SDM_SETBANK UBASE+u, SDM_SETADDR BUFA, write the pattern (POKE table or SOUPF LFSR, count 240). Zero FRONT[u]/GEN[u]. (No Gosper gun — dies on the 40×48 torus.)

### Step 4: GOLIFE — budget=0 (R1)
Stage slot-0 mailbox (coproc $0F80: skill_id $00, **budget $00**, arg0 $00); ring slot 0 (`LDA #0 / STA $C0C5`).

### Step 5: SURF — keys + gen-skip (copy MVERSE's SURF)
Poll `$C000`; on key clear `$C010`; `0`-`7`→CHAN, arrows ±1 wrap; render on gen-change or channel-change (the snapshot-retry handles coherence). At GR speed GEN changes ~every render → near-continuous; keyboard polled between (fine).

### Step 6: MAIN — wire it
`MAIN`: SDM_READY (BCC) → GRON → GRCLR → REGSKILL → SEED → GOLIFE → CHAN=0 → JMP SURF.

### Step 7: build + disk
```bash
make life8gr grverse 2>&1 | tail -3       # LIFE8GR (+ DFB gen), GRVERSE assemble clean
make sdmdisk 2>&1 | tail -5               # add GRVERSE to the pack (keep MVERSE); bump .po size if full
grep -cP '\t' software/SDM/GRVERSE.S      # 0
grep -cP '[^\x00-\x7F]' software/SDM/GRVERSE.S  # 0
```
Confirm GRVERSE + MVERSE both in the catalog.

### Step 8: Commit
```bash
git add software/SDM/GRVERSE.S software/SDM/LIFE8GR.DFB.S Makefile
git commit -m "feat(multiverse-gr): GRVERSE orchestration - auto-gen blob, GR seeds, GO budget=0, surf

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 5: Build + bench-ready

- [ ] **Step 1: Sim regression** — `make sim DESIGN=project_obscurus REV=rev2 2>&1 | grep -iE "PASS|FAIL"`. `PASS multiverse-GR TICK1` + the DHGR `TICK1`/`round-robin` + all prior, 0 errors. (No gateware build — no RTL change.)
- [ ] **Step 2: Format verify** all GR `.S` (`LIFEMAPGR`/`LIFE8GR`/`GRVERSE`): `grep -cP '\t'`==0, ASCII==0, no mixed `+...*` precedence.
- [ ] **Step 3: Hand the bench procedure** (no flash — gateware unchanged):
```
1. make sdmdisk DESIGN=project_obscurus REV=rev2
2. Boot the disk (current bitstream runs it). BRUN GRVERSE
   -> 8 GR universes, FAST near-real-time, fat visible blocks. 0-7 flips channels;
      soup channels churn visibly. (BRUN MVERSE still there for the DHGR version.)
   WATCH-FOR: (a) the bit->nibble render lands cells correctly (a known seed in the right
   blocks), (b) no tearing (the snapshot-retry), (c) BRUN GRVERSE after BRUN MVERSE works
   (the undo-DHGR switches).
```
- [ ] **Step 4: Record** — update `project_coproc_c0.md` + `MEMORY.md`: the GR variant (the watchable demo), the 4 alters + MUL5 + snapshot-retry + GR-block render, both games ship.

---

## Self-Review
**Spec coverage:** grid 40×48/8-byte/240B (Task 1+2); the FOUR alters incl MUL5 (Task 2); GR-dim oracle (Task 2); GR init undo-DHGR G4 (Task 3); line-table clear G5 (Task 3); snapshot-retry G2 + MUL5 //e G1 (Task 3); bit→nibble render (Task 3); auto-gen blob G3 (Task 4); GR-recomputed seeds + counts G6 (Task 4); budget=0 R1 (Task 4); surf gen-skip (Task 4); no gateware + Merlin-//e format (throughout). All spec deltas + G1-G6 covered.
**Placeholder scan:** the LIFE8GR alters are concrete (cp + 4 named edits with line refs + the MUL5 code); the GR-dim oracle reuses the DHGR harness (named); the DFB-gen awk is structural (emit DFB lines, ≤50 char) — a real generator, not a TODO; seeds say "recompute offsets for 5-byte/8-bit + count 240" (concrete rule, the exact bytes are the implementer's compute against the format). The render asm is structured (the snapshot-retry + the expand loop pinned). No bare TODOs.
**Type consistency:** LIFEMAPGR equates (ROWBYTES=5/GWIDTH=40/NROWS=48/banks) used Task 1↔2↔3↔4. GBUF row stride r*5 (MUL5) consistent Task 2 (coproc) ↔ Task 3 (//e). text-page interleave + low=top/high=bottom + $0/$F consistent Task 3. budget=0 Task 4. FRONT/GEN @ $0010+u/$0020+u consistent Task 3↔4.
**Executor notes:** Merlin-//e format every .S. The FOUR alters (esp MUL5 — easy to miss) are the LIFE8GR risk; the GR-dim sim oracle (blinker/glider-8byte-boundary/torus) is the gate. The render is //e/bench-only. Auto-gen the blob (no hand-paste). Keep BOTH the DHGR + GR sim oracles (don't clobber). No reflash.
