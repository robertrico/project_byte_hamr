# Conway's Multiverse Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans. Steps use checkbox (`- [ ]`) syntax.

**Goal:** 8 independent Conway's Life universes (560×192 DHGR mono) living in SDRAM, all evolving at once via a free-running coproc skill, while the //e channel-surfs them in Double Hi-Res.

**Architecture:** Coproc runs a resident `LIFE8` skill that forever round-robins ticking all 8 universes (3-row sliding-window bit-packed Life, torus, double-buffered). The //e (`MVERSE`) loads/seeds/GOes it, then renders the selected universe to DHGR (aux/main de-interleave) and flips channels. No gateware change.

**Tech Stack:** Merlin 6502 — **Merlin-on-//e format** (single-space delimited, labels col 0, one leading space for unlabeled, ASCII, ≤50-char lines, left-to-right expressions — see [[feedback_merlin_source_format]]; match `software/SDM/SDRAMLIB.S` + `software/ASM/LINEADDR.S`). Built via Merlin32 for sim/bench. iverilog `-g2005` for the tick sim. Branch `conway-multiverse`. Spec: `docs/superpowers/specs/2026-06-09-conway-multiverse-design.md`.

**Reuse:** `software/SDM/SDRAMLIB.S` (host SDRAM R/W), `software/ASM/LINEADDR.S` (HI/LO line-base tables ×192 — DHGR shares HGR line bases). Coproc SDRAM read window `$E004` RADDR_LO / `$E005` HI / `$E006` RBANK / `$E007` RTRIG (STA→read+latch+autoinc) / `$E008` RDATA; write window `$E000` SADDR_LO / `$E001` HI / `$E002` SBANK / `$E003` SDATA (STA→write, **no autoinc**).

---

## Fixed layout (every task references these)

**Grid:** 560×192, 7 cells/byte (**bit 0 = leftmost** — DHGR LSB-first, no bit reversal on blit), 80 bytes/row, 192 rows = **15,360 B/buffer**. Bit set = live cell.

**SDRAM map** (coproc + host SHARE these equates):
- `UBASE = 16` (universe banks 16..23 — clear of bank-0 scratch at $0050-$0090). Universe u = bank `UBASE+u`.
- Within a universe bank: buffer A at addr `$0000`, buffer B at addr `$4000`.
- Metadata bank `MBANK = 24`: `FRONT[u]` at addr `$0010+u` (0=A live, 1=B live), `GEN[u]` at addr `$0020+u` (free-running gen counter). (Offset ≥$10 so SDMTEST's $00-$09 sweep can't clobber it — and never run SDMTEST while seeded.)

**Coproc regs:** read `$E004-08`, write `$E000-03` (above). C4 spawn: LIFE8 = skill, **run-budget MUST be 0** (no watchdog kill).

**`$E000-08` are GLOBAL (not per-slot)** — LIFE8 is the ONLY SDRAM-touching task while it runs.

**DHGR:** main+aux `$2000-$3FFF`. Per line: 80 bytes = `AUX[0] MAIN[0] AUX[1] MAIN[1]…` → grid byte even→aux, odd→main. Line base from `LINEADDR.S`. Enable: `$C057`(HIRES) `$C05E`(DHIRES) `$C00D`(80VID) `$C050`(GR) `$C052`(full) `$C001`(80STORE); aux via `$C055`(PAGE2) / main via `$C054`.

---

## File Structure
- `software/SDM/LIFE8.S` — coproc Life skill: single-universe tick (Task 2) + 8-universe forever loop (Task 3). The hard part.
- `software/SDM/MVERSE.S` — //e app: DHGR render (Task 4) + orchestration/seeds/surf (Task 5).
- `software/SDM/LIFEMAP.S` — shared equates (SDRAM map, regs) `PUT` into both LIFE8 + MVERSE so they agree.
- REUSE: `SDRAMLIB.S`, `LINEADDR.S` (copy into `software/SDM/` for the disk, or `PUT` by path).
- `gateware/rev2/project_obscurus/project_obscurus_tb.v` — extend: the tick-correctness sim (Task 2).
- `Makefile` — `life8`/`mverse` targets + sdmdisk pack.

---

## Task 1: Shared equates + SDRAM map (`LIFEMAP.S`)

**Files:** Create `software/SDM/LIFEMAP.S`.

- [ ] **Step 1: Write `LIFEMAP.S`** — the shared map/regs both LIFE8 and MVERSE PUT. Merlin-//e format (single spaces, labels col 0, left-to-right expressions, ASCII, lean).
```
* LIFEMAP.S - SHARED MULTIVERSE EQUATES
* PUT INTO LIFE8 AND MVERSE
UBASE = 16
MBANK = 24
BUFA = $0000
BUFB = $4000
ROWBYTES = 80
NROWS = 192
NUNIV = 8
* COPROC SDRAM WINDOWS
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
(Note: `ROWBYTES*NROWS = 80*192 = 15360 = $3C00` per buffer; A at $0000, B at $4000 — no overlap.)

- [ ] **Step 2: Verify Merlin-//e format** — no tabs, ASCII only:
```bash
cd /Users/hambook/Development/project_byte_hamr
grep -cP '\t' software/SDM/LIFEMAP.S    # expect 0
grep -cP '[^\x00-\x7F]' software/SDM/LIFEMAP.S   # expect 0
```
Expected: both 0.

- [ ] **Step 3: Commit**
```bash
git add software/SDM/LIFEMAP.S
git commit -m "feat(multiverse): shared SDRAM-map + window equates (LIFEMAP)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: LIFE8 single-universe tick + sim correctness oracle (THE hard task)

**Files:** Create `software/SDM/LIFE8.S` (the `TICK1` routine); extend `gateware/rev2/project_obscurus/project_obscurus_tb.v`.

The bit-packed Life is the core risk. **TDD: the sim is the correctness oracle — write it first, make `TICK1` pass it.** The exact 6502 bit-fiddling will be iterated against the sim; the algorithm + the oracle are pinned here.

### The algorithm (`TICK1` — one universe, one generation)
Inputs: read-bank `RB` (the front buffer's bank+base), write-bank `WB` (the back buffer's bank+base). Three 80-B BRAM row buffers `ROWBUF0/1/2` with three role indices `UP`/`MID`/`DN` that ROTATE (a TRUE sliding window — read ONE new row per output row, not three).
1. **Prime** (before the row loop): read row 191 → the UP buffer (torus: row -1 = row 191), row 0 → MID, row 1 → DN.
2. **For each row r in 0..191:** compute ROWOUT from UP/MID/DN (steps 2-3 below), write it, then **SLIDE**: rotate roles (UP←MID, MID←DN), and read the next bottom row into the freed buffer (the new DN). For r in 0..189 the new DN = row r+2; for r=190 the new DN = row 0 (wait — the slide reads the row that will be DN for r+1, i.e. row (r+1)+1 = r+2, mod 192). So r=190 reads row 0 (torus), r=191 needs no further read (last row). Net: **192 reads (+1 prime) per gen, not 576** — 3× fewer SDRAM ops on the coproc's dominant cost.
- **Reading one row** (the read window): set `RADDRLO/HI` to the row's byte offset (`rowindex*80`), `RBANKR` to RB, then 80× (`STA RTRIG`/`LDA RDATA`) — the read pointer auto-increments, so one trigger/fetch per byte walks the row. (Torus vertical handled by the prime + the wrap read above.)
- Rotation is by INDEX, not copy: keep a 3-entry table of buffer base addresses; `UP/MID/DN` are indices into it that cycle 0→1→2→0. No byte copying.
3. **Column sums** `COLSUM[0..559]`: for each column x, `COLSUM[x] = bit(UP,x) + bit(MID,x) + bit(DN,x)` (0..3), where UP/MID/DN are the rotating row buffers. Iterate byte-by-byte (80 bytes) × bit-by-bit (bits 0..6), tracking column x = byte*7+bit incrementally (NO mod-7 division — use loop counters).
4. **Build result row** `ROWOUT` (80 B): for each column x, `n = COLSUM[(x-1)%560] + COLSUM[x] + COLSUM[(x+1)%560] - bit(MID,x)`; `alive = (bit(MID,x) & (n==2 | n==3)) | (~bit(MID,x) & (n==3))`; set bit x of `ROWOUT`. (Torus horizontal: x-1 of column 0 → column 559; x+1 of column 559 → column 0.) Pack bit 0 = leftmost.
5. **Write `ROWOUT`** (80 B) to the back buffer row r: set `SADDRLO/HI` = `r*80`, `SBANKR` = WB; per byte bump the write pointer (`STA SADDRLO`, +`SADDRHI` on page cross — **write window has NO autoinc**), `STA SDATA`. (Carry a software write pointer.)

**Scratch BRAM — PINNED addresses (P3; below the $0F80 C4 mailbox, above the code at $0300):**
```
ROWBUF0 = $0C00   (80 B)   ; the 3 rotating row buffers
ROWBUF1 = $0C50   (80 B)
ROWBUF2 = $0CA0   (80 B)
ROWOUT  = $0CF0   (80 B)   ; result row being built
COLSUM  = $0D40   (560 B -> $0F6F)  ; ends below the $0F80 mailbox
```
Code occupies $0300-$0BFF (~2.3 KB — ample for the tick + loop). The 3-entry buffer-base table (for the index rotation) holds ROWBUF0/1/2. NO overlap: code < $0C00 ≤ scratch < $0F80 mailbox.

### Step 1: Write the sim correctness oracle in `project_obscurus_tb.v`
**The harness is the EXISTING skill-load pattern (P1 — pinned, no new mechanism):** reuse the
tb's proven idiom — the sdrtest/cmpskill blocks do exactly this (project_obscurus_tb.v ~L630-660):
1. Set `CP_LADDR=$0300` (`wr_reg 4'h9/4'hA`), `load_byte` each LIFE8 byte → coproc BRAM $0300.
2. Write TABLE[0]=$0300 (`load_byte $00,$03` at coproc $0200).
3. `stage_mbox(2'd0, 8'h00, 8'h00, 8'h00)` — slot 0, skill 0, **budget 0 (no watchdog kill, R1)**,
   arg 0. `ring(2'd0)`.
LIFE8 then runs forever — so **DON'T use `wait_done` (it never DONEs); SYNC ON `GEN[u]`** instead:
poll `sdram_read(MBANK, $0020+u, gen)` until it increments = one tick of universe u completed
(deterministic). Then `sdram_read` the front buffer rows back and assert.
**Sim scaling:** set `LROWS` small (e.g. 8) AND `NUNIV`=1 (test universe 0 only) via equates so a
tick is fast in sim — but KEEP `ROWBYTES`=80 (R10: shrinking width changes the byte-boundary math).
The seed + oracle: place a **glider** (travels + crosses a 7-cell byte boundary over gens) + a
**blinker** (period-2, the simplest hand-checkable oracle) + an edge pattern (**torus wrap**); seed
via `sdram_write(UBASE, addr, byte)` into buffer A, `FRONT[0]=0`. After each `GEN[0]` increment,
read back + assert the result equals the **hand-computed** Conway generation.
```verilog
        // ===== Multiverse: TICK1 correctness (the load-bearing test) =====
        // seed bank UBASE(16), buffer A: a blinker (3 horizontal cells) + a glider, + an
        // edge cell for torus. Use monitor-port writes (sdram_write) to set the bytes.
        // blinker at row 5, cols 10-12 -> byte 1 (cols 7-13), bits 3,4,5 set = $38
        // <<< seed bytes via sdram_write(bank, addr, val) >>>
        // run TICK1 once (gen 1): blinker should rotate to VERTICAL (cols 11, rows 4-6)
        // <<< trigger one tick >>>
        // read back the 3 affected bytes; assert blinker is now vertical (col 11 set in rows 4,5,6)
        // run again (gen 2): blinker back to horizontal -> assert original
        // glider: after 4 ticks it has moved +1,+1 (assert its new cell positions, incl a
        //   byte-boundary crossing)
        // torus: a live cell at col 0 row 0 with neighbors at col 559/row 191 -> assert wrap
        if (life_errors==0) $display("PASS multiverse TICK1 (blinker/glider/byte-boundary/torus)");
        else $display("FAIL multiverse TICK1 %0d", life_errors);
```
(IMPLEMENTER: compute the expected generations BY HAND for the seed patterns and hardcode the asserts. The blinker is the simplest oracle — horizontal↔vertical each gen. The glider + torus assertions catch the byte-boundary + wrap bugs the bench can't see. Reduce `LROWS` so the sim runs fast, but keep ROWBYTES=80.)

### Step 2: Run — expect FAIL (no TICK1)
```bash
cd /Users/hambook/Development/project_byte_hamr && make sim DESIGN=project_obscurus REV=rev2 2>&1 | grep -iE "multiverse|FAIL"
```
Expected: FAIL / no TICK1.

### Step 3: Implement `TICK1` in `LIFE8.S` (the algorithm above), Merlin-//e format
Write the 3-row read, COLSUM build, ROWOUT build (rule + torus), and the back-buffer write (software write pointer). Single-space format, ASCII, lean, ≤50-char lines, left-to-right expressions. Iterate against the sim (Step 4) — this is intricate bit-fiddling; the oracle pins correctness.

### Step 4: Run — expect PASS
```bash
make sim DESIGN=project_obscurus REV=rev2 2>&1 | grep -iE "multiverse|PASS|FAIL"
```
Expected: `PASS multiverse TICK1`. The blinker oscillates, the glider travels (incl. byte-boundary), torus wraps. Also confirm all prior tests still PASS (the tb additions are additive). Format check: `grep -cP '\t' software/SDM/LIFE8.S` == 0, ASCII == 0.

### Step 5: Commit
```bash
git add software/SDM/LIFE8.S gateware/rev2/project_obscurus/project_obscurus_tb.v
git commit -m "feat(multiverse): LIFE8 TICK1 - bit-packed 3-row-window Life + torus, sim-verified (blinker/glider/boundary)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: LIFE8 multiverse loop (round-robin 8, swap, gen bump, forever)

**Files:** Modify `software/SDM/LIFE8.S` (add the `LIFE8` entry that loops over universes + calls `TICK1`).

### Step 1: Write the loop
The skill entry `LIFE8` (at $0300, the registered entry): forever, for u in 0..7: read FRONT[u] from MBANK → derive RB (front buffer bank+base) + WB (back buffer); `JSR TICK1`; then **flip FRONT[u] FIRST, then bump GEN[u]** (R5 order — load-bearing). Loop. Never `JMP DONE`.
```
LIFE8
 LDX #0           ; u = 0
LULOOP
 STX CURU
* read FRONT[u] from MBANK addr $0010+u -> A (0 or 1)
 ... SDRAM read FRONT[u] ...
* RB = bank UBASE+u, base = (FRONT? BUFB : BUFA) ; WB = the other
 ... compute RB/WB ...
 JSR TICK1
* swap: FRONT[u] = 1 - FRONT[u]  (FLIP FIRST)
 ... write FRONT[u] ^ 1 to MBANK $0010+u ...
* then bump GEN[u]  (AFTER the flip)
 ... read GEN[u], INC, write to MBANK $0020+u ...
 LDX CURU
 INX
 CPX #NUNIV
 BCC LULOOP
 LDX #0
 JMP LULOOP       ; forever
```
(TICK1 reads the FRONT buffer, writes the BACK buffer; after, FRONT flips so the just-written BACK becomes the new FRONT. Flip-before-bump so the //e never sees a new gen with a stale pointer — R5.)

### Step 2: confirm budget=0 spawn (R1)
LIFE8 runs forever — it MUST be spawned with run-budget 0 or the C4 watchdog kills it (kernel.S:286-304). The HOST (Task 5) sets the mailbox budget byte = 0 on CALL. Document in LIFE8.S a header comment: `* SPAWN WITH BUDGET=0 (NO WATCHDOG KILL)`. If using batch-GO instead of CALL, the plan/host must confirm BUDGET[slot] is 0 on that path (CLEARSLOTS zeros it; verify no path sets it).

### Step 3: assemble + commit
```bash
cd /Users/hambook/Development/project_byte_hamr && make life8 2>&1 | tail -3   # add the target
grep -cP '\t' software/SDM/LIFE8.S    # 0
```
Expected: assembles, no tabs. Note byte length (must fit task region with the 880B scratch). Commit:
```bash
git add software/SDM/LIFE8.S Makefile
git commit -m "feat(multiverse): LIFE8 forever loop - round-robin 8 universes, flip-before-bump, budget=0

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: MVERSE DHGR render (init, line table, aux/main de-interleave)

**Files:** Create `software/SDM/MVERSE.S` (the DHGR render half); copy `software/ASM/LINEADDR.S` → `software/SDM/LINEADDR.S` (for the disk).

### Step 1: DHGR enable + clear
Write `DHGRON` (soft-switch sequence) + `DHGRCLR` (zero both banks via PAGE2 toggle). Merlin-//e format.
```
DHGRON
 STA $C001        ; 80STORE
 STA $C057        ; HIRES
 STA $C05E        ; DHIRES
 STA $C00D        ; 80VID
 STA $C052        ; FULL
 STA $C050        ; GRAPHICS
 RTS
```
(DHGRCLR: PAGE2 on, clear $2000-$3FFF; PAGE2 off, clear $2000-$3FFF.)

### Step 2: line-base table
`PUT LINEADDR` gives `HI`/`LO` tables (192 entries each, the $400/$80/$28 interleave — DHGR uses the same bases). Confirm LINEADDR.S has 192 HI + 192 LO bytes.

### Step 3: `RENDER` — blit channel C's front buffer to DHGR (de-interleave)
For the current channel C: read FRONT[C], pick its buffer (RB), then for each line Y in 0..191: read the 80 grid bytes from SDRAM (SDM_RDNEXT walk), split even→aux/odd→main, write 40 aux bytes (PAGE2 on) + 40 main bytes (PAGE2 off) to base `HI[Y]:LO[Y]`. **Grid bytes copy straight — bit 0 = leftmost, no reversal (R9).**
```
* per line Y: SDRAM row base = Y*80 ; read 80 bytes -> split
* even idx (0,2,4..78) -> AUX[base+0..39] ; odd (1,3..79) -> MAIN[base+0..39]
```
(IMPLEMENTER: the 40 aux + 40 main writes use the LINEADDR base for line Y; PAGE2 selects the bank. Keep $C0Cx reads non-indexed per SDRAMLIB invariant.)

### Step 4: a render smoke test (host-side, on bench — or a static-pattern check)
Hard to unit-sim the //e DHGR. Verify on bench (Task 6): a seeded known pattern (e.g. a vertical line at col 0) must land at the correct screen position/bank — confirms the de-interleave + line table. For now: assemble clean.
```bash
make mverse 2>&1 | tail -3   ; grep -cP '\t' software/SDM/MVERSE.S   # 0
```

### Step 5: Commit
```bash
git add software/SDM/MVERSE.S software/SDM/LINEADDR.S Makefile
git commit -m "feat(multiverse): MVERSE DHGR render - enable + line table + aux/main de-interleave blit

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 5: MVERSE orchestration — load/register/seed/GO + channel surf

**Files:** Modify `software/SDM/MVERSE.S`.

### Step 1: load + register LIFE8
Embed LIFE8's bytes as a `DFB` block (from `xxd software/SDM/LIFE8` — or load LIFE8 from the disk). `REGSKILL`: LOADBLK the bytes to coproc $0300 (load port, mirror CPDEMO.S), TABLE[0]=$0300.

### Step 2: seed 8 universes
`SEED`: write 8 patterns into universes 0..7 buffer A (FRONT[u]=0), via SDRAMLIB (SDM_SETBANK bank UBASE+u, SDM_SETADDR, SDM_WRNEXT the pattern bytes). Patterns: u0 Gosper glider gun, u1 gliders, u2 oscillators, u3 r-pentomino, u4-u7 random soup (a simple PRNG: LFSR seeded from a counter). Zero FRONT[u]/GEN[u] in MBANK. Keep seed data compact.
**u0 (Gosper gun) is the centerpiece — pin it as EXACT verified bytes.** It's the canonical 36×9
pattern; a single wrong byte = no gliders = no centerpiece. Hand-place it into the 7-cells/byte
layout, then VERIFY in the Task 2 sim (seed the gun, run ~30 gens, assert a glider emerges) before
trusting it on the bench. The other seeds are forgiving; the gun is not.

### Step 3: GO LIFE8 with budget=0 (R1)
Stage LIFE8's mailbox: skill_id 0, **budget byte = 0**, then CP_RING (CALL slot 0). (Or batch GO with budget confirmed 0.) LIFE8 starts ticking all 8.

### Step 4: surf loop
```
SURF
* poll keyboard
 LDA $C000
 BPL NOKEY
 STA $C010        ; clear strobe (R9)
 ... '0'-'7' -> CHAN ; arrows CHAN +/-1 wrap 0-7 ...
NOKEY
* read GEN[CHAN]; if changed since last OR chan changed -> RENDER
 ... read GEN[CHAN] from MBANK ; compare LASTGEN ...
 ... if different: JSR RENDER ; store LASTGEN ...
 JMP SURF
```
(The gen-skip avoids re-blitting a static frame — the //e only pays the ~300ms blit when the watched universe advanced or you flip channels.)

### Step 5: assemble + disk
```bash
make life8 mverse 2>&1 | tail -3
make sdmdisk 2>&1 | tail -5      # EDIT the sdmdisk recipe (Makefile ~L529): add the MVERSE
                                 # pack line (+ LIFE8 if loaded from disk vs embedded as DFB);
                                 # bump the .po size if the existing image is full
grep -cP '\t' software/SDM/MVERSE.S   # 0
grep -cP '[^\x00-\x7F]' software/SDM/MVERSE.S  # 0
```
Expected: both assemble clean, Merlin-//e-format-valid, MVERSE in the disk catalog.

### Step 6: Commit
```bash
git add software/SDM/MVERSE.S Makefile
git commit -m "feat(multiverse): MVERSE orchestration - load/register/seed/GO LIFE8 + channel-surf loop

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 6: Build + bench-ready

- [ ] **Step 1: Full sim regression** — `make sim DESIGN=project_obscurus REV=rev2 2>&1 | grep -iE "PASS|FAIL"`. The `PASS multiverse TICK1` + all prior tests (monitor/C1-C4/C-flash/sdram-read), 0 errors. **No gateware build needed** (no RTL change — the flashed bitstream runs it).
- [ ] **Step 2: Confirm Merlin-//e format on all shipped .S** — `for f in LIFEMAP LIFE8 MVERSE; do grep -cP '\t' software/SDM/$f.S; grep -cP '[^\x00-\x7F]' software/SDM/$f.S; done` → all 0. No mixed `+...*` precedence expressions.
- [ ] **Step 3: Hand the bench procedure to the user** (no flash — gateware unchanged):
```
1. make sdmdisk DESIGN=project_obscurus REV=rev2
2. Boot the disk (the CURRENT flashed bitstream runs it - no reflash).
   BRUN MVERSE
   -> 8 DHGR Life universes, all evolving. Press 0-7 to flip channels;
      each world keeps advancing while unwatched. Leave one, flip back, it moved on.
```
**Honest tearing note (P4):** the active channel may show occasional tearing — the ~300 ms blit
can span a coproc front-pointer flip if a round-robin pass happens to be shorter than the blit.
It self-heals on the next gen-change render (the gen-skip re-blits). Not corruption, just a
transient torn frame on the watched universe. Fine for the demo; don't claim flawless.
- [ ] **Step 4: Record** — update `project_coproc_c0.md` + `MEMORY.md`: Conway's Multiverse (8 live DHGR universes, LIFE8 free-running skill, //e channel-surf), the bit-packed-Life + DHGR-de-interleave + budget=0 + flip-before-bump notes, and that the hardware "Conway's Life Engine" is the separate future project.

---

## Self-Review
**Spec coverage:** 8 universes/DHGR/SDRAM layout → Task 1 + 4; LIFE8 tick (3-row window, bit-packed, torus, write-pointer R2) → Task 2; round-robin + flip-before-bump (R5) + budget=0 (R1) → Task 3; DHGR de-interleave + line table + bit-order invariant (R9) + KBDSTRB → Task 4; orchestration/seed/GO/surf + gen-skip → Task 5; sim oracle (R10 rows-only) → Task 2; SDRAM map (R7 UBASE≥1, metadata ≥$10) → Task 1; global-regs/sole-task (R4) → Task 3 note; no-gateware + Merlin-//e format → throughout. Tearing (R3) is a runtime invariant (round-robin gap), no task action — documented in spec.
**Placeholder scan:** the TICK1 6502 (Task 2 Step 3) and the seed patterns (Task 5) are flagged as implementer-iterated-against-the-oracle / compact-data — the ALGORITHM + the correctness TEST are fully pinned (the test is the spec of correct behavior; TDD drives the asm). The de-interleave/render asm (Task 4) gives the structure + the bench check. Not bare TODOs — each has a concrete oracle/structure. The intricate bit-packed asm genuinely requires sim iteration; the plan pins the algorithm + the pass/fail gate.
**Type/label consistency:** LIFEMAP equates (UBASE/MBANK/BUFA/BUFB/ROWBYTES/regs) used consistently Task 1↔2↔3↔5. FRONT[u]/GEN[u] at MBANK $0010+u/$0020+u consistent Task 3↔5. Grid bit0=leftmost / even-aux-odd-main consistent Task 2(pack)↔4(blit). budget=0 (R1) consistent Task 3↔5. flip-before-bump (R5) Task 3.
**Executor notes:** Merlin-//e format on EVERY .S (single spaces, ASCII, ≤50 char, left-to-right) — verify with grep. The TICK1 sim oracle is the load-bearing gate (blinker is the simplest hand-checkable; glider+torus catch boundary/wrap). No gateware change/no reflash. budget=0 or LIFE8 dies. Reduce sim rows via LROWS only, keep 80-byte width.
