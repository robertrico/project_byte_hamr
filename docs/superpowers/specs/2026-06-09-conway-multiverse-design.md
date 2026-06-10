# Conway's Multiverse — 8 live DHGR Life universes (design)

**Date:** 2026-06-09
**Branch:** TBD (off `main`, which holds C0→C4 + the SDRAM read window)
**Board:** Byte Hamr Rev 2, `gateware/rev2/project_obscurus/` + `software/SDM/`
**Status:** approved-in-discussion, pending adversarial review
**Builds on:** [[project_coproc_c0]] — coproc with SDRAM read ($E004-08) + write ($E000-03) windows, C4 async skills, SDRAMLIB.

## Why this exists

The coproc is now a full SDRAM read/write processor running registered skills in parallel with
the //e. This is the showpiece that proves it: **8 independent Conway's Life universes living in
SDRAM, all evolving at once, while the //e channel-surfs them in Double Hi-Res.** The coproc is a
free-running multiverse engine; the //e is a viewer that picks which world to watch. A 1 MHz
Apple II showing 8 simultaneously-alive 560×192 cellular worlds it could never store or compute
itself — the active-database thesis made visual.

## Goal

A host program seeds 8 universes, registers + GOes a `LIFE8` coproc skill that forever
round-robins ticking all 8, then renders the selected universe to DHGR and lets the user flip
channels (`0`-`7`). Each universe evolves every ~1-4 s (all 8 always live); flipping channels
shows each one mid-evolution. **No gateware change** (read/write windows exist). Source is
**Merlin-on-//e assemblable** (single-space format, ASCII, left-to-right expressions — see
[[feedback_merlin_source_format]]). Success = on the bench: 8 living DHGR worlds, channel-surf
between them, each kept evolving while unwatched.

## Architecture — two free-running halves, no per-tick handshake

- **Coproc = the multiverse engine.** A resident `LIFE8` skill (host-loaded, registered, GOed
  once) loops forever: `for u in 0..7: tick(u)`. Each `tick` reads universe u's live grid from
  SDRAM, computes the next generation, writes it to the other buffer, flips u's front pointer,
  bumps u's gen counter. Never returns. 8 worlds always advancing.
- **//e = the channel surfer.** Init DHGR + the line table, load/seed/GO the engine, then loop:
  read the keyboard (`0`-`7`/arrows → channel C); if channel C's gen counter changed (or the
  channel changed), read C's live grid from SDRAM and blit it to DHGR. The coproc keeps ticking
  all 8 the whole time.

Coproc owns the *world*, //e owns the *view*. They communicate only through SDRAM (grids + per-
universe front-pointer + gen-counter). The arbiter already prioritizes the host (`c0`) over the
coproc (`c1`), so the //e's render reads win and the coproc ticks in the gaps.

## DHGR — the display (560×192 mono)

DHGR = 560×192, two 8 KB banks (main `$2000-$3FFF` + aux `$2000-$3FFF`). Each scanline's 80 bytes
are **column-interleaved between banks**: `AUX[0] MAIN[0] AUX[1] MAIN[1] … AUX[39] MAIN[39]`,
7 pixels/byte (bit 7 ignored in DHGR mono), 80×7 = 560 px/line.
- **Scanline interleave** is the standard HGR `$400/$80/$28` formula, applied in BOTH banks.
  REUSE `software/ASM/LINEADDR.S` (HI/LO line-base tables, 192 entries via `HEX`) — DHGR shares
  the exact same line bases. The `$2000` bases serve both main and aux (the bank is selected by
  a soft switch, not the address).
- **Enable DHGR** (soft switches, exact sequence bench-verified in the plan): `HIRES` `$C057`,
  `DHIRES` `$C05E`, `80VID` `$C00D`, graphics `$C050`, full-screen `$C052`, `80STORE` `$C001`.
- **Bank aux writes:** with `80STORE` on, `PAGE2` on (`$C055`) routes `$2000-$3FFF` writes to
  AUX, `PAGE2` off (`$C054`) to MAIN.
- **Bit-order invariant (R9 — the whole reason the linear format works):** DHGR displays each
  byte LSB-first (bit 0 = leftmost pixel). The grid uses the same convention — bit 0 of a byte =
  leftmost cell, and "a cell's left neighbor = bit 6 of the previous byte." So the grid bytes ARE
  display bytes: **the blit copies bytes straight, NO bit reversal.** (If the grid packed MSB-
  first, every blit would need a bit-reverse — it doesn't.)

## SDRAM layout

Per universe (u = 0..7): a 560×192 bit-packed grid (7 cells/byte, **80 bytes/row** × 192 =
**15,360 B/buffer**), **double-buffered** (buffer A + buffer B) = 30,720 B, + a 1-byte front
pointer (0=A live, 1=B live) + a 1-byte gen counter. 8 universes ≈ **245 KB** — trivial in the
banked SDRAM (1024 banks).
- **Map (pinned in plan):** each universe u = its own SDRAM bank (UBASE+u), buffer A at addr
  `$0000`, buffer B at `$4000` (both fit one bank's 64 KB: `$3C00` each). Metadata (front
  pointers[8] + gen counters[8]) in a reserved metadata bank. The C-rung scratch is **bank-0
  ADDRESSES** ($0050-$0090), not whole banks — so **any UBASE ≥ 1 is clear** (R7). Two
  insurance pins: put metadata at **offset ≥ $10** within its bank (SDMTEST sweeps offsets
  $00-$09 of ALL 1024 banks → would clobber low-offset metadata), and **never run SDMTEST while
  the multiverse is seeded** (it overwrites every bank). The coproc and //e share these equates.
- **Grid is LINEAR** (row r at buffer_base + r*80, 80 bytes, 7 cells/byte, bit7=0). The coproc's
  Life works on clean linear rows (row above = -80); the DHGR aux/main interleave is purely the
  //e's render concern (de-interleave at blit).

## The coproc `LIFE8` skill (the tick)

Registered skill, ORG $0300, host-loaded (no reflash). Loops forever over the 8 universes.

**tick(u):** read u's front buffer, compute next gen into the back buffer, flip front pointer,
bump gen counter.
- **3-row sliding window** to avoid 9 SDRAM reads/cell: keep rows r-1, r, r+1 in coproc BRAM
  (3 × 80 = 240 bytes of BRAM scratch), read each SDRAM row once (read window `$E004-08`,
  **auto-increments** the read pointer), compute row r's next gen from the in-BRAM window, write
  row r to the back buffer (write window `$E000-03`), slide. ~1 SDRAM read/cell + bit-packed writes.
- **Asymmetry (R2 — must-pin):** the READ window auto-increments (`{rbank,raddr}+1` on done,
  coproc.v:215-222) but the **WRITE window does NOT** (`{sbank,saddr}` never auto-incs). So the
  back-buffer write must carry its own pointer: per byte, `STA $E000` (lo, + `$E001` hi at a page
  cross) then `STA $E003` (data). Cost is still trivial (~9 ms/tick), but the algorithm threads a
  write pointer it bumps in software. (A symmetric "writes also auto-inc" assumption would be
  WRONG.)
- **Bit-packed neighbor count (THE hard part — flag for review):** each byte holds 7 cells
  (bits 0-6). For each cell, sum its 8 neighbors: up/down (same bit position in the rows above/
  below), left/right (adjacent bits, crossing byte boundaries — cell 0's left neighbor is bit 6
  of the previous byte; cell 6's right is bit 0 of the next byte), and the 4 diagonals. Apply the
  rule: live cell with 2-3 neighbors survives, dead cell with exactly 3 is born. The byte-
  boundary + bit-position arithmetic on a 6502 is the fiddly core — the plan pins the exact
  algorithm (likely: expand each window row to a per-cell count via shift-and-add, or a
  3-row column-sum running total).
- **Torus wrap:** vertical — row -1 = row 191, row 192 = row 0 (the sliding window wraps at top/
  bottom). Horizontal — cell -1 = cell 559 (bit 6 of byte 79), cell 560 = cell 0 (bit 0 of byte
  0). So the leftmost/rightmost cells of each row read their missing neighbor from the opposite
  end. (Dead-edge fallback if wrap proves too costly: treat off-grid as dead — a build-time flag.)
- **Atomic swap — ORDER IS LOAD-BEARING (R5, REQUIRED):** after all 192 rows are written to the
  back buffer, **flip u's front pointer FIRST, THEN increment u's gen counter** (two single-byte
  writes, in that order). If gen bumped first, the //e could see the new gen with the stale
  pointer → blit the old/being-overwritten buffer. The //e's matching order: **read gen → (if
  changed) read front pointer → blit that buffer.** So a gen change always implies the pointer
  already points at the freshly-completed buffer.
- **Reentrancy / ZP:** LIFE8 is a single forever task (slot 0) — it can use the slot's ZP window
  + BRAM scratch freely. It never `JMP DONE`s (runs forever).
- **MUST spawn with run-budget = 0 (REQUIRED — R1):** the C4 kernel has a watchdog — `DISPATCH`
  decrements `BUDGET[slot]` per tick and `FORCEDONE` kills the task at 0 (kernel.S:286-304,
  proven by C4 test #6). A nonzero budget would kill LIFE8 after ≤255 ticks. The escape hatch is
  `BEQ DBNEXT` at kernel.S:294 — **budget 0 = no limit.** So LIFE8's mailbox budget byte MUST be
  0. On the CALL path the budget loads from the mailbox (kernel.S:270); if the plan uses batch-GO
  (`BOOTSTRAP`/`SPAWN`) instead, it MUST confirm `BUDGET[slot]` lands as 0 on that path too
  (CLEARSLOTS zeros it at boot — verify no path sets it nonzero). With budget 0, the timer just
  preempts the single task to itself — it runs forever.
- **Size:** LIFE8 code + 240 B row-window scratch must fit the task region ($0300-$0F7F, ~3.3 KB)
  alongside the C4 mailbox at $0F80. A Life tick is a few hundred bytes; fits with room.

## The //e `MVERSE` app (the surfer)

ORG $6000 (or $2000 if it conflicts with the DHGR page — pin in plan). `PUT SDRAMLIB`,
`PUT LINEADDR` (the line table). Single-space Merlin format, ASCII, lean.
1. **Init:** enable DHGR (soft switches), clear both HGR banks. The line-base table comes from
   `LINEADDR.S` (reused).
2. **Load + register + seed + GO:** load LIFE8 into coproc $0300 (load port, LOADBLK pattern) +
   TABLE[0]=$0300; write 8 seed patterns into the 8 universes' buffer-A via SDRAMLIB (SDM_WRITE/
   SDM_WRNEXT); set all front pointers = 0 (A live), gen counters = 0; GO LIFE8 (CP_CALL slot 0 /
   batch GO).
3. **Surf loop:**
   - Poll keyboard ($C000 bit7); on a key, read it then **clear the strobe (`$C010`)**: `0`-`7`
     set channel C; arrows C±1 (wrap 0-7); maybe `R` reseed.
   - Read C's gen counter (SDRAMLIB). If unchanged AND channel unchanged → skip (no redundant
     blit — the gen-counter optimization; avoids re-blitting a static frame).
   - Else **render C:** read C's front-pointer → its live buffer; for each line Y (0-191): read
     the 80 grid bytes (SDM_RDNEXT walk), **de-interleave** (even bytes → AUX, odd bytes → MAIN),
     write 40 bytes to AUX[lineaddr(Y)] (PAGE2 on) + 40 to MAIN[lineaddr(Y)] (PAGE2 off).
   - Loop.

**Render cost (honest, flag for review):** reading 15,360 grid bytes via SDRAMLIB (~10-15 cyc/
byte) ≈ 200-250 ms + the de-interleave/blit writes ≈ ~300 ms/frame → ~3 fps. The gen-counter
optimization means the //e only pays this when the watched universe actually advanced (~once/sec
or per channel flip), spending the rest polling keys cheaply.

## Seeding (8 varied starts)

The host writes 8 distinct patterns into the universes' buffer-A before GO (SDRAMLIB), for visual
variety: a Gosper glider gun (u0), gliders (u1), oscillators garden (u2), an r-pentomino /
methuselah (u3), random soup (u4-u7, host PRNG). Patterns stored as bit-packed rows matching the
grid format. Keep the seed data compact (a few patterns + a soup generator).

## Sync / tearing (flag for review)

Double-buffer + per-universe front-pointer + gen-counter. The coproc writes the BACK buffer while
the //e reads the FRONT; the swap is a single front-pointer flip after a full gen. The //e reads
the front pointer once at the start of a blit, then reads that buffer.
**What actually prevents corruption (R3 — the correct rationale): the round-robin.** A naive
"mid-blit flip is harmless / just 1-gen-old" claim is WRONG in general — after a flip the buffer
the //e is reading becomes the coproc's NEXT write target, so it COULD be overwritten mid-blit
→ a mixed-generation frame. What saves it is the schedule: after ticking universe C, the coproc
ticks the other 7 before returning to C — **~1.5-3 s of round-robin >> the ~300 ms blit.** So
while the //e blits C, the coproc is off in other universes and will not touch C's buffers. The
double-buffer + the round-robin gap give the //e a stable buffer for the whole blit. **This
silently breaks if the universe count drops to 1-2 or ticks get much faster** (the coproc would
return to C mid-blit) — a documented invariant, not a coincidence. (Belt-and-suspenders if ever
needed: //e re-checks the gen counter after the blit and re-blits on change.)

## Components / files (new branch off `main`)
- `software/SDM/LIFE8.S` — the coproc Life skill (forever loop, 3-row window tick, bit-packed
  neighbor count, torus, atomic swap, gen bump). Merlin-//e format. Assembles via Merlin32 for
  sim/bench. Embedded as DFB bytes in MVERSE for loading (or loaded from disk).
- `software/SDM/MVERSE.S` — the //e app (DHGR init, load/register/seed/GO LIFE8, surf loop,
  de-interleave DHGR render). `PUT SDRAMLIB`, `PUT LINEADDR`.
- REUSE `software/ASM/LINEADDR.S` (line-base table — copy/PUT into SDM) + `software/SDM/SDRAMLIB.S`.
- Makefile: `life8`/`mverse` targets + sdmdisk pack. No gateware target (no RTL change).
- `gateware/rev2/project_obscurus/project_obscurus_tb.v` — sim: seed a SMALL grid, run a few
  LIFE8 ticks, assert the next gen matches a hand-computed Conway step (the bit-packed math +
  torus correctness). (Full 560×192 × 8 is too slow for sim — test the tick on a tiny grid.)

## Error handling / edge cases
- **Bit-packed byte boundaries + torus wrap** — the core correctness risk; verified by the
  small-grid sim against a known Conway step (incl. a glider that crosses a byte boundary AND
  wraps an edge).
- **The never-DONE task + watchdog (R1)** — LIFE8 runs forever, so it MUST be spawned with
  run-budget = 0 (no limit) or the C4 watchdog `FORCEDONE`s it after ≤255 ticks. Pinned above.
- **SDRAM window regs are GLOBAL, not per-slot context (R4 — pinned constraint):** `$E000-$E008`
  (the read + write pointers/data) are coproc-global registers, NOT saved/restored on a context
  switch. Safe here ONLY because LIFE8 is the sole SDRAM-touching task. **Constraint: no second
  SDRAM-touching skill may run while LIFE8 runs** (a context switch between two SDRAM users would
  corrupt each other's pointers). Also verify the kernel tick ISR never touches `$E00x` (believed
  clean — the ISR uses ZP/stack + the C4 control regs only).
- **SDRAM map collisions** — UBASE must avoid the low-bank scratch ($0061 results, $0080 test).
  Pin UBASE so the 8 universe banks + metadata bank are clear of prior usage.
- **Arbiter contention** — the //e render (host, c0) starves the coproc (c1) during a blit (~300
  ms). At ~1 gen/sec the coproc easily catches up between blits. If render is too greedy, throttle
  (the gen-counter skip already does).
- **DHGR aux banking** — getting the soft-switch sequence + PAGE2 timing right is bench-fiddly;
  the de-interleave (even→aux/odd→main) must match the hardware's column order exactly. Verify
  with a known test pattern (a single vertical line at a known x → check it lands in the right
  bank/byte).
- **Merlin-//e format** — single spaces, labels col 0, ASCII, <=50 char lines, left-to-right
  expressions. Verify `grep -cP '\t'`==0 + ASCII on every shipped .S.

## Testing
- **Sim — tick correctness (the load-bearing test):** seed a small grid with a known pattern (a
  glider + a blinker), run N LIFE8 ticks via the kernel, read the result back, assert it equals
  the hand-computed Conway generations — including a glider crossing a 7-cell byte boundary and a
  pattern wrapping a torus edge. **Reduce ONLY the row count via an equate (R10) — KEEP the
  80-byte row width**; shrinking the width would test different byte-boundary arithmetic than the
  ship code. Note (R10): the torus row-191/row-0 vertical reads come from the FRONT buffer
  (untouched during the tick), so re-reading them is fine — or cache row 0 in +80 B of scratch
  (plan detail). iverilog `-g2005`. This pins the bit-packed neighbor math + torus, which the
  bench can't verify pixel-exact.
- **Build:** Merlin32 assembles LIFE8 + MVERSE clean (and they're Merlin-//e-format-valid). No
  gateware build needed (no RTL change). The existing bitstream runs it.
- **Bench:** flash NOT required (gateware unchanged — the current flashed bitstream works). Boot
  the disk, `BRUN MVERSE` → 8 DHGR universes; flip `0`-`7`, watch each one alive + evolving;
  leave one for a while, flip back, it's advanced. Screenshot.

## Success criteria
8 independent Life universes evolve simultaneously in SDRAM (coproc free-running), the //e renders
any one in DHGR and channel-surfs between them, each kept advancing while unwatched — on the
bench, no gateware change, Merlin-//e-assemblable source. The active-database multiverse, alive.

## Non-goals
- No hardware Life engine — "Conway's Life Engine" (parallel @100 MHz fabric) is a SEPARATE
  future project. This is the soft-6502 + SDRAM version on the current stack.
- No multi-core (one coproc round-robins the 8; partitioning universes across cores is a separate
  rung gated on the multi-core build).
- No color DHGR (mono 560×192 only).
- No interactive editing / pause / step (just seed → run → surf; reseed optional).
- No grid bigger than the BRAM-window/SDRAM-bandwidth allows at "a few sec/gen" — speed is
  accepted as slow-but-alive, not real-time.
- No new gateware, no reflash.
