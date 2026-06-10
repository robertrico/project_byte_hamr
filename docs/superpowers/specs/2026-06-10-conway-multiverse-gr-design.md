# Conway's Multiverse — Lo-Res (GR) variant (design)

**Date:** 2026-06-10
**Branch:** TBD (off `main`, which holds the DHGR multiverse)
**Board:** Byte Hamr Rev 2, `gateware/rev2/project_obscurus/` + `software/SDM/`
**Status:** approved-in-discussion, pending adversarial review
**Builds on:** [[project_coproc_c0]] — the bench-verified DHGR Conway's Multiverse (`LIFE8.S` + `MVERSE.S`, merged to main `d808335`).

## Why this exists

The DHGR multiverse is impressive but slow (~few sec/gen × 8) with tiny pixels — proves the limits but hard to *watch*. This is the **watchable** companion: the same 8-universe live Conway's Life, rendered in **Lo-Res (GR)** at 40×48. ~1,920 cells/universe vs DHGR's 107,520 → **~56× faster ticking** (near-real-time motion) and each cell is a **fat, obvious color block**. Ships alongside MVERSE — two demos: DHGR (grand) + GR (fun to watch). A direct copy/alter of the proven design; **no gateware change.**

## Goal

`BRUN GRVERSE` → 8 GR Life universes evolving fast, channel-surf `0`-`7`, fat visible blocks, near-real-time. Same architecture as the DHGR version, re-targeted to GR. Merlin-on-//e format (single spaces, ASCII, left-to-right — [[feedback_merlin_source_format]]). Success on the bench: 8 distinct, *visibly* evolving GR universes.

## Architecture (same shape as DHGR — only the deltas below differ)

- **Coproc `LIFE8GR`** = the DHGR `LIFE8` re-parameterized for the GR grid (40×48, 8 cells/byte). Forever round-robins ticking all 8 universes (same TICK1 algorithm). Registered skill, no reflash, budget=0.
- **//e `GRVERSE`** = the DHGR `MVERSE` re-targeted: GR display init, read the selected universe's tiny grid from SDRAM, **expand bit→nibble** into the GR text-page block format, channel-surf.
- Free-running, no per-tick handshake; the //e reads, the coproc writes; sync via per-universe `FRONT[u]`/`GEN[u]` (same as DHGR).

## Delta 1 — the grid (40×48, 8 cells/byte)

- 40 wide × 48 tall. **8 cells/byte** (bit 0 = leftmost), 40 = 5×8 exactly → **5 bytes/row** (`ROWBYTES=5`), 48 rows (`GROWS=48`), `GWIDTH=40`. Buffer = 5×48 = **240 B**; double-buffered = 480 B/universe. ~56× fewer cells than DHGR.
- `cell(row r, col x)` = bit `(x & 7)` of `grid[r*5 + (x>>3)]`. Bit set = live.
- Torus 40×48 (horizontal wrap col 39↔0, vertical row 47↔0).

## Delta 2 — `LIFE8GR.S` (copy `LIFE8.S`, three alters)

Same TICK1 (sliding 3-row window, COLSUM, B3/S23, software write pointer, flip-before-bump, never-DONE, budget=0). Three changes — all because DHGR was 7 cells/byte and GR is 8:
1. **Bit loop 0-7, not 0-6.** `DOCOLSUM` iterates each byte's bits — DHGR did bits 0-6 (7/byte); GR does **bits 0-7** (8/byte). Same for `DOROWOUT`'s pack.
2. **Equates:** `ROWBYTES=5`, `GROWS=48`, `GWIDTH=40` (in a `LIFEMAPGR.S` or via the existing LIFEMAP equates re-set).
3. **Parameterize the torus-wrap column.** DHGR hardcodes `LDA COLSUM+559` (its last column = 560-1). GR's last column is 39 → must be `COLSUM+GWIDTH-1` (=39). Replace the `559` literal with a `GWIDTH-1` equate so the wrap is correct. (Also: the COLSUM build now produces `ROWBYTES*8 = 40` entries exactly — clean, no wasted slots, unlike a 7/byte-on-40 would be.)

COLSUM array only needs 40 bytes now (vs 560); keep it pinned in BRAM scratch (the existing $0D40 region is plenty). Code + 240×3 window + 40 colsum + scratch all fit $0300-$0F7F easily (grid is tiny).

## Delta 3 — GR display + the bit→nibble render (`GRVERSE.S`)

**GR is a color-block mode, NOT bit-packed pixels (unlike DHGR):**
- 40×48 blocks in the **text page `$400-$7FF`** (40 bytes/text-row × 24 text-rows).
- Each **byte = 2 vertically-stacked blocks**: **low nibble (bits 0-3) = TOP block color, high nibble (bits 4-7) = BOTTOM block.** Each nibble = 4-bit color (`$0`=black/dead, `$F`=white/live).
- Text-row Y (0-23) holds block-rows `2Y` (top, low nibble) and `2Y+1` (bottom, high nibble).
- **Text-page line-base interleave** (the GR analog of LINEADDR): `textbase(Y) = $400 + (Y & 7)*$80 + (Y>>3)*$28` (3 groups of 8 lines). Build a 24-entry HI/LO table at startup (or hardcode).

**Enable GR:** `STA $C050` (graphics), `STA $C056` (LORES), `STA $C052` (full screen — or `$C053` mixed for a text status line). (Standard lo-res, page 1.)

**`RENDER(CHAN)` — the bit→nibble expansion (the one real new piece vs DHGR's straight copy):**
- read `FRONT[CHAN]` → live buffer base; for each text-row Y in 0..23:
  - for each col X in 0..39: `top = cell(2Y, X)`, `bot = cell(2Y+1, X)` (extract bits from the bit-packed grid rows 2Y and 2Y+1); `byte = (top ? $0F : 0) | (bot ? $F0 : 0)`; store to `textbase(Y) + X`.
- The grid read: pull the two grid rows (2Y, 2Y+1) — 5 bytes each — from SDRAM (SDRAMLIB), or read the whole 240-B grid into a //e buffer once then expand. **Not a memcpy — it expands each cell bit to a `$0`/`$F` nibble + packs 2 vertical cells per byte.** Tiny (240 B grid → 960 B text page), so fast despite the expansion.

## Delta 4 — seeds (scaled to 40×48, torus-friendly)

8 patterns into universes 0..7 buffer A (FRONT[u]=0), via SDRAMLIB. The DHGR Gosper gun (36×9) barely fits 40 wide and dies even faster on a tiny torus — so for GR, prefer torus-stable / busy seeds: gliders (they wrap + roam), small oscillators (pulsar/pentadecathlon if they fit), an r-pentomino (methuselah), and **dense random soup (LFSR) for several channels** (the visibly-churning stars). Scale patterns to the 40×48 grid. The soup channels are the watchable payoff.

## SDRAM map + metadata

Reuse the DHGR map shape (separate `BRUN` — GR and DHGR are never run simultaneously, so bank reuse is fine): universe u = bank `UBASE+u`, buffer A `$0000`, buffer B (now only needs +240, but keep generous separation, e.g. `$0400`). Metadata bank `MBANK`: `FRONT[u]`@`$0010+u`, `GEN[u]`@`$0020+u`. (Pin the GR buffer-B offset in the plan; grids are 240 B so any clear offset works.) GRVERSE seeds on launch, so prior DHGR state in those banks is overwritten — no conflict.

## Components / files (new branch off `main`)
- `software/SDM/LIFE8GR.S` — copy LIFE8.S, the three alters (8/byte bit loop, GR equates, GWIDTH-1 wrap). Same TICK1.
- `software/SDM/LIFEMAPGR.S` — GR equates (ROWBYTES=5, GROWS=48, GWIDTH=40, banks) — OR reuse LIFEMAP with GR values (decide in plan; separate file is cleaner since DHGR's are different).
- `software/SDM/GRVERSE.S` — copy MVERSE.S, GR init + the bit→nibble render + text-page line table + seeds + surf loop. `PUT SDRAMLIB`.
- Makefile: `life8gr`/`grverse` targets + sdmdisk pack (add GRVERSE; keep MVERSE — both ship).
- `gateware/rev2/project_obscurus/project_obscurus_tb.v` — sim: the TICK1 oracle re-parameterized for GR dims (small grid, blinker/glider/byte-boundary/torus at 8-cells/byte, GWIDTH=40) — proves the 8/byte + GWIDTH-1-wrap alters didn't break the Life math.

## Error handling / edge cases
- **8/byte vs 7/byte (the core alter):** the bit-loop bound (0-7) + the GWIDTH-1 wrap are the risk — verified by the re-parameterized sim oracle (blinker/glider/torus on a GR-dim grid). A glider crossing an 8-cell byte boundary + wrapping the col-39 torus seam is the key assertion.
- **GR render bit→nibble correctness:** top=low-nibble / bottom=high-nibble, `$0`/`$F`, text-page interleave. Bench-verified (no GR sim) — a known test pattern (e.g. a single column of live cells at x=0) must land in the right block column/nibble.
- **Gun-on-torus:** same Life fact as DHGR — guns collapse on a torus. GR seeds avoid the gun; use torus-stable/soup.
- **Buffer-B offset:** GR buffers are 240 B; pin B's offset clear of A (the plan); don't reuse a DHGR 15 KB layout literally.
- **never-DONE + budget=0 + R4 sole-SDRAM-task + flip-before-bump (R5):** all inherited from LIFE8 — unchanged, still apply.
- **Merlin-//e format:** single spaces, ASCII, ≤50 char, left-to-right. Verify `grep -cP '\t'`==0, ASCII==0 on every shipped .S.

## Testing
- **Sim — TICK1 at GR dims (the load-bearing test):** the DHGR TICK1 oracle re-parameterized — seed a small grid (keep `ROWBYTES=5`/8-cells-per-byte; reduce GROWS for sim), blinker + glider (crossing an 8-cell byte boundary) + torus-seam pattern (col 39↔0), run via the kernel, sync on GEN[0], assert vs hand-computed Conway. Pins the 8/byte + GWIDTH-1 alters. iverilog `-g2005`.
- **Build:** Merlin32 assembles LIFE8GR + GRVERSE clean, Merlin-//e-format-valid. No gateware build (no RTL change).
- **Bench (no reflash):** `make sdmdisk`; boot; `BRUN GRVERSE` → 8 GR universes, fat blocks, near-real-time evolution; `0`-`7` flips channels; soup channels visibly churn fast. Screenshot.

## Success criteria
8 GR (40×48) Life universes evolve near-real-time in SDRAM, the //e channel-surfs them with fat visible blocks, on the bench, no gateware change — the *watchable* multiverse, shipping beside the DHGR one.

## Non-goals
- No new gateware / no reflash (reuses LIFE8's algorithm + the SDRAM windows).
- No color Life (mono: `$0` dead / `$F` live; the 16-color GR palette is unused beyond black/white).
- No replacing MVERSE — both ship.
- No grid larger than 40×48 (GR's native block resolution).
- No hardware Life engine (the separate future "Conway's Life Engine").
