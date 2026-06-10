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

## Delta 2 — `LIFE8GR.S` (copy `LIFE8.S`, FOUR alters)

Same TICK1 (sliding 3-row window, COLSUM, B3/S23, software write pointer, flip-before-bump, never-DONE, budget=0). Four changes:
1. **Bit loop 0-7, not 0-6.** `DOCOLSUM` iterates each byte's bits — DHGR did bits 0-6 (7/byte, `LDX #7`); GR does **bits 0-7** (8/byte, `LDX #8`). Same for `DOROWOUT`'s pack. (Verified safe: the last-byte guards `CPX #1`/`CPX #2` are X-countdown-relative, survive unchanged.)
2. **Equates:** `ROWBYTES=5`, `GROWS=48`, `GWIDTH=40` (in `LIFEMAPGR.S`).
3. **Parameterize the torus-wrap column.** DHGR hardcodes `LDA COLSUM+559` (LIFE8.S:339, its last column = 560-1). GR's last column is 39 → must be `COLSUM+GWIDTH-1` (=39). Replace the `559` literal with a `GWIDTH-1` equate. (COLSUM now builds `ROWBYTES*8 = 40` entries exactly — clean, no wasted slots.)
4. **Row stride: `MUL80` → `MUL5` (G1 — must-fix, easy to miss).** `LIFE8.S:271-296`'s `MUL80` is a HARDCODED shift-add ×80 routine (not equate-driven) used by `READROW`/`WRITEROW` to compute the byte offset of row r (`r*ROWBYTES`). DHGR ROWBYTES=80; GR ROWBYTES=5 → it must become **`MUL5` (`PROD = A*4 + A`)**. Changing only the `ROWBYTES` equate does NOT fix the stride — `ROWBYTES` only bounds loops; the multiply is separate hand-rolled code. Wrong stride = garbage grid. (Same trap on the //e side — see Delta 3.)

COLSUM array only needs 40 bytes now (vs 560); keep it pinned in BRAM scratch (the existing $0D40 region is plenty). Code + 240×3 window + 40 colsum + scratch all fit $0300-$0F7F easily (grid is tiny).

## Delta 3 — GR display + the bit→nibble render (`GRVERSE.S`)

**GR is a color-block mode, NOT bit-packed pixels (unlike DHGR):**
- 40×48 blocks in the **text page `$400-$7FF`** (40 bytes/text-row × 24 text-rows).
- Each **byte = 2 vertically-stacked blocks**: **low nibble (bits 0-3) = TOP block color, high nibble (bits 4-7) = BOTTOM block.** Each nibble = 4-bit color (`$0`=black/dead, `$F`=white/live).
- Text-row Y (0-23) holds block-rows `2Y` (top, low nibble) and `2Y+1` (bottom, high nibble).
- **Text-page line-base interleave** (the GR analog of LINEADDR): `textbase(Y) = $400 + (Y & 7)*$80 + (Y>>3)*$28` (3 groups of 8 lines). Build a 24-entry HI/LO table at startup (or hardcode).

**Enable GR (G4 — must also UNDO DHGR mode, since MVERSE may have run this session):** if MVERSE ran first, 80VID/80STORE/HIRES/DHIRES are all on → a bare lo-res enable shows garbage. So turn the DHGR-mode switches OFF first: `STA $C00C` (80VID off), `STA $C05F` (DHIRES off), **`STA $C000`** (80STORE off — note: a WRITE to `$C000`), `STA $C054` (PAGE2 off). THEN enable GR: `STA $C050` (graphics), `STA $C056` (LORES), `STA $C052` (full screen — or `$C053` mixed for a text status line). Page 1.

**Clear GR (G5 — via the line table, NOT a blanket fill):** do NOT memset `$400-$7FF` — the text-page "screen holes" (`$478-$47F`, `$4F8-$4FF`, etc.) are slot-firmware scratch (80-col / SmartPort) and stomping them corrupts the card. Clear by the line table: 24 rows × 40 bytes = write `$00` to `textbase(Y)+0..39` for Y=0..23. (`RENDER` itself is already safe — it writes only `+0..39`.)

**`RENDER(CHAN)` — GEN-snapshot-retry, then bit→nibble expansion:**
1. **Snapshot with retry (G2 — REQUIRED at GR speed):** read `GEN[CHAN]`; read `FRONT[CHAN]` → live buffer base; **snapshot the whole 240-B grid into a //e-local buffer** (one `SDM_SETADDR` + 240 `SDM_RDNEXT` — rows are contiguous since stride = ROWBYTES = 5); read `GEN[CHAN]` again. If it changed during the snapshot → a tick+flip happened → **retry**. Snapshot ≈ 5-10 ms < the ~25-50 ms a given universe takes to re-tick (round-robin) → converges in ≤2 tries. Now you hold a coherent single-generation grid in //e RAM.
2. **Expand from the //e buffer** to GR memory: for each text-row Y in 0..23, for each col X in 0..39: `top = cell(2Y, X)`, `bot = cell(2Y+1, X)` (bits from local-buffer rows 2Y / 2Y+1 — **row r is at `buf + r*5`, so use MUL5, NOT MUL80** — G1 //e side); `byte = (top ? $0F : 0) | (bot ? $F0 : 0)`; store to `textbase(Y) + X`.
**Not a memcpy — it expands each cell bit to a `$0`/`$F` nibble + packs 2 vertical cells/byte.**

**WHY snapshot-retry (the R3 protection INVERTS at GR speed — do NOT inherit the DHGR rationale):** DHGR was safe because the coproc's round-robin return (~1.5-3 s) ≫ the //e blit (~300 ms), so a watched buffer wasn't touched mid-blit. GR flips that: a tick is ~3-6 ms/universe, a full round-robin ~25-50 ms, while a //e render (read + per-cell expand) is ~50-100 ms → the coproc flips `FRONT[CHAN]` **1-4× during every render**, so a direct read-and-expand off SDRAM would show **mixed-generation tears every frame**. The snapshot-retry (fast 240-B contiguous read, GEN-check around it) gives a coherent frame. (R4 sole-SDRAM-task + R5 flip-before-bump still inherit unchanged; only R3's *no-lock* rationale is replaced here.)

## Delta 4 — seeds (scaled to 40×48, torus-friendly)

8 patterns into universes 0..7 buffer A (FRONT[u]=0), via SDRAMLIB. The DHGR Gosper gun (36×9) barely fits 40 wide and dies even faster on a tiny torus — so for GR, prefer torus-stable / busy seeds: gliders (they wrap + roam), small oscillators (blinkers/pulsar if they fit), an r-pentomino (methuselah), and **dense random soup (LFSR) for several channels** (the visibly-churning stars).
**G6 — every seed table + fill count is DHGR-encoded; recompute ALL for GR (plan pin):** MVERSE's `GUNLO/GLILO/BLILO/PENLO/...` tables encode byte-offsets for 80-byte rows + 7-bit packing — every offset must be recomputed for **5-byte rows + 8-bit packing**. The `ZEROF`/`SOUPF` fills hardcode count **`$3C00` (15,360) → must become `240`**. "Scale patterns" is not a free equate change — the offset math + the fill counts are all baked. Pin both in the plan.

## SDRAM map + metadata

Reuse the DHGR map shape (separate `BRUN` — GR and DHGR are never run simultaneously, so bank reuse is fine): universe u = bank `UBASE+u`, buffer A `$0000`, buffer B (now only needs +240, but keep generous separation, e.g. `$0400`). Metadata bank `MBANK`: `FRONT[u]`@`$0010+u`, `GEN[u]`@`$0020+u`. (Pin the GR buffer-B offset in the plan; grids are 240 B so any clear offset works.) GRVERSE seeds on launch, so prior DHGR state in those banks is overwritten — no conflict.

## Components / files (new branch off `main`)
- `software/SDM/LIFE8GR.S` — copy LIFE8.S, the FOUR alters (8/byte bit loop, GR equates, GWIDTH-1 wrap, MUL80→MUL5 stride). Same TICK1.
- `software/SDM/LIFEMAPGR.S` — GR equates (ROWBYTES=5, GROWS=48, GWIDTH=40, banks) — OR reuse LIFEMAP with GR values (decide in plan; separate file is cleaner since DHGR's are different).
- `software/SDM/GRVERSE.S` — copy MVERSE.S, GR init (G4 undo-DHGR + enable) + GR clear (G5 line-table) + snapshot-retry + bit→nibble render + text-page line table + seeds + surf loop. `PUT SDRAMLIB`. **Embeds the LIFE8GR bytes via an auto-generated include (G3, below), NOT a hand-pasted DFB blob.**
- Makefile: `life8gr`/`grverse` targets + sdmdisk pack (add GRVERSE; keep MVERSE — both ship).
- **G3 — blob regen rule (avoid the stale-artifact trap):** MVERSE hand-pastes LIFE8's 689-byte LSIM=0 bytes as a DFB blob — exactly the `hamr_rom.mem`-stale failure class. For GR, add a Makefile rule that builds `LIFE8GR` at LSIM=0 and emits a `software/SDM/LIFE8GR.DFB.S` include (bin → `DFB` lines, e.g. via a tiny `xxd`/awk step), which `GRVERSE.S` `PUT`s. So the embedded coproc image is always regenerated from source, never hand-transcribed. (Apply the same rule retroactively to MVERSE if cheap — but at minimum do it for GR.)
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
