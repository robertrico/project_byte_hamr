# Farm v2 — Goods Inventory + Pantry Elimination (Design Spec)

Date: 2026-06-16
Status: approved in brainstorm
Amends: `docs/superpowers/specs/2026-06-12-farm-v2-recipe-shop-design.md`
(crafting/collect model). Recipe-shop increment as shipped is the base.
Pre-events work (do before increment 4 world events).

## Problem / intent

Two bench-driven changes:

1. **C1/C2 collect-and-auto-sell is poor UX.** Finished crafts should
   accumulate as goods, not force an immediate manual collect+sale. Keep the
   background cooking (the part that works); store the product instead.
2. **The deposit step (D-key, pantry) is plumbing leaking into the UI** — a
   software easement, not a designed mechanic. It exists only because the
   single-writer rule forces crafting ingredients into the workshop's own
   SDRAM bank, so crops must be shuttled bank 32 → bank 33. The //e can read
   both banks, so crafting can pull directly from harvested crops and the
   pantry can be deleted entirely.

Result: a new **inventory screen** showing the player's full holdings in
three sections (seeds, crops, goods), with goods and crops sellable from it.

## Decisions (locked in brainstorm)

1. Finished crafts auto-store as **goods**; no manual collect.
2. **Eliminate the pantry.** Craft consumes directly from farm crop
   inventory, //e-orchestrated. No deposit, no D-key.
3. New **INVENTORY screen** (4th screen, key `I`), three sections:
   **SEEDS** (read-only), **CROPS** (sellable @ market price), **GOODS**
   (sellable @ fixed value).
4. Sell from inventory: cursor-select a sellable item, `S` → qty prompt
   (1-255) → sell. BOOM doubles goods value.

## Architecture conventions honored

- Single-writer per GBANK: FARMTASK owns bank 32 (crops, cash, seeds),
  WORKTASK owns bank 33 (goods, skill, recipes, stations). The //e is the
  only reader of both and mediates all cross-bank value transfer.
- //e-as-bus, debit-first: every transfer removes from the source before
  adding to the destination, so a crash loses value, never duplicates it.
- Host writes coproc BRAM only while no task runs (quiesce); unaffected here
  (no blob reload changes).
- Verify hardware blobs are byte-current vs source after every build (the
  recurring stale-artifact hazard — see
  `feedback_farmtaskb_stale_embed.md`): `grep -c DFB *B.S` == `wc -c *.bin`,
  and the .bin size must reflect source changes.

## Data model

### Bank 33 (WORKTASK) — changes

| Addr | Content | Change |
|---|---|---|
| $0210-$0213 | (was PANTRY[4]) | **freed, unused** |
| $0214 | SKILL | unchanged |
| $0215-$0216 | DISC bitmap | unchanged |
| $0217 | MODE (bit0=BOOM) | unchanged |
| $0220-$0227 | STATIONS 2×4 | unchanged |
| $0228-$0233 | **GOODS[12]** (new) | per-product count, byte, clamp 255 |
| $0300+ | RECIPES 12×8 | unchanged |

GOODS placed after stations to avoid relocating existing fields. The old
pantry bytes ($0210-13) are left unused (zeroed at seed) rather than
reclaimed, to minimize churn.

### Bank 32 (FARMTASK) — unchanged

CROPS[4] $0226, SEEDS[4] $0222, CASH $0220 — all as shipped. No new
farm-blob bytes (the ~12 B FARMTASK headroom stays untouched; events still
need the later relocation).

## Components

### WORKTASK (coproc) — net simpler

**Removed:** pantry array references, ingredient checks in OPCRAFT, WCONSUM,
the OPDEPOSIT op (WOPDEP) and its handler.

**OPCRAFT(I0-I3)** — ingredients already debited by the //e; this op no
longer checks or consumes pantry. It: sort-matches the combo against the
recipe table → if no match, RUINED (RRUIN); if match + owned, known-fail
roll (cook or RFAIL); if match + unowned, discovery roll (cook+learn, or
RUINED). SKILL++ on every attempt (clamp $FF). The WKNOWN-not-WCGO mandate
and LFSR-polarity rules from the recipe-shop spec still apply — leave that
structure intact, only the consume step is removed.

**WTICK done-handler** — on a cooking station's timer reaching 0:
`GOODS[recipe]++` (clamp 255), set station STATE=0 (IDLE, immediately
reusable), `PUTEV WEVDONE(station, recipe)`. No DONE-waiting state.

**WOPSELL(I0=product, I1=qty)** (reuses the WOPCOLL op slot): validate
product < 12 and GOODS[product] ≥ qty (else RERRBAD/RERRCROP); decrement
GOODS[product] by qty; compute value = qty × RECIPE[product].VALUE, doubled
if MODE bit0 (BOOM) set, clamped to $FFFF (16-bit result in RES/RES1);
return ROK with the value. The //e then credits farm cash via OPADDC.
(Value is read from the recipe table's VALUE byte; goods stored at face,
priced at sell time — BOOM applies here, not at craft.)

**RECIPE.VALUE is pre-existing — migration-safe.** The recipe table entry
is `I0,I1,I2,I3,TIME,VALUE,RARITY,pad` (VALUE at offset +5), shipped and
correct in the recipe-shop increment: it already drove `RPRICE = 2×value`
and the old collect-sale (which read +5). WOPSELL reads the same offset.
The table layout/contents do NOT change this increment, so the selective
migration's "preserve recipe table" is correct — migrated players price
goods from the same correct VALUE bytes as cold-start players. No table
rewrite needed. **Known silent cap:**
the 16-bit value clamps at $FFFF; a very large qty × value × BOOM can hit the
cap and lose the overflow with no warning. Acceptable (requires hundreds of a
high-value good); not worth a guard now.

Blob: WORKTASK currently 1402/4096. Removing pantry/WCONSUM offsets adding
GOODS handling + WOPSELL; net expected ≈ flat, well under cap.

### FARMTASK (coproc) — unchanged

OPWITHDRAW(crop, qty) and OPADDC(lo, hi) already exist and are all the //e
needs (debit crops for crafting, credit cash for sales). No farm-blob
changes.

### //e (FARM.S)

**Craft flow (workshop screen)** — replaces the pantry/deposit path:
- Player builds MIX (up to 4 crop ids) as today.
- On craft (RETURN): the //e counts per-crop needs from the MIX, checks them
  against the CROP4 mirror (farm crops, RDMKT-fresh). Insufficient → message
  `NEED CROPS`, no debit, no craft.
- If sufficient: OPWITHDRAW each needed (crop, qty) from the farm
  (debit-first — all needs pre-validated so no partial debit), then
  OPCRAFT(sorted MIX) to the workshop. Result → `COOKING` / `RUINED!` /
  `CRAFT FAILED` as today.
- **Failed crafts do NOT refund.** RUINED (dud or discovery miss) and CRAFT
  FAILED (known-recipe roll fail) keep the crops already withdrawn — failed
  crafting wastes the ingredients, intentionally. This differs from the old
  pantry model only in that the crops come straight from the farm now; the
  consume-on-failure behavior is the same.
- Removed: WKDEPOS (D-key), WKDEPOK, OPDEPOSIT staging, the workshop pantry
  strip, the WPANT4 mirror, the C1/C2 collect handler (WKCOLLECT), and the
  station DONE-PRESS-C rendering.

**Workshop screen** after cleanup: title + skill, 2 stations (IDLE/COOKING
only — auto-clearing), recipe book, MIX row, legend (`1-4 MIX RTN CRAFT
I INV ESC`). No pantry, no deposit, no collect.

**New INVENTORY screen (SCREEN=3, key `I`** from farm/market/workshop;
ESC → returns to the screen it was entered from — see PREVSCR below).

**Scalable by design (N items).** Items, crops, and goods WILL grow (more
recipes, more crop types with future multi-plot). The screen must not assume
a fixed count that fits on one page. It is **paged + scrolling**:

- **Three category pages: SEEDS / CROPS / GOODS.** One page visible at a
  time. A key cycles pages (`TAB`/`P` → next page; wraps). The page name +
  index shown in the header (e.g. `INVENTORY  GOODS  (3/3)`).
- Each page is a **scrolling viewport** of `VROWS` rows (target ~16). The
  page holds `N` entries (SEEDS 4, CROPS 4, GOODS 12 today; any N later). A
  cursor moves with up/down; when it reaches a viewport edge the list scrolls
  (window offset advances). `>` marks the cursor row; a `^`/`v` hint shows
  when more rows exist above/below. This works identically for 4 or 400
  entries — no layout rewrite when counts grow.
- Row format per page: `<name> <count>` and, for sellable pages, the unit
  price (CROPS: market price; GOODS: value). SEEDS rows show count only.
- **SEEDS page is read-only** (no cursor-select, no sell) — included so
  "what do I have" is complete, but it costs only one page, not competing
  rows. CROPS and GOODS pages are sellable. `S` on the SEEDS page is inert
  (no-op). (Today's 4 seed types fit within VROWS without scrolling; if seed
  types ever exceed VROWS with multi-plot, add a read-only scroll path —
  out of scope now.)
- **Page-switch resets to top:** cycling pages (TAB/P) sets the cursor to
  row 0 and the window offset to 0 for the new page (no per-page cursor
  memory — simplest, predictable).
- Header line + a cash line (HUDCASH) + a legend line frame the viewport, so
  the budget is: 1 header + VROWS viewport + 1 cash + 1 legend ≈ 19-20 of 24
  rows, independent of N.

**Sell:** on a CROPS or GOODS page, `S` → qty prompt (1-255, existing
QTYPROMPT) on the cursor's entry → route by page:
- CROPS → OPSELL(crop, qty); FARMTASK debits the crop and credits cash
  atomically in bank 32 (as today). Sells at current market price.
- GOODS → WOPSELL(product, qty) returns the value; //e then OPADDC(value) to
  farm cash. Debit-first (goods down in bank 33, then cash up in bank 32).
- Message `SOLD <name> +nnn`. Refresh holdings + cash after.

**Mirrors:** WSYNC (bank 33) extended to read GOODS[12] into WGOODS (12
bytes of //e RAM). SEED4/CROP4/PRICE4 mirrors already exist (RDMKT). The
inventory screen calls RDMKT + WSYNC on entry and after each sale.

**PREVSCR:** a 1-byte var records the screen `I` was entered from (0 farm /
1 market / 2 workshop). ESC restores it. (Return-to-origin, not always-farm —
entering inventory from the workshop and landing on the farm is jarring.)

**Cross-bank sale ordering:**
- Good sale: WOPSELL debits goods (bank 33) and returns value; //e then
  OPADDC credits cash (bank 32). Debit-first.
- Crop sale: OPSELL (FARMTASK) debits crops and credits cash atomically
  within bank 32 — no cross-bank step, as today.

**CVER → 3 with a SELECTIVE migration (must not wipe progress).** The
layout bump must NOT erase discovered recipes (DISC) or craft SKILL — the
recipe-shop grind is the whole point of the workshop, and a full reseed on
every deploy would nuke it. Two distinct bank-33 init paths:

- **Cold start** (invalid SIG, e.g. power-cycle): full seed as today —
  SIG, SEQ/HEAD, SKILL=0, DISC=0, MODE=0, stations=0, GOODS=0, pantry
  bytes=0, recipe table written, CVER=3. Everything fresh.
- **Version migration** (valid SIG, CVER ≠ 3): zero ONLY the new/changed
  region — **GOODS[12] and the retired pantry bytes ($0210-13)** — set
  CVER=3, and **preserve DISC, SKILL, MODE, STATIONS, and the recipe
  table.** Players keep every learned recipe and their skill level across
  this deploy.

Deploy = ctrl-reset + BRUN; farm world (bank 32) survives; workshop keeps
discovered recipes + skill, gains an initialized (empty) GOODS inventory.

## Testbench

- WTICK auto-store: cook a recipe, wait done, assert GOODS[recipe]==1 and
  station STATE==0 (idle), and a WEVDONE(station, recipe) on the ring.
- WOPSELL: seed GOODS[product]=N, sell qty M≤N → GOODS=N−M, RES/RES1 ==
  M×value; with MODE BOOM → 2× (clamp); oversell qty>GOODS → RERRCROP, goods
  unchanged.
- OPCRAFT no-consume: craft no longer touches any pantry (pantry gone);
  assert a craft with the //e-debit model cooks/RUINs without reading
  $0210-13. (The crop debit is a FARMTASK OPWITHDRAW, tested separately.)
- Discovery + known-fail rolls unchanged (recipe-shop asserts still pass).
- Crop debit-for-craft: OPWITHDRAW(crop, qty) reduces FCROPS (already
  covered by recipe-shop tb; reuse).
- **End-to-end good sale (integration):** seed GOODS[p]=N, WOPSELL(p, M),
  then OPADDC the returned value; assert farm CASH increased by exactly the
  value AND GOODS[p]==N−M. Covers the cross-bank WOPSELL→OPADDC path incl.
  the BOOM-doubled value.
- **Migration preserves progress:** set bank-33 DISC and SKILL nonzero +
  CVER=2 + GOODS garbage, run the migration path, assert CVER==3, GOODS==0,
  pantry bytes==0, and **DISC + SKILL + recipe table UNCHANGED**. Plus a
  cold-start case asserting a full fresh seed (DISC=0, SKILL=0).
- **Stale-op safety:** assert the WOPCOLL op slot now performs WOPSELL
  semantics (no old collect-and-auto-sell behavior survives); no caller
  issues the old op. Keep the blob byte-currency check (`grep -c DFB
  WORKTASKB.S` == `wc -c WORKTASK.bin`, and WORKTASK.bin size reflects the
  source change — per the stale-embed lesson; the "blob ≈ flat" estimate is
  unverified until measured).
- wk_init seeds GOODS=0, CVER=3, pantry bytes 0.

## Out of scope

- Per-vendor pricing / SELL screen (v2.6).
- Goods as inputs to higher-tier recipes (crafting chains) — possible later;
  not now.
- World events (increment 4) — this is the pre-events cleanup.

## Resolved (were open items)

- **Row budget / N-scalability:** solved by paged + scrolling viewport (one
  category page at a time, cursor scrolls a VROWS window). Independent of
  item count — no rework when recipes/crops grow with multi-plot.
- **Cursor navigation:** up/down moves the cursor and scrolls the viewport at
  its edges; TAB/P cycles category pages. Single-column linear list per page.

## Plan-time detail (not blocking)

- Pick `VROWS` (≈16) and the exact header/cash/legend row assignments
  against the 24-row screen; confirm the `^`/`v` more-rows hints fit.
- WORKTASK blob size MUST be measured post-build (build + `wc -c` vs
  `grep -c DFB`), not assumed flat.
