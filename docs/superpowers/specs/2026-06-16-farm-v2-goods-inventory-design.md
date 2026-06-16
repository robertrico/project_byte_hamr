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
- Removed: WKDEPOS (D-key), WKDEPOK, OPDEPOSIT staging, the workshop pantry
  strip, the WPANT4 mirror, the C1/C2 collect handler (WKCOLLECT), and the
  station DONE-PRESS-C rendering.

**Workshop screen** after cleanup: title + skill, 2 stations (IDLE/COOKING
only — auto-clearing), recipe book, MIX row, legend (`1-4 MIX RTN CRAFT
I INV ESC`). No pantry, no deposit, no collect.

**New INVENTORY screen (SCREEN=3, key `I`** from farm/market/workshop;
ESC → farm):
- Section SEEDS: 4 rows `WHEAT nnn` … from the SEED4 mirror (bank 32). Shown
  for reference, **not selectable / not sellable**.
- Section CROPS: 4 rows from the CROP4 mirror. Sellable at **market price**
  (PRICE4 mirror) via OPSELL(crop, qty).
- Section GOODS: 12 rows `BREAD nnn` … from a new WGOODS mirror (bank 33).
  Sellable at **fixed value** (recipe VALUE) via WOPSELL(product, qty).
- Selection: a cursor (up/down arrows) over the **sellable** items only
  (4 crops + 12 goods = 16 entries); the seeds section renders but the
  cursor skips it. Selected row marked `*`.
- Sell: `S` → qty prompt (1-255, the existing QTYPROMPT) → route by selected
  type: crop → OPSELL(crop, qty) then read cash (crop sale credits farm cash
  inside FARMTASK as today); good → WOPSELL(product, qty) then OPADDC(value)
  to farm. Message `SOLD <name> +nnn`. Refresh holdings + cash after.
- A cash line is shown (HUDCASH) so sales reflect immediately.

**Mirrors:** WSYNC (bank 33) extended to read GOODS[12] into WGOODS (12
bytes of //e RAM). SEED4/CROP4/PRICE4 mirrors already exist (RDMKT). The
inventory screen calls RDMKT + WSYNC on entry and after each sale.

**Cross-bank sale ordering:**
- Good sale: WOPSELL debits goods (bank 33) and returns value; //e then
  OPADDC credits cash (bank 32). Debit-first.
- Crop sale: OPSELL (FARMTASK) debits crops and credits cash atomically
  within bank 32 — no cross-bank step, as today.

**CVER → 3** (bank-33 layout changed: GOODS added, pantry retired). Re-seed
on mismatch zeros GOODS (and the retired pantry bytes). Deploy = ctrl-reset
+ BRUN; farm world (bank 32) survives, workshop (bank 33) re-seeds.

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
- wk_init seeds GOODS=0, CVER=3, pantry bytes 0.

## Out of scope

- Per-vendor pricing / SELL screen (v2.6).
- Goods as inputs to higher-tier recipes (crafting chains) — possible later;
  not now.
- World events (increment 4) — this is the pre-events cleanup.

## Open items for the plan

- Exact INVENTORY screen row layout (24 text rows: seeds 4 + crops 4 +
  goods 12 = 20 rows + headers; may need a compact 2-column goods layout or
  scrolling — resolve in the plan against the row budget).
- Cursor navigation keys (up/down vs paging) given 16 selectable entries.
