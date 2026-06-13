# Farm v2 — Recipe Shop Amendment (Crafting Economy v2)

Date: 2026-06-12
Status: approved (bench-driven redesign of increment-3 crafting)
Amends: `docs/superpowers/specs/2026-06-11-farm-v2-sidework-design.md`
(Crafting / recipe model section). Increment-3 implementation as shipped
(WORKTASK @ $2000, bank 33, 12 recipes, discovery roll) is the base.

## Problem (bench finding)

Blind discovery is the only path to recipes, and every failed attempt —
whether a dud combo or a real combo with a failed roll — consumes
ingredients with an identical "RUINED!" message. The player cannot tell
"wrong recipe" from "right recipe, unlucky", and repeated attempts drive
cash/crops to zero with nothing learned. Morrowind's equivalent works only
because ingredients leak information; ours leaks none.

## Design: two roads to every recipe

### Road 1 — the shop (deterministic)

- The market screen gains a recipe row. **R buys the LOWEST unowned recipe**
  in the fixed 12-recipe list (BREAD → FEAST order, by rarity/value).
- Purchase gates: `skill >= SKILLREQ[rarity]` AND `cash >= price`.
  - SKILLREQ by rarity tier: r0=0, r1=25, r2=75, r3=150.
  - Price per recipe = 2 x product value (BREAD 56 ... FEAST 254; 16-bit
    math //e-side, prices held in a FARM.S table RPRICE 12 x 2 bytes —
    values may exceed 255 if retuned later).
- Market row display: `RECIPE BREAD 56  R BUY` when buyable;
  `RECIPE BREAD 56  NEED SKILL 25` when skill-gated;
  `RECIPES: ALL OWNED` when complete.
- Flow (//e-as-bus, debit first): farm `OPSPEND(value lo/hi)` → on ROK,
  workshop `WOPLEARN(idx)`. A crash between legs loses the cash, never
  dupes a recipe (consistent with the spec's transfer rule).

### Road 2 — discovery (the gamble, unchanged stakes)

- Mixing an UNOWNED combo is a real attempt, exactly as increment 3 shipped:
  - Dud combo → ingredients consumed, `RUINED!`.
  - Real-but-unowned combo → discovery roll
    (`LFSR < 40 + skill/2 − rarity*16`, floor 0): success = **recipe
    learned free AND starts cooking**; failure = ingredients consumed,
    `RUINED!`.
- One message for both failure cases — the mystery stays; the shop is the
  information channel. Punishment-by-consumption is deliberate.

### Known recipes can still fail

- Crafting an OWNED recipe (bought or discovered) rolls a failure chance:
  `failChance = FAILBASE[rarity] − skill/2`, floored at FAILFLOOR.
  - FAILBASE (of 256): r0=48 (~19%), r1=80 (~31%), r2=112 (~44%), r3=144 (~56%).
  - FAILFLOOR = 8 (~3%) — never free, always a craft.
- Failure consumes ingredients, result `RFAIL = $E9`, //e message
  `CRAFT FAILED` (distinct from RUINED — you KNOW this recipe is real).
- Success: cooks as today (station, timer, WEVDONE, collect).

### Skill

- +1 per craft attempt of ANY kind (dud, discovery fail, known fail,
  success), clamp $FF. Unchanged from increment 3.

## Mechanics

### Bank 33

Unchanged layout. DISC bitmap ($0215-$0216) remains the single ownership
truth (shop sets lowest unowned bit; discovery sets any bit). CVER bumps
to **2** — deploy forces a bank-33 re-seed (pantry/skill/recipes reset;
farm world untouched).

### WORKTASK changes

- WCRAFT, matched-combo path: if DISC bit set → known-recipe failure roll
  (LFSR < failChance → consume + skill++ + RFAIL; else consume + skill++ +
  cook). If bit clear → existing discovery path (success sets bit + cooks;
  failure consumes + skill++ + RRUIN).
- FAILBASE[4] + FAILFLOOR as blob tables (hot-ish, tiny).
- New op `WOPLEARN = $05` (I0 = recipe idx): validates idx < NRECIP and
  DISC bit clear → set bit, ROK; else RERRBAD. NO skill/cash validation in
  the task (the //e gates skill and pays first; task only guards
  double-learn). Blob cap 4096 — no pressure.

### FARMTASK changes

- New op `OPSPEND = $07` (TA0 = lo, TA1 = hi): cash >= amount → subtract,
  ROK; else RERRCASH. 16-bit compare-then-subtract (CBUY precedent).
- Blob at 1772/1792: apply the 37 B hi-byte reclaim (reviewer-verified:
  per-crop addr hi bytes are constant $02; replace the
  `LDA #0 / ADC #>tbl / STA tmp` + `LDY tmp` patterns with `LDY #>tbl`
  immediates in CPLANT/CBUY/CHARV/CSELL) BEFORE adding OPSPEND (~50 B).

### //e (FARM.S)

- RPRICE table (12 x 16-bit) + SKILLREQ table (4 bytes) — display + gating
  //e-side.
- Market screen: recipe row (placement at the implementer's discretion —
  rows 16-18 are free) + R key handler: find lowest unowned bit (needs
  WDISCV mirror, already synced by WSYNC), check skill (WSKILLV) + cash,
  OPSPEND → WOPLEARN → refresh + `LEARNED <name>!` message; error paths
  `NEED SKILL nnn` / `NEED CASH nnn` / coproc errors as usual.
- Workshop craft result dispatch adds `$E9 → CRAFT FAILED`; `$E8` message
  stays `RUINED!`.
- CVERNUM = 2.
- Recipe book rendering unchanged (bitmap-driven; bought and discovered
  recipes look identical — both "known").

### Result codes (cumulative)

| Code | Meaning |
|---|---|
| $01 ROK | ok / cooking |
| $E5 RERRCROP | not enough crops/ingredients |
| $E6 RERRBAD | bad args / double-learn / unknown op |
| $E7 RERRFULL | stations full / inventory full |
| $E8 RRUIN | unowned attempt failed (dud OR discovery miss) |
| $E9 RFAIL | KNOWN recipe craft failed |
| $E4 RERRCASH | OPSPEND insufficient funds |

### Testbench

- WOPLEARN: learn lowest (BREAD idx 0) → bit set; double-learn → RERRBAD.
- OPSPEND: exact-funds success + insufficient → RERRCASH + cash unchanged.
- Known-recipe fail curve: skill 0 + owned BREAD → loop attempts, expect a
  mix of ROK and $E9 (assert only those two codes appear, ingredients
  consumed either way); skill $FF → failChance floor → near-always ROK
  (assert first attempt ROK at FAILFLOOR=8: cycle-deterministic, verify
  empirically and pin whichever outcome the LFSR gives, commented).
- Discovery path asserts from increment 3 remain valid (unowned combos).
- wk_init seeds DISC=0 and CVER=2.

## Tuning knobs (all data)

| Knob | Value | Where |
|---|---|---|
| SKILLREQ | 0/25/75/150 | FARM.S table (//e gate) |
| Price | 2 x value | FARM.S RPRICE |
| FAILBASE | 48/80/112/144 | WORKTASK table |
| FAILFLOOR | 8 | WORKTASK equate |
| Discovery curve | 40 + skill/2 − 16 x rarity | unchanged |

## Out of scope

- Recipe-book UI for picking a craft (MIX entry stays — tactile).
- Workshop status widget on the farm screen (separate polish item, parked).
- D-key deposit crop picker (separate approved polish item, parked — was
  interrupted by this redesign; still wanted).
