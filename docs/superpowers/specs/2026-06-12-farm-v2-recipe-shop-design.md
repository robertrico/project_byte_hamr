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

## Design: two roads, different shapes

The shop is a **ladder** (guaranteed ownership, in price order). Discovery is
the only way to get a recipe **out of order** (random, free on success). They
are not two random-access paths to the same thing — that distinction is the
design.

**"Deterministic" means deterministic to OWNERSHIP, not to a successful
craft.** Buying a recipe guarantees you OWN it; the first (and every) craft
of it still rolls the known-fail curve (`FAILBASE[rarity] − skill/2`). So a
freshly-bought r3 FEAST recipe (price 254) still has ~27% chance its first
craft burns the ingredients at skill 150. Intended: the shop removes the
discovery gamble, not the crafting gamble. The market row should not imply
guaranteed product.

### Road 1 — the shop (deterministic ladder)

- The market screen gains a recipe row. **R buys the LOWEST unowned recipe**
  in the fixed 12-recipe list (BREAD → FEAST order, by rarity/value). You
  cannot target a specific recipe here — that is what discovery is for.
- Purchase gate: `cash >= price` ONLY. No skill gate on purchase — the price
  ladder paces progression, and crafting the cheap rungs builds the skill
  that tames the expensive rungs' failure curves (self-pacing). A skill gate
  here would double-gate and invert the safe/gamble ordering (the gamble is
  always available at any skill; the ladder should be too, just costly).
- **The displayed number is the PRICE.** Price = 2 x product value
  (BREAD value 28 → price 56 ... FEAST value 127 → price 254). Held in a
  FARM.S table RPRICE 12 x 2 bytes (16-bit; current max 254 fits a byte but
  the table is 16-bit for headroom). Cross-check: RPRICE[i] must equal
  2 x RECTAB[i].VALUE — same VALUEs as the base spec's recipe table.
- Market row display (the number is the PRICE — label it `$` or `BUY 56` so
  the player doesn't read it as the product value): `RECIPE BREAD BUY 56`
  when affordable; `RECIPE BREAD NEED CASH` when too poor;
  `RECIPES: ALL OWNED` when complete.
- Flow (//e-as-bus, debit first, **refund on reject**): farm
  `OPSPEND(price lo/hi)` → on ROK, workshop `WOPLEARN(idx)` → if WOPLEARN
  returns RERRBAD (recipe already owned — possible if discovery set the bit
  between the last WSYNC and the R press, leaving the //e's WDISCV mirror
  stale), the //e REFUNDS via farm `OPADDC(price lo/hi)`. A crash strictly
  between the two legs loses the cash (never dupes) — accepted, as for all
  //e-bus transfers. Before R, the //e re-reads WDISCV (WSYNC-lite) to
  minimize the stale-mirror window; the refund covers the residual race.

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
- **LFSR polarity warning (impl footgun):** both rolls compare the SAME LFSR
  byte but with OPPOSITE meaning — discovery is `LFSR < threshold = SUCCESS`,
  known-craft is `LFSR < failChance = FAILURE`. They sit a few lines apart in
  WCRAFT. Each comparison MUST carry an explicit `; <REL> = WIN/LOSE` comment;
  do not factor them into a shared helper (the polarity difference makes that
  a trap).
- **WCGO shared-label mandate (code-verified trap):** in shipped WORKTASK.S,
  `WCGO` (~line 485) is the COOK entry reached by BOTH the bit-set branch
  (`BNE WCGO` ~line 422, already-known) AND the fresh-discovery-success jump
  (`JMP WCGO` ~line 480). The known-fail roll MUST be inserted on the
  bit-set branch ONLY — between the `BNE WCGO` test and the cook — so that a
  fresh discovery that just passed its discovery roll cooks GUARANTEED (the
  "success learns free AND starts cooking" promise). DO NOT add the fail roll
  at WCGO itself, or fresh discoveries get double-jeopardy'd (pass discovery,
  then craft-fail on the same attempt). Restructure: the bit-set branch
  targets a NEW label (e.g. WKNOWN) that does the fail roll, falling through
  to WCGO on success; WCGO stays the shared guaranteed-cook tail.
- FAILBASE[4] + FAILFLOOR as blob tables (hot-ish, tiny). failChance =
  FAILBASE[rarity] − skill/2, clamped at FAILFLOOR (8-bit: the subtract
  borrows when skill/2 > FAILBASE → take the floor).
- New op `WOPLEARN = $05` (I0 = recipe idx): validates idx < NRECIP and
  DISC bit clear → set bit, ROK; else RERRBAD. NO skill/cash validation in
  the task (the //e gates skill and pays first; task only guards
  double-learn). Blob cap 4096 — no pressure.

### FARMTASK changes

- New op `OPSPEND = $07` (TA0 = lo, TA1 = hi): cash >= amount → subtract,
  ROK; else RERRCASH ($E4, FARMTASK-returned). 16-bit compare-then-subtract
  (CBUY precedent).
- Blob is at **1772 / 1792 = 20 B headroom** (verified 2026-06-12, post
  inc-3 — it already carries OPWITHDRAW/OPADDCASH). OPSPEND is ~50 B, so the
  37 B hi-byte reclaim is a REAL prerequisite, not optional: per-crop addr
  hi bytes are constant $02 (FSEEDS=$0222, FCROPS=$0226, NCROPS=4 → max +3 =
  $0225/$0229, low-byte ADC never carries → hi byte provably $02). Replace
  the `LDA #0 / ADC #>tbl / STA tmp` + `LDY tmp` patterns with `LDY #>tbl`
  immediates in CPLANT/CBUY/CHARV/CSELL, THEN add OPSPEND. Add a comment at
  each site: `; hi const $02 - breaks if FSEEDS/FCROPS cross a page`. Net
  must stay < 1792 (report the size; if still tight, the reclaim has more
  sites — the EVLIB/PORTLIB calls use the same pattern elsewhere).

### //e (FARM.S)

- RPRICE table (12 x 16-bit) — display + cash gate //e-side. No SKILLREQ
  table (skill gate dropped).
- Market screen: recipe row (placement at the implementer's discretion —
  rows 16-18 are free; verify R ($D2) is unbound on the market screen first)
  + R key handler: find lowest unowned bit (WDISCV mirror, synced by WSYNC;
  re-sync just before), check cash >= RPRICE[idx], OPSPEND(price) →
  WOPLEARN(idx) → on ROK refresh + `LEARNED <name>!`; on WOPLEARN RERRBAD
  → OPADDC(price) refund + re-sync + redraw (silent or `ALREADY KNOWN`);
  too poor → `NEED CASH`; all owned → no-op.
- Workshop craft result dispatch adds `$E9 → CRAFT FAILED`; `$E8` message
  stays `RUINED!`.
- CVERNUM = 2. **Destructive on deploy:** CVER bump forces a bank-33 re-seed
  → pantry, skill, and owned recipes all reset to zero. No migration. Farm
  world (bank 32) untouched. Acceptable for bench iteration.
- Recipe book rendering unchanged (bitmap-driven; bought and discovered
  recipes look identical — both "known").

### Result codes (cumulative)

| Code | Meaning |
|---|---|
| $01 ROK | ok / cooking |
| $E4 RERRCASH | OPSPEND insufficient funds |
| $E5 RERRCROP | not enough crops/ingredients |
| $E6 RERRBAD | bad args / double-learn / unknown op |
| $E7 RERRFULL | stations full / inventory full |
| $E8 RRUIN | unowned attempt failed (dud OR discovery miss) |
| $E9 RFAIL | KNOWN recipe craft failed |

### Testbench

- WOPLEARN: learn lowest (BREAD idx 0) → bit set; double-learn → RERRBAD.
- OPSPEND: exact-funds success (cash → 0) + insufficient → RERRCASH + cash
  unchanged.
- Known-recipe fail curve (distribution, NOT pin-to-observed): seed the LFSR
  deterministically (wk_init already seeds it), own BREAD, loop ~30 attempts
  at skill 0 refilling pantry each time. Assert: every result is ROK or $E9
  (no other code), pantry is debited on every attempt regardless, AND both
  ROK and $E9 occur at least once across the run (proves the roll is live,
  not stuck). At skill $FF, failChance = FAILFLOOR(8/256): assert the
  observed fail RATE over ~30 attempts is low (e.g. < 6 of 30) rather than
  pinning a single attempt.
- Discovery path asserts from increment 3 remain valid (unowned combos →
  $E8 on dud, roll on real-unowned).
- wk_init seeds DISC=0 and CVER=2.

## Tuning knobs (all data)

| Knob | Value | Where |
|---|---|---|
| Price | 2 x value | FARM.S RPRICE (cash gate only) |
| FAILBASE | 48/80/112/144 | WORKTASK table |
| FAILFLOOR | 8 | WORKTASK equate |
| Discovery curve | 40 + skill/2 − 16 x rarity | unchanged |

## Accepted trade-offs (noted, not fixed)

- **Dud-spam skill grind:** skill++ on every attempt incl. duds means a
  player can cheaply grind skill toward $FF (ingredient cost is the only
  brake), eventually flooring all craft failure at ~3%. Accepted — the
  ingredient cost and the ladder price still pace the early game; endgame
  mastery is a fine reward. Revisit only if bench shows it trivializes.

## Out of scope

- Recipe-book UI for picking a craft (MIX entry stays — tactile).
- Workshop status widget on the farm screen (separate polish item, parked).
- D-key deposit crop picker (separate approved polish item, parked — was
  interrupted by this redesign; still wanted).
