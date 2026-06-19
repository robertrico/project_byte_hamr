# NPC Market Replacement + UI Clarity — Phase 2C (Design Spec)

Date: 2026-06-19
Status: approved in brainstorm (pending spec review)
Scope: make the NPC purchaser/vendor system (Phase 2A engine + 2B UI) the
ONE market, removing the old anonymous per-crop market, and fix the
drill-in UI confusion surfaced on the bench. Branch `farm-v2`.
Parents: `2026-06-18-economy-rebalance-vendors-design.md` (§2 NPCs),
`2026-06-18-trade-ui-phase2b-design.md` (the TRADE screen this revises).

## Goal

On the bench the player has TWO redundant commerce channels: the old
anonymous market (M screen — sell raw crops at the drifting FMKT price, buy
seeds at static SEEDC) AND the NPC traders (T screen). GROCER duplicates the
market's crop-sell; the seed vendors duplicate its seed-buy. Two screens for
the same action. **Decision: the NPC system IS the market — there is no
separate anonymous market.** Plus the drill-in UI is confusing (unlabeled
columns, ambiguous item names), so this phase also fixes the presentation.

## §1 — The NPC system becomes the market

- **Rename "TRADE" → "MARKET"** in all UI strings. Drop the word "Trade".
- **The `M` key opens the Market** (the NPC picker — what was the `T`/TRADE
  screen). **Remove the `T` key.** **Remove the old market screen** entirely
  (its crop-sell `S`, seed-buy `B`, and per-crop price table display).
- **All commerce is via named NPCs:** GROCER/TRADER buy crops, SEEDSHOP/
  BULK/EXOTIC sell seeds, BAKER/TRADER buy goods (the Phase 2A roster,
  unchanged). There is no anonymous "market price" — every price is a
  named NPC's price.
- **Retire FMKT + DOMKT.** The per-crop FMKT table + its supply-crash drift
  (DOMKT in FARMTASK) are no longer read by anything. Remove DOMKT + its
  tick call (RELOADM) from the FARMTASK blob, and remove the FMKT base-price
  seeding from the //e cold-seed (COLDST). The NPC price tables (`FNPC`/
  `WBNPC`) are the only prices; `NPCDFT`/`WNPDFT` are the only price drift.
  The dump-crashes-price feel survives **per-NPC** (selling to GROCER drops
  GROCER's price; `NPCDFT` mean-reverts it). The old market ops `OPSELL`/
  `OPBUY` become unused by the //e — leave them in the blob (harmless dead
  code; removing them risks branch-distance churn for no gain).

Net: one commerce screen (`M` = MARKET), one pricing model (NPC tables).

## §2 — Recipe shop moves to the Workshop

The recipe-learning shop (the "BUY nnn to learn recipe" line, currently a
tenant on the old market screen via the `R`/SHOPBUY path) loses its home
when the market screen is removed. **Move recipe-learning to the Workshop
screen (`W`).** Thematically correct — you learn recipes where you craft.

- Remove the recipe-shop line + its key handler from the (now-deleted)
  market screen.
- Add a LEARN-recipe action to the Workshop screen: list undiscovered
  recipes with their learn-cost (`RPRICEL`/`RPRICEH` = 2× VAL), let the
  player pick one + confirm, and execute the existing two-leg learn
  (debit farm cash via `OPSPEND`, then `WOPLEARN` on the workshop blob —
  the same sequence the old SHOPBUY used, with the refund-on-fail guard).
  No engine/blob change — `WOPLEARN` + `OPSPEND` already exist.

## §3 — Drill-in UI redesign (the confusion fix)

The drill-in showed two unlabeled number columns and bare item names. Fix:

**Column headers (context-aware by NPC role), drawn under the title:**
- Vendor (SEEDSHOP/BULK/EXOTIC): `ITEM        OWN  COST`
- Purchaser (GROCER/TRADER/BAKER): `ITEM        OWN  PAYS`

`OWN` = the player's current count of that item (seeds for a vendor, crops/
goods for a purchaser). `COST` = what the player pays per unit (vendor).
`PAYS` = what the NPC pays per unit (purchaser). This labels the two number
columns that were previously bare.

**Item-name suffix by form:**
- Vendor item (you're buying the SEED form): append " SD." → `WHEAT SD.`
- Purchaser crop item (raw harvested crop): plain → `WHEAT`
- Goods item: the product name (already distinct, e.g. `BREAD`) → no suffix

The suffix is decided by NPC role + item type: vendor + crop-type → "SD.";
otherwise no suffix. (Goods names are unique, crops-to-a-buyer are the raw
crop.)

**Action legend by role:** `RET=BUY ESC=BACK` (vendor) / `RET=SELL ESC=BACK`
(purchaser) — replaces the generic "RET=TRADE". (The qty prompt + result
string are already role-aware from 2B: "BUY QTY:"/"SELL QTY:",
"BOUGHT"/"SOLD".)

**Picker title:** `MARKET  PICK SHOP` (or similar) — replaces
`TRADE  PICK NPC`.

Mock (after):
```
SEEDSHOP   SELLS SEEDS    CASH 00088     GROCER    BUYS CROPS     CASH 00088
 ITEM        OWN  COST                    ITEM        OWN  PAYS
>WHEAT SD.    11    2                     >WHEAT       12     8
 CARROT SD.    0    4                      CARROT       3    14
 BERRY SD.     0    7                      BERRY        0    20
 PUMPKN SD.    0   12                      PUMPKN       5    32
RET=BUY ESC=BACK                          RET=SELL ESC=BACK
```

## §4 — What stays unchanged

- Phase 2A engine: ops (OPTSELL/OPTBUY/WOPTSELL), NPC tables (FNPC/WBNPC),
  drift (NPCDFT/WNPDFT), base seeding (NPCBASE.S), CVER 6. No engine change.
- The 2B picker + drill-in scaffolding (SCREEN states, TRROW/TRDDRAW/TRDKEY/
  TRADEGO, QPMODE modal) — revised in place (rename, headers, suffix,
  M-key), not rebuilt.
- The Phase-1 recipe rebalance + crop/seed economy values.

## §5 — Migration

None for SDRAM/CVER — the NPC regions are already seeded (2A, CVER 6) and
their layout is unchanged. Removing FMKT seeding from COLDST is a //e change
only; FARMTASK losing DOMKT is a blob change (disk rebuild, NO re-flash —
the blob is runtime-loaded). The market-screen removal + workshop-learn +
UI rework are //e-only.

## §6 — Testing

Engine is FARM_TEST-green (2A, unchanged). This phase is //e paint +
screen-flow + a blob edit (DOMKT removal):
- **Blob:** FARMTASK still must spawn, service the mailbox, and run NPCDFT
  on the market tick after DOMKT is removed. Add/keep a FARM_TEST assertion
  that NPC drift still fires (the existing "NPC DRIFT TICK" case covers
  this — confirm it stays green after DOMKT removal).
- **UI:** bench play (paint code, not FARM_TEST-able) — visit each NPC,
  confirm headers/suffix/legend read clearly, buy + sell work, workshop
  recipe-learn works, the old M market screen is gone and `M` opens the
  NPC market.
- Add a static NPC-table consistency guard (the option-1 check raised
  earlier: NIOFF = cumulative NICNT, 34 items total, ids in range, 6-entry
  roster/name/hint tables) as a build-time gate, since the tables are the
  one silent-failure surface.

## Out of scope

- The cross-NPC "best price" marker (dropped in 2B, still dropped).
- Removing the dead OPSELL/OPBUY blob ops (left as harmless dead code).
- Crop-economy retune (BASEP/SEEDC values) — unchanged.
