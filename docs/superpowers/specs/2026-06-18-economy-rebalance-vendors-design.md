# Economy Rebalance + Purchaser/Vendor System (Design Spec)

Date: 2026-06-18
Status: approved in brainstorm (pending spec review)
Scope: farm/workshop economy redesign on branch farm-v2. Two phases:
(1) static rebalance of recipe payoffs to a rarity+risk curve, (2) a
purchaser/vendor trading layer with per-NPC supply/demand price tables.

## Goal

The play is sound but **baking bread is a runaway** — you get ahead too
fast by spamming it. Two coupled fixes:

1. **Rebalance** the values/weights/payoffs so cheap/common/safe earns a
   thin margin and rare/risky earns a fat one (the profit curve should
   reward harder play, not punish it).
2. **Purchasers + vendors** with their own fluctuating price tables — NPC
   buyers/sellers whose prices move with your trades, adding shop-around
   depth and dynamically capping any single product's profitability.

## The bread diagnosis (why it's a runaway)

Crops (wheat/carrot/berry/pumpkin): seed cost `2/4/7/12`, base sell
`8/14/24/40`, growth `every 1/2/4/8 ticks`. Bread = 2 wheat → VAL 28, RARITY 0.

Bread wins on **all three axes at once**: wheat is the **cheapest seed (2),
fastest grow (every tick), and RARITY-0 (near-zero craft fail)**. 2 wheat
(16 as crops, 4 in seeds) converts to 28 — a 75% markup with no risk and
minimal time.

Worse, the current markup curve is **inverted**. Markup % (VAL over the
sum of input crop base-values), by rarity:

| Recipe | inputs | input value | VAL | markup | RARITY |
|---|---|---|---|---|---|
| 0 bread | 2 wheat | 16 | 28 | +75% | 0 |
| 1 | wheat+carrot | 22 | 36 | +64% | 0 |
| 2 | 2 carrot | 28 | 48 | +71% | 0 |
| 8 | 2 carrot+pumpkin | 68 | 100 | +47% | 2 |
| 11 | all four | 86 | 127 | +48% | 3 |

The cheap/safe RARITY-0 recipes carry a **higher** markup than the
rare/risky RARITY-2/3 ones. Backwards. Fix: invert the curve.

## §1 — Phase 1: static rebalance (rarity+risk curve)

Retune each recipe's **VAL** (RECTAB byte 5) so markup scales with rarity.
Target markups: R0 ~12%, R1 ~35%, R2 ~65%, R3 ~max. VAL is a byte and
must stay ≤127 (goods values keep the high bit clear). New table:

| Recipe | inputs | input value | RARITY | new VAL | (old) |
|---|---|---|---|---|---|
| 0 bread | 2 wheat | 16 | 0 | **18** | 28 |
| 1 | wheat+carrot | 22 | 0 | **25** | 36 |
| 2 | 2 carrot | 28 | 0 | **32** | 48 |
| 3 | wheat+berry | 32 | 1 | **43** | 56 |
| 4 | 2 wheat+berry | 40 | 1 | **54** | 76 |
| 5 | 2 berry | 48 | 1 | **64** | 80 |
| 6 | wheat+pumpkin | 48 | 1 | **64** | 80 |
| 7 | carrot+pumpkin | 54 | 1 | **72** | 92 |
| 8 | 2 carrot+pumpkin | 68 | 2 | **112** | 100 |
| 9 | wheat+berry+pumpkin | 72 | 2 | **119** | 105 |
| 10 | 3 berry | 72 | 2 | **119** | 110 |
| 11 | all four | 86 | 3 | **127** | 127 |

Result: **bread = 18** is only +2 over selling its 2 wheat (16) — a
marginal early option, not a money pump. The all-four R3 recipe (127) is
the top absolute payoff and uses the rarest inputs + highest fail-risk.
The curve is monotonic-ish increasing with rarity; absolute profit per
craft spans ~20× (bread +2 → R3 +41).

**Risk is already rarity-scaled — keep it.** `FAILBASE = 48/80/112/144`
(per rarity), failChance = FAILBASE − skill/2 floored 8. R0 bread ~3-19%
fail; R3 ~56% at low skill. Higher reward tracks higher fail. No change.

**Crop economy (BASEP/SEEDC/GROWM) unchanged in Phase 1.** Raw-crop
per-tick still favors wheat (early-game), but rare crops gain value as the
inputs to the fat high-rarity recipes, and Phase 2's rare-crop purchaser
(TRADER) pays a premium for berry/pumpkin — so rare-crop value comes from
the craft + trade layers, not raw selling. (Crop-economy tuning is a
possible later pass, out of scope here.)

**FOUR tables must stay in sync — retune ALL of them to the new VALs.**
(Each is active; missing one silently breaks an invariant — assembles
clean, surfaces only on the bench.)
1. **`RECTAB` byte 5 (FARM.S)** — the cash authority, streamed to SDRAM on
   cold-seed. The source of truth.
2. **`RVALUE` (FARM.S)** — the //e goods-price DISPLAY. Must equal RECTAB
   byte 5.
3. **`RPRICEL`/`RPRICEH` (FARM.S)** — recipe LEARN-COST, driving SHOPBUY →
   OPSPEND debit + the shop "BUY nnn" display. The code's invariant is
   **price = 2× value** (RPRICE = 2× old VAL today). **Preserve it:** set
   RPRICEL/H = **2× the new VAL**. New values — all ≤254, so RPRICEH stays
   all 0: `36,50,64,86,108,128,128,144,224,238,238,254`.
4. **`RECTAB` in `FARMTEST.S`** — the self-test keeps its OWN duplicate
   RECTAB and streams it to SDRAM (`$0300`) for the test. Retune this copy
   too, or §4's WSELL assertions check stale VALs.

(RECTAB = cash authority; RVALUE = display; RPRICE = learn-cost; the
FARMTEST copy = test seed. All four agree on the new VALs, RPRICE = 2× VAL.)

Phase 1 is small, self-contained, and ships first as the immediate bread
fix — fully testable in FARM_TEST (assert WSELL returns the new per-unit
VAL).

## §2 — Phase 2: purchaser/vendor system

### NPCs (specialized, few)

**Purchasers** (buy from you, fluctuating buy prices):
- **GROCER** — buys raw crops, favors common (wheat/carrot); pays ~market.
- **BAKER** — buys baked goods (the bread/grain recipes); premium on goods.
- **TRADER** — buys rare crops (berry/pumpkin) + high-rarity goods (R2/R3);
  premium on rare.

**Vendors** (sell to you, fluctuating sell prices):
- **SEED-SHOP** — sells all 4 seeds at a baseline.
- **BULK-VENDOR** — common seeds (wheat/carrot) cheaper, in bulk.
- **EXOTIC-VENDOR** — rare seeds (berry/pumpkin), pricier.

"Specialized" = each NPC's table covers a subset and sits at a different
base, so matching the right buyer/seller and shopping around both matter.
The **existing channels stay as baselines**: the per-crop market (FMKT) for
selling raw crops, and static `SEEDC`/`OPBUY` for buying seeds. NPCs
*augment* these — sometimes better, sometimes worse, and fluctuating.

**TRADER spans both banks.** It buys rare crops (bank 32) AND high-rarity
goods (bank 33), so it has a table entry in each bank (updated by the
respective task). The //e UI presents it as one NPC; physically its two
tables live where the items live. (One logical NPC, two SDRAM entries.)

### Supply/demand fluctuation (your trades move prices)

Each NPC holds a price per item it trades: a **current price** and an
implicit **base** (from a static table). The mechanic mirrors the proven
DOMKT drift, per NPC per item:
- **Selling** an item to a purchaser **drops** that NPC's price for it
  (it's saturating) — like the current market's SUPPLY bump on a sale.
- **Buying** from a vendor **raises** that NPC's price for it (demand).
- Each **market tick**, every NPC price **mean-reverts one step toward its
  base** (like the market's TGT drift + SUPPLY decay).

This caps runaways dynamically: dumping bread on the BAKER crashes the
baker's bread price; spreading across the (few) buyers helps but each
saturates, so total bread income is bounded. The static rebalance (§1)
sets the base; supply/demand modulates around it.

### Architecture (approach A — extend the existing tasks)

No new task slot, no cross-bank dance. Single-writer-per-bank preserved:
- **Crop-trading NPCs (GROCER, TRADER-crops, the seed vendors) live in
  bank 32**, and **FARMTASK** updates their tables on its market tick
  (extend DOMKT's per-crop drift to per-NPC-per-crop).
- **Goods-trading NPCs (BAKER, TRADER-goods) live in bank 33**, and
  **WORKTASK** updates them on its tick.
- Trade ops route to the owning task (crop trades → FARMTASK, goods trades
  → WORKTASK), reusing the mailbox protocol.

### SDRAM layout (MAX-sized, per the itr4.1 count≠layout discipline)

Reserve NPC regions in the existing reserved space. Sizes are MAX
(NCROPS_MAX=16, NRECIP_MAX/NGOODS_MAX=64); live counts bound loops.

- **Bank 32 (crops)** — new NPC region in the reserved `$0B00+` area. Each
  crop-NPC table = item price array. For a crop-purchaser/vendor covering
  up to NCROPS_MAX(16): 16 × (price lo, price hi) = 32 bytes; + a base
  table (static, in the blob, not SDRAM). Allocate fixed bases for
  NCROPS_MAX-sized tables per NPC: GROCER, TRADER-crops, SEED-SHOP,
  BULK-VENDOR, EXOTIC-VENDOR (5 crop-NPCs × 32 B = 160 B) at e.g.
  `$0B00`-`$0B9F`, each NPC at a fixed `$0B00 + npc*32` base.
- **Bank 33 (goods)** — new NPC region after the recipe reserve (`$0500+`).
  Goods-purchasers (BAKER, TRADER-goods) covering up to NGOODS_MAX(64):
  64 × (price lo, price hi) = 128 B per NPC. 2 NPCs × 128 = 256 B at
  `$0500`-`$05FF`, each at `$0500 + npc*128`.
- **Static base-price tables** (per NPC per item) live in the **blob**
  (DFB tables, like BASEP/RECTAB) — read-only, define each NPC's
  specialization. SDRAM holds only the live (fluctuating) current price.

### Trade ops (new mailbox ops)

Farm side (FARMTASK, crops/seeds):
- **OPTSELL** (npc, crop, qty) — debit crops, credit cash at NPC's current
  crop price, drop that NPC's price (supply impact), clamp.
- **OPTBUY** (npc, crop, qty) — for seed vendors: debit cash at NPC's
  current seed price, credit seeds, raise that NPC's price (demand).

Workshop side (WORKTASK, goods):
- **WOPTSELL** (npc, good, qty) — debit goods, credit cash at NPC's good
  price (returned per-unit in WRES1 like the existing WSELL op), drop price.

Validation mirrors the existing sell/buy (insufficient inventory →
RERRCROP, insufficient cash → RERRCASH, bad args → RERRBAD). The
single-writer rule holds: the owning task is the sole writer of its bank's
NPC prices; the //e reads them for display + sends ops.

### Trade UI (//e)

A **TRADE screen** (new key, e.g. `T`), paged like the inventory screen.
Shows the NPCs and their current prices for the relevant items
(purchasers' buy columns, vendors' sell columns), highlights the
best buyer/seller per item, and lets the player pick NPC + item + qty +
buy/sell → sends the trade op, repaints on the EV/price events. The
existing market screen (`M`) stays for the baseline per-crop market.

## §3 — Migration

Phase 1 (recipe VALs) changes bank-33 recipe content. The //e streams
RECTAB into SDRAM only on a **re-seed**, which a warm world (matching CVER,
SIG present) skips — so WORKTASK would keep reading the OLD VALs from SDRAM
and the rebalance would never reach existing saves. Phase 1 therefore
**bumps CVER 4 → 5** to force the re-seed of the rebalanced recipes on a
warm world. (FARM_TEST is always-cold and re-seeds regardless — CVER only
matters for the game.)

Phase 2 adds NPC regions to both banks → a further **CVER bump (5 → 6)**
that cold-seeds both banks fresh (init NPC current-prices to their bases).
Per the itr4.1 seed model: //e is sole seeder, writes the NPC base prices
into the live tables during cold-seed (before spawn), single-writer by
sequencing. Bench world resets once.

## §4 — Testing

All in FARM_TEST (the //e self-test, on hardware — the proven pattern:
force preconditions, deterministic, no slow-tick waits):
- **Phase 1 rebalance:** assert WSELL returns the new per-unit VALs
  (force a good, sell, check value); spot-check a few recipes craft +
  store the rebalanced value.
- **Phase 2 trade ops:** force a known NPC price (write the live table),
  OPTSELL/OPTBUY/WOPTSELL, assert cash delta = qty × that price, inventory
  delta, and the NPC's price moved the right direction (supply down / demand
  up). Deterministic — force the starting price, drive one op, check.
- **Mean-revert:** force a price off-base, let one market tick fire (the
  one allowed "see a click" wait), assert it stepped toward base.
- Keep it forced/deterministic; the fluctuation tuning (feel) is bench-play.

## Out of scope

- World events (itr 4) — orthogonal; events could later swing NPC prices,
  but not here.
- Crop-economy retune (BASEP/SEEDC/GROWM) — Phase 1 touches recipe VALs
  only; raw-crop balance is a possible later pass.
- More NPCs / NPC reputation / contracts — the reserved MAX-sized regions
  leave room, but Phase 2 ships the ~3+3 specialized set only.
