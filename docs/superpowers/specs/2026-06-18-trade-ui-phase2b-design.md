# TRADE UI — Phase 2B (Design Spec)

Date: 2026-06-18
Status: approved in brainstorm (pending spec review)
Scope: the player-facing TRADE screen for the purchaser/vendor trade engine
on branch `farm-v2`. Builds on Phase 2A (the engine: NPC price tables +
OPTSELL/OPTBUY/WOPTSELL ops + drift, all FARM_TEST-green on hardware).
Parent spec: `2026-06-18-economy-rebalance-vendors-design.md` §2 "Trade UI".

## Goal

Phase 2A built and bench-validated the trade engine, but trades are only
reachable via the mailbox (the self-test drives them). Phase 2B adds the
//e screen that makes trading playable: browse NPCs, see their live
(fluctuating) prices, and sell/buy. No engine changes — 2A already built,
seeded, and tested the ops + tables + drift; 2B is pure //e presentation.

## Architecture

All in `software/SDM/FARM.S`, following the existing market/inventory
screen idiom (key dispatch by `SCREEN`, paint routines, `SENDCMD`/
`WSENDCMD` for ops, `DRAIN`/`EVDISP` for event-driven repaint). The drill-in
reuses the inventory cursor+viewport scroll machinery (the `INVCUR`/`INVOFF`
+ `INVUP`/`INVDOWN` pattern) for NPCs whose item list exceeds the viewport.

Two new screen states:
- **TRADE picker** (level 1): the NPC list.
- **TRADE drill-in** (level 2): one NPC's items + prices + sell/buy.

No new blob, SDRAM region, or CVER change. 2A's `FNPC`/`FNPCB` (bank 32)
and `WBNPC`/`WBNPCB` (bank 33) tables are read for display; 2A's ops
(`OPTSELL`/`OPTBUY`/`WOPTSELL`) execute the trades.

`FARM.S` is already ~4500 lines; 2B adds ~250–350 lines in its established
screen style. Per project convention (don't unilaterally restructure large
files), the trade screens live in `FARM.S` alongside the other screens.

## Level 1 — NPC picker

Entered by the `T` key from the farm screen (a new `SCREEN` value; ESC
returns to the farm screen). A cursor list of the 6 NPCs with a one-line
role hint:

```
TRADE  PICK NPC        CASH 01234
>GROCER   buys crops
 TRADER   buys rare crop+goods
 SEEDSHOP sells seeds
 BULK     cheap common seed
 EXOTIC   rare seed
 BAKER    buys goods
RET=OPEN ESC=EXIT
```

- Up/down move the cursor (`>`); RET opens the highlighted NPC's drill-in;
  ESC exits to the farm screen.
- The NPC roster is fixed (6 entries): GROCER, TRADER, SEEDSHOP, BULK,
  EXOTIC, BAKER. TRADER is ONE entry (it trades both banks — see curation).

## Level 2 — drill-in (per NPC)

Shows the NPC's traded items, each with the player's HAVE count and the
NPC's current price (read live from SDRAM). Cursor selects an item; the
action is **role-based** — a single action key per NPC:
- **Purchasers** (GROCER, TRADER, BAKER) → RET = **SELL** (player sells the
  item to the NPC).
- **Vendors** (SEEDSHOP, BULK, EXOTIC) → RET = **BUY** (player buys seeds).

RET on an item opens the existing `QTYPROMPT` modal (3-digit qty, clamps to
255), then sends the op and repaints. ESC returns to the picker.

```
GROCER (BUYS CROPS)    CASH 01234      TRADER (BUYS RARE)   CASH 01234
 ITEM     HAVE  PRICE                   ITEM     HAVE  PRICE
>WHEAT      12     8                   >BERRY c     0    40
 CARROT      3    14                    PUMPKIN c   5    64
 BERRY       0    20                    CAKE g      2   145
 PUMPKIN     5    32                    PIE  g      1   145
RET=SELL ESC=BACK                      RET=SELL ESC=BACK
```

NPCs whose item list exceeds the viewport (BAKER = 12 goods; TRADER = 6
mixed) scroll via the inventory cursor+window pattern (cursor walks within
the window; window scrolls at the edges; up/down-arrow glyphs mark more
above/below). Short lists (4 items) need no scroll.

## NPC item curation

A small static table in `FARM.S` defines which items each NPC trades (and,
for TRADER's mixed list, a per-row crop/good tag). The engine's ops accept
more than the UI presents (e.g. OPTSELL works for any crop to any
purchaser); the curation is the UI's specialization layer:

| NPC | role | items shown | op |
|---|---|---|---|
| GROCER | purchaser | 4 crops (wheat,carrot,berry,pumpkin) | OPTSELL (npc 0) |
| TRADER | purchaser | rare crops (berry,pumpkin) + high-rarity goods (recipes 8,9,10,11) | OPTSELL (npc 1, crop rows) / WOPTSELL (npc 1, good rows) |
| BAKER | purchaser | 12 goods (recipes 0–11) | WOPTSELL (npc 0) |
| SEEDSHOP | vendor | 4 seeds (crops 0–3) | OPTBUY (npc 2) |
| BULK | vendor | 4 seeds | OPTBUY (npc 3) |
| EXOTIC | vendor | 4 seeds | OPTBUY (npc 4) |

TRADER's rows are tagged `c` (crop) or `g` (good); SELL routes to OPTSELL
(crop, bank 32) or WOPTSELL (good, bank 33) by the tag. TRADER's crop price
comes from `FNPC + 1*32 + crop*2`; its goods price from `WBNPC + 1*128 +
recipe*2`. (The goods-side NPC index for TRADER is 1 = NPTRDG; the
crop-side index is 1 = NPTRDC — both equal 1 by 2A's equates.)

## Data + ops

- **Prices**: read live from SDRAM on each paint. Crop-side from `FNPC`
  ($0B00) bank 32; goods-side from `WBNPC` ($0500) bank 33. Price is the
  low byte (`FRD`/`WFRD` helpers).
- **Player HAVE**: from the existing display caches — crops `CROP4`, seeds
  `SEED4` (refreshed by `RDMKT`), goods `WGOODS` (refreshed by `WSYNC`).
- **Cash**: the existing HUD cash read.
- **Ops**: SELL crop → `SENDCMD` with `OPTSELL`; SELL good → `WSENDCMD`
  with `WOPTSELL` (returns per-unit price in `WRES1V`; the //e multiplies
  by qty and credits cash, exactly as the existing goods-sell path does);
  BUY seed → `SENDCMD` with `OPTBUY`. Args: NPC index, item index, qty.
  Result handling reuses the existing `DOCMD`-style dispatch (timeout →
  message, error code → hex, ROK → confirm + repaint).

## Repaint

- Re-read NPC prices from SDRAM on drill-in entry and after every trade
  (a trade changes the price).
- While on a trade screen, an `EVPRICE` event draining in `EVDISP`
  triggers a price re-read + repaint of the current drill-in (NPC drift
  piggybacks `DOMKT`'s per-tick `EVPRICE`, so the displayed prices track
  the live drift). On the picker screen, `EVPRICE` is ignored (no prices
  shown there).

## Dropped (YAGNI)

The cross-NPC "best buyer/seller" marker from parent-spec §2 is **not**
implemented. It fit the rejected column-layout; in the chosen drill-in
model the player is inside one NPC at a time, so a cross-NPC marker has no
place. Shopping around = visiting NPCs. Deliberate omission, not a gap.

## Testing

This is //e paint code: rendering cannot be asserted via SDRAM readback, so
it is **not** covered by FARM_TEST (consistent with the existing market and
inventory screens, which are also untested paint code). The engine beneath
it — the ops, tables, and drift — is already FARM_TEST-green from Phase 2A.

2B is validated by **bench play**: enter TRADE, visit each NPC, confirm
prices display and track drift, execute a sell and a buy at each role, and
confirm cash/inventory/price update and the screen repaints. The 2A
self-tests remain the regression guard for the engine.

## Migration

None. Phase 2A already bumped CVER 5→6 and cold-seeds the NPC regions; 2B
reads what 2A seeded. No CVER bump, no re-seed, no blob change. A blob-free
//e-only change ships by disk rebuild (no re-flash).

## Out of scope

- Engine changes (ops/tables/drift) — done + tested in 2A.
- New NPCs / reputation / contracts — parent-spec "out of scope".
- The cross-NPC best-price marker (see Dropped).
