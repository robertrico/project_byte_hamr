# Farm v2 — Sidework, Seeds, Screens, Events (Design Spec)

Date: 2026-06-11
Status: approved in brainstorm (this doc is the record)
Base: farm/market v1 @ main `a0be44e` — see `software/SDM/HANDOFF_FARM_V2.md` and
`docs/superpowers/specs/2026-06-10-farm-market-event-game-design.md`

## Goal

Make the farm game *playable*: give the player productive sidework during growth
waits (crafting with discovery), better money velocity (sell quantities, seed
variety), and a world-events layer that makes all of it swing. Architecturally:
prove the multi-task story — two live coproc tasks, two SDRAM banks, one //e
client — which is the founding async-multitasking goal of project_obscurus.

## Decisions made (with rationale)

1. **Sidework character = management depth**, not an active minigame. Decisions
   that resolve over time (crafting queues, recipe experiments) fit the
   event-driven coproc architecture natively. Active minigame (fishing/critters)
   parked as a v2.5 candidate on a third task slot.
2. **Full arc, staged**: sell-qty UX → seeds → workshop task → world events.
   One spec, one plan per increment, each increment bench-playable alone.
3. **Crafting = second coproc task (WORKSHOP)**, own SDRAM bank, own
   ring/mailbox. Rejected: extending FARMTASK (blob budget collision with
   seeds+events) and //e-side crafting (loses offline progression — crafts
   must keep cooking while the player quits or soft-resets).
4. **Multi-screen //e UI**: FARM (GR mixed), MARKET (text), WORKSHOP (text);
   SELL screen with per-vendor pricing reserved for v2.6. Too much game for
   one screen.
5. **Morrowind-style crafting**: hidden table-based recipe book, up to 4
   ingredient elements, discovery gated by RNG + a craft-skill stat that
   grows with every attempt.
6. **Multiple crops grow side-by-side** on the existing 20x20 grid (crop type
   is per-plot). Multiple *fields* (second grid screen) is a v3 candidate —
   the screen manager makes it cheap later.

## Architecture conventions (new, project-wide)

### Per-game coproc RAM ownership

Task RAM **$0300-$0F7F belongs to the ACTIVE game**. One game runs at a time;
each game brings its own task set and scratch map. Conway demo (skills 0/1) is
one game; farm is another. Overlap between games is the design, not a conflict.
Invariant across games: ZP (kernel), $0100-$01FF stacks, $0200-$02FF TABLE
(write-protected), $0F80-$0FFF kernel mailboxes, $1000-$1FFF kernel.

Farm game footprint:

| Region        | Use                                            |
|---------------|------------------------------------------------|
| $0300-$05FF   | WORKSHOP code (skill 3, 768 B Makefile cap)    |
| $0600-$0BFF   | FARMTASK code (skill 2, 1536 B cap, unchanged) |
| $0C00-$0DFF   | workshop scratch (reclaimed from Conway)       |
| $0E00-$0E2C   | EVLIB + FARMTASK scratch (unchanged)           |
| $0E30-$0F7F   | spare                                          |

Switching games is cold-start territory: the new game's blobs load, the old
game's tasks die, each game's world persists in its own GBANK(s).

### Data placement rules

The coproc executes from the 8 KB BRAM only — SDRAM is data-only via the
$E000-$E008 windows. Therefore:

- **Code → blob.** Blobs hold logic only.
- **World state → SDRAM bank** (v1 pattern: grid, market, ring).
- **Static content → SDRAM, //e-seeded at cold start** (recipe table, crop
  profiles, news/headline strings, future vendor tables). Content edits never
  touch a blob. Seeding happens before the SIG write, so the single-writer
  invariant is never violated (same ordering as v1 grid init).
- **Blob-resident data only for hot-loop constants**; optionally copy
  SDRAM→scratch once at task start.

Speed: //e via SDMLIB is a few bus cycles per byte (re-entry already reads the
400 B grid this way); coproc window access is RDY-stalled at ~25 MHz — table
scans are effectively free. SDRAM is plenty fast for everything but per-plot
per-tick inner loops.

### Single-writer + //e-as-bus

GBANK single-writer invariant holds: FARMTASK owns bank 32, WORKSHOP owns
bank 33. All cross-bank transfers are mediated by the //e via two mailbox
round-trips (e.g. withdraw from farm, deposit to workshop). No task ever
writes another task's bank.

## Components

### WORKSHOP task (new, skill id 3)

- ORG $0300, cap 768 B, budget=0 forever loop, sole writer of GBANK 33.
- Reuses EVLIB (PUTEV) and the FARMEQU pattern via a new WORKEQU.S PUT-include.
- Loop: poll mailbox → handle op → tick station timers → emit ring events.

GBANK 33 layout (signature "WK" = $57,$4B):

| Addr        | Content                                              |
|-------------|------------------------------------------------------|
| $0000-$0001 | SIG $57,$4B ("WK")                                   |
| $0002       | SEQCTR                                               |
| $0003       | HEAD                                                 |
| $0004       | heartbeat (increments every loop pass)               |
| $0100-$01FF | RING 64x4, page-aligned (EVLIB verbatim)             |
| $0200-$0207 | MAILBOX, 8 B: FLAG, OP, I0-I3, RES, spare            |
| $0210+      | pantry: 4 per-crop ingredient counts; goods          |
|             | inventory; 2 station records (recipe, timer, state); |
|             | CRAFT SKILL byte                                     |
| $0240       | discovered-recipe bitmap                             |
| $0300+      | recipe table (//e-seeded static content)             |

Mailbox ops: OPDEPOSIT(crop, qty), OPCRAFT(I0-I3), OPCOLLECT(→ product,
value), OPSTAT. Ring events: WEVDONE(station, product); heartbeat is polled,
not evented.

### Crafting / recipe model

- Recipe record (8 B): 4 ingredient slots (crop ids, $FF = empty, sorted
  canonical; a repeated id means "needs 2 of it") + product id + craft-time
  class + value + flags (rarity tier → harder discovery roll).
- ~12-16 real recipes over 4 crops; combo space (2-4 elements with repetition)
  is 65, so most combos are duds. Table lives in GBANK 33 SDRAM; blob scans it
  through the read window on each OPCRAFT.
- Discovery: undiscovered + correct combo → LFSR roll vs (BASE + craft skill
  − rarity). Pass = bitmap bit set + product crafted. Fail = "RUINED",
  ingredients lost. **Every attempt increments craft skill**, success, fail,
  or dud combo. Discovered recipes always succeed.
- Stations: 2 concurrent crafts; seconds-scale divider per tick decrements
  active station timers; WEVDONE on completion. Crafts keep cooking while the
  player is on another screen, quit, or after a soft reset (task respawn).
- Cash-out: //e OPCOLLECT (workshop) → OPADDCASH (farm task, new op). One
  wallet, lives in farm bank as today.
- Product value > ingredient cost x1.5-2.5; BOOM event doubles craft margins.

### Seeds (FARMTASK changes)

- Plot byte = `crop*8 + stage`: crop = byte>>3 (0-3), stage = byte&7
  (0 empty [global $00], 1-5 grow, 6 ripe, 7 dead). Shift/mask arithmetic, no
  lookups. Old saves incompatible — acceptable, world is session-only.
- 4 crop profiles (all data, SDRAM where possible):

| Crop    | Seed cost | Growth  | Price base | Role                          |
|---------|-----------|---------|------------|-------------------------------|
| WHEAT   | cheap     | fast    | low        | volume + craft staple         |
| CARROT  | mid       | mid     | mid        | balanced                      |
| BERRY   | dear      | slow    | high       | margin play                   |
| PUMPKIN | dearest   | slowest | highest    | craft-heavy, event-sensitive  |

- Growth: single global GROWD tick retained; per-crop speed = advance every
  Nth tick via a 4-entry mask table.
- Market: farm-bank $0210 area becomes 4 x (PRICE lo/hi, SUPPLY) with
  per-crop drift; state already in SDRAM, no blob data cost.
- Inventory: FSEEDS/FCROPS become 4-wide arrays.
- Mailbox: OPPLANT(+crop), OPSELL(+crop, qty), OPBUY(+crop); new
  OPWITHDRAW(crop, qty) and OPADDCASH(value lo/hi) for the //e bus.
- Render: per-crop GR base color, brightness/pattern by stage, ripe bright,
  dead magenta ($1). Exact colors tuned at plan time with bench screenshots.
- Blob budget: ~330 B headroom now; seeds ≈ 150-200 B (indexed loops replace
  scalars); events take the remainder; SDRAM offload is the relief valve.

### //e screen manager

- Screen id byte + per-screen jump tables: render-full, key-handler,
  event-hook.
- Main loop unchanged in shape: poll ring(s) → dispatch each event to state
  updaters (always) → current screen's event-hook decides redraw; poll
  keyboard → current screen's key-handler.
- Screens: FARM (GR mixed, the 20x20 grid), MARKET (text 40x24: 4-crop price
  board with trends, buy seeds, sell crops with 1-3 digit quantity prompt),
  WORKSHOP (text: pantry, 2 stations with timers, recipe book — known recipes
  named, unknown as "?????", combo picker). SELL/vendor screen reserved v2.6.
- Keys: global M = market, W = workshop, ESC = back to farm; Q quits from
  farm screen (confirm). Farm legend becomes PLANT HARVEST MARKET WORKSHOP
  QUIT (inverse first letters). **S is removed; selling lives on the market
  screen only.**
- Screen switch = soft-switch GR/TEXT + full repaint from local state; the
  farm repaint reuses the existing re-entry resync renderer.
- News/status line pinned on every screen.
- Re-entry/respawn: probe BOTH SIGs, respawn whichever task is dead
  (SPAWNT x2), resync two rings with two expected-SEQ counters — the proven
  v1 pattern, doubled.

### World events (FARMTASK)

- Day counter in farm bank, ticks every N market ticks → EV_DAY; every screen
  shows DAY nn.
- EV_NEWS: LFSR roll per day → one of CALM (default), DROUGHT (growth masks
  x2 slower, crop prices climb), BOOM (craft values x2 — //e forwards a
  MODESET op to the workshop task; bus pattern, no cross-bank read), BLIGHT
  (random growing/ripe plots → stage 7 dead; per-crop susceptibility).
- One active event at a time; state = (event id, days left) in farm bank.
  Headline strings are SDRAM content, //e-seeded.
- Dead plots cleared by replant only (P on a dead plot = clear + plant).
- Intended dynamics: drought → sell the stockpile; blight → seed variety as
  insurance; boom → workshop margin play.

### Heartbeat

Both tasks increment byte $0004 of their own bank every loop pass. //e (or
the monitor) can diagnose a wedged-vs-live task at a glance. Ships in
increment 1.

## Ship order (one plan per increment, each bench-playable)

1. **Sell-qty UX + heartbeat** — market-row quantity prompt is superseded by
   the market screen later, so increment 1 ships the heartbeat (blob change,
   deploy via soft reset) plus the S→qty prompt on the current farm HUD as an
   interim win. Smallest possible bench pass.
2. **Screen manager + MARKET screen + seeds** — manager lands here because
   buying seeds forces it; FARMTASK seed changes; S key removed in favor of
   M/W navigation.
3. **WORKSHOP task + workshop screen** — second task, GBANK 33, recipes,
   discovery, stations.
4. **World events** — EV_NEWS/EV_DAY/blight; news row everywhere; dead-plot
   render.

## Testing

- Sim (`make DESIGN=project_obscurus REV=rev2 sim PLUSARGS=+farmonly`, ~90 s)
  gains phases per increment: seeds (two crops planted, divergent growth
  rates), workshop (deposit → craft → RUINED → forced discovery → collect,
  byte-exact mailbox+ring), events (poked LFSR seed forces EV_NEWS; verify
  blight kills + day tick), dual-task respawn after reset.
- Full suite stays the pre-merge gate (~9 min); `+vcd` only when waves needed.
- Bench per increment with `/obs-screenshot`; monitor pokes force rare paths
  (discovery roll, blight) on hardware.
- Makefile: WORKSHOP_MAXLEN=768 cap + workshop.mem staleness guard; tb
  measures blob lengths from readmemh X-scan, never hardcoded (FARMLEN
  lesson).

## Deferred / out of scope

- v2.5: active minigame (fishing/critters) on a third task slot.
- v2.6: SELL screen with per-vendor prices (backend hooks: vendor table as
  SDRAM content; mailbox op shape already generalizes).
- v3: multiple fields (second grid screen); seasons (DAY/30 modulates growth
  and price bases).
- M5 growth tuning remains deferred (user OK with ~60 s/stage).
