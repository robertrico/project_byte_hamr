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

| Region        | Use                                              |
|---------------|--------------------------------------------------|
| $0300-$05FF   | WORKSHOP code (skill 3, 768 B Makefile cap)      |
| $0600-$0CFF   | FARMTASK code (skill 2, cap raised 1536→1792 B)  |
| $0D00-$0DFF   | workshop scratch (temps only; state is in SDRAM) |
| $0E00-$0E2C   | FARMTASK scratch incl. its EVLIB scratch         |
| $0E30-$0F7F   | spare                                            |

The FARMTASK cap raise (FARMTASK_MAXLEN 1536→1792, lands with increment 2)
is the blob-budget relief: FARMTASK.bin is 1299 B today, and seeds
(~150-200 B) plus events (~200 B) do not fit under the old 1536 cap. The
$0C00-$0CFF page was Conway scratch — farm reclaims it for code under the
per-game ownership rule. Workshop scratch needs only EVLIB temps + mailbox/
station temps; 256 B at $0D00 is ample.

**Switching games starts with a kernel reset.** Budget=0 forever-loop tasks
never reach DONE, CP_COLLECT only frees done slots, and there are 4 slots
total — without a reset, one stale game (2 slots) plus farm v2's two spawns
exhausts the table and the next CP_CALL returns $FF. Kernel reset
(CLEARSLOTS) frees everything; each game's world persists in its own
GBANK(s), so this is cold-start-cheap, not data loss.

### Data placement rules

The coproc executes from the 8 KB BRAM only — SDRAM is data-only via the
$E000-$E008 windows. Therefore:

- **Code → blob.** Blobs hold logic only.
- **World state → SDRAM bank** (v1 pattern: grid, market, ring).
- **Static content → SDRAM, //e-seeded at cold start** (recipe table, crop
  profiles, news/headline strings, future vendor tables). Content edits never
  touch a blob. Seeding happens before the SIG write, so the single-writer
  invariant is never violated (same ordering as v1 grid init).
- **Content versioning**: each bank carries a content-version byte CVER at
  $0005, written by the //e at seed time; the FARM binary holds the expected
  value. Valid SIG + mismatched CVER at entry → stale content from an older
  binary → take the quiesce path and re-seed. Without this, "content edits
  never touch a blob" silently runs new code against old tables.
- **Blob-resident data only for hot-loop constants**; optionally copy
  SDRAM→scratch once at task start.

Speed: //e via SDMLIB is a few bus cycles per byte (re-entry already reads the
400 B grid this way); coproc window access is RDY-stalled at ~25 MHz — table
scans are effectively free. SDRAM is plenty fast for everything but per-plot
per-tick inner loops.

**Window-access convention:** every $E000-$E008 burst sits inside an SEI/CLI
pair. FARMTASK already does this via its RDB/WRB helpers; the kernel ISR
touches only the $E010+ control registers, never the data windows, so
SEI-guarded bursts cannot be torn by a context switch. WORKSHOP inherits the
rule. To share the code, RDB/WRB move out of FARMTASK.S into a new
**PORTLIB.S PUT-include** — bank bound by the includer's GBANK symbol, the
same binding trick as EVLIB.

### Single-writer + //e-as-bus

GBANK single-writer invariant holds: FARMTASK owns bank 32, WORKSHOP owns
bank 33. All cross-bank transfers are mediated by the //e via two mailbox
round-trips (e.g. withdraw from farm, deposit to workshop). No task ever
writes another task's bank.

### Host BRAM-write quiesce invariant (new, hardware-mandated)

The coproc BRAM has ONE shared write port (Yosys won't infer DP16KD with two
write ports — coproc.v); the host loader has priority and a same-cycle Arlet
write is **silently dropped**. The RTL's stated safety assumption is "they
never legitimately collide" — true only while no task runs. Streaming a blob
through the loader while another task is alive drops dozens-to-hundreds of
that task's stack/scratch writes per load: crashes on RTS/RTI, silent world
corruption, and sim won't reproduce the collision density.

**Invariant: the host writes coproc BRAM only while no task is RUNNING.**
Consequences:

- There is no single-task respawn. Any dead-task recovery, blob upgrade, or
  content re-seed = **kernel reset → reload BOTH blobs → respawn BOTH tasks →
  resync both rings**. This is the already-proven soft-reset path; worlds
  live in SDRAM, so the only cost is restarting a healthy task.
- Rejected alternative: RTL fix (stall Arlet for the loader-write cycle) —
  gateware change + sim proof for something the software invariant gets free.
- v1 carries a latent low-probability version (tick ISR pushes idle-stack
  frames during LOADBLOB); the sim collision assertion below covers both.

## Components

### WORKSHOP task (new, skill id 3)

- ORG $0300, cap 768 B, budget=0 forever loop, sole writer of GBANK 33.
- Reuses EVLIB (PUTEV) and the FARMEQU pattern via a new WORKEQU.S PUT-include.
- Loop: poll mailbox → handle op → tick station timers → emit ring events.

**EVLIB scratch parameterization (required — cross-task race otherwise).**
EVLIB.S currently hard-codes its scratch at $0E00-$0E05. The kernel is
preemptive (tasks run I-clear), and PUTEV has interrupt-enabled windows:
callers store EVTYPE/EVP0/EVP1 before the JSR, and PUTEV itself drops to CLI
between its read burst and write burst while EVSEQ/EVHEAD/EVRLO are live. Two
tasks sharing those six bytes corrupt each other's ring publishes
intermittently. Fix: move the scratch equates OUT of EVLIB.S into the
includer — FARMEQU keeps $0E00-$0E05; WORKEQU defines its own at $0D00-$0D05.
EVLIB binds by symbol name, so WORKEQU must also define GBANK/FSEQC/FHEAD/
FRING (same F-prefixed names) with workshop values (33/$0002/$0003/$0100).
Zero code change to EVLIB; each blob compiles its own correctly-bound PUTEV.

GBANK 33 layout (signature "WK" = $57,$4B):

| Addr        | Content                                              |
|-------------|------------------------------------------------------|
| $0000-$0001 | SIG $57,$4B ("WK")                                   |
| $0002       | SEQCTR                                               |
| $0003       | HEAD                                                 |
| $0004       | heartbeat (increments every loop pass)               |
| $0005       | CVER content version (//e-seeded)                    |
| $0100-$01FF | RING 64x4, page-aligned (EVLIB verbatim)             |
| $0200-$0207 | MAILBOX, 8 B: FLAG, OP, I0-I3, RES, RES1             |
| $0210+      | pantry: 4 per-crop ingredient counts; goods          |
|             | inventory; 2 station records (recipe, timer, state); |
|             | CRAFT SKILL byte                                     |
| $0240       | discovered-recipe bitmap                             |
| $0300+      | recipe table (//e-seeded static content)             |

Mailbox ops: OPDEPOSIT(crop, qty), OPCRAFT(I0-I3), OPCOLLECT(→ RES =
product id, RES1 = value), OPMODE(flags) — event modifiers, currently bit 0
= BOOM (craft values x2). No OPSTAT: liveness is the heartbeat byte, state
is read directly from the bank (grid-resync precedent). Ring events:
WEVDONE(station, product); heartbeat is polled, not evented.

**Loop-pass timing note:** GROWD/MKTD and station dividers count task loop
passes. Round-robin with a second budget=0 task roughly halves each task's
pass rate — growth/market run ~2x slower wall-clock once WORKSHOP spawns
(increment 3), and drift with workshop load thereafter. Accepted: dividers
get retuned at increment 3 (the deferred M5 tuning pass lands there). Sim
FSIM dividers and the ~90 s farmonly figure stretch the same way.

**Cross-bank transfer rule:** //e-mediated two-phase transfers
(OPWITHDRAW→OPDEPOSIT, OPCOLLECT→OPADDCASH) can lose goods if a reset lands
between legs. Accepted: world is session-only and stakes are in-game cash.
Ordering rule is normative — **debit first, credit second** — so a crash
loses value but can never duplicate it.

### Crafting / recipe model

- Recipe record (8 B): 4 ingredient slots (crop ids, $FF = empty, sorted
  canonical; a repeated id means "needs 2 of it") + product id + craft-time
  class + value + flags (rarity tier → harder discovery roll).
- ~12-16 real recipes over 4 crops; combo space (2-4 elements with repetition)
  is 65, so most combos are duds. Table lives in GBANK 33 SDRAM; blob scans it
  through the read window on each OPCRAFT.
- Discovery: undiscovered + correct combo → LFSR roll vs (BASE + craft skill
  − rarity), threshold floored at 0 — a high-rarity recipe with low skill is
  simply impossible until skill grows (intended). Pass = bitmap bit set +
  product crafted. Fail = "RUINED", ingredients lost. **Every attempt
  increments craft skill** (clamped at $FF — no wrap), success, fail, or dud
  combo. Discovered recipes always succeed.
- Stations: 2 concurrent crafts; seconds-scale divider per tick decrements
  active station timers; WEVDONE on completion. Crafts keep cooking while the
  player is on another screen, quit, or after a soft reset (task respawn).
- Cash-out: //e OPCOLLECT (workshop) → OPADDCASH (farm task, new op). One
  wallet, lives in farm bank as today.
- Product value > ingredient cost x1.5-2.5; BOOM event doubles craft margins.
  Recipe value is one byte; base values stay <128 so the BOOM x2 still fits a
  byte, and OPADDCASH credits in 16-bit on the farm side.

### Seeds (FARMTASK changes)

- Plot byte = `crop*8 + stage`: crop = byte>>3 (0-3), stage = byte&7
  (0 empty [global $00], 1-5 grow, 6 ripe, 7 dead). Shift/mask arithmetic, no
  lookups. Old saves incompatible — acceptable, world is session-only.
  Edge rule: $08/$10/$18 (crop≠0, stage 0) are illegal — harvest/clear always
  writes $00, and all logic treats stage==0 as empty regardless of crop bits.
- 4 crop profiles (all data, SDRAM where possible):

| Crop    | Seed cost | Growth  | Price base | Role                          |
|---------|-----------|---------|------------|-------------------------------|
| WHEAT   | cheap     | fast    | low        | volume + craft staple         |
| CARROT  | mid       | mid     | mid        | balanced                      |
| BERRY   | dear      | slow    | high       | margin play                   |
| PUMPKIN | dearest   | slowest | highest    | craft-heavy, event-sensitive  |

- Growth: single global GROWD tick retained; per-crop speed = advance every
  Nth tick via a 4-entry mask table. Masks are copied SDRAM→scratch at task
  spawn (hot inner loop stays off the window); the DROUGHT handler mutates
  the scratch copy and restores it on expiry.
- Farm bank (GBANK 32) v2 layout — the v1 $0210-$0216 scalar block is
  REPLACED (collision otherwise: FCASHL $0213 sits inside the new market
  array):

| Addr        | Content                                  |
|-------------|------------------------------------------|
| $0210-$021B | MARKET: 4 x (PRICE lo, PRICE hi, SUPPLY) |
| $0220-$0221 | FCASH lo/hi                              |
| $0222-$0225 | FSEEDS[4]                                |
| $0226-$0229 | FCROPS[4]                                |

  Per-crop drift; state already in SDRAM, no blob data cost. //e HUD readers
  and cold-start init update to the new map in the same increment. Farm bank
  also gains FHBEAT $0004 and CVER $0005 (free today).
- EVPRICE payload redefined for 4 crops: **P0 = crop id, P1 = price lo**,
  with prices capped <256 by the drift logic (v1 base 10/floor 2 never
  approached it; the cap is now normative). Market-screen trend arrows
  compare against the //e's last-seen price per crop.
- Saturation rules (silent-wrap bugs otherwise): cash credits saturate at
  $FFFF; SUPPLY += qty saturates at $FF.
- Mailbox: OPPLANT(x, y, crop), OPSELL(crop, qty), OPBUY(crop, qty); new
  OPWITHDRAW(crop, qty) and OPADDCASH(value lo/hi) for the //e bus. Note
  OPPLANT now uses all three arg bytes — the farm mailbox is arg-full; any
  future op needing more args must widen the mailbox (bank layout change).
- Sell/buy quantity prompt is **2 digits (1-99)** — qty must fit one mailbox
  arg byte; 1-99 keeps the prompt simple, repeat the op for more. The prompt
  is modal: digits/RETURN/ESC only; M/W/ESC navigation suspends until it
  closes.
- Render: per-crop GR base color, brightness/pattern by stage, ripe bright,
  dead magenta ($1). Exact colors tuned at plan time with bench screenshots.
- Blob budget: FARMTASK.bin is 1299 B; under the raised 1792 B cap that is
  493 B for seeds (~150-200 B, indexed loops replace scalars) plus events
  (~200 B), with margin. The cap raise ships with increment 2.

### //e screen manager

- Screen id byte + per-screen jump tables: render-full, key-handler,
  event-hook.
- Main loop unchanged in shape: poll ring(s) → dispatch each event to state
  updaters (always) → current screen's event-hook decides redraw; poll
  keyboard → current screen's key-handler.
- Screens: FARM (GR mixed, the 20x20 grid), MARKET (text 40x24: 4-crop price
  board with trends, buy seeds, sell crops with 2-digit quantity prompt),
  WORKSHOP (text: pantry, 2 stations with timers, recipe book — known recipes
  named, unknown as "?????", combo picker). SELL/vendor screen reserved v2.6.
- Farm-screen text rows (GR mixed gives rows 20-23) are assigned now so
  increments 2 and 4 don't fight: row 20 = status + CASH + DAY nn; row 21 =
  messages; row 22 = news headline; row 23 = key legend. (Market/seed detail
  rows leave the farm screen — that data lives on the market screen.)
- Keys: global M = market, W = workshop, ESC = back to farm; Q quits from
  farm screen (confirm). Farm legend becomes PLANT HARVEST MARKET WORKSHOP
  QUIT (inverse first letters). **S is removed; selling lives on the market
  screen only.**
- Screen switch = soft-switch GR/TEXT + full repaint from local state; the
  farm repaint reuses the existing re-entry resync renderer.
- News/status line pinned on every screen.
- Re-entry/respawn: probe BOTH SIGs + heartbeats. All-alive → resync two
  rings (two expected-SEQ counters) and play. ANY task dead, blob stale, or
  content-version mismatch → the quiesce path: kernel reset, reload both
  blobs, re-seed content if versions differ, respawn both, resync both. No
  single-task respawn (see quiesce invariant).

### World events (FARMTASK)

- Day counter in farm bank, ticks every N market ticks → EV_DAY; every screen
  shows DAY nn. Event ids: EV_DAY = 3, EV_NEWS = 4 (EVRIPE = 1, EVPRICE = 2
  taken).
- EV_NEWS: LFSR roll per day → one of CALM (default), DROUGHT (growth masks
  x2 slower, crop prices climb), BOOM (craft values x2 — //e forwards
  OPMODE(BOOM) to the workshop task; bus pattern, no cross-bank read), BLIGHT
  (random growing/ripe plots → stage 7 dead; per-crop susceptibility).
- One active event at a time; state = (event id, days left) in farm bank.
  **Expiry is evented**: when the active event's days-left hits zero,
  FARMTASK emits EV_NEWS(CALM); the //e forwards OPMODE(clear) to the
  workshop on seeing it. No //e polling of event state.
  Headline strings are SDRAM content, //e-seeded.
- Dead plots cleared by replant only (P on a dead plot = clear + plant).
- Intended dynamics: drought → sell the stockpile; blight → seed variety as
  insurance; boom → workshop margin play.

### Heartbeat

Both tasks increment byte $0004 of their own bank every loop pass (equates
FHBEAT/WHBEAT = $0004). //e (or the monitor) diagnoses wedged-vs-live by
reading **twice with a delay** — the byte wraps every 256 passes at coproc
speed, so two immediate reads can alias equal on a live task. Ships in
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
  blight kills + day tick), dual-task quiesce-recovery after reset (kernel
  reset → both reload → both respawn → both rings resync).
- tb assertion: host loader write (`b_wr_ok`) coincident with an Arlet BRAM
  write (`a_wr_ok`) = test failure. Guards the quiesce invariant forever and
  catches v1's latent idle-stack-during-LOADBLOB case.
- Full suite stays the pre-merge gate (~9 min); `+vcd` only when waves needed.
- Bench per increment with `/obs-screenshot`; monitor pokes force rare paths
  (discovery roll, blight) on hardware. A monitor poke is a second writer —
  poke only while the owning task is quiesced (dead, or between ticks with
  care), same caveat the tb carries.
- WORKEQU.S gets a `DO FSIM` tiny-divider block for station timers, mirroring
  FARMEQU's, or the workshop sim phase takes minutes instead of seconds.
- Makefile: WORKSHOP_MAXLEN=768 cap + workshop.mem staleness guard;
  FARMTASK_MAXLEN 1536→1792 with increment 2; tb measures blob lengths from
  readmemh X-scan, never hardcoded (FARMLEN lesson).

## Deferred / out of scope

- v2.5: active minigame (fishing/critters) on a third task slot.
- v2.6: SELL screen with per-vendor prices (backend hooks: vendor table as
  SDRAM content; mailbox op shape already generalizes).
- v3: multiple fields (second grid screen); seasons (DAY/30 modulates growth
  and price bases).
- M5 growth tuning remains deferred (user OK with ~60 s/stage).
