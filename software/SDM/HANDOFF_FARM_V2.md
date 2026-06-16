# HANDOFF — Farm v2: Playability + Sidework (read this first in new sessions)

## Where we are (2026-06-11, all pushed to main @ a0be44e)

The farm/market event game is **shipped and bench-verified** on the project_obscurus
C4 coproc. Everything below is sim-tested (38+ PASS suite) AND played on real
hardware:

- **Event ring + command mailbox protocols** (the reusable deliverables) —
  spec: `docs/superpowers/specs/2026-06-10-farm-market-event-game-design.md`
- **GAMETASK** (coproc skill 2, blob ~1300 B): growth sim, market drift,
  rare-yield LFSR RNG (0.4% crop death / 1.6% double / 0.8% triple)
- **FARM** (//e, BIN $6000): GR mixed 20×20 grid, event-driven HUD,
  re-entry resync, **soft-reset survival** (kernel reset → auto task respawn,
  world preserved — also the blob-upgrade path: soft reset reloads from disk)
- Memory file `project_farm_event_game.md` has every address/gotcha. Plans in
  `docs/superpowers/plans/2026-06-10-farm-*.md`.
- **v2 increment 1 SHIPPED (branch farm-v2)**: heartbeat (FHBEAT $0004,
  task increments every pass; HBV scratch $0E2D, blob now 1312 B) +
  sell-qty modal prompt (S -> "SELL QTY:" row 20, 2 digits, RETURN sends
  OPSELL qty, ESC/0 cancels). Spec (READ before increment 2):
  `docs/superpowers/specs/2026-06-11-farm-v2-sidework-design.md` — 3 review
  rounds: quiesce invariant, EVLIB scratch param, farm-bank v2 map. Plan:
  `docs/superpowers/plans/2026-06-11-farm-v2-inc1-sellqty-heartbeat.md`.
  Next: increment 2 = screen manager + market screen + seeds
  (FARMTASK_MAXLEN raise to 1792 lands there).
- **v2 increment 2 SHIPPED (branch farm-v2)**: 4 crops (WHEAT/CARROT/BERRY/
  PUMPKN, plot byte = crop*8+stage, mask growth off GTICK, per-crop market
  $0210+c*3, cash $0220, SEEDS[4] $0222, CROPS[4] $0226, EVPRICE=(crop,price),
  OPPLANT +crop / OPSELL/OPBUY=(crop,qty), FARMTASK_MAXLEN now 1792, blob
  ~1617 B / 175 B headroom - see inc-4 note) + //e screen manager (M=market
  text screen, ESC=farm, 1-4 select seed, B/S qty prompts on market; EVPRICE
  repaints one row, no flash; farm rows: 20 seed+cash, 21 messages, 23
  legend). COLD START REQUIRED after deploy (bank map moved). Inc-4 budget
  note: ~37 B reclaimable in FARMTASK via LDY #>page immediates (per-crop
  addr hi bytes are constant $02 - reviewer-verified). Next: increment 3 =
  WORKSHOP task (read spec quiesce invariant + EVLIB scratch param +
  CP_FSTAT restore_done rule FIRST).
- **v2 increment 3 SHIPPED (branch farm-v2)**: WORKSHOP task — skill 3,
  bank 33 ("WK"), blob 1240 B ORG $2000 (BRAM GREW 8->16 KB this increment:
  task space $2000-$3FFF, NOT in cflash snapshots; NEEDS FPGA REFLASH).
  12 recipes (values <=127), discovery roll < 40+SKILL/2-RARITY*16 (floor 0:
  FEAST impossible at low skill), 2 stations, WEVDONE=5, RRUIN=$E8.
  Farm-side OPWDRAW=5/OPADDC=6 (blob 1772/1792). PORTLIB.S + EVLIB scratch
  param (byte-identical refactor). CVER=1 @ $0005 both banks. //e: W =
  workshop screen, MIX 1-4 + RETURN craft, D deposit (debit-first), C 1/2
  collect->OPADDC. QUIESCE rules now load-everything-stage-everything-ring-
  last in tb AND FARM.S (CP_CALL's stage+ring split — it staged mailboxes
  through the BRAM port after ring 0). Deploy: FLASH new bitstream, then
  boot + BRUN (farm world survives if SDRAM kept power; else cold start).
  Next: increment 4 = world events (FARMTASK has 20 B headroom — use the
  37 B hi-byte reclaim first).
- **v2 recipe-shop SHIPPED (branch farm-v2)**: dual-road crafting -
  market R buys lowest-unowned recipe (price=2xvalue 56..254, debit-first
  OPSPEND=7 + refund-on-WOPLEARN-reject via OPADDC, guarded SENDCMD);
  discovery gamble unchanged (RUINED on fail); KNOWN recipes now roll a
  fail curve (FAILBASE[r]=48/80/112/144 - skill/2, floor 8, RFAIL=$E9
  'CRAFT FAILED', pinned to WKNOWN branch NOT shared WCGO). New ops
  WOPLEARN=5 (workshop, double-learn guard). CVER=2 (re-seeds bank 33 on
  deploy: pantry/skill/recipes reset). PORTLIB extract done inc-3.
  FARMTASK 1780/1792 (MKADR+4-site addr-hi reclaim + WRCASH helper);
  **only ~10 B headroom - inc-4 world events MUST find another mechanism
  (overflow PUT-include, host-side, or 2nd op-handler region) - see
  RS T3 quality review**. Polish: deposit DEP 1-4? crop-picker (QPCROP,
  kills hidden-SELCROP bug), farm-screen workshop widget (W:.. station
  glyphs + done BELL). Deploy: ctrl-reset + BRUN, farm world (bank 32)
  survives, workshop (bank 33) re-seeds. Next: increment 4 = world events.
- **v2 goods-inventory SHIPPED (branch farm-v2)**: PANTRY/DEPOSIT DELETED -
  craft pulls crops straight from the farm (//e prechecks a free STATION
  via STREC then OPWITHDRAW-debits crops, then OPCRAFT cooks w/o consuming;
  station-precheck-before-debit avoids RERRFULL crop-loss). Finished crafts
  AUTO-STORE to GOODS[12] (bank 33 $0228), station auto-idles (no STATE 2,
  no collect), WEVDONE still fires. WOPCOLL slot ($03) repurposed to SELL:
  WOPSELL returns PER-UNIT value in WRES1 (WMFIN owns WRES; mailbox too
  narrow for a 16-bit total), //e multiplies by qty + OPADDC. New paged,
  SCROLLING INVENTORY screen (key I, PREVSCR return-to-origin, VROWS=16):
  SEEDS(read-only) / CROPS(sell @ market price OPSELL) / GOODS(sell @ value
  WOPSELL); TAB pages, up/dn scroll - N-scalable for future multi-plot.
  CVER=3 SELECTIVE migration (MIGRATE33): preserves DISC/SKILL/recipes,
  zeros only GOODS + retired pantry. RVALUE table in FARM.S is goods-price
  DISPLAY only (MUST track RECTAB+5 = the cash authority). Deploy:
  ctrl-reset + BRUN. **//e FARM.bin now ~12.56 KB of ~13.8 KB ceiling
  (~1.3 KB headroom) - inc-4 //e additions risk overflow; consider an
  EXEC/screen split before large new //e features.** Minor deferred:
  INVDRAW calls INVCLRBOT then CLRROW20 (row 20 cleared twice, harmless);
  inventory scroll machinery unexercised until a page exceeds 16 items.
  Tb migrated to no-pantry model (dropped deposit/collect/pantry asserts).
  Next: increment 4 = world events.
- **Full-suite gate bug found+fixed (was pre-existing on main)**: after any
  reset, cflash boot-restore owns coproc BRAM port B until restore_done —
  host loads during it are silently dropped, and a valid flash snapshot
  replays stale TABLE/code over $0200-$0FFF. The suite's cflash phases left
  such a snapshot; the farm reset phase then spawned a stale $0340 stub
  instead of FARMTASK ("FLAG never cleared", farmonly green / full suite
  red). HOST RULE: poll CP_FSTAT ($C0CF) bit 1 (restore_done) after reset
  before any BRAM load. tb farm_reset now waits; FARM.S quiesce reload
  (increment 3) must do the same. //e exposure today ~nil (ProDOS reboot
  seconds >> restore ms — why bench never saw it).

## Working facts (cost of ignoring these: hours)

- Build: `make farm` (does NOT depend on FARM.S — run explicitly), `make sdmdisk`.
- Sim: `make DESIGN=project_obscurus REV=rev2 sim PLUSARGS=+farmonly` = **90 s**;
  full suite (pre-merge gate) ~9 min; `+vcd` only when waves needed (20 GB!).
- Blob bound: code must end < $0C00 (1536 B, Makefile-enforced; scratch at $0E00+,
  LIFE8GR owns $0C00-$0D67, mailboxes $0F80). ~240 B headroom currently.
- Coproc kernel has **4 task slots**; farm uses one. ZP belongs to the kernel.
- Deploy: //e-only changes → just BRUN. Blob changes → soft reset (respawn
  reloads blob) or power-cycle. World is session-only (SDRAM).
- GBANK=32 single-writer invariant: ONE task owns a bank's state. A second
  game/task gets its OWN bank + own ring/mailbox (protocols are PUT-includes:
  FARMEQU/EVLIB patterns copy cleanly).
- Bench: user runs the //e; `/obs-screenshot` skill captures the screen — use it.
- HUD budget: 4 text rows (20-23); row 20 cols 0-28 = messages, controls row
  freed 10 cols by inverse-key legend. GR rows 0-39 = the 20×20 plot grid.
- Process that works here: brainstorm → short spec → plan w/ full code →
  subagent execute → farmonly sim → bench with screenshots. Reviews catch real
  bugs; the 90 s sim loop catches them faster.

## The v2 brief (user's words, distilled)

**Goal: investigate playability. Build a fun SIDEWORK loop — something to do
while crops grow.** The farm's plant→wait→harvest→sell cadence is solid but
"sell... wait... sell... wait" feels bad. Requirements:

1. **Brainstorm sidework ideas** — each pitch MUST include a quick
   "here's how we get there" grounded in the existing infrastructure
   (ring/mailbox/extra task slots/LFSR/GR grid/monitor). Simple to learn,
   complex enough to hold engagement — same bar the base farm cleared.
2. **Money velocity** — earning needs to feel better. Explicit asks:
   - Sell UX: type a quantity at the current price point (vs S-mashing 1×).
   - Something productive in the interim: **crafting / experimenting**.
   - **Seed variety** (different crops: cost/growth/price profiles).
3. **World events layer on top** — drought/boom/blight headlines, day counter,
   dead-plot red state (magenta $1) — already sketched in
   `project_farm_event_game.md` v2 wishlist + this is ON TOP of the sidework
   experience, not instead of it.
4. Heartbeat byte (task increments a GBANK counter) — cheap diagnosability,
   fold into whatever ships first.

## Seed ideas to start the brainstorm (not decisions)

- **Second task slot = second machine**: a sidework minigame as its OWN coproc
  task + bank (market speculation/futures? fishing the Wa-Tor pond? a critter
  ecosystem you manage between harvests?). The protocols make this ~copy-paste.
- **Crafting**: combine N crops → goods worth more than N×price (bread, cider),
  with timed coproc "processing" (another use of the wait loop — but one YOU
  schedule). Recipes discovered by experimenting = engagement.
- **Sell-quantity UX**: S prompts for a 1-3 digit quantity on the status row
  (keyboard digits, ESC cancels) — pure //e change, ships in an afternoon.
- **Seed variety**: plot byte has headroom (values 8+ free; 7 reserved for dead);
  per-crop growth dividers + price columns. Pairs naturally with crafting inputs.
- World events make all of the above swing: drought spikes crop prices (sell
  the stockpile!), boom doubles craft margins, blight makes variety = insurance.

## Open items from this session

- v2 world events: spec not yet written (wishlist is in memory + above).
- M5 growth tuning skipped on purpose (user OK with ~60 s/stage for now).
- Zombie-slot residual documented in spec (4 leaked cold starts → CP_CALL $FF).
- `farm-game` branch merged; deletable.
