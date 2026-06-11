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
