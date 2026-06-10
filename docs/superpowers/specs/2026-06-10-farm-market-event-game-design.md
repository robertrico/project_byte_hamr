# Farm + Market Event Game — Design

**Date:** 2026-06-10
**Status:** Approved design, pre-plan
**Depends on:** C4 async-dispatch kernel (`kernel.S`), SDM driver lib (`SDRAMLIB.S`), CP host lib (`CPLIB.S`), GRVERSE Lo-Res render patterns

## Goal

Build the project's first **event system** — coproc publishes asynchronous events to a ring buffer in SDRAM, the //e consumes them — and prove it with a playable farm/market game. The coproc simulates crop growth and a drifting commodity price in real time, independent of the //e; the //e is a viewport that plants, harvests, and trades.

The event ring and command mailbox protocols are the reusable deliverables. The game is the test fixture that makes them fun.

## Why this shape

- The coproc kernel (C4) runs resident tasks 24/7 regardless of what the //e does. That is the one capability no stock Apple II game has: **the world keeps moving while you look away.** Quit to BASIC, come back, crops grew and the price moved.
- Farm + market exercises both event directions: async world→player notifications (crop ripe, price change) and player→world commands (plant, harvest, sell, buy).
- No gateware changes. Pure Merlin software on the existing `project_obscurus` design.

## Architecture overview

```
coproc (C4 kernel, preemptive)          //e (FARM bin)
┌─────────────────────────┐             ┌──────────────────────┐
│ GAMETASK (skill, slot,  │   SDRAM     │ main loop:           │
│ budget=0 forever loop)  │  ┌───────┐  │  drain event ring    │
│  - consume mailbox      │←─│MAILBOX│←─│  keyboard            │
│  - grow crops (divider) │  ├───────┤  │  send commands       │
│  - drift price (divider)│─→│ RING  │─→│  redraw dirty cells  │
│  - publish events       │  ├───────┤  │                      │
│                         │←→│ STATE │←─│  (state read only at │
└─────────────────────────┘  └───────┘  │   start/resync/cmd)  │
                                        └──────────────────────┘
```

**One coproc task, not two.** The C4 kernel preempts tasks on a timer tick, so two tasks sharing bytes (inventory, ring head) would race on read-modify-write. A single GAMETASK is the sole coproc writer of all game state and the ring — the whole class of races disappears. (Two-task split is a future exercise once a kernel-owned atomic primitive exists; flagged in Out of scope.)

## Memory map — game bank (GBANK)

All game state lives in one SDRAM bank, `GBANK` — default **bank 32** (verify at plan time it is clear of Conway's `MBANK=24`; SDMTEST writes are transient). All offsets below are within GBANK.

```
$0000  SIG      2 bytes  $46,$4D ("FM") — present = world live
$0002  SEQCTR   1 byte   next event sequence number (rolling)
$0003  HEAD     1 byte   ring write index 0-63
$0010  RING     256 bytes  64 records x 4 bytes
$0110  MAILBOX  6 bytes  FLAG, OP, ARG0, ARG1, ARG2, RESULT
$0120  MARKET   7 bytes  PRICEL, PRICEH, SUPPLY, CASHL, CASHH, SEEDS, CROPS
$0200  GRID     400 bytes  20x20 plots, row-major
```

Plot byte: `0` empty, `1` seeded, `2`-`5` growing, `6` ripe.

Coproc reaches GBANK through the SDRAM ports (`SADDRLO/HI $E000`, `SBANKR $E002`, `SDATA $E003`; read side `RADDR/RBANKR/RTRIG/RDATA $E004-$E008`). The //e reaches it through the SDM library (`SDM_SETBANK/SETADDR/READ/WRITE/RDNEXT/WRNEXT`).

## Event ring protocol (reusable)

Record: `SEQ, TYPE, P0, P1` (4 bytes). Record address = `$10 + (index * 4)`, index wraps at 64 (`AND #$3F`).

Event types (v1):

| Type | Name | P0 | P1 |
|---|---|---|---|
| $01 | EV_RIPE | plot x (0-19) | plot y (0-19) |
| $02 | EV_PRICE | price lo | price hi |

**Writer rule (coproc, PUTEV order is normative):** write all 4 record bytes (SEQ = current SEQCTR), then SEQCTR++, then HEAD++ **last**. HEAD is a single byte, so the reader can never see a half-written record behind a published HEAD. SEQCTR therefore always equals the SEQ of the *next* event to be published.

**Reader rule (//e):** keep local TAIL and expected SEQ. While TAIL ≠ HEAD: read record, check `record.SEQ == expected`; optionally re-read the record's SEQ byte after the other 3 bytes and re-check (guards the improbable exact-lap tear where the writer overwrites a record mid-read). Match → dispatch, TAIL++, expected++. Mismatch → the ring lapped us (//e was away > 64 events) → **resync** (see below). Overrun is a designed-for recoverable condition, not an error.

Known theoretical hole, accepted for v1: with an 8-bit SEQ, a lap of exactly 256·k events false-matches. Requires the reader starved 256+ events mid-play; the re-entry path always resyncs, so in practice unreachable.

Single writer (GAMETASK), single reader (//e). No locks needed.

## Command mailbox protocol (reusable)

```
FLAG   0 = empty / task owns nothing; 1 = command pending
OP     command opcode
ARG0-2 arguments
RESULT written by task before clearing FLAG
```

**Handoff discipline (level, not pulse):** the //e writes OP/ARGs first, sets FLAG=1 last. GAMETASK sees FLAG=1, executes, writes RESULT, clears FLAG=0 last. The //e polls FLAG until 0, then reads RESULT. Ownership strictly alternates; safe under preemption with no atomics. One command in flight — sufficient for keypress-driven play. The //e applies a ~1 s timeout counter on the FLAG poll; timeout → "COPROC NOT RESPONDING" on the message line, command abandoned. **After a timeout the coproc may still own the mailbox (slow, not dead): the //e must observe FLAG=0 before writing any subsequent command** — never write OP/ARGs while FLAG=1.

Ops (v1):

| OP | Name | Args | RESULT |
|---|---|---|---|
| $00 | STATUS | — | OK (liveness probe) |
| $01 | PLANT | x, y | OK / ERR_OCCUPIED / ERR_NO_SEEDS |
| $02 | HARVEST | x, y | OK / ERR_NOT_RIPE |
| $03 | SELL | qty | OK / ERR_NO_CROPS |
| $04 | BUYSEED | qty | OK / ERR_NO_CASH |

RESULT codes: `$01` OK; `$E1` ERR_OCCUPIED, `$E2` ERR_NOT_RIPE, `$E3` ERR_NO_SEEDS, `$E4` ERR_NO_CASH, `$E5` ERR_NO_CROPS, `$E6` ERR_BAD_OP.

After any OK result for SELL/BUYSEED/PLANT/HARVEST, the //e re-reads the 7-byte MARKET block to pick up authoritative CASH/SEEDS/CROPS (no EV_SOLD event needed — every market mutation is player-initiated, so the player is present to read the result).

## GAMETASK (coproc, Merlin)

Registered as a skill — default **skill id 3** (GRVERSE uses 2; verify the $0200 table at plan time) — spawned via `CP_CALL` with **budget=0** (run forever), forever loop that never reaches DONE — the LIFE8 multiverse pattern. ORG **$0600** in coproc BRAM: clear of resident LIFE8GR at $0300 (~$2B1 long), below mailboxes at $0F80, ~2.4 KB available. Uses **no zero page** (kernel owns $80-$EF) — absolute BRAM scratch only.

Per loop pass:

1. **Mailbox:** if FLAG=1, dispatch OP, write RESULT, clear FLAG.
2. **Growth divider:** every `GROWDIV` passes, scan 400 plots; stages 1-5 advance by 1; a plot reaching 6 publishes `EV_RIPE x,y`.
3. **Market divider:** every `MKTDIV` passes: `TARGET = max(FLOOR, BASE - SUPPLY/2)`; PRICE steps by 1 toward TARGET; on change publish `EV_PRICE`. Every `DECAYDIV` market ticks, SUPPLY decrements toward 0 (standing demand).

Command semantics: PLANT requires plot=0 and SEEDS>0 (plot=1, SEEDS−−). HARVEST requires plot=6 (plot=0, CROPS++). SELL qty requires CROPS≥qty (CROPS−=qty, CASH+=qty×PRICE clamped at $FFFF, SUPPLY+=qty — add-loop multiply). BUYSEED qty requires CASH≥qty×SEEDCOST (SEEDS+=qty, CASH−=).

**Interrupt discipline:** each individual SDRAM port access sequence is bracketed `SEI`/`CLI` (racetask3 precedent) — one bracket per `SADDR set → SDATA stream` write burst and per `RADDR set → RTRIG → RDATA` read, **never around a whole pass** (the 400-plot growth scan brackets per plot access, or interrupts would be off for the entire scan under the preemptive tick). The port address register is shared hardware — a preempting co-resident task (e.g. LIFE8GR) interleaving port ops would corrupt addressing. v1 assumes GAMETASK is the only port user, but the brackets make co-residency safe and cost nothing.

**Pacing:** no coproc wall clock; dividers count loop passes. Defaults sized for ~2-3 min seed-to-ripe and price moves every few seconds — tuned on bench (M5). All in `FARMEQU.S`.

Event publish (`PUTEV` in `EVLIB.S`, PUT-include): reads SEQCTR + HEAD, writes record, bumps both. Kept as a separate include so future tasks reuse it (under the single-writer-per-ring rule).

## //e program (`FARM.S` → FARM bin)

**Startup:** read SIG via SDM.
- Absent → **cold start:** initialize GBANK (SEQCTR=0, HEAD=0, MARKET defaults: CASH=100, SEEDS=5, CROPS=0, PRICE=BASE, SUPPLY=0; grid zeroed; mailbox FLAG=0), load GAMETASK blob into coproc BRAM via `CP_LADDRLO/HI` + `CP_WDATA`, write its skill-table vector (coproc BRAM $0200+id*2 — distinct address space from the GBANK GRID offset $0200), `CP_CALL` skill id with budget=0, **then write SIG last**. SIG present therefore implies the spawn completed.
- Present → **probable re-entry:** SIG alone is not proof of life (FPGA reflash, other coproc software loaded over GAMETASK, stray bytes). Send `STATUS` (OP $00) probe: OK → resync and resume play (the idle hook). Timeout → world is dead; report it and offer cold start.

**Resync:** re-seed the reader's cursor from a consistent (SEQCTR, HEAD) pair: read SEQCTR, read HEAD, read SEQCTR again; retry until the two SEQCTR reads match (a publish bumps both, so an unchanged SEQCTR brackets a stable HEAD). Then `TAIL = HEAD`, `expected = SEQCTR`. Read MARKET block, read 400-byte grid via `SDM_RDNEXT` streaming, full redraw. 16-bit fields (PRICE, CASH) read twice until consecutive reads match (coproc may write between byte reads). Without the `expected = SEQCTR` re-seed, every post-resync event would mismatch and re-trigger resync forever.

**Screen:** Lo-Res **mixed mode** — GRVERSE GRON switch list with `$C053` (mixed) instead of `$C052`, full undo list retained (80VID, 80STORE, DHIRES, HIRES, PAGE2 off). Clear via line table rows 0-39 only; never blanket-fill $400-$7FF (screen holes are slot-firmware scratch). Grid: 20×20 plots × 2×2 Lo-Res cells = 40×40, exactly the mixed-mode GR area. Low nibble = top row of the cell pair.

Colors (HDMI-converted display; all 16 GR colors solid, no artifact constraints):

| Stage | Color (nibble) |
|---|---|
| 0 empty | black ($0) |
| 1 seeded | brown ($8) |
| 2 sprout | dark green ($4) |
| 3 growing | green ($C) |
| 4 mature | aqua ($E) |
| 5 budding | orange ($9) |
| 6 ripe | yellow ($D) |
| cursor | XOR #$FF on both bytes of its 2×2 block (each byte = two stacked nibbles) |

**HUD (text rows 20-23):** row 20 `CASH/SEEDS/CROPS`; row 21 `PRICE` + trend arrow (sign of last EV_PRICE delta); row 22 message line (results, errors, RIPE pings); row 23 key help.

**Main loop:**
1. Drain ring: TAIL≠HEAD → EV_RIPE recolors plot + message-line ping, EV_PRICE updates HUD. SEQ mismatch → resync.
2. **Periodic grid re-stream** (growth visibility): every ~2-3 s (loop-pass counter), stream the 400-byte grid via `SDM_RDNEXT` into a local shadow, repaint only bytes that changed (dirty-compare). Without this, stages 2-5 would never render during play — grid is otherwise read only at start/resync/command, so a plot would sit brown until EV_RIPE snapped it yellow. ~400 byte reads every few seconds, negligible. EV_RIPE stays for the instant ping.
3. Keyboard: arrows move cursor; `P` plant, `H` harvest, `S` sell 1, `B` buy 1 seed, `Q` quit.
4. Command: write mailbox, FLAG=1, poll with timeout, show RESULT, on OK redraw affected plot + re-read MARKET block, update HUD.

Only dirty cells redraw. Zero-ZP discipline throughout (SDM and CP libs are already zero-ZP; game scratch in main RAM, no ProDOS ZP).

**Quit:** restore text mode, exit to ProDOS. GAMETASK keeps running; world persists until power-off.

## Gameplay (v1 minimal loop)

Plant seeds → crops grow in real time (~2-3 min) → harvest into CROPS counter → sell into a living market. Selling raises SUPPLY which drags PRICE down toward the floor; standing demand decays SUPPLY so PRICE recovers. Strategy = don't dump the harvest; time the market. Buy seeds with proceeds and scale up. Sandbox; CASH is the score. Defaults: CASH 100, SEEDS 5, PRICE base 10, floor 2, seed cost 3 — all `FARMEQU.S` equates, tuned in M5.

## Concurrency invariants (the contract)

1. GAMETASK is the **only coproc writer** of GBANK. The //e writes only MAILBOX OP/ARG/FLAG (and cold-init before the task is spawned).
2. Ring: writer publishes record before HEAD; reader reads HEAD before records; HEAD/SEQ single-byte atomic.
3. Mailbox: FLAG ownership alternates; each side writes its payload before handing FLAG over.
4. Coproc SDRAM port sequences are SEI/CLI-bracketed.
5. //e 16-bit reads from live state: read-twice-compare.
6. **C3/C4 preemption note:** any future second task touching GBANK or the ring violates invariant 1 and needs a kernel-owned atomic or per-task rings.

## Files + build

```
software/SDM/FARMEQU.S    shared equates (GBANK, offsets, ops, events, dividers, prices)
software/SDM/EVLIB.S      PUTEV — coproc-side event publish (PUT-include)
software/SDM/FARMTASK.S   GAMETASK source (PUTs FARMEQU, EVLIB)
software/SDM/FARMTASKB.S  auto-generated DFB blob include — never hand-edited
software/SDM/FARM.S       //e game (PUTs FARMEQU, SDRAMLIB, CPLIB, FARMTASKB)
```

Makefile: `farm` target assembles FARMTASK.S → bin → generates FARMTASKB.S (bin→DFB rule, GRVERSE pattern) → assembles FARM.S. `farmdisk` target builds bootable .po (SDMTEST pattern). Sim build regenerates a divider-shrunk task source via sed (committed source keeps bench dividers — LIFE8GR LSIM mechanism). Merlin source format: single spaces, ASCII, ≤50-char lines.

## Testing + milestones

**M1 — sim green.** iverilog (-g2005) tb, `coproc_c4_tb.v` pattern: load kernel + game blob, host-model drives CP ports (register + CP_CALL budget=0) and SDM ports. Assert, in order: (a) PLANT 3,3 → FLAG clears, RESULT=OK, grid[3,3]=1; (b) run → plot walks 1→6, EV_RIPE in ring, SEQ/HEAD consistent; (c) HARVEST → CROPS=1; SELL → CASH/SUPPLY move, EV_PRICE eventually published; (d) error paths: PLANT occupied → ERR_OCCUPIED, HARVEST unripe → ERR_NOT_RIPE; (e) STATUS probe → OK. Sim build uses tiny dividers.

**M2 — protocol on silicon, no UI.** Register + spawn GAMETASK, then use the project_obscurus $C800 monitor to watch HEAD advance and hand-poke a mailbox command. Isolates protocol from game code.

**M3 — game.** Cold start, plant/harvest/sell on screen, HUD live, events repaint plots.

**M4 — re-entry + resync.** Quit to BASIC, wait, BRUN again → world advanced, clean resync. Force ring overrun (stay away > 64 events) → clean resync.

**M5 — tuning.** Dividers to ~2-3 min growth; price drift feel; starting numbers.

## Out of scope (v1)

- Persistence across power-off (C-flash save/restore exists and is the natural v2 path; session-only for now).
- Multiple crops, water, pests, weather, win condition, market screen, Wa-Tor ecosystem layer.
- Two-task split (FARM/MARKET as separate skills) — blocked on a kernel atomic primitive; see concurrency invariant 6.
- Multi-universe farms (GRVERSE-style channel surfing across 8 farms) — the memory map generalizes (GBANK+u), deliberately not built now.
