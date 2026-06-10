# Coprocessor SDRAM Read Window (design)

**Date:** 2026-06-09
**Branch:** `coproc-sdram-read` (off `main`, which holds C0→C4)
**Board:** Byte Hamr Rev 2, `gateware/rev2/project_obscurus/`
**Status:** approved-in-discussion, pending spec review
**Builds on:** [[project_coproc_c0]] — the coproc + its `$E000-03` SDRAM write window + `sdram_arb`.

## Why this exists

The coprocessor can WRITE SDRAM (the `$E000-03` window) but cannot READ it — its only
SDRAM door is write-only. That makes the coproc a write-only output device: it can post
results to SDRAM but can't process data that lives there. Every "active database" idea (the
coproc running queries / simulations / Life over an SDRAM-resident dataset bigger than its 8 KB
BRAM) needs the coproc to READ the big store. This is the single keystone primitive that turns
the coproc from an output device into a full SDRAM processor. The arbiter already serves coproc
reads (`c1_rdata` exists); the read path is simply unwired on the coproc side.

## Goal

A coproc skill can read any SDRAM byte via an auto-incrementing read pointer: set the pointer
once, then `STA $E007`/`LDA $E008` per byte walks the store. Success = a skill
scan-reads an SDRAM region the host pre-seeded (via SDRAMLIB) and round-trips it (e.g. reads a
cell, increments, writes it back), verified by the host — in sim and on the bench — with the
existing write window and the C2/C3/C4 behavior untouched.

## Architecture

Purely **additive**: a new auto-incrementing READ window beside the proven write window. Both
share the single arbiter `c1` port (the coproc issues one SDRAM op at a time, RDY-stalled, so
there is no contention). The write path (`$E000-03`) is **unchanged** — do not risk the
bench-verified races by touching it.

### Register map (`$E004-07` — free; below C4's `$E010+`, above the write window `$E000-03`)
| Reg | Addr | Dir | Op |
|---|---|---|---|
| `RADDR_LO` | `$E004` | W | read pointer low byte |
| `RADDR_HI` | `$E005` | W | read pointer high byte |
| `RBANK`    | `$E006` | W | read pointer bank byte |
| `RTRIG`    | `$E007` | W | **`STA $E007`** → trigger read of `SDRAM[rptr]`: stall-until-done (the proven write-window stall), latch the byte into `sread`, then `rptr++` |
| `RDATA`    | `$E008` | R | **`LDA $E008`** → returns the latched `sread` (registered DI mux, no stall) |

**Primary design = the two-step** (`STA $E007` trigger, `LDA $E008` result), because **both
halves reuse proven mechanisms**: `STA $E007` is the *exact* write-window stall (an early
advance is benign — no data is being returned on the trigger), and `LDA $E008` is a plain
registered DI-mux read (`is_e008_q ? sread`), identical to `is_count_q`/`is_e014_q`/`is_callreq_q`
which C1–C4 prove align with Arlet's sampling. **Zero new stall timing.** Scan = `STA $E007` /
`LDA $E008` per byte (2 ops/byte). A blocking single-op `LDA $E007` is an *optimization* (1
op/byte) — see the crux: it requires a different, combinational-`rdy` architecture and is NOT
the proven path.

- The read pointer `rptr` is a **flat auto-incrementing address** driven onto the 26-bit
  `phys_addr` as **`{2'b00, rbank, raddr}`** (8+16=24 bits zero-extended to 26 — EXACTLY the
  write window's `phys_addr<={2'b00, sbank, saddr}` form at coproc.v:192; `{rbank,raddr}` alone
  is only 24 bits and mis-maps the port). Auto-increment
  carries `raddr` lo→hi and into `rbank`, so a scan crosses bank boundaries — you can walk a
  contiguous region larger than one bank.
- Usage: write `$E004/05/06` to set the start, then per byte `STA $E007` (trigger+latch+inc) /
  `LDA $E008` (fetch). A scan walks the region with the pointer set once.
- Separate from the write pointer: the read window has its own `raddr`/`rbank`; the write
  window keeps its own `saddr`/`sbank`. A skill can hold a read-scan pointer AND a write-scan
  pointer simultaneously (the Life loop: scan-read the grid, write the next gen elsewhere).

### FSM (extend the existing `$E00x` state machine)
Mirror the write path's stall-until-done, with `we<=0` and a result latch:
- `RADDR_LO/HI`, `RBANK` writes set `raddr[7:0]`/`raddr[15:8]`/`rbank` (gated `& WE & rdy`,
  exactly like `saddr`/`sbank`).
- On `STA $E007` (`is_e007 & WE` in `ST_RUN`): post `req<=1`, `we<=0`,
  `phys_addr<={2'b00, rbank, raddr}`, `rdy<=0` (the **registered** stall — proven-identical to
  the write's `STA $E003`; an early advance on the trigger is benign because no data is being
  returned), enter a read-wait state. (Reuse `ST_WAIT` with a `rd_pending` flag, or add a 4th
  state — `state` is `[1:0]`, slot 3 free.) `req` is pulsed (self-clears via the `req<=0`
  default) — confirm the read-wait→`ST_RUN` return leaves `req` deasserted.
- On `done`: latch `sread<=rdata` (SEE R2 — this is the one new timing), advance the pointer as a
  flat 24-bit counter `{rbank,raddr} <= {rbank,raddr} + 1` (carry `raddr`→`rbank`), then `rdy<=1`
  and return to `ST_RUN`.
- `LDA $E008` returns `sread` via the DI mux: add `is_e008` + a registered `is_e008_q`
  (alongside the existing `is_count_q`/`is_e014_q`/`is_callreq_q`) → `is_e008_q ? sread : ...`.
  This is a plain zero-latency registered read of an already-latched value — the proven pattern,
  no stall.

### Why two-step is race-free, and why blocking-`LDA` is NOT the proven path
The two-step splits the op into two halves that each reuse a mechanism this core already proves:
- **`STA $E007` (trigger):** identical to the write window's `STA $E003` stall. The registered
  `rdy<=0` arriving "one cycle late" is **benign here** — the trigger returns no data, so an
  early advance just means the CPU moves to the next instruction while the SDRAM read runs; the
  result is captured into `sread` regardless. This is exactly why the write path works.
- **`LDA $E008` (result):** a registered DI-mux read of an already-latched value — zero load
  latency, identical to `is_count_q`/`is_e014_q`/`is_callreq_q`, which C1–C4 prove align with
  Arlet's sampling. No stall, no new timing.
So the primitive has **no blocking-read race at all**.

**Why the blocking single-op `LDA $E007` is an optimization, not the default — confirmed at
RTL.** A blocking load needs the CPU to *wait during the data cycle*. Trace it against Arlet:
`DIHOLD <= DI` when `RDY` (arlet_cpu.v:854), `DIMUX = ~RDY ? DIHOLD : DI` (:857), and state/PC
advance only when `RDY` (:865/:349). On the `LDA $E007` data cycle N, `AB=$E007` is registered/
stable; the coproc FSM detects it at `posedge(N)` and does the **registered** `rdy<=0` → `RDY`
drops at N+1. But at `posedge(N)`, the CPU sampled `RDY(N)=1` → it latches `DIHOLD<=DI(N)` (stale
`sread`) and advances PC. **The load already completed with garbage; the stall arrives a cycle
too late and does nothing.** A registered `rdy` cannot stall the cycle that triggers it.
To make blocking work you need a **different `rdy` architecture** — drive `rdy` *combinationally*
off the decode plus a `read_done` latch so it can re-rise while `AB` still holds `$E007`:
`assign rdy = rdy_reg & ~(is_e007 & ~WE & in_read & ~read_done);`. Combinational `rdy` is safe
(not DI → no C0 `AB→DI→AB` delta loop; `AB` is frozen while stalled). But it's a genuinely
different stall path from the proven registered write stall.

**Decision: ship the two-step primitive. Treat blocking-`LDA $E007` as a later optimization,
gated on the combinational-`rdy` + `read_done` rework closing in sim.** Do NOT "build blocking
first" — it rediscovers the confirmed race.

## Data flow
```
skill: STA RADDR_LO/HI + RBANK   (set rptr = start of region)
loop:  STA $E007                 (trigger: read SDRAM[rptr], stall-until-done, latch, rptr++)
       LDA $E008                 (fetch the latched byte -> A)
       ... process the byte ...
       (repeat -> scans the region; pointer auto-advances)
round-trip example: STA $E007 / LDA $E008 (read cell) ; INC ; STA $E000-03 (write back)
```

## Components / files (branch `coproc-sdram-read`)
- `coproc.v` — add `raddr[15:0]`/`rbank[7:0]` registers (set via `$E004/05/06`), the `$E007`
  read TRIGGER (`is_e007 & WE` → `we<=0` post + registered `rdy<=0` stall, mirroring the write's
  `$E003`) + a read-wait state, the `sread<=rdata` latch + pointer auto-inc on `done` (mind R2),
  and the `$E008` result read (`is_e008`/`is_e008_q` → `is_e008_q ? sread`, a plain registered
  DI-mux read). The arbiter port (`req`/`we`/`phys_addr`/`rdata`/`busy`) is already wired — no
  new ports.
- `coproc.v` testbench (or `coproc_sdram_read_tb.v`) — unit-test the read window against a
  stub arbiter/sdram: set rptr, `STA $E007`/`LDA $E008`, assert A == the stub byte + rptr auto-incremented.
- `project_obscurus_top.v` — no change (the coproc's arbiter `c1` wiring already exists).
- `sdram_arb.v` — no change (it already returns `c1_rdata` for `c1_we==0`).
- A skill (e.g. `software/SDM/sdrtest.S` or extend an existing one) — scan-read + round-trip,
  for the integration test / bench.
- Host (e.g. extend an SDM test program) — seed an SDRAM region (SDRAMLIB `SDM_WRITE`), trigger
  the skill, verify the result (SDRAMLIB `SDM_READ`).

## Error handling / edge cases
- **Read during SDRAM not-ready (boot):** the `$E00x` FSM already gates start on `ready`
  (`ST_BOOT` holds `rdy=0` until `ready`) — the read inherits this; a read issued before SDRAM
  init simply waits like the write does. No new deadlock path (the C0 deadlock lesson holds).
- **Pointer wrap:** `{rbank,raddr}` auto-increments as a flat counter; wrapping past the top of
  SDRAM wraps the pointer (benign — a skill scanning a bounded region won't reach it; document
  that the pointer is the skill's responsibility, no bounds-check in hardware).
- **Read vs write same op:** impossible — the coproc is single-threaded and the FSM serves one
  op at a time (a write via `$E003` or a read via `$E007`), each RDY-stalling the core. No
  concurrent read+write on the shared `c1` port.
- **Arbiter priority:** the monitor (`c0`) still outranks the coproc (`c1`); a coproc read can
  wait behind a host monitor access (a few cycles) — the RDY-stall absorbs it transparently.
- **Snapshot (C-flash) interaction:** SDRAM reads are independent of the BRAM snapshot
  machinery; no interaction (C-flash touches BRAM + the config flash, not the coproc's SDRAM
  port). No mask needed.
- **Auto-inc surprise:** a read advances `rptr`; re-reading the same byte requires re-setting
  `RADDR_*`. This is the documented streaming semantic (same as the host load port's `CP_RDATA`).
- **R2 — `sread<=rdata` phase (the ONE genuinely new timing, must-verify):** the result latch
  crosses a **double `busy_d` chain** the write path never exercised. The arbiter has its own
  `busy_d`/`op_complete` and registers `c1_rdata` (sdram_arb.v:36-37,57 — valid the cycle AFTER
  the arbiter's op-complete edge); the coproc has a SECOND `busy_d` on `c1_busy` computing its
  own `done` (coproc.v:176). Those two edge-detects can be off by one, so the cycle `c1_rdata` is
  valid and the cycle coproc-`done` fires may not coincide — latching `sread<=rdata` on `done`
  could grab the byte one cycle early/late (a clean off-by-one that still "looks like data").
  The write window reads `busy`/`done` but NEVER reads `rdata`, so this phase has literally never
  been tested on this core. **Required fix-if-wrong:** latch `sread` one cycle offset from `done`,
  or sample `c1_rdata` directly on the cycle `c1_busy` falls. Pinned by the slow-stub sim gate
  below — this applies to BOTH the two-step and any future blocking variant.

## Testing
- **Sim — read window unit (a loose "`A == seeded byte`" check is NOT enough — pin the phase):**
  1. **Consecutive correctness:** a coproc program sets rptr, does several `STA $E007`/`LDA $E008`
     pairs in a row storing each byte to BRAM; assert the bytes equal *consecutive* pattern bytes
     (catches a broken auto-inc or a wrong-phase latch that repeats/skips).
  2. **Slow-stub PHASE gate (the R2 must-pass):** make the sim SDRAM/arbiter take MANY cycles
     (e.g. 8+) before `c1_busy` falls, and drive `c1_rdata` to a DECOY value early, switching to
     the correct byte only at the true data-valid cycle; assert `sread`/`A` holds the **correct
     post-latency** byte, not the decoy. This pins the `sread<=rdata` ↔ `done` phase (R2) — the
     one genuinely new timing — instead of accidentally passing on an off-by-one. REQUIRED gate.
  3. **Two-pointer independence:** a skill scan-reads via `$E007/$E008` while also writing via
     `$E000-03`; assert the read pointer and write pointer don't interfere (the Life loop shape).
  iverilog `-g2005`.
- **Sim — round-trip:** host seeds an SDRAM cell, a skill reads it (`$E007`), increments, writes
  it back (`$E003`); assert the host reads back the incremented value. Proves read+write compose.
- **Sim — regression:** all prior tests (monitor/C1/C2/C3/C3.1/C-flash/C4) still pass — the
  write window + everything else is untouched.
- **Build:** `make DESIGN=project_obscurus REV=rev2` — DP16KD 8 (read window is a few regs +
  states, no BRAM), timing PASS, no multi-driven nets. The read adds a latch + a state + a
  decode.
- **Bench:** a skill scan-reads a host-seeded SDRAM region and the host verifies the result
  (e.g. the skill sums a region; the host seeds it + checks the sum) → the coproc provably read
  SDRAM on silicon.

## Success criteria
A coproc skill reads SDRAM via `STA $E007`/`LDA $E008` with auto-increment (set-pointer-once, scan), round-
trips through the write window, and the host verifies it — on the bench — with every prior rung
intact. The coproc is now a full SDRAM read/write processor, unlocking SDRAM-resident datasets
(active database, big Life, large-dataset skills).

## Non-goals
- **No auto-increment on the WRITE window** (keep the bench-proven `$E003` path byte-identical;
  a symmetric auto-inc write is a separate future add).
- **No block / strided / DMA transfers** — this is the seed primitive (stride-1 streaming);
  block-read (auto-inc + length) and strided come later.
- **No arbiter change** (it already serves `c1` reads).
- **No multi-pointer beyond one read + one write** (one `raddr`, one `saddr`).
- **No hardware bounds-checking** on the pointer (the skill owns its region).
