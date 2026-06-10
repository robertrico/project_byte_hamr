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
once, then `LDA $E007` repeatedly walks the store one instruction per byte. Success = a skill
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
| `RDATA`    | `$E007` | R | **`LDA $E007`** → blocking read of `SDRAM[rptr]`; returns the byte; then `rptr++` (auto-increment) |

- The read pointer `rptr` is a **flat auto-incrementing address** driven onto the 26-bit
  `phys_addr` as **`{2'b00, rbank, raddr}`** (8+16=24 bits zero-extended to 26 — EXACTLY the
  write window's `phys_addr<={2'b00, sbank, saddr}` form at coproc.v:192; `{rbank,raddr}` alone
  is only 24 bits and mis-maps the port). Auto-increment
  carries `raddr` lo→hi and into `rbank`, so a scan crosses bank boundaries — you can walk a
  contiguous region larger than one bank.
- Usage: write `$E004/05/06` to set the start, then `LDA $E007` repeatedly to scan — **one
  instruction per byte.**
- Separate from the write pointer: the read window has its own `raddr`/`rbank`; the write
  window keeps its own `saddr`/`sbank`. A skill can hold a read-scan pointer AND a write-scan
  pointer simultaneously (the Life loop: scan-read the grid, write the next gen elsewhere).

### FSM (extend the existing `$E00x` state machine)
Mirror the write path's stall-until-done, with `we<=0` and a result latch:
- `RADDR_LO/HI`, `RBANK` writes set `raddr[7:0]`/`raddr[15:8]`/`rbank` (gated `& WE & rdy`,
  exactly like `saddr`/`sbank`).
- On `LDA $E007` (`is_e007 & ~WE` in `ST_RUN`): post `req<=1`, `we<=0`,
  `phys_addr<={2'b00, rbank, raddr}`, drive `rdy` combinationally low (see the crux below),
  enter a read-wait state. (Reuse `ST_WAIT` with a `rd_pending` flag, or add a 4th state —
  `state` is `[1:0]` and slot 3 is free.) `req` is pulsed (self-clears via the `req<=0` default)
  — confirm the read-wait→`ST_RUN` return leaves `req` deasserted.
- On `done` (the existing `busy_d & ~busy` edge): latch `sread<=rdata`, advance the pointer as a
  flat 24-bit counter `{rbank,raddr} <= {rbank,raddr} + 1` (carry `raddr`→`rbank`), then release
  `rdy` high and return to `ST_RUN`.
- The DI mux returns `sread` for the `$E007` read: add `is_e007` + a registered `is_e007_q`
  (alongside the existing `is_count_q`/`is_e014_q`/`is_callreq_q`) → `is_e007_q ? sread : ...`.

### The crux — the registered-`rdy` race (this is NEW silicon, not a solved lesson)
`LDA $E007` is a **blocking read**, and this is genuinely new behavior for this core: **no
existing `$E0xx` read stalls** — `is_count_q`/`is_e014_q`/`is_callreq_q` all read a value
that's *always present* (zero load latency). The write window RDY-stalls, but a write that
advances one cycle early is harmless (the data `DO` was already captured, the op still posts).
A *read* that advances early lands the WRONG byte in A. So the C0 "register the DI" lesson does
NOT de-risk this — C0 was 1-cycle synchronous RAM; this is a multi-cycle RDY-stall on a read.

**The hazard (must-fix):** `rdy` is a REGISTERED reg, set only at `posedge clk`. On the `LDA
$E007` data cycle N: `is_e007` is combinational-high in cycle N; the FSM sees it at `posedge(N)`
and sets `rdy<=0` → `rdy` is low only in cycle N+1. But at that *same* `posedge(N)`, Arlet
samples `DI(N)` and `RDY(N)` — and `RDY(N)` is still 1. So Arlet latches the stale `sread` and
advances. **The read never waits.** A registered `rdy` drop is one cycle too late.

**The fix (mandated for the read path):** drive `rdy` **combinationally** low on the read
access — `rdy_out = rdy_reg & ~(is_e007 & ~WE & (state==ST_RUN) & ~read_done)`. This engages the
stall in cycle N itself, so Arlet sees `RDY(N)=0` and holds. Combinational `rdy` is SAFE (it is
NOT DI, so it can't form the C0 `AB→DI→AB` delta loop; and while `RDY=0` Arlet is frozen so `AB`
is held → `is_e007` is stable → `rdy_out` is stable — no oscillation). When the read completes,
latch `sread`, present it via the DI mux, auto-inc the pointer, and release `rdy_out` high; Arlet
resumes and samples the correct byte.

**Fallback if combinational-`rdy` still won't close:** the two-step form — `STA $E007` triggers
the read (the write-style stall: an early advance is benign, data isn't being returned yet) +
latch + auto-inc, and a plain registered `LDA $E008` returns the already-latched `sread`. 2
ops/byte, no stall-timing fight. **Build combinational-`rdy` blocking first; fall back to
two-step only if it can't be closed in sim.**

## Data flow
```
skill: STA RADDR_LO/HI + RBANK   (set rptr = start of region)
loop:  LDA $E007                 (blocking: read SDRAM[rptr] -> A, rptr++)
       ... process the byte ...
       (repeat -> scans the region one instruction per byte)
round-trip example: LDA $E007 (read cell) ; INC ; STA $E000-03 (write back)
```

## Components / files (branch `coproc-sdram-read`)
- `coproc.v` — add `RADDR_LO/HI`/`RBANK` registers (`raddr[15:0]`, `rbank[7:0]`), the `$E007`
  read trigger + read-wait state in the `$E00x` FSM (mirror the write post with `we<=0`),
  `sread` latch + pointer auto-increment on `done`, and the `is_e007`/`is_e007_q` DI-mux read.
  The arbiter port (`req`/`we`/`phys_addr`/`rdata`/`busy`) is already wired — no new ports.
- `coproc.v` testbench (or `coproc_sdram_read_tb.v`) — unit-test the read window against a
  stub arbiter/sdram: set rptr, `LDA $E007`, assert A == the stub byte + rptr auto-incremented.
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

## Testing
- **Sim — read window unit (must defeat the early-advance race, not just match a byte):** a
  loose "`A == seeded byte`" check can PASS on a wrong-but-plausible early sample — so assert the
  stall actually happened:
  1. **Consecutive correctness:** a coproc program sets rptr, does several `LDA $E007` in a row
     storing each to BRAM; assert the bytes equal *consecutive* pattern bytes (a stale/early
     sample would repeat or skip — catches a failed stall + the auto-inc).
  2. **Deliberately-slow stub:** make the sim SDRAM/arbiter take MANY cycles (e.g. 8+) before
     `busy` falls, with `rdata` driven to a DIFFERENT value early then the correct value at
     `done`; assert `A` holds the **post-stall** value (proves the CPU waited, didn't latch the
     early `rdata`). This is the assertion that actually validates the combinational-`rdy` fix.
  3. **rdata-stable-at-`done`:** assert `rdata` is stable on the exact `done` edge where `sread`
     is latched (the write path never sampled `rdata`, so this is an unproven assumption — make
     it explicit). iverilog `-g2005`.
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
A coproc skill reads SDRAM via `LDA $E007` with auto-increment (set-pointer-once, scan), round-
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
