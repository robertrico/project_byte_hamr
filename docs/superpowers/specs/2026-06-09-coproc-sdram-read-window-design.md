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

- The read pointer `rptr` is a **flat auto-incrementing address** formed as `{rbank, raddr}`
  (the same `{sbank, saddr}` mapping the write window uses for `phys_addr`). Auto-increment
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
  `phys_addr<={rbank, raddr}`, `rdy<=0` (stall the core), enter a read-wait state.
- On `done` (the existing `busy_d & ~busy` edge): latch `sread<=rdata`, advance the pointer
  `{rbank,raddr} <= {rbank,raddr} + 1`, then `rdy<=1` and return to `ST_RUN`.
- The DI mux returns `sread` for the `$E007` read: add `is_e007` + a registered `is_e007_q`
  (alongside the existing `is_count_q`/`is_e014_q`/`is_callreq_q`) → `is_e007_q ? sread : ...`.

### The one care-point — Arlet registered DI (the C0 lesson)
`LDA $E007` is a **blocking read**: the CPU stalls mid-instruction (RDY low) until the SDRAM
read completes, then samples DI when RDY rises. The latched `sread` must be presented as DI at
the cycle the core resumes. This is the same "Arlet wants SYNCHRONOUS 1-cycle-latency memory;
register the DI select" discipline that C0 solved (a combinational `DI=bram[AB]` boots wrong
and can form a delta loop). Get the `sread`→DI presentation aligned with the RDY-rise; verify
in sim (the value the `LDA` lands in A must equal the seeded SDRAM byte).
**Fallback if the blocking read fights the DI timing:** a two-step form — `STA $E007` triggers
the read (stall-until-done + latch + auto-inc, identical to the write's stall-on-STA) and a
plain registered `LDA $E008` returns `sread`. Simpler (no blocking-LDA timing) but 2 ops/byte.
Build blocking first; fall back only if the timing can't be closed.

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
- **Sim — read window unit:** drive `coproc` with a stub SDRAM (the existing sim sdram model)
  pre-loaded with a known pattern; a tiny coproc program sets rptr, does `LDA $E007` a few
  times, stores the bytes to BRAM; assert the bytes match the pattern AND that consecutive
  reads returned consecutive addresses (auto-inc works). iverilog `-g2005`.
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
