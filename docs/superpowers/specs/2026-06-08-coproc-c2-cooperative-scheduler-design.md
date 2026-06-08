# Coprocessor C2 — Cooperative Scheduler + The Race (design)

**Date:** 2026-06-08
**Branch:** `coproc` (continues from C1, bench-verified `9977`)
**Board:** Byte Hamr Rev 2, `gateware/rev2/project_obscurus/` + `software/SDM/`
**Status:** approved-in-discussion, pending spec review
**Builds on:** [[project_coproc_c0]] C1 — resident protected kernel + live task registration.

## Why this exists

The founding goal: **run two authored tasks concurrently, see which finishes first, tune
one to flip the winner — prove async work.** C1 dispatches tasks *run-to-completion* (no
interleaving, no race). C2 makes them **interleave** via a cooperative scheduler with a
**context switch**, so completion order depends on each task's work and is tunable. This
is the headline demo of the whole coprocessor effort.

Built **extensibly**: an **N-task** scheduler (not hardwired to 2) and **multi-core-ready
firmware** (core-relative addressing via a `CORE_ID`), so the same `kernel.mem` will later
run unchanged on additional Arlet cores.

## Goal

Two host-registered tasks run interleaved under a cooperative scheduler on the soft-6502.
Each counts down its own iteration budget, yielding each step; when done it atomically
grabs the next "finish order" number and stamps it to SDRAM. The host reads both orders —
the task with the smaller budget finishes first (order 1). Raising that task's budget
flips the orders. Success = the two SDRAM order cells reflect the interleaved finish
order, and changing a budget flips them.

## Architecture

Almost entirely **firmware** (a `kernel.S` rewrite, re-baking `kernel.mem`). The only
gateware change is a tiny `CORE_ID` register.

### Gateware (minimal): `CORE_ID` for multi-core-ready firmware
`coproc.v` gains a **module parameter `CORE_ID` (default 0)** and decodes a read at
**`$E011` → `CORE_ID`** (read-only, like `$E010` COUNT). The top instantiates the single
C2 coproc with `CORE_ID=0`. The firmware addresses SDRAM **core-relative**:
`result = RESULT_BASE + CORE_ID*CORE_STRIDE + taskid`. So a future multi-core rung = drop
another `coproc` (with `CORE_ID=1`) into the already-N-extensible `sdram_arb`, load the
**same** `kernel.mem`, and it writes to its own SDRAM region with **zero firmware change**.
(That's the only gateware edit; no IRQ wiring — cooperative.)

### Firmware: N-task cooperative scheduler (kernel.S rewrite)
Replaces C1's run-to-completion dispatch loop. Everything via Merlin **equates** (no magic
numbers): `MAX_TASKS` (default 4), `STACK_SIZE`, `TCB_*` offsets, `RESULT_BASE`,
`CORE_STRIDE`.

**Where mutable kernel state lives.** The kernel region `$1000-$1FFF` is write-protected
*even from the kernel itself* (port-A gate), and the TABLE `$0200-$02FF` is protected too.
So the scheduler's **mutable** state lives in **kernel-reserved high ZP** (e.g.
`$00D0-$00FF`) — writable (port-A allows `$0000-$01FF`), cooperatively shared (tasks
trusted not to clobber kernel ZP, consistent with C1's data-plane boundary). Tasks use low
ZP (`$0002-$00CF`).

**TCB** (parallel ZP arrays indexed by task id, 6502-friendly):
`TCB_SP[id], TCB_A[id], TCB_X[id], TCB_Y[id], TCB_P[id], TCB_STATE[id]` (state =
EMPTY/READY/DONE). The resume **PC is NOT in the TCB** — it lives on each task's own stack
(the `YIELD` return address), the standard 6502 coroutine trick.

**Per-task stacks** — page 1 (`$0100-$01FF`) partitioned: task `id`'s stack tops at
`STACK_BASE + id*STACK_SIZE + (STACK_SIZE-1)`, grows down within its partition. Sized from
equates so `MAX_TASKS` partitions + a small kernel-boot reserve fit page 1 (e.g.
`MAX_TASKS=4`, `STACK_SIZE=$30` → 192 bytes tasks + `$01C0-$01FF` kernel boot). The
scheduler swaps the real 6502 `SP` (`TXS`) to the active task's saved `SP`.

**`YIELD`** (kernel routine, called `JSR YIELD` by a task):
```
save A/X/Y (to temp ZP), PHP->P
TSX ; store SP + A/X/Y/P into TCB[CUR_TASK]
pick next READY task round-robin (skip DONE/EMPTY) -> CUR_TASK
LDX TCB_SP[CUR_TASK] / TXS            ; switch to next task's stack
restore P (PHA/PLP), X, Y, A from TCB[CUR_TASK]
RTS                                   ; resumes next task at ITS last YIELD return
```
The switch happens mid-routine: `YIELD` enters on the old task's stack, the `TXS` swaps
to the new task's stack, and the `RTS` returns into the new task. No separate scheduler
stack needed.

**Bootstrap** (when COUNT goes nonzero — count-last, as C1): for `id = 0..COUNT-1`, init
`TCB[id]`: push `(entry-1)` (hi,lo) onto task `id`'s stack partition (so the first restore
`RTS`es to `entry`), `TCB_SP[id]` = partition top - 2, `TCB_A/X/Y[id]=0`, `TCB_P[id]`=`$04`
(I set, cooperative), `STATE=READY`. Set `CUR_TASK=0`, init the `ORDER` counter = 0, then
restore `TCB[0]` and `RTS` → task0 entry. (The kernel writes each task's page-1 stack
directly — `$0100-$01FF` is port-A-writable.)

**`DONE`** (task calls `JSR DONE` when finished): `STATE[CUR_TASK]=DONE`, then fall into
the round-robin (yield permanently). When no READY task remains, the scheduler spins in an
idle loop.

### Firmware: the race task (one parameterized binary)
A single task binary, parameterized by id so it scales to N racers:
```
RACETASK ($0300):
  LDX NPARAM,Y      ; Y=CUR_TASK; my iteration budget (host-set per task)
loop:
  DEX
  JSR YIELD         ; budget rides in X across yields (TCB save/restore preserves X)
  BNE loop          ; (re-load X from TCB on resume is automatic)
  ; finished -> atomically grab finish order:
  LDA ORDER / CLC / ADC #1 / STA ORDER     ; A = my order (no yield -> atomic)
  ; write A to SDRAM result = RESULT_BASE + CORE_ID*CORE_STRIDE + CUR_TASK via $E00x
  JSR DONE
```
`CUR_TASK` (ZP, scheduler-maintained) tells the task its id. `NPARAM[id]` (ZP table,
host-set via the load port) is each task's budget. The task reads `CORE_ID` (`$E011`) to
compute its core-relative result cell. (8-bit budget ≤255 is fine for the demo;
16-bit/extensible later.)

### Host loader (`CPRACE.S`)
Like C1's CPREG but registers 2 race-task instances + sets up the race:
1. Load the race-task binary into the task region (one copy, or two slots).
2. Set `NPARAM[0]`, `NPARAM[1]` (different budgets — e.g. 10 and 20) via the load port (ZP cells).
3. Init the SDRAM result cells (host writes 0 via the monitor port).
4. TABLE entry0/entry1 → the task entry, COUNT=2 LAST (count-last).
5. Wait, then read SDRAM `RESULT_BASE+0` / `RESULT_BASE+1` → the two finish orders; print.

## Data flow
```
host: load race task code + NPARAM[0]=10, NPARAM[1]=20 (load port), zero result cells, COUNT=2
kernel: COUNT>0 -> init 2 TCBs -> start task0
task0(id0,N=10) <-> task1(id1,N=20) interleave via YIELD
task0 hits 0 first -> ORDER:0->1, writes order 1 -> SDRAM RESULT_BASE+0
task1 hits 0 later -> ORDER:1->2, writes order 2 -> SDRAM RESULT_BASE+1
host: read RESULT_BASE+0 (=1), RESULT_BASE+1 (=2) -> task0 won
tune: NPARAM[0]=30 > NPARAM[1]=20 -> re-run -> task1 wins (orders flip)
```

## The proof
1. **Race runs interleaved:** both order cells are written (both tasks ran), and they are
   `1` and `2` (a real order, not both-1) → the scheduler interleaved them.
2. **Tune flips the winner:** with `NPARAM[0] < NPARAM[1]`, `RESULT_BASE+0 = 1`. Raise
   `NPARAM[0]` above `NPARAM[1]` → `RESULT_BASE+0 = 2`, `RESULT_BASE+1 = 1`. **Async proven
   — completion order is a scheduling result the host can tune.**

## Components / files (branch `coproc`)
- `coproc.v` — add `parameter CORE_ID = 0` + decode `$E011 → CORE_ID` (registered select,
  read-only). ~5 lines. (top instantiates `coproc #(.CORE_ID(0)) u_coproc`.)
- `project_obscurus_top.v` — pass `.CORE_ID(0)` (or rely on default); no other change.
- `kernel.S` → `kernel.mem` — REWRITE: cooperative N-task scheduler (TCB ZP arrays,
  per-task page-1 stacks, `YIELD`, `DONE`, bootstrap-on-COUNT, idle), core-relative result
  addressing. ISR stubs stay at `$1F00/$1F40` (IRQ still tied 0). Re-bakes `kernel.mem`.
- `software/SDM/racetask.S` — the parameterized race task. `software/SDM/CPRACE.S` — host
  loader (register 2, set NPARAM, read orders). Disk.
- `project_obscurus_tb.v` — integration: register 2 race tasks with different NPARAM via
  the load port, run, assert order cells = 1/2; then swap NPARAM and assert they flip.

## Error handling / edge cases
- **Uncooperative task (never yields):** hangs the others — inherent to cooperative
  scheduling. Acceptable for C2 (tasks are authored to yield); **preemption (C3, timer
  tick) fixes it** by reusing this context switch from an IRQ.
- **Stack overflow:** a task that pushes past `STACK_SIZE` corrupts a neighbor's stack —
  cooperative-trust (tasks kept simple; sizes from equates). Per-task overflow guards
  deferred.
- **Count-last (carried from C1):** all task code + NPARAM loaded while COUNT=0 (kernel
  idle, no BRAM writes), COUNT set last — avoids the single-write-port priority-mux
  collision. The host loader and tb both count-last.
- **ORDER atomicity:** the finish-order read-modify-write is done with no `YIELD` between
  read and write → atomic w.r.t. other tasks under cooperative scheduling.
- **CORE_ID read-only:** like `$E010`, a task `STA $E011` is a no-op (gateware reg).
- **Reset/ready:** keep C0/C1's SDRAM-`ready` gate before the first `$E003` post.

## Testing
- **Sim (integration):** register 2 race tasks (NPARAM 10/20) via the load port, run, assert
  `RESULT_BASE+0 == 1` and `RESULT_BASE+1 == 2`; then re-register with NPARAM 30/20 and
  assert the orders flip (`+0 == 2`, `+1 == 1`). Confirm both tasks ran (both cells
  written, distinct). iverilog `-g2005`.
- **Build:** re-bake `kernel.mem`; confirm DP16KD still 8 EBR, timing PASS. (CORE_ID adds a
  trivial decode.)
- **Bench:** flash; boot; `BRUN CPRACE` → prints the two orders (e.g. `01 02`); a second
  variant (or a re-run with swapped NPARAM) prints `02 01` — the flip.

## Success criteria
Two tasks interleave under the cooperative scheduler and finish in a host-observable order;
raising one task's budget flips the order. The kernel is an N-task scheduler (demo uses 2)
and the firmware is core-relative (`CORE_ID`) so it runs unchanged on future cores — async
multitasking on the soft-6502, extensible to N tasks and N cores.

## Non-goals (later rungs)
- **C3 — preemptive timer tick:** a coproc timer → IRQ → the ISR calls this same context
  switch; uncooperative tasks get sliced. (The protected ISR vectors/region from C1 are
  ready.)
- **Multi-core:** instantiate N `coproc` cores (each `CORE_ID=k`) into the N-extensible
  `sdram_arb`; same `kernel.mem`. (C2 makes the firmware ready; the instantiation is its
  own rung.)
- **C-flash:** persist registered tasks/params to flash (secondary boot).
- No 16-bit budgets, no per-task heap, no task priorities (round-robin only), no inter-task
  messaging beyond the shared ORDER cell.
