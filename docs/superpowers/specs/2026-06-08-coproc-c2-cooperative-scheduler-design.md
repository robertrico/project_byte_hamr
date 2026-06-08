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
C2 coproc with `CORE_ID=0`. The firmware computes a **core-relative base** at bootstrap:
`COREBASE = RESULT_BASE + (CORE_ID << STRIDE_LOG2)` — `CORE_STRIDE` is a **power of two**
(e.g. `$10`, `STRIDE_LOG2=4`) so it's a **shift**, not a multiply (the 6502 has no `MUL`).
Each task's result cell is then `COREBASE + CUR_TASK`.

**Honest scope of the multi-core claim:** at `CORE_ID=0` the shift yields 0, so `COREBASE
== RESULT_BASE` — the C2 demo exercises the base region. The addressing *code* (read `$E011`,
shift, add) **does run** at `CORE_ID=0` (producing the correct base), so it is not dead
code — but C2 **proves the scheduler, not multi-core operation**. A future rung instantiates
a second `coproc #(.CORE_ID(1))` into the N-extensible `sdram_arb` and loads the **same**
`kernel.mem`; only *then* is the `CORE_ID<<4` offset actually nonzero and the multi-core
claim tested. C2's contribution is making the firmware **core-ID-aware** (the code path
exists and is correct for core 0); it does not claim "proven on N cores." (This is the
only gateware edit; no IRQ wiring — cooperative.)

### Firmware: N-task cooperative scheduler (kernel.S rewrite)
Replaces C1's run-to-completion dispatch loop. Everything via Merlin **equates** (no magic
numbers): `MAX_TASKS` (default 4), `STACK_SIZE`, `TCB_*` offsets, `RESULT_BASE`,
`CORE_STRIDE`.

**Where mutable kernel state lives.** The kernel region `$1000-$1FFF` is write-protected
*even from the kernel itself* (port-A gate), and the TABLE `$0200-$02FF` is protected too.
So the scheduler's **mutable** state lives in **kernel-reserved high ZP** (`$00D0-$00FF`,
all via equates) — writable (port-A allows `$0000-$01FF`), cooperatively shared (tasks
trusted not to clobber kernel ZP, consistent with C1's data-plane boundary). Tasks use low
ZP (`$0002-$00CF`). Kernel-ZP map (pinned, equate-driven):
- `TCB_SP/A/X/Y/P/STATE` — six `MAX_TASKS`-byte parallel arrays (24 bytes at MAX_TASKS=4).
- `NPARAM` — `MAX_TASKS`-byte per-task budget table (**host-set via the load port** — the
  host writes ZP through `CP_LADDR`/`CP_WDATA`; task-read).
- `CUR_TASK` (1), `ORDER` (1), `COREBASE` (2, the computed core-relative result base),
  `TMPA/TMPX/TMPY/TMPP` (4 scratch).

**TCB** (parallel ZP arrays indexed by task id, 6502-friendly):
`TCB_SP[id], TCB_A[id], TCB_X[id], TCB_Y[id], TCB_P[id], TCB_STATE[id]` (state =
EMPTY/READY/DONE). The resume **PC is NOT in the TCB** — it lives on each task's own stack
(the `YIELD` return address), the standard 6502 coroutine trick.

**Per-task stacks** — page 1 (`$0100-$01FF`) partitioned: task `id`'s stack tops at
`STACK_BASE + id*STACK_SIZE + (STACK_SIZE-1)`, grows down within its partition. Sized from
equates so `MAX_TASKS` partitions + a small kernel-boot reserve fit page 1 (e.g.
`MAX_TASKS=4`, `STACK_SIZE=$30` → 192 bytes tasks + `$01C0-$01FF` kernel boot). The
scheduler swaps the real 6502 `SP` (`TXS`) to the active task's saved `SP`. **Stack
budget per task:** each `JSR YIELD` costs 2 bytes (return addr) + the task's own pushes;
`$30` (48 B) is ample for a counter task but a task author must stay within `STACK_SIZE`
(overflow corrupts the neighbor partition — cooperative-trust, noted below).

**`YIELD`** (kernel routine, called `JSR YIELD` by a task):
```
; SAVE current task — ORDER MATTERS:
STA TMPA / STX TMPX / STY TMPY        ; save X to TMPX BEFORE the TSX below (TSX clobbers X)
PHP / PLA / STA TMPP                   ; capture P
TSX                                    ; X = SP (real X already saved in TMPX)
LDY CUR_TASK                           ; Y = index into the TCB arrays
TXA / STA TCB_SP,Y                     ; store SP, then TMPA/TMPX/TMPY/TMPP -> TCB_*,Y
; pick next READY task round-robin from CUR_TASK+1 (wrap, skip DONE/EMPTY) -> CUR_TASK
; RESTORE next task — restore Y (the index) LAST:
LDY CUR_TASK / LDX TCB_SP,Y / TXS      ; switch to next task's stack
LDA TCB_P,Y / PHA / PLP                ; P
LDX TCB_X,Y / LDA TCB_A,Y              ; X, A
LDA TCB_Y,Y / ... TAY  (restore Y last; the index Y is consumed only after all loads)
RTS                                    ; resume next task at ITS last YIELD return
```
The switch happens mid-routine: `YIELD` enters on the old task's stack, the `TXS` swaps to
the new task's stack, and the `RTS` returns into the new task. No separate scheduler stack.
**Two ordering traps the plan must honor:** (1) save the task's `X` to a temp *before* `TSX`
(which overwrites `X` with `SP`); (2) `Y` holds the TCB array index throughout, so restore
the task's `Y` value **last**, after every `TCB_*,Y` load. Tie-break: equal budgets finish
the same round; the order is then decided by the round-robin service order (lower task id
goes first) — deterministic, not nondeterminism.

**Kernel top-level structure (COUNT-edge re-bootstrap — enables re-run/flip):** C2 is
stateful (TCBs init once, tasks go DONE), so unlike C1's free-running re-dispatch it must
**re-bootstrap on each `COUNT` 0→nonzero edge**:
```
RESET: SEI ; set kernel boot SP (top of page 1 reserve)
KWAIT0: LDA $E010 / BNE KWAIT0     ; wait until COUNT == 0  (re-arm prep)
KWAITN: LDA $E010 / BEQ KWAITN     ; wait until COUNT != 0  (the arm edge)
  JSR BOOTSTRAP                    ; (re)init TCBs[0..COUNT-1], ORDER=0, CUR_TASK=0, COREBASE
  ; BOOTSTRAP ends by restoring TCB[0] + RTS -> task0 (does NOT return here)
KIDLE: ; reached by DONE when no READY task remains
  LDX #<kernel boot SP> / TXS      ; abandon task stacks, back to kernel stack
  JMP KWAIT0                       ; re-arm: wait for COUNT 0 then nonzero again
```
First run: POR `COUNT=0` → KWAIT0 falls straight through → KWAITN waits → host sets
`COUNT=2` → BOOTSTRAP. Re-run/flip: all tasks DONE → KIDLE → KWAIT0; host writes `COUNT=0`,
reloads `NPARAM`, writes `COUNT=2` → the kernel re-bootstraps with the new budgets.

**`BOOTSTRAP`** (run on each arm edge): compute `COREBASE = RESULT_BASE + (CORE_ID<<STRIDE_LOG2)`
(read `$E011`, shift — see Multi-core); `ORDER=0`; for `id = 0..COUNT-1`: push `(entry-1)`
(hi,lo) onto task `id`'s page-1 stack partition (so the first restore `RTS`es to `entry`),
`TCB_SP[id]` = partition top - 2, `TCB_A/X/Y[id]=0`, `TCB_P[id]`=`$04` (I set), `STATE=READY`;
for `id = COUNT..MAX_TASKS-1`: `STATE=EMPTY`. Then `CUR_TASK=0`, restore `TCB[0]`, `RTS` →
task0. (The kernel writes each task's page-1 stack directly — `$0100-$01FF` is port-A-writable.)

**`DONE`** (task calls `JSR DONE` when finished): `STATE[CUR_TASK]=DONE`; scan TCBs for any
`READY` — if found, switch to it (round-robin); if none, `JMP KIDLE` (re-arm).

### Firmware: the race task (one parameterized binary)
A single task binary, parameterized by id so it scales to N racers:
```
RACETASK ($0300):
  LDY CUR_TASK      ; *** my id from the scheduler ZP — do NOT assume Y on entry ***
  LDX NPARAM,Y      ; my iteration budget (host-set per task)
loop:
  DEX
  JSR YIELD         ; budget rides in X across yields (TCB save/restore preserves X + Z)
  BNE loop          ; Z from this task's own DEX is restored with P -> BNE valid
  ; finished -> atomically grab finish order (no YIELD between LDA/STA -> atomic):
  LDA ORDER / CLC / ADC #1 / STA ORDER     ; A = my order
  ; result cell = COREBASE + CUR_TASK (COREBASE = RESULT_BASE + (CORE_ID<<STRIDE_LOG2),
  ; precomputed once at bootstrap); write A there via the $E00x SDRAM window
  JSR DONE
```
**Bug-1 note:** the bootstrap sets `TCB_Y[id]=0` for all tasks, so a task must NOT assume
`Y == id` on entry — it loads `CUR_TASK` explicitly (`LDY CUR_TASK`). `CUR_TASK` (ZP,
scheduler-maintained, updated on every switch) is the task's id. `NPARAM[id]` (kernel-ZP
table, host-set via the load port) is each task's budget. (8-bit budget ≤255; the task
keeps the live counter in `X`, preserved across `YIELD` by the TCB.)

### Host loader (`CPRACE.S`)
Registers 2 race-task instances, runs the race, then **re-arms and runs it again with
swapped budgets to show the flip — all in one `BRUN`**:
1. Load the race-task binary into the task region (one copy at `$0300`; both TABLE entries
   point at it — same code, per-task budget via `NPARAM`).
2. Set `NPARAM[0]=10`, `NPARAM[1]=20` via the load port (kernel-ZP cells).
3. TABLE entry0/entry1 → `$0300`; `COUNT=2` LAST (count-last) → kernel bootstraps + races.
4. Wait, read SDRAM `RESULT_BASE+0` / `RESULT_BASE+1` → print (expect `01 02`).
5. **Re-arm:** write `COUNT=0` (kernel → KWAIT0), set `NPARAM[0]=30`, `NPARAM[1]=20`,
   write `COUNT=2` (the arm edge → re-bootstrap).
6. Wait, read the result cells again → print (expect `02 01` — flipped).

(SDRAM result cells need no pre-zeroing — each race writes both; but the host MAY zero them
via the monitor port between runs for a clean read.)

## Data flow
```
host: load race task code + NPARAM[0]=10, NPARAM[1]=20 (load port), zero result cells, COUNT=2
kernel: COUNT>0 -> init 2 TCBs -> start task0
task0(id0,N=10) <-> task1(id1,N=20) interleave via YIELD
task0 hits 0 first -> ORDER:0->1, writes order 1 -> SDRAM RESULT_BASE+0
task1 hits 0 later -> ORDER:1->2, writes order 2 -> SDRAM RESULT_BASE+1
host: read RESULT_BASE+0 (=1), RESULT_BASE+1 (=2) -> task0 won
re-arm: COUNT=0 ; NPARAM[0]=30 > NPARAM[1]=20 ; COUNT=2 (arm edge -> kernel re-bootstraps)
host: read again -> RESULT_BASE+0 (=2), RESULT_BASE+1 (=1) -> task1 wins (orders flipped)
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
- **Sim (integration):** register 2 race tasks (NPARAM 10/20) via the load port, set COUNT=2,
  wait, assert `RESULT_BASE+0 == 1` and `RESULT_BASE+1 == 2` (both written, distinct → both
  ran, interleaved). Then **re-arm**: COUNT=0, NPARAM 30/20, COUNT=2, wait, assert the orders
  flip (`+0 == 2`, `+1 == 1`) — proves both the re-bootstrap edge and tunability. iverilog
  `-g2005`. (Use a smaller NPARAM in sim, e.g. 4/8, to keep cycle counts short.)
- **Build:** re-bake `kernel.mem`; confirm DP16KD still 8 EBR, timing PASS. (CORE_ID adds a
  trivial decode.)
- **Bench:** flash; boot; `BRUN CPRACE` → prints the first race's orders (`01 02`), then —
  after the in-program re-arm (COUNT 2→0→2 with swapped NPARAM) — the second race's orders
  (`02 01`). One run shows the race AND the flip.

## Success criteria
Two tasks interleave under the cooperative scheduler and finish in a host-observable order;
re-arming with a raised budget flips the order. The kernel is an N-task scheduler (demo uses
2) and the firmware is **core-ID-aware** (`CORE_ID`/`$E011`, core-relative result base) so a
future second core runs the same `kernel.mem` — async multitasking on the soft-6502,
structured for N tasks now and N cores later (multi-core *instantiation* is its own rung,
not proven by C2).

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
