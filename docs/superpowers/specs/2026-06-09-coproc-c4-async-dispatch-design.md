# Coprocessor C4 — Async Skill Dispatch + CALL ABI + CPLIB (design)

**Date:** 2026-06-09
**Branch:** TBD (off `coproc` / after PR#1)
**Board:** Byte Hamr Rev 2, `gateware/rev2/project_obscurus/` + `software/SDM/`
**Status:** approved-in-discussion, pending spec review
**Builds on:** [[project_coproc_c0]] C3 (preemptive scheduler), C3.1 (GO trigger), C-flash (persistent registry).

## Why this exists

The reason for a *second* CPU (not time-slicing the host) is **parallelism**: a //e program
fires a skill and keeps running while the coproc computes alongside it. Today skills only run
as a host-triggered batch race (GO bootstraps COUNT tasks and races them); there is no way for
an arbitrary //e program to *call* a registered skill, asynchronously, mid-execution, and
collect its result later. C4 makes skills **async-callable from any //e program** — the payoff
of everything built so far.

## Goal

A //e program calls a registered skill (`CP_CALL skill_id, args → handle`) without blocking;
the coproc runs it concurrently with the caller and with other in-flight skills; the program
collects the result by handle (`CP_POLL`/`CP_WAIT`/`CP_RESULT`). Up to 4 skills in flight at
once. Success = a demo //e program fires skills, keeps doing visible work, and collects results
mid-stream — proving the coproc ran in parallel — in sim and on the bench, with the existing
C2/C3/C3.1/C-flash behavior intact.

## Architecture

### The model: spawn-on-demand (generalizes C3.1's `go_pending`)
C3.1 had ONE `go_pending` bit (a host write the kernel acks) that batch-bootstraps COUNT
tasks. C4 generalizes that into **per-slot spawn requests** + a **slot lifecycle**, layered on
the existing preemptive scheduler. The batch GO path is preserved (additive — C3.1 regression
holds); CALL is the new per-slot front-end. Long-term the batch model can retire once joins
exist; v1 keeps it.

### Skill vs call vs slot
- **Skill** — registered code by `skill_id` (entry vector in the TABLE, persisted in flash via
  C-flash). *What* to run. Resident at boot (no load needed if persisted).
- **Slot** — one of 4 execution contexts (the 4 per-task page-1 stacks `$2F/$5F/$8F/$BF`
  already exist). The slot index **is the handle**.
- **Call** — instantiate `skill_id` into a FREE slot. The same skill can run in two slots at
  once (separate stacks + mailboxes → reentrant, see the skill ABI).

### Slot lifecycle (the new kernel core)
`FREE → (host rings) REQUESTED → (kernel spawns) RUNNING → (skill JMP DONE / budget expiry)
DONE → (host CP_RESULT) FREE`. Per-slot state lives in kernel ZP; the host sees it through the
control-plane registers below.

### Control plane = gateware registers (dual-faced host `$C0Cx` / kernel `$E0xx`)
Handshake flags are registers, not BRAM bytes (keeps the C1 port-B-vs-port-A write-contention
lesson out of the control path). **The host card window is FULL — all 16 bytes `$C0C0-$C0CF`
are allocated** (SDRAM monitor `$C0C0-C8` + coproc load/COUNT/SAVE/svc_count `$C0C9-CF`). There
are NO free full R/W registers. BUT the top's decode **splits read vs write per nibble**, so a
nibble used for READ by one function is free for WRITE (and vice-versa). C4's host-facing regs
ride those free half-nibbles — exact pins pinned here, near-zero margin (the plan implements):
- **`CALL_REQ[3:0]`** — doorbell. Host rings via **`CP_RING` = `$C0C5` WRITE** (free write side;
  `$C0C5` read = SDRAM `R_STAT`): host writes slot # (0-3) as the data byte → gateware sets
  `CALL_REQ[slot]`. Kernel reads `CALL_REQ` on the **`$E0xx`** side (room: `$E015`+ free) and
  acks-clears a bit after spawning (kernel `$E0xx` write of the slot #).
- **`DONE[3:0]`** — kernel sets bit S on completion (kernel `$E0xx` write); host reads the mask
  via **`$C0C0` READ** (free read side; `$C0C0` write = SDRAM `R_ADRLO`) for `CP_POLL`; host
  clears bit S on collect via **`CP_COLLECT` = `$C0CC` WRITE** (free write side; `$C0CC` read =
  `CP_RDATA`), writing the slot #.
- **`ACTIVE[3:0]`** = `CALL_REQ | RUNNING | DONE`, host reads via **`$C0C1` READ** — so CPLIB
  picks a FREE slot. **`RUNNING[3:0]` MUST be published by the kernel to a gateware register**
  (one `$E0xx` write on spawn + on completion) — load-bearing, not decorative: without a
  host-visible RUNNING, the window between ack (`CALL_REQ` cleared) and `DONE` has no busy bit,
  so CPLIB sees the slot FREE and **double-spawns into a live slot**. (Host-local allocation
  tracking is REJECTED: it's lost if the //e program restarts mid-flight → silicon must be the
  source of truth.)
- **`TIMEDOUT[3:0]`** — per-slot watchdog-kill flag; host reads via **`$C0C2` READ** to
  distinguish a clean result from a run-budget kill.
- **`irq_enable`** (host write, 1 bit) — completion-IRQ master enable (v2; default 0 in v1).
The HOST side is the tight constraint (free write half-nibbles `$C0C5`/`$C0CC`/`$C0CE`, free read
half-nibbles `$C0C0-C4` etc.). Kernel-side `$E0xx` has ample room (`$E004-E00F`, `$E015`+) for the
kernel's `CALL_REQ` read + its `DONE`/`RUNNING`/`TIMEDOUT`/ack writes.

### Data plane = mailbox (hybrid)
Per-slot mailbox in coproc BRAM (a reserved region indexed by slot): `skill_id` (1B),
`run_budget` (1–2B), small **arg** bytes, small **result** bytes. Host writes args via the
`$C0Cx` load port before ringing; the skill reads its mailbox, writes its result there; host
reads the result via the load port after `DONE`. **Bulk** data goes in SDRAM, managed by the
skill (host via SDRAMLIB) — the BRAM mailbox carries an SDRAM pointer/length when needed.
**MAILBOX MUST live in `$0300+`, NOT `$0200-$02FF`.** The Arlet write-protect
(`a_wr_ok = WE & in_bram & rdy & ~AB[12] & ~(AB[11:8]==4'h2)`) blocks port-A writes to `$1000+`
*and* to `$02xx` (the TABLE) — a skill writing its result into a `$02xx` mailbox would **silently
drop** (write masked). So the mailbox is carved from the Arlet-writable `$0300-$0FFF` (e.g. a
reserved block at the top of the region; the task-code region shrinks accordingly — exact split
pinned in the plan). The host load port (port B, `b_wr_ok = lwr & ~laddr[12]`) can write
`$0000-$0FFF`, so it stages args there fine; the host reads results back via `CP_RDATA`.
(Mailbox bytes fall inside the C-flash-persisted `$0300-$0FFF` region, but they're transient
runtime data re-staged each call — harmless to persist; no special handling.)

### CALL request path (doorbell → dispatcher) — dispatch runs IN THE TICK ISR
```
HOST:  pick FREE slot S (read ACTIVE) -> stage S's mailbox (skill_id, budget, args via load port)
       -> CP_RING(S) sets CALL_REQ[S]            (the doorbell)
KERNEL dispatcher — RUNS EVERY TICK in the timer ISR ($1F00), BEFORE the SWITCH, so calls
       are serviced WHILE other slots run (NOT only at idle):
       each tick: scan CALL_REQ -> for each set bit S with a FREE slot:
         read S.mailbox.skill_id -> TABLE[skill_id] entry
         spawn into slot S (seed stack + IRQ-frame exactly like BOOTSTRAP, load run_budget)
         mark slot RUNNING (publish RUNNING[S]), clear CALL_REQ[S] (ack)
       then: decrement each RUNNING slot's run_budget; budget==0 -> force-complete+TIMEDOUT
       then: SWITCH among RUNNING slots (existing C3 preemptive scheduler)
ON slot S done (skill JMP DONE, or budget expiry): result already in S.mailbox -> set DONE[S],
       clear RUNNING[S], drop S from the ready set
HOST:  CP_POLL(S)=DONE[S]? -> CP_RESULT(S): read S.mailbox.result, clear DONE[S] (slot -> FREE)
```
Many concurrent calls fall out for free: the host sets several `CALL_REQ` bits; the dispatcher
(in the ISR) spawns each into its slot over the next tick(s); the scheduler runs them together.

### THE core kernel change: free-running tick + dispatch-in-ISR (not a `KWAITGO` tweak)
This is the heart of C4 and is NOT a small evolution of `KWAITGO`. In C3 the timer arms on
bootstrap and **disarms at `KIDLE`** (idle = no timer), and the tick ISR only context-switches
existing `TCB_ST==1` tasks — it never scans `CALL_REQ`. If the dispatch stayed at idle only, a
`CP_RING` issued while slot 0 is mid-race would not be spawned until the race fully drains —
breaking "call mid-execution, run concurrently." So C4:
- **Keeps the timer free-running once the kernel boots** (it does NOT disarm at idle), so the
  ISR fires periodically even with no slots running — that's how a first call from a fully-idle
  coproc gets noticed.
- **Moves the `CALL_REQ` scan + spawn + budget-decrement INTO the tick ISR**, ahead of the
  `SWITCH`. New calls are picked up every tick regardless of what else is running.
- **Idle state = ticking with an empty ready set**: the ISR scans, finds nothing READY, returns
  (RTI) to a low-power idle spin until the next tick. (Replaces C3's disarm-and-`KWAITGO`-park.)
The batch `go_pending` path (C3.1) is preserved as a second front-end that also feeds the slot
spawn machinery (for the regression demos).

### Run-budget (the abort-lite, weighted to CALL)
`CP_CALL` carries a `run_budget` (ticks). The tick ISR decrements each RUNNING slot's budget;
at 0 the kernel **force-completes** the slot (tears down its stack, sets `DONE[S]` + `TIMEDOUT`,
so a runaway/wedged skill can't hang a slot forever). Optional secondary **wait-timeout** on
`CP_WAIT` bounds the *caller's* spin (returns "timed out", skill keeps running). Budget=0 (or a
sentinel) = "no limit".

### Completion: poll-first, IRQ-ready (same surface)
v1: host polls `DONE` (via CPLIB). v2: gateware drives `nIRQ = (DONE & ~collected) & irq_enable`;
the //e IRQ handler reads the *same* `DONE` register. No ABI change between v1/v2. (`nIRQ` is
wired in hardware, currently `1'bZ`; the daisy-chain `INT_IN`/`INT_OUT` exists too.) IRQ +
ProDOS IRQ-chain integration is **out of scope for v1** (deferred to keep the fragile
ProDOS-IRQ work off the critical path).

### Skill ABI (the reentrancy contract — with a real per-slot scratch mechanism)
- On entry the skill knows its slot (kernel provides `CURSLOT`); `SLOTBASE = mailbox_base +
  slot*block`. Read args from `SLOTBASE`, write result to `SLOTBASE+result_off`.
- End with `JMP DONE` (kernel marks `DONE[slot]`). Never `RTS` (the C2 lesson).
- **Per-slot isolation today is stacks only** (`$2F/$5F/$8F/$BF`); **ZP `$00-$CF` is SHARED
  across all slots**, and the kernel owns `$D0-$F2` (TCBs/scratch). A bare "no globals" rule
  effectively bans ZP — near-impossible for real 6502. So C4 **carves a per-slot ZP scratch
  window**: `SLOTZP = SLOTZP_BASE + slot*N` (N≈4-8 bytes × 4 slots, in low ZP; exact size/base
  pinned in the plan). A skill addresses its private scratch through its slot (the kernel hands
  it `CURSLOT`/a `SLOTZP` pointer). This is the *mechanism* that makes a skill reentrant — same
  code in two slots, each with its own stack + ZP-scratch + mailbox.
- **Reentrancy contract:** a skill must use only its stack, its `SLOTZP` window, its mailbox, and
  slot-relative SDRAM — never fixed ZP/SDRAM globals. (Not silicon-enforced; a non-reentrant
  skill simply must not be called twice in flight.)

### CPLIB (host library — the ergonomics)
Assembly `JSR` entries + a BASIC hook (`&`/`CALL`). The lib owns the spin; the programmer uses
verbs (never hand-rolls a poll loop).
- `CP_CALL(skill_id, args, run_budget) → handle` — find FREE slot (read ACTIVE), stage mailbox,
  `CP_RING`. Non-blocking. Returns handle (or "no free slot").
- `CP_POLL(handle) → done?` — read `DONE[handle]`. Non-blocking.
- `CP_WAIT(handle [,timeout])` — spin until `DONE[handle]` (lib loop), optional timeout.
- `CP_RESULT(handle) → result` — read mailbox result, clear `DONE[handle]` (free slot).
- (Deferred: `CP_WAITALL`/`CP_WAITANY` joins, `CP_ABORT`.)

Three documented paradigms: **async-interleave** (`CALL` → work → `POLL`/`RESULT`),
**blocking-wait** (`CALL` → `WAIT` → `RESULT`), and (later) **fan-out/join**.

## Components / files (new branch off `coproc`)
- `coproc.v` — add the dual-faced control-plane register block: `CALL_REQ[3:0]` (set by
  `CP_RING`=`$C0C5` host write of slot#, kernel reads via `$E0xx`, kernel ack-clears),
  `DONE[3:0]` (kernel `$E0xx` set, host reads `$C0C0`, host clears via `CP_COLLECT`=`$C0CC`),
  `RUNNING[3:0]` (kernel `$E0xx` publishes), `ACTIVE`=`CALL_REQ|RUNNING|DONE` (host reads
  `$C0C1`), `TIMEDOUT[3:0]` (kernel set, host reads `$C0C2`), `irq_enable` (v1 unused). Uses the
  free R/W half-nibbles (host side) + free `$E0xx` (kernel side). `nIRQ` stays `1'bZ` in v1.
- `kernel.S` — **the dispatcher-in-ISR rewrite (the core change):** the tick ISR (`$1F00`) gains
  a `CALL_REQ` scan + per-slot spawn (generalized `BOOTSTRAP` into a chosen slot, loading
  `run_budget` + publishing `RUNNING`) + per-slot budget decrement/force-complete, all BEFORE the
  `SWITCH`. The **timer free-runs** (no disarm at idle); idle = ISR scans, finds nothing READY,
  RTIs. Per-slot lifecycle/state + `DONE`/`RUNNING`/`TIMEDOUT` posting. Carve the per-slot
  `SLOTZP` scratch window. Keep the batch `go_pending` path (regression) feeding the same spawn
  machinery. Reuses RESTORE/SWITCH/PICKNEXT. Re-bake `kernel.mem`. Keep the API table
  ($1000/1003/1006) + the `$1F00` ISR offset stable.
- `software/SDM/CPLIB.S` — the host library (`PUT`-include, like SDRAMLIB): `CP_CALL`/`CP_POLL`/
  `CP_WAIT`/`CP_RESULT` + a BASIC `&` hook. Zero-ZP-safe (ProDOS rules), non-indexed `$C0Cx`.
- `software/SDM/CPDEMO.S` — the demo/acceptance caller: fire 2–3 skills, do visible //e work
  (print/animate), collect mid-stream; prove parallel. (+ a BASIC variant if the `&` hook lands.)
- `software/SDM/` skills — adapt `racetask3` (or a fresh `cmpskill`) to the slot mailbox ABI
  (read args from SLOTBASE, write result there, reentrant). Keep a registered, flash-persistable
  skill for the demo.
- `project_obscurus_top.v` — decode the new `$C0Cx` regs (`CP_RING`, `CP_COLLECT`, `DONE`/
  `ACTIVE` reads) + wire the coproc control-plane ports. `nIRQ` unchanged (v1).
- `project_obscurus_tb.v` — integration: a host sequence that CALLs 2 skills concurrently,
  asserts both run + both results collected by handle + ACTIVE/DONE transitions; run-budget
  timeout (a skill that never finishes → TIMEDOUT, slot freed); all prior 21 tests still PASS.

## Error handling / edge cases
- **No free slot on `CP_CALL`:** `ACTIVE`==1111 → CPLIB returns a "busy/no-slot" status; caller
  retries later (or waits on an existing handle). Never blocks silently.
- **Doorbell vs ack race:** `CALL_REQ` set (host) vs clear (kernel ack) — per-slot, set-wins on
  same-cycle collision (like C3.1's go_pending order). Different slots are independent bits.
- **DONE set vs collect-clear race:** kernel sets `DONE[S]` (completion) while host clears a
  *different* `DONE[S']` (collect) — independent bits. Same-bit collision (collect while kernel
  re-... can't happen: a slot can't complete again until re-called, which needs collect first).
- **Run-budget expiry mid-work:** tick ISR force-completes — must tear the slot down cleanly
  (reset its stack/state) so the slot is reusable; result bytes flagged `TIMEDOUT` (host sees a
  kill vs a real result).
- **Reentrancy violation:** a skill using fixed globals breaks when called twice concurrently —
  documented contract, not silicon-enforced. (A non-reentrant skill simply shouldn't be called
  twice in flight.)
- **BRAM write contention (C1 lesson):** control-plane is registers (no contention). Mailbox
  args are host-written while the slot is FREE (kernel not touching it); result is kernel-written
  while RUNNING then read by host after DONE — no concurrent same-byte write. Preserve count-last
  discipline for any code loads.
- **Batch GO coexistence:** `go_pending` (C3.1) still bootstraps the batch race for the
  regression demos; it shares the slot machinery. A program must not mix batch-GO and CALL on the
  same slots concurrently (protocol; the demos use one or the other).

## Testing
- **Sim — single CALL:** stage mailbox, `CP_RING(0)`, assert kernel spawns slot 0, runs it, sets
  `DONE[0]`, result readable; ACTIVE/DONE transitions correct.
- **Sim — two concurrent CALLs:** ring slots 0 and 1 (different skills/args), assert both run +
  both results collected by handle, no cross-slot corruption.
- **Sim — mid-race CALL (the dispatch-in-ISR proof):** with slot 0 running a long skill, ring
  slot 1 *while slot 0 is still RUNNING*; assert slot 1 spawns + makes progress BEFORE slot 0
  finishes — i.e. the call was serviced mid-execution by the tick ISR, not deferred to idle.
- **Sim — RUNNING visibility:** after CALL + ack (CALL_REQ cleared) but before DONE, assert
  `ACTIVE[S]`==1 (so CPLIB won't double-allocate the live slot).
- **Sim — run-budget timeout:** call a never-finishing skill with a small budget → tick ISR
  force-completes → `DONE` + `TIMEDOUT`, slot freed + reusable (a subsequent CALL into it works).
- **Sim — regression:** all 21 prior PASS (monitor/C1/C2/C3/C3.1/C-flash) — the batch GO path
  unbroken. iverilog `-g2005`.
- **Build:** re-bake `kernel.mem`; DP16KD 8 (mailbox fits existing BRAM), timing PASS, the new
  register block adds FFs + decode.
- **Bench:** `BRUN CPDEMO` → fires skills, the //e shows its own work proceeding, results
  collected mid-stream (e.g. two compute skills return while a counter prints) → async parallel
  on silicon. With a flash-persisted skill, no load needed (C-flash + C4 compose).

## Success criteria
A //e program calls registered skills asynchronously, up to 4 concurrent, and collects results
by handle while continuing its own execution — proven by a demo whose //e-side work visibly
overlaps the coproc work — with a kernel-enforced per-call run-budget watchdog, and all prior
rungs intact. Skills are now a callable, parallel service library for the Apple II.

## Non-goals (v1)
- No joins (`CP_WAITALL`/`CP_WAITANY`) — single-handle verbs first.
- No explicit `CP_ABORT` — the run-budget watchdog covers runaways.
- No completion IRQ — poll-first; `nIRQ`/ProDOS-IRQ-chain is a v2 toggle on the same `DONE`
  surface.
- No multi-core (orthogonal rung) and no on-//e authoring workshop (separate later sub-project;
  it becomes a CPLIB client).
- No removal of the batch GO/COUNT path (kept for regression; may retire after joins exist).
- No dynamic skill count > 4 (bounded by the 4 existing stacks).
