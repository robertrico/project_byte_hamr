# Coprocessor C3 — Preemptive Timer Tick (design)

**Date:** 2026-06-08
**Branch:** `coproc` (continues from C2, bench-verified `0102`/`0201`)
**Board:** Byte Hamr Rev 2, `gateware/rev2/project_obscurus/` + `software/SDM/`
**Status:** approved-in-discussion, pending spec review
**Builds on:** [[project_coproc_c0]] C2 — cooperative scheduler + the race.

## Why this exists

C2 is cooperative: a task must `YIELD` to give up the CPU; an uncooperative (non-yielding)
task hangs the others. C3 adds a **preemptive timer tick** — a card timer fires a periodic
IRQ into the soft-6502; the ISR preempts whatever's running and switches tasks. This makes
it a true RTOS: **a task that never yields gets sliced anyway**, and the scheduler can't be
hung by a runaway task. Hybrid: cooperative `YIELD` stays (voluntary), the tick adds
involuntary preemption.

**Robustness is the goal** (solo use, not production — but no silent-corruption footguns).
This spec folds in a 14-item red-team (below) as design requirements.

## Goal

Two **non-yielding** tight-loop tasks run interleaved purely by the timer tick, finish in a
host-observable order, and re-arming with a raised budget flips the order — with no `YIELD`
in the tasks. The C2 **cooperative** racetask still works (its `YIELD` produces the same
context format). Success = the tight-loop race shows `01 02` then `02 01`, the cooperative
race still passes, and the robustness checks (B-bit, no-stale-tick, stack budget) hold.

## Architecture

The center is a **uniform IRQ-frame context** rework + a small timer. Mostly firmware
(`kernel.S`); one small gateware add (the timer + IRQ wire).

### Uniform context — the TCB drops P; the frame carries it
Every suspended task has an **IRQ frame `[PCH, PCL, P]` on its own page-1 stack** plus
`{SP, A, X, Y}` in its TCB (the C2 `TCB_P` array is **removed** — P lives only in the
frame). One resume path everywhere: `SP←TCB`, restore A/X/Y, **`RTI`** (pops P+PC). Three
producers of that one format:
- **TICK ISR** (`$1F00`): the 6502 IRQ already pushed `[PCH,PCL,P]` → the natural frame.
- **`YIELD`** (`$1003`): manufactures it — pull the `JSR` return, **16-bit `+1`** (`INC`
  lo; `BNE` skip; `INC` hi — JSR pushes `ret-1`, RTI wants the exact PC; the carry into PCH
  matters if `JSR YIELD`'s return straddles a page), push back `[PCH,PCL,P]` with **I-clear**.
- **`BOOTSTRAP`**: seeds each new task's stack `[PCH(entry), PCL(entry), P=$00]`
  (I-clear → preemptible), `SP = top-3`. `RTI` resumes at the **exact** entry (no `-1`,
  unlike C2's RTS seeding).

`RESTORE` (one routine): `LDY CUR / LDX TCB_SP,Y / TXS`, stage A (`PHA`), restore X then Y
(index retired last), `PLA` (A), **`RTI`**. P is NOT in the TCB — `RTI` pulls it from the
frame, so no flag-staging juggling (simpler + robuster than C2's RESTORE).

`SWITCH` (shared core): save current `{SP,A,X,Y}`→TCB, `JSR PICKNEXT`, `JMP RESTORE`. Both
the tick ISR and `YIELD` funnel into `SWITCH`.

**Register-save contract (pin — silent-clobber trap).** The TCB store is indexed by
`Y=CUR` (`STA TCB_A,Y` ...), but `LDY CUR` would clobber the task's `Y` *before* it's
saved. So **the caller stages `A/X/Y` to non-indexed scratch ZP (`TMPA/TMPX/TMPY`) FIRST**,
then `SWITCH` does `LDY CUR / TSX / STX TCB_SP,Y` and copies `TMPA/TMPX/TMPY → TCB_A/X/Y,Y`.
- **Tick ISR:** `STA TMPA / STX TMPX / STY TMPY` **FIRST** — before `TSX` and the B-check.
  ZP stores don't move `SP`, so the `$0101,X` offset stays valid; and this saves the task's
  `X` *before* `TSX` overwrites it with `SP` and saves `A` before `LDA $0101,X` overwrites
  it. (The BRK leg then restores `A`/`X` from `TMP*` before `RTI` — see #1.)
- **`YIELD`:** `STA TMPA / STX TMPX / STY TMPY` at the very start — *before* the frame
  `+1` manipulation (which uses `A`). Then build the frame, then `JMP SWITCH`.
The `TMP*` scratch is safe (kernel is non-reentrant: tick is I-set; `YIELD`/`DONE` `SEI`).
`TMPP` is gone — P rides the frame. So `SWITCH`'s precondition is "A/X/Y already in `TMP*`."

### I-flag discipline (the robustness backbone)
- **Tasks run I-clear** (seeded/yield/restored frame P has I=0) → preemptible.
- **Every task→kernel entry SEIs on entry:** `YIELD`, `DONE`, and a task's finish critical
  section. The scheduler must run I-set so a tick can't split it.
- The **tick ISR is auto-I-set** (6502 sets I on IRQ entry) → non-reentrant by hardware.
- `RTI` to the next task restores its I-clear P → preemption re-enabled. So the *only*
  I-clear code is task bodies; all kernel/ISR/scheduler code runs I-set.

### Timer gateware (small, in `coproc.v`)
- `$E012` **TICK_PERIOD/arm** (write; `0` = disarmed; nonzero = tick every `period*256`
  clk cycles), `$E013` **TICK_ACK** (write clears `irq_pending`). `irq_pending` →
  `cpu .IRQ` (was tied 0).
- A counter while armed; at `period*256` it sets `irq_pending` (held = level IRQ) and
  reloads. The ISR acks (`$E013`) to clear it; it re-asserts next period. **On arm (`$E012`
  write nonzero) the counter resets to 0** so the first tick period is deterministic (not
  whatever phase it stalled at), and arm acks any stale pending (#9).
- **Disarmed until the kernel arms it** (in `BOOTSTRAP`, after seeding TCBs, right before
  starting task0) and **disarmed + pending-acked on `KIDLE`** (idle). **Ack-first on arm**
  (clear any stale pending). Minimum period enforced (≫ ISR time — see #10).

### Demo (proves preemption, not cooperation)
C3 racetask = a **tight countdown loop with NO `YIELD`** (`LDX NPARAM,Y` / `DEX` / `BNE`).
The tick slices the two → they interleave → smaller budget finishes first → re-arm flips.
The task caches its id in `Y` (read `CUR` once; `Y` preserved across the switch via TCB —
see #8). Finish: `SEI` / `ORDER` RMW / stamp `result[COREBASE+id]` / `JMP DONE` (the `SEI`
covers the RMW + stamp + the transition into `DONE`).

## The 14 robustness requirements (red-team → design)

**6502 / IRQ:**
1. **BRK shares `$FFFE`.** **Pinned ISR order** (a `TSX`/`LDA` clobbers A/X, so save them
   first — ZP stores leave `SP` untouched, keeping the `$0101,X` offset valid):
   ```
   ISR ($1F00):
     STA TMPA / STX TMPX / STY TMPY   ; save task A/X/Y FIRST (ZP; SP unchanged)
     TSX                              ; X = SP (= entry, orig-3); task X safe in TMPX
     LDA $0101,X                      ; stacked P
     AND #$10                         ; B bit
     BNE BRKLEG
     ; B=0 (real tick): ack + switch
     STA TICK_ACK (any) ; $E013       ; (A here = masked B bit; ack is value-agnostic; task A safe in TMPA)
     JMP SWITCH                       ; SWITCH stages from TMP* (A/X/Y already saved)
   BRKLEG:                            ; B=1 (stray BRK): NOT a tick
     LDA TMPA / LDX TMPX              ; restore A,X (TSX/LDA clobbered them; Y untouched)
     RTI                              ; resume the task; do NOT ack timer, do NOT switch
   ```
   "Stack untouched" is insufficient for BRK — registers must be restored too. `SWITCH`
   owns the authoritative `TSX` (the YIELD path's `SP` differs); the ISR's `TSX` is only
   for the B-check and is harmless to repeat (no stack push between).
2. **Tick during the `$E003` RDY-stall** is deferred until the post finishes (Arlet frozen
   while `rdy=0`; `irq_pending` is a held level) → SDRAM posts are atomic vs ticks. Verify
   in sim that Arlet services the pending IRQ after `rdy` returns.
3. **Arlet IRQ fidelity** — a focused sim check (assert IRQ → frame pushed → ISR at `$1F00`
   → switch → RTI) before the integration race.

**Scheduler integrity:**
4. **Scheduler runs on the task's stack.** Each `$30` partition must hold task depth + IRQ
   frame (3) + ISR saves + `PICKNEXT` `JSR` (2) ≈ 8 B kernel transient. Budget it; the sim
   adds a **max-stack-depth assert per partition** (fail if a task's SP crosses its
   partition floor). (No separate kernel stack — YAGNI for solo use.)
5. **`YIELD` `+1`** — manufacture the frame PC as a **16-bit** `JSR_return + 1` (carry into
   PCH). The kept C2 cooperative racetask test exercises the path but only at its fixed
   addresses — it won't hit the page-cross carry unless a `JSR YIELD` happens to sit at
   `$xxFF`. **Coverage caveat:** the impl must do the 16-bit `+1`; the carry leg isn't
   directly tested (acceptable — solo use).
6. **ISR/switch stack-balanced** per task (staging `PHA`/`PLA` net zero) so a task's own
   preempted `PHA`/`PLA` pair survives.

**Shared state:**
7. **`ORDER` RMW** wrapped `SEI` (covered by the finish critical section).
8. **Cache task id in `Y`** (read `CUR` once at task start; `Y` preserved across switches
   via TCB) — never re-read `CUR`, avoiding any window.

**Lifecycle / timer:**
9. **No stale tick:** disarm + **ack pending on `KIDLE`**, **ack before arm** in bootstrap.
10. **Period ≫ ISR+switch time** (~50–100 cyc). Enforce a minimum `period` (the `*256`
    scale already makes the smallest armed period 256 cyc — fine; document the floor).
11. **Re-arm-wait** (C2 carryover): host waits for all-DONE (→ `KIDLE` → disarmed →
    `KWAIT0`) before `COUNT 0→2`, or the kernel hangs in `KWAIT0`.

**Host bus vs tick:**
12. **Host load (port B) vs kernel stack write (port A) during a tick** — the single-write-
    port mux drops Arlet's write on collision. **Safe via count-last + timer-disarmed-
    during-load:** the host loads only while `COUNT=0` (kernel idle in `KWAIT0`, no BRAM
    writes) and the timer is disarmed then. Load-bearing — confirm the timer is disarmed
    throughout `KWAIT0`.

**Testing:**
13. **Wide budget margin** in the race tests (e.g. 50 vs 200) so the winner is unambiguous
    regardless of exact tick phase; the flip uses an equally clear gap.
14. **Preemption is a win:** a runaway non-yielding task can't hang the scheduler (it gets
    sliced). (It still blocks "all DONE" if it never terminates — a task bug, not a kernel
    hang; out of scope to guard.)

## Components / files (branch `coproc`)
- `coproc.v` — add the timer: `$E012` period/arm reg, `$E013` ack, `irq_pending` counter,
  wire `.IRQ(irq_pending)` (was `1'b0`). Reset → disarmed. **Confirm the gateware still
  synthesizes the IRQ vector `$FFFE/$FFFF → $1F00`** (C1's all-6-vectors DI mux:
  `is_irqlo_q→$00`, `is_irqhi_q→$1F`) — that's where the tick lands; without it the IRQ
  goes nowhere. (NMI `$FFFA/B→$1F40` + reset `$FFFC/D→$1000` unchanged.)
- `project_obscurus_top.v` — no change (timer is internal to `coproc`; CORE_ID inst stays).
- `kernel.S` — rework to the uniform frame: `RESTORE`(RTI, no P), `SWITCH`, `YIELD`(+1
  frame, SEI), `DONE`(SEI), `BOOTSTRAP`(seed `[PCH,PCL,$00]` exact entry, arm timer last,
  ack-first), `KIDLE`(disarm+ack), **TICK ISR — MUST be located at exactly `$1F00`** (the
  gateware IRQ vector; pad/`ORG` so the ISR entry is `$1F00`) doing the pinned order
  (ZP-save A/X/Y → `TSX` → B-check → BRK-leg restore+`RTI`, else ack+`JMP SWITCH`). Drop
  `TCB_P` (keep `TMPA/TMPX/TMPY` scratch). Re-bake `kernel.mem`. (Verify `$1F00` lands the
  ISR, like C1/C2 verified the API table — `xxd`/`sed` the offset.)
- `software/SDM/racetask3.S` — tight-loop (no-yield) race task; id cached in Y; finish
  `SEI`/RMW/stamp/`JMP DONE`. `CPRACE3.S` host loader (load, register 2, **arm via `$E012`
  is done by the kernel — host just sets COUNT**, race, re-arm flip). Disk.
- `project_obscurus_tb.v` — (a) focused IRQ check (#3), (b) C3 tight-loop race + flip (wide
  margins), (c) keep the C2 cooperative race (proves YIELD-uniform-frame), (d) stack-depth
  assert (#4), (e) a BRK-in-task check (#1: a task `BRK` doesn't corrupt the scheduler).

## Error handling / edge cases
- Covered by the 14 above. Additionally: **reset** disarms the timer (`irq_pending=0`,
  `period=0`) so no tick before the kernel; `nRES` mid-run → timer off + kernel re-boots to
  `KWAIT0`.
- A task that finishes (`JMP DONE`) never returns; `DONE` (I-set) switches to the next
  READY or `KIDLE` (disarm). No task ever executes its own `RTS`/`RTI` (would underflow) —
  finish is always `JMP DONE` (carried from C2).

## Testing
- **Sim — focused IRQ (#3):** arm the timer at a short period with one task; assert the ISR
  runs (a marker written) and the task resumes. iverilog `-g2005`.
- **Sim — C3 tight-loop race:** register 2 no-yield tasks (NPARAM 50/200), arm, assert
  `result[$0061]=01`/`[$0062]=02`; re-arm (250/200) → flip `02`/`01`.
- **Sim — C2 cooperative race still passes** (the YIELD-uniform path).
- **Sim — BRK safety (#1):** a task that executes `BRK` does not corrupt the scheduler (the
  other task still completes).
- **Sim — stack-depth assert (#4)** active during all of the above.
- **Build:** re-bake `kernel.mem`; DP16KD 8, timing PASS, timer adds a counter + compare.
- **Bench:** flash; `BRUN CPRACE3` → `01 02` then `02 01` (tasks never yield — pure
  preemption); the cooperative `CPRACE` still prints `01 02`/`02 01`.

## Success criteria
Two non-yielding tasks are preemptively interleaved by the timer tick, finish in a
host-observable order, and re-arming flips it — with the B-bit, no-stale-tick, stack-budget,
and disarm-during-load guards in place, and the C2 cooperative path still working. A true
(if minimal) preemptive RTOS on the soft-6502, without silent-corruption footguns.

## Non-goals
- No task priorities (round-robin only), no time-accounting/quotas, no per-task stack
  overflow *traps* (just the sim assert + budget), no separate kernel stack.
- No nested interrupts, no second IRQ source, no `CLI`-in-task fine-grained critical
  sections beyond the finish RMW.
- C-flash registry, multi-core instantiation — separate later rungs.
