# Coprocessor C1 — Resident Kernel + Task Registration (design)

**Date:** 2026-06-08
**Branch:** `coproc` (continues from C0, bench-verified `0040: 42`)
**Board:** Byte Hamr Rev 2, `gateware/rev2/project_obscurus/` + `software/SDM/`
**Status:** approved-in-discussion, pending spec review
**Builds on:** [[project_coproc_c0]] — Arlet soft-6502 + 8KB BRAM + SDRAM via the
2-client arbiter, proven on silicon.

## Why this exists

The goal is a **mini-RTOS on the coprocessor**: a resident "fn that runs" (a kernel)
that you **add fns to** (register tasks), so you can eventually run two Merlin-authored
tasks concurrently and race them (C2 = the async proof). C0 ran a single synth-baked
program to completion — a job-runner, not an RTOS. C1 turns the coproc into a
**resident kernel with live task registration**, with the kernel + interrupt handlers
in a **hardware-protected** region a task cannot corrupt.

The protection is the same principle proven in the hypervisor detour (immutable vectors
+ a region untrusted code can't touch — [[project_obscurus_hypervisor]] 5c-2b), reused
here for the coproc kernel.

## Goal

A baked kernel boots on the coproc and loops forever. The host **registers** a task
(loads its code into the task region + adds its entry to the kernel's table); the kernel
dispatches it (`JSR`); the task writes a result to SDRAM; the host reads it. Registering
a second task → both run. A load write aimed at the kernel region is **refused by
hardware**. Success = both registered tasks' results appear in SDRAM, and the protection
test shows the kernel byte unchanged.

## Architecture — three memories, one job each

- **BRAM = code.** True dual-port (ECP5 DP16KD). **Port A = Arlet** (kernel + tasks
  execute). **Port B = host load port.** The kernel **never halts** — tasks load *live*
  on port B while the kernel runs on port A.
- **SDRAM = runtime data/result mailbox** (tasks write results via the coproc's SDRAM
  window; host reads via the monitor port). Unchanged from C0 except the window gains an
  address.
- **Flash = persistence** — deferred to the C-flash rung (registry that survives boot).

### BRAM map (8 KB, `$0000–$1FFF`)
```
$0000-$00FF  ZP
$0100-$01FF  stack
$0200-$02FF  task TABLE          (entry ptrs, 2 bytes each — up to 128 tasks, idx 0..127)
$0300-$0FFF  TASK code region    (host-loaded, unprotected)
$1000-$1FFF  KERNEL code + ISRs  (synth-baked, WRITE-PROTECTED)
```
**Protection is two-axis (per-port × per-region) — one shared bit cannot express it.**
The TABLE (`$0200–$02FF`) must be **host-writable** (registration) yet **task-blocked**
(a task `STA $0200` would repoint a dispatch entry → hijack the kernel). Same addresses,
opposite rules → two distinct gates:

| Region | Port B (host load) | Port A (running task) |
|---|---|---|
| `$0000–$01FF` ZP/stack | (host doesn't write) | **allow** — shared runtime (cooperative) |
| `$0200–$02FF` TABLE | **allow** (register) | **BLOCK** (anti-hijack) |
| `$0300–$0FFF` task code | **allow** (load) | allow (user space) |
| `$1000–$1FFF` kernel/ISR | **BLOCK** | **BLOCK** |

- **Port B (host load):** `CP_WDATA` write ignored when `laddr[12]==1` (kernel only).
- **Port A (Arlet/task):** C0's `if (WE & in_bram & rdy) bram[AB[12:0]]<=DO` becomes
  `if (WE & in_bram & rdy & ~AB[12] & ~(AB[11:8]==4'h2))` — blocks **both** the kernel
  (`$1000+`) and the TABLE (`$0200–$02FF`). A task can still write its own region
  (`$0300–$0FFF`) and the shared ZP/stack.

**Control plane vs data plane (honest boundary).** Hardware-protected from tasks: kernel
**code+ISRs**, the **dispatch TABLE**, the **vectors** (gateware constants), and the
**COUNT register** (`$E010` is *read-only* to Arlet — a task `STA $E010` is a no-op). So a
task **cannot** alter kernel code, repoint dispatch, change the vectors, or bump the task
count → it cannot hijack kernel control flow. *Cooperative (shared, trusted):* ZP/stack
(`$0000–$01FF`), the task-code region (`$0300–$0FFF`), and SDRAM data — a misbehaving task
can corrupt its own/another task's data or the shared ZP/stack. **Per-task ZP/stack
isolation (separate context) is a C2+ feature**, not C1. C1's guarantee: the kernel's
*control structures* are hardware-protected.

**COUNT is a register, not BRAM** (see Host load port), so the one host/kernel-shared var
never collides in the dual-port BRAM. The TABLE holds 128 entries exactly
(`$0200`+2*127 = `$02FE/$02FF`, no spill).

### Dual-port BRAM collision mode (load-bearing)
Port A (Arlet) and port B (load) are inferred as one read + one write each into a
single reg-array, **in separate `always @(posedge clk)` blocks** so Yosys maps them to
ECP5 `DP16KD` with **read-returns-OLD-data on a same-address collision** (no
write-through, no `X`). The TABLE/code region is published *before* it is read
(count-last barrier), so a live same-cell read-during-write never happens for a
dispatched task; the old-data mode is the belt-and-suspenders guarantee. The sim BRAM
model must mirror this (read old value on collision, never `X`).

### Immutable vectors (synthesized in `coproc.v`, not in writable memory)
```
$FFFC/D RESET -> $1000  (kernel entry)
$FFFE/F IRQ   -> $1F00  (kernel IRQ ISR, in the protected region)
$FFFA/B NMI   -> $1F40  (kernel NMI ISR, in the protected region)
```
A task cannot repoint these — they are gateware constants. **All 6 vector bytes
(`$FFFA–$FFFF`) MUST be decoded** — not just reset (C0 only did `$FFFC/D`). The IRQ/NMI
lines are tied 0 (no async interrupt fires in C1), but a task executing **`BRK`** reads
`$FFFE/F`; if those return `$00` the core jumps to `$0000` (ZP) and derails the kernel.
So `$FFFE/F→$1F00` and `$FFFA/B→$1F40` are implemented even though no line is wired, with
`RTI` stubs at `$1F00/$1F40`. The preemptive timer tick that *uses* IRQ is C2.

### Coproc address decode (Arlet side) — priority + holes
The coproc read mux resolves Arlet's `AB` in this priority:
1. `AB` in `$FFFA–$FFFF` → the **6 vector constants** above (ahead of everything).
2. `AB == $E010` → **COUNT register**, **read-only to Arlet** (a task `STA $E010` is
   ignored; COUNT is host-owned via `CP_COUNT`).
3. `AB` in `$E000–$E003` → SDRAM write window (write-side; reads return don't-care).
4. `AB` in `$0000–$1FFF` → BRAM (port A), writes gated by the port-A protection above.
5. anything else (`$2000–$DFFF`, `$E004–$E00F`, `$E011–$FFF9`) → **don't-care** (`$00`).
A correct task/kernel never reads the holes; this just pins behavior. The DI 1-cycle
registered alignment (C0) applies to all paths (vector/count/BRAM selects registered).
**`$E000–$E003` has NO read-autoinc**, so a task may write the SDRAM window with any
addressing mode — an indexed `STA $E003,X` dummy-reads `$E003` (`WE` low → no post) then
writes once (one post). Only the *host* `$C0CB` carries the double-post hazard (it has a
write-autoinc; use non-indexed `STA $C0CB`).

## Host load port (`$C0Cx`, 5 registers in the free `$C0C9–CF` scratch space)

```
$C0C9  CP_LADDR_LO   W   low  8 bits of the 13-bit BRAM load address
$C0CA  CP_LADDR_HI   W   high 5 bits (laddr[12:8]); laddr[12] selects protected half
$C0CB  CP_WDATA      W   BRAM[laddr] <= data, then laddr++   (IGNORED if laddr[12]==1)
$C0CC  CP_RDATA      R   data = BRAM[laddr], then laddr++    (read-back to verify)
$C0CD  CP_COUNT      W   registered task count (the release barrier; kernel reads at $E010)
```
Drives **port B** of the dual-port BRAM. Write-protect (`laddr[12]==1` → write ignored)
is the hardware guarantee. `CP_COUNT` is a register, not BRAM (see collision note).

**SEPARATE write-autoinc and read-autoinc addresses — mandatory (CLAUDE.md rule #1).**
`STA abs,X` does a *dummy read of the target* (an `R_nW=1` pulse) on cycle 4 before the
write on cycle 5 — so a single address that auto-increments on BOTH read and write would
**double-bump `laddr`** and corrupt the load (a bug this project has hit repeatedly). C1
splits the paths exactly like the monitor's `TRIG_RD`/`DATA`: **`CP_WDATA` ($C0CB)
auto-increments only on WRITE-commit (`R_nW=0`); `CP_RDATA` ($C0CC) auto-increments only
on READ.** The `STA $C0CB`/`STA $C0CB,X` dummy read hits `$C0CB` *as a read* — which has
no read-autoinc — so it cannot bump. Read-back uses `$C0CC` (its own read-autoinc). The
loader SHOULD still prefer non-indexed `STA $C0CB` / `LDA $C0CC` (4-cycle, single pulse),
but the split makes it safe even under indexed addressing.

**Read-back latency — port B read is REGISTERED (use the monitor's DATA pattern).** The
dual-port BRAM read is 1-cycle registered (like Arlet's port A), but the host register
read mux is combinational same-cycle — naive wiring returns stale data. Continuously
latch `ldata_q <= bram_portB[laddr]`; the `CP_RDATA` ($C0CC) read returns `ldata_q`, then
post-increments `laddr`. The new `ldata_q` settles in ~40 ns (1 clk @ 25 MHz) — far
inside the next ~1 µs 6502 bus cycle — so the *next* `LDA $C0CC` sees the right byte.
Same class of fix as the monitor's sticky-busy/DATA latch.

## Coproc SDRAM write window (generalize C0's fixed `$E000`)

In Arlet's address space (not `$C0Cx`):
```
$E000  SADDR_LO   write -> latch SDRAM target addr low
$E001  SADDR_HI   write -> latch addr high
$E002  SBANK      write -> latch bank low (bank<256 enough for C1)
$E003  SDATA      write -> post SDRAM write {SBANK,SADDR}=data; RDY-stall till done
```
A task sets `SADDR/SBANK` then `STA SDATA` to write a result anywhere in SDRAM (so two
tasks write distinct cells). RDY-stall + single-post handshake as C0's `$E000`.

**Build-it-exactly (Arlet `WE` is held HIGH for the whole stall).** Arlet's `WE` is
`always @*` off state, **not gated by `RDY`** — when the coproc drops `rdy<=0` on a
`STA SDATA`, Arlet freezes in the WRITE state with `WE=1` held for the entire stall. So:
- **Address/bank latches** capture on `is_e000/1/2 & WE & rdy` — the `& rdy` is mandatory:
  without it, a held `WE` re-latches `SADDR/SBANK` every stall cycle of a *later* post.
- **`$E003` post** transitions state on the first `is_e003 & WE` (in the RUN state, where
  `rdy==1`) → drop `rdy`, post exactly one SDRAM op, WAIT for done, then `rdy<=1`. The
  state transition (not just `WE`) is what guarantees a single post despite held `WE`.
- This is the same reason C0 gates its BRAM write `& rdy` and leaves `ST_RUN` on first
  detect. The **port-A protection gate also keeps `& rdy`** (its `WE & in_bram & rdy &
  ...`) so a held `WE` during a stall can't re-fire it.

## Kernel firmware (`kernel.S`, Merlin, baked into the protected region)

```
RESET ($1000):
  init SP ($01FF), any kernel state
  ; COUNT is a coproc register, reset to 0 by gateware on POR — kernel need not clear it
MAINLOOP:
  LDX #0
NEXT:
  CPX $E010          ; X vs COUNT register (CPX abs reads $E010; carry set if X>=count)
  BCS MAINLOOP       ; X >= count -> nothing more, restart loop
  TXA
  ASL                ; A = X*2 entry offset (X<=127 -> no overflow)
  TAY
  LDA $0200,Y / STA JVEC       ; TABLE base $0200
  LDA $0201,Y / STA JVEC+1
  TXA
  PHA                ; *** SAVE index on the stack — survives the task's RTS ***
  JSR CALLVEC        ; CALLVEC: JMP (JVEC). Task may trash A/X/Y; must balance its stack.
  PLA
  TAX                ; *** RESTORE index ***
  INX
  JMP NEXT
```
**Index lives on the stack across the call, not in X/ZP** — a task is arbitrary code that
trashes A/X/Y; keeping the loop counter in X (`inx` after the `JSR`) would index off
garbage and derail dispatch. `JVEC` (ZP) is set fresh each iteration and consumed by
`CALLVEC` (`JMP (JVEC)`) *before* the task can touch it. `CPX $E010` reads the COUNT
register (no ZP temp). COUNT is gateware-reset to 0 on POR; host bumps it via `CP_COUNT`.
Tasks are subroutines ending in `RTS`. Cooperative: each runs to its `RTS` once per pass.

## Live registration protocol (lock-free, single-producer/single-consumer)

Host, in THIS ORDER:
1. Write the task **code** bytes → BRAM `$0300 + slot_offset` (via `CP_LADDR` + `CP_WDATA`).
2. Write the task **entry ptr** (lo,hi) → BRAM `TABLE + 2*idx` (`$0200+`, via the load port).
3. Write `CP_COUNT` (`$C0CD`) = `idx+1` **last** (the register, the release barrier).

The kernel reads `COUNT` (the `$E010` register) then dispatches `TABLE[0..COUNT-1]`.
Because `CP_COUNT` is bumped last, a task is dispatched only once its code + entry are
fully in BRAM. COUNT being a register (not a BRAM cell) means the one host/kernel-shared
variable never hits a dual-port collision; a single-byte register read returns old-or-new
(both safe — old → picked up next pass), atomic in the single 25 MHz clock domain. No lock.

## Data flow

```
(boot) coproc reset -> kernel $1000 -> COUNT=0 -> MAINLOOP (dispatches nothing)
host loader (Merlin, on disk):
  for each task:
    set CP_LADDR=$0300.. ; STA CP_WDATA per byte ...        (load code, port B)
    set CP_LADDR=$0200+2*idx ; STA CP_WDATA entry lo/hi
    STA CP_COUNT = idx+1                                    (register, last)
kernel: reads COUNT@$E010 > 0 -> JSR task -> task sets SADDR/SBANK, STA SDATA -> SDRAM write
host: monitor R <cell> -> the task's result
```

## The proof

1. **Register + dispatch:** host registers task A (`writes $99 to SDRAM bank0 $0050`) →
   `R 0050` = `99`. Register task B (`writes $77 to bank0 $0051`) → `R 0050`=`99`,
   `R 0051`=`77`. Both registered fns run under the resident kernel. "fn that runs" +
   "add fns" proven.
2. **Load-port protection:** host sets `CP_LADDR=$1000`, `STA CP_WDATA=$EE`; then
   `CP_LADDR=$1000`, `LDA CP_RDATA` returns the original kernel byte (write refused).
3. **Task-write protection (THE control-plane guarantee):** register a hostile task whose
   body is `LDA #$EE / STA $1000 / STA $0200 / RTS`. After the kernel dispatches it, the
   host reads `$1000` (kernel) AND `$0200` (TABLE entry 0 lo) via `CP_RDATA` → **both**
   still hold their original bytes. This exercises the **port-A** gate on *both* protected
   regions — the test that actually proves "a task cannot corrupt the kernel code or
   repoint dispatch." (The load-port test alone passes even with the port-A hole open, so
   it is necessary but not sufficient.) Also assert a task `STA $E010` leaves COUNT
   unchanged (count register read-only to Arlet).

## Components / files (branch `coproc`)

- `coproc.v` — replace the synth-baked single program with: dual-port BRAM (port A
  Arlet, port B load; separate `always` blocks → DP16KD old-data-on-collision);
  **port-A write gate** `WE & in_bram & rdy & ~AB[12] & ~(AB[11:8]==4'h2)` (block kernel
  `$1000+` AND TABLE `$0200–$02FF`); **port-B write gate** `CP_WDATA & ~laddr[12]`;
  `CP_WDATA` write-autoinc + `CP_RDATA` read-autoinc (separate addresses — anti-double-bump)
  with the registered-read `ldata_q` latch; a `COUNT` register (POR-reset 0, host-written
  via `CP_COUNT`, Arlet reads `$E010` read-only); synthesized immutable vectors — **all 6**
  ($FFFC/D→$1000, $FFFE/F→$1F00, $FFFA/B→$1F40) per the decode priority; the generalized
  `$E000–$E003` SDRAM write window (replacing C0's fixed `$0040` post); kernel BRAM init
  from `kernel.mem` at offset `$1000`; IRQ/NMI tied 0. C0's HALT/LOAD/RUN FSM is removed —
  the kernel is resident, Arlet runs from reset (keep the SDRAM-`ready` gate before the
  first post).
- `kernel.S` → `kernel.mem` — the baked kernel (boot + dispatch loop + ISR stubs at
  `$1F00/$1F40`), Merlin `ORG $1000`. **Real `.S→.mem` Makefile target in the synth
  graph** (retire `coproc_prog.mem`; the "hamr_rom.mem stale for weeks" lesson — verify
  the artifact is actually consumed by synth). `$readmemh` loads from index 0, so EITHER
  emit a full 8 KB padded image (zeros `$0000–$0FFF`, kernel `$1000+`) OR
  `$readmemh("kernel.mem", bram, 13'h1000)` with a kernel-only `.mem`. Verify `bram[$1000]`
  = the kernel's first opcode after build.
- `project_obscurus_top.v` — decode `CP_LADDR_LO/HI` ($C0C9/CA), `CP_WDATA` ($C0CB),
  `CP_RDATA` ($C0CC), `CP_COUNT` ($C0CD) → drive coproc port B + the count register;
  route `CP_RDATA` read-back into the register read mux. `CP_WDATA` auto-inc fires only on
  the write-commit (`reg_wr`); `CP_RDATA` auto-inc only on its read strobe.
- `software/SDM/` — host loader (Merlin, e.g. `CPREG.S`) that loads + registers task(s)
  + reads results + runs the protection test; two tiny task binaries (`taskA.S`,
  `taskB.S`) authored in Merlin, their bytes carried by / loaded via the loader.
- `coproc_tb.v` / `project_obscurus_tb.v` — unit + integration: load a task via port B,
  register it, confirm the kernel dispatches it and the SDRAM result appears; confirm a
  `$1000` load write is refused (read-back unchanged).

## Error handling / edge cases

- **Boot ordering:** the `COUNT` register is gateware-reset to 0 on POR (µs after
  config); the host registers seconds later — no race, and the kernel never writes COUNT.
  Re-register = host bumps `CP_COUNT` to the new total.
- **A-read / B-write collision:** the kernel (port A) only reads a task's code after its
  COUNT is bumped (post-load); the task being loaded isn't dispatched yet — no same-cell
  read-during-write on live tasks.
- **Protected write:** `laddr[12]==1` writes are dropped silently; read-back lets the
  host verify (and detect a mis-aimed load).
- **Arlet sync memory:** port A keeps the registered 1-cycle DI path from C0 (Arlet
  expects `DI[n]=mem[AB[n-1]]`); the dual-port array must preserve that timing.
- **Re-dispatch / write storm:** `MAINLOOP` re-runs every registered task every pass,
  forever. C1's tasks write a **constant** ($99/$77), so the host reads the same value
  regardless of timing — proof holds. **C2 must add run-once / yield** (a task writing
  varying data would be re-run continuously, and a host read could catch a cell
  mid-update). Out of scope for C1; noted so C2 doesn't inherit a silent storm.
- **Full vs minimum protection (deliberate):** the port-A gate blocks the TABLE
  (`$0200–$02FF`) in addition to the kernel (`$1000+`) — one extra comparator term. Kept
  (not minimized to kernel-only) because a **buggy** task with a wild `STA` into
  `$0200–$02FF` would silently repoint dispatch and derail the kernel; the term costs
  nothing legitimate (tasks never write the TABLE). Claim, mechanism, and the proof tests
  ($1000 + $0200 + $E010) are all at this level — no claim > mechanism.
- **Reset/POR + SDRAM ready:** keep C0's gate — the coproc must not issue SDRAM writes
  until `sdram_ctrl.ready` (the ST_BOOT lesson); a task's first `STA SDATA` stalls on
  RDY until the controller is live.

## Testing

- **Sim (unit):** port B write/read-back + protection (`$1000` write ignored); the
  SDRAM-window post.
- **Sim (integration):** drive the load port to register a task into a kernel-loaded
  BRAM, run, assert the SDRAM result; assert BOTH protection refusals — load-port write to
  `$1000` AND a dispatched task's `STA $1000` (port-A gate). iverilog `-g2005`.
- **Build/EBR check:** after synth, confirm the synth report maps the coproc 8 KB BRAM to
  **EBR (DP16KD)**, NOT distributed LUTs — adding port B (second `always` block) can defeat
  DP inference and explode to LUTRAM. Pin the DP16KD coding style so a port-A/port-B
  same-address same-cycle collision returns **old data, not `X`** (the TABLE/code region's
  publish-before-read makes live collisions unlikely, but the mode is the guarantee). Report
  EBR count (C0 was 4).
- **Bench:** flash; boot; run the Merlin loader; `R 0050`/`R 0051` show both task results;
  the load-port AND task-write read-backs show the kernel byte intact.

## Success criteria

Two host-registered Merlin tasks run under a resident, never-halting kernel and write
distinct SDRAM results the host reads; a running task's writes to **both** the kernel
region (`$1000`) and the dispatch TABLE (`$0200`) are refused by hardware (read-back
unchanged), a task `STA $E010` leaves COUNT unchanged, and a host load to `$1000` is
refused. The coproc is a programmable mini-RTOS whose **control plane** (kernel code,
ISRs, vectors, dispatch table, count) is hardware-protected from tasks — the foundation
for C2 (scheduling + the race; per-task data isolation also lands there).

## Non-goals (later rungs)

- **C2:** preemptive/yielding scheduling (timer-tick ISR using the protected vectors),
  task interleaving, the completion-order race, tune-to-flip-winner (the async proof).
- **C-flash:** persist registered tasks BRAM↔flash (survive boot); load kernel from flash.
- No task→task IPC, no coproc SDRAM *reads* for inputs (tasks compute from constants /
  their own code in C1), no host-settable kernel relocation. The kernel is fixed/baked.
