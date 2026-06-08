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
Split on **bit 12**: `$1000–$1FFF` (`addr[12]==1`) is the protected kernel region.
**BOTH BRAM write ports are gated on `~addr[12]`** — this is the critical fix:
- **Port B (host load):** `CP_WDATA` write ignored when `laddr[12]==1`.
- **Port A (Arlet / running task):** the C0 core write `if (WE & in_bram & rdy)
  bram[AB[12:0]]<=DO` becomes `if (WE & in_bram & rdy & ~AB[12])`. Without this, a
  **task** doing `STA $1000` overwrites the kernel — port B alone does NOT stop a task
  (the actual threat). Arlet never legitimately writes `$1000–$1FFF`: kernel code is
  baked, kernel/ZP data lives at `$0000–$01FF` (`bit12==0`). So gating port A costs the
  task nothing and closes the real hole.

**COUNT is NOT a BRAM cell** — it is a register (see Host load port), so the one
variable touched concurrently by host and kernel never collides in the dual-port BRAM.
The TABLE region holds 128 entries exactly (`$0200`+2*127 = `$02FE/$02FF`, no spill).

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
2. `AB == $E010` → **COUNT register** (kernel reads the registered task count here).
3. `AB` in `$E000–$E003` → SDRAM write window (write-side; reads return don't-care).
4. `AB` in `$0000–$1FFF` → BRAM (port A).
5. anything else (`$2000–$DFFF`, `$E004–$E00F`, `$E011–$FFF9`) → **don't-care** (`$00`).
A correct task/kernel never reads the holes; this just pins behavior. The DI 1-cycle
registered alignment (C0) applies to all paths (vector/count/BRAM selects registered).

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

## Kernel firmware (`kernel.S`, Merlin, baked into the protected region)

```
RESET ($1000):
  init SP ($01FF), any kernel state
  ; COUNT is a coproc register, reset to 0 by gateware on POR — kernel need not clear it
MAINLOOP:
  LDX #0
NEXT:
  LDA $E010          ; COUNT register (read-only to the kernel)
  STX TMP            ; compare X to COUNT
  CMP TMP            ; A(count) - X
  BEQ MAINLOOP       ; X == count -> nothing more, restart loop
  ; (X < count: dispatch task_table[X])
  txa; asl; tay      ; Y = X*2 index into table
  lda TABLE,Y   / sta JVEC      ; TABLE = $0200
  lda TABLE+1,Y / sta JVEC+1
  jsr CALLVEC        ; CALLVEC: jmp (JVEC)
  inx
  jmp NEXT
```
COUNT lives in the `$E010` register (gateware-reset to 0 on POR; host bumps it via
`CP_COUNT`). Tasks are subroutines ending in `RTS` (return to the kernel). Cooperative:
each task runs to its `RTS` once per loop pass.

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
3. **Task-write protection (THE guarantee):** register a hostile task whose body is
   `LDA #$EE / STA $1000 / RTS`. After the kernel dispatches it, the host reads
   `$1000` via `CP_RDATA` → still the original kernel byte. This exercises the **port-A**
   gate — the test that actually proves "a task cannot corrupt the kernel/ISRs." (The
   load-port test alone passes even with the port-A hole open, so it is necessary but not
   sufficient.)

## Components / files (branch `coproc`)

- `coproc.v` — replace the synth-baked single program with: dual-port BRAM (port A
  Arlet, port B load; separate `always` blocks → DP16KD old-data-on-collision); `laddr`
  register + `CP_WDATA` write-autoinc (protect `laddr[12]`) + `CP_RDATA` read-autoinc
  (separate addresses — anti-double-bump); a `COUNT` register (POR-reset 0, host-written,
  Arlet reads at `$E010`); synthesized immutable vectors ($FFFC/D→$1000, $FFFE/F→$1F00,
  $FFFA/B→$1F40) with the decode priority above; the generalized `$E000–$E003` SDRAM
  write window (replacing C0's fixed `$0040` post); kernel-region BRAM init from
  `kernel.mem`; IRQ/NMI tied 0. C0's HALT/LOAD/RUN job-runner FSM is removed — the kernel
  is resident, Arlet runs from reset (keep the SDRAM-`ready` gate before the first post).
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
distinct SDRAM results the host reads; **both** a host load AND a running task's write
aimed at the kernel region (`$1000`) are refused by hardware (read-back unchanged). The
coproc is now a programmable mini-RTOS with a genuinely protected kernel — the foundation
for C2 (scheduling + the race).

## Non-goals (later rungs)

- **C2:** preemptive/yielding scheduling (timer-tick ISR using the protected vectors),
  task interleaving, the completion-order race, tune-to-flip-winner (the async proof).
- **C-flash:** persist registered tasks BRAM↔flash (survive boot); load kernel from flash.
- No task→task IPC, no coproc SDRAM *reads* for inputs (tasks compute from constants /
  their own code in C1), no host-settable kernel relocation. The kernel is fixed/baked.
