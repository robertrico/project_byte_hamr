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
$0200        task COUNT          (host-written, last; kernel reads)
$0201-$02FF  task TABLE          (entry ptrs, 2 bytes each — up to ~127 tasks)
$0300-$0FFF  TASK code region    (host-loaded, unprotected)
$1000-$1FFF  KERNEL code + ISRs  (synth-baked, WRITE-PROTECTED)
```
Split on **bit 12**: `laddr[12]==0` → `$0000–$0FFF` writable by the load port;
`laddr[12]==1` → `$1000–$1FFF` protected.

### Immutable vectors (synthesized in `coproc.v`, not in writable memory)
```
$FFFC/D RESET -> $1000  (kernel entry)
$FFFE/F IRQ   -> $1F00  (kernel IRQ ISR, in the protected region)
$FFFA/B NMI   -> $1F40  (kernel NMI ISR, in the protected region)
```
A task cannot repoint these — they are gateware constants. For C1 the IRQ/NMI lines are
tied 0 (no interrupts fire yet); the ISRs are `RTI` stubs. The vector + protected-region
infrastructure is built and tested in C1; the *preemptive timer tick* that uses them is
C2.

## Host load port (`$C0Cx`, 3 new registers in the free `$C0C9–CF` scratch space)

```
$C0C9  CP_LADDR_LO   W   low  8 bits of the 13-bit BRAM load address
$C0CA  CP_LADDR_HI   W   high 5 bits (laddr[12:8]); laddr[12] selects protected half
$C0CB  CP_LDATA      W   BRAM[laddr] <= data, then laddr++  (IGNORED if laddr[12]==1)
                     R   data = BRAM[laddr], then laddr++   (read-back to verify a load)
```
Drives **port B** of the dual-port BRAM. The write-protect (`laddr[12]==1` → write
ignored) is the hardware guarantee. Read-back is allowed for all addresses (debug/verify).

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
  ; do NOT clear COUNT here unless first-boot — see note
  ; (host registers AFTER the kernel has booted; kernel boots in us, host in seconds)
  clear COUNT=0 once at boot
MAINLOOP:
  LDX #0
NEXT:
  CPX COUNT          ; $0200
  BCS MAINLOOP       ; X >= count -> nothing more, restart loop
  ; dispatch task_table[X] (2-byte entry) via an indirect JSR trampoline
  txa; asl; tay      ; Y = X*2 index into table
  lda TABLE,Y   / sta JVEC
  lda TABLE+1,Y / sta JVEC+1
  jsr CALLVEC        ; CALLVEC: jmp (JVEC)
  inx
  jmp NEXT
```
Tasks are subroutines ending in `RTS` (return to the kernel). Cooperative: each task runs
to its `RTS` once per loop pass.

## Live registration protocol (lock-free, single-producer/single-consumer)

Host, via the load port (port B), in THIS ORDER:
1. Write the task **code** bytes → `$0300 + slot_offset`.
2. Write the task **entry ptr** (lo,hi) → `TABLE + 2*idx` (`$0201+`).
3. Write **COUNT = idx+1** → `$0200` **last**.

The kernel reads `COUNT` then dispatches `TABLE[0..COUNT-1]`. Because COUNT is bumped
last, a task is dispatched only once its code + entry are fully in place. A single-byte
COUNT write is atomic w.r.t. the kernel's read. No lock needed.

## Data flow

```
(boot) coproc reset -> kernel $1000 -> COUNT=0 -> MAINLOOP (dispatches nothing)
host loader (Merlin, on disk):
  for each task:
    POKE CP_LADDR=$0300.. ; POKE CP_LDATA=<task byte> ...   (load code, port B)
    POKE CP_LADDR=TABLE+2*idx ; POKE entry lo/hi
    POKE CP_LADDR=$0200 ; POKE COUNT=idx+1                  (register, last)
kernel: sees COUNT>0 -> JSR task -> task sets SADDR/SBANK, STA SDATA -> SDRAM write
host: monitor R <cell> -> the task's result
```

## The proof

1. **Register + dispatch:** host registers task A (`writes $99 to SDRAM bank0 $0050`) →
   `R 0050` = `99`. Register task B (`writes $77 to bank0 $0051`) → `R 0050`=`99`,
   `R 0051`=`77`. Both registered fns run under the resident kernel. "fn that runs" +
   "add fns" proven.
2. **Protection:** host loads a byte to `CP_LADDR=$1000`, `CP_LDATA=$EE`; read-back of
   `$1000` returns the original kernel byte (write refused). A task cannot corrupt the
   kernel/ISRs.

## Components / files (branch `coproc`)

- `coproc.v` — replace the synth-baked single program with: dual-port BRAM (port A
  Arlet, port B load), `laddr` register + write-protect (`laddr[12]`), read-back;
  synthesized immutable vectors ($FFFC/D→$1000, $FFFE/F→$1F00, $FFFA/B→$1F40); the
  generalized `$E000–$E003` SDRAM write window (replacing the fixed `$0040` post);
  kernel-region BRAM init from `kernel.mem`; IRQ/NMI tied 0. The HALT/LOAD/RUN job-runner
  FSM is removed — the kernel is resident, Arlet runs from reset.
- `kernel.S` → `kernel.mem` — the baked kernel (boot + dispatch loop + ISR stubs),
  assembled to the `$1000–$1FFF` image (Makefile rule, in synth graph).
- `project_obscurus_top.v` — decode `CP_LADDR_LO/HI`/`CP_LDATA` ($C0C9–CB) → drive
  coproc port B (addr/data/we + read-back data to the register read mux).
- `software/SDM/` — host loader (Merlin, e.g. `CPREG.S`) that loads + registers task(s)
  + reads results + runs the protection test; two tiny task binaries (`taskA.S`,
  `taskB.S`) authored in Merlin, their bytes carried by / loaded via the loader.
- `coproc_tb.v` / `project_obscurus_tb.v` — unit + integration: load a task via port B,
  register it, confirm the kernel dispatches it and the SDRAM result appears; confirm a
  `$1000` load write is refused (read-back unchanged).

## Error handling / edge cases

- **Boot ordering:** kernel clears COUNT once at boot (µs after reset); host registers
  seconds later — no race. If a re-register happens, host bumps COUNT to the new total.
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
  BRAM, run, assert the SDRAM result; assert the protection refusal. iverilog `-g2005`.
- **Bench:** flash; boot; run the Merlin loader; `R 0050`/`R 0051` show both task
  results; the protection read-back shows the kernel byte intact.

## Success criteria

Two host-registered Merlin tasks run under a resident, never-halting kernel and write
distinct SDRAM results the host reads; a load aimed at the kernel region is refused by
hardware. The coproc is now a programmable mini-RTOS with a protected kernel — the
foundation for C2 (scheduling + the race).

## Non-goals (later rungs)

- **C2:** preemptive/yielding scheduling (timer-tick ISR using the protected vectors),
  task interleaving, the completion-order race, tune-to-flip-winner (the async proof).
- **C-flash:** persist registered tasks BRAM↔flash (survive boot); load kernel from flash.
- No task→task IPC, no coproc SDRAM *reads* for inputs (tasks compute from constants /
  their own code in C1), no host-settable kernel relocation. The kernel is fixed/baked.
