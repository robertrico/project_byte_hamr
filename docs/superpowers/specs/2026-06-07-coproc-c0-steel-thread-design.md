# Coprocessor C0 — Steel Thread: Arlet soft-6502 writes SDRAM, host reads (design)

**Date:** 2026-06-07
**Branch:** `coproc` (forked from `4b34c99`, the clean SDRAM-monitor substrate — no hypervisor)
**Board:** Byte Hamr Rev 2, `gateware/rev2/project_obscurus/` (slot 4) + `software/SDM/`
**Status:** approved-in-discussion, pending spec review

## Why this exists

The real goal (stated by the user): boot Merlin → author/assemble/**register two 6502
projects** → run them **concurrently** on a soft CPU → watch which finishes first →
tune one to flip the winner → **prove async work**. That needs a **soft-6502
coprocessor** (Arlet `verilog-6502`) running *in parallel* with the host, computing
into SDRAM. The host posts jobs + reads results via the existing `$C0Cx`/SDRAMLIB
channel.

The whole feature hangs off one foundational proof. **C0 is that steel thread:** get
the Arlet core synthesized + clocked on the ECP5, running a fixed program, writing a
byte to SDRAM that the host can read. No job-loading, no kernel, no concurrency yet —
just prove the core lives and shares SDRAM. Every later rung (C1 load, C2 race) builds
on this.

(The hypervisor work 5a–5c — suspend/resume a booted disk — is preserved on branch
`obscurus-sdram-monitor`. Separate capability; out of scope here.)

## Goal

Release reset → the Arlet soft-6502 runs a fixed BRAM program → it writes `$42` to
SDRAM bank 0, address 0 → the **host reads bank0/addr0 via the monitor (`R 0000`) and
sees `$42`.** Success = that read returns `$42` in sim *and* on the bench.

## Architecture

```
        +-- monitor (host $C0Cx) -----+
        |                              +-- sdram_arb (NEW) -- sdram_ctrl -- SDRAM
   coproc.v (Arlet 6502 + BRAM) ------+        (2-client)      (existing)
```

All in the existing single **25 MHz** `clk` domain (`CLK_100MHz/4`) — no CDC, matching
the project's bench-proven single-clock approach.

### 1. `coproc.v` — Arlet core + BRAM + SDRAM window

- Instantiates the vendored Arlet `verilog-6502` core (`cpu.v` + `ALU.v`).
- **8 KB BRAM** (`$0000–$1FFF`) holds the coprocessor's ZP, stack, and program;
  initialized via `$readmemh("coproc_prog.mem")`.
- **Address decode** on the Arlet bus (`AB`, `DI`, `DO`, `WE`, `RDY`):
  - `AB` in `$0000–$1FFF` → BRAM (read: registered 1-cycle; write: `WE` into BRAM).
  - `AB == $FFFC` → `DI = $00`, `AB == $FFFD` → `DI = $02` (synthesized **reset
    vector → `$0200`**; the program lives at `$0200`).
  - `AB == $E000` & `WE` → **post an SDRAM write** (fixed bank 0, addr 0, data = `DO`)
    to the coproc's arbiter client; hold Arlet's **`RDY` low** until the SDRAM op
    completes, then release (so a multi-cycle SDRAM access can't be lost).
  - else → `DI = $00`.
- Exposes a clean **SDRAM-client interface** (`req`/`we`/`phys_addr`/`wdata` out,
  `grant`/`busy`/`rdata` in) to the arbiter. Self-contained + instantiable (more cores
  later = instantiate again).
- Arlet reset is active-high; drive it from the top's `~rst_n` (held during POR).

### 2. `sdram_arb.v` — fresh 2-client priority arbiter

- Inputs: two client request ports (c0 = monitor, c1 = coproc), each
  `req`/`we`/`phys_addr[25:0]`/`wdata[7:0]`. Output: the muxed request to `sdram_ctrl`
  + per-client `grant` and a shared `rdata`/`busy`.
- **Priority: monitor (c0) > coproc (c1)** — the host is interactive; the coproc can
  wait a cycle. A client holds its request until granted + the op completes (`busy`
  fall), then drops.
- **N-client-shaped**: adding a 3rd/Nth client = another port + another priority slot,
  no rewrite. (This is NOT the old mirror-shaped arbiter; built fresh for this purpose.)
- Combinational grant + registered hold-latch, mirroring `sdram_ctrl`'s `req`/`busy`
  contract.

### 3. `sdram_ctrl` (existing, unchanged) + monitor rewire

- `sdram_ctrl` keeps its `req`/`we`/`phys_addr`/`wdata`/`rdata`/`busy`/`ready`
  contract. The arbiter sits in front of it.
- The monitor's existing `m_req`/`m_we`/address/`wdata` path is **rewired to the
  arbiter's c0 port** instead of directly to `sdram_ctrl`. `op_done` (busy-fall) and
  the sticky-busy STATUS logic are unchanged (the arbiter preserves the `busy`
  semantics the monitor depends on).

### 4. `coproc_prog.S` — the proof program (Merlin32)

```
 ORG $0200
START LDA #$42
 STA $E000      ; posts SDRAM write -> bank0 addr0 = $42
LOOP JMP LOOP    ; spin forever
```
Assembled by Merlin32 → `coproc_prog.bin` → `coproc_prog.mem` (8 KB BRAM image,
program bytes at offset `$0200`). The reset vector is supplied by the `coproc.v`
decode (`$FFFC/D → $0200`), so no vector bytes are needed in the image.

## Data flow

1. Top releases `rst_n` after POR → Arlet reset deasserts → PC = `$0200`.
2. Arlet executes `LDA #$42` (BRAM), then `STA $E000`.
3. `coproc.v` decodes `$E000`+`WE`: latches `{bank0, addr0, $42}`, raises its arbiter
   `req`, drops Arlet `RDY` low.
4. Arbiter grants c1 (no monitor traffic) → `sdram_ctrl` writes `$42` to phys addr 0.
5. `busy` falls → coproc clears its `req`, releases `RDY` → Arlet resumes (spins).
6. Host: monitor `R 0000` → arbiter c0 read → `sdram_ctrl` returns `$42`.

## Components / files (all on `coproc` branch)

- **Vendor:** `gateware/rev2/project_obscurus/coproc/arlet/cpu.v`, `.../ALU.v`
  (github.com/Arlet/verilog-6502 — open/free-to-use; keep the author's header +
  add a `VENDORED.md` noting source + commit + attribution).
- **New:** `coproc.v`, `sdram_arb.v` (+ their `_tb.v`), `coproc_prog.S`,
  `coproc_prog.mem`.
- **Modify:** `project_obscurus_top.v` (instantiate `coproc` + `sdram_arb`, rewire
  monitor through the arbiter, drive Arlet reset from `~rst_n`),
  `project_obscurus_tb.v` (the C0 proof check), `Makefile` (assemble `coproc_prog.S` →
  `coproc_prog.mem` rule, in the synth build graph alongside the existing ROM rules).

## Error handling / edge cases

- **Reset / POR:** Arlet held in reset until `rst_n` (POR ~16 clk). No SDRAM activity
  until the controller's own init (`sdram_ctrl` ST_INIT) completes — the arbiter must
  not grant before `sdram_ctrl.ready`. Coproc's first `STA $E000` may stall on `RDY`
  until the controller is ready; that's correct.
- **Arbiter contention:** if monitor and coproc request the same cycle, monitor wins;
  coproc holds `req` (Arlet stays `RDY`-stalled) until granted. No lost ops.
- **RDY semantics (CONFIRMED from Arlet `cpu.v`):** `RDY` is a **global clock-enable** —
  `RDY=0` freezes *all* register updates on **both read and write** cycles, and
  `AB`/`DO`/`WE` **hold stable** (combinational off the frozen state). So the
  `$E000` write-stall is: on `AB==$E000 & WE & !posted`, latch `{bank0,addr0,DO}`,
  pulse the arbiter `req` **once**, set `posted`, hold `RDY=0`; when `busy` falls,
  release `RDY=1` and clear `posted` (Arlet then advances past the `STA`,
  deasserting `$E000`/`WE`). The single `req` pulse prevents a double write while
  stalled. Arlet **reset is active-high** (drive from `~rst_n`).
- **BRAM read latency:** Arlet expects `DI` the cycle after `AB` (synchronous). ECP5
  BRAM registered read matches; the decode mux for vector/`$E000`/else must align to
  the same 1-cycle latency (register the decoded `DI`).

## Testing

- **Unit (sim):** `sdram_arb_tb.v` — two clients; monitor-priority; held requests;
  no lost ops; `busy` contract preserved.
- **Integration (sim):** `project_obscurus_tb.v` — release reset, run the SDRAM model
  long enough for Arlet to execute `LDA/STA`, then drive a monitor `R 0000` and assert
  the returned byte `=== $42`. Also assert the monitor still reads/writes normally with
  the arbiter in front (no regression).
- **Bench:** flash; boot; in the `$C800` monitor type `R 0000` → `0000: 42`. (Or BRUN a
  one-line reader.) Proves Arlet runs on silicon + shares SDRAM + host sees output.
  iverilog **`-g2005` only** (project lesson).

## Success criteria

`R 0000` returns `$42` in sim and on the bench, with the monitor otherwise unaffected.
That proves: Arlet `verilog-6502` synthesizes + clocks at 25 MHz on the ECP5, runs a
program from BRAM, posts an SDRAM write through the new arbiter, and the host reads the
coprocessor's output. The spine for C1/C2 exists.

## Non-goals (later rungs)

- **C1:** host *loads* a program into coproc BRAM via `$C0Cx` (the "register" step) +
  a proper SDRAM job/result address map (not the hardwired `$E000`→0).
- **C2:** a tiny multitasking kernel runs **two** loaded tasks concurrently; each writes
  a completion-order marker → the race → tune one to flip the winner (the user's goal).
- **C3+:** more Arlet cores (the arbiter/coproc modularity pays off), pub/sub mailbox,
  scheduler tuning.
- No host↔coproc interrupts, no coproc access to host RAM, no Merlin-authored coproc
  jobs yet (C0's program is fixed/built-in).
