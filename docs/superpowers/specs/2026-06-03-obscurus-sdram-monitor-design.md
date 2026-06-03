# project_obscurus — SDRAM Monitor (design)

**Date:** 2026-06-03
**Board:** Byte Hamr Rev 2, slot 4 (hardcoded), `gateware/rev2/project_obscurus/`
**Status:** approved, pending implementation plan

## Goal

A self-contained 6502 monitor living in the card's expansion ROM that performs
live random-access read/write to the full 64 MB SDRAM through a bus register
port, with 64 KB bank switching. No disk image, no FujiNet, no external probing —
everything ships in the FPGA bitstream and runs on the Apple II keyboard +
screen. Foundation for later "hardware support for software" experiments.

Supersedes the boot-time "HELLO, WORLD! / HELLO, SDRAM!" string demo (milestone
complete, captured `captures/project_obscurus_20260602_214253.png`).

## Memory model

- SDRAM: AS4C32M16SB = 32M × 16 bits = **64 MB**.
- **Byte-packed**: two bytes per 16-bit word. `phys_word = phys_byte[25:1]`,
  `phys_byte[0]` selects high/low byte via DQM. Uses the whole chip.
- Physical byte address = `{bank[9:0], addr[15:0]}` = 26 bits = 64 MB.
- **1024 banks of 64 KB**. Each bank is a full 6502-style 64 KB space.
- `R 4D22` reads offset `$4D22` inside the current bank.
- All monitor numbers are **hex** (monitor convention). `B 1` = bank `$001`.
  Bank range `$000–$3FF`; offset range `$0000–$FFFF`.

## RTL register port (slot 4, `$C0Cx`)

| Addr     | R/W | Name    | Action |
|----------|-----|---------|--------|
| `$C0C0`  | W   | ADDR_LO | addr[7:0] |
| `$C0C1`  | W   | ADDR_HI | addr[15:8] |
| `$C0C2`  | W   | BANK_LO | bank[7:0] |
| `$C0C3`  | W   | BANK_HI | bank[9:8] |
| `$C0C4`  | W   | TRIG_RD | strobe: start SDRAM read → RD_HOLD, then addr++ |
| `$C0C5`  | R   | STATUS  | **bit7 = busy**, **bit6 = ready** (SDRAM init done); other bits 0 |
| `$C0C6`  | R   | DATA    | returns RD_HOLD (last byte read) |
| `$C0C6`  | W   | DATA    | latch byte + start SDRAM write, then addr++ |

**STATUS flag layout (deliberate).** `BIT $C0C5` lands bit7→N, bit6→V, so the
6502 tests both without `LSR`/`AND`:
- busy poll (hot path, every byte): `BIT $C0C5 : BMI *-3` (loop while N=busy).
- ready gate (once, before first op): `BIT $C0C5 : BVC *-3` (wait for V=ready).

**Handshake.** SDRAM access latency is many 25 MHz cycles — far longer than one
~1 µs 6502 bus cycle — so there is no in-cycle round-trip. The 6502 always:
set ADDR/BANK → strobe (TRIG_RD or DATA write) → **poll STATUS.busy until 0** →
read DATA / done.

**busy assert timing (required).** `busy` is set in `monitor_regs` on the
strobe's `nds_rise` commit — NOT gated on `sdram_ctrl` accepting the request.
This guarantees the very next 6502 poll sees `busy=1` even though the controller
accepts in ~ns; the 6502 cannot observe a stale `busy=0` and read a stale DATA.
`busy` clears when `sdram_ctrl` signals the op complete.

**Auto-increment (required semantics).** The internal 16-bit `addr` increments on
**operation completion (busy falling edge)**, NOT on the `$C0C6` DATA register
read. This prevents the `D` loop's `LDA $C0C6` from double-stepping the address.
Increment wraps within the 64 KB bank. The `D` dump loop is then: set start addr
once, then `{strobe TRIG_RD; poll busy; LDA DATA; print}` ×N.

**6502 bus safety.** `STA $C0Cx` / `LDA $C0Cx` absolute = 4 cycles, single
`nDEVICE_SELECT` pulse — no indexed-addressing dummy-read, so no double-step of
auto-increment. (CLAUDE.md 6502 bus gotcha.) Commit register writes on
`nds_rise` (existing 2-FF sync pattern in the top module).

## RTL structure (isolation)

- **`sdram_ctrl.v`** — extract the SDRAM controller into its own module with a
  clean request interface: `req`, `we`, `phys_addr[25:0]`, `wdata[7:0]`,
  `rdata[7:0]`, `busy`, `ready`, plus internal init + periodic refresh. Replaces
  the inlined boot-string FSM in the top module.
  **AS4C32M16 geometry decomposition (the meat — pin it explicitly).** The
  chip is 32M×16 = `{BA[1:0], ROW[12:0], COL[9:0]}` = 2+13+10 = 25 word-address
  bits. `phys_word = phys_byte[25:1]` (25 bits). Split:
  - `COL = phys_word[9:0]`
  - `ROW = phys_word[22:10]` (13 bits)
  - `BA  = phys_word[24:23]` (2 bits)

  Per access: `ACTIVE` (BA, ROW) → wait tRCD → `READ`/`WRITE` with **A10=1
  (auto-precharge)** and `A[9:0]=COL` → wait. Keep the existing per-access
  auto-precharge pattern (current FSM sets `A10=1` on READ/WRITE) — simplest
  close, do not lose it in the rewrite. Keep the **CL=2, burst=1** mode word
  (current LOAD_MODE value). The current inline FSM only ever drives row 0 /
  col=`str_idx` / bank 0; this generalizes it to arbitrary `phys_word`.

  **Byte-packed DQM (currently hardwired `SDRAM_DQM0/1 = 1'b0` — must become
  FSM-driven regs).** Lane keyed on `phys_byte[0]` (even = low byte D[7:0]):
  - **Write** cycle: `wdata` duplicated on both lanes (`dq_out = {wdata,wdata}`);
    drive masks so only the target lane writes —
    `SDRAM_DQM0 = phys_byte[0]` (low lane enabled when even),
    `SDRAM_DQM1 = ~phys_byte[0]` (high lane enabled when odd).
    (`DQM` high = lane masked.)
  - **Read** cycle: `SDRAM_DQM0 = SDRAM_DQM1 = 1'b0` (read both lanes — do NOT
    mask), then select returned byte in logic:
    `rdata = phys_byte[0] ? dq_in[15:8] : dq_in[7:0]`. Masking on read would
    suppress data; only writes mask.
  - Idle/init: `DQM = 0` (don't-care for non-access).
  *(CLAUDE memory: SDRAM FSM edits have corrupted synth before. Mitigation: this
  is a fresh, isolated module, not an edit to flash_hamr's arbiter, and it is
  sim-verified before any build.)*

  **Scaffolding to DELETE (strip the hello-world demo, do not bolt onto it).**
  `sdram_ctrl.v` replaces the inline FSM, so these get removed entirely:
  `str_src[]`, `mirror[]`, `str_idx` + seek/auto-advance, `STR_LEN`, the
  boot-time WRITE_STR/READ_STR states, and the string read-mux (`mirror_byte`).
  **Keep** the reusable bus glue: 2-FF sync + `nds_rise` commit, the
  `wr_data_latch`/`wr_addr_latch` path, POR/`rst_n`, heartbeat, and the
  scratch-loopback registers.
- **`monitor_regs`** (in top module) — bus-side glue: latch ADDR/BANK/DATA on
  `nds_rise`, raise `req`/`we` to `sdram_ctrl`, hold `RD_HOLD`, expose STATUS,
  drive auto-increment.
- **ROMs in bitstream:**
  - `slot_rom.mem` — 256 B at `$C400`. Stub: restore CSW, `JMP $C800`.
  - `monitor.mem` — 2 KB at `$C800–$CFFF`. The monitor program.
- **Expansion ROM enable (delta from current — today it is deliberately OFF).**
  The current top module **never** drives `$C800–$CFFF`: `slot_active` and
  `d_oe`/`d_out` assert only on `nDEVICE_SELECT`/`nI_O_SELECT` (the `nI_O_STROBE`
  drive was intentionally dropped to avoid bus-fighting another card's expansion
  ROM). Re-enabling it is undoing that guard, so do it properly:
  1. Add an **expansion-ROM enable FF**: set on `$Cn00` access (`nI_O_SELECT`),
     clear on `$CFFF` access. (`$CFFF` is the shared disable convention — see the
     `$CFFF` reserve note below.)
  2. `rom_en` gates `~nI_O_STROBE` into **both**:
     - `slot_active` (→ `DATA_OE`, the U12 level-shifter OE), and
     - the `d_oe` / `d_out` mux (add `exp_read = rom_en & ~nI_O_STROBE & R_nW`
       → `d_out = monitor_mem[apple_addr[10:0]]`).
  Two assigns change, not one. Single-card bring-up assumption noted; the FF
  keeps it correct if another card is present.
- **`$CFFF` is reserved — leave it unused.** The enable FF clears on ANY `$CFFF`
  access, including an instruction fetch. The monitor MUST NOT execute, read, or
  place any code/data at `$CFFF`, or it disables its own ROM mid-run. Constraint:
  monitor occupies `$C800–$CFFE` only (max 2046 bytes); `$CFFF` stays `$00`. The
  build/sim asserts the byte at offset `$7FF` of `monitor.mem` is unused.

## 6502 monitor (`monitor.S`, Merlin32, ORG $C800)

**Launch + exit contract (pin it — "return to BASIC" is otherwise undefined).**
- BASIC `PR#4` does `JSR $C400`, leaving its return address on the stack. The
  `$C400` stub: restore CSWL/CSWH = `$FDF0` (COUT1) so output reaches the screen
  — the current `slot_rom.S` writes the text page direct *specifically to avoid
  CSW recursion* (`$FDED` would recurse through `(CSW)=$C400` → stack overflow);
  the monitor can use `COUT` **only because CSW is restored first**. Then
  `JMP $C800` (a jump, not a call — the monitor seizes the machine).
- Because the stub `JMP`s (no push), the SP at `$C800` entry still holds BASIC's
  `JSR $C400` return address. **`Q` exits via `RTS`**, popping that address back
  into BASIC's PR# handler, which finishes and drops to the immediate-mode
  prompt — the clean exit. **Requirement: the monitor keeps the stack balanced
  across the whole session** (`GETLN`/`COUT` self-balance; do not leak pushes),
  or the `RTS` returns to garbage.
- **Dead until RTL lands:** reaching `$C800` at all requires the expansion-ROM
  enable FF + `nI_O_STROBE`/`d_oe` drive (RTL structure section). `JMP $C800` is
  a no-op brick until that exists — implement the RTL before testing the stub.

**Ready gate.** Before the first SDRAM op (cheap insurance; init is ~200 µs and
banner+GETLN already covers it): `BIT $C0C5 : BVC *-3` — wait for V=ready.

**Main loop.**
1. Print banner + current `BANK xxx`.
2. Print prompt `*`.
3. `GETLN` (`$FD6A`) — line input into `$0200`, chars high-bit set, `$8D` term.
4. Parse: first non-space char = command. Own small hex parser for operands.
5. Dispatch; on bad input print `?` and reprompt.

**Commands.**
- `R aaaa` — read one byte. Output: `aaaa: bb`.
- `W aaaa bb` — write one byte.
- `B bbb` — set bank (`$000–$3FF`); update displayed bank.
- `D aaaa` — dump 16 bytes from `aaaa`: hex + ASCII, two rows of 8.
- `Q` — return to BASIC.

**ROM routines used:** `GETLN $FD6A`, `COUT $FDED`, `CROUT $FD8E`,
`PRBYTE $FDDA`, `RDKEY $FD0C`. Hex parse + formatting hand-written.

**Zero-page scratch (pinned — do NOT leave to implementation).** The monitor
uses exactly these ZP bytes, all free under Applesoft and outside every
forbidden range:

| ZP | Use |
|----|-----|
| `$06` / `$07` | operand pointer / hex-parse accumulator (16-bit addr) |
| `$08` | parsed data byte (W) |
| `$09` | scratch / current bank shadow |

Everything else lives in `A`/`X`/`Y` and the hardware stack. Loop counters use
`X`/`Y`; nested values are pushed.

Rationale and forbidden ranges:
- The traditional Apple monitor scratch `$3C–$3F` (A1/A2) is **forbidden** here
  (ProDOS/`wait_ready` territory — caused Relo/Conf errors before). Do not use it
  even though it is the textbook choice.
- Also avoid `$42–$47` (ProDOS) and the rest of `$36–$3F`. The only `$36/$37`
  touch is the required CSW restore in the `$C400` stub.
- `$06–$09` are confirmed unused by the ROM routines we call (`COUT`/`CROUT` use
  `$24/$25/$28/$29`; `GETLN` uses the `$0200` buffer + `$32/$33`; `PRBYTE` uses
  `A`). Do not touch BASIC's program/variable pointers (`$67`+).

**Context.** Monitor runs in BASIC (Applesoft) context, not as a ProDOS driver,
so the `$36/$37` CSW restore is intended behavior (same as the existing
slot_rom).

## Testing

**Simulation (`project_obscurus_tb.v`, must pass before any build).** This is a
**rewrite of the TB SDRAM model, not an extension** — the current model
(`reg [15:0] sdram_model[0:255]`, col captured @ACTIVE from `A[7:0]`, full-16-bit
write ignoring DQM) is wrong for random access and would **pass falsely**. The
new model is the safety net per CLAUDE's "sim before build" rule, so it must be
faithful on these four points or sim lies while hardware fails:

1. **Sparse, not dense.** A dense `reg[15:0] mem[0:33554431]` is ~512 MB of sim
   RAM. Use a SystemVerilog associative array: `reg [15:0] mem [int];` (iverilog
   `-g2012`). Default-read unwritten words as a known sentinel (e.g. `'x` or
   `16'h0000`) so "read before write" is visible.
2. **Capture row @ACTIVE, col @READ/WRITE — opposite of today.** New `sdram_ctrl`
   puts `{BA,ROW}=A[12:0]` on `ACTIVE` and `COL=A[9:0]` on `READ`/`WRITE`. Model
   must latch `{BA,ROW}` on cmd `0011` and form the word index from the latched
   row + the col sampled on `0101`/`0100`. (Today's model captures col@ACTIVE —
   inverting it would round-trip by accident and hide bugs.)
3. **Mask A10 out of the column.** `READ`/`WRITE` carry `A10=1` (auto-precharge);
   the column is `A[9:0]` only. If the model includes A10 the index is corrupted
   by `+1024`. (Today's `A[7:0]` model never hit this; the full-width one will.)
4. **Honor DQM lanes.** On `WRITE`, update only the unmasked byte lane:
   `if (!DQM0) mem[idx][7:0] <= dq[7:0]; if (!DQM1) mem[idx][15:8] <= dq[15:8];`
   A model that writes all 16 bits passes the byte-isolation test even when DQM
   is broken — i.e. it can't verify the one thing byte-packing adds.

Also add a **`poll_busy` task** (read `$C0C5`, loop until bit7=0) used before
every DATA sample — the current `read_reg`/`write_reg` fixed `#200`/`#400` waits
do not model the busy handshake and would sample DATA mid-latency.

Scenarios (each via set-addr/bank → strobe → `poll_busy` → sample):
- W-then-R round-trip at several `{bank,addr}`.
- Bank isolation: same `addr`, different `bank` → independent bytes.
- Byte-packed lane isolation: write `addr 2k` and `2k+1` (same `phys_word`) →
  two independent bytes (proves DQM select).
- Auto-increment on op completion: after a strobe+poll, `addr` advanced by 1;
  and a DATA **read with no preceding strobe must NOT advance `addr`** (guards
  the `D`-loop double-step).
- `busy` asserts on the strobe's `nds_rise` (same cycle) — sample STATUS.bit7
  immediately after the strobe write, before any settle.
- `D` dump: 16 sequential reads return the written sequence.
- Build check (script/assert, not RTL): `monitor.mem[$7FF]` (`$CFFF`) is `$00`.

**Hardware (Merlin32 + `make DESIGN=project_obscurus REV=rev2`; user flashes):**
- `W 0000 AA` then `R 0000` → `AA`.
- `B 1`, `R 0000` → not `AA` (bank isolation).
- `D 0000` → dump renders.
- Screenshot via `obs-screenshot`.

## Deferred (YAGNI)

| Item | What | Why deferred | Cost to add later |
|------|------|--------------|-------------------|
| F — Fill | `F aaaa bbbb vv` fill range with constant | Not needed to prove R/W/B/D; auto-inc write path already exists | Low — 3rd operand in parser; no RTL |
| Search / Find | Scan range for byte/pattern | Pure convenience, no new hardware path | Low-med — 6502 read+compare loop |
| Write-protect / bank lock | Mark bank read-only | No safety need on a playground | Low RTL register + gate, low 6502 |
| >1024 banks | Address beyond 64 MB | Physically impossible — chip is 64 MB | N/A unless board changes |
| ASCII-input mode | `W aaaa "TEXT"` string literals | Hex-only proves the path | Med — string-literal parser in 6502 |

All deferred items are additive and (except write-protect) require no change to
`sdram_ctrl.v`. The only hard ceiling is the 64 MB chip = 1024 banks physical max.

## Build / toolchain

- 6502 source: Merlin32 `.S` → `.bin` → `.mem` via `scripts/rom2mem.py`
  (CLAUDE.md rule — no Python hack assemblers). Two ROMs: `slot_rom.S`
  (`$C400`) and `monitor.S` (`$C800`).
- Makefile: extend the existing `OBSCURUS_ROM_MEM` recipe to also build
  `monitor.mem`; both gate `$(JSON)` for `DESIGN=project_obscurus`.
- **`monitor.mem`: `rom2mem.py monitor.bin monitor.mem 0xC000 2048`** — base
  `0xC000`, **NOT** `0xC800`.
  - `rom2mem.py` computes `offset = base - 0xC000` and writes bin bytes starting
    at `rom[offset]`. It ignores the `.S` ORG entirely — `base` only sets where
    `mem[0]` lands. With `base=0xC800`, `offset = 2048 = size`, so the guard
    `offset+i < size` is false for every byte → **`monitor.mem` is all-`$FF`,
    monitor never runs** (silent brick). `base=0xC000` → `offset=0` → bin lands
    at `mem[0]`, matching the existing `slot_rom` recipe (`0xC000 256`, even
    though `slot_rom.S` ORGs `$C400`).
  - Verilog indexes `monitor_mem[apple_addr[10:0]]`: `$C800–$CFFF` = 2 KB = 11
    bits; `$C800` low-11 = `0`, `$CFFF` low-11 = `$7FF`. So `mem[0]` = first
    monitor byte at `$C800`. Page alignment makes the ORG-vs-offset mismatch
    harmless (same trick as `slot_rom_mem[apple_addr[7:0]]`).
- Never flash without explicit user request.
