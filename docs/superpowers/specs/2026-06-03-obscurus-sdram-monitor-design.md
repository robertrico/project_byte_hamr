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
  Byte-packed addressing keyed on `phys_addr[0]`:
  - **Write**: `DQM = {phys_addr[0], ~phys_addr[0]}` (mask the lane not being
    written), `wdata` duplicated onto both byte lanes.
  - **Read**: `DQM = 2'b00` (read both lanes — do NOT mask), then select the
    returned byte lane in logic via `phys_addr[0]`. Masking on read would
    suppress data; only writes mask.
  *(CLAUDE memory: SDRAM FSM edits have corrupted synth before. Mitigation: this
  is a fresh, isolated module, not an edit to flash_hamr's arbiter, and it is
  sim-verified before any build.)*
- **`monitor_regs`** (in top module) — bus-side glue: latch ADDR/BANK/DATA on
  `nds_rise`, raise `req`/`we` to `sdram_ctrl`, hold `RD_HOLD`, expose STATUS,
  drive auto-increment.
- **ROMs in bitstream:**
  - `slot_rom.mem` — 256 B at `$C400`. Stub: restore CSW, `JMP $C800`.
  - `monitor.mem` — 2 KB at `$C800–$CFFF`. The monitor program.
- **Expansion ROM enable.** Re-enable `nI_O_STROBE` data drive (currently
  dropped). Standard expansion-ROM enable FF: set on `$Cn00` access
  (`nI_O_SELECT`), clear on `$CFFF` access. Single-card bring-up assumption
  noted; the FF keeps it correct if another card is present.
- **`$CFFF` is reserved — leave it unused.** The enable FF clears on ANY `$CFFF`
  access, including an instruction fetch. The monitor MUST NOT execute, read, or
  place any code/data at `$CFFF`, or it disables its own ROM mid-run. Constraint:
  monitor occupies `$C800–$CFFE` only (max 2046 bytes); `$CFFF` stays `$00`. The
  build/sim asserts the byte at offset `$7FF` of `monitor.mem` is unused.

## 6502 monitor (`monitor.S`, Merlin32, ORG $C800)

**Launch.** `PR#4` → `$C400` stub. Stub restores CSWL/CSWH = `$FDF0` (COUT1) so
output reaches the screen (not recursing through `$C400`), then `JMP $C800`.

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

**Simulation (`project_obscurus_tb.v`, must pass before any build):**
- Extend the behavioral SDRAM model to a full addressable array (byte-packed).
- W-then-R round-trip at several addresses.
- Bank isolation: same `addr`, different `bank` → independent bytes.
- Auto-increment fires on op completion (busy falling edge): a DATA read with NO
  preceding strobe must NOT advance `addr` (guards against `D`-loop double-step).
- `busy` asserts on the strobe's `nds_rise` (same cycle), before any poll could
  observe it.
- Byte-packed: writing addr `2k` and `2k+1` (same `phys_word`) stores two
  independent bytes (DQM lane select works).
- `D` dump: 16 sequential reads return the written sequence.
- `monitor.mem[$7FF]` ($CFFF) is unused/`$00`.

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
- `monitor.mem`: `rom2mem.py monitor.bin monitor.mem 0xC800 2048`.
- Never flash without explicit user request.
