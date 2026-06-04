# project_obscurus — Armed Expansion ROM (design)

**Date:** 2026-06-04
**Board:** Byte Hamr Rev 2, `gateware/rev2/project_obscurus/` (slot 4), + `software/SDM/`
**Status:** approved-in-discussion, pending spec review
**Supersedes behavior in:** `2026-06-03-obscurus-sdram-monitor-design.md` (the always-on
`$C800` expansion ROM), `2026-06-03-sdram-driver-lib-design.md` (unchanged, referenced)

## Problem

The card drives the shared `$C800–$CFFF` expansion-ROM bus whenever its `rom_en`
latch is set. `rom_en` arms on any `$C4xx` access (ProDOS's boot slot-scan does
this) and only clears on `$CFFF`. So after boot the card holds the shared bus and
**collides with the //e internal ROM / other cards' `$C800` firmware** that
CopyUtils, CATALOG, and BASIC.SYSTEM use. Because our expansion ROM is only 520
bytes with `$FF` fill above it, a collided fetch executes garbage → crash at
`$CExx` (`$CEF3` observed) + cross-disk corruption.

This is a **known, documented bug class** on this hardware:
- `iwm_hamr_write_bug.md`: card claimed `$C800` and never released it → CATALOG
  SYNTAX ERROR; fix was "only serve `$Cn` slot ROM, never `$C800`."
- `flash_hamr_v5_cache.md`: `$C800` expansion ROM → Relo/Config Error, bus
  contention with the //e internal ROM; workaround was dropping `$C800`.

It is **not an Apple constraint** — `$C800` is a documented shared resource used
correctly by many cards. The defect is our gateware holding the bus when not in
use, plus `$FF` (crash) fill instead of a valid/benign ROM.

## Goal

Make the card **silent on the shared `$C800–$CFFF` bus unless explicitly armed by
software**, default disarmed, auto-disarmed on reset — while keeping the `$C800`
space fully usable on demand (the `PR#4` monitor). SDRAM access stays always-live
and unarmed. Result: CATALOG, file copies, and disk I/O never collide with the
card; the monitor still works when you invoke it.

## 1. Gateware — `rom_armed` soft-switch

A new flip-flop `rom_armed`, default **0**:

- **Set** (arm): a register write of magic `$AA` to `$C0C7`.
- **Clear** (disarm): a register write of magic `$AA` to `$C0C8`; **or** `nRES`
  (Ctrl-Reset, via the existing `rst_n`); **or** POR. Any non-`$AA` write to
  either address is ignored (no state change) — neither arm nor disarm can be
  triggered by garbage.
- Reset/POR clearing is the hard safety net: a crash that can't send the disarm
  magic still recovers to silent on Ctrl-Reset.

The `$C800` drive is gated by `rom_armed`:
```
exp_read   = rom_en & rom_armed & ~nI_O_STROBE & R_nW;   // FPGA -> D drive
slot_active = ~nDEVICE_SELECT | ~nI_O_SELECT | (rom_en & rom_armed & ~nI_O_STROBE);
assign DATA_OE = ~slot_active;                            // U12 '245 OE
```
With `rom_armed = 0` by default, the card **never** drives or buffers
`$C800–$CFFF` during boot, CATALOG, disk I/O, or copies. (`rom_en` keeps its
existing set-on-`$Cn00` / clear-on-`$CFFF` behavior; it is simply ANDed with
`rom_armed`.)

**Unused-ROM fill:** `monitor.mem` unused region (`$CA08–$CFFE`) filled with
`$60` (RTS) instead of `$FF`. A stray fetch into our space then returns
harmlessly instead of executing garbage. (Build: pad the assembled `monitor.bin`
to the fill length with `$60` before `rom2mem`, or post-process the `.mem`.)

**Unchanged & safe:** `$C0C0–$C0C6` (SDRAM addr/bank/trig/status/data) and the
`$C400` slot ROM remain slot-gated (`nDEVICE_SELECT` / `nI_O_SELECT`) and
always-live. They can only respond to our slot's own address space — they never
touch the shared `$C800` bus and cannot collide.

## 2. Register map (slot 4)

| Addr | R/W | Name | Notes |
|------|-----|------|-------|
| `$C0C0` | W | ADDR_LO | always live |
| `$C0C1` | W | ADDR_HI | |
| `$C0C2` | W | BANK_LO | |
| `$C0C3` | W | BANK_HI | |
| `$C0C4` | W | TRIG_RD | |
| `$C0C5` | R | STATUS | bit7 busy, bit6 ready |
| `$C0C6` | R/W | DATA | |
| `$C0C7` | W | **ROM_ARM** | write `$AA` → arm `$C800`; other values ignored |
| `$C0C8` | W | **ROM_DISARM** | write `$AA` → disarm; other values ignored |
| `$C0C9–$C0CF` | R/W | SCRATCH | loopback (shrunk from `$C0C7`) |

## 3. Software

- **`$C400` slot ROM stub** (`slot_rom.S`, always live): on `PR#4`/`JSR $C400` →
  `LDA #$AA : STA $C0C7` (arm) → restore CSW = `$FDF0` → `JMP $C800`.
- **`$C800` monitor** (`monitor.S`, armed): existing R/W/B/D commands, **plus
  `T`** = run the 1024-bank sweep (shared `SDMSWEEP.S`), print `PASS` / `nnnn
  FAILURES`. **`Q`** → `LDA #$AA : STA $C0C8` (disarm) → touch `$CFFF` (release
  the hardware `rom_en` latch too) → `RTS` to BASIC.
- **`SDRAMLIB.S`** (`software/SDM/`): unchanged public API (pure `$C0Cx`,
  unarmed). Add two helpers for completeness: `SDM_ROMON` (`LDA #$AA : STA
  $C0C7 : RTS`) and `SDM_ROMOFF` (`LDA #$AA : STA $C0C8 : RTS`). The bank
  test does **not** call these (it never needs `$C800`).
- **`SDMSWEEP.S`** (new, `software/SDM/`): the two-phase 1024-bank sweep factored
  out of `SDMTEST.S`, `PUT`-included by **both** the standalone test and the
  monitor's `T` command — one source of the sweep logic (DRY).
- **`SDMTEST` (BRUN)**: kept, unchanged behavior — the unarmed proof; uses only
  `$C0Cx`, never arms `$C800`. Now `PUT`s `SDMSWEEP.S`.

The card is thus usable two independent ways: **unarmed** (BRUN `SDMTEST`, or any
user program via `SDRAMLIB` — SDRAM only, never touches the shared bus), and
**armed** (`PR#4` monitor — `$C800` live only for that session).

## 4. Safety / recovery semantics

- Power-on: `rom_armed = 0` → silent.
- Normal ProDOS / CATALOG / copy: nothing writes `$AA` to `$C0C7` → stays
  silent → no collision.
- `PR#4`: stub arms → monitor session drives `$C800` → `Q` disarms.
- Crash / Ctrl-Reset during a session: `nRES` clears `rom_armed` → silent →
  disks safe.

## 5. Testing

**Simulation (must pass before build):**
- Disarmed (default): assert the card never drives `D` nor asserts `DATA_OE` for
  `$C800–$CFFF` reads, even with `rom_en` set (simulate a `$C4xx` access then
  `$C8xx` reads → bus stays `Z`, `DATA_OE` deasserted).
- Arm: write `$AA`→`$C0C7`, then `$C8xx` read → card drives `exp_rom_data`.
- Disarm: write `$AA`→`$C0C8` → `$C8xx` reads go `Z` again.
- Magic guard: write non-`$AA` to `$C0C7`/`$C0C8` → no state change.
- Reset: assert `nRES` while armed → `rom_armed` clears.
- Regression: the existing `$C0Cx` integration TB (unarmed SDRAM R/W/bank/dump)
  still PASSES unchanged.

**Bench (the real regression — user runs):**
- Card installed, **no `PR#4`**: `CATALOG`, then **copy a file** → no crash, no
  "no disk in drive", disks intact. (This is the bug that's been biting.)
- `PR#4` → monitor banner; `R`/`W`/`B`/`D` work; **`T`** → `PASS`; `Q` → back to
  `]`; then `CATALOG`/copy again → still clean.
- Ctrl-Reset mid-monitor → returns to BASIC, card silent, disks fine.

## Deferred (YAGNI)

| Item | Why deferred |
|------|--------------|
| Full valid 2KB ROM (vs RTS fill) | RTS fill + arm-gate is sufficient; no need to fill 2KB with real code |
| Read-strobe arming (Apple `$C05x`-style) | Magic-value write strobes are robust enough and explicit |
| Arm status read-back register | Not needed; software knows what it armed; reset is the recovery |
| Per-slot generality | Single `SLOT` equate already covers it |
