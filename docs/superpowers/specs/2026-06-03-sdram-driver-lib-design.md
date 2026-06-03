# SDRAM Driver Library + Bank Test (design)

**Date:** 2026-06-03
**Target:** Apple II software for the Byte Hamr Rev 2 `project_obscurus` card (slot 4)
**Depends on:** the `project_obscurus` SDRAM monitor RTL (register port `$C0C0–$C0C6`),
already bench-verified. See `2026-06-03-obscurus-sdram-monitor-design.md`.
**Status:** approved, pending implementation plan

## Goal

A reusable Merlin **driver library** for the card's SDRAM register port, written
once and consumed two ways:
1. `PUT`-included into the user's own Merlin programs (assembled on the Apple II
   *or* with the repo's Merlin32), and
2. `PUT`-included into a small **bank test** here that programmatically exercises
   the first 10 offsets of all 1024 banks and reports PASS/FAIL.

The library talks to the register port directly (`STA`/`LDA $C0Cx`). It does
**not** use `PR#4`, the `$C800` expansion ROM, or the `$C400` slot ROM — those
are only for the interactive monitor. The register port is always live, so the
library is an independent second client of the same hardware.

**Naming:** prefix `SDM_` (SDRAM driver). Deliberately not `OBS` (collides with
the OBS capture tool / `obs-screenshot`).

## Register port recap (slot 4)

| Addr    | R/W | Meaning |
|---------|-----|---------|
| `$C0C0` | W   | ADDR_LO |
| `$C0C1` | W   | ADDR_HI |
| `$C0C2` | W   | BANK_LO |
| `$C0C3` | W   | BANK_HI (bits 1:0) |
| `$C0C4` | W   | TRIG_RD (write any byte → start read) |
| `$C0C5` | R   | STATUS — bit7 busy, bit6 ready |
| `$C0C6` | R/W | DATA (write → SDRAM write; read → last result) |

Handshake: set ADDR/BANK → strobe (TRIG_RD or DATA write) → poll STATUS until
bit7 clears → read DATA. `addr` auto-increments on each completed access.

## A. The library — `software/SDM/sdram_lib.S`

A `PUT`-includable source file. **Constraints that make it safe to import
anywhere:** no `ORG`, no zero page, no reliance on the monitor ROM. All access is
`$C0Cx` absolute. Uses only `A`/`X`/`Y` internally and its own labeled `DS`
storage. Syntax kept to the Merlin-8 / Merlin32 common subset (equates, `DS`,
plain mnemonics; no Merlin32-only directives) so it assembles on the Apple II too.

**Configuration (the only knob):**
```
SLOT      = 4                  ; change per install
SDM_BASE  = $C080+SLOT*16      ; $C0C0
```
Internal register equates derive from `SDM_BASE` (`+0..+6`).

**Public parameter variables** (library reserves with `DS`):
| Label | Size | Use |
|-------|------|-----|
| `SDM_BANK` | 2 | bank (10-bit; high 6 bits ignored by hardware) |
| `SDM_ADDR` | 2 | 16-bit offset within bank |
| `SDM_VAL`  | 1 | byte to write / last byte read |

**Public entry points** (caller sets vars, then `JSR`):
| Routine | Contract |
|---------|----------|
| `SDM_READY`  | Block until card ready (`BIT STATUS` / `BVC`). Call once at start. |
| `SDM_SETBANK`| Copy `SDM_BANK` → BANK_LO/HI registers. |
| `SDM_SETADDR`| Copy `SDM_ADDR` → ADDR_LO/HI registers. |
| `SDM_WRITE`  | `SETBANK`+`SETADDR`, write `SDM_VAL` (strobe DATA), poll busy. |
| `SDM_READ`   | `SETBANK`+`SETADDR`, strobe TRIG_RD, poll busy, load DATA → `A` and `SDM_VAL`. |
| `SDM_WRNEXT` | Write `SDM_VAL` at the hardware's auto-incremented addr (no addr re-set), poll. For fast sequential writes. |
| `SDM_RDNEXT` | Strobe TRIG_RD at the auto-incremented addr, poll, load → `A`/`SDM_VAL`. For fast sequential reads. |

Internal: `SDM_POLL` (`LDA STATUS` / `BMI SDM_POLL`). Auto-increment caveat: a
`WRNEXT`/`RDNEXT` sequence must be preceded by a `SETADDR` (or a `WRITE`/`READ`)
to establish the starting address; the hardware then advances by one per
completed access. `SETBANK` does not need re-issuing between same-bank accesses.

**No `RTS`-time register state:** every entry point leaves the card idle
(not busy). Caller may freely interleave other code between calls.

## B. The bank test — `software/SDM/sdram_libtest.S`

`PUT sdram_lib.S`, then run a **two-phase** sweep over offsets `$0000–$0009` of
every bank `0..1023`:

- **Value scheme:** `expected(bank, off) = lo(bank) EOR hi(bank) EOR off EOR $5A`.
  Folding both bank bytes means two banks that differ only in the high byte (e.g.
  0 vs 256) still get different values, so a dropped bank-address bit shows up.
- **Phase 1 — write all:** for each bank, `SETBANK` + `SETADDR $0000`, then 10×
  (`SDM_VAL = expected; SDM_WRNEXT`). Auto-increment walks the 10 offsets.
- **Phase 2 — read all:** for each bank, `SETBANK` + `SETADDR $0000`, then 10×
  (`SDM_RDNEXT`; compare `A` to recomputed `expected`). On mismatch, print
  `bbb/o exp ee got gg` and bump a 16-bit failure counter.

Two-phase (write *everything* before reading *anything*) is what catches bank
aliasing — immediate read-after-write would mask it. ~20,480 ops ≈ sub-second.

**Output (quiet + failures):**
- Banner line at start (`SDM BANK TEST`).
- A `.` every 64 banks during each phase as a progress pulse.
- Each mismatch on its own line: bank (hex), offset, expected, got.
- Final line: `PASS` or `nnnn FAILURES` (decimal or hex count).
- All via `COUT $FDED` / `PRBYTE $FDDA` / `CROUT $FD8E`. A normal program (not
  launched via `PR#`), so `COUT` goes straight to the screen — no CSW games.

## C. Packaging & delivery

- Source lives in `software/SDM/`: `sdram_lib.S`, `sdram_libtest.S`.
- Build: Merlin32 assembles `sdram_libtest.S` → `SDMTEST` BIN (`ORG $2000`).
- Disk: a copy of the ProDOS 2.4.1 base image (`software/SMARTPORT/images/
  ProDOS_2_4_1.po`) with **BASIC.SYSTEM** present, plus:
  - `SDMTEST` — BIN, load/aux `$2000`, run via `BRUN SDMTEST` from the `]` prompt.
  - `SDRAM.LIB` — the `sdram_lib.S` text, imported as a ProDOS **TXT** file so it
    can be `PUT` from Merlin *on the Apple II* (same source we build with here).
- Built with AppleCommander (`AC_JAR`), following the existing `create-dsk`
  pattern. Delivered to a real floppy via ADTPro (existing flow).

## D. On-Apple usage (the point of the library)

In the user's own Merlin source on the Apple II:
```
        ORG $2000
        ... my program ...
        LDA #$AA
        STA SDM_VAL
        LDA #$22 : STA SDM_ADDR : LDA #$4D : STA SDM_ADDR+1   ; $4D22
        STZ SDM_BANK : STZ SDM_BANK+1                          ; bank 0
        JSR SDM_WRITE
        ...
        PUT SDRAM.LIB          ; the library, last
```
(`STZ` is 65C02; the library itself avoids 65C02-only opcodes so it runs on a
stock 6502 — the example caller can use whatever its CPU supports.)

## Testing / verification

- **Assembles clean** under Merlin32 here (`make assemble ASM_SRC=...`), both
  files; `SDMTEST` first byte is the expected entry opcode.
- **Library has zero ZP writes and zero `$C800`/`$C400`/`PR#` references** —
  grep the assembled listing / source.
- **On hardware (user runs):** `BRUN SDMTEST` → `SDM BANK TEST` … `PASS`. A
  deliberately mis-typed value scheme (or pulling the card) should produce
  `FAILURES`, confirming the test actually checks. Capture via `obs-screenshot`.
- No FPGA rebuild or flashing involved — this is host-side software only.

## Deferred (YAGNI)

| Item | Why deferred |
|------|--------------|
| Full 64 KB / full-chip sweep | ~tens of µs/op → an hour for 64 MB; not needed to prove decode |
| Walking-1s / march-C memory patterns | Standard 10-offset value sweep catches dropped data/addr bits for now |
| BASIC front-end / menu | Pure-asm BRUN is enough; add later if interactive bank poking is wanted |
| Configurable bank/offset ranges at runtime | Fixed 0..1023 × 0..9 for v1; parameterize later |
| Slot auto-detection | Single `SLOT` equate is fine |
