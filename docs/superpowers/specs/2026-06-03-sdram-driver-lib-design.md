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
bit7 clears → read DATA.

**Auto-increment rule (confirmed in RTL — the hinge for RDNEXT):** `addr`
advances by one **only on a completed SDRAM access**, which is exactly a
**TRIG_RD strobe** (`$C0C4` W) or a **DATA write** (`$C0C6` W). A **DATA read**
(`$C0C6` R) is a pure latch read of the last result — it does **NOT** advance
`addr`. So `SDM_RDNEXT` (strobe TRIG_RD → poll → `LDA DATA`) advances exactly
once per call; the `LDA DATA` is inert. (Verified: `monitor_regs` increments on
`op_done` = `sdram_busy` falling, and only TRIG_RD/DATA-write raise `sdram_busy`.)

**Strobe→poll is race-free (sticky busy):** `m_busy` is set on the strobe cycle
and cleared only by the first STATUS read after the access completes, so the
first poll always reads busy=1; data is never sampled before the access finishes.

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
| `SDM_READY`  | **Bounded** wait for ready (bit6). Polls up to a fixed retry budget (16-bit/24-bit loop). Returns **carry clear = ready**, **carry set = timeout** (card absent/wrong slot → `$C0C5` floats, bit6 never sets). Caller MUST branch on carry — do not assume. Call once at start. |
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

**Design invariant — no indexed access to the register port:** the library
touches `$C0Cx` only with plain absolute `STA`/`LDA` (never `STA $C0Cx,X` etc.).
Absolute access is a single `nDEVICE_SELECT` pulse, so it dodges the `STA abs,X`
dummy-read that would double-count a strobe (CLAUDE.md 6502 bus gotcha). Any
future addition to the library must preserve this.

## B. The bank test — `software/SDM/sdram_libtest.S`

`PUT sdram_lib.S`, then run a **two-phase** sweep over offsets `$0000–$0009` of
every bank `0..1023`:

- **Value scheme:** `expected(bank, off) = lo(bank) EOR hi(bank) EOR off EOR $5A`.
- **Detection guarantee (and its limit):** an 8-bit value cannot uniquely tag
  1024 banks, so this targets the realistic failure — a **single-bit bank-address
  decode fault** (a stuck/swapped/dropped bank line). Any two banks that differ by
  exactly one bit produce different `lo EOR hi` (the differing bit lands in `lo`
  for bits 0–7, or in `hi` for bits 8–9), so a single-bit alias always mismatches
  in phase 2. **Limit:** banks that fold to the same value (e.g. bank 1 vs 256,
  both `lo EOR hi = 1`) are ≥2 bits apart and therefore unreachable by a
  single-bit fault — multi-bit aliasing is not distinguished. (Offsets don't add
  bank discrimination: `off` cancels in `expected(X)-expected(Y)`.)
- **Start:** `JSR SDM_READY`; if carry set → print `NO CARD` and quit (no hang).
- **Phase 1 — write all:** for each bank, `SETBANK` + `SETADDR $0000`, then 10×
  (`SDM_VAL = expected; SDM_WRNEXT`). Auto-increment walks the 10 offsets.
- **Phase 2 — read all:** for each bank, `SETBANK` + `SETADDR $0000`, then 10×
  (`SDM_RDNEXT`; compare `A` to recomputed `expected`). On mismatch, bump a
  16-bit failure counter and print `bbb/o exp ee got gg` — **but cap printed
  lines at the first 16**, then suppress further lines (keep counting) so a
  glitch/wrong-slot run that mismatches all 10,240 reads doesn't scroll the count
  off-screen.

Auto-increment does **not** cross bank boundaries: `m_addr` is a 16-bit `+1`
with no carry into the bank, so a sequential `WRNEXT`/`RDNEXT` run stays in the
current bank (fine here — offsets 0–9). A walk past `$FFFF` needs an explicit
`SETBANK`+`SETADDR`.

Two-phase (write *everything* before reading *anything*) is what catches bank
aliasing — immediate read-after-write would mask it. ~20,480 ops ≈ sub-second.

**Output (quiet + failures):**
- Banner line at start (`SDM BANK TEST`).
- `NO CARD` + quit if `SDM_READY` times out (carry set).
- A `.` every 64 banks during each phase as a progress pulse.
- Each mismatch on its own line: bank (hex), offset, expected, got — **first 16
  only**, then suppressed (counter keeps running).
- Final line: `PASS` or `nnnn FAILURES` (hex — the 16-bit counter is two `PRBYTE`).
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
    **High-bit caveat (verify, don't assume):** Merlin-on-Apple expects source as
    high-ASCII text; AppleCommander's text import may store low-ASCII and break
    `PUT`. This bit the project before (the `acx import --raw` note). The build
    must import the lib so an on-Apple `PUT SDRAM.LIB` actually loads cleanly —
    verify by loading/PUTing it in Merlin on the machine, not just that the file
    lands on the disk. (Pick the AppleCommander text-type/`--raw` mode that
    produces Merlin-loadable source; nail the exact invocation in the plan.)
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
        PUT SDRAM.LIB          ; the library
```
**`PUT` placement:** Merlin resolves forward references and sizes unknown-forward
operands as 16-bit (correct here, since `SDM_*` vars are absolute `DS`), so `PUT`
at the end works. **Recommended: `PUT` the library FIRST**, so all `SDM_*` labels
are defined before use — avoids any forward-ref surprise and reads cleaner. The
test program (§B) `PUT`s it first.
(`STZ` is 65C02; the library itself avoids 65C02-only opcodes so it runs on a
stock 6502 — the example caller can use whatever its CPU supports.)

## Testing / verification

- **Assembles clean** under Merlin32 here (`make assemble ASM_SRC=...`), both
  files; `SDMTEST` first byte is the expected entry opcode.
- **Library has zero ZP writes and zero `$C800`/`$C400`/`PR#` references** —
  grep the assembled listing / source.
- **On-Apple `PUT` loads:** confirm `PUT SDRAM.LIB` actually assembles in Merlin
  on the machine (high-bit caveat above) — not merely that the file copied.
- **On hardware (user runs):** `BRUN SDMTEST` → `SDM BANK TEST` … `PASS`.
- **Negative test (that the test actually checks):** the canonical, deterministic
  way is a deliberately wrong value scheme (write with one constant, verify
  against another) → `FAILURES`. Note the card-pull cases precisely:
  - Card **absent at start** → `SDM_READY` times out → `NO CARD` (NOT a fail
    count — it never reaches the sweep).
  - Card pulled **mid-run** (after READY passed) → garbage reads → `FAILURES`.
  Capture via `obs-screenshot`.
- **Refresh is not the failure source (confirmed, but watch):** this 20,480-op
  back-to-back sweep is the most sustained access the card will see. `sdram_ctrl`
  has a free-running refresh counter with refresh-priority over pending requests
  (verified in RTL), so a `FAIL` should mean a real decode/data fault, not
  refresh-induced bit-rot. If a FAIL ever correlates with sweep length/timing,
  re-examine refresh before blaming decode.
- No FPGA rebuild or flashing involved — this is host-side software only.

## Deferred (YAGNI)

| Item | Why deferred |
|------|--------------|
| Full 64 KB / full-chip sweep | ~tens of µs/op → an hour for 64 MB; not needed to prove decode |
| Walking-1s / march-C memory patterns | Standard 10-offset value sweep catches dropped data/addr bits for now |
| BASIC front-end / menu | Pure-asm BRUN is enough; add later if interactive bank poking is wanted |
| Configurable bank/offset ranges at runtime | Fixed 0..1023 × 0..9 for v1; parameterize later |
| Slot auto-detection | Single `SLOT` equate is fine |
