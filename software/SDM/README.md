# SDM — SDRAM driver library + bank test

Apple II software for the Byte Hamr Rev 2 `project_obscurus` card (slot 4). A
reusable Merlin driver library for the card's SDRAM register port, plus a test
that exercises all 1024 banks.

Talks to the always-live `$C0C0–$C0C6` register port directly — **no `PR#4`, no
`$C800` monitor ROM**. The interactive `$C800` monitor and this library are two
independent clients of the same hardware.

## Files

| File | What |
|------|------|
| `SDRAMLIB.S` | The driver. `PUT`-include into any Merlin program. No `ORG`, no zero page. |
| `SDMTEST.S`  | The bank test. `ORG $2000`, `PUT SDRAMLIB`, BRUN it. |
| `SDMTEST` / `SDMTEST.po` | Build artifacts (assembled BIN / bootable disk). |

## Library API (`SDRAMLIB.S`)

Configure the slot with one equate at the top: `SLOT = 4` (→ `SDMBASE = $C0C0`).

**Param variables** (set, then call an entry point):

| Var | Size | Meaning |
|-----|------|---------|
| `SDM_BANK` | 2 | bank `$000–$3FF` (high 6 bits ignored) |
| `SDM_ADDR` | 2 | 16-bit offset within the bank |
| `SDM_VAL`  | 1 | value to write / last value read |

**Entry points** (`JSR`):

| Routine | Contract |
|---------|----------|
| `SDM_READY`  | Bounded wait for ready. **Returns `C=0` ready, `C=1` timeout** (card absent / wrong slot). Caller MUST branch on carry. |
| `SDM_SETBANK`| `SDM_BANK` → bank registers |
| `SDM_SETADDR`| `SDM_ADDR` → addr registers |
| `SDM_WRITE`  | set bank+addr, write `SDM_VAL`, wait |
| `SDM_READ`   | set bank+addr, read → `A` and `SDM_VAL`, wait |
| `SDM_WRNEXT` | write `SDM_VAL` at the auto-incremented addr (no addr re-set), wait |
| `SDM_RDNEXT` | read at the auto-incremented addr → `A`/`SDM_VAL`, wait |

**Handshake** is internal: every entry point sets the strobe then spins on
STATUS busy (bit7) until the access completes, so callers never poll manually.

**Auto-increment rules:**
- `addr` advances by one on each *completed access* (a `WRITE`/`WRNEXT` or a
  `READ`/`RDNEXT`). A bare DATA read does not advance — only the library's
  triggered ops do.
- `WRNEXT`/`RDNEXT` need a prior `SETADDR` (or `WRITE`/`READ`) to establish the
  start address; the hardware then walks forward by one per call.
- **No bank carry on wrap:** the 16-bit `addr` wraps `$FFFF→$0000` within the
  same bank. A walk past `$FFFF` needs an explicit `SETBANK`+`SETADDR`.

**Invariant:** the library uses only plain absolute `STA`/`LDA $C0Cx` (never
indexed) — a single `nDEVICE_SELECT` pulse each, dodging the `STA abs,X`
dummy-read double-count. Preserve this in any addition.

## Using it from your own Merlin source (on the Apple II)

```
        ORG $2000
        ...
        LDA #$22 : STA SDM_ADDR : LDA #$4D : STA SDM_ADDR+1   ; offset $4D22
        LDA #$00 : STA SDM_BANK : STA SDM_BANK+1               ; bank 0
        LDA #$AA : STA SDM_VAL
        JSR SDM_WRITE
        JSR SDM_READ        ; A = byte read back
        ...
        PUT SDRAMLIB        ; include the library (first is cleanest)
```
On disk the file is named **`SDRAMLIB.S`** (TXT, sequential, high-bit). Merlin-Pro
auto-appends `.S` to Load/Save/`PUT`/`USE`, so `L SDRAMLIB` and `PUT SDRAMLIB`
both resolve `SDRAMLIB.S`. (If a file ever lands without the `.S` suffix, Merlin
reports `46 FILE NOT FOUND` — rename it via the Disk command: `RENAME
SDRAMLIB,SDRAMLIB.S`.) Note: AppleCommander stamps TXT files with record length
$2000 by default — the build forces `--aux 0` (sequential), because a
random-access (`L=8192`) text file makes ProDOS copy utilities read past EOF and
crash.

## The bank test (`SDMTEST.S`)

Two-phase sweep over offsets `$0000–$0009` of every bank `0..1023`:
`expected(bank,off) = lo(bank) EOR hi(bank) EOR off EOR $5A`.
- **Phase 1** writes the expected value to every (bank, offset).
- **Phase 2** reads every (bank, offset) back and compares.

Writing *everything* before reading *anything* is what catches **bank aliasing**
(a dropped/stuck bank-address line) — an immediate read-after-write would mask
it. The value folds both bank bytes, so any **single-bit** bank-decode fault
yields a different expected value and is caught. (An 8-bit value can't uniquely
tag 1024 banks, so multi-bit aliases — banks ≥2 bits apart — are not
distinguished; those aren't reachable by a single-bit fault anyway.)

Output: `SDM BANK TEST` banner, a `.` every 64 banks, the first 16 mismatches as
`bbbb/oo ee gg` (bank/offset expected got), then `PASS` or `nnnn FAILURES`
(hex). `NO CARD` if `SDM_READY` times out (no hang).

## Build & run

```bash
make sdmtest        # assemble SDMTEST.S (+ SDRAMLIB) -> software/SDM/SDMTEST
make sdmdisk        # build bootable software/SDM/SDMTEST.po (BIN + lib source)
```
Deliver: copy `software/SDM/SDMTEST.po` to the ADTPro disks folder, send it to a
5.25" floppy, boot it (lands at `]`), then:
```
BRUN SDMTEST
```

## Bench verification checklist

1. Boot `SDMTEST.po` → `]` → `BRUN SDMTEST`.
2. Expect: `SDM BANK TEST`, ~16 `.` per phase, then **`PASS`**.
3. **Negative test** (proves the test actually checks): in `SDMTEST.S` change
   phase-2's `CALCEXP` constant `$5A`→`$5B` so every read mismatches → `make
   sdmdisk` → expect `FAILURES` count `2800` (hex = 10240) with the first 16
   mismatch lines, then revert.
4. Card absent at boot → `NO CARD` (not a hang, not a fail count).
5. Capture the screen via the `obs-screenshot` skill.

## Notes

- This is host-side software only — no FPGA rebuild or flashing.
- The card RTL is `gateware/rev2/project_obscurus/`; the interactive monitor is
  separate (`PR#4`).
