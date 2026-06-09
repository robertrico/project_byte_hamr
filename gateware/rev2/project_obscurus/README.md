# project_obscurus

Apple II SDRAM monitor for the Byte Hamr Rev 2. A self-contained 6502 monitor
in the card's `$C800` expansion ROM does live random-access read/write to the
full 64 MB SDRAM through a `$C0Cx` register port, with 64 KB bank switching.
No disk image, no FujiNet, no external probing — everything ships in the
bitstream and runs on the Apple II keyboard + screen. Slot 4 hardcoded.

Foundation for later "hardware support for software" experiments. (Supersedes
the original boot-time "HELLO, WORLD! / HELLO, SDRAM!" string demo.)

## What it does

On power-on the `sdram_ctrl` controller initializes SDRAM (precharge, 2×
refresh, mode register CL=2 / burst=1) and then idles, servicing one byte
read/write per request with a free-running, priority refresh. From the Apple II:

```
PR#4
```

does `JSR $C400`. The `$C400` stub **arms the `$C800` expansion ROM** (writes
`$AA` to `$C0C7`), restores the BASIC output vector (`CSW = $FDF0` / COUT1) so
screen output works, then `JMP $C800` into the monitor. The monitor prints a
banner + `*` prompt and accepts commands:

| Command       | Action                                            |
| ------------- | ------------------------------------------------- |
| `R aaaa`      | Read one byte; prints `aaaa: bb`                  |
| `W aaaa bb`   | Write byte `bb` to offset `aaaa` in current bank  |
| `B bbb`       | Set bank (`$000`–`$3FF`)                          |
| `D aaaa`      | Dump 16 bytes from `aaaa` (hex + ASCII)           |
| `T`           | Run the 1024-bank sweep; prints `TEST PASS`/`nnnn TEST FAIL` |
| `Q`           | Disarm `$C800` and return to BASIC                |

All numbers are hex. `Q` jumps to a `DISARM` routine at `$C420` in the always-live
slot ROM, which writes `$AA` to `$C0C8` and `RTS`es to BASIC.

## Expansion ROM arming (the `$C800` bus-contention fix)

`$C800–$CFFF` is a **shared** Apple expansion-ROM bus. The card is **silent**
there by default — it only drives it when software arms it:

- **Arm:** write `$AA` to `$C0C7`. **Disarm:** write `$AA` to `$C0C8`. Any other
  value is ignored (symmetric magic guard).
- **Reset / Ctrl-Reset / POR auto-disarm** — a crash recovers to silent.
- The card drives `$C800–$CFFF` only when `rom_en & rom_armed`. With `rom_armed=0`
  by default, **CATALOG, file copies, and disk I/O never collide** with the //e
  internal ROM or other cards (this was the `$CEF3`-crash bug).
- Unused expansion ROM (`$CA08–$CFFE`) is filled with `$60` (RTS), so a stray
  fetch returns harmlessly.

**Software contract:**
- **SDRAM data access (`$C0C0–$C0C6`) never arms** — it's slot-gated and always
  safe. `SDMTEST`/`SDRAMLIB` use only this path.
- **Arm only to run `$C800` ROM code** (`arm → run → disarm`). Keep the window
  short; don't do disk I/O or call other `$C800` firmware while armed (`SEI` if an
  IRQ could touch `$C800`). `rom_en` must also be set (a `$C4xx` access) to drive
  `$C800` — `PR#4` does this via `JSR $C400`.
- Timing: `rom_armed` settles in ~80 ns (≪ one 6502 instruction) — no delay needed
  after `STA $C0C7`. `SDRAMLIB` provides `SDM_ROMON`/`SDM_ROMOFF` helpers.

## Memory model

- SDRAM AS4C32M16 = 32M × 16 = **64 MB**, **byte-packed** (two bytes per 16-bit
  word, DQM-selected): `phys_word = phys_byte[25:1]`, `phys_byte[0]` picks the
  lane.
- Physical byte address = `{bank[9:0], addr[15:0]}` = 26 bits = **1024 banks of
  64 KB**. Each bank is a full 6502-style 64 KB space; `R 4D22` reads offset
  `$4D22` inside the selected bank.

## Register port (slot 4, `$C0Cx`)

| Address       | R/W | Name      | Description                                       |
| ------------- | --- | --------- | ------------------------------------------------- |
| `$C0C0`       | W   | ADDR_LO   | `addr[7:0]`                                       |
| `$C0C1`       | W   | ADDR_HI   | `addr[15:8]`                                      |
| `$C0C2`       | W   | BANK_LO   | `bank[7:0]`                                        |
| `$C0C3`       | W   | BANK_HI   | `bank[9:8]`                                        |
| `$C0C4`       | W   | TRIG_RD   | Write any byte → start SDRAM read at `{bank,addr}`|
| `$C0C5`       | R   | STATUS    | **bit7 = busy**, **bit6 = ready**; others 0       |
| `$C0C6`       | W   | DATA      | Write → SDRAM write at `{bank,addr}`              |
| `$C0C6`       | R   | DATA      | Read → last SDRAM read result (RD_HOLD)           |
| `$C0C7`       | W   | ROM_ARM   | Write `$AA` → arm `$C800` ROM (other values ignored) |
| `$C0C8`       | W   | ROM_DISARM| Write `$AA` → disarm `$C800` ROM                  |
| `$C0C9-$C0CF` | R/W | SCRATCH   | Loopback registers                                |
| `$C400-$C4FF` | R   | slot ROM  | `$C400` stub (`slot_rom.S`)                        |
| `$C800-$CFFF` | R   | exp ROM   | monitor program (`monitor.S`); `$CFFF` unused     |

**Handshake.** SDRAM latency >> one 6502 bus cycle, so there is no in-cycle
round-trip. Sequence: set ADDR/BANK → strobe (TRIG_RD or DATA write) → **poll
STATUS until bit7 (busy) clears** → read DATA / done.

- `BIT $C0C5` lands busy→N and ready→V, so polling is branchless:
  `BIT $C0C5 : BMI *-3` (loop while busy); `BIT $C0C5 : BVC *-3` (wait ready).
- `busy` is **sticky** from the strobe until the first STATUS read after the
  access completes, so the first poll always observes busy=1 even for a fast op.
- `addr` **auto-increments on access completion** (not on the DATA read), so a
  `D` dump loop is: set start addr once, then `{strobe; poll; read}` ×N. A bare
  DATA read with no strobe does not advance `addr`.

`STA $C0Cx` / `LDA $C0Cx` absolute = 4 cycles, single `nDEVICE_SELECT` pulse —
no indexed dummy-read double-step. (CLAUDE.md "6502 Bus Timing Gotchas".)

## Expansion ROM enable

`rom_en` flip-flop: set on `$Cn00` access (`nI_O_SELECT`), cleared on `$CFFF`
access. Gates `~nI_O_STROBE` into both the U12 `DATA_OE` and the `$C800` data
drive. **`$CFFF` is reserved unused** — touching it disables the ROM; the
monitor (520 bytes) lives only in `$C800–$CA07`.

## Files

| File                      | Description                                         |
| ------------------------- | --------------------------------------------------- |
| `project_obscurus_top.v`  | Top — clock, POR, bus iface, monitor_regs, exp ROM  |
| `sdram_ctrl.v`            | SDRAM controller: init, priority refresh, byte R/W  |
| `sim/sdram_model.v`       | Behavioral SDRAM model (bounded dense) for sim      |
| `sdram_ctrl_tb.v`         | Unit TB for `sdram_ctrl`                             |
| `project_obscurus_tb.v`   | Integration TB (register-port protocol)             |
| `slot_rom.S` / `.mem`     | `$C400` stub (Merlin32)                             |
| `monitor.S` / `.mem`      | `$C800` monitor program (Merlin32)                  |

## Build & run

```bash
make clean && make DESIGN=project_obscurus REV=rev2     # bitstream
make DESIGN=project_obscurus REV=rev2 sim               # integration TB
make unit DESIGN=project_obscurus MODULE=sdram_ctrl REV=rev2   # controller unit TB
make assemble ASM_SRC=gateware/rev2/project_obscurus/monitor.S # reassemble monitor
```

> **iverilog note:** this machine's Icarus 13.0 hangs on `-g2009`/`-g2012`; the
> build uses default `-g2005`. The SDRAM sim model is a bounded dense array
> (64K words, banks 0–1) rather than a SystemVerilog associative array.

To program flash (only when you intend to flash):
`make DESIGN=project_obscurus REV=rev2 prog-flash`. After flashing, type `PR#4`.

## Bench verification checklist

**The regression that motivated the arm switch (do this first):**
1. Card installed, **without** `PR#4`: `CATALOG`, then **copy a file** (CopyUtils)
   → no crash, no `$CEF3`, disks intact. (The card is disarmed = silent on `$C800`.)

**Monitor + test:**
2. `PR#4` → monitor banner + `*` prompt.
3. `W 0000 AA`, then `R 0000` → `0000: AA`.
4. `B 1`, `R 0000` → value ≠ `AA` (bank isolation); `B 0`, `R 0000` → `AA`.
5. `D 0000` → 16-byte hex + ASCII dump, first byte `AA`.
6. `T` → `TEST PASS`.
7. `Q` → returns to BASIC `]` prompt.
8. After `Q`: `CATALOG` / copy a file again → still clean (card re-disarmed).
9. Ctrl-Reset mid-monitor → back to BASIC, card silent, disks fine.

Capture via the `obs-screenshot` skill.

## Why "obscurus"

Placeholder name — this is the foundation. The next thing built on top will
need a real name.
