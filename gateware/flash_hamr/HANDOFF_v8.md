# Flash Hamr v8 — Single-Drive-with-Swap (Session Handoff)

## Goal

Simplify Flash Hamr from a 4-unit multi-drive SmartPort device to a **single-drive device that swaps images**. The picker selects an SD card image, the firmware makes S4D1 serve that image, and the Apple II boots from it like a normal hard drive. No ghost drives, no multi-unit confusion. This is how FujiNet works and how Apple II hard drives worked in the 80s.

## Starting Point

Tag `v7.4-multidrive` (commit `ba32235`). Everything below this line describes that commit's state.

## What Works Today (v7.4)

- SmartPort card in Slot 4 with 4 units (S4D1-S4D4)
- S4D1: menu volume loaded from SPI flash at boot (280 blocks, has picker.sys + Bitsy Bye + BASIC.SYSTEM)
- S4D2-S4D4: mountable from SD card .PO/.2MG files via picker UI
- On-demand cache: first access reads block from SD → SDRAM, subsequent reads from SDRAM
- Persist writes: Apple II writes → SDRAM → firmware writes back to SD file via FatFS
- SD write reliability: 3 retries for command rejection, extended busy waits (5M iterations) for slow SDSC cards
- Merlin32 picker (picker.S): M)ount, U)nmount, B)oot, Q)uit
- Merlin32 ROM (hamr_rom.S): SmartPort + ProDOS protocol, STATUS with DIB, 4 units

## What's Wrong With Multi-Drive

1. **ProDOS ghost-maps units 3-4** to other slot numbers (S1D1/S1D2) — confusing, can't boot from them
2. **Picker always comes back** — S4D1 is permanently the picker volume, so every boot/reset shows the drive manager instead of the user's image
3. **$43 unit byte only handles 2 units** in the ROM boot handler — units 2-3 both map to $C0
4. **Multi-drive writes are unreliable** — 3 files open simultaneously triggers SD card GC storms with multi-second write latencies
5. **The use case is weak** — with 32MB max volumes, one drive holds everything. The softcore CPU can copy files between images directly.

## Target Architecture (v8)

### Single-drive UX flow:
1. **Cold boot (POR)**: S4D1 = picker volume (from SPI flash). Picker runs, shows SD card images.
2. **User selects image**: Firmware opens the .PO file, marks S4D1 as now backed by that SD file via on-demand cache. S4D1 block count updated to match the image.
3. **Reboot**: ROM reads block 0 from S4D1 (which now serves the selected image). ProDOS boots. $43=$40 (Slot 4 D1). Clean, standard boot.
4. **Ctrl-Reset**: ProDOS re-enters via its reset handler ($3F2-$3F4). S4D1 still serves the same image. Works exactly like a real hard drive.
5. **Return to picker**: User power-cycles (POR resets firmware, S4D1 back to SPI flash picker). Or a magic keystroke/register write that tells firmware to swap back.

### What changes:
- **ROM**: Simplify to 1 unit. Remove multi-unit STATUS/DIB. Boot always from unit 0. Remove ACTIVE_UNIT/BOOT_UNIT register complexity.
- **picker.S**: Remove slot selection from Mount. Just "pick image, boot it." Remove Unmount. Remove Boot (mounting IS booting).
- **bus_interface.v**: Remove unit_offset logic, active_unit mux. Single block_num, single block_count. Still needs the on-demand cache intercept and persist mechanisms.
- **cpu_soc.v**: Remove per-unit registers (MBOX_UNIT_BLKCNT, MBOX_UNIT_OFFSET). Simplify to single drive's block count.
- **firmware/main.c**: Remove drives[] array, find_drive(), multi-slot mount logic. Single FIL handle for the active image. On mount: open file, set block count, enable cache, trigger reboot.
- **flash_hamr_top.v**: Remove unit offset CDC wiring, simplify mux.

### What stays the same:
- PicoRV32 + FatFS architecture
- SPI SD card driver (spi_sd.c with retry logic)
- SDRAM as block cache (on-demand reads, persist writes)
- Cache bitmap (1 bit per block)
- Block buffer mechanism (bus_interface ↔ arbiter ↔ SDRAM)
- SPI flash boot loader (loads picker volume to SDRAM on POR)
- Magic block $FFFF for catalog data

### Return-to-picker mechanism (pick one):
- **Option A: Power cycle only** — simplest. POR resets firmware, reloads picker from SPI flash. User just power-cycles to switch images. No code needed.
- **Option B: Magic register** — picker installs a .SYSTEM file (PICKER.SYSTEM) on the mounted image. User runs it, it writes a "swap back" command to the firmware, firmware reloads picker from SPI flash, triggers reboot.
- **Option C: Open-Apple + Ctrl-Reset** — ROM checks keyboard modifier state on boot entry. If Open-Apple held, write a "reset to picker" command to firmware before booting. Requires reading the keyboard latch ($C061).
- **Recommended: Start with Option A, add Option B later.** Option C requires hardware changes (nRES is output-only on current board, so Ctrl-Reset can't be detected by FPGA).

## Hardware Architecture

- **FPGA**: Lattice ECP5 (LFE5U-25F) on custom "Byte Hamr" Apple II card
- **Clock**: 25 MHz system clock, Apple II 7M/PHI0 for bus timing
- **SDRAM**: 64 MB (32M x 16), used as block cache
- **SPI Flash**: Holds bitstream + picker .PO volume (boot_loader.v copies to SDRAM on POR)
- **SD Card**: SPI mode via GPIO pins, driven by PicoRV32 firmware
- **UART**: TX-only debug output via GPIO (115200 baud)
- **Bus interface**: nDEVICE_SELECT-driven, register-mapped I/O at $C0C0-$C0CF

## Key Files

| File | Lines | Purpose |
|------|-------|---------|
| `flash_hamr_top.v` | 657 | Top-level wiring, CDC, SDRAM mux |
| `bus_interface.v` | 387 | Apple II register I/O, block read/write state machine |
| `cpu_soc.v` | 621 | PicoRV32 + peripherals (SPI, UART, SDRAM port, mailbox) |
| `sdram_arbiter.v` | 300 | SDRAM read/write arbiter (DO NOT MODIFY — synthesis corruption) |
| `block_ready_gate.v` | 138 | Persist trigger gating (MUST stay separate module) |
| `block_buffer.v` | 120 | Dual-port block buffer (bus_interface ↔ SDRAM) |
| `boot_loader.v` | 170 | SPI flash → SDRAM copy on POR |
| `flash_reader.v` | 218 | SPI flash byte reader |
| `spi_master.v` | 193 | SPI master for SD card |
| `hamr_rom.S` | 586 | 65C02 ROM (Merlin32) — SmartPort + ProDOS protocol |
| `picker.S` | 679 | 65C02 picker UI (Merlin32) — ProDOS .SYS file |
| `firmware/main.c` | 616 | PicoRV32 firmware — SD init, mount, cache, persist |
| `firmware/spi_sd.c` | 387 | SD card SPI driver with retry logic |
| `firmware/diskio.c` | 93 | FatFS disk I/O layer |
| `firmware/hal.h` | 128 | Memory-mapped peripheral addresses |

## Register Map (bus_interface, offset from $C0C0)

Current (v7.4 multi-drive):
```
0: STATUS(R) / COMMAND(W)    5: DATA_WRITE(W)
1: DATA_READ(R)              6: TOTAL_BLOCKS_LO(R)
2: BLOCK_LO(R/W)            7: TOTAL_BLOCKS_HI(R)
3: BLOCK_HI(R/W)            8: SD_STATUS(R) / SD_CMD(W)
4: SOFT_RESET(W)             9: IMG_SELECT(W)
B: UNIT_BLK_LO(R)           D: BOOT_UNIT(R) / ACTIVE_UNIT(W)
C: UNIT_BLK_HI(R)           E: BOOT_UNIT(W)
```

v8 target (simplified): remove B/C/D/E multi-unit registers. Registers 6/7 serve the single drive's block count directly.

## Critical Lessons (Don't Repeat These)

1. **sdram_arbiter.v MUST NOT be modified** — adding states causes Yosys synthesis corruption that passes simulation but fails on hardware
2. **block_ready_gate.v MUST be a separate module** — same Yosys issue
3. **hamr_rom.mem staleness** — always verify generated artifacts (ROM .bin → .mem) are in the build graph
4. **ProDOS ZP $42-$47 must NOT be modified** by the ROM driver — causes Relocation/Configuration Error
5. **STA abs,X is 5 cycles** — cycle 4 is a dummy READ, use separate read/write register addresses
6. **posedge nDEVICE_SELECT** is correct write capture point
7. **$C800 bus contention** — expansion ROM causes errors with other cards. Current ROM fits in $Cn page + $C800, deferred.
8. **SDSC byte addressing** — this SD card has OCR=80FF8000 (CCS=0), needs `addr *= 512` in diskio.c
9. **SD write busy waits need 5M+ iterations** — SDSC cards doing GC after multi-file seeks can take seconds

## Build

```bash
cd ~/Development/project_byte_hamr
make clean && make DESIGN=flash_hamr                    # build only
make DESIGN=flash_hamr prog-flash                       # flash FPGA (ask first!)
make DESIGN=flash_hamr prog-flash-with-image DISK_IMAGE=gateware/flash_hamr/menu.po
```

Merlin32 assembly (picker + ROM):
```bash
Merlin32 -V . picker.S     # → picker.bin → picker.sys (via build graph)
Merlin32 -V . hamr_rom.S   # → hamr_rom.bin → hamr_rom.mem
```

## SD Card Images

On `/Volumes/FUJI`:
- `PRODOS.PO` — reference ProDOS 2.4.1 image (source for boot blocks + system files)
- `LEAN.PO` — 280 blocks, /LEAN, PRODOS + Bitsy Bye + BASIC.SYSTEM
- `EIGHT.PO` — 8MB (16384 blocks), /EIGHT, same system files
- `TWELVE.PO` — 12MB (24576 blocks), /TWELVE, same system files
- `PD.2MG` — 32MB 2MG image

Use `/mkprodos` skill to create new images (AppleCommander acx for any size).

## What to Do

1. Read the current source files to understand the full state
2. Simplify ROM (hamr_rom.S) to single unit — remove multi-unit STATUS/DIB, ACTIVE_UNIT, BOOT_UNIT
3. Simplify bus_interface.v — remove unit_offset, active_unit mux
4. Simplify cpu_soc.v — remove per-unit registers
5. Simplify firmware/main.c — single drive, mount = swap S4D1 backing
6. Simplify picker.S — select image → mount → reboot (one action, not three)
7. Update flash_hamr_top.v — remove unit CDC wiring
8. Test: mount LEAN.PO, boot into BASIC, CREATE/SAVE, cold reboot, verify persist
