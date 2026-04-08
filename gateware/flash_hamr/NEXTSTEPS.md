# Flash Hamr — Next Steps

## Context
Full plan: `/Users/hambook/.claude/plans/merry-wandering-mccarthy.md`

PicoRV32 + FatFS SD card controller. Phases 1-9 substantially complete.
Branch: `PicoRV32FatFs`. Commit history has Phase 1-8 in one commit,
Phase 9 (picker + mount rewrite) in the next.

## What Works (hardware verified)
- PicoRV32 CPU at 25MHz, UART TX debug on GPIO7 (115200 baud)
- SPI master at 12.5 MHz talks to SD card (SDSC + SDHC)
- FatFS mounts FAT16/FAT32, scans root dir for .PO/.2MG files
- Mailbox CDC (toggle-based) bridges 7MHz Apple II bus ↔ 25MHz CPU
- Magic block intercept: READ_BLOCK $FFFF → CPU fills block buffer with catalog
- Picker (A.PICKER.SYSTEM on flash volume) shows catalog, user selects image
- Mount: streams SD → SDRAM at block 0 via FatFS f_read + SDRAM port
- JMP $C400 reboots into mounted image — ProDOS boots, CATALOG works
- Persist WORKS: CREATE, SAVE survive cold reboot (Phase 10)

## Bug: CREATE/SAVE I/O ERROR — FIXED (Phase 10)

Three bugs were found and fixed:

### Bug 1: gated_block_ready glitch (block_ready_gate.v)
block_hamr's write_through runs at 25MHz (same domain as arbiter), so the
trigger fires on the EXACT cycle arb_block_ready rises — no glitch.
flash_hamr's block_ready_gate runs at 7MHz. arb_block_ready (25MHz) passes
directly through the combinational gated output for 1-2 cycles before the
7MHz edge detector fires the trigger. bus_interface captures this glitch
and prematurely exits S_BUSY.

**Fix**: Use `arb_ready_delayed` (1-FF registered copy of arb_block_ready) for
the gated output. The edge detector and delay register sample at the same
clock edge, so trigger fires BEFORE the delayed signal goes HIGH.
```verilog
assign gated_block_ready = (holding || trigger) ? 1'b0 : arb_ready_delayed;
```

### Bug 2: UART blocking during persist (main.c)
The old code printed 45+ UART chars (~4ms) BEFORE f_write/f_sync. Combined
with SD card busy time, the total exceeded the slot ROM's ~1.28s wait_ready
timeout. ProDOS timed out before persist finished.

**Fix**: Move ALL UART output to after PERSIST_DONE. The hot path is now:
buffer read → f_lseek → f_write → f_sync → PERSIST_DONE → then print.

### Bug 3: SD card SPI idle wake-up (spi_sd.c / main.c)
After mount streams 280 blocks to SDRAM, the SD card is idle for seconds
(ProDOS boots, user types commands). Some SDSC cards need an SPI transaction
to "wake up" — the first CMD17 after idle returns tok=0x00 (MISO stuck LOW).

**Fix**: f_lseek(0) + f_read(512) after mount primes the SPI interface.
Also increased sd_wait_response timeout from 4096 to 200000 iterations
(~4ms → ~200ms) for cards with slow TAAC.

### Status: VERIFIED on hardware
CREATE DEV (3 writes) + SAVE TEST (5 writes) = 8 persists, all OK.
Data survives cold reboot.

## Other known issues

### Picker timeout for large images
The mount poll loop in `assemble_picker.py` has a 60-second timeout, but
large images (PD8.PO = 8MB) take ~17 seconds at 12.5MHz SPI. Should be OK.
The 32MB .2MG takes ~2 minutes which exceeds the timeout.
**Fix**: Either increase timeout or (better) implement on-demand block reads
instead of preloading entire image to SDRAM.

### Double SD init on boot
The firmware calls `sd_init()` directly, then `f_mount()` calls `disk_initialize()`
which calls `sd_init()` again. Harmless but wastes ~500ms.
**Fix**: Skip the direct `sd_init()` call, let `f_mount()` handle it.

### On-demand reads (future optimization)
Current architecture preloads entire disk image to SDRAM. With FatFS, the CPU
could intercept each READ_BLOCK, f_lseek to the right offset, f_read 512 bytes,
and return via block buffer. No SDRAM needed for disk data. Boot would be instant
for any size image. Writes would f_write directly to SD.
This is a significant architecture change but eliminates the SDRAM bottleneck.

## File map

### RTL (gateware/flash_hamr/)
- `flash_hamr_top.v` — top level with SDRAM mux, magic block intercept, all wiring
- `cpu_soc.v` — PicoRV32 + bus decoder + IMEM/DMEM + all peripherals
- `picorv32.v` — CPU core (downloaded, unmodified)
- `spi_master.v` — SPI Mode 0, dual-speed (195kHz/12.5MHz)
- `uart_tx.v` — 115200 baud TX-only
- `mailbox.v` — toggle-CDC between 7MHz bus and 25MHz CPU
- `block_ready_gate.v` — persist gating (renamed write_through, own module)
- `sdram_controller.v` — SDRAM driver (unchanged from block_hamr)
- `sdram_arbiter.v` — DO NOT MODIFY (Yosys synthesis corruption)
- `block_buffer.v` — 512B dual-port BRAM (unchanged)
- `bus_interface.v` — Apple II register interface (minor mods: sd_cmd_data/wr ports)
- `boot_loader.v` — SPI flash → SDRAM at power-on (unchanged)
- `flash_reader.v` — SPI flash reader (unchanged)
- `addr_decoder.v`, `boot_rom.v` — slot ROM (unchanged)
- `assemble_rom.py` — slot ROM assembler (unchanged)

### Firmware (gateware/flash_hamr/firmware/)
- `main.c` — command loop: SD init, catalog, mount, persist
- `spi_sd.c/h` — SD card SPI driver (CMD0/8/55/41/58/17/24)
- `diskio.c` — FatFS disk I/O hooks
- `ff.c/h`, `ffconf.h`, `diskio.h` — elm-chan FatFS R0.15
- `hal.h` — peripheral register addresses + UART helpers
- `libc_stubs.c` — memcpy, memset, memcmp, strchr, strlen
- `crt0.S` — startup (stack, BSS zero, data copy)
- `linker.ld` — memory map (IMEM 32KB @ 0x00000000, DMEM 16KB @ 0x10000000)

### Picker (gateware/flash_hamr/)
- `assemble_picker.py` — 6502 picker assembler (produces picker.sys)
- `build_menu.sh` — builds menu.po (PRODOS + A.PICKER.SYSTEM)
- `picker.sys` — assembled binary
- `menu.po` — base volume for SPI flash

### Build
- `make firmware` — compile C → .elf → .bin → firmware.mem
- `make DESIGN=flash_hamr` — synthesize (includes firmware as dependency)
- `make DESIGN=flash_hamr prog-flash-with-image DISK_IMAGE=gateware/flash_hamr/menu.po`
- `scripts/bin2mem.py` — binary to $readmemh hex converter

### Key constants
- Slot 4 hardcoded (picker + assemble_rom.py)
- SPI clock: 195kHz init, 12.5MHz data
- IMEM: 32KB, DMEM: 16KB
- SDRAM: mount writes at byte address 0 (overwrites menu volume)
- Firmware ~16KB, Picker ~800 bytes
- Resources: ~3,400 LUTs (4%), 24 BRAM (11%)
