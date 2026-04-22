# Flash Hamr

Full-featured SD card-based disk system with an embedded PicoRV32 (RISC-V) CPU running FatFS, a Merlin picker menu, and multi-drive support.

## What It Does

- Embeds a **PicoRV32 RISC-V CPU** running at 25 MHz with 32KB IMEM and 16KB DMEM
- Reads `.po`/`.2mg` disk images from **SD card** via SPI using FatFS (FAT32)
- Presents up to **4 drives** (S4D1-S4D4) to the Apple II as ProDOS block devices
- **Magic block interception**: CPU fills the block buffer for catalog/metadata blocks (`$FFFF`/`$FFFE`)
- **On-demand caching**: Optionally intercepts all reads for performance
- **Menu volume**: Picker program + ProDOS on SPI flash for disk image selection at boot
- Toggle-based **mailbox CDC** for safe 7 MHz / 25 MHz cross-domain communication

## Architecture

```
PicoRV32 CPU (25 MHz)
  +-- IMEM (32KB, firmware.mem)
  +-- DMEM (16KB, stack/heap)
  +-- SPI Master -> SD card (GPIO1-4)
  +-- UART TX -> serial debug
  +-- GPIO output [7:0]

Apple II Bus (7.16 MHz)
  +-- Boot ROM (4KB, $C0C0/$C400/$C800)
  +-- bus_interface (register file)
  |     +-- Block request arbiter
  |     +-- Magic block detection
  |     +-- Cache gating
  +-- Block buffer (512B dual-port)
  +-- SDRAM (shared with CPU)

Mailbox (toggle-CDC): 7 MHz <-> 25 MHz
  - Command/status, persist requests, unit metadata
```

## CPU Memory Map

| Address | Size | Purpose |
|---------|------|---------|
| `0x00000000` | 32KB | IMEM (firmware) |
| `0x10000000` | 16KB | DMEM (stack/heap) |
| `0x20000000` | -- | SPI Master (SD card) |
| `0x30000000` | -- | Mailbox |
| `0x40000000` | -- | SDRAM port |
| `0x50000000` | -- | UART TX |
| `0x60000000` | -- | Block buffer port |
| `0x70000000` | -- | GPIO output |

## Key Modules

### Gateware
| File | Purpose |
|------|---------|
| `flash_hamr_top.v` | Top-level integration (CDC, multiplexing) |
| `cpu_soc.v` | PicoRV32 wrapper (IMEM, DMEM, peripherals) |
| `picorv32.v` | PicoRV32 CPU core |
| `bus_interface.v` | Enhanced register file (multi-unit support) |
| `block_buffer.v` | 512B dual-port BRAM |
| `boot_loader.v` | Flash-to-SDRAM copy, ProDOS header parsing |
| `sdram_controller.v` | SDRAM interface |
| `sdram_arbiter.v` | Mux boot/block/CPU requests |
| `mailbox.v` | Toggle-CDC mailbox |
| `spi_master.v` | SD card SPI interface |
| `uart_tx.v` | Serial debug output |

### Firmware (C / RISC-V)
| File | Purpose |
|------|---------|
| `main.c` | Main program (SD mount, FatFS, catalog) |
| `spi_sd.c/h` | SD card SPI driver |
| `diskio.c/h` | FatFS disk I/O layer |
| `ff.c/h` | FatFS filesystem library |
| `hal.h` | Hardware abstraction layer |
| `crt0.S` | RISC-V startup |
| `linker.ld` | Linker script |

### 6502 Boot
| File | Purpose |
|------|---------|
| `hamr_rom.S` | ProDOS block driver (Merlin32) |
| `picker.S` | Menu picker (selects disk image) |
| `images/menu.po` | ProDOS disk image with picker + ProDOS |

## Build

```bash
make DESIGN=flash_hamr              # Build gateware
make DESIGN=flash_hamr prog         # JTAG program
make DESIGN=flash_hamr prog-flash   # Flash program (persistent)
make menu                           # Build menu volume
make DESIGN=flash_hamr report       # Timing and utilization
make DESIGN=flash_hamr sim          # Testbench simulation
```
