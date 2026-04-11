# Gateware

FPGA designs for the Byte Hamr Apple II expansion card, targeting a Lattice ECP5-85F with 64 MB SDRAM, 16 MB SPI flash, and a 12-pin GPIO header.

All designs are built with the open-source ECP5 toolchain: **Yosys** (synthesis) -> **nextpnr** (place & route) -> **ecppack** (bitstream).

## Designs

| Design | Purpose | Clocks |
|--------|---------|--------|
| [block_hamr](block_hamr/) | ProDOS block storage (SDRAM + flash persistence) | 7M / 25M |
| [flash_hamr](flash_hamr/) | SD card disk system with PicoRV32 CPU and FatFS | 7M / 25M |
| [smart_hamr](smart_hamr/) | IWM/SmartPort disk controller (ESP32 FujiNet bridge) | 7M only |
| [logic_hamr](logic_hamr/) | 8-channel logic analyzer with Apple II display | 25M |
| [signal_check](signal_check/) | Board bring-up: SDRAM test, GPIO walking 1s, bus monitor | 25M |
| [pattern_rom](pattern_rom/) | SDRAM read-path validation (test patterns) | 25M |
| [sdram_test](sdram_test/) | Direct register-driven SDRAM access (full 25-bit addressing) | 25M |
| [sdram_bank](sdram_bank/) | 4-bank independent SDRAM addressing PoC | 25M |
| [constraints](constraints/) | Shared pin constraint file (ECP5 LPF) | -- |

## Quick Start

```bash
make DESIGN=<name>              # Full bitstream build
make DESIGN=<name> prog         # JTAG program (volatile)
make DESIGN=<name> prog-flash   # Flash program (persistent)
make DESIGN=<name> sim          # Testbench simulation
make DESIGN=<name> report       # Timing and utilization summary
```

Default design (no `DESIGN=` argument) is `signal_check`.

## Disk Image Management

```bash
make DESIGN=block_hamr prog-flash-with-image DISK_IMAGE=path/to/disk.po
make write-flash DISK_IMAGE=new.po    # Swap disk without bitstream rebuild
make read-flash                       # Dump image from flash
```
