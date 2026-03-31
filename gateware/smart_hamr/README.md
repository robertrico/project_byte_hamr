# Smart Hamr

Apple IIe-compatible Liron (IWM) disk controller for the Byte Hamr FPGA card. Emulates the Integrated Woz Machine to provide a SmartPort interface that communicates with an ESP32 running FujiNet-compatible disk emulation firmware.

## What It Does

Smart Hamr implements a complete IWM disk controller in gateware, allowing the Apple II to use SmartPort-protocol disk drives provided by an ESP32 companion microcontroller:

1. **IWM Emulation** - Full Apple II disk controller with phase control, serial data I/O, motor control, and mode register
2. **SmartPort Bridge** - Translates between IWM bus protocol and ESP32 via 4-bit phase pins + control signals
3. **Boot ROM** - 4KB Liron firmware (UniDisk 3.5 interface card, A2M2053) for ProDOS SmartPort support
4. **Dual drive** - Drive 1 and Drive 2 select with SmartPort drive gating

Ported from Steve Chamberlin's Yellowstone project (originally MachXO2), adapted for the ECP5 and Byte Hamr hardware.

## Memory Map (Slot 4)

| Address Range | Signal | Content |
|---------------|--------|---------|
| `$C0C0-$C0CF` | nDEVICE_SELECT | IWM registers (8 state latches) |
| `$C400-$C4FF` | nI_O_SELECT | Boot ROM bytes `$000-$0FF` |
| `$C800-$CFFF` | nI_O_STROBE | Expansion ROM bytes `$100-$7FF` |

## ESP32 GPIO Connections

FujiNet Rev0-compatible pinout on the GPIO header:

| GPIO | Direction | Signal | ESP32 Pin |
|------|-----------|--------|-----------|
| GPIO1 | Out | _enbl1 (Drive 1 enable) | IO36 |
| GPIO2-5 | Out | phase[3:0] (SmartPort cmd) | IO34/35/32/33 |
| GPIO7 | Out | _wrreq (Write request) | IO26 |
| GPIO8 | In | sense/ACK | IO27 |
| GPIO9 | In | rddata (inverted) | IO14 |
| GPIO10 | Out | wrdata (Serial write) | IO22 |
| GPIO11 | Out | _enbl2 (Drive 2 enable) | IO21 |

## Architecture

```
Apple II Bus (sig_7M = 7.16 MHz)
       |
  addr_decoder.v ──> boot_rom.v (4KB Liron firmware)
       |
  iwm.v (583 lines)
  ├── /DEV-clocked state latches (phase, motor, Q6/Q7)
  ├── Read shift register (GCR + latch mode)
  ├── Write shift register + bit-cell timer
  ├── Mode register (SmartPort/fast/async/timer)
  └── Drive gating (forces /ENBLx HIGH during SmartPort)
       |
  GPIO header ──> ESP32 (FujiNet firmware)
```

**Single clock domain** - everything runs from `sig_7M` (7.16 MHz Apple II clock). No 25 MHz oscillator used.

### Key Design Details

- **DEV-clocked latches**: IWM state latches clock on nDEVICE_SELECT falling edge, then 2-FF sync into fclk domain (280ns latency)
- **SmartPort drive gating**: Forces /ENBL1 and /ENBL2 inactive during SmartPort sessions to prevent ESP32 from seeing false Disk II mode
- **Combinatorial ROM**: No registered BRAM - uses LUT logic for async read to avoid metastability from address bus glitches
- **Drain delay**: 1430-cycle delay after write underrun gives ESP32 time to decode command packets
- **rddata inversion**: ESP32 sends idle-LOW, IWM expects idle-HIGH - simple `~GPIO9`

## Makefile Commands

All commands run from the project root (`project_byte_hamr/`).

```bash
# Build gateware
make DESIGN=smart_hamr         # Build complete bitstream
make DESIGN=smart_hamr synth   # Synthesis only
make DESIGN=smart_hamr pnr     # Place & route only
make DESIGN=smart_hamr bit     # Generate bitstream

# Program FPGA
make DESIGN=smart_hamr prog        # JTAG (volatile)
make DESIGN=smart_hamr prog-flash  # SPI flash (persistent)

# Simulate
make DESIGN=smart_hamr sim         # Run testbench
make DESIGN=smart_hamr wave        # Simulation + GTKWave

# ESP32 firmware (requires: source ~/esp/esp-idf/export.sh)
make esp-build                 # Build ESP32 firmware
make esp-flash                 # Flash to ESP32
make esp-monitor               # Serial monitor
make esp-all                   # Build + flash + monitor
make esp-help                  # Full ESP32 target list

# Reports
make DESIGN=smart_hamr report  # View build summary
make clean                     # Remove build artifacts
```

## Files

| File | Description |
|------|-------------|
| `smart_hamr_top.v` | Top-level - bus mux, GPIO registration, level shifter OE |
| `iwm.v` | Full IWM controller - state latches, shift registers, timing |
| `addr_decoder.v` | Expansion ROM state flag and output enable |
| `boot_rom.v` | 4KB combinatorial ROM (Liron firmware) |
| `smart_hamr_tb.v` | Testbench - register, ROM, drive, and serial read tests |
| `smart_hamr.lpf` | Pin constraints (design-specific, sig_7M clock) |
| `liron_rom.mem` | Liron firmware hex data (loaded at synthesis) |
| `liron.asm` | UniDisk 3.5 drive firmware disassembly (reference) |
| `liron-if.asm` | UniDisk 3.5 interface card firmware disassembly (reference) |
| `docs/` | IWM documentation PDFs + phantom_read.txt |
| `firmware/` | ESP32 firmware projects (FujiNet-compatible) |
