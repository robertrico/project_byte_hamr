# Logic Hamr

8-channel digital logic analyzer for the Apple II, implemented on the Byte Hamr FPGA card. Captures digital signals at 1 MHz, stores raw samples in SDRAM, and regenerates decimated display data in Apple II HIRES format.

## What It Does

Logic Hamr turns the Byte Hamr into a self-contained logic analyzer:

1. **Capture** - Arms on a configurable trigger (rising/falling edge on any channel), captures 8 bits of digital input at 1 MHz into SDRAM with a pre-trigger window
2. **Regenerate** - Processes raw samples through 8 parallel `decimate_pack` engines that convert to Apple II HIRES 7-bit byte format with configurable zoom
3. **Read** - Apple II software reads the display buffer (8 channels x 38 bytes) and renders waveforms to screen

Key capabilities:
- 4 window presets: 38us, 88us, 133us, 266us capture depth
- Configurable pre/post trigger split
- Rising or falling edge trigger on any channel
- **Re-zoom without recapture** - regenerate the same raw data at different zoom levels
- Built-in debug pattern generator with 8 test frequencies

## Register Map

Base address: `$C0C0` (slot 4)

| Offset | Name | R/W | Description |
|--------|------|-----|-------------|
| `$00` | CHANNEL | R/W | Select channel (0-7) for display read |
| `$01` | ADDR | R/W | Byte offset (0-37) within channel |
| `$02` | DATA | R | Display buffer read result |
| `$03` | CMD | W | `$02` = read, `$10` = regenerate, `$FF` = reset |
| `$04` | STATUS | R | init_done, busy, armed, captured, regen_done |
| `$05` | STRETCH | R | Current zoom stretch factor (1-7) |
| `$06` | TRIG_CH | R/W | Trigger channel (0-7) |
| `$07` | TRIG_MODE | R/W | `0` = rising edge, `1` = falling edge |
| `$08` | WINDOW | R/W | Window preset (0-3) |
| `$09` | ARM | W | Arm capture |
| `$0A` | DEBUG_EN | R/W | Enable debug test patterns |

## Probe Inputs

Connect digital signals to the GPIO header:

| GPIO | Channel |
|------|---------|
| GPIO4 | Channel 0 |
| GPIO5 | Channel 1 |
| GPIO6 | Channel 2 |
| GPIO7 | Channel 3 |
| GPIO8 | Channel 4 |
| GPIO10 | Channel 5 |

GPIO1 outputs a 250 kHz heartbeat. GPIO2/GPIO3 indicate ready/armed status.

## Architecture

```
Probes (GPIO) --> Capture Engine --> SDRAM (raw samples)
                                       |
                                       v
Apple II <-- Bus Interface <-- Regen Engine (8x parallel decimate_pack)
                                       |
                                       v
                               SDRAM (display buffer)
```

- **Capture engine**: Pre-trigger BRAM circular buffer + edge detector + SDRAM streaming
- **Regen engine**: 8 parallel `decimate_pack` instances process all channels in a single SDRAM pass
- **SDRAM controller**: 25 MHz, CAS latency 2, with pause_refresh during capture
- **Clock**: 25 MHz system, 1 MHz sample strobe, PHI0 sampled (not used as clock)

## Makefile Commands

All commands run from the project root (`project_byte_hamr/`).

```bash
# Build
make DESIGN=logic_hamr         # Build complete bitstream
make DESIGN=logic_hamr synth   # Synthesis only
make DESIGN=logic_hamr pnr     # Place & route only
make DESIGN=logic_hamr bit     # Generate bitstream

# Program
make DESIGN=logic_hamr prog        # JTAG (volatile)
make DESIGN=logic_hamr prog-flash  # SPI flash (persistent)

# Simulate
make DESIGN=logic_hamr sim         # Run testbench
make DESIGN=logic_hamr wave        # Simulation + GTKWave

# Unit test a specific module
make DESIGN=logic_hamr unit MODULE=capture_engine
make DESIGN=logic_hamr unit MODULE=decimate_pack
make DESIGN=logic_hamr unit-wave MODULE=regen_engine

# Reports
make DESIGN=logic_hamr report      # View build summary
make clean                         # Remove build artifacts
```

## Files

| File | Description |
|------|-------------|
| `logic_hamr_top.v` | Top-level integration - phase state machine, clock gen, probe sync |
| `capture_engine.v` | Trigger detection, pre-trigger buffer, raw sample capture |
| `regen_engine.v` | Parallel decimation controller - reads SDRAM, feeds 8 channels |
| `decimate_pack.v` | Converts 1-bit samples to HIRES 7-bit bytes with stretch/zoom |
| `sdram_controller.v` | SDRAM init, refresh, read/write with pause_refresh support |
| `bus_interface.v` | Apple II register file with PHI0 synchronization |
| `logic_hamr.lpf` | Pin constraints (design-specific) |
