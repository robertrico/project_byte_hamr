# Logic Hamr v1 - Capture Engine

## Overview

Logic Hamr v1 is an 8-channel logic analyzer for the Byte Hamr FPGA card. It captures real-time signals at 1 MHz sample rate, detects trigger conditions with pre-trigger buffering, and generates Apple II HIRES-compatible display data through decimation.

## Features

- 8-channel simultaneous capture via GPIO5-12
- Configurable trigger: rising or falling edge on any channel
- Pre-trigger buffering via BRAM circular buffer
- 4 window presets (38us, 88us, 133us, 266us)
- Built-in debug test pattern generator for testing
- SDRAM-backed capture and display buffers

## GPIO Pin Assignment

| Pin | Direction | Function |
|-----|-----------|----------|
| GPIO1 | Output | Heartbeat (250kHz square wave) |
| GPIO2 | Output | Ready (init complete + display loaded) |
| GPIO3 | Output | Armed (waiting for trigger) |
| GPIO4 | Output | Captured (capture complete) |
| GPIO5 | Input | PROBE[0] |
| GPIO6 | Input | PROBE[1] |
| GPIO7 | Input | PROBE[2] |
| GPIO8 | Input | PROBE[3] |
| GPIO9 | Input | PROBE[4] |
| GPIO10 | Input | PROBE[5] |
| GPIO11 | Input | PROBE[6] |
| GPIO12 | Input | PROBE[7] |

## Register Map

All registers are at offsets from $C0C0 (slot 4) or the appropriate slot base address.

| Offset | Name | R/W | Description |
|--------|------|-----|-------------|
| $00 | CHANNEL | R/W | Channel select (0-7) for display buffer read |
| $01 | ADDR | R/W | Byte index within channel (0-37) |
| $02 | DATA | R | Read result from SDRAM display buffer |
| $03 | CMD | W | Command register (see below) |
| $04 | STATUS | R | Status register (see below) |
| $05 | STRETCH | R | Stretch factor (from window preset, read-only) |
| $06 | TRIG_CH | R/W | Trigger channel (0-7) |
| $07 | TRIG_MODE | R/W | Trigger mode (0=rising edge, 1=falling edge) |
| $08 | WINDOW | R/W | Window preset (0-3) |
| $09 | ARM | W | Write any value to ARM capture engine |
| $0A | DEBUG_EN | R/W | Debug pattern enable (0=real probes, 1=test pattern) |

### Command Register ($03)

| Value | Action |
|-------|--------|
| $02 | Read byte from display buffer (use CHANNEL/ADDR first) |
| $10 | Regenerate display buffer from captured data |

### Status Register ($04)

| Bit | Name | Description |
|-----|------|-------------|
| 0 | BUSY | SDRAM operation in progress |
| 1 | READY | SDRAM initialized AND display buffer loaded |
| 2 | ARMED | Capture engine armed, waiting for trigger |
| 3 | CAPTURED | Capture complete, data available |

## Window Presets

| Preset | Total Samples | Pre-trigger | Post-trigger | Stretch | Time Window |
|--------|--------------|-------------|--------------|---------|-------------|
| 0 | 38 | 2 | 36 | 7 | 38 us (max zoom) |
| 1 | 88 | 4 | 84 | 3 | 88 us (default) |
| 2 | 133 | 7 | 126 | 2 | 133 us |
| 3 | 266 | 13 | 253 | 1 | 266 us (min zoom) |

The stretch factor determines how many pixels each sample occupies on screen:
- Stretch 1: Each sample = 1 pixel (compressed view, 266 samples visible)
- Stretch 7: Each sample = 7 pixels (zoomed view, 38 samples visible)

## Capture Flow

1. **Configure trigger**: Set TRIG_CH (0-7) and TRIG_MODE (0=rising, 1=falling)
2. **Select window**: Set WINDOW preset (0-3) based on desired time scale
3. **ARM capture**: Write any value to ARM register ($09)
4. **Wait for trigger**: Monitor STATUS bit 3 (CAPTURED) or GPIO4
5. **Regenerate display**: Write $10 to CMD register ($03)
6. **Wait for ready**: Monitor STATUS bit 1 (READY) or GPIO2
7. **Read display data**: Set CHANNEL/ADDR, write $02 to CMD, read DATA

## Debug Test Pattern

When DEBUG_EN ($0A) is set to 1, the probe inputs are replaced with an internally generated test pattern. Each channel generates a different frequency square wave:

| Channel | Frequency |
|---------|-----------|
| PROBE[0] | 3.9 kHz |
| PROBE[1] | 7.8 kHz |
| PROBE[2] | 15.6 kHz |
| PROBE[3] | 31.25 kHz |
| PROBE[4] | 62.5 kHz |
| PROBE[5] | 125 kHz |
| PROBE[6] | 250 kHz |
| PROBE[7] | 500 kHz |

This allows testing the capture engine without external signals.

## SDRAM Memory Map

| Address Range | Size | Description |
|---------------|------|-------------|
| 0x0000-0x010F | 270 bytes | Capture buffer (max 266 samples + margin) |
| 0x1000-0x112F | 304 bytes | Display buffer (8 channels x 38 bytes) |

## Decimation Algorithm

The capture engine uses the decimate_pack module to convert raw samples into Apple II HIRES-compatible bytes.

### Pipeline
```
[Captured Samples] -> [Stretch] -> [Pixel FIFO] -> [7-bit Pack] -> [Display Buffer]
```

### Apple II HIRES Byte Format

- Bits 0-6: Pixel data (7 pixels per byte)
- Bit 7: Always 0 (palette select, unused)
- Bit 0 = leftmost pixel on screen
- First sample in time = leftmost on screen

### Screen Dimensions

- 8 channels displayed
- 38 columns (bytes) per channel
- 266 usable pixels per channel (38 x 7)

## Design-Specific Constraints

This design uses a design-specific LPF file (`logic_hamr_v1.lpf`) that configures GPIO5-12 as inputs with pull-down resistors. This overrides the base constraints file when building.

## Build and Test

```bash
# Synthesize
make DESIGN=logic_hamr_v1

# Run simulation
make sim DESIGN=logic_hamr_v1

# View waveform
make wave DESIGN=logic_hamr_v1

# Program FPGA (volatile)
make prog DESIGN=logic_hamr_v1

# Program flash (persistent)
make prog-flash DESIGN=logic_hamr_v1
```

## Example Usage (Apple II)

```asm
* Configure trigger on channel 0, rising edge
        LDA #$00
        STA $C0C6        ; TRIG_CH = 0
        STA $C0C7        ; TRIG_MODE = rising

* Select window preset 1 (88us)
        LDA #$01
        STA $C0C8        ; WINDOW = 1

* Enable debug pattern (optional)
        LDA #$01
        STA $C0CA        ; DEBUG_EN = 1

* ARM capture
        STA $C0C9        ; Write anything to ARM

* Wait for capture
WAIT    LDA $C0C4        ; Read STATUS
        AND #$08         ; Check CAPTURED bit
        BEQ WAIT

* Regenerate display
        LDA #$10
        STA $C0C3        ; CMD = regenerate

* Wait for ready
WAIT2   LDA $C0C4
        AND #$02         ; Check READY bit
        BEQ WAIT2

* Read channel 0, byte 0
        LDA #$00
        STA $C0C0        ; CHANNEL = 0
        STA $C0C1        ; ADDR = 0
        LDA #$02
        STA $C0C3        ; CMD = read

* Wait not busy
WAIT3   LDA $C0C4
        AND #$01
        BNE WAIT3

* Get data
        LDA $C0C2        ; Read DATA
```
