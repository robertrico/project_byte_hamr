# Pattern ROM

SDRAM-backed test pattern generator for the Byte Hamr. Stores 8 channels of alternating patterns in SDRAM on power-up, then serves them to Apple II software through a register interface.

## What It Does

On initialization, the design writes 304 bytes (8 channels x 38 bytes) of test patterns into SDRAM. The Apple II can then read any channel/byte combination through memory-mapped registers at `$C0C0-$C0CF`.

**Pattern values**: Alternating `$00` and `$7F` with phase shift per channel:
- Even channels (0,2,4,6): `$00, $7F, $00, $7F, ...`
- Odd channels (1,3,5,7): `$7F, $00, $7F, $00, ...`

This design was used to validate the SDRAM read path and Apple II bus interface before building the logic analyzer.

## Register Map

Base address: `$C0C0` (slot 4)

| Offset | Name | R/W | Description |
|--------|------|-----|-------------|
| `$00` | CHANNEL | R/W | Select channel (0-7) |
| `$01` | ADDR | R/W | Byte index within channel (0-37) |
| `$02` | DATA | R | Read result from SDRAM |
| `$03` | CMD | W | `$02` = Read byte from SDRAM |
| `$04` | STATUS | R | `[0]` busy, `[1]` ready (init + patterns loaded) |
| `$05-$0F` | SCRATCH | R/W | 11 loopback registers |

## Usage from Apple II

```
; Read channel 3, byte 5
LDA #$03 : STA $C0C0    ; CHANNEL = 3
LDA #$05 : STA $C0C1    ; ADDR = 5
LDA #$02 : STA $C0C3    ; CMD = READ
:wait BIT $C0C4 : BMI :wait  ; Wait for busy to clear
LDA $C0C2               ; DATA = pattern value
```

## GPIO Debug Outputs

| GPIO | Signal |
|------|--------|
| GPIO1 | Heartbeat (250 kHz) |
| GPIO2 | Ready (init done + patterns loaded) |
| GPIO3 | Pattern loading in progress |
| GPIO4 | SDRAM busy |
| GPIO5 | PHI0 passthrough |
| GPIO6 | Device select active |

## Makefile Commands

All commands run from the project root (`project_byte_hamr/`).

```bash
# Build
make DESIGN=pattern_rom         # Build complete bitstream
make DESIGN=pattern_rom synth   # Synthesis only
make DESIGN=pattern_rom pnr     # Place & route only
make DESIGN=pattern_rom bit     # Generate bitstream

# Program
make DESIGN=pattern_rom prog        # JTAG (volatile)
make DESIGN=pattern_rom prog-flash  # SPI flash (persistent)

# Simulate
make DESIGN=pattern_rom sim         # Run testbench
make DESIGN=pattern_rom wave        # Simulation + GTKWave

# Reports
make DESIGN=pattern_rom report      # View build summary
make clean                          # Remove build artifacts
```

## Files

| File | Description |
|------|-------------|
| `pattern_rom_top.v` | Top module - SDRAM controller, pattern generator, bus interface |
| `pattern_rom_tb.v` | Testbench - scratch loopback, full channel verification, random access |
