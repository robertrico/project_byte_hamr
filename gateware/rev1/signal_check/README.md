# Signal Check

Board bring-up test gateware for the Byte Hamr Apple II FPGA card. Run this first after assembling a new board to validate that the FPGA, SDRAM, GPIO, and Apple II bus interface are all functional.

## What It Does

On power-up, signal_check runs three automatic tests:

1. **SDRAM Test** - Writes unique patterns to 8 addresses, reads them back, and compares. GPIO2 lights on pass, GPIO4 on fail.
2. **GPIO Walking 1s** - Shifts a single HIGH bit across GPIO8-12 every 0.5s for oscilloscope or LED verification.
3. **Apple II Bus Monitor** - Samples PHI0 transitions to detect whether the Apple II is running and connected. Falls back to a fake 1 MHz PHI0 if no external clock is detected within 20ms.

The design also exposes a 16-byte register file at `$C0C0-$C0CF` (slot 4) for basic read/write loopback testing from the Apple II side.

## Status Indicators

| GPIO | Signal | Meaning |
|------|--------|---------|
| GPIO1 | Heartbeat | 250 kHz square wave - FPGA is alive |
| GPIO2 | SDRAM Pass | Solid ON = all 8 addresses passed |
| GPIO3 | Bus Activity | ON when I/O select or strobe detected |
| GPIO4 | SDRAM Fail | ON = read-back mismatch |
| GPIO5 | R/nW | HIGH during read cycles |
| GPIO8-12 | Walking 1s | Rotating pattern (0.5s per step) |

## Expected Results

1. GPIO1 starts blinking immediately (FPGA alive)
2. After ~200us, SDRAM initialization begins
3. After a few ms, GPIO2 lights up (pass) OR GPIO4 lights up (fail)
4. If Apple II is connected and running, GPIO3 will be lit
5. GPIO8-12 rotate continuously

## Makefile Commands

All commands run from the project root (`project_byte_hamr/`). Signal check is the default design.

```bash
# Build
make                          # Build bitstream (signal_check is default)
make synth                    # Synthesis only (Yosys -> JSON)
make pnr                      # Place & route only (nextpnr -> config)
make bit                       # Generate bitstream (ecppack -> .bit)

# Program
make prog                     # Program via JTAG (volatile, lost on power cycle)
make prog-flash               # Program SPI flash (persistent)
make prog-detect              # Detect FPGA over JTAG

# Simulate
make sim                      # Run testbench simulation
make wave                     # Run simulation + open GTKWave

# Reports
make report                   # View build summary (timing, utilization)
make clean                    # Remove build artifacts
```

## Files

| File | Description |
|------|-------------|
| `signal_check_top.v` | Top-level module - SDRAM controller, bus monitor, GPIO driver |
| `signal_check_tb.v` | Testbench with SDRAM model and Apple II bus simulation |

## Notes

- This design does NOT drive the Apple II data bus (except during register reads) - it primarily observes
- DMA_OUT and INT_OUT are held low (inactive)
- The SDRAM test is a basic sanity check - it doesn't test all memory or all bit patterns
- Uses the base `byte_hamr.lpf` constraints (no design-specific LPF)
