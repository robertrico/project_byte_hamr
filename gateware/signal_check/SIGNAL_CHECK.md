# Signal Check

Board bring-up test gateware for the Byte Hamr.

## Purpose

Validates that all major interfaces are functional after assembly:

1. **SDRAM** - Write/read pattern test to 8 addresses
2. **GPIO** - Walking 1s pattern on GPIO5-12 (0.5s per step)
3. **Apple II Bus** - Monitors PHI0 for bus activity

## Status Indicators

| GPIO | Signal | Meaning |
|------|--------|---------|
| GPIO1 | Heartbeat | Blinks at 1Hz when FPGA is running |
| GPIO2 | SDRAM Pass | Solid ON = SDRAM test passed |
| GPIO3 | Apple Activity | ON when PHI0 transitions detected |
| GPIO4 | Error | ON = SDRAM test failed |
| GPIO5-12 | Walking 1s | Rotating pattern for scope/LED verification |

## Build

From the project root:

```bash
make                    # Build signal_check (default)
make prog               # Program via JTAG (volatile)
make prog-flash         # Program SPI flash (persistent)
```

## Simulation

A testbench is included that simulates the SDRAM and Apple II bus:

```bash
make sim                # Run simulation
make wave               # Run simulation and open GTKWave
```

The testbench (`signal_check_tb.v`) includes:
- 25 MHz system clock
- Simple SDRAM model that stores/returns written data
- PHI0 toggling at ~1 MHz to simulate Apple II bus activity

VCD output is written to `build/signal_check_tb.vcd`.

## Expected Results

On power-up or programming:

1. GPIO1 starts blinking immediately (FPGA alive)
2. After ~200us, SDRAM initialization begins
3. After a few ms, GPIO2 lights up (SDRAM pass) OR GPIO4 lights up (fail)
4. If Apple II is connected and running, GPIO3 will be lit

## Files

| File | Description |
|------|-------------|
| `signal_check_top.v` | Top-level module with SDRAM controller, bus monitor, GPIO driver |
| `signal_check_tb.v` | Testbench with SDRAM model and Apple II bus simulation |

## Notes

- The signal "7M" from the schematic is mapped to `sig_7M` in Verilog (identifiers can't start with numbers)
- This test does NOT drive the Apple II data bus - it only observes
- DMA_OUT and INT_OUT are held low (inactive)
- The SDRAM test is basic - it doesn't test all memory or all patterns
