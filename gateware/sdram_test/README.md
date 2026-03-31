# SDRAM Test

Register-driven SDRAM read/write test for the Byte Hamr. Allows Apple II software to directly read and write arbitrary SDRAM addresses through memory-mapped I/O registers, providing full 25-bit linear addressing across the 64MB SDRAM.

## What It Does

Exposes the onboard SDRAM to the Apple II CPU through a register interface at `$C0C0-$C0CF`. The CPU stages an address and data value in registers, issues a read or write command, polls for completion, then reads the result. Supports full 16-bit SDRAM data width.

The design handles SDRAM initialization automatically on power-up (precharge, refresh, mode register load) and runs periodic refresh in the background.

## Register Map

Base address: `$C0C0` (slot 4)

| Offset | Name | R/W | Description |
|--------|------|-----|-------------|
| `$00` | ADDR_LO | R/W | Address bits [7:0] |
| `$01` | ADDR_MID | R/W | Address bits [15:8] |
| `$02` | ADDR_HI | R/W | Address bits [24:16] (7 bits) |
| `$03` | DATA_LO | R/W | Write data / read result [7:0] |
| `$04` | DATA_HI | R/W | Write data / read result [15:8] |
| `$05` | CMD | W | `$01` = Write, `$02` = Read |
| `$06` | STATUS | R | `[0]` busy, `[1]` init_done, `[2]` last cmd type |
| `$07-$0F` | SCRATCH | R/W | 9 general-purpose loopback registers |

### Address Decomposition

The 25-bit linear address maps to SDRAM signals:
- Column[9:0] = `{ADDR_MID[1:0], ADDR_LO[7:0]}`
- Row[12:0] = `{ADDR_HI[4:0], ADDR_MID[7:2]}`
- Bank[1:0] = `ADDR_HI[6:5]`

## Usage from Apple II

```
; Write $BEEF to SDRAM address $000000
LDA #$00 : STA $C0C0    ; ADDR_LO
LDA #$00 : STA $C0C1    ; ADDR_MID
LDA #$00 : STA $C0C2    ; ADDR_HI
LDA #$EF : STA $C0C3    ; DATA_LO
LDA #$BE : STA $C0C4    ; DATA_HI
LDA #$01 : STA $C0C5    ; CMD = WRITE
:wait BIT $C0C6 : BNE :wait  ; Poll busy

; Read it back
LDA #$02 : STA $C0C5    ; CMD = READ
:wait2 BIT $C0C6 : BNE :wait2
LDA $C0C3               ; DATA_LO = $EF
LDA $C0C4               ; DATA_HI = $BE
```

## GPIO Debug Outputs

| GPIO | Signal |
|------|--------|
| GPIO1 | Heartbeat (250 kHz) |
| GPIO2 | SDRAM init done |
| GPIO3 | I/O select activity |
| GPIO4 | SDRAM busy |
| GPIO5 | R/nW debug |
| GPIO8-12 | Walking 1s pattern (0.5s) |

## Makefile Commands

All commands run from the project root (`project_byte_hamr/`).

```bash
# Build
make DESIGN=sdram_test         # Build complete bitstream
make DESIGN=sdram_test synth   # Synthesis only
make DESIGN=sdram_test pnr     # Place & route only
make DESIGN=sdram_test bit     # Generate bitstream

# Program
make DESIGN=sdram_test prog        # JTAG (volatile)
make DESIGN=sdram_test prog-flash  # SPI flash (persistent)

# Simulate
make DESIGN=sdram_test sim         # Run testbench
make DESIGN=sdram_test wave        # Simulation + GTKWave

# Reports
make DESIGN=sdram_test report      # View build summary
make clean                         # Remove build artifacts
```

## Files

| File | Description |
|------|-------------|
| `sdram_test_top.v` | Top module - register file, SDRAM state machine, bus interface |
| `sdram_test_tb.v` | Testbench - SDRAM model, bus transactions, multi-address verification |
