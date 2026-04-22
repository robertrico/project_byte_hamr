# SDRAM Bank

Minimal SDRAM bank-switching controller for the Byte Hamr. Proves that the 4 independent SDRAM banks hold separate data and can be addressed individually from Apple II software.

## What It Does

Provides register-driven access to SDRAM organized into 4 independent banks, each with 256 addressable byte locations. The Apple II CPU selects a bank, sets an address, and issues read/write commands through memory-mapped registers at `$C0C0-$C0CF`.

This is a proof-of-concept for bank-independent storage - write to the same address in different banks and read back distinct values.

## Register Map

Base address: `$C0C0` (slot 4)

| Offset | Name | R/W | Description |
|--------|------|-----|-------------|
| `$00` | BANK | R/W | Active bank (0-3) -> SDRAM BA[1:0] |
| `$01` | ADDR | R/W | Byte address within bank (0-255) |
| `$02` | DATA | R/W | Write staging / read result (8-bit) |
| `$03` | CMD | W | `$01` = Write, `$02` = Read |
| `$04` | STATUS | R | `[0]` busy, `[1]` init_done, `[2]` last cmd type |
| `$05-$0F` | SCRATCH | R/W | 11 loopback registers |

### SDRAM Mapping

All accesses use Row 0. The bank select goes to BA[1:0] and the 8-bit address maps directly to the column.

## Usage from Apple II

```
; Write $42 to Bank 0, Address 5
LDA #$00 : STA $C0C0    ; BANK = 0
LDA #$05 : STA $C0C1    ; ADDR = 5
LDA #$42 : STA $C0C2    ; DATA = $42
LDA #$01 : STA $C0C3    ; CMD = WRITE
:wait BIT $C0C4 : BNE :wait

; Write $99 to Bank 2, Address 5
LDA #$02 : STA $C0C0    ; BANK = 2
LDA #$99 : STA $C0C2    ; DATA = $99
LDA #$01 : STA $C0C3    ; CMD = WRITE
:wait2 BIT $C0C4 : BNE :wait2

; Read back Bank 0, Address 5 -> should be $42
LDA #$00 : STA $C0C0
LDA #$02 : STA $C0C3    ; CMD = READ
```

## GPIO Debug Outputs

| GPIO | Signal |
|------|--------|
| GPIO1 | Heartbeat (250 kHz) |
| GPIO2 | SDRAM init done |
| GPIO3 | Bank select bit 0 |
| GPIO4 | Bank select bit 1 |
| GPIO5 | SDRAM busy |
| GPIO8-12 | Walking 1s pattern (0.5s) |

## Makefile Commands

All commands run from the project root (`project_byte_hamr/`).

```bash
# Build
make DESIGN=sdram_bank         # Build complete bitstream
make DESIGN=sdram_bank synth   # Synthesis only
make DESIGN=sdram_bank pnr     # Place & route only
make DESIGN=sdram_bank bit     # Generate bitstream

# Program
make DESIGN=sdram_bank prog        # JTAG (volatile)
make DESIGN=sdram_bank prog-flash  # SPI flash (persistent)

# Simulate
make DESIGN=sdram_bank sim         # Run testbench
make DESIGN=sdram_bank wave        # Simulation + GTKWave

# Reports
make DESIGN=sdram_bank report      # View build summary
make clean                         # Remove build artifacts
```

## Files

| File | Description |
|------|-------------|
| `sdram_bank_top.v` | Top module - 4-bank SDRAM controller, register file, bus interface |
| `sdram_bank_tb.v` | Testbench - bank independence, address independence, cross-bank tests |
