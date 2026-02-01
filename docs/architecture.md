# Byte Hamr Architecture

## System Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              APPLE II SLOT                                  │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │  Address Bus (A0-A15)  Data Bus (D0-D7)  Control (PHI0/1, R/W, etc) │    │
│  └──────────────────────────────┬──────────────────────────────────────┘    │
└─────────────────────────────────┼───────────────────────────────────────────┘
                                  │
                                  │ 5V ↔ 3.3V
                    ┌─────────────┴─────────────┐
                    │       LEVEL SHIFTERS      │
                    │        (SN74LVC245)       │
                    └─────────────┬─────────────┘
                                  │ 3.3V
┌─────────────────────────────────┼────────────────────────────────────────────┐
│                          BYTE HAMR PCB                                       │
│  ┌──────────────────────────────┴───────────────────────────────────────┐    │
│  │                                                                      │    │
│  │                      ┌─────────────────────┐                         │    │
│  │   Apple II Bus ──────│                     │────── SDRAM (64MB)      │    │
│  │   A0-A15, D0-D7      │                     │       AS4C32M16SB       │    │
│  │   PHI0, PHI1         │    LATTICE ECP5     │                         │    │
│  │   R/W, RDY           │    LFE5U-85F-8      │────── SPI Flash         │    │
│  │   IRQ, NMI, RES      │                     │       (Bitstream)       │    │
│  │   DMA, INH           │    381-ball BGA     │                         │    │
│  │   I/O SELECT         │    85K LUTs         │────── GPIO Header (J1)  │    │
│  │   I/O STROBE         │                     │       12 pins direct    │    │
│  │   DEVICE SELECT      │                     │                         │    │
│  │   7M, Q3, SYNC       └─────────────────────┘────── JTAG (FT2232H)    │    │
│  │                              │                                       │    │
│  │                         CLK_25MHz                                    │    │
│  │                        (Oscillator)                                  │    │
│  └──────────────────────────────────────────────────────────────────────┘    │
└──────────────────────────────────────────────────────────────────────────────┘
```

## Signal Categories

### Apple II Bus Interface (active low signals marked with n)

| Category | Signals | Direction | Description |
|----------|---------|-----------|-------------|
| Address | A0-A15 | Input | 16-bit address from 6502 |
| Data | D0-D7 | Bidirectional | 8-bit data bus |
| Timing | PHI0, PHI1 | Input | System clocks (~1MHz) |
| Timing | 7M, Q3 | Input | High-speed timing (7.16MHz, 1.79MHz) |
| Timing | SYNC | Input | Opcode fetch indicator |
| Control | R/nW | Input | Read/Write direction |
| Control | RDY | Input | Ready (active high) |
| Interrupt | nIRQ, nNMI | Output (active low) | Interrupt requests |
| Reset | nRES | Input | System reset |
| DMA | nDMA, DMA_IN, DMA_OUT | Mixed | DMA control |
| Select | nDEVICE_SELECT | Input | Slot device select ($C0n0-$C0nF) |
| Select | nI/O_SELECT | Input | Slot I/O select ($Cn00-$CnFF) |
| Select | nI/O_STROBE | Input | Expansion ROM ($C800-$CFFF) |
| Control | nINH | Output | Inhibit main ROM |
| Misc | INT_IN, INT_OUT | Mixed | Interrupt daisy chain |

### SDRAM Interface

| Signal | Width | Description |
|--------|-------|-------------|
| SDRAM_CLK | 1 | Clock (directly from FPGA, 25MHz) |
| SDRAM_CKE | 1 | Clock enable |
| SDRAM_nCS | 1 | Chip select |
| SDRAM_nRAS | 1 | Row address strobe |
| SDRAM_nCAS | 1 | Column address strobe |
| SDRAM_nWE | 1 | Write enable |
| SDRAM_BA | 2 | Bank address |
| SDRAM_A | 13 | Address (row/column multiplexed) |
| SDRAM_D | 16 | Data (bidirectional) |
| SDRAM_DQM | 2 | Data mask (byte lanes) |

**SDRAM: AS4C32M16SB-7TCNTR**
- 512Mbit (32M x 16bit = 64MB)
- 143MHz max, running at 25MHz (plenty of margin)
- CAS Latency 2 at this speed

### GPIO Header (J1)

12 pins directly connected to FPGA (no level shifters):
- GPIO1-GPIO12: General purpose I/O
- 3.3V logic levels
- Direct FPGA connection for maximum flexibility

## Clock Domains

```
                    ┌──────────────┐
   25MHz OSC ───────│  CLK_25MHz   │──────┬────── FPGA Logic
                    └──────────────┘      │
                                          └────── SDRAM_CLK

   ~1MHz (Apple) ───│    PHI0      │──────────── Sampled by FPGA
                    │    PHI1      │             (oversampled at 25MHz)

   7.16MHz ─────────│    7M        │──────────── Available for timing
   1.79MHz ─────────│    Q3        │
```

**Key Insight**: PHI0/PHI1 are NOT used as FPGA clocks. They are sampled as inputs using the 25MHz system clock. This avoids clock domain crossing issues and provides 25 samples per Apple II bus cycle.

## Memory Map (Apple II Perspective)

| Address Range | Signal Asserted | Typical Use |
|---------------|-----------------|-------------|
| $C0n0-$C0nF | nDEVICE_SELECT | Slot n I/O registers (soft switches) |
| $Cn00-$CnFF | nI/O_SELECT | Slot n ROM space (directly addressable) |
| $C800-$CFFF | nI/O_STROBE | Expansion ROM (shared, banked by slot access) |

Where n = slot number (1-7, typically 4-7 for peripheral cards)

## Build Flow

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   Verilog   │────▶│   Yosys     │────▶│  nextpnr    │────▶│  ecppack    │
│   Sources   │     │  (synth)    │     │   (PnR)     │     │ (bitstream) │
└─────────────┘     └──────┬──────┘     └──────┬──────┘     └──────┬──────┘
                           │                   │                   │
                           ▼                   ▼                   ▼
                      .json             .config               .bit/.svf
                                           │
┌─────────────┐                            │
│    LPF      │────────────────────────────┘
│ Constraints │    (pin locations, I/O settings)
└─────────────┘
```

## File Structure

```
project_byte_hamr/
├── gateware/                    # FPGA designs
│   ├── signal_check/            # Board bring-up test
│   │   ├── signal_check_top.v   # Top module
│   │   └── signal_check_tb.v    # Testbench
│   └── <future_projects>/
├── hardware/
│   └── byte_hamr/
│       ├── kicad/               # Schematic & PCB
│       └── constraints/
│           └── byte_hamr.lpf    # Pin constraints (generated)
├── scripts/
│   ├── extract_fpga_pinout.py   # Parse KiCad netlist
│   ├── augment_fpga_pinout.py   # Add ECP5 pin info
│   └── generate_lpf.py          # Generate LPF from JSON
├── docs/
│   ├── fpga_pinout.json         # Master pinout (generated)
│   └── architecture.md          # This file
├── build/                       # Build outputs (gitignored)
└── Makefile                     # Build system
```

## Design Conventions

### Signal Naming
- Active-low signals: prefix with `n` (e.g., `nRES`, `nIRQ`)
- Apple II signals: match slot documentation where possible
- SDRAM signals: prefix with `SDRAM_`
- GPIO header: `GPIO1` through `GPIO12`
- Signals starting with numbers: prefix with `sig_` (e.g., `sig_7M`)

### I/O Standards
- All I/O: LVCMOS33 (3.3V)
- SDRAM: FAST slewrate for signal integrity
- Apple II bus: SLOW slewrate (through level shifters)
- Default drive: 8mA
