# Byte Hamr

An open-source FPGA expansion card for the Apple II family of computers.

## Overview

Byte Hamr brings modern FPGA capabilities to the Apple II, II+, IIe, and IIgs. Unlike purpose-built cards that serve a single function, Byte Hamr is a general-purpose platform for developing and running custom hardware designs on vintage Apple hardware.

The card sits between the simplicity of the A2FPGA (which targets HDMI output) and the complexity of the Carte Blanche (which is difficult to obtain and uses proprietary tooling). Byte Hamr uses the Lattice ECP5 FPGA with full support for the open-source Yosys/nextpnr toolchain.

## Features

- Lattice ECP5-85F FPGA (85K LUTs, BGA381 package)
- 64MB SDRAM (AS4C32M16SB-7TCNTR)
- 16MB SPI Flash for bitstream storage
- USB programming via FTDI FT231XQ
- JTAG header for development/debugging
- 13-pin GPIO header for FPGA and peripheral expansion
- Full Apple II bus interface with bidirectional level shifting
- Dual power input (USB or Apple II bus) with automatic switching
- On-board 25MHz oscillator
- Open-source hardware and gateware

## Specifications

| Parameter | Value |
|-----------|-------|
| FPGA | Lattice ECP5-85F (LFE5U-85F-8BG381I) |
| Logic Elements | 84K LUTs |
| Block RAM | 3744 Kbit |
| SDRAM | 64MB, 16-bit bus |
| Flash | 16MB SPI |
| USB | Full-speed via FT231XQ |
| GPIO | 13 pins, 3.3V |
| Power | +5V from USB or Apple II slot |
| Dimensions | Standard Apple II card form factor |

## Architecture

```
                                    +------------------+
                                    |     SDRAM        |
                                    |   (64MB x16)     |
                                    +--------+---------+
                                             |
+-------------+     +----------+     +-------+--------+     +----------+
| Apple II    |     | Level    |     |                |     |   SPI    |
| Expansion   +---->+ Shifters +---->+   ECP5-85F     +---->+  Flash   |
| Slot        |     | (5V/3.3V)|     |     FPGA       |     |  (16MB)  |
+-------------+     +----------+     +-------+--------+     +----------+
                                             |
                                     +-------+--------+
                                     |      USB       |
                                     |  (FT231XQ)     |
                                     +----------------+
```

## Power Distribution

Three on-board switching regulators generate the required voltages:

- +3.3V for FPGA I/O, SDRAM, Flash, level shifters
- +2.5V for FPGA auxiliary power
- +1.1V for FPGA core

Power can be sourced from USB (+5V) or the Apple II slot, with Schottky diode OR-ing for automatic switching.

## Bus Interface

Six 74LVC8T245 bidirectional level shifters translate between the Apple II's 5V bus and the FPGA's 3.3V I/O. The interface provides access to:

- 16-bit address bus
- 8-bit data bus (bidirectional)
- Control signals (R/W, PHI0, device select, etc.)

## Project Status

**Rev 1** - Sent to fabrication (December 2025)

See [VERSION.md](VERSION.md) for full version history.

## Repository Structure

```
byte-hamr/
├── hardware/
│   ├── kicad/          # KiCad project files
│   ├── gerbers/        # Manufacturing files
│   └── bom/            # Bill of materials
├── gateware/
│   └── (coming soon)   # FPGA designs
├── software/
│   └── (coming soon)   # Apple II software
└── docs/
    └── (coming soon)   # Documentation
```

## Getting Started

### Prerequisites

- Yosys (synthesis)
- nextpnr-ecp5 (place and route)
- openFPGALoader (programming)
- Apple IIe or compatible system

### Building Gateware

(Coming after Rev 1 hardware validation)

### Programming

Via USB:
```bash
openFPGALoader --board bytehamr bitstream.bit
```

Via JTAG:
```bash
openFPGALoader --cable yourjtag bitstream.bit
```

## Testing

Bringup sequence for new boards:

1. Visual inspection
2. Power rail verification (+5V, +3.3V, +2.5V, +1.1V)
3. USB enumeration (FTDI detection)
4. FPGA configuration (LED blink test)
5. Oscillator verification (25MHz on scope)
6. Apple II bus communication (PEEK/POKE test)
7. SDRAM test
8. Full system integration

## Design References

This project builds on the work of others:

- [ULX3S](https://github.com/emard/ulx3s) - ECP5 reference design, power architecture
- [A2FPGA](https://github.com/a2fpga/a2fpga_core) - Apple II FPGA integration concepts
- [Understanding the Apple IIe](https://archive.org/details/understanding_the_apple_iie) - Jim Sather's essential reference

## Why "Byte Hamr"?

"Hamr" is Old Norse for "shape" or "form" — the concept of shapeshifting in Norse mythology. Byte Hamr: shapeshifting bytes. An FPGA that can become any hardware you define. The shape changes based on what you program it to be.

## License

This project is released under the [MIT License](LICENSE.md).

## Author

Robert Rico

## Contributing

This is an early-stage project. Issues, suggestions, and pull requests welcome once Rev 1 hardware is validated.

---

*Byte Hamr Rev. 1 - 2025*