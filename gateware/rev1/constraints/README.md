# Constraints

Shared FPGA pin constraints for the Byte Hamr ECP5-85F board.

## What It Contains

- `byte_hamr.lpf` -- Base pin constraint file (Lattice LPF format)

## Pin Groups

- **Apple II bus**: Address, data, and control lines (PHI0, sig_7M, R/W, nDEVICE_SELECT, nI_O_SELECT, nI_O_STROBE)
- **SDRAM**: Address, data, and control (CKE, CLK, nCS, nRAS, nCAS, nWE, BA, DQM)
- **SPI flash**: CS, CLK, MOSI, MISO
- **GPIO header**: 12 pins for expansion/debugging
- **Clock inputs**: sig_7M, PHI0, CLK_25MHz

Each design may include its own `.lpf` file for design-specific overrides; the build system checks for a design-specific file first, then falls back to this base.
