# Byte Hamr - TODO

## Gateware Infrastructure

### Project Scaffolding
- [ ] Add `make new_project PROJECT=<name>` target that creates:
  - `gateware/<name>/` directory
  - `gateware/<name>/<name>_top.v` - minimal top module with all I/O ports
  - `gateware/<name>/<name>_tb.v` - testbench template with clock, SDRAM model, Apple II bus
  - `gateware/<name>/README.md` - project description template
  - Copy/symlink to shared LPF (or project-specific if needed)
- [ ] Create `gateware/common/` for shared modules:
  - SDRAM controller
  - Apple II bus interface
  - Clock/reset generation
- [ ] Add `make list` to show all available designs

### Build System
- [ ] Add timing report extraction to `make pnr`
- [ ] Add resource utilization summary
- [ ] Consider adding Verilator support for faster simulation

## Assembly Workflow (Future)

### 6502 Assembly Toolchain
- [ ] Integrate ca65/ld65 (cc65 suite) or similar assembler
- [ ] Add `make asm SRC=<file>.s` target
- [ ] Support for generating ROM images
- [ ] Memory map configuration for Apple II slots

### Test Programs
- [ ] Create `programs/` directory structure
- [ ] Basic "hello slot" program - respond to device select
- [ ] Memory test program - exercise SDRAM via card
- [ ] ROM emulation test

## Signal Check Improvements

- [ ] Add more comprehensive SDRAM test (all rows, walking 1s pattern)
- [ ] Add Apple II bus response test (respond to reads at slot address)
- [ ] LED blink patterns for different error codes
- [ ] Serial output via FTDI for debug messages

## Future Gateware Projects

- [ ] `serial_card` - Simple serial port for Apple II
- [ ] `ram_card` - SDRAM-backed RAM expansion
- [ ] `rom_emu` - ROM emulator (replace physical ROMs)
- [ ] `disk_emu` - Disk II emulator (SD card based)

## Hardware

- [ ] Verify all pin assignments match schematic
- [ ] Test level shifters with real Apple II
- [ ] Document jumper/configuration options
- [ ] Create programming guide for SPI flash

## Documentation

- [ ] Architecture overview diagram
- [ ] Apple II bus timing diagram with FPGA sampling points
- [ ] SDRAM timing analysis at 25MHz
- [ ] Pin mapping reference card
