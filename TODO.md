# Byte Hamr - TODO

## Current Focus: Logic Analyzer (logic_hamr_v1)

### Gateware
- [x] Decimation algorithm (decimate_pack module)
- [x] SDRAM pattern storage
- [x] Apple II bus register interface
- [ ] Sample rate configuration via soft switch
- [ ] Trigger detection (edge, pattern)
- [ ] Pre-trigger buffer

### Software (6502 Assembly)
- [x] HIRES waveform rendering (LOGIC.S → LOGIC5.S iterations)
- [x] Channel display routines
- [ ] Cursor/marker functionality
- [ ] Time measurement display
- [ ] Scroll through capture buffer
- [ ] Save/load captures to disk

## Signal Check (Board Bring-up)

- [x] SDRAM basic test (8 addresses)
- [x] Apple II bus response (Device Select registers)
- [x] GPIO heartbeat and status LEDs
- [ ] More comprehensive SDRAM test (all rows, walking 1s)
- [ ] LED blink patterns for different error codes

## BASIC Support

Goal: Simple BASIC programs for card interaction and testing.

- [x] Document PEEK/POKE addresses (docs/apple2_cheatsheet.md)
- [ ] Example program: Read card status
- [ ] Example program: Configure sample rate

## Hardware

- [ ] Verify pin assignments match schematic
- [ ] Test level shifters with real Apple II
- [ ] Document jumper/configuration options
- [ ] Create SPI flash programming guide

## Documentation

### Completed
- [x] Architecture overview (docs/architecture.md)
- [x] Pin mapping reference (docs/pinout.md)
- [x] Apple II address cheat sheet (docs/apple2_cheatsheet.md)
- [x] Register maps for gateware (in cheat sheet)

### To Create
- [ ] BASIC/Assembly tutorial for beginners

## Build System

- [ ] Add timing report extraction to `make pnr`
- [ ] Add resource utilization summary
- [ ] `make new_project PROJECT=<name>` scaffolding
