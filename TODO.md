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

## Software Examples

### Directory Structure
- [ ] Create `programs/` directory structure:
  ```
  programs/
  ├── asm/             # 6502 assembly source (.s)
  ├── basic/           # Applesoft BASIC programs (.bas)
  ├── bin/             # Compiled binaries (.bin, .rom)
  └── dsk/             # Disk images for emulators/real hardware
  ```

## Assembly Workflow

### 6502 Assembly Toolchain
- [ ] Integrate ca65/ld65 (cc65 suite):
  - Add `CC65_HOME` detection to Makefile
  - Support for `.s` → `.o` → `.bin` pipeline
- [ ] Add `make asm SRC=<file>.s` target
- [ ] Add `make asm-all` to build all assembly programs
- [ ] Support for generating ROM images (2K, 4K, 8K sizes)
- [ ] Memory map configuration for Apple II slots:
  - Linker configs for $Cn00-$CnFF (slot ROM)
  - Linker configs for $C800-$CFFF (expansion ROM)
  - Linker configs for device drivers

### Assembly Build Pipeline
- [ ] Create `programs/asm/Makefile` with:
  ```makefile
  # Assemble: .s -> .o
  ca65 -t apple2 -o $@ $<

  # Link: .o -> .bin
  ld65 -t apple2 -C config.cfg -o $@ $<

  # Optional: Create disk image
  ```
- [ ] Add linker configuration files:
  - `programs/asm/slot_rom.cfg` - For $Cn00-$CnFF code
  - `programs/asm/expansion_rom.cfg` - For $C800-$CFFF code
  - `programs/asm/driver.cfg` - For relocatable drivers

### Assembly Test Programs (Paired with Gateware)

#### signal_check
- [ ] `programs/asm/signal_check_test.s` - Fast status polling
  - Assembly version for precise timing control
  - Continuous display of GPIO status
  - Cycle-accurate timing measurements

#### serial_card (Future)
- [ ] `programs/asm/serial_driver.s` - Serial port driver
  - Pascal-compatible driver interface
  - Interrupt-driven RX with buffer
  - Polled TX
- [ ] `programs/asm/serial_term.s` - Terminal program
  - 80-column support
  - ANSI escape sequence handling

#### ram_card (Future)
- [ ] `programs/asm/ram_driver.s` - RAM card driver
  - ProDOS-compatible RAM disk driver
  - Bank switching routines
- [ ] `programs/asm/ram_test.s` - Comprehensive memory test
  - Walking 1s/0s patterns
  - Address line tests
  - Speed benchmarks

#### rom_emu (Future)
- [ ] `programs/asm/rom_loader.s` - ROM image loader
  - Load ROM images from disk to card
  - Verify loaded image

## BASIC Examples (Contributor-Friendly)

Good first issues for contributors - simple test programs in Applesoft BASIC:

### BASIC Test Programs (Paired with Gateware)

#### signal_check (Board Bring-up)
- [ ] `programs/basic/signal_check.bas` - Read GPIO status pins
  - PEEK soft switch to read GPIO1-4 status
  - Display SDRAM pass/fail, heartbeat, error status
  - Simple infinite loop to monitor card health

#### serial_card (Future)
- [ ] `programs/basic/serial_test.bas` - Serial loopback test
  - Send characters via POKE to TX register
  - Read characters via PEEK from RX register
  - IN#n / PR#n for slot redirection
  - Simple terminal program

#### ram_card (Future)
- [ ] `programs/basic/ram_test.bas` - Memory test from BASIC
  - Write patterns to extended RAM via soft switches
  - Read back and verify
  - Report errors with addresses

#### rom_emu (Future)
- [ ] `programs/basic/rom_select.bas` - ROM bank selector
  - Menu to select which ROM image to use
  - POKE to control register to switch banks

### Applesoft BASIC Reference
- [ ] Document PEEK/POKE addresses for each project
- [ ] Create `docs/basic_reference.md`:
  - Slot address calculations: `49280 + (SLOT * 16) + REGISTER`
  - Common patterns for reading/writing soft switches
  - Timing considerations (how fast can BASIC poll?)
  - Error handling patterns
  - Example: Reading a status byte
    ```basic
    10 SLOT = 7
    20 BASE = 49280 + SLOT * 16
    30 STATUS = PEEK(BASE + 0)
    40 PRINT "STATUS: "; STATUS
    ```

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

### Byte Hamr (Main Board)
- [ ] Verify all pin assignments match schematic
- [ ] Test level shifters with real Apple II
- [ ] Document jumper/configuration options
- [ ] Create programming guide for SPI flash

### Byte Ravn (Breakout Board)
- [ ] Create `hardware/byte_ravn/README.md` with:
  - Board purpose and features
  - Pin header mappings
  - Connection guide to Byte Hamr
- [ ] Document which GPIO pins are exposed
- [ ] Add schematic walkthrough
- [ ] Create example use cases (logic analyzer hookup, external peripherals)
- [ ] Finish any incomplete schematic/PCB work

## Documentation

### Completed
- [x] Architecture overview diagram

### Apple II Bus & Timing
- [ ] Apple II bus timing diagram with FPGA sampling points
- [ ] Apple IIe bus protocol guide (how to be a proper slot peripheral)
  - Device select ($C0n0-$C0nF) soft switch handling
  - I/O select ($Cn00-$CnFF) ROM space response
  - I/O strobe ($C800-$CFFF) expansion ROM banking
  - Proper PHI0/PHI1 timing for data bus driving
  - DMA considerations
  - Interrupt (IRQ/NMI) protocol

### Hardware Programming Guide
- [ ] Create `docs/apple2_programming_guide.md` - Comprehensive guide:

#### Part 1: Apple II Slot Architecture
- [ ] Memory map deep dive:
  - Main RAM: $0000-$BFFF
  - I/O and ROM: $C000-$FFFF
  - Slot-specific addresses and how they're decoded
- [ ] Soft switches explained:
  - What is a soft switch? (memory-mapped I/O)
  - Read vs Write soft switches
  - Toggle soft switches (any access triggers)
- [ ] The three slot regions:
  - Device Select ($C0n0-$C0nF): 16 I/O registers per slot
  - I/O Select ($Cn00-$CnFF): 256 bytes of ROM per slot
  - I/O Strobe ($C800-$CFFF): 2K shared expansion ROM

#### Part 2: Talking to Hardware from Assembly
- [ ] 6502 addressing modes for I/O:
  - Absolute: `LDA $C070` (read a soft switch)
  - Zero page indexed: Fast register access
  - Indexed absolute: `LDA $C080,X` (X = slot * 16)
- [ ] Slot-independent code:
  - Using the X register for slot number
  - Self-modifying code patterns (historical)
  - Relocation techniques
- [ ] Reading and writing examples:
  ```asm
  ; Read status from slot (slot number in X)
  LDA $C080,X    ; Read device select register 0
  STA STATUS     ; Store result

  ; Write to control register
  LDA #$FF       ; Value to write
  STA $C081,X    ; Write to register 1
  ```
- [ ] Timing-critical operations:
  - Cycle-counted loops
  - Synchronizing with PHI0/PHI1
  - NOP sleds for delays

#### Part 3: Talking to Hardware from BASIC
- [ ] PEEK and POKE fundamentals:
  - Syntax and addressing
  - Decimal vs Hex conversion table
  - Slot address calculation formula
- [ ] Reading status from a card:
  ```basic
  10 REM Read status from slot 7
  20 SLOT = 7
  30 BASE = 49280 + SLOT * 16: REM $C0n0 in decimal
  40 STATUS = PEEK(BASE + 0)
  50 PRINT "Card status: "; STATUS
  ```
- [ ] Writing commands to a card:
  ```basic
  10 REM Write to slot 7 control register
  20 SLOT = 7
  30 BASE = 49280 + SLOT * 16
  40 POKE BASE + 1, 255: REM Write $FF to register 1
  ```
- [ ] BASIC timing limitations:
  - Each BASIC statement takes ~1-2ms minimum
  - Can't bit-bang fast protocols
  - Good for configuration, not real-time

#### Part 4: Designing Your Card's Register Map
- [ ] Best practices for soft switch design:
  - Status registers (read-only)
  - Control registers (write-only or R/W)
  - Data registers (bidirectional)
  - How many registers do you need? (max 16)
- [ ] Example register maps:
  - Simple GPIO card (2-3 registers)
  - Serial card (4-6 registers)
  - RAM card (bank select + data window)
- [ ] Register map documentation template

#### Part 5: FPGA Implementation Patterns
- [ ] Detecting slot access in Verilog:
  ```verilog
  wire slot_select = !nDEVICE_SELECT;  // Active when $C0n0-$C0nF accessed
  wire slot_read = slot_select && R_nW;
  wire slot_write = slot_select && !R_nW;
  wire [3:0] reg_addr = {A3, A2, A1, A0};  // Which register (0-15)
  ```
- [ ] Driving the data bus:
  - Tristate control
  - When to drive (PHI1 high, read cycle)
  - Setup and hold times
- [ ] Responding to writes:
  - Latching data on PHI0 falling edge
  - Register decoding
- [ ] Clock domain considerations:
  - 25MHz FPGA clock vs 1MHz Apple II
  - Synchronizing asynchronous inputs
  - Metastability prevention

#### Part 6: Putting It All Together
- [ ] Tutorial: Building a "Hello Card"
  1. Define register map (1 read, 1 write register)
  2. Write Verilog for FPGA
  3. Write BASIC test program
  4. Write Assembly test program
  5. Debug with logic analyzer / simulation
- [ ] Tutorial: Building a Simple Serial Card
  1. Register map design (TX, RX, status, control)
  2. UART implementation in Verilog
  3. BASIC terminal program
  4. Assembly driver

### Technical References
- [ ] SDRAM timing analysis at 25MHz
- [ ] Pin mapping reference card
- [ ] Common Apple II addresses cheat sheet
- [ ] 6502 instruction timing reference
