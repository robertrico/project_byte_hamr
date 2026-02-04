# Yellow Hamr - Requirements Document

**Project**: Yellow Hamr
**Version**: 0.2
**Date**: 2026-02-03
**Status**: Draft

---

## 1. Overview

### 1.1 Purpose

Yellow Hamr implements an Apple IIe-compatible **Liron disk controller** (IWM-based SmartPort interface) on the Byte Hamr FPGA card. The drive interface signals are exposed on the GPIO header for connection to external SmartPort-compatible devices.

### 1.2 Reference Design

This project adapts Steve Chamberlin's open-source **Yellowstone** project:
- Local source: `~/Development/fpga-disk-controller`
- Original target: Lattice MachXO2-1200HC
- Our target: Lattice ECP5-85F (Byte Hamr)

### 1.3 Scope

**In Scope:**
- IWM chip emulation in Verilog
- 4KB Liron boot ROM
- Address decoding for slot 4
- Drive interface signals on GPIO header

**Out of Scope:**
- External device firmware (FujiNet, etc.)
- SD card interface
- Network functionality

---

## 2. Architecture

### 2.1 Block Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           APPLE IIe SLOT 4                                  │
│                                                                             │
│   Address Bus (A0-A11) ───────┐                                            │
│   Data Bus (D0-D7) ───────────┼──── nDEVICE_SELECT ($C0C0-$C0CF)          │
│   Control (R/W, 7M, Q3) ──────┤──── nI_O_SELECT ($C400-$C4FF)             │
│   nRES ───────────────────────┘──── nI_O_STROBE ($C800-$CFFF)             │
│                                                                             │
└───────────────────────────────┬─────────────────────────────────────────────┘
                                │ (directly via level shifters on PCB)
┌───────────────────────────────┴─────────────────────────────────────────────┐
│                          BYTE HAMR FPGA                                     │
│                                                                             │
│  ┌─────────────────┐   ┌─────────────────┐   ┌─────────────────────────┐   │
│  │  addr_decoder   │   │      iwm        │   │      boot_rom           │   │
│  │                 │   │                 │   │      (4KB EBR)          │   │
│  │  ROM enable     │   │  Serial I/O     │   │                         │   │
│  │  IWM select     │   │  Phase control  │   │  Liron firmware         │   │
│  │                 │   │  Handshake      │   │                         │   │
│  └────────┬────────┘   └────────┬────────┘   └────────────┬────────────┘   │
│           │                     │                         │                 │
│           └─────────── Data Bus Mux ──────────────────────┘                 │
│                                                                             │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │                      Drive Interface                                  │  │
│  │   phase[3:0]  wrdata  rddata  sense  _enbl1  _enbl2  _wrreq  _en35   │  │
│  └──────────────────────────────────┬───────────────────────────────────┘  │
│                                     │                                       │
└─────────────────────────────────────┼───────────────────────────────────────┘
                                      │
                               GPIO Header (directly)
                                      │
                                      ▼
                           External SmartPort Device
```

### 2.2 Module Hierarchy

```
yellow_hamr_top
├── addr_decoder      (37 lines)   - ROM/IWM address decoding
├── iwm               (295 lines)  - IWM chip emulation
└── boot_rom          (~50 lines)  - 4KB EBR ROM wrapper
```

---

## 3. Apple II Bus Interface

### 3.1 Accent Signals Used

| Signal | FPGA Pin | Direction | Purpose |
|--------|----------|-----------|---------|
| A0-A11 | Various | Input | Address bus (12 bits sufficient) |
| D0-D7 | Various | Bidir | Data bus |
| sig_7M | A9 | Input | 7 MHz clock (IWM FCLK) |
| Q3 | B10 | Input | 2 MHz timing reference |
| R_nW | D10 | Input | Read/Write |
| nDEVICE_SELECT | D16 | Input | $C0C0-$C0CF (IWM registers) |
| nI_O_SELECT | A19 | Input | $C400-$C4FF (boot ROM) |
| nI_O_STROBE | C11 | Input | $C800-$CFFF (expansion ROM) |
| nRES | A8 | Input | System reset |

### 3.2 Memory Map (Slot 4)

| Address Range | Select Signal | Content |
|---------------|---------------|---------|
| $C0C0-$C0CF | nDEVICE_SELECT | IWM registers |
| $C400-$C4FF | nI_O_SELECT | Boot ROM ($000-$0FF) |
| $C800-$CFFF | nI_O_STROBE | Boot ROM ($100-$7FF) when active |

### 3.3 Bus Timing

IWM uses **7M (sig_7M)** as its primary clock, not CLK_25MHz.

```
7M period: ~140ns (7.16 MHz)
Bit cell:  28 × 7M = 4 µs
```

---

## 4. IWM Implementation

### 4.1 State Register

The IWM has 8 state bits, individually addressable via A3-A1. The value on A0 is latched into the selected bit on the falling edge of nDEVICE_SELECT.

| A3-A1 | Bit | Reset | Function |
|-------|-----|-------|----------|
| 0 | phase[0] | 0 | Stepper phase 0 |
| 1 | phase[1] | 0 | Stepper phase 1 |
| 2 | phase[2] | 0 | Stepper phase 2 |
| 3 | phase[3] | 0 | Stepper phase 3 |
| 4 | motorOn | 0 | Drive motor enable |
| 5 | driveSelect | 0 | Drive select (0=D1, 1=D2) |
| 6 | Q6 | 0 | Mode bit |
| 7 | Q7 | 0 | Mode bit |

### 4.2 Read Registers

Read operations occur when A0=0 and nDEVICE_SELECT is low:

| Q7 | Q6 | Register | Bit 7 | Bit 6 | Bit 5 | Bits 4-0 |
|----|----| ---------|-------|-------|-------|----------|
| 0 | 0 | Data | MSB (valid flag) | D6 | D5 | D4-D0 |
| 0 | 1 | Status | sense | 0 | motorOn | mode (00111) |
| 1 | 0 | Handshake | bufEmpty | _underrun | 0 | 00000 |
| 1 | 1 | — | (write mode, no read) | | | |

### 4.3 Write Operations

When Q7=1, Q6=1, motorOn=1, and A0=1, data is written to the write buffer.

### 4.4 Serial I/O

**Read (Q7=0, Q6=0):**
- Falling edge on `rddata` within bit cell = 1-bit
- No transition for 1.5 bit cells = 0-bit
- Bits shift LSB→MSB into shifter
- When MSB=1, byte is complete → latch to buffer

**Write (Q7=1):**
- Every 28 FCLK cycles, shift next bit
- Toggle `wrdata` if bit is 1
- Underrun if buffer empty when needed

### 4.5 Timing Constants

| Constant | Value | Time |
|----------|-------|------|
| Bit cell | 28 FCLK | 4 µs |
| Half bit cell | 14 FCLK | 2 µs |
| 1.5 bit cells | 42 FCLK | 6 µs |
| Buffer clear delay | 14 FCLK | 2 µs |

### 4.6 Mode Register

Hardcoded for Apple II compatibility: **SCMHL = 00111**

| Bit | Value | Meaning |
|-----|-------|---------|
| S | 0 | 7 MHz clock |
| C | 0 | 4 µs/bit (slow mode) |
| M | 0 | 1 sec motor-off delay |
| H | 1 | Synchronous handshake |
| L | 1 | Latch mode |

---

## 5. Drive Interface (GPIO Header)

### 5.1 Signal Mapping

| GPIO | Signal | Direction | Description |
|------|--------|-----------|-------------|
| GPIO1 | phase[0] | Output | Stepper phase 0 |
| GPIO2 | phase[1] | Output | Stepper phase 1 |
| GPIO3 | phase[2] | Output | Stepper phase 2 |
| GPIO4 | phase[3] | Output | Stepper phase 3 |
| GPIO5 | wrdata | Output | Serial write data |
| GPIO6 | rddata | Input | Serial read data |
| GPIO7 | sense | Input | Write-protect / ACK |
| GPIO8 | _enbl1 | Output | Drive 1 enable (active low) |
| GPIO9 | _enbl2 | Output | Drive 2 enable (active low) |
| GPIO10 | _wrreq | Output | Write request (active low) |
| GPIO11 | _en35 | Output | 3.5" drive enable (active low) |
| GPIO12 | — | Reserved | (spare) |

### 5.2 Signal Active Levels

| Signal | Active State | Idle State |
|--------|--------------|------------|
| phase[3:0] | High | Low |
| wrdata | Transitions = 1-bits | Stable |
| rddata | Falling edge = 1-bit | High |
| sense | Active high (directly directly directly directly polled) | — |
| _enbl1 | Low | High |
| _enbl2 | Low | High |
| _wrreq | Low | High |
| _en35 | Low | High |

### 5.3 Enable Logic

```verilog
assign _enbl1 = ~(motorOn & ~driveSelect);  // Drive 1 when motor on, drive 0 selected
assign _enbl2 = ~(motorOn & driveSelect);   // Drive 2 when motor on, drive 1 selected
assign _wrreq = ~(q7 & _underrun & (motorOn));
```

---

## 6. Boot ROM

### 6.1 Source

File: `~/Development/fpga-disk-controller/rom-full-4k.mem`

This is the original Liron ROM dump in hex format, ready for synthesis.

### 6.2 Implementation

Use ECP5 EBR (Embedded Block RAM) initialized from `.mem` file:

```verilog
reg [7:0] rom [0:4095];
initial $readmemh("liron_rom.mem", rom);

always @(posedge clk) begin
    rom_data <= rom[addr[11:0]];
end
```

### 6.3 Address Mapping

| ROM Address | Access Method |
|-------------|---------------|
| $000-$0FF | Read $C400-$C4FF (nI_O_SELECT) |
| $100-$7FF | Read $C800-$CFFF (nI_O_STROBE) when expansion active |

Expansion ROM active flag:
- Set when nI_O_SELECT goes low
- Cleared when $CFFF is accessed

---

## 7. Differences from Yellowstone

| Aspect | Yellowstone | Yellow Hamr |
|--------|-------------|-------------|
| FPGA | MachXO2-1200HC | ECP5-85F |
| System clock | — | 25 MHz (unused by IWM) |
| IWM clock | 7M directly | 7M directly (sig_7M) |
| ROM | SCUBA-generated EBR | Simple EBR with $readmemh |
| Level shifters | On-card 74LVC245 | On Byte Hamr PCB |
| SPI Flash | Present (unused) | Not used |
| Debug LEDs | 8 pins | Use Logic Hamr instead |

---

## 8. Files to Create

```
gateware/yellow_hamr/
├── REQUIREMENTS.md          # This document
├── yellow_hamr_top.v        # Top-level module
├── iwm.v                    # IWM emulation (adapt from Yellowstone)
├── addr_decoder.v           # Address decoding (adapt from Yellowstone)
├── boot_rom.v               # ROM wrapper
├── liron_rom.mem            # ROM contents (copy from fpga-disk-controller)
├── yellow_hamr_tb.v         # Testbench
└── yellow_hamr.lpf          # Pin constraints (optional, can use base)
```

---

## 9. Test Plan

### 9.1 Simulation Tests

| Test | Description | Pass Criteria |
|------|-------------|---------------|
| T1 | IWM register write | Write $C0C1 → phase[0]=1 |
| T2 | IWM register read | Set Q6=1, read $C0C0 → status byte |
| T3 | ROM read | Read $C400 → first ROM byte |
| T4 | Expansion ROM | Access $C400, then $C800 → ROM[$100] |
| T5 | Expansion clear | Access $CFFF → clears expansion flag |
| T6 | Serial write | Set Q7=1, write data → wrdata toggles |
| T7 | Serial read | Pulse rddata → data appears in buffer |

### 9.2 Hardware Tests

**Test Equipment Options:**
- Multimeter (for static GPIO states)
- Oscilloscope or logic analyzer (external, not Logic Hamr - that's a different bitstream!)
- LED + resistor on GPIO pins for visual feedback

| Test | Method | Pass Criteria |
|------|--------|---------------|
| H1 | `C400L` in monitor | Disassembles Liron ROM code |
| H2 | `C0C9` in monitor | GPIO8 (_enbl1) goes low (measure with meter/scope) |
| H3 | `C0C8` in monitor | GPIO8 returns high |
| H4 | `C0CB` then `C0C9` | GPIO9 (_enbl2) goes low instead |
| H5 | PR#4 boot attempt | Shows "UNABLE TO BOOT" or similar (no device attached) |

### 9.3 Integration Tests

| Test | Description |
|------|-------------|
| I1 | Connect external SmartPort device, attempt boot |
| I2 | ProDOS loads and shows device |
| I3 | Read/write disk operations work |

---

## 10. Development Phases

### Phase 1: Port & Synthesize
- [ ] Copy/adapt `iwm.v` from Yellowstone
- [ ] Copy/adapt `addr_decoder.v`
- [ ] Create `boot_rom.v` with ECP5 EBR
- [ ] Create `yellow_hamr_top.v` with Byte Hamr pins
- [ ] Copy ROM file, convert if needed
- [ ] Synthesize, fix any errors
- [ ] Verify timing closure

### Phase 2: Simulation
- [ ] Create testbench
- [ ] Test register read/write
- [ ] Test ROM access
- [ ] Test serial I/O timing

### Phase 3: Hardware Bring-up
- [ ] Load bitstream (`make prog DESIGN=yellow_hamr`)
- [ ] Verify ROM reads from Apple II (`C400L` in monitor)
- [ ] Verify IWM register writes affect GPIO (multimeter/scope on GPIO8/9)
- [ ] Test PR#4 boot attempt (should fail gracefully without device)

### Phase 4: Integration
- [ ] Wire GPIO to external device
- [ ] Test SmartPort communication
- [ ] Debug until boot works

---

## Appendix A: IWM Quick Reference

### Register Writes (accent accent via address)

```
$C0C0  phase[0]=0     $C0C1  phase[0]=1
$C0C2  phase[1]=0     $C0C3  phase[1]=1
$C0C4  phase[2]=0     $C0C5  phase[2]=1
$C0C6  phase[3]=0     $C0C7  phase[3]=1
$C0C8  motorOn=0      $C0C9  motorOn=1
$C0CA  driveSelect=0  $C0CB  driveSelect=1
$C0CC  Q6=0           $C0CD  Q6=1
$C0CE  Q7=0           $C0CF  Q7=1
```

### Register Reads (Q7:Q6 selects register)

```
Q7=0 Q6=0: Data buffer     {valid, d6, d5, d4, d3, d2, d1, d0}
Q7=0 Q6=1: Status          {sense, 0, motorOn, 0, 0, 1, 1, 1}
Q7=1 Q6=0: Handshake       {empty, ~underrun, 0, 0, 0, 0, 0, 0}
```

---

## Appendix B: Yellowstone Source Reference

Key files in `~/Development/fpga-disk-controller/lattice/`:

| File | Lines | Notes |
|------|-------|-------|
| `iwm.v` | 295 | Core IWM - use directly directly with minor edits |
| `addrDecoder.v` | 37 | Address decode - use directly |
| `top.v` | 112 | Top module - rewrite for Byte Hamr |
| `codeROM.v` | 295 | MachXO2-specific - replace with simple EBR |

ROM file: `rom-full-4k.mem` (12,352 bytes = 4096 × 3 chars per byte)
