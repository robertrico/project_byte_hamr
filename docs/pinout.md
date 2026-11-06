# Apple IIe Peripheral Slot Pinout

This document describes the 50-pin peripheral slot connector used in the Apple IIe. Each expansion slot provides address decoding, data bus access, power, and control signals for peripheral cards.

---

## Pin Reference

| Pin | Signal | Direction | Description |
|-----|--------|-----------|-------------|
| 1 | I/O SELECT´ | To FPGA | Active low when the MMU references page `$Cn`, where *n* is the slot number. Active during Ø0. |
| 2–17 | A0–A15 | To FPGA | Three-state buffered address bus. Valid during Ø1 through Ø0. |
| 18 | R/W´ | To FPGA | Three-state buffered Read/Write signal. Valid with address bus; high = read, low = write. |
| 19 | SYNC | To FPGA | **Slot 7 only.** Connected to the video timing generator's composite sync signal. |
| 20 | I/O STROBE´ | To FPGA | Active low during Ø0 when address is `$C800`–`$CFFF`. |
| 21 | RDY | To Apple IIe | 6502 RDY input. Pulling low during Ø1 halts the processor with the current address held on the bus. |
| 22 | DMA´ | To Apple IIe | Pulling low disables the 6502 address bus and halts the processor. Held high by 3kΩ resistor to +5V. |
| 23 | INT OUT | To FPGA | Daisy-chain input from higher-priority slot. Active = no higher slot is using interrupts. |
| 24 | DMA OUT | To FPGA | Daisy-chain input from higher-priority slot. Active = no higher slot is using DMA. |
| 25 | +5V | — | +5 volt power supply. 500mA available across *all* peripheral cards. |
| 26 | GND | — | System ground. |
| 27 | DMA IN | To Apple IIe | Daisy-chain output to lower-priority slot. Directly connect to DMA OUT (pin 24), or disable when asserting DMA´. |
| 28 | INT IN | To Apple IIe | Daisy-chain output to lower-priority slot. Directly connect to INT OUT (pin 23), or disable when asserting IRQ´/NMI´. |
| 29 | NMI´ | To Apple IIe | Non-Maskable Interrupt. Pulling low initiates an interrupt cycle; jumps to vector at `$03FB`. This line has a 3.3kΩ pullup to +5v.|
| 30 | IRQ´ | To Apple IIe | Interrupt Request. Pulling low initiates an interrupt cycle if the 6502's I flag is clear. Vector at `$03FE`–`$03FF`. This line has a 3.3kΩ pullup to +5v. |
| 31 | RES´ | To Apple IIe | Reset. Pulling low initiates a processor reset cycle. |
| 32 | INH´ | To Apple IIe | Inhibit. Pulling low disables all onboard ROMs. This line has a 2kΩ pullup to +5v.|
| 33 | −12V | — | −12 volt power supply. 200mA maximum across all peripheral cards. |
| 34 | −5V | — | −5 volt power supply. 200mA maximum across all peripheral cards. |
| 35 | COLOR REF / 3.58M | To FPGA | **Slot 7 only.** 3.58 MHz color reference signal from the video generator. |
| 36 | 7M | To FPGA | 7 MHz clock. |
| 37 | Q3 | To FPGA | 2 MHz asymmetrical clock. |
| 38 | Ø1 | To FPGA | Microprocessor phase-one clock. |
| 39 | µPSYNC | To FPGA | The 6502 drives this line high during the first read cycle of each instruction (opcode fetch). |
| 40 | Ø0 | To FPGA | Microprocessor phase-zero clock. |
| 41 | DEVICE SELECT´ | To FPGA | Active low when address is `$C0n0`–`$C0nX`, where *n* = slot number + 8 and X is between 1 & F. |
| 42–49 | D0–D7 | Both | Three state buffered bidirectional data bus. Write data valid 300ns into Ø0; read data must be stable 100ns before end of Ø0. |
| 50 | +12V | — | +12 volt power supply. 250mA maximum across all peripheral cards. |

---

## Regarding DMA´ and INT, IRQ´, NMI´

The DMA and interrupt signals use a hardware daisy-chain for priority arbitration. Priority is determined by slot position — higher slot numbers have higher priority:

```
Slot 7 (highest) → 6 → 5 → 4 → 3 → 2 → 1 (lowest)
```

**To use interrupts (IRQ´ or NMI´):**
1. Check INT OUT (pin 23) — if active, no higher-priority slot is interrupting
2. Assert IRQ´ or NMI´ to request the interrupt
3. Disable INT IN (pin 28) — this blocks lower-priority slots until you release

**To use DMA:**
1. Check DMA OUT (pin 24) — if active, no higher-priority slot is using DMA
2. Assert DMA´ (pin 22) to take control of the bus
3. Disable DMA IN (pin 27) — this blocks lower-priority slots until you release

For the daisy-chain to function, all slots between yours and slot 7 must either have cards that pass INT OUT → INT IN and DMA OUT → DMA IN, or be empty (with the signals connected on the backplane). [1]

---

## Regarding µPSYNC

The µPSYNC signal enables peripheral cards to detect specific 6502 instructions in hardware. When µPSYNC is high, the data bus contains an opcode being fetched. By sampling the data bus when Ø0 falls, a card can identify which instruction is executing.

For example, detecting an RTI instruction: watch for µPSYNC high with `$40` on the data bus at the falling edge of Ø0. This allows peripheral cards to respond directly to program instructions without software intervention. [2]

---

## Notes

- Active-low signals are indicated with a prime suffix (´), matching Apple's convention. In KiCad, use `~{SIGNAL}` for the equivalent overline notation.
- Slot 7 has unique signals (SYNC, COLOR REF) for video applications.
- The daisy-chain connections (INT IN/OUT, DMA IN/OUT) enable priority-based arbitration across slots.

---

## References

- [1] _Reference Manual for the Apple IIe_ by Apple
- [2] _Understanding the Apple IIe_ by Jim Sather
