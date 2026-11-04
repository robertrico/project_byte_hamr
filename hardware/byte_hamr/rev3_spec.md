# Byte Hamr Rev 3 spec (DRAFT)

**Draft, 2026-11-03. I haven't settled the full design.** This captures
what came out of the Rev 2 errata session (`rev2_errata.md`, E1). Items are
split into Decided, Leaning (current direction, not
committed), and Open.

Rev 3 is close to a clean redesign. **Low cost is the #1 constraint.**

## Decided

| Area | Decision |
|---|---|
| Cost | Lowest cost is the #1 constraint |
| FPGA | **LFE5U-25F-7TG144C**: TQFP-144, 24K LUT, 56 EBR (18 Kb), 2 PLL, 96 user I/O. Listed in `docs/FPGA-DS-02012-3-4-ECP5-ECP5G-Family-Data-Sheet.pdf` (Rev 3.4, Sept 2025). Same Yosys/nextpnr-ecp5 flow: `--25k --package TQFP144 --speed 7` (Trellis database has `TQFP144` for the 25F). |
| Assembly | All parts on the top side. No two-sided assembly. Stencil on the next order. |
| Passives | 1206 preferred everywhere. Inductors stay Murata DFE322520 2.2 uH (1008). |
| Board | Bigger card is fine. 2 or 4 layers. |
| Clock | 25 MHz (Rev 2's 100 MHz was only ever divided down to 25) |
| USB | Keep the ULX3S-derived FT231X circuit. Connector reachable from outside the case so reflashing never means reaching into the Apple. |
| Slot | Full support for all 50 slot pins, every signal working as Apple designed it |
| Wired-OR lines | IRQ, NMI, INH, RES, DMA (and RDY) open-drain. Fixes Rev 2 E1. |
| DMA | Must work |
| RAM | 4x 8 MB QSPI PSRAM (32 MB), one chip-select each |
| Header | One shrouded header carries GPIO and power. Sized from spare pins. IDC-40 was electrically solid on Rev 2; it only failed on physical fit. |
| SmartPort | At least 10 header signals |

## Leaning

| Area | Direction |
|---|---|
| Bus transceivers | 74LVC245A (SOIC-20) at 3.3 V: 5 V tolerant inputs, 3.3 V outputs meet the IIe's TTL thresholds. Commodity, multi-source. |
| Open-drain drivers | 74LVC07A (SOIC-14) class, or one N-MOSFET per line |
| Chain outputs | Push-pull buffer, always driven |
| Power | Buck 5 V to 3.3 V (TLV62569, as Rev 2), LDOs 3.3 V to 2.5 V and 3.3 V to 1.1 V |
| Header | 2x20 with 20 signals (10 SmartPort, 10 general), 5 power, 15 GND |
| Connector | Through-hole shell tabs (DP051FY9's micro-B tore off) |

## Apple II slot: all 50 pins

Directions are the card's view. Names are Apple's (`docs/pinout.md`). The
daisy chain is named from the Apple's side: the card **reads** INT OUT (23)
and DMA OUT (24) from the higher-priority slot and **drives** INT IN (28)
and DMA IN (27) to the lower-priority slot. Rev 2 already does this
correctly.

| Pin | Signal | Card direction | Type | Rev 2 | Rev 3 |
|---|---|---|---|---|---|
| 1 | I/O SELECT' | in | | ok | ok |
| 2-17 | A0-A15 | in, **out during DMA** | tri-state | in only | bidirectional |
| 18 | R/W' | in, **out during DMA** | tri-state | in only | bidirectional |
| 19 | SYNC | in (slot 7 only) | | not connected | connect |
| 20 | I/O STROBE' | in | | ok | ok |
| 21 | RDY | out (and read) | open-collector | in only | open-drain out + read |
| 22 | DMA' | out (and read) | open-collector, 3 k pull-up | push-pull (E1) | open-drain out + read |
| 23 | INT OUT | in (from higher slot) | | ok | ok |
| 24 | DMA OUT | in (from higher slot) | | ok | ok |
| 25 | +5 V | power | 500 mA across all cards | | |
| 26 | GND | power | | | |
| 27 | DMA IN | out (to lower slot) | push-pull | ok (via U14) | ok |
| 28 | INT IN | out (to lower slot) | push-pull | ok (via U14) | ok |
| 29 | NMI' | out (and read) | open-collector, 3.3 k pull-up | push-pull (E1) | open-drain out + read |
| 30 | IRQ' | out (and read) | open-collector, 3.3 k pull-up | push-pull (E1) | open-drain out + read |
| 31 | RES' | out and in | open-collector | push-pull out (E1) + read | open-drain out + read |
| 32 | INH' | out (and read) | open-collector, 2 k pull-up | push-pull (E1) | open-drain out + read |
| 33 | -12 V | power | 200 mA across all cards | | |
| 34 | -5 V | power | 200 mA across all cards | | |
| 35 | COLOR REF | in (slot 7 only) | | not connected | connect |
| 36 | 7M | in | | ok | ok |
| 37 | Q3 | in | | ok | ok |
| 38 | PHI1 | in | | ok | ok |
| 39 | uPSYNC | in | | ok | ok |
| 40 | PHI0 | in | | ok | ok |
| 41 | DEVICE SELECT' | in | | ok | ok |
| 42-49 | D0-D7 | bidirectional | tri-state | ok (DIR = inverted R/W) | DIR on an FPGA pin |
| 50 | +12 V | power | 250 mA across all cards | | |

Pull-ups and current limits from `docs/pinout.md`. Open-drain sink at INH's
2 k: about 2.5 mA.

## DMA

From the Apple IIe Technical Notes #2, "Hardware Protocol for Doing DMA"
(`docs/Apple_IIe_Technical_Notes.pdf`):

1. Pull DMA' low during PHI1, never during PHI0 (it flips the IIe's data
   '245).
2. Wait 30 ns, then drive address and R/W'.
3. Address and R/W' valid **within 213 ns of PHI1's rising edge**. The note
   calls this the main cause of IIe DMA card failures.
4. Don't drive the data bus during PHI1.
5. Release DMA' only outside PHI0.

Hardware this needs:

| Need | Rev 2 | Rev 3 |
|---|---|---|
| Drive the address bus | U8/U11/U13 DIR to +5 V, OE to GND (input only) | address 245s with DIR and OE on FPGA pins (one pair shared) |
| Drive R/W' | input only | output path, enabled with the address OE net |
| Data direction | DIR = R/W' through U7 (74AHC1G04) | DIR on an FPGA pin; U7 dropped. In a DMA read R/W' is high but the card receives, so the inverter would point the wrong way. |

Timing at 25 MHz: about 80 ns to sync the PHI1 edge plus 5 to 10 ns
through the transceiver, so address valid about 90 to 130 ns after the
edge. Inside 213 ns. The edge can also be predicted from the previous cycle.

## Safe defaults while the FPGA is unconfigured

DATA_OE and the new DIR/OE pins are FPGA-driven, so the card must sit off
the bus during power-up and configuration:

- Data 245 OE pulled to disabled.
- Address 245 DIR pulled to slot-to-FPGA (input), OE either way.
- R/W' output buffer enable pulled off.
- Open-drain driver inputs pulled to released.
- Chain outputs: pass-through state or a defined level, never floating.
- Same rule when only USB powers the card (Rev 2's diode OR lets USB
  power the bus side): nothing may drive an unpowered Apple.

## Pin budget (96 user I/O)

| Block | Pins |
|---|---|
| A0-A15 + R/W' (bidirectional) | 17 |
| D0-D7 | 8 |
| Inputs: IOSEL, IOSTROBE, DEVSEL, 7M, Q3, PHI1, uPSYNC, PHI0, INT OUT, DMA OUT, SYNC, COLOR REF | 12 |
| Open-drain: 6 drives + 6 readbacks (RDY, DMA, NMI, IRQ, RES, INH) | 12 |
| Chain outputs: INT IN, DMA IN | 2 |
| Transceiver control: DATA_OE, data DIR, address DIR, address OE | 4 |
| **Apple bus** | **55** |
| PSRAM x4 (clock, 4 data, 4 CS) | 9 |
| SPI flash (CS, SCK, MOSI, MISO; HOLD/WP tied high) | 4 |
| Oscillator | 1 |
| **Used** | **69** |
| **Spare for the header** | **about 27** |

Plus or minus 3: bank 8 shares pins with configuration (INITN, DONE,
PROGRAMN, flash). Whether those count against the 96 comes from the
TQFP-144 pinout file. JTAG is dedicated and not counted.

For reference, Rev 2 placed 118 pins on the 85F (20 GPIO, 39 SDRAM, 4 JTAG).

## Header

| Option | Signals | Power | GND | Notes |
|---|---|---|---|---|
| 2x20 | 20 | 5 | 15 | 10 SmartPort + 10 general. About 7 FPGA pins left for LEDs or test pads. |
| 2x25 | 27 | 5 | about 18 | Every spare pin out |

Power on the same header (Rev 2's separate 2x3 J4/J6 breakout folds in):

- +12 V and -12 V at one end, away from signal pins, with GND between.
  (The A2M2053 header puts +12 V next to RD/WRTDATA, which is why it is
  probed with the supply off.)
- Shrouded and keyed. Pin order chosen so a one-pin offset can't put 12 V
  on an FPGA pin.
- Polyfuse per power output.
- SmartPort devices are 5 V; ECP5 GPIO is not 5 V tolerant. The SmartPort
  group needs its own level shifter, so that part of the header is 5 V and
  the rest 3.3 V. Label both on the silkscreen.
- Footprints for series R or shunt C on the SmartPort lines (Rev 1 needed
  two 22 pF bodge caps for the ESP32 FujiNet).

Final size depends on where it physically fits on the bigger board.

## Power

ECP5 rails (datasheet Rev 3.4):

| Rail | Range | 25F standby current |
|---|---|---|
| VCC (core) | 1.045 to 1.155 V | 77 mA (ICC) |
| VCCAUX | 2.375 to 2.625 V | 16 mA (ICCAUX) |
| VCCIO | 3.3 V (max 3.465) | |

Power-up rule: VCCIO8 (configuration bank) must reach the SPI flash's VIH
before VCC or VCCAUX reaches VPORUP; otherwise hold PROGRAMN or INITN low
until it does.

Tree, every regulator fed from the one before, so VCCIO8 always comes up
first:

```
Apple +5V --> 3.3V buck --+--> 3.3V (VCCIO, PSRAM, flash, LVC parts, FT231X)
                          +--> LDO 2.5V (VCCAUX, ~16 mA)
                          +--> LDO 1.1V (VCC core)
```

Not yet estimated with Lattice's power calculator. Apple budget: +5 V is
500 mA across all cards (`docs/pinout.md`).

## RAM

| Option | Capacity | Cost (1 off, 2026-11-03) | FPGA pins | Fits 96 with SmartPort? |
|---|---|---|---|---|
| AS4C32M16SB (Rev 2 part, x16) | 64 MB | $30 | 39 | no (104) |
| AS4C16M16SA-6TCN (x16) | 32 MB | $16.91 | 39 | no (104) |
| x8 SDRAM | 32/64 MB | 0 in stock | about 30 | yes, 1 to 6 spare |
| **4x 8 MB QSPI PSRAM** | **32 MB** | **4 x $5.40 = $21.60** | **9** | **yes** |

Parts: APS6404 class, ISSI IS66WVS8M8 / IS66WVR8M8 (8 MB, SOIC-8, 104 MHz).
Distributor listings say "64MB"; that is 64 Mbit. Check whether the WVR
"continuous operation" parts drop the 8 us CS-low limit.

Behavior:

| | QSPI PSRAM | Rev 2 SDRAM |
|---|---|---|
| First byte | about 600 ns at 25 MHz, about 300 ns at 50 MHz | about 160 to 240 ns |
| Bandwidth | about 12 MB/s | about 50 MB/s |
| Refresh | inside the chip; CS low max 8 us (APS6404) | FPGA (`sdram_ctrl.v`) |
| Banks | 32 MB = 512 x 64 KB; top 2 bank bits pick the chip | 1024 x 64 KB |

Fitting 1 to 4 chips gives 8 to 32 MB; gateware reads each chip's ID at
boot. A new QSPI controller replaces `sdram_ctrl.v`, so every SDRAM design
is ported once (block_hamr, project_obscurus, coprocessor window,
flash_hamr if moved).

## Gateware fit (from `reporting/*_utilization.txt`)

| Design | LUT4 | DP16KD |
|---|---|---|
| logic_hamr (Rev 1) | 7,130 | 0 |
| flash_hamr | 3,981 | 24 |
| smartport_host | 3,427 | 24 |
| project_obscurus | 2,978 | 48 |
| smart_hamr_rev2 | 2,135 | 0 |
| b8008_hamr | 1,496 | 10 |
| block_hamr | 1,197 | 1 |
| irq_check | 539 | 0 |

All fit the 25F (24K LUT, 56 EBR). Clocks today: block_hamr PLL 100 to 25;
obscurus and b8008 divide 100 by 4; smart_hamr_rev2 runs on `sig_7M`;
irq_check runs at 100 and would move to 40 ns steps.

## Rev 2 lessons

| Lesson | Source | Rev 3 |
|---|---|---|
| E1: U14 drives wired-OR lines push-pull | `rev2_errata.md` | open-drain drivers |
| Can't bus-master | Rev 2 open problems | DMA section above |
| 8T245 out of stock, about 5x price | errata | commodity LVC parts |
| BGA forces two-sided assembly (10 caps + U7 on the back) | board file | TQFP, all top side |
| 85F far larger than needed (max 8% LUT) | `reporting/` | 25F |
| 100 MHz only divided to 25 | gateware | 25 MHz |
| 0603 too small | hand assembly | 1206 |
| IDC-40 didn't fit physically | Rev 2 build | header placement |
| Micro-B tore off (DP051FY9) | bytehamr page | through-hole shell tabs |
| USB blocked the lid (Rev 1), hard to reach (Rev 2) | bytehamr page | reachable from outside the case |
| Protection diodes fitted backwards (Rev 1) | bytehamr log 2026-01-17 | polarity marks on silk, check assembly drawing |
| nRES read and drive separate | reset notes | keep |
| USB alone powers the bus side | bytehamr page | keep, with safe defaults |
| SmartPort devices are 5 V | smart_hamr_rev2 handoff | level-shifted SmartPort group |
| FujiNet needed 22 pF bodge caps (Rev 1) | bytehamr page | R/C footprints |
| ECP5 power-up order | datasheet | cascaded regulators |
| KiCad Value drifted from MPN (U2 says MT48LC16M16A2TG, built AS4C32M16SB-7TCNTR) | fab BOM | Value matches MPN on every part |
| LPF generator script lived in `/tmp` | bring-up notes | commit the netlist-to-LPF script |
| Bit-bang JTAG takes minutes | Rev 2 flashing | accepted (FT231X kept) |
| 25F has 2 PLLs (85F had 4) | datasheet | check no design needs more |

## Open

1. TQFP-144 pinout: firm user-I/O count, bank 8 config pins, exposed pad
   or not (package drawing).
2. PSRAM latency: check whether any design answers a random Apple read
   straight from RAM within one bus cycle.
3. Header size and position; final GPIO count.
4. Transceiver and driver part choices (LVC245 / LVC07 / buffer); pricing.
5. Power: Lattice power estimate for the 25F at 25 MHz; buck vs LDO for
   1.1 V and 3.3 V; total draw against the Apple's 500 mA.
6. Layer count (2 vs 4) once placement is known.
7. USB connector type and position.
8. PLL count across designs.
9. Edge fingers: hard gold + bevel vs ENIG.
