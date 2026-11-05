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
| FPGA | **LFE5U-25F-7TG144C**. Any speed grade is fast enough for the Apple IIe, so the rule is cheapest in stock; on DigiKey (2026-11-04) -7 is the only grade stocked (103, $22.33 @1; -6 and -8 MOQ 300, 40 weeks): TQFP-144, 24K LUT, 56 EBR (18 Kb), 2 PLL, 98 user I/O, 20 x 20 mm body, 0.5 mm pitch. Listed in `docs/FPGA-DS-02012-3-4-ECP5-ECP5G-Family-Data-Sheet.pdf` (Rev 3.4, Sept 2025). Same Yosys/nextpnr-ecp5 flow: `--25k --package TQFP144 --speed 7` (Trellis database has `TQFP144` for the 25F). |
| Assembly | All parts on the top side. No two-sided assembly. Stencil on the next order. |
| Passives | 1206 preferred everywhere. Inductors stay Murata DFE322520 2.2 uH (1008). |
| Board | 114.5 x 69.55 mm above the finger tab (left edge x 122, top edge y 51.95, so body plus Rev 2's finger tab stays within Apple's 3.05" total and the body under its 2.75"; length well under the 7.00" slot 1-3 limit; layout pass 3 2026-11-05 (routed entirely by Freerouting 2.5.0 with its optimizer), DRC 0 errors / 0 unconnected / parity clean, clock nets (netclass CLK) held at 0.3 mm to other nets by the `clk_spacing` rule in project_byte_hamr.kicad_dru, except inside the U1, U11, Y1 and R3 courtyards (0.5 mm-pitch FPGA pads cannot give 0.3 mm)): grows only toward the keyboard (left); no vias within 9.5 mm of the bottom edge (keepout from y 112); rear edge, top, fingers and notches as Rev 2. 4 layers (decided 2026-11-04 after 2 layers would not route): F.Cu/B.Cu signal, In1 GND plane (unbroken), In2 +3V3 plane with a +5V strip along the board edges (D2 to the bucks) and a +1V1 island under the FPGA; +2V5 routed at 0.4 mm. |
| Clock | 25 MHz (Rev 2's 100 MHz was only ever divided down to 25) |
| Programming | No USB on the card (decided 2026-11-04). The 2x3 JTAG header (Rev 2's J4 pinout: 3V3, GND, TCK, TDI, TMS, TDO) is the only programming port. Each card gets its own external Pico 2 running pico-dirtyJtag on a short ribbon; a long USB cable runs from the Pico out of the case. The FT231X circuit (and every FTDI USB part, all out of stock at DigiKey and Mouser) is gone. |
| Slot | Full support for all 50 slot pins, every signal working as Apple designed it |
| Wired-OR lines | IRQ, NMI, INH, RES, DMA (and RDY) open-drain. Fixes Rev 2 E1. |
| DMA | Must work |
| RAM | 4x 8 MB QSPI PSRAM (32 MB), one chip-select each |
| Header | One shrouded 2x20 header on the rear edge (vertical, cable out the back of the case, courtyard at least 3 mm from the edge) carries GPIO and power: 20 plain 3.3 V FPGA signals (`GPIO_1`..`GPIO_20`; no clocks on the header, a daughterboard brings its own), 5 power, 15 GND. IDC-40 was electrically solid on Rev 2; it only failed on physical fit. |
| Power | Rev 2 power circuit unchanged: three TLV62569 bucks (1.1 V, 2.5 V, 3.3 V), each fed from +5 V, EN tied to +5 V, same divider values, 0 R rail jumpers, STPS2L40AF (D2) from Apple +5 V. Rev 2's USB-side diode (D1) goes with USB. |
| Config flash | 128 Mbit SPI. Disk images live there on designs without an SD card. |
| SmartPort | Off the card (decided 2026-11-05): a daughterboard on J1 does the 5 V level shifting, pulls, series R and shunt C; J1 gives it 3.3 V GPIO and power. |

## Leaning

| Area | Direction |
|---|---|
| Bus transceivers | 74LVC245A (SOIC-20) at 3.3 V: 5 V tolerant inputs, 3.3 V outputs meet the IIe's TTL thresholds. Commodity, multi-source. |
| Open-drain drivers | 74LVC07A (SOIC-14) class, or one N-MOSFET per line |
| Chain outputs | Push-pull buffer, always driven |

## Apple II slot: all 50 pins

Directions are the card's view. Names are Apple's (`docs/pinout.md`). The
card **reads** INT IN (28) and DMA IN (27) from the higher-priority slot
and **drives** INT OUT (23) and DMA OUT (24) to the lower-priority slot
(MAME `src/devices/bus/a2bus/a2bus.cpp`: "DMA IN - daisy chain of DMA
signal from higher priority devices"; the //e Reference Manual pinout
says the same). Rev 2 has it reversed: it reads 23/24 and drives 27/28
(Rev 2 errata E2).

| Pin | Signal | Card direction | Type | Rev 2 | Rev 3 |
|---|---|---|---|---|---|
| 1 | I/O SELECT' | in | | ok | ok |
| 2-17 | A0-A15 | in, **out during DMA** | tri-state | in only | bidirectional |
| 18 | R/W' | in, **out during DMA** | tri-state | in only | bidirectional |
| 19 | SYNC | in (slot 7 only) | | not connected | connect |
| 20 | I/O STROBE' | in | | ok | ok |
| 21 | RDY | out (and read) | open-collector | in only | open-drain out + read |
| 22 | DMA' | out (and read) | open-collector, 3 k pull-up | push-pull (E1) | open-drain out + read |
| 23 | INT OUT | out (to lower slot) | push-pull | reversed (E2) | out |
| 24 | DMA OUT | out (to lower slot) | push-pull | reversed (E2) | out |
| 25 | +5 V | power | 500 mA across all cards | | |
| 26 | GND | power | | | |
| 27 | DMA IN | in (from higher slot) | | reversed (E2) | in |
| 28 | INT IN | in (from higher slot) | | reversed (E2) | in |
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
| Drive R/W' | input only | output path (74LVC125A), enabled by the address DIR net, so it drives only while the address bus points out (DMA). The address OE net is low on every normal cycle and would drive R/W' constantly. |
| Data direction | DIR = R/W' through U7 (74AHC1G04) | DIR on an FPGA pin; U7 dropped. In a DMA read R/W' is high but the card receives, so the inverter would point the wrong way. |

Timing at 25 MHz: about 80 ns to sync the PHI1 edge plus 5 to 10 ns
through the transceiver, so address valid about 90 to 130 ns after the
edge. Inside 213 ns. The edge can also be predicted from the previous cycle.

## Safe defaults while the FPGA is unconfigured

DATA_OE and the new DIR/OE pins are FPGA-driven, so the card must sit off
the bus during power-up and configuration:

- Data 245 OE pulled to disabled.
- Address 245 DIR pulled to slot-to-FPGA (input), OE either way.
- R/W' output buffer enable pulled off (it is the address DIR net, pulled to input).
- Open-drain driver inputs pulled to released.
- Chain outputs: pass-through state or a defined level, never floating.
- JTAG header: an external Pico 2 can be powered while the card is off,
  so TCK, TDI and TMS have 1.1 k series resistors, limiting current into
  an unpowered FPGA to about 3 mA (same part as the config resistor), and TDO has a 4.7 k pull-up (RP2350 TDO
  latching, pico-dirtyJtag issue 35). The 3V3 pin on the header lets the
  Pico firmware wait for card power before driving.

## Pin budget (98 user I/O)

| Block | Pins |
|---|---|
| A0-A15 (bidirectional) + R/W' out + R/W' in | 18 |
| D0-D7 | 8 |
| Inputs: IOSEL, IOSTROBE, DEVSEL, 7M, Q3, PHI1, uPSYNC, PHI0, INT IN, DMA IN, SYNC, COLOR REF | 12 |
| Open-drain: 6 drives + 6 readbacks (RDY, DMA, NMI, IRQ, RES, INH) | 12 |
| Chain outputs: INT OUT, DMA OUT | 2 |
| Transceiver control: DATA_OE, data DIR, address DIR, address OE | 4 |
| **Apple bus** | **56** |
| PSRAM x4 (clock, 4 data, 4 CS) | 9 |
| SPI flash (CSSPIN 49, MOSI 47, MISO 46; SCK is the dedicated CCLK pin 54, reached from user mode through USRMCLK; HOLD/WP tied high) | 3 |
| Oscillator | 1 |
| **Used** | **69** |
| **Spare for the header** | **29** |

The 98 comes from Lattice's ECP5U-25 pinout file
(`docs/FPGA-SC-02033-2-0-ECP5U-25-Pinout.csv`, TQFP144 column), and the
same 98 pins are in the Trellis database nextpnr uses
(`share/trellis/database/ECP5/LFE5U-25F/iodb.json`). The other 46 pins
are power, ground, 2 NC, and dedicated CCLK, INITN, DONE, PROGRAMN,
CFG_0-2 and JTAG (TCK 63, TDI 61, TDO 60, TMS 64). Bank 8 has 12 I/O
(pins 39-41, 44-52); the config flash uses 3 of them, so 9 are free after
configuration (dual-function SPI/parallel config pins, plain I/O in user
mode).

For reference, Rev 2 placed 118 pins on the 85F (20 GPIO, 39 SDRAM, 4 JTAG).

## Header

2x20, shrouded: 20 plain 3.3 V GPIO signals, 5 power, 15 GND.
About 9 FPGA pins are left for LEDs or test pads.

Power on the same header (Rev 2's separate 2x3 J6 power breakout folds in; J4 is the 2x3 JTAG header and stays):

- +12 V and -12 V at one end, away from signal pins, with GND between.
  (The A2M2053 header puts +12 V next to RD/WRTDATA, which is why it is
  probed with the supply off.)
- Shrouded and keyed. Pin order chosen so a one-pin offset can't put 12 V
  on an FPGA pin.
- Polyfuse per power output.
- SmartPort devices are 5 V; ECP5 GPIO is not 5 V tolerant. The level
  shifter, pulls and the series R / shunt C footprints (Rev 1 needed two
  22 pF bodge caps for the ESP32 FujiNet) live on the SmartPort
  daughterboard; every J1 signal pin is 3.3 V.

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

Tree, unchanged from Rev 2 (all three bucks start together from +5 V;
Rev 2 configured reliably this way on the 85F):

```
Apple +5V --D2--+--> TLV62569 1.1V (VCC core)      RA1 15k / RB1 18k
                +--> TLV62569 2.5V (VCCAUX)         RA2 15k / RB2 4.7k
                +--> TLV62569 3.3V (VCCIO, PSRAM,   RA3 15k / RB3 3.3k
                                    flash, LVC)
```

If the 25F misses the power-up rule above, the fix is holding PROGRAMN or
INITN low until 3.3 V is up, not a different tree.

Not yet estimated with Lattice's power calculator. Apple budget: +5 V is
500 mA across all cards (`docs/pinout.md`).

## RAM

| Option | Capacity | Cost (1 off, 2026-11-03) | FPGA pins | Fits 98 with SmartPort? |
|---|---|---|---|---|
| AS4C32M16SB (Rev 2 part, x16) | 64 MB | $30 | 39 | no (104) |
| AS4C16M16SA-6TCN (x16) | 32 MB | $16.91 | 39 | no (104) |
| x8 SDRAM | 32/64 MB | 0 in stock | about 30 | yes, 1 to 6 spare |
| **4x 8 MB QSPI PSRAM** | **32 MB** | **4 x $1.75 = $7.00** (ESP-PSRAM64H, DigiKey 2026-11-04) | **9** | **yes** |

Part: Espressif ESP-PSRAM64H (8 MB, SOIC-8 150 mil, 2.7 to 3.6 V,
133 MHz), DigiKey 1528-4677-ND (sold by Adafruit), $1.75. DigiKey doesn't
carry AP Memory's APS6404L; Mouser does (APS6404L-3SQR-SN, 51k in stock,
$2.90), which makes it the deep-stock fallback. Further fallbacks: ISSI
IS66WVR8M8FALL ($5.72) and IS66WVS8M8FBLL ($6.14). Distributor listings say "64MB"; that is 64 Mbit.
Check whether the WVR "continuous operation" parts drop the 8 us CS-low
limit, and confirm the ESP-PSRAM64H CS-low limit from its datasheet.

Behavior:

| | QSPI PSRAM | Rev 2 SDRAM |
|---|---|---|
| First byte | about 600 ns at 25 MHz, about 300 ns at 50 MHz | about 160 to 240 ns |
| Bandwidth | about 12 MB/s | about 50 MB/s |
| Refresh | inside the chip; CS low max 8 us (APS6404 class) | FPGA (`sdram_ctrl.v`) |
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
| Micro-B tore off (DP051FY9) | bytehamr page | no USB connector on the card |
| USB blocked the lid (Rev 1), hard to reach (Rev 2) | bytehamr page | no USB on the card; external Pico 2 per card, USB cable out of the case |
| Protection diodes fitted backwards (Rev 1) | bytehamr log 2026-01-17 | polarity marks on silk, check assembly drawing |
| nRES read and drive separate | reset notes | keep |
| USB alone powers the bus side | bytehamr page | dropped with USB: the card powers from the slot only |
| SmartPort devices are 5 V | smart_hamr_rev2 handoff | level shifting on the SmartPort daughterboard |
| FujiNet needed 22 pF bodge caps (Rev 1) | bytehamr page | R/C footprints on the SmartPort daughterboard |
| ECP5 power-up order | datasheet | Rev 2 tree kept; hold PROGRAMN/INITN if the 25F needs it |
| KiCad Value drifted from MPN (U2 says MT48LC16M16A2TG, built AS4C32M16SB-7TCNTR) | fab BOM | Value matches MPN on every part |
| LPF generator script lived in `/tmp` | bring-up notes | commit the netlist-to-LPF script |
| Bit-bang JTAG takes minutes | Rev 2 flashing | pico-dirtyJtag: ~660 KB/s measured on RP2040, about 1 s for a 25F bitstream (extrapolated) |
| 25F has 2 PLLs (85F had 4) | datasheet | check no design needs more |

## Open

1. PSRAM latency: check whether any design answers a random Apple read
   straight from RAM within one bus cycle.
2. Transceiver and driver part choices (LVC245 / LVC07 / buffer); pricing.
3. Power: Lattice power estimate for the 25F at 25 MHz; total draw
   against the Apple's 500 mA.
4. Programmer: bench-prove a Pico 2 with pico-dirtyJtag on a Rev 2 J4
   (`--detect`, SRAM load, `-f`, timing), per-card USB PID with
   openFPGALoader v1.1.0+ (`--pid`; installed is v1.0.0), and the open
   LFE5U-25F + dirtyJtag IDCODE bug (openFPGALoader issue 727) on a 25F.
5. PLL count across designs.
6. Edge fingers: hard gold + bevel vs ENIG.
