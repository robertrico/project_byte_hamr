# Byte Hamr Rev 3 schematic plan

The rework of `kicad_rev3/` (a copy of Rev 2's `kicad/`) into Rev 3. Parts
and prices: `rev3_bom.csv`. Requirements: `rev3_spec.md`. FPGA pins:
`rev3_pinmap.csv`, re-assigned from the pass 2 placement (reason
"placement-optimized pass 2"); the original side-based map came from
`scripts/rev3_pinmap.py` (Lattice pinout CSV, cross-checked against
Trellis). Re-running the script overwrites the placement-optimized map.

## Conventions (same as Rev 2)

- Sheets connect through **global labels**.
- Slot-side nets end `_5V`, FPGA-side nets end `_3V3`. Rev 2 label names
  are kept where the signal still exists (`A0_5V`, `~{IRQ}_3V3`,
  `µSync_5V`, `DATA_OE`, `CARD5V`, `JTAG_TCK` ...).
- Readbacks of open-drain lines: `<line>_READ_3V3` (Rev 2's
  `~{RES}__READ_3V3` becomes `~{RES}_READ_3V3`).
- J1's signal pins are plain 3.3 V FPGA GPIO (`GPIO_1`..`GPIO_20`), no
  clocks on the header (a daughterboard brings its own). SmartPort level shifting, pulls, series
  resistors and shunts live on daughterboards, not on the card.
- Every IC's Value is its manufacturer part number. Every part carries
  `MPN` and `Manufacturer` fields from `rev3_bom.csv`.
- All passives 1206 (`Resistor_SMD:R_1206_3216Metric`,
  `Capacitor_SMD:C_1206_3216Metric`), except Rev 2's inductor footprint.

## 74LVC245A wiring

Pinout: 1 DIR, 2-9 A1-A8, 10 GND, 11-18 B8-B1, 19 ~OE, 20 VCC. DIR high
passes A to B. **A side = slot (`_5V`), B side = FPGA (`_3V3`).** VCC is
+3V3 on every part (5 V tolerant I/O).

Channels are listed in order 1 to 8 (A1/B1 = pins 2/18 ... A8/B8 = pins 9/11),
matched to the order of the edge-finger pins under each part (layout pass 2).

| Ref | Job | DIR | ~OE | Channels (A to B) |
|---|---|---|---|---|
| U8 | address low | `ADDR_DIR` | `ADDR_OE` | A0-A7 |
| U9 | address high | `ADDR_DIR` | `ADDR_OE` | A8-A15 |
| U12 | data | `DATA_DIR` | `DATA_OE` | D0-D7 |
| U11 | input buffer | +3V3 | GND | ~I/O_SELECT, ~DEVICE_SELECT, PHI0, µSync, PHI1, Q3, 7M, COLOR_REF |
| U13 | input buffer | +3V3 | GND | R/~W, ~INH (readback), SYNC, ~RES (readback), ~I/O_STROBE, ~IRQ (readback), RDY (readback), ~DMA (readback) |
| U14 | input buffer | +3V3 | GND | ~NMI (readback), INT_IN (slot pin 28), DMA_IN (slot pin 27); 5 unused A inputs to GND, their B outputs no-connect |

Readback B outputs go to `<line>_READ_3V3`. Input-buffer B outputs for
plain inputs go to the `_3V3` name (`7M_3V3`, `PHI0_3V3` ...).

## 74LVC125A (U15) and 74LVC07A (U16)

U15 pinout: 1 1~OE, 2 1A, 3 1Y, 4 2~OE, 5 2A, 6 2Y, 7 GND, 8 3Y, 9 3A,
10 3~OE, 11 4Y, 12 4A, 13 4~OE, 14 VCC.

| Ch | A | Y | ~OE |
|---|---|---|---|
| 1 | `R{slash}~{W}_OUT_3V3` | `R{slash}~{W}_5V` | `ADDR_DIR` (drives R/W' only while the address bus points out, i.e. during DMA) |
| 2 | `INT_OUT_3V3` | `INT_OUT_5V` (slot pin 23) | GND |
| 3 | `DMA_OUT_3V3` | `DMA_OUT_5V` (slot pin 24) | GND |
| 4 | GND | nc | +3V3 |

U16 pinout: 1 1A, 2 1Y, 3 2A, 4 2Y, 5 3A, 6 3Y, 7 GND, 8 4Y, 9 4A, 10 5Y,
11 5A, 12 6Y, 13 6A, 14 VCC. A inputs `RDY_3V3`, `~{DMA}_3V3`,
`~{NMI}_3V3`, `~{IRQ}_3V3`, `~{RES}_3V3`, `~{INH}_3V3`; Y outputs the
matching `_5V` slot nets (open-drain; the Apple's pull-ups hold them
high).

## Safe defaults while the FPGA is unconfigured (10k each)

| Net | Pull | Effect |
|---|---|---|
| `ADDR_DIR` | up | address bus slot to FPGA; R/W' driver off |
| `ADDR_OE` | up | address transceivers off |
| `DATA_OE` | up | data transceiver off |
| `DATA_DIR` | up | data slot to FPGA |
| U16 A inputs (6) | up | all open-drain lines released |
| `INT_OUT_3V3`, `DMA_OUT_3V3` | up | chain passes priority on (my call; spec says "defined level") |
| `~{PSRAM_CS0..3}` (4) | up | PSRAM deselected |

16 resistors.

## Sheet by sheet

### Root (`project_byte_hamr.kicad_sch`)
Remove the `usb` sheet. Title block revision: 3.

### `card_fpga`
- Replace U1 (LFE5U-85F BGA-381) with the new LFE5U-25F-7TG144C symbol
  (one unit per bank plus a power/config unit, built from
  `docs/FPGA-SC-02033-2-0-ECP5U-25-Pinout.csv`). Footprint
  `Package_QFP:LQFP-144_20x20mm_P0.5mm`, 3D model
  `${KIPRJMOD}/3d_models/QFP50P2200X2200X160-144N.step`.
- Every I/O pin gets the global label from `rev3_pinmap.csv`. Spare pins
  get a no-connect flag.
- Y1: 25 MHz oscillator, Jauch `O 25,0-JO32-B-1V3-1-T1-LF`
  (pin 1 EN to +3V3, 2 GND, 3 OUT through R3 (33 R series,
  Stackpole RMCF1206FT33R0) to `CLK_25MHz`, 4 +3V3), 100 nF.
- J6 (2x3 power breakout): delete.
- J1: 2x20 shrouded (`Connector_IDC:IDC-Header_2x20_P2.54mm_Vertical`),
  pinout below. F1-F5 polyfuses in series with each power pin.
- No SmartPort parts on the card: J1's signal pins wire straight to FPGA
  I/O.
- No series resistors on J1's signal pins.

J1 pinout (DRAFT):

| Pin | Net | Pin | Net |
|---|---|---|---|
| 1 | +12 V (`CARD12V`, F) | 2 | GND |
| 3 | -12 V (`CARD-12V`, F) | 4 | GND |
| 5 | -5 V (`CARD-5V`, F) | 6 | GND |
| 7 | +5 V (`CARD5V`, F) | 8 | GND |
| 9 | +3.3 V (F) | 10 | GND |
| 11 | GPIO_1 | 12 | GPIO_2 |
| 13 | GPIO_3 | 14 | GPIO_4 |
| 15 | GND | 16 | GND |
| 17 | GPIO_5 | 18 | GPIO_6 |
| 19 | GPIO_7 | 20 | GPIO_8 |
| 21 | GND | 22 | GND |
| 23 | GPIO_9 | 24 | GPIO_10 |
| 25 | GPIO_19 | 26 | GPIO_20 |
| 27 | GND | 28 | GND |
| 29 | GPIO_11 | 30 | GPIO_12 |
| 31 | GPIO_13 | 32 | GPIO_14 |
| 33 | GND | 34 | GND |
| 35 | GPIO_15 | 36 | GPIO_16 |
| 37 | GPIO_17 | 38 | GPIO_18 |
| 39 | GND | 40 | GND |

### `card`
- Keep J3 (edge connector) and its existing `_5V` labels. Connect pin 19
  (`SYNC_5V`) and pin 35 (`COLOR_REF_5V`), unconnected on Rev 2.
- Delete U7 (74AHC1G04) and all six SN74LVC8T245 (U8, U9, U11-U14).
- Add U8, U9, U11, U12, U13, U14 (74LVC245AD,118), U15 (74LVC125AD,118),
  U16 (74LVC07AD,118) wired as above, 100 nF each, plus the safe-default
  pulls.

### `ram`
- Delete U2 (SDRAM) and its decoupling.
- Add U2, U18, U19, U20: ESP-PSRAM64H (symbol
  `Memory_RAM:APS6404L-3SQRx-SN`, same pinout: 1 ~CE, 2 SO/SIO1, 3 SIO2,
  4 VSS, 5 SI/SIO0, 6 SCLK, 7 SIO3, 8 VCC). Shared `PSRAM_SCLK`,
  `PSRAM_SIO0-3`; ~CE to `~{PSRAM_CS0..3}`. 100 nF each, CS pull-ups.

### `flash`
- U10 Value `GD25Q128ESIGR`, footprint
  `Package_SO:SOIC-8_5.3x5.3mm_P1.27mm`. Connect to `FLASH_MOSI`,
  `FLASH_MISO`, `~{FLASH_CS}`, and the FPGA's dedicated CCLK pin.
- Keep the config resistors (R11, R12, R27-R33) on PROGRAMN, INITN,
  DONE, CFG_0-2 at the new symbol's pins: CCLK 54, INITN 55, DONE 56,
  PROGRAMN 57, CFG_2 58, CFG_1 59, CFG_0 62. Re-check against sysCONFIG
  (`docs/FPGA-TN-02039-2-5-ECP5-and-ECP5-5G-sysCONFIG.pdf`) for SPI
  master mode.
- Move J4 (JTAG 2x3, Rev 2 pinout) here from `usb`. 1.1k series on TCK,
  TDI, TMS between J4 (`JTAG_*_HDR`) and the FPGA (`JTAG_*`; TCK 63,
  TDI 61, TDO 60, TMS 64). 4.7k pull-up on `JTAG_TDO`.

### `power`
- Delete D1 and the `USB5V` net. Keep D2 and everything else.
- FPGA decoupling: one 100 nF per supply pin: VCC 20, 29, 38, 66, 83, 130
  (6); VCCAUX 17, 53, 96, 132 (4); VCCIO 9, 16, 36, 43, 70, 86, 100, 122,
  137 (9). 19 total, replacing Rev 2's 22 nF caps. Keep the 2.2 uF bulk.
  All VCCIO banks on +3V3.

### `usb`
Delete (J4 moved to `flash`).

## Not in this pass
PCB placement and routing (done separately in layout passes 1 and 2). ERC
must pass per sheet.
