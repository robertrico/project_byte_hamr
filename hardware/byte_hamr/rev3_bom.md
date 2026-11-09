# Byte Hamr Rev 3 BOM (DRAFT)

Source of truth: `rev3_bom.csv`, one row per job. Spec and reasoning:
`rev3_spec.md`.

Order of work: BOM, then footprints from the BOM (`footprint` column),
then the schematic. Lowest board cost is the #1 constraint, so parts get
priced and stock-checked before the footprint step.

## CSV columns for pricing

| Column | Use |
|---|---|
| `mpn`, `alt_mpn` | Exact lookup. `alt_mpn` may hold several, `;`-separated; take the cheapest in stock. |
| `dk_search` | Parametric keywords used to find the generic rows (passives, oscillator, connectors, polyfuses). |
| `digikey_pn`, `unit_price_usd` | DigiKey pick and unit price at the 1-board quantity, from `dkp` on 2026-11-04. |
| `qty` | Per board. |

Same-value rows (15k, 10k, 4.7k, 0R) are separate jobs but one part
number; merge them when ordering.

## Cost rules used

- Commodity, multi-source parts only (Nexperia / TI second source on
  every logic part).
- Fewest unique part numbers: one transceiver (74LVC245A) does the
  address, data and input jobs (6 off).
- 1206 passives, one value per job where the circuit allows.
- Every part on the top side.
- Proven Rev 2 circuits kept as is: power, config, JTAG header.
- No USB on the card. Programming is the 2x3 JTAG header plus an external
  Pico 2 per card running pico-dirtyJtag (tooling, not on the card BOM).

## Counts

| | Rev 2 | Rev 3 draft |
|---|---|---|
| Part numbers (same value merged) | 27 | 27 |
| ICs (incl. oscillator) | 15 | 18 |
| R / C / L (placed) | 70 | 99 |

More placements than Rev 2, but no BGA, no two-sided assembly, no 8T245s
and no USB.

## Dropped from Rev 2

| Rev 2 | Why |
|---|---|
| LFE5U-85F-8BG381I | 25F TQFP-144 |
| SDRAM (TSOP-54) | 4x QSPI PSRAM |
| 6x SN74LVC8T245 | out of stock, ~5x price; 74LVC245A at 3.3 V |
| U7 74AHC1G04 | data DIR comes from an FPGA pin |
| 100 MHz oscillator | 25 MHz |
| FT231X, micro-B, 27 R x2, R9, D1 | no USB on the card: every FTDI USB part is out of stock at DigiKey and Mouser; an external Pico 2 per card does JTAG |
| J6 2x3 power breakout | folded into the 2x20 header |
| (planned) SmartPort '245, 10x 0R, 10x DNP C | SmartPort level shifting, pulls and R/C live on the daughterboard; J1 is plain 3.3 V GPIO + power |
| 22 nF 0603 decoupling | 100 nF 1206 |

## Design consequences found while writing the BOM

- **R/W' needs two FPGA pins** (out through the 74LVC125A, in through an
  input buffer). The pin budget in `rev3_spec.md` counts both.
- **Every 5 V input is buffered.** ECP5 inputs aren't 5 V tolerant, so
  all 19 slot inputs go through 74LVC245A
  (VCC 3.3 V, 5 V tolerant I/O).
- **Chain outputs (INT IN, DMA IN) need a defined level while the FPGA is
  unconfigured.** Pull-ups on the '125 inputs pass priority on to
  lower slots while the FPGA is unconfigured.
- **Safe-default pull-ups are 1.1k (pass 3d).** An ECP5 pin pulls down
  with 30 to 150 uA before and during configuration (data sheet pin table
  and DC table). 10k left a net at 1.8 V worst case, under the 74LVC 2.0 V
  VIH, and outside the PSRAM's CE# within 200 mV of Vcc at power-up. 1.1k
  drops 165 mV at 150 uA.
- **Slot inputs that can float get pulls (pass 3d).** INT OUT and DMA OUT
  float when the higher-priority slot is empty; SYNC and COLOR REF exist
  only in slot 7. 10k each: up on 23/24, down on 19/35.
- **Header fuses are two parts.** The 0.25 A fuse is 0.55 to 2.3 ohm, too
  much on the +5 V and +3.3 V pins, which get the 0.75 A part (0.29 ohm
  max, 8 V). The +12 V, -12 V and -5 V pins keep the 16 V part.
- **JTAG header protection.** The external Pico can be powered while the
  card is off: 1.1 k in series with TDI and TMS (about 3 mA into an
  unpowered FPGA), 4.7 k pull-up on TDO. TCK's series part is 330 R
  (pass 3e): against the 4.7 k pull-down, 1.1 k made a 2.67 V high and
  25-33 ns edges, so TCK above ~8 MHz never reached VIH; 330 R gives
  3.08 V and ~8 ns at ~10 mA into an unpowered, hot-socket-rated FPGA.
- **PSRAM clock series slot.** R36 (0 R fitted, 1206) sits in
  `PSRAM_SCLK` between U1 pin 26 and the four PSRAMs, so a 22-33 R can go
  in if the shared clock rings at a higher SPI rate.

## Pricing (DigiKey, 2026-11-06, `dkp bom`)

| Boards | Total | Per board |
|---|---|---|
| 1 | $52.62 | $52.62 |
| 5 | $235.48 | $47.10 |

Parts only: no PCB, stencil or assembly, and not the external Pico 2s.
Same-value rows merged. All 26 lines in stock. Before pass 3d (2026-11-05)
it was $51.34 for one and $46.06 a board for five.

| Share of 1-board cost | |
|---|---|
| FPGA | $22.33 (42%) |
| 4x PSRAM | $7.00 |
| 6x 74LVC245A | $3.36 |
| Flash | $2.98 |
| Everything else | $16.95 |

Findings:

- **FPGA:** -7 is the only 25F TQFP grade DigiKey stocks (103 pcs, 40 week
  lead). -6 and -8: 0 stock, MOQ 300.
- **PSRAM:** DigiKey doesn't sell AP Memory's APS6404L. ESP-PSRAM64H at
  $1.75 (Adafruit, bulk) replaces it; ISSI is $5.72 to $6.14.
- **Flash:** GD25Q128ESIGR $2.98 beats W25Q128JVSIQ ($5.99); IS25LP128F
  is out of stock.
- **FTDI:** FT231X, FT230X, FT232R, FT232H and FT2232H are all 0 stock at
  DigiKey (50 weeks) and Mouser (210 to 294 days). That is why USB left
  the card.

### Mouser comparison (2026-11-06, `mouser bom`)

Mouser can't supply the whole BOM. For 5 boards, the lines it has in
stock come to $241.87 ($67.42 as listed, plus the -6 FPGA at $120.25 and
APS6404L PSRAM at $54.20 in place of the -7 and the ESP-PSRAM64H). It
doesn't carry the Jauch oscillator or either Sullins header, and the
74LVC07A and the Murata inductor are at 0 stock. DigiKey's $235.48 covers
every line.

| Part | DigiKey | Mouser |
|---|---|---|
| LFE5U-25F TG144C | -7: 103 in stock, $22.33 | -6: 190 in stock, $24.05 ($21 @25) |
| PSRAM | ESP-PSRAM64H $1.75 (Adafruit) | APS6404L-3SQR-SN $2.90 ($2.71 @20), 49.5k in stock |
| GD25Q128ESIGR | 11,584, $2.98 | 1,512, $2.88 |
| IS25LP128F (Rev 2 flash) | 0 | 2,175, $7.18 |

Useful from Mouser: a second FPGA source (-6) and the original APS6404L as
a deep-stock PSRAM fallback.

## Still open

1. Programmer bench test: Pico + pico-dirtyJtag on a Rev 2 J4
   (`--detect`, SRAM load, `-f`, timing). openFPGALoader v1.0.0 (installed)
   is the version known to work with dirtyJtag + LFE5U-25F; v1.1.1 fails
   (issue 727, open), and per-card `--pid` needs v1.1.0+. Details and the
   J4 / Pico pin map: `rev3_spec.md` Open 3.
2. GD25Q128E with openFPGALoader: its ID (0xC84018) isn't in the flash
   table (checked at v1.2.0), so it gets basic protection handling and no
   sector lock or quad mode. Try it on hardware; W25Q128JVSIQ is in the
   table if it has to change.
3. ESP-PSRAM64H is single source through Adafruit at DigiKey. Its
   datasheet (V1.0) gives CE# low 8 us max, QPI enter 0x35 and exit 0xF5.
