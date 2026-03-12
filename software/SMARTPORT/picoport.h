#ifndef PICOPORT_H
#define PICOPORT_H

/*
 * PicoPort — Pico W Pin Assignments for Byte Hamr SmartPort
 *
 * 11 wires + GND between FPGA and Pico.
 * Pico GP0-1 reserved for UART0 (debug probe).
 *
 *   Pico GP  |  FPGA GPIO  |  Signal         |  Direction
 *   -------  |  ---------- |  -------------- |  ----------
 *   GP21     |  GPIO9      |  rddata          |  Pico → FPGA (PIO TX)
 *   GP3      |  GPIO2      |  phase[2]        |  FPGA → Pico
 *   GP4      |  GPIO3      |  phase[3]        |  FPGA → Pico
 *   GP5      |  GPIO4      |  phase[0] / REQ  |  FPGA → Pico
 *   GP6      |  GPIO5      |  phase[1]        |  FPGA → Pico
 *   GP7      |  GPIO6      |  ~DEV_SEL        |  FPGA → Pico (debug)
 *   GP8      |  GPIO7      |  _wreq           |  FPGA → Pico
 *   GP9      |  GPIO8      |  sense/ACK       |  Pico → FPGA (open-drain)
 *   GP10     |  GPIO1      |  _enbl1          |  FPGA → Pico
 *   GP11     |  GPIO10     |  wrdata          |  FPGA → Pico (PIO RX)
 *   GP12     |  GPIO11     |  _enbl2          |  FPGA → Pico
 *
 * Phase pins are contiguous on GP3-6, but in FPGA order
 * (PH2, PH3, PH0, PH1). Software unscrambles via LUT.
 */

/* Raw phase pins (INPUT) — contiguous GP3-6 */
#define SP_PHI_BASE     3       /* GP3 = first phase pin */
#define SP_PHI_MASK     0x0F    /* 4 bits after shifting */

/* REQ = PH0 = GP5 */
#define SP_REQ          5

/* Data lines — PIO pins */
#define SP_WRDATA       11      /* GP11 INPUT  — FM data from Apple II (PIO RX) */
#define SP_RDDATA       21      /* GP21 OUTPUT — FM data to Apple II (PIO TX) */

/* Handshake */
#define SP_ACK          9       /* GP9 OUTPUT open-drain — active LOW, hi-Z = deasserted */

/* Other signals */
#define SP_ENBL1        10      /* GP10 INPUT — drive 1 enable (active LOW) */
#define SP_DEVSEL       7       /* GP7  INPUT — ~DEV_SEL debug (active LOW) */
#define SP_WREQ         8       /* GP8  INPUT — write request (active LOW) */
#define SP_ENBL2        12      /* GP12 INPUT — drive 2 enable (active LOW) */

/* SPI0 — SD Card daughter board */
#define SD_PIN_MISO     16      /* GP16 — SPI0 RX  (SD DO)  */
#define SD_PIN_CS       17      /* GP17 — SPI0 CSn (SD CS)  */
#define SD_PIN_SCK      18      /* GP18 — SPI0 SCK (SD CLK) */
#define SD_PIN_MOSI     19      /* GP19 — SPI0 TX  (SD DI)  */

/* Bus state constants (logical phase values after LUT unscramble) */
#define SP_BUS_COMMAND  0x0B    /* PH3:PH2:PH1:PH0 = 1011 */
#define SP_BUS_IDLE     0x0A    /* PH3:PH2:PH1:PH0 = 1010 */
#define SP_BUS_RESET    0x05    /* PH3:PH2:PH1:PH0 = 0101 */

#endif /* PICOPORT_H */
