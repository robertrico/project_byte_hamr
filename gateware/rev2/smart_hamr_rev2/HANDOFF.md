# HANDOFF — FujiNet back on Byte Hamr (smart_hamr_rev2)

Started 2026-10-11. Goal: put the original custom FujiBoard (ESP32
WROVER + HW-125 SD on perfboard, IDC40 ribbon) back on the Byte Hamr
Rev 2 GPIO header running `smart_hamr_rev2`, and resume bench testing
with scope automation.

Last known state of the rig: it was rewired as
`real Liron <-> DB19 + 2x SN74LVC245AN shifter adapter <-> FujiBoard IDC`
to test FujiNet against a real Liron. That did not work. Now back to
`Byte Hamr (acting as the Liron) <-> ribbon <-> FujiBoard`. Treat the
FujiBoard wiring as **unknown** until Phase A is done.

**Order (2026-10-12): Step 0 below comes first.** Bench the card
against the A2M2053 controller, a SmartPort device already proven on a
real Liron, before FujiNet goes back on. FujiNet has its own open
faults (see the end of this file); with a known-good device, any
failure is ours. FujiNet Phases A-E start only when Step 0 exits.

## Where things stand

| Item | State |
|------|-------|
| `smart_hamr_rev2` RTL | Phases 0-4 bench PASS 2026-05-03 (`docs/bringup_log.md`): ProDOS boot, 10-device enum, READ_BLOCK/STATUS/CONTROL |
| `iwm_asic.v` read-path fixes | 2026-10-12, **uncommitted, not flashed**. MSB framing in every mode; latch-mode D7 hold (`rr7`). See Step 0.1 |
| Build on current branch | `make DESIGN=smart_hamr_rev2 REV=rev2` clean 2026-10-12 with the fixes, Fmax 201.17 MHz vs 7.16 MHz. **Not flashed.** |
| Phase 5 (WRITE_BLOCK, FORMAT, 2nd drive) | Not started |
| Large images (8 MB / 32 MB) | Unverified on this design. Notes are incomplete; start fresh. |
| FujiNet firmware | `~/Development/fujinet-firmware`, branch `byte_hamr_rev2` @ `bef5e8c` = upstream v1.6.x + `fujiapple-custom-wrover` pinmap + SD SPI capped at 4 MHz |

## Step 0 — A2M2053 as a known-good device (before FujiNet)

The A2M2053 controller (`~/Development/a2m2053_controller`) is an
RP2350 that replaces a UniDisk 3.5 board: it answers a Liron card as a
SmartPort drive. Bench-proven on a real Liron + real IWM:
- M6 (2026-10-12): ProDOS boot from its RAM disk, SAVE/LOAD. 150
  packets, 0 checksum errors. ACK 5.3 us after /WRREQ rises (card
  window about 105 us).
- M7 (2026-10-12): real 800K floppy behind it. Read, write, Copy II+
  copy. 314 packets, 0 cksum, 0 late, ACK max 6 us, data-packet ACK
  max 34 us, store hold-off up to 2 s (the card waits).

Real-IWM wire numbers from its build notes (`docs/build_log/m6.md`):
REQ rises 7.7 us before /WRREQ falls; first WR edge 3.95 us after
/WRREQ falls; cell 3.91 us; the card flips Q6 about 6 us after REQ.

### 0.1 Sim: IWM read path vs a software oracle (DONE 2026-10-12)

`vtb/` (Verilator; `make` in `vtb/`). Compiles the A2M2053
`sp_wire.c` by path (`A2M_DIR`), never copied. Packets from its
`sp_encode` go onto rddata; a 6502 model polls like `Lc987`; the
oracle frames the same edges with MSB framing (spec p9). A case
fails the DUT only when the oracle reads the stream.

| Cases | Before | After |
|-------|--------|-------|
| late read start (152) | 84 fail | 0 |
| stray edge before preamble (76) | 20 fail | 0 |
| extra edge inside preamble (136) | 12 fail | 0 |
| 512 B, 6502 latches < 100 ns setup (24) | 24 fail | 0 |
| 512 B, spec 100 ns setup (16) | 0 | 0 |

Two fixes in `iwm_asic.v`:
1. Framing: latch mode synced on the first `$FF` then counted 8
   bits, never re-aligning. Now MSB framing in every mode; the 10-bit
   `$FF`s of the preamble self-sync from any start bit.
2. D7 hold: x7 set on the last fclk edge before PHI0 fell could
   reach the CPU without starting the x7 clear -> same nibble read
   twice. Latch mode now holds D7 (`rr7`) while /DEV is low (spec p8),
   and the clear starts only on a data-register read. Whether real
   hardware hits this depends on sig_7M vs PHI0 phase (not measured).
   If it does, it shows as a duplicate nibble -> checksum -> silent
   ROM retry. Reverting fix 2 alone fails only the 24 setup cases.

### 0.2 Hookup: Byte Hamr header -> A2M2053 host side (unpowered)

**Voltage first.** ECP5 GPIO is LVCMOS33, NOT 5 V tolerant. On the
A2M2053, U1 (host inputs) and U3 (RD + ACK out) run VCCB = bench +5:
- U3 drives RD and ACK at VCCB. At 5 V that kills GPIO_12/13.
- ACK, /WRREQ, /HSTEN have 10k pull-ups to +5 on the B side.
- U1 input-high at VCCB 5 V is 3.5 V; Byte Hamr drives 3.3 V (no
  margin).

Pick one before anything is powered (my call):
- (a) U1 + U3 VCCB to 3.3 V and their B-side pull-ups to 3.3 V.
  Mechanism unplugged (U1 also carries mechanism RD), RAM disk mode.
- (b) A 3.3 V <-> 5 V translator between the two (like the
  `hardware/fujinet_adapter/` '245 board, directions reversed).
- [ ] Choice made, then meter: U3 B side and every A2M2053 pull-up
      rail <= 3.3 V before the ribbon goes on.

Wiring (A2M2053 J1 = its DB-19 IDC breakout, `docs/build_log/m6.md`):

| Byte Hamr | Signal | Dir | A2M2053 J1 | Beep |
|-----------|--------|-----|------------|------|
| GPIO_16 | PH0 / REQ | out | 2 | [ ] |
| GPIO_15 | PH1 | out | 4 | [ ] |
| GPIO_18 | PH2 | out | 6 | [ ] |
| GPIO_17 | PH3 | out | 8 | [ ] |
| GPIO_14 | /WRREQ | out | 10 | [ ] |
| GPIO_10 | wrdata -> WR | out | 18 | [ ] |
| GPIO_12 | rddata <- RD | in | 16 | [ ] |
| GPIO_13 | sense <- ACK | in | 20 | [ ] |
| GND | ground | - | 1 | [ ] |

- J1 9, 11, 13, 15 (Apple rails) and 14 (/HSTEN): no wire. /HSTEN
  stays on its pull-up; sp_bus uses PH1+PH3 as bus enable.
- [ ] Each signal vs its neighbours: no shorts. No shared supply
      rail; GND only.
- rddata: Byte Hamr LPF has `PULLMODE=DOWN` (FujiNet idles low). The
  A2M2053 RD idles high when driven, and the real Liron has a
  pull-up. U3 on/off then makes edges at ENBUS changes. The tb covers
  stray edges; note it, and consider a 3.3 V pull-up for this test.

### 0.3 Bench ladder

Record in `docs/bringup_log.md` (date, bitstream SHA, A2M2053 SHA).
A2M2053 side: `sp` counters + `splog` per transaction.

1. [ ] Flash the fixed `smart_hamr_rev2` (**only when I say go**).
2. [ ] Power order: A2M2053 Pico, its board, then the IIe.
3. [ ] `PR#4`, RAM disk (`sp disk ram`): boots `/M6BOOT`. Pass =
       `cksum=0 late=0 rereads=0 tmo=0`, ACK in the 4-6 us band.
4. [ ] SAVE/LOAD/RUN a file (WRITEBLOCK path, Phase 5 item).
5. [ ] Scope vs the real-Liron numbers above: REQ -> /WRREQ, first
       WR edge after /WRREQ falls (sim estimate for ours: about 5.9 us
       vs 3.95 us real; decoder does not care, record it), cell width.
6. [ ] Floppy mode (`sp disk floppy`, mechanism on): CATALOG, copy a
       file. Hold-offs up to 2 s must not hang the card.
7. [ ] Optional hardware check of fix 1: shorten A2M2053
       `SP_T_TX_PAUSE` (20 us) to 0-5 us. Real Liron still reads it;
       ours should too.

**Exit:** ladder 1-6 pass with the same A2M2053 counters as on the
real Liron. Then FujiNet Phase A.

## Phase A — rig inspection (everything unpowered)

Nothing gets plugged in until every box here is ticked.

### A1. Strip the Liron experiment
- [ ] Photograph the FujiBoard and the adapter as found (notes in log).
- [ ] Adapter schematic for reference: `hardware/fujinet_adapter/`
      (KiCad, untracked: IDC40 -> 2x10 through 2x SN74LVC245AN).
- [ ] Remove the DB19 / SN74LVC245 adapter. Byte Hamr *is* the Liron
      here: its header is 3.3 V LVCMOS33, same as the ESP32, so the
      ribbon goes straight to the FujiBoard. Record anything that was
      cut or jumpered on the FujiBoard for the adapter, and restore it.
- [ ] Is anything wired to ESP32 **IO25** (`SP_RD_BUFFER`, firmware
      drives it as a buffer enable, active LOW)? If a '245 OE went
      there, remove it. On Byte Hamr nothing should hang off IO25.

### A2. Beep table: Byte Hamr header (ribbon end) -> ESP32 pad

Expected wiring = `docs/pin_mapping.md` + firmware
`include/pinmap/a2_custom_wrover.h`. Beep each, then check each
against its neighbours for shorts.

| Hdr GPIO | FPGA pad | Dir | Signal | ESP32 (doc) | ESP32 (firmware) | Beep | Short-free |
|----------|---------|-----|--------|-------------|------------------|------|------------|
| GPIO_10 | K2 | out | wrdata | IO22 | SP_WRDATA 22 | [ ] | [ ] |
| GPIO_12 | H2 | in | rddata | IO14 | SP_RDDATA 14 | [ ] | [ ] |
| GPIO_13 | H1 | in | sense / ACK | IO27 | SP_WRPROT/ACK 27 | [ ] | [ ] |
| GPIO_14 | G2 | out | _wrreq | IO26 | SP_WREQ 26 | [ ] | [ ] |
| GPIO_15 | G1 | out | phase[1] | IO33 | SP_PHI1 33 | [ ] | [ ] |
| GPIO_16 | F2 | out | phase[0] / REQ | IO32 | SP_PHI0 32 | [ ] | [ ] |
| GPIO_17 | F1 | out | phase[3] | IO35 | SP_PHI3 35 | [ ] | [ ] |
| GPIO_18 | E2 | out | phase[2] | IO34 | SP_PHI2 34 | [ ] | [ ] |
| GPIO_20 | D2 | out | _enbl1 | IO36 | SP_DRIVE1 36 | [ ] | [ ] |
| GPIO_9 | K1 | out | _enbl2 | **IO3** | **SP_DRIVE2 21** | [ ] | [ ] |
| GND | — | — | ground | GND | — | [ ] | — |

**Open discrepancy, GPIO_9 / `_enbl2`:** the doc says it lands on
ESP32 **IO3**, which is UART0 RX (USB console + flashing). The firmware
expects `_enbl2` on **IO21** and also uses IO21 as `SP_EXTRA` (an LA
debug *output*). Find out which it is physically:
- If it's on IO3: the FPGA drives the ESP32 console RX line. Expect
  flash/monitor trouble while the Byte Hamr is powered. Move it to
  IO21, or leave GPIO_9 unconnected (second drive isn't tested yet).
- If it's on IO21: the firmware toggles IO21 as a debug output into
  an FPGA output pin = contention. Cut it, or set `SP_EXTRA` to NC.

### A3. Power plan
- [ ] ESP32 powered from its own USB. **GND is the only shared
      rail** with the Byte Hamr. No 3V3 or 5V crosses the ribbon.
- [ ] HW-125 SD module: confirm its supply and that it shares ESP32 GND.

## Phase B — each side alone (powered)

### B1. FujiBoard alone (ribbon unplugged)
- [ ] `./build.sh -b` (`-cb` if stale), `./build.sh -u`, `./build.sh -m`.
- [ ] Boots, SD mounts, WiFi up, no reboot loop.
- [ ] Idle pin levels with a meter or scope: IO14 (rddata) LOW;
      IO27 (ACK) hi-Z (floats; FPGA pulls it up once connected).
- [ ] Keep debug prints LOW for the whole bench. Print floods stall
      the ESP32 receive path and fake "gateware" errors.

### B2. Byte Hamr alone (FujiBoard unplugged)
- [ ] Flash `smart_hamr_rev2`: **only when I explicitly say go**
      (`make DESIGN=smart_hamr_rev2 REV=rev2 prog-flash`).
- [ ] `PR#4`: expect the Liron ROM to report no device / hang, not a crash.
- [ ] Scope with nothing attached: phases cycle on PR#4; sense reads
      HIGH (FPGA pull-up); rddata LOW (pull-down).

## Phase C — connect

1. Apple off, ESP32 unplugged.
2. Ribbon on, GND confirmed first.
3. Power ESP32, wait for boot, then power the Apple.
4. `PR#4` -> FujiNet CONFIG should come up.

## Phase D — scope automation (Siglent SDS1204X-E, oscope MCP)

Probe ground at the Byte Hamr header ground pins (interleaved).

| Ch | Signal | Hdr | V/div | Why |
|----|--------|-----|-------|-----|
| C1 | phase[0] / REQ | GPIO_16 | 1 V | trigger: REQ rise = host starts a packet |
| C2 | sense / ACK | GPIO_13 | 1 V | device handshake |
| C3 | wrdata (Apple -> FujiNet) | GPIO_10 | 1 V | command/data packets out |
| C4 | rddata (FujiNet -> Apple) | GPIO_12 | 1 V | replies |

Extra LA points already exported: GPIO_1 = `/DEVSEL`, GPIO_3 = `nRES_READ`.

Standard captures (trigger C1 rising, SINGLE, confirm READY before
the event; delay = -(6 x tdiv) puts the trigger one division from the left):
- **Command packet**: 200 us/div. Measure REQ-rise -> ACK and
  last wrdata edge -> ACK-assert (CSV + Python; screen reads are estimates).
- **Block reply**: 2 ms/div (~604 bytes x 32 us = ~19.3 ms). Measure
  rddata cell (~4.0 us) and pulse width (~1 us).
- **wrdata cell**: expect 28 fclk = 3.91 us (`iwm_asic.v:140`).
- **Hang triage**: if the Apple ever goes silent, capture GPIO_1 /DEVSEL
  + phases. Continuous `$C0Cx` traffic = Liron waiting on the device
  (`liron-if.asm` `Lc97d`, no timeout). No `/DEVSEL` = 6502 stuck elsewhere.

Captures go to `docs/captures/fujinet/<date>_<what>_<timebase>.png`.
Console log: `docs/captures/fujinet/<date>_console.log`, with
channels / V/div / trigger / timebase recorded next to each result.

## Phase E — test ladder

Record each rung in `docs/bringup_log.md` (date, bitstream SHA, result).

1. [ ] FujiNet CONFIG boots over SmartPort (re-proves Phase 4 baseline).
2. [ ] Mount a 140 KB `.po`, boot, CATALOG.
3. [ ] 800 KB, then 8 MB image: boot + BLOAD a file.
4. [ ] 32 MB image (`../smart_hamr/images/PD-test.2mg`, NS.CLOCK.SYSTEM
       renamed to NSCLOCK.SYS): boot, then BLOAD a file stored above
       block 24576 (e.g. PRODRIVE, key block $B1E7). Compare bytes with
       the image on the Mac.
5. [ ] Same 32 MB image, original volume with NS.CLOCK.SYSTEM active.
6. [ ] Phase 5: WRITE_BLOCK (SAVE a file, cold boot, read back).
7. [ ] Phase 5: second drive (needs the GPIO_9 question settled).
8. [ ] Soak: 50 cold + warm boots.

## Known static risks to watch (from code review 2026-10-11)

- ~~`iwm_asic.v` latch-mode framing never re-aligns~~: confirmed in
  `vtb/` and fixed 2026-10-12 (Step 0.1), with a second read-path bug
  (x7 clear race). Bench confirmation is Step 0.3.
- FujiNet `iwm_ll.cpp:427-487`: receive decode is paced by a timer,
  not the DMA write position; `spi_buffer` not cleared (`:391`). Shows
  up as command checksum errors under CPU load (prints, WiFi, SD).
- FujiNet SD path: FatFS fast-seek off + 4 MHz SD clock means deep
  random reads in big images are slow. Liron waits, so slow, not fatal.
