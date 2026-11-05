# Byte Hamr Rev 2 errata

## E1: U14 drives the slot's wired-OR lines push-pull

**Status:** confirmed on the bench 2026-11-03 (`gateware/rev2/irq_check/`).
No repair done; boards in use as they are. Possible Rev 2 repairs below;
proper fix is Rev 3.

### Fault

U14 (SN74LVC8T245DBQR) carries FPGA outputs to the slot. VCCA = +5 V (slot
side, A), VCCB = 3.3 V (FPGA side, B). **DIR (pin 2) and OE (pin 22) are
tied to GND**, so it always drives B -> A, push-pull.

| U14 pin (A, 5 V) | Net | Slot line | Wired-OR? |
|---|---|---|---|
| 3 | `~{INH}_5V` | ~INH | yes |
| 4 | `~{RES}_5V` | ~RES | yes |
| 5 | `~{IRQ}_5V` | ~IRQ | yes |
| 6 | `~{NMI}_5V` | ~NMI | yes |
| 7 | `~{DMA}_5V` | ~DMA | yes |
| 8 | `INT_IN_5V` | INT daisy chain | no, point to point |
| 9 | `DMA_IN_5V` | DMA daisy chain | no, point to point |

The RTL "releases" a line with `1'bZ`, but U14 still sees the B pin as high
and drives the A side high, which is what an idle card does. Any other device
pulling that line low fights an LVC output rated to +/-32 mA at 4.5 V.

### Measured

Two Rev 2 cards, slots 4 and 2, `irq_check` bitstream, `PR#4`
(`docs/captures/irq_check/2026-11-03_pr4_both_pull_screen.png`):

```
IRQ_CHECK SLOT 4 OPP 2
IRQ T=00 M=64 L=FF-00 S=00 O=00 FAIL
NMI T=00 M=64 L=FF-00 S=00 O=00 FAIL
IRQ+ T=64 M=00 L=06-08 S=00 O=64 PASS
NMI+ T=64 M=00 L=07-09 S=00 O=64 PASS
```

- One card pulls ~IRQ or ~NMI while the other is idle: 0 of 100 taken.
- Both cards pull together: 100 of 100 taken, normal latency.
- So the output path works; the idle card holds the line high.
- Earlier `PR#2` run (single pull only) failed the same way with roles
  swapped.

Not seen so far: Ctrl-Reset always works, and a Super Serial Card has never
misbehaved (its IRQ was probably never used). ~RES still fights U14 during
every reset; the motherboard reset driver wins.

What this breaks: any other card's IRQ or NMI (Mockingboard, SSC
interrupts, a clock card), and any other card's ~DMA or ~INH (Z-80
SoftCard, RAM/ROM cards), whenever a Byte Hamr is plugged in. One Byte
Hamr can't interrupt while a second one is in the machine.

Seen 2026-11-03: a Z-80 card freezes at CP/M boot with both Byte Hamrs in
and boots immediately with them pulled. The Z-80 card holds ~DMA low for
as long as the Z-80 runs, so that is a continuous fight against U14:
don't leave it powered in that state.

### Why a resistor doesn't fix it

- **Pull-up:** the line is already driven high. It changes nothing.
- **Pull-down on the line:** it has to beat 32 mA, so under about 150 ohms.
  That holds the line low all the time (IRQ stuck asserted).
- **Pull on the FPGA side (B):** U14 still drives A push-pull, whatever B
  is.
- **Series resistor, e.g. 1 k:** it weakens our high but also weakens our
  low. Our low against the motherboard pull-up (about 3.3 k) gives
  5 V x 1k/4.3k = 1.2 V, above the 6502's 0.8 V VIL. Two bodged cards, one
  low and one high, sit at about 2.5 V. Fails both ways.

The only per-line fix is a part that conducts one way: a series Schottky,
cathode toward U14. U14 low pulls the line to about 0.4 V (0.1 V LVC VOL
+ 0.3 V Schottky at about 1.5 mA). U14 high reverse-biases the diode, and
the motherboard pull-up owns the high.

RTL can't do it. The FPGA only sets U14's B inputs. B=0 drives A low, B=1
drives A high, and B=Z reads as 1 (pull-up) or floats (no pull-up, worse).
Only OE turns the A side off, and OE is on GND copper. The FPGA can't see
~IRQ or ~NMI either (the nets touch only U14 and the finger), so "join in
when someone else pulls" isn't possible.

### Possible repairs

None done. The boards are in use as they are; with a Byte Hamr in the
machine, other cards' IRQ/NMI/DMA/INH don't work. Each option below is a
possible repair, not a plan.

Feature sizes for all of them: traces 0.15 mm, vias 0.35 mm outer / 0.2 mm
drill (a 0.075 mm ring, probably tented under mask), U14 pins 0.3 mm on
0.635 mm pitch. The edge fingers (1.4 x 7.1 mm) are the only large copper.

| Repair | Lines | Card can still assert? | Work per card |
|---|---|---|---|
| A. Cut ~DMA | DMA | no (Rev 2 can't bus-master anyway) | 1 trace cut, no solder |
| B. Kapton on fingers | IRQ, NMI, DMA | no | none, reversible |
| C. Series Schottky per line | IRQ, NMI (+ DMA, INH) | yes | 1 cut + 2 joints per line |
| D. OE (pin 22) to a GPIO | all seven | yes | 1 pin lift + 3 wires, 2 parts |
| E. OE (pin 22) to +5 V | all seven | no | 1 pin lift + 1 wire |

#### A. Cut ~DMA

Rev 2 can't drive an address (U8, U11, U13 are input-only), so the card has
no use for ~DMA. Cutting the trace takes the card off ~DMA for good and
fixes the Z-80 card. ~DMA (U14 pin 7, finger pad 22) runs on the top side
(F.Cu) with no via: pin 7 down x 217.92 to y 119.57, diagonal to
(218.98, 120.63), then to the finger. Cut the diagonal (higher up it is
0.64 mm from the NMI stub). Two scalpel cuts 0.5 mm apart, lift the sliver,
meter U14 pin 7 to the finger: open.

The same logic applies to ~INH only if no design will ever use it (the
hypervisor's trap plan did: NMI + INH).

#### B. Kapton on the fingers

Thin Kapton (1 mil) over the finger takes the card off that line, the same
as pulling it. Reversible; clean residue with IPA.

| Finger | Side | Slot pin | Note |
|---|---|---|---|
| pad 30 | back (B.Cu) | 30 ~IRQ | opposite pad 21 RDY: tape the back only |
| pad 29 | back (B.Cu) | 29 ~NMI | directly opposite pad 22 |
| pad 22 | front (F.Cu) | 22 ~DMA | one strip wrapped over the edge covers 29 + 22 |

Strips about 1.2 mm wide so they don't touch neighbors. Check after each
insertion that the strip hasn't slid.

#### C. Series Schottky per line

Cut the trace between U14's via and the finger, put a Schottky across the
cut, cathode toward U14. Don't do it at U14.

| Line | U14 pin | Finger | Cut where |
|---|---|---|---|
| ~IRQ | 5 | pad 30, back | B.Cu, via (216.6, 119) to finger, x 216.44, about 2.1 mm |
| ~NMI | 6 | pad 29, back | B.Cu, via (217.35, 119) to finger, x 218.98 |
| ~DMA | 7 | pad 22, front | as in A (or just do A) |
| ~INH | 3 | pad 32, back | B.Cu, via (214.6, 118.25), then a 3.9 mm diagonal to (211.36, 120.99) |
| ~RES | 4 | pad 31, back | not possible here: U13 pin 8 (nRES_READ) taps it at the via (214.85, 119); Ctrl-Reset works anyway |

The bare trace from the via ring to the top of the finger (y about 121.4)
is about 2.1 mm, and the finger top is about 0.4 mm above the card-edge
shoulder, so keep parts up by the via, clear of the slot connector housing.

Parts: Schottky only (a silicon diode's 0.7 V leaves no margin under the
0.8 V VIL), leakage under 1 uA at 5 V.
- Wire: BAT85 (DO-34 axial) glued flat within 1 to 2 cm, band toward U14,
  32 to 36 AWG solderable-enamel magnet wire to the via and to a scraped
  stub just above the finger. 1 s touch per joint, anchor every wire with
  UV resin or epoxy plus Kapton.
- Direct: SOD-523 (RB751S40 or BAT54XV2, about 1.6 mm with leads) across
  the cut. Needs a stereo microscope.

Meter finger to via: about 0.2 to 0.3 V one way, open the other. Every card
that can sit idle while another pulls needs it.

#### D. OE (pin 22) to a GPIO

Lift pin 22 (its GND trace runs under the chip, so the pad stays
grounded) and let the FPGA enable U14 only while it asserts. Idle, all
seven U14 outputs are off: the card is absent from every line.

From the datasheet (TI SN74LVC8T245, SCES584): control inputs are
referenced to VCCA; at VCCA 4.5 to 5.5 V, VIH = 0.7 x VCCA (3.5 V at 5 V)
and VIL = 0.3 x VCCA (1.5 V). "Pull OE high to place all outputs in
3-state mode." 3.3 V from the FPGA can't hold OE off; 0 V turns it on fine.
Circuit, no inversion:

```
 +5V --[10k]--+-- U14 pin 22 (OE, lifted)
              |
            BAT85, band toward FPGA
              |
 GPIO_n ------+   (header pin, 3.3 V)
```

- GPIO 0: OE about 0.3 V, U14 on.
- GPIO 1, Z, or FPGA unconfigured: diode off, OE 5 V, U14 off. Off is the
  default through power-up and in any design that never drives the pin.
  The ECP5 pin never sees 5 V.
- RTL: designs that assert drive GPIO_n low while any U14 line is
  asserted (and for nRES, if a design resets the Apple). tPZL about 24 ns.
- While on, the other U14 lines drive high for the length of the pulse
  (microseconds).
- U14 pins 8/9 (INT/DMA daisy chain to the next slot) float while off.
  Most cards ignore the chain; if one doesn't, 10 k to +5 V on each.
- 5 V from the 2x3 power breakout.

#### E. OE (pin 22) to +5 V

Lift pin 22, pull it to +5 V. U14 is off for good: the card never asserts
anything, and the daisy chain floats as in D.

### Verifying a repair

1. Repair one card. Meter each repaired line with the card idle: about 5 V
   from the motherboard pull-up. If one floats, add 3.3 to 10 k to +5 V.
2. Z-80 card plus the repaired card: CP/M boots (A, B, C, D, E).
3. Both Byte Hamrs in, `PR#` on the unrepaired card: IRQ and NMI PASS
   (B, C, D, E).
4. Where the card can still assert (C, D): `PR#` on the repaired card,
   with the other card repaired too, all four lines PASS. D needs the OE
   drive added to `irq_check`.

## E2: INT/DMA daisy chain wired in reverse

**Status:** found 2026-11-04 in the Rev 3 schematic review, from the slot
pinout. Not measured on the bench.

### Fault

The card should read INT IN (pin 28) and DMA IN (pin 27) from the
higher-priority slot and drive INT OUT (pin 23) and DMA OUT (pin 24) to
the lower-priority slot (MAME `src/devices/bus/a2bus/a2bus.cpp`: "DMA IN -
daisy chain of DMA signal from higher priority devices"; the //e Reference
Manual pinout says the same). Rev 2 does the opposite: U14 pins 8/9 drive
`INT_IN_5V` and `DMA_IN_5V` (slot pins 28/27) push-pull, and the FPGA reads
`INT_OUT_5V` and `DMA_OUT_5V` (pins 23/24). `docs/pinout.md` had the
directions swapped, which is where the error came from.

### Effect

- The card drives the same line as the card one slot higher (its INT OUT /
  DMA OUT feeds this card's pins 28/27): a push-pull fight whenever a
  higher card is in the slot above.
- The card never passes priority on through pins 23/24, so lower slots see
  whatever the floating or pulled level is.
- None of the Rev 2 designs use interrupts or DMA, so no failure was seen.

### Repair

No Rev 2 repair planned. Rev 3 reads 27/28 through an input buffer and
drives 23/24 from a 74LVC125A.

## Rev 3 notes

Rev 3 isn't scheduled. These are what it should carry if it happens.

- **Wired-OR outputs open-drain.** INH, RES, IRQ, NMI and DMA come off U14
  and go on a 74LVC07A: hex, open-drain, 5.5 V tolerant outputs, powered
  from 3.3 V, non-inverting (FPGA 0 = assert, 1 = release), 24 mA sink.
  Alternative: one N-MOSFET per line (2N7002/BSS138 class, FPGA 1 =
  assert, invert in RTL). No pull-ups on the card, or fit them DNP: the
  motherboard pull-up owns the high, and every card adding one lowers the
  total.
- **RDY as an output.** Today it's input-only (E10, `RDY_5V` reaches only
  the input shifter). RDY is wired-OR; driving it lets the card stretch
  CPU cycles (slow SDRAM, DMA). It takes the LVC07A's sixth gate.
- **Readback on ~IRQ and ~NMI.** ~RES already has `nRES_READ`. Add input
  shifter channels for ~IRQ and ~NMI (~DMA and ~INH if there's room) so
  the FPGA can see who holds a line: debugging, sharing, stuck-line
  detection.
- **Daisy chain stays push-pull.** INT OUT/DMA OUT (slot pins 23/24, the
  card's outputs to the next slot; see E2) are point to point with one
  driver: 74AHCT1G125 at 5 V
  (TTL input takes 3.3 V) or 74LVC1T45. Always driven, so the chain never
  floats.
- **DMA.** Address transceivers U8, U11 and U13 need DIR and OE on FPGA
  pins, and R/W needs an FPGA output, before the card can bus-master.
- **Fewer '245s.** U12 to U14 are out of stock and held stock is about 5x
  the price. Moving the five wired-OR lines to an LVC07 leaves U14 with two
  daisy-chain signals, which a pair of 74LVC1T45s could cover. For building
  more Rev 2 boards, SN74LVCH8T245DBQR is a drop-in for the shifters (see
  `gateware/rev2/irq_check/README.md`). It doesn't fix E1: still push-pull.
