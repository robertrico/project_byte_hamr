# Byte Hamr Rev 2 errata

## E1: U14 drives the slot's wired-OR lines push-pull

**Status:** confirmed on the bench 2026-11-03 (`gateware/rev2/irq_check/`).
Bodge below; proper fix is Rev 3.

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

Only a part that conducts one way makes it open-drain: a series Schottky,
cathode toward U14. U14 low pulls the line to about 0.4 V (0.1 V LVC VOL
+ 0.3 V Schottky at about 1.5 mA). U14 high reverse-biases the diode, and
the motherboard pull-up owns the high.

RTL can't do it. The FPGA only sets U14's B inputs. B=0 drives A low, B=1
drives A high, and B=Z reads as 1 (pull-up) or floats (no pull-up, worse).
Only OE turns the A side off, and OE is on GND copper. The FPGA can't see
~IRQ or ~NMI either (the nets touch only U14 and the finger), so "join in
when someone else pulls" isn't possible.

Rejected alternatives to the diode:
- **OE (pin 22) to +5 V:** turns all of U14 off. The card can never assert
  any of these lines, and the INT/DMA daisy chain to the slots below
  floats.
- **OE or DIR (pin 2) to a GPIO:** one enable shared by all seven channels.
  Enabling to assert IRQ also drives INH/RES/NMI/DMA high for as long as
  IRQ is held. When disabled, the daisy chain floats. DIR and OE look to be
  referenced to VCCA (5 V; check the datasheet), so 3.3 V is marginal and
  needs a 2N7002 inverter plus a pull-up. Pin 22/2 is a 0.635 mm pin lift,
  and every design needs OE logic plus a GPIO. DIR's only win is
  readback, which needs break-before-make so U14 and the FPGA don't fight.

### No-surgery option: tape the fingers

If the card doesn't need to interrupt or DMA (true of every Rev 2 design
except `irq_check` and the hypervisor's NMI trap), insulate its fingers
with thin Kapton (1 mil). U14 then drives nothing on those lines, and the
card is electrically absent on them, the same as with the card pulled.
Reversible; clean residue with IPA.

| Finger | Side | Slot pin | Note |
|---|---|---|---|
| pad 30 | back (B.Cu) | 30 ~IRQ | opposite pad 21 RDY: tape the back only |
| pad 29 | back (B.Cu) | 29 ~NMI | directly opposite pad 22 |
| pad 22 | front (F.Cu) | 22 ~DMA | one strip wrapped over the edge covers 29 + 22 |

Fingers are 1.4 mm wide on 2.54 mm pitch: cut strips about 1.2 mm so they
don't touch neighbors. Check after each insertion that the strip hasn't
slid.

### Bodge (card keeps the ability to assert)

Per line, between U14's via and the edge finger (~DMA has no via: top
side). Don't do it at U14: the pins are 0.635 mm apart.

Full repair is four lines per card. Every net below touches only U14 and
the finger, except ~RES (see its row).

| Line | U14 pin | Finger | Cut where | Repair? |
|---|---|---|---|---|
| ~IRQ | 5 | pad 30, back | B.Cu, via (216.6, 119) to finger, x 216.44, about 2.1 mm | yes |
| ~NMI | 6 | pad 29, back | B.Cu, via (217.35, 119) to finger, x 218.98 | yes |
| ~DMA | 7 | pad 22, front | F.Cu, no via: pin 7 down x 217.92 to y 119.57, diagonal to (218.98, 120.63), then to the finger. Cut the diagonal (higher up it's 0.64 mm from the NMI stub) | yes (Z-80 card confirmed) |
| ~INH | 3 | pad 32, back | B.Cu, via (214.6, 118.25), then a 3.9 mm diagonal to (211.36, 120.99): roomiest run of the four | yes |
| ~RES | 4 | pad 31, back | no: U13 pin 8 (nRES_READ) taps it at the via (214.85, 119), so a cut must go on the F.Cu stub at U14 pins | no, Ctrl-Reset works |

The bare trace from the via ring to the top of the finger (y about 121.4)
is only about 2.1 mm. The top of the finger is about 0.4 mm above the
card-edge shoulder, so keep parts up by the via, clear of the slot
connector housing.

**Parts:** 4 per card, 8 for two cards; buy spares. Schottky only (a
silicon diode's 0.7 V leaves no margin under the 0.8 V VIL). Leakage must
be low (under 1 uA at 5 V) so an idle card stays invisible.

**Wire method (recommended for hand work): BAT85 (DO-34 axial).** Cut the
trace (two scalpel cuts 0.5 mm apart, lift the sliver, meter open). Glue the
diode flat in open board area within 1 to 2 cm, band toward U14. 32 to 36
AWG solderable-enamel magnet wire from the via (DMA: a scraped spot on the
U14 side) to the band end, and from a scraped stub just above the finger to
the other end. Flux, 1 s touch per joint. Anchor every wire with UV resin or
epoxy plus Kapton right away; a snagged wire lifts the trace. Meter finger
to via: about 0.2 to 0.3 V one way, open the other.

**Direct on the trace (microscope / microsoldering shop): SOD-523,
about 1.6 mm with leads, RB751S40 or BAT54XV2.** No wires, cleanest result,
but it needs a stereo microscope and practice.
1. Cut the trace just below the via ring.
2. Scrape the mask on the finger-side stub. Keep solder off the finger and
   off the pour next to it.
3. Cathode to the via side, anode to the finger-side stub.

Every card that can sit idle while another pulls needs the bodge, so in
practice every card.

### Verifying the bodge

1. Bodge ~IRQ only, on the slot 2 card.
2. Meter ~IRQ with the card idle: about 5 V from the motherboard pull-up. If
   it floats, add 3.3 to 10 k to +5 V.
3. `PR#4`. Expect `IRQ` PASS, `NMI` FAIL (control, not bodged), `IRQ+`
   S=64 (slot 4 now wins alone).
4. Bodge ~NMI, `PR#4` and `PR#2`: all four lines PASS on both.

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
- **Daisy chain stays push-pull.** INT_IN/DMA_IN (the card's outputs to
  the next slot) are point to point with one driver: 74AHCT1G125 at 5 V
  (TTL input takes 3.3 V) or 74LVC1T45. Always driven, so the chain never
  floats.
- **DMA.** Address transceivers U8, U11 and U13 need DIR and OE on FPGA
  pins, and R/W needs an FPGA output, before the card can bus-master.
- **Fewer '245s.** U12 to U14 are out of stock and held stock is about 5x
  the price. Moving the five wired-OR lines to an LVC07 leaves U14 with two
  daisy-chain signals, which a pair of 74LVC1T45s could cover. For building
  more Rev 2 boards, SN74LVCH8T245DBQR is a drop-in for the shifters (see
  `gateware/rev2/irq_check/README.md`). It doesn't fix E1: still push-pull.
