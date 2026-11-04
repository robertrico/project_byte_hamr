# irq_check: U14 contention test

Build: `make DESIGN=irq_check REV=rev2` (no design LPF; falls back to
`gateware/rev2/constraints/byte_hamr.lpf`; assembles `irqrom.S` ->
`irq_rom.mem` first). Unit sim: `make sim DESIGN=irq_check REV=rev2`.
System sim (Arlet 6502 runs the real slot ROM against two cards):
`make irqsyssim`.

## Why

U12 to U14 (SN74LVC8T245DBQR) are out of stock; held stock is about 5x
price. Candidate replacement: **SN74LVCH8T245DBQR**, same DBQ24 package and
pinout, plus bus hold on the A/B I/Os.

Checked against `project_byte_hamr.kicad_pcb`: no shifter I/O has a pull
resistor (every resistor belongs to flash config, power feedback, USB or the
33 Ω GPIO series parts), so bus hold has nothing to fight and the LVCH is a
**drop-in**. Small bonus: unused inputs left floating today (U9 A6 to A8,
DIR=5V; U14 B8) are out of spec on plain LVC and held on LVCH.

## Latent issue under test

U14 drives FPGA to Apple on ~INH, ~RES, ~IRQ, ~NMI, ~DMA (plus INT_IN,
DMA_IN). Its **OE and DIR are tied to GND**, so the A side is always
push-pull. The RTL releases these lines (`1'bZ`, nRES OPENDRAIN, PULLMODE=UP),
but a released FPGA pin reads as a high B input, so U14 actively drives the
5V wired-OR line **high**. Any other device pulling the line low fights a
push-pull driver.

Ctrl-Reset always works and the Super Serial Card has never had a problem
(motherboard/SSC drivers probably win, or the software in use never used SSC
IRQ). **Bench 2026-11-03 confirmed it is a real failure for card-to-card:**
one Byte Hamr pulling ~IRQ or ~NMI low against an idle Byte Hamr is never
seen by the 6502 (0/100). Both pulling together works (100/100). See Status.

RTL cannot fix it (OE/DIR not on the FPGA). Possible repairs (cut ~DMA,
Kapton on the fingers, series Schottky per line, OE to a GPIO, interposer)
with steps, and Rev 3 notes: `hardware/byte_hamr/rev2_errata.md`, E1. None
done yet.
- **Rev3:** those five lines on a 5V-tolerant open-drain buffer (e.g.
  74LVC07A on 3.3V). INT_OUT/DMA_OUT stay push-pull (daisy chain is driven).

## What the design does

Same bitstream on **both** cards. The card you command is the tester; the
idle card is the opponent and, via U14, drives ~IRQ/~NMI high the whole time.

The 6502 is the sensor: fabric watches the address bus for vector fetches
($FFFE IRQ, $FFFA NMI) and counts taken, missed and latency.

- PHI0 synced at 100 MHz. Address, R/W, DEVICE SELECT sampled 200 ns into
  PHI0 high; write data at 380 ns.
- 100 shots per run, 2048 PHI0 cycles (~2 ms) apart. Each shot asserts the
  line, releases on the matching vector fetch, or after 64 cycles (miss).
- Scope mode: ~1 µs IRQ pulse every ~10 ms, trigger on GPIO_1.
- GPIO_1 IRQ asserted, GPIO_2 NMI asserted, GPIO_3 vector fetch seen,
  GPIO_4 done.
- INH/DMA at Z, nRES released, daisy chain passed through, SDRAM idle
  (CKE=0, nCS=1).

### Registers ($C080 + slot*$10)

| Offset | Write | Read |
|---|---|---|
| +0 | 00 stop/clear, 01 IRQ test, 02 NMI test, 03 scope | status: b7 done, b6 busy, b1:0 mode |
| +1 | | interrupts taken |
| +2 | | missed |
| +3 | | min latency (PHI0 cycles) |
| +4 | | max latency |
| +5 | | spurious vector fetches (not ours) |
| +6 | | shots fired |
| +7 | | $A5 ID |
| +8 | slot ROM page (bit 0); Ctrl-Reset clears | |

Any command write clears all counters.

### Slot ROM: `PR#n` runs everything

The bitstream carries a 512-byte slot ROM (`irqrom.S`, 2 pages at $Cn00,
page picked by +8). `PR#n` (Applesoft or monitor) runs on card n:

1. Copies itself to $1000-$11FF (so one image works in any slot), sets the
   IRQ handler (`LDA $45 / RTI` at $300, $3FE -> $300) and NMI `RTI` at $3FB.
2. Scans slots 7..1 for the other irq_check card by ROM signature `IRQK`.
3. Single pull: IRQ then NMI on this card, opponent idle (cleared first).
4. Both pull (`+` lines, only with an opponent): the opponent gets the same
   command first, so both cards pull the line low together. This is the
   no-pull-the-card baseline: two U14s low beat nothing.
5. Prints (system sim output, no contention modelled):

```
IRQ_CHECK SLOT 4 OPP 5
IRQ T=64 M=00 L=03-05 S=00 O=64 PASS
NMI T=64 M=00 L=04-07 S=00 O=64 PASS
IRQ+ T=64 M=00 L=05-1F S=64 O=64 PASS
NMI+ T=64 M=00 L=06-08 S=64 O=64 PASS
```

T taken, M missed, L min-max latency (PHI0 cycles), S own spurious. O: on
single lines the opponent's spurious (should equal T: it saw every vector
fetch); on `+` lines the opponent's taken. PASS = T=$64 and M=$00.
`OPP NONE` = no second card found.

Reading it: single FAIL + both-pull PASS = contention (idle card's U14
holds the line high). Both-pull S=00 means the line only fell once both
pulled (contention); S=64 means the first card to pull won alone (no
contention, as in sim). Both FAIL = card output path or test broken.

### Manual alternative (monitor, tester in slot 4)

```
300:A9 01 8D C0 C0 58 AD C0 C0 10 FB 78 60   IRQ test: start, CLI, poll done, SEI, RTS
310:A9 02 8D C0 C0 AD C0 C0 10 FB 60         NMI test: start, poll done, RTS
320:A5 45 40                                 IRQ handler: LDA $45, RTI
3FB:40                                       NMI handler: RTI
3FE:20 03                                    IRQ user vector -> $0320
300G  then  C0C0.C0C7
310G  then  C0C0.C0C7
```

Cold boot, no disk, so nothing else uses IRQ. Don't poke NMI/INH/DMA ad hoc;
they crash or stall the machine.

## Test plan

1. **Contention:** both cards in, `PR#n`. See "Reading it" above.
2. **Swap roles:** `PR#5`.
3. **Baseline:** built in as the `+` lines. Pulling one card and running
   `PR#n` (`OPP NONE`) is the other way.
4. **Cross-check** is built in: O= (opponent saw every vector fetch) must
   equal T=.
5. **Scope:** monitor `C0C0:03`, probe ~IRQ at the slot, trigger GPIO_1. One
   card: ~0 V. Two cards: look for a mid-level (2 to 3 V expected for matched
   LVC drivers). `C0C0:00` stops.
6. **After a repair:** see "Verifying a repair" in
   `hardware/byte_hamr/rev2_errata.md`.

## Status

- yosys + nextpnr-ecp5 (85k, CABGA381, speed 6), base rev2 LPF:
  577 LUT4 (512x8 ROM in LUTs), 119 FF, Fmax 183 MHz (PASS at 100).
- `make irqsyssim` PASS: real ROM on Arlet, two cards (slots 4, 5) behind
  U12 models, `PR#4` then `PR#5`: both print the PASS lines above, opponent
  found and cleared, O= equals T=, no wrong-card bus enable.
- `irq_check_tb.v` PASS: bus model at 1.02 MHz, 6502 interrupt model.
  Proves write capture at 380 ns, ID/status read path, 100/100 taken (IRQ and
  NMI, latency 6 to 8 in the model), 100/100 missed with I set, counter
  clear, foreign-fetch spurious count, scope pulse 980 ns / period 10.0 ms,
  DATA_OE only low inside DEVICE SELECT.
- **Bench 2026-11-03, cards in slots 4 and 2, `PR#4`**
  (`docs/captures/irq_check/2026-11-03_pr4_both_pull_screen.png`):

  ```
  IRQ_CHECK SLOT 4 OPP 2
  IRQ T=00 M=64 L=FF-00 S=00 O=00 FAIL
  NMI T=00 M=64 L=FF-00 S=00 O=00 FAIL
  IRQ+ T=64 M=00 L=06-08 S=00 O=64 PASS
  NMI+ T=64 M=00 L=07-09 S=00 O=64 PASS
  ```

  Single pull: no vector fetch, ever, for IRQ or NMI. Both pull: 100/100
  taken, normal latency, both cards count every fetch, S=00 (the line fell
  only once both pulled). Card output path, write capture, ROM and handler
  all work, so the single FAIL is the idle card's U14 holding the line high.
  **Hypothesis confirmed.** An earlier `PR#2` run (old ROM, single pull
  only) failed the same way with roles swapped.
- Contention pulses are short and spaced out, keeping two fighting LVC
  drivers far under thermal limits. Don't hold a line asserted for long
  against the other card.
