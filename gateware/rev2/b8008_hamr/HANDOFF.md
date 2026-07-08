# b8008_hamr — Intel 8008 Coprocessor Card: Bench Handoff

**What this is:** your b8008 core (cycle-exact 8008, from intel-8008-vhdl @ 2ff4659)
living in a Byte Hamr slot, running the *unmodified* silicon-proven b8008_monitor
firmware. **`PR#4` is the front door**: the card's own slot ROM is the terminal —
no OS, no disk, no FujiNet required. The monitor's UART ports (IN 1 / OUT 9) are
backed by 64-byte FIFOs on `$C0Cx`; keyboard and screen are the wires.

ProDOS appears in exactly one place: the Merlin leg (Merlin 8 needs it). Write
8008 source in Merlin with MAC8008, assemble, then `B8RUN` streams the object
into 8008 RAM through the monitor's own `L` command (Intel HEX over the FIFO;
FIFO backpressure replaces send_hex.py pacing) and issues `G`.

## Artifacts (all built + sim-verified 2026-07-07)

| Thing | Where | Verified how |
|---|---|---|
| Bitstream | `build/b8008_hamr.bit` | clean rebuild; ROM chain byte-verified (monitor fw fresh-assemble == shipped; slot ROM from b8fw.S) |
| Gateware tb | `make DESIGN=b8008_hamr REV=rev2 sim` | test-ROM suite + real-monitor boot (banner + G→HLT) + slot ROM reads |
| Slot ROM terminal | `b8fw.S` → `b8008_slot.mem` (122 bytes) | in-bitstream; PR#4 entry/exit = proven obscurus slot_rom.S pattern |
| Merlin-leg disk | `software/B8008/B8008.po` | tools BIN + MAC8008.S/HELLO8.S/B8LIB.S as TXT; HELLO8 byte-identical to ASL golden |
| MAC8008.S | on disk + `software/B8008/` | 6/6 samples byte-identical vs ASL golden (`make mac8008`) |

## Register map ($C0C0 = slot 4) + slot ROM $C400-$C4FF

| Reg | R | W |
|---|---|---|
| $C0C0 | TX_STAT: b7=avail, b6:0=count | — |
| $C0C1 | TX_DATA (pop; absolute LDA only) | — |
| $C0C2 | RX_STAT: b7=space, b6:0=free | RX push (keyboard byte) |
| $C0C3 | last OUT 31 (checkpoint) | — |
| $C0C4 | checkpoint count | — |
| $C0C5 | CPU_STAT: b7=boot b6=run b5:3=s2s1s0 b2=halt b1=txav b0=rxsp | $52='R' hold reset · $47='G' release |
| $C0C6/7 | live PC lo/hi | — |
| $C0C8 | last OUT 8 (LED port) | — |
| $C0C9 | — | host INT: b2:0 = RST vector (post-boot only) |
| $C0CF | ID = $B8 | — |

## Bench session — exact steps

1. **Flash** (your call, per standing rule):
   `make DESIGN=b8008_hamr REV=rev2 prog-flash`
2. **Power on / Ctrl-Reset.** No disk needed. From the BASIC or monitor prompt:
3. **`PR#4`** — the card takes over: `B8008 TERM ^Q=EXIT ^R=RESET`, then the
   8008's own boot banner: **`8008 Monitor`** and a `> ` prompt. You are typing
   at a 1972 CPU. (The 8008 boots at power-on; its ~330 ms startup delay is
   long gone by the time you type PR#4. If the screen is silent: Ctrl-R
   reboots the 8008 and the banner reprints.)
4. **Drive the monitor**: `H` help · `D 0000` dump monitor ROM (real bytes) ·
   `W 2000,42` + `D 2000` RAM write/readback · `G addr` run. Ctrl-Q returns
   to BASIC; Ctrl-Reset always works.
5. **Merlin leg** (the workflow goal): boot your Merlin 8 disk (its own ProDOS),
   with `software/B8008/B8008.po` available (ADTPro/floppy or however you mount
   volumes for Merlin work). `MAC8008.S`, `HELLO8.S`, `B8LIB.S` are TXT sources.
   - Sanity first: `BRUN B8TEST` (card check + banner drain), then
     `BRUN B8RUN`, filename `HELLO8` → dots, `G 2040`, and
     `HI` / `0123456789 B8008-OK` on screen, live TTY after.
   - Then yours: edit in Merlin 8, assemble (object aux type = ORG, Merlin does
     this), `BRUN B8RUN`, name your file. **That's the ecosystem.**
   - Merlin-8-vs-Merlin32 seam check: HELLO8.S assembled on the //e should be
     457 bytes, identical to the shipped HELLO8 BIN.

## Iteration costs
- 8008 program bug → re-assemble + `B8RUN`. No reflash, no disk rebuild.
- Terminal firmware (b8fw.S) or monitor firmware change → rebuild + flash
  (ecpbram hot-patch flow from the 8008 repo is a candidate to avoid resynth).
- 6502 disk tools → `make b8008disk`, re-send disk. No reflash.

## Known non-obvious facts
- PR#4 entry: slot ROM restores CSW=COUT1 first (else its own COUT re-enters
  $C400); Ctrl-Q RTSes back over PR#'s JSR frame (obscurus exit pattern).
  With no OS loaded, exit via Ctrl-Reset instead of Ctrl-Q.
- Monitor startup delay is ~330 ms of 8008 time (its "50ms" comment undersells).
- Fill discipline: **$00 everywhere** ($00 = HLT → wild jumps freeze
  diagnosably; dodges the documented GHDL/yosys $FF-fill BRAM corruption).
- TX_STAT bit7 is the avail flag — mask to 7 bits for the count.
- 8008new `HLT` assembles as $00 (ASL convention), not $FF.
- MAC8008 renames (6502 collisions): `JMP8 ADC8 ORA8 CMP8`. Two-operand forms
  use semicolons: `MOV A;B`.
- Roadmap (user vision): SCELBAL personality (write BASIC *on* the 8008),
  direct RAM load port, self-hosted 8008 assembler — the fully internal
  ecosystem. Block-device merge (self-booting single card) is a candidate
  later phase if the Merlin leg wants disk service from this card too.
