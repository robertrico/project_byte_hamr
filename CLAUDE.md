# CLAUDE.md

## Working Style
- **Never stop prematurely.** Do not suggest "shipping what works" and deferring bugs to later sessions. Keep debugging until the problem is solved or the user explicitly says to stop. This is a productivity blocker.
- When stuck, try a different approach — don't give up.

## Merlin .S Style Guide (PROJECT-WIDE — all 6502 .S sources)
Every 6502 .S in this repo — hand-written AND generated — must be editable
and loadable in Merlin Pro natively on the //e, even when it is normally
assembled with Merlin32 on the Mac. Merlin32 tolerates violations; Merlin Pro
does not, and the breakage only surfaces on the machine.
Scope: this guide is for 6502 .S. 8008 sources authored/edited in Merlin on
the //e follow the same styling (they live in Merlin's editor and get QA'd
there); verbatim upstream 8008 sources consumed as TXT input (never loaded
in Merlin) are exempt.
- **Single spaces between fields** (`LABEL OPCODE OPERAND COMMENT`) — no tabs,
  no multi-space column alignment
- **Labels at column 0**; unlabeled lines start with exactly ONE leading space
- **ASCII only** — disk import sets the high bit; UTF-8 becomes garbage
- **Lines <= 40 cols** — Merlin Pro throws "OPERAND TOO LONG" (even on `*` banners)
- **Lean**: terse UPPERCASE comments, `*` full-line comments, no banner bars
- **Expressions are left-to-right, NO precedence**: multiply/shift BEFORE add
  (`SLOT*16+$C080` works; `$C080+SLOT*16` overflows)
- Generated sources (gen_* scripts) must conform too — clamp widths in the generator
- Canonical examples: `software/SDM/SDRAMLIB.S`, `software/ASM/*.S`
- Verify: `grep -cP '\t' file.S` == 0, `grep -cP '[^\x00-\x7F]' file.S` == 0,
  and Merlin32 still assembles clean

## Build Commands
- FPGA: `make clean && make DESIGN=block_hamr && make DESIGN=block_hamr prog-flash`
- With disk image: `make DESIGN=block_hamr prog-flash-with-image DISK_IMAGE=path.po`
- Pass extra Verilog defines: `make DESIGN=block_hamr EXTRA_VFLAGS="-DFOO"`
- Push: `gh auth setup-git && git push`

## PCB layout (KiCad 10 + Freerouting)
Rev 2 is frozen at `hardware/byte_hamr/kicad/` (tag `rev2`); Rev 3 lives in
`hardware/byte_hamr/kicad_rev3/`. Spec, BOM, pin map and plan sit beside them
(`rev3_*.md/.csv`).
- **KiCad files change only through Konnect MCP or KiCad itself** (headless
  pcbnew Python, kicad-cli). Never text-edit `.kicad_*`. Exception:
  `project_byte_hamr.kicad_dru` (custom-rules text).
- **Never write while KiCad holds the project**: check for `~*.lck` in the
  project folder first; stop if present.
- kicad-cli hangs scanning the KiCad Documents plugin folder: run it with
  `KICAD_DOCUMENTS_HOME=<empty dir>`.
- **Placement order (Apple II cards)**: transceivers first, staggered in two
  rows over their finger groups, finger groups assigned left to right across
  both rows, channels swapped to finger order; then the FPGA straight above;
  then everything else. FPGA pin maps come from the placement, last.
- **Card envelope (Apple IIe)**: 7.00" max length (slots 1-3), body 2.75",
  3.05" with fingers. Grow only toward the keyboard; rear edge, fingers and
  notches stay as Rev 2. No vias within 9.5 mm of the bottom edge.
- **J1 is generic**: power + plain 3.3 V GPIO. Interface-specific parts
  (SmartPort level shifting, pulls, R/C) and clocks belong on daughterboards.
- **Power from zones**, not pre-drawn tracks excluded from the router: In1 GND,
  In2 +3V3 with split zones (+5V strip, +1V1 island).
- **Freerouting 2.5.0 on JDK 25** (`/Library/Java/JavaVirtualMachines/jdk-25.jdk`;
  the PATH java is 21 and too old). Headless flags that matter:
  `--router.optimizer.enable_preflight_guards=false`
  `--router.job_timeout=HH:MM:SS` (never kill externally: no SES is written)
  `--router.automatic_neckdown=false --router.neck_width_um=150`
  `--router.scoring.via_costs=<n>`; optimizer `improvement_threshold` is a
  percentage (0.005 = 0.5%). Leave fanout off (it escapes every SMD pin).
  Lock pre-routed tracks (only locked tracks export as `fix`; SES import
  deletes unlocked ones). `.kicad_dru` rules are not exported to the router.
  Run 3-5 variants in parallel (own folder, own stdout log, `-mt 2`,
  `-Xmx4g`) and keep the best: 0 unrouted, then fewest vias, then track.
