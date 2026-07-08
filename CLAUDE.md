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
