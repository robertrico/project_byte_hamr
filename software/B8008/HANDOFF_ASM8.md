---
project: ASM8 — native 8008 assembler for the Apple //e (6502, ProDOS)
status: not started; this file is the v1 spec + implementation plan
date: 2026-07-07
depends_on:
  - b8008_hamr card working + bench-verified (gateware/rev2/b8008_hamr/HANDOFF.md)
  - B8RUN loader contract: BIN file, aux type = 8008 load address (= ORG)
  - Merlin 8 as the editor (sources are ProDOS TXT; that leg is solved)
  - upstream core repo ~/Development/intel-8008-vhdl (CONSUMER relationship —
    we never modify it; docs/isa.json is the encoding source of truth)
session_context:
  - Today the full ecosystem shipped and bench-verified end to end - user wrote
    ECHO8 in Merlin 8 on the //e, assembled via MAC8008 macros (object proved
    byte-identical to Merlin32 by monitor D-dump), B8RUN-loaded, ran it live.
  - MAC8008 (55 Merlin macros generated from isa.json) works but the dialect is
    goofy - semicolon args (MOV A;B), renames (JMP8/ADC8/ORA8/CMP8). ASM8 exists
    to delete the dialect: TRUE 8008new syntax, repo sources assemble verbatim.
  - MAC8008 stays as the proven bootstrap/fallback; ASM8 replaces only the
    assemble step, nothing else in the pipeline.
---

# ASM8 — v1 Handoff

## Desired v1 workflow (user-specified, verbatim contract)

1. Write code in Merlin (editor only), Save          -> ProDOS TXT source
2. `BRUN ASM8`  -> choose file                       -> BIN written, aux = ORG
3. `BRUN B8RUN` -> choose file                       -> streamed into 8008 RAM
4. `PR#4` -> `G $ORG`                                -> user runs it themselves

ASM8 prints the ORG on success (same courtesy as B8RUN's "LOADED" line).
The aux-type field is the address contract across the whole chain — one ORG
line in the source flows to the G command untouched.

## What ASM8 accepts (v1 syntax)

- ALL 8008new mnemonics with REAL names: MOV A,B / MVI C,41h / JMP / ADC /
  ORA / CMP — commas, no renames, no semicolons.
- Labels col 0, trailing colon optional. `;` comments (full-line + trailing).
- Numbers: decimal, ASL-style `0FFh`, `$FF`, `'c'` char literals.
- Expressions: left-to-right `+` `-` only.
- Directives: ORG, EQU, DB (strings + bytes), DW, DS, END.
  ACCEPT-AND-IGNORE: `cpu`, `page` — so intel-8008-vhdl sources run VERBATIM.
- Case-insensitive everywhere (upcase on read).
- Input tolerant of Merlin-saved TXT: CR line ends, high-bit chars (strip).

## v1 acceptance test (the golden gate)

Fixtures are ALREADY VENDORED in software/B8008/ and shipped by `b8008disk`:
- `HELLO8R.ASM` — hello_8008_ram.asm from the core repo, true ASL syntax,
  byte-for-byte UNMODIFIED (goes to disk as TXT)
- `HELLO8R.REF` — 457-byte reference BIN derived from the ASL golden hex
  (base $2040, $00 gap fill; cross-checked == the Merlin-built HELLO8)
On the //e: ASM8 assembles HELLO8R.ASM, a small
B8CMP tool (also v1 scope, ~80 lines: MLI-read two files, compare, print PASS
or first-diff offset) proves the output byte-identical, then B8RUN + G 2040
runs it. Byte-identical on real hardware = shipped. (Same gate MAC8008 passed
6/6 via Merlin32; ASM8's gate must run ON the //e — 6502 software is only
tested on hardware, per project rule.)

## Implementation plan

All 6502 Merlin source in software/B8008/. ORG $2000, BRUN program, NO zero
page (self-modifying pointers are fine — program runs from RAM).

| # | Component | Notes |
|---|-----------|-------|
| 1 | File I/O | LIFT FROM B8RUN.S: auto-prefix idiom (GET_PREFIX / ON_LINE on $BF30 / SET_PREFIX), GETLN1 $FD6F filename prompt, upcase, MERR error reporting. Output: MLI CREATE (type $06, aux=ORG) + OPEN/WRITE/CLOSE. |
| 2 | Two-pass driver | STREAMING: no whole-source buffer. Read 512-byte chunks, extract CR-terminated lines into an 80-byte line buffer. Pass 1 = symbols only; re-open + re-read for pass 2 = emit. Removes any source-size limit. |
| 3 | Lexer | label / mnemonic / operand / comment split. Fixed grammar, ~150 lines. |
| 4 | Symbol table | Linear list at $7000+: name (8 chars max) + 16-bit value. Hundreds of symbols max at 8008 scale — no hashing. Two-pass resolves forwards. |
| 5 | Mnemonic table | GENERATED: scripts/gen_asm8_table.py reads intel-8008-vhdl/docs/isa.json, emits ASM8TAB.S as DFB data (name, class, base opcode). Same source of truth as the CPU and MAC8008. ~80 entries. |
| 6 | Encoders | FIVE shapes cover the whole ISA: none (RET/RLC/HLT...), reg (INR/ADD... base+reg or base+reg*8), reg-pair (MOV = C0+d*8+s), imm (byte follows), addr (little-endian word follows), port/rst (base+n*2 / base+n*8). Registers A B C D E H L M = 0-7. |
| 7 | Expressions | number/label/char, then left-to-right +/-. ~150 lines. |
| 8 | Output | Emit to buffer at $4000 (12KB max = full 8008 RAM), track ORG offset, write once at end, then SET_FILE_INFO aux=ORG. |
| 9 | Errors | Line number + message + the offending line. Two-pass consistency check (pass1/pass2 PC mismatch = phase error). |

Memory map: program $2000-$3FFF, output buffer $4000-$6FFF, symbols $7000-$8BFF,
MLI IOBUF $8C00 (two needed if both files open — $8C00 + $9000; check HIMEM).

Makefile: `asm8` target (Merlin32, like b8test), add ASM8 + B8CMP + the
verbatim hello source/reference BIN to `b8008disk`. `make copy-dsk` ships it.

## Hard-won gotchas from today's session (do not relearn)

- **Helper routines must preserve X/Y.** B8_RXPUT clobbering X sent a loop
  runaway sweeping all 64K — reads of $C0xx flip soft switches and can arm
  Disk II write mode (a floppy was nearly casualty). ASM8 touches no card regs,
  but the register-discipline lesson is universal.
- **MLI prefix is empty at boot.** BASIC.SYSTEM resolves partial names itself;
  raw MLI returns $40. B8RUN.S has the working auto-prefix code — copy it.
- **ProDOS pathnames: uppercase A-Z/0-9/. only** — upcase user input ($40
  otherwise). $46 = not found, $44 = bad path, $27 = I/O.
- **PRSTR eats A.** Save error codes BEFORE printing message text.
- **Don't assume register/A survival across helpers** — the HEXOUT-returns-A
  bug shipped a malformed hex EOF record for hours.
- **GETLN1 = $FD6F** (no prompt char); buffer $200, X = length, high bits set.
- **Merlin 8 line width: keep ALL generated .S lines <= 40 cols** ("Operand
  too long" otherwise). gen_* scripts already clamp; ASM8TAB.S must too.
- **Merlin source format**: single-space delimited, labels col 0, ASCII.
  TYP $06 / DSK <name> headers for Merlin32-built tools (Merlin 8 ignores
  neither needed on //e — but these are Mac-assembled).
- **Users will `BRUN <8008 bin>`** (it happened): 8008 bytes execute as 6502
  garbage. ASM8's success message should say "RUN WITH: B8RUN <name>".
- Interactive 8008 programs that wait silently for input read as "hung" —
  if ASM8 examples ship, give them a prompt char.

## Out of scope v1 (explicitly)

Macros, INCLUDE/USE, listing output, expressions beyond +/-, local labels,
a custom editor (Merlin's editor is the editor — sources are TXT; any TXT
editor works later, e.g. Apple Writer). Self-hosted 8008 port of ASM8 is the
long-game follow-on: once this design exists in 6502, that becomes a
translation project, not a design project.
