---
project: ASM8 — native 8008 assembler for the Apple //e (6502, ProDOS)
status: SHIPPED 2026-07-08 — ALL bench stages PASSED on the //e the
  same day it was built: B8CMP plumbing, GOLDEN GATE (HELLO8R.ASM
  assembled byte-identical to the ASL-derived reference + ran via
  B8RUN/G 2040), error paths, style contract, AND stage D: user wrote
  COUNT8 from scratch in Merlin in true 8008 syntax, ASM8'd it, ran it.
  The MAC8008 dialect is now officially deletable (kept as fallback).
  ASM8.S + B8CMP.S + generated ASM8TAB.S assemble clean (Merlin32), all
  style-checked (no tabs, ASCII, <=40 cols), shipped on B8008.po with
  sources as TXT. scripts/asm8_check.py (Python reference model of THIS
  spec, same generated table) assembles all 6 upstream samples
  byte-identical to ASL golden hex — design is proven; the bench tests
  only the 6502 implementation. See "Bench checklist" below.
date: 2026-07-08
depends_on:
  - b8008_hamr card working + bench-verified 
  - B8RUN loader contract: BIN file, aux type = 8008 load address (= ORG)
  - Merlin Pro as the editor (sources are ProDOS TXT; that leg is solved)
  - upstream core repo ~/Development/intel-8008-vhdl (CONSUMER relationship —
    we never modify it; docs/isa.json is the encoding source of truth)
session_context:
  - Today the full ecosystem shipped and bench-verified end to end - user wrote
    ECHO8 in Merlin Pro on the //e, assembled via MAC8008 macros (object proved
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
- Numbers: decimal, ASL-style `0FFh` (digit-first, trailing h/H), `$FF`,
  `'c'` char literals.
- Expressions: left-to-right (NO precedence) `+ - & | << >>` plus parens.
  NOTE: the spec originally said +/- only, but the golden fixture itself
  uses `(MSG>>8)&0FFH` — the fixture is the bar, so shifts/AND/parens are
  v1-mandatory. Left-to-right keeps it one flat loop + recursion on `(`.
- Directives: ORG, EQU, DB (strings + bytes), DW, DS, END.
  ACCEPT-AND-IGNORE: `cpu`, `page` — so intel-8008-vhdl sources run VERBATIM.
- Case-insensitive everywhere (upcase on read).
- Input tolerant of Merlin-saved TXT: CR line ends, high-bit chars (strip).

## v1 acceptance test (the golden gate)

Fixtures are ALREADY VENDORED in software/B8008/ and shipped by `b8008disk`:
- `HELLO8R.ASM` — hello_8008_ram.asm from the core repo, true ASL syntax,
  byte-for-byte UNMODIFIED (goes to disk as TXT). NOTE: lines up to 81 cols —
  this is ASM8 INPUT, never load it in Merlin (the <=40-col rule is for files
  Merlin loads; ASM8 must digest real-world-wide ASL sources — that IS the bar)
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

**Style contract: EVERY .S file — ASM8.S, B8CMP.S, AND the GENERATED
ASM8TAB.S — must be editable and loadable in Merlin Pro natively on the //e.**
Follow the project-wide Merlin .S Style Guide in CLAUDE.md: single-space
fields, labels col 0, one leading space unlabeled, ASCII-only, lines <= 40
cols, left-to-right expressions (multiply before add). Merlin32 on the Mac
still builds these, but Merlin32 tolerating a violation is NOT a pass —
Merlin-Pro-loadable is the bar.

The guide governs 6502 .S. 8008 sources the user writes/edits in Merlin on
the //e follow the same styling too (they live in Merlin's editor — that is
how ASM8 gets QA'd). But ASM8 the TOOL must still digest wide verbatim
upstream ASL sources delivered as TXT: HELLO8R.ASM (81 cols) is exempt
because it is ASM8 *input*, never loaded in Merlin.

| # | Component | Notes |
|---|-----------|-------|
| 1 | File I/O | LIFT FROM B8RUN.S: auto-prefix idiom (GET_PREFIX / ON_LINE on $BF30 / SET_PREFIX), GETLN1 $FD6F filename prompt, upcase, MERR error reporting. Output: MLI CREATE (type $06, aux=ORG) + OPEN/WRITE/CLOSE. |
| 2 | Two-pass driver | STREAMING: no whole-source buffer. Read 512-byte chunks, extract CR-terminated lines into a 128-byte line buffer (fixture reaches 81 cols; overflow = line-too-long error, never truncate silently). Pass 1 = symbols only; re-open + re-read for pass 2 = emit. Removes any source-size limit. |
| 3 | Lexer | label / mnemonic / operand / comment split. Fixed grammar, ~150 lines. |
| 4 | Symbol table | Linear list at $7000+: name (12 chars max — fixture has DIGIT_LOOP = 10; the old 8-char spec would silently truncate) + 16-bit value. Hundreds of symbols max at 8008 scale — no hashing. Two-pass resolves forwards. |
| 5 | Mnemonic table | GENERATED: scripts/gen_asm8_table.py reads intel-8008-vhdl/docs/isa.json, emits ASM8TAB.S as DFB data (name, class, base opcode). Same source of truth as the CPU and MAC8008. ~80 entries. |
| 6 | Encoders | FIVE shapes cover the whole ISA: none (RET/RLC/HLT...), reg (INR/ADD... base+reg or base+reg*8), reg-pair (MOV = C0+d*8+s), imm (byte follows), addr (little-endian word follows), port/rst (base+n*2 / base+n*8). Registers A B C D E H L M = 0-7. |
| 7 | Expressions | number/label/char/paren-group, then flat left-to-right + - & \| << >> (fixture needs shifts/AND/parens). ~250 lines. |
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
- **Merlin Pro line width: ALL .S lines <= 40 cols** — hand-written and
  generated ("Operand too long" otherwise). gen_* scripts already clamp;
  gen_asm8_table.py must clamp ASM8TAB.S too.
- **Merlin source format is PROJECT-WIDE** — see the Merlin .S Style Guide
  in CLAUDE.md. Single-space delimited, labels col 0, one leading space
  unlabeled, ASCII-only, left-to-right expressions. Every ASM8 .S must load
  clean in Merlin Pro on the //e; "it's Mac-assembled" is not an exemption.
- **Users will `BRUN <8008 bin>`** (it happened): 8008 bytes execute as 6502
  garbage. ASM8's success message should say "RUN WITH: B8RUN <name>".
- Interactive 8008 programs that wait silently for input read as "hung" —
  if ASM8 examples ship, give them a prompt char.

## Build + validation chain (as built 2026-07-08)

- `make asm8` = gen table from isa.json + run asm8_check.py gate + Merlin32
  assemble (TYP/DSK wrapper like hello8 — ASM8.S itself stays //e-clean).
- `make b8cmp`, `make b8008disk` (ships everything incl. sources as TXT).
- `scripts/gen_asm8_table.py` exports build_table(); `asm8_check.py`
  imports it so model and 6502 share the byte-identical table. Model
  passed 6/6 upstream samples vs ASL golden hex (same gate as MAC8008).
- Implementation notes vs the plan: DIRTAB (8 directives, classes 10-17)
  sits directly before `PUT ASM8TAB` — one contiguous 6-byte-entry scan.
  Expressions: + - & | << >> with parens (fixture-mandated). Symbols 12
  chars. Errors print "LINE n: msg" + the offending line, then abort via
  saved-stack reset. Object written via DESTROY(ignore $46)+CREATE with
  aux=ORG (no SET_FILE_INFO needed). Self-mod loops use GLOBAL labels
  only — real Merlin scopes locals between globals; do not copy B8RUN's
  local-across-SRC pattern into new code.

## Bench checklist (summary — full steps at the end of this doc)

Golden gate in one breath: B8CMP sanity -> ASM8 HELLO8R.ASM ->
B8CMP vs HELLO8R.REF PASS -> B8RUN + G 2040 runs. See
"Full bench validation" below for the incremental procedure with
expected output at every step, plus COUNT8, the first from-scratch
program to write on the machine.

## Out of scope v1 (explicitly)

Macros, INCLUDE/USE, listing output, expressions beyond +/-, local labels,
a custom editor (Merlin's editor is the editor — sources are TXT; any TXT
editor works later, e.g. Apple Writer). Self-hosted 8008 port of ASM8 is the
long-game follow-on: once this design exists in 6502, that becomes a
translation project, not a design project.

## Full bench validation (2026-07-08 build)

Ordered so each stage only trusts what the previous stage proved.
If a stage fails, the bug is in what that stage introduced — nothing
later needs looking at. All commands at the BASIC prompt on the //e
unless noted.

### Stage A — B8CMP alone (proves the MLI plumbing, no ASM8 yet)

A1. `make copy-dsk DSK=software/B8008/B8008.po` on the Mac, boot it.
    `CAT` shows: ASM8, B8CMP (BIN) + ASM8.S, ASM8TAB.S, B8CMP.S,
    HELLO8R.ASM (TXT) + HELLO8R.REF (BIN A=$2040) alongside the
    proven B8TEST/B8TERM/B8RUN/HELLO8 set.

A2. Known-equal compare: `BRUN B8CMP`, FILE 1: `HELLO8R.REF`,
    FILE 2: `HELLO8`.
    EXPECT: `PASS $01C9 BYTES`
    (Proves: auto-prefix, both IOBUFs, chunked reads, compare loop,
    EOF handling. $01C9 = 457.)

A3. Known-different compare: `BRUN B8CMP`, `B8TEST` vs `B8TERM`.
    EXPECT: `DIFF AT $xxxx F1=$aa F2=$bb` (any offset — the point is
    a sane diff report, not a specific value).

A4. Error path: `BRUN B8CMP`, FILE 1: `NOSUCH`.
    EXPECT: `MLI ERR $46` (file not found). $44/$45 here means the
    prefix idiom is off — stop and debug that before touching ASM8.

### Stage B — the golden gate (ASM8 on the verbatim fixture)

B1. `BRUN ASM8`, source: `HELLO8R.ASM`.
    EXPECT, in order:
      `OBJ: HELLO8R`
      `OK $01C9 BYTES AT $2040`
      `RUN WITH: B8RUN HELLO8R`
    Any `LINE n: <msg>` + echoed line instead = parser/encoder bug;
    the line number tells you which construct. (The design is model-
    proven, so suspect the 6502 transcription, not the spec.)

B2. `CAT` — HELLO8R is BIN, A=$2040 (aux carried the ORG).

B3. THE GATE: `BRUN B8CMP`, `HELLO8R` vs `HELLO8R.REF`.
    EXPECT: `PASS $01C9 BYTES` — byte-identical to the reference
    derived from the ASL golden hex. A DIFF here: the offset is the
    first wrong byte; look it up in HELLO8R against the model output
    (`python3 scripts/asm8_check.py` on the Mac prints the same).

B4. Prove it runs: `BRUN B8RUN`, file `HELLO8R`, then `PR#4`,
    `G 2040`.
    EXPECT: `HI` / `0123456789 B8008-OK`, then back to the monitor.
    Byte-identical AND runs = v1 SHIPPED.

B5. Re-assemble HELLO8R.ASM a second time (object already exists).
    EXPECT: same `OK` output — proves the DESTROY+CREATE rewrite
    path, not just first-write.

### Stage C — error paths (five minutes, catches the ugly half)

Make a throwaway TXT in Merlin (three lines, any name, e.g. BAD8):
line 1 ` org 2000h`, line 2 ` FROB A`, line 3 ` end`.

C1. `BRUN ASM8` on it.
    EXPECT: `LINE 2: BAD MNEMONIC` + the offending line echoed,
    clean return to BASIC (stack-reset abort path).

C2. Edit line 2 to ` JMP NOPLACE`.
    EXPECT: `LINE 2: UNDEFINED SYMBOL` (surfaces in pass 2).

C3. Edit line 2 to ` MVI A,'X'` and DELETE line 1 (no org).
    EXPECT: `LINE 1: CODE BEFORE ORG` (line number = first emit).

C4. Style contract: load ASM8.S into Merlin's editor.
    EXPECT: loads and lists clean — no scrambled columns, no
    OPERAND TOO LONG on any line. Quit without saving.

### Stage D — COUNT8: first program written ON the machine

The point of the whole exercise: write true 8008 syntax in Merlin,
no MAC8008 dialect, and run it. Type this in Merlin, save as TXT
named `COUNT8.ASM` (same save-as-TXT leg ECHO8 already proved).
Column alignment is free-form — ASM8 only needs one space between
fields, so type it flat if you like:

No `cpu 8008new` needed: that directive tells the multi-target AS
assembler which CPU to emit for. ASM8 only speaks 8008 — `cpu`/`page`
are accept-and-ignored solely so verbatim upstream sources assemble.
Native ASM8 sources skip them (`end` is optional too; EOF ends it).

```
; COUNT8 - FIRST ASM8 PROGRAM
OUTP equ 09H
 org 2000h
START:
 MVI C,'0'
LOOP:
 MOV A,C
 CALL ECHO
 INR C
 MOV A,C
 CPI '9'+1
 JNZ LOOP
 MVI A,0DH
 CALL ECHO
 MVI A,0AH
 CALL ECHO
 JMP 0
ECHO:
 ANI 7FH
 OUT OUTP
 MVI B,14H
DLY:
 DCR B
 JNZ DLY
 RET
 end
```

Exercises: equ, org, char literals, '9'+1 expression, forward AND
backward jump targets, CALL/RET, INR/DCR, ANI/CPI immediates, OUT
via a symbol, and the ECHO pacing idiom from HELLO8R.

D1. `BRUN ASM8`, source `COUNT8.ASM`.
    EXPECT: `OBJ: COUNT8` / `OK $0024 BYTES AT $2000` /
    `RUN WITH: B8RUN COUNT8` (36 bytes — model-verified, first
    bytes 16 30 C2 46 1A 20 10 C2 3C 3A 48 02 20 ...).

D2. `BRUN B8RUN`, file `COUNT8`, then `PR#4`, `G 2000`.
    EXPECT: `0123456789` + CR/LF, then the monitor banner (JMP 0
    reboots the monitor — that's the clean exit, not a crash).

D3. Victory lap: change `'0'` to `'A'` and `'9'+1` to `'Z'+1` in
    Merlin, re-assemble, re-run. EXPECT: the alphabet. If that
    round-trip feels instant, the tool is doing its job — that IS
    the on-Apple dev loop ASM8 exists for.
