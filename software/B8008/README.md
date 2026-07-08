# B8008 — On-Apple Development Ecosystem for the Intel 8008 Coprocessor

Complete toolchain for writing, assembling, loading, and running Intel
8008 programs **entirely on the Apple //e**, targeting the `b8008_hamr`
coprocessor card (`gateware/rev2/b8008_hamr/`). No Mac in the loop once
the disk is built.

The 8008 core and its instruction encodings come from the upstream
`~/Development/intel-8008-vhdl` repo (consumer relationship — never
modified here). `docs/isa.json` in that repo is the single source of
truth for the ISA: the CPU, the assembler table, and the validation
model are all generated from or checked against it.

## The tools (all ProDOS BIN, `BRUN` from the shipped disk)

| Tool | What it does |
|------|--------------|
| `ASM8` | Native 8008 assembler. True 8008new syntax (`MOV A,B`, `MVI C,41h`) — no dialect. Reads a ProDOS TXT source, writes a BIN object whose **aux type = ORG**. |
| `B8RUN` | Loader. Reads a BIN object, streams it into 8008 RAM via the monitor's `L` (Intel-hex) command, prints the `G` address. |
| `B8CMP` | Byte-compare two files via MLI; prints `PASS $nnnn BYTES` or the first-diff offset. The acceptance gate tool. |
| `B8TERM` | Glass TTY attached to the 8008 monitor (Ctrl-Q exits, Ctrl-R reboots the 8008). |
| `B8TEST` | Card presence + FIFO self-test. Run first on a fresh flash. |
| `MAC8008.S` | Legacy: 8008 as Merlin macros (`MOV A;B`, `JMP8` renames). Proven, kept as fallback — ASM8 exists to delete this dialect. |

Fixtures: `HELLO8R.ASM` (verbatim upstream ASL source, ASM8's golden
input) and `HELLO8R.REF` (457-byte reference object derived from the
ASL golden hex). `HELLO8.S` is the same program in MAC8008 dialect.

## Build (Mac side)

Requirements: Merlin32 (`~/Development/Merlin32_v1.2`), AppleCommander
jars (paths in the top-level Makefile), python3, and the
`intel-8008-vhdl` repo checked out as a sibling of this project.

```
make asm8tab    # regenerate ASM8TAB.S from isa.json (opcode cross-check)
make asm8       # table + Python model gate + Merlin32 assemble of ASM8.S
make b8cmp      # assemble B8CMP.S
make b8008disk  # everything + bootable B8008.po (tools BIN, sources TXT)
make copy-dsk DSK=software/B8008/B8008.po   # ship to the //e via ADTPro
```

`make asm8` refuses to build if the design gate fails:
`scripts/asm8_check.py` is a Python reference model of the ASM8 spec —
same generated table (imports `build_table()` from
`scripts/gen_asm8_table.py`), same lexer/expression/encoder rules — and
must assemble `HELLO8R.ASM` byte-identical to `HELLO8R.REF` before
Merlin32 runs. The model passed 6/6 upstream sample programs against
their ASL golden hex. Design bugs die on the Mac; only 6502
implementation faithfulness is tested on hardware.

## Workflow on the //e

1. Write 8008 source in an editor, save as ProDOS TXT (see Merlin
   notes below).
2. `BRUN ASM8` → filename → object written, `OK $nnnn BYTES AT $oooo`.
3. `BRUN B8RUN` → object name → streamed into 8008 RAM.
4. `PR#4` then `G <org>` — run it. (Ctrl-Q exits the terminal;
   objects are run with B8RUN, never `BRUN <8008 bin>`.)

### ASM8 v1 syntax

- All 8008new mnemonics, real names, comma operands. Registers
  `A B C D E H L M`.
- Labels col 0 (trailing `:` optional), `;` comments, case-insensitive.
- Numbers: decimal, ASL `0FFh` (digit-first, trailing h), `$FF`, `'c'`.
- Expressions: flat left-to-right `+ - & | << >>` and parens.
- Directives: `ORG EQU DB DW DS END`; `cpu`/`page` accepted and
  ignored so verbatim upstream ASL sources assemble unmodified.
- Errors print `LINE n: MESSAGE` plus the offending line.
- Limits: symbols 12 chars, lines 127 chars, output 12KB (= full 8008
  RAM), sources any length (streamed in 512-byte chunks).

### Merlin usage notes

Merlin Pro is the **editor only** in this workflow — ASM8 does the
assembling. Save sources as TXT; ASM8 tolerates Merlin's CR line ends
and high-bit characters. Column alignment is free-form (one space
between fields is enough). Notes from the bench:

- Editing 6502 `.S` files here (ASM8.S, B8CMP.S, ASM8TAB.S) in Merlin
  works and is part of QA — they follow the project-wide Merlin style
  guide in `CLAUDE.md` (single-space fields, ASCII, <=40 cols).
- Do NOT load `HELLO8R.ASM` into Merlin: it has 81-col lines (that's
  ASM8 *input*, deliberately wider than Merlin's comfort).
- MAC8008 macro assembly inside Merlin still works as the fallback
  path, dialect warts and all (`MOV A;B`, `JMP8`).

The Merlin-as-editor loop works — it shipped ECHO8 and validated all
of ASM8 — but it is a multi-step round trip (save, quit, BRUN ASM8,
BRUN B8RUN, PR#4, G). That friction is the motivation for the next
project in this line: a single all-in-one fullscreen
editor/assembler/runner for the 8008. ASM8's assembler core was built
to be embeddable for exactly that future.

## Memory map (ASM8 at runtime)

| Range | Use |
|-------|-----|
| `$2000-$3FFF` | ASM8 program + buffers (BRUN target) |
| `$4000-$6FFF` | Output buffer (12KB = full 8008 RAM) |
| `$7000-$8BFF` | Symbol table (14 bytes/entry: 12-char name + value) |
| `$8C00` | MLI IOBUF |

No zero page is touched anywhere (self-modifying absolute pointers) —
ProDOS ZP is a minefield; see `CLAUDE.md` and the SDM lessons.

## Validation status

ASM8 v1 SHIPPED 2026-07-08. All bench gates passed on real hardware
the day it was built: B8CMP plumbing, the golden gate (HELLO8R.ASM
assembled on the //e byte-identical to HELLO8R.REF, then run via
B8RUN + `G 2040`), the error paths, and a from-scratch program
(COUNT8) written in Merlin, assembled with ASM8, and run — the
founding "write 8008 code on the Apple" goal, in true 8008 syntax.
