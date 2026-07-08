#!/usr/bin/env python3
"""Generate ASM8TAB.S — 8008 mnemonic table for the ASM8 native assembler.

Emits the full 8008new mnemonic set as a flat data table consumed by
ASM8.S (PUT-include): 4-char space-padded name (high bit clear), one
class byte, one base-opcode byte, zero-terminated. Real names, commas —
no MAC8008 dialect renames.

Encoder classes (must match ASM8.S ENCODE dispatch and asm8_check.py):
    0 NONE  1 byte : base                    (RET RLC HLT RNZ ...)
    1 REG8  1 byte : base + r*8              (INR DCR; r != A)
    2 REGS  1 byte : base + r                (ADD ADC ... CMP)
    3 MOV   1 byte : $C0 + d*8 + s
    4 MVI   2 bytes: base + r*8, imm
    5 IMM   2 bytes: base, imm               (ADI ACI ... CPI)
    6 ADDR  3 bytes: base, lo, hi            (JMP Jcc CALL Ccc)
    7 RST   1 byte : base + n*8              (n = 0-7)
    8 IN    1 byte : base + p*2              (p = 0-7)
    9 OUT   1 byte : base + p*2              (p = 8-31)

Base opcodes are cross-checked against the bit patterns in the b8008
core's docs/isa.json — table, CPU, and MAC8008 share one source of truth.

Usage: gen_asm8_table.py <isa.json> <ASM8TAB.S>
"""
import json
import sys

C_NONE, C_REG8, C_REGS, C_MOV, C_MVI = 0, 1, 2, 3, 4
C_IMM, C_ADDR, C_RST, C_IN, C_OUT = 5, 6, 7, 8, 9


def pat_base(pattern, subst='0'):
    """Bit pattern like '01CCC000' -> base int with letters as subst."""
    return int(''.join(b if b in '01' else subst for b in pattern), 2)


def crosscheck(isa_path):
    isa = {i['operation']: i['D[7:0]']
           for i in json.load(open(isa_path))['instructions']}

    def check(op, want):
        got = pat_base(isa[op])
        assert got == want, f"{op}: isa.json {got:02x} != expected {want:02x}"

    check('Lr_1r_2', 0xC0)   # MOV d,s  = 11 ddd sss
    check('LrI',     0x06)   # MVI d    = 00 ddd 110
    check('INr',     0x00)   # INR d    = 00 ddd 000
    check('DCr',     0x01)   # DCR d    = 00 ddd 001
    check('ALU OP r', 0x80)  # 10 ppp sss
    check('ALU OP I', 0x04)  # 00 ppp 100
    check('RLC', 0x02); check('RRC', 0x0A)
    check('RAL', 0x12); check('RAR', 0x1A)
    check('JMP', 0x44)       # 01 xxx 100
    check('JFc', 0x40)       # 01 0cc 000
    check('CAL', 0x46)       # 01 xxx 110
    check('RET', 0x07)       # 00 xxx 111
    check('RST', 0x05)       # 00 aaa 101
    check('INP', 0x41)       # 01 00m mm1
    check('OUT', 0x41)       # 01 rrmmm1


def build_table():
    """(name, class, base) triples — the single table definition."""
    t = []
    t.append(('MOV', C_MOV, 0xC0))
    t.append(('MVI', C_MVI, 0x06))
    t.append(('INR', C_REG8, 0x00))
    t.append(('DCR', C_REG8, 0x01))
    # ALU register form 10 ppp sss
    for i, n in enumerate(['ADD', 'ADC', 'SUB', 'SBB',
                           'ANA', 'XRA', 'ORA', 'CMP']):
        t.append((n, C_REGS, 0x80 + i*8))
    # ALU immediate form 00 ppp 100
    for i, n in enumerate(['ADI', 'ACI', 'SUI', 'SBI',
                           'ANI', 'XRI', 'ORI', 'CPI']):
        t.append((n, C_IMM, 0x04 + i*8))
    # rotates
    for n, op in [('RLC', 0x02), ('RRC', 0x0A),
                  ('RAL', 0x12), ('RAR', 0x1A)]:
        t.append((n, C_NONE, op))
    # jumps: 01 t cc 000; cc: 0=carry 1=zero 2=sign 3=parity
    t.append(('JMP', C_ADDR, 0x44))
    for i, n in enumerate(['JNC', 'JNZ', 'JP', 'JPO']):
        t.append((n, C_ADDR, 0x40 + i*8))
    for i, n in enumerate(['JC', 'JZ', 'JM', 'JPE']):
        t.append((n, C_ADDR, 0x60 + i*8))
    # calls: 01 t cc 010
    t.append(('CALL', C_ADDR, 0x46))
    for i, n in enumerate(['CNC', 'CNZ', 'CP', 'CPO']):
        t.append((n, C_ADDR, 0x42 + i*8))
    for i, n in enumerate(['CC', 'CZ', 'CM', 'CPE']):
        t.append((n, C_ADDR, 0x62 + i*8))
    # returns: 00 t cc 011
    t.append(('RET', C_NONE, 0x07))
    for i, n in enumerate(['RNC', 'RNZ', 'RP', 'RPO']):
        t.append((n, C_NONE, 0x03 + i*8))
    for i, n in enumerate(['RC', 'RZ', 'RM', 'RPE']):
        t.append((n, C_NONE, 0x23 + i*8))
    # misc
    t.append(('RST', C_RST, 0x05))
    t.append(('IN', C_IN, 0x41))
    t.append(('OUT', C_OUT, 0x41))
    t.append(('HLT', C_NONE, 0x00))  # ASL emits 00 (00/01/FF all HLT)
    return t


def main():
    isa_path, out_path = sys.argv[1], sys.argv[2]
    crosscheck(isa_path)
    print("isa.json cross-check: all base opcodes agree")

    table = build_table()
    L = []
    o = L.append
    # Merlin Pro chokes on long lines ("Operand too long") -
    # keep ALL emitted lines under 40 cols
    o("* ASM8TAB - 8008 MNEMONIC TABLE")
    o("* GENERATED - DO NOT EDIT")
    o("* ENTRY: 4CH NAME,CLASS,BASE")
    o("* CLASS 0 NONE 1 R*8 2 R 3 MOV")
    o("* 4 MVI 5 IMM 6 ADDR 7 RST")
    o("* 8 IN 9 OUT")
    o("ASM8TAB")
    for name, cls, base in table:
        o(f" ASC '{name:<4}'")
        o(f" DFB {cls},${base:02X}")
    o(" DFB 0 ; TABLE END")

    for line in L:
        assert len(line) <= 40, f"line too long: {line!r}"
        assert all(ord(c) < 128 for c in line), f"non-ASCII: {line!r}"
        assert '\t' not in line, f"tab: {line!r}"

    with open(out_path, 'w') as f:
        f.write('\n'.join(L) + '\n')
    print(f"wrote {out_path}: {len(table)} mnemonics, {len(L)} lines")


if __name__ == '__main__':
    main()
