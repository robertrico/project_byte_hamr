#!/usr/bin/env python3
"""Generate MAC8008.S — Intel 8008 assembly as a Merlin Pro macro library.

Emits the 8008new (1975 Intel / 8080-style) mnemonic set as Merlin macros:
flat byte-emitters only (no internal labels — Merlin restriction), operands
first in every DFB expression (Merlin evaluates strictly left-to-right).

Renames forced by 6502/65C02 opcode collisions (real opcodes beat macros):
    JMP -> JMP8,  ADC -> ADC8,  ORA -> ORA8,  CMP -> CMP8

Base opcode values are cross-checked against the bit patterns in the b8008
core's docs/isa.json, so the macro library and the CPU share one source of
truth. Validation: translated samples assemble byte-identical to the ASL
golden hex (see software/B8008/).

Usage: gen_mac8008.py <isa.json> <MAC8008.S>
"""
import json
import sys


def pat_base(pattern, subst='0'):
    """Bit pattern like '01CCC000' -> base int with letters as subst."""
    return int(''.join(b if b in '01' else subst for b in pattern), 2)


def main():
    isa_path, out_path = sys.argv[1], sys.argv[2]
    isa = {i['operation']: i['D[7:0]']
           for i in json.load(open(isa_path))['instructions']}

    # --- cross-check derived bases against isa.json patterns ---
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
    check('JMP', 0x44)       # 01 xxx 100 (x=0, matches ASL)
    check('JFc', 0x40)       # 01 0cc 000
    check('CAL', 0x46)       # 01 xxx 110
    check('RET', 0x07)       # 00 xxx 111
    check('RST', 0x05)       # 00 aaa 101
    check('INP', 0x41)       # 01 00m mm1
    check('OUT', 0x41)       # 01 rrmmm1
    print("isa.json cross-check: all base opcodes agree")

    L = []
    o = L.append
    # Merlin Pro chokes on long lines ("Operand too long") -
    # keep ALL emitted lines, comments included, under ~40 cols
    o("* MAC8008 - 8008 AS MERLIN MACROS")
    o("* GENERATED - DO NOT EDIT")
    o("* RENAMED: JMP8 ADC8 ORA8 CMP8")
    o("* TWO-OP USES ; LIKE: MOV A;B")
    o("* NO INR A / DCR A (=HLT)")
    o("* REGISTER CODES")
    for i, r in enumerate("ABCDEHLM"):
        o(f"{r} EQU {i}")

    def mac(name, *body):
        o(f"{name} MAC")
        for line in body:
            o(f" {line}")
        o(" <<<")

    # -- data movement --
    mac("MOV", "DFB ]1*8+$C0+]2")          # MOV d;s  (d,s in A..M)
    mac("MVI", "DFB ]1*8+$06", "DFB ]2")   # MVI d;imm
    mac("INR", "DFB ]1*8")                 # d != A
    mac("DCR", "DFB ]1*8+$01")             # d != A

    # -- ALU: register/memory form 10 ppp sss --
    alu = [("ADD", 0), ("ADC8", 1), ("SUB", 2), ("SBB", 3),
           ("ANA", 4), ("XRA", 5), ("ORA8", 6), ("CMP8", 7)]
    for name, p in alu:
        mac(name, f"DFB ]1+${0x80 + p*8:02X}")
    # -- ALU: immediate form 00 ppp 100 --
    alui = [("ADI", 0), ("ACI", 1), ("SUI", 2), ("SBI", 3),
            ("ANI", 4), ("XRI", 5), ("ORI", 6), ("CPI", 7)]
    for name, p in alui:
        mac(name, f"DFB ${0x04 + p*8:02X}", "DFB ]1")

    # -- rotates --
    for name, op in [("RLC", 0x02), ("RRC", 0x0A), ("RAL", 0x12), ("RAR", 0x1A)]:
        mac(name, f"DFB ${op:02X}")

    # -- jumps: 01 t cc 000 (t=1 true), cc: 0=carry 1=zero 2=sign 3=parity --
    mac("JMP8", "DFB $44", "DA ]1")
    jumps = [("JNC", 0x40), ("JNZ", 0x48), ("JP", 0x50), ("JPO", 0x58),
             ("JC", 0x60), ("JZ", 0x68), ("JM", 0x70), ("JPE", 0x78)]
    for name, op in jumps:
        mac(name, f"DFB ${op:02X}", "DA ]1")

    # -- calls: 01 t cc 010 --
    mac("CALL", "DFB $46", "DA ]1")
    calls = [("CNC", 0x42), ("CNZ", 0x4A), ("CP", 0x52), ("CPO", 0x5A),
             ("CC", 0x62), ("CZ", 0x6A), ("CM", 0x72), ("CPE", 0x7A)]
    for name, op in calls:
        mac(name, f"DFB ${op:02X}", "DA ]1")

    # -- returns: 00 t cc 011 --
    mac("RET", "DFB $07")
    rets = [("RNC", 0x03), ("RNZ", 0x0B), ("RP", 0x13), ("RPO", 0x1B),
            ("RC", 0x23), ("RZ", 0x2B), ("RM", 0x33), ("RPE", 0x3B)]
    for name, op in rets:
        mac(name, f"DFB ${op:02X}")

    # -- misc --
    mac("RST", "DFB ]1*8+$05")             # n = 0-7
    mac("IN",  "DFB ]1*2+$41")             # port 0-7
    mac("OUT", "DFB ]1*2+$41")             # port 8-31
    mac("HLT", "DFB $00")   # ASL emits 00 (both 00/FF are HLT); matches fill byte

    with open(out_path, 'w') as f:
        f.write('\n'.join(L) + '\n')
    n_mac = sum(1 for l in L if l.endswith(' MAC'))
    print(f"wrote {out_path}: {n_mac} macros, {len(L)} lines")


if __name__ == '__main__':
    main()
