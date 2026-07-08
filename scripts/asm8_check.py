#!/usr/bin/env python3
"""asm8_check.py — Python reference model of the ASM8 v1 spec.

NOT a product: this is the design-validation gate for the 6502 ASM8.
It implements exactly what ASM8.S implements — same generated table
(build_table from gen_asm8_table), same encoder classes, same lexer,
same expression rules, same two-pass flow, same 16-bit wraparound —
so a design bug surfaces here on the Mac instead of on the bench.
The bench then only tests the 6502 *implementation* of this design.

Deliberate mirrors of 6502 behavior:
  - whole line upcased on read (string contents included)
  - high bit stripped, TAB -> space, CR/LF line ends
  - line > 127 chars = hard error (128-byte line buffer)
  - symbols truncated to 12 chars
  - expressions: flat left-to-right + - & | << >>, parens recurse
  - numbers: decimal, digit-first trailing-h hex, $hex, 'c' char
  - all arithmetic masked to 16 bits

Usage: asm8_check.py <source.asm> <reference.bin>
"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_asm8_table import build_table  # noqa: E402

C_NONE, C_REG8, C_REGS, C_MOV, C_MVI = 0, 1, 2, 3, 4
C_IMM, C_ADDR, C_RST, C_IN, C_OUT = 5, 6, 7, 8, 9
SIZES = {C_NONE: 1, C_REG8: 1, C_REGS: 1, C_MOV: 1, C_MVI: 2,
         C_IMM: 2, C_ADDR: 3, C_RST: 1, C_IN: 1, C_OUT: 1}
REGS = "ABCDEHLM"
DIRECTIVES = {'ORG', 'EQU', 'DB', 'DW', 'DS', 'END', 'CPU', 'PAGE'}
MAXLINE = 127
SYMLEN = 12
OUTMAX = 0x3000  # 12KB output buffer = full 8008 RAM


class AsmError(Exception):
    pass


class Asm8:
    def __init__(self):
        self.table = {name: (cls, base) for name, cls, base in build_table()}
        self.syms = {}
        self.out = bytearray(OUTMAX)

    # --- line reader (mirrors RDLINE) ---
    def read_lines(self, path):
        raw = open(path, 'rb').read()
        line = []
        for b in raw:
            b &= 0x7F                    # strip high bit
            if b == 0x0D or b == 0x0A:   # CR ends line, LF skipped
                if b == 0x0D or line:
                    yield ''.join(line)
                    line = []
                continue
            if b == 0x09:
                b = 0x20                 # TAB -> space
            c = chr(b)
            if 'a' <= c <= 'z':
                c = c.upper()            # upcase whole line
            line.append(c)
            if len(line) > MAXLINE:
                raise AsmError("LINE TOO LONG")
        if line:
            yield ''.join(line)

    # --- cursor helpers (mirror LINEPOS parsing) ---
    def peek(self):
        return self.line[self.pos] if self.pos < len(self.line) else None

    def getch(self):
        c = self.peek()
        if c is not None:
            self.pos += 1
        return c

    def skipsp(self):
        while self.peek() == ' ':
            self.pos += 1

    def at_end(self):
        """end of statement: EOL or comment start."""
        self.skipsp()
        c = self.peek()
        return c is None or c == ';'

    def ident(self):
        c = self.peek()
        if c is None or not (c.isalpha() or c == '_'):
            return None
        s = []
        while (c := self.peek()) is not None and (c.isalnum() or c == '_'):
            s.append(c)
            self.pos += 1
        return ''.join(s)

    # --- expressions: flat left-to-right, parens recurse ---
    def term(self):
        self.skipsp()
        c = self.peek()
        if c is None:
            raise AsmError("MISSING VALUE")
        if c == '(':
            self.pos += 1
            v = self.expr()
            self.skipsp()
            if self.getch() != ')':
                raise AsmError("MISSING )")
            return v
        if c == '$':
            self.pos += 1
            s = []
            while (c := self.peek()) is not None and c in '0123456789ABCDEF':
                s.append(c)
                self.pos += 1
            if not s:
                raise AsmError("BAD HEX")
            return int(''.join(s), 16) & 0xFFFF
        if c == "'":
            self.pos += 1
            ch = self.getch()
            if ch is None or self.getch() != "'":
                raise AsmError("BAD CHAR")
            return ord(ch)
        if c.isdigit():
            s = []
            while (c := self.peek()) is not None and (c.isalnum()):
                s.append(c)
                self.pos += 1
            t = ''.join(s)
            if t[-1] == 'H':                     # 0FFH trailing-h hex
                body = t[:-1]
                if not body or any(x not in '0123456789ABCDEF'
                                   for x in body):
                    raise AsmError("BAD NUMBER")
                return int(body, 16) & 0xFFFF
            if not t.isdigit():
                raise AsmError("BAD NUMBER")
            return int(t, 10) & 0xFFFF
        name = self.ident()
        if name is None:
            raise AsmError("BAD TERM")
        name = name[:SYMLEN]
        if name in self.syms:
            return self.syms[name]
        if self.passno == 1:
            self.undef = True                    # placeholder, size known
            return 0
        raise AsmError(f"UNDEF: {name}")

    def expr(self):
        v = self.term()
        while True:
            self.skipsp()
            c = self.peek()
            if c == '+':
                self.pos += 1
                v = (v + self.term()) & 0xFFFF
            elif c == '-':
                self.pos += 1
                v = (v - self.term()) & 0xFFFF
            elif c == '&':
                self.pos += 1
                v = v & self.term()
            elif c == '|':
                self.pos += 1
                v = v | self.term()
            elif c == '<' and self.line[self.pos:self.pos+2] == '<<':
                self.pos += 2
                n = self.term()
                for _ in range(n & 0xFF):
                    v = (v << 1) & 0xFFFF
            elif c == '>' and self.line[self.pos:self.pos+2] == '>>':
                self.pos += 2
                n = self.term()
                for _ in range(n & 0xFF):
                    v = v >> 1
            else:
                return v

    def need_expr(self):
        """expr that must resolve even in pass 1 (ORG/EQU/DS)."""
        self.undef = False
        v = self.expr()
        if self.undef:
            raise AsmError("FWD REF IN DIRECTIVE")
        return v

    def comma(self):
        self.skipsp()
        if self.peek() == ',':
            self.pos += 1
            return True
        return False

    def reg(self):
        self.skipsp()
        c = self.getch()
        if c is None or c not in REGS:
            raise AsmError("BAD REGISTER")
        nxt = self.peek()
        if nxt is not None and (nxt.isalnum() or nxt == '_'):
            raise AsmError("BAD REGISTER")
        return REGS.index(c)

    # --- emit (mirrors EMIT: pass 2 stores, both passes advance PC) ---
    def emit(self, byte):
        if self.passno == 2:
            if not self.orgset:
                raise AsmError("ORG FIRST")
            off = (self.pc - self.orgbase) & 0xFFFF
            if off >= OUTMAX:
                raise AsmError("OUTPUT OVERFLOW")
            self.out[off] = byte & 0xFF
            if off + 1 > self.outend:
                self.outend = off + 1
        elif not self.orgset:
            raise AsmError("ORG FIRST")
        self.pc = (self.pc + 1) & 0xFFFF

    # --- one statement ---
    def statement(self, line):
        self.line = line
        self.pos = 0
        label = None
        if line and line[0] not in (' ', ';'):
            label = self.ident()
            if label is None:
                raise AsmError("BAD LABEL")
            label = label[:SYMLEN]
            if self.peek() == ':':
                self.pos += 1
        if self.at_end():
            if label is not None:
                self.define(label, self.pc)
            return True
        self.skipsp()
        mnem = self.ident()
        if mnem is None:
            raise AsmError("BAD MNEMONIC")

        if mnem == 'EQU':
            if label is None:
                raise AsmError("EQU NEEDS LABEL")
            self.define(label, self.need_expr())
            return True
        if label is not None:
            self.define(label, self.pc)
        if mnem in ('CPU', 'PAGE'):          # accept-and-ignore
            return True
        if mnem == 'END':
            return False
        if mnem == 'ORG':
            v = self.need_expr()
            if not self.orgset:
                self.orgbase = v
                self.orgset = True
            elif v < self.orgbase:
                raise AsmError("ORG BACKWARD")
            self.pc = v
            return True
        if mnem == 'DS':
            self.pc = (self.pc + self.need_expr()) & 0xFFFF
            return True
        if mnem == 'DB':
            while True:
                self.skipsp()
                if self.peek() == '"':
                    self.pos += 1
                    n = 0
                    while (c := self.getch()) != '"':
                        if c is None:
                            raise AsmError("BAD STRING")
                        self.emit(ord(c))
                        n += 1
                    if n == 0:
                        raise AsmError("EMPTY STRING")
                else:
                    self.emit(self.expr())
                if not self.comma():
                    break
            if not self.at_end():
                raise AsmError("BAD OPERAND")
            return True
        if mnem == 'DW':
            while True:
                v = self.expr()
                self.emit(v & 0xFF)
                self.emit(v >> 8)
                if not self.comma():
                    break
            if not self.at_end():
                raise AsmError("BAD OPERAND")
            return True
        if mnem in DIRECTIVES:
            raise AsmError("DIRECTIVE MISUSE")

        # --- instruction via table ---
        if mnem not in self.table:
            raise AsmError(f"BAD MNEMONIC: {mnem}")
        cls, base = self.table[mnem]
        if cls == C_NONE:
            self.emit(base)
        elif cls == C_REG8:
            r = self.reg()
            if r == 0:
                raise AsmError("INR/DCR A INVALID")  # = HLT encodings
            self.emit(base + r*8)
        elif cls == C_REGS:
            self.emit(base + self.reg())
        elif cls == C_MOV:
            d = self.reg()
            if not self.comma():
                raise AsmError("NEED ,")
            s = self.reg()
            if d == 7 and s == 7:
                raise AsmError("MOV M,M = HLT")
            self.emit(0xC0 + d*8 + s)
        elif cls == C_MVI:
            r = self.reg()
            if not self.comma():
                raise AsmError("NEED ,")
            v = self.expr()
            self.emit(base + r*8)
            self.emit(v & 0xFF)
        elif cls == C_IMM:
            v = self.expr()
            self.emit(base)
            self.emit(v & 0xFF)
        elif cls == C_ADDR:
            v = self.expr()
            self.emit(base)
            self.emit(v & 0xFF)
            self.emit(v >> 8)
        elif cls == C_RST:
            n = self.expr()
            if n > 7:
                raise AsmError("RST 0-7")
            self.emit(base + n*8)
        elif cls == C_IN:
            p = self.expr()
            if p > 7:
                raise AsmError("IN PORT 0-7")
            self.emit(base + p*2)
        elif cls == C_OUT:
            p = self.expr()
            if p < 8 or p > 31:
                raise AsmError("OUT PORT 8-31")
            self.emit(base + p*2)
        if not self.at_end():
            raise AsmError("BAD OPERAND")
        return True

    def define(self, name, val):
        if self.passno == 1:
            if name in self.syms:
                raise AsmError(f"DUP LABEL: {name}")
            self.syms[name] = val
        else:                                # pass 2: phase verify
            if self.syms.get(name) != val:
                raise AsmError(f"PHASE ERR: {name}")

    def run_pass(self, path, passno):
        self.passno = passno
        self.pc = 0
        self.orgset = False
        self.orgbase = 0
        self.lineno = 0
        if passno == 2:
            self.outend = 0
        for line in self.read_lines(path):
            self.lineno += 1
            self.undef = False
            try:
                if not self.statement(line):
                    break
            except AsmError as e:
                raise AsmError(
                    f"LINE {self.lineno}: {e}\n  {line}") from None
        return self.pc

    def assemble(self, path):
        p1 = self.run_pass(path, 1)
        p2 = self.run_pass(path, 2)
        if p1 != p2:
            raise AsmError(f"PHASE ERR: PASS1 PC {p1:04X} != {p2:04X}")
        return self.orgbase, bytes(self.out[:self.outend])


def main():
    src, ref_path = sys.argv[1], sys.argv[2]
    a = Asm8()
    try:
        org, obj = a.assemble(src)
    except AsmError as e:
        print(f"ASM8 MODEL ERROR: {e}")
        sys.exit(1)
    ref = open(ref_path, 'rb').read()
    print(f"model: ORG ${org:04X}, {len(obj)} bytes; ref: {len(ref)} bytes")
    if obj == ref:
        print("PASS: byte-identical to reference")
        return
    n = min(len(obj), len(ref))
    for i in range(n):
        if obj[i] != ref[i]:
            print(f"FAIL: first diff at +${i:04X}: "
                  f"model {obj[i]:02X} ref {ref[i]:02X}")
            sys.exit(1)
    print(f"FAIL: length mismatch ({len(obj)} vs {len(ref)})")
    sys.exit(1)


if __name__ == '__main__':
    main()
