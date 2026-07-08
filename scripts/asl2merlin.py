#!/usr/bin/env python3
"""Translate an ASL 8008new source (b8008 repo style) to Merlin MAC8008 form.

Mechanical dialect transform — validated by assembling the output with
Merlin32 + MAC8008.S and diffing against the ASL golden hex:
  - operands: comma -> semicolon (macro args), 0FFh -> $FF, decimals kept
  - renames: JMP->JMP8, ADC->ADC8, ORA->ORA8, CMP->CMP8
  - (X>>8)&0FFH -> >X ; X&0FFH -> <X   (page-pointer idiom)
  - db -> ASC '...' / DFB parts;  org/equ/ds -> ORG/EQU/DS
  - labels to col 0 (colon stripped), everything uppercased
  - drops cpu/page/end directives; ';' full-line comments -> '*'

Usage: asl2merlin.py <in.asm> <out.S>
"""
import re
import sys

RENAME = {'JMP': 'JMP8', 'ADC': 'ADC8', 'ORA': 'ORA8', 'CMP': 'CMP8'}
TWO_OP = {'MOV', 'MVI'}


def num(tok):
    """ASL number -> Merlin. 0FFh/0xFF -> $FF, decimal stays, chars stay."""
    tok = tok.strip()
    m = re.fullmatch(r'0?([0-9A-Fa-f]+)[Hh]', tok)
    if m:
        return '$' + m.group(1).upper()
    m = re.fullmatch(r'0[Xx]([0-9A-Fa-f]+)', tok)
    if m:
        return '$' + m.group(1).upper()
    return tok


def expr(e):
    """Translate an operand expression."""
    e = e.strip()
    # page-pointer idioms
    m = re.fullmatch(r'\(\s*(\w+)\s*>>\s*8\s*\)\s*&\s*0?FF[Hh]', e)
    if m:
        return '>' + m.group(1).upper()
    m = re.fullmatch(r'(\w+)\s*&\s*0?FF[Hh]', e)
    if m:
        return '<' + m.group(1).upper()
    # tokenwise number conversion, preserving quoted chars
    out, i = [], 0
    for part in re.split(r"('[^']*')", e):
        if part.startswith("'"):
            out.append(part)
        else:
            out.append(re.sub(r'\b0?[0-9A-Fa-f]+[Hh]\b|\b0[Xx][0-9A-Fa-f]+\b',
                              lambda m: num(m.group(0)), part).upper())
    return ''.join(out)


def db_items(rest):
    """Split a db operand list into ASC/DFB lines."""
    items = re.findall(r'"[^"]*"|\'[^\']*\'|[^,]+', rest)
    lines = []
    for it in items:
        it = it.strip()
        if not it:
            continue
        if it.startswith('"') or (it.startswith("'") and len(it) > 3):
            lines.append(f" ASC '{it[1:-1]}'")
        else:
            lines.append(f" DFB {expr(it)}")
    return lines


def main():
    src, dst = sys.argv[1], sys.argv[2]
    out = [("* FROM " + src.split('/')[-1].upper())[:40],
           " USE MAC8008"]
    seen_org = False
    for raw in open(src):
        line = raw.rstrip('\n')
        s = line.strip()
        if not s:
            continue
        if s.startswith(';'):
            # Merlin Pro "Operand too long" on wide lines - clamp comments
            out.append(('* ' + s.lstrip('; ').rstrip())[:40])
            continue
        # strip trailing comment (outside quotes)
        code = re.split(r";(?=(?:[^']*'[^']*')*[^']*$)", line)[0].rstrip()
        if not code.strip():
            continue
        label = ''
        if not code[0].isspace():
            # col-0 word is a label, with or without colon (ASL allows both)
            m = re.match(r'^(\w+):?\s*(.*)$', code.strip())
            label, code = m.group(1).upper(), m.group(2)
        body = code.strip()
        if not body:
            out.append(label)
            continue
        parts = body.split(None, 1)
        op = parts[0].upper()
        rest = parts[1].strip() if len(parts) > 1 else ''

        if op in ('CPU', 'PAGE', 'END'):
            continue
        if op == 'ORG':
            if not seen_org:
                out.append(f"{label or ''} ORG {expr(rest)}" if label
                           else f" ORG {expr(rest)}")
                seen_org = True
            else:
                # mid-file org -> pad ($00 fill) to keep one contiguous object
                out.append(f" DS {expr(rest)}-*")
                if label:
                    out.append(label)
            continue
        if op == 'EQU':
            out.append(f"{label} EQU {expr(rest)}")
            continue
        if op == 'DS':
            out.append(f"{label} DS {expr(rest)}" if label
                       else f" DS {expr(rest)}")
            continue
        if op == 'DB':
            first = True
            for l in db_items(rest):
                out.append((label + l) if (first and label) else l)
                first = False
            continue

        op = RENAME.get(op, op)
        if op in TWO_OP and ',' in rest:
            d, s2 = rest.split(',', 1)
            operand = f"{expr(d)};{expr(s2)}"
        else:
            operand = expr(rest)
        out.append(f"{label} {op} {operand}".rstrip() if label
                   else f" {op} {operand}".rstrip())

    with open(dst, 'w') as f:
        f.write('\n'.join(out) + '\n')
    print(f"wrote {dst}: {len(out)} lines")


if __name__ == '__main__':
    main()
