#!/usr/bin/env python3
"""Convert .dsk (DOS 3.3 sector order) to .po (ProDOS block order).

Uses the same interleave table as FujiNet's mediaTypeDO.cpp.
Verified: perfect match against known-good ProDOS_2_4_1.po.
"""
import sys

# ProDOS block (0-7 within track) → two DOS logical sector numbers
PRODOS2DOS = [
    (0, 14), (13, 12), (11, 10), (9, 8),
    (7, 6),  (5, 4),   (3, 2),   (1, 15),
]

def dsk2po(dsk_path, po_path):
    with open(dsk_path, 'rb') as f:
        dsk = f.read()
    if len(dsk) != 143360:
        print(f"Error: expected 143360 bytes, got {len(dsk)}", file=sys.stderr)
        sys.exit(1)

    po = bytearray(143360)
    for track in range(35):
        for block in range(8):
            dos_s0, dos_s1 = PRODOS2DOS[block]
            src0 = (track * 16 + dos_s0) * 256
            src1 = (track * 16 + dos_s1) * 256
            dst = (track * 8 + block) * 512
            po[dst:dst+256] = dsk[src0:src0+256]
            po[dst+256:dst+512] = dsk[src1:src1+256]

    with open(po_path, 'wb') as f:
        f.write(po)
    print(f"{dsk_path} -> {po_path}")

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} input.dsk output.po")
        sys.exit(1)
    dsk2po(sys.argv[1], sys.argv[2])
