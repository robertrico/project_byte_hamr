#!/usr/bin/env python3
"""rom2mem.py — Convert Merlin32 ROM binary to Verilog $readmemh format.

Usage: python3 rom2mem.py input.bin output.mem [base_addr] [rom_size]

  base_addr: ORG address of binary (default 0xC400)
  rom_size:  total ROM bytes (default 4096)

Output: hex file with one byte per line. Unused space filled with $FF.
"""
import sys

def rom2mem(bin_path, mem_path, base=0xC400, size=4096):
    with open(bin_path, "rb") as f:
        data = f.read()

    rom = bytearray([0xFF] * size)

    offset = base - 0xC000
    for i, b in enumerate(data):
        if offset + i < size:
            rom[offset + i] = b

    with open(mem_path, "w") as f:
        for b in rom:
            f.write(f"{b:02X}\n")

    print(f"{bin_path} ({len(data)} bytes at ${base:04X}) -> {mem_path} ({size} bytes)")

if __name__ == "__main__":
    bin_path = sys.argv[1]
    mem_path = sys.argv[2]
    base = int(sys.argv[3], 0) if len(sys.argv) > 3 else 0xC400
    size = int(sys.argv[4], 0) if len(sys.argv) > 4 else 4096
    rom2mem(bin_path, mem_path, base, size)
