#!/usr/bin/env python3
"""bin2mem.py — Convert a raw binary to Verilog $readmemh format.

Usage: python3 bin2mem.py input.bin output.mem [word_size] [mem_size]

  word_size: bytes per word (default 4 for 32-bit PicoRV32)
  mem_size:  total memory bytes (default 32768 for 32KB IMEM)

Output: hex file with one word per line, suitable for $readmemh.
Words are little-endian (matching RISC-V byte order).
Unused memory is filled with 0x00.
"""

import sys
import struct


def bin2mem(bin_path, mem_path, word_bytes=4, mem_bytes=32768):
    with open(bin_path, "rb") as f:
        data = f.read()

    if len(data) > mem_bytes:
        print(f"ERROR: binary ({len(data)} bytes) exceeds memory ({mem_bytes} bytes)")
        sys.exit(1)

    # Pad to full memory size
    data = data + b'\x00' * (mem_bytes - len(data))

    num_words = mem_bytes // word_bytes

    with open(mem_path, "w") as f:
        for i in range(num_words):
            offset = i * word_bytes
            if word_bytes == 4:
                # 32-bit word, little-endian -> hex
                word = struct.unpack_from("<I", data, offset)[0]
                f.write(f"{word:08X}\n")
            elif word_bytes == 1:
                # Byte-addressable (for 8-bit BRAMs)
                f.write(f"{data[offset]:02X}\n")
            else:
                # Generic: little-endian word
                word = int.from_bytes(data[offset:offset + word_bytes], "little")
                f.write(f"{word:0{word_bytes * 2}X}\n")

    print(f"{bin_path} -> {mem_path}: {len(data)} bytes, "
          f"{num_words} x {word_bytes * 8}-bit words")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    bin_path = sys.argv[1]
    mem_path = sys.argv[2]
    word_bytes = int(sys.argv[3]) if len(sys.argv) > 3 else 4
    mem_bytes = int(sys.argv[4]) if len(sys.argv) > 4 else 32768

    bin2mem(bin_path, mem_path, word_bytes, mem_bytes)
