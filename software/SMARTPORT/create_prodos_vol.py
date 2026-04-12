#!/usr/bin/env python3
"""
create_prodos_vol.py — Create a ProDOS volume of arbitrary size.
Writes boot blocks, volume directory, bitmap, and adds PRODOS + BASIC.SYSTEM.
"""

import struct
import sys
import math
from datetime import datetime

def prodos_date(dt=None):
    """Encode a datetime as ProDOS date + time words (little-endian)."""
    if dt is None:
        dt = datetime.now()
    year = dt.year % 100  # ProDOS uses 0-99
    date_word = (year << 9) | (dt.month << 5) | dt.day
    time_word = (dt.hour << 8) | dt.minute
    return struct.pack('<HH', date_word, time_word)

def make_volume(output_path, vol_name, total_blocks, boot_blocks_path,
                files):
    """
    Create a ProDOS volume image.

    files: list of (filename, filetype, aux_type, data_bytes)
    """
    assert len(vol_name) <= 15
    assert total_blocks <= 65535

    img = bytearray(total_blocks * 512)

    # ---- Block 0-1: Boot blocks ----
    with open(boot_blocks_path, 'rb') as f:
        boot = f.read(1024)
    img[0:len(boot)] = boot

    # ---- Bitmap sizing ----
    bitmap_bytes = math.ceil(total_blocks / 8)
    bitmap_blocks = math.ceil(bitmap_bytes / 512)
    bitmap_start = 6  # standard ProDOS location

    # ---- Directory blocks: 2-5 (4 blocks, standard) ----
    dir_blocks = [2, 3, 4, 5]

    # Link directory blocks (prev/next pointers at bytes 0-3)
    for i, blk in enumerate(dir_blocks):
        off = blk * 512
        prev_blk = dir_blocks[i-1] if i > 0 else 0
        next_blk = dir_blocks[i+1] if i < len(dir_blocks)-1 else 0
        struct.pack_into('<HH', img, off, prev_blk, next_blk)

    # ---- Volume directory header (block 2, starting at byte 4) ----
    entry_length = 0x27  # 39 bytes
    entries_per_block = 0x0D  # 13

    hdr_off = 2 * 512 + 4  # block 2, byte 4
    # Storage type $F | name length
    img[hdr_off + 0] = 0xF0 | len(vol_name)
    # Volume name
    for i, c in enumerate(vol_name.upper()):
        img[hdr_off + 1 + i] = ord(c)
    # Reserved: bytes 16-23 = 0 (already zero)
    # Creation date/time
    img[hdr_off + 24:hdr_off + 28] = prodos_date()
    # Version, min version
    img[hdr_off + 28] = 0x00
    img[hdr_off + 29] = 0x00
    # Access
    img[hdr_off + 30] = 0xC3
    # Entry length
    img[hdr_off + 31] = entry_length
    # Entries per block
    img[hdr_off + 32] = entries_per_block
    # File count (fill in after adding files)
    # Bitmap pointer
    struct.pack_into('<H', img, hdr_off + 35, bitmap_start)
    # Total blocks
    struct.pack_into('<H', img, hdr_off + 37, total_blocks)

    # ---- Allocate and write files ----
    # Track allocated blocks: 0-1 boot, 2-5 dir, 6-(6+bitmap_blocks-1) bitmap
    next_free = bitmap_start + bitmap_blocks
    file_count = 0
    entry_idx = 1  # entry 0 is the volume header

    for filename, filetype, aux_type, data in files:
        data_len = len(data)
        data_blocks_needed = math.ceil(data_len / 512) if data_len > 0 else 1

        if data_blocks_needed <= 1:
            # Seedling file: key block IS the data block
            storage_type = 0x1
            key_block = next_free
            blocks_used = 1
            # Write data
            img[key_block * 512:key_block * 512 + data_len] = data
            next_free += 1
        elif data_blocks_needed <= 256:
            # Sapling file: key block is index, data blocks follow
            storage_type = 0x2
            index_block = next_free
            next_free += 1
            key_block = index_block

            # Allocate data blocks
            data_block_list = []
            for i in range(data_blocks_needed):
                data_block_list.append(next_free)
                chunk = data[i*512:(i+1)*512]
                img[next_free * 512:next_free * 512 + len(chunk)] = chunk
                next_free += 1

            # Write index block: low bytes at 0-255, high bytes at 256-511
            for i, blk in enumerate(data_block_list):
                img[index_block * 512 + i] = blk & 0xFF
                img[index_block * 512 + 256 + i] = (blk >> 8) & 0xFF

            blocks_used = 1 + data_blocks_needed  # index + data
        else:
            raise ValueError(f"Tree files not implemented (need {data_blocks_needed} blocks)")

        # Write directory entry
        # Find which directory block and offset for this entry
        dir_entry_num = entry_idx
        block_in_dir = dir_entry_num // entries_per_block
        entry_in_block = dir_entry_num % entries_per_block

        dir_blk = dir_blocks[block_in_dir]
        ent_off = dir_blk * 512 + 4 + entry_in_block * entry_length

        # Storage type | name length
        img[ent_off + 0] = (storage_type << 4) | len(filename)
        # Filename
        for i, c in enumerate(filename.upper()):
            img[ent_off + 1 + i] = ord(c)
        # File type
        img[ent_off + 16] = filetype
        # Key pointer
        struct.pack_into('<H', img, ent_off + 17, key_block)
        # Blocks used
        struct.pack_into('<H', img, ent_off + 19, blocks_used)
        # EOF (3 bytes LE)
        img[ent_off + 21] = data_len & 0xFF
        img[ent_off + 22] = (data_len >> 8) & 0xFF
        img[ent_off + 23] = (data_len >> 16) & 0xFF
        # Creation date/time
        img[ent_off + 24:ent_off + 28] = prodos_date()
        # Version, min version
        img[ent_off + 28] = 0x00
        img[ent_off + 29] = 0x00
        # Access
        img[ent_off + 30] = 0xC3
        # Aux type (load address)
        struct.pack_into('<H', img, ent_off + 31, aux_type)
        # Last modified
        img[ent_off + 33:ent_off + 37] = prodos_date()
        # Header pointer (block containing the directory header)
        struct.pack_into('<H', img, ent_off + 37, dir_blk)

        entry_idx += 1
        file_count += 1
        print(f"  {filename:15s}  type=${filetype:02X}  key={key_block:5d}  "
              f"blocks={blocks_used:4d}  eof={data_len:6d}  "
              f"{'seedling' if storage_type==1 else 'sapling'}")

    # Update file count in volume header
    struct.pack_into('<H', img, hdr_off + 33, file_count)

    # ---- Build bitmap ----
    # Bit=1 = free, Bit=0 = allocated. MSB first within each byte.
    # Mark blocks 0 through (next_free-1) as allocated
    bmp_offset = bitmap_start * 512

    # Start all free
    for byte_idx in range(bitmap_bytes):
        block_base = byte_idx * 8
        val = 0
        for bit in range(8):
            block_num = block_base + (7 - bit)  # MSB first
            if block_num < total_blocks:
                val |= (1 << bit)  # free
        if bmp_offset + byte_idx < len(img):
            img[bmp_offset + byte_idx] = val

    # Mark allocated blocks as used (clear their bits)
    for blk in range(next_free):
        byte_idx = blk // 8
        bit_idx = 7 - (blk % 8)  # MSB first
        img[bmp_offset + byte_idx] &= ~(1 << bit_idx)

    # ---- Write output ----
    with open(output_path, 'wb') as f:
        f.write(img)

    free_blocks = total_blocks - next_free
    print(f"\nVolume /{vol_name}/")
    print(f"  Total blocks:  {total_blocks}")
    print(f"  Used blocks:   {next_free}")
    print(f"  Free blocks:   {free_blocks}")
    print(f"  Free space:    {free_blocks * 512 // 1024}KB ({free_blocks * 512 / (1024*1024):.1f}MB)")
    print(f"  Bitmap blocks: {bitmap_blocks} (blocks {bitmap_start}-{bitmap_start + bitmap_blocks - 1})")
    print(f"  Image size:    {len(img):,} bytes ({len(img) / (1024*1024):.1f}MB)")
    print(f"  Output:        {output_path}")

if __name__ == '__main__':
    total_blocks = 16384  # 8MB
    vol_name = "HAMR"
    output = "images/ProDOS_8MB.po"

    if len(sys.argv) > 1:
        total_blocks = int(sys.argv[1])
    if len(sys.argv) > 2:
        vol_name = sys.argv[2]
    if len(sys.argv) > 3:
        output = sys.argv[3]

    # Load system files
    with open('/tmp/PRODOS.bin', 'rb') as f:
        prodos_data = f.read()
    with open('/tmp/BASIC.SYSTEM.bin', 'rb') as f:
        basic_data = f.read()

    files = [
        ('PRODOS',       0xFF, 0x0000, prodos_data),
        ('BASIC.SYSTEM', 0xFF, 0x2000, basic_data),
    ]

    print(f"Creating {total_blocks}-block ProDOS volume /{vol_name}/")
    make_volume(output, vol_name, total_blocks, '/tmp/boot_blocks.bin', files)
