#!/usr/bin/env python3
"""
assemble_picker.py — Byte Hamr SD Card Picker (65C02)

ProDOS SYS file loaded at $2000. Reads catalog via magic block $FFFF.
After mount, reboots via JMP $Cs00 (slot ROM boot vector).

Catalog format in block buffer (512 bytes):
  Header (32 bytes): [0] entry_count, [1] page, [2] total_pages
  Entry  (32 bytes): [0-7] name, [8-10] ext, [11] flags, [12-13] blocks LE,
                     [14-17] fsize LE
"""

import os
import sys

BASE_ADDR = 0x2000
MAX_SIZE  = 0x2000   # 8KB max

# Slot (hardcoded for now — Phase 9+ adds discovery)
SLOT = 4
IO   = 0xC080 + SLOT * 0x10   # $C0C0
SLOT_ROM = 0xC000 + SLOT * 0x100  # $C400

R_CMD     = IO + 0     # W: $01=READ, $02=WRITE
R_STATUS  = IO + 0     # R: {ready, error, 5'b0, boot_done}
R_DATA    = IO + 1     # R: data_read (auto-increment)
R_BLKLO   = IO + 2     # W: block_num low
R_BLKHI   = IO + 3     # W: block_num high
R_SD_STAT = IO + 8     # R: {sd_ready,sd_error,s4d2_mounted,s4d2_loading,...}
R_SD_CMD  = IO + 8     # W: $01=mount, $02=sd_init
R_SELECT  = IO + 9     # W: img_select (0-based)

# Monitor ROM
HOME   = 0xFC58
COUT   = 0xFDED
CROUT  = 0xFD8E
RDKEY  = 0xFD0C
PRBYTE = 0xFDDA

# ProDOS MLI
MLI  = 0xBF00
QUIT = 0x65

# Zero page
ZP_PTR    = 0x06   # 2 bytes: pointer into catalog buffer
ZP_COUNT  = 0x08   # entry count
ZP_CUR    = 0x09   # current display index
ZP_SEL    = 0x0A   # user selection
ZP_TMP    = 0x0B   # scratch
ZP_NAMLEN = 0x0C
ZP_BLK_LO = 0x0D
ZP_BLK_HI = 0x0E
ZP_D16_HI = 0x1C
ZP_D16_LO = 0x1D
ZP_D16_FL = 0x1E

# Catalog buffer at $0800 (512 bytes, text page 2 — not displayed)
CAT_BUF = 0x0800

# ============================================================
# Simple assembler (same as assemble_startup.py)
# ============================================================
rom = bytearray(MAX_SIZE)
pc = [BASE_ADDR]
labels = {}
fixups = []

def here(): return pc[0]
def label(name): labels[name] = pc[0]

def emit(*bs):
    for b in bs:
        rom[pc[0] - BASE_ADDR] = b & 0xFF
        pc[0] += 1

def imm(op, v): emit(op, v & 0xFF)
def zpg(op, a): emit(op, a & 0xFF)
def abso(op, a): emit(op, a & 0xFF, (a >> 8) & 0xFF)
def branch(op, name):
    emit(op, 0x00)
    fixups.append((pc[0], name, 'rel'))
def jump(op, name):
    emit(op, 0x00, 0x00)
    fixups.append((pc[0], name, 'abs'))

# Instructions
def LDA_i(v): imm(0xA9, v)
def LDX_i(v): imm(0xA2, v)
def LDY_i(v): imm(0xA0, v)
def CMP_i(v): imm(0xC9, v)
def CPX_i(v): imm(0xE0, v)
def ADC_i(v): imm(0x69, v)
def SBC_i(v): imm(0xE9, v)
def AND_i(v): imm(0x29, v)
def ORA_i(v): imm(0x09, v)
def LDA_z(a): zpg(0xA5, a)
def STA_z(a): zpg(0x85, a)
def LDX_z(a): zpg(0xA6, a)
def STX_z(a): zpg(0x86, a)
def LDY_z(a): zpg(0xA4, a)
def STY_z(a): zpg(0x84, a)
def STZ_z(a): zpg(0x64, a)
def INC_z(a): zpg(0xE6, a)
def DEC_z(a): zpg(0xC6, a)
def CMP_z(a): zpg(0xC5, a)
def CPY_z(a): zpg(0xC4, a)
def ADC_z(a): zpg(0x65, a)
def LSR_z(a): zpg(0x46, a)
def LDA_a(a): abso(0xAD, a)
def STA_a(a): abso(0x8D, a)
def LDX_a(a): abso(0xAE, a)
def STZ_a(a): emit(0x9C, a & 0xFF, (a >> 8) & 0xFF)
def JMP_a(a): abso(0x4C, a)
def JSR_a(a): abso(0x20, a)
def BEQ(n): branch(0xF0, n)
def BNE(n): branch(0xD0, n)
def BCS(n): branch(0xB0, n)
def BCC(n): branch(0x90, n)
def BMI(n): branch(0x30, n)
def BPL(n): branch(0x10, n)
def BRA(n): branch(0x80, n)
def JMP_l(n): jump(0x4C, n)
def JSR_l(n): jump(0x20, n)
def CLC(): emit(0x18)
def SEC(): emit(0x38)
def RTS(): emit(0x60)
def INX(): emit(0xE8)
def DEX(): emit(0xCA)
def INY(): emit(0xC8)
def DEY(): emit(0x88)
def TAX(): emit(0xAA)
def TAY(): emit(0xA8)
def TXA(): emit(0x8A)
def TYA(): emit(0x98)
def PHA(): emit(0x48)
def PLA(): emit(0x68)
def PHX(): emit(0xDA)
def PLX(): emit(0xFA)
def ASL_A(): emit(0x0A)
def LSR_A(): emit(0x4A)
def NOP(): emit(0xEA)
def INC_A(): emit(0x1A)
# LDA (ZP),Y
def LDA_iy(a): emit(0xB1, a & 0xFF)

def resolve():
    for fix_pc, name, kind in fixups:
        if name not in labels:
            print(f"ERROR: undefined label '{name}'", file=sys.stderr)
            sys.exit(1)
        target = labels[name]
        off = fix_pc - BASE_ADDR
        if kind == 'rel':
            diff = target - fix_pc
            if not (-128 <= diff <= 127):
                print(f"ERROR: branch '{name}' out of range: {diff}", file=sys.stderr)
                sys.exit(1)
            rom[off - 1] = diff & 0xFF
        else:
            rom[off - 2] = target & 0xFF
            rom[off - 1] = (target >> 8) & 0xFF

def string(s):
    for c in s:
        emit(ord(c))
    emit(0x00)

def emit_print_inline():
    label('print_string')
    PLA()
    STA_z(0xFE)
    PLA()
    STA_z(0xFF)
    LDY_i(0x00)
    label('ps_loop')
    INC_z(0xFE)
    BNE('ps_noinc')
    INC_z(0xFF)
    label('ps_noinc')
    emit(0xB1, 0xFE)  # LDA ($FE),Y
    BEQ('ps_done')
    ORA_i(0x80)
    JSR_a(COUT)
    BRA('ps_loop')
    label('ps_done')
    LDA_z(0xFF)
    PHA()
    LDA_z(0xFE)
    PHA()
    RTS()

# ============================================================
# PICKER PROGRAM
# ============================================================
pc[0] = BASE_ADDR

# --- Title ---
JSR_a(HOME)
JSR_l('print_string')
string("BYTE HAMR SD CARD MANAGER")
JSR_a(CROUT)
LDX_i(25)
label('sep_loop')
LDA_i(ord('=') | 0x80)
JSR_a(COUT)
DEX()
BNE('sep_loop')
JSR_a(CROUT)
JSR_a(CROUT)

# --- Trigger SD init then wait ---
LDA_i(0x02)
STA_a(R_SD_CMD)

JSR_l('print_string')
string("INITIALIZING SD CARD...")

# Poll with generous timeout (~10 seconds)
LDX_i(0x00)
LDY_i(0x00)
STZ_z(ZP_TMP)            # outer-outer counter
label('sd_poll')
LDA_a(R_SD_STAT)
BMI('sd_ready')           # bit 7 = sd_ready
# Check for error (bit 6)
AND_i(0x40)
BNE('sd_error')
INY()
BNE('sd_poll')
INX()
BNE('sd_poll')
INC_z(ZP_TMP)
LDA_z(ZP_TMP)
CMP_i(0x0A)               # 10 * 65536 iterations ≈ 10 seconds
BCC('sd_poll')

# Timeout
label('sd_error')
JSR_a(CROUT)
JSR_l('print_string')
string("NO SD CARD DETECTED")
JSR_a(CROUT)
JMP_l('do_quit')

label('sd_ready')
JSR_a(CROUT)

# --- Read catalog via magic block $FFFF ---
# Set block number to $FFFF
LDA_i(0xFF)
STA_a(R_BLKLO)
STA_a(R_BLKHI)

# Issue READ_BLOCK command
LDA_i(0x01)
STA_a(R_CMD)

# Wait for block ready (bit 7 of STATUS)
label('cat_poll')
LDA_a(R_STATUS)
BPL('cat_poll')            # wait for ready (bit 7)

# --- Read 512 bytes from block buffer into CAT_BUF ---
# Set up pointer
LDA_i(CAT_BUF & 0xFF)
STA_z(ZP_PTR)
LDA_i((CAT_BUF >> 8) & 0xFF)
STA_z(ZP_PTR + 1)

LDY_i(0)                   # byte index within page
LDX_i(2)                   # 2 pages = 512 bytes
label('read_loop')
LDA_a(R_DATA)              # read byte (auto-increment)
emit(0x91, ZP_PTR)         # STA (ZP_PTR),Y
INY()
BNE('read_loop')
INC_z(ZP_PTR + 1)          # next page
DEX()
BNE('read_loop')

# --- Parse catalog header ---
# CAT_BUF[0] = entry_count
LDA_a(CAT_BUF)
BNE('has_files')

JSR_l('print_string')
string("NO IMAGES ON SD CARD")
JSR_a(CROUT)
JMP_l('do_quit')

label('has_files')
STA_z(ZP_COUNT)

# --- List images ---
STZ_z(ZP_CUR)

# Point to first entry: CAT_BUF + 32
LDA_i((CAT_BUF + 32) & 0xFF)
STA_z(ZP_PTR)
LDA_i(((CAT_BUF + 32) >> 8) & 0xFF)
STA_z(ZP_PTR + 1)

label('list_loop')

# Print 1-based index
LDA_z(ZP_CUR)
CLC()
ADC_i(1)
JSR_l('print_dec_byte')
LDA_i(ord(')') | 0x80)
JSR_a(COUT)
LDA_i(ord(' ') | 0x80)
JSR_a(COUT)

# Print filename: bytes 0-7 (name) + "." + bytes 8-10 (ext)
# Trim trailing spaces from name part
LDY_i(7)
label('trim_loop')
LDA_iy(ZP_PTR)             # LDA (ZP_PTR),Y — read name[Y]
CMP_i(0x20)
BNE('trim_found')
DEY()
BPL('trim_loop')
LDY_i(0)
label('trim_found')
STY_z(ZP_NAMLEN)

# Print name chars 0..NAMLEN
LDY_i(0)
label('pname_loop')
LDA_iy(ZP_PTR)
ORA_i(0x80)
JSR_a(COUT)
CPY_z(ZP_NAMLEN)
BEQ('pname_done')
INY()
BRA('pname_loop')
label('pname_done')

# Print dot + extension (bytes 8-10)
LDA_i(ord('.') | 0x80)
JSR_a(COUT)
LDY_i(8)
LDA_iy(ZP_PTR)
ORA_i(0x80)
JSR_a(COUT)
INY()
LDA_iy(ZP_PTR)
ORA_i(0x80)
JSR_a(COUT)
INY()
LDA_iy(ZP_PTR)
CMP_i(0x20)                # skip trailing space in ext
BEQ('ext_done')
ORA_i(0x80)
JSR_a(COUT)
label('ext_done')

# Print " (XXXXX BLOCKS)"
LDA_i(ord(' ') | 0x80)
JSR_a(COUT)
LDA_i(ord('(') | 0x80)
JSR_a(COUT)

# Block count at entry bytes 12-13 (LE)
LDY_i(13)
LDA_iy(ZP_PTR)             # block_count high
STA_z(ZP_D16_HI)
DEY()
LDA_iy(ZP_PTR)             # block_count low
STA_z(ZP_D16_LO)
STZ_z(ZP_D16_FL)
JSR_l('print_dec_16')

JSR_l('print_string')
string(" BLOCKS)")
JSR_a(CROUT)

# Advance pointer to next entry (+32 bytes)
CLC()
LDA_z(ZP_PTR)
ADC_i(32)
STA_z(ZP_PTR)
LDA_z(ZP_PTR + 1)
ADC_i(0)
STA_z(ZP_PTR + 1)

# Next image
INC_z(ZP_CUR)
LDA_z(ZP_CUR)
CMP_z(ZP_COUNT)
BCS('list_done2')
JMP_l('list_loop')
label('list_done2')

# --- Prompt ---
JSR_a(CROUT)
JSR_l('print_string')
string("SELECT IMAGE (0=QUIT): ")

STZ_z(ZP_SEL)
label('input_loop')
JSR_a(RDKEY)
CMP_i(0x8D)                # RETURN
BEQ('input_done')
CMP_i(0xB0)
BCC('input_loop')
CMP_i(0xBA)
BCS('input_loop')
JSR_a(COUT)
AND_i(0x0F)
PHA()
LDA_z(ZP_SEL)
ASL_A()
STA_z(ZP_TMP)
ASL_A()
ASL_A()
CLC()
ADC_z(ZP_TMP)              # *10
STA_z(ZP_SEL)
PLA()
CLC()
ADC_z(ZP_SEL)
STA_z(ZP_SEL)
BRA('input_loop')

label('input_done')
JSR_a(CROUT)

LDA_z(ZP_SEL)
BNE('sel_ok')
JMP_l('do_quit')
label('sel_ok')
SEC()
SBC_i(1)                   # to 0-based
CMP_z(ZP_COUNT)
BCC('sel_valid')
JMP_l('do_quit')
label('sel_valid')

# --- Mount ---
STA_a(R_SELECT)             # write image index
LDA_i(0x01)
STA_a(R_SD_CMD)             # mount command

JSR_l('print_string')
string("MOUNTING...")

# Poll for s4d2_mounted (bit 5 of SD_STAT) — generous timeout ~60 sec
LDX_i(0x00)
LDY_i(0x00)
STZ_z(ZP_TMP)
label('mount_poll')
LDA_a(R_SD_STAT)
AND_i(0x20)                 # s4d2_mounted bit
BNE('mount_ok')
INY()
BNE('mount_poll')
INX()
BNE('mount_poll')
INC_z(ZP_TMP)
LDA_z(ZP_TMP)
CMP_i(0x3C)                 # 60 * 65536 iterations ≈ 60 seconds
BCC('mount_poll')

label('mount_ok')
JSR_a(CROUT)
JSR_l('print_string')
string("BOOTING...")
JSR_a(CROUT)

# Reboot via slot ROM boot vector
JMP_a(SLOT_ROM)

label('mount_fail')
JSR_a(CROUT)
JSR_l('print_string')
string("MOUNT TIMEOUT")
JSR_a(CROUT)

# --- Quit ---
label('do_quit')
JSR_a(MLI)
emit(QUIT)
emit(0x00, 0x00)
emit(0x04, 0x00, 0x00, 0x00)  # quit parms

# ============================================================
# Subroutines
# ============================================================
emit_print_inline()

# --- print_dec_byte ---
label('print_dec_byte')
LDX_i(0)
CMP_i(100)
BCC('pdb_tens')
label('pdb_h_loop')
SEC()
SBC_i(100)
INX()
CMP_i(100)
BCS('pdb_h_loop')
PHA()
TXA()
ORA_i(0xB0)
JSR_a(COUT)
PLA()
LDX_i(1)
label('pdb_tens')
TAY()
TXA()
PHA()
TYA()
LDX_i(0)
CMP_i(10)
BCC('pdb_ones')
label('pdb_t_loop')
SEC()
SBC_i(10)
INX()
CMP_i(10)
BCS('pdb_t_loop')
label('pdb_ones')
TAY()
PLA()
BNE('pdb_print_tens')
TXA()
BEQ('pdb_skip_tens')
label('pdb_print_tens')
TXA()
ORA_i(0xB0)
JSR_a(COUT)
label('pdb_skip_tens')
TYA()
ORA_i(0xB0)
JSR_a(COUT)
RTS()

# --- print_dec_16 (value in ZP_D16_HI:ZP_D16_LO) ---
label('print_dec_16')
# 10000s
LDX_i(0)
label('d16_10k')
SEC()
LDA_z(ZP_D16_LO)
SBC_i(0x10)
TAY()
LDA_z(ZP_D16_HI)
SBC_i(0x27)
BCC('d16_10k_done')
STA_z(ZP_D16_HI)
STY_z(ZP_D16_LO)
INX()
BRA('d16_10k')
label('d16_10k_done')
JSR_l('d16_digit')

# 1000s
LDX_i(0)
label('d16_1k')
SEC()
LDA_z(ZP_D16_LO)
SBC_i(0xE8)
TAY()
LDA_z(ZP_D16_HI)
SBC_i(0x03)
BCC('d16_1k_done')
STA_z(ZP_D16_HI)
STY_z(ZP_D16_LO)
INX()
BRA('d16_1k')
label('d16_1k_done')
JSR_l('d16_digit')

# 100s
LDX_i(0)
label('d16_100')
SEC()
LDA_z(ZP_D16_LO)
SBC_i(100)
TAY()
LDA_z(ZP_D16_HI)
SBC_i(0)
BCC('d16_100_done')
STA_z(ZP_D16_HI)
STY_z(ZP_D16_LO)
INX()
BRA('d16_100')
label('d16_100_done')
JSR_l('d16_digit')

# 10s
LDX_i(0)
label('d16_10')
LDA_z(ZP_D16_LO)
CMP_i(10)
BCC('d16_10_done')
SBC_i(10)
STA_z(ZP_D16_LO)
INX()
BRA('d16_10')
label('d16_10_done')
JSR_l('d16_digit')

# 1s
LDA_z(ZP_D16_LO)
ORA_i(0xB0)
JSR_a(COUT)
RTS()

label('d16_digit')
TXA()
BNE('d16_nonzero')
LDA_z(ZP_D16_FL)
BEQ('d16_skip')
label('d16_nonzero')
LDA_i(1)
STA_z(ZP_D16_FL)
TXA()
ORA_i(0xB0)
JSR_a(COUT)
label('d16_skip')
RTS()

# ============================================================
# Resolve and output
# ============================================================
resolve()

size = pc[0] - BASE_ADDR
print(f"Picker assembled: {size} bytes (${BASE_ADDR:04X}-${pc[0]-1:04X})")

# Debug: dump the sd_poll area
poll_addr = labels.get('sd_poll', 0)
ready_addr = labels.get('sd_ready', 0)
error_addr = labels.get('sd_error', 0)
print(f"Labels: sd_poll=${poll_addr:04X} sd_ready=${ready_addr:04X} sd_error=${error_addr:04X}")
if poll_addr:
    off = poll_addr - BASE_ADDR
    print(f"Bytes at sd_poll (${poll_addr:04X}):")
    for i in range(30):
        if i > 0 and i % 16 == 0: print()
        print(f"{rom[off+i]:02X} ", end='')
    print()
    # Decode the BMI
    bmi_off = off + 3  # BMI is 4th byte (after 3-byte LDA abs)
    if rom[bmi_off] == 0x30:
        rel = rom[bmi_off + 1]
        if rel > 127: rel -= 256
        target = poll_addr + 5 + rel  # PC after BMI + offset
        print(f"BMI offset: {rel} -> target ${target:04X} (sd_ready=${ready_addr:04X})")

if size > MAX_SIZE:
    print(f"ERROR: exceeds {MAX_SIZE} bytes!", file=sys.stderr)
    sys.exit(1)

outfile = os.path.join(os.path.dirname(__file__), "picker.sys")
with open(outfile, "wb") as f:
    f.write(rom[:size])

print(f"Written to {outfile}")
