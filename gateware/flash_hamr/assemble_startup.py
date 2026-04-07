#!/usr/bin/env python3
"""
assemble_startup.py — Byte Hamr SD Card Manager (65C02)

ProDOS SYS file. Loaded at $2000 by BASIC.SYSTEM.
Lists .PO/.2MG files from SD card, lets user mount one as S4,D2.
"""

import sys

BASE_ADDR = 0x2000
MAX_SIZE  = 0x1000   # 4KB max

# Slot 4 I/O registers (directly addressed, not indexed)
SLOT = 4
IO = 0xC080 + SLOT * 0x10  # $C0C0

R_STATUS  = IO + 0   # R: {ready,error,5'b0,boot_done}  W: CMD
R_SD_STAT = IO + 8   # R: {sd_ready,sd_error,mounted,loading,4'b0}
R_SD_CMD  = IO + 8   # W: $01=mount, $02=sd_init
R_COUNT   = IO + 9   # R: img_count
R_SELECT  = IO + 9   # W: img_select
R_NAME    = IO + 0xA # W: name_idx, R: name_char
R_BLKLO   = IO + 0xB # R: s4d2_block_count lo
R_BLKHI   = IO + 0xC # R: s4d2_block_count hi
R_FLAGS   = IO + 0xD # R: {7'b0, is_2mg}

# Monitor ROM
HOME   = 0xFC58
COUT   = 0xFDED
CROUT  = 0xFD8E
RDKEY  = 0xFD0C
PRBYTE = 0xFDDA

# ProDOS MLI
MLI    = 0xBF00
QUIT   = 0x65

# Zero page temporaries (safe to use under ProDOS)
ZP_TMP    = 0x06   # scratch
ZP_COUNT  = 0x07   # img_count
ZP_CUR    = 0x08   # current image index
ZP_SEL    = 0x09   # user selection
ZP_BLK_LO = 0x19   # block count lo
ZP_BLK_HI = 0x1A   # block count hi
ZP_NAMLEN = 0x1B   # name length (for trimming)

# ============================================================
# Simple assembler
# ============================================================
rom = bytearray(MAX_SIZE)
pc = [BASE_ADDR]
labels = {}
fixups = []

def org(addr): pc[0] = addr
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

# 65C02 instructions
def LDA_i(v): imm(0xA9, v)
def LDX_i(v): imm(0xA2, v)
def LDY_i(v): imm(0xA0, v)
def CMP_i(v): imm(0xC9, v)
def CPX_i(v): imm(0xE0, v)
def CPY_i(v): imm(0xC0, v)
def ADC_i(v): imm(0x69, v)
def SBC_i(v): imm(0xE9, v)
def AND_i(v): imm(0x29, v)
def ORA_i(v): imm(0x09, v)
def EOR_i(v): imm(0x49, v)
def LDA_z(a): zpg(0xA5, a)
def STA_z(a): zpg(0x85, a)
def STX_z(a): zpg(0x86, a)
def LDX_z(a): zpg(0xA6, a)
def STZ_z(a): zpg(0x64, a)  # 65C02
def STY_z(a): zpg(0x84, a)
def LDY_z(a): zpg(0xA4, a)
def CPY_z(a): zpg(0xC4, a)
def CMP_z(a): zpg(0xC5, a)
def ADC_z(a): zpg(0x65, a)
def SBC_z(a): zpg(0xE5, a)
def INC_z(a): zpg(0xE6, a)
def DEC_z(a): zpg(0xC6, a)
def LSR_z(a): zpg(0x46, a)
def ROR_z(a): zpg(0x66, a)
def LDA_a(a):  abso(0xAD, a)
def STA_a(a):  abso(0x8D, a)
def LDX_a(a):  abso(0xAE, a)
def STZ_a(a):  emit(0x9C, a & 0xFF, (a >> 8) & 0xFF)  # 65C02
def BIT_a(a):  abso(0x2C, a)
def JMP_a(a):  abso(0x4C, a)
def JSR_a(a):  abso(0x20, a)
def BEQ(n): branch(0xF0, n)
def BNE(n): branch(0xD0, n)
def BCS(n): branch(0xB0, n)
def BCC(n): branch(0x90, n)
def BMI(n): branch(0x30, n)
def BPL(n): branch(0x10, n)
def BRA(n): branch(0x80, n)  # 65C02
def JMP_l(n): jump(0x4C, n)
def JSR_l(n): jump(0x20, n)
def CLC(): emit(0x18)
def SEC(): emit(0x38)
def CLD(): emit(0xD8)
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
def PHX(): emit(0xDA)  # 65C02
def PLX(): emit(0xFA)  # 65C02
def PHY(): emit(0x5A)  # 65C02
def PLY(): emit(0x7A)  # 65C02
def ASL_A(): emit(0x0A)
def LSR_A(): emit(0x4A)
def NOP(): emit(0xEA)
def INC_A(): emit(0x1A)  # 65C02

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
    """Emit null-terminated ASCII string (no high bit)."""
    for c in s:
        emit(ord(c))
    emit(0x00)

def print_str(lbl):
    """JSR print_string, .word addr — prints null-terminated string."""
    JSR_l('print_string')
    jump(0x00, lbl)  # emit address as 2 bytes (not an opcode)
    # Actually this won't work cleanly. Let me use a different approach.

# Better approach: inline string printing via indexed load
def emit_print_inline():
    """Emit the print_string subroutine that prints inline string after JSR."""
    label('print_string')
    # Pull return address from stack (points to byte AFTER JSR = first string byte - 1)
    PLA()
    STA_z(0xFE)       # lo
    PLA()
    STA_z(0xFF)       # hi
    LDY_i(0x00)
    label('ps_loop')
    # Increment pointer (return addr is 1 less than first byte)
    INC_z(0xFE)
    BNE('ps_noinc')
    INC_z(0xFF)
    label('ps_noinc')
    LDA_i(0x00)       # placeholder - indirect load
    # LDA ($FE),Y
    emit(0xB1, 0xFE)
    BEQ('ps_done')
    ORA_i(0x80)        # set high bit for display
    JSR_a(COUT)
    BRA('ps_loop')     # 65C02
    label('ps_done')
    # Push updated address back as return address
    LDA_z(0xFF)
    PHA()
    LDA_z(0xFE)
    PHA()
    RTS()

# ============================================================
# STARTUP program
# ============================================================
org(BASE_ADDR)

# --- Clear screen and print title ---
JSR_a(HOME)
JSR_l('print_string')
string("BYTE HAMR SD CARD MANAGER")
JSR_a(CROUT)

# Separator line
LDX_i(25)
label('sep_loop')
LDA_i(ord('=') | 0x80)
JSR_a(COUT)
DEX()
BNE('sep_loop')
JSR_a(CROUT)
JSR_a(CROUT)

# --- Trigger SD init ---
LDA_i(0x02)
STA_a(R_SD_CMD)

JSR_l('print_string')
string("INITIALIZING SD CARD...")

# --- Poll for sd_ready (bit 7 of SD_STATUS) ---
LDX_i(0x00)
LDY_i(0x00)
label('poll_loop')
LDA_a(R_SD_STAT)
BMI('poll_ready')       # bit 7 set = done
INY()
BNE('poll_loop')
INX()
CPX_i(0x80)             # generous timeout
BNE('poll_loop')

# Timeout — no SD card
JSR_a(CROUT)
JSR_l('print_string')
string("NO SD CARD DETECTED")
JSR_a(CROUT)
JMP_l('do_quit')

# --- SD ready ---
label('poll_ready')
JSR_a(CROUT)

# Check file count
LDA_a(R_COUNT)
BNE('has_files')

JSR_l('print_string')
string("NO IMAGES ON SD CARD")
JSR_a(CROUT)
JMP_l('do_quit')

label('has_files')
STA_z(ZP_COUNT)
STZ_z(ZP_CUR)

# --- List files ---
label('list_loop')
LDA_z(ZP_CUR)
STA_a(R_SELECT)         # select image

# Print 1-based index
CLC()
ADC_i(1)
JSR_l('print_dec_byte')
LDA_i(ord(')') | 0x80)
JSR_a(COUT)
LDA_i(ord(' ') | 0x80)
JSR_a(COUT)

# Print filename (catalog bytes 0-7, trim trailing spaces)
# Find last non-space char in bytes 0-7
LDY_i(7)
label('trim_loop')
TYA()
STA_a(R_NAME)           # write index
LDA_a(R_NAME)           # read char
CMP_i(0x20)
BNE('trim_found')
DEY()
BPL('trim_loop')
LDY_i(0)                # all spaces — show at least byte 0
label('trim_found')
STY_z(ZP_NAMLEN)        # last printable index

# Print chars 0..ZP_NAMLEN
LDY_i(0)
label('name_loop')
TYA()
STA_a(R_NAME)           # write index
LDA_a(R_NAME)           # read char
ORA_i(0x80)
JSR_a(COUT)
CPY_z(ZP_NAMLEN)
BEQ('name_done')        # printed last char
INY()
BRA('name_loop')
label('name_done')

# Print extension from catalog flags (byte 11, bit 0 = is_2mg)
LDA_i(11)
STA_a(R_NAME)
LDA_a(R_NAME)
AND_i(0x01)
BNE('ext_2mg')

# .PO
JSR_l('print_string')
string(".PO")
BRA('after_ext')

label('ext_2mg')
JSR_l('print_string')
string(".2MG")

label('after_ext')
LDA_i(ord(' ') | 0x80)
JSR_a(COUT)
LDA_i(ord('(') | 0x80)
JSR_a(COUT)

# Block count = file_size / 512 (catalog bytes 16-19)
# file_size >> 9: drop byte 0, shift bytes 1-3 right by 1
LDA_i(17)
STA_a(R_NAME)
LDA_a(R_NAME)           # fs_byte1
LSR_A()                  # >> 1, carry = bit 0
STA_z(ZP_BLK_LO)

LDA_i(18)
STA_a(R_NAME)
LDA_a(R_NAME)           # fs_byte2
PHA()                    # save for carry
LSR_A()
STA_z(ZP_BLK_HI)
PLA()
AND_i(0x01)              # fs_byte2 bit 0 → blk_lo bit 7
BEQ('no_carry1')
LDA_z(ZP_BLK_LO)
ORA_i(0x80)
STA_z(ZP_BLK_LO)
label('no_carry1')

LDA_i(19)
STA_a(R_NAME)
LDA_a(R_NAME)           # fs_byte3
AND_i(0x01)              # fs_byte3 bit 0 → blk_hi bit 7
BEQ('no_carry2')
LDA_z(ZP_BLK_HI)
ORA_i(0x80)
STA_z(ZP_BLK_HI)
label('no_carry2')

# Print block count as decimal
LDA_z(ZP_BLK_HI)
LDX_z(ZP_BLK_LO)
JSR_l('print_dec_16')

JSR_l('print_string')
string(" BLOCKS)")
JSR_a(CROUT)

# Next image
INC_z(ZP_CUR)
LDA_z(ZP_CUR)
CMP_z(ZP_COUNT)
BCS('list_done')
JMP_l('list_loop')
label('list_done')

# --- Prompt for selection ---
JSR_a(CROUT)
JSR_l('print_string')
string("SELECT IMAGE (0=QUIT): ")

# Read decimal input (digits until RETURN)
STZ_z(ZP_SEL)
label('input_loop')
JSR_a(RDKEY)
CMP_i(0x8D)             # RETURN
BEQ('input_done')
CMP_i(0xB0)             # '0'
BCC('input_loop')
CMP_i(0xBA)             # '9'+1
BCS('input_loop')
JSR_a(COUT)             # echo digit
AND_i(0x0F)             # ASCII → digit value
PHA()
# ZP_SEL = ZP_SEL * 10 + digit
LDA_z(ZP_SEL)
ASL_A()                  # *2
STA_z(ZP_TMP)
ASL_A()                  # *4
ASL_A()                  # *8
CLC()
ADC_z(ZP_TMP)            # *10
STA_z(ZP_SEL)
PLA()
CLC()
ADC_z(ZP_SEL)
STA_z(ZP_SEL)
BRA('input_loop')

label('input_done')
JSR_a(CROUT)

# Check selection
LDA_z(ZP_SEL)
BNE('sel_ok')
JMP_l('do_quit')         # 0 = quit
label('sel_ok')
SEC()
SBC_i(1)                 # to 0-based
CMP_z(ZP_COUNT)
BCC('sel_valid')
JMP_l('do_quit')         # out of range
label('sel_valid')

# --- Mount selected image ---
STA_a(R_SELECT)
LDA_i(0x01)
STA_a(R_SD_CMD)          # mount command

JSR_l('print_string')
string("LOADING")

# Poll for mount complete (loading bit 4 clears) with long timeout
# ~65K * 256 * 13 cycles ≈ 217M cycles ≈ 212 seconds at 1MHz
STZ_z(0x1C)        # outer counter hi byte
LDX_i(0x00)
LDY_i(0x00)
label('mount_poll')
LDA_a(R_SD_STAT)
AND_i(0x10)
BEQ('mount_ok')
INY()
BNE('mount_poll')
INX()
BNE('mount_poll')
INC_z(0x1C)
LDA_z(0x1C)
CMP_i(0x14)        # 20 * 65536 iterations ≈ 60 seconds
BCC('mount_poll')    # CMP sets carry if >= 4
JMP_l('mount_fail')

label('mount_ok')

JSR_a(CROUT)
JSR_l('print_string')
string("MOUNTED AS S4,D2")
JSR_a(CROUT)
JSR_l('print_string')
string("TYPE: CATALOG S4,D2")
JSR_a(CROUT)
RTS()                    # return to BASIC.SYSTEM

# --- Quit to ProDOS ---
label('mount_fail')
JSR_a(CROUT)
JSR_l('print_string')
string("MOUNT TIMEOUT")
JSR_a(CROUT)

label('do_quit')
JSR_a(MLI)
emit(QUIT)
emit(0x00, 0x00)         # quit parms address (inline)
# Quit parameter block (4 bytes)
emit(0x04)               # parm_count = 4
emit(0x00)               # quit type = 0
emit(0x00, 0x00)         # reserved

# ============================================================
# Subroutines
# ============================================================

# --- print_string: print inline null-terminated string ---
# Usage: JSR print_string / .byte "hello",0
# Modifies: A, Y, $FE-$FF
emit_print_inline()

# --- print_dec_byte: print A as 1-3 digit decimal ---
# Input: A = 0-255
# Modifies: A, X
label('print_dec_byte')
LDX_i(0)                # leading zero suppression flag
CMP_i(100)
BCC('pdb_tens')
# Hundreds digit
label('pdb_h_loop')
SEC()
SBC_i(100)
INX()
CMP_i(100)
BCS('pdb_h_loop')
PHA()
TXA()
ORA_i(0xB0)             # digit → ASCII
JSR_a(COUT)
PLA()
LDX_i(1)                # have printed a digit
label('pdb_tens')
TAY()                    # save remainder
TXA()
PHA()                    # save suppress flag
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
TAY()                    # save ones
PLA()                    # suppress flag
BNE('pdb_print_tens')   # already printed hundreds
TXA()
BEQ('pdb_skip_tens')    # tens = 0 and no hundreds → skip
label('pdb_print_tens')
TXA()
ORA_i(0xB0)
JSR_a(COUT)
label('pdb_skip_tens')
TYA()                    # ones digit (always print)
ORA_i(0xB0)
JSR_a(COUT)
RTS()

# --- print_dec_16: print 16-bit decimal (A=hi, X=lo) ---
# Input: A = high byte, X = low byte
# Prints 0-65535 with leading zero suppression
label('print_dec_16')
# Store value in ZP_BLK_HI:ZP_BLK_LO (already there from caller)
# Use repeated subtraction for each power of 10
# Powers: 10000, 1000, 100, 10, 1

STA_z(0x1C)              # value_hi
STX_z(0x1D)              # value_lo
STZ_z(0x1E)              # leading zero flag

# 10000s
LDX_i(0)
label('d16_10k')
# Subtract 10000 ($2710): lo=0x10, hi=0x27
SEC()
LDA_z(0x1D)
SBC_i(0x10)
TAY()
LDA_z(0x1C)
SBC_i(0x27)
BCC('d16_10k_done')
STA_z(0x1C)
STY_z(0x1D)
INX()
BRA('d16_10k')
label('d16_10k_done')
JSR_l('d16_digit')

# 1000s
LDX_i(0)
label('d16_1k')
SEC()
LDA_z(0x1D)
SBC_i(0xE8)             # 1000 = $03E8
TAY()
LDA_z(0x1C)
SBC_i(0x03)
BCC('d16_1k_done')
STA_z(0x1C)
STY_z(0x1D)
INX()
BRA('d16_1k')
label('d16_1k_done')
JSR_l('d16_digit')

# 100s
LDX_i(0)
label('d16_100')
SEC()
LDA_z(0x1D)
SBC_i(100)
TAY()
LDA_z(0x1C)
SBC_i(0)
BCC('d16_100_done')
STA_z(0x1C)
STY_z(0x1D)
INX()
BRA('d16_100')
label('d16_100_done')
JSR_l('d16_digit')

# 10s
LDX_i(0)
label('d16_10')
LDA_z(0x1D)
CMP_i(10)
BCC('d16_10_done')
SBC_i(10)
STA_z(0x1D)
INX()
BRA('d16_10')
label('d16_10_done')
JSR_l('d16_digit')

# 1s (always print)
LDA_z(0x1D)
ORA_i(0xB0)
JSR_a(COUT)
RTS()

# Helper: print digit X with leading zero suppression
label('d16_digit')
TXA()
BNE('d16_nonzero')
LDA_z(0x1E)             # suppress flag
BEQ('d16_skip')         # no digits yet → skip zero
label('d16_nonzero')
LDA_i(1)
STA_z(0x1E)             # mark that we've printed a digit
TXA()
ORA_i(0xB0)
JSR_a(COUT)
label('d16_skip')
RTS()

# ============================================================
# Resolve labels and output
# ============================================================
resolve()

size = pc[0] - BASE_ADDR
print(f"STARTUP assembled: {size} bytes (${BASE_ADDR:04X}-${pc[0]-1:04X})")

if size > MAX_SIZE:
    print(f"ERROR: exceeds {MAX_SIZE} bytes!", file=sys.stderr)
    sys.exit(1)

# Write raw binary
with open("startup.sys", "wb") as f:
    f.write(rom[:size])

print(f"Written to startup.sys")
