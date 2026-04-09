#!/usr/bin/env python3
"""
assemble_rom.py — Flash Hamr ProDOS block device ROM ($Cn page only)

Entire driver fits in 256-byte $Cn page. No $C800 expansion ROM.
This avoids bus contention with the Apple IIe's internal ROM at $C800.

S4D1 (unit 1): Menu volume from SPI flash (block count from FPGA regs)
S4D2 (unit 2): SD card image (block count from FPGA regs)

Register interface ($C080 + slot*16):
  0: STATUS(R) / COMMAND(W)  — R:{ready,error,5'b0,boot_done} W:$01=READ,$02=WRITE
  1: DATA_READ(R)             — auto-increment
  2: BLOCK_LO(R/W)
  3: BLOCK_HI(R/W)
  5: DATA_WRITE(W)            — auto-increment (separate addr avoids STA dummy-read bug)
  6: TOTAL_BLOCKS_LO(R)       — S4D1 block count (from mailbox)
  7: TOTAL_BLOCKS_HI(R)
  8: SD_CMD(W)                — mailbox command ($01=mount, $02=sd_init)
  B: S4D2_BLOCKS_LO(R)
  C: S4D2_BLOCKS_HI(R)
"""

SLOT = 4

# Zero page (ProDOS standard)
ZP_CMD    = 0x42
ZP_UNIT   = 0x43
ZP_BUF    = 0x44
ZP_BUF_H  = 0x45
ZP_BLK    = 0x46
ZP_BLK_H  = 0x47
ZP_TEMP   = 0x3A  # safe ZP for long wait_ready

# I/O registers (indexed by X = slot*16)
IO_CMD    = 0xC080
IO_DATA   = 0xC081
IO_BLLO   = 0xC082
IO_BLHI   = 0xC083
IO_DATAW  = 0xC085
IO_CNTLO  = 0xC086
IO_CNTHI  = 0xC087
IO_D2BLLO = 0xC08B
IO_D2BLHI = 0xC08C

ERR_NONE = 0x00; ERR_IO = 0x27; ERR_NODEV = 0x28; ERR_BADCMD = 0x01
CMD_READ = 0x01; CMD_WRITE = 0x02

rom = bytearray(4096)
rom[:] = b'\xFF' * 4096

pc = [0]
labels = {}
fixups = []

def org(addr): pc[0] = addr
def here(): return pc[0]
def label(name): labels[name] = pc[0]

def emit(*bs):
    for b in bs:
        rom[pc[0] - 0xC000] = b & 0xFF
        pc[0] += 1

def imm(op, v): emit(op, v)
def zpg(op, a): emit(op, a)
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
def CPX_i(v): imm(0xE0, v)  # CPX — signature bytes (same operand as LDX)
def CMP_i(v): imm(0xC9, v)
def ADC_i(v): imm(0x69, v)
def SBC_i(v): imm(0xE9, v)
def AND_i(v): imm(0x29, v)
def LDA_z(a): zpg(0xA5, a)
def STA_z(a): zpg(0x85, a)
def STX_z(a): zpg(0x86, a)
def INC_z(a): zpg(0xE6, a)
def DEC_z(a): zpg(0xC6, a)
def LDA_iy(a): zpg(0xB1, a)
def STA_iy(a): zpg(0x91, a)
def LDA_ax(a): abso(0xBD, a)
def STA_ax(a): abso(0x9D, a)
def LDA_ay(a): abso(0xB9, a)
def STA_ay(a): abso(0x99, a)
def LDA_a(a):  abso(0xAD, a)
def STA_a(a):  abso(0x8D, a)
def JMP_a(a):  abso(0x4C, a)
def JSR_a(a):  abso(0x20, a)
def BEQ(n): branch(0xF0, n)
def BNE(n): branch(0xD0, n)
def BCS(n): branch(0xB0, n)
def BCC(n): branch(0x90, n)
def BMI(n): branch(0x30, n)
def JMP_l(n): jump(0x4C, n)
def JSR_l(n): jump(0x20, n)
def CLC(): emit(0x18)
def SEC(): emit(0x38)
def CLD(): emit(0xD8)
def RTS(): emit(0x60)
def INY(): emit(0xC8)
def DEY(): emit(0x88)
def TAX(): emit(0xAA)
def TAY(): emit(0xA8)
def TXA(): emit(0x8A)
def PHA(): emit(0x48)
def PLA(): emit(0x68)
def ASL_A(): emit(0x0A)

def resolve():
    for fix_pc, name, kind in fixups:
        target = labels[name]
        off = fix_pc - 0xC000
        if kind == 'rel':
            diff = target - fix_pc
            assert -128 <= diff <= 127, f"Branch {name} out of range: {diff} at ${fix_pc-1:04X}"
            rom[off - 1] = diff & 0xFF
        else:
            rom[off - 2] = target & 0xFF
            rom[off - 1] = (target >> 8) & 0xFF

# ==========================================================================
# $Cn page — entire driver
# ==========================================================================
org(0xC400)

# ---- Signature bytes (ProDOS checks $Cn01, $Cn03, $Cn05) ----
# Use CPX so the operand bytes match LDX but opcode is different (old ROM style)
CPX_i(0x20)        # $Cn01 = $20: block device
CPX_i(0x00)        # $Cn03 = $00
CPX_i(0x03)        # $Cn05 = $03

# $Cn06-$Cn07: boot detection — CPX with slot*16, if X matches we have our slot
CPX_i(SLOT << 4)   # $Cn07 = $40 (slot*16, used by old ROM for slot detection)

# ---- Boot entry: carry set from Apple II ROM scan ----
LDA_i(SLOT << 4)   # A = slot*16
JMP_l('boot')      # boot handler

# ---- ProDOS entry ($CnFF points here) ----
label('prodos_entry')
LDX_i(SLOT << 4)   # X = slot*16
JMP_l('dispatch')   # command dispatch

# ---- Boot handler ----
label('boot')
TAX()               # X = slot*16 (from LDA above)
STX_z(ZP_UNIT)      # save for later
LDA_i(CMD_READ); STA_z(ZP_CMD)
LDA_i(0x00); STA_z(ZP_BLK); STA_z(ZP_BLK_H)
STA_z(ZP_BUF)
LDA_i(0x08); STA_z(ZP_BUF_H)  # buffer = $0800
JSR_l('dispatch')   # read block 0 to $0800
JMP_a(0x0801)       # jump to boot code

# ---- Command dispatch ----
label('dispatch')
LDA_z(ZP_CMD)
BEQ('cmd_status')
CMP_i(CMD_READ)
BEQ('cmd_read')
CMP_i(CMD_WRITE)
BEQ('cmd_write')
# Unknown command
SEC(); LDA_i(ERR_BADCMD); RTS()

# ---- STATUS ----
label('cmd_status')
LDA_ax(IO_CMD); AND_i(0x01)  # check boot_done
BEQ('status_offline')
# Read block count from FPGA registers (works for both units)
LDA_ax(IO_CNTHI); TAY()      # Y = blocks hi
LDA_ax(IO_CNTLO); TAX()      # X = blocks lo
LDA_i(ERR_NONE); CLC(); RTS()
label('status_offline')
LDA_i(ERR_NODEV); SEC(); RTS()

# ---- READ ----
label('cmd_read')
LDA_z(ZP_BLK);   STA_ax(IO_BLLO)
LDA_z(ZP_BLK_H); STA_ax(IO_BLHI)
LDA_i(CMD_READ);  STA_ax(IO_CMD)
JSR_l('wait_ready')
BCS('read_done')
LDY_i(0x00)
label('rd1')
LDA_ax(IO_DATA); STA_iy(ZP_BUF); INY(); BNE('rd1')
INC_z(ZP_BUF_H)
JSR_l('rd_page2')
DEC_z(ZP_BUF_H)
LDA_i(ERR_NONE); CLC()
label('read_done')
RTS()

# Second page read (reusable subroutine saves bytes)
label('rd_page2')
LDY_i(0x00)
label('rd2')
LDA_ax(IO_DATA); STA_iy(ZP_BUF); INY(); BNE('rd2')
RTS()

# ---- WRITE ----
label('cmd_write')
LDA_z(ZP_BLK);   STA_ax(IO_BLLO)
LDA_z(ZP_BLK_H); STA_ax(IO_BLHI)
LDY_i(0x00)
label('wr1')
LDA_iy(ZP_BUF); STA_ax(IO_DATAW); INY(); BNE('wr1')
INC_z(ZP_BUF_H)
LDY_i(0x00)
label('wr2')
LDA_iy(ZP_BUF); STA_ax(IO_DATAW); INY(); BNE('wr2')
DEC_z(ZP_BUF_H)
LDA_i(CMD_WRITE); STA_ax(IO_CMD)
JSR_l('wait_ready')
BCS('write_done')
LDA_i(ERR_NONE); CLC()
label('write_done')
RTS()

# ---- wait_ready (short: ~16ms timeout for normal ops) ----
label('wait_ready')
LDY_i(0x00)
label('wr_out')
LDA_i(0x00)
label('wr_in')
PHA()
LDA_ax(IO_CMD); BMI('wr_ok')
PLA(); SEC(); SBC_i(1); BNE('wr_in')
DEY(); BNE('wr_out')
# Timeout
PLA(); LDA_i(ERR_IO); SEC(); RTS()
label('wr_ok')
AND_i(0x40); BNE('wr_err')
PLA(); LDA_i(ERR_NONE); CLC(); RTS()
label('wr_err')
PLA(); LDA_i(ERR_IO); SEC(); RTS()

# ---- Pad and signature ----
cn_used = here() - 0xC400
assert cn_used <= 251, f"$Cn page overflow: {cn_used} bytes used, max 251"

while here() < 0xC4FB: emit(0x00)
emit(0x00)         # $CnFB: SmartPort device type
emit(0x00, 0x00)   # $CnFC-FD: block count (0 = requires STATUS)
emit(0x17)         # $CnFE: read+write+SmartPort, 2 volumes (matches old working ROM)
emit(labels['prodos_entry'] & 0xFF)  # $CnFF: ProDOS entry point

# ==========================================================================
resolve()

print(f"$Cn page: {cn_used} bytes used ({251 - cn_used} free)")
print(f"\nLabels:")
for name, addr in sorted(labels.items(), key=lambda x: x[1]):
    print(f"  {name:24s} = ${addr:04X}")
print(f"\n$CnFE = $17, $CnFF = ${labels['prodos_entry'] & 0xFF:02X}")

with open("hamr_rom.mem", "w") as f:
    for i in range(4096):
        f.write(f"{rom[i]:02X}\n")

print(f"\nWrote hamr_rom.mem (4096 bytes)")
