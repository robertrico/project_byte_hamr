#!/usr/bin/env python3
"""
assemble_rom.py — Flash Hamr ProDOS block device ROM

$Cn page: signature, entry points, trampolines (~40 bytes)
$C800: full driver (dispatch, read, write, status, boot, wait_ready)

The addr_decoder.v clears rom_expansion_active when another slot
accesses $C800, preventing bus contention with the Apple IIe's
internal ROM (80-col firmware at slot 3, etc.).

Register interface ($C080 + slot*16):
  0: STATUS(R) / COMMAND(W)
  1: DATA_READ(R) auto-increment
  2: BLOCK_LO(R/W)
  3: BLOCK_HI(R/W)
  5: DATA_WRITE(W) auto-increment
  6: TOTAL_BLOCKS_LO(R)
  7: TOTAL_BLOCKS_HI(R)
"""

SLOT = 4

ZP_CMD    = 0x42
ZP_UNIT   = 0x43
ZP_BUF    = 0x44
ZP_BUF_H  = 0x45
ZP_BLK    = 0x46
ZP_BLK_H  = 0x47
ZP_TEMP   = 0x3A

IO_CMD    = 0xC080
IO_DATA   = 0xC081
IO_BLLO   = 0xC082
IO_BLHI   = 0xC083
IO_DATAW  = 0xC085
IO_CNTLO  = 0xC086
IO_CNTHI  = 0xC087

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

def LDA_i(v): imm(0xA9, v)
def LDX_i(v): imm(0xA2, v)
def LDY_i(v): imm(0xA0, v)
def CPX_i(v): imm(0xE0, v)
def CMP_i(v): imm(0xC9, v)
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
# $Cn page — thin wrapper, jumps to $C800 for real work
# ==========================================================================
org(0xC400)

# Signature bytes
CPX_i(0x20)        # $Cn01 = $20: block device
CPX_i(0x00)        # $Cn03 = $00
CPX_i(0x03)        # $Cn05 = $03
CPX_i(SLOT << 4)   # $Cn07 = $40

# Boot entry (carry set from Apple II ROM scan)
LDA_i(0xC0 + SLOT)
STA_a(0x07F8)      # claim $C800
LDX_i(SLOT)
LDA_a(0xCFFF)      # deactivate other $C800 ROMs
JMP_l('boot')      # → $C800

# ProDOS entry ($CnFF points here)
label('prodos_entry')
LDA_i(0xC0 + SLOT)
STA_a(0x07F8)      # claim $C800
LDX_i(SLOT << 4)   # X = slot*16
LDA_a(0xCFFF)      # deactivate other $C800 ROMs
JMP_l('dispatch')   # → $C800

# ---- Trampolines: $C800 code returns here to release expansion ROM ----
label('cn_rts')
LDA_a(0xCFFF)      # deactivate $C800
RTS()

label('cn_boot_jmp')
LDA_a(0xCFFF)      # deactivate $C800
JMP_a(0x0801)

# Pad and signature
cn_used = here() - 0xC400
while here() < 0xC4FB: emit(0x00)
emit(0x00)         # $CnFB
emit(0x00, 0x00)   # $CnFC-FD
emit(0x17)         # $CnFE
emit(labels['prodos_entry'] & 0xFF)  # $CnFF

# ==========================================================================
# $C800 shared ROM — full driver
# ==========================================================================
org(0xC800)

# ---- Command dispatch (ProDOS entry) ----
label('dispatch')
LDA_z(ZP_CMD)
BEQ('cmd_status')
CMP_i(CMD_READ)
BEQ('cmd_read')
CMP_i(CMD_WRITE)
BNE('bad_cmd')
JMP_l('cmd_write')
label('bad_cmd')
SEC(); LDA_i(ERR_BADCMD)
JMP_l('cn_rts')

# ---- STATUS ----
label('cmd_status')
LDA_ax(IO_CMD); AND_i(0x01)
BEQ('status_offline')
LDA_ax(IO_CNTHI); TAY()
LDA_ax(IO_CNTLO); TAX()
LDA_i(ERR_NONE); CLC()
JMP_l('cn_rts')
label('status_offline')
LDA_i(ERR_NODEV); SEC()
JMP_l('cn_rts')

# ---- READ ----
label('cmd_read')
LDA_z(ZP_BLK);   STA_ax(IO_BLLO)
LDA_z(ZP_BLK_H); STA_ax(IO_BLHI)
LDA_i(CMD_READ);  STA_ax(IO_CMD)
JSR_l('wait_ready')
BCS('read_err')
LDY_i(0x00)
label('rd1')
LDA_ax(IO_DATA); STA_iy(ZP_BUF); INY(); BNE('rd1')
INC_z(ZP_BUF_H)
LDY_i(0x00)
label('rd2')
LDA_ax(IO_DATA); STA_iy(ZP_BUF); INY(); BNE('rd2')
DEC_z(ZP_BUF_H)
LDA_i(ERR_NONE); CLC()
JMP_l('cn_rts')
label('read_err')
JMP_l('cn_rts')

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
BCS('write_err')
LDA_i(ERR_NONE); CLC()
JMP_l('cn_rts')
label('write_err')
JMP_l('cn_rts')

# ---- wait_ready ----
label('wait_ready')
LDY_i(0x00)
label('wro')
LDA_i(0x00)
label('wri')
PHA()
LDA_ax(IO_CMD); BMI('wr_ok')
PLA(); SEC(); SBC_i(1); BNE('wri')
DEY(); BNE('wro')
PLA(); LDA_i(ERR_IO); SEC(); RTS()
label('wr_ok')
AND_i(0x40); BNE('wr_err')
PLA(); LDA_i(ERR_NONE); CLC(); RTS()
label('wr_err')
PLA(); LDA_i(ERR_IO); SEC(); RTS()

# ---- Boot handler ----
label('boot')
LDA_i(SLOT << 4); TAX()
STX_z(ZP_UNIT)
LDA_i(CMD_READ); STA_z(ZP_CMD)
LDA_i(0x00); STA_z(ZP_BLK); STA_z(ZP_BLK_H); STA_z(ZP_BUF)
LDA_i(0x08); STA_z(ZP_BUF_H)
# Read block 0 via dispatch (reuses cmd_read)
JSR_l('dispatch')
JMP_l('cn_boot_jmp')

# ---- ID string ----
while here() < 0xCFDB: emit(0x00)
for c in "FLASH HAMR": emit(ord(c))
emit(0x00)

# ==========================================================================
resolve()

c8_used = here() - 0xC800
print(f"$Cn page: {cn_used} bytes, $C800: {c8_used} bytes ({2048 - c8_used} free)")
print(f"\nLabels:")
for name, addr in sorted(labels.items(), key=lambda x: x[1]):
    print(f"  {name:24s} = ${addr:04X}")
print(f"\n$CnFE = $17, $CnFF = ${labels['prodos_entry'] & 0xFF:02X}")

with open("hamr_rom.mem", "w") as f:
    for i in range(4096):
        f.write(f"{rom[i]:02X}\n")

print(f"\nWrote hamr_rom.mem (4096 bytes)")
