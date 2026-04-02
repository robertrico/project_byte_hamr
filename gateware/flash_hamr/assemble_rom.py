#!/usr/bin/env python3
"""
assemble_rom.py — Flash Hamr 2-device ProDOS/SmartPort ROM

S4D1 (unit 1): Menu volume from SPI flash
S4D2 (unit 2): SD card image (UNIT2_OFFSET=2048 blocks)

$Cn page: boot/ProDOS/SmartPort entry, signature
$C800: dispatch, STATUS, READ, WRITE, INIT, wait_ready, boot handler
"""

SLOT = 4

# Zero page
ZP_CMD    = 0x42
ZP_UNIT   = 0x43
ZP_BUF    = 0x44
ZP_BUF_H  = 0x45
ZP_BLK    = 0x46
ZP_BLK_H  = 0x47
ZP_SLOT   = 0x58
ZP_TEMP   = 0x3A

# Screen holes
SH_PFLAG  = 0x0478  # +slot: MSB=1 ProDOS, 0=SmartPort
SH_UCNT   = 0x07F8  # +slot: unit count
SH_SPLO   = 0x05F8  # +slot: SP return lo
SH_SPHI   = 0x0678  # +slot: SP return hi

# I/O (indexed by X = slot*16)
IO_CMD    = 0xC080
IO_DATA   = 0xC081
IO_BLLO   = 0xC082
IO_BLHI   = 0xC083
IO_DATAW  = 0xC085
IO_CNTLO  = 0xC086
IO_CNTHI  = 0xC087
IO_SDSTAT = 0xC088
IO_D2BLLO = 0xC08B
IO_D2BLHI = 0xC08C

ERR_NONE = 0x00; ERR_IO = 0x27; ERR_NODEV = 0x28; ERR_BADCMD = 0x01
CMD_READ = 0x01; CMD_WRITE = 0x02
U2_LO = 0x00; U2_HI = 0x08  # UNIT2_OFFSET = 2048
D1_BLLO = 0x18; D1_BLHI = 0x01  # 280 blocks

rom = bytearray(4096)
rom[:] = b'\xFF' * 4096

def w8(addr, val):
    rom[addr - 0xC000] = val & 0xFF

def w16(addr, lo, hi):
    rom[addr - 0xC000] = lo & 0xFF
    rom[addr - 0xC000 + 1] = hi & 0xFF

# Simple assembler with raw byte emission
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
def CMP_i(v): imm(0xC9, v)
def ADC_i(v): imm(0x69, v)
def SBC_i(v): imm(0xE9, v)
def AND_i(v): imm(0x29, v)
def EOR_i(v): imm(0x49, v)
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
def BPL(n): branch(0x10, n)
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
def NOP(): emit(0xEA)

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
# $Cn page
# ==========================================================================
org(0xC400)

# ID bytes
LDX_i(0x20)        # $Cn01 = $20
LDX_i(0x00)        # $Cn03 = $00
LDX_i(0x03)        # $Cn05 = $03
CMP_i(0x00)        # $Cn07 = $00
BCS('cn14')        # carry set = boot request

# ProDOS entry ($Cn09)
label('prodos_entry')
SEC()
BCS('cn0e')

# SmartPort entry ($Cn0C = prodos_entry + 3)
label('sp_entry')
CLC()

# Common entry
label('cn0e')
LDX_i(SLOT)
# ROR SH_PFLAG,X — rotate carry into MSB
emit(0x7E, SH_PFLAG & 0xFF, (SH_PFLAG >> 8) & 0xFF)  # ROR abs,X
CLC()

label('cn14')
LDA_i(0xC0 + SLOT)
STA_a(0x07F8)      # claim $C800
LDX_i(SLOT)
LDA_a(0xCFFF)      # deactivate other ROMs
JMP_l('shared_entry')

# ---- Trampolines in $Cn page (always served, no expansion flag needed) ----
# $C800 code jumps here to release $C800 before returning

label('cn_rts_trampoline')
LDA_a(0xCFFF)      # clear expansion flag
RTS()              # return to ProDOS/caller

label('cn_boot_trampoline')
LDA_a(0xCFFF)      # clear expansion flag
JMP_a(0x0801)      # jump to boot code

# Pad and signature
while here() < 0xC4FB: emit(0x00)
emit(0x00)         # $CnFB
emit(0x00, 0x00)   # $CnFC-FD
emit(0x17)         # $CnFE: status+rw+smartport (same as Block Hamr)
emit(labels['prodos_entry'] & 0xFF)  # $CnFF

# ==========================================================================
# $C800 shared ROM
# ==========================================================================
org(0xC800)

label('shared_entry')
BCC('execute_command')
JMP_l('boot')

label('execute_command')
CLD()
STX_z(ZP_SLOT)
JSR_l('get_slot_x')

LDY_i(SLOT)
LDA_ay(SH_PFLAG)
BMI('prodos_cmd')

# ---- SmartPort inline param decode ----
PLA()
STA_ay(SH_SPLO)
CLC(); ADC_i(3); TAX()
PLA()
STA_ay(SH_SPHI)
ADC_i(0); PHA(); TXA(); PHA()

LDA_ay(SH_SPLO); STA_z(ZP_BUF)
LDA_ay(SH_SPHI); STA_z(ZP_BUF_H)

LDY_i(1)
LDA_iy(ZP_BUF);  STA_z(ZP_CMD)   # command
INY()
LDA_iy(ZP_BUF);  TAX()            # param list lo
INY()
LDA_iy(ZP_BUF);  STA_z(ZP_BUF_H) # param list hi
STX_z(ZP_BUF)                     # param list lo

# Read from param list
LDY_i(1); LDA_iy(ZP_BUF); PHA()   # unit → stack
INY(); LDA_iy(ZP_BUF); TAX()      # buf lo → X
INY(); LDA_iy(ZP_BUF); PHA()      # buf hi → stack
INY(); LDA_iy(ZP_BUF); STA_z(ZP_BLK)   # block lo
INY(); LDA_iy(ZP_BUF); STA_z(ZP_BLK_H) # block hi

PLA(); STA_z(ZP_BUF_H)            # buf hi
STX_z(ZP_BUF)                     # buf lo
PLA(); STA_z(ZP_UNIT)             # unit

# ---- ProDOS dispatch ----
label('prodos_cmd')
JSR_l('get_slot_x')

LDA_z(ZP_CMD)
BEQ('cmd_status')
CMP_i(CMD_READ)
BEQ('cmd_read')
CMP_i(CMD_WRITE)
BNE('not_write')
JMP_l('cmd_write')
label('not_write')
CMP_i(0x05)
BNE('not_init')
JMP_l('cmd_init')
label('not_init')

LDA_i(ERR_BADCMD); SEC()
JMP_l('cmd_done')

# ---- STATUS ----
label('cmd_status')
LDA_ax(IO_CMD); AND_i(0x01)
BEQ('status_offline')
LDA_z(ZP_UNIT); CMP_i(2)
BEQ('status_u2')
# Unit 1
LDX_i(D1_BLLO); LDY_i(D1_BLHI)
LDA_i(ERR_NONE); CLC()
JMP_l('cmd_done')
label('status_u2')
JSR_l('get_slot_x')
LDA_ax(IO_D2BLLO); PHA()
LDA_ax(IO_D2BLHI); TAY()
PLA(); TAX()
LDA_i(ERR_NONE); CLC()
JMP_l('cmd_done')
label('status_offline')
LDA_i(ERR_NODEV); SEC()
JMP_l('cmd_done')

# ---- Unit offset helper ----
label('apply_offset')
LDA_z(ZP_UNIT); CMP_i(2); BNE('ao_done')
CLC()
LDA_z(ZP_BLK); ADC_i(U2_LO); STA_z(ZP_BLK)
LDA_z(ZP_BLK_H); ADC_i(U2_HI); STA_z(ZP_BLK_H)
label('ao_done')
RTS()

# ---- READ ----
label('cmd_read')
JSR_l('apply_offset')
JSR_l('get_slot_x')
LDA_z(ZP_BLK);   STA_ax(IO_BLLO)
LDA_z(ZP_BLK_H); STA_ax(IO_BLHI)
LDA_i(CMD_READ);  STA_ax(IO_CMD)
JSR_l('wait_ready')
BCS('cmd_done')
LDY_i(0x00)
label('rd1')
LDA_ax(IO_DATA); STA_iy(ZP_BUF); INY(); BNE('rd1')
INC_z(ZP_BUF_H)
LDY_i(0x00)
label('rd2')
LDA_ax(IO_DATA); STA_iy(ZP_BUF); INY(); BNE('rd2')
DEC_z(ZP_BUF_H)
LDA_i(ERR_NONE); CLC()
JMP_l('cmd_done')

# ---- WRITE ----
label('cmd_write')
JSR_l('apply_offset')
JSR_l('get_slot_x')
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
BCS('cmd_done')
LDA_i(ERR_NONE); CLC()
JMP_l('cmd_done')

# ---- INIT ----
label('cmd_init')
LDY_i(SLOT)
LDA_i(2); STA_ay(SH_UCNT)  # 2 units
LDA_i(ERR_NONE); CLC()
# fall through

label('cmd_done')
JMP_l('cn_rts_trampoline')  # release $C800 + RTS (in $Cn page)

# ---- wait_ready ----
label('wait_ready')
LDY_i(0x00)
label('wr_out')
LDA_i(0x00)
label('wr_in')
PHA()
LDA_ax(IO_CMD); BMI('wr_ok')
PLA(); SEC(); SBC_i(1); BNE('wr_in')
DEY(); BNE('wr_out')
PLA(); LDA_i(ERR_IO); SEC(); RTS()
label('wr_ok')
AND_i(0x40); BNE('wr_err')
PLA(); LDA_i(ERR_NONE); CLC(); RTS()
label('wr_err')
PLA(); LDA_i(ERR_IO); SEC(); RTS()

# ---- get_slot_x ----
label('get_slot_x')
LDA_i(SLOT)
ASL_A(); ASL_A(); ASL_A(); ASL_A()
TAX(); RTS()

# ---- Boot ----
label('boot')
STX_z(ZP_SLOT)
LDY_i(SLOT)
LDA_i(0xAA); STA_ay(SH_PFLAG)  # ProDOS mode

JSR_l('get_slot_x')
LDA_i(0x00); STA_ax(IO_BLLO); STA_ax(IO_BLHI)
LDA_i(CMD_READ); STA_ax(IO_CMD)
JSR_l('wait_ready'); BCS('boot_err')

LDY_i(0x00)
label('bc1')
LDA_ax(IO_DATA); STA_ay(0x0800); INY(); BNE('bc1')
label('bc2')
LDA_ax(IO_DATA); STA_ay(0x0900); INY(); BNE('bc2')

# Verify boot block
LDA_a(0x0800); CMP_i(0x01); BNE('boot_err')
LDA_a(0x0801); BEQ('boot_err')

# Init
LDY_i(SLOT)
LDA_i(2); STA_ay(SH_UCNT)  # 2 units
JSR_l('get_slot_x')
JMP_l('cn_boot_trampoline')  # release $C800 + JMP $0801 (in $Cn page)

label('boot_err')
SEC(); RTS()

# ---- ID string ----
while here() < 0xCFDB: emit(0x00)
label('id_string')
for c in "FLASH HAMR": emit(ord(c))
emit(0x00)

# ==========================================================================
resolve()

c8_used = labels.get('id_string', here()) - 0xC800
print(f"$C800 code: {c8_used} bytes ({2048 - c8_used} free)")
print(f"\nLabels:")
for name, addr in sorted(labels.items(), key=lambda x: x[1]):
    print(f"  {name:24s} = ${addr:04X}")
print(f"\n$CnFE = $BF, $CnFF = ${labels['prodos_entry'] & 0xFF:02X}")

with open("hamr_rom.mem", "w") as f:
    for i in range(4096):
        f.write(f"{rom[i]:02X}\n")

print(f"\nWrote hamr_rom.mem (4096 bytes)")
