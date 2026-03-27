#!/usr/bin/env python3
"""
assemble_rom.py — Byte Hamr ProDOS block device ROM
Modeled after Apple2-IO-RPi's DriveFirmware.asm:
  - Entire driver fits in $Cn00-$CnFF (256 bytes, no $C800)
  - No ZP save/restore (ProDOS doesn't require it for block devices)
  - No $CFFF handling (no shared ROM to deactivate)
  - No trampoline (no $C800 to return from)
  - Simple: set registers, transfer data, RTS
"""

rom = bytearray(4096)
rom[:] = b'\xff' * 4096

SLOT = 4

# Zero page (ProDOS command block)
Command  = 0x42
Unit     = 0x43
BufferLo = 0x44
BufferHi = 0x45
BlockLo  = 0x46
BlockHi  = 0x47

# I/O registers (indexed by slot*16)
REG_CMD    = 0xC080  # +X: STATUS read / COMMAND write
REG_DATA   = 0xC081  # +X: DATA read (auto-increment)
REG_DATA_WR= 0xC085  # +X: DATA write (separate to avoid STA dummy-read bug)
REG_BLK_LO = 0xC082  # +X: block number lo
REG_BLK_HI = 0xC083  # +X: block number hi

# Error codes / commands
ERR_NONE    = 0x00
ERR_BAD_CMD = 0x01
ERR_IO      = 0x27
ERR_NO_DEV  = 0x28
CMD_READ    = 0x01
CMD_WRITE   = 0x02
BLK_CNT_LO  = 0x18  # 280 blocks
BLK_CNT_HI  = 0x01

# =========================================================================
# Simple assembler (label-resolving, single-pass with fixups)
# =========================================================================
class Asm:
    def __init__(self):
        self.pc = 0
        self.code = bytearray()
        self.labels = {}
        self.fixups = []
        self.base = 0

    def org(self, addr):
        self.pc = addr
        self.base = addr

    def label(self, name):
        self.labels[name] = self.pc

    def _emit(self, *bytes_):
        for b in bytes_:
            self.code.append(b & 0xFF)
            self.pc += 1

    def db(self, *bytes_):
        for b in bytes_: self._emit(b)

    def pad_to(self, addr):
        while self.pc < addr: self._emit(0x00)

    def _branch(self, op, target):
        self._emit(op, 0x00)
        self.fixups.append((len(self.code)-1, self.pc, target, True))

    def _abs(self, op, target):
        if isinstance(target, str):
            self._emit(op, 0x00, 0x00)
            self.fixups.append((len(self.code)-2, self.pc, target, False))
        else:
            self._emit(op, target & 0xFF, target >> 8)

    def BEQ(self, t): self._branch(0xF0, t)
    def BNE(self, t): self._branch(0xD0, t)
    def BCS(self, t): self._branch(0xB0, t)
    def BMI(self, t): self._branch(0x30, t)
    def BPL(self, t): self._branch(0x10, t)
    def JSR(self, t): self._abs(0x20, t)
    def JMP(self, t): self._abs(0x4C, t)

    def LDA_imm(self, v): self._emit(0xA9, v)
    def LDX_imm(self, v): self._emit(0xA2, v)
    def LDY_imm(self, v): self._emit(0xA0, v)
    def CPX_imm(self, v): self._emit(0xE0, v)
    def CMP_imm(self, v): self._emit(0xC9, v)
    def LDA_zp(self, a): self._emit(0xA5, a)
    def STA_zp(self, a): self._emit(0x85, a)
    def STX_zp(self, a): self._emit(0x86, a)
    def INC_zp(self, a): self._emit(0xE6, a)
    def DEC_zp(self, a): self._emit(0xC6, a)
    def LDA_iny(self, a): self._emit(0xB1, a)
    def STA_iny(self, a): self._emit(0x91, a)
    def LDA_abx(self, a): self._emit(0xBD, a & 0xFF, a >> 8)
    def STA_abx(self, a): self._emit(0x9D, a & 0xFF, a >> 8)
    def LDA_abs(self, a): self._emit(0xAD, a & 0xFF, a >> 8)
    def CLC(self): self._emit(0x18)
    def SEC(self): self._emit(0x38)
    def RTS(self): self._emit(0x60)
    def INY(self): self._emit(0xC8)
    def DEY(self): self._emit(0x88)
    def TAX(self): self._emit(0xAA)
    def NOP(self): self._emit(0xEA)

    def resolve(self):
        for code_off, branch_pc, lbl, is_rel in self.fixups:
            target = self.labels[lbl]
            if is_rel:
                diff = target - branch_pc
                assert -128 <= diff <= 127, f"Branch {lbl} out of range: {diff}"
                self.code[code_off] = diff & 0xFF
            else:
                self.code[code_off] = target & 0xFF
                self.code[code_off+1] = (target >> 8) & 0xFF

    def place(self, rom_array):
        offset = self.base - 0xC000
        for i, b in enumerate(self.code):
            rom_array[offset + i] = b

# =========================================================================
# Build the driver — everything in $C400-$C4FF
# =========================================================================
a = Asm()
a.org(0xC400)

# ---- ID bytes ($Cn00-$Cn07) ----
a.CPX_imm(0x20)       # $Cn01 = $20 (ProDOS block device)
a.CPX_imm(0x00)       # $Cn03 = $00
a.CPX_imm(0x03)       # $Cn05 = $03
a.CPX_imm(0x3C)       # $Cn07 = $3C (Autostart ROM, NOT $00 = not SmartPort)

# ---- Boot entry ($Cn08) ----
a.LDA_imm(SLOT * 0x10)
a.JMP('boot')

# ---- ProDOS driver entry ($Cn0D) ----
a.label('driver_entry')
a.LDX_imm(SLOT * 0x10)  # X = slot*16 for register access
a.JMP('driver')

# ---- Boot handler ----
a.label('boot')
a.TAX()                  # X = slot*16 (was loaded into A at $Cn08)
a.STX_zp(Unit)
a.LDA_imm(CMD_READ)
a.STA_zp(Command)
a.LDA_imm(0x00)
a.STA_zp(BlockLo)
a.STA_zp(BlockHi)
a.STA_zp(BufferLo)
a.LDA_imm(0x08)
a.STA_zp(BufferHi)
a.JSR('driver')          # read block 0 to $0800
a.JMP(0x0801)            # execute boot block

# ---- ProDOS Driver ----
a.label('driver')
a.LDA_abs(0xCFFF)        # Release $C800 ownership (Apple II bus latch)
a.LDA_zp(Command)
a.BEQ('cmd_status')
a.CMP_imm(CMD_READ)
a.BEQ('cmd_read')
a.CMP_imm(CMD_WRITE)
a.BEQ('cmd_write')
a.SEC()
a.LDA_imm(ERR_BAD_CMD)
a.RTS()

# ---- STATUS ----
a.label('cmd_status')
a.LDX_imm(BLK_CNT_LO)
a.LDY_imm(BLK_CNT_HI)
a.LDA_imm(ERR_NONE)
a.CLC()
a.RTS()

# ---- READ BLOCK ----
a.label('cmd_read')
a.LDA_zp(BlockLo)
a.STA_abx(REG_BLK_LO)
a.LDA_zp(BlockHi)
a.STA_abx(REG_BLK_HI)
a.LDA_imm(CMD_READ)
a.STA_abx(REG_CMD)
a.JSR('wait_ready')
a.BCS('read_err')
a.LDY_imm(0x00)
a.label('read_p1')
a.LDA_abx(REG_DATA)
a.STA_iny(BufferLo)
a.INY()
a.BNE('read_p1')
a.INC_zp(BufferHi)
a.JSR('read256')
a.DEC_zp(BufferHi)
a.LDA_imm(ERR_NONE)
a.CLC()
a.label('read_err')
a.RTS()

a.label('read256')
a.LDY_imm(0x00)
a.label('read_p2')
a.LDA_abx(REG_DATA)
a.STA_iny(BufferLo)
a.INY()
a.BNE('read_p2')
a.RTS()

# ---- WRITE BLOCK ----
a.label('cmd_write')
a.LDA_zp(BlockLo)
a.STA_abx(REG_BLK_LO)
a.LDA_zp(BlockHi)
a.STA_abx(REG_BLK_HI)
a.LDY_imm(0x00)
a.label('write_p1')
a.LDA_iny(BufferLo)
a.STA_abx(REG_DATA_WR)
a.INY()
a.BNE('write_p1')
a.INC_zp(BufferHi)
a.JSR('write256')
a.DEC_zp(BufferHi)
# Issue WRITE command — triggers BRAM→SDRAM transfer in arbiter
a.LDA_imm(CMD_WRITE)
a.STA_abx(REG_CMD)
a.JSR('wait_ready')
a.BCS('write_err')
a.LDA_imm(ERR_NONE)
a.CLC()
a.label('write_err')
a.RTS()

a.label('write256')
a.LDY_imm(0x00)
a.label('write_p2')
a.LDA_iny(BufferLo)
a.STA_abx(REG_DATA_WR)
a.INY()
a.BNE('write_p2')
a.RTS()

# ---- wait_ready: poll STATUS until ready or timeout ----
a.label('wait_ready')
a.LDY_imm(0x00)
a.label('wr_outer')
a.LDA_abx(REG_CMD)
a.BMI('wr_done')
a.DEY()
a.BNE('wr_outer')
# Timeout
a.SEC()
a.LDA_imm(ERR_IO)
a.RTS()
a.label('wr_done')
a.CLC()
a.LDA_imm(ERR_NONE)
a.RTS()

# ---- Pad to signature area ----
a.pad_to(0xC4FB)
a.db(0x00)               # $CnFB: device type
a.db(0x00, 0x00)         # $CnFC-FD: block count (use STATUS)
a.db(0x17)               # $CnFE: characteristics (read+write+status, 1 vol)
a.db(a.labels['driver_entry'] & 0xFF)  # $CnFF: ProDOS entry offset

# ---- Resolve and place ----
a.resolve()
a.place(rom)

# Report
used = a.pc - a.base
print(f"Driver: {used} bytes used out of 256 ({256-used} free)")
print(f"Labels:")
for name, addr in sorted(a.labels.items(), key=lambda x: x[1]):
    print(f"  {name:20s} = ${addr:04X}")
print(f"\n$CnFE = $17, $CnFF = ${a.labels['driver_entry'] & 0xFF:02X}")
print(f"Output: liron_rom.mem")

with open("liron_rom.mem", "w") as f:
    for i in range(4096):
        f.write(f"{rom[i]:02X}\n")
