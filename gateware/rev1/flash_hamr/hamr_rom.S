* =============================================================================
* hamr_rom.S — Flash Hamr Duo-Drive ProDOS/SmartPort ROM (Merlin32)
* =============================================================================
* 2 SmartPort units (Duo Drive): D1 = boot, D2 = data
*
* Register interface ($C0n0 + slot*16):
*   0: STATUS(R) / COMMAND(W)    5: DATA_WRITE(W)
*   1: DATA_READ(R)              6: TOTAL_BLOCKS_LO(R) — D1
*   2: BLOCK_LO(R/W)            7: TOTAL_BLOCKS_HI(R) — D1
*   3: BLOCK_HI(R/W)            B: D2_BLOCKS_LO(R)
*   D: ACTIVE_UNIT(W)           C: D2_BLOCKS_HI(R)
*
* CRITICAL: ZP $42-$47 must NOT be modified — ProDOS reads them back.
* =============================================================================

            TYP   $06
            DSK   hamr_rom.bin

SLOT        =     4
SLOT16      =     SLOT*16
NUM_UNITS   =     2

* ---- Zero page (ProDOS command block — DO NOT MODIFY $42-$47) ----
ZP_CMD      =     $42
ZP_UNIT     =     $43
ZP_BUF      =     $44
ZP_BUF_H    =     $45
ZP_BLK      =     $46
ZP_BLK_H    =     $47
ZP_SLOT     =     $58
ZP_PTR      =     $3C
ZP_PTR_H    =     $3D

* ---- Screen holes ----
SH_PRODOS   =     $0478          ; +slot: MSB=1 ProDOS, 0 SmartPort
SH_SP_RETLO =     $05F8          ; +slot: SmartPort return addr lo
SH_SP_RETHI =     $0678          ; +slot: SmartPort return addr hi
SH_MAGIC1   =     $06F8
SH_MAGIC2   =     $0778
SH_UNITCNT  =     $07F8          ; +slot: unit count

* ---- I/O registers (abs,X with X = slot*16) ----
IO_CMD      =     $C080
IO_DATA     =     $C081
IO_BLLO     =     $C082
IO_BLHI     =     $C083
IO_DATAW    =     $C085
IO_CNTLO    =     $C086
IO_CNTHI    =     $C087
IO_D2BLLO   =     $C08B
IO_D2BLHI   =     $C08C
IO_UNIT     =     $C08D

* ---- Error codes ----
ERR_NONE    =     $00
ERR_BADCMD  =     $01
ERR_IO      =     $27
ERR_NODEV   =     $28

CMD_READ    =     $01
CMD_WRITE   =     $02


* =============================================================================
* $Cn page ($C400-$C4FF)
* =============================================================================
            ORG   $C400

* ---- Signature bytes ----
* $Cn01=$20, $Cn03=$00, $Cn05=$03, $Cn07=$00 (SmartPort!)
            LDX   #$20
            LDX   #$00
            LDX   #$03
            CMP   #$00            ; $Cn07=$00 → SmartPort signature
            BCS   claim_c800      ; carry set = boot

* ---- ProDOS entry ----
prodos_entry
            SEC
            BCS   cn_dispatch

* ---- SmartPort entry (prodos_entry + 3) ----
smartport_entry
            CLC

* ---- Protocol dispatch ----
cn_dispatch
            LDX   #SLOT
            ROR   SH_PRODOS,X    ; carry → bit 7 (1=ProDOS, 0=SmartPort)
            CLC

* ---- Claim $C800 ----
claim_c800
            LDA   #$C0+SLOT
            STA   $07F8
            LDX   #SLOT
            LDA   $CFFF
            JMP   shared_entry

* ---- Return trampolines ----
cn_rts
            PHA
            LDA   $CFFF
            PLA
            RTS

cn_boot_jmp
            LDA   $CFFF
            JMP   $0801

* =============================================================================
* SmartPort STATUS handler — lives in $Cn page (plenty of room)
* =============================================================================
* Called from cmd_status when protocol = SmartPort.
* ZP_BLK ($46) = status code, ZP_BUF ($44-45) = output buffer
* ZP_UNIT ($43) = unit (1-based, NOT modified)
* X = slot*16

sp_status
* ---- Unit 0: device-level STATUS ----
            LDA   ZP_UNIT
            BNE   sp_unit_status

            LDA   ZP_BLK         ; status code
            BNE   sp_bad_ctl

* Fill 8-byte status: byte 0 = unit count, rest = 0
            LDY   #$07
            LDA   #$00
:fill       STA   (ZP_BUF),Y
            DEY
            BNE   :fill
            LDA   #NUM_UNITS
            STA   (ZP_BUF),Y     ; Y=0
            LDX   #$08
            LDY   #$00
            LDA   #ERR_NONE
            CLC
            RTS

sp_bad_ctl  LDA   #$21            ; bad status/control code
            SEC
            RTS

* ---- Unit > 0: per-unit STATUS (2 units: D1 + D2) ----
sp_unit_status
            CMP   #$03            ; A still has ZP_UNIT from sp_status
            BCS   sp_nodev        ; unit 3+ → no device
            CMP   #$02
            BEQ   sp_d2_blk       ; unit 2 → D2

* Unit 1: D1 block count from regs 6/7
            LDA   IO_CNTHI,X
            PHA
            LDA   IO_CNTLO,X
            JMP   sp_got_blk

* Unit 2: D2 block count from regs B/C
sp_d2_blk   LDA   IO_D2BLHI,X
            PHA
            LDA   IO_D2BLLO,X

sp_got_blk  TAX                   ; X = blk_lo
            PLA                   ; A = blk_hi
            TAY                   ; Y = blk_hi

* Check offline (block count = 0)
            TXA
            STY   ZP_PTR          ; temp save blk_hi (safe ZP)
            ORA   ZP_PTR
            BNE   sp_online

sp_nodev    LDA   #ERR_NODEV
            SEC
            RTS

sp_online
* Status code from ZP_BLK (not modified — safe to read)
            LDA   ZP_BLK
            CMP   #$03
            BEQ   sp_dib

* ---- Status code 0: general status (1 byte) ----
            LDY   #$00
            LDA   #$F8            ; block device, online, R/W
            STA   (ZP_BUF),Y
            LDX   #$01
            LDY   #$00
            LDA   #ERR_NONE
            CLC
            RTS

* ---- Status code 3: DIB (25 bytes) ----
sp_dib
* Reload block count into ZP_PTR/ZP_PTR_H for later
            STX   ZP_PTR          ; blk_lo (X still has it)
            STY   ZP_PTR_H        ; blk_hi (Y still has it)

            LDY   #$00
            LDA   #$F8
            STA   (ZP_BUF),Y     ; [0] status
            INY
            LDA   ZP_PTR
            STA   (ZP_BUF),Y     ; [1] blk count lo
            INY
            LDA   ZP_PTR_H
            STA   (ZP_BUF),Y     ; [2] blk count mid
            INY
            LDA   #$00
            STA   (ZP_BUF),Y     ; [3] blk count hi
            INY

* [4-20] Device name (Pascal string: len + 16 chars)
            LDA   #$08
            STA   (ZP_BUF),Y     ; [4] name length
            INY
            LDX   #$00
:name       LDA   dib_name,X
            STA   (ZP_BUF),Y
            INY
            INX
            CPX   #16
            BNE   :name

* [21-24] type, subtype, version
            LDA   #$02
            STA   (ZP_BUF),Y     ; [21] type: block
            INY
            LDA   #$20
            STA   (ZP_BUF),Y     ; [22] subtype: hard disk
            INY
            LDA   #$01
            STA   (ZP_BUF),Y     ; [23] version lo
            INY
            LDA   #$00
            STA   (ZP_BUF),Y     ; [24] version hi

            LDX   #25
            LDY   #$00
            LDA   #ERR_NONE
            CLC
            RTS

* Device name (16 bytes in $Cn page)
dib_name
            ASC   "BYTEHAMR"
            DFB   $20,$20,$20,$20,$20,$20,$20,$20

* ---- Fill to signature area ----
            DS    $C4FB-*,$00

            DFB   $00             ; $CnFB: block device type
            DFB   $00,$00         ; $CnFC-FD
            DFB   $BF             ; $CnFE: removable + status + R/W + SmartPort
            DFB   <prodos_entry   ; $CnFF


* =============================================================================
* $C800 shared ROM
* =============================================================================
            DS    $C800-*,$00

shared_entry
            BCC   execute_command
            JMP   boot

* ---- Command entry ----
execute_command
            CLD
            STX   ZP_SLOT
            JSR   get_slot_x      ; X = slot*16

* Check protocol
            LDY   #SLOT
            LDA   SH_PRODOS,Y
            BMI   prodos_cmd      ; MSB=1 → ProDOS

* =========================================================================
* SmartPort inline parameter parsing
* =========================================================================
            PLA
            STA   SH_SP_RETLO,Y
            CLC
            ADC   #$03
            TAX
            PLA
            STA   SH_SP_RETHI,Y
            ADC   #$00
            PHA
            TXA
            PHA                   ; push adjusted return addr

            LDA   SH_SP_RETLO,Y
            STA   ZP_PTR
            LDA   SH_SP_RETHI,Y
            STA   ZP_PTR_H

            LDY   #$01
            LDA   (ZP_PTR),Y     ; command byte
            STA   ZP_CMD
            INY
            LDA   (ZP_PTR),Y     ; param list lo
            TAX
            INY
            LDA   (ZP_PTR),Y     ; param list hi
            STA   ZP_PTR_H
            STX   ZP_PTR

            LDY   #$00
            LDA   (ZP_PTR),Y     ; param count (skip)
            INY
            LDA   (ZP_PTR),Y     ; unit
            STA   ZP_UNIT
            INY
            LDA   (ZP_PTR),Y     ; buffer lo
            STA   ZP_BUF
            INY
            LDA   (ZP_PTR),Y     ; buffer hi
            STA   ZP_BUF_H
            INY
            LDA   (ZP_PTR),Y     ; block lo
            STA   ZP_BLK
            INY
            LDA   (ZP_PTR),Y     ; block hi
            STA   ZP_BLK_H

            JSR   get_slot_x
            JMP   dispatch_cmd

* =========================================================================
* ProDOS command path — NO modification of $42-$47!
* =========================================================================
prodos_cmd
            JMP   dispatch_cmd    ; straight through, $43 untouched

* =========================================================================
* Command dispatcher
* =========================================================================
dispatch_cmd
            LDA   ZP_CMD
            BEQ   cmd_status
            CMP   #CMD_READ
            BEQ   cmd_read
            CMP   #CMD_WRITE
            BEQ   jmp_write
            CMP   #$05
            BEQ   jmp_init
            SEC
            LDA   #ERR_BADCMD
            JMP   cn_rts

jmp_write   JMP   cmd_write
jmp_init    JMP   cmd_init


* =========================================================================
* STATUS (cmd $00)
* =========================================================================
cmd_status
            LDA   IO_CMD,X
            AND   #$01
            BEQ   status_offline

* Check protocol
            LDY   ZP_SLOT
            LDA   SH_PRODOS,Y
            BPL   sp_status_jmp   ; MSB=0 → SmartPort

* ---- ProDOS STATUS: check $43 bit7 for D1/D2 ----
            LDA   ZP_UNIT
            BMI   :pd_d2
            LDA   IO_CNTHI,X     ; D1 block count from regs 6/7
            TAY
            LDA   IO_CNTLO,X
            TAX
            LDA   #ERR_NONE
            CLC
            JMP   cn_rts
:pd_d2      LDA   IO_D2BLHI,X   ; D2 block count from regs B/C
            ORA   IO_D2BLLO,X   ; check if both bytes are 0
            BNE   :pd_d2_ok
            LDA   #ERR_NODEV    ; D2 not mounted → no device
            SEC
            JMP   cn_rts
:pd_d2_ok   LDA   IO_D2BLHI,X
            TAY
            LDA   IO_D2BLLO,X
            TAX
            LDA   #ERR_NONE
            CLC
            JMP   cn_rts

sp_status_jmp
            JMP   sp_status       ; → handler in $Cn page

status_offline
            LDX   #$00
            LDY   #$00
            LDA   #ERR_NODEV
            SEC
            JMP   cn_rts


* =========================================================================
* READ BLOCK (cmd $01)
* =========================================================================
cmd_read
            JSR   set_unit_block
            LDA   #CMD_READ
            STA   IO_CMD,X

            JSR   wait_ready
            BCS   read_done

            LDY   #$00
:rd1        LDA   IO_DATA,X
            STA   (ZP_BUF),Y
            INY
            BNE   :rd1
            INC   ZP_BUF_H
            LDY   #$00
:rd2        LDA   IO_DATA,X
            STA   (ZP_BUF),Y
            INY
            BNE   :rd2
            DEC   ZP_BUF_H
            LDA   #ERR_NONE
            CLC
read_done   JMP   cn_rts


* =========================================================================
* WRITE BLOCK (cmd $02)
* =========================================================================
cmd_write
            JSR   set_unit_block
            LDY   #$00
:wr1        LDA   (ZP_BUF),Y
            STA   IO_DATAW,X
            INY
            BNE   :wr1
            INC   ZP_BUF_H
            LDY   #$00
:wr2        LDA   (ZP_BUF),Y
            STA   IO_DATAW,X
            INY
            BNE   :wr2
            DEC   ZP_BUF_H
            LDA   #CMD_WRITE
            STA   IO_CMD,X
            JSR   wait_ready
            BCS   write_done
            LDA   #ERR_NONE
            CLC
write_done  JMP   cn_rts


* =========================================================================
* INIT (cmd $05)
* =========================================================================
cmd_init
            LDY   #SLOT
            LDA   #NUM_UNITS
            STA   SH_UNITCNT,Y
            LDA   #$A5
            STA   SH_MAGIC1,Y
            EOR   #$FF
            STA   SH_MAGIC2,Y
            LDA   #ERR_NONE
            CLC
            JMP   cn_rts


* =========================================================================
* set_unit_block — write active_unit + block to hardware
* =========================================================================
* Duo Drive: derives active_unit from ZP_UNIT WITHOUT modifying $43.
* ProDOS: $40=D1→unit0, $C0=D2→unit1
* SmartPort: unit 1-based, unit-1 → active_unit (0 or 1)

set_unit_block
            LDY   ZP_SLOT
            LDA   SH_PRODOS,Y
            BMI   :prodos

* SmartPort path: unit-1 → active_unit
            LDA   ZP_UNIT
            SEC
            SBC   #$01
            JMP   :write_unit

* ProDOS path: bit7 of $43 → 0=D1, 1=D2
:prodos     LDA   ZP_UNIT
            ASL                   ; bit7 → carry
            LDA   #$00
            ADC   #$00            ; 0(D1) or 1(D2)

:write_unit STA   IO_UNIT,X       ; write active_unit
            LDA   ZP_BLK
            STA   IO_BLLO,X
            LDA   ZP_BLK_H
            STA   IO_BLHI,X
            RTS


* =========================================================================
* wait_ready — extended timeout (~30s) for persist-blocked reads
* =========================================================================
* Uses stack for the 3rd loop level (NO ZP — $3C is ProDOS territory).
* 32 * 256 * 256 = 2M iterations ≈ 30 seconds @ 1MHz

wait_ready
            LDA   #$20           ; outer-outer: 32 iterations
            PHA                   ; store on stack (not ZP!)
:oo_loop    LDY   #$00
:outer      LDA   #$00
:inner      PHA
            LDA   IO_CMD,X
            BMI   :ready
            PLA
            SEC
            SBC   #1
            BNE   :inner
            DEY
            BNE   :outer
            PLA                   ; pop oo counter
            SEC
            SBC   #1
            BEQ   :timeout
            PHA
            BNE   :oo_loop       ; always taken (A>0)
:timeout    LDA   #ERR_IO
            SEC
            RTS
:ready      AND   #$40
            BNE   :error
            PLA                   ; pop inner PHA
            PLA                   ; pop oo counter
            LDA   #ERR_NONE
            CLC
            RTS
:error      PLA                   ; pop inner PHA
            PLA                   ; pop oo counter
            LDA   #ERR_IO
            SEC
            RTS


* =========================================================================
* get_slot_x
* =========================================================================
get_slot_x
            LDA   ZP_SLOT
            ASL
            ASL
            ASL
            ASL
            TAX
            RTS


* =========================================================================
* Boot handler
* =========================================================================
boot
            STX   ZP_SLOT
            LDY   #SLOT
            LDA   #$AA
            STA   SH_PRODOS,Y

            JSR   get_slot_x

* Always boot from D1: active_unit = 0, $43 = $40
            LDA   #$00
            STA   IO_UNIT,X      ; active_unit = 0 (D1)
            LDA   #SLOT*16       ; $40 = slot 4, D1
            STA   ZP_UNIT        ; $43 — boot block code reads this

            LDA   #$00
            STA   IO_BLLO,X
            STA   IO_BLHI,X
            LDA   #CMD_READ
            STA   IO_CMD,X

            JSR   wait_ready
            BCS   boot_error

            LDY   #$00
:cp1        LDA   IO_DATA,X
            STA   $0800,Y
            INY
            BNE   :cp1
:cp2        LDA   IO_DATA,X
            STA   $0900,Y
            INY
            BNE   :cp2

            LDA   $0800
            CMP   #$01
            BNE   boot_error
            LDA   $0801
            BEQ   boot_error

            JSR   get_slot_x
            JMP   cn_boot_jmp

boot_error
            SEC
            RTS


* ---- ID string ----
            DS    $CFDB-*,$00
            ASC   "FLASH HAMR"
            DFB   $00
