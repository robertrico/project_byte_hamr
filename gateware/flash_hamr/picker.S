* =============================================================================
* picker.S — Byte Hamr Drive Manager (65C02, Merlin32)
* =============================================================================
* ProDOS SYS file at $2000.  Multi-slot drive manager:
*   M) Mount SD image to S4D2/S4D3/S4D4
*   U) Unmount drive slot
*   B) Boot from slot 1-4
*   Q) Quit to ProDOS
* =============================================================================

            XC                    ; 65C02 mode
            TYP   $06
            DSK   picker.bin
            ORG   $2000

SLOT        =     4
IO          =     $C080+SLOT*16  ; $C0C0

* ---- I/O registers ----
R_CMD       =     IO+0
R_STATUS    =     IO+0
R_DATA      =     IO+1
R_BLKLO     =     IO+2
R_BLKHI     =     IO+3
R_SD_STAT   =     IO+8
R_SD_CMD    =     IO+8
R_SELECT    =     IO+9           ; W: {slot[7:6], 2'b0, idx[3:0]}
R_UBLKLO    =     IO+$B
R_UBLKHI    =     IO+$C
R_UNIT      =     IO+$D          ; W: active_unit / R: boot_unit
R_BUNIT     =     IO+$E          ; W: boot_unit
SLOT_ROM    =     $C000+SLOT*$100

* ---- Monitor ROM ----
HOME        =     $FC58
COUT        =     $FDED
CROUT       =     $FD8E
RDKEY       =     $FD0C

* ---- ProDOS MLI ----
MLI         =     $BF00
QUIT_CMD    =     $65

* ---- Zero page ----
ZP_PTR      =     $06
ZP_PTR_H    =     $07
ZP_COUNT    =     $08
ZP_CUR      =     $09
ZP_SEL      =     $0A
ZP_TMP      =     $0B
ZP_NAMLEN   =     $0C
ZP_SD_DONE  =     $0D            ; nonzero after first SD init
ZP_D16_HI   =     $1C
ZP_D16_LO   =     $1D
ZP_D16_FL   =     $1E
ZP_SLOT_SEL =     $1F

CAT_BUF     =     $0800
SLOT_TABLE  =     $0300

* =============================================================================
* Entry point
* =============================================================================

* Initialize slot table to $FF (empty)
            LDA   #$FF
            STA   SLOT_TABLE
            STA   SLOT_TABLE+1
            STA   SLOT_TABLE+2
            STZ   ZP_SD_DONE

* ---- One-time SD init ----
            LDA   #$02
            STA   R_SD_CMD

            JSR   print_string
            ASC   "INITIALIZING SD CARD..."
            DFB   $00

            LDX   #$00
            LDY   #$00
            STZ   ZP_TMP
sd_poll     LDA   R_SD_STAT
            BMI   sd_ready
            AND   #$40
            BNE   sd_error
            INY
            BNE   sd_poll
            INX
            BNE   sd_poll
            INC   ZP_TMP
            LDA   ZP_TMP
            CMP   #$0A
            BCC   sd_poll

sd_error    JSR   CROUT
            JSR   print_string
            ASC   "NO SD CARD DETECTED"
            DFB   $00
            JSR   CROUT
            JMP   do_quit

sd_ready    INC   ZP_SD_DONE

* =============================================================================
* Main screen redraw
* =============================================================================
redraw
            JSR   HOME
            JSR   print_string
            ASC   "BYTE HAMR DRIVE MANAGER"
            DFB   $00
            JSR   CROUT
            LDX   #23
:sep        LDA   #"="
            JSR   COUT
            DEX
            BNE   :sep
            JSR   CROUT
            JSR   CROUT

* ---- Read catalog via magic block $FFFF ----
sd_ok       LDA   #$FF
            STA   R_BLKLO
            STA   R_BLKHI
            LDA   #$01
            STA   R_CMD

cat_poll    LDA   R_STATUS
            BPL   cat_poll

* Read 512 bytes to CAT_BUF
            LDA   #<CAT_BUF
            STA   ZP_PTR
            LDA   #>CAT_BUF
            STA   ZP_PTR_H
            LDY   #0
            LDX   #2
read_loop   LDA   R_DATA
            STA   (ZP_PTR),Y
            INY
            BNE   read_loop
            INC   ZP_PTR_H
            DEX
            BNE   read_loop

* Parse header
            LDA   CAT_BUF
            BNE   has_files
            JSR   print_string
            ASC   "NO IMAGES ON SD CARD"
            DFB   $00
            JSR   CROUT
            JMP   do_quit

has_files   STA   ZP_COUNT

* ---- List images ----
            JSR   print_string
            ASC   "SD CARD IMAGES:"
            DFB   $00
            JSR   CROUT

            STZ   ZP_CUR
            LDA   #<{CAT_BUF+32}
            STA   ZP_PTR
            LDA   #>{CAT_BUF+32}
            STA   ZP_PTR_H

list_loop   LDA   #" "
            JSR   COUT
            LDA   ZP_CUR
            CLC
            ADC   #1
            JSR   print_dec_byte
            LDA   #")"
            JSR   COUT
            LDA   #" "
            JSR   COUT

* Print filename (8.3 with trailing spaces trimmed)
            LDY   #7
trim_loop   LDA   (ZP_PTR),Y
            CMP   #$20
            BNE   trim_found
            DEY
            BPL   trim_loop
            LDY   #0
trim_found  STY   ZP_NAMLEN

            LDY   #0
pname_loop  LDA   (ZP_PTR),Y
            ORA   #$80
            JSR   COUT
            CPY   ZP_NAMLEN
            BEQ   pname_done
            INY
            BRA   pname_loop
pname_done
            LDA   #"."
            JSR   COUT
            LDY   #8
            LDA   (ZP_PTR),Y
            ORA   #$80
            JSR   COUT
            INY
            LDA   (ZP_PTR),Y
            ORA   #$80
            JSR   COUT
            INY
            LDA   (ZP_PTR),Y
            CMP   #$20
            BEQ   ext_done
            ORA   #$80
            JSR   COUT
ext_done
* Block count from catalog entry bytes 12-13
            LDA   #" "
            JSR   COUT
            LDA   #"("
            JSR   COUT
            LDY   #13
            LDA   (ZP_PTR),Y
            STA   ZP_D16_HI
            DEY
            LDA   (ZP_PTR),Y
            STA   ZP_D16_LO
            STZ   ZP_D16_FL
            JSR   print_dec_16
            JSR   print_string
            ASC   " BLK)"
            DFB   $00
            JSR   CROUT

* Next entry (32 bytes each)
            CLC
            LDA   ZP_PTR
            ADC   #32
            STA   ZP_PTR
            LDA   ZP_PTR_H
            ADC   #0
            STA   ZP_PTR_H
            INC   ZP_CUR
            LDA   ZP_CUR
            CMP   ZP_COUNT
            BCS   list_done
            JMP   list_loop
list_done

* ---- Drive slot display ----
            JSR   CROUT
            JSR   print_string
            ASC   "DRIVE SLOTS:"
            DFB   $00
            JSR   CROUT

            LDX   #0              ; 0=S4D2, 1=S4D3, 2=S4D4
slot_disp   PHX
            LDA   #" "
            JSR   COUT
            JSR   print_string
            ASC   "S4,D"
            DFB   $00
            PLA
            PHA
            CLC
            ADC   #$32            ; '2' + slot_index
            ORA   #$80
            JSR   COUT
            LDA   #":"
            JSR   COUT
            LDA   #" "
            JSR   COUT

* Read block count via indexed register
            PLA
            PHA
            INC                   ; active_unit = slot_idx + 1
            STA   R_UNIT
            LDA   R_UBLKLO
            STA   ZP_D16_LO
            LDA   R_UBLKHI
            STA   ZP_D16_HI
            ORA   ZP_D16_LO
            BNE   slot_mounted

            JSR   print_string
            ASC   "[EMPTY]"
            DFB   $00
            BRA   slot_next

slot_mounted
            STZ   ZP_D16_FL
            JSR   print_dec_16
            JSR   print_string
            ASC   " BLOCKS"
            DFB   $00

slot_next   JSR   CROUT
            PLX
            INX
            CPX   #3
            BCS   slots_done
            JMP   slot_disp
slots_done

* ---- Menu ----
            JSR   CROUT
            JSR   print_string
            ASC   "M)OUNT U)NMOUNT B)OOT Q)UIT"
            DFB   $00
            JSR   CROUT
            JSR   print_string
            ASC   "> "
            DFB   $00

menu_key    JSR   RDKEY
            AND   #$DF            ; uppercase
            CMP   #"M"
            BEQ   jmp_mount
            CMP   #"U"
            BEQ   jmp_unmount
            CMP   #"B"
            BEQ   jmp_boot
            CMP   #"Q"
            BEQ   jmp_quit
            BRA   menu_key

jmp_mount   JMP   do_mount
jmp_unmount JMP   do_unmount
jmp_boot    JMP   do_boot
jmp_quit    JMP   do_quit


* =============================================================================
* MOUNT
* =============================================================================
do_mount    JSR   print_string
            ASC   "IMAGE #? "
            DFB   $00
            JSR   read_number
            LDA   ZP_SEL
            BEQ   mount_bail
            SEC
            SBC   #1              ; 0-based
            CMP   ZP_COUNT
            BCS   mount_bail
            STA   ZP_TMP          ; image index

            JSR   print_string
            ASC   "SLOT (2-4)? "
            DFB   $00
            JSR   RDKEY
            PHA                   ; save raw key (COUT/CROUT trash A)
            JSR   COUT
            JSR   CROUT
            PLA                   ; restore raw key
            AND   #$0F
            CMP   #2
            BCC   mount_bail
            CMP   #5
            BCC   mount_slot_ok
mount_bail  JMP   mount_cancel

mount_slot_ok
            SEC
            SBC   #2              ; slot index 0-2
            STA   ZP_SLOT_SEL
* Encode: {slot[7:6], 2'b0, img_idx[3:0]}
            ASL
            ASL
            ASL
            ASL
            ASL
            ASL                   ; << 6
            ORA   ZP_TMP
            STA   R_SELECT

            LDA   #$01            ; CMD_MOUNT
            STA   R_SD_CMD

            JSR   print_string
            ASC   "MOUNTING..."
            DFB   $00

* Poll for mount done (bit 5 of SD_STATUS)
            LDX   #$00
            LDY   #$00
            STZ   ZP_TMP
mount_poll  LDA   R_SD_STAT
            AND   #$20
            BNE   mount_done
            INY
            BNE   mount_poll
            INX
            BNE   mount_poll
            INC   ZP_TMP
            LDA   ZP_TMP
            CMP   #$3C
            BCC   mount_poll

            JSR   print_string
            ASC   "TIMEOUT"
            DFB   $00
            JSR   CROUT
            JMP   mount_cancel

mount_done  JSR   print_string
            ASC   "OK"
            DFB   $00
            JSR   CROUT

mount_cancel
            LDX   #0
            LDY   #0
:pause      INY
            BNE   :pause
            INX
            BNE   :pause
            JMP   redraw


* =============================================================================
* UNMOUNT
* =============================================================================
do_unmount  JSR   print_string
            ASC   "SLOT (2-4)? "
            DFB   $00
            JSR   RDKEY
            PHA                   ; save raw key (COUT/CROUT trash A)
            JSR   COUT
            JSR   CROUT
            PLA                   ; restore raw key
            AND   #$0F
            CMP   #2
            BCC   unmount_cancel
            CMP   #5
            BCS   unmount_cancel

            SEC
            SBC   #2              ; slot index 0-2
            ASL
            ASL
            ASL
            ASL
            ASL
            ASL                   ; << 6
            STA   R_SELECT

            LDA   #$03            ; CMD_UNMOUNT
            STA   R_SD_CMD

            JSR   print_string
            ASC   "UNMOUNTED"
            DFB   $00
            JSR   CROUT

unmount_cancel
            LDX   #0
            LDY   #0
:pause      INY
            BNE   :pause
            INX
            BNE   :pause
            JMP   redraw


* =============================================================================
* BOOT
* =============================================================================
do_boot     JSR   print_string
            ASC   "BOOT SLOT (1-4)? "
            DFB   $00
            JSR   RDKEY
            PHA                   ; save raw key (COUT/CROUT trash A)
            JSR   COUT
            JSR   CROUT
            PLA                   ; restore raw key
            AND   #$0F
            CMP   #1
            BCC   boot_cancel
            CMP   #5
            BCS   boot_cancel

            SEC
            SBC   #1              ; 0-based unit
            STA   R_BUNIT         ; write BOOT_UNIT register

            JSR   print_string
            ASC   "BOOTING..."
            DFB   $00
            JSR   CROUT

            SEC                   ; ROM uses BCS for boot detection
            JMP   SLOT_ROM

boot_cancel JMP   redraw


* =============================================================================
* QUIT
* =============================================================================
do_quit     JSR   MLI
            DFB   QUIT_CMD
            DA    quit_parms
quit_parms  DFB   $04,$00,$00,$00


* =============================================================================
* read_number — multi-digit decimal input → ZP_SEL
* =============================================================================
read_number STZ   ZP_SEL
:loop       JSR   RDKEY
            CMP   #$8D            ; RETURN
            BEQ   :done
            CMP   #$B0            ; < '0'
            BCC   :loop
            CMP   #$BA            ; > '9'
            BCS   :loop
            JSR   COUT
            AND   #$0F
            PHA
            LDA   ZP_SEL
            ASL
            STA   ZP_TMP
            ASL
            ASL
            CLC
            ADC   ZP_TMP          ; *10
            STA   ZP_SEL
            PLA
            CLC
            ADC   ZP_SEL
            STA   ZP_SEL
            BRA   :loop
:done       JSR   CROUT
            RTS


* =============================================================================
* print_string — print inline null-terminated string (normal ASCII)
* =============================================================================
print_string
            PLA
            STA   $FE
            PLA
            STA   $FF
            LDY   #$00
:loop       INC   $FE
            BNE   :noinc
            INC   $FF
:noinc      LDA   ($FE),Y
            BEQ   :done
            ORA   #$80
            JSR   COUT
            BRA   :loop
:done       LDA   $FF
            PHA
            LDA   $FE
            PHA
            RTS


* =============================================================================
* print_dec_byte — print A as decimal (suppress leading zeros)
* =============================================================================
print_dec_byte
            LDX   #0
            CMP   #100
            BCC   :tens
:h_loop     SEC
            SBC   #100
            INX
            CMP   #100
            BCS   :h_loop
            PHA
            TXA
            ORA   #$B0
            JSR   COUT
            PLA
            LDX   #1              ; flag: printed hundreds
:tens       TAY
            TXA
            PHA
            TYA
            LDX   #0
            CMP   #10
            BCC   :ones
:t_loop     SEC
            SBC   #10
            INX
            CMP   #10
            BCS   :t_loop
:ones       TAY
            PLA                   ; hundreds flag
            BNE   :print_t
            TXA
            BEQ   :skip_t
:print_t    TXA
            ORA   #$B0
            JSR   COUT
:skip_t     TYA
            ORA   #$B0
            JSR   COUT
            RTS


* =============================================================================
* print_dec_16 — print 16-bit ZP_D16_HI:ZP_D16_LO as decimal
* =============================================================================
print_dec_16
            LDX   #0
:_10k       SEC
            LDA   ZP_D16_LO
            SBC   #$10
            TAY
            LDA   ZP_D16_HI
            SBC   #$27
            BCC   :_10k_d
            STA   ZP_D16_HI
            STY   ZP_D16_LO
            INX
            BRA   :_10k
:_10k_d     JSR   d16_digit

            LDX   #0
:_1k        SEC
            LDA   ZP_D16_LO
            SBC   #$E8
            TAY
            LDA   ZP_D16_HI
            SBC   #$03
            BCC   :_1k_d
            STA   ZP_D16_HI
            STY   ZP_D16_LO
            INX
            BRA   :_1k
:_1k_d      JSR   d16_digit

            LDX   #0
:_100       SEC
            LDA   ZP_D16_LO
            SBC   #100
            TAY
            LDA   ZP_D16_HI
            SBC   #0
            BCC   :_100_d
            STA   ZP_D16_HI
            STY   ZP_D16_LO
            INX
            BRA   :_100
:_100_d     JSR   d16_digit

            LDX   #0
:_10        LDA   ZP_D16_LO
            CMP   #10
            BCC   :_10_d
            SBC   #10
            STA   ZP_D16_LO
            INX
            BRA   :_10
:_10_d      JSR   d16_digit
            LDA   ZP_D16_LO
            ORA   #$B0
            JSR   COUT
            RTS

d16_digit   TXA
            BNE   :nonzero
            LDA   ZP_D16_FL
            BEQ   :skip
:nonzero    LDA   #1
            STA   ZP_D16_FL
            TXA
            ORA   #$B0
            JSR   COUT
:skip       RTS
