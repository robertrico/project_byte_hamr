* =============================================================================
* picker.S — Byte Hamr Duo Drive Image Picker (65C02, Merlin32)
* =============================================================================
* ProDOS SYS file at $2000.  Duo-drive image selector:
*   Select SD image for D1 (boot) and D2 (data), then reboot.
*   Put PICKER.SYS on any mounted disk to re-swap without power cycle.
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
R_SELECT    =     IO+9           ; W: {mount_slot[7:6], 2'b0, img_idx[3:0]}
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
ZP_D1_IDX   =     $0D            ; 0-based D1 image index
ZP_D2_IDX   =     $0E            ; 0-based D2 image index, $FF = skip
ZP_D16_HI   =     $1C
ZP_D16_LO   =     $1D
ZP_D16_FL   =     $1E

CAT_BUF     =     $0800

* =============================================================================
* Entry point
* =============================================================================

* ---- SD init (only if not already ready) ----
            LDA   R_SD_STAT
            BMI   sd_ready        ; bit 7 set = already initialized, skip

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

sd_ready

* =============================================================================
* Main screen redraw
* =============================================================================
redraw
            JSR   HOME
            JSR   print_string
            ASC   "BYTE HAMR DUO DRIVE"
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
            LDA   #$FF
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

* =============================================================================
* Drive selection
* =============================================================================
            JSR   CROUT

* ---- D1 (boot drive) — required ----
            JSR   print_string
            ASC   "D1 IMAGE # (Q=QUIT)? "
            DFB   $00

            JSR   read_number
            LDA   ZP_SEL
            BPL   :d1_nq
            JMP   do_quit
:d1_nq      BNE   :d1_ok
            JMP   redraw
:d1_ok      SEC
            SBC   #1              ; 0-based
            CMP   ZP_COUNT
            BCS   bad_d1
            STA   ZP_D1_IDX

* ---- D2 (data drive) — optional ----
            JSR   print_string
            ASC   "D2 IMAGE # (ENTER=SKIP)? "
            DFB   $00

            JSR   read_number
            LDA   ZP_SEL
            BPL   :d2_nq
            JMP   do_quit
:d2_nq      BEQ   d2_skip         ; empty → no D2
            SEC
            SBC   #1              ; 0-based
            CMP   ZP_COUNT
            BCS   bad_d2
            STA   ZP_D2_IDX
            JMP   do_mounts

d2_skip     LDA   #$FF
            STA   ZP_D2_IDX
            JMP   do_mounts

bad_d1
bad_d2      JSR   print_string
            ASC   "INVALID"
            DFB   $00
            JSR   CROUT
            JMP   pause_redraw


* =============================================================================
* Mount sequence
* =============================================================================
do_mounts
* ---- Mount D1 ----
            LDA   ZP_D1_IDX
            STA   R_SELECT        ; mount_slot=0 (D1), img_idx in bits [3:0]

            LDA   #$01            ; CMD_MOUNT
            STA   R_SD_CMD

            JSR   print_string
            ASC   "MOUNTING D1..."
            DFB   $00

            JSR   wait_mount
            BCS   mount_fail

            JSR   print_string
            ASC   "OK"
            DFB   $00
            JSR   CROUT

* ---- Mount D2 (if selected) ----
            LDA   ZP_D2_IDX
            CMP   #$FF
            BEQ   mount_boot      ; skip D2

            ORA   #$40            ; mount_slot=1 (D2) — bit 6
            STA   R_SELECT

            LDA   #$01            ; CMD_MOUNT
            STA   R_SD_CMD

            JSR   print_string
            ASC   "MOUNTING D2..."
            DFB   $00

            JSR   wait_mount
            BCS   mount_fail

            JSR   print_string
            ASC   "OK"
            DFB   $00
            JSR   CROUT

* ---- Reboot into D1 ----
mount_boot
            JSR   print_string
            ASC   "BOOTING..."
            DFB   $00

* Brief delay for user to see message
            LDX   #0
            LDY   #0
:bpause     INY
            BNE   :bpause
            INX
            BNE   :bpause

            SEC                   ; ROM uses BCS for boot detection
            JMP   SLOT_ROM

mount_fail  JSR   print_string
            ASC   "MOUNT FAILED"
            DFB   $00
            JSR   CROUT

pause_redraw
            LDX   #0
            LDY   #0
:pause      INY
            BNE   :pause
            INX
            BNE   :pause
            JMP   redraw


* =============================================================================
* wait_mount — 2-phase poll: wait for MOUNTED to clear, then set
* Returns: C=0 success, C=1 timeout
* =============================================================================
wait_mount
* Phase 1: wait for MOUNTED (bit 5 of SD_STATUS) to clear
            LDX   #$00
            LDY   #$00
            STZ   ZP_TMP
:wstart     LDA   R_SD_STAT
            AND   #$20
            BEQ   :phase2
            INY
            BNE   :wstart
            INX
            BNE   :wstart
            INC   ZP_TMP
            LDA   ZP_TMP
            CMP   #$10            ; ~1s timeout for start
            BCC   :wstart
            SEC
            RTS

* Phase 2: wait for MOUNTED to set
:phase2     LDX   #$00
            LDY   #$00
            STZ   ZP_TMP
:wdone      LDA   R_SD_STAT
            AND   #$20
            BNE   :done
            INY
            BNE   :wdone
            INX
            BNE   :wdone
            INC   ZP_TMP
            LDA   ZP_TMP
            CMP   #$3C            ; ~6s timeout for mount
            BCC   :wdone
            SEC
            RTS

:done       CLC
            RTS


* =============================================================================
* QUIT
* =============================================================================
do_quit     JSR   MLI
            DFB   QUIT_CMD
            DA    quit_parms
quit_parms  DFB   $04,$00,$00,$00


* =============================================================================
* read_number — multi-digit decimal input → ZP_SEL (Q sets $FF)
* =============================================================================
read_number STZ   ZP_SEL
:loop       JSR   RDKEY
            CMP   #$8D            ; RETURN
            BEQ   :done
            CMP   #$D1            ; 'Q' (high-bit ASCII)
            BEQ   :quit
            CMP   #$F1            ; 'q' (high-bit ASCII)
            BEQ   :quit
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
:quit       LDA   #$FF
            STA   ZP_SEL
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
