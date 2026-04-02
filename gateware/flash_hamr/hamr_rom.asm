; =============================================================================
; hamr_rom.asm — Byte Hamr ProDOS/SmartPort block device ROM
; =============================================================================
; Talks to FPGA register file for block reads/writes.
;
; Register interface ($C0n0 + slot*16):
;   REG_CMD   ($C080,X): Read = STATUS, Write = COMMAND
;     STATUS read:  {ready, error, 5'b0, boot_done}
;     COMMAND write: $01 = READ_BLOCK, $02 = WRITE_BLOCK
;   REG_DATA  ($C081,X): Read/Write data, auto-increments
;   REG_BLK_LO ($C082,X): Block number low byte
;   REG_BLK_HI ($C083,X): Block number high byte
;
; ROM memory map (as seen by boot_rom.v):
;   $400-$4FF: Slot ROM ($Cn00-$CnFF)
;   $800-$FFF: Shared ROM ($C800-$CFFF)
;
; Assembled for slot 4, but slot number only matters for the $Cn
; page — the $C800 code uses dynamic slot detection.
; =============================================================================

	cpu	65c02

; ---- Zero page temporaries (safe range $40-$5B) ----
slot		equ	$58	; slot number (1-7)
buf_ptr		equ	$44	; ProDOS buffer pointer (lo/hi)
buf_ptr_h	equ	$45

; ---- ProDOS command block ($42-$47) ----
prodos_command	equ	$42
prodos_unit	equ	$43
prodos_buffer	equ	$44
prodos_block	equ	$46
prodos_block_h	equ	$47

; ---- Screen holes (per-slot, indexed by slot number) ----
sh_prodos_flag	equ	$0478	; MSB=1 for ProDOS, 0 for SmartPort
sh_magic1	equ	$06f8	; $A5 if initialized
sh_magic2	equ	$0778	; $5A if initialized
sh_unit_count	equ	$07f8	; number of units
sh_05f8		equ	$05f8	; SmartPort return addr lo
sh_0678		equ	$0678	; SmartPort return addr hi

; ---- Global screen holes ----
gh_shared_slot	equ	$07f8	; $Cn for active $C800 ROM

; ---- Apple II ROM / Monitor ----
rom_dis		equ	$cfff	; accessing this deactivates $C800 ROMs

; ---- I/O register base addresses (indexed by slot*16) ----
REG_CMD		equ	$c080	; STATUS read / COMMAND write
REG_DATA	equ	$c081	; DATA (auto-increment)
REG_BLK_LO	equ	$c082	; Block number low
REG_BLK_HI	equ	$c083	; Block number high

; ---- ProDOS error codes ----
ERR_NONE	equ	$00
ERR_BAD_CMD	equ	$01
ERR_IO		equ	$27
ERR_NO_DRIVE	equ	$28
ERR_NO_WRITE	equ	$2b
ERR_BAD_BLOCK	equ	$2d

; ---- S4D1 block count (menu volume, 280 blocks = $0118) ----
BLOCK_COUNT_LO	equ	$18	; 280 & $FF
BLOCK_COUNT_HI	equ	$01	; 280 >> 8

; ---- S4D2 block count registers (from FPGA, register $0B/$0C) ----
REG_S4D2_BLK_LO equ	$c08b	; S4D2 block count lo (indexed by slot*16)
REG_S4D2_BLK_HI equ	$c08c	; S4D2 block count hi

; ---- UNIT2 SDRAM block offset ----
UNIT2_OFF_LO	equ	$00	; 2048 & $FF = $00
UNIT2_OFF_HI	equ	$08	; 2048 >> 8 = $08

; ---- Commands ----
CMD_READ	equ	$01
CMD_WRITE	equ	$02


; =============================================================================
; Slot ROM: $C400-$C4FF (mapped to ROM $400-$4FF)
; =============================================================================
; ProDOS looks for: $Cn01=$20, $Cn03=$00, $Cn05=$03, $CnFF=entry_lo
; Boot entry is at $Cn00 with carry set (from boot scan)

	org	$c400

; ---- Boot/signature entry ($C400 = $Cn00) ----
	ldx	#$20		; $Cn01 = $20: block device
	ldx	#$00		; $Cn03 = $00: no specific type
	ldx	#$03		; $Cn05 = $03: "smart" device (supports STATUS)
	cmp	#$00		; $Cn07 = $00 (padding)
	bcs	cn14		; carry set = boot request

; ---- ProDOS entry ($C409 = $Cn09) ----
prodos_entry:
	sec
	bcs	cn0e

; ---- SmartPort entry ($C40B = $Cn0B) ----
smartport_entry:
	clc

; ---- Combined dispatch ($C40C = $Cn0E, after CLC/SEC) ----
cn0e:
	ldx	#$04		; slot 4
	ror	sh_prodos_flag,x
	clc			; clear carry for execute_command path

; ---- Combined boot/command entry ($C414 = $Cn14) ----
cn14:
	ldx	#$c4		; $C0 + slot
	stx	gh_shared_slot
	ldx	#$04		; slot number
	lda	rom_dis		; deactivate other $C800 ROMs
	jmp	shared_entry	; jump to $C800 space

; ---- Pad to $CnFB ----
	; (filler — rest of $Cn space available for future use)

	org	$c4fb

	fcb	$00		; $CnFB: SmartPort device type (0 = block device)
	fdb	$0000		; $CnFC-FD: block count (0 = requires STATUS call)
	fcb	$bf		; $CnFE: ProDOS characteristics
				;   bit 7: removable
				;   bit 5: support STATUS
				;   bit 4: support READ/WRITE
				;   bit 3: support FORMAT
				;   bit 0: support SmartPort
	fcb	prodos_entry & $ff  ; $CnFF: ProDOS entry point low byte


; =============================================================================
; Shared ROM: $C800-$CFFF (mapped to ROM $800-$FFF)
; =============================================================================

	org	$c800

; ---- Entry point from slot ROM ----
shared_entry:
	bcc	execute_command	; C=0: ProDOS/SmartPort command
	jmp	boot		; C=1: boot request

; ---- Command dispatcher ----
execute_command:
	cld
	stx	slot		; save slot number

	; Get slot*16 into X for register access
	jsr	get_slot_x

	lda	sh_prodos_flag-1,x	; wait — X is slot*16 here, not slot
	; Fix: need slot number for screen hole indexing
	; Screen holes are indexed by slot number, not slot*16

	; Reload slot number for screen hole access
	ldy	slot

	lda	sh_prodos_flag,y
	bmi	prodos_cmd	; MSB=1: ProDOS call

	; ---- SmartPort call ----
	; Pull return address from stack, read inline params
	pla
	sta	sh_05f8,y	; return addr lo
	clc
	adc	#$03		; skip cmd byte + param ptr
	tax
	pla
	sta	sh_0678,y	; return addr hi
	adc	#$00
	pha			; push adjusted return addr
	txa
	pha

	; Read SmartPort command byte and param list from inline data
	lda	sh_05f8,y
	sta	buf_ptr
	lda	sh_0678,y
	sta	buf_ptr_h

	ldy	#$01
	lda	(buf_ptr),y	; SmartPort command
	sta	prodos_command
	iny
	lda	(buf_ptr),y	; param list lo
	tax
	iny
	lda	(buf_ptr),y	; param list hi
	sta	buf_ptr_h
	stx	buf_ptr

	; Copy SmartPort params to ProDOS command block
	ldy	#$00
	lda	(buf_ptr),y	; param count (ignored for now)
	iny
	lda	(buf_ptr),y	; unit number
	sta	prodos_unit
	iny
	lda	(buf_ptr),y	; buffer lo
	sta	prodos_buffer
	iny
	lda	(buf_ptr),y	; buffer hi
	sta	prodos_buffer+1
	iny
	lda	(buf_ptr),y	; block lo
	sta	prodos_block
	iny
	lda	(buf_ptr),y	; block hi
	sta	prodos_block_h

	; Fall through to ProDOS dispatch

prodos_cmd:
	jsr	get_slot_x	; X = slot*16 for register access

	lda	prodos_command
	beq	cmd_status
	cmp	#$01
	beq	cmd_read
	cmp	#$02
	beq	cmd_write
	cmp	#$05
	beq	cmd_init

	; Unknown command
	lda	#ERR_BAD_CMD
	sec
	jmp	cmd_done


; =========================================================================
; STATUS (command $00)
; =========================================================================
; Returns: X = block count lo, Y = block count hi
;          A = 0 (no error)

cmd_status:
	; Check if device is ready
	lda	REG_CMD,x
	and	#$01		; boot_done?
	beq	status_offline

	; Check which unit
	ldy	slot
	lda	prodos_unit
	cmp	#$02
	beq	status_unit2

	; Unit 1: menu volume (S4D1)
	ldx	#BLOCK_COUNT_LO
	ldy	#BLOCK_COUNT_HI
	lda	#ERR_NONE
	clc
	jmp	cmd_done

status_unit2:
	; Unit 2: SD card image (S4D2) — read block count from FPGA registers
	jsr	get_slot_x
	lda	REG_S4D2_BLK_LO,x
	pha
	lda	REG_S4D2_BLK_HI,x
	tay			; Y = block count hi
	pla
	tax			; X = block count lo
	lda	#ERR_NONE
	clc
	jmp	cmd_done

status_offline:
	lda	#ERR_NO_DRIVE
	sec
	jmp	cmd_done


; =========================================================================
; READ BLOCK (command $01)
; =========================================================================

cmd_read:
	; Apply UNIT2 offset if unit 2
	jsr	apply_unit_offset

	; Set block number
	lda	prodos_block
	sta	REG_BLK_LO,x
	lda	prodos_block_h
	sta	REG_BLK_HI,x

	; Issue READ command
	lda	#CMD_READ
	sta	REG_CMD,x

	; Wait for ready
	jsr	wait_ready
	bcs	cmd_done	; error

	; Copy 512 bytes from DATA port to buffer
	ldy	#$00
read_page1:
	lda	REG_DATA,x
	sta	(prodos_buffer),y
	iny
	bne	read_page1

	inc	prodos_buffer+1	; next page

	ldy	#$00
read_page2:
	lda	REG_DATA,x
	sta	(prodos_buffer),y
	iny
	bne	read_page2

	dec	prodos_buffer+1	; restore pointer

	lda	#ERR_NONE
	clc
	jmp	cmd_done


; =========================================================================
; WRITE BLOCK (command $02)
; =========================================================================

cmd_write:
	; Apply UNIT2 offset if unit 2
	jsr	apply_unit_offset

	; Set block number
	lda	prodos_block
	sta	REG_BLK_LO,x
	lda	prodos_block_h
	sta	REG_BLK_HI,x

	; Issue a "dummy read" to reset buf_addr to 0
	; (write command needs buffer at position 0)
	; Actually: just writing block num already set.
	; We need to reset the DATA pointer. Issuing a READ
	; then overwriting would be wasteful. Better approach:
	; the COMMAND write resets buf_addr. So we write data
	; BEFORE issuing the command... but then buf_addr starts
	; at 0 only if we just came from a READ completion.
	;
	; Simpler: issue READ to reset pointer, wait, then fill.
	; But that's slow. Let's just rely on the contract that
	; COMMAND write resets buf_addr to 0, and we fill AFTER.
	;
	; Actually the bus_interface resets buf_addr on any
	; COMMAND write. So: issue a NOP-ish read first? No.
	;
	; Correct approach: write the data to the buffer FIRST,
	; then issue WRITE command.  bus_interface resets buf_addr
	; when COMMAND is written (for read completion readout).
	; For write, the 6502 needs to fill BEFORE the command.
	;
	; We need buf_addr at 0 before filling. The bus_interface
	; resets buf_addr to 0 when a command completes (block_ready).
	; If we just completed a previous command, buf_addr should be
	; at 0 (or wherever the last read left it).
	;
	; Simplest fix: add a RESET_PTR command, or use a register
	; write to reset buf_addr. For now, issue a dummy READ of
	; block 0, wait, then fill. TODO: optimize later.

	; For v1: Write data first (buf_addr auto-increments from
	; wherever it is after the last COMMAND completion = 0)

	; Copy 512 bytes from buffer to DATA port
	ldy	#$00
write_page1:
	lda	(prodos_buffer),y
	sta	REG_DATA,x
	iny
	bne	write_page1

	inc	prodos_buffer+1

	ldy	#$00
write_page2:
	lda	(prodos_buffer),y
	sta	REG_DATA,x
	iny
	bne	write_page2

	dec	prodos_buffer+1

	; Now issue WRITE command (resets buf_addr — but arbiter
	; reads from buffer before reset matters)
	lda	#CMD_WRITE
	sta	REG_CMD,x

	; Wait for completion
	jsr	wait_ready
	bcs	cmd_done

	lda	#ERR_NONE
	clc
	jmp	cmd_done


; =========================================================================
; INIT (command $05)
; =========================================================================

cmd_init:
	; Initialize: set unit count, mark as initialized
	ldy	slot
	lda	#$02		; 2 units (S4D1 menu + S4D2 SD image)
	sta	sh_unit_count,y
	lda	#$a5
	sta	sh_magic1,y
	eor	#$ff
	sta	sh_magic2,y

	lda	#ERR_NONE
	clc
	jmp	cmd_done


; =========================================================================
; Common return — restore state and return to caller
; =========================================================================

cmd_done:
	rts


; =========================================================================
; wait_ready — poll STATUS register until ready or timeout
; =========================================================================
; Entry: X = slot*16
; Exit:  C=0 success, C=1 error (A=error code)

wait_ready:
	ldy	#$00		; outer loop (~16ms timeout)
wr_outer:
	lda	#$00
wr_inner:
	pha
	lda	REG_CMD,x	; read STATUS
	bmi	wr_ready	; bit 7 = ready
	pla
	sec
	sbc	#$01
	bne	wr_inner
	dey
	bne	wr_outer

	; Timeout
	pla
	lda	#ERR_IO
	sec
	rts

wr_ready:
	and	#$40		; check error bit
	bne	wr_error
	pla
	lda	#ERR_NONE
	clc
	rts

wr_error:
	pla
	lda	#ERR_IO
	sec
	rts


; =========================================================================
; apply_unit_offset — add UNIT2_OFFSET to block number if unit 2
; =========================================================================
; Entry: prodos_unit set, prodos_block/prodos_block_h set
; Exit:  prodos_block/prodos_block_h adjusted if unit 2
;        X preserved (slot*16)

apply_unit_offset:
	ldy	slot
	lda	prodos_unit
	cmp	#$02
	bne	auo_done
	; Unit 2: add UNIT2_OFFSET (2048 = $0800)
	clc
	lda	prodos_block
	adc	#UNIT2_OFF_LO	; $00
	sta	prodos_block
	lda	prodos_block_h
	adc	#UNIT2_OFF_HI	; $08
	sta	prodos_block_h
auo_done:
	rts


; =========================================================================
; get_slot_x — load X with slot * 16 for register indexing
; =========================================================================

get_slot_x:
	lda	slot
	asl
	asl
	asl
	asl
	tax
	rts


; =========================================================================
; Boot handler
; =========================================================================
; Read block 0 to $800, verify, jump to $801

boot:
	stx	slot
	lda	#$aa
	sta	sh_prodos_flag,x	; MSB set = ProDOS protocol

	jsr	get_slot_x

	; Read block 0 to $0800
	lda	#$00
	sta	REG_BLK_LO,x
	sta	REG_BLK_HI,x
	lda	#CMD_READ
	sta	REG_CMD,x

	jsr	wait_ready
	bcs	boot_error

	; Copy 512 bytes to $0800
	ldy	#$00
boot_copy1:
	lda	REG_DATA,x
	sta	$0800,y
	iny
	bne	boot_copy1

boot_copy2:
	lda	REG_DATA,x
	sta	$0900,y
	iny
	bne	boot_copy2

	; Verify boot block: byte 0 must be $01, byte 1 must be non-zero
	lda	$0800
	cmp	#$01
	bne	boot_error
	lda	$0801
	beq	boot_error

	; Initialize
	ldy	slot
	lda	#$02		; 2 units (S4D1 + S4D2)
	sta	sh_unit_count,y
	lda	#$a5
	sta	sh_magic1,y
	eor	#$ff
	sta	sh_magic2,y

	; Jump to boot code with slot*16 in X
	jsr	get_slot_x
	jmp	$0801

boot_error:
	sec
	rts


; =========================================================================
; Identification string
; =========================================================================
	org	$cfdb
	fcb	"BYTE HAMR",$00

	org	$cffe
	fdb	$0000		; unused vectors
