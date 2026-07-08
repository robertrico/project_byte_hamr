; Intel 8008 "Hello 8008!" - RAM build for the b8008 monitor
; For AS Macro Assembler
;
; RAM port of hello_8008.asm: loaded with the monitor's L command and run
; with G 2040. Output goes through the monitor USART (OUT 9, 115200 baud)
; instead of the original 2400-bps bitbang; ends with a jump back to the
; monitor instead of HLT.
;
; Output:
;   HI
;   0123456789 B8008-OK
;
; Load/run:
;   make assemble-sample PROG=hello_8008_ram
;   ./send_hex.py test_programs/samples/hello_8008_ram.hex   (minicom closed)
;   then in minicom:  G 2040

        cpu     8008new
        page    0

OUTPORT equ     09H             ; monitor USART TX port (OUT 9)

; Code lives in user RAM; string data on its own page (0x2200) so the
; PUTS pointer math stays page-local like the original (MSG at 0x0200).
        org     2040h
START:
        ; Test 1: Output "HI" directly (simple test)
        MVI     A,'H'
        CALL    ECHO
        MVI     A,'I'
        CALL    ECHO
        MVI     A,0DH           ; CR
        CALL    ECHO
        MVI     A,0AH           ; LF
        CALL    ECHO

        ; Test 2: Output digits 0-9 using a loop (tests INR, CPI, JNZ)
        MVI     C,'0'           ; Start with ASCII '0'
DIGIT_LOOP:
        MOV     A,C
        CALL    ECHO
        INR     C               ; Next digit
        MOV     A,C
        CPI     '9'+1           ; Past '9'?
        JNZ     DIGIT_LOOP      ; Keep looping if not

        ; Output space
        MVI     A,' '
        CALL    ECHO

        ; Test 3: Output string from memory using MOV A,M (tests memory indirect)
        MVI     H,(MSG>>8)&0FFH ; Point to MSG page
        MVI     L,MSG&0FFH
        CALL    PUTS

        ; Output CR/LF
        MVI     A,0DH
        CALL    ECHO
        MVI     A,0AH
        CALL    ECHO

        ; All done - back to the monitor
        JMP     0

;------------------------------------------------------------------------
; PUTS - Output null-terminated string via HL pointer
; Tests: MOV A,M (memory indirect read), CPI, RZ, INR L, JMP
;------------------------------------------------------------------------
PUTS:
        MOV     A,M             ; Get character from memory at [HL]
        CPI     00H             ; Null terminator?
        RZ                      ; Return if end of string
        CALL    ECHO            ; Output character
        INR     L               ; Next character (assumes string < 256 bytes)
        JMP     PUTS

;------------------------------------------------------------------------
; Character output subroutine
; Sends the character in A through the monitor USART. The pacing loop
; covers one 115200-baud frame before the next OUT.
; Uses A and B. Returns with the 7-bit character in A.
;------------------------------------------------------------------------
ECHO:
        ANI     7FH             ; Mask off MSB
        OUT     OUTPORT         ; USART sends immediately
        MVI     B,14H           ; ~40 instructions > one character frame
ECHO_DLY:
        DCR     B
        JNZ     ECHO_DLY
        RET

;------------------------------------------------------------------------
; String data on its own page (like the original's 0x0200)
;------------------------------------------------------------------------
        org     2200h
MSG:
        db      "B8008-OK",00H

        end
