* FROM HELLO_8008_RAM.ASM
 USE MAC8008
* Intel 8008 "Hello 8008!" - RAM build f
* For AS Macro Assembler
* 
* RAM port of hello_8008.asm: loaded wit
* with G 2040. Output goes through the m
* instead of the original 2400-bps bitba
* monitor instead of HLT.
* 
* Output:
* HI
* 0123456789 B8008-OK
* 
* Load/run:
* make assemble-sample PROG=hello_8008_r
* ./send_hex.py test_programs/samples/he
* then in minicom:  G 2040
OUTPORT EQU $9
* Code lives in user RAM; string data on
* PUTS pointer math stays page-local lik
 ORG $2040
START
* Test 1: Output "HI" directly (simple t
 MVI A;'H'
 CALL ECHO
 MVI A;'I'
 CALL ECHO
 MVI A;$D
 CALL ECHO
 MVI A;$A
 CALL ECHO
* Test 2: Output digits 0-9 using a loop
 MVI C;'0'
DIGIT_LOOP
 MOV A;C
 CALL ECHO
 INR C
 MOV A;C
 CPI '9'+1
 JNZ DIGIT_LOOP
* Output space
 MVI A;' '
 CALL ECHO
* Test 3: Output string from memory usin
 MVI H;>MSG
 MVI L;<MSG
 CALL PUTS
* Output CR/LF
 MVI A;$D
 CALL ECHO
 MVI A;$A
 CALL ECHO
* All done - back to the monitor
 JMP8 0
* --------------------------------------
* PUTS - Output null-terminated string v
* Tests: MOV A,M (memory indirect read),
* --------------------------------------
PUTS
 MOV A;M
 CPI $0
 RZ
 CALL ECHO
 INR L
 JMP8 PUTS
* --------------------------------------
* Character output subroutine
* Sends the character in A through the m
* covers one 115200-baud frame before th
* Uses A and B. Returns with the 7-bit c
* --------------------------------------
ECHO
 ANI $7F
 OUT OUTPORT
 MVI B;$14
ECHO_DLY
 DCR B
 JNZ ECHO_DLY
 RET
* --------------------------------------
* String data on its own page (like the 
* --------------------------------------
 DS $2200-*
MSG
 ASC 'B8008-OK'
 DFB $0
