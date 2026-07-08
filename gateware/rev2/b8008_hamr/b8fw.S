* =============================================================================
* b8fw.S - b8008_hamr $C400 slot ROM: PR#4 -> 8008 terminal (Merlin32)
* =============================================================================
* PR#4 -> JSR $C400 (via CSW). ENTRY restores CSW=COUT1 (else our own COUT
* calls would re-enter $C400), prints the banner, then runs the glass TTY
* against the FIFO registers. Whole terminal lives in the 256-byte slot ROM:
* no $C800, no arm/disarm, no SEI. Ctrl-Q RTSes back to BASIC over PR#'s
* JSR return (obscurus slot_rom.S exit pattern). Ctrl-R reboots the 8008.
* No OS required: from the monitor prompt, PR#4 works the same; exit via
* Ctrl-Reset if there is nothing to RTS to.
* =============================================================================
 TYP $06
 DSK b8fw.bin
 ORG $C400

CSWL = $36
CSWH = $37
COUT1 = $FDF0
COUT = $FDED
KBD = $C000
KBDSTRB = $C010
TXST = $C0C0
TXDAT = $C0C1
RXST = $C0C2
CTL = $C0C5

ENTRY LDA #<COUT1
 STA CSWL
 LDA #>COUT1
 STA CSWH
 LDA #$8D
 JSR COUT
 LDX #0
:b LDA BANNER,X
 BEQ LOOP
 JSR COUT
 INX
 BNE :b

* GLASS TTY: KBD -> RX FIFO, TX FIFO -> COUT
LOOP LDA KBD
 BPL :notx
 STA KBDSTRB
 CMP #$91 ; CTRL-Q = EXIT
 BEQ QUIT
 CMP #$92 ; CTRL-R = 8008 REBOOT
 BEQ RBOOT
 AND #$7F
 TAY
:w LDA RXST ; WAIT FIFO SPACE (BIT7)
 BPL :w
 TYA
 STA RXST
:notx LDA TXST ; BIT7 = BYTE AVAILABLE
 BPL LOOP
 LDA TXDAT
 CMP #$0A ; DROP LF (COUT CR DOES BOTH)
 BEQ LOOP
 ORA #$80
 JSR COUT
 JMP LOOP

RBOOT LDA #$52 ; 'R' HOLD RESET
 STA CTL
 LDA #$47 ; 'G' RELEASE - FULL BOOT
 STA CTL
 JMP LOOP

QUIT LDA #$8D
 JSR COUT
 RTS ; BACK TO BASIC OVER PR#'S JSR

BANNER ASC "B8008 TERM ^Q=EXIT ^R=RESET"
 DFB $8D,0
