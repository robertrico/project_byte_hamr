* kernel.S - C1 coproc resident kernel. ORG $1000 (write-protected region).
* Boots, loops dispatching TABLE[$0200..] entries 0..COUNT-1 (COUNT = $E010 reg).
* Index kept on the STACK across the task call (tasks trash A/X/Y). ISR stubs at
* $1F00/$1F40 (vectors point here; IRQ/NMI tied 0 in C1, but BRK reads $FFFE/F).
 LST OFF
 TYP $06
 DSK kernel.bin
 ORG $1000

JVEC = $00          ; ZP 2-byte indirect dispatch pointer (consumed before task runs)

RESET
 SEI
 LDX #$FF
 TXS
KMAIN
 LDX #0
KNEXT
 CPX $E010          ; X vs COUNT (abs read of the count register)
 BCS KMAIN          ; X >= COUNT -> nothing more, restart
 TXA
 ASL                ; A = X*2 (entry offset; X<=127 -> no overflow)
 TAY
 LDA $0200,Y
 STA JVEC
 LDA $0201,Y
 STA JVEC+1
 TXA
 PHA                ; save index on the stack (survives the task's RTS)
 JSR KCALL
 PLA
 TAX                ; restore index
 INX
 JMP KNEXT
KCALL JMP (JVEC)

 DS $1F00-*         ; pad to the IRQ ISR
IRQH RTI
 DS $1F40-*         ; pad to the NMI ISR
NMIH RTI
