* coproc_prog.S - C0 steel-thread program for the Arlet soft-6502.
* Reset vector ($FFFC/D) is synthesized to $0200 by coproc.v.
 TYP $06
 DSK coproc_prog.bin
 ORG $0200
START LDA #$42
 STA $E000      ; coproc.v posts SDRAM write bank0 $0040 = $42
LOOP JMP LOOP
