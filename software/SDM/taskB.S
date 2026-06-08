* taskB - coproc task: write $77 to SDRAM bank0 $0051, RTS to kernel.
 TYP $06
 DSK taskB.bin
 ORG $0300
 LDA #$51
 STA $E000
 LDA #$00
 STA $E001
 STA $E002
 LDA #$77
 STA $E003
 RTS
