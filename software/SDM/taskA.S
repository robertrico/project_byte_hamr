* taskA - coproc task: write $99 to SDRAM bank0 $0050, RTS to kernel.
 TYP $06
 DSK taskA.bin
 ORG $0300
 LDA #$50
 STA $E000
 LDA #$00
 STA $E001
 STA $E002
 LDA #$99
 STA $E003
 RTS
