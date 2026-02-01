********************************
*   READING FROM PERIPHERAL    *
********************************
 ORG $8000
 JMP MAIN

MAIN
 LDA #$00
 STA $C0C0
 STA $C0C1
 STA $C0C2
 LDA #$FE
 STA $C0C3
 LDA #$CA
 STA $C0C4
 LDA #$01
 STA $C0C5
 LDA $C0C6
 LDA #$00
 STA $C0C3
 STA $C0C4
 LDA #$02
 STA $C0C5
 LDA $C0C6

 END
