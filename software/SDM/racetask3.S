* racetask3 - C3 preemptive race task. NO yield: runs a budget*delay loop; the
* timer tick slices it. id in Y, budget in X, inner delay in A (all TCB-saved
* across preemption). Finish: SEI / atomic ++ORDER / stamp result[COREBASE+id] /
* JMP DONE.
 TYP $06
 DSK racetask3.bin
 ORG $0300

CUR    = $EC
NPARAM = $E8
ORDER  = $ED
COREBL = $EE
COREBH = $EF
SADRLO = $E000
SADRHI = $E001
SBANK  = $E002
SDATA  = $E003
DONE   = $1006

RACE
 LDY CUR
 LDX NPARAM,Y
OUTER
 LDA #$80
 SEC
INNER
 SBC #$01
 BNE INNER
 DEX
 BNE OUTER
 SEI
 LDA ORDER
 CLC
 ADC #$01
 STA ORDER
 PHA
 TYA
 CLC
 ADC COREBL
 STA SADRLO
 LDA COREBH
 ADC #$00
 STA SADRHI
 LDA #$00
 STA SBANK
 PLA
 STA SDATA
 JMP DONE
