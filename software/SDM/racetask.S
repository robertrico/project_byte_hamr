* racetask - coproc race task. Counts down NPARAM[id] yielding each step, then
* atomically grabs the finish order and stamps it to SDRAM result[COREBASE+id].
 TYP $06
 DSK racetask.bin
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
YIELD  = $1003
DONE   = $1006

RACE
 LDY CUR
 LDX NPARAM,Y
RLOOP
 DEX
 JSR YIELD
 BNE RLOOP
 LDA ORDER
 CLC
 ADC #$01
 STA ORDER
 PHA
 LDA COREBL
 CLC
 ADC CUR
 STA SADRLO
 LDA COREBH
 ADC #$00
 STA SADRHI
 LDA #$00
 STA SBANK
 PLA
 STA SDATA
 JSR DONE
