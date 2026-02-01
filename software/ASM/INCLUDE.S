********************************
* INCLUDE                      *
********************************
GRAPHICS = $C050
HIRES = $C057
PAGE1 = $C054
MIXOFF = $C053
LOW = $1A
HIGH = $1B
*
********************************
* UTILITIES                    *
********************************
*
*HI-RES CLEAR SCREEN SUBROUTINE
*
HGR LDA GRAPHICS
 LDA HIRES
 LDA PAGE1
 LDA MIXOFF
 LDA #$00
 STA $26
 LDA #$20
 STA $27
CLR1 LDY #$00
 LDA #$00
CLR STA ($26),Y
 INY
 BNE CLR
 INC $27
 LDA $27
 CMP #$40
 BLT CLR1
 RTS
*
