* CPREG.S - C1 host loader. Registers taskA ($0300) + taskB ($0320) into the
* coproc (COUNT-LAST), then reads SDRAM bank0 $0050/$0051 and prints the two
* result bytes.
*
* COUNT-LAST: load both tasks' code + both TABLE entries while COUNT=0, then
* write CP_COUNT=2 LAST. Single host-priority write port -> writing BRAM during
* dispatch would drop kernel stack writes; COUNT=0 keeps the kernel idle.
*
* All STA CPWDATA are NON-INDEXED (indexed STA dummy-reads could mis-trigger).
 TYP $06
 DSK CPREG
 ORG $6000

CPLADDRLO = $C0C9
CPLADDRHI = $C0CA
CPWDATA   = $C0CB
CPCOUNT   = $C0CD
COUT  = $FDED
CROUT = $FD8E
PRBYTE = $FDDA
SRCL = $06
SRCH = $07

 JMP MAIN
 PUT SDRAMLIB

LADLO DS 1
LADHI DS 1
LEN   DS 1

* load LEN bytes from (SRCL),Y to coproc BRAM starting at LADHI:LADLO
LOADBLK
 LDA LADLO
 STA CPLADDRLO
 LDA LADHI
 STA CPLADDRHI
 LDY #0
:lp LDA (SRCL),Y
 STA CPWDATA          ; non-indexed -> one write + gateware auto-inc
 INY
 CPY LEN
 BNE :lp
 RTS

MAIN JSR SDM_READY    ; card present + SDRAM ready
 BCC :ok
 RTS                  ; timeout -> back to BASIC
:ok
* --- load taskA code -> coproc $0300 ---
 LDA #$00
 STA LADLO
 LDA #$03
 STA LADHI
 LDA #19
 STA LEN
 LDA #<TASKA
 STA SRCL
 LDA #>TASKA
 STA SRCH
 JSR LOADBLK
* --- load taskB code -> coproc $0320 ---
 LDA #$20
 STA LADLO
 LDA #$03
 STA LADHI
 LDA #19
 STA LEN
 LDA #<TASKB
 STA SRCL
 LDA #>TASKB
 STA SRCH
 JSR LOADBLK
* --- TABLE entry0 = $0300 @ coproc $0200 ---
 LDA #$00
 STA CPLADDRLO
 LDA #$02
 STA CPLADDRHI
 LDA #$00
 STA CPWDATA          ; entry0 lo
 LDA #$03
 STA CPWDATA          ; entry0 hi -> $0300
* --- TABLE entry1 = $0320 @ coproc $0202 ---
 LDA #$02
 STA CPLADDRLO
 LDA #$02
 STA CPLADDRHI
 LDA #$20
 STA CPWDATA          ; entry1 lo
 LDA #$03
 STA CPWDATA          ; entry1 hi -> $0320
* --- COUNT = 2 LAST (release) ---
 LDA #$02
 STA CPCOUNT
* --- wait for the kernel to dispatch both, then read results ---
 LDX #$00
:w1 LDY #$00
:w2 INY
 BNE :w2
 INX
 BNE :w1              ; ~16M coproc cycles of host spin - ample
* --- read bank0 $0050 -> print, then $0051 -> print ---
 LDA #$00
 STA SDM_BANK
 STA SDM_BANK+1
 LDA #$50
 STA SDM_ADDR
 LDA #$00
 STA SDM_ADDR+1
 JSR SDM_READ         ; result in SDM_VAL (and A)
 LDA SDM_VAL
 JSR PRBYTE
 LDA #$00
 STA SDM_BANK
 STA SDM_BANK+1
 LDA #$51
 STA SDM_ADDR
 LDA #$00
 STA SDM_ADDR+1
 JSR SDM_READ
 LDA SDM_VAL
 JSR PRBYTE
 JSR CROUT
 RTS

TASKA  DFB $A9,$50,$8D,$00,$E0,$A9,$00,$8D,$01,$E0,$8D,$02,$E0,$A9,$99,$8D,$03,$E0,$60
TASKAEND
TASKB  DFB $A9,$51,$8D,$00,$E0,$A9,$00,$8D,$01,$E0,$8D,$02,$E0,$A9,$77,$8D,$03,$E0,$60
TASKBEND
