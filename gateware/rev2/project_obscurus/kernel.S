* kernel.S - C4 coproc ASYNC SKILL DISPATCH scheduler. ORG $1000 (write-protected).
* Dispatcher-in-ISR: free-running timer tick -> $1F00 ISR scans CALL_REQ, spawns
* skills on-demand (SPAWN from mailbox), decrements per-slot budgets, force-completes
* expired slots, then SWITCHes. Batch path (go_pending -> BOOTSTRAP -> race) preserved.
* Uniform IRQ-frame context (TCB drops P; frame carries P; one RTI resume).
* Tasks run I-clear (preemptible); kernel/ISR I-set. Idle is CLI'd so the ISR fires.
* NOTE: DISPATCH/MBADDR/FORCEDONE/DBIT are JSR'd subroutines placed BEFORE $1F00
* (gateware pins IRQ@$1F00 / NMI@$1F40, only 64B apart). IRQH@$1F00 = entry+decision.
 LST OFF
 TYP $06
 DSK kernel.bin
 ORG $1000

TCB_SP = $D0
TCB_A  = $D4
TCB_X  = $D8
TCB_Y  = $DC
TCB_ST = $E4
TMPL   = $E0
TMPH   = $E1
NPARAM = $E8
CUR    = $EC
ORDER  = $ED
COREBL = $EE
COREBH = $EF
TMPA   = $F0
TMPX   = $F1
TMPY   = $F2
TMPP   = $F3
BUDGET = $C0
TMPS   = $C4
TMPSK  = $C5
SLOTZP = $80

COUNT   = $E010
COREID  = $E011
TICKCTL = $E012
TICKACK = $E013
GOSTAT  = $E014
CALLREQ = $E015
CALLACK = $E016
RUNSET  = $E017
RUNCLR  = $E018
DONESET = $E019
TMOSET  = $E01A
SNAPBSY = $E01B
TICKPER = $02
MBOXL   = $80
MBOXH   = $0F

 JMP RESET
 JMP YIELD
 JMP DONE

RESET
 SEI
 LDX #$FF
 TXS
 LDA #$00
 STA CUR
 JSR CLEARSLOTS
 LDA COREID
 ASL
 ASL
 ASL
 ASL
 CLC
 ADC #$61
 STA COREBL
 LDA #$00
 ADC #$00
 STA COREBH
 LDA #TICKPER
 STA TICKCTL        ; arm timer ONCE, free-running
KIDLE
 SEI                ; reset the idle stack to $01FF (ABOVE all task stacks, max
 LDX #$FF           ; STKTOP=$BF) so a later GO's JSR BOOTSTRAP / SPAWN frame-build
 TXS                ; can't collide with leftover idle-stack return bytes
 CLI                ; idle MUST be I-clear so the free-running tick ISR fires
KISPIN
 LDA GOSTAT
 BEQ KISPIN
 SEI
 STA GOSTAT         ; ack go_pending
 JSR BOOTSTRAP
 JMP KIDLE

CLEARSLOTS
 LDX #$00
CSL
 LDA #$00
 STA TCB_ST,X
 STA BUDGET,X
 INX
 CPX #$04
 BCC CSL
 RTS

BOOTSTRAP
 LDA #$00
 STA ORDER
 LDX #$00
BSL
 CPX COUNT
 BCS BSGO
 TXA                ; batch: skill_id = slot index (preserves CPRACE/CPRACE3)
 JSR SPAWN
 INX
 JMP BSL
BSGO
 LDA #$00
 STA CUR
 JMP RESTORE

* SPAWN: X=slot, A=skill_id. Seeds stack+TCB, RUNSET. BUDGET set by caller.
SPAWN
 STX TMPS
 STA TMPSK
 LDA #$01
 STA TCB_ST,X
 LDA #$00
 STA TCB_A,X
 STA TCB_X,X
 STA TCB_Y,X
 LDA TMPSK
 ASL
 TAY
 LDA $0201,Y
 STA TMPH
 LDA $0200,Y
 STA TMPL
 LDX TMPS
 LDY STKTOP,X
 LDA TMPH
 STA $0100,Y
 DEY
 LDA TMPL
 STA $0100,Y
 DEY
 LDA #$00
 STA $0100,Y
 DEY
 TYA
 STA TCB_SP,X
 LDA TMPS
 STA RUNSET
 RTS

RESTORE
 LDY CUR
 LDX TCB_SP,Y
 TXS
 LDA TCB_A,Y
 PHA
 LDA TCB_Y,Y
 LDX TCB_X,Y
 TAY
 PLA
 RTI

SWITCH
 TSX
 LDY CUR
 TXA
 STA TCB_SP,Y
 LDA TMPA
 STA TCB_A,Y
 LDA TMPX
 STA TCB_X,Y
 LDA TMPY
 STA TCB_Y,Y
 JSR PICKNEXT
 JMP RESTORE

YIELD
 PHP                ; capture caller flags (N/Z/C/V) BEFORE we touch them
 SEI
 STA TMPA
 STX TMPX
 STY TMPY
 PLA                ; A = caller P (I-bit reflects pre-SEI, irrelevant)
 AND #$FB           ; force I clear so the resumed task stays preemptible
 STA TMPP           ; stash flags for the manufactured frame
 PLA
 STA TMPL
 PLA
 STA TMPH
 INC TMPL
 BNE YNC
 INC TMPH
YNC
 LDA TMPH
 PHA
 LDA TMPL
 PHA
 LDA TMPP           ; frame P = caller flags (was #$00 -> destroyed Z across yield)
 PHA
 JMP SWITCH

DONE
 SEI
 LDY CUR
 LDA #$02
 STA TCB_ST,Y
 STY TMPS
 LDA TMPS
 STA DONESET
 STA RUNCLR
 LDY TMPS
 JSR PICKANY
 BCC DGO
 JMP KIDLE
DGO
 JMP RESTORE

PICKNEXT
 LDX CUR
PNL
 INX
 CPX #$04
 BCC PNNW
 LDX #$00
PNNW
 LDA TCB_ST,X
 CMP #$01
 BEQ PNF
 CPX CUR
 BNE PNL
PNF
 STX CUR
 RTS

PICKANY
 LDX #$00
PAL
 LDA TCB_ST,X
 CMP #$01
 BEQ PAF
 INX
 CPX #$04
 BCC PAL
 SEC
 RTS
PAF
 STX CUR
 CLC
 RTS

* ---- dispatcher subroutines (JSR'd from the $1F00 ISR; placed here to keep
*      NMIH at the gateware-pinned $1F40 with IRQH at $1F00) ----
DISPATCH
 LDA #$00
 STA TMPSK          ; freshly-spawned mask = 0
 LDX #$00
DSCAN
 LDA CALLREQ
 AND DBIT,X
 BEQ DNEXT
 JSR MBADDR         ; TMPL/TMPH = $0F80 + X*32
 LDY #$00
 LDA (TMPL),Y       ; skill_id
 PHA
 LDY #$01
 LDA (TMPL),Y       ; budget
 STA BUDGET,X
 PLA                ; A=skill_id, X=slot
 JSR SPAWN
 LDA DBIT,X
 ORA TMPSK
 STA TMPSK          ; mark X freshly-spawned
 STX TMPS
 LDA TMPS
 STA CALLACK        ; ack call_req[X]
 LDX TMPS
DNEXT
 INX
 CPX #$04
 BCC DSCAN
 LDX #$00
DBUD
 LDA TCB_ST,X
 CMP #$01
 BNE DBNEXT
 LDA DBIT,X
 AND TMPSK          ; freshly-spawned this tick? skip (off-by-one R4)
 BNE DBNEXT
 LDA BUDGET,X
 BEQ DBNEXT         ; 0 = no limit
 SEC
 SBC #$01
 STA BUDGET,X
 BNE DBNEXT
 JSR FORCEDONE
DBNEXT
 INX
 CPX #$04
 BCC DBUD
 RTS
DBIT DFB $01,$02,$04,$08

MBADDR
 TXA
 ASL
 ASL
 ASL
 ASL
 ASL
 CLC
 ADC #MBOXL         ; MBOXL=$80
 STA TMPL
 LDA #MBOXH         ; MBOXH=$0F
 STA TMPH
 RTS

FORCEDONE
 LDA #$02
 STA TCB_ST,X
 STX TMPS
 LDA TMPS
 STA DONESET
 STA TMOSET
 STA RUNCLR
 LDX TMPS
 RTS

* ISERV: out-of-line ISR service body (JMP'd from $1F00; placed here so the $1F00
* entry stub stays under the gateware-pinned $1F40 NMI vector). Decides: skip during
* snapshot, dispatch on CALL_REQ/budget, then SWITCH only if a task is actually
* runnable -- otherwise clean-return to the (CLI'd, free-running-tick) idle loop
* WITHOUT a context switch, so idle never depends on a stale TCB_SP frame.
ISERV
 LDA SNAPBSY        ; snapshot mask: skip dispatch during C-flash restore/save
 AND #$01
 BNE ISRET
 LDA CALLREQ        ; fast-path: nothing pending AND no budgets -> just SWITCH/idle
 BNE ISDISP
 LDX #$00
ISBUD
 LDA BUDGET,X
 BNE ISDISP
 INX
 CPX #$04
 BCC ISBUD
 JMP ISRUN
ISDISP
 JSR DISPATCH
ISRUN
 LDX #$00          ; any task RUNNABLE (TCB_ST==1)? if so preempt via SWITCH
ISRCL
 LDA TCB_ST,X
 CMP #$01
 BEQ ISGOSW
 INX
 CPX #$04
 BCC ISRCL
ISRET                ; nothing runnable -> clean idle return (no context switch)
 LDA TMPA
 LDX TMPX
 LDY TMPY
 RTI
ISGOSW
 JMP SWITCH

STKTOP DFB $2F,$5F,$8F,$BF

 DS $1F00-*
IRQH
 STA TMPA
 STX TMPX
 STY TMPY
 TSX
 LDA $0101,X
 AND #$10
 BNE IRQBRK
 STA TICKACK
 JMP ISERV
IRQBRK
 LDA TMPA
 LDX TMPX
 RTI

 DS $1F40-*
NMIH RTI
