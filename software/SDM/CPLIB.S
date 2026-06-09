* CPLIB.S - C4 ASYNC SKILL-CALL HOST LIBRARY
* PUT-INCLUDE. NO ORG. NO ZERO PAGE.
* ABSOLUTE STA/LDA $C0Cx ONLY (NEVER ,X WRITES TO $C0Cx)
*
* REGISTERS (SLOT 4):
*  CP_DONE     $C0C0  R  bit S = slot S completed + uncollected
*  CP_ACTIVE   $C0C1  R  bit S = slot S busy (call_req|running|done)
*  CP_TIMEDOUT $C0C2  R  bit S = slot S killed by run-budget watchdog
*  CP_RING     $C0C5  W  write slot# -> kernel spawns that slot's skill
*  CP_LADDRLO  $C0C9  W  set coproc BRAM load address lo
*  CP_LADDRHI  $C0CA  W  set coproc BRAM load address hi
*  CP_WDATA    $C0CB  W  write byte -> BRAM[laddr], laddr++
*  CP_RDATA    $C0CC  R  read  byte <- BRAM[laddr], laddr++   ($C0CC READ face)
*  CP_COLLECT  $C0CC  W  write slot# -> free slot             ($C0CC WRITE face)
*
* MAILBOX in coproc BRAM at $0F80+slot*32:
*  +0  skill_id  +1  run_budget  +2  arg0  +16  result
*
* ENTRY POINTS:
*  CP_CALL    A=skill_id X=arg0 Y=budget -> A=handle ($FF=no free slot)
*  CP_POLL    A=handle   -> A=$80 done / $00 pending
*  CP_WAIT    A=handle      spin until done (no timeout)
*  CP_RESULT  A=handle   -> A=result; frees the slot

CP_DONE     = $C0C0
CP_ACTIVE   = $C0C1
CP_TIMEDOUT = $C0C2
CP_RING     = $C0C5
CP_LADDRLO  = $C0C9
CP_LADDRHI  = $C0CA
CP_WDATA    = $C0CB
CP_RDATA    = $C0CC
CP_COLLECT  = $C0CC

* CP_CALL: A=skill_id, X=arg0, Y=budget.
* Returns handle in A (and CPLSLOT). Returns $FF if no slot free.
CP_CALL
 STA CPLSK        ; save skill_id
 STX CPLARG       ; save arg0
 STY CPLBUD       ; save budget
 LDA CP_ACTIVE    ; bitmask: bit S=1 -> slot S busy; want lowest CLEAR bit
 LDX #$00
:free
 LSR              ; shift bit0 into carry
 BCC :got         ; carry clear -> slot X is free
 INX
 CPX #$04
 BCC :free        ; X<4 -> more slots to try
 LDA #$FF         ; all 4 slots busy -> no handle
 STA CPLSLOT
 RTS
:got
 STX CPLSLOT
 ; mailbox lo = slot*32 + $80, hi = $0F
 ; slot 0->$80  slot 1->$A0  slot 2->$C0  slot 3->$E0  (no carry)
 TXA
 ASL
 ASL
 ASL
 ASL
 ASL              ; X * 32
 CLC
 ADC #$80
 STA CP_LADDRLO
 LDA #$0F
 STA CP_LADDRHI
 LDA CPLSK
 STA CP_WDATA     ; mbox+0 skill_id  (autoinc)
 LDA CPLBUD
 STA CP_WDATA     ; mbox+1 budget
 LDA CPLARG
 STA CP_WDATA     ; mbox+2 arg0
 LDA CPLSLOT
 STA CP_RING      ; trigger kernel spawn
 LDA CPLSLOT      ; return handle in A
 RTS

* CP_POLL: A=handle. Returns A=$80 if done, $00 if pending.
CP_POLL
 TAX
 LDA CP_DONE
 AND CPMASK,X     ; CPMASK is our own table, indexed LDA is safe
 BEQ :pnd
 LDA #$80
 RTS
:pnd
 LDA #$00
 RTS

* CP_WAIT: A=handle. Spins reading CP_DONE until the slot's bit is set.
CP_WAIT
 STA CPLSLOT
:wl
 LDX CPLSLOT
 LDA CP_DONE
 AND CPMASK,X
 BEQ :wl
 RTS

* CP_RESULT: A=handle. Reads result from mailbox+16; frees the slot. Returns result in A.
CP_RESULT
 STA CPLSLOT
 ; result lo = slot*32 + $90, hi = $0F
 ; slot 0->$90  slot 1->$B0  slot 2->$D0  slot 3->$F0  (no carry)
 ASL
 ASL
 ASL
 ASL
 ASL              ; slot * 32
 CLC
 ADC #$90         ; + $10 result offset from mailbox base
 STA CP_LADDRLO
 LDA #$0F
 STA CP_LADDRHI
 LDA CP_RDATA     ; read result byte (laddr autoincs, but we only need 1 byte)
 STA CPLRES
 LDA CPLSLOT
 STA CP_COLLECT   ; free the slot (write face of $C0CC)
 LDA CPLRES       ; return result
 RTS

CPMASK DFB $01,$02,$04,$08   ; bit mask for slot 0..3
CPLSK  DS 1                  ; saved skill_id
CPLARG DS 1                  ; saved arg0
CPLBUD DS 1                  ; saved budget
CPLSLOT DS 1                 ; working handle / free slot
CPLRES DS 1                  ; saved result (across CP_COLLECT strobe)
