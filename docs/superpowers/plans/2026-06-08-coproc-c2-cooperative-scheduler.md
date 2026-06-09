# Coprocessor C2 — Cooperative Scheduler + The Race Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Two host-registered tasks interleave under a cooperative scheduler on the soft-6502, finish in a host-observable order, and re-arming with a raised budget flips the order — the async-multitasking race.

**Architecture:** Almost entirely firmware — a `kernel.S` rewrite into an N-task cooperative scheduler (per-task TCBs in ZP, per-task page-1 stacks, a `YIELD` context switch + centralized `RESTORE`, `BOOTSTRAP` on each `COUNT` 0→nonzero edge, `DONE`/idle). One small gateware add: a `CORE_ID` parameter + `$E011` read so the firmware is core-relative (multi-core-ready). A fixed kernel API jump table (`$1003`=YIELD, `$1006`=DONE) decouples tasks from kernel internals.

**Tech Stack:** Verilog-2005 (Icarus `-g2005` only), Yosys/nextpnr ECP5, Merlin32. Build `make DESIGN=project_obscurus REV=rev2`; sim `make sim DESIGN=project_obscurus REV=rev2`. Branch `coproc`. Spec: `docs/superpowers/specs/2026-06-08-coproc-c2-cooperative-scheduler-design.md`.

**Fixed constants:** MAX_TASKS=4, STACK_SIZE=`$30`. Kernel ZP `$D0-$F3` (TCB arrays + NPARAM + CUR/ORDER/COREBASE/TMP). Kernel API: `$1000`=JMP RESET, `$1003`=JMP YIELD, `$1006`=JMP DONE. Task entry `$0300`. RESULT_BASE = SDRAM bank0 `$0061`; STRIDE_LOG2=4. CORE_ID read at `$E011`. Per-task stack tops `$012F/$015F/$018F/$01BF`, kernel boot stack `$01FF`.

---

## File Structure
- `coproc.v` — add `parameter CORE_ID` + `$E011` decode.
- `project_obscurus_top.v` — instantiate `coproc #(.CORE_ID(0))`.
- `kernel.S` → `kernel.mem` — REWRITE: cooperative scheduler.
- `software/SDM/racetask.S` — the parameterized race task.
- `software/SDM/CPRACE.S` — host loader (register 2, race, re-arm, flip).
- `project_obscurus_tb.v` — integration: race + flip.
- `Makefile` — `cprace` target + disk pack.

`sdram_ctrl.v`, `sdram_arb.v`, `arlet_*` unchanged.

---

## Task 1: `coproc.v` — `CORE_ID` parameter + `$E011`

**Files:** Modify `coproc.v`, `project_obscurus_top.v`.

- [ ] **Step 1: Add the parameter + `$E011` decode to `coproc.v`**

Change the module declaration line `module coproc (` to add a parameter:
```verilog
module coproc #(
    parameter [7:0] CORE_ID = 8'd0
) (
```
Add the `$E011` select next to the existing `is_count` (find `wire is_count=(AB==16'hE010);`):
```verilog
    wire is_count =(AB==16'hE010);
    wire is_coreid=(AB==16'hE011);
```
Add its registered select to the port-A `always @(posedge clk)` block (alongside `is_count_q <= is_count;`):
```verilog
        is_count_q  <= is_count;
        is_coreid_q <= is_coreid;
```
Declare `is_coreid_q` in the same `reg ... is_count_q;` line. Add it to the DI mux (find `: is_count_q ? task_count`):
```verilog
              : is_count_q  ? task_count
              : is_coreid_q ? CORE_ID
              : in_bram_q   ? bram_qa
```

- [ ] **Step 2: Instantiate with `CORE_ID=0` in the top**

In `project_obscurus_top.v`, change `coproc u_coproc (` to:
```verilog
    coproc #(.CORE_ID(8'd0)) u_coproc (
```
(All ports unchanged.)

- [ ] **Step 3: Build**

Run: `make clean && make DESIGN=project_obscurus REV=rev2`
Expected: clean bitstream, timing PASS, DP16KD still 8. (CORE_ID adds one decode + one constant.) Report Fmax + DP16KD. Do NOT flash. (`kernel.mem` from C1 is still valid for this build — Task 2 rewrites it.)

- [ ] **Step 4: Commit**

```bash
git add gateware/rev2/project_obscurus/coproc.v gateware/rev2/project_obscurus/project_obscurus_top.v
git commit -m "feat(coproc-c2): CORE_ID parameter + \$E011 read (multi-core-ready firmware)"
```
Trailer on every commit: `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`

---

## Task 2: `kernel.S` — cooperative scheduler (the core)

**Files:** Rewrite `gateware/rev2/project_obscurus/kernel.S`.

- [ ] **Step 1: Write the full `kernel.S`** (Merlin32, single-space fields, ASCII)

```
* kernel.S - C2 coproc cooperative scheduler. ORG $1000 (write-protected region).
* Fixed API: $1000 JMP RESET (reset vector), $1003 JMP YIELD, $1006 JMP DONE.
* Per-task TCBs in ZP $D0-, per-task page-1 stacks, YIELD/RESTORE context switch,
* BOOTSTRAP on each COUNT 0->nonzero edge, DONE/idle. ISR stubs $1F00/$1F40.
 LST OFF
 TYP $06
 DSK kernel.bin
 ORG $1000

* ---- kernel-reserved ZP (MAX_TASKS=4) ----
TCB_SP = $D0
TCB_A  = $D4
TCB_X  = $D8
TCB_Y  = $DC
TCB_P  = $E0
TCB_ST = $E4      ; 0=EMPTY 1=READY 2=DONE
NPARAM = $E8
CUR    = $EC
ORDER  = $ED
COREBL = $EE
COREBH = $EF
TMPA   = $F0
TMPX   = $F1
TMPY   = $F2
TMPP   = $F3

COUNT  = $E010
COREID = $E011
SADRLO = $E000
SADRHI = $E001
SBANK  = $E002
SDATA  = $E003

* ---- fixed API jump table (task-facing) ----
 JMP RESET        ; $1000  (reset vector -> here)
 JMP YIELD        ; $1003
 JMP DONE         ; $1006

RESET
 SEI
 LDX #$FF
 TXS
KWAIT0
 LDA COUNT
 BNE KWAIT0       ; wait COUNT==0
KWAITN
 LDA COUNT
 BEQ KWAITN       ; wait COUNT!=0 (arm edge)
 JSR BOOTSTRAP    ; ends JMP RESTORE -> task0; never returns here
KIDLE
 LDX #$FF
 TXS
 JMP KWAIT0       ; re-arm

* ---- BOOTSTRAP: COREBASE, ORDER=0, init TCBs[0..COUNT-1], EMPTY the rest, start task0 ----
BOOTSTRAP
 LDA COREID
 ASL
 ASL
 ASL
 ASL              ; A = CORE_ID*16
 CLC
 ADC #$61         ; + RESULT_BASE lo ($0061)
 STA COREBL
 LDA #$00         ; RESULT_BASE hi
 ADC #$00
 STA COREBH
 LDA #$00
 STA ORDER
 LDX #$00
BSL
 CPX COUNT
 BCS BSEMP
 LDA #$01
 STA TCB_ST,X
 LDA #$00
 STA TCB_A,X
 STA TCB_X,X
 STA TCB_Y,X
 LDA #$04
 STA TCB_P,X
 LDY STKTOP,X     ; partition top (page-1 offset)
 LDA #$02         ; >(ENTRY-1) = >$02FF
 STA $0100,Y
 DEY
 LDA #$FF         ; <(ENTRY-1) = <$02FF
 STA $0100,Y
 DEY
 TYA
 STA TCB_SP,X     ; SP = top-2 (RESTORE RTS -> $0300)
 INX
 JMP BSL
BSEMP
 CPX #$04         ; MAX_TASKS
 BCS BSGO
 LDA #$00
 STA TCB_ST,X
 INX
 JMP BSEMP
BSGO
 LDA #$00
 STA CUR
 JMP RESTORE

* ---- RESTORE: resume task CUR (shared by YIELD, DONE, BOOTSTRAP) ----
* Stages A+P on the NEW task's stack so Y (the TCB index) is restored after all
* TCB_*,Y reads, and PLP is the LAST flag op before RTS (task's BNE sees its own Z).
RESTORE
 LDY CUR
 LDX TCB_SP,Y
 TXS
 LDA TCB_P,Y
 PHA              ; stage P
 LDA TCB_A,Y
 PHA              ; stage A
 LDA TCB_Y,Y      ; A = task's Y
 LDX TCB_X,Y      ; X restored (final TCB_*,Y read)
 TAY              ; Y restored (index retired)
 PLA              ; A restored
 PLP              ; P restored (LAST flag op)
 RTS

* ---- YIELD: save current task via TMP*, pick next, fall into RESTORE ----
YIELD
 STA TMPA
 STX TMPX         ; save X BEFORE the TSX below
 STY TMPY
 PHP
 PLA
 STA TMPP
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
 LDA TMPP
 STA TCB_P,Y
 JSR PICKNEXT     ; CUR = next READY (round-robin)
 JMP RESTORE

* ---- DONE: mark CUR done, switch to any READY or idle ----
DONE
 LDY CUR
 LDA #$02
 STA TCB_ST,Y
 JSR PICKANY      ; CUR=any READY; carry set if none
 BCS KIDLE
 JMP RESTORE

* ---- PICKNEXT: CUR = next READY after CUR (wrap); if none other, CUR stays ----
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

* ---- PICKANY: CUR = lowest-id READY; carry set if none ----
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

STKTOP DFB $2F,$5F,$8F,$BF

 DS $1F00-*
IRQH RTI
 DS $1F40-*
NMIH RTI
```

- [ ] **Step 2: Build `kernel.mem` (full-8KB image) + verify the API table**

The Makefile rule already emits the full-8KB image (`m[0x1000:...]=b`, from the DP16KD fix). Run:
```bash
cd /Users/hambook/Development/project_byte_hamr/gateware/rev2/project_obscurus
/Users/hambook/Development/Merlin32_v1.2/MacOs/Merlin32 /Users/hambook/Development/Merlin32_v1.2/Library kernel.S
python3 -c "b=open('kernel.bin','rb').read(); m=bytearray(8192); m[0x1000:0x1000+len(b)]=b; open('kernel.mem','w').write(chr(10).join('%02x'%x for x in m)+chr(10))"
sed -n '4097p;4100p;4103p' kernel.mem
```
Expected: line 4097 (`$1000`) = `4c` (JMP RESET); line 4100 (`$1003`) = `4c` (JMP YIELD); line 4103 (`$1006`) = `4c` (JMP DONE). The reset vector ($FFFC/D→$1000) lands on `JMP RESET`. If not `4c` at all three, the API table layout is wrong — STOP and report.

- [ ] **Step 3: Build the bitstream + confirm EBR/timing**

Run: `cd /Users/hambook/Development/project_byte_hamr && make clean && make DESIGN=project_obscurus REV=rev2`
Expected: clean bitstream, DP16KD 8, timing PASS. (kernel.mem grew but stays in `$1000-$1FFF`.) Report numbers. Do NOT flash.

- [ ] **Step 4: Commit**

```bash
git add gateware/rev2/project_obscurus/kernel.S gateware/rev2/project_obscurus/kernel.mem
git commit -m "feat(coproc-c2): kernel.S cooperative scheduler (YIELD/RESTORE/BOOTSTRAP/DONE, API table)"
```

---

## Task 3: race task + host loader + disk

**Files:** Create `software/SDM/racetask.S`, `software/SDM/CPRACE.S`; modify `Makefile`.

- [ ] **Step 1: Write `software/SDM/racetask.S`** (Merlin, ORG `$0300`; loaded into the coproc, not run on the host)

```
* racetask - coproc race task. Counts down NPARAM[id] yielding each step, then
* atomically grabs the finish order and stamps it to SDRAM result[COREBASE+id].
* Shares the kernel ZP ABI (CUR/NPARAM/ORDER/COREBASE) + the API table.
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
 LDY CUR          ; my id (do NOT assume Y on entry)
 LDX NPARAM,Y     ; my budget
RLOOP
 DEX
 JSR YIELD        ; X (budget) + Z (from DEX) preserved across yield via TCB
 BNE RLOOP
* finished: order = ++ORDER (atomic - no YIELD between)
 LDA ORDER
 CLC
 ADC #$01
 STA ORDER
 PHA              ; save my order
* result addr = COREBASE + CUR
 LDA COREBL
 CLC
 ADC CUR
 STA SADRLO
 LDA COREBH
 ADC #$00
 STA SADRHI
 LDA #$00
 STA SBANK
 PLA              ; my order
 STA SDATA        ; post SDRAM write result[COREBASE+CUR] = order
 JSR DONE
```
Assemble + capture bytes:
```bash
cd /Users/hambook/Development/project_byte_hamr/software/SDM
/Users/hambook/Development/Merlin32_v1.2/MacOs/Merlin32 /Users/hambook/Development/Merlin32_v1.2/Library racetask.S
xxd racetask.bin
```
Record the byte string + length (`RTLEN`) — needed for the loader's DFB table and the sim.

- [ ] **Step 2: Write `software/SDM/CPRACE.S`** (host loader, ORG `$6000`)

Structure (reuse C1 CPREG's `LOADBLK` + `SDM_READ`/`PRBYTE` patterns; read `software/SDM/CPREG.S` + `SDRAMLIB.S` first):
```
* CPRACE.S - C2 host loader. Registers ONE racetask binary at $0300 with 2 TABLE
* entries (ids 0,1), sets NPARAM 10/20, runs the race, prints orders; then re-arms
* (COUNT=0 / NPARAM 30/20 / COUNT=2) and prints the flipped orders.
 TYP $06
 DSK CPRACE
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

* load LEN bytes from (SRCL),Y to coproc BRAM at LADHI:LADLO (non-indexed STA CPWDATA)
LOADBLK
 LDA LADLO
 STA CPLADDRLO
 LDA LADHI
 STA CPLADDRHI
 LDY #0
:lp LDA (SRCL),Y
 STA CPWDATA
 INY
 CPY LEN
 BNE :lp
 RTS

* write one byte D to coproc BRAM at A=lo (page $00xx) [helper for NPARAM/TABLE]
* set CP_LADDR then STA CPWDATA once
PBYTE
 STA CPLADDRLO        ; A = laddr lo
 LDA #0
 STA CPLADDRHI        ; high (page 0 / 2 set by caller via direct stores below)
 RTS

MAIN JSR SDM_READY
* --- load racetask code -> coproc $0300 ---
 LDA #$00
 STA LADLO
 LDA #$03
 STA LADHI
 LDA #RTLEN            ; racetask length (from Step 1 xxd)
 STA LEN
 LDA #<RTASK
 STA SRCL
 LDA #>RTASK
 STA SRCH
 JSR LOADBLK
* --- TABLE entry0 = $0300 @ coproc $0200 ; entry1 = $0300 @ $0202 ---
 LDA #$00
 STA CPLADDRLO
 LDA #$02
 STA CPLADDRHI        ; laddr $0200
 LDA #$00
 STA CPWDATA          ; e0 lo
 LDA #$03
 STA CPWDATA          ; e0 hi
 LDA #$00
 STA CPWDATA          ; e1 lo ($0202, auto-inc continued)
 LDA #$03
 STA CPWDATA          ; e1 hi
* --- NPARAM[0]=10, NPARAM[1]=20 @ coproc ZP $E8/$E9 ---
 LDA #$E8
 STA CPLADDRLO
 LDA #$00
 STA CPLADDRHI        ; laddr $00E8
 LDA #10
 STA CPWDATA          ; NPARAM[0]
 LDA #20
 STA CPWDATA          ; NPARAM[1] ($00E9)
* --- COUNT = 2 (arm) ---
 LDA #$02
 STA CPCOUNT
* --- wait for completion (both result cells written) then read+print ---
 JSR WAITRACE
 JSR SHOWRESULTS      ; prints result[$0061] result[$0062]
* --- re-arm: COUNT=0, NPARAM 30/20, COUNT=2 ---
 LDA #$00
 STA CPCOUNT
 LDA #$E8
 STA CPLADDRLO
 LDA #$00
 STA CPLADDRHI
 LDA #30
 STA CPWDATA          ; NPARAM[0]
 LDA #20
 STA CPWDATA          ; NPARAM[1]
 LDA #$02
 STA CPCOUNT
 JSR WAITRACE
 JSR SHOWRESULTS      ; prints flipped orders
 RTS

* WAITRACE: ample spin so both tasks reach DONE (kernel parks in KWAIT0) BEFORE
* the caller re-arms. (Re-arm-before-completion HANGS the kernel - see spec.)
WAITRACE
 LDX #$00
:w1 LDY #$00
:w2 INY
 BNE :w2
 INX
 BNE :w1
 RTS

* SHOWRESULTS: read SDRAM bank0 $0061 + $0062, print each as hex, CR
SHOWRESULTS
 LDA #$00
 STA SDM_BANK
 STA SDM_BANK+1
 LDA #$61
 STA SDM_ADDR
 LDA #$00
 STA SDM_ADDR+1
 JSR SDM_READ
 LDA SDM_VAL
 JSR PRBYTE
 LDA #$00
 STA SDM_BANK
 STA SDM_BANK+1
 LDA #$62
 STA SDM_ADDR
 LDA #$00
 STA SDM_ADDR+1
 JSR SDM_READ
 LDA SDM_VAL
 JSR PRBYTE
 JSR CROUT
 RTS

RTASK  DFB <paste exact racetask.bin bytes from Step 1>
RTLEN  = *-RTASK
```
NOTE TO IMPLEMENTER:
- Paste the EXACT `racetask.bin` bytes into the `RTASK` `DFB` (verify vs xxd); `RTLEN` derives from it. Replace the `#RTLEN` immediate in MAIN with the literal length (or use `LDA #RTLEN` — confirm Merlin resolves `*-RTASK` before MAIN; if not, hardcode the length).
- Confirm `SDM_READ`/`SDM_VAL`/`PRBYTE` against C1's CPREG (same proven pattern: poke `SDM_BANK`/`SDM_ADDR`, JSR `SDM_READ`, value in `SDM_VAL`).
- All `STA CPWDATA` / `STA CPCOUNT` are non-indexed.
- The `PBYTE` helper above is unused scaffolding — delete it; direct stores are used.

- [ ] **Step 3: Makefile `cprace` target + disk pack**

After the `cpreg` target add:
```makefile
cprace:
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) racetask.S
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) CPRACE.S
```
Add `cprace` to `.PHONY` and to the `sdmdisk` deps; add the pack line:
```makefile
	$(AC_CLASSIC) -p $(SDM_PO) CPRACE BIN 0x6000 < $(SDM_DIR)/CPRACE
```
Run `make cprace` (CPRACE first byte `4c`), `make sdmdisk`; confirm `CPRACE` in the catalog.

- [ ] **Step 4: Commit**

```bash
git add software/SDM/racetask.S software/SDM/CPRACE.S Makefile
git commit -m "feat(coproc-c2): racetask + CPRACE host loader (race + re-arm flip)"
```

---

## Task 4: Integration sim — race, then flip

**Files:** Modify `project_obscurus_tb.v`.

- [ ] **Step 1: Add the C2 integration block**

READ the tb to confirm `wr_reg`/`rd_reg`/`sdram_read`/`load_byte`/`cp_read`/`tmp`/`errors` (C1 added `load_byte` + `cp_read`). Use SMALL budgets (4/8) to keep cycle counts short. Insert after the existing checks, before the final summary. The task code bytes are the `racetask.bin` from Task 3 — paste the exact bytes (here shown as `RT_x`; substitute real values):
```verilog
        // ===== C2: cooperative race (COUNT-LAST, re-arm flip) =====
        // load racetask at coproc $0300
        wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h03);   // CP_LADDR = $0300
        // <<< load_byte(8'hXX) for each racetask.bin byte, in order >>>
        // TABLE: entry0=$0300 @ $0200, entry1=$0300 @ $0202
        wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h02);
        load_byte(8'h00); load_byte(8'h03); load_byte(8'h00); load_byte(8'h03);
        // NPARAM[0]=4, NPARAM[1]=8 @ coproc ZP $E8/$E9
        wr_reg(4'h9, 8'hE8); wr_reg(4'hA, 8'h00);
        load_byte(8'd4); load_byte(8'd8);
        // arm
        wr_reg(4'hD, 8'h02);                        // COUNT=2
        repeat (20000) @(posedge clk);              // ample: both finish, kernel parks
        sdram_read(10'd0, 16'h0061, tmp);
        if (tmp!==8'h01) begin errors=errors+1; $display("FAIL C2 race r0 %02X (want 01)",tmp); end
        else $display("PASS C2 race task0 finished first (r0=01)");
        sdram_read(10'd0, 16'h0062, tmp);
        if (tmp!==8'h02) begin errors=errors+1; $display("FAIL C2 race r1 %02X (want 02)",tmp); end
        else $display("PASS C2 race task1 second (r1=02)");
        // re-arm + flip: COUNT=0, NPARAM 12/8, COUNT=2
        wr_reg(4'hD, 8'h00);                        // COUNT=0 (kernel parked sees it)
        wr_reg(4'h9, 8'hE8); wr_reg(4'hA, 8'h00);
        load_byte(8'd12); load_byte(8'd8);
        wr_reg(4'hD, 8'h02);                        // COUNT=2 (arm edge)
        repeat (20000) @(posedge clk);
        sdram_read(10'd0, 16'h0061, tmp);
        if (tmp!==8'h02) begin errors=errors+1; $display("FAIL C2 flip r0 %02X (want 02)",tmp); end
        else $display("PASS C2 flip task0 now second (r0=02)");
        sdram_read(10'd0, 16'h0062, tmp);
        if (tmp!==8'h01) begin errors=errors+1; $display("FAIL C2 flip r1 %02X (want 01)",tmp); end
        else $display("PASS C2 flip task1 now first (r1=01)");
```

- [ ] **Step 2: Run the sim**

Run: `cd /Users/hambook/Development/project_byte_hamr && make sim DESIGN=project_obscurus REV=rev2`
Expected: the 4 C2 PASS lines + all pre-existing (C0 taskC, C1 protection) still PASS, final 0 errors.

DEBUG (root-cause, no masking):
- `r0=00`/never written: the scheduler didn't run a task. Check kernel boots (`$1000` JMP), COUNT reaches it, BOOTSTRAP seeds stacks, RESTORE RTSes to `$0300`. cp_read coproc `$0300` to confirm racetask loaded; cp_read `$00E8/$E9` for NPARAM.
- both `01` (no order): the ORDER counter or finish path is wrong, or one task hung (uncooperative). Confirm racetask's `JSR YIELD` ($1003) + `JSR DONE` ($1006) addresses match the API table.
- flip didn't happen: the re-arm edge (`COUNT 0→2`) didn't re-bootstrap — check KWAIT0/KWAITN; ensure the `repeat` wait is long enough that both tasks DONE before the COUNT=0 (else the kernel hangs in KWAIT0 — increase the first `repeat`).
- HANG/X: budget too large for the `repeat` window; or RESTORE register order wrong (task's BNE mis-fires). Verify RESTORE matches the spec (PLP last).

- [ ] **Step 3: Commit**

```bash
git add gateware/rev2/project_obscurus/project_obscurus_tb.v
git commit -m "test(coproc-c2): integration — cooperative race orders 1/2, re-arm flips to 2/1"
```

---

## Task 5: Bench verification (user-run)

**Files:** none. The **user** flashes.

- [ ] **Step 1: Build**

Run: `make clean && make DESIGN=project_obscurus REV=rev2` — clean, timing, DP16KD 8. Report "ready".

- [ ] **Step 2: Hand the procedure to the user**

```
1. (you) flash build/project_obscurus.bit
2. Boot the /SDRAM/ disk; BRUN CPRACE
   -> prints:  01 02      (task0 N=10 finished first, task1 N=20 second)
               02 01      (after re-arm with N=30/20 -> task1 wins)
```
Pass = `01 02` then `02 01`. Two tasks interleaved under the cooperative scheduler, finished in a host-observable order, and raising one budget flipped the winner — **async multitasking on the soft-6502, proven.**

If it prints `01 02` then hangs (no second line): re-arm fired before completion → kernel stuck in KWAIT0 (the `WAITRACE` spin was too short — lengthen it). If `00 00`: scheduler didn't run — check kernel.mem in the bitstream + the API table.

- [ ] **Step 3: Record result**

Update `project_coproc_c0.md` + `MEMORY.md` with C2 bench status (the race + flip), the scheduler/ABI, and C3 (preemptive tick) as next.

---

## Self-Review

**Spec coverage:** CORE_ID + $E011 → Task 1. Cooperative scheduler (TCB ZP, per-task stacks, YIELD/RESTORE/BOOTSTRAP/DONE, COUNT-edge re-bootstrap, idle) → Task 2 kernel.S. Centralized RESTORE (stage A+P, PLP last, restore Y last) → Task 2 RESTORE. Core-relative COREBASE (shift not mul) → Task 2 BOOTSTRAP. Race task (LDY CUR, budget in X across YIELD, atomic ORDER, core-relative result) → Task 3 racetask. Host loader register-2 + re-arm flip + WAITRACE completion wait → Task 3 CPRACE. count-last + re-arm-wait → CPRACE/tb. Proof (1/2 then 2/1) → Task 4/5. Multi-core honesty (CORE_ID=0 exercises base) → CORE_ID=0 instantiation.

**Placeholder scan:** racetask.bin bytes / RTLEN flagged as paste-from-xxd (Task 3 Step 1, Task 4 Step 1) with the exact source given — a real capture step, not a TODO. `PBYTE` scaffolding explicitly deleted. No other gaps.

**Type/label consistency:** kernel ZP addresses ($D0-$F3) consistent kernel↔racetask ABI (CUR=$EC, NPARAM=$E8, ORDER=$ED, COREBL/H=$EE/$EF). API table $1003/$1006 = racetask's YIELD/DONE. $E011 CORE_ID consistent coproc↔kernel. RESULT_BASE $0061 consistent kernel COREBASE↔CPRACE/tb reads. STKTOP $2F/$5F/$8F/$BF matches STACK_SIZE $30. RESTORE sequence matches spec exactly (stage P then A, restore Y last via the staged A, PLP last). Budgets: 10/20 bench, 4/8 sim.

**Executor notes:** iverilog `-g2005`. `make ... REV=rev2`. Non-indexed `STA $C0CB/$C0CD`. RESTORE/YIELD register order is load-bearing (PLP last, save X before TSX, restore Y last). WAITRACE must outlast both tasks (re-arm-before-DONE hangs KWAIT0). Don't modify sdram_ctrl/sdram_arb/arlet_*.
