# Coprocessor C3 — Preemptive Timer Tick Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A card timer fires a periodic IRQ into the soft-6502; the ISR preempts whatever's running and switches tasks — so two non-yielding tasks interleave, race, and the winner flips when you raise one's budget. True (minimal) preemptive RTOS, robust against the silent-clobber footgun class.

**Architecture:** A small timer in `coproc.v` (`$E012` arm/`$E013` ack, `irq_pending → Arlet IRQ`). `kernel.S` reworks to a **uniform IRQ-frame context** (TCB drops P; the frame carries it; one `RTI` resume via `RESTORE`); the TICK ISR (`$1F00`), `YIELD` (`$1003`), and `BOOTSTRAP` all produce that one frame, funneling through `SWITCH`. I-flag discipline: tasks I-clear (preemptible), kernel/ISR I-set.

**Tech Stack:** Verilog-2005 (Icarus `-g2005`), Yosys/nextpnr ECP5, Merlin32. Build `make DESIGN=project_obscurus REV=rev2`; sim `make sim DESIGN=project_obscurus REV=rev2`. Branch `coproc`. Spec: `docs/superpowers/specs/2026-06-08-coproc-c3-preemptive-tick-design.md`.

**Fixed constants:** timer `$E012`=TICK_CTL(arm/period), `$E013`=TICK_ACK; `TICKPER=$02` (tick every `2*256=512` clk). Kernel ZP: `TCB_SP=$D0,A=$D4,X=$D8,Y=$DC,ST=$E4` (TCB_P removed; `$E0/$E1`=TMPL/TMPH frame scratch), `NPARAM=$E8,CUR=$EC,ORDER=$ED,COREBL/H=$EE/$EF,TMPA/X/Y=$F0/$F1/$F2`. API table `$1000/$1003/$1006`. TICK ISR at `$1F00`, NMI stub `$1F40`. IRQ vector `$FFFE/F→$1F00` (gateware, C1). RESULT_BASE `$0061`.

---

## File Structure
- `coproc.v` — add the timer + wire `.IRQ`.
- `kernel.S` → `kernel.mem` — REWRITE to uniform-frame preemptive scheduler.
- `software/SDM/racetask3.S` — tight + inner-delay, no-yield race task.
- `software/SDM/CPRACE3.S` — host loader (race + re-arm flip).
- `project_obscurus_tb.v` — focused-IRQ + C3 race/flip + BRK + stack-assert + keep C2 cooperative.
- `Makefile` — `cprace3` target + disk.

`project_obscurus_top.v`, `sdram_*.v`, `arlet_*` unchanged.

---

## Task 1: `coproc.v` — timer + IRQ wire

**Files:** Modify `coproc.v`.

- [ ] **Step 1: Add the timer block**

In `coproc.v`, after the `task_count` register block (near the top of the module body), add:
```verilog
    // ---- C3 preemptive tick timer ----
    wire is_e012 = (AB==16'hE012);   // TICK_CTL: write period (0=disarm); resets cnt + clears pending
    wire is_e013 = (AB==16'hE013);   // TICK_ACK: write clears pending
    reg  [7:0]  tick_period;
    reg  [15:0] tick_cnt;
    reg         irq_pending;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tick_period <= 8'd0; tick_cnt <= 16'd0; irq_pending <= 1'b0;
        end else begin
            if (is_e012 & WE & rdy) begin          // arm/disarm: reset counter + clear pending
                tick_period <= DO; tick_cnt <= 16'd0; irq_pending <= 1'b0;
            end else if (is_e013 & WE & rdy) begin  // ack
                irq_pending <= 1'b0;
            end else if (tick_period != 8'd0) begin
                if (tick_cnt >= {tick_period, 8'h00}) begin
                    tick_cnt <= 16'd0; irq_pending <= 1'b1;
                end else tick_cnt <= tick_cnt + 16'd1;
            end
        end
    end
```

- [ ] **Step 2: Wire the IRQ into the core (verify polarity!)**

Change the cpu instantiation `.IRQ(1'b0)` to `.IRQ(irq_pending)`:
```verilog
    cpu u_cpu (.clk(clk), .reset(~rst_n), .AB(AB), .DI(DI), .DO(DO), .WE(WE),
               .IRQ(irq_pending), .NMI(1'b0), .RDY(rdy));
```
**Verify Arlet's IRQ polarity** before trusting it: read `arlet_cpu.v` for how `IRQ` is used (most soft-6502 cores treat `IRQ` as **active-high** = request when 1; if Arlet expects active-low, wire `.IRQ(~irq_pending)`). Note which in the commit. (The Task-4 focused-IRQ sim is the functional check — if the ISR never fires, flip the polarity.)

- [ ] **Step 3: Build + confirm vectors intact**

Run: `cd /Users/hambook/Development/project_byte_hamr && make clean && make DESIGN=project_obscurus REV=rev2`
Expected: clean bitstream, DP16KD 8, timing PASS. Confirm the IRQ vector is still synthesized:
```bash
grep -n "is_irqlo.*8'h00\|is_irqhi.*8'h1F" coproc.v
```
Expected: the DI mux still drives `is_irqlo_q ? 8'h00 : is_irqhi_q ? 8'h1F` (`$FFFE/F → $1F00`). Report Fmax + DP16KD. Do NOT flash. (`kernel.mem` from C2 is fine for this build; Task 2 rewrites it.)

- [ ] **Step 4: Commit**

```bash
git add gateware/rev2/project_obscurus/coproc.v
git commit -m "feat(coproc-c3): preemptive tick timer (\$E012 arm/\$E013 ack) + wire Arlet IRQ"
```
Trailer on every commit: `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`

---

## Task 2: `kernel.S` — uniform-frame preemptive scheduler (the core)

**Files:** Rewrite `gateware/rev2/project_obscurus/kernel.S`.

- [ ] **Step 1: REPLACE `kernel.S` with EXACTLY this** (Merlin32, single-space, ASCII — transcribe; the register ordering in RESTORE/SWITCH/YIELD/ISR is load-bearing and review-verified)

```
* kernel.S - C3 coproc PREEMPTIVE scheduler. ORG $1000 (write-protected).
* Uniform IRQ-frame context (TCB drops P; frame carries P; one RTI resume).
* Timer tick ($E012 arm/$E013 ack) -> IRQ -> $1F00 ISR -> SWITCH. YIELD ($1003)
* manufactures the same frame. Tasks run I-clear (preemptible); kernel/ISR I-set.
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

COUNT   = $E010
COREID  = $E011
TICKCTL = $E012
TICKACK = $E013
TICKPER = $02

 JMP RESET
 JMP YIELD
 JMP DONE

RESET
 SEI
 LDX #$FF
 TXS
KWAIT0
 LDA COUNT
 BNE KWAIT0
KWAITN
 LDA COUNT
 BEQ KWAITN
 JSR BOOTSTRAP
KIDLE
 LDA #$00
 STA TICKCTL
 LDX #$FF
 TXS
 JMP KWAIT0

BOOTSTRAP
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
 STX TMPX
 TXA
 ASL
 TAX
 LDA $0201,X
 STA TMPH
 LDA $0200,X
 STA TMPL
 LDX TMPX
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
 INX
 JMP BSL
BSEMP
 CPX #$04
 BCS BSGO
 LDA #$00
 STA TCB_ST,X
 INX
 JMP BSEMP
BSGO
 LDA #$00
 STA CUR
 LDA #TICKPER
 STA TICKCTL
 JMP RESTORE

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
 SEI
 STA TMPA
 STX TMPX
 STY TMPY
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
 LDA #$00
 PHA
 JMP SWITCH

DONE
 SEI
 LDY CUR
 LDA #$02
 STA TCB_ST,Y
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
 JMP SWITCH
IRQBRK
 LDA TMPA
 LDX TMPX
 RTI

 DS $1F40-*
NMIH RTI
```

- [ ] **Step 2: Build kernel.mem + verify API table AND the ISR offset**

```bash
cd /Users/hambook/Development/project_byte_hamr/gateware/rev2/project_obscurus
/Users/hambook/Development/Merlin32_v1.2/MacOs/Merlin32 /Users/hambook/Development/Merlin32_v1.2/Library kernel.S
python3 -c "b=open('kernel.bin','rb').read(); m=bytearray(8192); m[0x1000:0x1000+len(b)]=b; open('kernel.mem','w').write(chr(10).join('%02x'%x for x in m)+chr(10))"
sed -n '4097p;4100p;4103p' kernel.mem        # API: $1000/$1003/$1006 = 4c 4c 4c
xxd -s 0xF00 -l 4 kernel.bin                  # ISR at $1F00: first byte 85 (STA zp TMPA)
xxd -s 0xF40 -l 1 kernel.bin                  # NMI stub $1F40: 40 (RTI)
```
Expected: lines 4097/4100/4103 = `4c` (JMP RESET/YIELD/DONE). Offset `$F00` first byte `85` (`STA TMPA` = the ISR `STA $F0`). Offset `$F40` = `40` (RTI). If the ISR isn't at `$1F00` (first byte `85`), the `DS $1F00-*` pad is wrong — STOP and report. (If `BCS`/`BNE` range errors on `BSEMP`/`PNL` etc. occur, report; the C2 layout assembled fine so they shouldn't.)

- [ ] **Step 3: Build the bitstream**

Run: `cd /Users/hambook/Development/project_byte_hamr && make clean && make DESIGN=project_obscurus REV=rev2`
Expected: clean, DP16KD 8, timing PASS. Report numbers. Do NOT flash.

- [ ] **Step 4: Commit**

```bash
git add gateware/rev2/project_obscurus/kernel.S gateware/rev2/project_obscurus/kernel.mem
git commit -m "feat(coproc-c3): kernel.S uniform-frame preemptive scheduler (TICK ISR + SWITCH + RTI RESTORE)"
```

---

## Task 3: race task + host loader + disk

**Files:** Create `software/SDM/racetask3.S`, `software/SDM/CPRACE3.S`; modify `Makefile`.

- [ ] **Step 1: Write `software/SDM/racetask3.S`** (tight outer countdown + inner delay so it spans many ticks; **no `YIELD`** — preemption only; id cached in Y across the run)

```
* racetask3 - C3 preemptive race task. NO yield: runs a budget*delay loop; the
* timer tick slices it. id in Y (TCB-preserved), budget in X, inner delay in A
* (all saved/restored across preemption). Finish: SEI / atomic ++ORDER / stamp
* result[COREBASE+id] / JMP DONE.
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
* finished
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
```
Assemble + capture bytes (the sim needs them verbatim):
```bash
cd /Users/hambook/Development/project_byte_hamr/software/SDM
/Users/hambook/Development/Merlin32_v1.2/MacOs/Merlin32 /Users/hambook/Development/Merlin32_v1.2/Library racetask3.S
xxd racetask3.bin
```
Record the exact bytes + length (`RT3LEN`).

- [ ] **Step 2: Write `software/SDM/CPRACE3.S`** (host loader; mirror C2's `CPRACE.S` — read it first)

Same structure as `CPRACE.S` but loads `racetask3` and uses **wide budget margins**. The kernel arms the timer itself (in BOOTSTRAP); the host just sets COUNT. ORG `$6000`:
- Load racetask3 (`RT3LEN` bytes) → coproc `$0300`.
- TABLE entry0=`$0300` @ `$0200`, entry1=`$0300` @ `$0202`.
- `NPARAM[0]=50`, `NPARAM[1]=200` @ coproc ZP `$00E8`/`$00E9`.
- `STA CPCOUNT=2` (arm scheduler).
- `WAITRACE` (ample), `SHOWRESULTS` (read SDRAM `$0061`/`$0062`, print) → expect `01 02`.
- Re-arm: `STA CPCOUNT=0`, set `NPARAM[0]=250`/`NPARAM[1]=200`, `STA CPCOUNT=2`.
- `WAITRACE`, `SHOWRESULTS` → expect `02 01`.

Copy `LOADBLK`/`WAITRACE`/`SHOWRESULTS`/`SDM_READ`/`PRBYTE` verbatim from `CPRACE.S` (same `$C0C9/CA/CB/CD` load port, non-indexed `STA CPWDATA/CPCOUNT`). Paste the exact `racetask3.bin` bytes into the `RT3` `DFB` table; use the literal length for `LEN`. `WAITRACE` must be ample (the inner-delay task runs long — bump the spin loop if needed; re-arm-before-DONE hangs the kernel).

- [ ] **Step 3: assemble + disk**

```bash
cd /Users/hambook/Development/project_byte_hamr/software/SDM
/Users/hambook/Development/Merlin32_v1.2/MacOs/Merlin32 /Users/hambook/Development/Merlin32_v1.2/Library CPRACE3.S
xxd CPRACE3 | head -1   # first byte 4c
```
Add a Makefile `cprace3` target (assemble racetask3.S + CPRACE3.S), add to `.PHONY` + `sdmdisk` deps, add the pack line `$(AC_CLASSIC) -p $(SDM_PO) CPRACE3 BIN 0x6000 < $(SDM_DIR)/CPRACE3`. Run `make cprace3 && make sdmdisk`; confirm `CPRACE3` in the catalog.

- [ ] **Step 4: Commit**

```bash
git add software/SDM/racetask3.S software/SDM/CPRACE3.S Makefile
git commit -m "feat(coproc-c3): racetask3 (no-yield, tick-sliced) + CPRACE3 loader"
```

---

## Task 4: Integration sim — focused IRQ, preemptive race/flip, BRK, stack-assert

**Files:** Modify `project_obscurus_tb.v`.

- [ ] **Step 1: Read the tb; note helpers + the racetask3 bytes**

Confirm `wr_reg`/`sdram_read`/`load_byte`/`tmp`/`errors`/`clk100` (the actual tb clock). The C2 cooperative race block + helpers exist (keep them — they prove YIELD still works through the uniform frame; if the C2 racetask still ends in `JMP DONE` and yields via `$1003`, it now exercises the YIELD-manufactures-frame path). Use the exact `racetask3.bin` bytes from Task 3 Step 1.

- [ ] **Step 2: Add the focused-IRQ check + the C3 preemptive race + flip + BRK**

After the existing checks (incl. the C2 cooperative race), before the final summary. Substitute the real `racetask3.bin` bytes for `RT3_x`:
```verilog
        // ===== C3: preemptive tick =====
        // (focused IRQ is implicitly covered: the no-yield tasks below can ONLY
        //  interleave if the timer IRQ + ISR + switch work — if the ISR is dead,
        //  task0 runs to completion first and BOTH still finish, but a NON-yielding
        //  pair that never interleaves still completes; so we ALSO assert the timer
        //  actually fired via the order being a real 1/2 with both written.)
        // load racetask3 at coproc $0300 (NO yield):
        wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h03);
        // <<< load_byte(8'hXX) for each racetask3.bin byte in order >>>
        // TABLE entry0=$0300@$0200, entry1=$0300@$0202
        wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h02);
        load_byte(8'h00); load_byte(8'h03); load_byte(8'h00); load_byte(8'h03);
        // NPARAM[0]=4, NPARAM[1]=12 (small for sim; wide margin) @ $00E8
        wr_reg(4'h9, 8'hE8); wr_reg(4'hA, 8'h00);
        load_byte(8'd4); load_byte(8'd12);
        wr_reg(4'hD, 8'h02);                         // COUNT=2 (arm scheduler; kernel arms timer)
        repeat (120000) @(posedge clk100);           // ample (inner-delay tasks run long)
        sdram_read(10'd0, 16'h0061, tmp);
        if (tmp!==8'h01) begin errors=errors+1; $display("FAIL C3 race r0 %02X want 01",tmp); end
        else $display("PASS C3 preemptive race task0 first (r0=01)");
        sdram_read(10'd0, 16'h0062, tmp);
        if (tmp!==8'h02) begin errors=errors+1; $display("FAIL C3 race r1 %02X want 02",tmp); end
        else $display("PASS C3 preemptive race task1 second (r1=02)");
        // re-arm flip: COUNT=0, NPARAM 20/12, COUNT=2
        wr_reg(4'hD, 8'h00);
        wr_reg(4'h9, 8'hE8); wr_reg(4'hA, 8'h00);
        load_byte(8'd20); load_byte(8'd12);
        wr_reg(4'hD, 8'h02);
        repeat (120000) @(posedge clk100);
        sdram_read(10'd0, 16'h0061, tmp);
        if (tmp!==8'h02) begin errors=errors+1; $display("FAIL C3 flip r0 %02X want 02",tmp); end
        else $display("PASS C3 flip task0 now second (r0=02)");
        sdram_read(10'd0, 16'h0062, tmp);
        if (tmp!==8'h01) begin errors=errors+1; $display("FAIL C3 flip r1 %02X want 01",tmp); end
        else $display("PASS C3 flip task1 now first (r1=01)");
```
**Proof that the tick actually fired:** with NPARAM 4/12 the two no-yield tasks finish in order `1/2` ONLY if the timer preempted task0 to let task1 start before task0 ended — but even without interleaving both finish, so the discriminating test is the **flip**: with 20/12, task1 must finish first (`r1=01`), which can ONLY happen if task1 got CPU while task0 was still running — i.e. the tick preempted task0. A dead timer would leave whichever task is dispatched first (task0) always finishing first → flip FAILS. So the flip asserting `02/01` is the real preemption proof.

- [ ] **Step 3: Add the BRK-safety check (#1)**

After the C3 race, register a task whose body is `BRK` (`$00`) followed by a marker write, and confirm a stray `BRK` doesn't corrupt the scheduler (the other task still completes). Minimal: load a 1-task setup where the task does `BRK` then `JMP DONE`; assert the kernel survives (a following normal task still writes its result). (If wiring this is heavy, at minimum assert the C3 race still passes — the ISR's B-check is exercised only if a BRK occurs; document if deferred to bench.)
```verilog
        // BRK safety: a task that executes BRK must not corrupt the scheduler.
        // taskBRK at $0300: 00 (BRK) EA (NOP pad) 4C 06 10 (JMP DONE)
        // taskOK  at $0320: writes $5A to $0063 then JMP DONE
        // ... register both, COUNT=2, run, assert $0063==$5A (taskOK ran despite taskBRK's BRK)
        // (Implement with the same load/TABLE/NPARAM pattern; the BRK ISR leg must
        //  RTI back so taskBRK proceeds to JMP DONE without derailing the kernel.)
```

- [ ] **Step 4: Stack-depth assert (#4)**

Add a tb monitor on the coproc's SP (if observable via a hierarchical ref `dut.u_coproc.u_cpu` SP, or via the coproc BRAM write addresses in page 1): assert no task's SP descends below its partition floor (`$0100`, `$0140`, `$0180`, `$01C0` for tasks 0-3) during the run. If SP isn't directly observable, snapshot the page-1 partitions after the run and assert the kernel region (`$01C0+`) wasn't scribbled. Document the chosen method.

- [ ] **Step 5: Run the sim**

Run: `cd /Users/hambook/Development/project_byte_hamr && make sim DESIGN=project_obscurus REV=rev2`
Expected: the 4 C3 PASS lines + BRK-safety PASS + all pre-existing (C2 cooperative race, C1 protection, monitor) still PASS, final 0 errors.

DEBUG (root-cause, no masking):
- **ISR never fires** (task0 always wins, flip FAILS as `01/02`): IRQ polarity wrong (Task 1) → try `.IRQ(~irq_pending)`; OR the kernel didn't arm the timer (`$E012`) in BOOTSTRAP; OR the IRQ vector isn't `$FFFE/F→$1F00`; OR tasks aren't I-clear (frame P must be `$00`).
- **HANG**: re-arm before both DONE (lengthen the first `repeat`); or the YIELD/RESTORE/ISR register order is off (verify against the spec — `RTI` resume, `PLP`/`RTI` last, save-before-TSX in ISR); or a task's BNE mis-fires after preemption (P not preserved in the frame).
- **wrong order / both same**: ORDER RMW not `SEI`-protected (two tasks grab same order), or `COREBASE+id` wrong.
- Increase `repeat` if timing-marginal; widen NPARAM margin if a near-tie.

- [ ] **Step 6: Commit**

```bash
git add gateware/rev2/project_obscurus/project_obscurus_tb.v
git commit -m "test(coproc-c3): preemptive race/flip + BRK-safety + stack assert; C2 cooperative still passes"
```

---

## Task 5: Bench verification (user-run)

**Files:** none. The **user** flashes.

- [ ] **Step 1: Build** — `make clean && make DESIGN=project_obscurus REV=rev2`; clean, DP16KD 8, timing. Report "ready".
- [ ] **Step 2: Hand the procedure to the user**

```
1. (you) flash build/project_obscurus.bit
2. Boot the /SDRAM/ disk; BRUN CPRACE3   (no-yield tasks, pure preemption)
   -> prints:  01 02      (task0 budget 50 finished first)
               02 01      (re-arm 250/200 -> task1 first; flip)
3. (optional) BRUN CPRACE  (the C2 cooperative tasks still work) -> 01 02 / 02 01
```
Pass = `01 02` then `02 01` from CPRACE3 — two tasks that **never yield** were preemptively interleaved by the timer tick and the winner flipped. A preemptive RTOS on the soft-6502. The cooperative CPRACE still passing proves the uniform-frame YIELD path.

If CPRACE3 prints `01 02` then `01 02` (no flip): the timer/ISR isn't preempting (polarity/arm) — but the sim would have caught it; re-check the flashed bitstream is current.

- [ ] **Step 3: Record** — update `project_coproc_c0.md` + `MEMORY.md` with C3 bench status + the preemption/uniform-frame/timer notes; C-flash + multi-core as remaining rungs.

---

## Self-Review

**Spec coverage:** timer `$E012`/`$E013` + IRQ wire → Task 1. Uniform IRQ-frame (TCB drops P, RTI RESTORE) + SWITCH + TICK ISR (pinned order, B-bit, save-before-TSX, BRK-leg restore) + YIELD(+1, SEI) + DONE(SEI) + BOOTSTRAP(seed exact entry, arm last) + KIDLE(disarm) → Task 2 kernel.S. I-flag discipline (tasks I-clear via frame P=$00; YIELD/DONE SEI; ISR auto) → Task 2. No-stale-tick (arm resets cnt+clears pending; disarm on KIDLE) → Task 1 RTL + Task 2 KIDLE. Counter-reset-on-arm → Task 1. racetask3 no-yield + id-in-Y + inner delay + SEI finish → Task 3. Wide margins → Task 3/4. Focused-IRQ-via-flip + BRK + stack-assert + C2-cooperative-still → Task 4. Re-arm-wait → CPRACE3/tb.

**Placeholder scan:** racetask3 bytes flagged paste-from-xxd (Task 3/4) with source given. BRK-safety (Task 4 Step 3) + stack-assert (Step 4) give a concrete method + an explicit "document if deferred" — not silent TODOs. No other gaps.

**Type/label consistency:** kernel ZP ($D0-$F2, TMPL/TMPH=$E0/$E1, TCB_P removed) consistent across kernel + racetask3 ABI (CUR=$EC, NPARAM=$E8, ORDER=$ED, COREBL/H=$EE/$EF). API $1003/$1006; racetask3 uses DONE=$1006. Timer $E012/$E013, TICKPER=$02 consistent. RESULT_BASE $0061 ↔ COREBASE ↔ tb reads. STKTOP $2F/$5F/$8F/$BF ↔ partition floors. RESTORE=RTI, frame seed order [P,PCL,PCH] pull-matched. ISR `$0101,X` after ZP-save (SP unchanged).

**Executor notes:** iverilog `-g2005`. `make ... REV=rev2`. Non-indexed `STA $C0CB/$C0CD`. **Verify Arlet IRQ polarity (Task 1).** ISR/YIELD/RESTORE register order is load-bearing — transcribe exactly (save A/X/Y to TMP before TSX; PLP/RTI semantics; restore Y last). Verify ISR lands at `$1F00` (Task 2 Step 2). WAITRACE must outlast both inner-delay tasks. Don't modify sdram_ctrl/sdram_arb/arlet_*.
