# Coprocessor SDRAM Read Window Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development. Steps use checkbox (`- [ ]`) syntax.

**Goal:** Give the coproc an auto-incrementing SDRAM READ window so skills can scan the big store — `STA $E007` (trigger) / `LDA $E008` (result), pointer auto-advances.

**Architecture:** Purely additive to `coproc.v`'s `$E00x` FSM. Two-step read (proven mechanisms: the write-window STA-stall + a registered DI-mux read). Shares the arbiter `c1` port. Write window (`$E000-03`) untouched.

**Tech Stack:** Verilog-2005 (iverilog `-g2005` ONLY — `-g2009/-g2012` HANG). `make DESIGN=project_obscurus REV=rev2`; sim `make sim ...`. Merlin32. Branch `coproc-sdram-read`. Spec: `docs/superpowers/specs/2026-06-09-coproc-sdram-read-window-design.md`.

**Fixed:** `$E004` RADDR_LO (W), `$E005` RADDR_HI (W), `$E006` RBANK (W), `$E007` RTRIG (W = trigger read of `SDRAM[{2'b00,rbank,raddr}]`, stall-until-done, latch `sread`, `rptr++`), `$E008` RDATA (R = `sread`). Pointer = flat 24-bit `{rbank,raddr}` auto-inc.

---

## Task 1: coproc.v read window + unit test (with the R2 phase gate)

**Files:** Modify `gateware/rev2/project_obscurus/coproc.v`; Create `gateware/rev2/project_obscurus/coproc_sdrd_tb.v`.

- [ ] **Step 1: Write the failing unit test `coproc_sdrd_tb.v`** — a stub SDRAM/arbiter behind the coproc, exercising the read window. The LOAD-BEARING test is the **R2 slow-stub phase gate** (the `sread<=rdata` timing is the one genuinely new thing). Drive the coproc's arbiter port (`busy`/`rdata`) directly as a stub; run a tiny coproc program from BRAM that sets rptr, does `STA $E007`/`LDA $E008`, stores results to BRAM; the tb reads them back via the host load port.
```verilog
`timescale 1ns/1ps
module coproc_sdrd_tb;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0, ready=1;
    // coproc <-> stub-sdram (we model busy/rdata behavior, incl. deliberate latency + decoy)
    wire req, we; wire [25:0] phys_addr; wire [7:0] wdata;
    reg busy=0; reg [7:0] rdata=0;
    // host load port to read back what the coproc stored
    reg [12:0] laddr=0; reg [7:0] ldata_in=0; reg lwr=0; wire [7:0] ldata_out;
    integer errors=0;

    coproc dut(.clk(clk), .rst_n(rst_n), .ready(ready),
        .req(req), .we(we), .phys_addr(phys_addr), .wdata(wdata), .busy(busy), .rdata(rdata),
        .laddr(laddr), .ldata_in(ldata_in), .lwr(lwr), .ldata_out(ldata_out),
        .count_in(8'd0), .count_wr(1'b0),
        .c4_ring_wr(1'b0), .c4_collect_wr(1'b0), .c4_host_slot(2'd0), .snapshot_busy(1'b0),
        .c4_done(), .c4_active(), .c4_timedout(), .c4_callreq());

    // STUB SDRAM: on req, go busy for LAT cycles; for a READ (we==0) drive a DECOY value
    // early then the REAL value (= a function of phys_addr) only at the data-valid cycle.
    // mem model: SDRAM[a] = a[7:0] ^ 8'h5A (a deterministic pattern).
    integer cnt; reg [7:0] realbyte;
    localparam LAT = 8;                     // deliberately slow -> R2 phase gate
    always @(posedge clk) begin
        if (req) begin busy<=1; cnt<=LAT; realbyte<=phys_addr[7:0]^8'h5A;
                       rdata<=8'hED; end     // DECOY on the bus immediately
        else if (busy) begin
            cnt<=cnt-1;
            if (cnt<=1) begin busy<=0; rdata<=realbyte; end   // REAL byte only at valid cycle
            else        rdata<=8'hED;                          // hold decoy until then
        end
    end

    // The coproc program is baked in BRAM by the tb via the load port BEFORE releasing reset?
    // Simpler: this core boots from kernel.mem/coproc_prog. For a focused unit test, preload a
    // tiny program into BRAM via the load port while held in a pre-run state, OR (cleanest)
    // build a dedicated read-test program image. IMPLEMENTER: choose the lightest path that
    // runs a sequence "set rptr=$000010, STA $E007, LDA $E008, STA $0040" and lets the tb read
    // $0040 via the load port. If baking a program image is heavy, drive the $E004-08 sequence
    // by forcing the coproc's AB/DO/WE bus (the coproc_c4_tb force/release idiom) to exercise
    // the FSM + assert `sread`/`ldata_out`. Pick the approach that PROVES the phase, not the
    // happy path.

    initial begin
        rst_n=0; #50; rst_n=1; #20;
        // ... drive: set rptr to a known addr, STA $E007, wait, LDA $E008 ...
        // ASSERT (R2 phase gate): the byte the coproc latched == realbyte (addr^$5A), NOT $hED.
        //   if it ever sees $ED, the latch grabbed the decoy -> phase is off-by-one -> FAIL.
        // ASSERT (consecutive): two reads from consecutive addrs return consecutive pattern bytes
        //   (proves auto-inc + correct per-read phase).
        if (errors==0) $display("PASS coproc_sdrd (read window: post-latency byte, auto-inc, phase pinned)");
        else $display("FAIL coproc_sdrd %0d", errors);
        $finish;
    end
endmodule
```
NOTE to implementer: the hard part is driving a real coproc read sequence in the tb. Two viable approaches — (a) bake a tiny read-test program into the BRAM image and let the core run it, or (b) force the Arlet `AB`/`DO`/`WE` bus (the `coproc_c4_tb.v` `force`/`release` idiom) to issue the `$E004-08` accesses and observe `sread` via the DI mux / `ldata_out`. Either is fine — the REQUIRED outcome is the **decoy-rejection assertion** (the latched byte is the post-latency real value, never `$ED`) which pins the R2 phase. If the latch grabs the decoy, that's R2 firing — fix per Step 3's note.

- [ ] **Step 2: Run — expect failure (no read window)**
```bash
cd /Users/hambook/Development/project_byte_hamr/gateware/rev2/project_obscurus
$HOME/oss-cad-suite/bin/iverilog -g2005 -o /tmp/sdrd.out -s coproc_sdrd_tb coproc.v coproc_sdrd_tb.v arlet_cpu.v arlet_alu.v
$HOME/oss-cad-suite/bin/vvp /tmp/sdrd.out
```
Expected: FAIL/garbage (`$E007/$E008` not decoded yet).

- [ ] **Step 3: Implement the read window in `coproc.v`**
Near the `saddr`/`sbank` declarations, add:
```verilog
    reg [15:0] raddr; reg [7:0] rbank; reg [7:0] sread; reg rd_pending;
    wire is_e004=(AB==16'hE004), is_e005=(AB==16'hE005), is_e006=(AB==16'hE006),
         is_e007=(AB==16'hE007), is_e008=(AB==16'hE008);
```
In the `$E00x` FSM `always` block, add the register sets next to `saddr`/`sbank` (same `& WE & rdy` gating):
```verilog
            if (is_e004 & WE & rdy) raddr[7:0]  <= DO;
            if (is_e005 & WE & rdy) raddr[15:8] <= DO;
            if (is_e006 & WE & rdy) rbank       <= DO;
```
In `ST_RUN`, add the read trigger as an `else if` after the existing `$E003` write:
```verilog
                ST_RUN:  if (is_e003 & WE) begin
                            req<=1'b1; we<=1'b1; wdata<=DO; phys_addr<={2'b00,sbank,saddr};
                            rdy<=1'b0; state<=ST_WAIT;
                         end else if (is_e007 & WE) begin            // READ trigger
                            req<=1'b1; we<=1'b0; phys_addr<={2'b00,rbank,raddr};
                            rdy<=1'b0; rd_pending<=1'b1; state<=ST_WAIT;
                         end
```
Extend `ST_WAIT` to latch + auto-inc on a read:
```verilog
                ST_WAIT: if (done) begin
                            if (rd_pending) begin
                                sread <= rdata;                       // R2: the new timing
                                {rbank,raddr} <= {rbank,raddr} + 1'b1; // flat 24-bit auto-inc
                                rd_pending <= 1'b0;
                            end
                            rdy<=1'b1; state<=ST_RUN;
                         end
```
Reset: add `raddr<=0; rbank<=0; sread<=0; rd_pending<=0;` to the `!rst_n` branch.
DI mux: declare `is_e008_q`, register it (`is_e008_q <= is_e008;` in the registered-select block), and add to the DI mux chain before the bram fallback: `: is_e008_q ? sread`.
**R2 fix-if-the-test-fails:** if the decoy-rejection assertion fails (latched `$ED`), the `done` edge and the `rdata`-valid cycle are off by one — change the latch to sample one cycle relative (e.g. latch `sread<=rdata` the cycle `busy` actually falls vs the `done` pulse, or register `rdata` once more). Adjust until the slow-stub test passes with the REAL byte.

- [ ] **Step 4: Run — expect PASS**
```bash
$HOME/oss-cad-suite/bin/iverilog -g2005 -o /tmp/sdrd.out -s coproc_sdrd_tb coproc.v coproc_sdrd_tb.v arlet_cpu.v arlet_alu.v && $HOME/oss-cad-suite/bin/vvp /tmp/sdrd.out
```
Expected: `PASS coproc_sdrd` — the latched byte is the post-latency real value (never the decoy), and consecutive reads auto-increment.

- [ ] **Step 5: Commit**
```bash
git add coproc.v coproc_sdrd_tb.v
git commit -m "feat(sdram-read): coproc \$E004-08 auto-inc read window (two-step, R2 phase pinned by slow-stub)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: Round-trip skill + integration sim

**Files:** Create `software/SDM/sdrtest.S` (a coproc skill) + modify `project_obscurus_tb.v`.

- [ ] **Step 1: Write `sdrtest.S`** — a skill that reads an SDRAM region the host seeded, sums it (or copies it), and writes the result to a known SDRAM cell. ORG `$0300`, reentrant-ish (single-instance OK for this test). Reads via `STA $E007`/`LDA $E008`, writes via `$E000-03`. Concretely: set rptr to a region start, loop N times reading bytes + accumulating, then write the sum to SDRAM[result]. End `JMP $1006`.
```
* sdrtest.S - read an SDRAM region (rptr), sum N bytes, write sum to SDRAM result cell. ORG $0300
RADDRLO=$E004
RADDRHI=$E005
RBANK  =$E006
RTRIG  =$E007
RDATA  =$E008
WADDRLO=$E000
WADDRHI=$E001
WBANK  =$E002
WDATA  =$E003
 ; rptr = region start (e.g. bank 0, addr $0080) ; N from a fixed count
 LDA #$80 / STA RADDRLO / LDA #$00 / STA RADDRHI / STA RBANK
 LDA #$00 / TAY            ; sum=0 in Y? use a ZP byte; keep simple
 ... loop N: STA RTRIG / LDA RDATA / clc adc sum / sta sum ...
 ; write sum to SDRAM result cell (bank0 $0090)
 LDA #$90 / STA WADDRLO / LDA #$00 / STA WADDRHI / STA WBANK
 LDA sum / STA WDATA
 JMP $1006
```
(IMPLEMENTER: pick a small N, e.g. 8; use a ZP scratch byte for the sum. Keep it minimal + correct. The exact ZP must avoid kernel `$D0-$F3` + use the skill's allowed scratch.)
Assemble + Makefile target (mirror `cmpskill`).

- [ ] **Step 2: Integration sim** in `project_obscurus_tb.v` — after the C4 block: the host seeds an SDRAM region (8 known bytes at bank0 `$0080..$0087` via the monitor port / `sdram` model), registers `sdrtest` as a skill, CALLs it (or uses the batch GO), waits, then reads the result cell (`$0090`) via the monitor and asserts it equals the expected sum. This proves the coproc READ SDRAM (the region the host wrote) + computed + wrote back. Plus assert all prior tests still pass.
```
PASS sdram-read: coproc summed host-seeded region (got <sum>)
```
- [ ] **Step 3: Run + commit**
```bash
make sim DESIGN=project_obscurus REV=rev2 2>&1 | grep -iE "PASS|FAIL"
```
Expected: the new sdram-read PASS + all prior (monitor/C1/C2/C3/C3.1/C-flash/C4). Commit:
```bash
git add software/SDM/sdrtest.S Makefile gateware/rev2/project_obscurus/project_obscurus_tb.v
git commit -m "test(sdram-read): round-trip skill reads host-seeded SDRAM region + sums + writes back

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: Bench host program + disk

**Files:** Create `software/SDM/CPSDRD.S` + Makefile/sdmdisk.

- [ ] **Step 1: Write `CPSDRD.S`** (ORG $6000, `PUT SDRAMLIB` + `PUT CPLIB`) — seed an SDRAM region via SDRAMLIB (`SDM_WRITE` known bytes to bank0 `$0080..`), register `sdrtest` (LOADBLK + TABLE, mirror CPDEMO), CALL it, wait (CP_WAIT/poll), read the result cell via SDRAMLIB (`SDM_READ` `$0090`), print it via PRBYTE. Expected = the known sum. This proves on the bench that the coproc read the host-written SDRAM.
- [ ] **Step 2: Makefile + sdmdisk** (mirror cpdemo). `make sdrtest cpsdrd sdmdisk`; confirm in catalog.
- [ ] **Step 3: Commit**
```bash
git add software/SDM/CPSDRD.S Makefile
git commit -m "feat(sdram-read): CPSDRD bench - seed SDRAM, coproc reads+sums it, host verifies

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: Build + bench-ready (GOAL)

- [ ] **Step 1: Clean build** — `make clean && make DESIGN=project_obscurus REV=rev2`. Confirm DP16KD 8 (read window is regs+states, no BRAM), timing PASS, no multi-driven nets, USRMCLK placed (C-flash intact). Report Fmax/LUT delta. Do NOT flash.
- [ ] **Step 2: Hand the bench procedure to the user:**
```
1. make sdmdisk DESIGN=project_obscurus REV=rev2
2. (you) flash build/project_obscurus.bit
3. Boot /SDRAM/.  BRUN CPSDRD
   -> seeds an SDRAM region, the coproc reads+sums it, prints the sum.
      Correct sum = the coproc provably READ SDRAM that the host wrote = the keystone, on silicon.
```
- [ ] **Step 3: Record** — update `project_coproc_c0.md` + `MEMORY.md`: coproc SDRAM read window (`$E004-08` two-step auto-inc), the R2 phase note, unlocks SDRAM-resident skills (active DB / big Life).

---

## Self-Review
**Spec coverage:** read window regs + FSM + two-step + auto-inc (R3 width) → Task 1; the R2 slow-stub phase gate (the load-bearing assertion) → Task 1 Step 1/3; round-trip read+write → Task 2; bench proof → Task 3; build → Task 4. The blocking-LDA optimization is explicitly NOT built (spec non-goal / gated).
**Placeholder scan:** Task 1's tb-drive approach is an explicit implementer choice (bake-program vs force-bus) with the required OUTCOME pinned (decoy rejection) — not a blank. Task 2/3 skill ZP + N are "pick small, correct" with the pattern named. No bare TODOs.
**Consistency:** `$E004-08` map + `{2'b00,rbank,raddr}` + `STA $E007`/`LDA $E008` + `is_e008_q` consistent Task 1↔2↔3. Sum/seed region (bank0 `$0080`, result `$0090`) consistent Task 2↔3.
**Executor notes:** iverilog `-g2005`; the decoy-rejection (R2 phase) assertion is the must-pass gate, not a loose byte-match; write window untouched (regression must stay green); don't build the blocking-LDA variant.
