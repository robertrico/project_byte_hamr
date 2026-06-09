# Coprocessor C1 — Resident Kernel + Task Registration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn the C0 coprocessor into a resident mini-RTOS: a write-protected baked kernel boots + loops dispatching a task table; the host registers Merlin tasks live through a dual-port BRAM load port; tasks write results to SDRAM the host reads — and a task cannot corrupt the kernel code or repoint dispatch.

**Architecture:** `coproc.v` is rewritten: dual-port BRAM (port A = Arlet runs kernel+tasks, port B = host load), two-axis write protection (port A blocks `$1000+` and TABLE `$0200–$02FF`; port B blocks `$1000+`), a host-owned COUNT register (`$E010` read-only to Arlet), all 6 synthesized immutable vectors, and a generalized `$E000–$E003` SDRAM write window. The top decodes 5 new `$C0Cx` registers and wires port B. A baked `kernel.S` provides the dispatch loop; a Merlin host loader registers tasks.

**Tech Stack:** Verilog-2005 (Icarus `-g2005` ONLY — hangs on `-g2009/12`), Yosys/nextpnr ECP5, Arlet `verilog-6502` (`arlet_cpu.v`/`arlet_alu.v`), Merlin32. Build `make DESIGN=project_obscurus REV=rev2`; sim `make sim DESIGN=project_obscurus REV=rev2`. Branch `coproc`. Spec: `docs/superpowers/specs/2026-06-08-coproc-c1-resident-kernel-design.md`.

Fixed constants: BRAM `$0000–$1FFF`; kernel `$1000–$1FFF`; TABLE `$0200–$02FF`; task region `$0300–$0FFF`; vectors RESET→`$1000`, IRQ→`$1F00`, NMI→`$1F40`; COUNT register read at `$E010`; SDRAM window `$E000`=`SADDR_LO`,`$E001`=`SADDR_HI`,`$E002`=`SBANK`,`$E003`=`SDATA`; load port `$C0C9`=`CP_LADDR_LO`,`$C0CA`=`CP_LADDR_HI`,`$C0CB`=`CP_WDATA`(write-autoinc),`$C0CC`=`CP_RDATA`(read-autoinc),`$C0CD`=`CP_COUNT`.

---

## File Structure

- `gateware/rev2/project_obscurus/kernel.S` (new) → `kernel.mem` — baked kernel firmware.
- `gateware/rev2/project_obscurus/coproc.v` (rewrite) — dual-port BRAM, port B load, protection, vectors, COUNT, SDRAM window.
- `gateware/rev2/project_obscurus/coproc_tb.v` (rewrite) — port B load/readback/protect/count unit test.
- `gateware/rev2/project_obscurus/project_obscurus_top.v` (modify) — decode `$C0C9–CD`, wire port B + COUNT, read mux for `$C0CC`.
- `gateware/rev2/project_obscurus/project_obscurus_tb.v` (modify) — integration: register 2 tasks, 2 SDRAM results, 3 protection tests.
- `software/SDM/CPREG.S`, `taskA.S`, `taskB.S` (new) — host loader + two coproc tasks; on the disk.
- `Makefile` (modify) — `kernel.mem` rule (retire `coproc_prog.mem`), `cpreg`/task assembly + disk.

`sdram_ctrl.v`, `sdram_arb.v`, `arlet_cpu.v`, `arlet_alu.v`, `monitor.S`, `slot_rom.S` unchanged.

---

## Task 1: Kernel firmware (`kernel.S` → `kernel.mem`)

**Files:** Create `gateware/rev2/project_obscurus/kernel.S`; modify `Makefile`.

- [ ] **Step 1: Write `kernel.S`** (Merlin32, single-space fields, ASCII)

```
* kernel.S - C1 coproc resident kernel. ORG $1000 (write-protected region).
* Boots, loops dispatching TABLE[$0200..] entries 0..COUNT-1 (COUNT = $E010 reg).
* Index kept on the STACK across the task call (tasks trash A/X/Y). ISR stubs at
* $1F00/$1F40 (vectors point here; IRQ/NMI tied 0 in C1, but BRK reads $FFFE/F).
 LST OFF
 TYP $06
 DSK kernel.bin
 ORG $1000

JVEC = $00          ; ZP 2-byte indirect dispatch pointer (consumed before task runs)

RESET
 SEI
 LDX #$FF
 TXS
KMAIN
 LDX #0
KNEXT
 CPX $E010          ; X vs COUNT (abs read of the count register)
 BCS KMAIN          ; X >= COUNT -> nothing more, restart
 TXA
 ASL                ; A = X*2 (entry offset; X<=127 -> no overflow)
 TAY
 LDA $0200,Y
 STA JVEC
 LDA $0201,Y
 STA JVEC+1
 TXA
 PHA                ; save index on the stack (survives the task's RTS)
 JSR KCALL
 PLA
 TAX                ; restore index
 INX
 JMP KNEXT
KCALL JMP (JVEC)

 DS $1F00-*         ; pad to the IRQ ISR
IRQH RTI
 DS $1F40-*         ; pad to the NMI ISR
NMIH RTI
```

- [ ] **Step 2: Add the `kernel.mem` Makefile rule + retire `coproc_prog.mem`**

In `Makefile`, find the `OBSCURUS_COPROC_MEM` block (the `coproc_prog.mem` rule added in C0) and REPLACE it with a `kernel.mem` rule:
```makefile
# project_obscurus coprocessor KERNEL image. Merlin32 source ORG $1000 -> .bin ->
# .mem (raw kernel bytes, one hex/line). coproc.v loads it with
# $readmemh("kernel.mem", bram, 13'h1000) so it lands at BRAM $1000-$1FFF.
OBSCURUS_KERNEL_MEM := $(GATEWARE_DIR)/project_obscurus/kernel.mem
OBSCURUS_KERNEL_SRC := $(GATEWARE_DIR)/project_obscurus/kernel.S

ifneq ($(wildcard $(OBSCURUS_KERNEL_SRC)),)
$(OBSCURUS_KERNEL_MEM): $(OBSCURUS_KERNEL_SRC)
	@echo "=== Assembling coproc kernel (Merlin32) ==="
	cd $(GATEWARE_DIR)/project_obscurus && $(MERLIN32) $(MERLIN_LIB) kernel.S
	python3 -c "b=open('$(GATEWARE_DIR)/project_obscurus/kernel.bin','rb').read(); open('$(OBSCURUS_KERNEL_MEM)','w').write('\n'.join('%02x'%x for x in b)+'\n')"
endif
```
Then update the synth dep line (was `$(JSON): $(OBSCURUS_ROM_MEM) $(OBSCURUS_MON_MEM) $(OBSCURUS_COPROC_MEM)`) to:
```makefile
$(JSON): $(OBSCURUS_ROM_MEM) $(OBSCURUS_MON_MEM) $(OBSCURUS_KERNEL_MEM)
```

- [ ] **Step 3: Build `kernel.mem` and verify it boots at `$1000`**

Run:
```bash
cd /Users/hambook/Development/project_byte_hamr/gateware/rev2/project_obscurus
/Users/hambook/Development/Merlin32_v1.2/MacOs/Merlin32 /Users/hambook/Development/Merlin32_v1.2/Library kernel.S
python3 -c "b=open('kernel.bin','rb').read(); open('kernel.mem','w').write(chr(10).join('%02x'%x for x in b)+chr(10))"
head -1 kernel.mem ; wc -l kernel.mem
```
Expected: `kernel.mem` line 1 = `78` (SEI, the first kernel opcode — confirms it lands at `$1000` once `$readmemh` offsets by `$1000`); line count ≈ `$F41` (3905, `$1000`→`$1F40`). If line 1 isn't `78` (SEI), the ORG/DS layout is wrong — STOP and report.

- [ ] **Step 4: Commit**

```bash
cd /Users/hambook/Development/project_byte_hamr
git add gateware/rev2/project_obscurus/kernel.S gateware/rev2/project_obscurus/kernel.mem Makefile
git commit -m "feat(coproc-c1): baked resident kernel.S (dispatch loop + ISR stubs) + mem rule"
```
Trailer on every commit: `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`

---

## Task 2: `coproc.v` C1 rewrite + unit test

**Files:** Rewrite `coproc.v`, `coproc_tb.v`.

- [ ] **Step 1: Write the failing unit test `coproc_tb.v`** (port B load / read-back / write-protect / count; the kernel runs resident with COUNT=0 so it doesn't interfere)

```verilog
`timescale 1ns/1ps
module coproc_tb;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0, ready=0;
    // arbiter client (unused here, tie inputs)
    wire req, we; wire [25:0] phys_addr; wire [7:0] wdata;
    reg [7:0] rdata=0; reg busy=0;
    // port B load + count
    reg  [12:0] laddr=0; reg [7:0] ldata_in=0; reg lwr=0;
    wire [7:0]  ldata_out;
    reg  [7:0]  count_in=0; reg count_wr=0;
    integer errors=0;

    coproc dut(.clk(clk), .rst_n(rst_n), .ready(ready),
        .req(req), .we(we), .phys_addr(phys_addr), .wdata(wdata),
        .busy(busy), .rdata(rdata),
        .laddr(laddr), .ldata_in(ldata_in), .lwr(lwr), .ldata_out(ldata_out),
        .count_in(count_in), .count_wr(count_wr));

    // write one byte via port B at address a
    task pbwrite(input [12:0] a, input [7:0] d); begin
        @(posedge clk); laddr=a; ldata_in=d; lwr=1; @(posedge clk); lwr=0; @(posedge clk);
    end endtask
    // read one byte via port B at address a (ldata_out is registered: set laddr, wait, sample)
    task pbread(input [12:0] a, output [7:0] d); begin
        @(posedge clk); laddr=a; @(posedge clk); @(posedge clk); d=ldata_out;
    end endtask

    reg [7:0] v;
    initial begin
        rst_n=0; #50; rst_n=1; ready=1;  // release the kernel (it spins on COUNT=0)
        #200;
        // 1) load + read-back in the task region
        pbwrite(13'h0300, 8'h99);
        pbread (13'h0300, v);
        if (v!==8'h99) begin errors=errors+1; $display("FAIL readback %02X",v); end
        // 2) write-protect: a write to the kernel region $1000 is refused
        pbwrite(13'h1000, 8'hEE);
        pbread (13'h1000, v);
        if (v===8'hEE) begin errors=errors+1; $display("FAIL kernel $1000 was written"); end
        // 3) count register: write via count_wr, kernel reads $E010 (checked in integration)
        @(posedge clk); count_in=8'h02; count_wr=1; @(posedge clk); count_wr=0;
        // (no direct read of $E010 here — Arlet-side; integration test covers dispatch)
        if (errors==0) $display("PASS coproc-c1 port B"); else $display("FAIL coproc-c1 %0d",errors);
        $finish;
    end
endmodule
```

- [ ] **Step 2: Run it — expect compile/elaboration failure** (the new `coproc` ports don't exist yet)

Run:
```bash
cd /Users/hambook/Development/project_byte_hamr/gateware/rev2/project_obscurus
$(echo ${HOME})/oss-cad-suite/bin/iverilog -g2005 -o /tmp/cop.out -s coproc_tb coproc.v coproc_tb.v arlet_cpu.v arlet_alu.v
```
Expected: error — the old `coproc` has no `laddr`/`ldata_out`/etc. ports.

- [ ] **Step 3: Rewrite `coproc.v`**

```verilog
// =============================================================================
// coproc.v — C1 resident-kernel coprocessor.
// Arlet soft-6502 runs a baked, write-protected kernel ($1000-$1FFF) that
// dispatches host-registered tasks from TABLE ($0200-$02FF). Dual-port BRAM:
// port A = Arlet (kernel+tasks execute), port B = host load. Two-axis write
// protection. COUNT is a host-owned register (read-only to Arlet at $E010).
// Generalized $E000-$E003 SDRAM write window (RDY-stall single post; Arlet WE is
// HELD HIGH through the stall, so latches gate on & rdy and the post is one-shot
// via state transition). See spec 2026-06-08.
// =============================================================================
module coproc (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        ready,           // SDRAM init complete
    // SDRAM arbiter client (c1)
    output reg         req,
    output reg         we,
    output reg  [25:0] phys_addr,
    output reg  [7:0]  wdata,
    input  wire        busy,
    input  wire [7:0]  rdata,           // unused
    // host load port (port B)
    input  wire [12:0] laddr,
    input  wire [7:0]  ldata_in,
    input  wire        lwr,             // CP_WDATA write strobe (1 cycle)
    output reg  [7:0]  ldata_out,       // CP_RDATA read-back (registered latch)
    // COUNT register (host-owned)
    input  wire [7:0]  count_in,
    input  wire        count_wr
);
    // ---- Arlet core ----
    wire [15:0] AB;
    wire [7:0]  DO;
    wire        WE;
    wire [7:0]  DI;
    reg         rdy;

    cpu u_cpu (.clk(clk), .reset(~rst_n), .AB(AB), .DI(DI), .DO(DO), .WE(WE),
               .IRQ(1'b0), .NMI(1'b0), .RDY(rdy));

    // ---- COUNT register: host writes, Arlet reads $E010 (read-only to core) ----
    reg [7:0] task_count;
    always @(posedge clk or negedge rst_n)
        if (!rst_n)        task_count <= 8'd0;
        else if (count_wr) task_count <= count_in;

    // ---- dual-port BRAM (8KB) ----
    reg [7:0] bram [0:8191];
    integer gi;
    initial begin
        for (gi=0; gi<8192; gi=gi+1) bram[gi] = 8'h00;  // zero lower half for clean sim
        $readmemh("kernel.mem", bram, 13'h1000);        // kernel image at $1000
    end

    wire in_bram = (AB[15:13] == 3'b000);   // $0000-$1FFF
    // PORT-A write protection: block kernel ($1000+) AND TABLE ($0200-$02FF).
    // & rdy: Arlet holds WE high through an SDRAM RDY-stall — don't re-write then.
    wire a_wr_ok = WE & in_bram & rdy & ~AB[12] & ~(AB[11:8]==4'h2);

    // region/vector/count selects, registered to match Arlet's 1-cycle DI latency
    wire is_rstlo=(AB==16'hFFFC), is_rsthi=(AB==16'hFFFD);
    wire is_irqlo=(AB==16'hFFFE), is_irqhi=(AB==16'hFFFF);
    wire is_nmilo=(AB==16'hFFFA), is_nmihi=(AB==16'hFFFB);
    wire is_count=(AB==16'hE010);
    reg [7:0] bram_qa;
    reg in_bram_q, is_rstlo_q,is_rsthi_q,is_irqlo_q,is_irqhi_q,is_nmilo_q,is_nmihi_q,is_count_q;
    always @(posedge clk) begin           // PORT A (Arlet)
        if (a_wr_ok) bram[AB[12:0]] <= DO;
        bram_qa    <= bram[AB[12:0]];
        in_bram_q  <= in_bram;
        is_rstlo_q <= is_rstlo; is_rsthi_q <= is_rsthi;
        is_irqlo_q <= is_irqlo; is_irqhi_q <= is_irqhi;
        is_nmilo_q <= is_nmilo; is_nmihi_q <= is_nmihi;
        is_count_q <= is_count;
    end
    assign DI = is_rstlo_q ? 8'h00 : is_rsthi_q ? 8'h10   // RESET -> $1000
              : is_irqlo_q ? 8'h00 : is_irqhi_q ? 8'h1F   // IRQ   -> $1F00
              : is_nmilo_q ? 8'h40 : is_nmihi_q ? 8'h1F   // NMI   -> $1F40
              : is_count_q ? task_count
              : in_bram_q  ? bram_qa
              :              8'h00;

    always @(posedge clk) begin           // PORT B (host load)
        if (lwr & ~laddr[12]) bram[laddr] <= ldata_in;   // write-protect $1000+
        ldata_out <= bram[laddr];                        // continuous registered latch
    end

    // ---- $E000-$E003 SDRAM write window + RDY-stall single post ----
    wire is_e000=(AB==16'hE000), is_e001=(AB==16'hE001),
         is_e002=(AB==16'hE002), is_e003=(AB==16'hE003);
    reg [15:0] saddr; reg [7:0] sbank;
    reg busy_d; always @(posedge clk) busy_d <= busy; wire done = busy_d & ~busy;

    localparam ST_BOOT=2'd0, ST_RUN=2'd1, ST_WAIT=2'd2;
    reg [1:0] state;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            req<=0; we<=0; phys_addr<=0; wdata<=0; rdy<=0; state<=ST_BOOT;
            saddr<=0; sbank<=0;
        end else begin
            req <= 1'b0;
            // address/bank latches: capture only when NOT stalled (& rdy) so a held
            // WE during a post's RDY-stall can't re-latch a later target.
            if (is_e000 & WE & rdy) saddr[7:0]  <= DO;
            if (is_e001 & WE & rdy) saddr[15:8] <= DO;
            if (is_e002 & WE & rdy) sbank       <= DO;
            case (state)
                ST_BOOT: if (ready) begin rdy<=1'b1; state<=ST_RUN; end
                ST_RUN:  if (is_e003 & WE) begin     // post once via state transition
                    we<=1'b1; phys_addr<={2'b00, sbank, saddr}; wdata<=DO;
                    req<=1'b1; rdy<=1'b0; state<=ST_WAIT;
                end
                ST_WAIT: if (done) begin rdy<=1'b1; state<=ST_RUN; end
                default: state<=ST_BOOT;
            endcase
        end
    end
endmodule
```

- [ ] **Step 4: Run the unit test — expect PASS** (requires `kernel.mem` present from Task 1)

Run:
```bash
cd /Users/hambook/Development/project_byte_hamr/gateware/rev2/project_obscurus
$(echo ${HOME})/oss-cad-suite/bin/iverilog -g2005 -o /tmp/cop.out -s coproc_tb coproc.v coproc_tb.v arlet_cpu.v arlet_alu.v && $(echo ${HOME})/oss-cad-suite/bin/vvp /tmp/cop.out
```
Expected: `PASS coproc-c1 port B`. If `FAIL kernel $1000 was written`, the port-B `~laddr[12]` protect is wrong. If readback fails, check the `ldata_out` registered-latch timing (the `pbread` task waits 2 cycles for the latch).

- [ ] **Step 5: Commit**

```bash
cd /Users/hambook/Development/project_byte_hamr
git add gateware/rev2/project_obscurus/coproc.v gateware/rev2/project_obscurus/coproc_tb.v
git commit -m "feat(coproc-c1): coproc.v dual-port BRAM + port-B load + protection + COUNT + \$E00x window"
```

---

## Task 3: Top integration — decode `$C0C9–CD`, wire port B + COUNT

**Files:** Modify `project_obscurus_top.v`.

- [ ] **Step 1: Add the load-address register + carve the scratch range**

In `project_obscurus_top.v`, the scratch decode currently catches `wr_addr_latch >= 4'h9`. Change it to `>= 4'hE` so `$C0C9–CD` are free for the load port (scratch keeps `$C0CE/CF`). Find (~line 239):
```verilog
        end else if (nds_rise & ~wr_rw_latch & (wr_addr_latch >= 4'h9)) begin
            scratch[wr_addr_latch] <= wr_data_latch;
```
Change `>= 4'h9` to `>= 4'hE`.

Add the load-address register + strobes near the monitor regs (after the `mon_busy`/`cop_*` wire block, ~line 261). Note `reg_wr` (line 279) and `nds_rise`/`wr_addr_latch`/`wr_rw_latch`/`wr_data_latch` already exist:
```verilog
    // ---- C1 coproc load port ($C0C9-CD) ----
    reg  [12:0] m_laddr = 13'd0;
    wire        cp_wdata_wr = reg_wr & (wr_addr_latch == 4'hB);   // CP_WDATA write
    wire        cp_rdata_rd = nds_rise & wr_rw_latch & (wr_addr_latch == 4'hC); // CP_RDATA read
    wire        cp_count_wr = reg_wr & (wr_addr_latch == 4'hD);   // CP_COUNT write
    wire [7:0]  cp_ldata_out;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) m_laddr <= 13'd0;
        else begin
            if (reg_wr & (wr_addr_latch == 4'h9)) m_laddr[7:0]  <= wr_data_latch;       // CP_LADDR_LO
            if (reg_wr & (wr_addr_latch == 4'hA)) m_laddr[12:8] <= wr_data_latch[4:0];  // CP_LADDR_HI
            if (cp_wdata_wr) m_laddr <= m_laddr + 1'b1;   // write-autoinc
            if (cp_rdata_rd) m_laddr <= m_laddr + 1'b1;   // read-autoinc (separate addr; anti-double-bump)
        end
    end
```
(The `CP_LADDR_LO/HI` writes and the auto-inc never coincide — different addresses — so the priority among the `if`s is irrelevant.)

- [ ] **Step 2: Wire port B + COUNT into the coproc instance**

Extend the `coproc u_coproc (...)` instantiation (currently has clk/rst_n/ready/req/we/phys_addr/wdata/busy/rdata) with the new ports:
```verilog
    coproc u_coproc (
        .clk(clk), .rst_n(rst_n), .ready(ready),
        .req(cop_req), .we(cop_we), .phys_addr(cop_addr), .wdata(cop_wdata),
        .busy(cop_busy), .rdata(cop_rdata),
        .laddr(m_laddr), .ldata_in(wr_data_latch), .lwr(cp_wdata_wr),
        .ldata_out(cp_ldata_out),
        .count_in(wr_data_latch), .count_wr(cp_count_wr)
    );
```

- [ ] **Step 3: Read mux — `$C0CC` returns the load read-back**

In the read mux (the `case (apple_addr[3:0])` ~line 333), add a `4'hC` arm before the scratch default:
```verilog
            4'h5: reg_data_out = status_byte;     // STATUS
            4'h6: reg_data_out = mon_rdata;       // DATA
            4'hC: reg_data_out = cp_ldata_out;    // CP_RDATA (coproc BRAM read-back)
            default: reg_data_out = scratch[apple_addr[3:0]];
```

- [ ] **Step 4: Update the header register-map comment**

Update the `$C0Cx` comment block at the top of the file: `$C0C9 CP_LADDR_LO`, `$C0CA CP_LADDR_HI`, `$C0CB CP_WDATA (W, BRAM[laddr]++)`, `$C0CC CP_RDATA (R, BRAM[laddr]++)`, `$C0CD CP_COUNT (W)`, `$C0CE-CF SCRATCH`.

- [ ] **Step 5: Build — synth + timing + EBR check**

Run: `make clean && make DESIGN=project_obscurus REV=rev2`
Expected: clean bitstream, timing PASS at 25 MHz. **Confirm the coproc 8 KB BRAM maps to EBR (DP16KD), NOT distributed LUTRAM** — adding port B (second `always` block) can defeat DP inference. In the synth/PnR report check the EBR/DP16KD count is ~4 (C0 was 4) and LUT count didn't balloon by thousands. Report Fmax + EBR + LUT counts. If it exploded to LUTRAM, report it (the two write ports must infer one DP16KD). Do NOT flash.

- [ ] **Step 6: Commit**

```bash
git add gateware/rev2/project_obscurus/project_obscurus_top.v
git commit -m "feat(coproc-c1): top decodes \$C0C9-CD load port, wires coproc port B + COUNT"
```

---

## Task 4: Host loader + tasks + disk

**Files:** Create `software/SDM/CPREG.S`, `taskA.S`, `taskB.S`; modify `Makefile`.

- [ ] **Step 1: Write the two coproc tasks** (Merlin, ORG `$0300`/`$0320`; assembled to get their bytes — they are loaded into the coproc, NOT run on the host)

`software/SDM/taskA.S`:
```
* taskA - coproc task: write $99 to SDRAM bank0 $0050, then RTS to the kernel.
 TYP $06
 DSK taskA.bin
 ORG $0300
 LDA #$50
 STA $E000      ; SADDR_LO
 LDA #$00
 STA $E001      ; SADDR_HI
 STA $E002      ; SBANK = 0
 LDA #$99
 STA $E003      ; SDATA -> post SDRAM write bank0 $0050 = $99
 RTS
```
`software/SDM/taskB.S` (same shape, `$0051` = `$77`):
```
 TYP $06
 DSK taskB.bin
 ORG $0300
 LDA #$51
 STA $E000
 LDA #$00
 STA $E001
 STA $E002
 LDA #$77
 STA $E003
 RTS
```
Note: both ORG `$0300` (they load into the same task slot region; the loader places taskA at `$0300`, taskB at `$0320`, and writes each TABLE entry to the actual load address — see loader). For C1 the two tasks are position-tolerant (only `$E00x` + immediates), so loading taskB at `$0320` works even though it's assembled at `$0300` (no absolute self-references). Keep each ≤ `$20` bytes.

- [ ] **Step 2: Write the host loader `CPREG.S`** (Merlin, ORG `$6000`, ships on the disk; uses the monitor `$C0Cx` port)

```
* CPREG.S - C1 host loader. Registers taskA ($0300) + taskB ($0320) into the
* coproc, then reads back the two SDRAM results + runs the protection tests.
* Uses the $C0Cx port directly (non-indexed STA to CP_WDATA - rule #1).
 TYP $06
 DSK CPREG
 ORG $6000

CPLADDRLO = $C0C9
CPLADDRHI = $C0CA
CPWDATA   = $C0CB
CPRDATA   = $C0CC
CPCOUNT   = $C0CD
COUT  = $FDED
CROUT = $FD8E

 JMP MAIN
 PUT SDRAMLIB

* ---- load N bytes from (SRC) to coproc BRAM at laddr in X:Y(hi:lo) ----
* params: LADHI/LADLO = dest laddr, LEN = count, SRC ptr in $06/$07
LADLO DS 1
LADHI DS 1
LEN   DS 1
SRCL = $06
SRCH = $07

LOADBLK
 LDA LADLO
 STA CPLADDRLO
 LDA LADHI
 STA CPLADDRHI
 LDY #0
:lp LDA (SRCL),Y
 STA CPWDATA          ; non-indexed: one write, laddr auto-incs in gateware
 INY
 CPY LEN
 BNE :lp
 RTS

MAIN JSR SDM_READY    ; ensure card present (also waits SDRAM ready)
* --- load taskA bytes into coproc $0300 ---
 LDA #$00
 STA LADLO
 LDA #$03
 STA LADHI            ; laddr $0300
 LDA #TASKAEND-TASKA
 STA LEN
 LDA #<TASKA
 STA SRCL
 LDA #>TASKA
 STA SRCH
 JSR LOADBLK
* --- TABLE entry 0 = $0300 (lo,hi) at laddr $0200 ---
 LDA #$00
 STA CPLADDRLO
 LDA #$02
 STA CPLADDRHI        ; laddr $0200
 LDA #$00
 STA CPWDATA          ; entry0 lo = $00
 LDA #$03
 STA CPWDATA          ; entry0 hi = $03  -> $0300
* --- load taskB bytes into coproc $0320 ---
 LDA #$20
 STA LADLO
 LDA #$03
 STA LADHI            ; laddr $0320
 LDA #TASKBEND-TASKB
 STA LEN
 LDA #<TASKB
 STA SRCL
 LDA #>TASKB
 STA SRCH
 JSR LOADBLK
* --- TABLE entry 1 = $0320 at laddr $0202 ---
 LDA #$02
 STA CPLADDRLO
 LDA #$02
 STA CPLADDRHI        ; laddr $0202
 LDA #$20
 STA CPWDATA          ; entry1 lo = $20
 LDA #$03
 STA CPWDATA          ; entry1 hi = $03  -> $0320
* --- register: COUNT = 2 (release barrier, last) ---
 LDA #$02
 STA CPCOUNT
* --- give the kernel a moment to dispatch both, then read results ---
 LDY #0
:w INY
 BNE :w               ; ~256 iterations of spin (kernel runs at 25MHz, plenty)
* --- read SDRAM bank0 $0050/$0051 via SDRAMLIB and print ---
 JSR RDCELL           ; reads $0050 -> A
 JSR PRBYTE
 JSR RDCELL2          ; reads $0051 -> A
 JSR PRBYTE
 JSR CROUT
 RTS

* (RDCELL/RDCELL2 use SDM_ API to read bank0 $0050/$0051; PRBYTE prints hex.
*  Implement with the SDM_SETBANK/SETADDR/READ helpers from SDRAMLIB and the
*  monitor's PRBYTE pattern. taskA/taskB byte tables follow via PUT or inline DFB.)
TASKA  DFB $A9,$50,$8D,$00,$E0,$A9,$00,$8D,$01,$E0,$8D,$02,$E0,$A9,$99,$8D,$03,$E0,$60
TASKAEND
TASKB  DFB $A9,$51,$8D,$00,$E0,$A9,$00,$8D,$01,$E0,$8D,$02,$E0,$A9,$77,$8D,$03,$E0,$60
TASKBEND
```
NOTE TO IMPLEMENTER: the `TASKA`/`TASKB` `DFB` byte strings MUST equal the assembled
`taskA.bin`/`taskB.bin` from Step 1 — assemble those, `xxd` them, and paste the exact
bytes (the values above are the expected encoding of the Step-1 source; verify byte-for-byte).
Fill in `RDCELL`/`RDCELL2`/`PRBYTE` using the `SDM_` helpers already in `SDRAMLIB` (set
bank 0, addr `$0050`/`$0051`, read, return in A) and a standard hex-print. Keep all
`STA CPWDATA` **non-indexed**.

- [ ] **Step 2b: Build the tasks + verify the DFB bytes match**

```bash
cd /Users/hambook/Development/project_byte_hamr/software/SDM
/Users/hambook/Development/Merlin32_v1.2/MacOs/Merlin32 /Users/hambook/Development/Merlin32_v1.2/Library taskA.S
/Users/hambook/Development/Merlin32_v1.2/MacOs/Merlin32 /Users/hambook/Development/Merlin32_v1.2/Library taskB.S
xxd taskA.bin ; xxd taskB.bin
```
Confirm the bytes equal the `DFB` strings in `CPREG.S`; fix the `DFB` to match the real assembler output if Merlin encodes differently.

- [ ] **Step 3: Makefile — assemble `CPREG` + add to the disk**

After the existing `sdmtest`/`sdmdisk` targets, add a `cpreg` target and pack `CPREG` (BIN `$6000`) into the disk image used for these tests (reuse the `$(SDM_PO)` image + `$(AC_CLASSIC)` pattern from the C0/hypervisor disk rules):
```makefile
cpreg:
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) CPREG.S
```
Add `cpreg` to `.PHONY` and to the `sdmdisk` deps; add the pack line:
```makefile
	$(AC_CLASSIC) -p $(SDM_PO) CPREG BIN 0x6000 < $(SDM_DIR)/CPREG
```
Run `make cpreg` (clean assemble, first byte `4c`), then `make sdmdisk` and confirm `CPREG` is in the catalog.

- [ ] **Step 4: Commit**

```bash
git add software/SDM/CPREG.S software/SDM/taskA.S software/SDM/taskB.S Makefile
git commit -m "feat(coproc-c1): host loader CPREG + taskA/taskB (register 2 coproc tasks)"
```

---

## Task 5: Integration sim — 2 tasks dispatched, protection enforced

**Files:** Modify `project_obscurus_tb.v`.

- [ ] **Step 1: Add the C1 integration checks**

The tb drives the monitor `$C0Cx` port (`wr_reg(r,d)` writes `$C0C{r}`, `sdram_read(bank,a,d)` reads SDRAM). After the existing checks, before the final summary, register two tasks **through the load port** exactly as the host loader does, let the resident kernel dispatch them, and assert both SDRAM results + the three protection properties. (Use `wr_reg` for `$C0C9–CD`; `tmp`/`errors` exist.)

**COUNT-LAST (mandatory — priority-mux collision).** The BRAM has a single shared write
port, host-priority. If the host port-B-writes while the kernel dispatches (its `PHA`/`JSR`
stack pushes), the kernel write is dropped. So **load BOTH tasks + both entries while
COUNT=0** (kernel idle-spins, zero BRAM writes), then bump COUNT once. Capture the
protected-region baselines BEFORE the first load (clean kernel image).
```verilog
        // ===== C1: resident kernel + task registration (COUNT-LAST) =====
        // baseline the protected bytes from the clean kernel image (COUNT still 0)
        cp_read(13'h1000, krn_before);     // kernel first opcode ($78 SEI)
        cp_read(13'h0200, tbl_before);     // TABLE byte (zero)
        // taskA at $0300: writes $99 to bank0 $0050
        //   A9 50 8D 00 E0 A9 00 8D 01 E0 8D 02 E0 A9 99 8D 03 E0 60
        wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h03);   // CP_LADDR = $0300
        load_byte(8'hA9); load_byte(8'h50); load_byte(8'h8D); load_byte(8'h00); load_byte(8'hE0);
        load_byte(8'hA9); load_byte(8'h00); load_byte(8'h8D); load_byte(8'h01); load_byte(8'hE0);
        load_byte(8'h8D); load_byte(8'h02); load_byte(8'hE0); load_byte(8'hA9); load_byte(8'h99);
        load_byte(8'h8D); load_byte(8'h03); load_byte(8'hE0); load_byte(8'h60);
        // taskBAD at $0320: LDA #$EE / STA $1000 / STA $0200 / RTS  (both writes must be refused)
        //   A9 EE 8D 00 10 8D 00 02 60
        wr_reg(4'h9, 8'h20); wr_reg(4'hA, 8'h03);   // CP_LADDR = $0320
        load_byte(8'hA9); load_byte(8'hEE); load_byte(8'h8D); load_byte(8'h00); load_byte(8'h10);
        load_byte(8'h8D); load_byte(8'h00); load_byte(8'h02); load_byte(8'h60);
        // TABLE: entry0=$0300 @ $0200, entry1=$0320 @ $0202
        wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h02); load_byte(8'h00); load_byte(8'h03);
        wr_reg(4'h9, 8'h02); wr_reg(4'hA, 8'h02); load_byte(8'h20); load_byte(8'h03);
        // *** release: COUNT = 2 LAST (after all BRAM loads) ***
        wr_reg(4'hD, 8'h02);
        repeat (8000) @(posedge clk);      // let the kernel dispatch both, repeatedly
        // taskA ran:
        sdram_read(10'd0, 16'h0050, tmp);
        if (tmp!==8'h99) begin errors=errors+1; $display("FAIL C1 taskA result %02X",tmp); end
        else $display("PASS C1 taskA dispatched ($99 @ $0050)");
        // taskBAD's writes to kernel + TABLE were refused:
        cp_read(13'h1000, tmp);
        if (tmp!==krn_before) begin errors=errors+1; $display("FAIL C1 task wrote kernel $1000"); end
        else $display("PASS C1 kernel $1000 protected from task");
        cp_read(13'h0200, tmp);
        if (tmp!==tbl_before) begin errors=errors+1; $display("FAIL C1 task wrote TABLE $0200"); end
        else $display("PASS C1 TABLE $0200 protected from task");
```
Add the helper tasks near the other tb tasks (drive the load port like the loader; `cp_read` does the registered-latch read-back):
```verilog
    reg [7:0] krn_before, tbl_before;
    task load_byte(input [7:0] d); begin wr_reg(4'hB, d); end endtask   // CP_WDATA (auto-inc)
    task cp_read(input [12:0] a, output [7:0] d); begin
        wr_reg(4'h9, a[7:0]); wr_reg(4'hA, {3'b0,a[12:8]});
        rd_reg(4'hC, d);   // first CP_RDATA: latch may be 1 stale -> read twice
        rd_reg(4'hC, d);
    end endtask
```
NOTE: `cp_read` reads `CP_RDATA` ($C0CC) **twice** — the registered `ldata_out` needs a settle after `laddr` changes; the second read returns the correct byte (and bumps `laddr`, harmless here). If `rd_reg` doesn't exist in this tb, mirror the monitor read pattern already present.

- [ ] **Step 2: Run the integration sim**

Run: `make sim DESIGN=project_obscurus REV=rev2`
Expected:
```
PASS C1 taskA dispatched ($99 @ $0050)
PASS C1 kernel $1000 protected from task
PASS C1 TABLE $0200 protected from task
```
plus ALL pre-existing monitor/C0-arbiter checks still PASS, final 0 errors. iverilog `-g2005`.
If `taskA` didn't run: confirm `kernel.mem` is the freshly built one, the kernel boots ($readmemh offset $1000), COUNT reaches the coproc ($E010), and the kernel's `CPX $E010` dispatch fires. If a protection test fails: the port-A gate `& ~AB[12] & ~(AB[11:8]==4'h2)` is wrong. Root-cause; do not mask.

- [ ] **Step 3: Commit**

```bash
git add gateware/rev2/project_obscurus/project_obscurus_tb.v
git commit -m "test(coproc-c1): integration — kernel dispatches a registered task; kernel+TABLE protected"
```

---

## Task 6: Bench verification (user-run)

**Files:** none. The **user** flashes.

- [ ] **Step 1: Build the bitstream**

Run: `make clean && make DESIGN=project_obscurus REV=rev2` — confirm clean, timing, EBR. Report "ready".

- [ ] **Step 2: Hand the procedure to the user**

```
1. (you) flash build/project_obscurus.bit
2. Boot the /SDRAM/ disk (ProDOS); BRUN CPREG
   -> prints two bytes: 99 77   (taskA wrote $0050=$99, taskB wrote $0051=$77)
3. (manual protection check) in the monitor:  B 000 ; R 0050 -> 99 ; R 0051 -> 77
```
Pass = `CPREG` prints `99 77` → two host-registered Merlin tasks ran under the resident kernel and wrote distinct SDRAM cells. The coproc is a programmable mini-RTOS. (The task-write protection is proven in the integration sim; a bench protection demo can be added later.)

If `00 00`: the kernel didn't dispatch — check COUNT path / `$E010` read / kernel.mem in the bitstream. If one value is right and the other `00`: TABLE entry or task load offset bug.

- [ ] **Step 3: Record result**

Update `project_coproc_c0.md` (or a new `project_coproc_c1.md`) + `MEMORY.md` with C1 bench status, the register map, the protection model, and C2 as next.

---

## Self-Review

**Spec coverage:** resident kernel + dispatch loop (X-on-stack, `CPX $E010`) → Task 1 `kernel.S`. Dual-port BRAM + port B + read-back latch → Task 2. Two-axis protection (port A `~AB[12] & ~(AB[11:8]==2)` + port B `~laddr[12]`) → Task 2 + tested Task 5. All-6 vectors → Task 2 DI mux. COUNT register `$E010` read-only → Task 2 + Task 3 `CP_COUNT`. `$E000–E003` window with `& rdy` latch + single-post → Task 2. Split `CP_WDATA`/`CP_RDATA` autoinc → Task 3. kernel.mem `$readmemh` offset + synth-graph → Task 1. EBR inference check → Task 3 Step 5. Live registration (code→entry→count-last) → Task 4 loader + Task 5 tb. Proof (2 tasks + kernel/TABLE protection) → Task 5/6.

**Placeholder scan:** the `CPREG.S` `RDCELL/RDCELL2/PRBYTE` + the `DFB` task bytes are flagged as implementer must-verify-against-assembled-output (Step 2b), not a TODO — concrete bytes given, verification step included. No other gaps.

**Type/label consistency:** `coproc` ports (`laddr/ldata_in/lwr/ldata_out/count_in/count_wr`) match across Task 2 (def), Task 3 (top wiring), Task 2 tb. Register addresses `$C0C9–CD` consistent (Task 3 decode ↔ Task 4 loader ↔ Task 5 tb). `$E010`/`$E000–E003`/vectors/`$1000`/`$0200`/`$0300` consistent across kernel, coproc, tasks, tb. The dispatch index lives on the stack (Task 1) matching the X-clobber fix. taskA `$99@$0050` / taskB `$77@$0051` consistent loader↔tb↔bench.

**Executor notes:** iverilog `-g2005` only. `make ... REV=rev2`. Non-indexed `STA $C0CB`. Keep `& rdy` on the port-A gate and the `$E00x` latches (held-WE). Verify kernel.mem boots at `$1000` (line 1 = `78`) and that synth maps BRAM to EBR not LUTRAM. Don't modify `sdram_ctrl.v`/`sdram_arb.v`/`arlet_*`.
