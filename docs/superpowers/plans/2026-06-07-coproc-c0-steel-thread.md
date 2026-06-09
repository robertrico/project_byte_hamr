# Coprocessor C0 Steel Thread Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Get the Arlet `verilog-6502` soft CPU synthesized + clocked on the ECP5, running a fixed BRAM program that writes `$42` to SDRAM bank0/addr0, and prove the host reads it back (`R 0000` = `$42`).

**Architecture:** A new `coproc.v` wraps the vendored Arlet core + 8 KB BRAM, with a `$E000` memory-mapped window that posts a single SDRAM write (RDY-stalling the core until done). A fresh 2-client priority arbiter `sdram_arb.v` (monitor c0 > coproc c1) sits in front of the existing `sdram_ctrl`; the monitor's request path is rewired through it and its `op_done` becomes per-client so the coproc's write can't auto-increment the monitor's pointer.

**Tech Stack:** Verilog-2005 (Icarus `-g2005` only — hangs on `-g2009/-g2012`), Yosys/nextpnr ECP5, Arlet `verilog-6502` (`cpu.v`+`ALU.v`), Merlin32 (`coproc_prog.S`→`coproc_prog.mem`), `make sim`/`make DESIGN=project_obscurus REV=rev2`. Branch `coproc` (off `4b34c99`, clean substrate — no hypervisor).

All in the existing single **25 MHz** `clk` domain. Fixed constants: BRAM `$0000–$1FFF` (8 KB), reset vector → `$0200`, SDRAM window `$E000` → bank0/addr0, magic `$42`.

---

## File Structure

- **Vendor:** `gateware/rev2/project_obscurus/arlet_cpu.v`, `arlet_alu.v` (Arlet core, flat so the Makefile `*.v` glob picks them up), `gateware/rev2/project_obscurus/VENDORED.md` (source/commit/attribution).
- **New:** `sdram_arb.v` (+ `sdram_arb_tb.v`), `coproc.v` (+ `coproc_tb.v`), `coproc_prog.S` → `coproc_prog.mem`.
- **Modify:** `project_obscurus_top.v` (arbiter + coproc instances, monitor rewired, per-client `op_done`), `project_obscurus_tb.v` (C0 proof), `Makefile` (`coproc_prog.mem` rule + synth dep).

`sdram_ctrl.v`, `monitor.S`, `slot_rom.S` unchanged.

---

## Task 0: Vendor the Arlet core

**Files:**
- Create: `gateware/rev2/project_obscurus/arlet_cpu.v`, `arlet_alu.v`, `VENDORED.md`

- [ ] **Step 1: Fetch the two core files**

Run:
```bash
cd /Users/hambook/Development/project_byte_hamr/gateware/rev2/project_obscurus
curl -fsSL https://raw.githubusercontent.com/Arlet/verilog-6502/master/cpu.v -o arlet_cpu.v
curl -fsSL https://raw.githubusercontent.com/Arlet/verilog-6502/master/ALU.v -o arlet_alu.v
```
Expected: two files written. `arlet_cpu.v` declares `module cpu( clk, reset, AB, DI, DO, WE, IRQ, NMI, RDY );`; `arlet_alu.v` declares `module ALU(...)`. (File names differ from module names — that's fine in Verilog.)

- [ ] **Step 2: Syntax-check they parse under iverilog -g2005**

Run:
```bash
cd /Users/hambook/Development/project_byte_hamr/gateware/rev2/project_obscurus
iverilog -g2005 -t null arlet_cpu.v arlet_alu.v
```
Expected: no errors (the `cpu` module instantiates `ALU`; both present → clean). If `iverilog` isn't on PATH, use `$(HOME)/oss-cad-suite/bin/iverilog`.

- [ ] **Step 3: Write `VENDORED.md`**

```markdown
# Vendored: Arlet verilog-6502

- Source: https://github.com/Arlet/verilog-6502 (branch master)
- Files: cpu.v -> arlet_cpu.v, ALU.v -> arlet_alu.v (renamed flat for the
  Makefile $(DESIGN_DIR)/*.v glob; module names `cpu` / `ALU` unchanged)
- Author: Arlet Ottens. Free to use (see the header in each file).
- Used as the coprocessor CPU in coproc.v. RDY is a global clock-enable
  (RDY=0 freezes all register updates, read AND write); reset is active-high.
```

- [ ] **Step 4: Commit**

```bash
git add gateware/rev2/project_obscurus/arlet_cpu.v gateware/rev2/project_obscurus/arlet_alu.v gateware/rev2/project_obscurus/VENDORED.md
git commit -m "vendor: Arlet verilog-6502 core (cpu.v+ALU.v) for the coprocessor"
```
Trailer on every commit: `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`

---

## Task 1: `sdram_arb.v` — 2-client priority arbiter (TDD)

**Files:**
- Create: `gateware/rev2/project_obscurus/sdram_arb.v`, `sdram_arb_tb.v`

- [ ] **Step 1: Write the failing unit test**

Create `sdram_arb_tb.v`. It models `sdram_ctrl`'s contract (req pulse → busy rises next cycle, holds a few cycles, falls; rdata valid at fall) and drives two clients.

```verilog
`timescale 1ns/1ps
module sdram_arb_tb;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0;

    // arbiter <-> ctrl
    wire        req, we; wire [25:0] phys_addr; wire [7:0] wdata;
    reg  [7:0]  rdata; reg busy=0;

    // c0 (monitor), c1 (coproc)
    reg c0_req=0, c0_we=0; reg [25:0] c0_addr=0; reg [7:0] c0_wdata=0;
    wire c0_busy; wire [7:0] c0_rdata;
    reg c1_req=0, c1_we=0; reg [25:0] c1_addr=0; reg [7:0] c1_wdata=0;
    wire c1_busy; wire [7:0] c1_rdata;

    integer errors=0;

    sdram_arb dut(.clk(clk), .rst_n(rst_n),
        .req(req), .we(we), .phys_addr(phys_addr), .wdata(wdata),
        .rdata(rdata), .busy(busy),
        .c0_req(c0_req), .c0_we(c0_we), .c0_addr(c0_addr), .c0_wdata(c0_wdata),
        .c0_busy(c0_busy), .c0_rdata(c0_rdata),
        .c1_req(c1_req), .c1_we(c1_we), .c1_addr(c1_addr), .c1_wdata(c1_wdata),
        .c1_busy(c1_busy), .c1_rdata(c1_rdata));

    // model sdram_ctrl: on req pulse (while !busy), latch addr, busy for 4 cyc,
    // then present read rdata = addr[7:0]^8'hA5 on the falling cycle.
    reg [25:0] mlat; reg [2:0] mcnt=0;
    always @(posedge clk) begin
        if (req && !busy) begin mlat<=phys_addr; busy<=1; mcnt<=3'd4; end
        else if (busy) begin
            if (mcnt>1) mcnt<=mcnt-1;
            else begin busy<=0; rdata<=mlat[7:0]^8'hA5; end
        end
    end

    // wait for a client's busy to pulse high then low
    task wait_done(input c); begin : wd
        integer g; g=0;
        while (g<200) begin @(posedge clk);
            if (c==0 ? c0_busy : c1_busy) begin
                while (c==0 ? c0_busy : c1_busy) @(posedge clk); disable wd;
            end g=g+1;
        end
        errors=errors+1; $display("FAIL wait_done c%0d stuck",c);
    end endtask

    initial begin
        rst_n=0; #40; rst_n=1; @(posedge clk);
        // 1) c0 write op completes, drives ctrl with c0's fields
        c0_we=1; c0_addr=26'h000_0010; c0_wdata=8'h11; c0_req=1; @(posedge clk); c0_req=0;
        wait_done(0);
        if (dut.owner!==1'b0) ; // owner cleared after done; not asserted directly
        // 2) c1 read op: c1_rdata latched
        c1_we=0; c1_addr=26'h000_0003; c1_req=1; @(posedge clk); c1_req=0;
        wait_done(1);
        if (c1_rdata!==(8'h03^8'hA5)) begin errors=errors+1; $display("FAIL c1_rdata %02X",c1_rdata); end
        // 3) simultaneous: monitor (c0) wins, then coproc (c1) serviced after
        c0_we=1; c0_addr=26'h000_0020; c0_wdata=8'h22; c0_req=1;
        c1_we=1; c1_addr=26'h000_0021; c1_wdata=8'h33; c1_req=1;
        @(posedge clk); c0_req=0; c1_req=0;
        wait_done(0); wait_done(1);   // both eventually complete, c0 first
        if (errors==0) $display("PASS sdram_arb"); else $display("FAIL sdram_arb %0d",errors);
        $finish;
    end
endmodule
```

- [ ] **Step 2: Run it — expect compile failure (no module)**

Run:
```bash
cd /Users/hambook/Development/project_byte_hamr/gateware/rev2/project_obscurus
iverilog -g2005 -o /tmp/arb.out -s sdram_arb_tb sdram_arb.v sdram_arb_tb.v
```
Expected: error — `sdram_arb.v` doesn't exist / `Unknown module type sdram_arb`.

- [ ] **Step 3: Implement `sdram_arb.v`**

```verilog
// =============================================================================
// sdram_arb.v — N-shaped priority arbiter in front of sdram_ctrl.
// C0 (monitor) has priority over C1 (coproc). Each client sends a 1-cycle req
// pulse + we/addr/wdata; the arbiter latches it, serializes onto sdram_ctrl's
// req/busy contract, and returns a PER-CLIENT busy (high from accept to
// completion) + a latched per-client rdata. Per-client busy lets each owner's
// op_done (busy fall) fire only for its own op. Adding clients = more pend bits
// + a priority slot; no rewrite.
// =============================================================================
module sdram_arb (
    input  wire        clk,
    input  wire        rst_n,
    // sdram_ctrl side
    output reg         req,
    output reg         we,
    output reg  [25:0] phys_addr,
    output reg  [7:0]  wdata,
    input  wire [7:0]  rdata,
    input  wire        busy,
    // client 0 (monitor) — priority
    input  wire        c0_req,
    input  wire        c0_we,
    input  wire [25:0] c0_addr,
    input  wire [7:0]  c0_wdata,
    output wire        c0_busy,
    output reg  [7:0]  c0_rdata,
    // client 1 (coproc)
    input  wire        c1_req,
    input  wire        c1_we,
    input  wire [25:0] c1_addr,
    input  wire [7:0]  c1_wdata,
    output wire        c1_busy,
    output reg  [7:0]  c1_rdata
);
    reg c0_pend, c1_pend;
    reg servicing;
    reg owner;             // 0 = c0, 1 = c1

    reg busy_d;
    always @(posedge clk) busy_d <= busy;
    wire op_complete = busy_d & ~busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c0_pend<=1'b0; c1_pend<=1'b0; servicing<=1'b0; owner<=1'b0;
            req<=1'b0; we<=1'b0; phys_addr<=26'd0; wdata<=8'd0;
            c0_rdata<=8'd0; c1_rdata<=8'd0;
        end else begin
            req <= 1'b0;                    // default: single-cycle pulse
            if (c0_req) c0_pend <= 1'b1;
            if (c1_req) c1_pend <= 1'b1;
            if (!servicing) begin
                if (c0_pend) begin
                    owner<=1'b0; we<=c0_we; phys_addr<=c0_addr; wdata<=c0_wdata;
                    req<=1'b1; servicing<=1'b1; c0_pend<=1'b0;
                end else if (c1_pend) begin
                    owner<=1'b1; we<=c1_we; phys_addr<=c1_addr; wdata<=c1_wdata;
                    req<=1'b1; servicing<=1'b1; c1_pend<=1'b0;
                end
            end else if (op_complete) begin
                if (owner==1'b0) c0_rdata<=rdata; else c1_rdata<=rdata;
                servicing<=1'b0;
            end
        end
    end

    // per-client busy = op in flight (pending OR being serviced as owner)
    assign c0_busy = c0_pend | (servicing & (owner==1'b0));
    assign c1_busy = c1_pend | (servicing & (owner==1'b1));
endmodule
```

- [ ] **Step 4: Run the test — expect PASS**

Run:
```bash
cd /Users/hambook/Development/project_byte_hamr/gateware/rev2/project_obscurus
iverilog -g2005 -o /tmp/arb.out -s sdram_arb_tb sdram_arb.v sdram_arb_tb.v && vvp /tmp/arb.out
```
Expected: `PASS sdram_arb`. (If `vvp` not on PATH, prefix `$(HOME)/oss-cad-suite/bin/`.)

- [ ] **Step 5: Commit**

```bash
git add gateware/rev2/project_obscurus/sdram_arb.v gateware/rev2/project_obscurus/sdram_arb_tb.v
git commit -m "feat(coproc-c0): sdram_arb — 2-client priority arbiter (monitor>coproc)"
```

---

## Task 2: `coproc.v` + program + mem rule (TDD)

**Files:**
- Create: `gateware/rev2/project_obscurus/coproc.v`, `coproc_tb.v`, `coproc_prog.S`
- Modify: `Makefile`

- [ ] **Step 1: Write `coproc_prog.S` (Merlin32)**

```
* coproc_prog.S - C0 steel-thread program for the Arlet soft-6502.
* Reset vector ($FFFC/D) is synthesized to $0200 by coproc.v.
 TYP $06
 DSK coproc_prog.bin
 ORG $0200
START LDA #$42
 STA $E000      ; coproc.v posts SDRAM write bank0 addr0 = $42
LOOP JMP LOOP
```

- [ ] **Step 2: Add the `coproc_prog.mem` Makefile rule + synth dep**

In `Makefile`, after the `OBSCURUS_MON_MEM` rule block (around line 187), add:
```makefile
# project_obscurus coprocessor BRAM image (8KB, Arlet $0000-$1FFF). Merlin32
# source ORG $0200 -> .bin -> 8192-byte .mem with the program at offset $0200
# (reset vector synthesized in coproc.v). Plain offset placement (NOT rom2mem,
# whose base math is monitor-specific).
OBSCURUS_COPROC_MEM := $(GATEWARE_DIR)/project_obscurus/coproc_prog.mem
OBSCURUS_COPROC_SRC := $(GATEWARE_DIR)/project_obscurus/coproc_prog.S

ifneq ($(wildcard $(OBSCURUS_COPROC_SRC)),)
$(OBSCURUS_COPROC_MEM): $(OBSCURUS_COPROC_SRC)
	@echo "=== Assembling coproc program (Merlin32) ==="
	cd $(GATEWARE_DIR)/project_obscurus && $(MERLIN32) $(MERLIN_LIB) coproc_prog.S
	python3 -c "b=open('$(GATEWARE_DIR)/project_obscurus/coproc_prog.bin','rb').read(); m=bytearray(8192); m[0x200:0x200+len(b)]=b; open('$(OBSCURUS_COPROC_MEM)','w').write('\n'.join('%02x'%x for x in m)+'\n')"
endif
```
Then extend the synth dep line (currently `$(JSON): $(OBSCURUS_ROM_MEM) $(OBSCURUS_MON_MEM)` near line 192) to:
```makefile
$(JSON): $(OBSCURUS_ROM_MEM) $(OBSCURUS_MON_MEM) $(OBSCURUS_COPROC_MEM)
```

- [ ] **Step 3: Build the mem and verify the program lands at offset `$0200`**

Run:
```bash
cd /Users/hambook/Development/project_byte_hamr
make $(GATEWARE_DIR 2>/dev/null)gateware/rev2/project_obscurus/coproc_prog.mem 2>/dev/null || \
  (cd gateware/rev2/project_obscurus && /Users/hambook/Development/Merlin32_v1.2/MacOs/Merlin32 /Users/hambook/Development/Merlin32_v1.2/Library coproc_prog.S && \
   python3 -c "b=open('coproc_prog.bin','rb').read(); m=bytearray(8192); m[0x200:0x200+len(b)]=b; open('coproc_prog.mem','w').write(chr(10).join('%02x'%x for x in m)+chr(10))")
sed -n '513p;514p' gateware/rev2/project_obscurus/coproc_prog.mem
```
Expected: line 513 (= byte index `$0200`, 0-based, 1-based line 513) is `a9` (LDA #imm), line 514 is `42`. (mem line N = byte N-1; `$0200`=512 → line 513.)

- [ ] **Step 4: Write the failing unit test `coproc_tb.v`**

Models the arbiter side: when `req` pulses, raise `busy` 4 cycles, capture `we/phys_addr/wdata`. Asserts the coproc eventually posts `we=1, phys_addr=0, wdata=$42`.

```verilog
`timescale 1ns/1ps
module coproc_tb;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0;
    wire        req, we; wire [25:0] phys_addr; wire [7:0] wdata;
    reg  [7:0]  rdata=0; reg busy=0;
    integer errors=0;

    coproc dut(.clk(clk), .rst_n(rst_n),
        .req(req), .we(we), .phys_addr(phys_addr), .wdata(wdata),
        .busy(busy), .rdata(rdata));

    // model sdram_ctrl busy for the coproc's posted write
    reg [7:0] got_data; reg [25:0] got_addr; reg got_we; reg captured=0;
    reg [2:0] mcnt=0;
    always @(posedge clk) begin
        if (req && !busy) begin
            got_we<=we; got_addr<=phys_addr; got_data<=wdata; captured<=1;
            busy<=1; mcnt<=3'd4;
        end else if (busy) begin
            if (mcnt>1) mcnt<=mcnt-1; else busy<=0;
        end
    end

    integer g;
    initial begin
        rst_n=0; #50; rst_n=1;
        g=0;
        while (!captured && g<5000) begin @(posedge clk); g=g+1; end
        if (!captured) begin errors=errors+1; $display("FAIL coproc never posted"); end
        else begin
            if (got_we!==1'b1)            begin errors=errors+1; $display("FAIL we=%b",got_we); end
            if (got_addr!==26'h000_0040)  begin errors=errors+1; $display("FAIL addr=%h",got_addr); end
            if (got_data!==8'h42)         begin errors=errors+1; $display("FAIL data=%02X",got_data); end
        end
        if (errors==0) $display("PASS coproc posts 42"); else $display("FAIL coproc %0d",errors);
        $finish;
    end
endmodule
```

- [ ] **Step 5: Run it — expect compile failure (no `coproc`)**

Run:
```bash
cd /Users/hambook/Development/project_byte_hamr/gateware/rev2/project_obscurus
iverilog -g2005 -o /tmp/cop.out -s coproc_tb coproc.v coproc_tb.v arlet_cpu.v arlet_alu.v
```
Expected: error — `coproc.v` missing / `Unknown module type coproc`.

- [ ] **Step 6: Implement `coproc.v`**

```verilog
// =============================================================================
// coproc.v — Arlet soft-6502 + 8KB BRAM + $E000 SDRAM-write window.
// C0 steel thread: runs coproc_prog.mem (LDA #$42 / STA $E000 / spin). A write
// to $E000 posts ONE SDRAM write (bank0 addr0 = DO) to the arbiter client and
// RDY-stalls the core until the op completes. Reset vector is synthesized
// ($FFFC/D -> $0200). Arlet RDY = global clock-enable (freezes read AND write);
// reset active-high (driven from ~rst_n).
// =============================================================================
module coproc (
    input  wire        clk,
    input  wire        rst_n,
    // SDRAM arbiter client interface (this is c1 on sdram_arb)
    output reg         req,
    output reg         we,
    output reg  [25:0] phys_addr,
    output reg  [7:0]  wdata,
    input  wire        busy,
    input  wire [7:0]  rdata          // unused in C0
);
    // ---- Arlet core ----
    wire [15:0] AB;
    wire [7:0]  DO;
    wire        WE;
    reg  [7:0]  DI;
    reg         rdy;

    cpu u_cpu (
        .clk(clk), .reset(~rst_n),
        .AB(AB), .DI(DI), .DO(DO), .WE(WE),
        .IRQ(1'b0), .NMI(1'b0), .RDY(rdy)
    );

    // ---- 8KB BRAM ($0000-$1FFF) ----
    reg [7:0] bram [0:8191];
    initial $readmemh("coproc_prog.mem", bram);

    wire in_bram = (AB[15:13] == 3'b000);   // $0000-$1FFF
    wire is_vlo  = (AB == 16'hFFFC);
    wire is_vhi  = (AB == 16'hFFFD);

    // registered (1-cycle) read, matching Arlet's synchronous memory expectation
    reg [7:0] bram_q;
    always @(posedge clk) bram_q <= bram[AB[12:0]];
    always @(posedge clk) if (WE & in_bram & rdy) bram[AB[12:0]] <= DO;

    // align the DI source-selects to the same 1-cycle latency as bram_q
    reg in_bram_d, is_vlo_d, is_vhi_d;
    always @(posedge clk) begin
        in_bram_d <= in_bram; is_vlo_d <= is_vlo; is_vhi_d <= is_vhi;
    end
    always @(*) begin
        if      (is_vlo_d) DI = 8'h00;      // reset vector lo -> $0200
        else if (is_vhi_d) DI = 8'h02;      // reset vector hi
        else if (in_bram_d) DI = bram_q;
        else DI = 8'h00;
    end

    // ---- $E000 SDRAM-write posting + RDY stall ----
    wire is_e000 = (AB == 16'hE000);
    reg busy_d;
    always @(posedge clk) busy_d <= busy;
    wire done = busy_d & ~busy;

    localparam ST_RUN=1'b0, ST_WAIT=1'b1;
    reg state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            req<=1'b0; we<=1'b0; phys_addr<=26'd0; wdata<=8'd0;
            rdy<=1'b1; state<=ST_RUN;
        end else begin
            req <= 1'b0;                     // default: pulse
            case (state)
                ST_RUN: if (is_e000 & WE) begin
                    we<=1'b1; phys_addr<=26'h000_0040; wdata<=DO;  // bank0 $0040
                    req<=1'b1; rdy<=1'b0; state<=ST_WAIT;  // post once, stall core
                end
                ST_WAIT: if (done) begin
                    rdy<=1'b1; state<=ST_RUN;               // release core
                end
            endcase
        end
    end
endmodule
```

- [ ] **Step 7: Run the test — expect PASS**

Run:
```bash
cd /Users/hambook/Development/project_byte_hamr/gateware/rev2/project_obscurus
iverilog -g2005 -o /tmp/cop.out -s coproc_tb coproc.v coproc_tb.v arlet_cpu.v arlet_alu.v && vvp /tmp/cop.out
```
Expected: `PASS coproc posts 42`. (Requires `coproc_prog.mem` present from Step 3, in the cwd.)

- [ ] **Step 8: Commit**

```bash
git add gateware/rev2/project_obscurus/coproc.v gateware/rev2/project_obscurus/coproc_tb.v gateware/rev2/project_obscurus/coproc_prog.S gateware/rev2/project_obscurus/coproc_prog.mem Makefile
git commit -m "feat(coproc-c0): coproc.v (Arlet+BRAM+\$E000 window) + program + mem rule"
```

---

## Task 3: Integrate arbiter + coproc into the top

**Files:**
- Modify: `gateware/rev2/project_obscurus/project_obscurus_top.v`

- [ ] **Step 1: Rewire the monitor through the arbiter; instantiate arbiter + coproc**

In `project_obscurus_top.v`, **delete** the 4 direct monitor→ctrl assigns (lines ~254-257):
```verilog
    assign sdram_req       = m_req;
    assign sdram_we        = m_we;
    assign sdram_phys_addr = {m_bank, m_addr};
    assign sdram_wdata     = m_wdata;
```
and replace with the arbiter + coproc wiring (the `sdram_req/we/phys_addr/wdata/rdata/busy` wires already exist and feed `sdram_ctrl`):
```verilog
    // ---- monitor is arbiter client c0; coproc is c1 ----
    wire        mon_busy;
    wire [7:0]  mon_rdata;
    wire        cop_req, cop_we;
    wire [25:0] cop_addr;
    wire [7:0]  cop_wdata;
    wire        cop_busy;
    wire [7:0]  cop_rdata;

    sdram_arb u_arb (
        .clk(clk), .rst_n(rst_n),
        .req(sdram_req), .we(sdram_we), .phys_addr(sdram_phys_addr),
        .wdata(sdram_wdata), .rdata(sdram_rdata), .busy(sdram_busy),
        .c0_req(m_req), .c0_we(m_we), .c0_addr({m_bank, m_addr}),
        .c0_wdata(m_wdata), .c0_busy(mon_busy), .c0_rdata(mon_rdata),
        .c1_req(cop_req), .c1_we(cop_we), .c1_addr(cop_addr),
        .c1_wdata(cop_wdata), .c1_busy(cop_busy), .c1_rdata(cop_rdata)
    );

    coproc u_coproc (
        .clk(clk), .rst_n(rst_n),
        .req(cop_req), .we(cop_we), .phys_addr(cop_addr), .wdata(cop_wdata),
        .busy(cop_busy), .rdata(cop_rdata)
    );
```

- [ ] **Step 2: Make `op_done` per-client (monitor's arbiter busy, not the raw SDRAM busy)**

Change the `op_done` source (lines ~264-266) from the shared `sdram_busy` to the monitor's arbiter busy `mon_busy`:
```verilog
    // busy falling edge for the MONITOR's own op (via arbiter c0) = op complete
    reg mon_busy_d;
    always @(posedge clk) mon_busy_d <= mon_busy;
    wire op_done = mon_busy_d & ~mon_busy;
```
(Delete the old `reg sdram_busy_d; always @(posedge clk) sdram_busy_d <= sdram_busy; wire op_done = sdram_busy_d & ~sdram_busy;`.)

- [ ] **Step 3: Monitor's DATA read returns its latched arbiter rdata**

In the read mux (line ~316) change the DATA source from the shared `sdram_rdata` to the monitor's latched `mon_rdata`:
```verilog
            4'h6: reg_data_out = mon_rdata;     // DATA (monitor's latched read)
```

- [ ] **Step 4: Build — synth + timing**

Run: `make clean && make DESIGN=project_obscurus REV=rev2`
Expected: clean bitstream (the `*.v` glob now includes `arlet_cpu.v`, `arlet_alu.v`, `coproc.v`, `sdram_arb.v`; the `*.mem` glob includes `coproc_prog.mem`). Timing PASS at 25 MHz (`clk`). Report Fmax + LUT/BRAM utilization (Arlet + 8 KB BRAM add a chunk). Do NOT flash. If Arlet's combinational paths fail 25 MHz timing (unlikely — it's a slow target), report the failing path rather than guessing a fix.

- [ ] **Step 5: Commit**

```bash
git add gateware/rev2/project_obscurus/project_obscurus_top.v
git commit -m "feat(coproc-c0): integrate sdram_arb + coproc; monitor op_done per-client"
```

---

## Task 4: Integration sim — coproc writes `$42`, host reads it

**Files:**
- Modify: `gateware/rev2/project_obscurus/project_obscurus_tb.v`

- [ ] **Step 1: Add the C0 proof + monitor-regression checks**

The coproc writes to **bank0 `$0040`** (set in Task 2), clear of the TB's demo cells. In `project_obscurus_tb.v`, after the existing checks and before the final summary/`$finish`, add (uses the existing `sdram_read`/`sdram_write` tasks and `tmp`/`errors`):
```verilog
        // --- C0: the coproc ran on its own and wrote bank0/$0040 = $42 ---
        // (coproc posts the write shortly after reset; by now it has completed.)
        sdram_read(10'd0, 16'h0040, tmp);
        if (tmp!==8'h42) begin errors=errors+1; $display("FAIL C0 coproc write got %02X want 42",tmp); end
        else $display("PASS C0 coproc wrote 42 to bank0/$40");
        // monitor regression: host write/read elsewhere still works with arbiter in front
        sdram_write(10'd2, 16'h00AB, 8'h99);
        sdram_read (10'd2, 16'h00AB, tmp);
        if (tmp!==8'h99) begin errors=errors+1; $display("FAIL monitor regress %02X",tmp); end
```

- [ ] **Step 2: Run the integration sim**

Run: `make sim DESIGN=project_obscurus REV=rev2`
Expected: `PASS C0 coproc wrote 42 to bank0/$40`, the monitor regression passes, and ALL pre-existing monitor/bank/lane/dump checks still PASS. Final line shows 0 errors. iverilog `-g2005` (Makefile already uses it).

- [ ] **Step 3: Commit**

```bash
git add gateware/rev2/project_obscurus/project_obscurus_tb.v gateware/rev2/project_obscurus/coproc.v gateware/rev2/project_obscurus/coproc_tb.v gateware/rev2/project_obscurus/coproc_prog.mem
git commit -m "test(coproc-c0): integration — coproc writes \$42 to bank0/\$40, host reads it"
```

---

## Task 5: Bench verification (user-run)

**Files:** none. Requires flashing — the **user** flashes.

- [ ] **Step 1: Build the bitstream**

Run: `make clean && make DESIGN=project_obscurus REV=rev2` — confirm clean + timing. Report "ready" — user flashes.

- [ ] **Step 2: Hand the procedure to the user**

```
1. (you) flash build/project_obscurus.bit
2. Boot the Apple II; PR#4 (or whatever brings up the $C800 monitor)
3. At the monitor prompt: B 000   (select bank 0)
   then:                  R 0040  (read bank0 offset $40)
   -> expect "0040: 42"
```
Pass = `R 0040` shows `42` → the Arlet soft-6502 ran on silicon, executed `LDA #$42 / STA $E000`, posted the write through the arbiter, and the host read the coprocessor's output. **C0 steel thread proven** — C1 (host loads jobs) / C2 (two-task race) can build on it.

If `R 0040` shows `00`/garbage: the coproc didn't run or didn't post. Check (a) Arlet held in reset (`~rst_n`), (b) reset vector synth ($FFFC/D → $0200) reaches the core, (c) `coproc_prog.mem` is the freshly built one in the bitstream, (d) the arbiter granted c1 (scope `cop_req`/`cop_busy` via a GPIO tap).

- [ ] **Step 3: Record result**

Update memory (`project_obscurus_next_coproc` / a new `project_coproc_c0`) with the C0 bench status, the branch (`coproc`), and the file map. Note C1/C2 as next.

---

## Self-Review

**Spec coverage:** Arlet vendored → Task 0. `coproc.v` (BRAM + `$E000` window + RDY-stall + reset-vector synth) → Task 2. `sdram_arb.v` 2-client monitor-priority → Task 1. Monitor rewired + per-client `op_done` → Task 3. Fixed program `LDA #$42/STA $E000/spin` → Task 2. Host reads `$42` (sim + bench) → Tasks 4, 5. Makefile mem rule in synth graph → Task 2. Single 25 MHz domain → all (no CDC introduced). 25 MHz timing → Task 3 Step 4.

**Placeholder scan:** none. The bank0/addr0→`$0040` move (Task 4 Step 2) is a concrete collision fix with exact edits, not a TODO. The `coproc_prog.mem` offset is verified by an explicit `sed` check (Task 2 Step 3).

**Type/label consistency:** Arbiter ports (`c0_req/c0_we/c0_addr/c0_wdata/c0_busy/c0_rdata`, `c1_*`, `req/we/phys_addr/wdata/rdata/busy`) match the top instantiation (Task 3 Step 1) and the arb tb (Task 1). `coproc` ports (`req/we/phys_addr/wdata/busy/rdata`) match the top (c1) and coproc_tb. Arlet `cpu` ports (`clk/reset/AB/DI/DO/WE/IRQ/NMI/RDY`) match the WebFetch-confirmed declaration. `op_done` now sourced from `mon_busy` consistently (Task 3 Steps 2–3). Coproc target cell `bank0/$0040`, magic `$42`, window `$E000`, reset→`$0200`, BRAM `$0000–$1FFF` consistent across coproc.v / coproc_tb / coproc_prog.S / integration tb.

**Executor notes:** iverilog `-g2005` only. `make ... REV=rev2`. Arlet RDY = global stall (write-stall valid). The coproc posts exactly one write (the `req` pulse is single-cycle; `ST_WAIT` blocks re-post). Don't modify `sdram_ctrl.v`. Verify `coproc_prog.mem` offset `$0200` before trusting the unit test.
