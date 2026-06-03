# project_obscurus SDRAM Monitor — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A self-contained 6502 monitor in the card's `$C800` expansion ROM that does live random-access read/write to the full 64 MB SDRAM through a `$C0Cx` register port, with 64 KB bank switching (R/W/B/D commands).

**Architecture:** A new isolated `sdram_ctrl.v` (request interface + byte-packed addressing + priority refresh) replaces the inline boot-string FSM. Bus-side glue in `project_obscurus_top.v` latches address/bank/data registers, drives the controller, and exposes a busy/ready STATUS for a poll handshake. A Merlin32 monitor (`monitor.S` at `$C800`, launched by a `$C400` stub) parses commands and drives the register protocol. RTL is verified by an isolated `sdram_ctrl` unit testbench plus an integration testbench before any bitstream build; the 6502 monitor is verified on the bench.

**Tech Stack:** Verilog (iverilog `-g2012` sim, yosys/nextpnr/ecppack synth for ECP5 LFE5U-85F), Merlin32 6502 assembler, `scripts/rom2mem.py`.

**Spec:** `docs/superpowers/specs/2026-06-03-obscurus-sdram-monitor-design.md`

---

## File Structure

| File | Responsibility | Action |
|------|----------------|--------|
| `gateware/rev2/project_obscurus/sdram_ctrl.v` | SDRAM controller: init, **priority free-running refresh**, single-byte read/write request interface, byte-packed DQM, `{BA,ROW,COL}` decomposition | Create |
| `gateware/rev2/project_obscurus/sim/sdram_model.v` | Faithful behavioral SDRAM model (sparse assoc array, row@ACTIVE/col@R/W, A10-masked column, DQM-honoring) — shared by both TBs | Create |
| `gateware/rev2/project_obscurus/sdram_ctrl_tb.v` | Unit TB: drives the request interface, asserts round-trip / bank-isolation / byte-lane / auto-increment / busy@strobe | Create |
| `gateware/rev2/project_obscurus/project_obscurus_top.v` | Strip boot-string demo; instantiate `sdram_ctrl`; add `monitor_regs` (addr/bank/data latches, strobe, busy@strobe, auto-inc on done, STATUS); add expansion-ROM enable FF + `monitor_mem` + gated `d_oe`/`DATA_OE` | Modify |
| `gateware/rev2/project_obscurus/project_obscurus_tb.v` | Integration TB: register-port protocol with `poll_busy`, full command sequences, `$CFFF` assert | Rewrite |
| `gateware/rev2/project_obscurus/slot_rom.S` | `$C400` stub: restore CSW, `JMP $C800` | Rewrite |
| `gateware/rev2/project_obscurus/monitor.S` | Monitor program at `$C800`: GETLN loop, hex parser (high-bit masked), R/W/B/D/Q | Create |
| `Makefile` | Add `-g2012` + `$(SIM_MODELS)` to sim/unit recipes; add `monitor.mem` recipe; gate `$(JSON)` | Modify |
| `gateware/rev2/project_obscurus/README.md` | Update memory map / usage | Modify |

**Geometry (used throughout — derive once, reuse):** physical byte address `phys_addr[25:0]`, `phys_word = phys_addr[25:1]`:
- `COL[9:0]  = phys_addr[10:1]`
- `ROW[12:0] = phys_addr[23:11]`
- `BA[1:0]   = phys_addr[25:24]`
- `byte_lane = phys_addr[0]` (0 = low byte D[7:0])

**SDRAM command encoding** `{nCS,nRAS,nCAS,nWE}` (matches existing top.v): NOP=`0111`, ACTIVE=`0011`, READ=`0101`, WRITE=`0100`, PRECHARGE=`0010`, REFRESH=`0001`, LOAD_MODE=`0000`.

---

## Task 1: Build wiring (sim `-g2012`, shared sim models, `monitor.mem` recipe)

**Files:**
- Modify: `Makefile:316-318` (sim compile recipe), `Makefile:355` (unit compile recipe), `Makefile:160-173` (obscurus ROM recipes)

> **ENVIRONMENT NOTE (discovered at execution):** This machine's iverilog
> (Icarus 13.0 devel) **hangs on `-g2009` and `-g2012`** — only the default
> `-g2005` works. So do **NOT** add any `-gXXXX` flag. The SDRAM sim model
> (Task 2) therefore uses a **bounded dense array**, not a SystemVerilog
> associative array.

- [ ] **Step 1: Add `$(SIM_MODELS)` to the sim + unit recipes (no `-gXXXX` flag)**

In `Makefile`, the sim compile recipe (around line 327-331) already includes
`$(SIM_MODELS)` on the compile line — leave that line as-is (no flag):

```make
	$(IVERILOG) -o $@ -s $(DESIGN)_tb $(VERILOG_SRC) $(SIM_MODELS) $(SIM_MAIN_TB) $(SIM_AUX_TB)
```

The unit recipe (around line 366-368) must gain `$(SIM_MODELS)` in BOTH the
prerequisites and the compile command so unit TBs can use `sim/*.v` models:

```make
$(UNIT_OUT): $(VERILOG_SRC) $(SIM_MODELS) $(UNIT_TB) | $(BUILD_DIR)
	@echo "=== Compiling Unit Testbench: $(MODULE) ==="
	$(IVERILOG) -o $@ -s $(MODULE)_tb $(VERILOG_SRC) $(SIM_MODELS) $(UNIT_TB)
```

- [ ] **Step 2: Add the `monitor.mem` recipe and gate it into the build**

In `Makefile`, immediately after the existing `OBSCURUS_ROM_MEM` block (ends near line 169 with the `slot_rom.bin` → `.mem` `rom2mem.py` call), add:

```make
# project_obscurus monitor ROM (2KB at $C800). Merlin32 source -> .bin -> .mem.
# NOTE: rom2mem base MUST be 0xC000 (not 0xC800): rom2mem computes
# offset = base - 0xC000 and writes bin at rom[offset]; base=0xC800 gives
# offset==size -> the guard rejects every byte -> all-$FF brick. base=0xC000
# -> offset 0 -> bin lands at mem[0], indexed by monitor_mem[apple_addr[10:0]].
OBSCURUS_MON_MEM := $(GATEWARE_DIR)/project_obscurus/monitor.mem
OBSCURUS_MON_SRC := $(GATEWARE_DIR)/project_obscurus/monitor.S

$(OBSCURUS_MON_MEM): $(OBSCURUS_MON_SRC)
	@echo "=== Assembling project_obscurus monitor ROM (Merlin32) ==="
	cd $(GATEWARE_DIR)/project_obscurus && $(MERLIN32) $(MERLIN_LIB) monitor.S
	python3 scripts/rom2mem.py $(GATEWARE_DIR)/project_obscurus/monitor.bin $@ 0xC000 2048
```

Then extend the existing gate block (currently `ifeq ($(DESIGN),project_obscurus)` / `$(JSON): $(OBSCURUS_ROM_MEM)` near line 172) so it reads:

```make
ifeq ($(DESIGN),project_obscurus)
$(JSON): $(OBSCURUS_ROM_MEM) $(OBSCURUS_MON_MEM)
endif
```

- [ ] **Step 3: Verify the existing sim still compiles under `-g2012`**

Run: `make sim DESIGN=project_obscurus REV=rev2`
Expected: compiles and runs the *current* TB; prints `PASS` (the existing hello-world TB). This confirms `-g2012` did not break the toolchain. (After Task 5 this TB is replaced.)

- [ ] **Step 4: Commit**

```bash
git add Makefile
git commit -m "build: add -g2012 + shared sim models + monitor.mem recipe for obscurus"
```

---

## Task 2: Faithful SDRAM behavioral model (`sim/sdram_model.v`)

This model is the safety net. It must honor the four traps from the spec or sim passes while hardware fails.

**Files:**
- Create: `gateware/rev2/project_obscurus/sim/sdram_model.v`

- [ ] **Step 1: Write the model**

```verilog
`timescale 1ns / 1ps
// =============================================================================
// sdram_model.v — faithful behavioral model for project_obscurus sim
// =============================================================================
// Honors:
//   1. BOUNDED dense array (Verilog-2005, NO -g2012 — this iverilog hangs on
//      SystemVerilog). 64K words (128KB) covers banks 0-1 = all tests touch.
//      Accesses beyond WORDS print an error so a stray high address is caught,
//      not silently aliased.
//   2. Row+bank captured on ACTIVE; column captured on READ/WRITE (NOT ACTIVE).
//   3. Column = A[9:0] only — A10 (auto-precharge) is masked out.
//   4. DQM byte lanes honored on write (only the unmasked lane updates).
// Word index = {ba[1:0], row[12:0], col[9:0]} = 25-bit word address.
// =============================================================================
module sdram_model (
    input              clk,        // SDRAM_CLK
    input  [3:0]       cmd,        // {nCS,nRAS,nCAS,nWE}
    input  [1:0]       ba,
    input  [12:0]      a,
    input              dqm0,
    input              dqm1,
    inout  [15:0]      dq
);
    localparam CMD_ACTIVE = 4'b0011;
    localparam CMD_READ   = 4'b0101;
    localparam CMD_WRITE  = 4'b0100;

    localparam WORDS = 65536;      // banks 0-1 (word 0..65535)
    reg [15:0] mem [0:WORDS-1];
    integer ii;
    initial for (ii = 0; ii < WORDS; ii = ii + 1) mem[ii] = 16'h0000;

    reg [1:0]  cur_ba  = 2'd0;
    reg [12:0] cur_row = 13'd0;

    reg [15:0] dq_drive = 16'h0000;
    reg        dq_oe    = 1'b0;
    assign dq = dq_oe ? dq_drive : 16'hZZZZ;

    // CL=2 read pipeline
    reg        rd_pending = 1'b0;
    integer    rd_latency = 0;
    reg [24:0] rd_widx    = 25'd0;

    function [24:0] widx(input [1:0] b, input [12:0] r, input [9:0] c);
        widx = {b, r, c};
    endfunction

    reg [24:0] w;

    always @(posedge clk) begin
        // ---- ACTIVE: latch bank + row ----
        if (cmd == CMD_ACTIVE) begin
            cur_ba  <= ba;
            cur_row <= a;            // full 13-bit row
        end
        // ---- WRITE: column = a[9:0] (A10 masked), DQM lane select ----
        if (cmd == CMD_WRITE) begin
            w = widx(cur_ba, cur_row, a[9:0]);
            if (w >= WORDS) $display("MODEL ERR: write widx %0d out of range", w);
            else begin
                if (!dqm0) mem[w][7:0]  = dq[7:0];   // same-cycle masked write
                if (!dqm1) mem[w][15:8] = dq[15:8];
            end
        end
        // ---- READ: column = a[9:0], schedule CL=2 drive ----
        if (cmd == CMD_READ) begin
            rd_pending <= 1'b1;
            rd_latency <= 2;
            rd_widx    <= widx(cur_ba, cur_row, a[9:0]);
        end else if (rd_pending) begin
            if (rd_latency == 0) begin
                dq_drive   <= (rd_widx < WORDS) ? mem[rd_widx[15:0]] : 16'h0000;
                dq_oe      <= 1'b1;
                rd_pending <= 1'b0;
            end else begin
                rd_latency <= rd_latency - 1;
                dq_oe      <= 1'b0;
            end
        end else begin
            dq_oe <= 1'b0;
        end
    end
endmodule
```

- [ ] **Step 2: Commit**

```bash
git add gateware/rev2/project_obscurus/sim/sdram_model.v
git commit -m "sim: faithful SDRAM model (sparse, row@ACTIVE/col@R-W, A10 mask, DQM)"
```

---

## Task 3: `sdram_ctrl` unit testbench (failing test first)

**Files:**
- Create: `gateware/rev2/project_obscurus/sdram_ctrl_tb.v`

- [ ] **Step 1: Write the unit testbench**

```verilog
`timescale 1ns / 1ps
// Unit TB for sdram_ctrl: drives the request interface against sdram_model.
module sdram_ctrl_tb;
    reg clk = 0;
    always #20 clk = ~clk;          // 25 MHz

    reg         rst_n = 0;
    reg         req   = 0;
    reg         we    = 0;
    reg  [25:0] phys_addr = 0;
    reg  [7:0]  wdata = 0;
    wire [7:0]  rdata;
    wire        busy, ready;

    // SDRAM wires
    wire [3:0]  cmd = {SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE};
    wire SDRAM_CLK, SDRAM_CKE, SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE;
    wire SDRAM_DQM0, SDRAM_DQM1, SDRAM_BA0, SDRAM_BA1;
    wire [12:0] sdram_a;
    wire [15:0] dq_out;
    wire        dq_oe;
    wire [15:0] dq;
    assign dq = dq_oe ? dq_out : 16'hZZZZ;

    sdram_ctrl dut (
        .clk(clk), .rst_n(rst_n),
        .req(req), .we(we), .phys_addr(phys_addr), .wdata(wdata),
        .rdata(rdata), .busy(busy), .ready(ready),
        .SDRAM_CKE(SDRAM_CKE),
        .SDRAM_nCS(SDRAM_nCS), .SDRAM_nRAS(SDRAM_nRAS),
        .SDRAM_nCAS(SDRAM_nCAS), .SDRAM_nWE(SDRAM_nWE),
        .SDRAM_DQM0(SDRAM_DQM0), .SDRAM_DQM1(SDRAM_DQM1),
        .SDRAM_BA0(SDRAM_BA0), .SDRAM_BA1(SDRAM_BA1),
        .sdram_a(sdram_a),
        .dq_out(dq_out), .dq_oe(dq_oe), .dq_in(dq)
    );

    sdram_model model (
        .clk(SDRAM_CLK), .cmd(cmd), .ba({SDRAM_BA1, SDRAM_BA0}),
        .a(sdram_a), .dqm0(SDRAM_DQM0), .dqm1(SDRAM_DQM1), .dq(dq)
    );
    assign SDRAM_CLK = clk;          // ctrl drives clk out; tie model to same

    integer errors = 0;
    reg [7:0] got;

    // Issue one request and wait for completion.
    task do_req(input rw_we, input [25:0] a, input [7:0] d);
        begin
            @(posedge clk);
            we <= rw_we; phys_addr <= a; wdata <= d; req <= 1'b1;
            @(posedge clk);
            req <= 1'b0;
            // busy must be asserted by now
            wait (busy);
            wait (!busy);
            @(posedge clk);
        end
    endtask

    task rd(input [25:0] a, output [7:0] d);
        begin
            do_req(1'b0, a, 8'h00);
            d = rdata;
        end
    endtask

    initial begin
        $dumpfile("sdram_ctrl_tb.vcd");
        $dumpvars(0, sdram_ctrl_tb);
        rst_n = 0; #200; rst_n = 1;
        wait (ready);
        $display("[%0t] ready", $time);

        // 1. round-trip
        do_req(1'b1, 26'h00_0000, 8'hA5);
        rd(26'h00_0000, got);
        if (got !== 8'hA5) begin errors=errors+1; $display("FAIL rt: %02X",got); end

        // 2. byte-lane isolation: 2k and 2k+1 share a word
        do_req(1'b1, 26'h00_0010, 8'h11);   // even -> low lane
        do_req(1'b1, 26'h00_0011, 8'h22);   // odd  -> high lane
        rd(26'h00_0010, got);
        if (got !== 8'h11) begin errors=errors+1; $display("FAIL lane lo: %02X",got); end
        rd(26'h00_0011, got);
        if (got !== 8'h22) begin errors=errors+1; $display("FAIL lane hi: %02X",got); end

        // 3. bank isolation: bank 1 offset 0 = phys 0x10000
        do_req(1'b1, 26'h01_0000, 8'h5A);
        rd(26'h00_0000, got);
        if (got !== 8'hA5) begin errors=errors+1; $display("FAIL bank0 clobbered: %02X",got); end
        rd(26'h01_0000, got);
        if (got !== 8'h5A) begin errors=errors+1; $display("FAIL bank1: %02X",got); end

        if (errors==0) $display("PASS"); else $display("FAIL: %0d errors", errors);
        $finish;
    end

    initial begin #2_000_000; $display("TIMEOUT"); $finish; end
endmodule
```

- [ ] **Step 2: Run to verify it fails (no `sdram_ctrl` yet)**

Run: `make unit DESIGN=project_obscurus MODULE=sdram_ctrl REV=rev2`
Expected: FAIL — iverilog error `Unknown module type: sdram_ctrl`.

- [ ] **Step 3: Commit the failing test**

```bash
git add gateware/rev2/project_obscurus/sdram_ctrl_tb.v
git commit -m "test: sdram_ctrl unit TB (round-trip, byte-lane, bank isolation)"
```

---

## Task 4: Implement `sdram_ctrl.v` (make the unit test pass)

**Files:**
- Create: `gateware/rev2/project_obscurus/sdram_ctrl.v`

- [ ] **Step 1: Write the controller**

```verilog
// =============================================================================
// sdram_ctrl.v — byte-addressed SDRAM controller for project_obscurus
// =============================================================================
// AS4C32M16: {BA[1:0], ROW[12:0], COL[9:0]} = 25-bit word address.
//   phys_word = phys_addr[25:1]
//   COL = phys_addr[10:1], ROW = phys_addr[23:11], BA = phys_addr[25:24]
//   byte_lane = phys_addr[0]   (0 = low byte D[7:0])
// Byte-packed: write masks one lane via DQM; read takes both lanes, selects.
// Refresh: free-running counter, PRIORITY over a pending request when due.
// Request: pulse `req` (with `we`,`phys_addr`,`wdata`) when !busy. `busy` rises
// next cycle, falls when the access completes; `rdata` valid while !busy after
// a read.
// =============================================================================
module sdram_ctrl (
    input  wire        clk,
    input  wire        rst_n,

    // request interface
    input  wire        req,
    input  wire        we,
    input  wire [25:0] phys_addr,
    input  wire [7:0]  wdata,
    output reg  [7:0]  rdata,
    output reg         busy,
    output reg         ready,

    // SDRAM (dq tristate kept in top via dq_out/dq_oe/dq_in)
    output reg         SDRAM_CKE,
    output reg         SDRAM_nCS,
    output reg         SDRAM_nRAS,
    output reg         SDRAM_nCAS,
    output reg         SDRAM_nWE,
    output reg         SDRAM_DQM0,
    output reg         SDRAM_DQM1,
    output reg         SDRAM_BA0,
    output reg         SDRAM_BA1,
    output reg [12:0]  sdram_a,
    output reg [15:0]  dq_out,
    output reg         dq_oe,
    input  wire [15:0] dq_in
);
    localparam [3:0]
        CMD_NOP=4'b0111, CMD_ACTIVE=4'b0011, CMD_READ=4'b0101,
        CMD_WRITE=4'b0100, CMD_PRECHARGE=4'b0010, CMD_REFRESH=4'b0001,
        CMD_LOAD_MODE=4'b0000;

    localparam [3:0]
        ST_INIT=4'd0, ST_PRECH=4'd1, ST_REF1=4'd2, ST_REF2=4'd3, ST_LMR=4'd4,
        ST_IDLE=4'd5, ST_ACT=4'd6, ST_RW=4'd7, ST_WAIT=4'd8, ST_RDLAT=4'd9,
        ST_REFNOW=4'd10;

    reg [3:0]  st = ST_INIT;
    reg [15:0] dly = 16'd5000;
    reg [3:0]  cmd;

    // latched request
    reg        pend = 1'b0;
    reg        pend_we;
    reg [1:0]  pend_ba;
    reg [12:0] pend_row;
    reg [9:0]  pend_col;
    reg        pend_lane;
    reg [7:0]  pend_wdata;

    // free-running refresh
    reg [15:0] ref_cnt = 16'd0;
    reg        ref_due = 1'b0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st<=ST_INIT; dly<=16'd5000; ready<=1'b0; busy<=1'b0;
            SDRAM_CKE<=1'b0; pend<=1'b0; ref_cnt<=16'd0; ref_due<=1'b0;
            dq_oe<=1'b0; rdata<=8'h00;
        end else begin
            cmd        = CMD_NOP;     // default this cycle
            dq_oe     <= 1'b0;
            SDRAM_DQM0<= 1'b0;
            SDRAM_DQM1<= 1'b0;

            // ---- free-running refresh timer (NOT reset by accesses) ----
            if (ref_cnt == 16'd195) begin ref_cnt<=16'd0; ref_due<=1'b1; end
            else ref_cnt <= ref_cnt + 1'b1;

            // ---- latch an incoming request (accepted only when idle/!busy) ----
            if (req && !busy && !pend && ready) begin
                pend       <= 1'b1;
                pend_we    <= we;
                pend_ba    <= phys_addr[25:24];
                pend_row   <= phys_addr[23:11];
                pend_col   <= phys_addr[10:1];
                pend_lane  <= phys_addr[0];
                pend_wdata <= wdata;
            end

            case (st)
                ST_INIT: begin
                    SDRAM_CKE<=1'b1;
                    if (dly==0) st<=ST_PRECH; else dly<=dly-1'b1;
                end
                ST_PRECH: begin
                    cmd=CMD_PRECHARGE; sdram_a<=13'b0_0100_0000_0000; // A10=1 all
                    dly<=16'd2; st<=ST_REF1;
                end
                ST_REF1: begin
                    if (dly==0) begin cmd=CMD_REFRESH; dly<=16'd8; st<=ST_REF2; end
                    else dly<=dly-1'b1;
                end
                ST_REF2: begin
                    if (dly==0) begin cmd=CMD_REFRESH; dly<=16'd8; st<=ST_LMR; end
                    else dly<=dly-1'b1;
                end
                ST_LMR: begin
                    if (dly==0) begin
                        cmd=CMD_LOAD_MODE;
                        {SDRAM_BA1,SDRAM_BA0}<=2'd0;
                        sdram_a<=13'b000_0_00_010_0_000;  // CL=2, burst=1
                        dly<=16'd2; ready<=1'b1; st<=ST_IDLE;
                    end else dly<=dly-1'b1;
                end

                ST_IDLE: begin
                    busy <= (pend || ref_due);  // hold busy while work outstanding
                    if (ref_due) begin
                        ref_due<=1'b0; st<=ST_REFNOW;
                    end else if (pend) begin
                        {SDRAM_BA1,SDRAM_BA0}<=pend_ba;
                        st<=ST_ACT;
                    end else begin
                        busy<=1'b0;
                    end
                end

                ST_REFNOW: begin
                    cmd=CMD_REFRESH; dly<=16'd8; st<=ST_IDLE;
                end

                ST_ACT: begin
                    cmd=CMD_ACTIVE; sdram_a<={pend_row};  // 13-bit row
                    dly<=16'd2; st<=ST_RW;
                end
                ST_RW: begin
                    if (dly==0) begin
                        // A10=1 auto-precharge, A[9:0]=col
                        sdram_a <= {2'b00, 1'b1, pend_col};
                        if (pend_we) begin
                            cmd=CMD_WRITE;
                            dq_out    <= {pend_wdata, pend_wdata};
                            dq_oe     <= 1'b1;
                            SDRAM_DQM0<= pend_lane;     // low lane enabled when even
                            SDRAM_DQM1<= ~pend_lane;    // high lane enabled when odd
                            dly<=16'd3; st<=ST_WAIT;
                        end else begin
                            cmd=CMD_READ;
                            SDRAM_DQM0<=1'b0; SDRAM_DQM1<=1'b0;  // read both lanes
                            dly<=16'd2; st<=ST_WAIT;
                        end
                    end else dly<=dly-1'b1;
                end
                ST_WAIT: begin
                    if (dly==0) begin
                        if (pend_we) begin pend<=1'b0; busy<=1'b0; st<=ST_IDLE; end
                        else st<=ST_RDLAT;
                    end else dly<=dly-1'b1;
                end
                ST_RDLAT: begin
                    rdata <= pend_lane ? dq_in[15:8] : dq_in[7:0];
                    pend<=1'b0; busy<=1'b0; st<=ST_IDLE;
                end

                default: st<=ST_INIT;
            endcase

            {SDRAM_nCS,SDRAM_nRAS,SDRAM_nCAS,SDRAM_nWE} <= cmd;
        end
    end
endmodule
```

- [ ] **Step 2: Run the unit test to verify it passes**

Run: `make unit DESIGN=project_obscurus MODULE=sdram_ctrl REV=rev2`
Expected: `PASS` (round-trip, byte-lane isolation, bank isolation all match).

- [ ] **Step 3: Commit**

```bash
git add gateware/rev2/project_obscurus/sdram_ctrl.v
git commit -m "feat: sdram_ctrl byte-addressed controller (req iface, DQM, priority refresh)"
```

> **AS-BUILT (commit `615c94b`, TB reorder `d440237`):** Two deviations from the
> reference above, both verified against the model and unit test:
> 1. **Read wait `dly` 2→3** in the ST_RW read branch — CL=2 is counted from when
>    the model *sees* the registered READ command (one cycle after `cmd=CMD_READ`
>    is set), so the reference sampled one cycle early and latched `z`.
> 2. **`busy` is request-scoped, accept gate is `req && !pend && ready`** (the
>    `!busy` term was removed and refresh no longer drives `busy`). The reference
>    raised `busy` for standalone refreshes and gated accept on `!busy`, which
>    silently dropped any request arriving during a refresh. **This also benefits
>    Task 5:** `op_done` (busy falling edge) now pulses exactly once per real
>    access and never on a refresh, so auto-increment can't spuriously step.
> The unit-TB `wire cmd` continuous assignment was moved below its net
> declarations (Icarus 13.0 rejects use-before-declaration) — no test logic changed.

---

## Task 5: Integrate into `project_obscurus_top.v` (strip demo, add monitor_regs + exp-ROM)

**Files:**
- Modify: `gateware/rev2/project_obscurus/project_obscurus_top.v`

- [ ] **Step 1: Delete the boot-string demo scaffolding**

Remove from `project_obscurus_top.v`:
- the `str_src` ROM block (`localparam STR_LEN ...` through its `initial`),
- the entire inline SDRAM FSM (`localparam CMD_* ...`, the `localparam [4:0] ST_*`, all `sdram_state`/`sdram_delay`/`str_idx_w`/`str_idx_r`/`init_done`/`load_done`/`refresh_cnt` regs, the big `always @(posedge clk ...)` SDRAM case, and the `wire ready = (sdram_state == ST_READY)` line),
- the `mirror[]` array + its `initial`,
- the `str_idx` register and the `always` block that seeks/auto-advances it,
- `mirror_byte` and its use in the read mux.

**Keep:** `clk_div`/`clk`, POR + `nres_*` sync + `rst_n`, heartbeat, the 2-FF bus sync (`nds_d1/d2`, `rw_d1/d2`, `addr_d1/d2`, `data_d1/d2`, `nds_rise`), `wr_data_latch`/`wr_addr_latch`/`wr_rw_latch`, the `scratch[]` loopback, the daisy-chain/passive lines, and the SDRAM_D tristate assigns + `sdram_dq_in` bundle.

- [ ] **Step 2: Instantiate `sdram_ctrl` and wire SDRAM pins**

Replace the deleted FSM's SDRAM pin assignments with controller outputs. Add these wires + instance (keep the existing `SDRAM_D*` tristate assigns, but drive them from `ctrl_dq_out`/`ctrl_dq_oe`):

```verilog
    // ----- SDRAM controller -----
    wire        sdram_req;
    wire        sdram_we;
    wire [25:0] sdram_phys_addr;
    wire [7:0]  sdram_wdata;
    wire [7:0]  sdram_rdata;
    wire        sdram_busy;
    wire        ready;              // SDRAM init complete

    wire [3:0]  ctrl_cmd;
    wire [12:0] ctrl_a;
    wire [15:0] ctrl_dq_out;
    wire        ctrl_dq_oe;

    sdram_ctrl u_sdram (
        .clk(clk), .rst_n(rst_n),
        .req(sdram_req), .we(sdram_we), .phys_addr(sdram_phys_addr),
        .wdata(sdram_wdata), .rdata(sdram_rdata),
        .busy(sdram_busy), .ready(ready),
        .SDRAM_CKE(SDRAM_CKE),
        .SDRAM_nCS(SDRAM_nCS), .SDRAM_nRAS(SDRAM_nRAS),
        .SDRAM_nCAS(SDRAM_nCAS), .SDRAM_nWE(SDRAM_nWE),
        .SDRAM_DQM0(SDRAM_DQM0), .SDRAM_DQM1(SDRAM_DQM1),
        .SDRAM_BA0(SDRAM_BA0), .SDRAM_BA1(SDRAM_BA1),
        .sdram_a(ctrl_a),
        .dq_out(ctrl_dq_out), .dq_oe(ctrl_dq_oe), .dq_in(sdram_dq_in)
    );

    assign SDRAM_CLK = clk;
    assign SDRAM_A0=ctrl_a[0];  assign SDRAM_A1=ctrl_a[1];  assign SDRAM_A2=ctrl_a[2];
    assign SDRAM_A3=ctrl_a[3];  assign SDRAM_A4=ctrl_a[4];  assign SDRAM_A5=ctrl_a[5];
    assign SDRAM_A6=ctrl_a[6];  assign SDRAM_A7=ctrl_a[7];  assign SDRAM_A8=ctrl_a[8];
    assign SDRAM_A9=ctrl_a[9];  assign SDRAM_A10=ctrl_a[10]; assign SDRAM_A11=ctrl_a[11];
    assign SDRAM_A12=ctrl_a[12];
```

Change the 16 `SDRAM_D*` tristate assigns to use `ctrl_dq_oe ? ctrl_dq_out[n] : 1'bZ` (replacing the old `sdram_dq_oe`/`sdram_dq_out`). Keep the `sdram_dq_in` bundle unchanged.

- [ ] **Step 3: Add `monitor_regs` (address/bank/data latches, busy@strobe, auto-inc, STATUS)**

```verilog
    // ----- monitor register port -----
    reg [15:0] m_addr = 16'd0;    // 16-bit offset within bank
    reg [9:0]  m_bank = 10'd0;
    reg [7:0]  m_wdata = 8'd0;
    reg        m_we = 1'b0;
    reg        m_busy = 1'b0;      // STATUS bit7 — latched at strobe
    reg        m_req = 1'b0;       // 1-cycle pulse to sdram_ctrl

    assign sdram_req       = m_req;
    assign sdram_we        = m_we;
    assign sdram_phys_addr = {m_bank, m_addr};
    assign sdram_wdata     = m_wdata;

    // register-write decode (commit on nds_rise, R_nW low)
    wire reg_wr = nds_rise & ~wr_rw_latch;

    // busy falling edge from controller = op complete -> auto-increment
    reg sdram_busy_d;
    always @(posedge clk) sdram_busy_d <= sdram_busy;
    wire op_done = sdram_busy_d & ~sdram_busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_addr<=16'd0; m_bank<=10'd0; m_wdata<=8'd0;
            m_we<=1'b0; m_busy<=1'b0; m_req<=1'b0;
        end else begin
            m_req <= 1'b0;                       // default: pulse
            if (reg_wr) begin
                case (wr_addr_latch)
                    4'h0: m_addr[7:0]  <= wr_data_latch;       // ADDR_LO
                    4'h1: m_addr[15:8] <= wr_data_latch;       // ADDR_HI
                    4'h2: m_bank[7:0]  <= wr_data_latch;       // BANK_LO
                    4'h3: m_bank[9:8]  <= wr_data_latch[1:0];  // BANK_HI
                    4'h4: begin                                // TRIG_RD
                        m_we<=1'b0; m_req<=1'b1; m_busy<=1'b1; // busy @strobe
                    end
                    4'h6: begin                                // DATA write
                        m_wdata<=wr_data_latch; m_we<=1'b1;
                        m_req<=1'b1; m_busy<=1'b1;             // busy @strobe
                    end
                    default: ;
                endcase
            end
            if (op_done) begin
                m_busy <= 1'b0;
                m_addr <= m_addr + 1'b1;          // auto-inc on completion
            end
        end
    end

    wire [7:0] status_byte = {m_busy, ready, 6'b0};  // bit7=busy, bit6=ready
```

- [ ] **Step 4: Update the read mux**

Replace the old read mux with:

```verilog
    reg [7:0] reg_data_out;
    always @(*) begin
        case (apple_addr[3:0])
            4'h5: reg_data_out = status_byte;   // STATUS
            4'h6: reg_data_out = sdram_rdata;   // DATA (RD_HOLD from controller)
            default: reg_data_out = scratch[apple_addr[3:0]];
        endcase
    end
```

(Scratch loopback remains for `$C0C7-$C0CF`; `$C0C0-$C0C4`,`$C0C6` are the port. Note `$C0C5` read = STATUS.)

- [ ] **Step 5: Add the expansion-ROM enable FF + `monitor_mem` + gated drive**

```verilog
    // ----- expansion ROM ($C800-$CFFF) -----
    reg [7:0] monitor_mem [0:2047];
    initial $readmemh("monitor.mem", monitor_mem);

    // enable FF: set on $Cn00 (nI_O_SELECT), clear on $CFFF access
    reg rom_en = 1'b0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rom_en <= 1'b0;
        else begin
            if (~nI_O_SELECT) rom_en <= 1'b1;
            if (~nI_O_STROBE && apple_addr[10:0]==11'h7FF) rom_en <= 1'b0; // $CFFF
        end
    end

    wire [7:0] exp_rom_data = monitor_mem[apple_addr[10:0]];
    wire       exp_read = rom_en & ~nI_O_STROBE & R_nW;
```

- [ ] **Step 6: Update the D bus drive + DATA_OE**

```verilog
    wire device_read = ~nDEVICE_SELECT & R_nW;
    wire rom_read    = ~nI_O_SELECT    & R_nW;

    wire [7:0] d_out = rom_read    ? slot_rom_data :
                       exp_read    ? exp_rom_data  :
                       device_read ? reg_data_out  : 8'h00;
    wire       d_oe  = rom_read | exp_read | device_read;
```

(Keep the 8 `assign Dn = d_oe ? d_out[n] : 1'bZ;` lines.)

For `DATA_OE` — add `rom_en & ~nI_O_STROBE` to `slot_active`, **leave it asserting on both read and write** (it is the U12 '245 buffer OE, must be live both directions):

```verilog
    wire slot_active = ~nDEVICE_SELECT | ~nI_O_SELECT | (rom_en & ~nI_O_STROBE);
    assign DATA_OE = ~slot_active;
```

- [ ] **Step 7: Update GPIO debug block to drop deleted signals**

Remove references to `init_done`, `load_done`, `str_idx`, `mirror_byte`, `device_read`/`d_oe` if those debug taps were removed. Drive the GPIO assigns from still-existing signals (e.g. `ready`, `m_busy`, `rom_en`, `sdram_busy`, bus-event latches). Minimal: keep `GPIO_1=hb_led` and tie unused GPIO to `1'b0`. (GPIO is not routed on this board; correctness of these taps is not load-bearing — they must just compile.)

- [ ] **Step 8: Verify synthesis-level compile (lint via sim build, no bitstream yet)**

Run: `make sim DESIGN=project_obscurus REV=rev2`
Expected: At this point the OLD `project_obscurus_tb.v` will FAIL to compile (it references deleted `dut.mirror`, `dut.str_idx`, etc.). That is expected — Task 6 rewrites it. To check the top module compiles standalone, instead run the unit build which only needs the design sources:

Run: `make unit DESIGN=project_obscurus MODULE=sdram_ctrl REV=rev2`
Expected: still `PASS` (top.v not pulled into unit build, but this confirms design sources still compile together via `$(VERILOG_SRC)`).

- [ ] **Step 9: Commit**

```bash
git add gateware/rev2/project_obscurus/project_obscurus_top.v
git commit -m "feat: integrate sdram_ctrl + monitor_regs + expansion ROM into top"
```

---

## Task 6: Rewrite the integration testbench (`project_obscurus_tb.v`)

**Files:**
- Rewrite: `gateware/rev2/project_obscurus/project_obscurus_tb.v`

- [ ] **Step 1: Write the new integration TB**

```verilog
`timescale 1ns / 1ps
// Integration TB: drives the $C0Cx register port like the 6502 monitor would.
module project_obscurus_tb;
    reg clk100 = 0;
    always #5 clk100 = ~clk100;

    reg [15:0] apple_addr = 16'h0000;
    wire [7:0] apple_d;
    reg [7:0]  apple_d_drive = 8'hZZ;
    reg        apple_d_oe = 1'b0;
    reg nDEVICE_SELECT=1, nI_O_SELECT=1, nI_O_STROBE=1, R_nW=1, PHI0=0, nRES_READ=1;
    assign apple_d = apple_d_oe ? apple_d_drive : 8'hZZ;

    wire SDRAM_CLK, SDRAM_CKE, SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE;
    wire SDRAM_DQM0, SDRAM_DQM1, SDRAM_BA0, SDRAM_BA1;
    wire SDRAM_A0,SDRAM_A1,SDRAM_A2,SDRAM_A3,SDRAM_A4,SDRAM_A5,SDRAM_A6;
    wire SDRAM_A7,SDRAM_A8,SDRAM_A9,SDRAM_A10,SDRAM_A11,SDRAM_A12;
    wire [12:0] sdram_a = {SDRAM_A12,SDRAM_A11,SDRAM_A10,SDRAM_A9,SDRAM_A8,
                           SDRAM_A7,SDRAM_A6,SDRAM_A5,SDRAM_A4,SDRAM_A3,
                           SDRAM_A2,SDRAM_A1,SDRAM_A0};
    wire [3:0] sdram_cmd = {SDRAM_nCS,SDRAM_nRAS,SDRAM_nCAS,SDRAM_nWE};
    wire [15:0] sdram_dq;

    project_obscurus_top dut (
        .CLK_100MHz(clk100),
        .SDRAM_CLK(SDRAM_CLK), .SDRAM_CKE(SDRAM_CKE), .SDRAM_nCS(SDRAM_nCS),
        .SDRAM_nRAS(SDRAM_nRAS), .SDRAM_nCAS(SDRAM_nCAS), .SDRAM_nWE(SDRAM_nWE),
        .SDRAM_DQM0(SDRAM_DQM0), .SDRAM_DQM1(SDRAM_DQM1),
        .SDRAM_BA0(SDRAM_BA0), .SDRAM_BA1(SDRAM_BA1),
        .SDRAM_A0(SDRAM_A0),.SDRAM_A1(SDRAM_A1),.SDRAM_A2(SDRAM_A2),
        .SDRAM_A3(SDRAM_A3),.SDRAM_A4(SDRAM_A4),.SDRAM_A5(SDRAM_A5),
        .SDRAM_A6(SDRAM_A6),.SDRAM_A7(SDRAM_A7),.SDRAM_A8(SDRAM_A8),
        .SDRAM_A9(SDRAM_A9),.SDRAM_A10(SDRAM_A10),.SDRAM_A11(SDRAM_A11),
        .SDRAM_A12(SDRAM_A12),
        .SDRAM_D0(sdram_dq[0]),.SDRAM_D1(sdram_dq[1]),.SDRAM_D2(sdram_dq[2]),
        .SDRAM_D3(sdram_dq[3]),.SDRAM_D4(sdram_dq[4]),.SDRAM_D5(sdram_dq[5]),
        .SDRAM_D6(sdram_dq[6]),.SDRAM_D7(sdram_dq[7]),.SDRAM_D8(sdram_dq[8]),
        .SDRAM_D9(sdram_dq[9]),.SDRAM_D10(sdram_dq[10]),.SDRAM_D11(sdram_dq[11]),
        .SDRAM_D12(sdram_dq[12]),.SDRAM_D13(sdram_dq[13]),.SDRAM_D14(sdram_dq[14]),
        .SDRAM_D15(sdram_dq[15]),
        .A0(apple_addr[0]),.A1(apple_addr[1]),.A2(apple_addr[2]),.A3(apple_addr[3]),
        .A4(apple_addr[4]),.A5(apple_addr[5]),.A6(apple_addr[6]),.A7(apple_addr[7]),
        .A8(apple_addr[8]),.A9(apple_addr[9]),.A10(apple_addr[10]),.A11(apple_addr[11]),
        .A12(apple_addr[12]),.A13(apple_addr[13]),.A14(apple_addr[14]),.A15(apple_addr[15]),
        .D0(apple_d[0]),.D1(apple_d[1]),.D2(apple_d[2]),.D3(apple_d[3]),
        .D4(apple_d[4]),.D5(apple_d[5]),.D6(apple_d[6]),.D7(apple_d[7]),
        .PHI0(PHI0),.PHI1(~PHI0),.sig_7M(1'b0),.Q3(1'b0),.uSync(1'b0),
        .R_nW(R_nW),.nDEVICE_SELECT(nDEVICE_SELECT),.nI_O_SELECT(nI_O_SELECT),
        .nI_O_STROBE(nI_O_STROBE),.DMA_OUT(1'b1),.INT_OUT(1'b1),.RDY(1'b1),
        .nRES_READ(nRES_READ),
        .nIRQ(),.nNMI(),.nINH(),.nDMA(),.nRES(),.DMA_IN(),.INT_IN(),.DATA_OE(),
        .GPIO_1(),.GPIO_2(),.GPIO_3(),.GPIO_4(),.GPIO_5(),.GPIO_6(),.GPIO_7(),
        .GPIO_8(),.GPIO_9(),.GPIO_10(),.GPIO_11(),.GPIO_12(),.GPIO_13(),.GPIO_14(),
        .GPIO_15(),.GPIO_16(),.GPIO_17(),.GPIO_18(),.GPIO_19(),.GPIO_20()
    );

    sdram_model model (
        .clk(SDRAM_CLK), .cmd(sdram_cmd), .ba({SDRAM_BA1,SDRAM_BA0}),
        .a(sdram_a), .dqm0(SDRAM_DQM0), .dqm1(SDRAM_DQM1), .dq(sdram_dq)
    );

    integer errors = 0;
    reg [7:0] tmp;

    // ---- bus tasks (model a 6502 abs read/write to $C0Cx) ----
    task wr_reg(input [3:0] r, input [7:0] d);
        begin
            apple_addr = {8'hC0, 4'hC, r}; R_nW=1'b0;
            apple_d_drive=d; apple_d_oe=1'b1; nDEVICE_SELECT=1'b0;
            #300; nDEVICE_SELECT=1'b1; apple_d_oe=1'b0; #200;
        end
    endtask
    task rd_reg(input [3:0] r, output [7:0] d);
        begin
            apple_addr = {8'hC0, 4'hC, r}; R_nW=1'b1; apple_d_oe=1'b0;
            nDEVICE_SELECT=1'b0; #300; d=apple_d; nDEVICE_SELECT=1'b1; #200;
        end
    endtask
    // poll STATUS bit7 (busy) until clear
    task poll_busy;
        begin : pb
            integer guard; guard=0;
            rd_reg(4'h5, tmp);
            while (tmp[7]) begin
                rd_reg(4'h5, tmp);
                guard=guard+1; if (guard>10000) begin
                    $display("FAIL poll_busy stuck"); errors=errors+1; disable pb;
                end
            end
        end
    endtask

    task set_addr(input [15:0] a); begin
        wr_reg(4'h0, a[7:0]); wr_reg(4'h1, a[15:8]); end
    endtask
    task set_bank(input [9:0] b); begin
        wr_reg(4'h2, b[7:0]); wr_reg(4'h3, {6'd0,b[9:8]}); end
    endtask
    task sdram_write(input [9:0] bank, input [15:0] a, input [7:0] d); begin
        set_bank(bank); set_addr(a); wr_reg(4'h6, d); poll_busy; end
    endtask
    task sdram_read(input [9:0] bank, input [15:0] a, output [7:0] d); begin
        set_bank(bank); set_addr(a); wr_reg(4'h4, 8'h00); poll_busy;
        rd_reg(4'h6, d); end
    endtask

    initial begin
        $dumpfile("project_obscurus_tb.vcd"); $dumpvars(0, project_obscurus_tb);
        nRES_READ=1'b0; #1000; nRES_READ=1'b1;
        wait (dut.ready);
        $display("[%0t] ready", $time);

        // 1. W-then-R round trip
        sdram_write(10'd0, 16'h0000, 8'hAA);
        sdram_read (10'd0, 16'h0000, tmp);
        if (tmp!==8'hAA) begin errors=errors+1; $display("FAIL rt %02X",tmp); end

        // 2. bank isolation
        sdram_write(10'd1, 16'h0000, 8'h5A);
        sdram_read (10'd0, 16'h0000, tmp);
        if (tmp!==8'hAA) begin errors=errors+1; $display("FAIL bank0 %02X",tmp); end
        sdram_read (10'd1, 16'h0000, tmp);
        if (tmp!==8'h5A) begin errors=errors+1; $display("FAIL bank1 %02X",tmp); end

        // 3. byte-lane isolation
        sdram_write(10'd0, 16'h0010, 8'h11);
        sdram_write(10'd0, 16'h0011, 8'h22);
        sdram_read (10'd0, 16'h0010, tmp);
        if (tmp!==8'h11) begin errors=errors+1; $display("FAIL lane lo %02X",tmp); end
        sdram_read (10'd0, 16'h0011, tmp);
        if (tmp!==8'h22) begin errors=errors+1; $display("FAIL lane hi %02X",tmp); end

        // 4. busy asserts at strobe (sample STATUS right after a write strobe)
        set_bank(10'd0); set_addr(16'h0100);
        wr_reg(4'h6, 8'h7E);          // strobe write
        rd_reg(4'h5, tmp);            // first poll
        if (!tmp[7]) begin errors=errors+1; $display("FAIL busy-at-strobe"); end
        poll_busy;

        // 5. auto-increment + D dump: write 0x20..0x2F to addr 0x0200.., read back
        for (tmp=0; tmp<16; tmp=tmp+1)
            sdram_write(10'd0, 16'h0200 + tmp, 8'h20 + tmp);
        // dump: set addr once, then strobe/poll/read with auto-inc
        set_bank(10'd0); set_addr(16'h0200);
        begin : dump
            integer i; reg [7:0] v;
            for (i=0; i<16; i=i+1) begin
                wr_reg(4'h4, 8'h00); poll_busy; rd_reg(4'h6, v);
                if (v !== 8'h20 + i) begin
                    errors=errors+1; $display("FAIL dump[%0d]=%02X",i,v); end
            end
        end

        // 6. read with NO preceding strobe must NOT advance addr (no double-step)
        //    addr is now 0x0210 (auto-inc'd past dump). A bare DATA read returns
        //    RD_HOLD and must leave addr unchanged.
        rd_reg(4'h6, tmp);            // no strobe
        set_addr(16'h0200);           // reset addr, re-read first dump byte
        wr_reg(4'h4, 8'h00); poll_busy; rd_reg(4'h6, tmp);
        if (tmp !== 8'h20) begin errors=errors+1; $display("FAIL no-strobe stepped"); end

        // 7. $CFFF byte unused
        if (dut.monitor_mem[11'h7FF] !== 8'h00)
            begin errors=errors+1; $display("FAIL $CFFF not 00"); end

        if (errors==0) $display("PASS"); else $display("FAIL: %0d errors", errors);
        $finish;
    end
    initial begin #20_000_000; $display("TIMEOUT"); $finish; end
endmodule
```

- [ ] **Step 2: Provide a placeholder `monitor.mem` so the integration TB can `$readmemh`**

The top module does `$readmemh("monitor.mem", ...)`. Until Task 8 builds the real one, create a zero-filled stub so sim runs:

Run:
```bash
python3 -c "open('gateware/rev2/project_obscurus/monitor.mem','w').write('00\n'*2048)"
```

- [ ] **Step 3: Run the integration sim**

Run: `make sim DESIGN=project_obscurus REV=rev2`
Expected: `PASS` (all 7 scenario groups). If any FAIL line prints, fix the RTL in `project_obscurus_top.v` / `sdram_ctrl.v` before proceeding — this is the gate before any bitstream.

- [ ] **Step 4: Commit**

```bash
git add gateware/rev2/project_obscurus/project_obscurus_tb.v gateware/rev2/project_obscurus/monitor.mem
git commit -m "test: integration TB for register-port protocol (poll_busy, auto-inc, banks)"
```

> **AS-BUILT (Tasks 5+6 combined, commit `4c2bc7c`):** Three deviations, all
> verified by the green integration sim (7/7 scenarios):
> 1. **Sticky `m_busy`** — set at strobe, cleared by the FIRST STATUS read after
>    completion (`op_complete` latches on `op_done`; `status_rd & (op_complete |
>    op_done)` clears `m_busy`). The plan's edge-only clear could be raced past
>    when the SDRAM op finishes faster than the 6502 issues its next poll; sticky
>    busy guarantees the first poll always sees busy=1 (the spec's requirement).
>    Auto-increment still fires only on `op_done`, so scenario 6 holds.
> 2. **TB fill loop uses a local `integer j`** (not the global `tmp`) — the
>    plan's loop reused `tmp`, which `poll_busy` (inside `sdram_write`) overwrites
>    with the STATUS byte, so the loop never reached 16. Bug in the plan's TB;
>    fix preserves the scenario (write 16 / read 16 back via auto-inc).
> 3. **Makefile guard** `ifneq ($(wildcard $(OBSCURUS_MON_SRC)),)` around the
>    `monitor.mem: monitor.S` rule, so the committed zero-stub `monitor.mem` isn't
>    derived from a not-yet-existing `monitor.S`. Self-disables once Task 8 lands.
> Scenario-7 `$CFFF` check accepts `$00` (stub) or `$FF` (rom2mem fill).

---

## Task 7: `$C400` stub (`slot_rom.S`)

**Files:**
- Rewrite: `gateware/rev2/project_obscurus/slot_rom.S`

- [ ] **Step 1: Write the stub**

```
* =============================================================================
* slot_rom.S — project_obscurus $C400 stub (Merlin32)
* =============================================================================
* PR#4 does JSR $C400. Restore the BASIC output vector to COUT1 (so the
* monitor's COUT reaches the screen instead of recursing through $C400), then
* JMP into the monitor at $C800. The JSR return address stays on the stack;
* the monitor's Q command RTSes back to BASIC.
* =============================================================================
            TYP   $06
            DSK   slot_rom.bin
            ORG   $C400

CSWL   =    $36
CSWH   =    $37
COUT1  =    $FDF0
MONITOR =   $C800

ENTRY       LDA   #<COUT1
            STA   CSWL
            LDA   #>COUT1
            STA   CSWH
            JMP   MONITOR
```

- [ ] **Step 2: Assemble and verify the first byte**

Run: `make assemble ASM_SRC=gateware/rev2/project_obscurus/slot_rom.S`
Then: `xxd gateware/rev2/project_obscurus/slot_rom.bin | head -1`
Expected: first byte `a9` (LDA #imm) — the stub entry.

- [ ] **Step 3: Commit**

```bash
git add gateware/rev2/project_obscurus/slot_rom.S
git commit -m "feat: slot_rom stub restores CSW + JMP \$C800 monitor"
```

---

## Task 8: Monitor program (`monitor.S`)

**Files:**
- Create: `gateware/rev2/project_obscurus/monitor.S`

- [ ] **Step 1: Write the monitor**

```
* =============================================================================
* monitor.S — project_obscurus SDRAM monitor (Merlin32), ORG $C800
* =============================================================================
* Commands (all numbers hex):
*   R aaaa        read one byte:  prints "aaaa: bb"
*   W aaaa bb     write one byte
*   B bbb         set bank ($000-$3FF)
*   D aaaa        dump 16 bytes (hex + ASCII)
*   Q             return to BASIC (RTS on PR#'s JSR return addr)
*
* Scratch lives in the $0300 page; ZP $06/$07 used only as the PRINTSTR
* indirect pointer (indirect addressing forces ZP). GETLN returns chars with
* the high bit set, so every input char is AND #$7F before use.
* =============================================================================
            TYP   $06
            DSK   monitor.bin
            ORG   $C800

* ---- register port ----
RADDRLO =   $C0C0
RADDRHI =   $C0C1
RBANKLO =   $C0C2
RBANKHI =   $C0C3
RTRIG   =   $C0C4
RSTATUS =   $C0C5
RDATA   =   $C0C6

* ---- $0300 scratch ----
ADDR    =   $0300        ; 2 bytes
DATB    =   $0302
BANK    =   $0303        ; 2 bytes
DBUF    =   $0305        ; 16-byte dump buffer ($0305-$0314)
STRPTR  =   $06          ; ZP indirect ptr (PRINTSTR only)

* ---- monitor ROM entries ----
GETLN   =   $FD6A
COUT    =   $FDED
CROUT   =   $FD8E
PRBYTE  =   $FDDA
PROMPTC =   $33
INBUF   =   $0200

* =============================================================================
MONITOR     JSR   READYW          ; wait SDRAM ready (BIT/BVC)
            LDA   #<BANNER
            STA   STRPTR
            LDA   #>BANNER
            STA   STRPTR+1
            JSR   PRINTSTR
            JSR   CROUT
            LDA   #0
            STA   BANK
            STA   BANK+1

PROMPT      LDA   #"*"
            ORA   #$80
            STA   PROMPTC
            JSR   GETLN           ; line -> INBUF, terminated $8D
            LDY   #0
            JSR   SKIPSP
            LDA   INBUF,Y
            INY
            AND   #$7F
            CMP   #"R"
            BNE   :nR
            JMP   DOREAD
:nR         CMP   #"W"
            BNE   :nW
            JMP   DOWRITE
:nW         CMP   #"B"
            BNE   :nB
            JMP   DOBANK
:nB         CMP   #"D"
            BNE   :nD
            JMP   DODUMP
:nD         CMP   #"Q"
            BNE   :nErr
            RTS                    ; back to BASIC
:nErr       JSR   PRERR
            JMP   PROMPT

* ---- handlers ----
DOREAD      JSR   SKIPSP
            JSR   PH16
            JSR   RDBYTE
            PHA
            JSR   PRADDR
            PLA
            JSR   PRBYTE
            JSR   CROUT
            JMP   PROMPT

DOWRITE     JSR   SKIPSP
            JSR   PH16
            JSR   SKIPSP
            JSR   PH8
            LDA   DATB
            JSR   WRBYTE
            JSR   CROUT
            JMP   PROMPT

DOBANK      JSR   SKIPSP
            JSR   PH16
            LDA   ADDR
            STA   BANK
            LDA   ADDR+1
            AND   #$03
            STA   BANK+1
            JMP   PROMPT

DODUMP      JSR   SKIPSP
            JSR   PH16
            JSR   SETADDR          ; first access sets addr; then auto-inc
            LDX   #0
:rdl        STA   RTRIG
            JSR   POLLB
            LDA   RDATA
            STA   DBUF,X
            INX
            CPX   #16
            BNE   :rdl
            JSR   PRADDR
            LDX   #0
:hex        LDA   DBUF,X
            JSR   PRBYTE
            LDA   #" "
            ORA   #$80
            JSR   COUT
            INX
            CPX   #16
            BNE   :hex
            LDA   #" "
            ORA   #$80
            JSR   COUT
            LDX   #0
:asc        LDA   DBUF,X
            AND   #$7F
            CMP   #$20
            BCC   :dot
            CMP   #$7F
            BCS   :dot
            BCC   :put
:dot        LDA   #"."
:put        ORA   #$80
            JSR   COUT
            INX
            CPX   #16
            BNE   :asc
            JSR   CROUT
            JMP   PROMPT

* ---- SDRAM access helpers ----
SETADDR     LDA   ADDR
            STA   RADDRLO
            LDA   ADDR+1
            STA   RADDRHI
            LDA   BANK
            STA   RBANKLO
            LDA   BANK+1
            STA   RBANKHI
            RTS

POLLB       LDA   RSTATUS
            BMI   POLLB            ; bit7=busy -> N -> loop
            RTS

READYW      BIT   RSTATUS
            BVC   READYW           ; bit6=ready -> V; wait until set
            RTS

RDBYTE      JSR   SETADDR
            STA   RTRIG            ; any value triggers a read
            JSR   POLLB
            LDA   RDATA
            RTS

WRBYTE      PHA
            JSR   SETADDR
            PLA
            STA   RDATA            ; latch + trigger write
            JSR   POLLB
            RTS

* ---- output helpers ----
PRADDR      LDA   ADDR+1
            JSR   PRBYTE
            LDA   ADDR
            JSR   PRBYTE
            LDA   #":"
            ORA   #$80
            JSR   COUT
            LDA   #" "
            ORA   #$80
            JSR   COUT
            RTS

PRINTSTR    LDY   #0
:l          LDA   (STRPTR),Y
            BEQ   :x
            ORA   #$80
            JSR   COUT
            INY
            BNE   :l
:x          RTS

PRERR       LDA   #"?"
            ORA   #$80
            JSR   COUT
            JSR   CROUT
            RTS

* ---- input helpers ----
SKIPSP      LDA   INBUF,Y
            AND   #$7F
            CMP   #" "
            BNE   :done
            INY
            BNE   SKIPSP
:done       RTS

* PH16: parse hex into ADDR (16-bit). Stops at first non-hex char.
PH16        LDA   #0
            STA   ADDR
            STA   ADDR+1
:l          LDA   INBUF,Y
            AND   #$7F
            JSR   HEXNIB
            BCC   :done
            PHA
            ASL   ADDR
            ROL   ADDR+1
            ASL   ADDR
            ROL   ADDR+1
            ASL   ADDR
            ROL   ADDR+1
            ASL   ADDR
            ROL   ADDR+1
            PLA
            ORA   ADDR
            STA   ADDR
            INY
            BNE   :l
:done       RTS

* PH8: parse up to 2 hex into DATB.
PH8         LDA   #0
            STA   DATB
:l          LDA   INBUF,Y
            AND   #$7F
            JSR   HEXNIB
            BCC   :done
            ASL   DATB
            ASL   DATB
            ASL   DATB
            ASL   DATB
            ORA   DATB
            STA   DATB
            INY
            BNE   :l
:done       RTS

* HEXNIB: A=ascii char (no high bit). Returns C=1 + A=nibble if hex, else C=0.
HEXNIB      CMP   #"0"
            BCC   :bad
            CMP   #"9"+1
            BCC   :dig
            CMP   #"A"
            BCC   :bad
            CMP   #"G"
            BCS   :bad
            SEC
            SBC   #"A"
            CLC
            ADC   #10
            SEC
            RTS
:dig        SEC
            SBC   #"0"
            SEC
            RTS
:bad        CLC
            RTS

BANNER      ASC   "OBSCURUS SDRAM MONITOR"
            DFB   $00
```

- [ ] **Step 2: Assemble and verify**

Run: `make assemble ASM_SRC=gateware/rev2/project_obscurus/monitor.S`
Then: `xxd gateware/rev2/project_obscurus/monitor.bin | head -1`
Expected: first byte `20` (JSR — the `JSR READYW` at MONITOR entry). No Merlin32 errors.

Also confirm size leaves `$CFFF` free:
Run: `wc -c < gateware/rev2/project_obscurus/monitor.bin`
Expected: well under 2046 bytes.

- [ ] **Step 3: Build the real `monitor.mem` and confirm `$CFFF`=00**

Run: `make sim DESIGN=project_obscurus REV=rev2` (the `monitor.mem` recipe regenerates it from `monitor.S`; sim's `$CFFF` assert covers the unused-byte check).
Expected: `PASS` — including scenario 7 (`$CFFF`=00) now that `monitor.mem` is the real build, not the zero stub. (The monitor occupies only the low bytes; the rest stays `$FF`/`$00` — if the `$CFFF` byte is `$FF` not `$00`, change the integration TB scenario-7 check to `!== 8'hFF` OR pad `monitor.S` to force `$CFFF`=$00; prefer asserting the byte is not live code, i.e. accept `$FF`.)

> **Implementer note:** `rom2mem.py` fills unused space with `$FF`, so `monitor.mem[$7FF]` will be `$FF`, not `$00`. Update the Task-6 scenario-7 assert to `!== 8'hFF` (unused/fill is fine — the requirement is only that `$CFFF` is never live monitor code, which it isn't since the monitor is far smaller than 2 KB). Make this one-line TB edit and re-run `make sim`.

- [ ] **Step 4: Commit**

```bash
git add gateware/rev2/project_obscurus/monitor.S gateware/rev2/project_obscurus/monitor.mem gateware/rev2/project_obscurus/project_obscurus_tb.v
git commit -m "feat: SDRAM monitor program (R/W/B/D/Q, hex parser, high-bit mask)"
```

---

## Task 9: Bitstream build + docs + memory

**Files:**
- Modify: `gateware/rev2/project_obscurus/README.md`
- Memory: `~/.claude/projects/.../memory/` (project note + index)

- [ ] **Step 1: Full clean build (no flashing — user flashes manually)**

Run: `make clean && make DESIGN=project_obscurus REV=rev2`
Expected: `build/project_obscurus.bit` produced, no errors. (Per CLAUDE.md: do NOT run `prog-flash` — stop here and report; the user flashes.)

- [ ] **Step 2: Update the README memory map**

Rewrite `gateware/rev2/project_obscurus/README.md` to document: the `$C0Cx` register port table (ADDR_LO/HI, BANK_LO/HI, TRIG_RD, STATUS, DATA), STATUS bit layout (bit7 busy / bit6 ready), the busy-poll handshake, 1024 byte-packed banks, the `PR#4` launch + `Q` exit, the R/W/B/D commands, and the build commands. Remove the stale hello-world description.

- [ ] **Step 3: Bench verification checklist (user runs on hardware)**

After the user flashes, verify via the keyboard + `obs-screenshot`:
- `PR#4` → monitor banner + `*` prompt appears.
- `W 0000 AA` then `R 0000` → prints `0000: AA`.
- `B 1`, `R 0000` → prints a value ≠ `AA` (bank isolation).
- `B 0`, `D 0000` → 16-byte hex + ASCII dump renders, first byte `AA`.
- `Q` → returns to BASIC `]` prompt cleanly.
Capture a screenshot via the `obs-screenshot` skill.

- [ ] **Step 4: Update project memory**

Create a memory file noting the monitor design is implemented (register port map, byte-packed 1024 banks, `sdram_ctrl.v` isolated module, monitor at `$C800`), and add the index line to `MEMORY.md`. Mark bench-verification status.

- [ ] **Step 5: Commit**

```bash
git add gateware/rev2/project_obscurus/README.md
git commit -m "docs: project_obscurus SDRAM monitor memory map + usage"
```

---

## Self-Review Notes (addressed)

- **Spec coverage:** register port (Task 5), byte-packed 1024 banks + geometry (Task 4), priority refresh (Task 4), DQM read/write (Task 4 + model Task 2), expansion-ROM FF + `$CFFF` + DATA_OE both-directions (Task 5), busy@strobe + auto-inc on completion (Task 5), `$0300` scratch + GETLN high-bit mask (Task 8), stub Q-exit/stack (Task 7 + monitor `RTS`), faithful TB rewrite with 4 traps + poll_busy (Tasks 2/3/6), rom2mem `0xC000` (Task 1), strip demo scaffolding (Task 5 step 1).
- **Type/name consistency:** request interface `req/we/phys_addr/wdata/rdata/busy/ready` identical across `sdram_ctrl.v`, `sdram_ctrl_tb.v`, and top instance. Register offsets `$C0C0-$C0C6` identical across top read/write mux, integration TB tasks, and `monitor.S` equates. STATUS bit7=busy/bit6=ready consistent in top (`status_byte`), TB (`tmp[7]`), and monitor (`BMI`/`BVC`).
- **Known follow-up baked into a step:** `$CFFF` fill is `$FF` (rom2mem), so the Task-6 scenario-7 assert is corrected to `!== 8'hFF` in Task 8 step 3.
