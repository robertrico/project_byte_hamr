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
    // NOTE: `cmd` continuous assignment moved below the wire declarations it
    // references — Icarus 13.0-devel rejects use-before-declaration of explicit
    // nets in a continuous assignment ("declaration after use"). Test semantics
    // are unchanged.
    wire SDRAM_CLK, SDRAM_CKE, SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE;
    wire SDRAM_DQM0, SDRAM_DQM1, SDRAM_BA0, SDRAM_BA1;
    wire [12:0] sdram_a;
    wire [15:0] dq_out;
    wire        dq_oe;
    wire [15:0] dq;
    wire [3:0]  cmd = {SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE};
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
