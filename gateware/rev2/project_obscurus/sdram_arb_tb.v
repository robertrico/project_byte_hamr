`timescale 1ns/1ps
module sdram_arb_tb;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0;

    wire        req, we; wire [25:0] phys_addr; wire [7:0] wdata;
    reg  [7:0]  rdata; reg busy=0;

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

    // model sdram_ctrl: on req pulse (while !busy) latch addr, busy 4 cyc, then
    // present rdata = addr[7:0]^8'hA5 on the falling cycle.
    reg [25:0] mlat; reg [2:0] mcnt=0;
    always @(posedge clk) begin
        if (req && !busy) begin mlat<=phys_addr; busy<=1; mcnt<=3'd4; end
        else if (busy) begin
            if (mcnt>1) mcnt<=mcnt-1;
            else begin busy<=0; rdata<=mlat[7:0]^8'hA5; end
        end
    end

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
        c0_we=1; c0_addr=26'h000_0010; c0_wdata=8'h11; c0_req=1; @(posedge clk); c0_req=0;
        wait_done(0);
        c1_we=0; c1_addr=26'h000_0003; c1_req=1; @(posedge clk); c1_req=0;
        wait_done(1);
        if (c1_rdata!==(8'h03^8'hA5)) begin errors=errors+1; $display("FAIL c1_rdata %02X",c1_rdata); end
        c0_we=1; c0_addr=26'h000_0020; c0_wdata=8'h22; c0_req=1;
        c1_we=1; c1_addr=26'h000_0021; c1_wdata=8'h33; c1_req=1;
        @(posedge clk); c0_req=0; c1_req=0;
        wait_done(0); wait_done(1);
        if (errors==0) $display("PASS sdram_arb"); else $display("FAIL sdram_arb %0d",errors);
        $finish;
    end
endmodule
