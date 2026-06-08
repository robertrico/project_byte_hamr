`timescale 1ns/1ps
module coproc_tb;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0, ready=0;
    wire req, we; wire [25:0] phys_addr; wire [7:0] wdata;
    reg [7:0] rdata=0; reg busy=0;
    reg  [12:0] laddr=0; reg [7:0] ldata_in=0; reg lwr=0;
    wire [7:0]  ldata_out;
    reg  [7:0]  count_in=0; reg count_wr=0;
    integer errors=0;

    coproc dut(.clk(clk), .rst_n(rst_n), .ready(ready),
        .req(req), .we(we), .phys_addr(phys_addr), .wdata(wdata),
        .busy(busy), .rdata(rdata),
        .laddr(laddr), .ldata_in(ldata_in), .lwr(lwr), .ldata_out(ldata_out),
        .count_in(count_in), .count_wr(count_wr));

    task pbwrite(input [12:0] a, input [7:0] d); begin
        @(posedge clk); laddr=a; ldata_in=d; lwr=1; @(posedge clk); lwr=0; @(posedge clk);
    end endtask
    task pbread(input [12:0] a, output [7:0] d); begin
        @(posedge clk); laddr=a; @(posedge clk); @(posedge clk); d=ldata_out;
    end endtask

    reg [7:0] v;
    initial begin
        rst_n=0; #50; rst_n=1; ready=1;
        #200;
        pbwrite(13'h0300, 8'h99);
        pbread (13'h0300, v);
        if (v!==8'h99) begin errors=errors+1; $display("FAIL readback %02X",v); end
        pbwrite(13'h1000, 8'hEE);
        pbread (13'h1000, v);
        if (v===8'hEE) begin errors=errors+1; $display("FAIL kernel $1000 was written"); end
        @(posedge clk); count_in=8'h02; count_wr=1; @(posedge clk); count_wr=0;
        if (errors==0) $display("PASS coproc-c1 port B"); else $display("FAIL coproc-c1 %0d",errors);
        $finish;
    end
endmodule
