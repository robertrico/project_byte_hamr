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
