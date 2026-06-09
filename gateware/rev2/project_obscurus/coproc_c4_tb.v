`timescale 1ns/1ps
module coproc_c4_tb;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0;
    reg        ring_wr=0, collect_wr=0; reg [1:0] host_slot=0;
    reg        snap_busy=0;
    wire [3:0] done_m, active_m, tmo_m, callreq_m;
    integer errors=0;

    coproc dut(.clk(clk), .rst_n(rst_n),
        .ready(1'b1), .req(), .we(), .phys_addr(), .wdata(), .busy(1'b0), .rdata(8'd0),
        .laddr(13'd0), .ldata_in(8'd0), .lwr(1'b0), .ldata_out(),
        .count_in(8'd0), .count_wr(1'b0),
        .c4_ring_wr(ring_wr), .c4_collect_wr(collect_wr), .c4_host_slot(host_slot),
        .snapshot_busy(snap_busy),
        .c4_done(done_m), .c4_active(active_m), .c4_timedout(tmo_m), .c4_callreq(callreq_m));

    task ringslot(input [1:0] s); begin host_slot=s; @(posedge clk); ring_wr=1; @(posedge clk); ring_wr=0; @(posedge clk); end endtask
    task collectslot(input [1:0] s); begin host_slot=s; @(posedge clk); collect_wr=1; @(posedge clk); collect_wr=0; @(posedge clk); end endtask
    // Drive the kernel-side Arlet bus by forcing the dut's real internal nets.
    // The Arlet CPU also drives AB/WE/DO, so force overrides it (no multi-driver
    // fight); WE is held at 0 between writes to suppress any CPU-issued strobes.
    task kwrite(input [15:0] a, input [7:0] d); begin
        force dut.AB=a; force dut.DO=d; force dut.WE=1'b1; force dut.rdy=1'b1;
        @(posedge clk);
        force dut.WE=1'b0; force dut.AB=16'd0;
        @(posedge clk);
    end endtask

    initial begin
        rst_n=0; #40; rst_n=1; #20;
        // Suppress CPU bus writes for the duration of the test; rdy held high so
        // the kernel-side register strobes commit.
        force dut.WE=1'b0; force dut.rdy=1'b1; force dut.AB=16'd0;
        ringslot(2'd2);
        if (callreq_m!==4'b0100) begin errors=errors+1; $display("FAIL callreq after ring %b",callreq_m); end
        if (active_m[2]!==1'b1)  begin errors=errors+1; $display("FAIL active after ring"); end
        kwrite(16'hE016, 8'd2);
        if (callreq_m!==4'b0000) begin errors=errors+1; $display("FAIL callreq after ack %b",callreq_m); end
        kwrite(16'hE017, 8'd2);
        if (active_m[2]!==1'b1) begin errors=errors+1; $display("FAIL active after runset"); end
        kwrite(16'hE019, 8'd2); kwrite(16'hE018, 8'd2);
        if (done_m!==4'b0100) begin errors=errors+1; $display("FAIL done after doneset %b",done_m); end
        collectslot(2'd2);
        if (done_m[2]!==1'b0 || active_m[2]!==1'b0) begin errors=errors+1; $display("FAIL not free after collect"); end
        kwrite(16'hE01A, 8'd1);
        if (tmo_m!==4'b0010) begin errors=errors+1; $display("FAIL tmo after tmoset %b",tmo_m); end
        collectslot(2'd1);
        if (tmo_m[1]!==1'b0) begin errors=errors+1; $display("FAIL tmo not cleared by collect"); end
        if (errors==0) $display("PASS coproc_c4 control-plane (per-bit flops, ring/ack/run/done/collect)");
        else $display("FAIL coproc_c4 %0d",errors);
        $finish;
    end
endmodule
