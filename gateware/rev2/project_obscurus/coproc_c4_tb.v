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
    // Collision task: asserts host ring_wr on the SAME posedge that the kernel
    // callack ($E016) fires, then captures the result before cleanup.
    //
    // iverilog -g2005 quirk: AB/WE/DO are output regs of Arlet's combinational
    // always@* blocks.  Forces on these wires take effect ONE posedge after they
    // are applied (the CPU's always@* overrides the force within the same Active
    // region, then the force wins stably on the NEXT posedge).  The cleanup force
    // applied between p2 and p3 therefore only takes effect at p4; at p3 the
    // callack strobe fires AGAIN (is_callack=1 still) with ring_wr=0, which would
    // clear call_req[s].  We snap callreq_m IMMEDIATELY after p2 — before p3's
    // spurious clear — to witness the collision result.
    task collide_ring_ack(input [1:0] s);
        reg [7:0] do_val;
        reg [3:0] snap;
    begin
        do_val = {6'b0, s};
        host_slot = s;
        force dut.AB=16'hE016; force dut.DO=do_val; force dut.WE=1'b1; force dut.rdy=1'b1;
        @(posedge clk);         // p1: old forces in effect (is_callack=0), ring=0 → no change
        ring_wr = 1;
        @(posedge clk);         // p2: new forces in effect (is_callack=1), ring=1 → COLLISION
        snap = callreq_m;       // capture: set-wins keeps bit s=1; clear-wins leaves it 0
        ring_wr = 0; force dut.WE=1'b0; force dut.AB=16'd0;
        @(posedge clk);         // p3: cleanup cycle (lingering callack clears bit s back to 0)
        if (snap[s] !== 1'b1) begin
            errors = errors + 1;
            $display("FAIL set-wins: ring+ack collision cleared bit %0d (clear won)", s);
        end else
            $display("PASS set-wins on same-cycle ring+ack collision");
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
        // --- same-cycle set+clear collision: set must win (THE bug case) ---
        // call_req[3]=0 here (all prior tests cleared it). ring+callack same edge → must be 1.
        // Pass/fail + error counting happen inside collide_ring_ack (snapped at p2).
        // The task's p3 cleanup cycle leaves call_req[3]=0, so no kwrite needed after.
        collide_ring_ack(2'd3);

        // --- multi-slot independence: clear of one slot must not disturb another ---
        ringslot(2'd0); ringslot(2'd2);    // call_req = 4'b0101
        kwrite(16'hE016, 8'd0);            // callack slot 0 → call_req should be 4'b0100
        if (callreq_m!==4'b0100) begin errors=errors+1; $display("FAIL multi-slot independence: callreq=%b want 4'b0100",callreq_m); end
        else $display("PASS multi-slot independence: clear slot 0 leaves slot 2 intact");
        kwrite(16'hE016, 8'd2);  // clean up

        if (errors==0) $display("PASS coproc_c4 control-plane (per-bit flops, ring/ack/run/done/collect)");
        else $display("FAIL coproc_c4 %0d",errors);
        $finish;
    end
endmodule
