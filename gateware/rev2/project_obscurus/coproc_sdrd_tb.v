`timescale 1ns/1ps
// =============================================================================
// coproc_sdrd_tb.v — SDRAM READ window ($E004-08) unit test.
//
// Pins the ONE genuinely new timing (spec R2): sread<=rdata latched on `done`.
// A SLOW stub SDRAM models the real arbiter faithfully: on a read it drives
// rdata to a DECOY ($ED) for the whole busy window and switches to the REAL
// byte (= addr[7:0]^$5A) ONLY on the same edge busy falls (exactly what
// sdram_arb does: c1_rdata<=rdata and servicing<=0 on the op_complete edge).
// If the coproc latch is off by one it grabs the decoy -> FAIL.
//
// Bus drive: force/release idiom (mirrors coproc_c4_tb.v). rdy is NOT forced —
// the FSM owns it, so the $E007 trigger's registered rdy<=0 stall is genuinely
// exercised (not bypassed). Register sets ($E004/05/06) capture under the FSM's
// own rdy=1 in ST_RUN.
// =============================================================================
module coproc_sdrd_tb;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0;

    wire        req, we;
    wire [25:0] phys_addr;
    wire [7:0]  wdata;

    // ---- slow stub SDRAM (models sdram_arb c1 port) ----
    localparam [7:0] LAT   = 8'd8;     // >=8 cycles busy before data valid
    localparam [7:0] DECOY = 8'hED;
    reg        sbusy=0;
    reg [7:0]  scnt=0;
    reg [7:0]  srdata=0;
    reg [25:0] saddr_l=0;
    reg        swe=0;
    integer    busy_cycles=0;          // how long the last op held busy
    always @(posedge clk) begin
        if (!rst_n) begin
            sbusy<=0; scnt<=0; srdata<=0;
        end else if (req && !sbusy) begin
            sbusy<=1'b1; scnt<=LAT; srdata<=DECOY; saddr_l<=phys_addr; swe<=we;
            busy_cycles<=0;
        end else if (sbusy) begin
            busy_cycles<=busy_cycles+1;
            if (scnt==8'd0) begin
                sbusy<=1'b0;
                // REAL byte appears ONLY now, on the same edge busy drops
                srdata <= swe ? 8'h00 : (saddr_l[7:0] ^ 8'h5A);
            end else scnt<=scnt-8'd1;
        end
    end

    coproc dut(.clk(clk), .rst_n(rst_n), .ready(1'b1),
        .req(req), .we(we), .phys_addr(phys_addr), .wdata(wdata),
        .busy(sbusy), .rdata(srdata),
        .laddr(13'd0), .ldata_in(8'd0), .lwr(1'b0), .ldata_out(),
        .count_in(8'd0), .count_wr(1'b0),
        .c4_ring_wr(1'b0), .c4_collect_wr(1'b0), .c4_host_slot(2'd0),
        .snapshot_busy(1'b0),
        .c4_done(), .c4_active(), .c4_timedout(), .c4_callreq());

    integer errors=0;

    // A bus transaction by forcing the Arlet nets. rdy is left to the FSM.
    // The force/WE=0 both lag one posedge (per coproc_c4_tb's documented quirk),
    // so the write commits at the SECOND posedge with exactly one capture.
    task busxn(input [15:0] a, input [7:0] d); begin
        force dut.AB=a; force dut.DO=d; force dut.WE=1'b1;
        @(posedge clk);   // force settles (CPU always@* yields)
        @(posedge clk);   // capture: AB/WE/rdy stable -> register/trigger commits
        force dut.WE=1'b0; force dut.AB=16'h00FF;
        @(posedge clk);
    end endtask

    // Fire STA $E007 (one trigger) then wait for the SDRAM read to complete
    // (FSM returns ST_WAIT->ST_RUN, rd_pending clears, sread latched).
    task rtrig; begin
        busxn(16'hE007, 8'h00);
        // rd_pending is set now; wait for the slow stub to finish + latch.
        while (dut.rd_pending !== 1'b0) @(posedge clk);
        @(posedge clk);
    end endtask

    // Read $E008 via the DI mux and return the byte the core would load.
    task read_e008(output [7:0] d); begin
        force dut.AB=16'hE008; force dut.WE=1'b0;
        @(posedge clk);   // AB force settles
        @(posedge clk);   // is_e008_q registers
        @(posedge clk);   // DI settles
        d = dut.DI;
        force dut.AB=16'h00FF;
        @(posedge clk);
    end endtask

    reg [7:0] di_byte;
    initial begin
        rst_n=0;
        // Keep the CPU off the bus during the whole test (rdy NOT forced).
        force dut.WE=1'b0; force dut.AB=16'h00FF; force dut.DO=8'h00;
        #50; rst_n=1; #50;   // let ST_BOOT -> ST_RUN (rdy=1)

        // ---- set read pointer = bank $03, addr $0010 (phys low byte $10) ----
        busxn(16'hE004, 8'h10);   // RADDR_LO
        busxn(16'hE005, 8'h00);   // RADDR_HI
        busxn(16'hE006, 8'h03);   // RBANK
        if (dut.raddr!==16'h0010 || dut.rbank!==8'h03) begin
            errors=errors+1;
            $display("FAIL pointer set: raddr=%04X rbank=%02X (want 0010/03)",dut.raddr,dut.rbank);
        end

        // ---- trigger read #1: addr low $10 -> REAL = $10^$5A = $4A ----
        rtrig;
        if (busy_cycles < 8) begin
            errors=errors+1;
            $display("FAIL stall too short: busy only %0d cycles (stall bypassed?)",busy_cycles);
        end
        // R2 PHASE GATE: must be the REAL post-latency byte, NEVER the decoy.
        if (dut.sread === DECOY) begin
            errors=errors+1;
            $display("FAIL R2: sread latched the DECOY $ED (off-by-one latch vs done)");
        end
        if (dut.sread !== 8'h4A) begin
            errors=errors+1;
            $display("FAIL read#1: sread=%02X want 4A",dut.sread);
        end else $display("PASS read#1 decoy-rejection: sread=%02X (real, not $ED)",dut.sread);
        // also confirm the LDA $E008 DI-mux path returns it
        read_e008(di_byte);
        if (di_byte !== 8'h4A) begin
            errors=errors+1;
            $display("FAIL $E008 DI mux: got %02X want 4A",di_byte);
        end else $display("PASS $E008 DI mux returns sread=%02X",di_byte);
        // auto-inc check: pointer advanced 0010 -> 0011
        if (dut.raddr!==16'h0011 || dut.rbank!==8'h03) begin
            errors=errors+1;
            $display("FAIL auto-inc#1: raddr=%04X rbank=%02X (want 0011/03)",dut.raddr,dut.rbank);
        end

        // ---- trigger read #2: addr low $11 -> REAL = $11^$5A = $4B ----
        rtrig;
        if (dut.sread === DECOY) begin
            errors=errors+1;
            $display("FAIL R2 read#2: sread latched DECOY");
        end
        if (dut.sread !== 8'h4B) begin
            errors=errors+1;
            $display("FAIL read#2 (auto-inc): sread=%02X want 4B",dut.sread);
        end else $display("PASS read#2 auto-inc: sread=%02X (addr A+1)",dut.sread);
        read_e008(di_byte);
        if (di_byte !== 8'h4B) begin
            errors=errors+1;
            $display("FAIL $E008 DI mux read#2: got %02X want 4B",di_byte);
        end
        if (dut.raddr!==16'h0012 || dut.rbank!==8'h03) begin
            errors=errors+1;
            $display("FAIL auto-inc#2: raddr=%04X rbank=%02X (want 0012/03)",dut.raddr,dut.rbank);
        end

        if (errors==0) $display("PASS coproc_sdrd");
        else           $display("FAIL coproc_sdrd %0d",errors);
        $finish;
    end

    // safety timeout
    initial begin #20000; $display("FAIL coproc_sdrd TIMEOUT"); $finish; end
endmodule
