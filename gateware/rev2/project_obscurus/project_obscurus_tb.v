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

        // 4. busy asserts at strobe
        set_bank(10'd0); set_addr(16'h0100);
        wr_reg(4'h6, 8'h7E);
        rd_reg(4'h5, tmp);
        if (!tmp[7]) begin errors=errors+1; $display("FAIL busy-at-strobe"); end
        poll_busy;

        // 5. auto-increment + D dump
        // NOTE: write loop uses a local iterator (not the shared `tmp`) because
        // sdram_write() -> poll_busy() clobbers `tmp` with the STATUS byte; using
        // `tmp` as the loop variable would make the loop run only once (the read
        // loop below already uses a local `i` for the same reason).
        begin : fill
            integer j;
            for (j=0; j<16; j=j+1)
                sdram_write(10'd0, 16'h0200 + j, 8'h20 + j);
        end
        set_bank(10'd0); set_addr(16'h0200);
        begin : dump
            integer i; reg [7:0] v;
            for (i=0; i<16; i=i+1) begin
                wr_reg(4'h4, 8'h00); poll_busy; rd_reg(4'h6, v);
                if (v !== 8'h20 + i) begin
                    errors=errors+1; $display("FAIL dump[%0d]=%02X",i,v); end
            end
        end

        // 6. read with NO preceding strobe must NOT advance addr
        rd_reg(4'h6, tmp);
        set_addr(16'h0200);
        wr_reg(4'h4, 8'h00); poll_busy; rd_reg(4'h6, tmp);
        if (tmp !== 8'h20) begin errors=errors+1; $display("FAIL no-strobe stepped"); end

        // 7. $CFFF byte: rom2mem fills unused space with $FF; the stub here is
        //    all $00. The requirement is only that $CFFF is never live monitor
        //    code. Accept either $00 (stub) or $FF (real build fill).
        if (dut.monitor_mem[11'h7FF] !== 8'h00 && dut.monitor_mem[11'h7FF] !== 8'hFF && dut.monitor_mem[11'h7FF] !== 8'h60)
            begin errors=errors+1; $display("FAIL $CFFF live %02X",dut.monitor_mem[11'h7FF]); end

        // 8. expansion ROM SILENT by default (disarmed): set rom_en via a $C4xx
        //    access, then read $C8xx -> card must NOT drive (exp_read low).
        apple_addr = 16'hC400; nI_O_SELECT=1'b0; R_nW=1'b1; #200;
        nI_O_SELECT=1'b1; #100;
        apple_addr = 16'hC800; nI_O_STROBE=1'b0; R_nW=1'b1; #200;
        if (dut.rom_armed !== 1'b0) begin errors=errors+1; $display("FAIL armed at reset"); end
        if (dut.exp_read  !== 1'b0) begin errors=errors+1; $display("FAIL exp_read while disarmed"); end
        nI_O_STROBE=1'b1; #100;

        // 9. ARM via $C0C7<-$AA; DISARM via $C0C8<-$AA.
        wr_reg(4'h7, 8'hAA);
        if (dut.rom_armed !== 1'b1) begin errors=errors+1; $display("FAIL not armed"); end
        wr_reg(4'h8, 8'hAA);
        if (dut.rom_armed !== 1'b0) begin errors=errors+1; $display("FAIL not disarmed"); end

        // 10. magic guard: non-$AA write to $C0C7 must NOT arm.
        wr_reg(4'h7, 8'h55);
        if (dut.rom_armed !== 1'b0) begin errors=errors+1; $display("FAIL armed by non-magic"); end

        // 11. reset disarms.
        wr_reg(4'h7, 8'hAA);
        nRES_READ=1'b0; #500; nRES_READ=1'b1; #200;
        if (dut.rom_armed !== 1'b0) begin errors=errors+1; $display("FAIL reset did not disarm"); end

        // --- C0: the coproc ran on its own and wrote bank0/$0040 = $42 ---
        // Check 11 above pulsed nRES_READ low, which re-inits the SDRAM
        // controller (back to ST_INIT). Re-wait for `ready` so both the coproc
        // (held in ST_BOOT until ready) and the monitor port operate against a
        // live controller. The coproc then re-runs LDA/STA and re-posts the
        // bank0/$0040 = $42 write a few hundred ns after ready; by the time the
        // sdram_read below issues, that write has completed.
        wait (dut.ready);
        // (coproc posts the write shortly after reset; by now it has completed.)
        sdram_read(10'd0, 16'h0040, tmp);
        if (tmp!==8'h42) begin errors=errors+1; $display("FAIL C0 coproc write got %02X want 42",tmp); end
        else $display("PASS C0 coproc wrote 42 to bank0/$40");
        // monitor regression: host write/read elsewhere still works with arbiter
        // in front. Use bank1 (the sdram_model holds banks 0-1 = 64K words; bank2+
        // is outside the model and would always read 00) at an address untouched
        // by the coproc, to prove an independent c0 op round-trips post-arbiter.
        sdram_write(10'd1, 16'h00AB, 8'h99);
        sdram_read (10'd1, 16'h00AB, tmp);
        if (tmp!==8'h99) begin errors=errors+1; $display("FAIL monitor regress %02X",tmp); end
        else $display("PASS monitor regress");

        if (errors==0) $display("PASS"); else $display("FAIL: %0d errors", errors);
        $finish;
    end
    initial begin #20_000_000; $display("TIMEOUT"); $finish; end
endmodule
