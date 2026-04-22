// =============================================================================
// Byte Hamr Rev 2 - Signal Check Testbench
// =============================================================================
// Drives the 100 MHz input; DUT divides internally to 25 MHz.
// Simple SDRAM model that echoes back written data.
// =============================================================================

`timescale 1ns / 100ps

module signal_check_tb;

    // 100 MHz (10 ns period)
    reg clk_100mhz = 0;
    always #5 clk_100mhz = ~clk_100mhz;

    // ~1 MHz Apple II PHI0 (1000 ns period)
    reg phi0 = 0;
    reg phi1 = 0;
    always #500 begin
        phi0 = ~phi0;
        phi1 = ~phi0;
    end

    // SDRAM wires
    wire sdram_clk, sdram_cke;
    wire sdram_ncs, sdram_nras, sdram_ncas, sdram_nwe;
    wire sdram_dqm0, sdram_dqm1;
    wire sdram_ba0, sdram_ba1;
    wire sdram_a0, sdram_a1, sdram_a2, sdram_a3;
    wire sdram_a4, sdram_a5, sdram_a6, sdram_a7;
    wire sdram_a8, sdram_a9, sdram_a10, sdram_a11, sdram_a12;
    wire sdram_d0, sdram_d1, sdram_d2, sdram_d3;
    wire sdram_d4, sdram_d5, sdram_d6, sdram_d7;
    wire sdram_d8, sdram_d9, sdram_d10, sdram_d11;
    wire sdram_d12, sdram_d13, sdram_d14, sdram_d15;

    // Apple II bus
    reg [15:0] apple_addr = 16'hC0F0;
    wire [7:0] apple_data;
    reg  [7:0] apple_data_drv = 8'hA5;
    reg        apple_data_oe = 0;
    assign apple_data = apple_data_oe ? apple_data_drv : 8'hZZ;

    reg r_nw = 1;
    reg ndevice_select = 1;
    reg nio_select = 1;
    reg nio_strobe = 1;
    reg rdy = 1;
    reg dma_out = 0;
    reg int_out = 0;
    reg sig_7m = 0;
    reg q3 = 0;
    reg usync = 1;
    reg nres_read = 1;  // Apple II not in reset

    // FPGA outputs
    wire nres_out, nirq, nnmi, ninh, ndma;
    wire dma_in, int_in;
    wire data_oe;
    wire gpio_1, gpio_2, gpio_3, gpio_4, gpio_5;
    wire gpio_6, gpio_7, gpio_8, gpio_9, gpio_10;
    wire gpio_11, gpio_12, gpio_13, gpio_14, gpio_15;
    wire gpio_16, gpio_17, gpio_18, gpio_19, gpio_20;

    // ---- Simple SDRAM model ----
    reg [15:0] sdram_mem [0:7];
    reg [15:0] sdram_dq_out;
    reg        sdram_dq_oe = 0;
    reg [2:0]  sdram_read_addr;
    reg [2:0]  sdram_cas_delay;

    wire [15:0] sdram_dq_in = {sdram_d15, sdram_d14, sdram_d13, sdram_d12,
                               sdram_d11, sdram_d10, sdram_d9, sdram_d8,
                               sdram_d7, sdram_d6, sdram_d5, sdram_d4,
                               sdram_d3, sdram_d2, sdram_d1, sdram_d0};
    wire [12:0] sdram_addr = {sdram_a12, sdram_a11, sdram_a10, sdram_a9, sdram_a8,
                              sdram_a7, sdram_a6, sdram_a5, sdram_a4,
                              sdram_a3, sdram_a2, sdram_a1, sdram_a0};

    assign sdram_d0  = sdram_dq_oe ? sdram_dq_out[0]  : 1'bZ;
    assign sdram_d1  = sdram_dq_oe ? sdram_dq_out[1]  : 1'bZ;
    assign sdram_d2  = sdram_dq_oe ? sdram_dq_out[2]  : 1'bZ;
    assign sdram_d3  = sdram_dq_oe ? sdram_dq_out[3]  : 1'bZ;
    assign sdram_d4  = sdram_dq_oe ? sdram_dq_out[4]  : 1'bZ;
    assign sdram_d5  = sdram_dq_oe ? sdram_dq_out[5]  : 1'bZ;
    assign sdram_d6  = sdram_dq_oe ? sdram_dq_out[6]  : 1'bZ;
    assign sdram_d7  = sdram_dq_oe ? sdram_dq_out[7]  : 1'bZ;
    assign sdram_d8  = sdram_dq_oe ? sdram_dq_out[8]  : 1'bZ;
    assign sdram_d9  = sdram_dq_oe ? sdram_dq_out[9]  : 1'bZ;
    assign sdram_d10 = sdram_dq_oe ? sdram_dq_out[10] : 1'bZ;
    assign sdram_d11 = sdram_dq_oe ? sdram_dq_out[11] : 1'bZ;
    assign sdram_d12 = sdram_dq_oe ? sdram_dq_out[12] : 1'bZ;
    assign sdram_d13 = sdram_dq_oe ? sdram_dq_out[13] : 1'bZ;
    assign sdram_d14 = sdram_dq_oe ? sdram_dq_out[14] : 1'bZ;
    assign sdram_d15 = sdram_dq_oe ? sdram_dq_out[15] : 1'bZ;

    wire [3:0] sdram_cmd = {sdram_ncs, sdram_nras, sdram_ncas, sdram_nwe};
    localparam CMD_ACTIVE = 4'b0011;
    localparam CMD_READ   = 4'b0101;
    localparam CMD_WRITE  = 4'b0100;

    reg [2:0] active_row;
    integer i;
    initial begin
        for (i = 0; i < 8; i = i + 1) sdram_mem[i] = 16'h0000;
        sdram_cas_delay = 0;
    end

    always @(posedge sdram_clk) begin
        if (sdram_cas_delay > 0) begin
            sdram_cas_delay <= sdram_cas_delay - 1;
            if (sdram_cas_delay == 1) begin
                sdram_dq_oe  <= 1;
                sdram_dq_out <= sdram_mem[sdram_read_addr];
            end
        end else begin
            sdram_dq_oe <= 0;
        end

        case (sdram_cmd)
            CMD_ACTIVE: active_row <= sdram_addr[2:0];
            CMD_WRITE: begin
                sdram_mem[active_row] <= sdram_dq_in;
                $display("[%0t] SDRAM WRITE: row=%0d data=%04h", $time, active_row, sdram_dq_in);
            end
            CMD_READ: begin
                sdram_read_addr <= active_row;
                sdram_cas_delay <= 2;
                $display("[%0t] SDRAM READ:  row=%0d data=%04h", $time, active_row, sdram_mem[active_row]);
            end
        endcase
    end

    // ---- DUT ----
    signal_check_top dut (
        .CLK_100MHz(clk_100mhz),

        .SDRAM_CLK(sdram_clk),   .SDRAM_CKE(sdram_cke),
        .SDRAM_nCS(sdram_ncs),   .SDRAM_nRAS(sdram_nras),
        .SDRAM_nCAS(sdram_ncas), .SDRAM_nWE(sdram_nwe),
        .SDRAM_DQM0(sdram_dqm0), .SDRAM_DQM1(sdram_dqm1),
        .SDRAM_BA0(sdram_ba0),   .SDRAM_BA1(sdram_ba1),
        .SDRAM_A0(sdram_a0),   .SDRAM_A1(sdram_a1),   .SDRAM_A2(sdram_a2),
        .SDRAM_A3(sdram_a3),   .SDRAM_A4(sdram_a4),   .SDRAM_A5(sdram_a5),
        .SDRAM_A6(sdram_a6),   .SDRAM_A7(sdram_a7),   .SDRAM_A8(sdram_a8),
        .SDRAM_A9(sdram_a9),   .SDRAM_A10(sdram_a10), .SDRAM_A11(sdram_a11),
        .SDRAM_A12(sdram_a12),
        .SDRAM_D0(sdram_d0),   .SDRAM_D1(sdram_d1),   .SDRAM_D2(sdram_d2),
        .SDRAM_D3(sdram_d3),   .SDRAM_D4(sdram_d4),   .SDRAM_D5(sdram_d5),
        .SDRAM_D6(sdram_d6),   .SDRAM_D7(sdram_d7),   .SDRAM_D8(sdram_d8),
        .SDRAM_D9(sdram_d9),   .SDRAM_D10(sdram_d10), .SDRAM_D11(sdram_d11),
        .SDRAM_D12(sdram_d12), .SDRAM_D13(sdram_d13), .SDRAM_D14(sdram_d14),
        .SDRAM_D15(sdram_d15),

        .A0(apple_addr[0]),   .A1(apple_addr[1]),   .A2(apple_addr[2]),
        .A3(apple_addr[3]),   .A4(apple_addr[4]),   .A5(apple_addr[5]),
        .A6(apple_addr[6]),   .A7(apple_addr[7]),   .A8(apple_addr[8]),
        .A9(apple_addr[9]),   .A10(apple_addr[10]), .A11(apple_addr[11]),
        .A12(apple_addr[12]), .A13(apple_addr[13]), .A14(apple_addr[14]),
        .A15(apple_addr[15]),

        .D0(apple_data[0]), .D1(apple_data[1]), .D2(apple_data[2]),
        .D3(apple_data[3]), .D4(apple_data[4]), .D5(apple_data[5]),
        .D6(apple_data[6]), .D7(apple_data[7]),

        .PHI0(phi0), .PHI1(phi1),
        .sig_7M(sig_7m), .Q3(q3), .uSync(usync),

        .R_nW(r_nw),
        .nDEVICE_SELECT(ndevice_select),
        .nI_O_SELECT(nio_select),
        .nI_O_STROBE(nio_strobe),
        .DMA_OUT(dma_out), .INT_OUT(int_out), .RDY(rdy),
        .nRES_READ(nres_read),

        .nIRQ(nirq), .nNMI(nnmi), .nINH(ninh), .nDMA(ndma), .nRES(nres_out),
        .DMA_IN(dma_in), .INT_IN(int_in),
        .DATA_OE(data_oe),

        .GPIO_1(gpio_1),  .GPIO_2(gpio_2),  .GPIO_3(gpio_3),  .GPIO_4(gpio_4),
        .GPIO_5(gpio_5),  .GPIO_6(gpio_6),  .GPIO_7(gpio_7),  .GPIO_8(gpio_8),
        .GPIO_9(gpio_9),  .GPIO_10(gpio_10), .GPIO_11(gpio_11), .GPIO_12(gpio_12),
        .GPIO_13(gpio_13), .GPIO_14(gpio_14), .GPIO_15(gpio_15), .GPIO_16(gpio_16),
        .GPIO_17(gpio_17), .GPIO_18(gpio_18), .GPIO_19(gpio_19), .GPIO_20(gpio_20)
    );

    initial begin
        $dumpfile("signal_check_tb.vcd");
        $dumpvars(0, signal_check_tb);

        $display("===========================================");
        $display("Byte Hamr Rev 2 Signal Check Testbench");
        $display("===========================================");

        #3_000_000;  // wait for reset
        $display("[%0t] Waiting for SDRAM init...", $time);
        #500_000;
        $display("[%0t] SDRAM test in progress...", $time);
        #1_000_000;

        $display("");
        $display("Test Results:");
        $display("  GPIO_1 (Heartbeat):       %b", gpio_1);
        $display("  GPIO_2 (SDRAM Pass):      %b", gpio_2);
        $display("  GPIO_3 (IO SEL/STROBE):   %b", gpio_3);
        $display("  GPIO_4 (DEVICE_SEL):      %b", gpio_4);
        $display("  GPIO_8 (nRES_READ):       %b", gpio_8);
        $display("  Walking 1s (GPIO_20..9):  %b%b%b%b%b%b%b%b%b%b%b%b",
                 gpio_20, gpio_19, gpio_18, gpio_17, gpio_16, gpio_15,
                 gpio_14, gpio_13, gpio_12, gpio_11, gpio_10, gpio_9);
        $display("  DATA_OE:                  %b (0 = enabled when slot active)", data_oe);

        if (gpio_2) $display("*** SDRAM TEST PASSED ***");
        else        $display("*** SDRAM TEST INCOMPLETE ***");

        $display("Simulation complete.");
        $finish;
    end

    reg prev_gpio2 = 0;
    always @(posedge sdram_clk) begin
        if (gpio_2 && !prev_gpio2)
            $display("[%0t] SDRAM test PASSED (GPIO_2 asserted)", $time);
        prev_gpio2 <= gpio_2;
    end

endmodule
