// =============================================================================
// Byte Hamr Signal Check - Testbench
// =============================================================================
//
// Simulates the signal_check_top module with:
// - 25 MHz clock
// - Simple SDRAM model (returns written data)
// - Apple II bus activity (PHI0 toggling at ~1 MHz)
//
// =============================================================================

`timescale 1ns / 100ps

module signal_check_tb;

    // =========================================================================
    // Clock generation - 25 MHz (40ns period)
    // =========================================================================
    reg clk_25mhz = 0;
    always #20 clk_25mhz = ~clk_25mhz;

    // =========================================================================
    // Apple II clock - ~1 MHz (1000ns period)
    // =========================================================================
    reg phi0 = 0;
    reg phi1 = 0;
    always #500 begin
        phi0 = ~phi0;
        phi1 = ~phi0;  // PHI1 is inverted PHI0
    end

    // =========================================================================
    // DUT Signals
    // =========================================================================

    // SDRAM
    wire        sdram_clk;
    wire        sdram_cke;
    wire        sdram_ncs;
    wire        sdram_nras;
    wire        sdram_ncas;
    wire        sdram_nwe;
    wire        sdram_dqm0;
    wire        sdram_dqm1;
    wire        sdram_ba0, sdram_ba1;
    wire        sdram_a0, sdram_a1, sdram_a2, sdram_a3;
    wire        sdram_a4, sdram_a5, sdram_a6, sdram_a7;
    wire        sdram_a8, sdram_a9, sdram_a10, sdram_a11, sdram_a12;
    wire        sdram_d0, sdram_d1, sdram_d2, sdram_d3;
    wire        sdram_d4, sdram_d5, sdram_d6, sdram_d7;
    wire        sdram_d8, sdram_d9, sdram_d10, sdram_d11;
    wire        sdram_d12, sdram_d13, sdram_d14, sdram_d15;

    // Apple II Address bus
    reg [15:0] apple_addr = 16'hC0F0;  // Simulate slot 7 I/O address

    // Apple II Data bus
    wire [7:0] apple_data;
    reg  [7:0] apple_data_drv = 8'hA5;
    reg        apple_data_oe = 0;
    assign apple_data = apple_data_oe ? apple_data_drv : 8'hZZ;

    // Apple II Control
    reg        r_nw = 1;           // Read mode
    reg        nres = 1;           // Not in reset
    reg        nirq = 1;           // No IRQ
    reg        nnmi = 1;           // No NMI
    reg        ndevice_select = 1; // Not selected
    reg        nio_select = 1;     // Not selected
    reg        nio_strobe = 1;     // Not active
    reg        rdy = 1;            // Ready
    reg        dma_in = 0;
    reg        int_in = 0;
    reg        ninh = 1;
    reg        ndma = 1;
    reg        sig_7m = 0;
    reg        q3 = 0;
    reg        usync = 1;

    // Output control signals
    wire       dma_out;
    wire       int_out;

    // GPIO
    wire       gpio1, gpio2, gpio3, gpio4;
    wire       gpio5, gpio6, gpio7, gpio8;
    wire       gpio9, gpio10, gpio11, gpio12;

    // =========================================================================
    // Simple SDRAM Model
    // =========================================================================
    // Stores data written during WRITE commands, returns it during READ

    reg [15:0] sdram_mem [0:7];  // 8 locations to match test
    reg [15:0] sdram_dq_out;
    reg        sdram_dq_oe = 0;
    reg [2:0]  sdram_read_addr;
    reg [2:0]  sdram_cas_delay;

    wire [12:0] sdram_addr = {sdram_a12, sdram_a11, sdram_a10, sdram_a9, sdram_a8,
                              sdram_a7, sdram_a6, sdram_a5, sdram_a4,
                              sdram_a3, sdram_a2, sdram_a1, sdram_a0};
    wire [15:0] sdram_dq_in = {sdram_d15, sdram_d14, sdram_d13, sdram_d12,
                               sdram_d11, sdram_d10, sdram_d9, sdram_d8,
                               sdram_d7, sdram_d6, sdram_d5, sdram_d4,
                               sdram_d3, sdram_d2, sdram_d1, sdram_d0};
    wire [1:0]  sdram_ba = {sdram_ba1, sdram_ba0};

    // Directly assign with tristate
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

    // SDRAM command decode
    wire [3:0] sdram_cmd = {sdram_ncs, sdram_nras, sdram_ncas, sdram_nwe};
    localparam CMD_NOP       = 4'b0111;
    localparam CMD_ACTIVE    = 4'b0011;
    localparam CMD_READ      = 4'b0101;
    localparam CMD_WRITE     = 4'b0100;
    localparam CMD_PRECHARGE = 4'b0010;
    localparam CMD_REFRESH   = 4'b0001;
    localparam CMD_LOAD_MODE = 4'b0000;

    reg [2:0] active_row;  // Track activated row

    integer i;
    initial begin
        for (i = 0; i < 8; i = i + 1)
            sdram_mem[i] = 16'h0000;
        sdram_cas_delay = 0;
    end

    always @(posedge sdram_clk) begin
        // Default: stop driving after 1 cycle
        if (sdram_cas_delay > 0) begin
            sdram_cas_delay <= sdram_cas_delay - 1;
            if (sdram_cas_delay == 1) begin
                sdram_dq_oe <= 1;
                sdram_dq_out <= sdram_mem[sdram_read_addr];
            end
        end else begin
            sdram_dq_oe <= 0;
        end

        case (sdram_cmd)
            CMD_ACTIVE: begin
                active_row <= sdram_addr[2:0];
            end
            CMD_WRITE: begin
                sdram_mem[active_row] <= sdram_dq_in;
                $display("[%0t] SDRAM WRITE: addr=%0d data=%04h", $time, active_row, sdram_dq_in);
            end
            CMD_READ: begin
                sdram_read_addr <= active_row;
                sdram_cas_delay <= 2;  // CAS latency 2
                $display("[%0t] SDRAM READ:  addr=%0d (data=%04h)", $time, active_row, sdram_mem[active_row]);
            end
        endcase
    end

    // =========================================================================
    // Device Under Test
    // =========================================================================

    signal_check_top dut (
        .CLK_25MHz(clk_25mhz),

        // SDRAM
        .SDRAM_CLK(sdram_clk),
        .SDRAM_CKE(sdram_cke),
        .SDRAM_nCS(sdram_ncs),
        .SDRAM_nRAS(sdram_nras),
        .SDRAM_nCAS(sdram_ncas),
        .SDRAM_nWE(sdram_nwe),
        .SDRAM_DQM0(sdram_dqm0),
        .SDRAM_DQM1(sdram_dqm1),
        .SDRAM_BA0(sdram_ba0),
        .SDRAM_BA1(sdram_ba1),
        .SDRAM_A0(sdram_a0),
        .SDRAM_A1(sdram_a1),
        .SDRAM_A2(sdram_a2),
        .SDRAM_A3(sdram_a3),
        .SDRAM_A4(sdram_a4),
        .SDRAM_A5(sdram_a5),
        .SDRAM_A6(sdram_a6),
        .SDRAM_A7(sdram_a7),
        .SDRAM_A8(sdram_a8),
        .SDRAM_A9(sdram_a9),
        .SDRAM_A10(sdram_a10),
        .SDRAM_A11(sdram_a11),
        .SDRAM_A12(sdram_a12),
        .SDRAM_D0(sdram_d0),
        .SDRAM_D1(sdram_d1),
        .SDRAM_D2(sdram_d2),
        .SDRAM_D3(sdram_d3),
        .SDRAM_D4(sdram_d4),
        .SDRAM_D5(sdram_d5),
        .SDRAM_D6(sdram_d6),
        .SDRAM_D7(sdram_d7),
        .SDRAM_D8(sdram_d8),
        .SDRAM_D9(sdram_d9),
        .SDRAM_D10(sdram_d10),
        .SDRAM_D11(sdram_d11),
        .SDRAM_D12(sdram_d12),
        .SDRAM_D13(sdram_d13),
        .SDRAM_D14(sdram_d14),
        .SDRAM_D15(sdram_d15),

        // Apple II Address
        .A0(apple_addr[0]),
        .A1(apple_addr[1]),
        .A2(apple_addr[2]),
        .A3(apple_addr[3]),
        .A4(apple_addr[4]),
        .A5(apple_addr[5]),
        .A6(apple_addr[6]),
        .A7(apple_addr[7]),
        .A8(apple_addr[8]),
        .A9(apple_addr[9]),
        .A10(apple_addr[10]),
        .A11(apple_addr[11]),
        .A12(apple_addr[12]),
        .A13(apple_addr[13]),
        .A14(apple_addr[14]),
        .A15(apple_addr[15]),

        // Apple II Data
        .D0(apple_data[0]),
        .D1(apple_data[1]),
        .D2(apple_data[2]),
        .D3(apple_data[3]),
        .D4(apple_data[4]),
        .D5(apple_data[5]),
        .D6(apple_data[6]),
        .D7(apple_data[7]),

        // Clocks and timing
        .PHI0(phi0),
        .PHI1(phi1),
        .sig_7M(sig_7m),
        .Q3(q3),
        .uSync(usync),

        // Control
        .R_nW(r_nw),
        .nRES(nres),
        .nIRQ(nirq),
        .nNMI(nnmi),
        .nDEVICE_SELECT(ndevice_select),
        .nI_O_SELECT(nio_select),
        .nI_O_STROBE(nio_strobe),
        .RDY(rdy),
        .DMA_IN(dma_in),
        .INT_IN(int_in),
        .nINH(ninh),
        .nDMA(ndma),

        // Output control
        .DMA_OUT(dma_out),
        .INT_OUT(int_out),

        // GPIO
        .GPIO1(gpio1),
        .GPIO2(gpio2),
        .GPIO3(gpio3),
        .GPIO4(gpio4),
        .GPIO5(gpio5),
        .GPIO6(gpio6),
        .GPIO7(gpio7),
        .GPIO8(gpio8),
        .GPIO9(gpio9),
        .GPIO10(gpio10),
        .GPIO11(gpio11),
        .GPIO12(gpio12)
    );

    // =========================================================================
    // Test Sequence
    // =========================================================================

    initial begin
        $dumpfile("signal_check_tb.vcd");
        $dumpvars(0, signal_check_tb);

        $display("===========================================");
        $display("Byte Hamr Signal Check Testbench");
        $display("===========================================");
        $display("");

        // Wait for reset to complete (~2.6ms at 25MHz = 65536 cycles)
        $display("[%0t] Waiting for reset...", $time);
        #3_000_000;  // 3ms for reset

        // Wait for SDRAM init (200us power-up + commands)
        $display("[%0t] Waiting for SDRAM initialization...", $time);
        #500_000;  // 500us

        // Now SDRAM test should be running
        $display("[%0t] SDRAM test in progress...", $time);

        // Wait for test to complete
        #1_000_000;  // 1ms for write/read cycles

        // Check results
        $display("");
        $display("===========================================");
        $display("Test Results:");
        $display("===========================================");
        $display("  GPIO1 (Heartbeat): %b", gpio1);
        $display("  GPIO2 (SDRAM Pass): %b", gpio2);
        $display("  GPIO3 (Apple Activity): %b", gpio3);
        $display("  GPIO4 (Error): %b", gpio4);
        $display("  GPIO5-12 (Walking): %b%b%b%b%b%b%b%b",
                 gpio12, gpio11, gpio10, gpio9, gpio8, gpio7, gpio6, gpio5);
        $display("");

        if (gpio2 && !gpio4) begin
            $display("*** SDRAM TEST PASSED ***");
        end else if (gpio4) begin
            $display("*** SDRAM TEST FAILED ***");
        end else begin
            $display("*** SDRAM TEST INCOMPLETE ***");
        end

        if (gpio3) begin
            $display("*** Apple II bus activity detected ***");
        end

        $display("");
        $display("Simulation complete.");
        $finish;
    end

    // =========================================================================
    // Monitor
    // =========================================================================

    // Watch for state changes
    reg prev_gpio2 = 0;
    reg prev_gpio4 = 0;

    always @(posedge clk_25mhz) begin
        if (gpio2 && !prev_gpio2)
            $display("[%0t] SDRAM test PASSED (GPIO2 asserted)", $time);
        if (gpio4 && !prev_gpio4)
            $display("[%0t] SDRAM test FAILED (GPIO4 asserted)", $time);
        prev_gpio2 <= gpio2;
        prev_gpio4 <= gpio4;
    end

endmodule
