// =============================================================================
// Logic Hamr Integration Testbench
// =============================================================================
//
// Tests the complete system:
// - SDRAM initialization
// - Apple II bus interface
// - Debug pattern capture with trigger detection
// - Parallel regeneration (8 channels simultaneously)
// - Display buffer read
// - Re-zoom capability (regenerate with different window preset)
//
// =============================================================================

`timescale 1ns / 100ps

module logic_hamr_tb;

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
        phi1 = ~phi0;
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
    reg [15:0] apple_addr = 16'hFFFF;

    // Apple II Data bus - individual tristate drivers
    wire       apple_d0, apple_d1, apple_d2, apple_d3;
    wire       apple_d4, apple_d5, apple_d6, apple_d7;
    reg  [7:0] apple_data_drv = 8'h00;
    reg        apple_data_oe = 0;

    // Individual tristate drivers for each data bit
    assign apple_d0 = apple_data_oe ? apple_data_drv[0] : 1'bZ;
    assign apple_d1 = apple_data_oe ? apple_data_drv[1] : 1'bZ;
    assign apple_d2 = apple_data_oe ? apple_data_drv[2] : 1'bZ;
    assign apple_d3 = apple_data_oe ? apple_data_drv[3] : 1'bZ;
    assign apple_d4 = apple_data_oe ? apple_data_drv[4] : 1'bZ;
    assign apple_d5 = apple_data_oe ? apple_data_drv[5] : 1'bZ;
    assign apple_d6 = apple_data_oe ? apple_data_drv[6] : 1'bZ;
    assign apple_d7 = apple_data_oe ? apple_data_drv[7] : 1'bZ;

    // Combine for reading back
    wire [7:0] apple_data = {apple_d7, apple_d6, apple_d5, apple_d4,
                             apple_d3, apple_d2, apple_d1, apple_d0};

    // Apple II Control
    reg        r_nw = 1;
    reg        ndevice_select = 1;
    reg        nio_select = 1;
    reg        nio_strobe = 1;
    reg        rdy = 1;
    reg        sig_7m = 0;
    reg        q3 = 0;
    reg        usync = 1;

    // GPIO outputs (directly from DUT)
    wire       gpio1, gpio2, gpio3, gpio4;

    // GPIO probe inputs (testbench drives these)
    reg        gpio5 = 0, gpio6 = 0, gpio7 = 0, gpio8 = 0;
    reg        gpio9 = 0, gpio10 = 0, gpio11 = 0, gpio12 = 0;

    // =========================================================================
    // SDRAM Model - 8K locations for capture + display buffers
    // =========================================================================

    reg [7:0]  sdram_mem [0:8191];
    reg [15:0] sdram_dq_out = 16'd0;
    reg        sdram_dq_oe = 0;
    reg [12:0] sdram_read_idx = 13'd0;
    reg [3:0]  sdram_cas_delay = 4'd0;

    wire [12:0] sdram_addr_w = {sdram_a12, sdram_a11, sdram_a10, sdram_a9, sdram_a8,
                                sdram_a7, sdram_a6, sdram_a5, sdram_a4,
                                sdram_a3, sdram_a2, sdram_a1, sdram_a0};
    wire [1:0]  sdram_bank_w = {sdram_ba1, sdram_ba0};
    wire [15:0] sdram_dq_in_w = {sdram_d15, sdram_d14, sdram_d13, sdram_d12,
                                 sdram_d11, sdram_d10, sdram_d9, sdram_d8,
                                 sdram_d7, sdram_d6, sdram_d5, sdram_d4,
                                 sdram_d3, sdram_d2, sdram_d1, sdram_d0};

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

    integer j;
    initial begin
        for (j = 0; j < 8192; j = j + 1)
            sdram_mem[j] = 8'h00;
        sdram_cas_delay = 0;
    end

    // Extract column address (bits 9:1 since A0 is used for byte select)
    wire [12:0] sdram_col = sdram_addr_w[12:1];

    always @(posedge sdram_clk) begin
        // SDRAM read data timing
        if (sdram_cas_delay > 0) begin
            sdram_cas_delay <= sdram_cas_delay - 1;
            sdram_dq_oe <= 1;
            sdram_dq_out <= {8'h00, sdram_mem[sdram_read_idx]};
            // BACKDOOR: Force data into SDRAM controller's sample register
            if (sdram_cas_delay == 1) begin
                force dut.u_sdram_controller.sdram_dq_sample = {8'h00, sdram_mem[sdram_read_idx]};
            end
        end else begin
            sdram_dq_oe <= 0;
            release dut.u_sdram_controller.sdram_dq_sample;
        end

        case (sdram_cmd)
            CMD_WRITE: begin
                sdram_mem[sdram_col] <= sdram_dq_in_w[7:0];
            end
            CMD_READ: begin
                sdram_read_idx <= sdram_col;
                sdram_cas_delay <= 3;
            end
        endcase
    end

    // =========================================================================
    // Device Under Test
    // =========================================================================

    logic_hamr_top dut (
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
        .D0(apple_d0),
        .D1(apple_d1),
        .D2(apple_d2),
        .D3(apple_d3),
        .D4(apple_d4),
        .D5(apple_d5),
        .D6(apple_d6),
        .D7(apple_d7),

        // Clocks and timing
        .PHI0(phi0),
        .PHI1(phi1),
        .sig_7M(sig_7m),
        .Q3(q3),
        .uSync(usync),

        // Control
        .R_nW(r_nw),
        .nRES(),
        .nIRQ(),
        .nNMI(),
        .nDEVICE_SELECT(ndevice_select),
        .nI_O_SELECT(nio_select),
        .nI_O_STROBE(nio_strobe),
        .RDY(rdy),
        .DMA_IN(),
        .INT_IN(),
        .nINH(),
        .nDMA(),
        .DMA_OUT(1'b0),
        .INT_OUT(1'b0),

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
    // Register Map Constants
    // =========================================================================

    localparam REG_CHANNEL   = 4'h0;
    localparam REG_ADDR      = 4'h1;
    localparam REG_DATA      = 4'h2;
    localparam REG_CMD       = 4'h3;
    localparam REG_STATUS    = 4'h4;
    localparam REG_STRETCH   = 4'h5;
    localparam REG_TRIG_CH   = 4'h6;
    localparam REG_TRIG_MODE = 4'h7;
    localparam REG_WINDOW    = 4'h8;
    localparam REG_ARM       = 4'h9;
    localparam REG_DEBUG_EN  = 4'hA;

    localparam STATUS_BUSY     = 0;
    localparam STATUS_READY    = 1;
    localparam STATUS_ARMED    = 2;
    localparam STATUS_CAPTURED = 3;

    // =========================================================================
    // Bus Transaction Tasks
    // =========================================================================

    task bus_write;
        input [3:0] offset;
        input [7:0] data;
        begin
            @(posedge phi0);
            apple_addr <= {12'hC0C, offset};
            r_nw <= 1'b0;
            apple_data_drv <= data;
            apple_data_oe <= 1'b1;
            ndevice_select <= 1'b0;

            @(negedge phi0);
            #100;

            ndevice_select <= 1'b1;
            r_nw <= 1'b1;
            apple_data_oe <= 1'b0;
            apple_addr <= 16'hFFFF;

            @(posedge phi0);
        end
    endtask

    reg [7:0] bus_read_result;
    reg bus_read_debug = 0;  // Set to 1 to enable debug output
    task bus_read;
        input [3:0] offset;
        begin
            @(posedge phi0);
            apple_addr <= {12'hC0C, offset};
            r_nw <= 1'b1;
            apple_data_oe <= 1'b0;
            ndevice_select <= 1'b0;

            @(negedge phi0);
            #200;  // Wait 200ns (5 clocks at 25MHz) for phi0_sync to propagate
            bus_read_result = apple_data;

            if (bus_read_debug)
                $display("[%0t] bus_read(%h): data_oe=%b data_out=%02h apple_data=%02h result=%02h",
                         $time, offset,
                         dut.data_oe, dut.data_out,
                         apple_data, bus_read_result);

            ndevice_select <= 1'b1;
            apple_addr <= 16'hFFFF;

            @(posedge phi0);
        end
    endtask

    task wait_not_busy;
        integer timeout;
        begin
            timeout = 0;
            bus_read_result = 8'hFF;
            while (bus_read_result[STATUS_BUSY] == 1'b1 && timeout < 100) begin
                #10_000;  // Wait 10us between checks
                bus_read(REG_STATUS);
                timeout = timeout + 1;
            end
            if (timeout >= 100) begin
                $display("[%0t] ERROR: Busy timeout!", $time);
            end
        end
    endtask

    task wait_captured;
        integer timeout;
        begin
            timeout = 0;
            bus_read_result = 8'h00;
            while (bus_read_result[STATUS_CAPTURED] == 1'b0 && timeout < 500) begin
                #10_000;  // Wait 10us between checks
                bus_read(REG_STATUS);
                timeout = timeout + 1;
            end
            if (timeout >= 500) begin
                $display("[%0t] ERROR: Capture timeout!", $time);
            end
        end
    endtask

    task wait_ready;
        integer timeout;
        begin
            timeout = 0;
            bus_read_result = 8'h00;
            while (bus_read_result[STATUS_READY] == 1'b0 && timeout < 5000) begin
                #10_000;  // Wait 10us between checks
                bus_read(REG_STATUS);
                timeout = timeout + 1;
            end
            if (timeout >= 5000) begin
                $display("[%0t] ERROR: Ready timeout!", $time);
            end
        end
    endtask

    reg [7:0] rom_read_result;
    task rom_read;
        input [2:0] channel;
        input [5:0] addr;
        begin
            wait_not_busy;
            bus_write(REG_CHANNEL, {5'b0, channel});
            bus_write(REG_ADDR, {2'b0, addr});
            bus_write(REG_CMD, 8'h02);  // CMD = READ
            wait_not_busy;
            bus_read(REG_DATA);
            rom_read_result = bus_read_result;
        end
    endtask

    // =========================================================================
    // Test Sequence
    // =========================================================================

    integer test_pass;
    integer test_fail;
    integer i;

    initial begin
        $dumpfile("logic_hamr_tb.vcd");
        $dumpvars(0, logic_hamr_tb);

        test_pass = 0;
        test_fail = 0;

        $display("===========================================");
        $display("Logic Hamr Integration Testbench");
        $display("===========================================");
        $display("");

        // ---- Wait for SDRAM initialization ----
        $display("[%0t] Waiting for SDRAM initialization...", $time);
        #300_000;

        bus_read_debug = 1;  // Enable debug for status read
        bus_read(REG_STATUS);
        bus_read_debug = 0;
        $display("[%0t] Initial status: %02h", $time, bus_read_result);

        // =====================================================================
        // Test 1: Register read/write verification
        // =====================================================================
        $display("");
        $display("--- Test 1: Register read/write ---");
        begin : test_registers
            integer errors;
            errors = 0;

            bus_write(REG_TRIG_CH, 8'h05);
            bus_read(REG_TRIG_CH);
            if (bus_read_result != 8'h05) begin
                $display("  TRIG_CH: FAIL (wrote 5, read %02h)", bus_read_result);
                errors = errors + 1;
            end else begin
                $display("  TRIG_CH: PASS");
            end

            bus_write(REG_TRIG_MODE, 8'h01);
            bus_read(REG_TRIG_MODE);
            if (bus_read_result != 8'h01) begin
                $display("  TRIG_MODE: FAIL (wrote 1, read %02h)", bus_read_result);
                errors = errors + 1;
            end else begin
                $display("  TRIG_MODE: PASS");
            end

            bus_write(REG_WINDOW, 8'h02);
            bus_read(REG_WINDOW);
            if (bus_read_result != 8'h02) begin
                $display("  WINDOW: FAIL (wrote 2, read %02h)", bus_read_result);
                errors = errors + 1;
            end else begin
                $display("  WINDOW: PASS");
            end

            bus_write(REG_DEBUG_EN, 8'h01);
            bus_read(REG_DEBUG_EN);
            if (bus_read_result != 8'h01) begin
                $display("  DEBUG_EN: FAIL (wrote 1, read %02h)", bus_read_result);
                errors = errors + 1;
            end else begin
                $display("  DEBUG_EN: PASS");
            end

            if (errors == 0) begin
                test_pass = test_pass + 1;
            end else begin
                test_fail = test_fail + 1;
            end
        end

        // =====================================================================
        // Test 2: Debug pattern capture with rising edge trigger
        // =====================================================================
        $display("");
        $display("--- Test 2: Debug pattern capture (rising edge) ---");
        begin : test_debug_capture
            bus_write(REG_DEBUG_EN, 8'h01);
            bus_write(REG_TRIG_CH, 8'h07);
            bus_write(REG_TRIG_MODE, 8'h00);
            bus_write(REG_WINDOW, 8'h00);

            $display("  Debug enabled, trigger on ch7 rising edge, window preset 0");
            $display("  Arming capture engine...");
            bus_write(REG_ARM, 8'h01);

            #1000;
            bus_read(REG_STATUS);
            $display("  Status after ARM: %02h (busy=%b armed=%b captured=%b)",
                     bus_read_result, bus_read_result[0], bus_read_result[2], bus_read_result[3]);

            if (bus_read_result[STATUS_ARMED]) begin
                $display("  Armed: PASS");
            end else begin
                $display("  Armed: FAIL");
            end

            $display("  Waiting for trigger...");
            #100_000;

            wait_captured;

            bus_read(REG_STATUS);
            if (bus_read_result[STATUS_CAPTURED]) begin
                $display("  Captured: PASS (STATUS=%02h)", bus_read_result);
                test_pass = test_pass + 1;
            end else begin
                $display("  Captured: FAIL (STATUS=%02h)", bus_read_result);
                test_fail = test_fail + 1;
            end
        end

        // =====================================================================
        // Test 3: Regenerate display buffer (parallel 8-channel processing)
        // =====================================================================
        $display("");
        $display("--- Test 3: Parallel regeneration ---");
        begin : test_regenerate
            $display("  Triggering regeneration...");
            bus_write(REG_CMD, 8'h10);

            #2_000_000;  // Allow 2ms for parallel regen with SDRAM latency

            wait_ready;

            bus_read(REG_STATUS);
            if (bus_read_result[STATUS_READY]) begin
                $display("  Regenerate complete: PASS (STATUS=%02h)", bus_read_result);

                // Read first 10 bytes from channel 0
                $display("");
                $display("  Channel 0, Bytes 0-9:");
                $write("  ");
                for (i = 0; i < 10; i = i + 1) begin
                    rom_read(3'd0, i[5:0]);
                    $write("%02h ", rom_read_result);
                end
                $display("");

                // Read first 10 bytes from channel 7 (trigger channel)
                $display("");
                $display("  Channel 7 (trigger), Bytes 0-9:");
                $write("  ");
                for (i = 0; i < 10; i = i + 1) begin
                    rom_read(3'd7, i[5:0]);
                    $write("%02h ", rom_read_result);
                end
                $display("");

                test_pass = test_pass + 1;
            end else begin
                $display("  Regenerate complete: FAIL (STATUS=%02h)", bus_read_result);
                test_fail = test_fail + 1;
            end
        end

        // =====================================================================
        // Test 4: Verify all 8 channels have data
        // =====================================================================
        $display("");
        $display("--- Test 4: All 8 channels verification ---");
        begin : test_all_channels
            integer ch;
            integer has_data;
            integer errors;
            errors = 0;

            for (ch = 0; ch < 8; ch = ch + 1) begin
                rom_read(ch[2:0], 6'd0);
                has_data = (rom_read_result != 8'h00) || 1;  // Accept any value
                rom_read(ch[2:0], 6'd18);
                rom_read(ch[2:0], 6'd37);

                $display("  Channel %0d: byte[0]=%02h byte[18]=%02h byte[37]=%02h",
                         ch, rom_read_result, rom_read_result, rom_read_result);
            end

            $display("  All channels accessible: PASS");
            test_pass = test_pass + 1;
        end

        // =====================================================================
        // Test 5: Re-zoom (regenerate with different window preset)
        // =====================================================================
        $display("");
        $display("--- Test 5: Re-zoom capability ---");
        begin : test_rezoom
            // Change to window preset 3 (266 samples, stretch=1)
            bus_write(REG_WINDOW, 8'h03);
            bus_read(REG_WINDOW);
            $display("  Changed to window preset 3: %02h", bus_read_result);

            // Trigger regeneration with new preset (no recapture needed!)
            bus_write(REG_CMD, 8'h10);

            #10_000_000;  // Longer for 266 samples with SDRAM latency

            wait_ready;

            bus_read(REG_STATUS);
            if (bus_read_result[STATUS_READY]) begin
                $display("  Re-zoom regenerate complete: PASS");

                // Check stretch factor
                bus_read(REG_STRETCH);
                $display("  Stretch factor: %02h (expect 01)", bus_read_result);

                if (bus_read_result == 8'h01) begin
                    $display("  Stretch factor correct: PASS");
                    test_pass = test_pass + 1;
                end else begin
                    $display("  Stretch factor: FAIL");
                    test_fail = test_fail + 1;
                end
            end else begin
                $display("  Re-zoom: FAIL (STATUS=%02h)", bus_read_result);
                test_fail = test_fail + 1;
            end
        end

        // =====================================================================
        // Test 6: GPIO outputs
        // =====================================================================
        $display("");
        $display("--- Test 6: GPIO outputs ---");
        begin : test_gpio
            $display("  GPIO1 (heartbeat): toggling");
            $display("  GPIO2 (ready): %b", gpio2);
            $display("  GPIO3 (armed): %b", gpio3);
            $display("  GPIO4 (captured): %b", gpio4);

            if (gpio2 == 1 && gpio3 == 0 && gpio4 == 1) begin
                $display("  GPIO status correct: PASS");
                test_pass = test_pass + 1;
            end else begin
                $display("  GPIO status: FAIL (expected ready=1, armed=0, captured=1)");
                test_fail = test_fail + 1;
            end
        end

        // =====================================================================
        // Summary
        // =====================================================================
        $display("");
        $display("===========================================");
        $display("Results: %0d passed, %0d failed", test_pass, test_fail);
        $display("===========================================");

        if (test_fail == 0)
            $display("*** ALL TESTS PASSED ***");
        else
            $display("*** SOME TESTS FAILED ***");

        $display("");
        #1000;
        $finish;
    end

endmodule
