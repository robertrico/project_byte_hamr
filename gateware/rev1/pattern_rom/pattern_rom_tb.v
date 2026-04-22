// =============================================================================
// Byte Hamr Pattern ROM - Testbench (8 Channels)
// =============================================================================
//
// Tests SDRAM-backed pattern ROM with 8 channels:
// - SDRAM init and pattern loading (304 bytes)
// - Scratch register loopback
// - Channel 0 pattern verification (00 7F 00 7F...)
// - Channel 1 pattern verification (7F 00 7F 00...)
// - Channel independence (same addr, different channels)
// - Full pattern verification across all channels
//
// =============================================================================

`timescale 1ns / 100ps

module pattern_rom_tb;

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

    // Apple II Data bus
    wire [7:0] apple_data;
    reg  [7:0] apple_data_drv = 8'h00;
    reg        apple_data_oe = 0;
    assign apple_data = apple_data_oe ? apple_data_drv : 8'hZZ;

    // Apple II Control
    reg        r_nw = 1;
    reg        ndevice_select = 1;
    reg        nio_select = 1;
    reg        nio_strobe = 1;
    reg        rdy = 1;
    reg        sig_7m = 0;
    reg        q3 = 0;
    reg        usync = 1;

    // GPIO
    wire       gpio1, gpio2, gpio3, gpio4;
    wire       gpio5, gpio6, gpio7, gpio8;
    wire       gpio9, gpio10, gpio11, gpio12;

    // =========================================================================
    // SDRAM Model - 512 locations (more than 304 needed)
    // =========================================================================

    reg [7:0]  sdram_mem [0:511];
    reg [15:0] sdram_dq_out;
    reg        sdram_dq_oe = 0;
    reg [8:0]  sdram_read_idx;
    reg [2:0]  sdram_cas_delay;

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
        for (j = 0; j < 512; j = j + 1)
            sdram_mem[j] = 8'h00;
        sdram_cas_delay = 0;
    end

    // Extract column address (bits 9:1 since A0 is used for byte select)
    wire [8:0] sdram_col = sdram_addr_w[9:1];

    always @(posedge sdram_clk) begin
        if (sdram_cas_delay > 0) begin
            sdram_cas_delay <= sdram_cas_delay - 1;
            if (sdram_cas_delay == 1) begin
                sdram_dq_oe <= 1;
                sdram_dq_out <= {8'h00, sdram_mem[sdram_read_idx]};
            end
        end else begin
            sdram_dq_oe <= 0;
        end

        case (sdram_cmd)
            CMD_WRITE: begin
                sdram_mem[sdram_col] <= sdram_dq_in_w[7:0];
                $display("[%0t] SDRAM WRITE: addr=%0d data=%02h",
                         $time, sdram_col, sdram_dq_in_w[7:0]);
            end
            CMD_READ: begin
                sdram_read_idx <= sdram_col;
                sdram_cas_delay <= 1;  // CAS latency 2 (NBA compensation)
                $display("[%0t] SDRAM READ:  addr=%0d (data=%02h)",
                         $time, sdram_col, sdram_mem[sdram_col]);
            end
        endcase
    end

    // =========================================================================
    // Device Under Test
    // =========================================================================

    pattern_rom_top dut (
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
    // Bus Transaction Tasks
    // =========================================================================

    // Register offsets (new map)
    localparam REG_CHANNEL = 4'h0;
    localparam REG_ADDR    = 4'h1;
    localparam REG_DATA    = 4'h2;
    localparam REG_CMD     = 4'h3;
    localparam REG_STATUS  = 4'h4;

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
    task bus_read;
        input [3:0] offset;
        begin
            @(posedge phi0);
            apple_addr <= {12'hC0C, offset};
            r_nw <= 1'b1;
            apple_data_oe <= 1'b0;
            ndevice_select <= 1'b0;

            @(negedge phi0);
            #100;
            bus_read_result = apple_data;

            ndevice_select <= 1'b1;
            apple_addr <= 16'hFFFF;

            @(posedge phi0);
        end
    endtask

    // Poll STATUS until not busy
    task wait_ready;
        integer timeout;
        begin
            timeout = 0;
            bus_read_result = 8'hFF;
            while (bus_read_result[0] == 1'b1 && timeout < 1000) begin
                bus_read(REG_STATUS);
                timeout = timeout + 1;
            end
            if (timeout >= 1000) begin
                $display("[%0t] ERROR: Ready timeout!", $time);
            end
        end
    endtask

    // High-level: Read byte from pattern ROM
    reg [7:0] rom_read_result;
    task rom_read;
        input [2:0] channel;
        input [5:0] addr;
        begin
            wait_ready;
            bus_write(REG_CHANNEL, {5'b0, channel});
            bus_write(REG_ADDR, {2'b0, addr});
            bus_write(REG_CMD, 8'h02);  // CMD = READ
            wait_ready;
            bus_read(REG_DATA);
            rom_read_result = bus_read_result;
        end
    endtask

    // Expected pattern value: (channel + byte_idx) odd = 7F, even = 00
    function [7:0] expected_pattern;
        input [2:0] channel;
        input [5:0] byte_idx;
        begin
            if ((channel[0] ^ byte_idx[0]) == 1'b1)
                expected_pattern = 8'h7F;
            else
                expected_pattern = 8'h00;
        end
    endfunction

    // =========================================================================
    // Test Sequence
    // =========================================================================

    integer test_pass;
    integer test_fail;

    initial begin
        $dumpfile("pattern_rom_tb.vcd");
        $dumpvars(0, pattern_rom_tb);

        test_pass = 0;
        test_fail = 0;

        $display("===========================================");
        $display("Byte Hamr Pattern ROM Testbench (8 Channels)");
        $display("===========================================");
        $display("");

        // ---- Wait for pattern loading (304 bytes) ----
        $display("[%0t] Waiting for SDRAM init and pattern loading...", $time);
        #2_500_000;  // 2.5ms should be enough for init + 304 writes

        begin : wait_ready_init
            integer timeout;
            timeout = 0;
            bus_read_result = 8'h00;
            while (bus_read_result[1] == 1'b0 && timeout < 500) begin
                bus_read(REG_STATUS);
                timeout = timeout + 1;
            end
            if (bus_read_result[1]) begin
                $display("[%0t] Pattern loading complete (STATUS=%02h)", $time, bus_read_result);
            end else begin
                $display("[%0t] ERROR: Pattern loading timeout!", $time);
                $finish;
            end
        end

        // ---- Test 1: Scratch register loopback ----
        $display("");
        $display("--- Test 1: Scratch register loopback ---");
        begin : test_scratch
            bus_write(4'h5, 8'hA5);
            bus_read(4'h5);
            if (bus_read_result == 8'hA5) begin
                $display("  REG $05: PASS (wrote $A5, read $%02h)", bus_read_result);
                test_pass = test_pass + 1;
            end else begin
                $display("  REG $05: FAIL (wrote $A5, read $%02h)", bus_read_result);
                test_fail = test_fail + 1;
            end
        end

        // ---- Test 2: Channel 0 byte 0 (expect $00) ----
        $display("");
        $display("--- Test 2: Channel 0, Byte 0 (expect $00) ---");
        begin : test_ch0_b0
            rom_read(3'd0, 6'd0);
            if (rom_read_result == 8'h00) begin
                $display("  Ch0 Byte0: PASS ($%02h)", rom_read_result);
                test_pass = test_pass + 1;
            end else begin
                $display("  Ch0 Byte0: FAIL (expected $00, got $%02h)", rom_read_result);
                test_fail = test_fail + 1;
            end
        end

        // ---- Test 3: Channel 0 byte 1 (expect $7F) ----
        $display("");
        $display("--- Test 3: Channel 0, Byte 1 (expect $7F) ---");
        begin : test_ch0_b1
            rom_read(3'd0, 6'd1);
            if (rom_read_result == 8'h7F) begin
                $display("  Ch0 Byte1: PASS ($%02h)", rom_read_result);
                test_pass = test_pass + 1;
            end else begin
                $display("  Ch0 Byte1: FAIL (expected $7F, got $%02h)", rom_read_result);
                test_fail = test_fail + 1;
            end
        end

        // ---- Test 4: Channel 1 byte 0 (expect $7F - odd channel starts with 7F) ----
        $display("");
        $display("--- Test 4: Channel 1, Byte 0 (expect $7F) ---");
        begin : test_ch1_b0
            rom_read(3'd1, 6'd0);
            if (rom_read_result == 8'h7F) begin
                $display("  Ch1 Byte0: PASS ($%02h)", rom_read_result);
                test_pass = test_pass + 1;
            end else begin
                $display("  Ch1 Byte0: FAIL (expected $7F, got $%02h)", rom_read_result);
                test_fail = test_fail + 1;
            end
        end

        // ---- Test 5: Channel 1 byte 1 (expect $00) ----
        $display("");
        $display("--- Test 5: Channel 1, Byte 1 (expect $00) ---");
        begin : test_ch1_b1
            rom_read(3'd1, 6'd1);
            if (rom_read_result == 8'h00) begin
                $display("  Ch1 Byte1: PASS ($%02h)", rom_read_result);
                test_pass = test_pass + 1;
            end else begin
                $display("  Ch1 Byte1: FAIL (expected $00, got $%02h)", rom_read_result);
                test_fail = test_fail + 1;
            end
        end

        // ---- Test 6: Channel independence - all 8 channels, byte 0 ----
        $display("");
        $display("--- Test 6: Channel independence (all channels, byte 0) ---");
        begin : test_independence
            integer ch;
            integer pass_cnt;
            reg [7:0] expected;
            pass_cnt = 0;

            for (ch = 0; ch < 8; ch = ch + 1) begin
                rom_read(ch[2:0], 6'd0);
                expected = (ch[0] == 1'b1) ? 8'h7F : 8'h00;
                if (rom_read_result == expected) begin
                    pass_cnt = pass_cnt + 1;
                end else begin
                    $display("  Ch%0d Byte0: FAIL (expected $%02h, got $%02h)",
                             ch, expected, rom_read_result);
                end
            end
            $display("  Channel independence: %0d/8 passed", pass_cnt);
            if (pass_cnt == 8) begin
                test_pass = test_pass + 1;
            end else begin
                test_fail = test_fail + 1;
            end
        end

        // ---- Test 7: Verify all 38 bytes of channel 0 ----
        $display("");
        $display("--- Test 7: Verify all 38 bytes of channel 0 ---");
        begin : test_ch0_all
            integer k;
            integer pass_cnt;
            reg [7:0] expected;
            pass_cnt = 0;

            for (k = 0; k < 38; k = k + 1) begin
                rom_read(3'd0, k[5:0]);
                expected = expected_pattern(3'd0, k[5:0]);
                if (rom_read_result == expected) begin
                    pass_cnt = pass_cnt + 1;
                end else begin
                    $display("  Byte %0d: FAIL (expected $%02h, got $%02h)",
                             k, expected, rom_read_result);
                end
            end
            $display("  Channel 0 verification: %0d/38 passed", pass_cnt);
            if (pass_cnt == 38) begin
                test_pass = test_pass + 1;
            end else begin
                test_fail = test_fail + 1;
            end
        end

        // ---- Test 8: Verify all 38 bytes of channel 1 ----
        $display("");
        $display("--- Test 8: Verify all 38 bytes of channel 1 ---");
        begin : test_ch1_all
            integer k;
            integer pass_cnt;
            reg [7:0] expected;
            pass_cnt = 0;

            for (k = 0; k < 38; k = k + 1) begin
                rom_read(3'd1, k[5:0]);
                expected = expected_pattern(3'd1, k[5:0]);
                if (rom_read_result == expected) begin
                    pass_cnt = pass_cnt + 1;
                end else begin
                    $display("  Byte %0d: FAIL (expected $%02h, got $%02h)",
                             k, expected, rom_read_result);
                end
            end
            $display("  Channel 1 verification: %0d/38 passed", pass_cnt);
            if (pass_cnt == 38) begin
                test_pass = test_pass + 1;
            end else begin
                test_fail = test_fail + 1;
            end
        end

        // ---- Test 9: Random access across channels ----
        $display("");
        $display("--- Test 9: Random access test ---");
        begin : test_random
            integer pass_cnt;
            reg [7:0] expected;
            pass_cnt = 0;

            // Channel 7, byte 37 (last byte of last channel)
            rom_read(3'd7, 6'd37);
            expected = expected_pattern(3'd7, 6'd37);
            if (rom_read_result == expected) begin
                $display("  Ch7 Byte37: PASS ($%02h)", rom_read_result);
                pass_cnt = pass_cnt + 1;
            end else begin
                $display("  Ch7 Byte37: FAIL (expected $%02h, got $%02h)", expected, rom_read_result);
            end

            // Channel 4, byte 20
            rom_read(3'd4, 6'd20);
            expected = expected_pattern(3'd4, 6'd20);
            if (rom_read_result == expected) begin
                $display("  Ch4 Byte20: PASS ($%02h)", rom_read_result);
                pass_cnt = pass_cnt + 1;
            end else begin
                $display("  Ch4 Byte20: FAIL (expected $%02h, got $%02h)", expected, rom_read_result);
            end

            // Channel 3, byte 15
            rom_read(3'd3, 6'd15);
            expected = expected_pattern(3'd3, 6'd15);
            if (rom_read_result == expected) begin
                $display("  Ch3 Byte15: PASS ($%02h)", rom_read_result);
                pass_cnt = pass_cnt + 1;
            end else begin
                $display("  Ch3 Byte15: FAIL (expected $%02h, got $%02h)", expected, rom_read_result);
            end

            if (pass_cnt == 3) begin
                test_pass = test_pass + 1;
            end else begin
                test_fail = test_fail + 1;
            end
        end

        // ---- Summary ----
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

    // =========================================================================
    // Monitor
    // =========================================================================

    reg prev_gpio2 = 0;
    reg prev_gpio3 = 0;

    always @(posedge clk_25mhz) begin
        if (gpio2 && !prev_gpio2)
            $display("[%0t] GPIO2: Pattern ROM ready", $time);
        if (!gpio3 && prev_gpio3)
            $display("[%0t] GPIO3: Pattern loading complete", $time);
        prev_gpio2 <= gpio2;
        prev_gpio3 <= gpio3;
    end

endmodule
