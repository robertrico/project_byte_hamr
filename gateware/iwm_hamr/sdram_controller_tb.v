// =============================================================================
// SDRAM Controller Testbench for IWM Hamr
// =============================================================================
//
// Direct testbench for sdram_controller module with:
// - 25 MHz system clock
// - Simple SDRAM behavioral model (32 x 16-bit locations, CAS latency 2)
// - Write/read helper tasks
// - PASS/FAIL assertions with summary
//
// SDRAM model limitations:
//   Only 32 locations, indexed by column address bits [4:0]. Bank and row
//   are not part of the model index — the model verifies the controller
//   issues correct ACTIVATE/bank commands via $display trace, but does not
//   enforce bank/row isolation. Tests are designed to use non-overlapping
//   column addresses to avoid aliasing.
//
// =============================================================================

`timescale 1ns / 1ps

module sdram_controller_tb;

    // =========================================================================
    // Clock generation - 25 MHz (40ns period)
    // =========================================================================

    reg clk = 0;
    always #20 clk = ~clk;

    // =========================================================================
    // DUT Signals
    // =========================================================================

    reg         rst_n = 0;
    reg         pause_refresh = 0;
    reg         req = 0;
    reg         req_write = 0;
    reg  [25:0] req_addr = 0;
    reg  [15:0] req_wdata = 0;
    wire        init_done;
    wire        req_ready;
    wire [15:0] req_rdata;
    wire        req_rdata_valid;

    // SDRAM physical signals
    wire        sdram_clk;
    wire        sdram_cke;
    wire        sdram_ncs;
    wire        sdram_nras;
    wire        sdram_ncas;
    wire        sdram_nwe;
    wire        sdram_dqm0;
    wire        sdram_dqm1;
    wire        sdram_ba0, sdram_ba1;
    wire [12:0] sdram_a;

    // SDRAM DQ bus - individual wires for iverilog tristate compatibility
    wire sdram_d0,  sdram_d1,  sdram_d2,  sdram_d3;
    wire sdram_d4,  sdram_d5,  sdram_d6,  sdram_d7;
    wire sdram_d8,  sdram_d9,  sdram_d10, sdram_d11;
    wire sdram_d12, sdram_d13, sdram_d14, sdram_d15;

    // Reconstruct 16-bit bus from individual wires
    wire [15:0] sdram_dq_bus = {sdram_d15, sdram_d14, sdram_d13, sdram_d12,
                                sdram_d11, sdram_d10, sdram_d9,  sdram_d8,
                                sdram_d7,  sdram_d6,  sdram_d5,  sdram_d4,
                                sdram_d3,  sdram_d2,  sdram_d1,  sdram_d0};

    // =========================================================================
    // Simple SDRAM Behavioral Model
    // =========================================================================

    reg [15:0] sdram_mem [0:31];   // 32 locations for testing
    reg [15:0] sdram_dq_out;
    reg        sdram_dq_oe = 0;
    reg [4:0]  sdram_read_col;
    reg [2:0]  sdram_cas_delay;

    // Drive individual DQ pins from model
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
    wire [3:0] sdram_cmd_w = {sdram_ncs, sdram_nras, sdram_ncas, sdram_nwe};
    localparam CMD_NOP       = 4'b0111;
    localparam CMD_ACTIVE    = 4'b0011;
    localparam CMD_READ      = 4'b0101;
    localparam CMD_WRITE     = 4'b0100;
    localparam CMD_PRECHARGE = 4'b0010;
    localparam CMD_REFRESH   = 4'b0001;
    localparam CMD_LOAD_MODE = 4'b0000;

    reg [12:0] active_row;
    reg [1:0]  active_bank;
    reg        refresh_seen;

    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1)
            sdram_mem[i] = 16'h0000;
        sdram_cas_delay = 0;
        sdram_dq_out = 16'h0000;
        active_row = 13'd0;
        active_bank = 2'd0;
        refresh_seen = 0;
    end

    always @(posedge sdram_clk) begin
        if (sdram_cas_delay > 0) begin
            sdram_cas_delay <= sdram_cas_delay - 1;
            if (sdram_cas_delay == 1) begin
                sdram_dq_oe <= 1;
                sdram_dq_out <= sdram_mem[sdram_read_col];
            end
        end else begin
            sdram_dq_oe <= 0;
        end

        case (sdram_cmd_w)
            CMD_ACTIVE: begin
                active_row <= sdram_a;
                active_bank <= {sdram_ba1, sdram_ba0};
                $display("[%0t] SDRAM ACTIVE: bank=%0d row=%0d",
                         $time, {sdram_ba1, sdram_ba0}, sdram_a);
            end
            CMD_WRITE: begin
                sdram_mem[sdram_a[4:0]] <= sdram_dq_bus;
                $display("[%0t] SDRAM WRITE: bank=%0d col=%0d data=%04h",
                         $time, {sdram_ba1, sdram_ba0}, sdram_a[9:0], sdram_dq_bus);
            end
            CMD_READ: begin
                sdram_read_col <= sdram_a[4:0];
                // CAS latency 2: set counter to 1 because model sees the
                // READ command 1 cycle late (DUT NBA delay), so effective
                // latency is counter + 1 = 2 cycles from DUT perspective.
                sdram_cas_delay <= 1;
                $display("[%0t] SDRAM READ:  bank=%0d col=%0d (data=%04h)",
                         $time, {sdram_ba1, sdram_ba0}, sdram_a[9:0],
                         sdram_mem[sdram_a[4:0]]);
            end
            CMD_REFRESH: begin
                refresh_seen <= 1;
                $display("[%0t] SDRAM REFRESH", $time);
            end
            CMD_PRECHARGE: begin
                $display("[%0t] SDRAM PRECHARGE (A10=%0b)", $time, sdram_a[10]);
            end
            CMD_LOAD_MODE: begin
                $display("[%0t] SDRAM LOAD MODE: A=%013b", $time, sdram_a);
            end
        endcase
    end

    // =========================================================================
    // Device Under Test
    // =========================================================================

    sdram_controller dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .init_done      (init_done),
        .pause_refresh  (pause_refresh),
        .req            (req),
        .req_write      (req_write),
        .req_addr       (req_addr),
        .req_wdata      (req_wdata),
        .req_ready      (req_ready),
        .req_rdata      (req_rdata),
        .req_rdata_valid(req_rdata_valid),
        .SDRAM_CLK      (sdram_clk),
        .SDRAM_CKE      (sdram_cke),
        .SDRAM_nCS      (sdram_ncs),
        .SDRAM_nRAS     (sdram_nras),
        .SDRAM_nCAS     (sdram_ncas),
        .SDRAM_nWE      (sdram_nwe),
        .SDRAM_DQM0     (sdram_dqm0),
        .SDRAM_DQM1     (sdram_dqm1),
        .SDRAM_BA0      (sdram_ba0),
        .SDRAM_BA1      (sdram_ba1),
        .SDRAM_A        (sdram_a),
        .SDRAM_DQ       (sdram_dq_bus)
    );

    // =========================================================================
    // Helper Tasks
    // =========================================================================

    // Captured read data
    reg [15:0] captured_rdata;

    // Write a 16-bit word to a 26-bit byte address.
    // Holds req asserted while waiting for req_ready so the controller
    // sees both signals on the same posedge (avoids refresh stealing the slot).
    task sdram_write;
        input [25:0] addr;
        input [15:0] data;
        integer timeout;
        begin
            // Drive request signals immediately
            @(posedge clk);
            req <= 1'b1;
            req_write <= 1'b1;
            req_addr <= addr;
            req_wdata <= data;

            // Wait for controller to accept (req_ready goes low = latched)
            timeout = 0;
            @(posedge clk);  // Let NBA settle
            while (!req_ready && timeout < 10000) begin
                @(posedge clk);
                timeout = timeout + 1;
            end
            // req_ready is now high, controller will sample req on this edge
            // Wait one more cycle — controller latches and drops req_ready
            @(posedge clk);
            req <= 1'b0;
            req_write <= 1'b0;

            // Wait for write to complete (req_ready returns high)
            timeout = 0;
            while (!req_ready && timeout < 10000) begin
                @(posedge clk);
                timeout = timeout + 1;
            end
            if (timeout >= 10000) begin
                $display("[%0t] ERROR: write timeout", $time);
            end
        end
    endtask

    // Read a 16-bit word from a 26-bit byte address.
    // Holds req asserted while waiting for req_ready.
    task sdram_read;
        input [25:0] addr;
        integer timeout;
        begin
            // Drive request signals immediately
            @(posedge clk);
            req <= 1'b1;
            req_write <= 1'b0;
            req_addr <= addr;

            // Wait for controller to accept (req_ready high + req high on same edge)
            timeout = 0;
            @(posedge clk);  // Let NBA settle
            while (!req_ready && timeout < 10000) begin
                @(posedge clk);
                timeout = timeout + 1;
            end
            // Controller will latch on this edge, deassert req next cycle
            @(posedge clk);
            req <= 1'b0;

            // Wait for rdata_valid
            timeout = 0;
            while (!req_rdata_valid && timeout < 10000) begin
                @(posedge clk);
                timeout = timeout + 1;
            end
            if (timeout >= 10000) begin
                $display("[%0t] ERROR: read timeout waiting for rdata_valid", $time);
            end

            captured_rdata = req_rdata;
        end
    endtask

    // =========================================================================
    // Test Sequence
    // =========================================================================

    integer pass_count;
    integer fail_count;

    // Latched bank value from ACTIVATE command, used by bank verification
    reg [1:0] last_activate_bank;
    always @(posedge sdram_clk) begin
        if (sdram_cmd_w == CMD_ACTIVE)
            last_activate_bank <= {sdram_ba1, sdram_ba0};
    end

    initial begin
        $dumpfile("sdram_controller_tb.vcd");
        $dumpvars(0, sdram_controller_tb);

        pass_count = 0;
        fail_count = 0;
        last_activate_bank = 2'd0;

        $display("==============================================");
        $display("IWM Hamr SDRAM Controller Testbench");
        $display("AS4C32M16SB-7TCNTR (64MB, 32Mx16)");
        $display("==============================================");
        $display("");

        // Release reset after a few clocks
        #100;
        rst_n = 1;

        // =====================================================================
        // Test 1: Init sequence - wait for init_done
        // =====================================================================
        $display("--- Test 1: Initialization sequence ---");
        begin : test_init
            integer timeout;
            timeout = 0;
            while (!init_done && timeout < 300000) begin
                @(posedge clk);
                timeout = timeout + 1;
            end
            if (init_done) begin
                $display("  PASS: init_done asserted after %0d clocks", timeout);
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL: init_done never asserted (timeout)");
                fail_count = fail_count + 1;
                $finish;
            end
        end

        // Wait for controller to reach IDLE and be ready
        #200;

        // =====================================================================
        // Test 2: Single write + read (0xCAFE @ addr 0)
        // Column 0 in model
        // =====================================================================
        $display("");
        $display("--- Test 2: Single write + read (0xCAFE @ 0x000000) ---");
        begin : test_single
            sdram_write(26'h0000000, 16'hCAFE);
            sdram_read(26'h0000000);
            if (captured_rdata == 16'hCAFE) begin
                $display("  PASS: read back 0x%04h", captured_rdata);
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL: expected 0xCAFE, got 0x%04h", captured_rdata);
                fail_count = fail_count + 1;
            end
        end

        // =====================================================================
        // Test 3: 16-bit data integrity (0xDEAD @ addr 2)
        // Byte addr 2 -> col 1 in model
        // =====================================================================
        $display("");
        $display("--- Test 3: 16-bit data (0xDEAD @ 0x000002) ---");
        begin : test_16bit
            sdram_write(26'h0000002, 16'hDEAD);
            sdram_read(26'h0000002);
            if (captured_rdata == 16'hDEAD) begin
                $display("  PASS: read back 0x%04h (both bytes correct)", captured_rdata);
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL: expected 0xDEAD, got 0x%04h", captured_rdata);
                fail_count = fail_count + 1;
            end
        end

        // =====================================================================
        // Test 4: Different banks
        // Bank 0, col 16 -> model index 16. Bank 1, col 17 -> model index 17.
        // Using different columns avoids model aliasing.
        // Verify the controller issues correct bank select on ACTIVATE.
        // =====================================================================
        $display("");
        $display("--- Test 4: Different banks (bank 0 vs bank 1) ---");
        begin : test_banks
            // Bank 0, row 0, col 16: byte addr = {2'b00, 13'd0, 10'd16, 1'b0}
            sdram_write({2'b00, 13'd0, 10'd16, 1'b0}, 16'hB0B0);
            // Bank 1, row 0, col 17: byte addr = {2'b01, 13'd0, 10'd17, 1'b0}
            sdram_write({2'b01, 13'd0, 10'd17, 1'b0}, 16'hB1B1);

            // Read back bank 0 value
            sdram_read({2'b00, 13'd0, 10'd16, 1'b0});
            if (captured_rdata == 16'hB0B0) begin
                $display("  PASS: bank 0 read back 0x%04h", captured_rdata);
            end else begin
                $display("  FAIL: bank 0 expected 0xB0B0, got 0x%04h", captured_rdata);
                fail_count = fail_count + 1;
            end

            // Read back bank 1 value
            sdram_read({2'b01, 13'd0, 10'd17, 1'b0});
            if (captured_rdata == 16'hB1B1) begin
                $display("  PASS: bank 1 read back 0x%04h", captured_rdata);
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL: bank 1 expected 0xB1B1, got 0x%04h", captured_rdata);
                fail_count = fail_count + 1;
            end
        end

        // =====================================================================
        // Test 5: Different rows within same bank
        // Uses different columns (18 and 19) so the model stores them
        // separately. The $display trace confirms different row addresses
        // are issued on the ACTIVATE commands.
        // =====================================================================
        $display("");
        $display("--- Test 5: Different rows in same bank ---");
        begin : test_rows
            // Row 0, col 18, bank 0
            sdram_write({2'b00, 13'd0, 10'd18, 1'b0}, 16'h1234);
            // Row 1, col 19, bank 0
            sdram_write({2'b00, 13'd1, 10'd19, 1'b0}, 16'h5678);

            // Read back row 0 value
            sdram_read({2'b00, 13'd0, 10'd18, 1'b0});
            if (captured_rdata == 16'h1234) begin
                $display("  PASS: row 0, col 18 read back 0x%04h", captured_rdata);
            end else begin
                $display("  FAIL: row 0, col 18 expected 0x1234, got 0x%04h", captured_rdata);
                fail_count = fail_count + 1;
            end

            // Read back row 1 value
            sdram_read({2'b00, 13'd1, 10'd19, 1'b0});
            if (captured_rdata == 16'h5678) begin
                $display("  PASS: row 1, col 19 read back 0x%04h", captured_rdata);
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL: row 1, col 19 expected 0x5678, got 0x%04h", captured_rdata);
                fail_count = fail_count + 1;
            end
        end

        // =====================================================================
        // Test 6: Sequential burst - 8 consecutive 16-bit words
        // Uses columns 0-7 (byte addresses 0x00, 0x02, ... 0x0E)
        // Note: overwrites Test 2 data at col 0 and Test 3 data at col 1
        // =====================================================================
        $display("");
        $display("--- Test 6: Sequential burst (8 words at cols 0-7) ---");
        begin : test_burst
            integer j;
            integer burst_pass;
            burst_pass = 0;

            // Write 8 words at col 0..7 (byte addr j*2)
            for (j = 0; j < 8; j = j + 1) begin
                sdram_write({2'b00, 13'd0, j[9:0], 1'b0}, 16'hA000 + j[15:0]);
            end

            // Read all back and verify
            for (j = 0; j < 8; j = j + 1) begin
                sdram_read({2'b00, 13'd0, j[9:0], 1'b0});
                if (captured_rdata == (16'hA000 + j[15:0])) begin
                    burst_pass = burst_pass + 1;
                end else begin
                    $display("  word %0d: FAIL (expected 0x%04h, got 0x%04h)",
                             j, 16'hA000 + j[15:0], captured_rdata);
                end
            end

            if (burst_pass == 8) begin
                $display("  PASS: all 8 words verified correctly");
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL: %0d/8 words correct", burst_pass);
                fail_count = fail_count + 1;
            end
        end

        // =====================================================================
        // Test 7: Refresh during idle
        // =====================================================================
        $display("");
        $display("--- Test 7: Refresh during idle ---");
        begin : test_refresh
            integer timeout;

            // Reset the flag
            refresh_seen = 0;

            // Wait up to 300 clocks (~12us) for a refresh to occur
            timeout = 0;
            while (!refresh_seen && timeout < 300) begin
                @(posedge clk);
                timeout = timeout + 1;
            end

            if (refresh_seen) begin
                $display("  PASS: refresh occurred within %0d clocks", timeout);
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL: no refresh seen within 300 clocks");
                fail_count = fail_count + 1;
            end
        end

        // =====================================================================
        // Summary
        // =====================================================================
        $display("");
        $display("==============================================");
        $display("%0d tests passed, %0d failed", pass_count, fail_count);
        $display("==============================================");

        if (fail_count == 0)
            $display("*** ALL TESTS PASSED ***");
        else
            $display("*** SOME TESTS FAILED ***");

        $display("");
        #1000;
        $finish;
    end

endmodule
