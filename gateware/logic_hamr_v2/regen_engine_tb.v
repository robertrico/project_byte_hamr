// =============================================================================
// Regeneration Engine Testbench
// =============================================================================
//
// Tests the regeneration engine with:
// - Basic regeneration from capture buffer
// - All 8 channels processed in parallel
// - Display buffer output verification
// - Window preset support
//
// =============================================================================

`timescale 1ns / 100ps

module regen_engine_tb;

    // =========================================================================
    // Clock generation - 25 MHz (40ns period)
    // =========================================================================

    reg clk = 0;
    always #20 clk = ~clk;

    reg rst_n = 0;

    // =========================================================================
    // DUT Signals
    // =========================================================================

    reg        start;
    reg        soft_reset;
    reg  [1:0] window_preset;

    wire       busy;
    wire       done;

    wire       sdram_rd_req;
    wire [12:0] sdram_rd_addr;
    reg        sdram_rd_ready;
    reg  [7:0] sdram_rd_data;
    reg        sdram_rd_valid;

    wire       sdram_wr_req;
    wire [12:0] sdram_wr_addr;
    wire [7:0] sdram_wr_data;
    reg        sdram_wr_ready;

    // =========================================================================
    // SDRAM Memory Model
    // =========================================================================

    reg [7:0] capture_buf [0:511];    // Capture buffer (raw samples)
    reg [7:0] display_buf [0:511];    // Display buffer (regenerated)

    reg [8:0] read_addr_latched;
    reg [3:0] read_delay;

    integer j;
    initial begin
        for (j = 0; j < 512; j = j + 1) begin
            capture_buf[j] = 8'h00;
            display_buf[j] = 8'h00;
        end
    end

    // Handle SDRAM reads with delay
    always @(posedge clk) begin
        sdram_rd_valid <= 1'b0;

        if (sdram_rd_req && sdram_rd_ready) begin
            read_addr_latched <= sdram_rd_addr[8:0];
            read_delay <= 4'd4;
        end

        if (read_delay > 0) begin
            read_delay <= read_delay - 1'b1;
            if (read_delay == 1) begin
                sdram_rd_data <= capture_buf[read_addr_latched];
                sdram_rd_valid <= 1'b1;
            end
        end
    end

    // Handle SDRAM writes
    always @(posedge clk) begin
        if (sdram_wr_req && sdram_wr_ready) begin
            if (sdram_wr_addr >= 13'h200 && sdram_wr_addr < 13'h400) begin
                display_buf[sdram_wr_addr - 13'h200] <= sdram_wr_data;
            end
        end
    end

    // =========================================================================
    // Device Under Test
    // =========================================================================

    regen_engine dut (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .soft_reset(soft_reset),
        .window_preset(window_preset),
        .busy(busy),
        .done(done),
        .sdram_rd_req(sdram_rd_req),
        .sdram_rd_addr(sdram_rd_addr),
        .sdram_rd_ready(sdram_rd_ready),
        .sdram_rd_data(sdram_rd_data),
        .sdram_rd_valid(sdram_rd_valid),
        .sdram_wr_req(sdram_wr_req),
        .sdram_wr_addr(sdram_wr_addr),
        .sdram_wr_data(sdram_wr_data),
        .sdram_wr_ready(sdram_wr_ready)
    );

    // =========================================================================
    // Test Sequence
    // =========================================================================

    integer test_pass;
    integer test_fail;
    integer i, ch;

    initial begin
        $dumpfile("regen_engine_tb.vcd");
        $dumpvars(0, regen_engine_tb);

        test_pass = 0;
        test_fail = 0;

        // Initialize signals
        start = 0;
        soft_reset = 0;
        window_preset = 2'd0;  // 38 samples, stretch=7
        sdram_rd_ready = 1;
        sdram_wr_ready = 1;
        sdram_rd_data = 8'h00;
        sdram_rd_valid = 0;
        read_delay = 0;

        $display("===========================================");
        $display("Regeneration Engine Testbench");
        $display("===========================================");
        $display("");

        // Release reset
        #100;
        rst_n = 1;
        #100;

        // =====================================================================
        // Test 1: Basic Regeneration with Alternating Pattern
        // =====================================================================
        $display("--- Test 1: Basic Regeneration (Preset 0, 38 samples) ---");

        // Fill capture buffer with alternating pattern
        // Channel 7 (bit 7) alternates every sample: 0x80, 0x00, 0x80, ...
        // This should produce 7F/00 output bytes with stretch=7
        for (i = 0; i < 38; i = i + 1) begin
            capture_buf[i] = (i & 1) ? 8'h80 : 8'h00;  // Channel 7 alternates
        end

        // Start regeneration
        @(posedge clk);
        start = 1;
        @(posedge clk);
        start = 0;

        // Wait for completion
        i = 0;
        while (!done && i < 100_000) begin
            @(posedge clk);
            i = i + 1;
        end

        if (done) begin
            $display("  Regeneration complete: PASS (after %0d clocks)", i);
            test_pass = test_pass + 1;
        end else begin
            $display("  Regeneration complete: FAIL (timeout)");
            test_fail = test_fail + 1;
        end

        // Check display buffer for channel 7
        // With 38 samples, stretch=7: each sample becomes 7 pixels
        // Total pixels = 38 * 7 = 266 pixels = 38 bytes
        $display("");
        $display("  Channel 7 display buffer (first 10 bytes):");
        $write("    ");
        for (i = 0; i < 10; i = i + 1) begin
            $write("%02h ", display_buf[7*38 + i]);
        end
        $display("");

        // =====================================================================
        // Test 2: All Channels Pattern
        // =====================================================================
        $display("");
        $display("--- Test 2: All Channels Pattern ---");

        // Clear display buffer
        for (j = 0; j < 512; j = j + 1)
            display_buf[j] = 8'h00;

        // Fill capture buffer: each channel has its bit set for all samples
        // Channel 0: 0x01 all samples -> should produce 0x7F bytes
        // Channel 7: 0x80 all samples -> should produce 0x7F bytes
        for (i = 0; i < 38; i = i + 1) begin
            capture_buf[i] = 8'hFF;  // All channels high
        end

        // Start regeneration
        @(posedge clk);
        start = 1;
        @(posedge clk);
        start = 0;

        // Wait for completion
        i = 0;
        while (!done && i < 100_000) begin
            @(posedge clk);
            i = i + 1;
        end

        if (done) begin
            $display("  Regeneration complete: PASS");
            test_pass = test_pass + 1;
        end else begin
            $display("  Regeneration complete: FAIL");
            test_fail = test_fail + 1;
        end

        // Check that all channels have 0x7F output (all high input -> all high output)
        begin : check_all_high
            integer errors;
            errors = 0;
            for (ch = 0; ch < 8; ch = ch + 1) begin
                // First byte of each channel should be 0x7F (all bits set)
                if (display_buf[ch*38] != 8'h7F) begin
                    $display("  Channel %0d byte 0: FAIL (expected 7F, got %02h)", ch, display_buf[ch*38]);
                    errors = errors + 1;
                end
            end
            if (errors == 0) begin
                $display("  All channels have 0x7F output: PASS");
                test_pass = test_pass + 1;
            end else begin
                $display("  All channels 0x7F check: FAIL (%0d errors)", errors);
                test_fail = test_fail + 1;
            end
        end

        // =====================================================================
        // Test 3: Window Preset 3 (266 samples, stretch=1)
        // =====================================================================
        $display("");
        $display("--- Test 3: Window Preset 3 (266 samples, stretch=1) ---");

        // Clear display buffer
        for (j = 0; j < 512; j = j + 1)
            display_buf[j] = 8'h00;

        // Fill capture buffer with pattern
        for (i = 0; i < 266; i = i + 1) begin
            capture_buf[i] = (i < 133) ? 8'hFF : 8'h00;  // First half high, second half low
        end

        window_preset = 2'd3;  // 266 samples, stretch=1

        // Start regeneration
        @(posedge clk);
        start = 1;
        @(posedge clk);
        start = 0;

        // Wait for completion (longer for 266 samples)
        i = 0;
        while (!done && i < 500_000) begin
            @(posedge clk);
            i = i + 1;
        end

        if (done) begin
            $display("  Regeneration complete: PASS (after %0d clocks)", i);
            test_pass = test_pass + 1;
        end else begin
            $display("  Regeneration complete: FAIL (timeout)");
            test_fail = test_fail + 1;
        end

        // With stretch=1, each sample becomes 1 pixel
        // 266 samples -> 38 bytes (266/7 = 38)
        // First ~19 bytes should be 0x7F (high), rest should have transition
        $display("  Channel 0 display buffer (bytes 0, 18, 19, 37):");
        $display("    byte 0: %02h (expect ~7F)", display_buf[0]);
        $display("    byte 18: %02h (expect ~7F)", display_buf[18]);
        $display("    byte 19: %02h (transition area)", display_buf[19]);
        $display("    byte 37: %02h (expect ~00)", display_buf[37]);

        // =====================================================================
        // Test 4: Soft Reset
        // =====================================================================
        $display("");
        $display("--- Test 4: Soft Reset ---");

        window_preset = 2'd0;

        // Start regeneration
        @(posedge clk);
        start = 1;
        @(posedge clk);
        start = 0;

        // Wait a bit then reset
        #1000;

        if (busy) begin
            $display("  Engine busy after start: PASS");
        end

        @(posedge clk);
        soft_reset = 1;
        @(posedge clk);
        soft_reset = 0;
        #100;

        if (!busy && !done) begin
            $display("  Soft reset stops engine: PASS");
            test_pass = test_pass + 1;
        end else begin
            $display("  Soft reset: FAIL (busy=%b done=%b)", busy, done);
            test_fail = test_fail + 1;
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
