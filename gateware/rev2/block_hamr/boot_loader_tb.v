// boot_loader_tb.v — Testbench for boot_loader module
// Tests boot completion, data integrity, addressing, odd byte count, boot_done persistence

`timescale 1ns / 1ps

module boot_loader_tb;

    // =========================================================================
    // Clock and reset
    // =========================================================================

    reg clk;
    reg rst_n;

    initial clk = 0;
    always #20 clk = ~clk;  // 25 MHz = 40ns period

    // =========================================================================
    // Test result tracking
    // =========================================================================

    integer pass_count;
    integer fail_count;
    integer i;

    // =========================================================================
    // Signals for boot_loader under test
    // =========================================================================

    reg         sdram_init_done;
    wire        boot_done;
    wire        flash_start;
    wire [23:0] flash_addr;
    wire [23:0] flash_count;
    reg         flash_busy;
    reg  [7:0]  flash_data;
    reg         flash_data_valid;
    wire        flash_data_ready;
    wire        sdram_req;
    wire        sdram_req_write;
    wire [25:0] sdram_req_addr;
    wire [15:0] sdram_req_wdata;
    reg         sdram_req_ready;

    // =========================================================================
    // Flash reader model
    // =========================================================================
    // After start, waits 2 clocks then outputs sequential bytes (0x00, 0x01, ...)
    // one per clock when data_ready is asserted. Respects backpressure.

    reg [23:0] flash_bytes_left;
    reg [7:0]  flash_pattern;
    reg [1:0]  flash_startup_delay;
    reg        flash_active;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            flash_busy          <= 1'b0;
            flash_data          <= 8'd0;
            flash_data_valid    <= 1'b0;
            flash_bytes_left    <= 24'd0;
            flash_pattern       <= 8'd0;
            flash_startup_delay <= 2'd0;
            flash_active        <= 1'b0;
        end else begin
            flash_data_valid <= 1'b0;  // Default: clear pulse

            if (flash_start && !flash_active) begin
                flash_busy          <= 1'b1;
                flash_bytes_left    <= flash_count;
                flash_pattern       <= 8'd0;
                flash_startup_delay <= 2'd2;
                flash_active        <= 1'b1;
            end else if (flash_active) begin
                if (flash_startup_delay != 0) begin
                    flash_startup_delay <= flash_startup_delay - 2'd1;
                end else if (flash_bytes_left != 0 && flash_data_ready) begin
                    flash_data       <= flash_pattern;
                    flash_data_valid <= 1'b1;
                    flash_pattern    <= flash_pattern + 8'd1;  // Wraps at 0xFF
                    flash_bytes_left <= flash_bytes_left - 24'd1;
                end else if (flash_bytes_left == 0) begin
                    flash_busy   <= 1'b0;
                    flash_active <= 1'b0;
                end
            end
        end
    end

    // =========================================================================
    // SDRAM controller model
    // =========================================================================
    // Simple memory: accepts writes with 1-clock ready latency

    reg [15:0] sdram_mem [0:511];  // 1KB (512 x 16-bit words)
    reg [25:0] sdram_last_addr;
    reg        sdram_ready_delay;

    // Track all write addresses for verification
    reg [25:0] sdram_write_addrs [0:255];
    integer    sdram_write_count;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sdram_req_ready  <= 1'b0;
            sdram_last_addr  <= 26'd0;
            sdram_ready_delay <= 1'b0;
            sdram_write_count <= 0;
        end else begin
            if (sdram_req && sdram_req_write) begin
                if (!sdram_ready_delay) begin
                    // First cycle of request: delay one clock
                    sdram_ready_delay <= 1'b1;
                    sdram_req_ready   <= 1'b0;
                end else begin
                    // Second cycle: accept the write
                    sdram_mem[sdram_req_addr[9:1]] <= sdram_req_wdata;
                    sdram_last_addr <= sdram_req_addr;
                    sdram_write_addrs[sdram_write_count] <= sdram_req_addr;
                    sdram_write_count <= sdram_write_count + 1;
                    sdram_req_ready   <= 1'b1;
                    sdram_ready_delay <= 1'b0;
                end
            end else begin
                sdram_req_ready   <= 1'b0;
                sdram_ready_delay <= 1'b0;
            end
        end
    end

    // =========================================================================
    // DUT instantiation — Even byte count test (32 bytes)
    // =========================================================================

    boot_loader #(
        .FLASH_OFFSET(24'h400000),
        .IMAGE_SIZE(24'd32)
    ) uut (
        .clk(clk),
        .rst_n(rst_n),
        .sdram_init_done(sdram_init_done),
        .boot_done(boot_done),
        .flash_start(flash_start),
        .flash_addr(flash_addr),
        .flash_count(flash_count),
        .flash_busy(flash_busy),
        .flash_data(flash_data),
        .flash_data_valid(flash_data_valid),
        .flash_data_ready(flash_data_ready),
        .sdram_req(sdram_req),
        .sdram_req_write(sdram_req_write),
        .sdram_req_addr(sdram_req_addr),
        .sdram_req_wdata(sdram_req_wdata),
        .sdram_req_ready(sdram_req_ready)
    );

    // =========================================================================
    // Second DUT for odd byte count test (33 bytes)
    // =========================================================================

    reg         sdram_init_done_odd;
    wire        boot_done_odd;
    wire        flash_start_odd;
    wire [23:0] flash_addr_odd;
    wire [23:0] flash_count_odd;
    reg         flash_busy_odd;
    reg  [7:0]  flash_data_odd;
    reg         flash_data_valid_odd;
    wire        flash_data_ready_odd;
    wire        sdram_req_odd;
    wire        sdram_req_write_odd;
    wire [25:0] sdram_req_addr_odd;
    wire [15:0] sdram_req_wdata_odd;
    reg         sdram_req_ready_odd;

    // Flash model for odd DUT
    reg [23:0] flash_bytes_left_odd;
    reg [7:0]  flash_pattern_odd;
    reg [1:0]  flash_startup_delay_odd;
    reg        flash_active_odd;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            flash_busy_odd          <= 1'b0;
            flash_data_odd          <= 8'd0;
            flash_data_valid_odd    <= 1'b0;
            flash_bytes_left_odd    <= 24'd0;
            flash_pattern_odd       <= 8'd0;
            flash_startup_delay_odd <= 2'd0;
            flash_active_odd        <= 1'b0;
        end else begin
            flash_data_valid_odd <= 1'b0;

            if (flash_start_odd && !flash_active_odd) begin
                flash_busy_odd          <= 1'b1;
                flash_bytes_left_odd    <= flash_count_odd;
                flash_pattern_odd       <= 8'd0;
                flash_startup_delay_odd <= 2'd2;
                flash_active_odd        <= 1'b1;
            end else if (flash_active_odd) begin
                if (flash_startup_delay_odd != 0) begin
                    flash_startup_delay_odd <= flash_startup_delay_odd - 2'd1;
                end else if (flash_bytes_left_odd != 0 && flash_data_ready_odd) begin
                    flash_data_odd       <= flash_pattern_odd;
                    flash_data_valid_odd <= 1'b1;
                    flash_pattern_odd    <= flash_pattern_odd + 8'd1;
                    flash_bytes_left_odd <= flash_bytes_left_odd - 24'd1;
                end else if (flash_bytes_left_odd == 0) begin
                    flash_busy_odd   <= 1'b0;
                    flash_active_odd <= 1'b0;
                end
            end
        end
    end

    // SDRAM model for odd DUT
    reg [15:0] sdram_mem_odd [0:511];
    reg        sdram_ready_delay_odd;
    integer    sdram_write_count_odd;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sdram_req_ready_odd   <= 1'b0;
            sdram_ready_delay_odd <= 1'b0;
            sdram_write_count_odd <= 0;
        end else begin
            if (sdram_req_odd && sdram_req_write_odd) begin
                if (!sdram_ready_delay_odd) begin
                    sdram_ready_delay_odd <= 1'b1;
                    sdram_req_ready_odd   <= 1'b0;
                end else begin
                    sdram_mem_odd[sdram_req_addr_odd[9:1]] <= sdram_req_wdata_odd;
                    sdram_write_count_odd <= sdram_write_count_odd + 1;
                    sdram_req_ready_odd   <= 1'b1;
                    sdram_ready_delay_odd <= 1'b0;
                end
            end else begin
                sdram_req_ready_odd   <= 1'b0;
                sdram_ready_delay_odd <= 1'b0;
            end
        end
    end

    boot_loader #(
        .FLASH_OFFSET(24'h400000),
        .IMAGE_SIZE(24'd33)
    ) uut_odd (
        .clk(clk),
        .rst_n(rst_n),
        .sdram_init_done(sdram_init_done_odd),
        .boot_done(boot_done_odd),
        .flash_start(flash_start_odd),
        .flash_addr(flash_addr_odd),
        .flash_count(flash_count_odd),
        .flash_busy(flash_busy_odd),
        .flash_data(flash_data_odd),
        .flash_data_valid(flash_data_valid_odd),
        .flash_data_ready(flash_data_ready_odd),
        .sdram_req(sdram_req_odd),
        .sdram_req_write(sdram_req_write_odd),
        .sdram_req_addr(sdram_req_addr_odd),
        .sdram_req_wdata(sdram_req_wdata_odd),
        .sdram_req_ready(sdram_req_ready_odd)
    );

    // =========================================================================
    // Test sequence
    // =========================================================================

    reg [15:0] expected_word;
    reg        test_failed;

    initial begin
        $dumpfile("boot_loader_tb.vcd");
        $dumpvars(0, boot_loader_tb);

        pass_count = 0;
        fail_count = 0;

        // Initialize
        rst_n           = 0;
        sdram_init_done = 0;
        sdram_init_done_odd = 0;

        // Reset
        #100;
        rst_n = 1;
        #100;

        // =====================================================================
        // TEST 1: Boot completes (32 bytes = 16 words)
        // =====================================================================

        // Assert sdram_init_done to start boot process
        sdram_init_done = 1;

        // Wait for boot_done with timeout
        i = 0;
        while (!boot_done && i < 5000) begin
            @(posedge clk);
            i = i + 1;
        end

        if (boot_done) begin
            $display("PASS: Test 1 - Boot completes after 32 bytes transferred");
            pass_count = pass_count + 1;
        end else begin
            $display("FAIL: Test 1 - Boot did not complete within timeout (%0d clocks)", i);
            fail_count = fail_count + 1;
        end

        // =====================================================================
        // TEST 2: Data integrity — verify SDRAM contents match expected pairing
        // =====================================================================
        // Flash pattern: 0x00, 0x01, 0x02, 0x03, ...
        // SDRAM word at addr 0: {0x01, 0x00} = 16'h0100
        // SDRAM word at addr 2: {0x03, 0x02} = 16'h0302
        // etc.

        test_failed = 0;
        for (i = 0; i < 16; i = i + 1) begin
            expected_word = {i[7:0] * 8'd2 + 8'd1, i[7:0] * 8'd2};
            if (sdram_mem[i] !== expected_word) begin
                $display("FAIL: Test 2 - SDRAM[%0d] = 0x%04h, expected 0x%04h",
                         i, sdram_mem[i], expected_word);
                test_failed = 1;
            end
        end

        if (!test_failed) begin
            $display("PASS: Test 2 - Data integrity verified for all 16 words");
            pass_count = pass_count + 1;
        end else begin
            fail_count = fail_count + 1;
        end

        // =====================================================================
        // TEST 3: SDRAM addressing — verify writes go to 0, 2, 4, 6, ...
        // =====================================================================

        test_failed = 0;
        for (i = 0; i < 16; i = i + 1) begin
            if (sdram_write_addrs[i] !== (i * 2)) begin
                $display("FAIL: Test 3 - Write %0d went to addr 0x%06h, expected 0x%06h",
                         i, sdram_write_addrs[i], i * 2);
                test_failed = 1;
            end
        end

        if (sdram_write_count !== 16) begin
            $display("FAIL: Test 3 - Expected 16 writes, got %0d", sdram_write_count);
            test_failed = 1;
        end

        if (!test_failed) begin
            $display("PASS: Test 3 - SDRAM addressing correct (0, 2, 4, ..., 30), 16 writes total");
            pass_count = pass_count + 1;
        end else begin
            fail_count = fail_count + 1;
        end

        // =====================================================================
        // TEST 4: Odd byte count (33 bytes) — pad high byte with 0x00
        // =====================================================================

        sdram_init_done_odd = 1;

        i = 0;
        while (!boot_done_odd && i < 5000) begin
            @(posedge clk);
            i = i + 1;
        end

        if (!boot_done_odd) begin
            $display("FAIL: Test 4 - Odd-count boot did not complete within timeout");
            fail_count = fail_count + 1;
        end else begin
            test_failed = 0;

            // First 16 words (32 bytes) should be normal pairs
            for (i = 0; i < 16; i = i + 1) begin
                expected_word = {i[7:0] * 8'd2 + 8'd1, i[7:0] * 8'd2};
                if (sdram_mem_odd[i] !== expected_word) begin
                    $display("FAIL: Test 4 - SDRAM_odd[%0d] = 0x%04h, expected 0x%04h",
                             i, sdram_mem_odd[i], expected_word);
                    test_failed = 1;
                end
            end

            // Last word (byte 32 = 0x20) should be {0x00, 0x20} = 16'h0020
            if (sdram_mem_odd[16] !== 16'h0020) begin
                $display("FAIL: Test 4 - Last word SDRAM_odd[16] = 0x%04h, expected 0x0020",
                         sdram_mem_odd[16]);
                test_failed = 1;
            end

            // Should be 17 writes total (16 paired + 1 padded)
            if (sdram_write_count_odd !== 17) begin
                $display("FAIL: Test 4 - Expected 17 writes for odd count, got %0d",
                         sdram_write_count_odd);
                test_failed = 1;
            end

            if (!test_failed) begin
                $display("PASS: Test 4 - Odd byte count (33) handled correctly, last word padded with 0x00");
                pass_count = pass_count + 1;
            end else begin
                fail_count = fail_count + 1;
            end
        end

        // =====================================================================
        // TEST 5: boot_done stays asserted after boot
        // =====================================================================

        // Wait additional clocks and verify boot_done stays high
        repeat (50) @(posedge clk);

        if (boot_done && boot_done_odd) begin
            $display("PASS: Test 5 - boot_done remains asserted after boot");
            pass_count = pass_count + 1;
        end else begin
            $display("FAIL: Test 5 - boot_done dropped after boot (even=%b, odd=%b)",
                     boot_done, boot_done_odd);
            fail_count = fail_count + 1;
        end

        // =====================================================================
        // Summary
        // =====================================================================

        $display("");
        $display("========================================");
        $display("  RESULTS: %0d PASSED, %0d FAILED", pass_count, fail_count);
        $display("========================================");

        if (fail_count == 0) begin
            $display("  ALL TESTS PASSED");
        end else begin
            $display("  SOME TESTS FAILED");
        end

        $display("========================================");
        $display("");

        #100;
        $finish;
    end

    // =========================================================================
    // Timeout watchdog
    // =========================================================================

    initial begin
        #500000;
        $display("FAIL: Global timeout — simulation took too long");
        $finish;
    end

endmodule
