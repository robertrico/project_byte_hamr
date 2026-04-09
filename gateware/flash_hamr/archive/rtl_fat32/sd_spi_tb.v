// sd_spi_tb.v — Testbench for sd_spi byte-level SPI engine
// 6 test cases: slow/fast mode, SPI Mode 0 timing, MISO capture,
// back-to-back transfers, idle state.

`timescale 1ns / 1ps

module sd_spi_tb;

    // =========================================================================
    // Clock: 25 MHz (40ns period)
    // =========================================================================
    reg clk = 0;
    always #20 clk = ~clk;

    // =========================================================================
    // DUT signals
    // =========================================================================
    reg        rst_n = 0;
    reg        start = 0;
    reg [7:0]  tx_byte = 8'hFF;
    wire [7:0] rx_byte;
    wire       done;
    wire       busy;
    reg        slow_mode = 0;

    wire       spi_sck;
    wire       spi_mosi;
    reg        spi_miso = 1'b1;   // idles high

    // =========================================================================
    // DUT
    // =========================================================================
    sd_spi dut (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (start),
        .tx_byte   (tx_byte),
        .rx_byte   (rx_byte),
        .done      (done),
        .busy      (busy),
        .slow_mode (slow_mode),
        .spi_sck   (spi_sck),
        .spi_mosi  (spi_mosi),
        .spi_miso  (spi_miso)
    );

    // =========================================================================
    // SPI slave model — captures MOSI bits, drives MISO with a pattern
    // =========================================================================
    // SPI Mode 0: MISO must be valid BEFORE the first SCK rising edge.
    // The test code sets spi_miso = miso_pattern[7] before each transfer.
    // On each SCK falling edge, we advance to the next bit.
    reg [7:0] captured_mosi;
    reg [2:0] capture_cnt;
    reg [7:0] miso_pattern;     // pattern to shift out on MISO
    reg [2:0] miso_cnt;         // counts 0..7, advances on SCK falling edge

    // Capture MOSI on SCK rising edge
    always @(posedge spi_sck) begin
        captured_mosi <= {captured_mosi[6:0], spi_mosi};
        capture_cnt   <= capture_cnt + 3'd1;
    end

    // Advance MISO to next bit on SCK falling edge
    // Bit 7 was already set up before the transfer started.
    // After first posedge samples bit 7, first negedge sets up bit 6, etc.
    always @(negedge spi_sck) begin
        miso_cnt <= miso_cnt + 3'd1;
        spi_miso <= miso_pattern[6 - miso_cnt[2:0]];
    end

    // =========================================================================
    // Scoreboard
    // =========================================================================
    integer tests_run = 0;
    integer tests_passed = 0;
    integer tests_failed = 0;

    task check;
        input [255:0] label;
        input [31:0] expected;
        input [31:0] actual;
        begin
            tests_run = tests_run + 1;
            if (expected === actual) begin
                tests_passed = tests_passed + 1;
            end else begin
                tests_failed = tests_failed + 1;
                $display("FAIL: %0s — expected 0x%0h, got 0x%0h", label, expected, actual);
            end
        end
    endtask

    task wait_done;
        input integer max_cycles;
        integer i;
        begin
            for (i = 0; i < max_cycles && !done; i = i + 1)
                @(posedge clk);
            if (!done)
                $display("TIMEOUT waiting for done");
        end
    endtask

    // =========================================================================
    // Cycle counter for timing verification
    // =========================================================================
    integer cycle_start, cycle_end;

    // =========================================================================
    // Test sequence
    // =========================================================================
    initial begin
        $dumpfile("sd_spi_tb.vcd");
        $dumpvars(0, sd_spi_tb);

        // Reset
        rst_n = 0;
        start = 0;
        tx_byte = 8'hFF;
        slow_mode = 0;
        captured_mosi = 8'd0;
        capture_cnt = 3'd0;
        miso_pattern = 8'hFF;
        miso_cnt = 3'd0;
        repeat(5) @(posedge clk);
        rst_n = 1;
        repeat(2) @(posedge clk);

        // =============================================================
        // Test 6: Idle state (run first, before any transfers)
        // =============================================================
        $display("\n--- Test 6: Idle state ---");
        check("idle_sck",  0, spi_sck);
        check("idle_busy", 0, busy);
        check("idle_done", 0, done);

        // =============================================================
        // Test 2: Fast mode — single byte TX/RX
        // =============================================================
        $display("\n--- Test 2: Fast mode TX/RX ---");
        slow_mode = 0;
        miso_pattern = 8'hD2;
        miso_cnt = 3'd0;
        capture_cnt = 3'd0;
        spi_miso = miso_pattern[7];  // pre-set bit 7 before first SCK rise

        tx_byte = 8'h3C;
        cycle_start = $time;
        @(posedge clk);
        start = 1;
        @(posedge clk);
        start = 0;

        wait_done(200);
        cycle_end = $time;

        check("fast_tx",     8'h3C, captured_mosi);
        check("fast_rx",     8'hD2, rx_byte);
        check("fast_done",   1,     done);
        // Fast mode: 8 bits × 2 phases × 2 CLK = 32 CLK cycles = 1280ns
        // Allow some margin for startup
        if ((cycle_end - cycle_start) < 2000)
            $display("  fast mode timing OK (%0d ns)", cycle_end - cycle_start);
        else
            $display("  WARNING: fast mode took %0d ns (expected ~1280ns)", cycle_end - cycle_start);

        repeat(5) @(posedge clk);

        // =============================================================
        // Test 1: Slow mode — single byte TX/RX
        // =============================================================
        $display("\n--- Test 1: Slow mode TX/RX ---");
        slow_mode = 1;
        miso_pattern = 8'h5A;
        miso_cnt = 3'd0;
        capture_cnt = 3'd0;
        spi_miso = miso_pattern[7];

        tx_byte = 8'hA5;
        cycle_start = $time;
        @(posedge clk);
        start = 1;
        @(posedge clk);
        start = 0;

        wait_done(20000);
        cycle_end = $time;

        check("slow_tx",     8'hA5, captured_mosi);
        check("slow_rx",     8'h5A, rx_byte);
        check("slow_done",   1,     done);
        // Slow mode: 8 bits × 2 phases × 64 CLK = 1024 CLK = 40960ns
        $display("  slow mode took %0d ns (expected ~40960ns)", cycle_end - cycle_start);

        repeat(5) @(posedge clk);

        // =============================================================
        // Test 3: SPI Mode 0 timing — MSB first, MOSI on SCK LOW
        // =============================================================
        $display("\n--- Test 3: SPI Mode 0 timing ---");
        slow_mode = 0;
        miso_pattern = 8'hFF;
        miso_cnt = 3'd0;
        capture_cnt = 3'd0;

        tx_byte = 8'hB7;  // 10110111
        @(posedge clk);
        start = 1;
        @(posedge clk);
        start = 0;

        wait_done(200);

        // captured_mosi should be 0xB7 (MSB first, captured on SCK rising)
        check("mode0_mosi", 8'hB7, captured_mosi);

        repeat(5) @(posedge clk);

        // =============================================================
        // Test 4: MISO capture
        // =============================================================
        $display("\n--- Test 4: MISO capture ---");
        slow_mode = 0;
        miso_pattern = 8'h42;
        miso_cnt = 3'd0;
        capture_cnt = 3'd0;
        spi_miso = miso_pattern[7];

        tx_byte = 8'hFF;  // dummy TX
        @(posedge clk);
        start = 1;
        @(posedge clk);
        start = 0;

        wait_done(200);

        check("miso_capture", 8'h42, rx_byte);

        repeat(5) @(posedge clk);

        // =============================================================
        // Test 5: Back-to-back transfers
        // =============================================================
        $display("\n--- Test 5: Back-to-back transfers ---");
        slow_mode = 0;
        begin : b2b_test
            reg [7:0] b2b_rx [0:3];
            integer b2b_count;

            b2b_count = 0;

            // Send first byte
            miso_pattern = 8'h11;
            miso_cnt = 3'd0;
            spi_miso = 1'b0;  // bit 7 of 0x11 = 0
            tx_byte = 8'h01;
            @(posedge clk);
            start = 1;
            @(posedge clk);
            start = 0;

            // Wait for done, then immediately start next
            wait_done(200);
            b2b_rx[0] = rx_byte;

            miso_pattern = 8'h22;
            miso_cnt = 3'd0;
            spi_miso = miso_pattern[7];
            tx_byte = 8'h02;
            start = 1;
            @(posedge clk);
            start = 0;
            wait_done(200);
            b2b_rx[1] = rx_byte;

            miso_pattern = 8'h33;
            miso_cnt = 3'd0;
            spi_miso = miso_pattern[7];
            tx_byte = 8'h03;
            start = 1;
            @(posedge clk);
            start = 0;
            wait_done(200);
            b2b_rx[2] = rx_byte;

            miso_pattern = 8'h44;
            miso_cnt = 3'd0;
            spi_miso = miso_pattern[7];
            tx_byte = 8'h04;
            start = 1;
            @(posedge clk);
            start = 0;
            wait_done(200);
            b2b_rx[3] = rx_byte;

            check("b2b_rx0", 8'h11, b2b_rx[0]);
            check("b2b_rx1", 8'h22, b2b_rx[1]);
            check("b2b_rx2", 8'h33, b2b_rx[2]);
            check("b2b_rx3", 8'h44, b2b_rx[3]);
        end

        // =============================================================
        // Results
        // =============================================================
        repeat(10) @(posedge clk);
        $display("\n========================================");
        $display("sd_spi_tb: %0d tests, %0d passed, %0d failed",
                 tests_run, tests_passed, tests_failed);
        $display("========================================\n");
        $finish;
    end

    // Timeout
    initial begin
        #5000000;
        $display("GLOBAL TIMEOUT");
        $finish;
    end

endmodule
