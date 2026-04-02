// sd_controller_tb.v — Testbench for sd_controller
// Tests init sequence, CMD17 read, CMD24 write against sd_card_model.

`timescale 1ns / 1ps

module sd_controller_tb;

    // =========================================================================
    // Clock: 25 MHz
    // =========================================================================
    reg clk = 0;
    always #20 clk = ~clk;

    // =========================================================================
    // DUT signals
    // =========================================================================
    reg        rst_n = 0;
    reg        init_start = 0;
    wire       init_done, init_error, is_sdhc;

    reg        read_start = 0;
    reg [31:0] read_addr = 0;
    wire [7:0] read_data;
    wire       read_data_valid;
    reg        read_data_ready = 1;
    wire       read_done, read_error;

    reg        write_start = 0;
    reg [31:0] write_addr = 0;
    reg [7:0]  write_data_reg = 0;
    reg        write_data_valid_reg = 0;
    wire       write_data_req;
    wire       write_done, write_error;

    wire       sd_sck, sd_mosi, sd_miso;
    wire       sd_cs;

    // =========================================================================
    // DUT
    // =========================================================================
    sd_controller dut (
        .clk(clk), .rst_n(rst_n),
        .init_start(init_start), .init_done(init_done),
        .init_error(init_error), .is_sdhc(is_sdhc),
        .read_start(read_start), .read_addr(read_addr),
        .read_data(read_data), .read_data_valid(read_data_valid),
        .read_data_ready(read_data_ready), .read_done(read_done),
        .read_error(read_error),
        .write_start(write_start), .write_addr(write_addr),
        .write_data(write_data_reg), .write_data_valid(write_data_valid_reg),
        .write_data_req(write_data_req), .write_done(write_done),
        .write_error(write_error),
        .sd_sck(sd_sck), .sd_mosi(sd_mosi), .sd_miso(sd_miso),
        .sd_cs(sd_cs)
    );

    // =========================================================================
    // SD card model
    // =========================================================================
    sd_card_model #(
        .SDHC(1),
        .ACMD41_RETRIES(3),
        .WRITE_BUSY_CLKS(50),
        .MEM_BYTES(262144)
    ) sd_card (
        .spi_sck(sd_sck),
        .spi_mosi(sd_mosi),
        .spi_miso(sd_miso),
        .spi_cs_n(sd_cs)
    );

    // =========================================================================
    // Data capture for read tests
    // =========================================================================
    reg [7:0] captured [0:511];
    integer   cap_idx;

    always @(posedge clk) begin
        if (read_data_valid && cap_idx < 512) begin
            captured[cap_idx] <= read_data;
            cap_idx <= cap_idx + 1;
        end
    end

    // =========================================================================
    // Write data feeder
    // =========================================================================
    reg [9:0]  wr_feed_cnt;
    reg [7:0]  wr_pattern_base;

    always @(posedge clk) begin
        if (write_data_req) begin
            write_data_reg       <= (wr_feed_cnt[7:0]) ^ wr_pattern_base;
            write_data_valid_reg <= 1'b1;
            wr_feed_cnt          <= wr_feed_cnt + 10'd1;
        end else begin
            write_data_valid_reg <= 1'b0;
        end
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

    task wait_for;
        input integer cond_addr;  // not used, just wait by signal
        input integer max_cycles;
        integer i;
        begin
            for (i = 0; i < max_cycles; i = i + 1)
                @(posedge clk);
        end
    endtask

    // =========================================================================
    // Test sequence
    // =========================================================================
    integer i;

    initial begin
        $dumpfile("sd_controller_tb.vcd");
        $dumpvars(0, sd_controller_tb);

        // Pre-fill SD card model memory
        // Block 0 (bytes 0-511): pattern byte[i] = i & 0xFF
        for (i = 0; i < 512; i = i + 1)
            sd_card.mem[i] = i & 8'hFF;

        // Block 5 (bytes 2560-3071): pattern byte[i] = (i*3+7) & 0xFF
        for (i = 0; i < 512; i = i + 1)
            sd_card.mem[2560 + i] = (i * 3 + 7) & 8'hFF;

        // Reset
        rst_n = 0;
        repeat(10) @(posedge clk);
        rst_n = 1;
        repeat(5) @(posedge clk);

        // =============================================================
        // Test 1: Successful SDHC init
        // =============================================================
        $display("\n--- Test 1: SDHC init ---");
        init_start = 1;
        @(posedge clk);
        init_start = 0;

        // Wait for init_done or init_error (with timeout)
        begin : wait_init
            integer w;
            for (w = 0; w < 2000000 && !init_done && !init_error; w = w + 1)
                @(posedge clk);
            if (w >= 2000000)
                $display("TIMEOUT waiting for init");
        end

        check("init_done",  1, init_done);
        check("init_error", 0, init_error);
        check("is_sdhc",    1, is_sdhc);
        $display("  Init took %0t", $time);

        repeat(100) @(posedge clk);

        // =============================================================
        // Test 7: Read block 0
        // =============================================================
        $display("\n--- Test 7: Read block 0 ---");
        cap_idx = 0;
        read_data_ready = 1;
        read_addr = 32'd0;
        read_start = 1;
        @(posedge clk);
        read_start = 0;

        begin : wait_read0
            integer w;
            for (w = 0; w < 500000 && !read_done && !read_error; w = w + 1)
                @(posedge clk);
            if (w >= 500000)
                $display("TIMEOUT waiting for read_done");
        end

        check("read0_done",    1, read_done);
        check("read0_error",   0, read_error);
        check("read0_count",   512, cap_idx);
        check("read0_byte0",   8'h00, captured[0]);
        check("read0_byte1",   8'h01, captured[1]);
        check("read0_byte255", 8'hFF, captured[255]);
        check("read0_byte256", 8'h00, captured[256]);
        check("read0_byte511", 8'hFF, captured[511]);

        repeat(100) @(posedge clk);

        // =============================================================
        // Test 8: Read block 5
        // =============================================================
        $display("\n--- Test 8: Read block 5 ---");
        cap_idx = 0;
        read_addr = 32'd5;
        read_start = 1;
        @(posedge clk);
        read_start = 0;

        begin : wait_read5
            integer w;
            for (w = 0; w < 500000 && !read_done && !read_error; w = w + 1)
                @(posedge clk);
        end

        check("read5_done",  1, read_done);
        check("read5_byte0", (0*3+7) & 8'hFF, captured[0]);
        check("read5_byte1", (1*3+7) & 8'hFF, captured[1]);
        check("read5_byte100", (100*3+7) & 8'hFF, captured[100]);

        repeat(100) @(posedge clk);

        // =============================================================
        // Test 11: Write block 50, then read back
        // =============================================================
        $display("\n--- Test 11+12: Write block 50, read back ---");
        wr_feed_cnt = 10'd0;
        wr_pattern_base = 8'hAA;
        write_addr = 32'd50;
        write_start = 1;
        @(posedge clk);
        write_start = 0;

        begin : wait_write
            integer w;
            for (w = 0; w < 500000 && !write_done && !write_error; w = w + 1)
                @(posedge clk);
            if (w >= 500000)
                $display("TIMEOUT waiting for write_done");
        end

        check("write_done",  1, write_done);
        check("write_error", 0, write_error);

        // Verify model memory directly
        check("model_byte0",   8'h00 ^ 8'hAA, sd_card.mem[50*512 + 0]);
        check("model_byte1",   8'h01 ^ 8'hAA, sd_card.mem[50*512 + 1]);
        check("model_byte255", 8'hFF ^ 8'hAA, sd_card.mem[50*512 + 255]);

        repeat(100) @(posedge clk);

        // Read back block 50
        cap_idx = 0;
        read_addr = 32'd50;
        read_start = 1;
        @(posedge clk);
        read_start = 0;

        begin : wait_readback
            integer w;
            for (w = 0; w < 500000 && !read_done && !read_error; w = w + 1)
                @(posedge clk);
        end

        check("readback_done",    1, read_done);
        check("readback_byte0",   8'h00 ^ 8'hAA, captured[0]);
        check("readback_byte1",   8'h01 ^ 8'hAA, captured[1]);
        check("readback_byte511", 8'hFF ^ 8'hAA, captured[511]);

        // =============================================================
        // Results
        // =============================================================
        repeat(100) @(posedge clk);
        $display("\n========================================");
        $display("sd_controller_tb: %0d tests, %0d passed, %0d failed",
                 tests_run, tests_passed, tests_failed);
        $display("========================================\n");
        $finish;
    end

    // Global timeout
    initial begin
        #200000000;  // 200ms
        $display("GLOBAL TIMEOUT");
        $finish;
    end

endmodule
