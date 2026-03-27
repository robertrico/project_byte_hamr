`timescale 1ns / 1ps
// =============================================================================
// iwm_hamr_tb.v — Integration test: bus_interface + block_buffer + sdram_arbiter
// =============================================================================
// Tests the full data path from Apple II bus registers through block_buffer
// and sdram_arbiter to a mock SDRAM. No flash/boot — injects data directly.
//
// Test flow:
//   1. Pre-fill mock SDRAM with known block data
//   2. Signal boot_done
//   3. Write block number via bus registers
//   4. Issue READ command
//   5. Wait for ready
//   6. Read 512 bytes via DATA port, verify each byte
//   7. Write 512 bytes via DATA port
//   8. Issue WRITE command
//   9. Verify data reached mock SDRAM
// =============================================================================

module iwm_hamr_tb;

    // ---- Clocks ----
    reg clk_7m = 0;
    reg clk_25m = 0;
    always #70  clk_7m  = ~clk_7m;   // 7.14 MHz
    always #20  clk_25m = ~clk_25m;   // 25 MHz

    // ---- Resets ----
    reg rst_7_n = 0;
    reg rst_25_n = 0;

    // ---- Apple II Bus signals ----
    reg  [3:0]  addr;
    reg  [7:0]  data_in;
    wire [7:0]  data_out;
    reg         nDEVICE_SELECT;
    reg         R_nW;

    // ---- System ----
    reg         boot_done;

    // ---- Mock SDRAM (word-addressed, 256 words = 512 bytes per block) ----
    reg [15:0] mock_sdram [0:65535];  // 128KB

    // ---- SDRAM controller mock signals ----
    wire        sdram_req;
    wire        sdram_write;
    wire [25:0] sdram_addr;
    wire [15:0] sdram_wdata;
    reg         sdram_ready;
    reg  [15:0] sdram_rdata;
    reg         sdram_rdata_valid;

    // ---- Block buffer wires ----
    wire [8:0]  buf_addr_a, buf_addr_b;
    wire [7:0]  buf_wdata_a, buf_rdata_a;
    wire        buf_we_a;
    wire [7:0]  buf_wdata_b, buf_rdata_b;
    wire        buf_we_b;

    // ---- Block request wires ----
    wire        dev_block_read_req;
    wire        dev_block_write_req;
    wire [15:0] dev_block_num;
    wire        dev_block_ready;

    // ---- Test tracking ----
    integer tests_run = 0;
    integer tests_passed = 0;
    integer tests_failed = 0;
    integer i;
    reg [7:0] rd;

    // =========================================================================
    // DUT: bus_interface
    // =========================================================================
    bus_interface u_bus (
        .clk              (clk_7m),
        .rst_n            (rst_7_n),
        .addr             (addr),
        .data_in          (data_in),
        .data_out         (data_out),
        .nDEVICE_SELECT   (nDEVICE_SELECT),
        .R_nW             (R_nW),
        .boot_done        (boot_done),
        .buf_addr         (buf_addr_b),
        .buf_rdata        (buf_rdata_b),
        .buf_wdata        (buf_wdata_b),
        .buf_we           (buf_we_b),
        .block_read_req   (dev_block_read_req),
        .block_write_req  (dev_block_write_req),
        .block_num        (dev_block_num),
        .block_ready      (dev_block_ready)
    );

    // =========================================================================
    // DUT: block_buffer
    // =========================================================================
    block_buffer u_buf (
        .clk_a   (clk_25m),
        .addr_a  (buf_addr_a),
        .wdata_a (buf_wdata_a),
        .we_a    (buf_we_a),
        .rdata_a (buf_rdata_a),
        .clk_b   (clk_7m),
        .addr_b  (buf_addr_b),
        .wdata_b (buf_wdata_b),
        .we_b    (buf_we_b),
        .rdata_b (buf_rdata_b)
    );

    // =========================================================================
    // DUT: sdram_arbiter
    // =========================================================================
    sdram_arbiter u_arb (
        .clk                (clk_25m),
        .rst_n              (rst_25_n),
        .boot_done          (boot_done),
        .boot_req           (1'b0),
        .boot_write         (1'b0),
        .boot_addr          (26'd0),
        .boot_wdata         (16'd0),
        .boot_ready         (),
        .dev_block_read_req (dev_block_read_req),
        .dev_block_write_req(dev_block_write_req),
        .dev_block_num      (dev_block_num),
        .dev_block_ready    (dev_block_ready),
        .buf_addr_a         (buf_addr_a),
        .buf_wdata_a        (buf_wdata_a),
        .buf_we_a           (buf_we_a),
        .buf_rdata_a        (buf_rdata_a),
        .sdram_req          (sdram_req),
        .sdram_write        (sdram_write),
        .sdram_addr         (sdram_addr),
        .sdram_wdata        (sdram_wdata),
        .sdram_ready        (sdram_ready),
        .sdram_rdata        (sdram_rdata),
        .sdram_rdata_valid  (sdram_rdata_valid)
    );

    // =========================================================================
    // Mock SDRAM controller behavior
    // =========================================================================
    // Responds to requests after 2 cycles (simulates CAS latency).
    // Read: returns data from mock_sdram array.
    // Write: stores data into mock_sdram array.
    reg [1:0]  sdram_pipe;
    reg [25:0] sdram_pipe_addr;

    always @(posedge clk_25m) begin
        sdram_ready       <= 1'b0;
        sdram_rdata_valid <= 1'b0;

        if (sdram_req && !sdram_ready) begin
            sdram_ready <= 1'b1;
            if (sdram_write) begin
                mock_sdram[sdram_addr[16:1]] <= sdram_wdata;
            end else begin
                sdram_pipe      <= 2'd2;  // 2-cycle read latency
                sdram_pipe_addr <= sdram_addr;
            end
        end

        if (sdram_pipe > 0) begin
            sdram_pipe <= sdram_pipe - 1;
            if (sdram_pipe == 1) begin
                sdram_rdata       <= mock_sdram[sdram_pipe_addr[16:1]];
                sdram_rdata_valid <= 1'b1;
            end
        end
    end

    // =========================================================================
    // Bus cycle tasks
    // =========================================================================
    task bus_write(input [3:0] a, input [7:0] d);
        begin
            @(posedge clk_7m);
            addr = a;
            R_nW = 1'b0;
            data_in = d;
            @(posedge clk_7m);
            nDEVICE_SELECT = 1'b0;
            repeat(4) @(posedge clk_7m);
            nDEVICE_SELECT = 1'b1;
            repeat(2) @(posedge clk_7m);
        end
    endtask

    task bus_read(input [3:0] a, output [7:0] d);
        begin
            @(posedge clk_7m);
            addr = a;
            R_nW = 1'b1;
            @(posedge clk_7m);
            nDEVICE_SELECT = 1'b0;
            repeat(3) @(posedge clk_7m);
            d = data_out;
            @(posedge clk_7m);
            nDEVICE_SELECT = 1'b1;
            repeat(2) @(posedge clk_7m);
        end
    endtask

    task wait_ready;
        reg [7:0] status;
        integer timeout;
        begin
            timeout = 0;
            status = 0;
            while (!(status & 8'h80) && timeout < 5000) begin
                bus_read(4'h0, status);
                timeout = timeout + 1;
            end
            if (timeout >= 5000) begin
                $display("  TIMEOUT waiting for ready!");
                tests_failed = tests_failed + 1;
                tests_run = tests_run + 1;
            end
        end
    endtask

    task check(input [255:0] name, input [7:0] actual, input [7:0] expected);
        begin
            tests_run = tests_run + 1;
            if (actual === expected) begin
                tests_passed = tests_passed + 1;
            end else begin
                tests_failed = tests_failed + 1;
                $display("  FAIL: %0s = $%02X, expected $%02X", name, actual, expected);
            end
        end
    endtask

    // =========================================================================
    // Test sequence
    // =========================================================================
    initial begin
        $dumpfile("iwm_hamr_tb.vcd");
        $dumpvars(0, iwm_hamr_tb);

        // Init
        nDEVICE_SELECT = 1;
        R_nW = 1;
        addr = 0;
        data_in = 0;
        boot_done = 0;
        sdram_ready = 0;
        sdram_rdata = 0;
        sdram_rdata_valid = 0;
        sdram_pipe = 0;

        // Pre-fill block 5 in mock SDRAM with known pattern
        // Block 5 starts at byte address 5*512 = 2560, word address 1280
        for (i = 0; i < 256; i = i + 1)
            mock_sdram[1280 + i] = {i[7:0] ^ 8'hA5, i[7:0]};
        // Byte layout: buf[0]=$00, buf[1]=$A5, buf[2]=$01, buf[3]=$A4, ...

        // Release resets
        repeat(8) @(posedge clk_25m);
        rst_25_n = 1;
        repeat(4) @(posedge clk_7m);
        rst_7_n = 1;
        repeat(4) @(posedge clk_7m);

        // Signal boot done
        boot_done = 1;
        repeat(4) @(posedge clk_7m);

        // =================================================================
        // TEST: Full block READ — bus → arbiter → mock SDRAM → buffer → bus
        // =================================================================
        $display("\n--- Integration Test: READ block 5 ---");

        bus_write(4'h2, 8'h05);  // BLOCK_LO = 5
        bus_write(4'h3, 8'h00);  // BLOCK_HI = 0
        bus_write(4'h0, 8'h01);  // COMMAND = READ

        wait_ready;

        // Read all 512 bytes and verify
        for (i = 0; i < 512; i = i + 1) begin
            bus_read(4'h1, rd);
            if (i[0] == 0)
                check("read_lo", rd, i[8:1]);
            else
                check("read_hi", rd, i[8:1] ^ 8'hA5);
        end

        $display("  READ block 5: %0d/%0d bytes correct",
                 tests_passed, tests_run);

        // =================================================================
        // TEST: Full block WRITE — bus → buffer → arbiter → mock SDRAM
        // =================================================================
        $display("\n--- Integration Test: WRITE block 10 ---");

        bus_write(4'h2, 8'h0A);  // BLOCK_LO = 10
        bus_write(4'h3, 8'h00);  // BLOCK_HI = 0

        // Fill buffer with pattern: byte[n] = n ^ $55 (addr 5 = DATA_WRITE)
        for (i = 0; i < 512; i = i + 1)
            bus_write(4'h5, i[7:0] ^ 8'h55);

        bus_write(4'h0, 8'h02);  // COMMAND = WRITE

        wait_ready;

        // Verify mock SDRAM — block 10 at word address 10*256 = 2560
        // Each SDRAM word = {hi_byte, lo_byte}
        begin : verify_write
            integer errs;
            reg [15:0] expected_word;
            errs = 0;
            for (i = 0; i < 256; i = i + 1) begin
                expected_word[7:0]  = (i*2)   ^ 8'h55;
                expected_word[15:8] = (i*2+1) ^ 8'h55;

                // Note: block 10 at word offset = 10*256 = 2560
                tests_run = tests_run + 1;
                if (mock_sdram[2560 + i] === expected_word) begin
                    tests_passed = tests_passed + 1;
                end else begin
                    tests_failed = tests_failed + 1;
                    if (errs < 5)
                        $display("  FAIL: sdram[%0d] = $%04X, expected $%04X",
                                 2560+i, mock_sdram[2560+i], expected_word);
                    errs = errs + 1;
                end
            end
            $display("  WRITE block 10: %0d errors", errs);
        end

        // =================================================================
        // TEST: READ block 5 again AFTER writing block 10
        // Verifies WRITE didn't corrupt unrelated blocks
        // =================================================================
        $display("\n--- Integration Test: RE-READ block 5 after WRITE ---");

        bus_write(4'h2, 8'h05);  // BLOCK_LO = 5
        bus_write(4'h3, 8'h00);  // BLOCK_HI = 0
        bus_write(4'h0, 8'h01);  // COMMAND = READ

        wait_ready;

        begin : verify_reread
            integer errs;
            errs = 0;
            for (i = 0; i < 512; i = i + 1) begin
                bus_read(4'h1, rd);
                tests_run = tests_run + 1;
                if (i[0] == 0) begin
                    if (rd === i[8:1]) tests_passed = tests_passed + 1;
                    else begin tests_failed = tests_failed + 1; errs = errs + 1;
                        if (errs < 5) $display("  FAIL: byte %0d = $%02X, expected $%02X", i, rd, i[8:1]);
                    end
                end else begin
                    if (rd === (i[8:1] ^ 8'hA5)) tests_passed = tests_passed + 1;
                    else begin tests_failed = tests_failed + 1; errs = errs + 1;
                        if (errs < 5) $display("  FAIL: byte %0d = $%02X, expected $%02X", i, rd, i[8:1] ^ 8'hA5);
                    end
                end
            end
            $display("  RE-READ block 5: %0d errors", errs);
        end

        // =================================================================
        // TEST: READ back block 10 (the one we wrote)
        // =================================================================
        $display("\n--- Integration Test: READ back written block 10 ---");

        bus_write(4'h2, 8'h0A);
        bus_write(4'h3, 8'h00);
        bus_write(4'h0, 8'h01);  // READ

        wait_ready;

        begin : verify_readback
            integer errs;
            errs = 0;
            for (i = 0; i < 512; i = i + 1) begin
                bus_read(4'h1, rd);
                tests_run = tests_run + 1;
                if (rd === ((i[7:0]) ^ 8'h55)) begin
                    tests_passed = tests_passed + 1;
                end else begin
                    tests_failed = tests_failed + 1; errs = errs + 1;
                    if (errs < 5)
                        $display("  FAIL: byte %0d = $%02X, expected $%02X", i, rd, i[7:0] ^ 8'h55);
                end
            end
            $display("  READ-BACK block 10: %0d errors", errs);
        end

        // =================================================================
        // Summary
        // =================================================================
        $display("\n========================================");
        $display("  Tests: %0d  Passed: %0d  Failed: %0d",
                 tests_run, tests_passed, tests_failed);
        $display("========================================\n");

        if (tests_failed > 0)
            $display("*** SOME TESTS FAILED ***");
        else
            $display("*** ALL TESTS PASSED ***");

        $finish;
    end

    // Timeout watchdog
    initial begin
        #200_000_000;
        $display("WATCHDOG TIMEOUT");
        $finish;
    end

endmodule
