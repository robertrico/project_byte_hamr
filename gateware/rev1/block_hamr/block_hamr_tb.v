`timescale 1ns / 1ps
// =============================================================================
// block_hamr_tb.v — Integration test: synchronous write-through flash persistence
// =============================================================================
// Architecture matches block_hamr_top.v:
//   - sdram_arbiter (original v2, never modified)
//   - write_through (synchronous flash write after SDRAM write)
//   - flash_writer (with prog_data_valid handshake)
//   - bus_interface (no dirty tracking)
//   - Mock SDRAM with variable latency (simulates refresh)
//
// Tests:
//   1. READ block from mock SDRAM (no flash write, fast)
//   2. WRITE block — wait_ready blocks until BOTH SDRAM and flash are done
//   3. Verify flash has correct data immediately after ready
//   4. Read still works after write-through

module block_hamr_tb;

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

    // ---- Mock SDRAM ----
    reg [15:0] mock_sdram [0:65535];

    // ---- SDRAM controller mock signals ----
    wire        mux_sdram_req;
    wire        mux_sdram_write;
    wire [25:0] mux_sdram_addr;
    wire [15:0] mux_sdram_wdata;
    reg         sdram_ready;
    reg  [15:0] sdram_rdata;
    reg         sdram_rdata_valid;

    // ---- Arbiter SDRAM signals ----
    wire        arb_sdram_req;
    wire        arb_sdram_write;
    wire [25:0] arb_sdram_addr;
    wire [15:0] arb_sdram_wdata;

    // ---- Write-through / flash-persist SDRAM signals ----
    wire        wt_sdram_req;
    wire [25:0] wt_sdram_addr;
    wire [15:0] wt_sdram_wdata;
    wire        wt_claim;

    // ---- SDRAM mux (matches block_hamr_top.v) ----
    assign mux_sdram_req   = wt_claim ? wt_sdram_req   : arb_sdram_req;
    assign mux_sdram_write = wt_claim ? 1'b0           : arb_sdram_write;
    assign mux_sdram_addr  = wt_claim ? wt_sdram_addr  : arb_sdram_addr;
    assign mux_sdram_wdata = wt_claim ? wt_sdram_wdata : arb_sdram_wdata;

    // ---- Block buffer ----
    wire [8:0]  buf_addr_a, buf_addr_b;
    wire [7:0]  buf_wdata_a, buf_rdata_a, buf_wdata_b, buf_rdata_b;
    wire        buf_we_a, buf_we_b;

    // ---- Block request ----
    wire        dev_block_read_req, dev_block_write_req;
    wire [15:0] dev_block_num;
    wire        arb_block_ready;    // raw from arbiter
    wire        dev_block_ready;    // gated by write_through

    // ---- Flash persist ----
    wire        fp_start;
    wire [5:0]  fp_sector;
    wire        fp_busy;

    // ---- Flash writer ----
    wire        fw_start_erase, fw_start_program, fw_busy, fw_done;
    wire [23:0] fw_flash_addr;
    wire [7:0]  fw_prog_data;
    wire        fw_prog_data_valid;
    wire        fw_prog_data_req;
    wire        fw_spi_sck, fw_spi_ncs, fw_spi_mosi, fw_spi_miso;

    // ---- Test tracking ----
    integer tests_run = 0, tests_passed = 0, tests_failed = 0;
    integer i;
    reg [7:0] rd;

    // =========================================================================
    // DUTs — matching block_hamr_top.v architecture
    // =========================================================================
    bus_interface u_bus (
        .clk(clk_7m), .rst_n(rst_7_n),
        .addr(addr), .data_in(data_in), .data_out(data_out),
        .nDEVICE_SELECT(nDEVICE_SELECT), .R_nW(R_nW),
        .boot_done(boot_done),
        .buf_addr(buf_addr_b), .buf_rdata(buf_rdata_b),
        .buf_wdata(buf_wdata_b), .buf_we(buf_we_b),
        .block_read_req(dev_block_read_req),
        .block_write_req(dev_block_write_req),
        .block_num(dev_block_num),
        .block_ready(dev_block_ready),
        .block_num_out()
    );

    block_buffer u_buf (
        .clk_a(clk_25m), .addr_a(buf_addr_a), .wdata_a(buf_wdata_a),
        .we_a(buf_we_a), .rdata_a(buf_rdata_a),
        .clk_b(clk_7m), .addr_b(buf_addr_b), .wdata_b(buf_wdata_b),
        .we_b(buf_we_b), .rdata_b(buf_rdata_b)
    );

    sdram_arbiter u_arb (
        .clk(clk_25m), .rst_n(rst_25_n), .boot_done(boot_done),
        .boot_req(1'b0), .boot_write(1'b0), .boot_addr(26'd0),
        .boot_wdata(16'd0), .boot_ready(),
        .dev_block_read_req(dev_block_read_req),
        .dev_block_write_req(dev_block_write_req),
        .dev_block_num(dev_block_num), .dev_block_ready(arb_block_ready),
        .buf_addr_a(buf_addr_a), .buf_wdata_a(buf_wdata_a),
        .buf_we_a(buf_we_a), .buf_rdata_a(buf_rdata_a),
        .sdram_req(arb_sdram_req), .sdram_write(arb_sdram_write),
        .sdram_addr(arb_sdram_addr), .sdram_wdata(arb_sdram_wdata),
        .sdram_ready(sdram_ready),
        .sdram_rdata(sdram_rdata),
        .sdram_rdata_valid(sdram_rdata_valid)
    );

    write_through u_wt (
        .clk(clk_25m), .rst_n(rst_25_n), .boot_done(boot_done),
        .arb_block_ready(arb_block_ready),
        .gated_block_ready(dev_block_ready),
        .dev_block_write_req(dev_block_write_req),
        .dev_block_num(dev_block_num),
        .fp_start(fp_start), .fp_sector(fp_sector),
        .fp_busy(fp_busy)
    );

    flash_persist u_fp (
        .clk(clk_25m), .rst_n(rst_25_n),
        .start(fp_start), .sector_num(fp_sector), .busy(fp_busy),
        .sdram_req(wt_sdram_req), .sdram_addr(wt_sdram_addr),
        .sdram_wdata(wt_sdram_wdata),
        .sdram_ready(wt_claim & sdram_ready),
        .sdram_rdata(sdram_rdata),
        .sdram_rdata_valid(wt_claim & sdram_rdata_valid),
        .fw_start_erase(fw_start_erase), .fw_start_program(fw_start_program),
        .fw_flash_addr(fw_flash_addr), .fw_prog_data(fw_prog_data),
        .fw_prog_data_valid(fw_prog_data_valid),
        .fw_prog_data_req(fw_prog_data_req),
        .fw_busy(fw_busy),
        .sdram_claim(wt_claim)
    );

    flash_writer u_fw (
        .clk(clk_25m), .rst_n(rst_25_n),
        .start_erase(fw_start_erase), .start_program(fw_start_program),
        .flash_addr(fw_flash_addr),
        .prog_data(fw_prog_data),
        .prog_data_valid(fw_prog_data_valid),
        .prog_data_req(fw_prog_data_req),
        .busy(fw_busy), .done(fw_done),
        .spi_sck(fw_spi_sck), .spi_ncs(fw_spi_ncs),
        .spi_mosi(fw_spi_mosi), .spi_miso(fw_spi_miso)
    );

    spi_flash_write_model #(.MEM_SIZE(65536), .ERASE_WIP(4), .PROG_WIP(2)) u_flash (
        .ncs(fw_spi_ncs), .sck(fw_spi_sck), .mosi(fw_spi_mosi), .miso(fw_spi_miso)
    );

    // =========================================================================
    // Mock SDRAM controller — variable latency to simulate refresh
    // =========================================================================
    localparam REFRESH_INTERVAL = 190;
    localparam REFRESH_PENALTY  = 14;

    reg [7:0]  refresh_cnt = REFRESH_INTERVAL;
    reg        refresh_active = 0;
    reg [4:0]  refresh_delay = 0;
    reg [2:0]  sdram_pipe = 0;
    reg [25:0] sdram_pipe_addr;

    always @(posedge clk_25m) begin
        sdram_ready       <= 1'b0;
        sdram_rdata_valid <= 1'b0;

        // Refresh timer
        if (refresh_cnt == 0 && !refresh_active && !(mux_sdram_req && !sdram_ready)) begin
            refresh_active <= 1'b1;
            refresh_delay  <= REFRESH_PENALTY;
            refresh_cnt    <= REFRESH_INTERVAL;
        end else if (refresh_active) begin
            if (refresh_delay == 0)
                refresh_active <= 1'b0;
            else
                refresh_delay <= refresh_delay - 1;
        end else begin
            if (refresh_cnt > 0)
                refresh_cnt <= refresh_cnt - 1;
        end

        // Accept requests (blocked during refresh)
        if (!refresh_active && mux_sdram_req && !sdram_ready) begin
            sdram_ready <= 1'b1;
            if (mux_sdram_write)
                mock_sdram[mux_sdram_addr[16:1]] <= mux_sdram_wdata;
            else begin
                sdram_pipe      <= 3'd3;
                sdram_pipe_addr <= mux_sdram_addr;
            end
        end

        // Read data pipeline
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
            @(posedge clk_7m); addr = a; R_nW = 1'b0; data_in = d;
            @(posedge clk_7m); nDEVICE_SELECT = 1'b0;
            repeat(4) @(posedge clk_7m);
            nDEVICE_SELECT = 1'b1;
            repeat(2) @(posedge clk_7m);
        end
    endtask

    task bus_read(input [3:0] a, output [7:0] d);
        begin
            @(posedge clk_7m); addr = a; R_nW = 1'b1;
            @(posedge clk_7m); nDEVICE_SELECT = 1'b0;
            repeat(3) @(posedge clk_7m);
            d = data_out;
            @(posedge clk_7m); nDEVICE_SELECT = 1'b1;
            repeat(2) @(posedge clk_7m);
        end
    endtask

    task wait_ready;
        reg [7:0] status;
        integer timeout;
        begin
            timeout = 0; status = 0;
            while (!(status & 8'h80) && timeout < 50000) begin
                bus_read(4'h0, status);
                timeout = timeout + 1;
            end
            if (timeout >= 50000) begin
                $display("  TIMEOUT waiting for ready!");
                tests_failed = tests_failed + 1;
                tests_run = tests_run + 1;
            end
        end
    endtask

    task check(input [255:0] name, input [7:0] actual, input [7:0] expected);
        begin
            tests_run = tests_run + 1;
            if (actual === expected)
                tests_passed = tests_passed + 1;
            else begin
                tests_failed = tests_failed + 1;
                $display("  FAIL: %0s = $%02X, expected $%02X", name, actual, expected);
            end
        end
    endtask

    // =========================================================================
    // Test sequence
    // =========================================================================
    initial begin
        $dumpfile("block_hamr_tb.vcd");
        $dumpvars(0, block_hamr_tb);

        nDEVICE_SELECT = 1; R_nW = 1; addr = 0; data_in = 0;
        boot_done = 0; sdram_ready = 0; sdram_rdata = 0;
        sdram_rdata_valid = 0; sdram_pipe = 0;

        // Pre-fill block 5 in mock SDRAM
        for (i = 0; i < 256; i = i + 1)
            mock_sdram[1280 + i] = {i[7:0] ^ 8'hA5, i[7:0]};

        // Pre-fill sector 1 (blocks 8-15) so write-through reads known data
        for (i = 0; i < 2048; i = i + 1)
            mock_sdram[2048 + i] = 16'h0000;

        // Release resets
        repeat(8) @(posedge clk_25m); rst_25_n = 1;
        repeat(4) @(posedge clk_7m);  rst_7_n = 1;
        repeat(4) @(posedge clk_7m);
        boot_done = 1;
        repeat(4) @(posedge clk_7m);

        // =============================================================
        // TEST 1: READ block 5 (no flash write, passes through fast)
        // =============================================================
        $display("\n--- READ block 5 ---");
        bus_write(4'h2, 8'h05); bus_write(4'h3, 8'h00);
        bus_write(4'h0, 8'h01);
        wait_ready;
        for (i = 0; i < 512; i = i + 1) begin
            bus_read(4'h1, rd);
            if (i[0] == 0) check("read_lo", rd, i[8:1]);
            else            check("read_hi", rd, i[8:1] ^ 8'hA5);
        end
        $display("  READ: %0d/%0d", tests_passed, tests_run);

        // =============================================================
        // TEST 2: WRITE block 10 — wait_ready blocks until flash done
        // =============================================================
        $display("\n--- WRITE block 10 (synchronous write-through) ---");
        bus_write(4'h2, 8'h0A); bus_write(4'h3, 8'h00);
        for (i = 0; i < 512; i = i + 1)
            bus_write(4'h5, i[7:0] ^ 8'h55);
        bus_write(4'h0, 8'h02);
        wait_ready;  // blocks until SDRAM + flash both done

        // Verify SDRAM has the data
        begin : vfy_sdram
            integer errs; reg [15:0] exp;
            errs = 0;
            for (i = 0; i < 256; i = i + 1) begin
                exp[7:0]  = (i*2)   ^ 8'h55;
                exp[15:8] = (i*2+1) ^ 8'h55;
                tests_run = tests_run + 1;
                if (mock_sdram[2560+i] === exp) tests_passed = tests_passed + 1;
                else begin tests_failed = tests_failed + 1; errs = errs + 1; end
            end
            $display("  WRITE SDRAM: %0d errors", errs);
        end

        // Verify flash has block 10 data IMMEDIATELY (no persist wait!)
        // Block 10 flash offset = 0x400000 + 10*512 = 0x401400
        // Flash model addr = 0x1400
        begin : vfy_flash
            integer errs;
            errs = 0;
            for (i = 0; i < 512; i = i + 1) begin
                tests_run = tests_run + 1;
                if (u_flash.mem[16'h1400 + i] === (i[7:0] ^ 8'h55))
                    tests_passed = tests_passed + 1;
                else begin
                    tests_failed = tests_failed + 1;
                    errs = errs + 1;
                    if (errs < 5)
                        $display("  FAIL: flash[%04X] = $%02X, expected $%02X",
                                 16'h1400+i, u_flash.mem[16'h1400+i], i[7:0] ^ 8'h55);
                end
            end
            $display("  FLASH verify block 10: %0d errors", errs);
        end

        // Verify other blocks in sector 1 are correct (from SDRAM, should be 0x00)
        begin : vfy_sector_other
            integer errs;
            errs = 0;
            tests_run = tests_run + 1;
            if (u_flash.mem[16'h1000] === 8'h00) tests_passed = tests_passed + 1;
            else begin
                tests_failed = tests_failed + 1; errs = errs + 1;
                $display("  FAIL: flash[1000] = $%02X, expected $00", u_flash.mem[16'h1000]);
            end
            $display("  FLASH other blocks in sector: %0d errors", errs);
        end

        // =============================================================
        // TEST 3: READ block 5 still works after write-through
        // =============================================================
        $display("\n--- RE-READ block 5 after write-through ---");
        bus_write(4'h2, 8'h05); bus_write(4'h3, 8'h00);
        bus_write(4'h0, 8'h01);
        wait_ready;
        begin : vfy_reread
            integer errs;
            errs = 0;
            for (i = 0; i < 512; i = i + 1) begin
                bus_read(4'h1, rd);
                tests_run = tests_run + 1;
                if (i[0] == 0) begin
                    if (rd === i[8:1]) tests_passed = tests_passed + 1;
                    else begin tests_failed = tests_failed + 1; errs = errs+1; end
                end else begin
                    if (rd === (i[8:1] ^ 8'hA5)) tests_passed = tests_passed + 1;
                    else begin tests_failed = tests_failed + 1; errs = errs+1; end
                end
            end
            $display("  RE-READ block 5: %0d errors", errs);
        end

        // =============================================================
        // TEST 4: Second write to same sector — verify re-erase works
        // =============================================================
        $display("\n--- WRITE block 9 (same sector as block 10) ---");
        bus_write(4'h2, 8'h09); bus_write(4'h3, 8'h00);
        for (i = 0; i < 512; i = i + 1)
            bus_write(4'h5, i[7:0] ^ 8'hBB);
        bus_write(4'h0, 8'h02);
        wait_ready;

        // Verify block 9 in flash
        begin : vfy_flash2
            integer errs;
            errs = 0;
            for (i = 0; i < 512; i = i + 1) begin
                tests_run = tests_run + 1;
                if (u_flash.mem[16'h1200 + i] === (i[7:0] ^ 8'hBB))
                    tests_passed = tests_passed + 1;
                else begin
                    tests_failed = tests_failed + 1;
                    errs = errs + 1;
                    if (errs < 5)
                        $display("  FAIL: flash[%04X] = $%02X, expected $%02X",
                                 16'h1200+i, u_flash.mem[16'h1200+i], i[7:0] ^ 8'hBB);
                end
            end
            $display("  FLASH verify block 9: %0d errors", errs);
        end

        // Block 10 should still be intact after sector re-erase+reprogram
        begin : vfy_flash_intact
            integer errs;
            errs = 0;
            for (i = 0; i < 512; i = i + 1) begin
                tests_run = tests_run + 1;
                if (u_flash.mem[16'h1400 + i] === (i[7:0] ^ 8'h55))
                    tests_passed = tests_passed + 1;
                else begin
                    tests_failed = tests_failed + 1;
                    errs = errs + 1;
                    if (errs < 5)
                        $display("  FAIL: flash[%04X] = $%02X, expected $%02X",
                                 16'h1400+i, u_flash.mem[16'h1400+i], i[7:0] ^ 8'h55);
                end
            end
            $display("  FLASH block 10 still intact: %0d errors", errs);
        end

        // =============================================================
        // Summary
        // =============================================================
        $display("\n========================================");
        $display("  Tests: %0d  Passed: %0d  Failed: %0d",
                 tests_run, tests_passed, tests_failed);
        $display("========================================\n");
        if (tests_failed > 0) $display("*** SOME TESTS FAILED ***");
        else                  $display("*** ALL TESTS PASSED ***");
        $finish;
    end

    // Watchdog
    initial begin
        #500_000_000;
        $display("WATCHDOG TIMEOUT");
        $finish;
    end

endmodule
