// boot_path_tb.v — Full boot-path verification testbench
// Tests: flash_reader → boot_loader → sdram_arbiter → sdram_controller → SDRAM model
// Verifies data integrity at specific ProDOS block addresses after boot completes.
//
// Flash pattern: every byte in block N = (N & 0xFF).
// Expected SDRAM word in block N = {N[7:0], N[7:0]}.
//
// This test covers the path that sp_validate_tb.v (which mocks SDRAM) does NOT test.

`define SIMULATION
`timescale 1ns / 1ps

module boot_path_tb;

    // =========================================================================
    // Parameters
    // =========================================================================
    localparam FLASH_OFFSET = 24'h400000;
    localparam IMAGE_SIZE   = 24'h023000;  // 143,360 bytes = 280 blocks

    // =========================================================================
    // Clock and Reset
    // =========================================================================
    reg clk = 0;
    always #20 clk = ~clk;  // 25 MHz

    reg rst_n = 0;

    // =========================================================================
    // Test Infrastructure
    // =========================================================================
    integer pass_count = 0;
    integer fail_count = 0;

    task check(input [511:0] name, input condition);
    begin
        if (condition) begin
            $display("  PASS: %0s", name);
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: %0s", name);
            fail_count = fail_count + 1;
        end
    end
    endtask

    // =========================================================================
    // Internal Wires — Flash Reader ↔ Boot Loader
    // =========================================================================
    wire        flash_start;
    wire [23:0] flash_start_addr;
    wire [23:0] flash_byte_count;
    wire        flash_busy, flash_done;
    wire [7:0]  flash_data_out;
    wire        flash_data_valid;
    wire        flash_data_ready;
    wire        flash_ncs, flash_mosi, flash_nwp, flash_nhold, flash_sck;

    // =========================================================================
    // Internal Wires — Boot Loader ↔ Arbiter
    // =========================================================================
    wire        boot_done;
    wire        boot_sdram_req, boot_sdram_write;
    wire [25:0] boot_sdram_addr;
    wire [15:0] boot_sdram_wdata;
    wire        boot_sdram_ready;

    // =========================================================================
    // Internal Wires — Arbiter ↔ SDRAM Controller
    // =========================================================================
    wire        arb_sdram_req, arb_sdram_write;
    wire [25:0] arb_sdram_addr;
    wire [15:0] arb_sdram_wdata;
    wire        sdram_req_ready;
    wire [15:0] sdram_req_rdata;
    wire        sdram_req_rdata_valid;
    wire        sdram_init_done;

    // =========================================================================
    // SDRAM Physical Bus
    // =========================================================================
    wire        sdram_CLK, sdram_CKE;
    wire        sdram_nCS, sdram_nRAS, sdram_nCAS, sdram_nWE;
    wire        sdram_DQM0, sdram_DQM1;
    wire        sdram_BA0, sdram_BA1;
    wire [12:0] sdram_A;

    // DQ bus — individual wires for iverilog tristate compatibility
    wire sd0, sd1, sd2, sd3, sd4, sd5, sd6, sd7;
    wire sd8, sd9, sd10, sd11, sd12, sd13, sd14, sd15;
    wire [15:0] sdram_DQ = {sd15, sd14, sd13, sd12, sd11, sd10, sd9, sd8,
                             sd7, sd6, sd5, sd4, sd3, sd2, sd1, sd0};

    // =========================================================================
    // Behavioral SDRAM Chip Model
    // =========================================================================
    // 256K x 16-bit words. Index = {bank[0], row[6:0], col[9:0]}.
    // Handles ACTIVATE, WRITE, READ (CAS=2). Sufficient for 280-block image.
    // =========================================================================
    reg [15:0] sdram_mem [0:262143];  // 256K words = 512KB
    reg [15:0] chip_dq_out;
    reg        chip_dq_oe = 0;
    reg [17:0] chip_read_idx;
    reg [2:0]  chip_cas_delay = 0;
    reg [12:0] chip_active_row = 0;
    reg [1:0]  chip_active_bank = 0;

    wire [3:0] chip_cmd = {sdram_nCS, sdram_nRAS, sdram_nCAS, sdram_nWE};

    // Drive DQ pins from model
    assign sd0  = chip_dq_oe ? chip_dq_out[0]  : 1'bZ;
    assign sd1  = chip_dq_oe ? chip_dq_out[1]  : 1'bZ;
    assign sd2  = chip_dq_oe ? chip_dq_out[2]  : 1'bZ;
    assign sd3  = chip_dq_oe ? chip_dq_out[3]  : 1'bZ;
    assign sd4  = chip_dq_oe ? chip_dq_out[4]  : 1'bZ;
    assign sd5  = chip_dq_oe ? chip_dq_out[5]  : 1'bZ;
    assign sd6  = chip_dq_oe ? chip_dq_out[6]  : 1'bZ;
    assign sd7  = chip_dq_oe ? chip_dq_out[7]  : 1'bZ;
    assign sd8  = chip_dq_oe ? chip_dq_out[8]  : 1'bZ;
    assign sd9  = chip_dq_oe ? chip_dq_out[9]  : 1'bZ;
    assign sd10 = chip_dq_oe ? chip_dq_out[10] : 1'bZ;
    assign sd11 = chip_dq_oe ? chip_dq_out[11] : 1'bZ;
    assign sd12 = chip_dq_oe ? chip_dq_out[12] : 1'bZ;
    assign sd13 = chip_dq_oe ? chip_dq_out[13] : 1'bZ;
    assign sd14 = chip_dq_oe ? chip_dq_out[14] : 1'bZ;
    assign sd15 = chip_dq_oe ? chip_dq_out[15] : 1'bZ;

    integer sdram_write_count = 0;
    integer sdram_init_i;

    initial begin
        for (sdram_init_i = 0; sdram_init_i < 262144; sdram_init_i = sdram_init_i + 1)
            sdram_mem[sdram_init_i] = 16'hDEAD;  // Poison — easy to spot if never written
        chip_dq_out = 16'h0000;
    end

    // SDRAM model index: {bank[0], row[6:0], col[9:0]} = 18 bits
    wire [17:0] chip_wr_idx = {chip_active_bank[0], chip_active_row[6:0], sdram_A[9:0]};

    always @(posedge sdram_CLK) begin
        // CAS latency pipeline
        if (chip_cas_delay > 0) begin
            chip_cas_delay <= chip_cas_delay - 3'd1;
            if (chip_cas_delay == 3'd1) begin
                chip_dq_oe  <= 1'b1;
                chip_dq_out <= sdram_mem[chip_read_idx];
            end
        end else begin
            chip_dq_oe <= 1'b0;
        end

        // Command processing
        case (chip_cmd)
            4'b0011: begin // ACTIVATE
                chip_active_row  <= sdram_A;
                chip_active_bank <= {sdram_BA1, sdram_BA0};
            end
            4'b0100: begin // WRITE
                sdram_mem[chip_wr_idx] <= sdram_DQ;
                sdram_write_count = sdram_write_count + 1;
            end
            4'b0101: begin // READ
                chip_read_idx   <= chip_wr_idx;  // same index formula
                chip_cas_delay  <= 3'd1;         // CAS=2 effective (model 1-cycle late)
            end
            default: ; // NOP, REFRESH, PRECHARGE, LOAD_MODE — no-op
        endcase
    end

    // =========================================================================
    // Behavioral SPI Flash Model
    // =========================================================================
    // 256KB memory. Pattern: byte at image offset O = (O >> 9) & 0xFF
    // i.e., every byte in block N has value (N & 0xFF).
    // =========================================================================
    reg [7:0] flash_mem [0:262143];  // 256KB

    integer flash_init_i;
    initial begin
        for (flash_init_i = 0; flash_init_i < 262144; flash_init_i = flash_init_i + 1)
            flash_mem[flash_init_i] = (flash_init_i >> 9) & 8'hFF;
    end

    // SPI state machine
    reg [7:0]  spi_cmd_reg = 0;
    reg [23:0] spi_addr_reg = 0;
    reg [5:0]  spi_bit_cnt = 0;
    reg        spi_cmd_phase = 1;
    reg [7:0]  spi_data_byte = 0;
    reg [2:0]  spi_out_bit_cnt = 0;
    reg [23:0] spi_read_addr = 0;
    reg        flash_miso_reg;
    assign flash_miso_reg_w = flash_miso_reg;

    // Get SCK from flash_reader (simulation only)
    wire spi_sck = flash_sck;

    // CS deasserted → reset
    always @(posedge flash_ncs) begin
        spi_bit_cnt   <= 6'd0;
        spi_cmd_phase <= 1'b1;
    end

    // CS asserted → prepare
    always @(negedge flash_ncs) begin
        spi_bit_cnt   <= 6'd0;
        spi_cmd_phase <= 1'b1;
        spi_cmd_reg   <= 8'd0;
        spi_addr_reg  <= 24'd0;
    end

    // Sample MOSI on SCK rising edge (command + address phase)
    always @(posedge spi_sck) begin
        if (!flash_ncs && spi_cmd_phase) begin
            if (spi_bit_cnt < 6'd8)
                spi_cmd_reg <= {spi_cmd_reg[6:0], flash_mosi};
            else if (spi_bit_cnt < 6'd32)
                spi_addr_reg <= {spi_addr_reg[22:0], flash_mosi};

            spi_bit_cnt <= spi_bit_cnt + 6'd1;

            if (spi_bit_cnt == 6'd31) begin
                spi_cmd_phase   <= 1'b0;
                spi_read_addr   <= {spi_addr_reg[22:0], flash_mosi} - FLASH_OFFSET;
                spi_out_bit_cnt <= 3'd0;
            end
        end
    end

    // Drive MISO on SCK falling edge (data phase)
    always @(negedge spi_sck) begin
        if (!flash_ncs && !spi_cmd_phase) begin
            if (spi_out_bit_cnt == 3'd0)
                spi_data_byte <= flash_mem[spi_read_addr[17:0]];

            flash_miso_reg <= (spi_out_bit_cnt == 3'd0) ?
                flash_mem[spi_read_addr[17:0]][7] :
                spi_data_byte[7 - spi_out_bit_cnt];

            spi_out_bit_cnt <= spi_out_bit_cnt + 3'd1;
            if (spi_out_bit_cnt == 3'd7) begin
                spi_read_addr   <= spi_read_addr + 24'd1;
                spi_out_bit_cnt <= 3'd0;
            end
        end
    end

    // =========================================================================
    // DUT: Flash Reader
    // =========================================================================
    flash_reader u_flash_reader (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (flash_start),
        .start_addr (flash_start_addr),
        .byte_count (flash_byte_count),
        .busy       (flash_busy),
        .done       (flash_done),
        .data_out   (flash_data_out),
        .data_valid (flash_data_valid),
        .data_ready (flash_data_ready),
        .flash_ncs  (flash_ncs),
        .flash_mosi (flash_mosi),
        .flash_miso (flash_miso_reg),
        .flash_nwp  (flash_nwp),
        .flash_nhold(flash_nhold),
        .flash_sck_pin(flash_sck)
    );

    // =========================================================================
    // DUT: Boot Loader
    // =========================================================================
    boot_loader #(
        .FLASH_OFFSET(FLASH_OFFSET),
        .IMAGE_SIZE  (IMAGE_SIZE)
    ) u_boot_loader (
        .clk             (clk),
        .rst_n           (rst_n),
        .sdram_init_done (sdram_init_done),
        .boot_done       (boot_done),
        .flash_start     (flash_start),
        .flash_addr      (flash_start_addr),
        .flash_count     (flash_byte_count),
        .flash_busy      (flash_busy),
        .flash_data      (flash_data_out),
        .flash_data_valid(flash_data_valid),
        .flash_data_ready(flash_data_ready),
        .sdram_req       (boot_sdram_req),
        .sdram_req_write (boot_sdram_write),
        .sdram_req_addr  (boot_sdram_addr),
        .sdram_req_wdata (boot_sdram_wdata),
        .sdram_req_ready (boot_sdram_ready)
    );

    // =========================================================================
    // DUT: SDRAM Arbiter (provides registered boot passthrough)
    // =========================================================================
    // Block buffer ports — unused during boot, tie off
    wire [8:0] buf_addr_a;
    wire [7:0] buf_wdata_a;
    wire       buf_we_a;
    wire [7:0] buf_rdata_a = 8'd0;  // not used during boot

    sdram_arbiter u_arbiter (
        .clk                (clk),
        .rst_n              (rst_n),
        .boot_done          (boot_done),
        // Boot loader port
        .boot_req           (boot_sdram_req),
        .boot_write         (boot_sdram_write),
        .boot_addr          (boot_sdram_addr),
        .boot_wdata         (boot_sdram_wdata),
        .boot_ready         (boot_sdram_ready),
        // Device engine — no runtime requests during boot test
        .dev_block_read_req (1'b0),
        .dev_block_write_req(1'b0),
        .dev_block_num      (16'd0),
        .dev_block_ready    (),
        // Block buffer (unused during boot)
        .buf_addr_a         (buf_addr_a),
        .buf_wdata_a        (buf_wdata_a),
        .buf_we_a           (buf_we_a),
        .buf_rdata_a        (buf_rdata_a),
        // SDRAM controller
        .sdram_req          (arb_sdram_req),
        .sdram_write        (arb_sdram_write),
        .sdram_addr         (arb_sdram_addr),
        .sdram_wdata        (arb_sdram_wdata),
        .sdram_ready        (sdram_req_ready),
        .sdram_rdata        (sdram_req_rdata),
        .sdram_rdata_valid  (sdram_req_rdata_valid)
    );

    // =========================================================================
    // DUT: SDRAM Controller
    // =========================================================================
    sdram_controller u_sdram_ctrl (
        .clk            (clk),
        .rst_n          (rst_n),
        .init_done      (sdram_init_done),
        .pause_refresh  (1'b0),
        .req            (arb_sdram_req),
        .req_write      (arb_sdram_write),
        .req_addr       (arb_sdram_addr),
        .req_wdata      (arb_sdram_wdata),
        .req_ready      (sdram_req_ready),
        .req_rdata      (sdram_req_rdata),
        .req_rdata_valid(sdram_req_rdata_valid),
        .SDRAM_CLK      (sdram_CLK),
        .SDRAM_CKE      (sdram_CKE),
        .SDRAM_nCS      (sdram_nCS),
        .SDRAM_nRAS     (sdram_nRAS),
        .SDRAM_nCAS     (sdram_nCAS),
        .SDRAM_nWE      (sdram_nWE),
        .SDRAM_DQM0     (sdram_DQM0),
        .SDRAM_DQM1     (sdram_DQM1),
        .SDRAM_BA0      (sdram_BA0),
        .SDRAM_BA1      (sdram_BA1),
        .SDRAM_A        (sdram_A),
        .SDRAM_DQ       (sdram_DQ)
    );

    // =========================================================================
    // Verification Helpers
    // =========================================================================

    // Convert ProDOS block number to SDRAM model index for word 0 of that block.
    // Block N → byte address N*512 → SDRAM addr decomposition:
    //   row = addr[23:11], col = addr[10:1], bank = addr[25:24]
    // Model index = {bank[0], row[6:0], col[9:0]}
    function [17:0] block_to_idx;
        input [15:0] block_num;
        input [8:0]  word_offset;  // 0-255
        reg [25:0] byte_addr;
        reg [9:0]  col;
        reg [12:0] row;
        reg [1:0]  bank;
        begin
            byte_addr = {10'd0, block_num} * 26'd512 + {17'd0, word_offset, 1'b0};
            col  = byte_addr[10:1];
            row  = byte_addr[23:11];
            bank = byte_addr[25:24];
            block_to_idx = {bank[0], row[6:0], col};
        end
    endfunction

    // =========================================================================
    // Main Test Sequence
    // =========================================================================
    integer timeout;
    integer block_i, word_i;
    integer errors_in_block;
    reg [15:0] expected_word;
    reg [15:0] actual_word;
    reg [17:0] idx;

    // Test blocks: same as hardware test
    reg [15:0] test_blocks [0:4];
    initial begin
        test_blocks[0] = 16'd7;
        test_blocks[1] = 16'd64;
        test_blocks[2] = 16'd117;
        test_blocks[3] = 16'd167;
        test_blocks[4] = 16'd217;
    end

    initial begin
        $dumpfile("boot_path_tb.vcd");
        $dumpvars(0, boot_path_tb);

        $display("");
        $display("==============================================");
        $display("Boot Path Verification Testbench");
        $display("==============================================");
        $display("Image: %0d bytes (%0d blocks) from flash 0x%06h",
                 IMAGE_SIZE, IMAGE_SIZE / 512, FLASH_OFFSET);

        // Reset
        rst_n = 0;
        #200;
        rst_n = 1;

        // -----------------------------------------------------------------
        // Test 1: Wait for SDRAM init
        // -----------------------------------------------------------------
        $display("\n--- Test 1: SDRAM Initialization ---");
        timeout = 0;
        while (!sdram_init_done && timeout < 300000) begin
            @(posedge clk);
            timeout = timeout + 1;
        end
        check("SDRAM init_done asserted", sdram_init_done === 1'b1);

        // -----------------------------------------------------------------
        // Test 2: Wait for boot to complete
        // -----------------------------------------------------------------
        $display("\n--- Test 2: Boot Completion ---");
        timeout = 0;
        while (!boot_done && timeout < 10000000) begin
            @(posedge clk);
            timeout = timeout + 1;
            // Progress indicator every 1M clocks
            if (timeout % 1000000 == 0)
                $display("    ... %0d M clocks, %0d SDRAM writes so far",
                         timeout / 1000000, sdram_write_count);
        end
        check("boot_done asserted", boot_done === 1'b1);

        // Wait for the SDRAM controller to finish processing the last write.
        // boot_done fires when the controller ACCEPTS the last request, but
        // the ACTIVATE → WRITE_CMD → WRITE_WAIT pipeline takes ~8 more cycles.
        repeat (20) @(posedge clk);

        $display("    Boot completed after %0d clocks (%0d us), %0d SDRAM writes",
                 timeout, timeout / 25, sdram_write_count);

        // Expected: IMAGE_SIZE / 2 = 71,680 words
        check("Correct number of SDRAM writes (71680)",
              sdram_write_count == IMAGE_SIZE / 2);

        // -----------------------------------------------------------------
        // Test 3: Verify SDRAM data at test blocks
        // -----------------------------------------------------------------
        $display("\n--- Test 3: Block Data Verification ---");

        for (block_i = 0; block_i < 5; block_i = block_i + 1) begin
            errors_in_block = 0;
            expected_word = {test_blocks[block_i][7:0], test_blocks[block_i][7:0]};

            // Check all 256 words (512 bytes) in the block
            for (word_i = 0; word_i < 256; word_i = word_i + 1) begin
                idx = block_to_idx(test_blocks[block_i], word_i[8:0]);
                actual_word = sdram_mem[idx];
                if (actual_word !== expected_word) begin
                    if (errors_in_block < 3)  // Only print first 3 errors per block
                        $display("    Block %0d word %0d: got 0x%04h, expected 0x%04h (idx=%0d)",
                                 test_blocks[block_i], word_i, actual_word, expected_word, idx);
                    errors_in_block = errors_in_block + 1;
                end
            end

            if (errors_in_block == 0) begin
                $display("  PASS: Block %0d — all 256 words = 0x%04h",
                         test_blocks[block_i], expected_word);
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL: Block %0d — %0d/256 words wrong",
                         test_blocks[block_i], errors_in_block);
                fail_count = fail_count + 1;
            end
        end

        // -----------------------------------------------------------------
        // Test 4: Spot-check a few boundary blocks
        // -----------------------------------------------------------------
        $display("\n--- Test 4: Boundary Block Checks ---");

        // Block 0 (first)
        idx = block_to_idx(16'd0, 9'd0);
        check("Block 0 word 0 = 0x0000", sdram_mem[idx] === 16'h0000);

        // Block 1
        idx = block_to_idx(16'd1, 9'd0);
        check("Block 1 word 0 = 0x0101", sdram_mem[idx] === 16'h0101);

        // Block 255
        idx = block_to_idx(16'd255, 9'd0);
        check("Block 255 word 0 = 0xFFFF", sdram_mem[idx] === 16'hFFFF);

        // Block 279 (last block in 280-block image)
        idx = block_to_idx(16'd279, 9'd0);
        expected_word = {8'h17, 8'h17};  // 279 & 0xFF = 0x17
        check("Block 279 word 0 = 0x1717", sdram_mem[idx] === 16'h1717);

        // -----------------------------------------------------------------
        // Test 5: Full sequential scan — check every word
        // -----------------------------------------------------------------
        $display("\n--- Test 5: Full Image Verification ---");
        begin : full_scan
            integer total_errors;
            integer word_addr;
            integer block;
            reg [7:0] expected_byte;
            total_errors = 0;

            for (word_addr = 0; word_addr < IMAGE_SIZE / 2; word_addr = word_addr + 1) begin
                block = (word_addr * 2) / 512;
                expected_byte = block[7:0];
                expected_word = {expected_byte, expected_byte};
                idx = block_to_idx(block[15:0], (word_addr - block * 256) & 9'h1FF);
                actual_word = sdram_mem[idx];
                if (actual_word !== expected_word) begin
                    if (total_errors < 10)
                        $display("    Word %0d (block %0d): got 0x%04h, expected 0x%04h",
                                 word_addr, block, actual_word, expected_word);
                    total_errors = total_errors + 1;
                end
            end

            if (total_errors == 0) begin
                $display("  PASS: All %0d words verified correct", IMAGE_SIZE / 2);
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL: %0d/%0d words incorrect", total_errors, IMAGE_SIZE / 2);
                fail_count = fail_count + 1;
            end
        end

        // -----------------------------------------------------------------
        // Summary
        // -----------------------------------------------------------------
        $display("");
        $display("==============================================");
        $display("  RESULTS: %0d PASSED, %0d FAILED", pass_count, fail_count);
        $display("==============================================");
        if (fail_count == 0)
            $display("  ALL TESTS PASSED");
        else
            $display("  SOME TESTS FAILED");
        $display("==============================================");
        $display("");

        #100;
        $finish;
    end

    // =========================================================================
    // Timeout Watchdog — 10M clocks = 400ms at 25 MHz
    // =========================================================================
    initial begin
        #400000000;  // 400ms
        $display("FAIL: Global timeout — simulation took too long");
        $finish;
    end

endmodule
