`timescale 1ns / 1ps
// =============================================================================
// flash_writer_tb.v — Unit test for SPI flash write controller
// =============================================================================
// Extended SPI flash model supports: READ (0x03), WREN (0x06),
// SECTOR_ERASE (0x20), PAGE_PROGRAM (0x02), READ_STATUS (0x05).

// =============================================================================
// SPI Flash Behavioral Model with Write Support
// =============================================================================
module spi_flash_write_model #(
    parameter MEM_SIZE   = 8192,  // 8KB (2 sectors)
    parameter ERASE_WIP  = 4,     // WIP cycles after erase (fast for sim)
    parameter PROG_WIP   = 2      // WIP cycles after page program
)(
    input  wire ncs,
    input  wire sck,
    input  wire mosi,
    output reg  miso
);

    reg [7:0] mem [0:MEM_SIZE-1];

    // Internal state
    reg        wel;              // Write Enable Latch
    reg        wip;              // Write In Progress
    reg [7:0]  cmd_reg;
    reg [23:0] addr_reg;
    reg [5:0]  bit_cnt;
    reg        cmd_phase;        // 1=receiving cmd+addr, 0=data phase
    reg [7:0]  data_byte;
    reg [2:0]  out_bit_cnt;
    reg [23:0] current_addr;
    reg [7:0]  status_reg;
    reg        in_status_read;   // reading status register continuously
    reg [15:0] wip_counter;      // countdown for WIP simulation
    reg [8:0]  pp_byte_cnt;      // page program byte counter

    integer i;

    initial begin
        for (i = 0; i < MEM_SIZE; i = i + 1)
            mem[i] = 8'hFF;  // erased state
        miso         = 1'bz;
        wel          = 1'b0;
        wip          = 1'b0;
        bit_cnt      = 6'd0;
        cmd_phase    = 1'b1;
        cmd_reg      = 8'd0;
        addr_reg     = 24'd0;
        in_status_read = 1'b0;
        wip_counter  = 16'd0;
        pp_byte_cnt  = 9'd0;
    end

    // WIP countdown (on clock edge, independent of SPI)
    always @(posedge sck) begin
        if (wip_counter > 0) begin
            wip_counter <= wip_counter - 1;
            if (wip_counter == 1)
                wip <= 1'b0;
        end
    end

    // CS deassert — complete pending operations
    always @(posedge ncs) begin
        bit_cnt      <= 6'd0;
        cmd_phase    <= 1'b1;
        out_bit_cnt  <= 3'd0;
        miso         <= 1'bz;
        in_status_read <= 1'b0;

        // Complete command on CS rising edge
        case (cmd_reg)
            8'h06: begin  // WREN
                wel <= 1'b1;
            end
            8'h20: begin  // SECTOR ERASE (4KB)
                if (wel && !wip) begin
                    // Erase 4KB sector
                    for (i = 0; i < 4096; i = i + 1) begin
                        if ((addr_reg[12:0] & 13'hF000) + i[12:0] < MEM_SIZE)
                            mem[(addr_reg[12:0] & 13'hF000) + i[12:0]] = 8'hFF;
                    end
                    wip <= 1'b1;
                    wip_counter <= ERASE_WIP;
                    wel <= 1'b0;
                end
            end
            8'h02: begin  // PAGE PROGRAM — data already written during SPI clocking
                if (wel && !wip) begin
                    wip <= 1'b1;
                    wip_counter <= PROG_WIP;
                    wel <= 1'b0;
                end
            end
        endcase
        cmd_reg <= 8'd0;
    end

    // CS assert — reset reception
    always @(negedge ncs) begin
        bit_cnt      <= 6'd0;
        cmd_phase    <= 1'b1;
        cmd_reg      <= 8'd0;
        addr_reg     <= 24'd0;
        pp_byte_cnt  <= 9'd0;
        in_status_read <= 1'b0;
    end

    // Sample MOSI on SCK rising edge
    always @(posedge sck) begin
        if (!ncs) begin
            if (cmd_phase) begin
                if (bit_cnt < 6'd8) begin
                    cmd_reg <= {cmd_reg[6:0], mosi};
                end else if (bit_cnt < 6'd32) begin
                    addr_reg <= {addr_reg[22:0], mosi};
                end

                bit_cnt <= bit_cnt + 6'd1;

                // After 8 bits: check if command needs address
                if (bit_cnt == 6'd7) begin
                    case ({cmd_reg[6:0], mosi})
                        8'h06: begin  // WREN — no address, done
                            cmd_reg <= {cmd_reg[6:0], mosi};
                        end
                        8'h05: begin  // READ_STATUS — no address, go to data
                            cmd_reg   <= {cmd_reg[6:0], mosi};
                            cmd_phase <= 1'b0;
                            in_status_read <= 1'b1;
                            out_bit_cnt <= 3'd0;
                        end
                    endcase
                end

                // After 32 bits (cmd + 24-bit addr): enter data phase
                if (bit_cnt == 6'd31) begin
                    cmd_phase   <= 1'b0;
                    current_addr <= {addr_reg[22:0], mosi};
                    out_bit_cnt <= 3'd0;
                end
            end else if (cmd_reg == 8'h02 && !in_status_read) begin
                // PAGE PROGRAM data reception
                data_byte <= {data_byte[6:0], mosi};
                out_bit_cnt <= out_bit_cnt + 3'd1;
                if (out_bit_cnt == 3'd7) begin
                    // Program byte: can only clear bits (1->0)
                    if (current_addr[12:0] < MEM_SIZE)
                        mem[current_addr[12:0]] <= mem[current_addr[12:0]] & {data_byte[6:0], mosi};
                    current_addr <= current_addr + 24'd1;
                    pp_byte_cnt  <= pp_byte_cnt + 9'd1;
                    out_bit_cnt  <= 3'd0;
                end
            end
        end
    end

    // Drive MISO on SCK falling edge
    always @(negedge sck) begin
        if (!ncs && !cmd_phase) begin
            if (in_status_read) begin
                // Status register: {5'b0, 1'b0, wel, wip}
                status_reg = {5'b0, 1'b0, wel, wip};
                miso <= status_reg[7 - out_bit_cnt];
                out_bit_cnt <= out_bit_cnt + 3'd1;
            end else if (cmd_reg == 8'h03) begin
                // READ data output
                if (out_bit_cnt == 3'd0)
                    data_byte <= (current_addr[12:0] < MEM_SIZE) ? mem[current_addr[12:0]] : 8'hFF;
                miso <= (out_bit_cnt == 3'd0) ?
                        ((current_addr[12:0] < MEM_SIZE) ? mem[current_addr[12:0]][7] : 1'b1) :
                        data_byte[7 - out_bit_cnt];
                out_bit_cnt <= out_bit_cnt + 3'd1;
                if (out_bit_cnt == 3'd7) begin
                    current_addr <= current_addr + 24'd1;
                    out_bit_cnt  <= 3'd0;
                end
            end
        end
    end

endmodule


// =============================================================================
// Testbench
// =============================================================================
module flash_writer_tb;

    // Clock: 25 MHz = 40ns period
    reg clk = 0;
    always #20 clk = ~clk;

    // DUT signals
    reg         rst_n;
    reg         start_erase;
    reg         start_program;
    reg  [23:0] flash_addr;
    reg  [7:0]  prog_data;
    wire        prog_data_req;
    wire        busy;
    wire        done;
    wire        spi_sck, spi_ncs, spi_mosi;
    wire        spi_miso;

    // Test tracking
    integer tests_run = 0;
    integer tests_passed = 0;
    integer tests_failed = 0;
    integer i;
    reg [7:0] prog_byte_idx;

    // ---- DUT ----
    flash_writer dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .start_erase   (start_erase),
        .start_program (start_program),
        .flash_addr    (flash_addr),
        .prog_data     (prog_data),
        .prog_data_valid(1'b1),       // always valid in unit test
        .prog_data_req (prog_data_req),
        .busy          (busy),
        .done          (done),
        .spi_sck       (spi_sck),
        .spi_ncs       (spi_ncs),
        .spi_mosi      (spi_mosi),
        .spi_miso      (spi_miso)
    );

    // ---- Flash model ----
    spi_flash_write_model #(
        .MEM_SIZE  (8192),
        .ERASE_WIP (4),
        .PROG_WIP  (2)
    ) flash (
        .ncs  (spi_ncs),
        .sck  (spi_sck),
        .mosi (spi_mosi),
        .miso (spi_miso)
    );

    // ---- Auto-feed prog_data when requested ----
    always @(posedge clk) begin
        if (prog_data_req)
            prog_byte_idx <= prog_byte_idx + 8'd1;
        prog_data <= prog_byte_idx ^ 8'hA5;  // test pattern
    end

    // ---- Wait for done with timeout ----
    task wait_done(input integer max_cycles);
        integer cyc;
        begin
            cyc = 0;
            while (!done && cyc < max_cycles) begin
                @(posedge clk);
                cyc = cyc + 1;
            end
            if (cyc >= max_cycles) begin
                $display("  FAIL: timeout after %0d cycles", max_cycles);
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
        $dumpfile("flash_writer_tb.vcd");
        $dumpvars(0, flash_writer_tb);

        rst_n         = 0;
        start_erase   = 0;
        start_program = 0;
        flash_addr    = 24'd0;
        prog_data     = 8'd0;
        prog_byte_idx = 8'd0;

        repeat(5) @(posedge clk);
        rst_n = 1;
        repeat(2) @(posedge clk);

        // =============================================================
        // TEST 1: Sector erase — verify 4KB region becomes 0xFF
        // =============================================================
        $display("\n--- Test 1: Sector erase at addr 0x000000 ---");

        // Pre-fill first sector with non-FF data
        for (i = 0; i < 4096; i = i + 1)
            flash.mem[i] = i[7:0];

        // Verify pre-fill
        check("pre-fill[0]", flash.mem[0], 8'h00);
        check("pre-fill[255]", flash.mem[255], 8'hFF);
        check("pre-fill[100]", flash.mem[100], 8'h64);

        // Issue sector erase
        flash_addr = 24'h000000;
        @(posedge clk); start_erase = 1;
        @(posedge clk); start_erase = 0;

        wait_done(50000);

        // Verify erased
        check("erased[0]", flash.mem[0], 8'hFF);
        check("erased[100]", flash.mem[100], 8'hFF);
        check("erased[4095]", flash.mem[4095], 8'hFF);

        // Verify second sector untouched (should be 0xFF from init)
        check("sector2[4096]", flash.mem[4096], 8'hFF);

        $display("  Sector erase: %0d passed", tests_passed);
        repeat(10) @(posedge clk);

        // =============================================================
        // TEST 2: Page program — write 256 bytes
        // =============================================================
        $display("\n--- Test 2: Page program at addr 0x000000 ---");

        prog_byte_idx = 8'd0;  // reset data feed
        flash_addr = 24'h000000;
        @(posedge clk); start_program = 1;
        @(posedge clk); start_program = 0;

        wait_done(100000);

        // Verify programmed data: pattern is idx ^ 0xA5
        begin : test2_verify
            integer errs;
            errs = 0;
            for (i = 0; i < 256; i = i + 1) begin
                tests_run = tests_run + 1;
                if (flash.mem[i] === (i[7:0] ^ 8'hA5)) begin
                    tests_passed = tests_passed + 1;
                end else begin
                    tests_failed = tests_failed + 1;
                    errs = errs + 1;
                    if (errs < 5)
                        $display("  FAIL: mem[%0d] = $%02X, expected $%02X",
                                 i, flash.mem[i], i[7:0] ^ 8'hA5);
                end
            end
            $display("  Page program: %0d errors out of 256 bytes", errs);
        end

        repeat(10) @(posedge clk);

        // =============================================================
        // TEST 3: Erase + program a different sector
        // =============================================================
        $display("\n--- Test 3: Erase sector 1, program page at 0x001000 ---");

        // Erase sector 1 (addr 0x1000)
        flash_addr = 24'h001000;
        @(posedge clk); start_erase = 1;
        @(posedge clk); start_erase = 0;
        wait_done(50000);

        // Program page at 0x1000
        prog_byte_idx = 8'h42;  // start pattern at 0x42
        flash_addr = 24'h001000;
        @(posedge clk); start_program = 1;
        @(posedge clk); start_program = 0;
        wait_done(100000);

        // Verify: pattern is (0x42 + idx) ^ 0xA5
        begin : test3_verify
            integer errs;
            errs = 0;
            for (i = 0; i < 256; i = i + 1) begin
                tests_run = tests_run + 1;
                if (flash.mem[4096 + i] === ((8'h42 + i[7:0]) ^ 8'hA5)) begin
                    tests_passed = tests_passed + 1;
                end else begin
                    tests_failed = tests_failed + 1;
                    errs = errs + 1;
                    if (errs < 5)
                        $display("  FAIL: mem[%0d] = $%02X, expected $%02X",
                                 4096+i, flash.mem[4096+i], (8'h42 + i[7:0]) ^ 8'hA5);
                end
            end
            $display("  Sector 1 program: %0d errors out of 256 bytes", errs);
        end

        // Verify sector 0 data (from test 2) is still intact
        check("sector0[0] intact", flash.mem[0], 8'h00 ^ 8'hA5);
        check("sector0[127] intact", flash.mem[127], 8'h7F ^ 8'hA5);

        repeat(10) @(posedge clk);

        // =============================================================
        // TEST 4: Busy/done signal behavior
        // =============================================================
        $display("\n--- Test 4: Busy/done signals ---");
        tests_run = tests_run + 1;
        if (!busy && !done) begin
            tests_passed = tests_passed + 1;
            $display("  PASS: idle state correct");
        end else begin
            tests_failed = tests_failed + 1;
            $display("  FAIL: not idle after operations");
        end

        // =============================================================
        // Summary
        // =============================================================
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

    // Watchdog
    initial begin
        #50_000_000;
        $display("WATCHDOG TIMEOUT");
        $finish;
    end

endmodule
