// sd_persist_tb.v — Test sd_persist (buffer-free version)
// Reads from block_buffer, writes to SD via sd_controller mock.

`timescale 1ns / 1ps

module sd_persist_tb;

    reg clk = 0;
    always #20 clk = ~clk;

    reg rst_n = 0;
    reg start = 0;
    reg [15:0] block_num = 0;
    wire busy;
    reg is_2mg = 0;
    reg [15:0] data_offset = 0;
    reg persist_enabled = 1;
    reg [31:0] fat32_data_start = 32'd2592;
    reg [7:0]  fat32_spc = 8'd64;

    // Chain map mock
    wire [9:0] chain_rd_addr;
    reg [31:0] chain_rd_data;
    reg [31:0] chain_map [0:15];
    always @(posedge clk) chain_rd_data <= chain_map[chain_rd_addr];

    // Block buffer mock (simulates block_buffer port A)
    reg [7:0] buf_mem [0:511];
    wire [8:0] buf_rd_addr;
    wire [7:0] buf_rd_data = buf_mem[buf_rd_addr];

    wire sdram_claim;

    // SD write mock
    wire sd_write_start;
    wire [31:0] sd_write_addr;
    wire [7:0] sd_write_data;
    wire sd_write_data_valid;
    reg sd_write_data_req = 0;
    reg sd_write_done = 0;

    reg [7:0] sd_captured [0:511];
    integer sd_cap_idx = 0;
    reg [31:0] sd_write_addr_cap;

    sd_persist #(.UNIT2_OFFSET(16'd2048)) dut (
        .clk(clk), .rst_n(rst_n),
        .start(start), .block_num(block_num), .busy(busy),
        .is_2mg(is_2mg), .data_offset(data_offset),
        .persist_enabled(persist_enabled),
        .fat32_data_start(fat32_data_start), .fat32_spc(fat32_spc),
        .chain_rd_addr(chain_rd_addr), .chain_rd_data(chain_rd_data),
        .buf_rd_addr(buf_rd_addr), .buf_rd_data(buf_rd_data),
        .sdram_claim(sdram_claim),
        .sd_write_start(sd_write_start), .sd_write_addr(sd_write_addr),
        .sd_write_data(sd_write_data), .sd_write_data_valid(sd_write_data_valid),
        .sd_write_data_req(sd_write_data_req), .sd_write_done(sd_write_done)
    );

    // SD write capture mock
    always @(posedge clk) begin
        sd_write_data_req <= 1'b0;
        sd_write_done     <= 1'b0;

        if (sd_write_start) begin
            sd_write_addr_cap <= sd_write_addr;
            sd_cap_idx <= 0;
        end

        if (dut.state == dut.SP_SD_STREAM) begin
            if (sd_write_data_valid) begin
                sd_captured[sd_cap_idx] <= sd_write_data;
                sd_cap_idx <= sd_cap_idx + 1;
                if (sd_cap_idx < 511)
                    sd_write_data_req <= 1'b1;
                else
                    sd_write_done <= 1'b1;
            end else begin
                sd_write_data_req <= 1'b1;
            end
        end
    end

    // Scoreboard
    integer tests_run = 0, tests_passed = 0, tests_failed = 0;
    task check;
        input [255:0] label;
        input [31:0] expected, actual;
        begin
            tests_run = tests_run + 1;
            if (expected === actual) tests_passed = tests_passed + 1;
            else begin
                tests_failed = tests_failed + 1;
                $display("FAIL: %0s — expected 0x%0h, got 0x%0h", label, expected, actual);
            end
        end
    endtask

    integer i;

    initial begin
        $dumpfile("sd_persist_tb.vcd");
        $dumpvars(0, sd_persist_tb);

        for (i = 0; i < 16; i = i + 1) chain_map[i] = 32'd0;
        chain_map[0] = 32'd3;

        // Fill block buffer with test pattern
        for (i = 0; i < 512; i = i + 1)
            buf_mem[i] = (i * 3 + 5) & 8'hFF;

        rst_n = 0;
        repeat(10) @(posedge clk);
        rst_n = 1;
        repeat(5) @(posedge clk);

        // Test 1: .po write block 2048
        $display("\n--- Test 1: .po write ---");
        block_num = 16'd2048;
        start = 1;
        @(posedge clk); start = 0;

        begin : w1
            integer w;
            for (w = 0; w < 100000 && busy; w = w + 1) @(posedge clk);
        end

        check("busy_cleared", 0, busy);
        check("sd_addr", 32'd2656, sd_write_addr_cap);
        check("byte0", (0*3+5) & 8'hFF, sd_captured[0]);
        check("byte1", (1*3+5) & 8'hFF, sd_captured[1]);
        check("byte511", (511*3+5) & 8'hFF, sd_captured[511]);

        // Test 2: persist disabled
        $display("\n--- Test 2: disabled ---");
        persist_enabled = 0;
        start = 1; @(posedge clk); start = 0;
        repeat(10) @(posedge clk);
        check("disabled", 0, busy);
        persist_enabled = 1;

        $display("\n========================================");
        $display("sd_persist_tb: %0d tests, %0d passed, %0d failed",
                 tests_run, tests_passed, tests_failed);
        $display("========================================\n");
        $finish;
    end

    initial begin #50000000; $display("TIMEOUT"); $finish; end
endmodule
