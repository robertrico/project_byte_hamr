// boot_loader_tb.v — Test boot_loader (flash copy only, no SD)

`timescale 1ns / 1ps

module boot_loader_tb;

    reg clk = 0;
    always #20 clk = ~clk;

    reg rst_n = 0;
    reg sdram_init_done = 0;
    wire boot_done;
    wire [15:0] total_blocks;

    wire flash_start;
    wire [23:0] flash_addr, flash_count;
    reg flash_busy = 0;
    reg [7:0] flash_data = 0;
    reg flash_data_valid = 0;
    wire flash_data_ready;

    wire sdram_req, sdram_req_write;
    wire [25:0] sdram_req_addr;
    wire [15:0] sdram_req_wdata;
    reg sdram_req_ready = 1;

    reg [7:0] sdram_mem [0:262143];

    always @(posedge clk) begin
        if (sdram_req && sdram_req_write) begin
            sdram_mem[sdram_req_addr]     <= sdram_req_wdata[7:0];
            sdram_mem[sdram_req_addr | 1] <= sdram_req_wdata[15:8];
        end
    end

    boot_loader #(.FLASH_OFFSET(24'h400000), .MAX_BLOCKS(16'd280)) dut (
        .clk(clk), .rst_n(rst_n),
        .sdram_init_done(sdram_init_done),
        .boot_done(boot_done), .total_blocks(total_blocks),
        .flash_start(flash_start), .flash_addr(flash_addr),
        .flash_count(flash_count), .flash_busy(flash_busy),
        .flash_data(flash_data), .flash_data_valid(flash_data_valid),
        .flash_data_ready(flash_data_ready),
        .sdram_req(sdram_req), .sdram_req_write(sdram_req_write),
        .sdram_req_addr(sdram_req_addr), .sdram_req_wdata(sdram_req_wdata),
        .sdram_req_ready(sdram_req_ready)
    );

    integer tests_run = 0, tests_passed = 0, tests_failed = 0;
    task check;
        input [255:0] label;
        input [31:0] expected, actual;
        begin
            tests_run = tests_run + 1;
            if (expected === actual) tests_passed = tests_passed + 1;
            else begin tests_failed = tests_failed + 1;
                $display("FAIL: %0s — expected 0x%0h, got 0x%0h", label, expected, actual);
            end
        end
    endtask

    // Flash mock
    reg [23:0] flash_byte_cnt;
    reg flash_active = 0;
    always @(posedge clk) begin
        flash_data_valid <= 1'b0;
        if (flash_start) begin
            flash_active <= 1'b1; flash_byte_cnt <= 24'd0; flash_busy <= 1'b1;
        end else if (flash_active && flash_data_ready) begin
            flash_data <= flash_byte_cnt[7:0];
            flash_data_valid <= 1'b1;
            flash_byte_cnt <= flash_byte_cnt + 24'd1;
            if (flash_byte_cnt == 24'd1065) flash_data <= 8'h18;
            if (flash_byte_cnt == 24'd1066) flash_data <= 8'h01;
            if (flash_byte_cnt >= 24'd143359) begin
                flash_active <= 1'b0; flash_busy <= 1'b0;
            end
        end
    end

    integer i;
    initial begin
        $dumpfile("boot_loader_tb.vcd");
        $dumpvars(0, boot_loader_tb);
        for (i = 0; i < 262144; i = i + 1) sdram_mem[i] = 8'h00;
        rst_n = 0; repeat(10) @(posedge clk); rst_n = 1; repeat(5) @(posedge clk);

        repeat(100) @(posedge clk);
        check("no_boot_yet", 0, boot_done);

        sdram_init_done = 1;
        begin : w1
            integer w;
            for (w = 0; w < 2000000 && !boot_done; w = w + 1) @(posedge clk);
        end

        check("boot_done", 1, boot_done);
        check("total_blocks", 16'd280, total_blocks);
        check("sdram_byte0", 8'h00, sdram_mem[0]);
        check("sdram_byte1", 8'h01, sdram_mem[1]);

        $display("\n========================================");
        $display("boot_loader_tb: %0d tests, %0d passed, %0d failed",
                 tests_run, tests_passed, tests_failed);
        $display("========================================\n");
        $finish;
    end
    initial begin #100000000; $display("TIMEOUT"); $finish; end
endmodule
