`timescale 1ns / 1ps

module write_through_tb;
    reg clk = 0;
    always #20 clk = ~clk;

    reg rst_n = 0;
    reg arb_block_ready = 0;
    wire gated_block_ready;
    reg dev_block_write_req = 0;
    reg [15:0] dev_block_num = 16'd2050;
    wire fp_start;
    wire [15:0] fp_block_num;
    reg fp_busy = 0;

    write_through #(.UNIT2_OFFSET(16'd2048)) dut (
        .clk(clk), .rst_n(rst_n), .boot_done(1'b1),
        .arb_block_ready(arb_block_ready), .gated_block_ready(gated_block_ready),
        .dev_block_write_req(dev_block_write_req), .dev_block_num(dev_block_num),
        .fp_start(fp_start), .fp_block_num(fp_block_num), .fp_busy(fp_busy),
        .persist_enabled(1'b1)
    );

    // Auto-assert busy when fp_start fires
    always @(posedge clk) if (fp_start) fp_busy <= 1'b1;

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

    // Capture fp_start pulse
    reg saw_fp_start = 0;
    always @(posedge clk) if (fp_start) saw_fp_start = 1;

    initial begin
        $dumpfile("write_through_tb.vcd");
        $dumpvars(0, write_through_tb);

        // Use $monitor to watch key signals
        $monitor("t=%0t arb=%b d1=%b hold=%b fp_start=%b busy=%b gated=%b",
                 $time, arb_block_ready, dut.arb_ready_d1,
                 dut.holding, fp_start, fp_busy, gated_block_ready);

        rst_n = 0;
        #200;
        rst_n = 1;

        // Set write request and wait for CDC
        @(posedge clk);
        dev_block_write_req = 1;

        // Wait for CDC (2 cycles for is_write)
        repeat(4) @(posedge clk);

        // Now pulse arb_block_ready
        @(posedge clk);
        arb_block_ready = 1;

        // Wait and observe
        repeat(5) @(posedge clk);

        // Simulate persist done
        fp_busy = 0;
        repeat(5) @(posedge clk);

        $monitoroff;

        check("saw_fp_start", 1, saw_fp_start);
        check("fp_block_num", 16'd2050, fp_block_num);

        $display("\n========================================");
        $display("write_through_tb: %0d tests, %0d passed, %0d failed",
                 tests_run, tests_passed, tests_failed);
        $display("========================================\n");
        $finish;
    end
endmodule
