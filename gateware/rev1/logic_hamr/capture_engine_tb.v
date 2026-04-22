// =============================================================================
// Capture Engine Testbench
// =============================================================================
//
// Tests the capture engine with:
// - ARM command and status flags
// - Pre-trigger buffering
// - Trigger detection (rising/falling edge)
// - All window presets
// - SDRAM write interface
//
// =============================================================================

`timescale 1ns / 100ps

module capture_engine_tb;

    // =========================================================================
    // Clock generation - 25 MHz (40ns period)
    // =========================================================================

    reg clk = 0;
    always #20 clk = ~clk;

    reg rst_n = 0;

    // =========================================================================
    // Sample clock - 1 MHz (1000ns period)
    // =========================================================================

    reg [4:0] sample_div = 0;
    wire sample_strobe = (sample_div == 5'd24);

    always @(posedge clk) begin
        if (sample_div == 5'd24)
            sample_div <= 5'd0;
        else
            sample_div <= sample_div + 1'b1;
    end

    // =========================================================================
    // DUT Signals
    // =========================================================================

    reg        arm;
    reg        soft_reset;
    reg  [7:0] probe_input;
    reg  [2:0] trig_ch;
    reg        trig_mode;
    reg  [1:0] window_preset;

    wire       armed;
    wire       captured;
    wire [8:0] total_samples;
    wire       pause_refresh;

    wire       sdram_wr_req;
    wire [12:0] sdram_wr_addr;
    wire [7:0] sdram_wr_data;
    reg        sdram_wr_ready;

    // =========================================================================
    // SDRAM Memory Model (simplified)
    // =========================================================================

    reg [7:0] sdram_mem [0:1023];
    reg [9:0] write_count;

    integer j;
    initial begin
        for (j = 0; j < 1024; j = j + 1)
            sdram_mem[j] = 8'h00;
        write_count = 0;
    end

    // Accept writes with a small delay
    reg [2:0] write_delay = 0;
    always @(posedge clk) begin
        if (sdram_wr_req && sdram_wr_ready) begin
            sdram_mem[sdram_wr_addr[9:0]] <= sdram_wr_data;
            write_count <= write_count + 1;
            $display("[%0t] SDRAM WRITE: addr=%03h data=%02h (count=%0d)",
                     $time, sdram_wr_addr[9:0], sdram_wr_data, write_count + 1);
        end
    end

    // =========================================================================
    // Device Under Test
    // =========================================================================

    capture_engine dut (
        .clk(clk),
        .rst_n(rst_n),
        .arm(arm),
        .soft_reset(soft_reset),
        .probe_input(probe_input),
        .sample_strobe(sample_strobe),
        .trig_ch(trig_ch),
        .trig_mode(trig_mode),
        .window_preset(window_preset),
        .armed(armed),
        .captured(captured),
        .total_samples(total_samples),
        .pause_refresh(pause_refresh),
        .sdram_wr_req(sdram_wr_req),
        .sdram_wr_addr(sdram_wr_addr),
        .sdram_wr_data(sdram_wr_data),
        .sdram_wr_ready(sdram_wr_ready)
    );

    // =========================================================================
    // Test Sequence
    // =========================================================================

    integer test_pass;
    integer test_fail;
    integer i;

    initial begin
        $dumpfile("capture_engine_tb.vcd");
        $dumpvars(0, capture_engine_tb);

        test_pass = 0;
        test_fail = 0;

        // Initialize signals
        arm = 0;
        soft_reset = 0;
        probe_input = 8'h00;
        trig_ch = 3'd0;
        trig_mode = 1'b0;  // Rising edge
        window_preset = 2'd0;  // 38 samples
        sdram_wr_ready = 1;

        $display("===========================================");
        $display("Capture Engine Testbench");
        $display("===========================================");
        $display("");

        // Release reset
        #100;
        rst_n = 1;
        #100;

        // =====================================================================
        // Test 1: ARM and Armed Flag
        // =====================================================================
        $display("--- Test 1: ARM and Armed Flag ---");

        // Send ARM pulse
        @(posedge clk);
        arm = 1;
        @(posedge clk);
        arm = 0;

        // Check armed flag
        #100;
        if (armed) begin
            $display("  Armed flag set: PASS");
            test_pass = test_pass + 1;
        end else begin
            $display("  Armed flag set: FAIL");
            test_fail = test_fail + 1;
        end

        // Check pause_refresh is active
        if (pause_refresh) begin
            $display("  pause_refresh active: PASS");
            test_pass = test_pass + 1;
        end else begin
            $display("  pause_refresh active: FAIL");
            test_fail = test_fail + 1;
        end

        // =====================================================================
        // Test 2: Rising Edge Trigger on Channel 0
        // =====================================================================
        $display("");
        $display("--- Test 2: Rising Edge Trigger ---");

        // Reset and re-arm with preset 0 (38 samples, 2 pre-trigger)
        soft_reset = 1;
        @(posedge clk);
        soft_reset = 0;
        #100;

        write_count = 0;
        trig_ch = 3'd0;
        trig_mode = 1'b0;  // Rising edge
        window_preset = 2'd0;  // 38 samples

        @(posedge clk);
        arm = 1;
        @(posedge clk);
        arm = 0;

        // Let pre-trigger buffer fill (need 2 samples at 1MHz = 2us)
        probe_input = 8'h00;  // Keep channel 0 low
        #5000;

        // Generate rising edge on channel 0
        $display("  Generating rising edge on channel 0...");
        probe_input = 8'h01;  // Channel 0 goes high

        // Wait for capture to complete
        i = 0;
        while (!captured && i < 100_000) begin
            @(posedge clk);
            i = i + 1;
        end

        if (captured) begin
            $display("  Capture complete: PASS (after %0d clocks)", i);
            test_pass = test_pass + 1;
        end else begin
            $display("  Capture complete: FAIL (timeout)");
            test_fail = test_fail + 1;
        end

        // Check sample count
        $display("  Expected samples: 38, Written: %0d", write_count);
        if (write_count == 38) begin
            $display("  Sample count: PASS");
            test_pass = test_pass + 1;
        end else begin
            $display("  Sample count: FAIL");
            test_fail = test_fail + 1;
        end

        // =====================================================================
        // Test 3: Falling Edge Trigger on Channel 5
        // =====================================================================
        $display("");
        $display("--- Test 3: Falling Edge Trigger ---");

        // Reset and re-arm
        soft_reset = 1;
        @(posedge clk);
        soft_reset = 0;
        #100;

        // Clear memory
        for (j = 0; j < 1024; j = j + 1)
            sdram_mem[j] = 8'h00;
        write_count = 0;

        trig_ch = 3'd5;
        trig_mode = 1'b1;  // Falling edge
        window_preset = 2'd0;  // 38 samples

        // Start with channel 5 high
        probe_input = 8'b00100000;  // Channel 5 = bit 5

        @(posedge clk);
        arm = 1;
        @(posedge clk);
        arm = 0;

        // Let pre-trigger buffer fill
        #5000;

        // Generate falling edge on channel 5
        $display("  Generating falling edge on channel 5...");
        probe_input = 8'h00;

        // Wait for capture
        i = 0;
        while (!captured && i < 100_000) begin
            @(posedge clk);
            i = i + 1;
        end

        if (captured) begin
            $display("  Capture complete: PASS");
            test_pass = test_pass + 1;
        end else begin
            $display("  Capture complete: FAIL");
            test_fail = test_fail + 1;
        end

        // Verify pre-trigger samples have channel 5 high
        if (sdram_mem[0][5] == 1'b1 && sdram_mem[1][5] == 1'b1) begin
            $display("  Pre-trigger data correct: PASS (ch5 was high)");
            test_pass = test_pass + 1;
        end else begin
            $display("  Pre-trigger data: FAIL (expected ch5 high)");
            $display("    sdram_mem[0] = %02h", sdram_mem[0]);
            $display("    sdram_mem[1] = %02h", sdram_mem[1]);
            test_fail = test_fail + 1;
        end

        // =====================================================================
        // Test 4: Window Preset 3 (266 samples)
        // =====================================================================
        $display("");
        $display("--- Test 4: Window Preset 3 (266 samples) ---");

        // Reset and re-arm
        soft_reset = 1;
        @(posedge clk);
        soft_reset = 0;
        #100;

        write_count = 0;

        trig_ch = 3'd0;
        trig_mode = 1'b0;  // Rising edge
        window_preset = 2'd3;  // 266 samples, 13 pre-trigger

        probe_input = 8'h00;

        @(posedge clk);
        arm = 1;
        @(posedge clk);
        arm = 0;

        // Let pre-trigger buffer fill (need 13 samples = 13us)
        #20000;

        // Generate trigger
        $display("  Generating rising edge...");
        probe_input = 8'h01;

        // Wait for capture (longer for 266 samples)
        i = 0;
        while (!captured && i < 500_000) begin
            @(posedge clk);
            i = i + 1;
        end

        $display("  Total samples config: %0d", total_samples);
        $display("  Samples written: %0d", write_count);

        if (captured && write_count == 266) begin
            $display("  Window preset 3: PASS");
            test_pass = test_pass + 1;
        end else begin
            $display("  Window preset 3: FAIL");
            test_fail = test_fail + 1;
        end

        // =====================================================================
        // Test 5: Soft Reset
        // =====================================================================
        $display("");
        $display("--- Test 5: Soft Reset ---");

        // Re-arm
        @(posedge clk);
        arm = 1;
        @(posedge clk);
        arm = 0;
        #100;

        if (armed) begin
            $display("  Armed after ARM: PASS");
        end

        // Issue soft reset
        @(posedge clk);
        soft_reset = 1;
        @(posedge clk);
        soft_reset = 0;
        #100;

        if (!armed && !captured) begin
            $display("  Soft reset clears flags: PASS");
            test_pass = test_pass + 1;
        end else begin
            $display("  Soft reset clears flags: FAIL (armed=%b captured=%b)", armed, captured);
            test_fail = test_fail + 1;
        end

        // =====================================================================
        // Summary
        // =====================================================================
        $display("");
        $display("===========================================");
        $display("Results: %0d passed, %0d failed", test_pass, test_fail);
        $display("===========================================");

        if (test_fail == 0)
            $display("*** ALL TESTS PASSED ***");
        else
            $display("*** SOME TESTS FAILED ***");

        $display("");
        #1000;
        $finish;
    end

endmodule
