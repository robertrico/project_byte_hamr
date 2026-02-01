// =============================================================================
// Decimation + Pack Module - Testbench
// =============================================================================
//
// Tests the decimate_pack module with various inputs:
//   - Test 1: 88 alternating samples (0101...) with stretch=3 -> 38 bytes
//   - Test 2: All-high samples -> $7F bytes
//   - Test 3: All-low samples -> $00 bytes
//   - Test 4: Stretch factor = 1 (no stretching)
//   - Test 5: Stretch factor = 7 (one sample per byte)
//
// Expected first byte for Test 1:
//   Samples: 0, 1, 0, ...
//   Pixels:  0 0 0 1 1 1 0 0 0 1 1 1 ...
//   Byte 0:  bits 0-6 = 0 0 0 1 1 1 0 = $38
//
// =============================================================================

`timescale 1ns / 100ps

module decimate_pack_tb;

    // =========================================================================
    // Clock generation - 25 MHz (40ns period)
    // =========================================================================

    reg clk = 0;
    always #20 clk = ~clk;

    // =========================================================================
    // DUT Signals
    // =========================================================================

    reg        rst = 0;
    reg        sample_in = 0;
    reg        sample_valid = 0;
    reg  [7:0] stretch_factor = 8'd3;

    wire [6:0] byte_out;
    wire       byte_valid;

    // =========================================================================
    // Device Under Test
    // =========================================================================

    decimate_pack dut (
        .clk(clk),
        .rst(rst),
        .sample_in(sample_in),
        .sample_valid(sample_valid),
        .stretch_factor(stretch_factor),
        .byte_out(byte_out),
        .byte_valid(byte_valid)
    );

    // =========================================================================
    // Byte Capture
    // =========================================================================

    reg [6:0] captured_bytes [0:63];
    integer   byte_count;

    initial begin
        byte_count = 0;
    end

    always @(posedge clk) begin
        if (byte_valid) begin
            captured_bytes[byte_count] = byte_out;
            byte_count = byte_count + 1;
            $display("[%0t] Byte %0d: $%02h", $time, byte_count - 1, byte_out);
        end
    end

    // =========================================================================
    // Test Tasks
    // =========================================================================

    task send_sample;
        input value;
        begin
            @(posedge clk);
            sample_in <= value;
            sample_valid <= 1'b1;
            @(posedge clk);
            sample_valid <= 1'b0;
            // Wait for stretch_factor cycles to complete
            repeat (stretch_factor) @(posedge clk);
        end
    endtask

    task send_samples_alternating;
        input integer count;
        integer i;
        begin
            for (i = 0; i < count; i = i + 1) begin
                send_sample(i[0]);  // Alternating 0, 1, 0, 1...
            end
        end
    endtask

    task send_samples_all_high;
        input integer count;
        integer i;
        begin
            for (i = 0; i < count; i = i + 1) begin
                send_sample(1'b1);
            end
        end
    endtask

    task send_samples_all_low;
        input integer count;
        integer i;
        begin
            for (i = 0; i < count; i = i + 1) begin
                send_sample(1'b0);
            end
        end
    endtask

    task apply_reset;
        begin
            @(posedge clk);
            rst <= 1'b1;
            @(posedge clk);
            @(posedge clk);
            rst <= 1'b0;
            @(posedge clk);
            byte_count = 0;
        end
    endtask

    // =========================================================================
    // Test Sequence
    // =========================================================================

    integer test_pass;
    integer test_fail;
    integer i;

    initial begin
        $dumpfile("decimate_pack_tb.vcd");
        $dumpvars(0, decimate_pack_tb);

        test_pass = 0;
        test_fail = 0;

        $display("===========================================");
        $display("Decimation Pack Module Testbench");
        $display("===========================================");
        $display("");

        // ---- Test 1: 89 alternating samples with stretch=3 ----
        // 89 samples × 3 pixels = 267 pixels ÷ 7 = 38 complete bytes
        $display("--- Test 1: 89 alternating samples, stretch=3 ---");
        $display("Expected: 38 bytes, first byte = $38");
        apply_reset;
        stretch_factor = 8'd3;
        send_samples_alternating(89);

        // Wait for final bytes
        repeat (20) @(posedge clk);

        $display("Captured %0d bytes", byte_count);

        if (byte_count == 38) begin
            $display("  Byte count: PASS (38 bytes)");
            test_pass = test_pass + 1;
        end else begin
            $display("  Byte count: FAIL (expected 38, got %0d)", byte_count);
            test_fail = test_fail + 1;
        end

        if (captured_bytes[0] == 7'h38) begin
            $display("  First byte: PASS ($38)");
            test_pass = test_pass + 1;
        end else begin
            $display("  First byte: FAIL (expected $38, got $%02h)", captured_bytes[0]);
            test_fail = test_fail + 1;
        end

        // ---- Test 2: All-high samples ----
        $display("");
        $display("--- Test 2: 14 all-high samples, stretch=1 ---");
        $display("Expected: 2 bytes of $7F");
        apply_reset;
        stretch_factor = 8'd1;
        send_samples_all_high(14);

        repeat (10) @(posedge clk);

        if (byte_count == 2) begin
            $display("  Byte count: PASS (2 bytes)");
            test_pass = test_pass + 1;
        end else begin
            $display("  Byte count: FAIL (expected 2, got %0d)", byte_count);
            test_fail = test_fail + 1;
        end

        if (captured_bytes[0] == 7'h7F && captured_bytes[1] == 7'h7F) begin
            $display("  All bytes $7F: PASS");
            test_pass = test_pass + 1;
        end else begin
            $display("  All bytes $7F: FAIL (got $%02h, $%02h)",
                     captured_bytes[0], captured_bytes[1]);
            test_fail = test_fail + 1;
        end

        // ---- Test 3: All-low samples ----
        $display("");
        $display("--- Test 3: 14 all-low samples, stretch=1 ---");
        $display("Expected: 2 bytes of $00");
        apply_reset;
        stretch_factor = 8'd1;
        send_samples_all_low(14);

        repeat (10) @(posedge clk);

        if (byte_count == 2) begin
            $display("  Byte count: PASS (2 bytes)");
            test_pass = test_pass + 1;
        end else begin
            $display("  Byte count: FAIL (expected 2, got %0d)", byte_count);
            test_fail = test_fail + 1;
        end

        if (captured_bytes[0] == 7'h00 && captured_bytes[1] == 7'h00) begin
            $display("  All bytes $00: PASS");
            test_pass = test_pass + 1;
        end else begin
            $display("  All bytes $00: FAIL (got $%02h, $%02h)",
                     captured_bytes[0], captured_bytes[1]);
            test_fail = test_fail + 1;
        end

        // ---- Test 4: Stretch factor = 7 (one sample = one byte) ----
        $display("");
        $display("--- Test 4: 4 samples with stretch=7 ---");
        $display("Expected: 4 bytes (1 sample fills 1 byte)");
        apply_reset;
        stretch_factor = 8'd7;

        send_sample(1'b1);  // Should produce $7F
        send_sample(1'b0);  // Should produce $00
        send_sample(1'b1);  // Should produce $7F
        send_sample(1'b0);  // Should produce $00

        repeat (10) @(posedge clk);

        if (byte_count == 4) begin
            $display("  Byte count: PASS (4 bytes)");
            test_pass = test_pass + 1;
        end else begin
            $display("  Byte count: FAIL (expected 4, got %0d)", byte_count);
            test_fail = test_fail + 1;
        end

        if (captured_bytes[0] == 7'h7F && captured_bytes[1] == 7'h00 &&
            captured_bytes[2] == 7'h7F && captured_bytes[3] == 7'h00) begin
            $display("  Pattern $7F $00 $7F $00: PASS");
            test_pass = test_pass + 1;
        end else begin
            $display("  Pattern: FAIL (got $%02h $%02h $%02h $%02h)",
                     captured_bytes[0], captured_bytes[1],
                     captured_bytes[2], captured_bytes[3]);
            test_fail = test_fail + 1;
        end

        // ---- Test 5: Verify bit ordering (LSB-first) ----
        $display("");
        $display("--- Test 5: Bit ordering verification ---");
        $display("Sample sequence: 1,0,0,0,0,0,0 with stretch=1");
        $display("Expected: $01 (bit 0 set, LSB-first)");
        apply_reset;
        stretch_factor = 8'd1;

        send_sample(1'b1);
        send_sample(1'b0);
        send_sample(1'b0);
        send_sample(1'b0);
        send_sample(1'b0);
        send_sample(1'b0);
        send_sample(1'b0);

        repeat (10) @(posedge clk);

        if (captured_bytes[0] == 7'h01) begin
            $display("  LSB-first ordering: PASS ($01)");
            test_pass = test_pass + 1;
        end else begin
            $display("  LSB-first ordering: FAIL (expected $01, got $%02h)",
                     captured_bytes[0]);
            test_fail = test_fail + 1;
        end

        // ---- Test 6: Bit 6 only ----
        $display("");
        $display("--- Test 6: Bit 6 verification ---");
        $display("Sample sequence: 0,0,0,0,0,0,1 with stretch=1");
        $display("Expected: $40 (bit 6 set)");
        apply_reset;
        stretch_factor = 8'd1;

        send_sample(1'b0);
        send_sample(1'b0);
        send_sample(1'b0);
        send_sample(1'b0);
        send_sample(1'b0);
        send_sample(1'b0);
        send_sample(1'b1);

        repeat (10) @(posedge clk);

        if (captured_bytes[0] == 7'h40) begin
            $display("  Bit 6 set: PASS ($40)");
            test_pass = test_pass + 1;
        end else begin
            $display("  Bit 6 set: FAIL (expected $40, got $%02h)",
                     captured_bytes[0]);
            test_fail = test_fail + 1;
        end

        // ---- Summary ----
        $display("");
        $display("===========================================");
        $display("Results: %0d passed, %0d failed", test_pass, test_fail);
        $display("===========================================");

        if (test_fail == 0)
            $display("*** ALL TESTS PASSED ***");
        else
            $display("*** SOME TESTS FAILED ***");

        $display("");
        #100;
        $finish;
    end

endmodule
