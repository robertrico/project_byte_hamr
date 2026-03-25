// fm_decoder_tb.v — Testbench for fm_decoder module
// Tests sync detection, byte decoding, multi-byte packets, PEND, enable gating.
// Uses FM waveform generator tasks to produce wrdata edges with correct timing.
//
// TOGGLE encoding: The IWM write serializer TOGGLES wrdata for each "1" bit.
// Each transition (rising or falling) = one "1" bit. For "0" bits, no
// transition. The decoder uses XOR edge detection to catch both edges.
//
// Each test sends a complete packet: sync(5+) + $C3 + payload [+ $C8].
// Between tests, enable is toggled to reset decoder state.

`timescale 1ns / 1ps

module fm_decoder_tb;

    // =========================================================================
    // Clock: ~7.16 MHz -> ~139.6 ns period (70 ns half-period)
    // =========================================================================
    reg clk;
    initial clk = 0;
    always #70 clk = ~clk;

    // =========================================================================
    // Test result tracking
    // =========================================================================
    integer pass_count;
    integer fail_count;

    // =========================================================================
    // DUT signals
    // =========================================================================
    reg        rst_n;
    reg        enable;
    reg        wrdata;
    wire [7:0] data_out;
    wire       data_valid;
    wire       sync_detected;
    wire       packet_end;

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    fm_decoder uut (
        .clk           (clk),
        .rst_n         (rst_n),
        .enable        (enable),
        .wrdata        (wrdata),
        .data_out      (data_out),
        .data_valid    (data_valid),
        .sync_detected (sync_detected),
        .packet_end    (packet_end)
    );

    // =========================================================================
    // FM waveform generation — TOGGLE encoding
    // =========================================================================
    //
    // IWM TOGGLE encoding at ~7 MHz fclk:
    //   Bit cell = 28 fclk cycles (~4 us)
    //   "1" bit = TOGGLE wrdata (rising or falling edge)
    //   "0" bit = no transition during cell
    //
    // The decoder uses XOR edge detection: wrdata_edge = wrdata_r2 ^ wrdata_r1
    // which catches BOTH rising and falling edges.
    //
    // For consecutive "1" bits, wrdata alternates: 1->0->1->0->...
    // Each transition is one "1" bit.

    localparam integer BIT_CELL  = 28;  // fclk cycles per bit cell

    // fm_send_byte: generate FM TOGGLE-encoded wrdata for one byte (MSB first).
    //
    // For each "1" bit: TOGGLE wrdata (invert it). Wait BIT_CELL cycles.
    // For each "0" bit: hold wrdata steady. Wait BIT_CELL cycles.
    //
    // This matches the IWM write serializer: wrdata <= ~wrdata on each "1" bit.

    task fm_send_byte;
        input [7:0] byte_val;
        integer b, j;
        begin
            for (b = 7; b >= 0; b = b - 1) begin
                if (byte_val[b]) begin
                    // "1" bit: TOGGLE wrdata
                    wrdata = ~wrdata;
                end
                // else "0" bit: no transition, hold steady
                // Wait one bit cell
                for (j = 0; j < BIT_CELL; j = j + 1)
                    @(posedge clk);
            end
        end
    endtask

    // fm_kickoff: send an initial toggle to prime the decoder's first_edge.
    // After this, wrdata is in the toggled state for the first "1" of the
    // next byte to build upon.
    task fm_kickoff;
        begin
            wrdata = ~wrdata;
            // Wait one bit cell for this kickoff edge to register
            repeat (BIT_CELL) @(posedge clk);
        end
    endtask

    // fm_send_sync: send a kickoff toggle followed by N bytes of $FF.
    task fm_send_sync;
        input integer count;
        integer n;
        begin
            fm_kickoff;
            for (n = 0; n < count; n = n + 1)
                fm_send_byte(8'hFF);
        end
    endtask

    // fm_idle: hold wrdata steady for some fclk cycles (no edges).
    task fm_idle;
        input integer num_clocks;
        integer j;
        begin
            for (j = 0; j < num_clocks; j = j + 1)
                @(posedge clk);
        end
    endtask

    // fm_wait_flush: wait long enough for the decoder's timeout flush
    // to trigger. The decoder flushes partial bytes after FLUSH_TIMEOUT=252
    // fclk cycles with no edge. We wait 270 to give margin.
    task fm_wait_flush;
        begin
            repeat (270) @(posedge clk);
        end
    endtask

    // =========================================================================
    // Byte capture: record decoded bytes in background
    // =========================================================================
    reg [7:0] captured_bytes [0:63];
    integer   capture_count;
    integer   pend_count;

    always @(posedge clk) begin
        if (data_valid) begin
            captured_bytes[capture_count] = data_out;
            capture_count = capture_count + 1;
        end
        if (packet_end)
            pend_count = pend_count + 1;
    end

    // =========================================================================
    // Helper: reset decoder between tests
    // =========================================================================
    task reset_decoder;
        begin
            enable = 0;
            repeat (10) @(posedge clk);
            enable = 1;
            wrdata = 1'b1;  // reset to known idle state
            repeat (5) @(posedge clk);
            capture_count = 0;
            pend_count    = 0;
        end
    endtask

    // =========================================================================
    // Test sequence
    // =========================================================================
    integer i;
    reg     test_failed;

    initial begin
        $dumpfile("fm_decoder_tb.vcd");
        $dumpvars(0, fm_decoder_tb);

        pass_count = 0;
        fail_count = 0;

        // Initialize
        rst_n  = 0;
        enable = 0;
        wrdata = 1'b1;
        capture_count = 0;
        pend_count    = 0;

        // Reset
        repeat (10) @(posedge clk);
        rst_n = 1;
        repeat (5) @(posedge clk);

        // =================================================================
        // TEST 1: Sync detection — 5x $FF + $C3
        // Verify sync_detected goes HIGH and no data bytes are emitted.
        // SYNC_ONES_MIN = 8, so 5x $FF = 40 ones is plenty.
        // Decoder uses sliding window C3 check on every bit.
        // =================================================================
        $display("");
        $display("--- Test 1: Sync detection ---");
        enable = 1;
        wrdata = 1'b1;
        capture_count = 0;

        fm_idle(20);

        // Send sync + PBEGIN
        fm_send_sync(5);
        fm_send_byte(8'hC3);

        // Wait for pipeline drain
        fm_wait_flush;

        if (sync_detected) begin
            $display("PASS: Test 1 - sync_detected asserted after 5x$FF + $C3");
            pass_count = pass_count + 1;
        end else begin
            $display("FAIL: Test 1 - sync_detected not asserted");
            fail_count = fail_count + 1;
        end

        if (capture_count == 0) begin
            $display("PASS: Test 1b - No data bytes emitted during sync phase");
            pass_count = pass_count + 1;
        end else begin
            $display("FAIL: Test 1b - %0d bytes emitted during sync (expected 0)",
                     capture_count);
            fail_count = fail_count + 1;
        end

        // =================================================================
        // TEST 2: Single byte decode — $A5 (10100101)
        // Send a complete packet: sync + $C3 + $A5 + $C8
        // =================================================================
        $display("");
        $display("--- Test 2: Single byte decode ($A5) ---");
        reset_decoder;

        fm_send_sync(5);
        fm_send_byte(8'hC3);
        fm_send_byte(8'hA5);
        fm_send_byte(8'hC8);

        fm_wait_flush;

        test_failed = 0;
        if (capture_count < 1) begin
            $display("FAIL: Test 2 - No bytes decoded");
            test_failed = 1;
        end else if (captured_bytes[0] != 8'hA5) begin
            $display("FAIL: Test 2 - Expected $A5, got $%02h", captured_bytes[0]);
            test_failed = 1;
        end

        if (!test_failed) begin
            $display("PASS: Test 2 - Single byte $A5 decoded correctly");
            pass_count = pass_count + 1;
        end else begin
            fail_count = fail_count + 1;
        end

        // =================================================================
        // TEST 3: Multi-byte decode — $D5, $3C, $81, $FE
        // =================================================================
        $display("");
        $display("--- Test 3: Multi-byte decode ($D5, $3C, $81, $FE) ---");
        reset_decoder;

        fm_send_sync(5);
        fm_send_byte(8'hC3);
        fm_send_byte(8'hD5);
        fm_send_byte(8'h3C);
        fm_send_byte(8'h81);
        fm_send_byte(8'hFE);

        fm_wait_flush;

        test_failed = 0;
        if (capture_count != 4) begin
            $display("FAIL: Test 3 - Expected 4 bytes, got %0d", capture_count);
            for (i = 0; i < capture_count; i = i + 1)
                $display("  byte %0d = $%02h", i, captured_bytes[i]);
            test_failed = 1;
        end else begin
            if (captured_bytes[0] != 8'hD5) begin
                $display("FAIL: Test 3 - Byte 0: expected $D5, got $%02h", captured_bytes[0]);
                test_failed = 1;
            end
            if (captured_bytes[1] != 8'h3C) begin
                $display("FAIL: Test 3 - Byte 1: expected $3C, got $%02h", captured_bytes[1]);
                test_failed = 1;
            end
            if (captured_bytes[2] != 8'h81) begin
                $display("FAIL: Test 3 - Byte 2: expected $81, got $%02h", captured_bytes[2]);
                test_failed = 1;
            end
            if (captured_bytes[3] != 8'hFE) begin
                $display("FAIL: Test 3 - Byte 3: expected $FE, got $%02h", captured_bytes[3]);
                test_failed = 1;
            end
        end

        if (!test_failed) begin
            $display("PASS: Test 3 - All 4 bytes decoded correctly");
            pass_count = pass_count + 1;
        end else begin
            fail_count = fail_count + 1;
        end

        // =================================================================
        // TEST 4: Packet end — verify $C8 triggers packet_end pulse
        // =================================================================
        $display("");
        $display("--- Test 4: Packet end ($C8) ---");
        reset_decoder;

        fm_send_sync(5);
        fm_send_byte(8'hC3);
        fm_send_byte(8'hFF);   // one payload byte
        fm_send_byte(8'hC8);   // PEND

        fm_wait_flush;

        test_failed = 0;
        if (capture_count < 2) begin
            $display("FAIL: Test 4 - Expected 2 bytes, got %0d", capture_count);
            test_failed = 1;
        end else begin
            if (captured_bytes[1] != 8'hC8) begin
                $display("FAIL: Test 4 - Expected $C8 as byte 1, got $%02h", captured_bytes[1]);
                test_failed = 1;
            end
        end
        if (pend_count == 0) begin
            $display("FAIL: Test 4 - packet_end never asserted");
            test_failed = 1;
        end

        if (!test_failed) begin
            $display("PASS: Test 4 - $C8 decoded correctly, packet_end asserted");
            pass_count = pass_count + 1;
        end else begin
            fail_count = fail_count + 1;
        end

        // =================================================================
        // TEST 5: Enable gating — disable mid-packet, verify state reset,
        //         re-enable and verify new sync + decode works
        // =================================================================
        $display("");
        $display("--- Test 5: Enable gating ---");

        // Start a packet
        reset_decoder;
        fm_send_sync(5);
        fm_send_byte(8'hC3);
        fm_send_byte(8'hFF);

        fm_wait_flush;

        // Verify we're synced
        if (!sync_detected) begin
            $display("FAIL: Test 5 - not synced before disable");
            fail_count = fail_count + 1;
        end

        // Disable mid-packet
        enable = 0;
        repeat (20) @(posedge clk);

        if (!sync_detected) begin
            $display("PASS: Test 5a - sync_detected cleared when enable deasserted");
            pass_count = pass_count + 1;
        end else begin
            $display("FAIL: Test 5a - sync_detected still HIGH after disable");
            fail_count = fail_count + 1;
        end

        // Re-enable, send fresh packet, verify decode works
        enable = 1;
        wrdata = 1'b1;  // reset to known idle
        capture_count = 0;
        pend_count    = 0;
        repeat (5) @(posedge clk);

        fm_send_sync(5);
        fm_send_byte(8'hC3);
        fm_send_byte(8'h42);

        fm_wait_flush;

        test_failed = 0;
        if (!sync_detected) begin
            $display("FAIL: Test 5b - sync not detected after re-enable");
            test_failed = 1;
        end
        if (capture_count != 1) begin
            $display("FAIL: Test 5b - Expected 1 byte, got %0d", capture_count);
            test_failed = 1;
        end else if (captured_bytes[0] != 8'h42) begin
            $display("FAIL: Test 5b - Expected $42, got $%02h", captured_bytes[0]);
            test_failed = 1;
        end

        if (!test_failed) begin
            $display("PASS: Test 5b - Re-sync and decode works after re-enable");
            pass_count = pass_count + 1;
        end else begin
            fail_count = fail_count + 1;
        end

        // =================================================================
        // TEST 6: 8-cell gap decode — $80 (10000000) = 7 zeros + 1 one
        // The decoder pipeline is now 8 bits wide and handles 8-cell gaps.
        // =================================================================
        $display("");
        $display("--- Test 6: 8-cell gap decode ($80) ---");
        reset_decoder;

        fm_send_sync(5);
        fm_send_byte(8'hC3);
        fm_send_byte(8'h80);

        fm_wait_flush;

        test_failed = 0;
        if (capture_count < 1) begin
            $display("FAIL: Test 6 - No bytes decoded for $80");
            test_failed = 1;
        end else if (captured_bytes[0] != 8'h80) begin
            $display("FAIL: Test 6 - Expected $80, got $%02h", captured_bytes[0]);
            test_failed = 1;
        end

        if (!test_failed) begin
            $display("PASS: Test 6 - 8-cell gap byte $80 decoded correctly");
            pass_count = pass_count + 1;
        end else begin
            fail_count = fail_count + 1;
        end

        // =================================================================
        // Summary
        // =================================================================
        $display("");
        $display("========================================");
        $display("  RESULTS: %0d PASSED, %0d FAILED", pass_count, fail_count);
        $display("========================================");

        if (fail_count == 0) begin
            $display("  ALL TESTS PASSED");
        end else begin
            $display("  SOME TESTS FAILED");
        end

        $display("========================================");
        $display("");

        #1000;
        $finish;
    end

    // =========================================================================
    // Timeout watchdog
    // =========================================================================
    initial begin
        #50000000;
        $display("FAIL: Global timeout — simulation took too long");
        $finish;
    end

endmodule
