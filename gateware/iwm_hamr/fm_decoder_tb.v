// fm_decoder_tb.v — Testbench for fm_decoder module
// Tests sync detection, byte decoding, multi-byte packets, PEND, enable gating.
// Uses FM waveform generator tasks to produce wrdata edges with correct timing.
//
// Each test sends a complete packet: sync(5) + $C3 + payload [+ $C8].
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
    // FM waveform generation
    // =========================================================================
    //
    // FM encoding at ~7 MHz fclk:
    //   Bit cell = 28 fclk cycles (~4 us)
    //   "1" bit = falling edge at cell midpoint
    //   "0" bit = no edge during cell
    //
    // The decoder measures the interval between consecutive falling edges:
    //   ~28 fclk  = consecutive "1" bits (1 bit cell apart)
    //   ~56 fclk  = "1","0","1" (2 bit cells apart)
    //   ~84 fclk  = "1","0","0","1" (3 bit cells apart)
    //   etc.
    //
    // The generator scans bits MSB-first. For each "1" bit, it produces
    // a falling edge after the appropriate delay. "0" bits accumulate
    // delay between edges. A pulse (LOW for PULSE_LEN clocks, then HIGH)
    // models each wrdata falling edge.

    localparam integer BIT_CELL   = 28;  // fclk cycles per bit cell
    localparam integer PULSE_LEN  = 4;   // fclk cycles wrdata stays LOW

    // fm_send_byte: generate FM-encoded wrdata for one byte (MSB first).
    //
    // Before calling for the first byte in a sequence, a kickoff edge
    // must have been sent (via fm_kickoff) to prime the decoder's
    // first_edge flag. Subsequent bytes can be sent back-to-back.
    //
    // Each "0" bit adds one bit-cell of silence before the next edge.
    // Each "1" bit fires an edge after the accumulated silence.
    // Trailing zeros add idle time that carries into the next byte.

    task fm_send_byte;
        input [7:0] byte_val;
        integer b, gap, j;
        begin
            gap = 0;
            for (b = 7; b >= 0; b = b - 1) begin
                if (byte_val[b]) begin
                    // "1" bit: produce a falling edge.
                    // Wait (gap+1) bit cells since last edge,
                    // minus PULSE_LEN already elapsed from last pulse.
                    if (gap == 0) begin
                        for (j = 0; j < BIT_CELL - PULSE_LEN; j = j + 1)
                            @(posedge clk);
                    end else begin
                        for (j = 0; j < (gap + 1) * BIT_CELL - PULSE_LEN; j = j + 1)
                            @(posedge clk);
                    end

                    // Falling edge: drive LOW for PULSE_LEN
                    wrdata = 1'b0;
                    for (j = 0; j < PULSE_LEN; j = j + 1)
                        @(posedge clk);
                    wrdata = 1'b1;

                    gap = 0;
                end else begin
                    gap = gap + 1;
                end
            end

            // Trailing zeros: add their idle time so the next byte's
            // first edge has the correct edge-to-edge interval.
            if (gap > 0) begin
                for (j = 0; j < gap * BIT_CELL; j = j + 1)
                    @(posedge clk);
            end
        end
    endtask

    // fm_kickoff: send an initial falling edge to prime the decoder.
    task fm_kickoff;
        integer j;
        begin
            wrdata = 1'b0;
            for (j = 0; j < PULSE_LEN; j = j + 1)
                @(posedge clk);
            wrdata = 1'b1;
        end
    endtask

    // fm_send_sync: send a kickoff edge followed by N bytes of $FF.
    task fm_send_sync;
        input integer count;
        integer n;
        begin
            fm_kickoff;
            for (n = 0; n < count; n = n + 1)
                fm_send_byte(8'hFF);
        end
    endtask

    // fm_idle: hold wrdata HIGH for some fclk cycles.
    task fm_idle;
        input integer num_clocks;
        integer j;
        begin
            wrdata = 1'b1;
            for (j = 0; j < num_clocks; j = j + 1)
                @(posedge clk);
        end
    endtask

    // fm_wait_flush: wait long enough for the decoder's timeout flush
    // to trigger. The decoder flushes partial bytes after 224 fclk
    // cycles with no edge. We wait 240 to give margin.
    task fm_wait_flush;
        begin
            repeat (240) @(posedge clk);
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
        // =================================================================
        $display("");
        $display("--- Test 1: Sync detection ---");
        enable = 1;
        capture_count = 0;

        fm_idle(20);

        // Send sync + PBEGIN
        fm_send_sync(5);
        fm_send_byte(8'hC3);

        // Wait for pipeline drain without adding inter-byte gap
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
