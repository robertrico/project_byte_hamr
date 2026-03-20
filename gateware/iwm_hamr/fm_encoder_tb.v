// ===================================================================
// FM Encoder Testbench
// ===================================================================
// Tests FM encoding of SmartPort data bytes.
// Verifies falling edge count, edge spacing, idle state, multi-byte
// transmission, and round-trip decode via fm_decoder.
// ===================================================================

`timescale 1ns / 1ps

module fm_encoder_tb;

    // -----------------------------------------------------------
    // Clock and DUT signals
    // -----------------------------------------------------------
    reg        clk;
    reg        rst_n;
    reg        start;
    reg  [7:0] data_in;
    reg        data_valid;
    wire       data_request;
    wire       rddata_enc;
    wire       busy;
    wire       done;

    // Clock: 7.16 MHz -> period ~139.6 ns -> half-period ~69.8 ns
    initial clk = 0;
    always #69.8 clk = ~clk;

    // -----------------------------------------------------------
    // DUT instantiation
    // -----------------------------------------------------------
    fm_encoder dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .start        (start),
        .data_in      (data_in),
        .data_valid   (data_valid),
        .data_request (data_request),
        .rddata       (rddata_enc),
        .busy         (busy),
        .done         (done)
    );

    // -----------------------------------------------------------
    // VCD dump
    // -----------------------------------------------------------
    initial begin
        $dumpfile("fm_encoder_tb.vcd");
        $dumpvars(0, fm_encoder_tb);
    end

    // -----------------------------------------------------------
    // Test bookkeeping
    // -----------------------------------------------------------
    integer tests_passed;
    integer tests_failed;
    integer total_tests;

    // -----------------------------------------------------------
    // Sticky done flag -- captures single-cycle done pulse
    // -----------------------------------------------------------
    reg done_seen;

    always @(posedge clk) begin
        if (done)
            done_seen <= 1;
    end

    // -----------------------------------------------------------
    // Test 3: concurrent falling edge counter (always block)
    // -----------------------------------------------------------
    integer test3_fall_count;
    reg     test3_counting;
    reg     test3_prev_rd;
    initial begin test3_counting = 0; test3_fall_count = 0; end

    always @(posedge clk) begin
        if (test3_counting) begin
            if (test3_prev_rd == 1'b1 && rddata_enc == 1'b0)
                test3_fall_count = test3_fall_count + 1;
        end
        test3_prev_rd = rddata_enc;
    end

    // -----------------------------------------------------------
    // Helper: count falling edges on rddata_enc over N clocks
    // -----------------------------------------------------------
    integer fall_count;
    reg     prev_rddata;

    task count_falling_edges;
        input integer window_clks;
        integer i;
        begin
            fall_count = 0;
            prev_rddata = rddata_enc;
            for (i = 0; i < window_clks; i = i + 1) begin
                @(posedge clk);
                if (prev_rddata == 1'b1 && rddata_enc == 1'b0)
                    fall_count = fall_count + 1;
                prev_rddata = rddata_enc;
            end
        end
    endtask

    // -----------------------------------------------------------
    // Helper: collect falling edge timestamps (up to 16)
    // -----------------------------------------------------------
    integer edge_times [0:15];
    integer edge_count_collected;

    task collect_falling_edges;
        input integer window_clks;
        integer i;
        reg prev_rd;
        begin
            edge_count_collected = 0;
            prev_rd = rddata_enc;
            for (i = 0; i < window_clks; i = i + 1) begin
                @(posedge clk);
                if (prev_rd == 1'b1 && rddata_enc == 1'b0) begin
                    if (edge_count_collected < 16) begin
                        edge_times[edge_count_collected] = $time;
                        edge_count_collected = edge_count_collected + 1;
                    end
                end
                prev_rd = rddata_enc;
            end
        end
    endtask

    // -----------------------------------------------------------
    // Helper: reset the DUT and clear done flag
    // -----------------------------------------------------------
    task do_reset;
        begin
            rst_n      = 0;
            start      = 0;
            data_in    = 8'h00;
            data_valid = 0;
            done_seen  = 0;
            repeat (4) @(posedge clk);
            rst_n = 1;
            repeat (2) @(posedge clk);
        end
    endtask

    // -----------------------------------------------------------
    // Helper: send a single byte (no continuation)
    // -----------------------------------------------------------
    task send_single_byte;
        input [7:0] byte_val;
        begin
            data_in    = byte_val;
            data_valid = 0;
            @(posedge clk);
            start = 1;
            @(posedge clk);
            start = 0;
        end
    endtask

    // -----------------------------------------------------------
    // Helper: wait for done_seen flag with timeout
    // -----------------------------------------------------------
    task wait_for_done;
        input integer timeout_clks;
        integer i;
        begin
            for (i = 0; i < timeout_clks; i = i + 1) begin
                if (done_seen) i = timeout_clks;
                else @(posedge clk);
            end
        end
    endtask

    // -----------------------------------------------------------
    // Helper: send a multi-byte packet using the handshake.
    // packet_buf and packet_len must be set before call.
    //
    // Protocol: data_in must hold the NEXT byte before the
    // encoder reaches BYTE_DONE. At start, byte 0 is captured.
    // We immediately pre-load byte 1. When data_request fires
    // (byte N consumed), we advance to byte N+1.
    // -----------------------------------------------------------
    reg [7:0] packet_buf [0:15];
    integer   packet_len;

    task send_packet;
        integer next_serve;
        begin
            data_in    = packet_buf[0];
            data_valid = (packet_len > 1) ? 1'b1 : 1'b0;
            next_serve = 1;

            @(posedge clk);
            start = 1;
            @(posedge clk);
            start = 0;

            // Encoder captured packet_buf[0]. Pre-load next byte.
            if (next_serve < packet_len) begin
                data_in    = packet_buf[next_serve];
                data_valid = 1'b1;
                next_serve = next_serve + 1;
            end else begin
                data_valid = 1'b0;
            end

            // Respond to data_request by advancing the feeder
            while (!done) begin
                @(posedge clk);
                if (data_request) begin
                    if (next_serve < packet_len) begin
                        data_in    = packet_buf[next_serve];
                        data_valid = 1'b1;
                        next_serve = next_serve + 1;
                    end else begin
                        data_valid = 1'b0;
                    end
                end
            end
        end
    endtask

    // =========================================================
    // Round-trip decoder signals and capture (test 5)
    // =========================================================
    // MUX rddata so we can inject a kick pulse before the
    // encoder starts, calibrating the decoder's first_edge.
    // In real hardware, this edge comes from the bus
    // idle-to-active transition.
    reg        kick_active;
    reg        rddata_kick;
    wire       rddata_wire = kick_active ? rddata_kick : rddata_enc;

    reg        dec_enable;
    wire [7:0] dec_data_out;
    wire       dec_data_valid;
    wire       dec_sync_detected;
    wire       dec_packet_end;

    fm_decoder dec (
        .clk           (clk),
        .rst_n         (rst_n),
        .enable        (dec_enable),
        .wrdata        (rddata_wire),
        .data_out      (dec_data_out),
        .data_valid    (dec_data_valid),
        .sync_detected (dec_sync_detected),
        .packet_end    (dec_packet_end)
    );

    // Decoder RX capture buffer
    reg [7:0] rx_buf [0:15];
    integer   rx_count;

    always @(posedge clk) begin
        if (dec_data_valid && rx_count < 16) begin
            rx_buf[rx_count] = dec_data_out;
            rx_count = rx_count + 1;
        end
    end

    // -----------------------------------------------------------
    // Main test sequence
    // -----------------------------------------------------------
    initial begin
        tests_passed = 0;
        tests_failed = 0;
        total_tests  = 0;
        kick_active  = 0;
        rddata_kick  = 1;
        dec_enable   = 0;
        rx_count     = 0;

        // =======================================================
        // TEST 1: Single byte $FF -- 8 "1" bits -> 8 falling edges
        // Each edge spaced ~28 fclk (one bit cell) apart.
        // =======================================================
        do_reset;
        $display("\n--- Test 1: Single byte $FF (8 ones) ---");
        total_tests = total_tests + 1;

        send_single_byte(8'hFF);

        collect_falling_edges(280);

        $display("  Falling edges: %0d (expected 8)", edge_count_collected);
        if (edge_count_collected == 8) begin
            begin : test1_spacing
                integer i, gap, nominal, tolerance;
                reg ok;
                ok        = 1;
                nominal   = 28 * 139;  // ~3892 ns
                tolerance = 300;
                for (i = 1; i < 8; i = i + 1) begin
                    gap = edge_times[i] - edge_times[i-1];
                    if (gap < (nominal - tolerance) || gap > (nominal + tolerance)) begin
                        $display("  Edge %0d->%0d: gap=%0d ns (expected ~%0d) BAD",
                                 i-1, i, gap, nominal);
                        ok = 0;
                    end
                end
                if (ok) begin
                    $display("PASS: $FF produces 8 falling edges at ~28-cycle intervals");
                    tests_passed = tests_passed + 1;
                end else begin
                    $display("FAIL: $FF edge spacing incorrect");
                    tests_failed = tests_failed + 1;
                end
            end
        end else begin
            $display("FAIL: Expected 8 falling edges, got %0d", edge_count_collected);
            tests_failed = tests_failed + 1;
        end

        // =======================================================
        // TEST 2: Mixed byte $A5 (10100101)
        // "1" at bit positions 7,5,2,0 -> 4 falling edges
        // =======================================================
        do_reset;
        $display("\n--- Test 2: Mixed byte $A5 (10100101) ---");
        total_tests = total_tests + 1;

        send_single_byte(8'hA5);
        collect_falling_edges(280);

        $display("  Falling edges: %0d (expected 4)", edge_count_collected);
        if (edge_count_collected == 4) begin
            begin : test2_spacing
                integer gap01, gap12, gap23;
                integer nom_2cell, nom_3cell, tol;
                reg ok;
                ok         = 1;
                // Each bit cell = 28 fclk in BIT_CELL + 1 fclk for NEXT_BIT = 29 effective.
                // Inter-edge gap for N cells = N * 29 * 139.6 ns (approx N * 29 * 139).
                nom_2cell  = 58 * 139;   // 2 cells (58 fclk) ~8062 ns
                nom_3cell  = 87 * 139;   // 3 cells (87 fclk) ~12093 ns
                tol        = 500;

                gap01 = edge_times[1] - edge_times[0];
                gap12 = edge_times[2] - edge_times[1];
                gap23 = edge_times[3] - edge_times[2];

                $display("  Gap 0->1: %0d ns (expect ~%0d = 2 cells)", gap01, nom_2cell);
                $display("  Gap 1->2: %0d ns (expect ~%0d = 3 cells)", gap12, nom_3cell);
                $display("  Gap 2->3: %0d ns (expect ~%0d = 2 cells)", gap23, nom_2cell);

                if (gap01 < (nom_2cell - tol) || gap01 > (nom_2cell + tol)) ok = 0;
                if (gap12 < (nom_3cell - tol) || gap12 > (nom_3cell + tol)) ok = 0;
                if (gap23 < (nom_2cell - tol) || gap23 > (nom_2cell + tol)) ok = 0;

                if (ok) begin
                    $display("PASS: $A5 edge timing matches expected FM pattern");
                    tests_passed = tests_passed + 1;
                end else begin
                    $display("FAIL: $A5 edge timing mismatch");
                    tests_failed = tests_failed + 1;
                end
            end
        end else begin
            $display("FAIL: Expected 4 falling edges, got %0d", edge_count_collected);
            tests_failed = tests_failed + 1;
        end

        // =======================================================
        // TEST 3: Multi-byte $FF + $C3 + $DB (3 bytes)
        // $FF(11111111)=8, $C3(11000011)=4, $DB(11011011)=6 edges
        // Total = 18 falling edges
        // =======================================================
        do_reset;
        $display("\n--- Test 3: Multi-byte $FF + $C3 + $DB ---");
        total_tests = total_tests + 1;

        packet_buf[0] = 8'hFF;
        packet_buf[1] = 8'hC3;
        packet_buf[2] = 8'hDB;
        packet_len = 3;

        // Reset edge counter (test3_fall_count), send packet,
        // then read the counter after done.
        test3_fall_count = 0;
        test3_counting   = 1;
        done_seen = 0;

        send_packet;

        // Small delay to catch any trailing edges
        repeat (10) @(posedge clk);
        test3_counting = 0;

        $display("  Falling edges: %0d (expected 18)", test3_fall_count);
        if (test3_fall_count == 18) begin
            $display("PASS: Multi-byte produces correct falling edge count (18)");
            tests_passed = tests_passed + 1;
        end else begin
            $display("FAIL: Expected 18 falling edges, got %0d", test3_fall_count);
            tests_failed = tests_failed + 1;
        end

        // =======================================================
        // TEST 4: Idle state -- after transmission, rddata=HIGH,
        // busy=LOW, done pulses exactly once
        // =======================================================
        do_reset;
        $display("\n--- Test 4: Idle state after completion ---");
        total_tests = total_tests + 1;

        send_single_byte(8'hFF);
        wait_for_done(300);

        begin : test4_block
            reg pass4;
            pass4 = 1;

            @(posedge clk);

            if (!done_seen) begin
                $display("  done never pulsed");
                pass4 = 0;
            end

            if (rddata_enc !== 1'b1) begin
                $display("  rddata=%b after done (expected 1)", rddata_enc);
                pass4 = 0;
            end

            if (busy !== 1'b0) begin
                $display("  busy=%b after done (expected 0)", busy);
                pass4 = 0;
            end

            if (done !== 1'b0) begin
                $display("  done still HIGH (expected single-cycle pulse)");
                pass4 = 0;
            end

            if (pass4) begin
                $display("PASS: rddata=HIGH, busy=LOW, done pulsed once");
                tests_passed = tests_passed + 1;
            end else begin
                $display("FAIL: Post-completion state incorrect");
                tests_failed = tests_failed + 1;
            end
        end

        // =======================================================
        // TEST 5: Round-trip with fm_decoder
        //
        // Encode 5x$FF sync + $C3 + test bytes + $C8, feed
        // encoder output through the fm_decoder, verify decoded
        // bytes match the transmitted payload.
        //
        // The fm_decoder requires a calibration falling edge
        // (first_edge) before productive decoding begins. In real
        // SmartPort hardware, this comes from the bus transition
        // when the device starts driving rddata. We simulate it
        // with a brief kick pulse on rddata before starting the
        // encoder.
        // =======================================================
        do_reset;
        $display("\n--- Test 5: Round-trip with fm_decoder ---");
        total_tests = total_tests + 1;

        dec_enable = 1;
        rx_count   = 0;
        repeat (2) @(posedge clk);

        // Kick pulse: brief LOW on rddata to calibrate first_edge
        kick_active = 1;
        rddata_kick = 1;
        repeat (5) @(posedge clk);
        rddata_kick = 0;          // falling edge for first_edge calibration
        repeat (4) @(posedge clk);
        rddata_kick = 1;
        kick_active = 0;          // hand control back to encoder

        // Build packet: 5x$FF sync + $C3 PBEGIN + payload + $C8 PEND
        packet_buf[0] = 8'hFF;
        packet_buf[1] = 8'hFF;
        packet_buf[2] = 8'hFF;
        packet_buf[3] = 8'hFF;
        packet_buf[4] = 8'hFF;
        packet_buf[5] = 8'hC3;   // PBEGIN
        packet_buf[6] = 8'hDB;   // test byte 1 (11011011)
        packet_buf[7] = 8'hA5;   // test byte 2 (10100101)
        packet_buf[8] = 8'hC8;   // PEND
        packet_len = 9;

        send_packet;

        // Wait for decoder timeout flush of trailing zeros
        repeat (300) @(posedge clk);

        begin : test5_block
            reg pass5;
            pass5 = 1;

            if (!dec_sync_detected) begin
                $display("  Sync not detected by decoder");
                pass5 = 0;
            end else begin
                $display("  Sync detected: OK");
            end

            if (rx_count < 3) begin
                $display("  Only %0d bytes decoded (expected >=3: $DB $A5 $C8)", rx_count);
                pass5 = 0;
            end else begin
                if (rx_buf[0] !== 8'hDB) begin
                    $display("  Byte 0: $%02X (expected $DB)", rx_buf[0]);
                    pass5 = 0;
                end else begin
                    $display("  Byte 0: $DB OK");
                end
                if (rx_buf[1] !== 8'hA5) begin
                    $display("  Byte 1: $%02X (expected $A5)", rx_buf[1]);
                    pass5 = 0;
                end else begin
                    $display("  Byte 1: $A5 OK");
                end
                if (rx_buf[2] !== 8'hC8) begin
                    $display("  Byte 2: $%02X (expected $C8)", rx_buf[2]);
                    pass5 = 0;
                end else begin
                    $display("  Byte 2: $C8 OK");
                end
            end

            if (pass5) begin
                $display("PASS: Round-trip encode->decode matches ($DB $A5 $C8)");
                tests_passed = tests_passed + 1;
            end else begin
                $display("FAIL: Round-trip decode mismatch");
                tests_failed = tests_failed + 1;
            end
        end

        dec_enable = 0;

        // =======================================================
        // Summary
        // =======================================================
        #500;
        $display("\n===================================================");
        $display("FM Encoder Testbench Summary");
        $display("===================================================");
        $display("  Total:  %0d", total_tests);
        $display("  Passed: %0d", tests_passed);
        $display("  Failed: %0d", tests_failed);
        if (tests_failed == 0)
            $display("  ALL TESTS PASSED");
        else
            $display("  *** SOME TESTS FAILED ***");
        $display("===================================================\n");

        $finish;
    end

    // -----------------------------------------------------------
    // Timeout watchdog
    // -----------------------------------------------------------
    initial begin
        #5000000;
        $display("FAIL: Testbench timed out");
        $finish;
    end

endmodule
