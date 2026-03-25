// ===================================================================
// FM Encoder Testbench — Buffer-driven interface
// ===================================================================
// Tests FM encoding of SmartPort data bytes via buffer read interface.
// The encoder reads sequentially from buf_data using buf_addr.
//
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
    reg  [9:0] byte_count;
    wire       rddata_enc;
    wire       busy;
    wire       done;
    wire [9:0] buf_addr;
    reg  [7:0] buf_data;

    // Clock: 7.16 MHz -> period ~139.6 ns -> half-period ~69.8 ns
    initial clk = 0;
    always #69.8 clk = ~clk;

    // -----------------------------------------------------------
    // TX buffer (encoder reads from this via buf_addr/buf_data)
    // -----------------------------------------------------------
    reg [7:0] tx_buf [0:1023];

    // Combinatorial read: encoder sets buf_addr, we provide buf_data
    always @(*) begin
        buf_data = tx_buf[buf_addr];
    end

    // -----------------------------------------------------------
    // DUT instantiation — new buffer-driven interface
    // -----------------------------------------------------------
    fm_encoder dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .start        (start),
        .byte_count   (byte_count),
        .rddata       (rddata_enc),
        .busy         (busy),
        .done         (done),
        .buf_addr     (buf_addr),
        .buf_data     (buf_data)
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
            byte_count = 10'd0;
            done_seen  = 0;
            repeat (4) @(posedge clk);
            rst_n = 1;
            repeat (2) @(posedge clk);
        end
    endtask

    // -----------------------------------------------------------
    // Helper: send a single byte via buffer
    // -----------------------------------------------------------
    task send_single_byte;
        input [7:0] byte_val;
        begin
            tx_buf[0] = byte_val;
            byte_count = 10'd1;
            @(posedge clk);
            start = 1;
            @(posedge clk);
            start = 0;
        end
    endtask

    // -----------------------------------------------------------
    // Helper: send a multi-byte packet via buffer
    // Caller must fill tx_buf[0..len-1] before calling.
    // -----------------------------------------------------------
    task send_packet;
        input integer len;
        begin
            byte_count = len;
            done_seen = 0;
            @(posedge clk);
            start = 1;
            @(posedge clk);
            start = 0;

            // Wait for done
            while (!done) begin
                @(posedge clk);
            end
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

    // =========================================================
    // Round-trip decoder signals and capture (test 5)
    // =========================================================
    // The FM encoder outputs falling-edge pulses (rddata LOW
    // for PULSE_WIDTH cycles per "1" bit). The FM decoder uses
    // XOR edge detection (any transition = "1" bit). For a
    // direct loopback, each pulse creates two edges (fall+rise),
    // doubling the bit count. We interpose a pulse-to-toggle
    // adapter that converts each falling edge into a toggle,
    // matching the IWM write serializer behavior.
    //
    // The kick_active/rddata_kick mux injects an initial toggle
    // to prime the decoder's first_edge before the encoder starts.
    // =========================================================

    // Pulse-to-toggle adapter: detect falling edge on rddata_enc,
    // toggle dec_wrdata. This simulates the IWM's behavior where
    // each "1" bit causes a wrdata toggle.
    reg        dec_wrdata = 1'b1;
    reg        enc_prev = 1'b1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dec_wrdata <= 1'b1;
            enc_prev   <= 1'b1;
        end else begin
            enc_prev <= rddata_enc;
            if (enc_prev && !rddata_enc)  // falling edge on encoder output
                dec_wrdata <= ~dec_wrdata;
        end
    end

    reg        kick_active;
    reg        rddata_kick;
    wire       rddata_wire = kick_active ? rddata_kick : dec_wrdata;

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
                // Bit 7=1, bit 6=0, bit 5=1: 2-cell gap
                // Bit 5=1, bit 4=0, bit 3=0, bit 2=1: 3-cell gap
                // Bit 2=1, bit 1=0, bit 0=1: 2-cell gap
                // Each bit cell = 28 fclk in BIT_CELL + 1 fclk for NEXT_BIT = 29 effective.
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

        tx_buf[0] = 8'hFF;
        tx_buf[1] = 8'hC3;
        tx_buf[2] = 8'hDB;

        // Reset edge counter, send packet, count edges
        test3_fall_count = 0;
        test3_counting   = 1;
        done_seen = 0;

        send_packet(3);

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
        // encoder output through pulse-to-toggle adapter to
        // the fm_decoder, verify decoded bytes match payload.
        //
        // The adapter converts each encoder falling edge into
        // a toggle on dec_wrdata, matching IWM behavior. The
        // first toggle from the encoder serves as the decoder's
        // first_edge calibration. No explicit kick pulse needed
        // since the sync pattern has enough 1-bits for recovery.
        // =======================================================
        do_reset;
        $display("\n--- Test 5: Round-trip with fm_decoder ---");
        total_tests = total_tests + 1;

        kick_active = 0;          // use adapter output directly
        dec_enable = 1;
        rx_count   = 0;
        repeat (2) @(posedge clk);

        // Build packet in buffer: 5x$FF sync + $C3 PBEGIN + payload + $C8 PEND
        tx_buf[0] = 8'hFF;
        tx_buf[1] = 8'hFF;
        tx_buf[2] = 8'hFF;
        tx_buf[3] = 8'hFF;
        tx_buf[4] = 8'hFF;
        tx_buf[5] = 8'hC3;   // PBEGIN
        tx_buf[6] = 8'hDB;   // test byte 1 (11011011)
        tx_buf[7] = 8'hA5;   // test byte 2 (10100101)
        tx_buf[8] = 8'hC8;   // PEND

        send_packet(9);

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
