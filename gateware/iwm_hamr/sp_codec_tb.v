`timescale 1ns / 1ps

// ===================================================================
// SmartPort Codec Testbench
// ===================================================================
// Tests group-of-7 encode/decode and packet framing.
//
// Test 1: Decode INIT command (no payload)
// Test 2: Decode STATUS command (no payload)
// Test 3: Encode STATUS response (no payload)
// Test 4: Encode READBLOCK response (4-byte payload, oddcnt=4)
// Test 5: Round-trip (encode then decode, verify fields match)
// ===================================================================

module sp_codec_tb;

    // -----------------------------------------------------------
    // Clock and DUT signals
    // -----------------------------------------------------------
    reg         clk;
    reg         rst_n;

    // Decode interface
    reg         decode_start;
    reg  [7:0]  decode_byte_in;
    reg         decode_byte_in_valid;
    wire [6:0]  decode_dest;
    wire [6:0]  decode_source;
    wire [6:0]  decode_ptype;
    wire [6:0]  decode_aux;
    wire [6:0]  decode_stat;
    wire        decode_header_valid;
    wire [7:0]  decode_payload_out;
    wire        decode_payload_valid;
    wire [8:0]  decode_payload_addr;
    wire        decode_done;
    wire        decode_error;

    // Encode interface
    reg         encode_start;
    reg  [6:0]  encode_dest;
    reg  [6:0]  encode_source;
    reg  [6:0]  encode_ptype;
    reg  [6:0]  encode_stat;
    reg  [8:0]  encode_payload_len;
    wire [7:0]  encode_byte_out;
    wire        encode_byte_valid;
    reg         encode_byte_ready;
    wire        encode_done;
    wire [8:0]  encode_buf_addr;
    reg  [7:0]  encode_buf_data;

    // Clock: ~10 MHz (50ns half-period, 100ns period)
    initial clk = 0;
    always #50 clk = ~clk;

    // -----------------------------------------------------------
    // DUT instantiation
    // -----------------------------------------------------------
    sp_codec dut (
        .clk                (clk),
        .rst_n              (rst_n),
        .decode_start       (decode_start),
        .decode_byte_in     (decode_byte_in),
        .decode_byte_in_valid (decode_byte_in_valid),
        .decode_dest        (decode_dest),
        .decode_source      (decode_source),
        .decode_ptype       (decode_ptype),
        .decode_aux         (decode_aux),
        .decode_stat        (decode_stat),
        .decode_header_valid (decode_header_valid),
        .decode_payload_out (decode_payload_out),
        .decode_payload_valid (decode_payload_valid),
        .decode_payload_addr (decode_payload_addr),
        .decode_done        (decode_done),
        .decode_error       (decode_error),
        .encode_start       (encode_start),
        .encode_dest        (encode_dest),
        .encode_source      (encode_source),
        .encode_ptype       (encode_ptype),
        .encode_stat        (encode_stat),
        .encode_payload_len (encode_payload_len),
        .encode_byte_out    (encode_byte_out),
        .encode_byte_valid  (encode_byte_valid),
        .encode_byte_ready  (encode_byte_ready),
        .encode_done        (encode_done),
        .encode_buf_addr    (encode_buf_addr),
        .encode_buf_data    (encode_buf_data)
    );

    // -----------------------------------------------------------
    // VCD dump
    // -----------------------------------------------------------
    initial begin
        $dumpfile("sp_codec_tb.vcd");
        $dumpvars(0, sp_codec_tb);
    end

    // -----------------------------------------------------------
    // Test buffer for encode payload (4 bytes for test 4)
    // -----------------------------------------------------------
    reg [7:0] test_buf [0:511];
    initial begin
        test_buf[0] = 8'hDE;
        test_buf[1] = 8'hAD;
        test_buf[2] = 8'hBE;
        test_buf[3] = 8'hEF;
    end

    // Combinatorial buffer read for encoder
    always @(*) begin
        encode_buf_data = test_buf[encode_buf_addr];
    end

    // -----------------------------------------------------------
    // Captured encode output buffer (up to 64 bytes for tests)
    // -----------------------------------------------------------
    reg [7:0]  enc_capture [0:63];
    integer    enc_capture_cnt;

    // -----------------------------------------------------------
    // Decoded payload capture buffer
    // -----------------------------------------------------------
    reg [7:0]  dec_payload_capture [0:511];
    integer    dec_payload_cnt;

    // -----------------------------------------------------------
    // Test bookkeeping
    // -----------------------------------------------------------
    integer tests_passed;
    integer tests_failed;
    integer total_tests;

    // -----------------------------------------------------------
    // Helper: reset the DUT
    // -----------------------------------------------------------
    task do_reset;
        begin
            rst_n              = 0;
            decode_start       = 0;
            decode_byte_in     = 8'd0;
            decode_byte_in_valid = 0;
            encode_start       = 0;
            encode_dest        = 7'd0;
            encode_source      = 7'd0;
            encode_ptype       = 7'd0;
            encode_stat        = 7'd0;
            encode_payload_len = 9'd0;
            encode_byte_ready  = 1;
            enc_capture_cnt    = 0;
            dec_payload_cnt    = 0;
            repeat (4) @(posedge clk);
            rst_n = 1;
            repeat (2) @(posedge clk);
        end
    endtask

    // -----------------------------------------------------------
    // Helper: feed one wire byte to decoder (single-cycle pulse)
    // -----------------------------------------------------------
    task feed_byte;
        input [7:0] b;
        begin
            @(posedge clk);
            decode_byte_in       = b;
            decode_byte_in_valid = 1;
            @(posedge clk);
            decode_byte_in_valid = 0;
        end
    endtask

    // -----------------------------------------------------------
    // Helper: run encoder until done, capture output bytes
    // -----------------------------------------------------------
    task run_encoder;
        begin
            enc_capture_cnt = 0;
            // Encoder runs one byte per cycle when ready
            while (!encode_done) begin
                @(posedge clk);
                if (encode_byte_valid) begin
                    if (enc_capture_cnt < 64) begin
                        enc_capture[enc_capture_cnt] = encode_byte_out;
                        enc_capture_cnt = enc_capture_cnt + 1;
                    end
                end
            end
        end
    endtask

    // -----------------------------------------------------------
    // Helper: compute expected checksum for a sequence of bytes
    // XOR all bytes, then split into two 7-bit halves with bit7=1
    // -----------------------------------------------------------
    // (Used in verification only)

    // -----------------------------------------------------------
    // Main test sequence
    // -----------------------------------------------------------
    integer i;
    integer saved_enc_cnt;
    reg test_pass;

    initial begin
        tests_passed = 0;
        tests_failed = 0;
        total_tests  = 0;

        // =======================================================
        // TEST 1: Decode INIT command packet
        // =======================================================
        // INIT command: dest=1, source=2, type=1(cmd), aux=0, stat=1(INIT)
        // No payload: oddcnt=0, grp7cnt=0
        // Wire bytes (after sync+C3 which are handled by FM decoder):
        //   $81 $82 $81 $80 $81  (header: dest|80, src|80, type|80, aux|80, stat|80)
        //   $80 $80              (oddcnt=0|80, grp7cnt=0|80)
        //   $xx $xx              (checksum — we skip verification)
        // =======================================================
        do_reset;
        $display("\n--- Test 1: Decode INIT command ---");
        total_tests = total_tests + 1;
        test_pass = 1;

        // Pulse decode_start
        @(posedge clk);
        decode_start = 1;
        @(posedge clk);
        decode_start = 0;

        // Feed header bytes
        feed_byte(8'h81); // DEST: 1 | $80
        feed_byte(8'h82); // SOURCE: 2 | $80
        feed_byte(8'h81); // TYPE: 1 (command) | $80
        feed_byte(8'h80); // AUX: 0 | $80
        feed_byte(8'h81); // STAT: 1 (INIT) | $80

        // Check header valid
        @(posedge clk);
        if (!decode_header_valid) begin
            $display("  ERROR: decode_header_valid not asserted after 5 header bytes");
            test_pass = 0;
        end

        // Feed counts
        feed_byte(8'h80); // ODDCNT: 0 | $80
        feed_byte(8'h80); // GRP7CNT: 0 | $80

        // Feed 2 dummy checksum bytes
        feed_byte(8'h80);
        feed_byte(8'h80);

        // Wait for decode_done
        repeat (4) @(posedge clk);

        if (!decode_done && !test_pass) begin
            // Already failed
        end

        // Give it a cycle to settle
        @(posedge clk);

        // Verify fields (they should be latched from when header_valid fired)
        if (decode_dest !== 7'd1) begin
            $display("  ERROR: decode_dest = %0d, expected 1", decode_dest);
            test_pass = 0;
        end
        if (decode_source !== 7'd2) begin
            $display("  ERROR: decode_source = %0d, expected 2", decode_source);
            test_pass = 0;
        end
        if (decode_ptype !== 7'd1) begin
            $display("  ERROR: decode_ptype = %0d, expected 1", decode_ptype);
            test_pass = 0;
        end
        if (decode_stat !== 7'd1) begin
            $display("  ERROR: decode_stat = %0d, expected 1 (INIT)", decode_stat);
            test_pass = 0;
        end

        if (test_pass) begin
            $display("PASS: Test 1 — Decode INIT command");
            tests_passed = tests_passed + 1;
        end else begin
            $display("FAIL: Test 1 — Decode INIT command");
            tests_failed = tests_failed + 1;
        end

        // =======================================================
        // TEST 2: Decode STATUS command
        // =======================================================
        // STATUS cmd: dest=1, source=3, type=1(cmd), aux=0, stat=0(STATUS)
        // =======================================================
        do_reset;
        $display("\n--- Test 2: Decode STATUS command ---");
        total_tests = total_tests + 1;
        test_pass = 1;

        @(posedge clk);
        decode_start = 1;
        @(posedge clk);
        decode_start = 0;

        feed_byte(8'h81); // DEST: 1 | $80
        feed_byte(8'h83); // SOURCE: 3 | $80
        feed_byte(8'h81); // TYPE: 1 (command) | $80
        feed_byte(8'h80); // AUX: 0 | $80
        feed_byte(8'h80); // STAT: 0 (STATUS) | $80

        @(posedge clk);
        if (!decode_header_valid) begin
            $display("  ERROR: decode_header_valid not asserted");
            test_pass = 0;
        end

        feed_byte(8'h80); // ODDCNT: 0
        feed_byte(8'h80); // GRP7CNT: 0

        feed_byte(8'h80); // checksum lo
        feed_byte(8'h80); // checksum hi

        repeat (4) @(posedge clk);
        @(posedge clk);

        if (decode_dest !== 7'd1) begin
            $display("  ERROR: decode_dest = %0d, expected 1", decode_dest);
            test_pass = 0;
        end
        if (decode_source !== 7'd3) begin
            $display("  ERROR: decode_source = %0d, expected 3", decode_source);
            test_pass = 0;
        end
        if (decode_ptype !== 7'd1) begin
            $display("  ERROR: decode_ptype = %0d, expected 1", decode_ptype);
            test_pass = 0;
        end
        if (decode_stat !== 7'd0) begin
            $display("  ERROR: decode_stat = %0d, expected 0 (STATUS)", decode_stat);
            test_pass = 0;
        end

        if (test_pass) begin
            $display("PASS: Test 2 — Decode STATUS command");
            tests_passed = tests_passed + 1;
        end else begin
            $display("FAIL: Test 2 — Decode STATUS command");
            tests_failed = tests_failed + 1;
        end

        // =======================================================
        // TEST 3: Encode STATUS response (no payload)
        // =======================================================
        // Encode: dest=3, source=1, type=2(response), stat=0(OK)
        // payload_len=0
        // Expected wire bytes:
        //   FF FF FF FF FF C3
        //   83 81 82 80 80     (dest=3|80, src=1|80, type=2|80, aux=0|80, stat=0|80)
        //   80 80              (oddcnt=0|80, grp7cnt=0|80)
        //   xx xx              (checksum)
        //   C8                 (PEND)
        // =======================================================
        do_reset;
        $display("\n--- Test 3: Encode STATUS response (no payload) ---");
        total_tests = total_tests + 1;
        test_pass = 1;

        encode_dest        = 7'd3;
        encode_source      = 7'd1;
        encode_ptype       = 7'd2;
        encode_stat        = 7'd0;
        encode_payload_len = 9'd0;
        encode_byte_ready  = 1;

        @(posedge clk);
        encode_start = 1;
        @(posedge clk);
        encode_start = 0;

        run_encoder;

        $display("  Encoded %0d bytes", enc_capture_cnt);

        // Expected: 5 sync + 1 PBEGIN + 5 header + 2 counts + 2 checksum + 1 PEND = 16
        if (enc_capture_cnt != 16) begin
            $display("  ERROR: expected 16 bytes, got %0d", enc_capture_cnt);
            test_pass = 0;
        end

        // Verify sync bytes
        for (i = 0; i < 5; i = i + 1) begin
            if (enc_capture[i] !== 8'hFF) begin
                $display("  ERROR: sync[%0d] = %02h, expected FF", i, enc_capture[i]);
                test_pass = 0;
            end
        end

        // Verify PBEGIN
        if (enc_capture[5] !== 8'hC3) begin
            $display("  ERROR: PBEGIN = %02h, expected C3", enc_capture[5]);
            test_pass = 0;
        end

        // Verify header
        if (enc_capture[6] !== 8'h83) begin // dest=3|80
            $display("  ERROR: DEST = %02h, expected 83", enc_capture[6]);
            test_pass = 0;
        end
        if (enc_capture[7] !== 8'h81) begin // source=1|80
            $display("  ERROR: SOURCE = %02h, expected 81", enc_capture[7]);
            test_pass = 0;
        end
        if (enc_capture[8] !== 8'h82) begin // type=2|80
            $display("  ERROR: TYPE = %02h, expected 82", enc_capture[8]);
            test_pass = 0;
        end
        if (enc_capture[9] !== 8'h80) begin // aux=0|80
            $display("  ERROR: AUX = %02h, expected 80", enc_capture[9]);
            test_pass = 0;
        end
        if (enc_capture[10] !== 8'h80) begin // stat=0|80
            $display("  ERROR: STAT = %02h, expected 80", enc_capture[10]);
            test_pass = 0;
        end

        // Verify counts
        if (enc_capture[11] !== 8'h80) begin // oddcnt=0|80
            $display("  ERROR: ODDCNT = %02h, expected 80", enc_capture[11]);
            test_pass = 0;
        end
        if (enc_capture[12] !== 8'h80) begin // grp7cnt=0|80
            $display("  ERROR: GRP7CNT = %02h, expected 80", enc_capture[12]);
            test_pass = 0;
        end

        // Verify PEND
        if (enc_capture[15] !== 8'hC8) begin
            $display("  ERROR: PEND = %02h, expected C8", enc_capture[15]);
            test_pass = 0;
        end

        // Verify checksum: XOR of header bytes + count bytes
        // $83 ^ $81 ^ $82 ^ $80 ^ $80 ^ $80 ^ $80 = ?
        begin : chk3
            reg [7:0] xor_sum;
            xor_sum = 8'h83 ^ 8'h81 ^ 8'h82 ^ 8'h80 ^ 8'h80 ^ 8'h80 ^ 8'h80;
            $display("  Checksum XOR = %02h", xor_sum);
            $display("  Expected chk_lo = %02h, chk_hi = %02h",
                     {1'b1, xor_sum[6:0]}, {1'b1, 6'b0, xor_sum[7]});
            if (enc_capture[13] !== {1'b1, xor_sum[6:0]}) begin
                $display("  ERROR: CHK_LO = %02h, expected %02h",
                         enc_capture[13], {1'b1, xor_sum[6:0]});
                test_pass = 0;
            end
            if (enc_capture[14] !== {1'b1, 6'b0, xor_sum[7]}) begin
                $display("  ERROR: CHK_HI = %02h, expected %02h",
                         enc_capture[14], {1'b1, 6'b0, xor_sum[7]});
                test_pass = 0;
            end
        end

        if (test_pass) begin
            $display("PASS: Test 3 — Encode STATUS response (no payload)");
            tests_passed = tests_passed + 1;
        end else begin
            $display("FAIL: Test 3 — Encode STATUS response");
            tests_failed = tests_failed + 1;
        end

        // =======================================================
        // TEST 4: Encode READBLOCK response (4-byte payload)
        // =======================================================
        // 4 bytes: $DE $AD $BE $EF
        // oddcnt = 4 % 7 = 4, grp7cnt = 4 / 7 = 0
        //
        // ODDMSB byte:
        //   bit 6 = MSB of byte 0 ($DE -> bit7=1) -> 1
        //   bit 5 = MSB of byte 1 ($AD -> bit7=1) -> 1
        //   bit 4 = MSB of byte 2 ($BE -> bit7=1) -> 1
        //   bit 3 = MSB of byte 3 ($EF -> bit7=1) -> 1
        //   bits 2..0 = 0
        //   ODDMSB = 1_1111_000 -> 0x78 -> wire: $78 | $80 = $F8
        //
        // Odd data bytes (MSB stripped, | $80):
        //   $DE -> $5E | $80 = $DE... no. Strip bit7: $DE & $7F = $5E, then |$80 = $DE.
        //   Wait. $DE = 1101_1110. Strip bit7 -> 0101_1110 = $5E. |$80 -> $DE. Same!
        //   $AD = 1010_1101. Strip bit7 -> $2D. |$80 -> $AD. Same!
        //   $BE = 1011_1110. Strip bit7 -> $3E. |$80 -> $BE. Same!
        //   $EF = 1110_1111. Strip bit7 -> $6F. |$80 -> $EF. Same!
        //
        // When all data bytes have bit7=1, the wire bytes equal the originals
        // (since strip+reapply $80 is identity for bytes with bit7 set).
        //
        // Expected output:
        //   FF FF FF FF FF C3
        //   83 81 82 80 80       (header: dest=3, src=1, type=2, aux=0, stat=0)
        //   84 80                (oddcnt=4|$80, grp7cnt=0|$80)
        //   F8                   (ODDMSB)
        //   DE AD BE EF          (odd data bytes)
        //   xx xx                (checksum)
        //   C8                   (PEND)
        // Total: 5+1+5+2+1+4+2+1 = 21 bytes
        // =======================================================
        do_reset;
        $display("\n--- Test 4: Encode READBLOCK response (4-byte payload) ---");
        total_tests = total_tests + 1;
        test_pass = 1;

        encode_dest        = 7'd3;
        encode_source      = 7'd1;
        encode_ptype       = 7'd2;
        encode_stat        = 7'd0;
        encode_payload_len = 9'd4;
        encode_byte_ready  = 1;

        @(posedge clk);
        encode_start = 1;
        @(posedge clk);
        encode_start = 0;

        run_encoder;

        $display("  Encoded %0d bytes", enc_capture_cnt);

        if (enc_capture_cnt != 21) begin
            $display("  ERROR: expected 21 bytes, got %0d", enc_capture_cnt);
            test_pass = 0;
        end

        // Verify ODDCNT and GRP7CNT
        if (enc_capture_cnt >= 13) begin
            if (enc_capture[11] !== 8'h84) begin
                $display("  ERROR: ODDCNT = %02h, expected 84", enc_capture[11]);
                test_pass = 0;
            end
            if (enc_capture[12] !== 8'h80) begin
                $display("  ERROR: GRP7CNT = %02h, expected 80", enc_capture[12]);
                test_pass = 0;
            end
        end

        // Verify ODDMSB
        if (enc_capture_cnt >= 14) begin
            if (enc_capture[13] !== 8'hF8) begin
                $display("  ERROR: ODDMSB = %02h, expected F8", enc_capture[13]);
                test_pass = 0;
            end
        end

        // Verify odd data bytes
        if (enc_capture_cnt >= 18) begin
            if (enc_capture[14] !== 8'hDE) begin
                $display("  ERROR: ODD[0] = %02h, expected DE", enc_capture[14]);
                test_pass = 0;
            end
            if (enc_capture[15] !== 8'hAD) begin
                $display("  ERROR: ODD[1] = %02h, expected AD", enc_capture[15]);
                test_pass = 0;
            end
            if (enc_capture[16] !== 8'hBE) begin
                $display("  ERROR: ODD[2] = %02h, expected BE", enc_capture[16]);
                test_pass = 0;
            end
            if (enc_capture[17] !== 8'hEF) begin
                $display("  ERROR: ODD[3] = %02h, expected EF", enc_capture[17]);
                test_pass = 0;
            end
        end

        // Verify PEND
        if (enc_capture_cnt >= 21) begin
            if (enc_capture[20] !== 8'hC8) begin
                $display("  ERROR: PEND = %02h, expected C8", enc_capture[20]);
                test_pass = 0;
            end
        end

        if (test_pass) begin
            $display("PASS: Test 4 — Encode READBLOCK response (4-byte payload)");
            tests_passed = tests_passed + 1;
        end else begin
            $display("FAIL: Test 4 — Encode READBLOCK response");
            tests_failed = tests_failed + 1;
        end

        // =======================================================
        // TEST 5: Round-trip (encode then decode)
        // =======================================================
        // Encode a packet with 4-byte payload, then decode the
        // captured wire bytes. Verify decoded fields and payload
        // match the original inputs.
        // =======================================================
        do_reset;
        $display("\n--- Test 5: Round-trip (encode then decode) ---");
        total_tests = total_tests + 1;
        test_pass = 1;

        // First encode
        encode_dest        = 7'd5;
        encode_source      = 7'd2;
        encode_ptype       = 7'd2;
        encode_stat        = 7'd0;
        encode_payload_len = 9'd4;
        encode_byte_ready  = 1;

        // Set up test buffer with known values
        test_buf[0] = 8'h41;  // bit7=0
        test_buf[1] = 8'hFF;  // bit7=1
        test_buf[2] = 8'h00;  // bit7=0
        test_buf[3] = 8'h7F;  // bit7=0

        @(posedge clk);
        encode_start = 1;
        @(posedge clk);
        encode_start = 0;

        run_encoder;

        $display("  Encoded %0d bytes for round-trip", enc_capture_cnt);

        // Now decode: skip sync (5x FF) and PBEGIN ($C3),
        // feed bytes starting after $C3 (index 6 onward),
        // and excluding PEND $C8 at the end.
        saved_enc_cnt = enc_capture_cnt;
        do_reset;
        dec_payload_cnt = 0;

        @(posedge clk);
        decode_start = 1;
        @(posedge clk);
        decode_start = 0;

        // Feed all bytes after C3, excluding final PEND.
        // After presenting each byte, wait one cycle for the DUT
        // to process, then check for payload output.
        for (i = 6; i < saved_enc_cnt - 1; i = i + 1) begin
            @(posedge clk);
            decode_byte_in       = enc_capture[i];
            decode_byte_in_valid = 1;
            @(posedge clk);
            decode_byte_in_valid = 0;
            // decode_payload_valid NBA from this posedge is now resolved
            if (decode_payload_valid) begin
                dec_payload_capture[dec_payload_cnt] = decode_payload_out;
                $display("    decoded payload[%0d] (addr=%0d) = %02h",
                         dec_payload_cnt, decode_payload_addr, decode_payload_out);
                dec_payload_cnt = dec_payload_cnt + 1;
            end
        end

        // Let decode_done fire
        repeat (4) @(posedge clk);

        // Verify header fields
        if (decode_dest !== 7'd5) begin
            $display("  ERROR: round-trip decode_dest = %0d, expected 5", decode_dest);
            test_pass = 0;
        end
        if (decode_source !== 7'd2) begin
            $display("  ERROR: round-trip decode_source = %0d, expected 2", decode_source);
            test_pass = 0;
        end
        if (decode_ptype !== 7'd2) begin
            $display("  ERROR: round-trip decode_ptype = %0d, expected 2", decode_ptype);
            test_pass = 0;
        end
        if (decode_stat !== 7'd0) begin
            $display("  ERROR: round-trip decode_stat = %0d, expected 0", decode_stat);
            test_pass = 0;
        end

        // Verify payload
        if (dec_payload_cnt != 4) begin
            $display("  ERROR: decoded %0d payload bytes, expected 4", dec_payload_cnt);
            test_pass = 0;
        end else begin
            if (dec_payload_capture[0] !== 8'h41) begin
                $display("  ERROR: payload[0] = %02h, expected 41", dec_payload_capture[0]);
                test_pass = 0;
            end
            if (dec_payload_capture[1] !== 8'hFF) begin
                $display("  ERROR: payload[1] = %02h, expected FF", dec_payload_capture[1]);
                test_pass = 0;
            end
            if (dec_payload_capture[2] !== 8'h00) begin
                $display("  ERROR: payload[2] = %02h, expected 00", dec_payload_capture[2]);
                test_pass = 0;
            end
            if (dec_payload_capture[3] !== 8'h7F) begin
                $display("  ERROR: payload[3] = %02h, expected 7F", dec_payload_capture[3]);
                test_pass = 0;
            end
        end

        if (test_pass) begin
            $display("PASS: Test 5 — Round-trip encode/decode");
            tests_passed = tests_passed + 1;
        end else begin
            $display("FAIL: Test 5 — Round-trip encode/decode");
            tests_failed = tests_failed + 1;
        end

        // =======================================================
        // Summary
        // =======================================================
        #500;
        $display("\n===================================================");
        $display("SmartPort Codec Testbench Summary");
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
        #500000;
        $display("FAIL: Testbench timed out");
        $finish;
    end

endmodule
