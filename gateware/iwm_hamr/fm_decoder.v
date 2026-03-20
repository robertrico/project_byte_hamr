// fm_decoder.v — FM (frequency modulation) decoder for SmartPort wrdata
// Decodes FM-encoded serial data from the IWM wrdata line into bytes.
// SmartPort FM encoding: 250 kbps over 7 MHz clock, ~28 fclk per bit cell.
//   "1" bit = falling edge at bit-cell midpoint
//   "0" bit = no transition during bit cell
// Packets: 5x $FF sync, $C3 PBEGIN, payload bytes, $C8 PEND.
//
// Algorithm: On each falling edge of wrdata, count elapsed fclk cycles
// since the previous edge. Compute how many bit cells elapsed, then
// queue (cells-1) zeros followed by one "1" into a pipeline (one bit
// drained per cycle), avoiding multi-bit shift complexity at byte
// boundaries. Pipeline is 7 bits wide to handle worst-case $81
// (10000001 = 6 zeros + 1 one = 7-cell gap).

`timescale 1ns / 1ps

module fm_decoder (
    input  wire       clk,             // 7.16 MHz (sig_7M / fclk)
    input  wire       rst_n,
    input  wire       enable,          // active when listening for packets
    input  wire       wrdata,          // FM serial input (idle HIGH, falling edge = 1)
    output reg  [7:0] data_out = 8'd0,        // decoded byte
    output reg        data_valid = 1'b0,      // pulse when byte is ready
    output reg        sync_detected = 1'b0,   // HIGH when sync+C3 framing achieved
    output reg        packet_end = 1'b0       // pulse when $C8 PEND byte detected
);

    // =========================================================================
    // Timing thresholds (in fclk cycles)
    // =========================================================================
    // Bit cell = ~28 cycles. Thresholds at midpoints between multiples:
    //   1 cell  =  28  -> threshold at  42 (midpoint of  28.. 56)
    //   2 cells =  56  -> threshold at  70 (midpoint of  56.. 84)
    //   3 cells =  84  -> threshold at  98 (midpoint of  84..112)
    //   4 cells = 112  -> threshold at 126 (midpoint of 112..140)
    //   5 cells = 140  -> threshold at 154 (midpoint of 140..168)
    //   6 cells = 168  -> threshold at 182 (midpoint of 168..196)
    //   7 cells = 196  -> threshold at 210 (midpoint of 196..224)
    //   8 cells = 224  -> anything above 210
    // $80 wire bytes (value 0x00) create 8-cell gaps: 10000000 → 7 zeros.
    localparam [7:0] THRESH_1 = 8'd42;
    localparam [7:0] THRESH_2 = 8'd70;
    localparam [7:0] THRESH_3 = 8'd98;
    localparam [7:0] THRESH_4 = 8'd126;
    localparam [7:0] THRESH_5 = 8'd154;
    localparam [7:0] THRESH_6 = 8'd182;
    localparam [7:0] THRESH_7 = 8'd210;

    // Timeout: if no edge for this many cycles while synced with a
    // partial byte, flush remaining bits as zeros and emit the byte.
    // 9 bit cells = 252 cycles — past any valid 8-cell gap (224).
    localparam [7:0] FLUSH_TIMEOUT = 8'd252;

    // Minimum consecutive "1" bits to qualify as sync preamble
    localparam [5:0] SYNC_ONES_MIN = 6'd8;

    // =========================================================================
    // Edge detection: 2-FF synchronizer + any-edge detect
    // =========================================================================
    // IWM TOGGLES wrdata (not pulses) — each "1" bit creates an edge,
    // alternating between falling and rising. Must detect BOTH edges.
    reg wrdata_r1 = 1'b1, wrdata_r2 = 1'b1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wrdata_r1 <= 1'b1;
            wrdata_r2 <= 1'b1;
        end else begin
            wrdata_r1 <= wrdata;
            wrdata_r2 <= wrdata_r1;
        end
    end

    wire wrdata_edge = wrdata_r2 ^ wrdata_r1;  // any transition = "1" bit

    // =========================================================================
    // Internal state
    // =========================================================================
    reg [7:0]  edge_timer = 8'd0;     // fclk cycles since last falling edge
    reg        first_edge = 1'b0;     // set after first falling edge seen
    reg [7:0]  shift_reg = 8'd0;      // bit assembly shift register
    reg [3:0]  bit_count = 4'd0;      // bits shifted in since last byte boundary
    reg [5:0]  ones_count = 6'd0;     // consecutive "1" bits (for sync detection)
    reg        sync_hunting = 1'b0;   // enough 1s seen, looking for $C3
    reg        enable_prev = 1'b0;    // for detecting enable rising edge

    // =========================================================================
    // Bit insertion pipeline
    // =========================================================================
    // On a falling edge, we compute how many bits to insert (1..8).
    // We load the bits into an 8-bit queue and drain one bit per cycle.
    //
    // insert_bits holds the queued bit values (MSB drained first).
    // insert_count holds how many remain (0..8).
    // While draining, new falling edges are held off (they can't arrive
    // faster than ~28 cycles anyway, and we drain in at most 8).

    reg [7:0] insert_bits = 8'd0;
    reg [3:0] insert_count = 4'd0;

    wire       inserting  = (insert_count != 4'd0);
    wire       insert_bit = insert_bits[7];  // next bit to shift in

    // Byte that would be completed by shifting in insert_bit
    wire [7:0] assembled_byte = {shift_reg[6:0], insert_bit};

    // Byte complete when we're about to shift the 8th bit
    wire       byte_complete = inserting && (bit_count == 4'd7);

    // =========================================================================
    // Main decode logic
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            edge_timer    <= 8'd0;
            first_edge    <= 1'b0;
            shift_reg     <= 8'd0;
            bit_count     <= 4'd0;
            ones_count    <= 6'd0;
            sync_hunting  <= 1'b0;
            sync_detected <= 1'b0;
            data_out      <= 8'd0;
            data_valid    <= 1'b0;
            packet_end    <= 1'b0;
            enable_prev   <= 1'b0;
            insert_bits   <= 8'd0;
            insert_count  <= 4'd0;
        end else begin
            // Default: clear single-cycle pulses
            data_valid <= 1'b0;
            packet_end <= 1'b0;

            enable_prev <= enable;

            if (!enable) begin
                // ---- Disabled: clear all state ----
                edge_timer    <= 8'd0;
                first_edge    <= 1'b0;
                shift_reg     <= 8'd0;
                bit_count     <= 4'd0;
                ones_count    <= 6'd0;
                sync_hunting  <= 1'b0;
                sync_detected <= 1'b0;
                data_out      <= 8'd0;
                insert_bits   <= 8'd0;
                insert_count  <= 4'd0;
            end else if (enable && !enable_prev) begin
                // ---- Rising edge of enable: fresh start ----
                edge_timer    <= 8'd0;
                first_edge    <= 1'b0;
                shift_reg     <= 8'd0;
                bit_count     <= 4'd0;
                ones_count    <= 6'd0;
                sync_hunting  <= 1'b0;
                sync_detected <= 1'b0;
                data_out      <= 8'd0;
                insert_bits   <= 8'd0;
                insert_count  <= 4'd0;
            end else begin
                // ---- Active decode ----

                // === Edge detection and insert-queue loading ===
                if (wrdata_edge && !inserting) begin
                    if (!first_edge) begin
                        // Very first edge: just start timing, no bits
                        first_edge <= 1'b1;
                        edge_timer <= 8'd0;
                    end else begin
                        // Subsequent edge: queue bits based on elapsed time.
                        // Each falling edge = a "1" bit. The number of "0"
                        // bits preceding it = (elapsed_cells - 1).
                        // insert_bits: zeros in MSBs, "1" at the right
                        // position, left-justified in the 7-bit field.
                        if (edge_timer < THRESH_1) begin
                            // 1 cell: "1"
                            insert_bits  <= 8'b10000000;
                            insert_count <= 4'd1;
                        end else if (edge_timer < THRESH_2) begin
                            // 2 cells: "01"
                            insert_bits  <= 8'b01000000;
                            insert_count <= 4'd2;
                        end else if (edge_timer < THRESH_3) begin
                            // 3 cells: "001"
                            insert_bits  <= 8'b00100000;
                            insert_count <= 4'd3;
                        end else if (edge_timer < THRESH_4) begin
                            // 4 cells: "0001"
                            insert_bits  <= 8'b00010000;
                            insert_count <= 4'd4;
                        end else if (edge_timer < THRESH_5) begin
                            // 5 cells: "00001"
                            insert_bits  <= 8'b00001000;
                            insert_count <= 4'd5;
                        end else if (edge_timer < THRESH_6) begin
                            // 6 cells: "000001"
                            insert_bits  <= 8'b00000100;
                            insert_count <= 4'd6;
                        end else if (edge_timer < THRESH_7) begin
                            // 7 cells: "0000001"
                            insert_bits  <= 8'b00000010;
                            insert_count <= 4'd7;
                        end else begin
                            // 8 cells: "00000001" ($80 wire bytes)
                            insert_bits  <= 8'b00000001;
                            insert_count <= 4'd8;
                        end
                        edge_timer <= 8'd0;
                    end
                end else if (!inserting) begin
                    // No edge, no pending inserts: free-run the timer
                    if (edge_timer < 8'd255)
                        edge_timer <= edge_timer + 8'd1;
                end

                // === Drain one bit per cycle from insert queue ===
                if (inserting) begin
                    shift_reg    <= {shift_reg[6:0], insert_bit};
                    insert_bits  <= {insert_bits[6:0], 1'b0};
                    insert_count <= insert_count - 4'd1;

                    // Track consecutive 1s for sync detection
                    if (insert_bit)
                        ones_count <= ones_count + 6'd1;
                    else
                        ones_count <= 6'd0;

                    if (!sync_detected) begin
                        // --- Sync hunting: sliding window on EVERY bit ---
                        // PicoPort uses a sliding accumulator to find $C3
                        // at any bit position, not just byte boundaries.
                        if (insert_bit && ones_count >= SYNC_ONES_MIN - 1)
                            sync_hunting <= 1'b1;
                        else if (!insert_bit && ones_count >= SYNC_ONES_MIN)
                            sync_hunting <= 1'b1;

                        if (sync_hunting && assembled_byte == 8'hC3) begin
                            sync_detected <= 1'b1;
                            bit_count     <= 4'd0;  // establish correct framing
                        end else begin
                            bit_count <= (bit_count == 4'd7) ? 4'd0 : bit_count + 4'd1;
                        end
                    end else begin
                        // --- Synced: count to byte boundaries ---
                        if (byte_complete) begin
                            bit_count  <= 4'd0;
                            data_out   <= assembled_byte;
                            data_valid <= 1'b1;
                            if (assembled_byte == 8'hC8)
                                packet_end <= 1'b1;
                        end else begin
                            bit_count <= bit_count + 4'd1;
                        end
                    end
                end

                // === Timeout flush: emit partial byte padded with zeros ===
                // If synced, have partial bits, not inserting, and the
                // edge timer hits FLUSH_TIMEOUT, pad the remaining bits
                // with zeros and emit the byte. This handles bytes with
                // trailing zeros at end-of-packet (e.g., $C8 = 11001000).
                //
                // shift_reg accumulates bits via {shift_reg[6:0], new_bit}.
                // After bit_count bits, the received bits occupy
                // shift_reg[bit_count-1:0], but the upper bits are stale
                // from the previous byte. We must mask out only the valid
                // bits and left-justify them. Use a case statement since
                // Verilog doesn't allow variable bit-range selection.
                if (!inserting && sync_detected && bit_count != 4'd0
                    && edge_timer == FLUSH_TIMEOUT && first_edge) begin
                    case (bit_count)
                        4'd1: begin
                            data_out <= {shift_reg[0],   7'd0};
                            if ({shift_reg[0],   7'd0} == 8'hC8) packet_end <= 1'b1;
                        end
                        4'd2: begin
                            data_out <= {shift_reg[1:0], 6'd0};
                            if ({shift_reg[1:0], 6'd0} == 8'hC8) packet_end <= 1'b1;
                        end
                        4'd3: begin
                            data_out <= {shift_reg[2:0], 5'd0};
                            if ({shift_reg[2:0], 5'd0} == 8'hC8) packet_end <= 1'b1;
                        end
                        4'd4: begin
                            data_out <= {shift_reg[3:0], 4'd0};
                            if ({shift_reg[3:0], 4'd0} == 8'hC8) packet_end <= 1'b1;
                        end
                        4'd5: begin
                            data_out <= {shift_reg[4:0], 3'd0};
                            if ({shift_reg[4:0], 3'd0} == 8'hC8) packet_end <= 1'b1;
                        end
                        4'd6: begin
                            data_out <= {shift_reg[5:0], 2'd0};
                            if ({shift_reg[5:0], 2'd0} == 8'hC8) packet_end <= 1'b1;
                        end
                        4'd7: begin
                            data_out <= {shift_reg[6:0], 1'd0};
                            if ({shift_reg[6:0], 1'd0} == 8'hC8) packet_end <= 1'b1;
                        end
                        default: begin
                            data_out <= shift_reg;
                        end
                    endcase
                    data_valid <= 1'b1;
                    bit_count  <= 4'd0;
                    ones_count <= 6'd0;
                end
            end
        end
    end

endmodule
