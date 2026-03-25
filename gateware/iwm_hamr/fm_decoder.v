// fm_decoder.v — FM (frequency modulation) decoder for SmartPort wrdata
// Decodes FM-encoded serial data from the IWM wrdata line into bytes.
// SmartPort FM encoding: 250 kbps, ~4µs per bit cell.
//   "1" bit = edge (toggle) at bit-cell midpoint
//   "0" bit = no transition during bit cell
// Packets: 5x $FF sync, $C3 PBEGIN, payload bytes, $C8 PEND.
//
// Runs on the same fclk (7.16 MHz) as IWM and sp_device — no clock
// domain crossing needed. wrdata and enable are same-domain signals.
// 28 fclk cycles per bit cell gives ~14-cycle midpoint margin, which
// is more than sufficient (PicoPort works with only ~8 samples/cell).

`timescale 1ns / 1ps

module fm_decoder (
    input  wire       clk,             // fclk (7.16 MHz), same domain as IWM
    input  wire       rst_n,
    input  wire       enable,          // same domain (from sp_device on fclk)
    input  wire       wrdata,          // FM serial input (from IWM on fclk)
    output reg  [7:0] data_out = 8'd0,        // decoded byte
    output reg        data_valid = 1'b0,      // pulse when byte is ready
    output reg        sync_detected = 1'b0,   // HIGH when sync+C3 framing achieved
    output reg        packet_end = 1'b0       // pulse when $C8 PEND byte detected
);

    // =========================================================================
    // Timing thresholds (in fclk cycles at 7.16 MHz)
    // =========================================================================
    // Bit cell = 28 fclk (3.91µs × 7.16 MHz). Thresholds at midpoints:
    //   1 cell  =  28  -> threshold at  42 (midpoint of  28.. 56)
    //   2 cells =  56  -> threshold at  70 (midpoint of  56.. 84)
    //   3 cells =  84  -> threshold at  98 (midpoint of  84..112)
    //   4 cells = 112  -> threshold at 126 (midpoint of 112..140)
    //   5 cells = 140  -> threshold at 154 (midpoint of 140..168)
    //   6 cells = 168  -> threshold at 182 (midpoint of 168..196)
    //   7 cells = 196  -> threshold at 210 (midpoint of 196..224)
    //   8 cells = 224  -> anything above 210
    localparam [9:0] THRESH_1 = 10'd42;
    localparam [9:0] THRESH_2 = 10'd70;
    localparam [9:0] THRESH_3 = 10'd98;
    localparam [9:0] THRESH_4 = 10'd126;
    localparam [9:0] THRESH_5 = 10'd154;
    localparam [9:0] THRESH_6 = 10'd182;
    localparam [9:0] THRESH_7 = 10'd210;

    // Timeout: if no edge for this many cycles while synced with a
    // partial byte, flush remaining bits as zeros and emit the byte.
    // 9 bit cells = 252 cycles — past any valid 8-cell gap (224).
    localparam [9:0] FLUSH_TIMEOUT = 10'd252;

    // Minimum consecutive "1" bits to qualify as sync preamble
    localparam [5:0] SYNC_ONES_MIN = 6'd8;

    // =========================================================================
    // Edge detection pipeline (same clock domain — no CDC needed)
    // =========================================================================
    // wrdata: 2-reg pipeline for clean edge detection. Both wrdata and
    // this decoder run on fclk, so no metastability risk. The 2-reg
    // structure adds 1 cycle of detection latency (140ns), which cancels
    // in edge-to-edge interval measurements.
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

    // enable edge detection (same domain)
    reg enable_prev = 1'b0;

    // =========================================================================
    // Internal state
    // =========================================================================
    reg [9:0]  edge_timer = 10'd0;    // clk cycles since last edge
    reg        first_edge = 1'b0;     // set after first edge seen
    reg [7:0]  shift_reg = 8'd0;      // bit assembly shift register
    reg [3:0]  bit_count = 4'd0;      // bits shifted in since last byte boundary
    reg [5:0]  ones_count = 6'd0;     // consecutive "1" bits (for sync detection)
    reg        sync_hunting = 1'b0;   // enough 1s seen, looking for $C3

    // =========================================================================
    // Bit insertion pipeline
    // =========================================================================
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
            edge_timer    <= 10'd0;
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
                edge_timer    <= 10'd0;
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
                edge_timer    <= 10'd0;
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
                        edge_timer <= 10'd0;
                    end else begin
                        // Subsequent edge: queue bits based on elapsed time.
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
                        edge_timer <= 10'd0;
                    end
                end else if (!inserting) begin
                    // No edge, no pending inserts: free-run the timer
                    if (edge_timer < 10'd1023)
                        edge_timer <= edge_timer + 10'd1;
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

                // === Timeout flush ===
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
