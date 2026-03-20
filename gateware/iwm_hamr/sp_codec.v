// ===================================================================
// SmartPort Packet Codec — Group-of-7 Encode/Decode + Framing
// ===================================================================
// Streaming byte processor for SmartPort packets. Two modes:
//
// DECODE: Accepts wire bytes from FM decoder, parses header fields,
//         decodes group-of-7 payload, outputs raw 8-bit data bytes.
//
// ENCODE: Accepts header fields + payload buffer, produces wire
//         bytes (sync, PBEGIN, header, group-of-7 encoded payload,
//         checksum, PEND) for FM encoder consumption.
//
// SmartPort wire format (all data bytes have bit 7 = 1):
//   [5x $FF sync] [$C3 PBEGIN]
//   [DEST] [SOURCE] [TYPE] [AUX] [STAT]
//   [ODDCNT] [GRP7CNT]
//   [ODDMSB] [ODD1..ODDn]          (if ODDCNT > 0)
//   [GRP7MSB] [D1..D7] x GRP7CNT
//   [CHK_LO] [CHK_HI]
//   [$C8 PEND]
// ===================================================================

`timescale 1ns / 1ps

module sp_codec (
    input  wire       clk,
    input  wire       rst_n,

    // --- Decode mode (wire bytes IN -> parsed fields + payload OUT) ---
    input  wire       decode_start,         // pulse to begin decode
    input  wire [7:0] decode_byte_in,       // next wire byte from FM decoder
    input  wire       decode_byte_in_valid, // pulse when byte available
    // Decoded header fields (valid after header parsed)
    output reg  [6:0] decode_dest = 7'd0,
    output reg  [6:0] decode_source = 7'd0,
    output reg  [6:0] decode_ptype = 7'd0,         // packet type
    output reg  [6:0] decode_aux = 7'd0,
    output reg  [6:0] decode_stat = 7'd0,          // command or status byte
    output reg        decode_header_valid = 1'b0,  // header fields ready
    // Decoded payload data (for DATA packets)
    output reg  [7:0] decode_payload_out = 8'd0,
    output reg        decode_payload_valid = 1'b0, // pulse per decoded payload byte
    output reg  [8:0] decode_payload_addr = 9'd0,  // 0-511 byte index
    output reg        decode_done = 1'b0,
    output reg        decode_error = 1'b0,

    // --- Encode mode (payload IN -> wire bytes OUT) ---
    input  wire       encode_start,
    input  wire [6:0] encode_dest,
    input  wire [6:0] encode_source,
    input  wire [6:0] encode_ptype,
    input  wire [6:0] encode_stat,
    input  wire [9:0] encode_payload_len,   // 0 for no payload, up to 512
    output reg  [7:0] encode_byte_out = 8'd0,
    output reg        encode_byte_valid = 1'b0,    // pulse per wire byte
    input  wire       encode_byte_ready,    // backpressure
    output reg        encode_done = 1'b0,
    // Block buffer read (for encode, to read payload data)
    output reg  [8:0] encode_buf_addr = 9'd0,
    input  wire [7:0] encode_buf_data
);

    // =================================================================
    // Decode state machine
    // =================================================================
    localparam D_IDLE       = 4'd0;
    localparam D_HEADER     = 4'd1;
    localparam D_COUNTS     = 4'd2;
    localparam D_ODD_MSB    = 4'd3;
    localparam D_ODD_DATA   = 4'd4;
    localparam D_GROUP_MSB  = 4'd5;
    localparam D_GROUP_DATA = 4'd6;
    localparam D_CHECKSUM   = 4'd7;
    localparam D_DONE       = 4'd8;

    reg [3:0]  d_state = D_IDLE;
    reg [2:0]  d_hdr_idx = 3'd0;       // 0..4 for 5 header bytes
    reg [6:0]  d_oddcnt = 7'd0;        // number of odd bytes (0-6)
    reg [6:0]  d_grp7cnt = 7'd0;       // number of groups of 7
    reg [6:0]  d_oddmsb = 7'd0;        // MSB bits for odd bytes
    reg [6:0]  d_grpmsb = 7'd0;        // MSB bits for current group
    reg [2:0]  d_byte_idx = 3'd0;      // index within odd section or group (0-6)
    reg [6:0]  d_grp_num = 7'd0;       // current group number (0..grp7cnt-1)
    reg [1:0]  d_chk_idx = 2'd0;       // checksum byte index (0 or 1)
    reg [8:0]  d_payload_ctr = 9'd0;   // running payload byte counter

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d_state           <= D_IDLE;
            d_hdr_idx         <= 3'd0;
            d_oddcnt          <= 7'd0;
            d_grp7cnt         <= 7'd0;
            d_oddmsb          <= 7'd0;
            d_grpmsb          <= 7'd0;
            d_byte_idx        <= 3'd0;
            d_grp_num         <= 7'd0;
            d_chk_idx         <= 2'd0;
            d_payload_ctr     <= 9'd0;
            decode_dest       <= 7'd0;
            decode_source     <= 7'd0;
            decode_ptype      <= 7'd0;
            decode_aux        <= 7'd0;
            decode_stat       <= 7'd0;
            decode_header_valid <= 1'b0;
            decode_payload_out  <= 8'd0;
            decode_payload_valid <= 1'b0;
            decode_payload_addr  <= 9'd0;
            decode_done       <= 1'b0;
            decode_error      <= 1'b0;
        end else begin
            // Clear single-cycle pulses
            decode_payload_valid <= 1'b0;
            decode_done          <= 1'b0;
            decode_error         <= 1'b0;

            case (d_state)
                // -------------------------------------------------
                D_IDLE: begin
                    decode_header_valid <= 1'b0;
                    if (decode_start) begin
                        d_state       <= D_HEADER;
                        d_hdr_idx     <= 3'd0;
                        d_payload_ctr <= 9'd0;
                    end
                end

                // -------------------------------------------------
                // Collect 5 header bytes: DEST, SOURCE, TYPE, AUX, STAT
                // -------------------------------------------------
                D_HEADER: begin
                    if (decode_byte_in_valid) begin
                        case (d_hdr_idx)
                            3'd0: decode_dest   <= decode_byte_in[6:0];
                            3'd1: decode_source <= decode_byte_in[6:0];
                            3'd2: decode_ptype  <= decode_byte_in[6:0];
                            3'd3: decode_aux    <= decode_byte_in[6:0];
                            3'd4: decode_stat   <= decode_byte_in[6:0];
                            default: ;
                        endcase

                        if (d_hdr_idx == 3'd4) begin
                            decode_header_valid <= 1'b1;
                            d_state  <= D_COUNTS;
                            d_hdr_idx <= 3'd0;
                        end else begin
                            d_hdr_idx <= d_hdr_idx + 3'd1;
                        end
                    end
                end

                // -------------------------------------------------
                // Read ODDCNT and GRP7CNT
                // -------------------------------------------------
                D_COUNTS: begin
                    if (decode_byte_in_valid) begin
                        if (d_hdr_idx == 3'd0) begin
                            d_oddcnt  <= decode_byte_in[6:0];
                            d_hdr_idx <= 3'd1;
                        end else begin
                            d_grp7cnt <= decode_byte_in[6:0];
                            d_hdr_idx <= 3'd0;
                            // Decide next state
                            if (decode_byte_in[6:0] == 7'd0 && d_oddcnt == 7'd0) begin
                                // No payload at all
                                d_state   <= D_CHECKSUM;
                                d_chk_idx <= 2'd0;
                            end else if (d_oddcnt != 7'd0) begin
                                d_state <= D_ODD_MSB;
                            end else begin
                                d_state   <= D_GROUP_MSB;
                                d_grp_num <= 7'd0;
                            end
                        end
                    end
                end

                // -------------------------------------------------
                // Read ODDMSB byte
                // -------------------------------------------------
                D_ODD_MSB: begin
                    if (decode_byte_in_valid) begin
                        d_oddmsb   <= decode_byte_in[6:0];
                        d_byte_idx <= 3'd0;
                        d_state    <= D_ODD_DATA;
                    end
                end

                // -------------------------------------------------
                // Read odd data bytes, restore MSBs from ODDMSB
                // MSB mapping: oddmsb bit (6-i) = MSB of odd byte i
                // -------------------------------------------------
                D_ODD_DATA: begin
                    if (decode_byte_in_valid) begin
                        decode_payload_out[6:0] <= decode_byte_in[6:0];
                        case (d_byte_idx)
                            3'd0: decode_payload_out[7] <= d_oddmsb[6];
                            3'd1: decode_payload_out[7] <= d_oddmsb[5];
                            3'd2: decode_payload_out[7] <= d_oddmsb[4];
                            3'd3: decode_payload_out[7] <= d_oddmsb[3];
                            3'd4: decode_payload_out[7] <= d_oddmsb[2];
                            3'd5: decode_payload_out[7] <= d_oddmsb[1];
                            3'd6: decode_payload_out[7] <= d_oddmsb[0];
                            default: decode_payload_out[7] <= 1'b0;
                        endcase
                        decode_payload_valid <= 1'b1;
                        decode_payload_addr <= d_payload_ctr;
                        d_payload_ctr       <= d_payload_ctr + 9'd1;

                        if (d_byte_idx == d_oddcnt[2:0] - 3'd1) begin
                            if (d_grp7cnt != 7'd0) begin
                                d_state   <= D_GROUP_MSB;
                                d_grp_num <= 7'd0;
                            end else begin
                                d_state   <= D_CHECKSUM;
                                d_chk_idx <= 2'd0;
                            end
                        end else begin
                            d_byte_idx <= d_byte_idx + 3'd1;
                        end
                    end
                end

                // -------------------------------------------------
                // Read GRP7MSB for current group
                // -------------------------------------------------
                D_GROUP_MSB: begin
                    if (decode_byte_in_valid) begin
                        d_grpmsb   <= decode_byte_in[6:0];
                        d_byte_idx <= 3'd0;
                        d_state    <= D_GROUP_DATA;
                    end
                end

                // -------------------------------------------------
                // Read 7 data bytes per group, restore MSBs
                // MSB mapping: grpmsb bit (6-i) = MSB of group byte i
                // -------------------------------------------------
                D_GROUP_DATA: begin
                    if (decode_byte_in_valid) begin
                        decode_payload_out[6:0] <= decode_byte_in[6:0];
                        case (d_byte_idx)
                            3'd0: decode_payload_out[7] <= d_grpmsb[6];
                            3'd1: decode_payload_out[7] <= d_grpmsb[5];
                            3'd2: decode_payload_out[7] <= d_grpmsb[4];
                            3'd3: decode_payload_out[7] <= d_grpmsb[3];
                            3'd4: decode_payload_out[7] <= d_grpmsb[2];
                            3'd5: decode_payload_out[7] <= d_grpmsb[1];
                            3'd6: decode_payload_out[7] <= d_grpmsb[0];
                            default: decode_payload_out[7] <= 1'b0;
                        endcase
                        decode_payload_valid <= 1'b1;
                        decode_payload_addr <= d_payload_ctr;
                        d_payload_ctr       <= d_payload_ctr + 9'd1;

                        if (d_byte_idx == 3'd6) begin
                            if (d_grp_num == d_grp7cnt - 7'd1) begin
                                d_state   <= D_CHECKSUM;
                                d_chk_idx <= 2'd0;
                            end else begin
                                d_grp_num <= d_grp_num + 7'd1;
                                d_state   <= D_GROUP_MSB;
                            end
                        end else begin
                            d_byte_idx <= d_byte_idx + 3'd1;
                        end
                    end
                end

                // -------------------------------------------------
                // Read 2 checksum bytes (skip/ignore for now)
                // -------------------------------------------------
                D_CHECKSUM: begin
                    if (decode_byte_in_valid) begin
                        if (d_chk_idx == 2'd1) begin
                            d_state <= D_DONE;
                        end else begin
                            d_chk_idx <= d_chk_idx + 2'd1;
                        end
                    end
                end

                // -------------------------------------------------
                D_DONE: begin
                    decode_done <= 1'b1;
                    d_state     <= D_IDLE;
                end

                default: d_state <= D_IDLE;
            endcase
        end
    end

    // =================================================================
    // Encode state machine
    // =================================================================
    //
    // Buffer read pipeline: encode_buf_data is combinatorial (async read).
    // Set encode_buf_addr, then read encode_buf_data on the SAME cycle
    // in the next state. We use a 2-phase approach for loading chunks:
    //   LOAD state: set addr for byte 0, advance to LOAD2
    //   LOAD2 state: capture buf_data into e_temp[load_cnt], set next addr,
    //                repeat until all bytes captured, then -> MSB state.
    // =================================================================

    localparam E_IDLE       = 4'd0;
    localparam E_SYNC       = 4'd1;
    localparam E_PBEGIN     = 4'd2;
    localparam E_HEADER     = 4'd3;
    localparam E_COUNTS     = 4'd4;
    localparam E_LOAD_SETUP = 4'd5;   // set first buf addr
    localparam E_LOAD_CAP   = 4'd6;   // capture buf_data, set next addr
    localparam E_EMIT_MSB   = 4'd7;   // output MSB byte
    localparam E_EMIT_DATA  = 4'd8;   // output data bytes
    localparam E_CHECKSUM   = 4'd9;
    localparam E_PEND       = 4'd10;
    localparam E_DONE       = 4'd11;

    reg [3:0]  e_state = E_IDLE;
    reg [3:0]  e_sync_cnt = 4'd0;     // 0..11 for preamble + PBEGIN
    reg [2:0]  e_hdr_idx = 3'd0;      // 0..4 for 5 header bytes
    reg [1:0]  e_cnt_idx = 2'd0;      // 0..1 for oddcnt/grp7cnt
    reg [6:0]  e_oddcnt = 7'd0;       // payload_len % 7
    reg [6:0]  e_grp7cnt = 7'd0;      // payload_len / 7
    reg [2:0]  e_byte_idx = 3'd0;     // index within current chunk (0-6)
    reg [6:0]  e_grp_num = 7'd0;      // current group number
    reg [8:0]  e_buf_ptr = 9'd0;      // base address for current chunk in buffer
    reg [1:0]  e_chk_idx = 2'd0;      // checksum output index (0 or 1)

    // Temp storage for current chunk (up to 7 bytes)
    reg [7:0]  e_temp [0:6];
    reg [6:0]  e_msb_bits = 7'd0;     // computed MSB byte for current chunk

    // Latched header fields
    reg [6:0]  e_dest = 7'd0;
    reg [6:0]  e_source = 7'd0;
    reg [6:0]  e_ptype = 7'd0;
    reg [6:0]  e_stat = 7'd0;
    reg [9:0]  e_payload_len = 10'd0;

    // Checksum: XOR of header wire bytes + raw data bytes (per Liron ROM)
    // Encoded as: (chk | $AA), (chk >> 1 | $AA) — even/odd interleaving
    reg [7:0] e_chksum = 8'd0;

    // Pre-computed payload_len mod 7 (for concatenation width safety)
    wire [6:0] e_pl_mod7 = e_payload_len % 7;

    // Load control
    reg [2:0]  e_load_cnt = 3'd0;     // current index being captured
    reg [2:0]  e_load_len = 3'd0;     // total bytes to load for this chunk
    reg        e_is_odd = 1'b0;       // 1 = loading odd chunk, 0 = loading group chunk

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            e_state       <= E_IDLE;
            e_sync_cnt    <= 4'd0;
            e_hdr_idx     <= 3'd0;
            e_cnt_idx     <= 2'd0;
            e_oddcnt      <= 7'd0;
            e_grp7cnt     <= 7'd0;
            e_byte_idx    <= 3'd0;
            e_grp_num     <= 7'd0;
            e_buf_ptr     <= 9'd0;
            e_chk_idx     <= 2'd0;
            e_msb_bits    <= 7'd0;
            e_load_cnt    <= 3'd0;
            e_load_len    <= 3'd0;
            e_is_odd      <= 1'b0;
            e_dest        <= 7'd0;
            e_source      <= 7'd0;
            e_ptype       <= 7'd0;
            e_stat        <= 7'd0;
            e_payload_len <= 10'd0;
            e_chksum      <= 8'd0;
            encode_byte_out   <= 8'd0;
            encode_byte_valid <= 1'b0;
            encode_done       <= 1'b0;
            encode_buf_addr   <= 9'd0;
            e_temp[0] <= 8'd0; e_temp[1] <= 8'd0; e_temp[2] <= 8'd0;
            e_temp[3] <= 8'd0; e_temp[4] <= 8'd0; e_temp[5] <= 8'd0;
            e_temp[6] <= 8'd0;
        end else begin
            // Clear single-cycle pulses
            encode_byte_valid <= 1'b0;
            encode_done       <= 1'b0;

            case (e_state)
                // -------------------------------------------------
                E_IDLE: begin
                    if (encode_start) begin
                        e_dest        <= encode_dest;
                        e_source      <= encode_source;
                        e_ptype       <= encode_ptype;
                        e_stat        <= encode_stat;
                        e_payload_len <= encode_payload_len;
                        e_chksum      <= 8'd0;
                        e_sync_cnt    <= 4'd0;
                        e_buf_ptr     <= 9'd0;
                        e_state       <= E_SYNC;
                    end
                end

                // -------------------------------------------------
                // Output preamble: 6x$FF + $3F $CF $F3 $FC $FF + $C3
                // Matches PicoPort build_packet() — required for
                // IWM clock recovery and byte framing.
                // Uses e_sync_cnt as index (0..11 = 12 bytes total)
                // -------------------------------------------------
                E_SYNC: begin
                    if (encode_byte_ready || !encode_byte_valid) begin
                        case (e_sync_cnt)
                            4'd0:  encode_byte_out <= 8'hFF;
                            4'd1:  encode_byte_out <= 8'hFF;
                            4'd2:  encode_byte_out <= 8'hFF;
                            4'd3:  encode_byte_out <= 8'hFF;
                            4'd4:  encode_byte_out <= 8'hFF;
                            4'd5:  encode_byte_out <= 8'hFF;
                            4'd6:  encode_byte_out <= 8'h3F;
                            4'd7:  encode_byte_out <= 8'hCF;
                            4'd8:  encode_byte_out <= 8'hF3;
                            4'd9:  encode_byte_out <= 8'hFC;
                            4'd10: encode_byte_out <= 8'hFF;
                            4'd11: encode_byte_out <= 8'hC3;  // PBEGIN
                            default: encode_byte_out <= 8'hFF;
                        endcase
                        encode_byte_valid <= 1'b1;
                        if (e_sync_cnt == 4'd11) begin
                            e_hdr_idx <= 3'd0;
                            e_state   <= E_HEADER;
                        end else begin
                            e_sync_cnt <= e_sync_cnt + 4'd1;
                        end
                    end
                end

                // -------------------------------------------------
                // Output 5 header bytes (each | $80), accumulate checksum
                // -------------------------------------------------
                E_HEADER: begin
                    if (encode_byte_ready || !encode_byte_valid) begin
                        case (e_hdr_idx)
                            3'd0: begin
                                encode_byte_out <= {1'b1, e_dest};
                                e_chksum <= e_chksum ^ {1'b1, e_dest};
                            end
                            3'd1: begin
                                encode_byte_out <= {1'b1, e_source};
                                e_chksum <= e_chksum ^ {1'b1, e_source};
                            end
                            3'd2: begin
                                encode_byte_out <= {1'b1, e_ptype};
                                e_chksum <= e_chksum ^ {1'b1, e_ptype};
                            end
                            3'd3: begin
                                encode_byte_out <= 8'h80;
                                e_chksum <= e_chksum ^ 8'h80;
                            end
                            3'd4: begin
                                encode_byte_out <= {1'b1, e_stat};
                                e_chksum <= e_chksum ^ {1'b1, e_stat};
                            end
                            default: ;
                        endcase
                        encode_byte_valid <= 1'b1;

                        if (e_hdr_idx == 3'd4) begin
                            e_state   <= E_COUNTS;
                            e_cnt_idx <= 2'd0;
                        end else begin
                            e_hdr_idx <= e_hdr_idx + 3'd1;
                        end
                    end
                end

                // -------------------------------------------------
                // Output ODDCNT and GRP7CNT, then start loading payload
                // -------------------------------------------------
                E_COUNTS: begin
                    if (encode_byte_ready || !encode_byte_valid) begin
                        if (e_cnt_idx == 2'd0) begin
                            e_oddcnt  <= e_payload_len % 7;
                            e_grp7cnt <= e_payload_len / 7;
                            encode_byte_out   <= {1'b1, e_pl_mod7};
                            encode_byte_valid <= 1'b1;
                            e_chksum  <= e_chksum ^ {1'b1, e_pl_mod7};
                            e_cnt_idx <= 2'd1;
                        end else begin
                            encode_byte_out   <= {1'b1, e_grp7cnt};
                            encode_byte_valid <= 1'b1;
                            e_chksum <= e_chksum ^ {1'b1, e_grp7cnt};

                            if (e_oddcnt != 7'd0) begin
                                // Start loading odd bytes
                                e_state       <= E_LOAD_SETUP;
                                e_load_len    <= e_oddcnt[2:0];
                                e_is_odd      <= 1'b1;
                                e_buf_ptr     <= 9'd0;
                            end else if (e_grp7cnt != 7'd0) begin
                                // No odd bytes, start loading first group
                                e_state       <= E_LOAD_SETUP;
                                e_load_len    <= 3'd7;
                                e_is_odd      <= 1'b0;
                                e_grp_num     <= 7'd0;
                                e_buf_ptr     <= 9'd0;
                            end else begin
                                // No payload
                                e_state   <= E_CHECKSUM;
                                e_chk_idx <= 2'd0;
                            end
                        end
                    end
                end

                // -------------------------------------------------
                // Set up first buffer address for chunk read
                // -------------------------------------------------
                E_LOAD_SETUP: begin
                    encode_buf_addr <= e_buf_ptr;
                    e_load_cnt      <= 3'd0;
                    e_state         <= E_LOAD_CAP;
                end

                // -------------------------------------------------
                // Capture encode_buf_data (available this cycle for
                // the address set last cycle), store in e_temp,
                // advance address. When all bytes loaded, go to MSB.
                // -------------------------------------------------
                E_LOAD_CAP: begin
                    e_temp[e_load_cnt] <= encode_buf_data;

                    if (e_load_cnt == e_load_len - 3'd1) begin
                        // All bytes captured
                        e_state <= E_EMIT_MSB;
                    end else begin
                        encode_buf_addr <= e_buf_ptr + {6'd0, e_load_cnt} + 9'd1;
                        e_load_cnt      <= e_load_cnt + 3'd1;
                    end
                end

                // -------------------------------------------------
                // Compute MSB byte from e_temp and output it
                // MSB mapping: bit (6-i) = bit 7 of e_temp[i]
                // -------------------------------------------------
                E_EMIT_MSB: begin
                    if (encode_byte_ready || !encode_byte_valid) begin
                        // Compute MSB bits from e_temp
                        // e_temp was latched on previous clock edges,
                        // so all values are stable here.
                        e_msb_bits[6] <= (e_load_len > 3'd0) ? e_temp[0][7] : 1'b0;
                        e_msb_bits[5] <= (e_load_len > 3'd1) ? e_temp[1][7] : 1'b0;
                        e_msb_bits[4] <= (e_load_len > 3'd2) ? e_temp[2][7] : 1'b0;
                        e_msb_bits[3] <= (e_load_len > 3'd3) ? e_temp[3][7] : 1'b0;
                        e_msb_bits[2] <= (e_load_len > 3'd4) ? e_temp[4][7] : 1'b0;
                        e_msb_bits[1] <= (e_load_len > 3'd5) ? e_temp[5][7] : 1'b0;
                        e_msb_bits[0] <= (e_load_len > 3'd6) ? e_temp[6][7] : 1'b0;

                        // Output MSB byte: compute combinationally for
                        // this cycle's output (e_temp is already stable)
                        encode_byte_out <= {
                            1'b1,
                            (e_load_len > 3'd0) ? e_temp[0][7] : 1'b0,
                            (e_load_len > 3'd1) ? e_temp[1][7] : 1'b0,
                            (e_load_len > 3'd2) ? e_temp[2][7] : 1'b0,
                            (e_load_len > 3'd3) ? e_temp[3][7] : 1'b0,
                            (e_load_len > 3'd4) ? e_temp[4][7] : 1'b0,
                            (e_load_len > 3'd5) ? e_temp[5][7] : 1'b0,
                            (e_load_len > 3'd6) ? e_temp[6][7] : 1'b0
                        };
                        encode_byte_valid <= 1'b1;

                        // MSB byte NOT included in checksum (per Liron ROM)

                        e_byte_idx <= 3'd0;
                        e_state    <= E_EMIT_DATA;
                    end
                end

                // -------------------------------------------------
                // Output data bytes (strip MSB, force bit 7 = 1)
                // -------------------------------------------------
                E_EMIT_DATA: begin
                    if (encode_byte_ready || !encode_byte_valid) begin
                        encode_byte_out   <= {1'b1, e_temp[e_byte_idx][6:0]};
                        encode_byte_valid <= 1'b1;
                        // XOR raw data byte (not wire byte) per Liron ROM
                        e_chksum <= e_chksum ^ e_temp[e_byte_idx];

                        if (e_byte_idx == e_load_len - 3'd1) begin
                            // Chunk done — advance buffer pointer
                            e_buf_ptr <= e_buf_ptr + {6'd0, e_load_len};

                            if (e_is_odd) begin
                                // Odd section done
                                if (e_grp7cnt != 7'd0) begin
                                    // Start loading first group
                                    e_state    <= E_LOAD_SETUP;
                                    e_load_len <= 3'd7;
                                    e_is_odd   <= 1'b0;
                                    e_grp_num  <= 7'd0;
                                    e_buf_ptr  <= e_buf_ptr + {6'd0, e_load_len};
                                end else begin
                                    e_state   <= E_CHECKSUM;
                                    e_chk_idx <= 2'd0;
                                end
                            end else begin
                                // Group done
                                if (e_grp_num == e_grp7cnt - 7'd1) begin
                                    // All groups done
                                    e_state   <= E_CHECKSUM;
                                    e_chk_idx <= 2'd0;
                                end else begin
                                    // Next group
                                    e_grp_num  <= e_grp_num + 7'd1;
                                    e_state    <= E_LOAD_SETUP;
                                    e_load_len <= 3'd7;
                                    e_buf_ptr  <= e_buf_ptr + {6'd0, e_load_len};
                                end
                            end
                        end else begin
                            e_byte_idx <= e_byte_idx + 3'd1;
                        end
                    end
                end

                // -------------------------------------------------
                // Output 2 checksum bytes — even/odd $AA interleave
                // chk_even = checksum | $AA
                // chk_odd  = (checksum >> 1) | $AA
                // -------------------------------------------------
                E_CHECKSUM: begin
                    if (encode_byte_ready || !encode_byte_valid) begin
                        if (e_chk_idx == 2'd0) begin
                            encode_byte_out   <= e_chksum | 8'hAA;
                            encode_byte_valid <= 1'b1;
                            e_chk_idx <= 2'd1;
                        end else begin
                            encode_byte_out   <= {1'b0, e_chksum[7:1]} | 8'hAA;
                            encode_byte_valid <= 1'b1;
                            e_state <= E_PEND;
                        end
                    end
                end

                // -------------------------------------------------
                // Output $C8 PEND
                // -------------------------------------------------
                E_PEND: begin
                    if (encode_byte_ready || !encode_byte_valid) begin
                        encode_byte_out   <= 8'hC8;
                        encode_byte_valid <= 1'b1;
                        e_state           <= E_DONE;
                    end
                end

                // -------------------------------------------------
                E_DONE: begin
                    encode_done <= 1'b1;
                    e_state     <= E_IDLE;
                end

                default: e_state <= E_IDLE;
            endcase
        end
    end

endmodule
