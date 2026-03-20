// ===================================================================
// SmartPort Device Engine — Protocol State Machine
// ===================================================================
// Instantiates fm_decoder, fm_encoder, and sp_codec to implement
// the SmartPort device protocol. Responds to IWM signals (wrdata,
// phase, _wrreq, _enbl1, _enbl2) and generates rddata and sense.
//
// Handles INIT, STATUS, READBLOCK, and WRITEBLOCK commands.
// Uses a dual-port block buffer for payload data and an SDRAM
// handshake interface for block-level storage access.
//
// Protocol reference: PicoPort firmware (sp_cmd.c) and Liron ROM.
//
// sp_codec encode_payload_len is [9:0] (supports 512-byte READBLOCK payloads).
// SmartPort DATA packets carry 512 bytes, which does not fit.
// This module passes the raw value 512 which truncates to 0 in
// the 9-bit port. The sp_codec must be widened to [9:0] for
// full 512-byte block support. This is tracked as a known issue.
// ===================================================================

`timescale 1ns / 1ps

module sp_device #(
    parameter BLOCK_COUNT = 16'd280     // Number of 512-byte blocks
)(
    input  wire        fclk,            // 7.16 MHz (sig_7M)
    input  wire        rst_n,
    input  wire        boot_done,       // don't respond until SDRAM loaded

    // IWM drive interface (internal connection, replaces GPIO)
    input  wire        wrdata,          // FM data FROM host (IWM write serializer)
    output wire        rddata,          // FM data TO host (IWM read shift register)
    input  wire [3:0]  phase,           // SmartPort command/address bits
    input  wire        _wrreq,          // write request from IWM (active LOW)
    input  wire        _enbl1,          // drive 1 enable (active LOW)
    input  wire        _enbl2,          // drive 2 enable (active LOW)
    output reg         sense = 1'b0,           // device present / ACK signal

    // Block buffer interface (dual-port BRAM, 7MHz side)
    output wire [8:0]  buf_addr,        // 0-511 byte address
    input  wire [7:0]  buf_rd_data,     // read data from buffer
    output wire [7:0]  buf_wr_data,     // write data to buffer
    output wire        buf_wr_en,       // write enable

    // SDRAM block request (crosses to 25MHz domain via handshake)
    output reg         block_read_req = 1'b0,  // request to read a block from SDRAM->buffer
    output reg         block_write_req = 1'b0, // request to write buffer->SDRAM
    output reg  [15:0] block_num = 16'd0,       // block number for SDRAM access
    input  wire        block_ready,     // SDRAM transfer complete

    // Debug
    output wire [4:0]  debug_state,     // expose state for top-level debug
    output wire        debug_sync       // FM decoder sync_detected
);

    // =================================================================
    // SmartPort command codes (decoded stat field values)
    // Match sp_proto.h: STATUS=0, READBLOCK=1, WRITEBLOCK=2, INIT=5
    // =================================================================
    localparam [6:0] CMD_STATUS     = 7'd0;
    localparam [6:0] CMD_READBLOCK  = 7'd1;
    localparam [6:0] CMD_WRITEBLOCK = 7'd2;
    localparam [6:0] CMD_INIT       = 7'd5;

    // SmartPort packet types (decoded ptype field, low 7 bits)
    // Wire: CMD=$80(ptype=0), STATUS=$81(ptype=1), DATA=$82(ptype=2)
    localparam [6:0] PTYPE_STATUS   = 7'd1;
    localparam [6:0] PTYPE_DATA     = 7'd2;

    // Our SmartPort unit ID — defaults to 1, assigned by INIT
    reg [6:0] unit_id = 7'd1;

    // =================================================================
    // Protocol state machine states
    // =================================================================
    localparam [4:0] ST_IDLE          = 5'd0;
    localparam [4:0] ST_RX_ENABLE     = 5'd1;
    localparam [4:0] ST_RX_WAIT_SYNC  = 5'd2;
    localparam [4:0] ST_RX_DECODE     = 5'd3;
    localparam [4:0] ST_ACK           = 5'd4;
    localparam [4:0] ST_ACK_DONE      = 5'd5;
    localparam [4:0] ST_STATUS_FILL   = 5'd6;   // write status bytes to buffer
    localparam [4:0] ST_WAIT_BLOCK    = 5'd7;
    localparam [4:0] ST_TX_WAIT_REQ   = 5'd8;
    localparam [4:0] ST_TX_DELAY      = 5'd9;   // 15us Q6 flush delay
    localparam [4:0] ST_TX_START      = 5'd10;
    localparam [4:0] ST_TX_FILL      = 5'd11;  // codec fills tx_pkt_buf
    localparam [4:0] ST_TX_SEND      = 5'd20;  // FM encoder streams from buf
    localparam [4:0] ST_TX_DONE_ACK   = 5'd12;
    localparam [4:0] ST_TX_DONE_WAIT  = 5'd13;
    localparam [4:0] ST_TX_DONE       = 5'd14;
    localparam [4:0] ST_WB_WAIT_WR    = 5'd15;  // WRITEBLOCK: wait for write mode
    localparam [4:0] ST_WB_RX_ENABLE  = 5'd16;
    localparam [4:0] ST_WB_RX_SYNC    = 5'd17;
    localparam [4:0] ST_WB_RX_DECODE  = 5'd18;
    localparam [4:0] ST_WB_STORE      = 5'd19;  // write buffer to SDRAM

    reg [4:0]  state = ST_IDLE;
    assign debug_state = state;
    reg [15:0] delay_counter = 16'd0;       // general-purpose counter

    // REQ is phase[0]
    wire req = phase[0];

    // Detect wrdata activity (falling edge = IWM serializing data)
    reg wrdata_prev = 1'b1;
    wire wrdata_fall = wrdata_prev & ~wrdata;

    always @(posedge fclk or negedge rst_n) begin
        if (!rst_n)
            wrdata_prev <= 1'b1;
        else
            wrdata_prev <= wrdata;
    end

    // =================================================================
    // Submodule wires
    // =================================================================

    // FM decoder
    reg        rx_enable = 1'b0;
    wire [7:0] rx_byte;
    wire       rx_byte_valid;
    wire       rx_sync;
    assign debug_sync = rx_sync;
    wire       rx_packet_end;

    // FM encoder (buffer-driven)
    reg        tx_start = 1'b0;
    reg  [9:0] tx_byte_count = 10'd0;
    wire       tx_busy;
    wire       tx_enc_done;
    wire [9:0] tx_enc_buf_addr;
    wire [7:0] tx_enc_buf_data;

    // TX packet buffer — holds complete response packet
    // Written by codec (one byte/clock), read by FM encoder (one byte/224 clocks)
    reg  [7:0] tx_pkt_buf [0:1023];
    reg  [9:0] tx_pkt_wr_addr = 10'd0;
    reg  [7:0] tx_pkt_rd_data = 8'd0;

    // Buffer write (from codec during TX_FILL)
    // Buffer read (from FM encoder during TX_SEND) — registered for BRAM inference
    always @(posedge fclk) begin
        tx_pkt_rd_data <= tx_pkt_buf[tx_enc_buf_addr];
    end
    assign tx_enc_buf_data = tx_pkt_rd_data;

    // SP codec — decode
    reg        codec_decode_start = 1'b0;
    wire [6:0] cmd_dest;
    wire [6:0] cmd_source;
    wire [6:0] cmd_ptype;
    wire [6:0] cmd_aux;
    wire [6:0] cmd_stat;
    wire       cmd_header_valid;
    wire [7:0] cmd_payload_out;
    wire       cmd_payload_valid;
    wire [8:0] cmd_payload_addr;
    wire       cmd_decode_done;
    wire       cmd_decode_error;

    // SP codec — encode
    reg        codec_encode_start = 1'b0;
    reg  [6:0] enc_dest = 7'd0;
    reg  [6:0] enc_source = 7'd0;
    reg  [6:0] enc_ptype = 7'd0;
    reg  [6:0] enc_stat = 7'd0;
    reg  [9:0] enc_payload_len = 9'd0;
    wire [7:0] tx_pkt_byte;
    wire       tx_pkt_byte_valid;
    wire       tx_pkt_byte_ready;
    wire       tx_pkt_done;
    wire [8:0] encode_buf_addr_w;

    // =================================================================
    // TX packet buffer write logic
    // =================================================================
    // Codec pushes bytes via tx_pkt_byte_valid. We write them into
    // tx_pkt_buf[] sequentially. Codec always ready (no backpressure
    // since buffer is large and codec is fast).
    // =================================================================
    assign tx_pkt_byte_ready = 1'b1;  // always accept codec bytes

    always @(posedge fclk or negedge rst_n) begin
        if (!rst_n) begin
            tx_pkt_wr_addr <= 10'd0;
        end else begin
            if (state == ST_TX_START) begin
                tx_pkt_wr_addr <= 10'd0;
            end else if (tx_pkt_byte_valid) begin
                tx_pkt_buf[tx_pkt_wr_addr] <= tx_pkt_byte;
                tx_pkt_wr_addr <= tx_pkt_wr_addr + 10'd1;
            end
        end
    end

    // =================================================================
    // Submodule instantiation
    // =================================================================
    fm_decoder u_fm_decoder (
        .clk            (fclk),
        .rst_n          (rst_n),
        .enable         (rx_enable),
        .wrdata         (wrdata),
        .data_out       (rx_byte),
        .data_valid     (rx_byte_valid),
        .sync_detected  (rx_sync),
        .packet_end     (rx_packet_end)
    );

    fm_encoder u_fm_encoder (
        .clk            (fclk),
        .rst_n          (rst_n),
        .start          (tx_start),
        .byte_count     (tx_byte_count),
        .rddata         (rddata),
        .busy           (tx_busy),
        .done           (tx_enc_done),
        .buf_addr       (tx_enc_buf_addr),
        .buf_data       (tx_enc_buf_data)
    );

    sp_codec u_sp_codec (
        .clk                    (fclk),
        .rst_n                  (rst_n),
        // Decode
        .decode_start           (codec_decode_start),
        .decode_byte_in         (rx_byte),
        .decode_byte_in_valid   (rx_byte_valid),
        .decode_dest            (cmd_dest),
        .decode_source          (cmd_source),
        .decode_ptype           (cmd_ptype),
        .decode_aux             (cmd_aux),
        .decode_stat            (cmd_stat),
        .decode_header_valid    (cmd_header_valid),
        .decode_payload_out     (cmd_payload_out),
        .decode_payload_valid   (cmd_payload_valid),
        .decode_payload_addr    (cmd_payload_addr),
        .decode_done            (cmd_decode_done),
        .decode_error           (cmd_decode_error),
        // Encode
        .encode_start           (codec_encode_start),
        .encode_dest            (enc_dest),
        .encode_source          (enc_source),
        .encode_ptype           (enc_ptype),
        .encode_stat            (enc_stat),
        .encode_payload_len     (enc_payload_len),
        .encode_byte_out        (tx_pkt_byte),
        .encode_byte_valid      (tx_pkt_byte_valid),
        .encode_byte_ready      (tx_pkt_byte_ready),
        .encode_done            (tx_pkt_done),
        .encode_buf_addr        (encode_buf_addr_w),
        .encode_buf_data        (buf_rd_data)
    );

    // =================================================================
    // Block buffer address/data mux
    // =================================================================
    // buf_addr_sel: 0 = RX path (decode writes, status fill writes)
    //              1 = TX path (encode reads from buffer)
    // =================================================================
    reg        buf_addr_sel = 1'b0;

    // Status fill write signals
    reg [8:0]  status_fill_addr = 9'd0;
    reg [7:0]  status_fill_data = 8'd0;
    reg        status_fill_wr = 1'b0;

    assign buf_addr    = buf_addr_sel   ? encode_buf_addr_w  :
                         status_fill_wr ? status_fill_addr   :
                         cmd_payload_addr;

    assign buf_wr_data = status_fill_wr ? status_fill_data : cmd_payload_out;

    assign buf_wr_en   = status_fill_wr                   ? 1'b1 :
                         (!buf_addr_sel && cmd_payload_valid) ? 1'b1 :
                         1'b0;

    // =================================================================
    // Command field latches
    // =================================================================
    reg [6:0]  latched_cmd = 7'd0;
    reg [6:0]  latched_dest = 7'd0;        // command destination (our unit)
    reg [6:0]  latched_source = 7'd0;      // host source address
    reg        header_seen = 1'b0;
    reg        cmd_for_us = 1'b0;

    // Block number from command payload (3 odd bytes)
    reg [7:0]  param_block_lo = 8'd0;
    reg [7:0]  param_block_mid = 8'd0;
    reg [7:0]  param_block_hi = 8'd0;

    always @(posedge fclk or negedge rst_n) begin
        if (!rst_n) begin
            param_block_lo  <= 8'd0;
            param_block_mid <= 8'd0;
            param_block_hi  <= 8'd0;
        end else begin
            if ((state == ST_RX_DECODE) && cmd_payload_valid) begin
                case (cmd_payload_addr)
                    9'd0: param_block_lo  <= cmd_payload_out;
                    9'd1: param_block_mid <= cmd_payload_out;
                    9'd2: param_block_hi  <= cmd_payload_out;
                    default: ;
                endcase
            end
            if (state == ST_IDLE) begin
                param_block_lo  <= 8'd0;
                param_block_mid <= 8'd0;
                param_block_hi  <= 8'd0;
            end
        end
    end

    // =================================================================
    // Status fill sequencer
    // =================================================================
    reg [2:0]  status_fill_idx = 3'd0;     // 0..3 then done

    // =================================================================
    // Main state machine
    // =================================================================
    always @(posedge fclk or negedge rst_n) begin
        if (!rst_n) begin
            state               <= ST_IDLE;
            sense               <= 1'b0;
            rx_enable           <= 1'b0;
            tx_start            <= 1'b0;
            codec_decode_start  <= 1'b0;
            codec_encode_start  <= 1'b0;
            delay_counter       <= 16'd0;
            unit_id             <= 7'd1;
            latched_cmd         <= 7'd0;
            latched_dest        <= 7'd0;
            latched_source      <= 7'd0;
            header_seen         <= 1'b0;
            cmd_for_us          <= 1'b0;
            block_read_req      <= 1'b0;
            block_write_req     <= 1'b0;
            block_num           <= 16'd0;
            enc_dest            <= 7'd0;
            enc_source          <= 7'd0;
            enc_ptype           <= 7'd0;
            enc_stat            <= 7'd0;
            enc_payload_len     <= 9'd0;
            buf_addr_sel        <= 1'b0;
            status_fill_addr    <= 9'd0;
            status_fill_data    <= 8'd0;
            status_fill_wr      <= 1'b0;
            status_fill_idx     <= 3'd0;
            // tx_encoder_started removed (buffer-driven TX)
        end else begin
            // Default: clear single-cycle pulses
            tx_start            <= 1'b0;
            codec_decode_start  <= 1'b0;
            codec_encode_start  <= 1'b0;
            block_read_req      <= 1'b0;
            block_write_req     <= 1'b0;
            status_fill_wr      <= 1'b0;

            case (state)
                // -------------------------------------------------
                // IDLE: sense HIGH = device present
                // Wait for host to enter write mode
                // -------------------------------------------------
                ST_IDLE: begin
                    sense              <= 1'b1;   // Always present
                    rx_enable          <= 1'b0;
                    buf_addr_sel       <= 1'b0;
                    header_seen        <= 1'b0;
                    cmd_for_us         <= 1'b0;
                    tx_encoder_started <= 1'b0;

                    if (wrdata_fall) begin
                        state <= ST_RX_ENABLE;
                    end
                end

                // -------------------------------------------------
                // RX_ENABLE: activate FM decoder + codec decode
                // -------------------------------------------------
                ST_RX_ENABLE: begin
                    rx_enable          <= 1'b1;
                    codec_decode_start <= 1'b1;
                    header_seen        <= 1'b0;
                    delay_counter      <= 16'd0;
                    state              <= ST_RX_WAIT_SYNC;
                end

                // -------------------------------------------------
                // RX_WAIT_SYNC: wait for sync bytes + $C3 detection
                // Timeout ~1024 cycles (~143us)
                // -------------------------------------------------
                ST_RX_WAIT_SYNC: begin
                    delay_counter <= delay_counter + 16'd1;
                    if (rx_sync) begin
                        state <= ST_RX_DECODE;
                    end else if (delay_counter >= 16'd10000) begin
                        rx_enable <= 1'b0;
                        state     <= ST_IDLE;
                    end
                end

                // -------------------------------------------------
                // RX_DECODE: codec is parsing command packet
                // Capture header, wait for decode_done
                // -------------------------------------------------
                ST_RX_DECODE: begin
                    if (cmd_header_valid && !header_seen) begin
                        header_seen    <= 1'b1;
                        latched_cmd    <= cmd_stat;
                        latched_dest   <= cmd_dest;
                        latched_source <= cmd_source;
                        // Accept command if: INIT (always accept),
                        // or dest matches our unit_id
                        if (cmd_stat == CMD_INIT)
                            cmd_for_us <= 1'b1;
                        else if (cmd_dest == unit_id)
                            cmd_for_us <= 1'b1;
                        else
                            cmd_for_us <= 1'b0;
                    end

                    if (cmd_decode_done || rx_packet_end) begin
                        rx_enable <= 1'b0;
                        if (cmd_for_us) begin
                            state <= ST_ACK;
                            delay_counter <= 16'd0;
                        end else begin
                            state <= ST_IDLE;
                        end
                    end

                    if (cmd_decode_error) begin
                        rx_enable <= 1'b0;
                        state     <= ST_IDLE;
                    end
                end

                // -------------------------------------------------
                // ACK: drive sense LOW, wait for REQ fall
                // Critical: must happen within 108us of _wrreq rise
                // (~772 fclk cycles at 7.16 MHz)
                // -------------------------------------------------
                ST_ACK: begin
                    sense <= 1'b0;
                    delay_counter <= delay_counter + 16'd1;

                    if (!req) begin
                        state <= ST_ACK_DONE;
                    end else if (delay_counter >= 16'd50000) begin
                        // Timeout ~7ms — generous for Liron ROM response
                        sense <= 1'b1;
                        state <= ST_IDLE;
                    end
                end

                // -------------------------------------------------
                // ACK_DONE: sense HIGH, dispatch by command type
                // -------------------------------------------------
                ST_ACK_DONE: begin
                    sense <= 1'b1;

                    case (latched_cmd)
                        CMD_INIT: begin
                            unit_id     <= latched_dest;
                            enc_dest    <= latched_source;
                            enc_source  <= latched_dest;
                            enc_ptype   <= PTYPE_STATUS;
                            // INIT response: stat = 0x7F (0xFF wire & 0x7F)
                            enc_stat    <= 7'h7F;
                            enc_payload_len <= 10'd0;
                            buf_addr_sel <= 1'b1;
                            state        <= ST_TX_WAIT_REQ;
                        end

                        CMD_STATUS: begin
                            // Fill block buffer with 4-byte status data
                            buf_addr_sel    <= 1'b0;
                            status_fill_idx <= 3'd0;
                            state           <= ST_STATUS_FILL;
                            // Prepare encode header
                            enc_dest        <= latched_source;
                            enc_source      <= unit_id;
                            enc_ptype       <= PTYPE_STATUS;
                            enc_stat        <= 7'd0;
                            enc_payload_len <= 10'd4;
                        end

                        CMD_READBLOCK: begin
                            block_num      <= {param_block_mid, param_block_lo};
                            block_read_req <= 1'b1;
                            enc_dest        <= latched_source;
                            enc_source      <= unit_id;
                            enc_ptype       <= PTYPE_DATA;
                            enc_stat        <= 7'd0;
                            enc_payload_len <= 10'd512;
                            buf_addr_sel    <= 1'b1;
                            state           <= ST_WAIT_BLOCK;
                        end

                        CMD_WRITEBLOCK: begin
                            buf_addr_sel  <= 1'b0;
                            state         <= ST_WB_WAIT_WR;
                            delay_counter <= 16'd0;
                        end

                        default: begin
                            state <= ST_IDLE;
                        end
                    endcase
                end

                // -------------------------------------------------
                // STATUS_FILL: write 4 status bytes to buffer
                // Byte 0: general status (0x00)
                // Byte 1: block count low
                // Byte 2: block count high
                // Byte 3: 0x00 reserved
                // -------------------------------------------------
                ST_STATUS_FILL: begin
                    status_fill_wr <= 1'b1;
                    case (status_fill_idx)
                        3'd0: begin
                            status_fill_addr <= 9'd0;
                            status_fill_data <= 8'h00;
                            status_fill_idx  <= 3'd1;
                        end
                        3'd1: begin
                            status_fill_addr <= 9'd1;
                            status_fill_data <= BLOCK_COUNT[7:0];
                            status_fill_idx  <= 3'd2;
                        end
                        3'd2: begin
                            status_fill_addr <= 9'd2;
                            status_fill_data <= BLOCK_COUNT[15:8];
                            status_fill_idx  <= 3'd3;
                        end
                        3'd3: begin
                            status_fill_addr <= 9'd3;
                            status_fill_data <= 8'h00;
                            status_fill_idx  <= 3'd4;
                        end
                        default: begin
                            status_fill_wr <= 1'b0;
                            buf_addr_sel   <= 1'b1;
                            state          <= ST_TX_WAIT_REQ;
                        end
                    endcase
                end

                // -------------------------------------------------
                // WAIT_BLOCK: wait for SDRAM transfer complete
                // -------------------------------------------------
                ST_WAIT_BLOCK: begin
                    if (block_ready) begin
                        state <= ST_TX_WAIT_REQ;
                    end
                end

                // -------------------------------------------------
                // TX_WAIT_REQ: wait for REQ (phase[0]) to rise
                // PicoPort: req_wait_rise(30000) → ~4ms timeout
                // -------------------------------------------------
                ST_TX_WAIT_REQ: begin
                    delay_counter <= delay_counter + 16'd1;
                    if (req) begin
                        delay_counter <= 16'd0;
                        state <= ST_TX_DELAY;
                    end else if (delay_counter >= 16'd30000) begin
                        sense <= 1'b1;
                        rx_enable <= 1'b0;
                        state <= ST_IDLE;
                    end
                end

                // -------------------------------------------------
                // TX_DELAY: 15us delay (~107 cycles at 7.16 MHz)
                // Q6 flush race protection: the Liron ROM reads
                // Q6 OFF ~6us after asserting REQ, flushing the
                // IWM read shift register. We must not send edges
                // until this flush completes on an idle line.
                // -------------------------------------------------
                ST_TX_DELAY: begin
                    delay_counter <= delay_counter + 16'd1;
                    if (delay_counter >= 16'd107) begin
                        state <= ST_TX_START;
                    end
                end

                // -------------------------------------------------
                // TX_START: kick off codec encoder into tx_pkt_buf
                // -------------------------------------------------
                ST_TX_START: begin
                    codec_encode_start <= 1'b1;
                    state              <= ST_TX_FILL;
                end

                // -------------------------------------------------
                // TX_FILL: wait for codec to finish writing packet
                // bytes into tx_pkt_buf. tx_pkt_wr_addr tracks count.
                // -------------------------------------------------
                ST_TX_FILL: begin
                    delay_counter <= delay_counter + 16'd1;
                    if (tx_pkt_done) begin
                        // Codec finished — tx_pkt_wr_addr = byte count
                        tx_byte_count <= tx_pkt_wr_addr;
                        state         <= ST_TX_SEND;
                    end else if (delay_counter >= 16'd5000) begin
                        // ~700us timeout
                        sense <= 1'b1;
                        rx_enable <= 1'b0;
                        state <= ST_IDLE;
                    end
                end

                // -------------------------------------------------
                // TX_SEND: start FM encoder to stream from buffer.
                // Continuous transmission, zero inter-byte gaps.
                // -------------------------------------------------
                ST_TX_SEND: begin
                    tx_start <= 1'b1;
                    state    <= 5'd21;  // TX_SEND_WAIT
                end

                // TX_SEND_WAIT: wait for FM encoder to finish
                5'd21: begin
                    delay_counter <= delay_counter + 16'd1;
                    if (tx_enc_done) begin
                        state <= ST_TX_DONE_ACK;
                    end else if (delay_counter >= 16'd50000) begin
                        // ~7ms timeout (full 512-byte packet)
                        sense <= 1'b1;
                        rx_enable <= 1'b0;
                        state <= ST_IDLE;
                    end
                end

                // -------------------------------------------------
                // TX_DONE_ACK: sense LOW = "response complete"
                // -------------------------------------------------
                ST_TX_DONE_ACK: begin
                    sense         <= 1'b0;
                    delay_counter <= 16'd0;
                    state         <= ST_TX_DONE_WAIT;
                end

                // -------------------------------------------------
                // TX_DONE_WAIT: wait for REQ to fall
                // -------------------------------------------------
                ST_TX_DONE_WAIT: begin
                    delay_counter <= delay_counter + 16'd1;
                    if (!req) begin
                        state <= ST_TX_DONE;
                    end else if (delay_counter >= 16'd1500) begin
                        sense <= 1'b1;
                        state <= ST_IDLE;
                    end
                end

                // -------------------------------------------------
                // TX_DONE: sense HIGH, return to idle
                // -------------------------------------------------
                ST_TX_DONE: begin
                    sense <= 1'b1;
                    state <= ST_IDLE;
                end

                // =====================================================
                // WRITEBLOCK states
                // =====================================================

                // -------------------------------------------------
                // WB_WAIT_WR: wait for host to re-enter write mode
                // to send DATA packet
                // -------------------------------------------------
                ST_WB_WAIT_WR: begin
                    delay_counter <= delay_counter + 16'd1;
                    if (wrdata_fall) begin
                        state <= ST_WB_RX_ENABLE;
                    end else if (delay_counter >= 16'd10000) begin
                        state <= ST_IDLE;
                    end
                end

                // -------------------------------------------------
                // WB_RX_ENABLE: enable FM decoder for DATA packet
                // -------------------------------------------------
                ST_WB_RX_ENABLE: begin
                    rx_enable          <= 1'b1;
                    codec_decode_start <= 1'b1;
                    delay_counter      <= 16'd0;
                    state              <= ST_WB_RX_SYNC;
                end

                // -------------------------------------------------
                // WB_RX_SYNC: wait for sync on DATA packet
                // -------------------------------------------------
                ST_WB_RX_SYNC: begin
                    delay_counter <= delay_counter + 16'd1;
                    if (rx_sync) begin
                        state <= ST_WB_RX_DECODE;
                    end else if (delay_counter >= 16'd10000) begin
                        rx_enable <= 1'b0;
                        state     <= ST_IDLE;
                    end
                end

                // -------------------------------------------------
                // WB_RX_DECODE: codec decodes DATA packet.
                // Payload bytes are written to buffer via
                // cmd_payload_valid / cmd_payload_addr / cmd_payload_out.
                // -------------------------------------------------
                ST_WB_RX_DECODE: begin
                    if (cmd_decode_done || rx_packet_end) begin
                        rx_enable <= 1'b0;
                        state     <= ST_WB_STORE;
                    end
                    if (cmd_decode_error) begin
                        rx_enable <= 1'b0;
                        state     <= ST_IDLE;
                    end
                end

                // -------------------------------------------------
                // WB_STORE: write buffer contents to SDRAM, then
                // send STATUS response (no payload)
                // -------------------------------------------------
                ST_WB_STORE: begin
                    block_num       <= {param_block_mid, param_block_lo};
                    block_write_req <= 1'b1;
                    // Prepare STATUS response
                    enc_dest        <= latched_source;
                    enc_source      <= unit_id;
                    enc_ptype       <= PTYPE_STATUS;
                    enc_stat        <= 7'd0;
                    enc_payload_len <= 10'd0;
                    buf_addr_sel    <= 1'b1;
                    state           <= ST_WAIT_BLOCK;
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
