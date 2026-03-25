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
// All protocol logic runs in a single clock domain (fclk, 7.16 MHz).
// FM decoder, encoder, codec, and state machine share the same clock.
// Only the SDRAM block request interface crosses to the 25 MHz domain.
//
// Protocol reference: PicoPort firmware (sp_cmd.c) and Liron ROM.
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
    output wire        debug_sync,      // FM decoder sync_detected
    output wire [2:0]  debug_cmd_code,  // latched command code for LA debug
    output wire        debug_wr_idle    // wrdata_was_idle for LA debug
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
    localparam [4:0] ST_WAIT_WR_IDLE  = 5'd22;  // wait for wrdata idle before ACK
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
    reg        block_ready_seen_low = 1'b0; // for WAIT_BLOCK edge detection

    // SmartPort bus phase decode (matches PicoPort SP_BUS_* constants)
    wire req = phase[0];                              // PH0 = REQ
    wire sp_bus_enabled = phase[1] & phase[3];        // PH1+PH3 = bus enable
    wire sp_bus_command = sp_bus_enabled & req;        // PH1+PH3+PH0 = command mode
    wire sp_bus_reset   = phase[0] & phase[2] &       // PH0+PH2 = bus reset
                          ~phase[1] & ~phase[3];       // (but NOT PH1/PH3)

    // Detect wrdata activity (falling edge = IWM serializing data)
    reg wrdata_prev = 1'b1;
    wire wrdata_fall = wrdata_prev & ~wrdata;

    // Track wrdata idle time: counts fclk cycles since last wrdata_fall.
    // wrdata_was_idle = true when wrdata has been quiet for 32+ fclk (~4.5µs).
    reg [6:0] wrdata_idle_cnt = 7'd127;
    wire wrdata_was_idle = wrdata_idle_cnt[5];  // bit 5 = counter >= 32

    always @(posedge fclk or negedge rst_n) begin
        if (!rst_n) begin
            wrdata_prev     <= 1'b1;
            wrdata_idle_cnt <= 7'd127;
        end else begin
            wrdata_prev <= wrdata;
            if (wrdata_fall)
                wrdata_idle_cnt <= 7'd0;
            else if (!wrdata_idle_cnt[6])
                wrdata_idle_cnt <= wrdata_idle_cnt + 7'd1;
        end
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
    assign debug_wr_idle = wrdata_was_idle;
    wire       rx_packet_end;

    // FM encoder (buffer-driven)
    reg        tx_start = 1'b0;
    reg  [9:0] tx_byte_count = 10'd0;
    wire       tx_busy;
    wire       tx_enc_done;
    wire [9:0] tx_enc_buf_addr;
    wire [7:0] tx_enc_buf_data;
    // (rddata driven directly by FM encoder, gated by tx_active enable)

    // FM encoder enable: active during TX states. When sp_device exits TX
    // (timeout/error/completion), the encoder aborts and rddata goes idle.
    // Matches PicoPort's behavior of tristating the rddata pin after TX.
    wire tx_active = (state == ST_TX_SEND) || (state == 5'd21) ||  // TX_SEND, TX_SEND_WAIT
                     (state == ST_TX_DONE_ACK) || (state == ST_TX_DONE_WAIT);

    // TX packet buffer — BRAM, holds complete response packet
    // Written by codec (one byte/clock), read by FM encoder (one byte/224 clocks)
    reg  [9:0] tx_pkt_wr_addr = 10'd0;
    reg  [7:0] tx_pkt_wr_data = 8'd0;
    reg        tx_pkt_we = 1'b0;

    // Mux address/control: write during TX_FILL, read during TX_SEND
    wire [9:0] tx_buf_addr = (state == ST_TX_FILL) ? tx_pkt_wr_addr : tx_enc_buf_addr;
    wire       tx_buf_we   = tx_pkt_we;

    tx_packet_buf u_tx_pkt_buf (
        .clk   (fclk),
        .addr  (tx_buf_addr),
        .wdata (tx_pkt_wr_data),
        .we    (tx_buf_we),
        .rdata (tx_enc_buf_data)
    );

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
            tx_pkt_wr_data <= 8'd0;
            tx_pkt_we      <= 1'b0;
        end else begin
            // Increment AFTER the write took effect (previous cycle).
            // BRAM posedge captured addr+data when we=1, so now advance.
            if (tx_pkt_we)
                tx_pkt_wr_addr <= tx_pkt_wr_addr + 10'd1;

            tx_pkt_we <= 1'b0;  // default: no write
            if (state == ST_TX_START) begin
                tx_pkt_wr_addr <= 10'd0;  // reset overrides increment
            end else if (tx_pkt_byte_valid) begin
                tx_pkt_wr_data <= tx_pkt_byte;
                tx_pkt_we      <= 1'b1;
            end
        end
    end

    // =================================================================
    // Submodule instantiation
    // =================================================================

    // FM decoder runs on fclk — same domain as sp_device, no CDC needed.
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
        .enable         (tx_active),
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
    reg [6:0]  latched_cmd_code = 7'd0;    // command code from payload[0]
    assign debug_cmd_code = latched_cmd_code[2:0];
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
                    9'd4: param_block_lo  <= cmd_payload_out;
                    9'd5: param_block_mid <= cmd_payload_out;
                    9'd6: param_block_hi  <= cmd_payload_out;
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
            block_ready_seen_low <= 1'b0;
            unit_id             <= 7'd1;
            latched_cmd_code    <= 7'd0;
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

                    // Bus reset: PH0+PH2 (no PH1/PH3) — Liron asserts
                    // this for 80ms before INIT. Reset our unit assignment
                    // so device responds to fresh INIT enumeration.
                    if (sp_bus_reset) begin
                        unit_id <= 7'd1;
                    end

                    // Enter RX on any wrdata edge, same as an external
                    // peripheral would. The FM decoder + sync detector
                    // handles noise/garbage — it looks for 8+ consecutive
                    // 1-bits then $C3 PBEGIN. If it's not a real command,
                    // WAIT_SYNC times out (280us) and we return to IDLE.
                    // This matches PicoPort's approach: just listen to
                    // wrdata, don't gate on phases or idle time.
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
                // Timeout 2000 fclk (~280us). Sync preamble (5x $FF +
                // $C3) takes ~200us from first wrdata edge. Was 10000
                // fclk (1.4ms!) which prevented recovery from false
                // triggers within the Liron ROM's ~108us ACK timeout.
                // -------------------------------------------------
                ST_RX_WAIT_SYNC: begin
                    delay_counter <= delay_counter + 16'd1;
                    if (rx_sync) begin
                        state <= ST_RX_DECODE;
                    end else if (delay_counter >= 16'd5000) begin
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
                        latched_dest   <= cmd_dest;
                        latched_source <= cmd_source;
                        // Tentative: accept if dest matches unit_id.
                        // Will override to accept if payload[0] == INIT.
                        if (cmd_dest == unit_id)
                            cmd_for_us <= 1'b1;
                        else
                            cmd_for_us <= 1'b0;
                    end

                    // Capture command code from payload byte 0.
                    // Liron ROM sends cmd code in payload[0], not header STAT.
                    if (cmd_payload_valid && cmd_payload_addr == 9'd0) begin
                        latched_cmd_code <= cmd_payload_out[6:0];
                        // INIT: always accept regardless of dest
                        if (cmd_payload_out[6:0] == CMD_INIT)
                            cmd_for_us <= 1'b1;
                    end

                    if (cmd_decode_done || rx_packet_end) begin
                        rx_enable <= 1'b0;
                        if (cmd_for_us) begin
                            // Wait for wrdata to go idle before ACK.
                            // PicoPort: capture_samples() returns only after
                            // PIO RX detects wrdata idle. We do the same.
                            state <= ST_WAIT_WR_IDLE;
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
                // WAIT_WR_IDLE: wait for wrdata to stop toggling.
                // The IWM may still be serializing the last byte when
                // the codec finishes decoding. PicoPort waits for
                // PIO RX idle before ACK — we do the same.
                // delay_counter counts consecutive idle fclk cycles.
                // After 250 cycles (~35us, >1 byte period), wrdata
                // is confirmed idle and we proceed to ACK.
                // -------------------------------------------------
                ST_WAIT_WR_IDLE: begin
                    if (wrdata_fall) begin
                        delay_counter <= 16'd0;  // reset on any edge
                    end else begin
                        delay_counter <= delay_counter + 16'd1;
                    end
                    if (delay_counter >= 16'd250) begin
                        delay_counter <= 16'd0;
                        state <= ST_ACK;
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

                    case (latched_cmd_code)
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
                            delay_counter  <= 16'd0;  // reset for WAIT_BLOCK CDC guard
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
                            status_fill_data <= 8'hF8;  // block device, online, R/W
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
                    delay_counter <= delay_counter + 16'd1;
                    // Wait for boot_done (disk image loaded) AND block_ready
                    // (SDRAM transfer complete). The 32-cycle skip lets the
                    // CDC propagate block_ready clear from the arbiter.
                    // INIT/STATUS work before boot — only READBLOCK/WRITEBLOCK
                    // need SDRAM, so the boot has time to finish.
                    if (delay_counter > 16'd32 && boot_done && block_ready) begin
                        delay_counter <= 16'd0;
                        state <= ST_TX_WAIT_REQ;
                    end else if (delay_counter >= 16'd50000) begin
                        // ~7ms timeout — SDRAM read should take <1ms
                        sense <= 1'b1;
                        rx_enable <= 1'b0;
                        state <= ST_IDLE;
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
                        // Codec finished — tx_pkt_wr_addr is last-written
                        // addr (increment pending as NBA). Total count = addr + 1.
                        tx_byte_count <= tx_pkt_wr_addr + 10'd1;
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
                    tx_start      <= 1'b1;
                    delay_counter <= 16'd0;
                    state         <= 5'd21;  // TX_SEND_WAIT
                end

                // TX_SEND_WAIT: wait for FM encoder to finish.
                // Recovery: if encoder becomes not-busy without asserting done
                // (e.g., enable glitch), or if wrdata_fall arrives (new command
                // from host while we're still sending — means host gave up),
                // return to IDLE so future commands aren't blocked.
                5'd21: begin
                    if (tx_enc_done) begin
                        state <= ST_TX_DONE_ACK;
                    end else if (!tx_busy && delay_counter > 16'd100) begin
                        // Encoder stopped without done — stuck state recovery
                        sense <= 1'b1;
                        state <= ST_IDLE;
                    end
                    delay_counter <= delay_counter + 16'd1;
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
                    end else if (delay_counter >= 16'd50000) begin
                        // ~7ms timeout — generous for Liron ROM which may
                        // retry the DATA send (Sca67 retry loop)
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
                    // Do NOT check rx_packet_end here — the DATA packet's
                    // group-of-7 encoded payload can contain $C8 as data.
                    // Only the codec knows the true packet boundary (from
                    // ODDCNT/GRP7CNT). Wait for cmd_decode_done only.
                    if (cmd_decode_done) begin
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
                    delay_counter   <= 16'd0;  // reset for WAIT_BLOCK CDC guard
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
