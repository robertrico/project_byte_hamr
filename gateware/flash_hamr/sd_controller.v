// sd_controller.v — SD card command/response layer (SPI mode)
//
// Handles initialization, CMD17 (read block), CMD24 (write block).
// Instantiates sd_spi for byte-level transfers.
//
// Init: power-up clocks → CMD0 → CMD8 → ACMD41 loop → CMD58 → done
// Read: CMD17 → R1 → start token → 512 data bytes → 2 CRC
// Write: CMD24 → R1 → 0xFE token → 512 data bytes → 2 CRC → resp → busy

`timescale 1ns / 1ps

module sd_controller (
    input  wire        clk,          // 25 MHz
    input  wire        rst_n,

    // ---- Init ----
    input  wire        init_start,
    output reg         init_done,
    output reg         init_error,
    output reg         is_sdhc,

    // ---- Block read (CMD17) ----
    input  wire        read_start,
    input  wire [31:0] read_addr,     // block number
    output wire [7:0]  read_data,
    output reg         read_data_valid,
    input  wire        read_data_ready,
    output reg         read_done,
    output reg         read_error,

    // ---- Block write (CMD24) ----
    input  wire        write_start,
    input  wire [31:0] write_addr,    // block number
    input  wire [7:0]  write_data,
    input  wire        write_data_valid,
    output reg         write_data_req,
    output reg         write_done,
    output reg         write_error,

    // ---- SPI pins ----
    output wire        sd_sck,
    output wire        sd_mosi,
    input  wire        sd_miso,
    output reg         sd_cs
);

    // =========================================================================
    // SPI byte engine
    // =========================================================================
    reg        spi_start;
    reg [7:0]  spi_tx;
    wire [7:0] spi_rx;
    wire       spi_done;
    wire       spi_busy;
    reg        spi_slow;

    sd_spi u_spi (
        .clk(clk), .rst_n(rst_n),
        .start(spi_start), .tx_byte(spi_tx), .rx_byte(spi_rx),
        .done(spi_done), .busy(spi_busy), .slow_mode(spi_slow),
        .spi_sck(sd_sck), .spi_mosi(sd_mosi), .spi_miso(sd_miso)
    );

    assign read_data = spi_rx;

    // =========================================================================
    // Sub-state: send_byte — fires spi_start, waits for spi_done
    // Used by macro-states via phase counters.
    // =========================================================================
    // Convention: set spi_tx and spi_start=1 to initiate. spi_done=1 when complete.
    // The state machine checks (spi_done) to advance, or (!spi_busy && !spi_start) to initiate.

    // =========================================================================
    // Command buffer + argument
    // =========================================================================
    reg [7:0]  cmd_frame [0:5];
    reg [2:0]  byte_idx;
    reg [7:0]  r1;
    reg [31:0] r7;
    reg [1:0]  r7_cnt;
    reg [7:0]  poll_cnt;
    reg [9:0]  data_cnt;
    reg [1:0]  crc_cnt;
    reg [24:0] acmd41_timeout;
    reg [19:0] token_timeout;
    reg        sd_v2;
    reg [7:0]  wr_latch;          // latched write data byte
    reg        wr_latched;        // write data ready to send

    task set_cmd;
        input [5:0]  idx;
        input [31:0] arg;
        input [7:0]  crc;
        begin
            cmd_frame[0] = {2'b01, idx};
            cmd_frame[1] = arg[31:24];
            cmd_frame[2] = arg[23:16];
            cmd_frame[3] = arg[15:8];
            cmd_frame[4] = arg[7:0];
            cmd_frame[5] = crc;
        end
    endtask

    // =========================================================================
    // State encoding — explicit states, no return_state trickery
    // =========================================================================
    localparam [5:0]
        IDLE            = 6'd0,
        // Init sequence
        POWERUP         = 6'd1,   // CS=HIGH, send 10× 0xFF
        I_CMD0_SEND     = 6'd2,   // send CMD0 bytes
        I_CMD0_POLL     = 6'd3,   // poll R1
        I_CMD0_DONE     = 6'd4,   // check R1, CS_HIGH gap
        I_CMD8_SEND     = 6'd5,
        I_CMD8_POLL     = 6'd6,
        I_CMD8_R7       = 6'd7,   // read 4 R7 bytes
        I_CMD8_DONE     = 6'd8,
        I_CMD55_SEND    = 6'd9,
        I_CMD55_POLL    = 6'd10,
        I_ACMD41_SEND   = 6'd11,
        I_ACMD41_POLL   = 6'd12,
        I_ACMD41_DONE   = 6'd13,
        I_CMD58_SEND    = 6'd14,
        I_CMD58_POLL    = 6'd15,
        I_CMD58_R7      = 6'd16,
        I_CMD58_DONE    = 6'd17,
        I_DONE          = 6'd18,
        I_ERROR         = 6'd19,
        GAP             = 6'd20,  // CS=HIGH + 8 clocks
        // Read CMD17
        R_CMD_SEND      = 6'd21,
        R_CMD_POLL      = 6'd22,
        R_TOKEN         = 6'd23,  // poll for 0xFE
        R_DATA          = 6'd24,  // 512 bytes
        R_CRC           = 6'd25,  // 2 CRC bytes
        R_DONE          = 6'd26,
        // Write CMD24
        W_CMD_SEND      = 6'd27,
        W_CMD_POLL      = 6'd28,
        W_TOKEN         = 6'd29,  // send 0xFE
        W_DATA          = 6'd30,  // 512 bytes
        W_CRC           = 6'd31,
        W_RESP          = 6'd32,
        W_BUSY          = 6'd33,
        W_DONE          = 6'd34;

    reg [5:0]  state = IDLE;
    reg [5:0]  gap_return;        // where to go after GAP state
    reg [4:0]  powerup_cnt;

    // =========================================================================
    // State machine
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= IDLE;
            sd_cs           <= 1'b1;
            spi_start       <= 1'b0;
            spi_tx          <= 8'hFF;
            spi_slow        <= 1'b1;
            init_done       <= 1'b0;
            init_error      <= 1'b0;
            is_sdhc         <= 1'b0;
            read_data_valid <= 1'b0;
            read_done       <= 1'b0;
            read_error      <= 1'b0;
            write_data_req  <= 1'b0;
            write_done      <= 1'b0;
            write_error     <= 1'b0;
            sd_v2           <= 1'b0;
            wr_latch        <= 8'd0;
            wr_latched      <= 1'b0;
            r1              <= 8'hFF;
            r7              <= 32'd0;
            byte_idx        <= 3'd0;
            poll_cnt        <= 8'd0;
            data_cnt        <= 10'd0;
            crc_cnt         <= 2'd0;
            powerup_cnt     <= 4'd0;
            acmd41_timeout  <= 25'd0;
            token_timeout   <= 20'd0;
            r7_cnt          <= 2'd0;
            gap_return      <= IDLE;
        end else begin
            spi_start       <= 1'b0;
            read_data_valid <= 1'b0;
            read_done       <= 1'b0;
            read_error      <= 1'b0;
            write_data_req  <= 1'b0;
            write_done      <= 1'b0;
            write_error     <= 1'b0;

            // Free-running timeout counter for ACMD41 loop
            // Increments every clock when in CMD55/ACMD41 states
            if (state >= I_CMD55_SEND && state <= I_ACMD41_DONE)
                acmd41_timeout <= acmd41_timeout + 25'd1;

            case (state)

                // =============================================================
                IDLE: begin
                    sd_cs <= 1'b1;
                    if (init_start) begin
                        init_done      <= 1'b0;
                        init_error     <= 1'b0;
                        spi_slow       <= 1'b1;
                        powerup_cnt    <= 5'd0;
                        acmd41_timeout <= 25'd0;
                        state          <= POWERUP;
                    end else if (read_start && init_done) begin
                        sd_cs <= 1'b0;
                        set_cmd(6'd17, is_sdhc ? read_addr : {read_addr[22:0], 9'd0}, 8'hFF);
                        byte_idx <= 3'd0;
                        state    <= R_CMD_SEND;
                    end else if (write_start && init_done) begin
                        sd_cs <= 1'b0;
                        set_cmd(6'd24, is_sdhc ? write_addr : {write_addr[22:0], 9'd0}, 8'hFF);
                        byte_idx <= 3'd0;
                        state    <= W_CMD_SEND;
                    end
                end

                // =============================================================
                // POWER UP — CS high, 160+ clocks (SDXC needs more)
                // =============================================================
                POWERUP: begin
                    sd_cs <= 1'b1;
                    if (spi_done || (!spi_busy && !spi_start && powerup_cnt == 5'd0)) begin
                        if (powerup_cnt >= 5'd20) begin
                            sd_cs <= 1'b0;
                            set_cmd(6'd0, 32'd0, 8'h95);
                            byte_idx <= 3'd0;
                            state    <= I_CMD0_SEND;
                        end else begin
                            spi_tx      <= 8'hFF;
                            spi_start   <= 1'b1;
                            powerup_cnt <= powerup_cnt + 5'd1;
                        end
                    end
                end

                // =============================================================
                // CMD0
                // =============================================================
                I_CMD0_SEND: begin
                    if (!spi_busy && !spi_start) begin
                        spi_tx    <= cmd_frame[byte_idx];
                        spi_start <= 1'b1;
                        byte_idx  <= byte_idx + 3'd1;
                        if (byte_idx == 3'd5) begin
                            poll_cnt <= 8'd0;
                            state    <= I_CMD0_POLL;
                        end
                    end
                end

                I_CMD0_POLL: begin
                    if (spi_done) begin
                        if (spi_rx != 8'hFF) begin
                            r1    <= spi_rx;
                            state <= I_CMD0_DONE;
                        end else if (poll_cnt >= 8'd100) begin
                            state <= I_ERROR;
                        end else begin
                            poll_cnt  <= poll_cnt + 8'd1;
                            spi_tx    <= 8'hFF;
                            spi_start <= 1'b1;
                        end
                    end else if (!spi_busy && !spi_start) begin
                        spi_tx    <= 8'hFF;
                        spi_start <= 1'b1;
                    end
                end

                I_CMD0_DONE: begin
                    if (r1 == 8'h01) begin
                        gap_return <= I_CMD8_SEND;
                        state      <= GAP;
                    end else
                        state <= I_ERROR;
                end

                // =============================================================
                // GAP — CS high, send 8 clocks, then go to gap_return
                // =============================================================
                GAP: begin
                    sd_cs <= 1'b1;
                    if (spi_done) begin
                        state <= gap_return;
                    end else if (!spi_busy && !spi_start) begin
                        spi_tx    <= 8'hFF;
                        spi_start <= 1'b1;
                    end
                end

                // =============================================================
                // CMD8
                // =============================================================
                I_CMD8_SEND: begin
                    sd_cs <= 1'b0;
                    if (!spi_busy && !spi_start) begin
                        if (byte_idx == 3'd0) begin
                            set_cmd(6'd8, 32'h000001AA, 8'h87);
                        end
                        spi_tx    <= cmd_frame[byte_idx];
                        spi_start <= 1'b1;
                        byte_idx  <= byte_idx + 3'd1;
                        if (byte_idx == 3'd5) begin
                            poll_cnt <= 8'd0;
                            state    <= I_CMD8_POLL;
                        end
                    end
                end

                I_CMD8_POLL: begin
                    if (spi_done) begin
                        if (spi_rx != 8'hFF) begin
                            r1 <= spi_rx;

                            if (spi_rx[2]) begin
                                // Illegal command = SD v1
                                sd_v2      <= 1'b0;
                                gap_return <= I_CMD55_SEND;
                                state      <= GAP;
                            end else begin
                                sd_v2  <= 1'b1;
                                r7     <= 32'd0;
                                r7_cnt <= 2'd0;
                                state  <= I_CMD8_R7;
                            end
                        end else if (poll_cnt >= 8'd100) begin
                            state <= I_ERROR;
                        end else begin
                            poll_cnt  <= poll_cnt + 8'd1;
                            spi_tx    <= 8'hFF;
                            spi_start <= 1'b1;
                        end
                    end else if (!spi_busy && !spi_start) begin
                        spi_tx    <= 8'hFF;
                        spi_start <= 1'b1;
                    end
                end

                I_CMD8_R7: begin
                    if (spi_done) begin
                        r7 <= {r7[23:0], spi_rx};
                        if (r7_cnt == 2'd3) begin
                            gap_return <= I_CMD55_SEND;
                            state      <= GAP;
                        end else begin
                            r7_cnt    <= r7_cnt + 2'd1;
                            spi_tx    <= 8'hFF;
                            spi_start <= 1'b1;
                        end
                    end else if (!spi_busy && !spi_start) begin
                        spi_tx    <= 8'hFF;
                        spi_start <= 1'b1;
                    end
                end

                // =============================================================
                // CMD55 + ACMD41
                // =============================================================
                I_CMD55_SEND: begin
                    sd_cs <= 1'b0;
                    if (!spi_busy && !spi_start) begin
                        if (byte_idx == 3'd0)
                            set_cmd(6'd55, 32'd0, 8'hFF);
                        spi_tx    <= cmd_frame[byte_idx];
                        spi_start <= 1'b1;
                        byte_idx  <= byte_idx + 3'd1;
                        if (byte_idx == 3'd5) begin
                            poll_cnt <= 8'd0;
                            state    <= I_CMD55_POLL;
                        end
                    end
                end

                I_CMD55_POLL: begin
                    if (spi_done) begin
                        if (spi_rx != 8'hFF) begin
                            r1       <= spi_rx;
                            byte_idx <= 3'd0;
                            state    <= I_ACMD41_SEND;
                        end else if (poll_cnt >= 8'd100) begin
                            state <= I_ERROR;
                        end else begin
                            poll_cnt  <= poll_cnt + 8'd1;
                            spi_tx    <= 8'hFF;
                            spi_start <= 1'b1;
                        end
                    end else if (!spi_busy && !spi_start) begin
                        spi_tx    <= 8'hFF;
                        spi_start <= 1'b1;
                    end
                end

                I_ACMD41_SEND: begin
                    if (!spi_busy && !spi_start) begin
                        if (byte_idx == 3'd0)
                            set_cmd(6'd41, sd_v2 ? 32'h40000000 : 32'd0, 8'hFF);
                        spi_tx    <= cmd_frame[byte_idx];
                        spi_start <= 1'b1;
                        byte_idx  <= byte_idx + 3'd1;
                        if (byte_idx == 3'd5) begin
                            poll_cnt <= 8'd0;
                            state    <= I_ACMD41_POLL;
                        end
                    end
                end

                I_ACMD41_POLL: begin
                    if (spi_done) begin
                        if (spi_rx != 8'hFF) begin
                            r1    <= spi_rx;
                            state <= I_ACMD41_DONE;
                        end else if (poll_cnt >= 8'd100) begin
                            state <= I_ERROR;
                        end else begin
                            poll_cnt  <= poll_cnt + 8'd1;
                            spi_tx    <= 8'hFF;
                            spi_start <= 1'b1;
                        end
                    end else if (!spi_busy && !spi_start) begin
                        spi_tx    <= 8'hFF;
                        spi_start <= 1'b1;
                    end
                end

                I_ACMD41_DONE: begin

                    if (r1 == 8'h00) begin
                        // Ready — get OCR
                        byte_idx   <= 3'd0;
                        gap_return <= I_CMD58_SEND;
                        state      <= GAP;
                    end else if (r1 == 8'h01) begin
                        // Still initializing — retry (timeout is clock-based, ~2s)
                        if (acmd41_timeout >= 25'd25000000) begin  // 25M clocks = 1s
                            state <= I_ERROR;
                        end else begin
                            byte_idx   <= 3'd0;
                            gap_return <= I_CMD55_SEND;
                            state      <= GAP;
                        end
                    end else begin
                        state <= I_ERROR;
                    end
                end

                // =============================================================
                // CMD58 — Read OCR
                // =============================================================
                I_CMD58_SEND: begin
                    sd_cs <= 1'b0;
                    if (!spi_busy && !spi_start) begin
                        if (byte_idx == 3'd0)
                            set_cmd(6'd58, 32'd0, 8'hFF);
                        spi_tx    <= cmd_frame[byte_idx];
                        spi_start <= 1'b1;
                        byte_idx  <= byte_idx + 3'd1;
                        if (byte_idx == 3'd5) begin
                            poll_cnt <= 8'd0;
                            state    <= I_CMD58_POLL;
                        end
                    end
                end

                I_CMD58_POLL: begin
                    if (spi_done) begin
                        if (spi_rx != 8'hFF) begin
                            r1     <= spi_rx;
                            r7     <= 32'd0;
                            r7_cnt <= 2'd0;
                            state  <= I_CMD58_R7;
                        end else if (poll_cnt >= 8'd100) begin
                            state <= I_ERROR;
                        end else begin
                            poll_cnt  <= poll_cnt + 8'd1;
                            spi_tx    <= 8'hFF;
                            spi_start <= 1'b1;
                        end
                    end else if (!spi_busy && !spi_start) begin
                        spi_tx    <= 8'hFF;
                        spi_start <= 1'b1;
                    end
                end

                I_CMD58_R7: begin
                    if (spi_done) begin
                        r7 <= {r7[23:0], spi_rx};
                        if (r7_cnt == 2'd3) begin
                            state <= I_CMD58_DONE;
                        end else begin
                            r7_cnt    <= r7_cnt + 2'd1;
                            spi_tx    <= 8'hFF;
                            spi_start <= 1'b1;
                        end
                    end else if (!spi_busy && !spi_start) begin
                        spi_tx    <= 8'hFF;
                        spi_start <= 1'b1;
                    end
                end

                I_CMD58_DONE: begin

                    if (r1 == 8'h00) begin
                        is_sdhc <= r7[30];  // CCS bit in OCR
                        state   <= I_DONE;
                    end else begin
                        state <= I_ERROR;
                    end
                end

                // =============================================================
                // Init complete
                // =============================================================
                I_DONE: begin
                    sd_cs     <= 1'b1;
                    spi_slow  <= 1'b0;  // switch to fast clock
                    init_done <= 1'b1;
                    state     <= IDLE;
                end

                I_ERROR: begin
                    sd_cs      <= 1'b1;
                    spi_slow   <= 1'b0;
                    init_error <= 1'b1;
                    state      <= IDLE;
                end

                // =============================================================
                // CMD17 — Read block
                // =============================================================
                R_CMD_SEND: begin
                    if (!spi_busy && !spi_start) begin
                        spi_tx    <= cmd_frame[byte_idx];
                        spi_start <= 1'b1;
                        byte_idx  <= byte_idx + 3'd1;
                        if (byte_idx == 3'd5) begin
                            poll_cnt <= 8'd0;
                            state    <= R_CMD_POLL;
                        end
                    end
                end

                R_CMD_POLL: begin
                    if (spi_done) begin
                        if (spi_rx != 8'hFF) begin
                            r1 <= spi_rx;
                            if (spi_rx != 8'h00) begin
                                read_error <= 1'b1;
                                sd_cs      <= 1'b1;
                                state      <= IDLE;
                            end else begin
                                token_timeout <= 20'd0;
                                state         <= R_TOKEN;
                            end
                        end else if (poll_cnt >= 8'd100) begin
                            read_error <= 1'b1;
                            sd_cs      <= 1'b1;
                            state      <= IDLE;
                        end else begin
                            poll_cnt  <= poll_cnt + 8'd1;
                            spi_tx    <= 8'hFF;
                            spi_start <= 1'b1;
                        end
                    end else if (!spi_busy && !spi_start) begin
                        spi_tx    <= 8'hFF;
                        spi_start <= 1'b1;
                    end
                end

                R_TOKEN: begin
                    if (spi_done) begin
                        if (spi_rx == 8'hFE) begin
                            data_cnt <= 10'd0;
                            state    <= R_DATA;
                            spi_tx    <= 8'hFF;
                            spi_start <= 1'b1;
                        end else begin
                            token_timeout <= token_timeout + 20'd1;
                            if (token_timeout >= 20'd500000) begin
                                read_error <= 1'b1;
                                sd_cs      <= 1'b1;
                                state      <= IDLE;
                            end else begin
                                spi_tx    <= 8'hFF;
                                spi_start <= 1'b1;
                            end
                        end
                    end else if (!spi_busy && !spi_start) begin
                        spi_tx    <= 8'hFF;
                        spi_start <= 1'b1;
                    end
                end

                R_DATA: begin
                    if (spi_done) begin
                        read_data_valid <= 1'b1;
                        data_cnt        <= data_cnt + 10'd1;
                        if (data_cnt == 10'd511) begin
                            crc_cnt <= 2'd0;
                            state   <= R_CRC;
                        end
                        // DON'T start next byte on same cycle as data delivery.
                        // Wait for consumer to process and re-check read_data_ready
                        // on the next cycle via the else-if below.
                    end else if (!spi_busy && !spi_start && read_data_ready && data_cnt < 10'd512) begin
                        spi_tx    <= 8'hFF;
                        spi_start <= 1'b1;
                    end
                end

                R_CRC: begin
                    if (spi_done) begin
                        if (crc_cnt == 2'd1) begin
                            state <= R_DONE;
                        end else begin
                            crc_cnt   <= crc_cnt + 2'd1;
                            spi_tx    <= 8'hFF;
                            spi_start <= 1'b1;
                        end
                    end else if (!spi_busy && !spi_start) begin
                        spi_tx    <= 8'hFF;
                        spi_start <= 1'b1;
                    end
                end

                R_DONE: begin
                    sd_cs     <= 1'b1;
                    read_done <= 1'b1;
                    state     <= IDLE;
                end

                // =============================================================
                // CMD24 — Write block
                // =============================================================
                W_CMD_SEND: begin
                    if (!spi_busy && !spi_start) begin
                        spi_tx    <= cmd_frame[byte_idx];
                        spi_start <= 1'b1;
                        byte_idx  <= byte_idx + 3'd1;
                        if (byte_idx == 3'd5) begin
                            poll_cnt <= 8'd0;
                            state    <= W_CMD_POLL;
                        end
                    end
                end

                W_CMD_POLL: begin
                    if (spi_done) begin
                        if (spi_rx != 8'hFF) begin
                            r1 <= spi_rx;
                            if (spi_rx != 8'h00) begin
                                write_error <= 1'b1;
                                sd_cs       <= 1'b1;
                                state       <= IDLE;
                            end else begin
                                state <= W_TOKEN;
                            end
                        end else if (poll_cnt >= 8'd100) begin
                            write_error <= 1'b1;
                            sd_cs       <= 1'b1;
                            state       <= IDLE;
                        end else begin
                            poll_cnt  <= poll_cnt + 8'd1;
                            spi_tx    <= 8'hFF;
                            spi_start <= 1'b1;
                        end
                    end else if (!spi_busy && !spi_start) begin
                        spi_tx    <= 8'hFF;
                        spi_start <= 1'b1;
                    end
                end

                W_TOKEN: begin
                    if (!spi_busy && !spi_start) begin
                        spi_tx         <= 8'hFE;
                        spi_start      <= 1'b1;
                        data_cnt       <= 10'd0;
                        wr_latched     <= 1'b0;
                        write_data_req <= 1'b1;
                        state          <= W_DATA;
                    end
                end

                W_DATA: begin
                    // Latch data when provider signals valid
                    if (write_data_valid && !wr_latched) begin
                        wr_latch   <= write_data;
                        wr_latched <= 1'b1;
                    end
                    // Send latched byte when SPI finishes previous
                    if (spi_done && wr_latched) begin
                        spi_tx     <= wr_latch;
                        spi_start  <= 1'b1;
                        wr_latched <= 1'b0;
                        data_cnt   <= data_cnt + 10'd1;
                        if (data_cnt == 10'd511) begin
                            crc_cnt <= 2'd0;
                            state   <= W_CRC;
                        end else begin
                            write_data_req <= 1'b1;
                        end
                    end
                end

                W_CRC: begin
                    if (spi_done) begin
                        if (crc_cnt == 2'd1) begin
                            spi_tx    <= 8'hFF;
                            spi_start <= 1'b1;
                            poll_cnt  <= 8'd0;
                            state     <= W_RESP;
                        end else begin
                            crc_cnt   <= crc_cnt + 2'd1;
                            spi_tx    <= 8'hFF;
                            spi_start <= 1'b1;
                        end
                    end else if (!spi_busy && !spi_start) begin
                        spi_tx    <= 8'hFF;
                        spi_start <= 1'b1;
                    end
                end

                W_RESP: begin
                    if (spi_done) begin
                        if (spi_rx == 8'hFF) begin
                            // Still waiting — poll again (up to 20 bytes)
                            if (poll_cnt >= 8'd20) begin
                                write_error <= 1'b1;
                                sd_cs       <= 1'b1;
                                state       <= IDLE;
                            end else begin
                                poll_cnt  <= poll_cnt + 8'd1;
                                spi_tx    <= 8'hFF;
                                spi_start <= 1'b1;
                            end
                        end else if ((spi_rx & 8'h1F) == 8'h05) begin
                            state <= W_BUSY;
                        end else begin
                            write_error <= 1'b1;
                            sd_cs       <= 1'b1;
                            state       <= IDLE;
                        end
                    end else if (!spi_busy && !spi_start) begin
                        spi_tx    <= 8'hFF;
                        spi_start <= 1'b1;
                    end
                end

                W_BUSY: begin
                    if (spi_done) begin
                        if (spi_rx == 8'hFF) begin
                            state <= W_DONE;
                        end else begin
                            spi_tx    <= 8'hFF;
                            spi_start <= 1'b1;
                        end
                    end else if (!spi_busy && !spi_start) begin
                        spi_tx    <= 8'hFF;
                        spi_start <= 1'b1;
                    end
                end

                W_DONE: begin
                    sd_cs      <= 1'b1;
                    write_done <= 1'b1;
                    state      <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
