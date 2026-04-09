// sd_card_model.v — Behavioral SD card model for simulation
//
// SPI-mode SD card slave. Samples MOSI on SCK rising, drives MISO on falling.
// Supports: CMD0, CMD8, CMD55, ACMD41, CMD58, CMD17, CMD24.
//
// Memory is exposed as `mem[]` — testbenches pre-fill with data.

`timescale 1ns / 1ps

module sd_card_model #(
    parameter SDHC             = 1,
    parameter ACMD41_RETRIES   = 3,
    parameter WRITE_BUSY_CLKS  = 100,
    parameter MEM_BYTES        = 262144
)(
    input  wire spi_sck,
    input  wire spi_mosi,
    output reg  spi_miso,
    input  wire spi_cs_n
);

    reg [7:0] mem [0:MEM_BYTES-1];

    // =========================================================================
    // Byte-level RX: shift register + done flag
    // =========================================================================
    reg [7:0]  rx_shift = 8'hFF;
    reg [2:0]  rx_cnt = 3'd0;      // counts 0..7, wraps
    reg        rx_byte_done = 0;    // registered: 1 cycle after 8th bit
    reg [7:0]  rx_byte_val = 8'hFF; // captured complete byte

    always @(posedge spi_sck) begin
        if (spi_cs_n) begin
            rx_cnt  <= 3'd0;
            rx_shift <= 8'hFF;
            rx_byte_done <= 1'b0;
        end else begin
            rx_shift <= {rx_shift[6:0], spi_mosi};
            rx_cnt   <= rx_cnt + 3'd1;
            // When rx_cnt==7 (OLD value), we're shifting in the 8th bit
            if (rx_cnt == 3'd7) begin
                rx_byte_done <= 1'b1;
                rx_byte_val  <= {rx_shift[6:0], spi_mosi};
            end else begin
                rx_byte_done <= 1'b0;
            end
        end
    end

    // =========================================================================
    // Byte-level TX: shift register with independent byte counter
    // =========================================================================
    reg [7:0]  tx_shift = 8'hFF;
    reg [2:0]  tx_bit_cnt = 3'd0;
    reg        tx_active = 0;
    reg        force_low = 0;
    reg        tx_byte_done = 0;   // pulses HIGH for 1 posedge when 8 bits sent

    always @(negedge spi_sck) begin
        if (spi_cs_n) begin
            spi_miso   <= 1'b1;
            tx_bit_cnt <= 3'd0;
        end else if (force_low) begin
            spi_miso <= 1'b0;
        end else if (tx_active) begin
            spi_miso   <= tx_shift[7];
            tx_shift   <= {tx_shift[6:0], 1'b1};
            tx_bit_cnt <= tx_bit_cnt + 3'd1;
        end else begin
            spi_miso <= 1'b1;
        end
    end

    // Detect TX byte completion: tx_bit_cnt wraps from 7→0
    // Registered on posedge so it's visible to the protocol state machine
    always @(posedge spi_sck) begin
        tx_byte_done <= (tx_active && tx_bit_cnt == 3'd7);
    end

    // =========================================================================
    // Protocol state machine — advances on posedge spi_sck when rx_byte_done
    // =========================================================================
    localparam [3:0]
        ST_IDLE     = 4'd0,
        ST_CMD_RX   = 4'd1,   // accumulating remaining 5 cmd bytes
        ST_R1       = 4'd2,   // sending R1 response byte
        ST_R1_WAIT  = 4'd3,   // wait for R1 TX to complete
        ST_R7       = 4'd4,   // sending 4 extra response bytes
        ST_DATA_OUT = 4'd5,   // sending 0xFE + 512 + 2 CRC
        ST_DATA_IN  = 4'd6,   // receiving 0xFE + 512 + 2 CRC
        ST_DRESP    = 4'd7,   // sending data response (0x05)
        ST_BUSY     = 4'd8;

    reg [3:0]  pstate = ST_IDLE;

    // Command accumulator
    reg [7:0]  cmd_bytes [0:5];
    reg [2:0]  cmd_byte_cnt;

    // Response
    reg [7:0]  r1_val;
    reg [31:0] r7_val;
    reg [1:0]  r7_byte_idx;
    reg [31:0] block_addr;
    reg [9:0]  data_cnt;
    reg [1:0]  crc_cnt;
    reg [15:0] busy_cnt;
    reg [9:0]  din_cnt;           // data-in byte counter
    reg        send_data_after_r1; // CMD17: send data after R1
    reg        recv_data_after_r1; // CMD24: receive data after R1
    reg        send_r7_after_r1;   // CMD8/58: send R7 after R1

    // Protocol tracking
    reg        card_init = 0;
    reg        app_cmd = 0;
    reg [3:0]  acmd41_cnt = 0;

    // =========================================================================
    // Main protocol state machine
    // =========================================================================
    always @(posedge spi_sck) begin
        if (spi_cs_n) begin
            pstate           <= ST_IDLE;
            tx_active        <= 1'b0;
            force_low        <= 1'b0;
            cmd_byte_cnt     <= 3'd0;
            send_data_after_r1 <= 1'b0;
            recv_data_after_r1 <= 1'b0;
            send_r7_after_r1   <= 1'b0;
        end else begin
            case (pstate)

                // ----- IDLE: wait for command start byte (01xxxxxx) -----
                ST_IDLE: begin
                    tx_active <= 1'b0;
                    force_low <= 1'b0;
                    if (rx_byte_done && rx_byte_val[7:6] == 2'b01) begin
                        cmd_bytes[0]     <= rx_byte_val;
                        cmd_byte_cnt     <= 3'd1;
                        pstate           <= ST_CMD_RX;
                    end
                end

                // ----- CMD_RX: collect remaining 5 bytes -----
                ST_CMD_RX: begin
                    if (rx_byte_done) begin
                        cmd_bytes[cmd_byte_cnt] <= rx_byte_val;
                        if (cmd_byte_cnt == 3'd5) begin
                            // Full command — process (blocking assigns r1_val)
                            process_command;
                            // Pre-drive R1 MSB on MISO. Pre-shift tx_shift so
                            // the first negedge drives bit 6 (not bit 7 again).
                            spi_miso    <= r1_val[7];
                            tx_shift    <= {r1_val[6:0], 1'b1};
                            tx_active   <= 1'b1;
                            tx_bit_cnt  <= 3'd1; // 1 bit already "sent"
                            pstate      <= ST_R1_WAIT;
                        end else begin
                            cmd_byte_cnt <= cmd_byte_cnt + 3'd1;
                        end
                    end
                end

                // ----- R1_WAIT: wait for R1 byte to finish sending -----
                ST_R1_WAIT: begin
                    if (tx_byte_done) begin
                        if (send_r7_after_r1) begin
                            send_r7_after_r1 <= 1'b0;
                            r7_byte_idx  <= 2'd0;
                            tx_shift     <= r7_val[31:24];
                            tx_bit_cnt   <= 3'd0;
                            pstate       <= ST_R7;
                        end else if (send_data_after_r1) begin
                            send_data_after_r1 <= 1'b0;
                            tx_shift   <= 8'hFE;
                            tx_bit_cnt <= 3'd0;
                            data_cnt   <= 10'd0;
                            pstate     <= ST_DATA_OUT;
                        end else if (recv_data_after_r1) begin
                            recv_data_after_r1 <= 1'b0;
                            tx_active <= 1'b0;
                            din_cnt   <= 10'd0;
                            pstate    <= ST_DATA_IN;
                        end else begin
                            tx_active <= 1'b0;
                            pstate    <= ST_IDLE;
                        end
                    end
                end

                // ----- R7: send 4 extra bytes -----
                ST_R7: begin
                    if (tx_byte_done) begin
                        if (r7_byte_idx == 2'd3) begin
                            tx_active <= 1'b0;
                            pstate    <= ST_IDLE;
                        end else begin
                            r7_byte_idx <= r7_byte_idx + 2'd1;
                            tx_bit_cnt  <= 3'd0;
                            case (r7_byte_idx)
                                2'd0: tx_shift <= r7_val[23:16];
                                2'd1: tx_shift <= r7_val[15:8];
                                2'd2: tx_shift <= r7_val[7:0];
                                default: tx_shift <= 8'h00;
                            endcase
                        end
                    end
                end

                // ----- DATA_OUT: 0xFE token (already loaded) + 512 data + 2 CRC -----
                ST_DATA_OUT: begin
                    if (tx_byte_done) begin
                        if (data_cnt <= 10'd511) begin
                            tx_shift   <= mem[block_addr + {22'd0, data_cnt}];
                            tx_bit_cnt <= 3'd0;
                            data_cnt   <= data_cnt + 10'd1;
                        end else if (data_cnt <= 10'd513) begin
                            tx_shift   <= 8'h00;
                            tx_bit_cnt <= 3'd0;
                            data_cnt   <= data_cnt + 10'd1;
                        end else begin
                            tx_active <= 1'b0;
                            pstate    <= ST_IDLE;
                        end
                    end
                end

                // ----- DATA_IN: wait for 0xFE + 512 data + 2 CRC -----
                ST_DATA_IN: begin
                    if (rx_byte_done) begin
                        if (din_cnt == 10'd0) begin
                            if (rx_byte_val == 8'hFE) begin

                                din_cnt <= 10'd1;
                            end
                            // else keep waiting for start token
                        end else if (din_cnt <= 10'd512) begin
                            mem[block_addr + {22'd0, din_cnt} - 1] <= rx_byte_val;
                            din_cnt <= din_cnt + 10'd1;
                        end else if (din_cnt <= 10'd514) begin
                            // CRC (ignore)
                            din_cnt <= din_cnt + 10'd1;
                            if (din_cnt == 10'd514) begin

                                // Data received — send response 0x05
                                spi_miso   <= 1'b0;  // pre-drive MSB (0x05 bit 7 = 0)
                                tx_shift   <= {7'b0000101, 1'b1};
                                tx_active  <= 1'b1;
                                tx_bit_cnt <= 3'd1;
                                pstate     <= ST_DRESP;
                            end
                        end
                    end
                end

                // ----- DRESP: wait for response token to finish -----
                ST_DRESP: begin
                    if (tx_byte_done) begin
                        tx_active <= 1'b0;
                        force_low <= 1'b1;
                        busy_cnt  <= WRITE_BUSY_CLKS;
                        pstate    <= ST_BUSY;
                    end
                end

                // ----- BUSY: hold MISO low -----
                ST_BUSY: begin
                    if (busy_cnt == 16'd0) begin
                        force_low <= 1'b0;
                        pstate    <= ST_IDLE;
                    end else begin
                        busy_cnt <= busy_cnt - 16'd1;
                    end
                end

                default: pstate <= ST_IDLE;
            endcase
        end
    end

    // =========================================================================
    // Command processor
    // =========================================================================
    // process_command uses BLOCKING assignments so r1_val etc. are
    // available immediately in the same always block cycle.
    task process_command;
        reg [5:0]  cidx;
        reg [31:0] carg;
        begin
            cidx = cmd_bytes[0][5:0];
            carg = {cmd_bytes[1], cmd_bytes[2], cmd_bytes[3], cmd_bytes[4]};

            send_data_after_r1 = 1'b0;
            recv_data_after_r1 = 1'b0;
            send_r7_after_r1   = 1'b0;

            if (app_cmd && cidx == 6'd41) begin
                // ACMD41
                app_cmd = 1'b0;
                if (acmd41_cnt < ACMD41_RETRIES) begin
                    r1_val     = 8'h01;
                    acmd41_cnt = acmd41_cnt + 4'd1;
                end else begin
                    r1_val    = 8'h00;
                    card_init = 1'b1;
                end
            end else begin
                app_cmd = 1'b0;
                case (cidx)
                    6'd0: begin
                        r1_val     = 8'h01;
                        card_init  = 1'b0;
                        acmd41_cnt = 4'd0;
                    end
                    6'd8: begin
                        r1_val = 8'h01;
                        r7_val = {20'd0, carg[11:0]};
                        send_r7_after_r1 = 1'b1;
                    end
                    6'd55: begin
                        app_cmd = 1'b1;
                        r1_val  = card_init ? 8'h00 : 8'h01;
                    end
                    6'd58: begin
                        r1_val = 8'h00;
                        r7_val = SDHC ? 32'hC0FF8000 : 32'h80FF8000;
                        send_r7_after_r1 = 1'b1;
                    end
                    6'd17: begin
                        if (!card_init) begin
                            r1_val = 8'h05;
                        end else begin
                            r1_val     = 8'h00;
                            block_addr = SDHC ? (carg << 9) : carg;
                            send_data_after_r1 = 1'b1;
                        end
                    end
                    6'd24: begin
                        if (!card_init) begin
                            r1_val = 8'h05;
                        end else begin
                            r1_val     = 8'h00;
                            block_addr = SDHC ? (carg << 9) : carg;
                            recv_data_after_r1 = 1'b1;
                        end
                    end
                    default: begin
                        r1_val = 8'h04;
                    end
                endcase
            end
        end
    endtask

    // =========================================================================
    // Init
    // =========================================================================
    integer init_i;
    initial begin
        spi_miso  = 1'b1;
        pstate    = ST_IDLE;
        tx_active = 1'b0;
        force_low = 1'b0;
        for (init_i = 0; init_i < MEM_BYTES; init_i = init_i + 1)
            mem[init_i] = 8'h00;
    end

endmodule
