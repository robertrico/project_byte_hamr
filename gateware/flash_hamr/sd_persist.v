// sd_persist.v — SD card write-back engine for Flash Hamr
//
// Writes a dirty SDRAM block back to the SD card.
// Reads data from block_buffer (port A, 25MHz) — no internal buffer needed.
// The block_buffer already has the dirty data from the arbiter's WRITE cycle.
//
// .po: direct CMD24 write. Stream 512 bytes from block_buffer to SD.
// .2mg: deferred (requires read-modify-write, future enhancement).

`timescale 1ns / 1ps

module sd_persist #(
    parameter UNIT2_OFFSET = 16'd2048
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---- Command (from write_through) ----
    input  wire        start,
    input  wire [15:0] block_num,
    output reg         busy,

    // ---- Config ----
    input  wire        is_2mg,
    input  wire [15:0] data_offset,
    input  wire        persist_enabled,

    // ---- FAT32 geometry ----
    input  wire [31:0] fat32_data_start,
    input  wire [7:0]  fat32_spc,

    // ---- Chain map BRAM read port ----
    output reg  [9:0]  chain_rd_addr,
    input  wire [31:0] chain_rd_data,

    // ---- Block buffer read port (port A, 25MHz) ----
    // The arbiter just wrote this block — data is still there
    output reg  [8:0]  buf_rd_addr,
    input  wire [7:0]  buf_rd_data,

    // ---- SDRAM (only needed for claim signal) ----
    output reg         sdram_claim,

    // ---- SD controller write ----
    output reg         sd_write_start,
    output reg  [31:0] sd_write_addr,
    output reg  [7:0]  sd_write_data,
    output reg         sd_write_data_valid,
    input  wire        sd_write_data_req,
    input  wire        sd_write_done
);

    // =========================================================================
    // Sector computation (all shifts — FAT32 sizes are powers of 2)
    // =========================================================================
    wire [7:0] spc = fat32_spc;

    // =========================================================================
    // State machine
    // =========================================================================
    localparam [3:0]
        SP_IDLE          = 4'd0,
        SP_COMPUTE       = 4'd1,
        SP_CHAIN_LOOKUP  = 4'd2,
        SP_CHAIN_WAIT    = 4'd3,  // BRAM latency cycle 1
        SP_CHAIN_WAIT2   = 4'd4,  // BRAM latency cycle 2
        SP_SD_WRITE      = 4'd5,  // issue CMD24
        SP_SD_STREAM     = 4'd6,  // stream 512 bytes from block_buffer
        SP_SD_WAIT       = 4'd7,  // wait for sd_write_done
        SP_DONE          = 4'd8;

    reg [3:0] state = SP_IDLE;

    reg [15:0] rel_block;
    reg [31:0] file_byte;
    reg [31:0] sd_sector;
    reg [31:0] cluster_num;
    reg [8:0]  sector_in_cluster;
    reg [8:0]  stream_cnt;       // 0..511 bytes streamed

    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= SP_IDLE;
            busy        <= 1'b0;
            sdram_claim <= 1'b0;
            sd_write_start <= 1'b0;
            sd_write_data_valid <= 1'b0;
        end else begin
            sd_write_start <= 1'b0;

            case (state)

                SP_IDLE: begin
                    sdram_claim <= 1'b0;
                    busy        <= 1'b0;
                    if (start && persist_enabled) begin
                        busy      <= 1'b1;
                        rel_block <= block_num - UNIT2_OFFSET;
                        state     <= SP_COMPUTE;
                    end
                end

                // ---- Compute file byte offset ----
                SP_COMPUTE: begin
                    if (!is_2mg) begin
                        file_byte <= {rel_block, 9'd0};  // rel_block * 512
                    end else begin
                        file_byte <= {16'd0, data_offset} + {rel_block, 9'd0};
                    end
                    state <= SP_CHAIN_LOOKUP;
                end

                // ---- Look up cluster from chain_map ----
                SP_CHAIN_LOOKUP: begin
                    // cluster_idx = file_byte / bytes_per_cluster
                    case (spc)
                        8'd1:   chain_rd_addr <= file_byte[18:9];
                        8'd2:   chain_rd_addr <= file_byte[19:10];
                        8'd4:   chain_rd_addr <= file_byte[20:11];
                        8'd8:   chain_rd_addr <= file_byte[21:12];
                        8'd16:  chain_rd_addr <= file_byte[22:13];
                        8'd32:  chain_rd_addr <= file_byte[23:14];
                        8'd64:  chain_rd_addr <= file_byte[24:15];
                        8'd128: chain_rd_addr <= file_byte[25:16];
                        default: chain_rd_addr <= file_byte[24:15];
                    endcase
                    sector_in_cluster <= file_byte[17:9] & ({1'b0, spc} - 9'd1);
                    state <= SP_CHAIN_WAIT;
                end

                SP_CHAIN_WAIT: begin
                    state <= SP_CHAIN_WAIT2;
                end

                SP_CHAIN_WAIT2: begin
                    cluster_num <= chain_rd_data;
                    sd_sector   <= fat32_data_start +
                                   ((chain_rd_data - 32'd2) * {24'd0, spc}) +
                                   {23'd0, sector_in_cluster};
                    state <= SP_SD_WRITE;
                end

                // ---- Issue CMD24 ----
                SP_SD_WRITE: begin
                    sd_write_addr  <= sd_sector;
                    sd_write_start <= 1'b1;
                    stream_cnt     <= 9'd0;
                    buf_rd_addr    <= 9'd0;
                    state          <= SP_SD_STREAM;
                end

                // ---- Stream 512 bytes from block_buffer to SD ----
                SP_SD_STREAM: begin
                    sd_write_data_valid <= 1'b0;
                    if (sd_write_data_req) begin
                        sd_write_data       <= buf_rd_data;
                        sd_write_data_valid <= 1'b1;
                        buf_rd_addr         <= buf_rd_addr + 9'd1;
                        stream_cnt          <= stream_cnt + 9'd1;
                    end
                    if (sd_write_done)
                        state <= SP_DONE;
                end

                SP_DONE: begin
                    sdram_claim         <= 1'b0;
                    busy                <= 1'b0;
                    sd_write_data_valid <= 1'b0;
                    state               <= SP_IDLE;
                end

                default: state <= SP_IDLE;
            endcase
        end
    end

endmodule
