// sd_mount.v — Image mount engine for Flash Hamr
//
// Streams SD card data directly to SDRAM — no sector buffer.
// Same pattern as boot_loader (flash→SDRAM streaming).
// Follows FAT cluster chain. Handles .2mg header detection.

`timescale 1ns / 1ps

module sd_mount #(
    parameter UNIT2_OFFSET = 16'd2048
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        mount_request,
    input  wire [3:0]  img_select,

    output reg         s4d2_mounted,
    output reg         s4d2_loading,
    output reg  [15:0] s4d2_block_count,
    output reg         is_2mg,
    output reg  [15:0] data_offset,

    // Catalog BRAM read
    output reg  [8:0]  cat_rd_addr,
    input  wire [7:0]  cat_rd_data,

    // FAT geometry
    input  wire [31:0] fat32_data_start,
    input  wire [31:0] fat32_fat_start,
    input  wire [7:0]  fat32_spc,
    input  wire        fat_is_fat16,

    // SD controller read
    output reg         sd_read_start,
    output reg  [31:0] sd_read_addr,
    input  wire [7:0]  sd_read_data,
    input  wire        sd_read_data_valid,
    output reg         sd_read_data_ready,
    input  wire        sd_read_done,

    // SDRAM write
    output reg         sdram_req,
    output reg         sdram_req_write,
    output reg  [25:0] sdram_req_addr,
    output reg  [15:0] sdram_req_wdata,
    input  wire        sdram_req_ready,
    output reg         sdram_claim,

    // Chain map BRAM write
    output reg  [9:0]  chain_wr_addr,
    output reg  [31:0] chain_wr_data,
    output reg         chain_wr_en,

    output reg  [31:0] sd_image_first_cluster,
    output reg  [15:0] dbg_sector_count,    // DEBUG: sectors read during mount
    output reg  [4:0]  dbg_mount_state,     // DEBUG: current state
    output reg  [15:0] dbg_fat_entry        // DEBUG: last FAT entry read
);

    // =========================================================================
    // State machine
    // =========================================================================
    localparam [4:0]
        IDLE         = 5'd0,
        READ_CAT     = 5'd1,
        PARSE_CAT    = 5'd2,
        CLAIM        = 5'd3,
        READ_SECTOR  = 5'd4,
        COPY_LO      = 5'd5,   // capture low byte from SD
        COPY_HI      = 5'd6,   // capture high byte, write word to SDRAM
        WRITE_SDRAM  = 5'd7,   // wait for SDRAM accept
        NEXT_SECTOR  = 5'd8,
        NEXT_CLUSTER = 5'd9,
        FAT_RECV     = 5'd10,
        FAT_DONE     = 5'd11,
        DONE         = 5'd12;

    reg [4:0] state = IDLE;
    reg [1:0] cat_sub = 2'd0;  // sub-state for READ_CAT BRAM pipeline

    // Catalog entry
    reg [7:0]  cat_buf [0:19];
    reg [4:0]  cat_byte_idx;
    reg [31:0] first_cluster;
    reg [31:0] file_size;

    // .2mg header capture (grabbed from byte stream inline)
    reg [7:0]  hdr0, hdr1, hdr2, hdr3;
    reg [7:0]  hdr20, hdr21, hdr24, hdr25;
    reg [7:0]  hdr28, hdr29, hdr30, hdr31;

    // Cluster/sector tracking
    reg [31:0] current_cluster;
    reg [8:0]  sector_in_cluster;
    reg [9:0]  chain_idx;
    reg [25:0] sdram_write_addr;
    reg [31:0] bytes_remaining;
    reg        first_sector;
    reg [15:0] skip_bytes;

    // Streaming state
    reg [7:0]  lo_byte;
    reg [9:0]  byte_in_sector;  // counts 0..511 per sector

    // FAT parsing
    reg [31:0] fat_entry;
    reg [9:0]  fat_collect_idx;
    reg [9:0]  fat_target_offset;

    // Cluster → sector
    wire [31:0] cluster_sector = fat32_data_start +
                                  ((current_cluster - 32'd2) * {24'd0, fat32_spc});

    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= IDLE;
            s4d2_mounted <= 1'b0;
            s4d2_loading <= 1'b0;
            s4d2_block_count <= 16'd0;
            is_2mg       <= 1'b0;
            data_offset  <= 16'd0;
            sd_read_start <= 1'b0;
            sd_read_data_ready <= 1'b1;
            sdram_req    <= 1'b0;
            sdram_claim  <= 1'b0;
            chain_wr_en  <= 1'b0;
            sd_image_first_cluster <= 32'd0;
            dbg_sector_count <= 16'd0;
            dbg_mount_state  <= 5'd0;
            dbg_fat_entry    <= 16'd0;
        end else begin
            sd_read_start <= 1'b0;
            chain_wr_en   <= 1'b0;
            dbg_mount_state <= state;

            case (state)

                IDLE: begin
                    sdram_claim <= 1'b0;
                    if (mount_request) begin
                        s4d2_mounted <= 1'b0;
                        s4d2_loading <= 1'b1;
                        is_2mg       <= 1'b0;
                        data_offset  <= 16'd0;
                        skip_bytes   <= 16'd0;
                        cat_byte_idx <= 5'd0;
                        cat_sub      <= 2'd0;
                        dbg_sector_count <= 16'd0;
                        state        <= READ_CAT;
                    end
                end

                // ---- Read 20 bytes from catalog BRAM ----
                // Use explicit wait cycles to avoid BRAM latency assumptions.
                // 4 cycles per byte: set addr → wait → wait → capture.
                READ_CAT: begin
                    case (cat_sub)
                        2'd0: begin
                            cat_rd_addr <= {img_select, cat_byte_idx};
                            cat_sub <= 2'd1;
                        end
                        2'd1: cat_sub <= 2'd2;  // wait for BRAM pipeline
                        2'd2: cat_sub <= 2'd3;  // extra wait for safety
                        2'd3: begin
                            cat_buf[cat_byte_idx] <= cat_rd_data;
                            if (cat_byte_idx == 5'd19) begin
                                state <= PARSE_CAT;
                            end else begin
                                cat_byte_idx <= cat_byte_idx + 5'd1;
                                cat_sub <= 2'd0;
                            end
                        end
                    endcase
                end

                PARSE_CAT: begin
                    first_cluster <= {cat_buf[15], cat_buf[14], cat_buf[13], cat_buf[12]};
                    file_size     <= {cat_buf[19], cat_buf[18], cat_buf[17], cat_buf[16]};
                    sd_image_first_cluster <= {cat_buf[15], cat_buf[14], cat_buf[13], cat_buf[12]};
                    current_cluster  <= {cat_buf[15], cat_buf[14], cat_buf[13], cat_buf[12]};
                    chain_idx        <= 10'd0;
                    sector_in_cluster <= 9'd0;
                    sdram_write_addr <= {1'b0, UNIT2_OFFSET, 9'd0};
                    bytes_remaining  <= {cat_buf[19], cat_buf[18], cat_buf[17], cat_buf[16]};
                    first_sector     <= 1'b1;
                    skip_bytes       <= 16'd0;
                    state <= CLAIM;
                end

                CLAIM: begin
                    // dbg_fat_entry set in PARSE_CAT with magic 0xBEEF
                    sdram_claim   <= 1'b1;
                    chain_wr_addr <= chain_idx;
                    chain_wr_data <= current_cluster;
                    chain_wr_en   <= 1'b1;
                    chain_idx     <= chain_idx + 10'd1;
                    state         <= READ_SECTOR;  // DEBUG: read one sector, drain only
                end

                // ---- Start reading a sector from SD ----
                READ_SECTOR: begin
                    sd_read_addr       <= cluster_sector + {23'd0, sector_in_cluster};
                    sd_read_start      <= 1'b1;
                    sd_read_data_ready <= 1'b1;
                    byte_in_sector     <= 10'd0;
                    state              <= COPY_LO;  // real copy path
                end

                // ---- Capture low byte from SD stream ----
                // Level-based handshake: keep sd_read_data_ready=1 always.
                // valid stays HIGH until we ack. No new byte starts until
                // valid clears. Flow control is automatic.
                COPY_LO: begin
                    sdram_req <= 1'b0;
                    sd_read_data_ready <= 1'b1;
                    // Only capture when both valid AND ready — ensures ack fires
                    // and valid clears, preventing double-read on state transitions
                    if (sd_read_data_valid && sd_read_data_ready) begin
                        lo_byte        <= sd_read_data;
                        byte_in_sector <= byte_in_sector + 10'd1;

                        // Capture .2mg header bytes inline (first sector only)
                        if (first_sector) begin
                            case (byte_in_sector)
                                10'd0:  hdr0  <= sd_read_data;
                                10'd1:  hdr1  <= sd_read_data;
                                10'd2:  hdr2  <= sd_read_data;
                                10'd3:  hdr3  <= sd_read_data;
                                10'd20: hdr20 <= sd_read_data;
                                10'd21: hdr21 <= sd_read_data;
                                10'd24: hdr24 <= sd_read_data;
                                10'd25: hdr25 <= sd_read_data;
                                10'd28: hdr28 <= sd_read_data;
                                10'd29: hdr29 <= sd_read_data;
                                10'd30: hdr30 <= sd_read_data;
                                10'd31: hdr31 <= sd_read_data;
                            endcase
                        end

                        // Skip .2mg header bytes
                        if (skip_bytes > 16'd0) begin
                            skip_bytes      <= skip_bytes - 16'd1;
                            bytes_remaining <= bytes_remaining - 32'd1;
                        end else begin
                            state <= COPY_HI;
                        end
                    end

                    // Sector complete (sd_read_done fires after 512 data + 2 CRC)
                    if (sd_read_done) begin
                        // Check .2mg on first sector completion
                        if (first_sector) begin
                            first_sector <= 1'b0;
                            if (hdr0 == 8'h32 && hdr1 == 8'h49 &&
                                hdr2 == 8'h4D && hdr3 == 8'h47) begin
                                is_2mg           <= 1'b1;
                                data_offset      <= {hdr25, hdr24};
                                s4d2_block_count <= {hdr21, hdr20};
                                skip_bytes       <= {hdr25, hdr24};
                                bytes_remaining  <= {hdr31, hdr30, hdr29, hdr28};
                            end
                        end
                        state <= NEXT_SECTOR;
                    end
                end

                // ---- Capture high byte, pack word ----
                COPY_HI: begin
                    sd_read_data_ready <= 1'b1;
                    if (sd_read_data_valid && sd_read_data_ready) begin
                        byte_in_sector <= byte_in_sector + 10'd1;

                        // Skip .2mg header bytes
                        if (skip_bytes > 16'd0) begin
                            skip_bytes      <= skip_bytes - 16'd1;
                            bytes_remaining <= bytes_remaining - 32'd1;
                            state           <= COPY_LO;
                        end else begin
                            // Real SDRAM write
                            sdram_req_wdata <= {sd_read_data, lo_byte};
                            sdram_req_addr  <= sdram_write_addr;
                            sdram_req       <= 1'b1;
                            sdram_req_write <= 1'b1;
                            bytes_remaining <= bytes_remaining - 32'd2;
                            state           <= WRITE_SDRAM;
                        end
                    end

                    if (sd_read_done) begin
                        // Odd byte at sector end — pad and write
                        if (bytes_remaining > 32'd0) begin
                            sdram_req_wdata <= {8'h00, lo_byte};
                            sdram_req_addr  <= sdram_write_addr;
                            sdram_req       <= 1'b1;
                            sdram_req_write <= 1'b1;
                            bytes_remaining <= bytes_remaining - 32'd1;
                            state           <= WRITE_SDRAM;
                        end else begin
                            state <= NEXT_SECTOR;
                        end
                    end
                end

                // ---- Wait for SDRAM accept ----
                WRITE_SDRAM: begin
                    sd_read_data_ready <= 1'b0;  // CRITICAL: pause SPI to prevent byte leak
                    if (sdram_req_ready) begin
                        sdram_req        <= 1'b0;
                        sdram_write_addr <= sdram_write_addr + 26'd2;
                        state            <= COPY_LO;
                    end
                end

                // ---- Next sector / cluster ----
                NEXT_SECTOR: begin
                    sdram_req <= 1'b0;
                    dbg_sector_count <= dbg_sector_count + 16'd1;
                    if (bytes_remaining == 32'd0) begin
                        state <= DONE;
                    end else if (sector_in_cluster + 9'd1 < {1'b0, fat32_spc}) begin
                        sector_in_cluster <= sector_in_cluster + 9'd1;
                        state             <= READ_SECTOR;
                    end else begin
                        state <= NEXT_CLUSTER;
                    end
                end

                // ---- Follow FAT chain ----
                // FAT16: 2 bytes/entry, 256 entries/sector
                // FAT32: 4 bytes/entry, 128 entries/sector
                NEXT_CLUSTER: begin
                    // Drain any stale read_data_valid before starting FAT read
                    sd_read_data_ready <= 1'b1;  // ack any pending byte
                    if (!sd_read_data_valid) begin
                        // Clean — start FAT sector read
                        if (fat_is_fat16) begin
                            sd_read_addr <= fat32_fat_start + (current_cluster >> 8);
                            fat_target_offset <= {current_cluster[7:0], 1'b0};  // *2
                        end else begin
                            sd_read_addr <= fat32_fat_start + (current_cluster >> 7);
                            fat_target_offset <= {current_cluster[6:0], 2'b00}; // *4
                        end
                        fat_collect_idx <= 10'd0;
                        fat_entry       <= 32'd0;
                        sd_read_start   <= 1'b1;
                        state           <= FAT_RECV;
                    end
                end

                FAT_RECV: begin
                    sd_read_data_ready <= 1'b1;
                    if (sd_read_data_valid && sd_read_data_ready) begin
                        if (fat_is_fat16) begin
                            // FAT16: 2-byte entries
                            if (fat_collect_idx >= fat_target_offset &&
                                fat_collect_idx < fat_target_offset + 10'd2) begin
                                fat_entry <= {16'd0, sd_read_data, fat_entry[15:8]};
                            end
                        end else begin
                            // FAT32: 4-byte entries
                            if (fat_collect_idx >= fat_target_offset &&
                                fat_collect_idx < fat_target_offset + 10'd4) begin
                                fat_entry <= {sd_read_data, fat_entry[31:8]};
                            end
                        end
                        fat_collect_idx <= fat_collect_idx + 10'd1;
                    end
                    if (sd_read_done)
                        state <= FAT_DONE;
                end

                FAT_DONE: begin
                    dbg_fat_entry <= fat_entry[15:0];  // DEBUG: show FAT entry VALUE read from SD
                    // End-of-chain: FAT16 >= 0xFFF8, FAT32 >= 0x0FFFFFF8
                    if (fat_is_fat16 ? (fat_entry[15:0] >= 16'hFFF8) :
                                       ((fat_entry & 32'h0FFFFFFF) >= 32'h0FFFFFF8)) begin
                        state <= DONE;
                    end else begin
                        current_cluster   <= fat_is_fat16 ? {16'd0, fat_entry[15:0]} :
                                                            (fat_entry & 32'h0FFFFFFF);
                        sector_in_cluster <= 9'd0;
                        chain_wr_addr     <= chain_idx;
                        chain_wr_data     <= fat_is_fat16 ? {16'd0, fat_entry[15:0]} :
                                                            (fat_entry & 32'h0FFFFFFF);
                        chain_wr_en       <= 1'b1;
                        chain_idx         <= chain_idx + 10'd1;
                        state             <= READ_SECTOR;
                    end
                end

                // DEBUG: drain one sector then go to DONE
                5'd14: begin
                    sd_read_data_ready <= 1'b1;
                    if (sd_read_done)
                        state <= DONE;
                end

                DONE: begin
                    sdram_claim  <= 1'b0;
                    s4d2_loading <= 1'b0;
                    s4d2_mounted <= 1'b1;
                    // dbg_fat_entry captured in CLAIM (initial value)
                    if (!is_2mg)
                        s4d2_block_count <= file_size[24:9];
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
