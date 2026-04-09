// fat32_reader.v — FAT32 filesystem parser for SD card
//
// Reads MBR → BPB → root directory. Populates catalog BRAM with
// .PO and .2MG file entries (8.3 names, cluster, size).
// Exports FAT32 geometry for sd_mount and sd_persist.
//
// Root directory only, 8.3 filenames only, max 15 files.
// Handles both MBR-partitioned and non-partitioned cards.

`timescale 1ns / 1ps

module fat32_reader (
    input  wire        clk,
    input  wire        rst_n,

    // Control
    input  wire        start,        // pulse: begin scanning
    output reg         done,         // scan complete
    output reg         error,        // filesystem error
    output reg  [3:0]  file_count,   // number of image files found

    // SD controller read interface
    output reg         sd_read_start,
    output reg  [31:0] sd_read_addr,
    input  wire [7:0]  sd_read_data,
    input  wire        sd_read_data_valid,
    output reg         sd_read_data_ready,
    input  wire        sd_read_done,
    input  wire        sd_read_error,

    // FAT32 geometry output (used by sd_mount / sd_persist)
    output reg [31:0]  fat32_partition_start,
    output reg [31:0]  fat32_fat_start,
    output reg [31:0]  fat32_data_start,
    output reg [7:0]   fat32_spc,           // sectors_per_cluster
    output reg [31:0]  fat32_root_cluster,

    // Catalog BRAM write port (32 bytes per entry × 15 entries = 480 bytes)
    output reg  [8:0]  cat_wr_addr,
    output reg  [7:0]  cat_wr_data,
    output reg         cat_wr_en,

    // Debug: first dir entry bytes (captured once, readable via bus regs)
    output reg  [7:0]  dbg_dir_byte0,   // first byte of first root dir entry
    output reg  [7:0]  dbg_dir_byte8,   // extension first char of first root dir entry
    output reg         dbg_is_fat16     // filesystem type detected
);

    // =========================================================================
    // Catalog entry layout (32 bytes each):
    //   [0-10]:  8.3 filename (11 chars)
    //   [11]:    flags (bit 0: is_2mg)
    //   [12-15]: first_cluster (uint32 LE)
    //   [16-19]: file_size (uint32 LE)
    //   [20-23]: reserved (data_offset, set during mount)
    //   [24-31]: reserved
    // =========================================================================
    localparam MAX_FILES = 4'd15;

    // =========================================================================
    // State machine
    // =========================================================================
    localparam [4:0]
        S_IDLE          = 5'd0,
        S_READ_MBR      = 5'd1,   // read SD block 0
        S_MBR_COLLECT   = 5'd2,   // collect 512 bytes, parse partition table
        S_READ_BPB      = 5'd3,   // read BPB at partition_start
        S_BPB_COLLECT   = 5'd4,   // collect 512 bytes, parse FAT32 fields
        S_BPB_COMPUTE   = 5'd5,   // compute fat_start, data_start
        S_READ_ROOTDIR  = 5'd6,   // read root directory cluster
        S_DIR_COLLECT   = 5'd7,   // collect 512 bytes of directory entries
        S_DIR_PARSE     = 5'd8,   // parse current 32-byte directory entry
        S_DRAIN         = 5'd13,  // drain remaining sector bytes before done
        S_DIR_STORE     = 5'd9,   // write entry to catalog BRAM
        S_DIR_NEXT      = 5'd10,  // advance to next entry
        S_DONE          = 5'd11,
        S_ERROR         = 5'd12;

    reg [4:0]  state = S_IDLE;

    // Byte capture buffer (512 bytes for current sector)
    // We only need specific bytes, so we capture selectively.
    reg [9:0]  byte_idx;           // 0..511 within current sector
    reg [8:0]  sector_in_cluster;  // which sector within root dir cluster

    // MBR fields
    reg [31:0] partition_lba;

    // BPB fields
    reg [15:0] reserved_sectors;
    reg [7:0]  num_fats;
    reg [31:0] sectors_per_fat;
    reg [31:0] root_cluster;
    reg [7:0]  spc;                // sectors_per_cluster

    // Directory parsing
    reg [4:0]  entry_offset;       // byte within 32-byte dir entry (0..31)
    reg [8:0]  entry_in_sector;    // which 32-byte entry in sector (0..15)
    reg [7:0]  dir_byte;           // current directory entry byte

    // Temp storage for current dir entry
    reg [87:0] entry_name;         // 11 bytes (8.3 filename), packed
    reg [7:0]  entry_attr;
    reg [15:0] entry_cluster_hi;
    reg [15:0] entry_cluster_lo;
    reg [31:0] entry_size;
    reg        entry_is_2mg;
    reg        entry_is_po;

    // Sector read helper
    reg        reading_sector;     // currently reading a 512-byte sector

    // =========================================================================
    // Capture specific BPB/MBR bytes as they stream in
    // =========================================================================
    // We read entire sectors but only latch the bytes we need.

    reg [7:0]  mbr_byte_446, mbr_byte_447, mbr_byte_448, mbr_byte_449;
    reg [7:0]  mbr_byte_450, mbr_byte_451, mbr_byte_452, mbr_byte_453;
    reg [7:0]  mbr_byte_454, mbr_byte_455, mbr_byte_456, mbr_byte_457;
    reg [7:0]  mbr_byte_510, mbr_byte_511;

    // BPB bytes we need
    reg [7:0]  bpb_byte_11, bpb_byte_12;   // bytes_per_sector
    reg [7:0]  bpb_byte_13;                 // sectors_per_cluster
    reg [7:0]  bpb_byte_14, bpb_byte_15;   // reserved_sectors
    reg [7:0]  bpb_byte_16;                 // num_fats
    reg [7:0]  bpb_byte_17, bpb_byte_18;   // root_entry_count (FAT16: non-zero)
    reg [7:0]  bpb_byte_22, bpb_byte_23;   // sectors_per_fat_16 (FAT16: non-zero)
    reg [7:0]  bpb_byte_36, bpb_byte_37, bpb_byte_38, bpb_byte_39; // sectors_per_fat_32
    reg [7:0]  bpb_byte_44, bpb_byte_45, bpb_byte_46, bpb_byte_47; // root_cluster (FAT32)
    reg [7:0]  bpb_byte_82, bpb_byte_83, bpb_byte_84, bpb_byte_85; // "FAT3" sig
    reg [7:0]  bpb_byte_510_b, bpb_byte_511_b;

    reg        is_fat16;  // detected FAT16 filesystem
    reg [31:0] root_dir_start;  // FAT16: fixed root dir sector

    // Directory entry bytes (captured during dir sector read)
    reg [7:0]  dir_bytes [0:31];

    // =========================================================================
    // Main state machine
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= S_IDLE;
            done            <= 1'b0;
            error           <= 1'b0;
            file_count      <= 4'd0;
            sd_read_start   <= 1'b0;
            sd_read_addr    <= 32'd0;
            sd_read_data_ready <= 1'b1;
            cat_wr_en       <= 1'b0;
            cat_wr_addr     <= 9'd0;
            cat_wr_data     <= 8'd0;
            reading_sector  <= 1'b0;
            byte_idx        <= 10'd0;
            dbg_dir_byte0   <= 8'd0;
            dbg_dir_byte8   <= 8'd0;
            dbg_is_fat16    <= 1'b0;
            fat32_partition_start <= 32'd0;
            fat32_fat_start <= 32'd0;
            fat32_data_start <= 32'd0;
            fat32_spc       <= 8'd0;
            fat32_root_cluster <= 32'd0;
        end else begin
            sd_read_start <= 1'b0;
            cat_wr_en     <= 1'b0;

            case (state)

                // =============================================================
                S_IDLE: begin
                    done  <= 1'b0;
                    error <= 1'b0;
                    if (start) begin
                        file_count <= 4'd0;
                        state      <= S_READ_MBR;
                    end
                end

                // =============================================================
                // Read block 0 (MBR or BPB)
                // =============================================================
                S_READ_MBR: begin
                    sd_read_addr       <= 32'd0;
                    sd_read_start      <= 1'b1;
                    sd_read_data_ready <= 1'b1;
                    byte_idx           <= 10'd0;
                    reading_sector     <= 1'b1;
                    state              <= S_MBR_COLLECT;
                end

                // Collect MBR bytes
                S_MBR_COLLECT: begin
                    sd_read_data_ready <= 1'b1;
                    if (sd_read_data_valid && sd_read_data_ready && reading_sector) begin
                        // Capture bytes of interest
                        case (byte_idx)
                            10'd446: mbr_byte_446 <= sd_read_data;
                            10'd447: mbr_byte_447 <= sd_read_data;
                            10'd448: mbr_byte_448 <= sd_read_data;
                            10'd449: mbr_byte_449 <= sd_read_data;
                            10'd450: mbr_byte_450 <= sd_read_data;
                            10'd451: mbr_byte_451 <= sd_read_data;
                            10'd452: mbr_byte_452 <= sd_read_data;
                            10'd453: mbr_byte_453 <= sd_read_data;
                            10'd454: mbr_byte_454 <= sd_read_data;
                            10'd455: mbr_byte_455 <= sd_read_data;
                            10'd456: mbr_byte_456 <= sd_read_data;
                            10'd457: mbr_byte_457 <= sd_read_data;
                            10'd510: mbr_byte_510 <= sd_read_data;
                            10'd511: mbr_byte_511 <= sd_read_data;
                            // Also capture BPB fields in case block 0 IS the BPB
                            10'd11:  bpb_byte_11  <= sd_read_data;
                            10'd12:  bpb_byte_12  <= sd_read_data;
                            10'd13:  bpb_byte_13  <= sd_read_data;
                            10'd14:  bpb_byte_14  <= sd_read_data;
                            10'd15:  bpb_byte_15  <= sd_read_data;
                            10'd16:  bpb_byte_16  <= sd_read_data;
                            10'd17:  bpb_byte_17  <= sd_read_data;
                            10'd18:  bpb_byte_18  <= sd_read_data;
                            10'd22:  bpb_byte_22  <= sd_read_data;
                            10'd23:  bpb_byte_23  <= sd_read_data;
                            10'd36:  bpb_byte_36  <= sd_read_data;
                            10'd37:  bpb_byte_37  <= sd_read_data;
                            10'd38:  bpb_byte_38  <= sd_read_data;
                            10'd39:  bpb_byte_39  <= sd_read_data;
                            10'd44:  bpb_byte_44  <= sd_read_data;
                            10'd45:  bpb_byte_45  <= sd_read_data;
                            10'd46:  bpb_byte_46  <= sd_read_data;
                            10'd47:  bpb_byte_47  <= sd_read_data;
                            10'd82:  bpb_byte_82  <= sd_read_data;
                            10'd83:  bpb_byte_83  <= sd_read_data;
                            10'd84:  bpb_byte_84  <= sd_read_data;
                            10'd85:  bpb_byte_85  <= sd_read_data;
                        endcase
                        byte_idx <= byte_idx + 10'd1;
                    end

                    if (sd_read_done) begin
                        reading_sector <= 1'b0;
                        // Check MBR: partition type 0x0B/0x0C (FAT32) or 0x04/0x06/0x0E (FAT16)
                        if (mbr_byte_510 == 8'h55 && mbr_byte_511 == 8'hAA) begin
                            if (mbr_byte_450 == 8'h0B || mbr_byte_450 == 8'h0C ||
                                mbr_byte_450 == 8'h04 || mbr_byte_450 == 8'h06 ||
                                mbr_byte_450 == 8'h0E) begin
                                // MBR with FAT partition (FAT16 or FAT32)
                                partition_lba <= {mbr_byte_457, mbr_byte_456,
                                                  mbr_byte_455, mbr_byte_454};
                                fat32_partition_start <= {mbr_byte_457, mbr_byte_456,
                                                         mbr_byte_455, mbr_byte_454};
                                state <= S_READ_BPB;
                            end else if (bpb_byte_82 == 8'h46 && bpb_byte_83 == 8'h41 &&
                                         bpb_byte_84 == 8'h54 && bpb_byte_85 == 8'h33) begin
                                // No MBR — BPB at block 0 with "FAT3" signature
                                partition_lba <= 32'd0;
                                fat32_partition_start <= 32'd0;
                                state <= S_BPB_COMPUTE;
                            end else begin
                                state <= S_ERROR;
                            end
                        end else begin
                            state <= S_ERROR;
                        end
                    end

                    if (sd_read_error) begin
                        state <= S_ERROR;
                    end
                end

                // =============================================================
                // Read BPB at partition_start
                // =============================================================
                S_READ_BPB: begin
                    sd_read_addr       <= partition_lba;
                    sd_read_start      <= 1'b1;
                    sd_read_data_ready <= 1'b1;
                    byte_idx           <= 10'd0;
                    reading_sector     <= 1'b1;
                    state              <= S_BPB_COLLECT;
                end

                S_BPB_COLLECT: begin
                    sd_read_data_ready <= 1'b1;
                    if (sd_read_data_valid && sd_read_data_ready && reading_sector) begin
                        case (byte_idx)
                            10'd11:  bpb_byte_11  <= sd_read_data;
                            10'd12:  bpb_byte_12  <= sd_read_data;
                            10'd13:  bpb_byte_13  <= sd_read_data;
                            10'd14:  bpb_byte_14  <= sd_read_data;
                            10'd15:  bpb_byte_15  <= sd_read_data;
                            10'd16:  bpb_byte_16  <= sd_read_data;
                            10'd17:  bpb_byte_17  <= sd_read_data;
                            10'd18:  bpb_byte_18  <= sd_read_data;
                            10'd22:  bpb_byte_22  <= sd_read_data;
                            10'd23:  bpb_byte_23  <= sd_read_data;
                            10'd36:  bpb_byte_36  <= sd_read_data;
                            10'd37:  bpb_byte_37  <= sd_read_data;
                            10'd38:  bpb_byte_38  <= sd_read_data;
                            10'd39:  bpb_byte_39  <= sd_read_data;
                            10'd44:  bpb_byte_44  <= sd_read_data;
                            10'd45:  bpb_byte_45  <= sd_read_data;
                            10'd46:  bpb_byte_46  <= sd_read_data;
                            10'd47:  bpb_byte_47  <= sd_read_data;
                            10'd510: bpb_byte_510_b <= sd_read_data;
                            10'd511: bpb_byte_511_b <= sd_read_data;
                        endcase
                        byte_idx <= byte_idx + 10'd1;
                    end

                    if (sd_read_done) begin
                        reading_sector <= 1'b0;
                        state <= S_BPB_COMPUTE;
                    end
                    if (sd_read_error) state <= S_ERROR;
                end

                // =============================================================
                // Compute FAT geometry (supports FAT16 and FAT32)
                // =============================================================
                S_BPB_COMPUTE: begin
                    spc      <= bpb_byte_13;
                    fat32_spc <= bpb_byte_13;

                    // Detect FAT16: root_entry_count != 0
                    is_fat16 <= ({bpb_byte_18, bpb_byte_17} != 16'd0);
                    dbg_is_fat16 <= ({bpb_byte_18, bpb_byte_17} != 16'd0);

                    // fat_start = partition_start + reserved_sectors
                    fat32_fat_start <= partition_lba + {16'd0, bpb_byte_15, bpb_byte_14};

                    if ({bpb_byte_18, bpb_byte_17} != 16'd0) begin
                        // ---- FAT16 ----
                        // sectors_per_fat from 16-bit field (offset 22-23)
                        // root_dir_start = fat_start + num_fats * sectors_per_fat_16
                        root_dir_start <= partition_lba +
                                          {16'd0, bpb_byte_15, bpb_byte_14} +
                                          ({16'd0, bpb_byte_23, bpb_byte_22} *
                                           {24'd0, bpb_byte_16});

                        // data_start = root_dir_start + root_dir_sectors
                        // root_dir_sectors = root_entry_count * 32 / 512 = root_entry_count >> 4
                        fat32_data_start <= partition_lba +
                                            {16'd0, bpb_byte_15, bpb_byte_14} +
                                            ({16'd0, bpb_byte_23, bpb_byte_22} *
                                             {24'd0, bpb_byte_16}) +
                                            {16'd0, 4'd0, bpb_byte_18, bpb_byte_17[7:4]};
                                            // root_entry_count >> 4

                        fat32_root_cluster <= 32'd0;  // not used for FAT16
                    end else begin
                        // ---- FAT32 ----
                        // sectors_per_fat from 32-bit field (offset 36-39)
                        fat32_data_start <= partition_lba +
                                            {16'd0, bpb_byte_15, bpb_byte_14} +
                                            ({bpb_byte_39, bpb_byte_38, bpb_byte_37, bpb_byte_36} *
                                             {24'd0, bpb_byte_16});

                        fat32_root_cluster <= {bpb_byte_47, bpb_byte_46, bpb_byte_45, bpb_byte_44};
                        root_dir_start <= 32'd0;  // not used for FAT32
                    end

                    if (bpb_byte_13 == 8'd0)
                        state <= S_ERROR;
                    else
                        state <= S_READ_ROOTDIR;
                end

                // =============================================================
                // Read root directory
                // FAT16: fixed location at root_dir_start
                // FAT32: cluster chain starting at root_cluster
                // =============================================================
                S_READ_ROOTDIR: begin
                    if (is_fat16)
                        sd_read_addr <= root_dir_start + {23'd0, sector_in_cluster};
                    else
                        sd_read_addr <= fat32_data_start +
                                    ((fat32_root_cluster - 32'd2) * {24'd0, fat32_spc}) +
                                    {23'd0, sector_in_cluster};
                    sd_read_start      <= 1'b1;
                    sd_read_data_ready <= 1'b1;
                    byte_idx           <= 10'd0;
                    entry_in_sector    <= 9'd0;
                    entry_offset       <= 5'd0;
                    reading_sector     <= 1'b1;
                    state              <= S_DIR_COLLECT;
                end

                // =============================================================
                // Collect directory sector bytes
                // =============================================================
                S_DIR_COLLECT: begin
                    sd_read_data_ready <= 1'b1;
                    if (sd_read_data_valid && sd_read_data_ready && reading_sector) begin
                        dir_bytes[entry_offset] <= sd_read_data;
                        entry_offset <= entry_offset + 5'd1;
                        byte_idx     <= byte_idx + 10'd1;

                        if (entry_offset == 5'd31) begin
                            // Got all 32 bytes — pause SD reads and parse
                            sd_read_data_ready <= 1'b0;
                            state <= S_DIR_PARSE;
                        end
                    end

                    if (sd_read_done && state == S_DIR_COLLECT) begin
                        reading_sector <= 1'b0;
                        // FAT16: scan root_dir_sectors. FAT32: scan one cluster.
                        if (is_fat16) begin
                            // root_dir_sectors = root_entry_count / 16
                            if (sector_in_cluster + 9'd1 <
                                {1'b0, bpb_byte_18, bpb_byte_17[7:4]}) begin
                                sector_in_cluster <= sector_in_cluster + 9'd1;
                                state <= S_READ_ROOTDIR;
                            end else begin
                                state <= S_DONE;
                            end
                        end else begin
                            if (sector_in_cluster + 9'd1 < {1'b0, fat32_spc}) begin
                                sector_in_cluster <= sector_in_cluster + 9'd1;
                                state <= S_READ_ROOTDIR;
                            end else begin
                                state <= S_DONE;
                            end
                        end
                    end
                    if (sd_read_error) state <= S_ERROR;
                end

                // =============================================================
                // Parse 32-byte directory entry
                // =============================================================
                S_DIR_PARSE: begin
                    // Capture debug bytes from very first entry
                    if (dbg_dir_byte0 == 8'd0 && dbg_dir_byte8 == 8'd0) begin
                        dbg_dir_byte0 <= dir_bytes[0];
                        dbg_dir_byte8 <= dir_bytes[8];
                    end
                    // Check first byte
                    if (dir_bytes[0] == 8'h00) begin
                        // End of directory — drain remaining sector bytes
                        // so sd_controller returns to IDLE
                        sd_read_data_ready <= 1'b1;
                        state <= S_DRAIN;
                    end else if (dir_bytes[0] == 8'hE5) begin
                        // Deleted entry — skip
                        state <= S_DIR_NEXT;
                    end else if (dir_bytes[11] == 8'h0F) begin
                        // LFN entry — skip
                        state <= S_DIR_NEXT;
                    end else if (dir_bytes[11][4] || dir_bytes[11][3]) begin
                        // Directory or volume label — skip
                        // bit 4 = directory, bit 3 = volume label
                        state <= S_DIR_NEXT;
                    end else begin
                        // Regular file — check extension
                        entry_is_po  <= (dir_bytes[8] == 8'h50 && dir_bytes[9] == 8'h4F &&
                                         dir_bytes[10] == 8'h20);   // "PO "
                        entry_is_2mg <= (dir_bytes[8] == 8'h32 && dir_bytes[9] == 8'h4D &&
                                         dir_bytes[10] == 8'h47);   // "2MG"

                        if ((dir_bytes[8] == 8'h50 && dir_bytes[9] == 8'h4F && dir_bytes[10] == 8'h20) ||
                            (dir_bytes[8] == 8'h32 && dir_bytes[9] == 8'h4D && dir_bytes[10] == 8'h47)) begin
                            // Image file — store to catalog
                            if (file_count < MAX_FILES)
                                state <= S_DIR_STORE;
                            else
                                state <= S_DIR_NEXT;  // catalog full
                        end else begin
                            state <= S_DIR_NEXT;  // not an image file
                        end
                    end
                end

                // =============================================================
                // Store entry to catalog BRAM (32 bytes)
                // =============================================================
                S_DIR_STORE: begin
                    cat_wr_addr <= {file_count, entry_offset};
                    cat_wr_en   <= 1'b1;

                    case (entry_offset)
                        5'd0:  cat_wr_data <= dir_bytes[0];
                        5'd1:  cat_wr_data <= dir_bytes[1];
                        5'd2:  cat_wr_data <= dir_bytes[2];
                        5'd3:  cat_wr_data <= dir_bytes[3];
                        5'd4:  cat_wr_data <= dir_bytes[4];
                        5'd5:  cat_wr_data <= dir_bytes[5];
                        5'd6:  cat_wr_data <= dir_bytes[6];
                        5'd7:  cat_wr_data <= dir_bytes[7];
                        5'd8:  cat_wr_data <= dir_bytes[8];
                        5'd9:  cat_wr_data <= dir_bytes[9];
                        5'd10: cat_wr_data <= dir_bytes[10];
                        5'd11: cat_wr_data <= {7'd0, entry_is_2mg};
                        5'd12: cat_wr_data <= dir_bytes[26];
                        5'd13: cat_wr_data <= dir_bytes[27];
                        5'd14: cat_wr_data <= dir_bytes[20];
                        5'd15: cat_wr_data <= dir_bytes[21];
                        5'd16: cat_wr_data <= dir_bytes[28];
                        5'd17: cat_wr_data <= dir_bytes[29];
                        5'd18: cat_wr_data <= dir_bytes[30];
                        5'd19: cat_wr_data <= dir_bytes[31];
                        default: cat_wr_data <= 8'd0;
                    endcase

                    entry_offset <= entry_offset + 5'd1;

                    if (entry_offset == 5'd31) begin
                        file_count <= file_count + 4'd1;
                        state      <= S_DIR_NEXT;
                    end
                end

                // =============================================================
                // Advance to next directory entry
                // =============================================================
                S_DIR_NEXT: begin
                    entry_offset    <= 5'd0;
                    entry_in_sector <= entry_in_sector + 9'd1;
                    sd_read_data_ready <= 1'b1;  // resume read

                    if (entry_in_sector >= 9'd15) begin
                        // 16 entries per sector (512/32), sector done
                        // Wait for sd_read_done in S_DIR_COLLECT
                        state <= S_DIR_COLLECT;
                    end else begin
                        state <= S_DIR_COLLECT;
                    end
                end

                // =============================================================
                // Drain remaining bytes from current CMD17 so
                // sd_controller returns to IDLE for sd_mount to use
                // =============================================================
                S_DRAIN: begin
                    sd_read_data_ready <= 1'b1;  // accept all bytes
                    if (sd_read_done) begin
                        state <= S_DONE;
                    end
                    if (sd_read_error) begin
                        state <= S_DONE;  // don't flag error, scan was ok
                    end
                end

                // =============================================================
                S_DONE: begin
                    done  <= 1'b1;
                    state <= S_IDLE;
                end

                S_ERROR: begin
                    error <= 1'b1;
                    done  <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // =========================================================================
    // Init
    // =========================================================================
    initial begin
        sector_in_cluster = 9'd0;
        entry_offset      = 5'd0;
        entry_in_sector   = 9'd0;
        partition_lba     = 32'd0;
    end

endmodule
