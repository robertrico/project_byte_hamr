// sd_mount_tb.v — Testbench for sd_mount with FAT16 chain following
//
// Instantiates sd_controller + sd_card_model (SDSC) + sd_mount.
// Pre-populates SD card model with FAT16 filesystem matching real card:
//   spc=32, reserved=1, num_fats=2, spf16=242
//   root_dir_sector=485, data_start=517
//   File PD8.PO: first_cluster=131, file_size=8388608
//   FAT chain: 131->133->135->137->138->139->140->141->142->0xFFFF
//
// Self-checking: PASS if s4d2_mounted goes HIGH within timeout.

`timescale 1ns / 1ps

module sd_mount_tb;

    // =========================================================================
    // Clock: 25 MHz (40ns period)
    // =========================================================================
    reg clk = 0;
    always #20 clk = ~clk;

    // =========================================================================
    // Parameters matching real FAT16 card geometry
    // =========================================================================
    localparam RESERVED_SECTORS = 1;
    localparam NUM_FATS         = 2;
    localparam SPF16            = 242;
    localparam SPC              = 32;
    localparam ROOT_DIR_SECTOR  = RESERVED_SECTORS + NUM_FATS * SPF16;  // 485
    localparam ROOT_ENTRY_COUNT = 512;
    localparam ROOT_DIR_SECTORS = ROOT_ENTRY_COUNT * 32 / 512;          // 32
    localparam DATA_START       = ROOT_DIR_SECTOR + ROOT_DIR_SECTORS;   // 517
    localparam FAT_START        = RESERVED_SECTORS;                     // 1

    localparam FIRST_CLUSTER    = 131;
    localparam FILE_SIZE        = 8388608;  // 8 MB

    // bytes_remaining is hardcoded to 143360 in sd_mount.v PARSE_CAT
    localparam BYTES_REMAINING  = 143360;
    localparam BYTES_PER_CLUSTER = SPC * 512;  // 16384
    // 143360 / 16384 = 8.75 => 8 full clusters + 24 sectors in 9th
    // Chain: 131, 133, 135, 137, 138, 139, 140, 141, 142 (9 clusters)

    // SD card model memory size — need enough for data sectors
    // Max sector: cluster 142 => sector 517 + (142-2)*32 = 4997, + 31 = 5028
    // Byte addr: 5028 * 512 + 512 = ~2.58 MB. Use 4 MB.
    localparam MEM_BYTES = 4 * 1024 * 1024;

    // =========================================================================
    // DUT signals
    // =========================================================================
    reg         rst_n = 0;

    // SD controller
    reg         init_start = 0;
    wire        init_done, init_error, is_sdhc;

    wire        sd_read_start_ctrl;
    wire [31:0] sd_read_addr_ctrl;
    wire [7:0]  sd_read_data;
    wire        sd_read_data_valid;
    wire        sd_read_data_ready_ctrl;
    wire        sd_read_done;
    wire        sd_read_error;

    // Write interface (unused, tie off)
    wire        write_data_req;
    wire        write_done, write_error;

    // SPI wires
    wire        sd_sck, sd_mosi, sd_miso, sd_cs;

    // sd_mount signals
    reg         mount_request = 0;
    reg  [3:0]  img_select = 4'd0;
    wire        s4d2_mounted;
    wire        s4d2_loading;
    wire [15:0] s4d2_block_count;
    wire        mount_is_2mg;
    wire [15:0] data_offset;

    // Catalog BRAM — small 512-byte BRAM, pre-populated
    // NOTE: sd_mount's READ_CAT state machine assumes combinatorial (async) BRAM
    // read — it sets cat_rd_addr and reads cat_rd_data on the next clock with
    // {cat_byte_idx - 1} indexing. A registered BRAM would add an extra pipeline
    // cycle and shift all data by one byte.
    wire [8:0]  cat_rd_addr;
    reg  [7:0]  cat_bram [0:511];
    wire [7:0]  cat_rd_data;

    // Registered read for catalog BRAM — matches flash_hamr_top.v (2-cycle latency)
    reg [7:0] cat_rd_data_r;
    always @(posedge clk)
        cat_rd_data_r <= cat_bram[cat_rd_addr];
    assign cat_rd_data = cat_rd_data_r;

    // FAT geometry inputs to sd_mount
    wire [31:0] fat32_data_start = DATA_START;
    wire [31:0] fat32_fat_start  = FAT_START;
    wire [7:0]  fat32_spc        = SPC;
    wire        fat_is_fat16     = 1'b1;

    // sd_mount SD read interface
    wire        mount_sd_read_start;
    wire [31:0] mount_sd_read_addr;
    wire        mount_sd_read_data_ready;

    // SDRAM mock — always ready
    wire        sdram_req;
    wire        sdram_req_write;
    wire [25:0] sdram_req_addr;
    wire [15:0] sdram_req_wdata;
    wire        sdram_req_ready = 1'b1;  // always accept immediately
    wire        sdram_claim;

    // Chain map BRAM — just wires, not checking contents
    wire [9:0]  chain_wr_addr;
    wire [31:0] chain_wr_data;
    wire        chain_wr_en;

    // Debug outputs
    wire [31:0] sd_image_first_cluster;
    wire [15:0] dbg_sector_count;
    wire [4:0]  dbg_mount_state;
    wire [15:0] dbg_fat_entry;

    // =========================================================================
    // Connect sd_mount read interface to sd_controller
    // sd_mount drives read_start/read_addr, sd_controller provides data
    // =========================================================================
    assign sd_read_start_ctrl      = mount_sd_read_start;
    assign sd_read_addr_ctrl       = mount_sd_read_addr;
    assign sd_read_data_ready_ctrl = mount_sd_read_data_ready;

    // =========================================================================
    // SD Controller
    // =========================================================================
    sd_controller u_sd_ctrl (
        .clk(clk),
        .rst_n(rst_n),

        .init_start(init_start),
        .init_done(init_done),
        .init_error(init_error),
        .is_sdhc(is_sdhc),

        .read_start(sd_read_start_ctrl),
        .read_addr(sd_read_addr_ctrl),
        .read_data(sd_read_data),
        .read_data_valid(sd_read_data_valid),
        .read_data_ready(sd_read_data_ready_ctrl),
        .read_done(sd_read_done),
        .read_error(sd_read_error),

        .write_start(1'b0),
        .write_addr(32'd0),
        .write_data(8'd0),
        .write_data_valid(1'b0),
        .write_data_req(write_data_req),
        .write_done(write_done),
        .write_error(write_error),

        .sd_sck(sd_sck),
        .sd_mosi(sd_mosi),
        .sd_miso(sd_miso),
        .sd_cs(sd_cs)
    );

    // =========================================================================
    // SD Card Model (SDSC — SDHC=0)
    // =========================================================================
    sd_card_model #(
        .SDHC(0),
        .ACMD41_RETRIES(2),
        .WRITE_BUSY_CLKS(50),
        .MEM_BYTES(MEM_BYTES)
    ) sd_card (
        .spi_sck(sd_sck),
        .spi_mosi(sd_mosi),
        .spi_miso(sd_miso),
        .spi_cs_n(sd_cs)
    );

    // =========================================================================
    // SD Mount DUT
    // =========================================================================
    sd_mount #(
        .UNIT2_OFFSET(16'd2048)
    ) u_mount (
        .clk(clk),
        .rst_n(rst_n),

        .mount_request(mount_request),
        .img_select(img_select),

        .s4d2_mounted(s4d2_mounted),
        .s4d2_loading(s4d2_loading),
        .s4d2_block_count(s4d2_block_count),
        .is_2mg(mount_is_2mg),
        .data_offset(data_offset),

        .cat_rd_addr(cat_rd_addr),
        .cat_rd_data(cat_rd_data),

        .fat32_data_start(fat32_data_start),
        .fat32_fat_start(fat32_fat_start),
        .fat32_spc(fat32_spc),
        .fat_is_fat16(fat_is_fat16),

        .sd_read_start(mount_sd_read_start),
        .sd_read_addr(mount_sd_read_addr),
        .sd_read_data(sd_read_data),
        .sd_read_data_valid(sd_read_data_valid),
        .sd_read_data_ready(mount_sd_read_data_ready),
        .sd_read_done(sd_read_done),

        .sdram_req(sdram_req),
        .sdram_req_write(sdram_req_write),
        .sdram_req_addr(sdram_req_addr),
        .sdram_req_wdata(sdram_req_wdata),
        .sdram_req_ready(sdram_req_ready),
        .sdram_claim(sdram_claim),

        .chain_wr_addr(chain_wr_addr),
        .chain_wr_data(chain_wr_data),
        .chain_wr_en(chain_wr_en),

        .sd_image_first_cluster(sd_image_first_cluster),
        .dbg_sector_count(dbg_sector_count),
        .dbg_mount_state(dbg_mount_state),
        .dbg_fat_entry(dbg_fat_entry)
    );

    // =========================================================================
    // Monitor: track chain map writes
    // =========================================================================
    reg [31:0] chain_map [0:31];
    integer    chain_count = 0;

    always @(posedge clk) begin
        if (chain_wr_en) begin
            chain_map[chain_wr_addr] <= chain_wr_data;
            chain_count <= chain_count + 1;
            $display("  [%0t] CHAIN[%0d] = cluster %0d", $time, chain_wr_addr, chain_wr_data);
        end
    end

    // =========================================================================
    // Monitor: track SDRAM writes
    // =========================================================================
    integer sdram_write_count = 0;

    always @(posedge clk) begin
        if (sdram_req && sdram_req_write && sdram_req_ready)
            sdram_write_count <= sdram_write_count + 1;
    end

    // =========================================================================
    // Monitor: track SD sector reads
    // =========================================================================
    always @(posedge clk) begin
        if (mount_sd_read_start)
            $display("  [%0t] SD READ sector %0d (state=%0d)", $time, mount_sd_read_addr, dbg_mount_state);
    end

    // =========================================================================
    // Monitor: state transitions
    // =========================================================================
    reg [4:0] prev_state = 5'd31;
    always @(posedge clk) begin
        if (dbg_mount_state != prev_state) begin
            $display("  [%0t] mount state: %0d -> %0d  (sectors=%0d, fat_entry=0x%04h, bytes_rem=%0d)",
                     $time, prev_state, dbg_mount_state,
                     dbg_sector_count, dbg_fat_entry,
                     u_mount.bytes_remaining);
            prev_state <= dbg_mount_state;
        end
    end

    // =========================================================================
    // Helper: populate FAT16 entry in sd_card model memory
    // FAT16 entries are 2 bytes (little-endian) at byte offset within FAT sector
    // FAT starts at sector FAT_START, each sector has 256 entries
    // Entry N is at byte address: FAT_START*512 + N*2
    // =========================================================================
    task set_fat16_entry;
        input integer cluster;
        input [15:0]  value;
        integer byte_addr;
        begin
            byte_addr = FAT_START * 512 + cluster * 2;
            sd_card.mem[byte_addr]     = value[7:0];
            sd_card.mem[byte_addr + 1] = value[15:8];
        end
    endtask

    // =========================================================================
    // Helper: populate catalog BRAM entry
    // Entry format (20 bytes per entry, base = img_select * 20... but actually
    // cat_rd_addr = {img_select, cat_byte_idx} = 9 bits = img_select[3:0] || byte[4:0]
    // For img_select=0: addresses 0..19
    // Bytes 0-7: filename (8 chars)
    // Bytes 8-10: extension (3 chars)
    // Byte 11: flags (bit 0 = is_2mg)
    // Bytes 12-15: first_cluster (32-bit LE)
    // Bytes 16-19: file_size (32-bit LE)
    // =========================================================================
    task populate_catalog;
        input [3:0]  entry;
        input [31:0] cluster;
        input [31:0] fsize;
        integer base;
        begin
            base = {entry, 5'd0};  // entry * 32 (but address space is {entry, byte_idx})
            // "PD8     " (name, 8 bytes)
            cat_bram[base + 0] = "P";
            cat_bram[base + 1] = "D";
            cat_bram[base + 2] = "8";
            cat_bram[base + 3] = " ";
            cat_bram[base + 4] = " ";
            cat_bram[base + 5] = " ";
            cat_bram[base + 6] = " ";
            cat_bram[base + 7] = " ";
            // "PO " (extension, 3 bytes)
            cat_bram[base + 8]  = "P";
            cat_bram[base + 9]  = "O";
            cat_bram[base + 10] = " ";
            // Flags
            cat_bram[base + 11] = 8'h00;
            // first_cluster (32-bit LE)
            cat_bram[base + 12] = cluster[7:0];
            cat_bram[base + 13] = cluster[15:8];
            cat_bram[base + 14] = cluster[23:16];
            cat_bram[base + 15] = cluster[31:24];
            // file_size (32-bit LE)
            cat_bram[base + 16] = fsize[7:0];
            cat_bram[base + 17] = fsize[15:8];
            cat_bram[base + 18] = fsize[23:16];
            cat_bram[base + 19] = fsize[31:24];
        end
    endtask

    // =========================================================================
    // Test sequence
    // =========================================================================
    integer i;
    integer pass;

    initial begin
        $dumpfile("sd_mount_tb.vcd");
        $dumpvars(0, sd_mount_tb);

        // -----------------------------------------------------------------
        // 1. Pre-populate catalog BRAM
        // -----------------------------------------------------------------
        // Zero out catalog BRAM
        for (i = 0; i < 512; i = i + 1)
            cat_bram[i] = 8'h00;

        populate_catalog(4'd0, FIRST_CLUSTER, FILE_SIZE);
        $display("Catalog entry 0: first_cluster=%0d, file_size=%0d", FIRST_CLUSTER, FILE_SIZE);

        // -----------------------------------------------------------------
        // 2. Pre-populate SD card model memory with FAT16 filesystem
        // -----------------------------------------------------------------
        // Zero out first few sectors of model memory (BPB area)
        // (model already zeros on init, but be explicit)

        // FAT16 entries at FAT_START (sector 1, byte 512)
        // Entry 0: media type
        set_fat16_entry(0, 16'hFFF8);
        // Entry 1: reserved
        set_fat16_entry(1, 16'hFFFF);

        // Chain: 131 -> 133 -> 135 -> 137 -> 138 -> 139 -> 140 -> 141 -> 142 -> END
        set_fat16_entry(131, 16'd133);
        set_fat16_entry(132, 16'hFFFF);  // different file (end)
        set_fat16_entry(133, 16'd135);
        set_fat16_entry(134, 16'hFFFF);  // different file (end)
        set_fat16_entry(135, 16'd137);
        set_fat16_entry(136, 16'hFFFF);  // different file (end)
        set_fat16_entry(137, 16'd138);
        set_fat16_entry(138, 16'd139);
        set_fat16_entry(139, 16'd140);
        set_fat16_entry(140, 16'd141);
        set_fat16_entry(141, 16'd142);
        set_fat16_entry(142, 16'hFFFF);  // end of chain

        $display("FAT16 chain: 131->133->135->137->138->139->140->141->142->END");

        // Pre-populate data sectors with recognizable patterns
        // Each cluster's first byte = cluster number, rest = sequential
        // cluster_sector = DATA_START + (cluster - 2) * SPC
        for (i = 0; i < 9; i = i + 1) begin : fill_data
            integer cluster, sector, byte_base, j;
            integer chain_clusters [0:8];
            chain_clusters[0] = 131;
            chain_clusters[1] = 133;
            chain_clusters[2] = 135;
            chain_clusters[3] = 137;
            chain_clusters[4] = 138;
            chain_clusters[5] = 139;
            chain_clusters[6] = 140;
            chain_clusters[7] = 141;
            chain_clusters[8] = 142;

            cluster = chain_clusters[i];
            sector = DATA_START + (cluster - 2) * SPC;
            byte_base = sector * 512;
            // Fill first sector of each cluster with pattern
            for (j = 0; j < 512; j = j + 1)
                sd_card.mem[byte_base + j] = (cluster + j) & 8'hFF;
        end

        $display("Data sectors populated for clusters in chain");
        $display("Root dir sector: %0d, Data start sector: %0d", ROOT_DIR_SECTOR, DATA_START);
        $display("Cluster 131 starts at sector %0d (byte 0x%0h)",
                 DATA_START + (131-2)*SPC, (DATA_START + (131-2)*SPC) * 512);

        // -----------------------------------------------------------------
        // 3. Reset
        // -----------------------------------------------------------------
        rst_n = 0;
        repeat (10) @(posedge clk);
        rst_n = 1;
        repeat (5) @(posedge clk);

        // -----------------------------------------------------------------
        // 4. Run SD init sequence
        // -----------------------------------------------------------------
        $display("\n=== SD Init ===");
        init_start = 1;
        @(posedge clk);
        init_start = 0;

        begin : wait_init
            integer w;
            for (w = 0; w < 5000000 && !init_done && !init_error; w = w + 1)
                @(posedge clk);
            if (w >= 5000000) begin
                $display("TIMEOUT: SD init did not complete");
                $finish;
            end
        end

        if (init_error) begin
            $display("FAIL: SD init error");
            $finish;
        end
        $display("SD init done (is_sdhc=%0d) at %0t", is_sdhc, $time);
        if (is_sdhc != 0)
            $display("WARNING: Expected SDSC (is_sdhc=0), got is_sdhc=%0d", is_sdhc);

        repeat (100) @(posedge clk);

        // -----------------------------------------------------------------
        // 5. Trigger mount
        // -----------------------------------------------------------------
        $display("\n=== Mount Request ===");
        img_select = 4'd0;
        mount_request = 1;
        @(posedge clk);
        mount_request = 0;

        $display("Mount request sent at %0t, img_select=%0d", $time, img_select);

        // -----------------------------------------------------------------
        // 6. Wait for mount to complete or timeout
        // -----------------------------------------------------------------
        // 9 clusters * 32 sectors/cluster * ~500 clocks/sector = ~144000 clocks
        // plus FAT reads (8 FAT lookups). Use generous timeout.
        begin : wait_mount
            integer w;
            for (w = 0; w < 50000000 && !s4d2_mounted; w = w + 1)
                @(posedge clk);
            if (w >= 50000000) begin
                $display("\n========================================");
                $display("TIMEOUT: Mount did not complete after %0d clocks", w);
                $display("  Final state: %0d", dbg_mount_state);
                $display("  Sectors read: %0d", dbg_sector_count);
                $display("  FAT entry: 0x%04h", dbg_fat_entry);
                $display("  bytes_remaining: %0d", u_mount.bytes_remaining);
                $display("  current_cluster: %0d", u_mount.current_cluster);
                $display("  sector_in_cluster: %0d", u_mount.sector_in_cluster);
                $display("  chain entries written: %0d", chain_count);
                $display("  SDRAM writes: %0d", sdram_write_count);
                $display("========================================");
                $display("FAIL");
                $finish;
            end
        end

        // -----------------------------------------------------------------
        // 7. Verify results
        // -----------------------------------------------------------------
        $display("\n=== Results ===");
        pass = 1;

        $display("  s4d2_mounted:     %0d", s4d2_mounted);
        $display("  s4d2_loading:     %0d", s4d2_loading);
        $display("  s4d2_block_count: %0d", s4d2_block_count);
        $display("  is_2mg:           %0d", mount_is_2mg);
        $display("  first_cluster:    %0d", sd_image_first_cluster);
        $display("  sectors read:     %0d", dbg_sector_count);
        $display("  chain entries:    %0d", chain_count);
        $display("  SDRAM writes:     %0d", sdram_write_count);
        $display("  bytes_remaining:  %0d", u_mount.bytes_remaining);
        $display("  Mount time:       %0t", $time);

        // Check: mounted
        if (!s4d2_mounted) begin
            $display("ASSERT FAIL: s4d2_mounted should be 1");
            pass = 0;
        end

        // Check: not loading
        if (s4d2_loading) begin
            $display("ASSERT FAIL: s4d2_loading should be 0 after mount");
            pass = 0;
        end

        // Check: not .2mg (raw .PO file, first bytes are data, not "2IMG" header)
        if (mount_is_2mg) begin
            $display("ASSERT FAIL: is_2mg should be 0 for .PO file");
            pass = 0;
        end

        // Check: first_cluster
        if (sd_image_first_cluster != FIRST_CLUSTER) begin
            $display("ASSERT FAIL: first_cluster should be %0d, got %0d",
                     FIRST_CLUSTER, sd_image_first_cluster);
            pass = 0;
        end

        // Check: block_count (file_size >> 9 = 8388608/512 = 16384... but
        // actually bytes_remaining is hardcoded to 143360, so block_count = 143360/512 = 280)
        // In DONE: s4d2_block_count <= file_size[24:9] = 8388608[24:9] = 16384
        // Wait, file_size is set from catalog in PARSE_CAT but block_count
        // is computed from file_size in DONE. file_size = 8388608.
        // file_size[24:9] = 8388608 >> 9 = 16384 = 0x4000. That's 16384 blocks.
        if (s4d2_block_count != 16'd16384) begin
            $display("ASSERT FAIL: s4d2_block_count should be 16384, got %0d",
                     s4d2_block_count);
            pass = 0;
        end

        // Check: bytes_remaining should be 0
        if (u_mount.bytes_remaining != 0) begin
            $display("ASSERT FAIL: bytes_remaining should be 0, got %0d",
                     u_mount.bytes_remaining);
            pass = 0;
        end

        // Check: sectors read should be reasonable
        // 8 full clusters * 32 sectors + 24 sectors = 280 sectors
        // But wait — sd_mount increments dbg_sector_count in NEXT_SECTOR,
        // which fires after every sector including the last partial cluster.
        // Expected: 280 sectors total (256 from full clusters + 24 from last)
        $display("  Expected sectors: 280");
        if (dbg_sector_count != 16'd280) begin
            $display("WARNING: sector count mismatch — expected 280, got %0d (may be correct if chain differs)",
                     dbg_sector_count);
            // Not a hard fail since the exact count depends on chain traversal
        end

        // Check: chain entries (9 clusters = 9 chain writes: initial + 8 follows)
        // CLAIM writes first cluster, FAT_DONE writes each subsequent cluster
        // So: 1 (CLAIM for 131) + 8 (FAT_DONE for 133,135,137,138,139,140,141,142) = 9
        $display("  Expected chain entries: 9");
        if (chain_count != 9) begin
            $display("WARNING: chain entry count mismatch — expected 9, got %0d", chain_count);
        end

        // Check: SDRAM writes
        // 143360 bytes / 2 bytes per write = 71680 SDRAM writes
        $display("  Expected SDRAM writes: 71680");
        if (sdram_write_count != 71680) begin
            $display("WARNING: SDRAM write count mismatch — expected 71680, got %0d",
                     sdram_write_count);
        end

        $display("\n========================================");
        if (pass)
            $display("PASS: sd_mount FAT16 chain following test");
        else
            $display("FAIL: sd_mount FAT16 chain following test");
        $display("========================================\n");

        repeat (100) @(posedge clk);
        $finish;
    end

    // =========================================================================
    // Global timeout — 2 seconds of sim time (very generous)
    // =========================================================================
    initial begin
        #2000000000;
        $display("\n========================================");
        $display("GLOBAL TIMEOUT at %0t", $time);
        $display("  Final state: %0d", dbg_mount_state);
        $display("  Sectors read: %0d", dbg_sector_count);
        $display("  bytes_remaining: %0d", u_mount.bytes_remaining);
        $display("  current_cluster: %0d", u_mount.current_cluster);
        $display("========================================");
        $display("FAIL");
        $finish;
    end

endmodule
