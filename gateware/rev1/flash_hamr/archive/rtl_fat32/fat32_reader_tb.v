// fat32_reader_tb.v — Testbench for fat32_reader
// Builds a FAT32 filesystem in sd_card_model memory, verifies parsing.

`timescale 1ns / 1ps

module fat32_reader_tb;

    reg clk = 0;
    always #20 clk = ~clk;  // 25 MHz

    reg rst_n = 0;
    reg start = 0;
    wire done, error;
    wire [3:0] file_count;

    // SD controller wires
    wire       sd_read_start;
    wire [31:0] sd_read_addr;
    wire [7:0] sd_read_data;
    wire       sd_read_data_valid;
    wire       sd_read_data_ready;
    wire       sd_read_done, sd_read_error;

    // FAT32 geometry
    wire [31:0] f32_part_start, f32_fat_start, f32_data_start, f32_root_cluster;
    wire [7:0]  f32_spc;

    // Catalog BRAM
    wire [8:0]  cat_wr_addr;
    wire [7:0]  cat_wr_data;
    wire        cat_wr_en;

    reg [7:0] catalog [0:511];  // catalog memory

    always @(posedge clk) begin
        if (cat_wr_en)
            catalog[cat_wr_addr] <= cat_wr_data;
    end

    // SD controller
    wire sd_sck, sd_mosi, sd_miso, sd_cs;

    reg ctrl_init_start = 0;
    wire ctrl_init_done;

    sd_controller u_sd_ctrl (
        .clk(clk), .rst_n(rst_n),
        .init_start(ctrl_init_start), .init_done(ctrl_init_done), .init_error(), .is_sdhc(),
        .read_start(sd_read_start), .read_addr(sd_read_addr),
        .read_data(sd_read_data), .read_data_valid(sd_read_data_valid),
        .read_data_ready(sd_read_data_ready), .read_done(sd_read_done),
        .read_error(sd_read_error),
        .write_start(1'b0), .write_addr(32'd0),
        .write_data(8'd0), .write_data_valid(1'b0),
        .write_data_req(), .write_done(), .write_error(),
        .sd_sck(sd_sck), .sd_mosi(sd_mosi), .sd_miso(sd_miso), .sd_cs(sd_cs)
    );

    sd_card_model #(.MEM_BYTES(2097152), .ACMD41_RETRIES(0)) u_sd_card (
        .spi_sck(sd_sck), .spi_mosi(sd_mosi), .spi_miso(sd_miso), .spi_cs_n(sd_cs)
    );

    fat32_reader dut (
        .clk(clk), .rst_n(rst_n),
        .start(start), .done(done), .error(error), .file_count(file_count),
        .sd_read_start(sd_read_start), .sd_read_addr(sd_read_addr),
        .sd_read_data(sd_read_data), .sd_read_data_valid(sd_read_data_valid),
        .sd_read_data_ready(sd_read_data_ready), .sd_read_done(sd_read_done),
        .sd_read_error(sd_read_error),
        .fat32_partition_start(f32_part_start), .fat32_fat_start(f32_fat_start),
        .fat32_data_start(f32_data_start), .fat32_spc(f32_spc),
        .fat32_root_cluster(f32_root_cluster),
        .cat_wr_addr(cat_wr_addr), .cat_wr_data(cat_wr_data), .cat_wr_en(cat_wr_en)
    );

    // =========================================================================
    // Scoreboard
    // =========================================================================
    integer tests_run = 0, tests_passed = 0, tests_failed = 0;

    task check;
        input [255:0] label;
        input [31:0] expected;
        input [31:0] actual;
        begin
            tests_run = tests_run + 1;
            if (expected === actual)
                tests_passed = tests_passed + 1;
            else begin
                tests_failed = tests_failed + 1;
                $display("FAIL: %0s — expected 0x%0h, got 0x%0h", label, expected, actual);
            end
        end
    endtask

    // =========================================================================
    // Helper: write a byte to SD card model memory
    // =========================================================================
    task sd_write_byte;
        input [31:0] addr;
        input [7:0]  data;
        begin
            u_sd_card.mem[addr] = data;
        end
    endtask

    task sd_write_le16;
        input [31:0] addr;
        input [15:0] val;
        begin
            u_sd_card.mem[addr]   = val[7:0];
            u_sd_card.mem[addr+1] = val[15:8];
        end
    endtask

    task sd_write_le32;
        input [31:0] addr;
        input [31:0] val;
        begin
            u_sd_card.mem[addr]   = val[7:0];
            u_sd_card.mem[addr+1] = val[15:8];
            u_sd_card.mem[addr+2] = val[23:16];
            u_sd_card.mem[addr+3] = val[31:24];
        end
    endtask

    // =========================================================================
    // Helper: write a FAT32 dir entry
    // =========================================================================
    task write_dir_entry;
        input [31:0] base_addr;     // byte address of 32-byte entry
        input [87:0] name;          // 11 chars, left-justified
        input [7:0]  attr;
        input [31:0] cluster;
        input [31:0] size;
        begin
            // Name (11 bytes)
            u_sd_card.mem[base_addr + 0] = name[87:80];
            u_sd_card.mem[base_addr + 1] = name[79:72];
            u_sd_card.mem[base_addr + 2] = name[71:64];
            u_sd_card.mem[base_addr + 3] = name[63:56];
            u_sd_card.mem[base_addr + 4] = name[55:48];
            u_sd_card.mem[base_addr + 5] = name[47:40];
            u_sd_card.mem[base_addr + 6] = name[39:32];
            u_sd_card.mem[base_addr + 7] = name[31:24];
            u_sd_card.mem[base_addr + 8] = name[23:16];
            u_sd_card.mem[base_addr + 9] = name[15:8];
            u_sd_card.mem[base_addr + 10] = name[7:0];
            // Attributes
            u_sd_card.mem[base_addr + 11] = attr;
            // Reserved bytes 12-19
            // Cluster hi (bytes 20-21)
            u_sd_card.mem[base_addr + 20] = cluster[23:16];
            u_sd_card.mem[base_addr + 21] = cluster[31:24];
            // Cluster lo (bytes 26-27)
            u_sd_card.mem[base_addr + 26] = cluster[7:0];
            u_sd_card.mem[base_addr + 27] = cluster[15:8];
            // File size (bytes 28-31)
            u_sd_card.mem[base_addr + 28] = size[7:0];
            u_sd_card.mem[base_addr + 29] = size[15:8];
            u_sd_card.mem[base_addr + 30] = size[23:16];
            u_sd_card.mem[base_addr + 31] = size[31:24];
        end
    endtask

    // =========================================================================
    // FAT32 filesystem layout constants
    // =========================================================================
    localparam PART_START  = 32'd2048;      // partition starts at block 2048
    localparam SPC         = 8'd64;          // 64 sectors per cluster
    localparam RESERVED    = 16'd32;         // 32 reserved sectors
    localparam NUM_FATS    = 8'd2;
    localparam SPF         = 32'd256;        // sectors per FAT
    localparam ROOT_CLUST  = 32'd2;

    // Computed
    localparam FAT_START   = PART_START + {16'd0, RESERVED};  // 2048+32=2080
    localparam DATA_START  = FAT_START + SPF * {24'd0, NUM_FATS};  // 2080+512=2592
    localparam ROOT_SECTOR = DATA_START + (ROOT_CLUST - 32'd2) * {24'd0, SPC}; // 2592+0=2592

    integer i;

    // =========================================================================
    // Build FAT32 filesystem in model memory
    // =========================================================================
    initial begin
        $dumpfile("fat32_reader_tb.vcd");
        $dumpvars(0, fat32_reader_tb);

        // Initialize catalog to 0xFF
        for (i = 0; i < 512; i = i + 1)
            catalog[i] = 8'hFF;

        // --- MBR at block 0 (byte 0) ---
        // Partition table entry 1 at bytes 446-461
        sd_write_byte(446, 8'h80);          // boot flag
        sd_write_byte(447, 8'h00);          // CHS start (ignored)
        sd_write_byte(448, 8'h00);
        sd_write_byte(449, 8'h00);
        sd_write_byte(450, 8'h0C);          // partition type: FAT32 LBA
        sd_write_byte(451, 8'h00);          // CHS end
        sd_write_byte(452, 8'h00);
        sd_write_byte(453, 8'h00);
        sd_write_le32(454, PART_START);     // LBA start = 2048
        sd_write_le32(458, 32'd65536);      // size in sectors
        // MBR signature
        sd_write_byte(510, 8'h55);
        sd_write_byte(511, 8'hAA);

        // --- BPB at partition_start (block 2048, byte 2048*512=1048576) ---
        // Jump boot code
        sd_write_byte(PART_START*512 + 0, 8'hEB);
        sd_write_byte(PART_START*512 + 1, 8'h58);
        sd_write_byte(PART_START*512 + 2, 8'h90);
        // bytes_per_sector = 512
        sd_write_le16(PART_START*512 + 11, 16'd512);
        // sectors_per_cluster
        sd_write_byte(PART_START*512 + 13, SPC);
        // reserved_sectors
        sd_write_le16(PART_START*512 + 14, RESERVED);
        // num_fats
        sd_write_byte(PART_START*512 + 16, NUM_FATS);
        // sectors_per_fat_32
        sd_write_le32(PART_START*512 + 36, SPF);
        // root_cluster
        sd_write_le32(PART_START*512 + 44, ROOT_CLUST);
        // FAT32 signature "FAT32   " at offset 82
        sd_write_byte(PART_START*512 + 82, 8'h46);  // F
        sd_write_byte(PART_START*512 + 83, 8'h41);  // A
        sd_write_byte(PART_START*512 + 84, 8'h54);  // T
        sd_write_byte(PART_START*512 + 85, 8'h33);  // 3
        // BPB signature
        sd_write_byte(PART_START*512 + 510, 8'h55);
        sd_write_byte(PART_START*512 + 511, 8'hAA);

        // --- Root directory at ROOT_SECTOR (block 2592, byte 2592*512=1327104) ---
        // Entry 0: TEST.PO (cluster 3, 32KB = 32768 bytes)
        //          "TEST    PO "  = 54 45 53 54 20 20 20 20 50 4F 20
        write_dir_entry(ROOT_SECTOR*512 + 0,
            88'h54_45_53_54_20_20_20_20_50_4F_20,  // "TEST    PO "
            8'h20,     // archive attribute
            32'd3,     // first cluster
            32'd32768  // file size
        );

        // Entry 1: GAME.2MG (cluster 4, 32832 bytes = 32768+64 header)
        //          "GAME    2MG"  = 47 41 4D 45 20 20 20 20 32 4D 47
        write_dir_entry(ROOT_SECTOR*512 + 32,
            88'h47_41_4D_45_20_20_20_20_32_4D_47,  // "GAME    2MG"
            8'h20,
            32'd4,
            32'd32832
        );

        // Entry 2: README.TXT (should be skipped - not .PO or .2MG)
        write_dir_entry(ROOT_SECTOR*512 + 64,
            88'h52_45_41_44_4D_45_20_20_54_58_54,  // "README  TXT"
            8'h20,
            32'd5,
            32'd1024
        );

        // Entry 3: BIG.PO (cluster 10, 98304 bytes = 192 blocks)
        write_dir_entry(ROOT_SECTOR*512 + 96,
            88'h42_49_47_20_20_20_20_20_50_4F_20,  // "BIG     PO "
            8'h20,
            32'd10,
            32'd98304
        );

        // Entry 4: deleted entry (byte 0 = 0xE5)
        sd_write_byte(ROOT_SECTOR*512 + 128, 8'hE5);

        // Entry 5: end of directory (byte 0 = 0x00)
        sd_write_byte(ROOT_SECTOR*512 + 160, 8'h00);

        // Reset
        rst_n = 0;
        repeat(10) @(posedge clk);
        rst_n = 1;
        repeat(5) @(posedge clk);

        // Init SD controller first (required for CMD17 to work)
        ctrl_init_start = 1;
        @(posedge clk);
        ctrl_init_start = 0;
        begin : wait_ctrl_init
            integer w;
            for (w = 0; w < 5000000 && !ctrl_init_done; w = w + 1)
                @(posedge clk);
            if (w >= 5000000) $display("TIMEOUT waiting for SD ctrl init");
        end
        $display("SD controller initialized");
        repeat(10) @(posedge clk);

        // =============================================================
        // Test 1-2: Parse MBR and BPB
        // =============================================================
        $display("\n--- Tests 1-2: MBR + BPB parsing ---");
        start = 1;
        @(posedge clk);
        start = 0;

        begin : wait_scan
            integer w;
            for (w = 0; w < 5000000 && !done; w = w + 1)
                @(posedge clk);
            if (w >= 5000000) $display("TIMEOUT waiting for done");
        end

        check("done",       1, done);
        check("error",      0, error);
        check("part_start", PART_START, f32_part_start);
        check("fat_start",  FAT_START, f32_fat_start);
        check("data_start", DATA_START, f32_data_start);
        check("spc",        SPC, f32_spc);
        check("root_cluster", ROOT_CLUST, f32_root_cluster);

        // =============================================================
        // Test 3: Root directory scan — find 3 image files
        // =============================================================
        $display("\n--- Test 3: Directory scan ---");
        check("file_count", 3, file_count);

        // Entry 0: TEST.PO
        check("e0_name0",  8'h54, catalog[0]);   // T
        check("e0_name1",  8'h45, catalog[1]);   // E
        check("e0_name2",  8'h53, catalog[2]);   // S
        check("e0_name3",  8'h54, catalog[3]);   // T
        check("e0_ext0",   8'h50, catalog[8]);   // P
        check("e0_ext1",   8'h4F, catalog[9]);   // O
        check("e0_flags",  8'h00, catalog[11]);  // is_2mg=0
        check("e0_clust0", 8'h03, catalog[12]);  // cluster lo = 3
        check("e0_clust1", 8'h00, catalog[13]);
        check("e0_size0",  8'h00, catalog[16]);  // 32768 = 0x8000
        check("e0_size1",  8'h80, catalog[17]);

        // Entry 1: GAME.2MG
        check("e1_name0",  8'h47, catalog[32+0]);  // G
        check("e1_flags",  8'h01, catalog[32+11]); // is_2mg=1
        check("e1_clust0", 8'h04, catalog[32+12]); // cluster 4
        check("e1_size0",  8'h40, catalog[32+16]); // 32832 = 0x8040
        check("e1_size1",  8'h80, catalog[32+17]);

        // Entry 2: BIG.PO (index 2 in catalog — README.TXT was skipped)
        check("e2_name0",  8'h42, catalog[64+0]);  // B
        check("e2_name1",  8'h49, catalog[64+1]);  // I
        check("e2_flags",  8'h00, catalog[64+11]); // is_2mg=0
        check("e2_clust0", 8'h0A, catalog[64+12]); // cluster 10
        check("e2_size0",  8'h00, catalog[64+16]); // 98304 = 0x18000
        check("e2_size1",  8'h80, catalog[64+17]);
        check("e2_size2",  8'h01, catalog[64+18]);

        // =============================================================
        // Results
        // =============================================================
        repeat(100) @(posedge clk);
        $display("\n========================================");
        $display("fat32_reader_tb: %0d tests, %0d passed, %0d failed",
                 tests_run, tests_passed, tests_failed);
        $display("========================================\n");
        $finish;
    end

    initial begin
        #500000000;
        $display("GLOBAL TIMEOUT");
        $finish;
    end

endmodule
