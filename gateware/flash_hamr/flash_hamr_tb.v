// flash_hamr_tb.v — Integration testbench for flash_hamr
//
// Tests the SD controller + FAT32 reader + sd_mount pipeline end-to-end.
// Uses sd_card_model with FAT32 filesystem, mock SDRAM array.

`timescale 1ns / 1ps

module flash_hamr_tb;

    reg clk = 0;
    always #20 clk = ~clk;  // 25 MHz

    reg rst_n = 0;

    // =========================================================================
    // SD Card Model
    // =========================================================================
    wire sd_sck, sd_mosi, sd_miso, sd_cs;

    sd_card_model #(.MEM_BYTES(2097152), .ACMD41_RETRIES(2), .WRITE_BUSY_CLKS(20)) u_sd_card (
        .spi_sck(sd_sck), .spi_mosi(sd_mosi), .spi_miso(sd_miso), .spi_cs_n(sd_cs)
    );

    // =========================================================================
    // SD Controller
    // =========================================================================
    reg         sd_init_start = 0;
    wire        sd_init_done, sd_init_error, sd_is_sdhc;

    wire        sd_read_start_f32, sd_read_start_mt, sd_read_start_ps;
    wire [31:0] sd_read_addr_f32, sd_read_addr_mt, sd_read_addr_ps;
    wire        sd_read_ready_f32, sd_read_ready_mt, sd_read_ready_ps;

    reg         use_f32 = 1;    // 1=fat32_reader owns SD reads, 0=sd_mount
    reg         use_mount = 0;

    wire        sd_read_start = use_f32 ? sd_read_start_f32 :
                                use_mount ? sd_read_start_mt : sd_read_start_ps;
    wire [31:0] sd_read_addr  = use_f32 ? sd_read_addr_f32 :
                                use_mount ? sd_read_addr_mt : sd_read_addr_ps;
    // Always ready — SECTOR_RECV buffers the entire sector without pausing
    wire        sd_read_ready = 1'b1;

    wire [7:0]  sd_read_data;
    wire        sd_read_data_valid, sd_read_done, sd_read_error;

    wire        sd_write_start_ps;
    wire [31:0] sd_write_addr_ps;
    wire [7:0]  sd_write_data_ps;
    wire        sd_write_data_valid_ps;
    wire        sd_write_data_req, sd_write_done, sd_write_error;

    sd_controller u_sd_ctrl (
        .clk(clk), .rst_n(rst_n),
        .init_start(sd_init_start), .init_done(sd_init_done),
        .init_error(sd_init_error), .is_sdhc(sd_is_sdhc),
        .read_start(sd_read_start), .read_addr(sd_read_addr),
        .read_data(sd_read_data), .read_data_valid(sd_read_data_valid),
        .read_data_ready(sd_read_ready), .read_done(sd_read_done),
        .read_error(sd_read_error),
        .write_start(sd_write_start_ps), .write_addr(sd_write_addr_ps),
        .write_data(sd_write_data_ps), .write_data_valid(sd_write_data_valid_ps),
        .write_data_req(sd_write_data_req), .write_done(sd_write_done),
        .write_error(sd_write_error),
        .sd_sck(sd_sck), .sd_mosi(sd_mosi), .sd_miso(sd_miso), .sd_cs(sd_cs)
    );

    // =========================================================================
    // FAT32 Reader
    // =========================================================================
    reg         fat32_start = 0;
    wire        fat32_done, fat32_error;
    wire [3:0]  img_count;
    wire [31:0] fat32_data_start, fat32_fat_start;
    wire [7:0]  fat32_spc;
    wire [31:0] fat32_root_cluster;

    // Catalog BRAM
    wire [8:0]  cat_wr_addr;
    wire [7:0]  cat_wr_data;
    wire        cat_wr_en;
    reg  [7:0]  catalog [0:511];
    always @(posedge clk) if (cat_wr_en) catalog[cat_wr_addr] <= cat_wr_data;

    fat32_reader u_fat32 (
        .clk(clk), .rst_n(rst_n),
        .start(fat32_start), .done(fat32_done), .error(fat32_error),
        .file_count(img_count),
        .sd_read_start(sd_read_start_f32), .sd_read_addr(sd_read_addr_f32),
        .sd_read_data(sd_read_data), .sd_read_data_valid(sd_read_data_valid),
        .sd_read_data_ready(sd_read_ready_f32), .sd_read_done(sd_read_done),
        .sd_read_error(sd_read_error),
        .fat32_partition_start(), .fat32_fat_start(fat32_fat_start),
        .fat32_data_start(fat32_data_start), .fat32_spc(fat32_spc),
        .fat32_root_cluster(fat32_root_cluster),
        .cat_wr_addr(cat_wr_addr), .cat_wr_data(cat_wr_data), .cat_wr_en(cat_wr_en)
    );

    // =========================================================================
    // Mock SDRAM (simple array, immediate response)
    // =========================================================================
    reg  [7:0]  sdram_mem [0:4194303];  // 4MB
    wire        mount_sdram_req, mount_sdram_write;
    wire [25:0] mount_sdram_addr;
    wire [15:0] mount_sdram_wdata;
    wire        sdram_ready_r;
    reg  [15:0] sdram_rdata_r = 0;
    reg         sdram_rdata_valid_r = 0;
    wire        mount_sdram_claim;

    // Simple SDRAM mock — combinational ready, write on posedge
    // sdram_req_ready is ALWAYS 1 (immediate accept)
    assign sdram_ready_r = 1'b1;

    integer sdram_wr_cnt = 0;
    always @(posedge clk) begin
        if (mount_sdram_req && mount_sdram_write) begin
            sdram_mem[mount_sdram_addr]       <= mount_sdram_wdata[7:0];
            sdram_mem[mount_sdram_addr | 26'd1] <= mount_sdram_wdata[15:8];
            sdram_wr_cnt = sdram_wr_cnt + 1;
            // Print first 3 and first 3 of sector 1 (addr >= 0x100200)
            if (sdram_wr_cnt <= 3 || (mount_sdram_addr >= 26'h100200 && mount_sdram_addr < 26'h100208))
                $display("[SDRAM] wr#%0d addr=0x%06h data=0x%04h",
                         sdram_wr_cnt, mount_sdram_addr, mount_sdram_wdata);
        end
    end

    // =========================================================================
    // SD Mount
    // =========================================================================
    reg         mount_request = 0;
    reg  [3:0]  mount_img_select = 0;
    wire        s4d2_mounted, s4d2_loading;
    wire [15:0] s4d2_block_count;
    wire        s4d2_is_2mg;
    wire [15:0] mount_data_offset;

    // Chain map BRAM
    wire [9:0]  chain_wr_addr;
    wire [31:0] chain_wr_data;
    wire        chain_wr_en;
    reg  [31:0] chain_map [0:1023];
    always @(posedge clk) if (chain_wr_en) chain_map[chain_wr_addr] <= chain_wr_data;

    // sd_mount reads catalog via its own port — wire to same catalog array
    wire [8:0]  mount_cat_rd_addr;
    wire [7:0]  mount_cat_rd_data = catalog[mount_cat_rd_addr];

    sd_mount u_sd_mount (
        .clk(clk), .rst_n(rst_n),
        .mount_request(mount_request), .img_select(mount_img_select),
        .s4d2_mounted(s4d2_mounted), .s4d2_loading(s4d2_loading),
        .s4d2_block_count(s4d2_block_count),
        .is_2mg(s4d2_is_2mg), .data_offset(mount_data_offset),
        .cat_rd_addr(mount_cat_rd_addr), .cat_rd_data(mount_cat_rd_data),
        .fat32_data_start(fat32_data_start), .fat32_fat_start(fat32_fat_start),
        .fat32_spc(fat32_spc),
        .sd_read_start(sd_read_start_mt), .sd_read_addr(sd_read_addr_mt),
        .sd_read_data(sd_read_data), .sd_read_data_valid(sd_read_data_valid),
        .sd_read_data_ready(sd_read_ready_mt), .sd_read_done(sd_read_done),
        .sdram_req(mount_sdram_req), .sdram_req_write(mount_sdram_write),
        .sdram_req_addr(mount_sdram_addr), .sdram_req_wdata(mount_sdram_wdata),
        .sdram_req_ready(mount_sdram_claim & sdram_ready_r),
        .sdram_claim(mount_sdram_claim),
        .chain_wr_addr(chain_wr_addr), .chain_wr_data(chain_wr_data),
        .chain_wr_en(chain_wr_en),
        .sd_image_first_cluster()
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
    // FAT32 setup (same as fat32_reader_tb)
    // =========================================================================
    localparam PART_START = 32'd2048;
    localparam SPC        = 8'd64;
    localparam RESERVED   = 16'd32;
    localparam NUM_FATS   = 8'd2;
    localparam SPF        = 32'd256;
    localparam ROOT_CLUST = 32'd2;
    localparam FAT_START  = PART_START + 32'd32;
    localparam DATA_START = FAT_START + 32'd512;
    localparam ROOT_SEC   = DATA_START;
    localparam CLUSTER3_SEC = DATA_START + 32'd64;

    integer i;

    // =========================================================================
    // Test sequence
    // =========================================================================
    initial begin
        $dumpfile("flash_hamr_tb.vcd");
        $dumpvars(0, flash_hamr_tb);

        // Init memories
        for (i = 0; i < 512; i = i+1) catalog[i] = 8'hFF;
        for (i = 0; i < 4194304; i = i+1) sdram_mem[i] = 8'h00;

        // Build FAT32 filesystem
        // MBR
        u_sd_card.mem[450] = 8'h0C;
        u_sd_card.mem[454] = PART_START[7:0];
        u_sd_card.mem[455] = PART_START[15:8];
        u_sd_card.mem[456] = PART_START[23:16];
        u_sd_card.mem[457] = PART_START[31:24];
        u_sd_card.mem[510] = 8'h55;
        u_sd_card.mem[511] = 8'hAA;

        // BPB
        u_sd_card.mem[PART_START*512 + 11] = 8'h00;
        u_sd_card.mem[PART_START*512 + 12] = 8'h02;
        u_sd_card.mem[PART_START*512 + 13] = SPC;
        u_sd_card.mem[PART_START*512 + 14] = RESERVED[7:0];
        u_sd_card.mem[PART_START*512 + 15] = RESERVED[15:8];
        u_sd_card.mem[PART_START*512 + 16] = NUM_FATS;
        u_sd_card.mem[PART_START*512 + 36] = SPF[7:0];
        u_sd_card.mem[PART_START*512 + 37] = SPF[15:8];
        u_sd_card.mem[PART_START*512 + 38] = SPF[23:16];
        u_sd_card.mem[PART_START*512 + 39] = SPF[31:24];
        u_sd_card.mem[PART_START*512 + 44] = ROOT_CLUST[7:0];
        u_sd_card.mem[PART_START*512 + 45] = ROOT_CLUST[15:8];
        u_sd_card.mem[PART_START*512 + 46] = ROOT_CLUST[23:16];
        u_sd_card.mem[PART_START*512 + 47] = ROOT_CLUST[31:24];
        u_sd_card.mem[PART_START*512 + 82] = 8'h46;
        u_sd_card.mem[PART_START*512 + 83] = 8'h41;
        u_sd_card.mem[PART_START*512 + 84] = 8'h54;
        u_sd_card.mem[PART_START*512 + 85] = 8'h33;
        u_sd_card.mem[PART_START*512 + 510] = 8'h55;
        u_sd_card.mem[PART_START*512 + 511] = 8'hAA;

        // Root directory: TEST.PO at cluster 3, 32KB
        u_sd_card.mem[ROOT_SEC*512 + 0]  = 8'h54;  // T
        u_sd_card.mem[ROOT_SEC*512 + 1]  = 8'h45;  // E
        u_sd_card.mem[ROOT_SEC*512 + 2]  = 8'h53;  // S
        u_sd_card.mem[ROOT_SEC*512 + 3]  = 8'h54;  // T
        u_sd_card.mem[ROOT_SEC*512 + 4]  = 8'h20;
        u_sd_card.mem[ROOT_SEC*512 + 5]  = 8'h20;
        u_sd_card.mem[ROOT_SEC*512 + 6]  = 8'h20;
        u_sd_card.mem[ROOT_SEC*512 + 7]  = 8'h20;
        u_sd_card.mem[ROOT_SEC*512 + 8]  = 8'h50;  // P
        u_sd_card.mem[ROOT_SEC*512 + 9]  = 8'h4F;  // O
        u_sd_card.mem[ROOT_SEC*512 + 10] = 8'h20;  // space
        u_sd_card.mem[ROOT_SEC*512 + 11] = 8'h20;  // archive attr
        u_sd_card.mem[ROOT_SEC*512 + 26] = 8'h03;  // cluster lo = 3
        u_sd_card.mem[ROOT_SEC*512 + 27] = 8'h00;
        u_sd_card.mem[ROOT_SEC*512 + 28] = 8'h00;  // size = 32768
        u_sd_card.mem[ROOT_SEC*512 + 29] = 8'h80;
        u_sd_card.mem[ROOT_SEC*512 + 30] = 8'h00;
        u_sd_card.mem[ROOT_SEC*512 + 31] = 8'h00;
        u_sd_card.mem[ROOT_SEC*512 + 32] = 8'h00;  // end of dir

        // FAT: cluster 3 → end
        u_sd_card.mem[FAT_START*512 + 12] = 8'hFF;
        u_sd_card.mem[FAT_START*512 + 13] = 8'hFF;
        u_sd_card.mem[FAT_START*512 + 14] = 8'hFF;
        u_sd_card.mem[FAT_START*512 + 15] = 8'h0F;

        // Test data at cluster 3 (64 sectors from data_start)
        for (i = 0; i < 32768; i = i + 1)
            u_sd_card.mem[CLUSTER3_SEC * 512 + i] = ((i/512)*7 + (i%512)) & 8'hFF;

        // Verify test data was written correctly
        $display("SD model verify: byte 0 at addr %0d = 0x%02h (expect 0x00)",
                 CLUSTER3_SEC * 512, u_sd_card.mem[CLUSTER3_SEC * 512]);
        $display("SD model verify: byte 1 at addr %0d = 0x%02h (expect 0x01)",
                 CLUSTER3_SEC * 512 + 1, u_sd_card.mem[CLUSTER3_SEC * 512 + 1]);
        $display("SD model verify: byte 512 at addr %0d = 0x%02h (expect 0x07)",
                 CLUSTER3_SEC * 512 + 512, u_sd_card.mem[CLUSTER3_SEC * 512 + 512]);

        // =====================================================================
        // Reset
        // =====================================================================
        rst_n = 0;
        repeat(10) @(posedge clk);
        rst_n = 1;
        repeat(5) @(posedge clk);

        // =====================================================================
        // Step 1: Init SD controller
        // =====================================================================
        $display("\n--- Step 1: SD controller init ---");
        sd_init_start = 1;
        @(posedge clk);
        sd_init_start = 0;
        begin : w1
            integer w;
            for (w = 0; w < 5000000 && !sd_init_done && !sd_init_error; w = w+1)
                @(posedge clk);
        end
        check("sd_init_done", 1, sd_init_done);
        check("sd_init_error", 0, sd_init_error);
        $display("  SD init OK");

        // =====================================================================
        // Step 2: FAT32 scan
        // =====================================================================
        $display("\n--- Step 2: FAT32 scan ---");
        use_f32 = 1;
        fat32_start = 1;
        @(posedge clk);
        fat32_start = 0;
        begin : w2
            integer w;
            for (w = 0; w < 5000000 && !fat32_done; w = w+1)
                @(posedge clk);
        end
        check("fat32_done", 1, fat32_done);
        check("fat32_error", 0, fat32_error);
        check("img_count", 1, img_count);
        check("cat_name_T", 8'h54, catalog[0]);
        check("cat_name_E", 8'h45, catalog[1]);
        check("cat_cluster", 8'h03, catalog[12]);
        $display("  FAT32 scan OK — found %0d file(s)", img_count);

        // =====================================================================
        // Step 3: Mount TEST.PO
        // =====================================================================
        $display("\n--- Step 3: Mount TEST.PO ---");
        use_f32 = 0;
        use_mount = 1;
        mount_img_select = 4'd0;
        // Wait for SD controller to return to IDLE after FAT32 scan
        begin : wait_ctrl_idle
            integer w;
            for (w = 0; w < 10000 && u_sd_ctrl.state != 0; w = w + 1)
                @(posedge clk);
        end
        repeat(10) @(posedge clk);
        mount_request = 1;
        @(posedge clk);
        mount_request = 0;
        begin : w3
            integer w;
            for (w = 0; w < 10000000 && !s4d2_mounted; w = w+1)
                @(posedge clk);
            if (w >= 10000000) $display("  TIMEOUT waiting for mount");
        end
        check("s4d2_mounted", 1, s4d2_mounted);
        check("s4d2_is_2mg", 0, s4d2_is_2mg);
        check("s4d2_block_count", 64, s4d2_block_count);  // 32768/512 = 64
        $display("  Mount OK — %0d blocks", s4d2_block_count);

        // =====================================================================
        // Step 4: Verify SDRAM contents
        // =====================================================================
        $display("\n--- Step 4: Verify SDRAM data ---");
        // UNIT2_OFFSET = 2048, so SDRAM addr = 2048 * 512 = 1048576 = 0x100000
        // Block 0, byte 0 should be (0*7 + 0) & 0xFF = 0x00
        // Block 0, byte 1 should be (0*7 + 1) & 0xFF = 0x01
        // Block 1, byte 0 should be (1*7 + 0) & 0xFF = 0x07
        check("sdram_blk0_byte0", 8'h00, sdram_mem[1048576]);
        check("sdram_blk0_byte1", 8'h01, sdram_mem[1048577]);
        check("sdram_blk0_byte255", 8'hFF, sdram_mem[1048576+255]);
        check("sdram_blk1_byte0", 8'h07, sdram_mem[1048576+512]);
        check("sdram_blk1_byte1", 8'h08, sdram_mem[1048576+513]);
        // Block 63 (last), byte 0 = (63*7 + 0) & 0xFF = 441 & 0xFF = 0xB9
        check("sdram_blk63_byte0", ((63*7) & 8'hFF), sdram_mem[1048576 + 63*512]);

        // =====================================================================
        // Step 5: Chain map verification
        // =====================================================================
        $display("\n--- Step 5: Chain map ---");
        check("chain_map_0", 32'd3, chain_map[0]);  // cluster 3

        // =====================================================================
        // Results
        // =====================================================================
        repeat(100) @(posedge clk);
        $display("\n========================================");
        $display("flash_hamr_tb: %0d tests, %0d passed, %0d failed",
                 tests_run, tests_passed, tests_failed);
        $display("========================================\n");
        $finish;
    end

    // Monitor all sd_read_start_mt pulses (from mount directly)
    integer mount_read_count = 0;
    always @(posedge clk) begin
        if (sd_read_start_mt) begin
            mount_read_count = mount_read_count + 1;
            if (mount_read_count <= 5)
                $display("[TB] sd_read_start_mt #%0d: addr=%0d mux_start=%0d ctrl_state=%0d mount_state=%0d",
                         mount_read_count, sd_read_addr_mt, sd_read_start,
                         u_sd_ctrl.state, u_sd_mount.state);
        end
    end

    // Periodic state monitoring
    integer mon_cnt = 0;
    always @(posedge clk) begin
        mon_cnt <= mon_cnt + 1;
        if (mon_cnt % 50000 == 0 && u_sd_mount.s4d2_loading)
            $display("[MON] mount_state=%0d sec=%0d remain=%0d",
                     u_sd_mount.state, u_sd_mount.sector_in_cluster,
                     u_sd_mount.bytes_remaining);
    end

    initial begin
        #2000000000;
        $display("GLOBAL TIMEOUT");
        $finish;
    end

endmodule
