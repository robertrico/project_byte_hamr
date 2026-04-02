`timescale 1ns / 1ps
// =============================================================================
// Flash Hamr — Top Level Module (FAT32 SD card two-device controller)
// =============================================================================
// S4D1: Menu volume from SPI flash (flash_reader + USRMCLK)
// S4D2: User image from SD card (sd_controller via GPIO SPI)
//
// Clock domains:
//   sig_7M   (7.16 MHz) — Apple II bus, bus_interface
//   CLK_25MHz (25 MHz)  — SDRAM, SPI flash, SD card
// =============================================================================

module flash_hamr_top (
    // ---- Apple II Bus ----
    input wire [11:0] addr,
    inout wire        D0, D1, D2, D3, D4, D5, D6, D7,
    input wire        sig_7M,
    input wire        Q3,
    input wire        R_nW,
    input wire        nDEVICE_SELECT,
    input wire        nI_O_SELECT,
    input wire        nI_O_STROBE,
    output wire       nRES,
    input  wire       RDY,
    output wire       nIRQ, nNMI, nDMA, nINH,
    input  wire       DMA_OUT,
    output wire       DMA_IN,
    input  wire       INT_OUT,
    output wire       INT_IN,
    input  wire       PHI0, PHI1,
    input  wire       uSync,

    // ---- 25 MHz System Clock ----
    input wire        CLK_25MHz,

    // ---- SDRAM ----
    output wire       SDRAM_CLK, SDRAM_CKE, SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE,
    output wire       SDRAM_DQM0, SDRAM_DQM1, SDRAM_BA0, SDRAM_BA1,
    output wire       SDRAM_A0, SDRAM_A1, SDRAM_A2, SDRAM_A3,
    output wire       SDRAM_A4, SDRAM_A5, SDRAM_A6, SDRAM_A7,
    output wire       SDRAM_A8, SDRAM_A9, SDRAM_A10, SDRAM_A11, SDRAM_A12,
    inout  wire       SDRAM_D0, SDRAM_D1, SDRAM_D2, SDRAM_D3,
    inout  wire       SDRAM_D4, SDRAM_D5, SDRAM_D6, SDRAM_D7,
    inout  wire       SDRAM_D8, SDRAM_D9, SDRAM_D10, SDRAM_D11,
    inout  wire       SDRAM_D12, SDRAM_D13, SDRAM_D14, SDRAM_D15,

    // ---- SPI Flash (menu volume only) ----
    output wire       FLASH_nCS, FLASH_MOSI,
    input  wire       FLASH_MISO,
    output wire       FLASH_nWP, FLASH_nHOLD,

    // ---- SD Card SPI (GPIO1-4) ----
    output wire       SD_CS, SD_SCK, SD_MOSI,
    input  wire       SD_MISO,

    // ---- GPIO (debug on 5-8, card detect 9) ----
    output wire       GPIO5, GPIO6, GPIO7, GPIO8,
    input  wire       GPIO9,
    output wire       GPIO10, GPIO11, GPIO12
);

    // =========================================================================
    // POR (7 MHz)
    // =========================================================================
    assign nRES = 1'b1;

    reg [3:0] por_counter = 4'd0;
    wire      por_n = &por_counter;
    always @(posedge sig_7M) begin
        if (!por_n) por_counter <= por_counter + 4'd1;
    end

    // POR (25 MHz)
    reg [7:0] por_25_counter = 8'd0;
    wire      por_25_n = &por_25_counter;
    always @(posedge CLK_25MHz) begin
        if (!por_25_n) por_25_counter <= por_25_counter + 8'd1;
    end

    // =========================================================================
    // Unused control signals
    // =========================================================================
    assign nIRQ = 1'bZ;
    assign nNMI = 1'bZ;
    assign nINH = 1'bZ;
    assign nDMA = 1'bZ;
    assign DMA_IN = DMA_OUT;
    assign INT_IN = INT_OUT;

    // =========================================================================
    // Data Bus
    // =========================================================================
    wire [7:0] rom_data;
    wire       rom_oe = ~nI_O_SELECT;  // $Cn page ONLY — no $C800 (prevents bus contention)
    wire [7:0] bus_data_out;

    wire [7:0] data_out_mux = rom_oe ? rom_data : bus_data_out;
    wire data_oe = R_nW && (rom_oe || !nDEVICE_SELECT);

    assign D0 = data_oe ? data_out_mux[0] : 1'bZ;
    assign D1 = data_oe ? data_out_mux[1] : 1'bZ;
    assign D2 = data_oe ? data_out_mux[2] : 1'bZ;
    assign D3 = data_oe ? data_out_mux[3] : 1'bZ;
    assign D4 = data_oe ? data_out_mux[4] : 1'bZ;
    assign D5 = data_oe ? data_out_mux[5] : 1'bZ;
    assign D6 = data_oe ? data_out_mux[6] : 1'bZ;
    assign D7 = data_oe ? data_out_mux[7] : 1'bZ;

    wire lvl_shift_oe = !nDEVICE_SELECT || rom_oe;
    assign GPIO12 = ~lvl_shift_oe;

    // =========================================================================
    // Debug GPIO
    // =========================================================================
    wire boot_done, s4d2_mounted, dev_block_ready;

    // Forward declarations for SD init sequencer (defined after boot_loader)
    wire        sd_ready;
    wire        sd_error;
    wire        sd_init_start;
    wire        fat32_start;
    wire        sd_init_request;
    wire        sd_init_done;
    wire        sd_init_error;
    // Forward: sequencer state + constants for SD read mux
    reg [2:0]   sd_seq_state;
    localparam SD_SEQ_FAT32_FWD = 3'd3, SD_SEQ_FAT32_WAIT_FWD = 3'd4;
    assign GPIO5  = boot_done;
    assign GPIO6  = sd_is_sdhc;       // DEBUG: is card detected as SDHC?
    assign GPIO7  = sd_init_request; // from bus_interface (KNOWN WORKING output)
    assign GPIO8  = sd_init_error;   // RAW from sd_controller
    assign GPIO10 = 1'b0;
    assign GPIO11 = 1'b0;

    // =========================================================================
    // Address Decoder + Boot ROM
    // =========================================================================
    wire rom_expansion_active;
    addr_decoder u_addr_decoder (
        .addr(addr), .clk(sig_7M), .nI_O_STROBE(nI_O_STROBE),
        .nI_O_SELECT(nI_O_SELECT), .nRES(por_n),
        .rom_oe(rom_oe), .rom_expansion_active(rom_expansion_active)
    );

    wire [11:0] rom_addr = !nI_O_SELECT ? {4'b0100, addr[7:0]} : {1'b1, addr[10:0]};
    boot_rom u_boot_rom (.clk(sig_7M), .addr(rom_addr), .data(rom_data));

    // =========================================================================
    // Catalog BRAM (512 bytes, dual-port)
    // Port A: fat32_reader writes during scan
    // Port B: bus_interface reads for registers 9-D
    // =========================================================================
    wire [8:0]  cat_wr_addr, cat_rd_addr;
    wire [7:0]  cat_wr_data, cat_rd_data;
    wire        cat_wr_en;
    (* ram_style = "block" *)
    reg [7:0]   catalog_mem [0:511];

    reg [7:0] cat_rd_data_r;
    always @(posedge CLK_25MHz) begin
        if (cat_wr_en) catalog_mem[cat_wr_addr] <= cat_wr_data;
        cat_rd_data_r <= catalog_mem[cat_rd_addr];
    end
    assign cat_rd_data = cat_rd_data_r;

    // =========================================================================
    // Chain Map BRAM (4KB, dual-port)
    // Port A: sd_mount writes during mount
    // Port B: sd_persist reads during write-back
    // =========================================================================
    wire [9:0]  chain_wr_addr, chain_rd_addr;
    wire [31:0] chain_wr_data, chain_rd_data;
    wire        chain_wr_en;
    (* ram_style = "block" *)
    reg [31:0]  chain_mem [0:1023];

    reg [31:0] chain_rd_data_r;
    always @(posedge CLK_25MHz) begin
        if (chain_wr_en) chain_mem[chain_wr_addr] <= chain_wr_data;
        chain_rd_data_r <= chain_mem[chain_rd_addr];
    end
    assign chain_rd_data = chain_rd_data_r;

    // =========================================================================
    // Bus Interface (7 MHz)
    // =========================================================================
    wire [8:0]  buf_addr_b;
    wire [7:0]  buf_rdata_b, buf_wdata_b;
    wire        buf_we_b;
    wire        dev_block_read_req, dev_block_write_req;
    wire [15:0] dev_block_num;
    wire        arb_block_ready;
    wire [15:0] total_blocks;

    // SD management signals (sd_error, sd_ready declared in SD init sequencer)
    wire [3:0]  img_count;
    wire [3:0]  img_select;
    wire [3:0]  img_name_idx;
    wire        mount_request;
    // sd_init_request declared above as forward declaration
    wire [15:0] s4d2_block_count;
    wire        s4d2_is_2mg;
    wire        s4d2_loading;

    bus_interface u_bus_interface (
        .clk(sig_7M), .rst_n(por_n),
        .addr(addr[3:0]),
        .data_in({D7, D6, D5, D4, D3, D2, D1, D0}),
        .data_out(bus_data_out),
        .nDEVICE_SELECT(nDEVICE_SELECT), .R_nW(R_nW),
        .boot_done(boot_done), .total_blocks(total_blocks),
        .buf_addr(buf_addr_b), .buf_rdata(buf_rdata_b),
        .buf_wdata(buf_wdata_b), .buf_we(buf_we_b),
        .block_read_req(dev_block_read_req), .block_write_req(dev_block_write_req),
        .block_num(dev_block_num), .block_ready(dev_block_ready),
        .block_num_out(),
        // SD management
        .sd_ready(sd_ready), .sd_error_in(sd_error),
        .s4d2_mounted(s4d2_mounted), .s4d2_loading(s4d2_loading),
        .img_count(img_count),
        .cat_rd_addr(cat_rd_addr), .cat_rd_data(cat_rd_data),
        .img_select(img_select), .img_name_idx(img_name_idx),
        .mount_request(mount_request), .sd_init_request(sd_init_request),
        .s4d2_block_count(s4d2_block_count), .s4d2_is_2mg(s4d2_is_2mg),
        // Debug
        .dbg_dir_byte0(fat32_dbg_byte0), .dbg_dir_byte8(fat32_dbg_byte8),
        .dbg_is_fat16(fat32_dbg_is_fat16)
    );

    // =========================================================================
    // Block Buffer (dual-port BRAM, 7MHz/25MHz)
    // =========================================================================
    wire [8:0]  buf_addr_a;
    wire [7:0]  buf_wdata_a, buf_rdata_a;
    wire        buf_we_a;

    // Forward declarations for persist buf mux (defined after sd_persist)
    wire        persist_sdram_claim;
    wire [8:0]  persist_buf_addr;
    wire [8:0]  mux_buf_addr_a  = persist_sdram_claim ? persist_buf_addr : buf_addr_a;
    wire [7:0]  mux_buf_wdata_a = buf_wdata_a;
    wire        mux_buf_we_a    = persist_sdram_claim ? 1'b0 : buf_we_a;

    block_buffer u_block_buffer (
        .clk_a(CLK_25MHz), .addr_a(mux_buf_addr_a), .wdata_a(mux_buf_wdata_a),
        .we_a(mux_buf_we_a), .rdata_a(buf_rdata_a),
        .clk_b(sig_7M), .addr_b(buf_addr_b), .wdata_b(buf_wdata_b),
        .we_b(buf_we_b), .rdata_b(buf_rdata_b)
    );

    // =========================================================================
    // SDRAM Address/Data Mapping
    // =========================================================================
    wire [12:0] sdram_a;
    wire [15:0] sdram_dq = {SDRAM_D15, SDRAM_D14, SDRAM_D13, SDRAM_D12,
                             SDRAM_D11, SDRAM_D10, SDRAM_D9,  SDRAM_D8,
                             SDRAM_D7,  SDRAM_D6,  SDRAM_D5,  SDRAM_D4,
                             SDRAM_D3,  SDRAM_D2,  SDRAM_D1,  SDRAM_D0};
    assign {SDRAM_A12, SDRAM_A11, SDRAM_A10, SDRAM_A9, SDRAM_A8,
            SDRAM_A7, SDRAM_A6, SDRAM_A5, SDRAM_A4,
            SDRAM_A3, SDRAM_A2, SDRAM_A1, SDRAM_A0} = sdram_a;

    // =========================================================================
    // SDRAM Controller + Mux
    // =========================================================================
    wire        arb_sdram_req, arb_sdram_write;
    wire [25:0] arb_sdram_addr;
    wire [15:0] arb_sdram_wdata;
    wire        sdram_req_ready;
    wire [15:0] sdram_req_rdata;
    wire        sdram_req_rdata_valid;
    wire        sdram_init_done;

    // SDRAM mux: mount_claim or persist_claim switches from arbiter
    wire        mount_sdram_req, mount_sdram_write;
    wire [25:0] mount_sdram_addr;
    wire [15:0] mount_sdram_wdata;
    wire        mount_sdram_claim;

    // SDRAM mux: mount_claim switches from arbiter (persist reads block_buffer, not SDRAM)
    wire        wt_claim = mount_sdram_claim;

    wire        mux_sdram_req   = mount_sdram_claim ? mount_sdram_req   : arb_sdram_req;
    wire        mux_sdram_write = mount_sdram_claim ? mount_sdram_write : arb_sdram_write;
    wire [25:0] mux_sdram_addr  = mount_sdram_claim ? mount_sdram_addr  : arb_sdram_addr;
    wire [15:0] mux_sdram_wdata = mount_sdram_claim ? mount_sdram_wdata : arb_sdram_wdata;

    sdram_controller u_sdram_controller (
        .clk(CLK_25MHz), .rst_n(por_25_n), .init_done(sdram_init_done),
        .pause_refresh(1'b0),
        .req(mux_sdram_req), .req_write(mux_sdram_write),
        .req_addr(mux_sdram_addr), .req_wdata(mux_sdram_wdata),
        .req_ready(sdram_req_ready),
        .req_rdata(sdram_req_rdata), .req_rdata_valid(sdram_req_rdata_valid),
        .SDRAM_CLK(SDRAM_CLK), .SDRAM_CKE(SDRAM_CKE),
        .SDRAM_nCS(SDRAM_nCS), .SDRAM_nRAS(SDRAM_nRAS),
        .SDRAM_nCAS(SDRAM_nCAS), .SDRAM_nWE(SDRAM_nWE),
        .SDRAM_DQM0(SDRAM_DQM0), .SDRAM_DQM1(SDRAM_DQM1),
        .SDRAM_BA0(SDRAM_BA0), .SDRAM_BA1(SDRAM_BA1),
        .SDRAM_A(sdram_a), .SDRAM_DQ(sdram_dq)
    );

    // =========================================================================
    // SPI Flash (menu volume boot only)
    // =========================================================================
    wire        flash_busy, flash_done;
    wire [7:0]  flash_data_out;
    wire        flash_data_valid, flash_data_ready;
    wire        flash_start;
    wire [23:0] flash_start_addr, flash_byte_count;
    wire        reader_sck, reader_ncs, reader_mosi;

    assign FLASH_nCS  = reader_ncs;
    assign FLASH_MOSI = reader_mosi;
    assign FLASH_nWP   = 1'b1;
    assign FLASH_nHOLD = 1'b1;

    `ifndef SYNTHESIS
    `else
        USRMCLK u_usrmclk (.USRMCLKI(reader_sck), .USRMCLKTS(1'b0));
    `endif

    flash_reader u_flash_reader (
        .clk(CLK_25MHz), .rst_n(por_25_n),
        .start(flash_start), .start_addr(flash_start_addr),
        .byte_count(flash_byte_count),
        .busy(flash_busy), .done(flash_done),
        .data_out(flash_data_out), .data_valid(flash_data_valid),
        .data_ready(flash_data_ready),
        .flash_ncs(reader_ncs), .flash_mosi(reader_mosi),
        .flash_miso(FLASH_MISO),
        .flash_nwp(), .flash_nhold(),
        .flash_sck_pin(reader_sck)
    );

    // =========================================================================
    // SD Card Controller (GPIO SPI)
    // =========================================================================
    wire        sd_is_sdhc;
    wire        sd_sck_w, sd_mosi_w;
    wire        sd_cs_w;

    // SD read interface (shared: boot_loader/fat32_reader → sd_mount → sd_persist)
    wire        sd_read_start_bl, sd_read_start_mt, sd_read_start_ps;
    wire [31:0] sd_read_addr_bl, sd_read_addr_mt, sd_read_addr_ps;
    wire        sd_read_ready_bl, sd_read_ready_mt, sd_read_ready_ps;

    // SD read mux: fat32_reader during scan, sd_mount during mount, sd_persist otherwise
    wire fat32_scanning = (sd_seq_state == SD_SEQ_FAT32_FWD) || (sd_seq_state == SD_SEQ_FAT32_WAIT_FWD);

    wire        sd_read_start = fat32_scanning  ? sd_read_start_bl :
                                s4d2_loading    ? sd_read_start_mt :
                                                  sd_read_start_ps;
    wire [31:0] sd_read_addr  = fat32_scanning  ? sd_read_addr_bl :
                                s4d2_loading    ? sd_read_addr_mt :
                                                  sd_read_addr_ps;
    wire        sd_read_ready = fat32_scanning  ? sd_read_ready_bl :
                                s4d2_loading    ? sd_read_ready_mt :
                                                  sd_read_ready_ps;

    wire [7:0]  sd_read_data;
    wire        sd_read_data_valid, sd_read_done, sd_read_error;

    // SD write interface (sd_persist only)
    wire        sd_write_start_ps;
    wire [31:0] sd_write_addr_ps;
    wire [7:0]  sd_write_data_ps;
    wire        sd_write_data_valid_ps;
    wire        sd_write_data_req, sd_write_done, sd_write_error;

    assign SD_CS   = sd_cs_w;
    assign SD_SCK  = sd_sck_w;
    assign SD_MOSI = sd_mosi_w;

    sd_controller u_sd_controller (
        .clk(CLK_25MHz), .rst_n(por_25_n),
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
        .sd_sck(sd_sck_w), .sd_mosi(sd_mosi_w), .sd_miso(SD_MISO), .sd_cs(sd_cs_w)
    );

    // =========================================================================
    // FAT32 Reader
    // =========================================================================
    wire        fat32_done, fat32_error;
    wire [31:0] fat32_data_start, fat32_fat_start;
    wire [7:0]  fat32_spc;
    wire [31:0] fat32_root_cluster;

    fat32_reader u_fat32_reader (
        .clk(CLK_25MHz), .rst_n(por_25_n),
        .start(fat32_start), .done(fat32_done), .error(fat32_error),
        .file_count(img_count),
        .sd_read_start(sd_read_start_bl), .sd_read_addr(sd_read_addr_bl),
        .sd_read_data(sd_read_data), .sd_read_data_valid(sd_read_data_valid),
        .sd_read_data_ready(sd_read_ready_bl), .sd_read_done(sd_read_done),
        .sd_read_error(sd_read_error),
        .fat32_partition_start(), .fat32_fat_start(fat32_fat_start),
        .fat32_data_start(fat32_data_start), .fat32_spc(fat32_spc),
        .fat32_root_cluster(fat32_root_cluster),
        .cat_wr_addr(cat_wr_addr), .cat_wr_data(cat_wr_data), .cat_wr_en(cat_wr_en),
        .dbg_dir_byte0(fat32_dbg_byte0), .dbg_dir_byte8(fat32_dbg_byte8),
        .dbg_is_fat16(fat32_dbg_is_fat16)
    );
    wire [7:0] fat32_dbg_byte0, fat32_dbg_byte8;
    wire       fat32_dbg_is_fat16;

    // =========================================================================
    // Boot Loader
    // =========================================================================
    wire        boot_sdram_req, boot_sdram_write;
    wire [25:0] boot_sdram_addr;
    wire [15:0] boot_sdram_wdata;
    wire        boot_sdram_ready;

    boot_loader u_boot_loader (
        .clk(CLK_25MHz), .rst_n(por_25_n),
        .sdram_init_done(sdram_init_done),
        .boot_done(boot_done),
        .total_blocks(total_blocks),
        .flash_start(flash_start), .flash_addr(flash_start_addr),
        .flash_count(flash_byte_count), .flash_busy(flash_busy),
        .flash_data(flash_data_out), .flash_data_valid(flash_data_valid),
        .flash_data_ready(flash_data_ready),
        .sdram_req(boot_sdram_req), .sdram_req_write(boot_sdram_write),
        .sdram_req_addr(boot_sdram_addr), .sdram_req_wdata(boot_sdram_wdata),
        .sdram_req_ready(boot_sdram_ready)
    );

    // =========================================================================
    // SD Init Sequencer — triggered by register write ($02 to reg 8)
    // Runs: sd_controller.init() → fat32_reader.scan() → sets sd_ready
    // =========================================================================
    // CDC: sd_init_request (7MHz level) → 25MHz domain
    // sd_init_request stays HIGH until sd_ready/sd_error clears it in bus_interface
    reg sd_init_r1 = 1'b0, sd_init_r2 = 1'b0, sd_init_r3 = 1'b0;
    always @(posedge CLK_25MHz) begin
        sd_init_r1 <= sd_init_request;
        sd_init_r2 <= sd_init_r1;
        sd_init_r3 <= sd_init_r2;
    end
    wire sd_init_trigger = sd_init_r2 & ~sd_init_r3;  // rising edge in 25MHz domain
    reg         sd_ready_r = 1'b0;
    reg         sd_error_r = 1'b0;
    assign      sd_ready = sd_ready_r;

    // sd_seq_state declared above as forward declaration
    localparam SD_SEQ_IDLE = 3'd0,
               SD_SEQ_INIT = 3'd1,
               SD_SEQ_INIT_GAP = 3'd6,  // 1-cycle gap for controller to clear init_error
               SD_SEQ_INIT_WAIT = 3'd2,
               SD_SEQ_FAT32 = 3'd3,
               SD_SEQ_FAT32_WAIT = 3'd4,
               SD_SEQ_DONE = 3'd5;

    assign  sd_error = sd_error_r;

    always @(posedge CLK_25MHz or negedge por_25_n) begin
        if (!por_25_n) begin
            sd_seq_state <= SD_SEQ_IDLE;
            sd_ready_r   <= 1'b0;
            sd_error_r   <= 1'b0;
        end else begin
            case (sd_seq_state)
                SD_SEQ_IDLE: begin
                    if (sd_init_trigger) begin
                        sd_ready_r   <= 1'b0;
                        sd_error_r   <= 1'b0;
                        sd_seq_state <= SD_SEQ_INIT;
                    end
                end
                SD_SEQ_INIT: begin
                    // sd_init_start fires this cycle (combinational wire)
                    sd_seq_state <= SD_SEQ_INIT_GAP;
                end
                SD_SEQ_INIT_GAP: begin
                    sd_seq_state <= SD_SEQ_INIT_WAIT;
                end
                SD_SEQ_INIT_WAIT: begin
                    if (sd_init_done) begin
                        sd_seq_state <= SD_SEQ_FAT32;
                    end else if (sd_init_error) begin
                        sd_error_r   <= 1'b1;
                        sd_seq_state <= SD_SEQ_DONE;
                    end
                end
                SD_SEQ_FAT32: begin
                    // fat32_start pulse handled below
                    sd_seq_state <= SD_SEQ_FAT32_WAIT;
                end
                SD_SEQ_FAT32_WAIT: begin
                    if (fat32_done) begin
                        sd_ready_r <= 1'b1;
                        if (fat32_error)
                            sd_error_r <= 1'b1;
                        sd_seq_state <= SD_SEQ_DONE;
                    end
                end
                SD_SEQ_DONE: begin
                    sd_seq_state <= SD_SEQ_IDLE;
                end
            endcase
        end
    end

    assign sd_init_start = (sd_seq_state == SD_SEQ_INIT);
    assign fat32_start   = (sd_seq_state == SD_SEQ_FAT32);

    // =========================================================================
    // SDRAM Arbiter
    // =========================================================================
    sdram_arbiter u_sdram_arbiter (
        .clk(CLK_25MHz), .rst_n(por_25_n), .boot_done(boot_done),
        .boot_req(boot_sdram_req), .boot_write(boot_sdram_write),
        .boot_addr(boot_sdram_addr), .boot_wdata(boot_sdram_wdata),
        .boot_ready(boot_sdram_ready),
        .dev_block_read_req(dev_block_read_req), .dev_block_write_req(dev_block_write_req),
        .dev_block_num(dev_block_num), .dev_block_ready(arb_block_ready),
        .buf_addr_a(buf_addr_a), .buf_wdata_a(buf_wdata_a),
        .buf_we_a(buf_we_a), .buf_rdata_a(buf_rdata_a),
        .sdram_req(arb_sdram_req), .sdram_write(arb_sdram_write),
        .sdram_addr(arb_sdram_addr), .sdram_wdata(arb_sdram_wdata),
        .sdram_ready(sdram_req_ready),
        .sdram_rdata(sdram_req_rdata), .sdram_rdata_valid(sdram_req_rdata_valid)
    );

    // =========================================================================
    // SD Mount
    // =========================================================================
    wire [15:0] mount_data_offset;
    wire [31:0] sd_image_first_cluster;

    sd_mount u_sd_mount (
        .clk(CLK_25MHz), .rst_n(por_25_n),
        .mount_request(mount_request), .img_select(img_select),
        .s4d2_mounted(s4d2_mounted), .s4d2_loading(s4d2_loading),
        .s4d2_block_count(s4d2_block_count),
        .is_2mg(s4d2_is_2mg), .data_offset(mount_data_offset),
        .cat_rd_addr(), .cat_rd_data(8'd0),  // TODO: wire to catalog BRAM port B
        .fat32_data_start(fat32_data_start), .fat32_fat_start(fat32_fat_start),
        .fat32_spc(fat32_spc),
        .sd_read_start(sd_read_start_mt), .sd_read_addr(sd_read_addr_mt),
        .sd_read_data(sd_read_data), .sd_read_data_valid(sd_read_data_valid),
        .sd_read_data_ready(sd_read_ready_mt), .sd_read_done(sd_read_done),
        .sdram_req(mount_sdram_req), .sdram_req_write(mount_sdram_write),
        .sdram_req_addr(mount_sdram_addr), .sdram_req_wdata(mount_sdram_wdata),
        .sdram_req_ready(mount_sdram_claim & sdram_req_ready),
        .sdram_claim(mount_sdram_claim),
        .chain_wr_addr(chain_wr_addr), .chain_wr_data(chain_wr_data),
        .chain_wr_en(chain_wr_en),
        .sd_image_first_cluster(sd_image_first_cluster)
    );

    // =========================================================================
    // Write-Through + SD Persist
    // =========================================================================
    wire        fp_start;
    wire [15:0] fp_block_num;
    wire        fp_busy;

    write_through u_write_through (
        .clk(CLK_25MHz), .rst_n(por_25_n), .boot_done(boot_done),
        .arb_block_ready(arb_block_ready), .gated_block_ready(dev_block_ready),
        .dev_block_write_req(dev_block_write_req), .dev_block_num(dev_block_num),
        .fp_start(fp_start), .fp_block_num(fp_block_num), .fp_busy(fp_busy),
        .persist_enabled(s4d2_mounted)
    );

    sd_persist u_sd_persist (
        .clk(CLK_25MHz), .rst_n(por_25_n),
        .start(fp_start), .block_num(fp_block_num), .busy(fp_busy),
        .is_2mg(s4d2_is_2mg), .data_offset(mount_data_offset),
        .persist_enabled(s4d2_mounted),
        .fat32_data_start(fat32_data_start), .fat32_spc(fat32_spc),
        .chain_rd_addr(chain_rd_addr), .chain_rd_data(chain_rd_data),
        .buf_rd_addr(persist_buf_addr), .buf_rd_data(buf_rdata_a),
        .sdram_claim(persist_sdram_claim),
        .sd_write_start(sd_write_start_ps), .sd_write_addr(sd_write_addr_ps),
        .sd_write_data(sd_write_data_ps),
        .sd_write_data_valid(sd_write_data_valid_ps),
        .sd_write_data_req(sd_write_data_req), .sd_write_done(sd_write_done)
    );

endmodule
