`timescale 1ns / 1ps
// =============================================================================
// Block Hamr — Top Level Module (register-based block device)
// =============================================================================
// Architecture:
//   - Custom slot ROM serves ProDOS block device driver
//   - bus_interface provides register file at $C0n0-$C0nF
//   - 6502 reads/writes blocks through DATA register, no SmartPort protocol
//   - SDRAM stores disk image, loaded from SPI flash at boot
//
// Future: IWM re-added for external SmartPort devices on DB-19
//
// Clock domains:
//   sig_7M   (7.16 MHz) - Apple II bus, bus_interface
//   CLK_25MHz (25 MHz)  - SDRAM controller, SPI flash, boot loader
//
// Boot sequence:
//   1. POR releases (both domains)
//   2. SDRAM controller initializes
//   3. boot_loader copies disk image from SPI flash to SDRAM
//   4. bus_interface becomes ready (STATUS bit 7 + bit 0)
// =============================================================================

module block_hamr_top (
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

    // ---- SDRAM (AS4C32M16SB, 64MB) ----
    output wire       SDRAM_CLK,
    output wire       SDRAM_CKE,
    output wire       SDRAM_nCS,
    output wire       SDRAM_nRAS,
    output wire       SDRAM_nCAS,
    output wire       SDRAM_nWE,
    output wire       SDRAM_DQM0,
    output wire       SDRAM_DQM1,
    output wire       SDRAM_BA0,
    output wire       SDRAM_BA1,
    output wire       SDRAM_A0, SDRAM_A1, SDRAM_A2, SDRAM_A3,
    output wire       SDRAM_A4, SDRAM_A5, SDRAM_A6, SDRAM_A7,
    output wire       SDRAM_A8, SDRAM_A9, SDRAM_A10, SDRAM_A11, SDRAM_A12,
    inout  wire       SDRAM_D0, SDRAM_D1, SDRAM_D2, SDRAM_D3,
    inout  wire       SDRAM_D4, SDRAM_D5, SDRAM_D6, SDRAM_D7,
    inout  wire       SDRAM_D8, SDRAM_D9, SDRAM_D10, SDRAM_D11,
    inout  wire       SDRAM_D12, SDRAM_D13, SDRAM_D14, SDRAM_D15,

    // ---- SPI Flash (IS25LP128F, 16MB) ----
    output wire       FLASH_nCS,
    output wire       FLASH_MOSI,
    input  wire       FLASH_MISO,
    output wire       FLASH_nWP,
    output wire       FLASH_nHOLD,

    // ---- GPIO Header (debug) ----
    output wire       GPIO1, GPIO2, GPIO3, GPIO4, GPIO5,
    output wire       GPIO6, GPIO7,
    input  wire       GPIO8,
    input  wire       GPIO9,
    output wire       GPIO10, GPIO11, GPIO12
);

    // =========================================================================
    // Internal Signals
    // =========================================================================
    wire [7:0] rom_data;
    wire       rom_oe_raw;
    // We don't use $C800 — only serve $Cn slot ROM
    wire       rom_oe = ~nI_O_SELECT;
    wire       rom_expansion_active;
    wire [7:0] bus_data_out;
    wire       boot_done;
    wire       sdram_init_done;

    // =========================================================================
    // Power-On Reset (7 MHz domain)
    // =========================================================================
    assign nRES = 1'b1;

    reg [3:0] por_counter = 4'd0;
    wire      por_n = &por_counter;

    always @(posedge sig_7M) begin
        if (!por_n)
            por_counter <= por_counter + 4'd1;
    end

    // =========================================================================
    // Power-On Reset (25 MHz domain)
    // =========================================================================
    reg [7:0] por_25_counter = 8'd0;
    wire      por_25_n = &por_25_counter;

    always @(posedge CLK_25MHz) begin
        if (!por_25_n)
            por_25_counter <= por_25_counter + 8'd1;
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
    // Data Bus — mux ROM and register reads
    // =========================================================================
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

    // Level shifter OE
    wire lvl_shift_oe = !nDEVICE_SELECT || rom_oe;
    assign GPIO12 = ~lvl_shift_oe;

    // =========================================================================
    // GPIO Debug — minimal for now
    // =========================================================================
    assign GPIO1  = boot_done;
    assign GPIO2  = wt_claim;
    assign GPIO3  = fw_busy;
    assign GPIO4  = dev_block_ready;
    assign GPIO5  = 1'b0;
    assign GPIO6  = 1'b0;
    assign GPIO7  = 1'b0;
    assign GPIO10 = 1'b0;
    assign GPIO11 = 1'b0;

    // =========================================================================
    // Address Decoder
    // =========================================================================
    // Override: we don't use $C800 shared ROM (driver fits in $Cn page).
    // Force rom_expansion_active=0 so we never claim $C800-$CFFF.
    // Without this, accessing our $C4xx sets rom_expansion_active,
    // and our card serves $FF for $C800 accesses, blocking other cards
    // (80-col, etc.) and breaking BASIC.SYSTEM's ProDOS command hooks.
    wire rom_expansion_active_unused;
    addr_decoder u_addr_decoder (
        .addr                (addr),
        .clk                 (sig_7M),
        .nI_O_STROBE         (nI_O_STROBE),
        .nI_O_SELECT         (nI_O_SELECT),
        .nRES                (por_n),
        .rom_oe              (rom_oe_raw),  // unused — we override with ~nI_O_SELECT
        .rom_expansion_active(rom_expansion_active_unused)
    );

    // =========================================================================
    // Boot ROM — serves slot ROM content to Apple II
    // =========================================================================
    wire [11:0] rom_addr = !nI_O_SELECT ? {4'b0100, addr[7:0]}
                                        : {1'b1, addr[10:0]};

    boot_rom u_boot_rom (
        .clk  (sig_7M),
        .addr (rom_addr),
        .data (rom_data)
    );

    // =========================================================================
    // Bus Interface — register file replacing SmartPort protocol
    // =========================================================================
    wire [8:0]  buf_addr_b;
    wire [7:0]  buf_rdata_b;
    wire [7:0]  buf_wdata_b;
    wire        buf_we_b;
    wire        dev_block_read_req;
    wire        dev_block_write_req;
    wire [15:0] dev_block_num;
    wire        arb_block_ready;    // raw from arbiter
    wire        dev_block_ready;    // gated by write_through

    bus_interface u_bus_interface (
        .clk              (sig_7M),
        .rst_n            (por_n),
        .addr             (addr[3:0]),
        .data_in          ({D7, D6, D5, D4, D3, D2, D1, D0}),
        .data_out         (bus_data_out),
        .nDEVICE_SELECT   (nDEVICE_SELECT),
        .R_nW             (R_nW),
        .boot_done        (boot_done),
        .buf_addr         (buf_addr_b),
        .buf_rdata        (buf_rdata_b),
        .buf_wdata        (buf_wdata_b),
        .buf_we           (buf_we_b),
        .block_read_req   (dev_block_read_req),
        .block_write_req  (dev_block_write_req),
        .block_num        (dev_block_num),
        .block_ready      (dev_block_ready),
        .block_num_out    ()   // used via dev_block_num wire
    );

    // =========================================================================
    // Block Buffer — dual-port BRAM bridging 25 MHz and 7 MHz
    // =========================================================================
    wire [8:0]  buf_addr_a;
    wire [7:0]  buf_wdata_a;
    wire        buf_we_a;
    wire [7:0]  buf_rdata_a;

    block_buffer u_block_buffer (
        .clk_a   (CLK_25MHz),
        .addr_a  (buf_addr_a),
        .wdata_a (buf_wdata_a),
        .we_a    (buf_we_a),
        .rdata_a (buf_rdata_a),
        .clk_b   (sig_7M),
        .addr_b  (buf_addr_b),
        .wdata_b (buf_wdata_b),
        .we_b    (buf_we_b),
        .rdata_b (buf_rdata_b)
    );

    // =========================================================================
    // SDRAM Address/Data Bus Mapping
    // =========================================================================
    wire [12:0] sdram_a;
    wire [15:0] sdram_dq = {SDRAM_D15, SDRAM_D14, SDRAM_D13, SDRAM_D12,
                             SDRAM_D11, SDRAM_D10, SDRAM_D9,  SDRAM_D8,
                             SDRAM_D7,  SDRAM_D6,  SDRAM_D5,  SDRAM_D4,
                             SDRAM_D3,  SDRAM_D2,  SDRAM_D1,  SDRAM_D0};

    assign SDRAM_A0  = sdram_a[0];
    assign SDRAM_A1  = sdram_a[1];
    assign SDRAM_A2  = sdram_a[2];
    assign SDRAM_A3  = sdram_a[3];
    assign SDRAM_A4  = sdram_a[4];
    assign SDRAM_A5  = sdram_a[5];
    assign SDRAM_A6  = sdram_a[6];
    assign SDRAM_A7  = sdram_a[7];
    assign SDRAM_A8  = sdram_a[8];
    assign SDRAM_A9  = sdram_a[9];
    assign SDRAM_A10 = sdram_a[10];
    assign SDRAM_A11 = sdram_a[11];
    assign SDRAM_A12 = sdram_a[12];

    // =========================================================================
    // SDRAM Controller (25 MHz) + persist SDRAM mux
    // =========================================================================
    wire        arb_sdram_req;
    wire        arb_sdram_write;
    wire [25:0] arb_sdram_addr;
    wire [15:0] arb_sdram_wdata;
    wire        sdram_req_ready;
    wire [15:0] sdram_req_rdata;
    wire        sdram_req_rdata_valid;

    // SDRAM mux: wt_claim switches SDRAM from arbiter to write_through
    wire        wt_sdram_req;
    wire [25:0] wt_sdram_addr;
    wire [15:0] wt_sdram_wdata;
    wire        wt_claim;

    wire        mux_sdram_req   = wt_claim ? wt_sdram_req   : arb_sdram_req;
    wire        mux_sdram_write = wt_claim ? 1'b0           : arb_sdram_write;
    wire [25:0] mux_sdram_addr  = wt_claim ? wt_sdram_addr  : arb_sdram_addr;
    wire [15:0] mux_sdram_wdata = wt_claim ? wt_sdram_wdata : arb_sdram_wdata;

    sdram_controller u_sdram_controller (
        .clk            (CLK_25MHz),
        .rst_n          (por_25_n),
        .init_done      (sdram_init_done),
        .pause_refresh  (1'b0),
        .req            (mux_sdram_req),
        .req_write      (mux_sdram_write),
        .req_addr       (mux_sdram_addr),
        .req_wdata      (mux_sdram_wdata),
        .req_ready      (sdram_req_ready),
        .req_rdata      (sdram_req_rdata),
        .req_rdata_valid(sdram_req_rdata_valid),
        .SDRAM_CLK      (SDRAM_CLK),
        .SDRAM_CKE      (SDRAM_CKE),
        .SDRAM_nCS      (SDRAM_nCS),
        .SDRAM_nRAS     (SDRAM_nRAS),
        .SDRAM_nCAS     (SDRAM_nCAS),
        .SDRAM_nWE      (SDRAM_nWE),
        .SDRAM_DQM0     (SDRAM_DQM0),
        .SDRAM_DQM1     (SDRAM_DQM1),
        .SDRAM_BA0      (SDRAM_BA0),
        .SDRAM_BA1      (SDRAM_BA1),
        .SDRAM_A        (sdram_a),
        .SDRAM_DQ       (sdram_dq)
    );

    // =========================================================================
    // SPI Flash — shared bus (USRMCLK is singleton on ECP5)
    // =========================================================================
    // Reader owns SPI during boot. Writer owns SPI after boot_done.
    // They never overlap, so a simple boot_done mux is safe.

    // ---- Flash reader signals ----
    wire        flash_busy;
    wire        flash_done;
    wire [7:0]  flash_data_out;
    wire        flash_data_valid;
    wire        flash_data_ready;
    wire        flash_start;
    wire [23:0] flash_start_addr;
    wire [23:0] flash_byte_count;
    wire        reader_sck;
    wire        reader_ncs;
    wire        reader_mosi;

    // ---- Flash writer signals (active after boot_done) ----
    wire        writer_sck;
    wire        writer_ncs;
    wire        writer_mosi;

    // ---- SPI bus mux: reader during boot, writer after ----
    wire spi_sck  = boot_done ? writer_sck  : reader_sck;
    wire spi_ncs  = boot_done ? writer_ncs  : reader_ncs;
    wire spi_mosi = boot_done ? writer_mosi : reader_mosi;

    assign FLASH_nCS  = spi_ncs;
    assign FLASH_MOSI = spi_mosi;
    assign FLASH_nWP   = 1'b1;
    assign FLASH_nHOLD = 1'b1;

    // ---- USRMCLK: ECP5 singleton for post-config SPI clock ----
    `ifndef SYNTHESIS
        // Simulation: spi_sck visible as a wire (no USRMCLK primitive)
    `else
        USRMCLK u_usrmclk (
            .USRMCLKI (spi_sck),
            .USRMCLKTS(1'b0)
        );
    `endif

    // ---- Flash reader (active during boot) ----
    flash_reader u_flash_reader (
        .clk            (CLK_25MHz),
        .rst_n          (por_25_n),
        .start          (flash_start),
        .start_addr     (flash_start_addr),
        .byte_count     (flash_byte_count),
        .busy           (flash_busy),
        .done           (flash_done),
        .data_out       (flash_data_out),
        .data_valid     (flash_data_valid),
        .data_ready     (flash_data_ready),
        .flash_ncs      (reader_ncs),
        .flash_mosi     (reader_mosi),
        .flash_miso     (FLASH_MISO),
        .flash_nwp      (),             // driven at top level
        .flash_nhold    (),             // driven at top level
        .flash_sck_pin  (reader_sck)
    );

    // ---- Flash writer (active after boot_done for background persist) ----
    wire        fw_start_erase;
    wire        fw_start_program;
    wire [23:0] fw_flash_addr;
    wire [7:0]  fw_prog_data;
    wire        fw_prog_data_valid;
    wire        fw_prog_data_req;
    wire        fw_busy;
    wire        fw_done;

    flash_writer u_flash_writer (
        .clk            (CLK_25MHz),
        .rst_n          (por_25_n),
        .start_erase    (fw_start_erase),
        .start_program  (fw_start_program),
        .flash_addr     (fw_flash_addr),
        .prog_data      (fw_prog_data),
        .prog_data_valid(fw_prog_data_valid),
        .prog_data_req  (fw_prog_data_req),
        .busy           (fw_busy),
        .done           (fw_done),
        .spi_sck        (writer_sck),
        .spi_ncs        (writer_ncs),
        .spi_mosi       (writer_mosi),
        .spi_miso       (FLASH_MISO)
    );

    // =========================================================================
    // Boot Loader (25 MHz)
    // =========================================================================
    wire        boot_sdram_req;
    wire        boot_sdram_write;
    wire [25:0] boot_sdram_addr;
    wire [15:0] boot_sdram_wdata;
    wire        boot_sdram_ready;

    boot_loader u_boot_loader (
        .clk             (CLK_25MHz),
        .rst_n           (por_25_n),
        .sdram_init_done (sdram_init_done),
        .boot_done       (boot_done),
        .flash_start     (flash_start),
        .flash_addr      (flash_start_addr),
        .flash_count     (flash_byte_count),
        .flash_busy      (flash_busy),
        .flash_data      (flash_data_out),
        .flash_data_valid(flash_data_valid),
        .flash_data_ready(flash_data_ready),
        .sdram_req       (boot_sdram_req),
        .sdram_req_write (boot_sdram_write),
        .sdram_req_addr  (boot_sdram_addr),
        .sdram_req_wdata (boot_sdram_wdata),
        .sdram_req_ready (boot_sdram_ready),
        .debug_state     ()
    );

    // =========================================================================
    // SDRAM Arbiter (25 MHz)
    // =========================================================================
    sdram_arbiter u_sdram_arbiter (
        .clk                (CLK_25MHz),
        .rst_n              (por_25_n),
        .boot_done          (boot_done),
        .boot_req           (boot_sdram_req),
        .boot_write         (boot_sdram_write),
        .boot_addr          (boot_sdram_addr),
        .boot_wdata         (boot_sdram_wdata),
        .boot_ready         (boot_sdram_ready),
        .dev_block_read_req (dev_block_read_req),
        .dev_block_write_req(dev_block_write_req),
        .dev_block_num      (dev_block_num),
        .dev_block_ready    (arb_block_ready),
        .buf_addr_a         (buf_addr_a),
        .buf_wdata_a        (buf_wdata_a),
        .buf_we_a           (buf_we_a),
        .buf_rdata_a        (buf_rdata_a),
        .sdram_req          (arb_sdram_req),
        .sdram_write        (arb_sdram_write),
        .sdram_addr         (arb_sdram_addr),
        .sdram_wdata        (arb_sdram_wdata),
        .sdram_ready        (sdram_req_ready),
        .sdram_rdata        (sdram_req_rdata),
        .sdram_rdata_valid  (sdram_req_rdata_valid)
    );

    // =========================================================================
    // Write-Through — gating only (separate module from flash ops)
    // =========================================================================
    wire        fp_start;
    wire [5:0]  fp_sector;
    wire        fp_busy;

    write_through u_write_through (
        .clk              (CLK_25MHz),
        .rst_n            (por_25_n),
        .boot_done        (boot_done),
        .arb_block_ready  (arb_block_ready),
        .gated_block_ready(dev_block_ready),
        .dev_block_write_req(dev_block_write_req),
        .dev_block_num    (dev_block_num),
        .fp_start         (fp_start),
        .fp_sector        (fp_sector),
        .fp_busy          (fp_busy)
    );

    // =========================================================================
    // Flash Persist — flash erase+program in separate module
    // =========================================================================
    flash_persist u_flash_persist (
        .clk              (CLK_25MHz),
        .rst_n            (por_25_n),
        .start            (fp_start),
        .sector_num       (fp_sector),
        .busy             (fp_busy),
        .sdram_req        (wt_sdram_req),
        .sdram_addr       (wt_sdram_addr),
        .sdram_wdata      (wt_sdram_wdata),
        .sdram_ready      (wt_claim & sdram_req_ready),
        .sdram_rdata      (sdram_req_rdata),
        .sdram_rdata_valid(wt_claim & sdram_req_rdata_valid),
        .fw_start_erase   (fw_start_erase),
        .fw_start_program (fw_start_program),
        .fw_flash_addr    (fw_flash_addr),
        .fw_prog_data     (fw_prog_data),
        .fw_prog_data_valid(fw_prog_data_valid),
        .fw_prog_data_req (fw_prog_data_req),
        .fw_busy          (fw_busy),
        .sdram_claim      (wt_claim)
    );

endmodule
