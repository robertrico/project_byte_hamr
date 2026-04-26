`timescale 1ns / 1ps
// =============================================================================
// Block Hamr — Top Level Module (register-based block device) — Rev 2
// =============================================================================
// Rev 2 differences vs Rev 1:
//   - Clock: CLK_25MHz @ G3  ->  CLK_100MHz @ F3 (divided /4 to 25 MHz)
//   - Address bus: individual A0..A15 inputs (matches Rev 2 base LPF)
//   - GPIO header: GPIO1..GPIO12  ->  GPIO_1..GPIO_20
//   - DATA_OE: dedicated level-shifter OE pin at B20 (was on GPIO12 in Rev 1)
//   - nRES_READ: new input at A5 (Apple II reset monitor; write side on A8)
//
// Architecture (unchanged from Rev 1):
//   - Custom slot ROM serves ProDOS block device driver
//   - bus_interface provides register file at $C0n0-$C0nF
//   - 6502 reads/writes blocks through DATA register, no SmartPort protocol
//   - SDRAM stores disk image, loaded from SPI flash at boot
//
// Clock domains:
//   sig_7M   (7.16 MHz) - Apple II bus, bus_interface
//   CLK_25MHz (25 MHz)  - SDRAM controller, SPI flash, boot loader
//                         (derived from CLK_100MHz by /4 counter)
//
// Boot sequence:
//   1. POR releases (both domains)
//   2. SDRAM controller initializes
//   3. boot_loader copies disk image from SPI flash to SDRAM
//   4. bus_interface becomes ready (STATUS bit 7 + bit 0)
// =============================================================================

module block_hamr_top (
    // ---- Apple II Address Bus ----
    input wire        A0,  A1,  A2,  A3,  A4,  A5,  A6,  A7,
    input wire        A8,  A9,  A10, A11, A12, A13, A14, A15,

    // ---- Apple II Data Bus ----
    inout wire        D0, D1, D2, D3, D4, D5, D6, D7,

    // ---- Apple II Control ----
    input  wire       sig_7M,
    input  wire       Q3,
    input  wire       R_nW,
    input  wire       nDEVICE_SELECT,
    input  wire       nI_O_SELECT,
    input  wire       nI_O_STROBE,
    output wire       nRES,
    input  wire       nRES_READ,       // Rev 2: monitor Apple II reset line
    input  wire       RDY,
    output wire       nIRQ, nNMI, nDMA, nINH,
    input  wire       DMA_OUT,
    output wire       DMA_IN,
    input  wire       INT_OUT,
    output wire       INT_IN,
    input  wire       PHI0, PHI1,
    input  wire       uSync,

    // ---- Rev 2: Data bus transceiver (U12) OE. Active-low ----
    output wire       DATA_OE,

    // ---- 100 MHz System Clock (Rev 2) ----
    input wire        CLK_100MHz,

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

    // ---- GPIO Header (Rev 2: 20 pins, all outputs for debug) ----
    output wire       GPIO_1,  GPIO_2,  GPIO_3,  GPIO_4,  GPIO_5,
    output wire       GPIO_6,  GPIO_7,  GPIO_8,  GPIO_9,  GPIO_10,
    output wire       GPIO_11, GPIO_12, GPIO_13, GPIO_14, GPIO_15,
    output wire       GPIO_16, GPIO_17, GPIO_18, GPIO_19, GPIO_20
);

    // =========================================================================
    // Address Bus Bundling
    // =========================================================================
    // Submodules (addr_decoder, bus_interface) expect a bundled addr[11:0]
    // representing A0..A11. A12..A15 aren't needed since slot decode is
    // handled by nDEVICE_SELECT / nI_O_SELECT / nI_O_STROBE.
    wire [11:0] addr = {A11, A10, A9, A8, A7, A6, A5, A4, A3, A2, A1, A0};

    // =========================================================================
    // Clock: 100 MHz -> 25 MHz via EHXPLLL (ECP5 PLL)
    // =========================================================================
    // Earlier diagnostic builds used a /4 counter (clk_div[1]) to derive the
    // 25 MHz clock. That works for signal_check but the resulting net goes on
    // LOCAL routing — nextpnr only promoted the SDRAM_CLK output pad to a
    // global clock network, not the internal clock feeding sdram_controller,
    // flash_reader, boot_loader, and bus_interface. With many flops distributed
    // across the die, local-routed clocks have skew and edge-quality problems
    // that can hang fast state machines (e.g. SPI byte-level transfers).
    //
    // Using EHXPLLL forces the 25 MHz output onto a dedicated clock network,
    // matching how Rev 1's direct G3 oscillator behaved.
    //
    // Config uses external (CLKOP) feedback so nextpnr's VCO estimate matches
    // the actual VCO:
    //   VCO = CLKI / CLKI_DIV * CLKFB_DIV * CLKOP_DIV = 100 / 4 * 1 * 20 = 500 MHz
    //   CLKOP = VCO / CLKOP_DIV = 500 / 20 = 25 MHz
    //   PFD   = CLKI / CLKI_DIV = 25 MHz (must equal CLKOP / CLKFB_DIV = 25 MHz ✓)
    wire CLK_25MHz;
    wire pll_locked;

    (* FREQUENCY_PIN_CLKI="100" *)
    (* FREQUENCY_PIN_CLKOP="25" *)
    (* ICP_CURRENT="12" *)
    (* LPF_RESISTOR="8" *)
    EHXPLLL #(
        .PLLRST_ENA       ("DISABLED"),
        .INTFB_WAKE       ("DISABLED"),
        .STDBY_ENABLE     ("DISABLED"),
        .DPHASE_SOURCE    ("DISABLED"),
        .OUTDIVIDER_MUXA  ("DIVA"),
        .OUTDIVIDER_MUXB  ("DIVB"),
        .OUTDIVIDER_MUXC  ("DIVC"),
        .OUTDIVIDER_MUXD  ("DIVD"),
        .CLKI_DIV         (4),
        .CLKOP_ENABLE     ("ENABLED"),
        .CLKOP_DIV        (20),
        .CLKOP_CPHASE     (9),              // 50% duty cycle for CLKOP_DIV=20
        .CLKOP_FPHASE     (0),
        .FEEDBK_PATH      ("CLKOP"),
        .CLKFB_DIV        (1)
    ) u_pll (
        .CLKI             (CLK_100MHz),
        .CLKFB            (CLK_25MHz),
        .PHASESEL0        (1'b0),
        .PHASESEL1        (1'b0),
        .PHASEDIR         (1'b0),
        .PHASESTEP        (1'b0),
        .PHASELOADREG     (1'b0),
        .STDBY            (1'b0),
        .PLLWAKESYNC      (1'b0),
        .RST              (1'b0),
        .ENCLKOP          (1'b0),
        .CLKOP            (CLK_25MHz),
        .LOCK             (pll_locked)
    );

    // =========================================================================
    // Internal Signals
    // =========================================================================
    wire [7:0] rom_data;
    wire       rom_oe_raw;
    // We don't use $C800 — only serve $Cn slot ROM
    wire       rom_oe = ~nI_O_SELECT;
    wire [7:0] bus_data_out;
    wire       boot_done;
    wire       sdram_init_done;
    wire [15:0] total_blocks;

    // =========================================================================
    // Power-On Reset (7 MHz domain)
    // =========================================================================
    assign nRES = 1'b1;  // open-drain, always released

    reg [3:0] por_counter = 4'd0;
    wire      por_n = &por_counter;

    always @(posedge sig_7M) begin
        if (!por_n)
            por_counter <= por_counter + 4'd1;
    end

    // =========================================================================
    // Power-On Reset (25 MHz domain) — held until PLL is locked
    // =========================================================================
    reg [7:0] por_25_counter = 8'd0;
    wire      por_25_n = &por_25_counter;

    always @(posedge CLK_25MHz) begin
        if (!pll_locked)
            por_25_counter <= 8'd0;
        else if (!por_25_n)
            por_25_counter <= por_25_counter + 8'd1;
    end

    // =========================================================================
    // Apple II Reset (nRES_READ) — 2-FF synchronizers per clock domain
    // =========================================================================
    // nRES_READ is the Apple II reset line monitored at pad A5. It comes from
    // an asynchronous source so we synchronize it separately into each clock
    // domain. Reset to 0 so a bouncing/floating line looks like reset-asserted
    // until the synchronizer fills with clean ones.
    reg [1:0] nres_sync_7m  = 2'b00;
    reg [1:0] nres_sync_25m = 2'b00;

    always @(posedge sig_7M) begin
        nres_sync_7m <= {nres_sync_7m[0], nRES_READ};
    end

    always @(posedge CLK_25MHz) begin
        nres_sync_25m <= {nres_sync_25m[0], nRES_READ};
    end

    wire nRES_READ_sync7  = nres_sync_7m[1];
    wire nRES_READ_sync25 = nres_sync_25m[1];

    // Combined reset (active low): POR AND Apple II reset, per domain.
    wire rst_7m_n  = por_n    & nRES_READ_sync7;
    wire rst_25m_n = por_25_n & nRES_READ_sync25;

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

    // =========================================================================
    // Rev 2 DATA_OE — dedicated level-shifter enable (active-low)
    // =========================================================================
    // Enable U12 whenever our slot is being accessed so the Apple II data bus
    // is gated through to the FPGA. In Rev 1 this signal rode on GPIO12.
    wire lvl_shift_oe = !nDEVICE_SELECT || rom_oe;
    assign DATA_OE = ~lvl_shift_oe;

    // =========================================================================
    // GPIO Debug (Rev 2: 20 pins, but only first few are meaningful for now)
    // =========================================================================
    assign GPIO_1  = boot_done;
    assign GPIO_2  = wt_claim;
    assign GPIO_3  = fw_busy;
    assign GPIO_4  = dev_block_ready;
    assign GPIO_5  = sdram_init_done;
    assign GPIO_6  = nRES_READ;        // Rev 2: visible reset monitor
    assign GPIO_7  = 1'b0;
    assign GPIO_8  = 1'b0;
    assign GPIO_9  = 1'b0;
    assign GPIO_10 = 1'b0;
    assign GPIO_11 = 1'b0;
    assign GPIO_12 = 1'b0;
    assign GPIO_13 = 1'b0;
    assign GPIO_14 = 1'b0;
    assign GPIO_15 = 1'b0;
    assign GPIO_16 = 1'b0;
    assign GPIO_17 = 1'b0;
    assign GPIO_18 = 1'b0;
    assign GPIO_19 = 1'b0;
    assign GPIO_20 = 1'b0;

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
        .nRES                (rst_7m_n),
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
    // Bus Interface — register file for block device commands
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
        .rst_n            (rst_7m_n),
        .addr             (addr[3:0]),
        .data_in          ({D7, D6, D5, D4, D3, D2, D1, D0}),
        .data_out         (bus_data_out),
        .nDEVICE_SELECT   (nDEVICE_SELECT),
        .R_nW             (R_nW),
        .boot_done        (boot_done),
        .total_blocks     (total_blocks),
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
        .rst_n          (rst_25m_n),
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
        .rst_n          (rst_25m_n),
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
        .rst_n          (rst_25m_n),
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
        .rst_n           (rst_25m_n),
        .sdram_init_done (sdram_init_done),
        .boot_done       (boot_done),
        .total_blocks    (total_blocks),
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
        .rst_n              (rst_25m_n),
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
    wire [11:0] fp_sector;
    wire        fp_busy;

    write_through u_write_through (
        .clk              (CLK_25MHz),
        .rst_n            (rst_25m_n),
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
        .rst_n            (rst_25m_n),
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
