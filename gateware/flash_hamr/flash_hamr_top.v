`timescale 1ns / 1ps
// =============================================================================
// Flash Hamr — Top Level Module (Phase 5: Full integration)
// =============================================================================
// PicoRV32 CPU + FatFS handles SD card. Apple II bus infrastructure from
// block_hamr (SDRAM, arbiter, bus_interface, boot_loader, flash_reader).
//
// SDRAM mux: CPU claims SDRAM via cpu_sdram_claim (same pattern as sd_mount).
// Block buffer Port A mux: CPU claims via cpu_buf_claim for magic blocks.
// Magic block intercept: $FFFF/$FFFE -> CPU fills buffer instead of arbiter.
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

    // ---- SPI Flash (menu volume) ----
    output wire       FLASH_nCS, FLASH_MOSI,
    input  wire       FLASH_MISO,
    output wire       FLASH_nWP, FLASH_nHOLD,

    // ---- SD Card SPI (GPIO1-4) ----
    output wire       SD_CS, SD_SCK, SD_MOSI,
    input  wire       SD_MISO,

    // ---- GPIO ----
    output wire       GPIO5, GPIO6, GPIO7, GPIO8,
    input  wire       GPIO9,
    output wire       GPIO10, GPIO11, GPIO12
);

    // =========================================================================
    // POR
    // =========================================================================
    assign nRES = 1'b1;

    reg [3:0] por_counter_7m = 4'd0;
    wire      por_7m_n = &por_counter_7m;
    always @(posedge sig_7M) begin
        if (!por_7m_n) por_counter_7m <= por_counter_7m + 4'd1;
    end

    reg [7:0] por_counter_25m = 8'd0;
    wire      por_25m_n = &por_counter_25m;
    always @(posedge CLK_25MHz) begin
        if (!por_25m_n) por_counter_25m <= por_counter_25m + 8'd1;
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
    wire       rom_oe;  // driven by addr_decoder
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
    // Address Decoder + Boot ROM
    // =========================================================================
    wire rom_expansion_active;
    addr_decoder u_addr_decoder (
        .addr(addr), .clk(sig_7M), .nI_O_STROBE(nI_O_STROBE),
        .nI_O_SELECT(nI_O_SELECT), .nRES(por_7m_n),
        .rom_oe(rom_oe), .rom_expansion_active(rom_expansion_active)
    );

    wire [11:0] rom_addr = !nI_O_SELECT ? {4'b0100, addr[7:0]} : {1'b1, addr[10:0]};
    boot_rom u_boot_rom (.clk(sig_7M), .addr(rom_addr), .data(rom_data));

    // =========================================================================
    // Bus Interface (7 MHz) — regs 0-7 unchanged from block_hamr
    // =========================================================================
    wire [8:0]  buf_addr_b;
    wire [7:0]  buf_rdata_b, buf_wdata_b;
    wire        buf_we_b;
    wire        dev_block_read_req_raw, dev_block_write_req;
    wire [15:0] dev_block_num;
    wire        dev_block_ready;

    // ---- Magic block intercept ----
    // $FFFF/$FFFE are magic blocks handled by CPU, not arbiter
    wire is_magic_block = (dev_block_num[15:1] == 15'h7FFF);  // $FFFE or $FFFF
    wire dev_block_read_req = dev_block_read_req_raw & ~is_magic_block;  // arbiter sees non-magic only
    wire magic_block_req    = dev_block_read_req_raw & is_magic_block;   // CPU sees magic only

    // CPU signals block ready for magic blocks (25MHz -> 7MHz CDC)
    wire cpu_block_ready_25;  // from cpu_soc
    reg  cpu_br_s1, cpu_br_s2;
    always @(posedge sig_7M or negedge por_7m_n) begin
        if (!por_7m_n) begin cpu_br_s1 <= 0; cpu_br_s2 <= 0; end
        else begin cpu_br_s1 <= cpu_block_ready_25; cpu_br_s2 <= cpu_br_s1; end
    end

    // Magic block active flag — forces dev_block_ready LOW during magic block
    // processing so bus_interface gets a proper LOW→HIGH edge when CPU finishes.
    // Without this, arb_block_ready is already HIGH from the last normal read,
    // and dev_block_ready never transitions, so br_rise never fires.
    reg magic_active, magic_req_prev;
    always @(posedge sig_7M or negedge por_7m_n) begin
        if (!por_7m_n) begin
            magic_active   <= 1'b0;
            magic_req_prev <= 1'b0;
        end else begin
            magic_req_prev <= magic_block_req;
            if (magic_block_req & ~magic_req_prev)
                magic_active <= 1'b1;   // rising edge of magic request
            if (cpu_br_s2)
                magic_active <= 1'b0;   // CPU done, release
        end
    end

    wire gated_block_ready;
    assign dev_block_ready = magic_active ? cpu_br_s2 : gated_block_ready;

    // Mailbox signals (bus side)
    wire [7:0]  mbox_bus_status_flags;
    wire [15:0] mbox_bus_total_blocks;
    wire [7:0]  sd_cmd_data;
    wire        sd_cmd_wr;
    wire [3:0]  img_select;

    bus_interface u_bus_interface (
        .clk(sig_7M), .rst_n(por_7m_n),
        .addr(addr[3:0]),
        .data_in({D7, D6, D5, D4, D3, D2, D1, D0}),
        .data_out(bus_data_out),
        .nDEVICE_SELECT(nDEVICE_SELECT), .R_nW(R_nW),
        .boot_done(boot_done),
        .total_blocks(mbox_bus_total_blocks),
        .buf_addr(buf_addr_b), .buf_rdata(buf_rdata_b),
        .buf_wdata(buf_wdata_b), .buf_we(buf_we_b),
        .block_read_req(dev_block_read_req_raw),
        .block_write_req(dev_block_write_req),
        .block_num(dev_block_num), .block_ready(dev_block_ready),
        .block_num_out(),
        // SD management — status from mailbox
        .sd_ready(mbox_bus_status_flags[0]),
        .sd_error_in(mbox_bus_status_flags[1]),
        .s4d2_mounted(mbox_bus_status_flags[2]),
        .s4d2_loading(mbox_bus_status_flags[3]),
        .img_count(mbox_bus_status_flags[7:4]),  // packed into upper nibble
        // Catalog — unused, catalog comes via magic block reads
        .cat_rd_addr(), .cat_rd_data(8'd0),
        .img_select(img_select), .img_name_idx(),
        // SD commands -> mailbox
        .mount_request(),
        .sd_init_request(),
        .sd_cmd_data(sd_cmd_data),
        .sd_cmd_wr(sd_cmd_wr),
        .s4d2_block_count(mbox_bus_total_blocks),
        .s4d2_is_2mg(1'b0),
        // Debug
        .dbg_dir_byte0(8'd0), .dbg_dir_byte8(8'd0),
        .dbg_is_fat16(1'b0),
        .mount_dbg_sectors(16'd0), .mount_dbg_state(5'd0),
        .mount_dbg_fat_entry(16'd0)
    );

    // =========================================================================
    // Block Buffer (dual-port BRAM, 7MHz/25MHz)
    // Port A: 25MHz — muxed between sdram_arbiter and CPU
    // Port B: 7MHz — bus_interface
    // =========================================================================
    wire [8:0]  buf_addr_a;
    wire [7:0]  buf_wdata_a, buf_rdata_a;
    wire        buf_we_a;

    // CPU block buffer access (for magic block fills and persist reads)
    wire        cpu_buf_claim;
    wire [8:0]  cpu_buf_addr;
    wire [7:0]  cpu_buf_wdata;
    wire        cpu_buf_we;

    // Port A mux: CPU or arbiter (CPU claims during magic block / persist)
    wire [8:0]  mux_buf_addr_a  = cpu_buf_claim ? cpu_buf_addr  : buf_addr_a;
    wire [7:0]  mux_buf_wdata_a = cpu_buf_claim ? cpu_buf_wdata : buf_wdata_a;
    wire        mux_buf_we_a    = cpu_buf_claim ? cpu_buf_we    : buf_we_a;

    block_buffer u_block_buffer (
        .clk_a(CLK_25MHz), .addr_a(mux_buf_addr_a), .wdata_a(mux_buf_wdata_a),
        .we_a(mux_buf_we_a), .rdata_a(buf_rdata_a),
        .clk_b(sig_7M), .addr_b(buf_addr_b), .wdata_b(buf_wdata_b),
        .we_b(buf_we_b), .rdata_b(buf_rdata_b)
    );

    // =========================================================================
    // SDRAM
    // =========================================================================
    wire [12:0] sdram_a;
    wire [15:0] sdram_dq = {SDRAM_D15, SDRAM_D14, SDRAM_D13, SDRAM_D12,
                             SDRAM_D11, SDRAM_D10, SDRAM_D9,  SDRAM_D8,
                             SDRAM_D7,  SDRAM_D6,  SDRAM_D5,  SDRAM_D4,
                             SDRAM_D3,  SDRAM_D2,  SDRAM_D1,  SDRAM_D0};
    assign {SDRAM_A12, SDRAM_A11, SDRAM_A10, SDRAM_A9, SDRAM_A8,
            SDRAM_A7, SDRAM_A6, SDRAM_A5, SDRAM_A4,
            SDRAM_A3, SDRAM_A2, SDRAM_A1, SDRAM_A0} = sdram_a;

    // SDRAM mux: CPU claims during mount/persist (same pattern as sd_mount)
    wire        arb_sdram_req, arb_sdram_write;
    wire [25:0] arb_sdram_addr;
    wire [15:0] arb_sdram_wdata;
    wire        sdram_req_ready;
    wire [15:0] sdram_req_rdata;
    wire        sdram_req_rdata_valid;
    wire        sdram_init_done;

    wire        cpu_sdram_claim;
    wire        cpu_sdram_req, cpu_sdram_write;
    wire [25:0] cpu_sdram_addr;
    wire [15:0] cpu_sdram_wdata;

    wire mux_sdram_req   = cpu_sdram_claim ? cpu_sdram_req   : arb_sdram_req;
    wire mux_sdram_write = cpu_sdram_claim ? cpu_sdram_write : arb_sdram_write;
    wire [25:0] mux_sdram_addr  = cpu_sdram_claim ? cpu_sdram_addr  : arb_sdram_addr;
    wire [15:0] mux_sdram_wdata = cpu_sdram_claim ? cpu_sdram_wdata : arb_sdram_wdata;

    sdram_controller u_sdram_controller (
        .clk(CLK_25MHz), .rst_n(por_25m_n), .init_done(sdram_init_done),
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
    // SPI Flash (menu volume boot)
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
        .clk(CLK_25MHz), .rst_n(por_25m_n),
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
    // Boot Loader (SPI flash -> SDRAM at power-on)
    // =========================================================================
    wire        boot_done;
    wire [15:0] boot_total_blocks;
    wire        boot_sdram_req, boot_sdram_write;
    wire [25:0] boot_sdram_addr;
    wire [15:0] boot_sdram_wdata;
    wire        boot_sdram_ready;

    boot_loader u_boot_loader (
        .clk(CLK_25MHz), .rst_n(por_25m_n),
        .sdram_init_done(sdram_init_done),
        .boot_done(boot_done),
        .total_blocks(boot_total_blocks),
        .flash_start(flash_start), .flash_addr(flash_start_addr),
        .flash_count(flash_byte_count), .flash_busy(flash_busy),
        .flash_data(flash_data_out), .flash_data_valid(flash_data_valid),
        .flash_data_ready(flash_data_ready),
        .sdram_req(boot_sdram_req), .sdram_req_write(boot_sdram_write),
        .sdram_req_addr(boot_sdram_addr), .sdram_req_wdata(boot_sdram_wdata),
        .sdram_req_ready(boot_sdram_ready)
    );

    // =========================================================================
    // SDRAM Arbiter (DO NOT MODIFY)
    // =========================================================================
    wire arb_block_ready;

    sdram_arbiter u_sdram_arbiter (
        .clk(CLK_25MHz), .rst_n(por_25m_n), .boot_done(boot_done),
        .boot_req(boot_sdram_req), .boot_write(boot_sdram_write),
        .boot_addr(boot_sdram_addr), .boot_wdata(boot_sdram_wdata),
        .boot_ready(boot_sdram_ready),
        .dev_block_read_req(dev_block_read_req),
        .dev_block_write_req(dev_block_write_req),
        .dev_block_num(dev_block_num), .dev_block_ready(arb_block_ready),
        .buf_addr_a(buf_addr_a), .buf_wdata_a(buf_wdata_a),
        .buf_we_a(buf_we_a), .buf_rdata_a(buf_rdata_a),
        .sdram_req(arb_sdram_req), .sdram_write(arb_sdram_write),
        .sdram_addr(arb_sdram_addr), .sdram_wdata(arb_sdram_wdata),
        .sdram_ready(sdram_req_ready),
        .sdram_rdata(sdram_req_rdata), .sdram_rdata_valid(sdram_req_rdata_valid)
    );

    // =========================================================================
    // Block Ready Gate (renamed write_through — own module, DO NOT MERGE)
    // =========================================================================
    wire [15:0] persist_block;
    wire        persist_wr;
    wire        persist_done_toggle;  // from mailbox (25MHz toggle, synced in gate)

    block_ready_gate u_block_ready_gate (
        .clk(sig_7M), .rst_n(por_7m_n), .boot_done(boot_done),
        .arb_block_ready(arb_block_ready),
        .gated_block_ready(gated_block_ready),
        .dev_block_write_req(dev_block_write_req),
        .dev_block_num(dev_block_num),
        .persist_block(persist_block),
        .persist_wr(persist_wr),
        .persist_done(persist_done_toggle),
        .persist_enabled(mbox_bus_status_flags[2])  // enabled when s4d2_mounted
    );

    // =========================================================================
    // Mailbox (toggle CDC between bus_interface 7MHz and CPU 25MHz)
    // =========================================================================
    wire [7:0]  mbox_cpu_cmd, mbox_cpu_arg0;
    wire        mbox_cpu_cmd_pending;
    wire [15:0] mbox_cpu_persist_block;
    wire        mbox_cpu_persist_pending;

    // CPU status outputs (directly from cpu_soc)
    wire [7:0]  cpu_status_flags;
    wire [15:0] cpu_total_blocks;
    wire        cpu_status_wr;
    wire        cpu_cmd_ack;
    wire        cpu_persist_done;

    mailbox u_mailbox (
        // 7 MHz side
        .clk_bus(sig_7M), .rst_bus_n(por_7m_n),
        .bus_cmd(sd_cmd_data), .bus_arg0({4'd0, img_select}),
        .bus_cmd_wr(sd_cmd_wr),
        .bus_status_flags(mbox_bus_status_flags),
        .bus_total_blocks(mbox_bus_total_blocks),
        .bus_persist_block(persist_block),
        .bus_persist_wr(persist_wr),
        .bus_persist_done_toggle(persist_done_toggle),
        // 25 MHz side
        .clk_cpu(CLK_25MHz), .rst_cpu_n(por_25m_n),
        .cpu_cmd(mbox_cpu_cmd), .cpu_arg0(mbox_cpu_arg0),
        .cpu_cmd_pending(mbox_cpu_cmd_pending),
        .cpu_status_flags(cpu_status_flags),
        .cpu_total_blocks(cpu_total_blocks),
        .cpu_status_wr(cpu_status_wr),
        .cpu_persist_block(mbox_cpu_persist_block),
        .cpu_persist_pending(mbox_cpu_persist_pending),
        .cpu_cmd_ack(cpu_cmd_ack),
        .cpu_persist_done(cpu_persist_done)
    );

    // =========================================================================
    // SD Card — driven by PicoRV32 SPI master
    // =========================================================================
    wire cpu_sd_cs, cpu_sd_sck, cpu_sd_mosi;
    assign SD_CS   = cpu_sd_cs;
    assign SD_SCK  = cpu_sd_sck;
    assign SD_MOSI = cpu_sd_mosi;

    // =========================================================================
    // PicoRV32 CPU SoC
    // =========================================================================
    wire [7:0] cpu_gpio;
    wire       uart_txd;

    // CDC magic_block_req (7MHz) -> 25MHz for CPU
    reg mbr_s1 = 0, mbr_s2 = 0;
    always @(posedge CLK_25MHz) begin
        mbr_s1 <= magic_block_req;
        mbr_s2 <= mbr_s1;
    end

    // CDC dev_block_num (7MHz, stable during request) -> 25MHz
    reg [15:0] magic_block_num_s1, magic_block_num_s2;
    always @(posedge CLK_25MHz) begin
        magic_block_num_s1 <= dev_block_num;
        magic_block_num_s2 <= magic_block_num_s1;
    end

    cpu_soc u_cpu_soc (
        .clk(CLK_25MHz),
        .rst_n(por_25m_n),
        .gpio_out(cpu_gpio),
        .uart_txd(uart_txd),
        .spi_sck(cpu_sd_sck),
        .spi_mosi(cpu_sd_mosi),
        .spi_miso(SD_MISO),
        .spi_cs(cpu_sd_cs),
        // Mailbox
        .mbox_cmd(mbox_cpu_cmd),
        .mbox_arg0(mbox_cpu_arg0),
        .mbox_cmd_pending(mbox_cpu_cmd_pending),
        .mbox_status_flags(cpu_status_flags),
        .mbox_total_blocks(cpu_total_blocks),
        .mbox_status_wr(cpu_status_wr),
        .mbox_cmd_ack(cpu_cmd_ack),
        .mbox_persist_block(mbox_cpu_persist_block),
        .mbox_persist_pending(mbox_cpu_persist_pending),
        .mbox_persist_done(cpu_persist_done),
        // Block buffer
        .buf_claim(cpu_buf_claim),
        .buf_addr(cpu_buf_addr),
        .buf_wdata(cpu_buf_wdata),
        .buf_we(cpu_buf_we),
        .buf_rdata(buf_rdata_a),
        // Magic block request (from 7MHz, CDC'd)
        .magic_block_req(mbr_s2),
        .magic_block_num(magic_block_num_s2),
        .magic_block_ready(cpu_block_ready_25),
        // SDRAM port
        .sdram_claim(cpu_sdram_claim),
        .sdram_req(cpu_sdram_req),
        .sdram_write(cpu_sdram_write),
        .sdram_addr(cpu_sdram_addr),
        .sdram_wdata(cpu_sdram_wdata),
        .sdram_ready(sdram_req_ready),
        .sdram_rdata(sdram_req_rdata),
        .sdram_rdata_valid(sdram_req_rdata_valid)
    );

    // =========================================================================
    // GPIO mapping
    // =========================================================================
    assign GPIO5  = cpu_gpio[0];
    assign GPIO6  = mbox_bus_status_flags[0];  // DEBUG: sd_ready after mailbox CDC
    assign GPIO7  = uart_txd;
    assign GPIO8  = boot_done;
    assign GPIO10 = 1'b0;
    assign GPIO11 = 1'b0;

endmodule
