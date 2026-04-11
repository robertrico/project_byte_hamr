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
    wire [15:0] dev_block_num;       // raw block from ROM (relative within unit)
    wire [15:0] dev_block_num_abs;   // with unit offset applied (absolute SDRAM addr)
    wire        dev_block_ready;

    // ---- Block read intercept ----
    // Magic blocks ($FFFF/$FFFE): always intercepted for catalog
    // On-demand cache: ALL reads intercepted when cache_enabled (after mount)
    // Magic block ($FFFF/$FFFE) must be checked on RAW block_num (before offset)
    // because $FFFF is a sentinel, not a real SDRAM address.
    wire is_magic_block = (dev_block_num[15:1] == 15'h7FFF);

    // CDC cache_enabled (25MHz) -> 7MHz
    wire cache_enabled_25;  // from cpu_soc
    reg  cache_en_s1, cache_en_s2;
    always @(posedge sig_7M or negedge por_7m_n) begin
        if (!por_7m_n) begin cache_en_s1 <= 0; cache_en_s2 <= 0; end
        else begin cache_en_s1 <= cache_enabled_25; cache_en_s2 <= cache_en_s1; end
    end

    wire is_intercepted_base = is_magic_block | cache_en_s2;

    // CPU signals block ready for intercepted reads (25MHz -> 7MHz CDC)
    wire cpu_block_ready_25;  // from cpu_soc (MAGIC_DONE stretch)
    reg  cpu_br_s1, cpu_br_s2;
    always @(posedge sig_7M or negedge por_7m_n) begin
        if (!por_7m_n) begin cpu_br_s1 <= 0; cpu_br_s2 <= 0; end
        else begin cpu_br_s1 <= cpu_block_ready_25; cpu_br_s2 <= cpu_br_s1; end
    end

    // ---- Cache release: firmware says "data is in SDRAM, let arbiter serve" ----
    // CDC cache_release pulse (25MHz) -> 7MHz toggle
    wire cache_release_25;  // from cpu_soc
    reg  crel_s1, crel_s2, crel_prev;
    always @(posedge sig_7M or negedge por_7m_n) begin
        if (!por_7m_n) begin crel_s1 <= 0; crel_s2 <= 0; crel_prev <= 0; end
        else begin crel_s1 <= cache_release_25; crel_s2 <= crel_s1; crel_prev <= crel_s2; end
    end
    wire crel_edge = crel_s2 & ~crel_prev;

    // When released: de-intercept current request so arbiter serves it.
    // Clears when the request completes (block_read_req goes LOW).
    reg cache_released;
    always @(posedge sig_7M or negedge por_7m_n) begin
        if (!por_7m_n)
            cache_released <= 1'b0;
        else begin
            if (crel_edge)
                cache_released <= 1'b1;
            if (cache_released & ~dev_block_read_req_raw)
                cache_released <= 1'b0;
        end
    end

    // Effective intercept: suppressed during cache release (arbiter handles it)
    wire is_intercepted = is_intercepted_base & ~cache_released;
    wire dev_block_read_req = dev_block_read_req_raw & ~is_intercepted;
    wire magic_block_req    = dev_block_read_req_raw & is_intercepted;

    // ---- Toggle-based CDC for intercepted read requests (7MHz → 25MHz) ----
    reg magic_req_toggle, magic_req_prev;
    reg [7:0] dbg_magic_rise_count;
    always @(posedge sig_7M or negedge por_7m_n) begin
        if (!por_7m_n) begin
            magic_req_toggle     <= 1'b0;
            magic_req_prev       <= 1'b0;
            dbg_magic_rise_count <= 8'd0;
        end else begin
            magic_req_prev <= magic_block_req;
            if (magic_block_req & ~magic_req_prev) begin
                magic_req_toggle     <= ~magic_req_toggle;
                dbg_magic_rise_count <= dbg_magic_rise_count + 8'd1;
            end
        end
    end

    // Magic/cache active flag — forces dev_block_ready LOW during intercepted
    // reads so bus_interface gets a proper LOW→HIGH edge when CPU finishes.
    // Also clears on cache_release (arbiter takes over via gated_block_ready).
    reg magic_active;
    always @(posedge sig_7M or negedge por_7m_n) begin
        if (!por_7m_n)
            magic_active <= 1'b0;
        else begin
            if (magic_block_req & ~magic_req_prev)
                magic_active <= 1'b1;
            if (magic_active & ~dev_block_read_req_raw)
                magic_active <= 1'b0;
            // NOTE: do NOT clear magic_active on crel_edge!
            // If we clear it, dev_block_ready jumps to gated_block_ready
            // (already HIGH from arbiter idle) → premature bus S_IDLE.
            // Instead, firmware fires MAGIC_DONE after the arbiter finishes.
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
    wire [1:0]  mount_slot;

    wire [7:0] bus_dbg_wr_count, bus_dbg_rd_count;

    // Per-unit data from cpu_soc (25MHz) — CDC to 7MHz for bus_interface
    wire [15:0] cpu_unit_blkcnt_0, cpu_unit_blkcnt_1, cpu_unit_blkcnt_2, cpu_unit_blkcnt_3;
    wire [15:0] cpu_unit_offset_1, cpu_unit_offset_2, cpu_unit_offset_3;
    reg  [15:0] ub0_s1, ub0_s2, ub1_s1, ub1_s2, ub2_s1, ub2_s2, ub3_s1, ub3_s2;
    reg  [15:0] uo1_s1, uo1_s2, uo2_s1, uo2_s2, uo3_s1, uo3_s2;
    always @(posedge sig_7M or negedge por_7m_n) begin
        if (!por_7m_n) begin
            {ub0_s1, ub0_s2} <= 32'd0; {ub1_s1, ub1_s2} <= 32'd0;
            {ub2_s1, ub2_s2} <= 32'd0; {ub3_s1, ub3_s2} <= 32'd0;
            {uo1_s1, uo1_s2} <= 32'd0; {uo2_s1, uo2_s2} <= 32'd0;
            {uo3_s1, uo3_s2} <= 32'd0;
        end else begin
            ub0_s1 <= cpu_unit_blkcnt_0; ub0_s2 <= ub0_s1;
            ub1_s1 <= cpu_unit_blkcnt_1; ub1_s2 <= ub1_s1;
            ub2_s1 <= cpu_unit_blkcnt_2; ub2_s2 <= ub2_s1;
            ub3_s1 <= cpu_unit_blkcnt_3; ub3_s2 <= ub3_s1;
            uo1_s1 <= cpu_unit_offset_1; uo1_s2 <= uo1_s1;
            uo2_s1 <= cpu_unit_offset_2; uo2_s2 <= uo2_s1;
            uo3_s1 <= cpu_unit_offset_3; uo3_s2 <= uo3_s1;
        end
    end

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
        .block_num_out(dev_block_num_abs),
        .dbg_wr_count_out(bus_dbg_wr_count),
        .dbg_rd_count_out(bus_dbg_rd_count),
        // SD management — status from mailbox
        .sd_ready(mbox_bus_status_flags[0]),
        .sd_error_in(mbox_bus_status_flags[1]),
        .s4d2_mounted(mbox_bus_status_flags[2]),
        .s4d2_loading(mbox_bus_status_flags[3]),
        .img_count(mbox_bus_status_flags[7:4]),
        // Catalog
        .cat_rd_addr(), .cat_rd_data(8'd0),
        .img_select(img_select), .img_name_idx(),
        // SD commands -> mailbox
        .mount_request(),
        .sd_init_request(),
        .sd_cmd_data(sd_cmd_data),
        .sd_cmd_wr(sd_cmd_wr),
        .mount_slot(mount_slot),
        // Per-unit data (CDC'd above)
        .unit_blkcnt_0(ub0_s2), .unit_blkcnt_1(ub1_s2),
        .unit_blkcnt_2(ub2_s2), .unit_blkcnt_3(ub3_s2),
        .unit_offset_1(uo1_s2), .unit_offset_2(uo2_s2),
        .unit_offset_3(uo3_s2)
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

    // Both ports on 25MHz — eliminates dual-clock cross-port visibility issue.
    // Port B signals (buf_addr_b, buf_we_b) change at 7MHz rate but are
    // sampled at 25MHz. Multiple samples per 7MHz cycle are idempotent.
    block_buffer u_block_buffer (
        .clk_a(CLK_25MHz), .addr_a(mux_buf_addr_a), .wdata_a(mux_buf_wdata_a),
        .we_a(mux_buf_we_a), .rdata_a(buf_rdata_a),
        .clk_b(CLK_25MHz), .addr_b(buf_addr_b), .wdata_b(buf_wdata_b),
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
        .dev_block_num(dev_block_num_abs), .dev_block_ready(arb_block_ready),
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

    wire [7:0] dbg_trigger_count;

    block_ready_gate u_block_ready_gate (
        .clk(sig_7M), .rst_n(por_7m_n), .boot_done(boot_done),
        .arb_block_ready(arb_block_ready),
        .gated_block_ready(gated_block_ready),
        .dev_block_write_req(dev_block_write_req),
        .dev_block_num(dev_block_num_abs),
        .persist_block(persist_block),
        .persist_wr(persist_wr),
        .persist_done(persist_done_toggle),
        .persist_enabled(mbox_bus_status_flags[2]),  // enabled when s4d2_mounted
        .dbg_trigger_count(dbg_trigger_count)
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
        .bus_cmd(sd_cmd_data), .bus_arg0({mount_slot, 2'b0, img_select}),
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

    // CDC magic_req_toggle (7MHz) -> 25MHz for CPU (toggle-based, reliable)
    reg mrt_s1 = 0, mrt_s2 = 0;
    always @(posedge CLK_25MHz) begin
        mrt_s1 <= magic_req_toggle;
        mrt_s2 <= mrt_s1;
    end

    // CDC block_num (7MHz, stable during request) -> 25MHz
    // Use raw block_num for magic blocks ($FFFF sentinel), absolute for normal reads
    wire [15:0] magic_block_capture = is_magic_block ? dev_block_num : dev_block_num_abs;
    reg [15:0] magic_block_num_s1, magic_block_num_s2;
    always @(posedge CLK_25MHz) begin
        magic_block_num_s1 <= magic_block_capture;
        magic_block_num_s2 <= magic_block_num_s1;
    end

    // CDC debug signals (7MHz) -> 25MHz for firmware to read
    reg [7:0] dbg_wr_s1, dbg_wr_s2;
    reg [7:0] dbg_trig_s1, dbg_trig_s2;
    reg [7:0] dbg_mrise_s1, dbg_mrise_s2;
    reg [7:0] dbg_rdcnt_s1, dbg_rdcnt_s2;  // CMD_READ count from bus_interface
    reg       dbg_holding_s1, dbg_holding_s2;
    reg       dbg_gated_s1, dbg_gated_s2;
    reg       dbg_arb_s1, dbg_arb_s2;
    reg       dbg_wr_req_s1, dbg_wr_req_s2;
    reg       dbg_rd_req_s1, dbg_rd_req_s2;   // block_read_req_raw (live)
    reg       dbg_mactive_s1, dbg_mactive_s2; // magic_active (live)
    reg       dbg_intercepted_s1, dbg_intercepted_s2; // is_intercepted (live)
    always @(posedge CLK_25MHz) begin
        dbg_wr_s1      <= bus_dbg_wr_count;  dbg_wr_s2      <= dbg_wr_s1;
        dbg_trig_s1    <= dbg_trigger_count;  dbg_trig_s2    <= dbg_trig_s1;
        dbg_mrise_s1   <= dbg_magic_rise_count; dbg_mrise_s2 <= dbg_mrise_s1;
        dbg_rdcnt_s1   <= bus_dbg_rd_count;  dbg_rdcnt_s2   <= dbg_rdcnt_s1;
        dbg_holding_s1 <= u_block_ready_gate.holding;
        dbg_holding_s2 <= dbg_holding_s1;
        dbg_gated_s1   <= gated_block_ready;  dbg_gated_s2   <= dbg_gated_s1;
        dbg_arb_s1     <= arb_block_ready;    dbg_arb_s2     <= dbg_arb_s1;
        dbg_wr_req_s1  <= dev_block_write_req; dbg_wr_req_s2 <= dbg_wr_req_s1;
        dbg_rd_req_s1  <= dev_block_read_req_raw; dbg_rd_req_s2 <= dbg_rd_req_s1;
        dbg_mactive_s1 <= magic_active;       dbg_mactive_s2 <= dbg_mactive_s1;
        dbg_intercepted_s1 <= is_intercepted; dbg_intercepted_s2 <= dbg_intercepted_s1;
    end

    // Debug register layout:
    // [31:24] magic_rise_count  — how many times magic_block_req rose (7MHz)
    // [23:16] CMD_READ count    — how many CMD_READs bus_interface processed
    // [15:8]  reserved
    // [7]     cache_en_s2       — is cache intercept enabled?
    // [6]     is_intercepted    — is the current read being intercepted?
    // [5]     magic_active      — is an intercepted read in progress?
    // [4]     block_read_req_raw — is bus_interface requesting a read?
    // [3]     gated_block_ready
    // [2]     holding
    // [1]     persist_enabled
    // [0]     boot_done
    wire [31:0] dbg_bus_state = {
        dbg_mrise_s2,                  // [31:24] magic_block_req rising edges
        dbg_rdcnt_s2,                  // [23:16] CMD_READ count from bus_interface
        8'd0,                          // [15:8]  reserved
        cache_en_s2,                   // [7]     cache_enabled (7MHz CDC'd)
        dbg_intercepted_s2,            // [6]     is_intercepted
        dbg_mactive_s2,                // [5]     magic_active
        dbg_rd_req_s2,                 // [4]     block_read_req_raw
        dbg_gated_s2,                  // [3]     gated_block_ready
        dbg_holding_s2,                // [2]     holding
        mbox_bus_status_flags[2],      // [1]     persist_enabled
        boot_done                      // [0]     boot_done
    };

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
        // Per-unit registers (multi-drive)
        .boot_unit(),  // not used yet — boot_unit is 7MHz-only for now
        .unit_blkcnt_0(cpu_unit_blkcnt_0), .unit_blkcnt_1(cpu_unit_blkcnt_1),
        .unit_blkcnt_2(cpu_unit_blkcnt_2), .unit_blkcnt_3(cpu_unit_blkcnt_3),
        .unit_offset_1(cpu_unit_offset_1), .unit_offset_2(cpu_unit_offset_2),
        .unit_offset_3(cpu_unit_offset_3),
        // Block buffer + cache control
        .buf_claim(cpu_buf_claim),
        .cache_enabled(cache_enabled_25),
        .cache_release(cache_release_25),
        .buf_addr(cpu_buf_addr),
        .buf_wdata(cpu_buf_wdata),
        .buf_we(cpu_buf_we),
        .buf_rdata(buf_rdata_a),
        // Magic block request (from 7MHz, CDC'd)
        // Debug
        .dbg_bus_state(dbg_bus_state),
        // Magic block request (from 7MHz, CDC'd)
        .magic_block_req(mrt_s2),  // toggle-based CDC: edge = new request
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
