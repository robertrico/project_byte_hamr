`timescale 1ns / 1ps
// =============================================================================
// IWM Hamr - Top Level Module
// =============================================================================
// Self-contained SmartPort device: IWM + internal device engine + SDRAM + Flash
//
// Two clock domains:
//   sig_7M   (7.16 MHz) - Apple II bus, IWM, device engine
//   CLK_25MHz (25 MHz)  - SDRAM controller, SPI flash reader, boot loader
//
// Boot sequence:
//   1. POR releases (both domains)
//   2. SDRAM controller initializes (200us power-up + mode register load)
//   3. boot_loader copies disk image from SPI flash to SDRAM
//   4. sp_device begins responding to SmartPort commands (sense HIGH)
//
// Data path:
//   Flash --[boot_loader]--> SDRAM
//   SDRAM <--[sdram_arbiter]--> block_buffer <--[sp_device]--> IWM
// =============================================================================

module iwm_hamr_top (
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
    // Note: SCK driven via USRMCLK primitive, not a top-level port
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
    // Internal Signals - Apple II / IWM
    // =========================================================================
    wire [3:0] phase;
    wire       wrdata;
    wire       _enbl1, _enbl2;
    wire       _wrreq;
    wire [7:0] rom_data;
    wire       rom_oe;
    wire       rom_expansion_active;
    wire [7:0] iwm_data_out;
    wire       iwm_q7, iwm_q7_stable;
    wire [7:0] data_out_mux;

    // Internal device signals (from sp_device, replacing GPIO inputs)
    wire       sp_rddata;
    wire       sp_sense;

    // Forward declarations for signals used in GPIO before instantiation
    wire       boot_done;
    wire       flash_start;
    wire       sdram_init_done;
    wire       flash_busy;
    wire       flash_data_valid;

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
    // GPIO Debug Outputs
    // =========================================================================
    wire [4:0] sp_debug_state;
    wire [2:0] bl_debug_state;
    wire       rx_sync_debug;
    wire [2:0] sp_debug_cmd;
    wire       dut_iwm_x7;
    wire       dut_iwm_latch_synced;
    wire       sp_debug_wr_idle;

    // GPIO Debug — full 5-bit state for deadlock diagnosis
    //
    //   Ch 0  GPIO1  = sense      (HIGH=present, LOW=ACK)
    //   Ch 1  GPIO2  = rddata     (device TX data)
    //   Ch 2  GPIO3  = state[0]   ─┐
    //   Ch 3  GPIO4  = state[1]    │
    //   Ch 4  GPIO6  = state[2]    │ Full 5-bit state (0-22)
    //   Ch 5  GPIO7  = state[3]    │
    //   Ch 6  GPIO10 = state[4]   ─┘
    //   Ch 7  GPIO11 = wrdata     (host command data)
    //
    assign GPIO1  = sp_sense;
    assign GPIO2  = sp_rddata;
    assign GPIO3  = sp_debug_state[0];
    assign GPIO4  = sp_debug_state[1];
    assign GPIO6  = sp_debug_state[2];
    assign GPIO5  = 1'b0;                  // unused (GPIO5 not on LA)
    assign GPIO7  = sp_debug_state[3];
    assign GPIO10 = sp_debug_state[4];
    assign GPIO11 = wrdata;

    // Level Shifter OE
    wire lvl_shift_oe = !nDEVICE_SELECT || rom_oe;
    assign GPIO12 = ~lvl_shift_oe;

    // Tri-state control signals
    assign nIRQ = 1'bZ;
    assign nNMI = 1'bZ;
    assign nINH = 1'bZ;
    assign nDMA = 1'bZ;

    // Daisy chain pass-through
    assign DMA_IN = DMA_OUT;
    assign INT_IN = INT_OUT;

    // =========================================================================
    // Data Bus Control
    // =========================================================================
    assign data_out_mux = rom_oe ? rom_data : iwm_data_out;
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
    // Address Decoder
    // =========================================================================
    addr_decoder u_addr_decoder (
        .addr                (addr),
        .clk                 (sig_7M),
        .nI_O_STROBE         (nI_O_STROBE),
        .nI_O_SELECT         (nI_O_SELECT),
        .nRES                (por_n),
        .rom_oe              (rom_oe),
        .rom_expansion_active(rom_expansion_active)
    );

    // =========================================================================
    // IWM — rddata and sense now come from internal sp_device
    // =========================================================================
    iwm u_iwm (
        .addr           (addr[3:0]),
        .nDEVICE_SELECT (nDEVICE_SELECT),
        .fclk           (sig_7M),
        .Q3             (Q3),
        .nRES           (por_n),
        .data_in        ({D7, D6, D5, D4, D3, D2, D1, D0}),
        .data_out       (iwm_data_out),
        .wrdata         (wrdata),
        .phase          (phase),
        ._wrreq         (_wrreq),
        ._enbl1         (_enbl1),
        ._enbl2         (_enbl2),
        .sense          (sp_sense),
        .rddata         (sp_rddata),
        .q7_out         (iwm_q7),
        .q7_stable_out  (iwm_q7_stable),
        .debug_x7       (dut_iwm_x7),
        .debug_latch_synced (dut_iwm_latch_synced)
    );

    // =========================================================================
    // Boot ROM
    // =========================================================================
    wire [11:0] rom_addr = !nI_O_SELECT ? {4'b0100, addr[7:0]}
                                        : {1'b1, addr[10:0]};

    boot_rom u_boot_rom (
        .clk  (sig_7M),
        .addr (rom_addr),
        .data (rom_data)
    );

    // =========================================================================
    // SDRAM Address and Data Bus Mapping
    // =========================================================================
    // The controller uses bused ports [12:0] SDRAM_A and [15:0] SDRAM_DQ.
    // The top module has individual pins. Wire them through intermediate buses.
    // =========================================================================
    wire [12:0] sdram_a;
    // sdram_dq is now the top-level bidirectional DQ bus, assembled
    // from individual inout pins. Both the controller and external
    // SDRAM model can drive it via tristate.
    wire [15:0] sdram_dq = {SDRAM_D15, SDRAM_D14, SDRAM_D13, SDRAM_D12,
                             SDRAM_D11, SDRAM_D10, SDRAM_D9,  SDRAM_D8,
                             SDRAM_D7,  SDRAM_D6,  SDRAM_D5,  SDRAM_D4,
                             SDRAM_D3,  SDRAM_D2,  SDRAM_D1,  SDRAM_D0};

    // Address pin assignments (output only)
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

    // Data pin assignments — controller drives top-level inout pins.
    // Since sdram_dq is now defined as the pin readback above, and
    // the controller's tristate assign drives SDRAM_DQ (= sdram_dq),
    // both the controller output and external model input share the
    // same wires. No separate pin assigns needed — the controller's
    // internal `assign SDRAM_DQ = sdram_dq_oe ? ... : 16'hZZZZ`
    // drives sdram_dq directly, and when hi-Z, the model's drive
    // on the individual SDRAM_Dx pins takes effect.

    // =========================================================================
    // SDRAM Controller (25 MHz domain)
    // =========================================================================
    // sdram_init_done declared above (forward ref)
    wire        arb_sdram_req;
    wire        arb_sdram_write;
    wire [25:0] arb_sdram_addr;
    wire [15:0] arb_sdram_wdata;
    wire        sdram_req_ready;
    wire [15:0] sdram_req_rdata;
    wire        sdram_req_rdata_valid;

    sdram_controller u_sdram_controller (
        .clk            (CLK_25MHz),
        .rst_n          (por_25_n),
        .init_done      (sdram_init_done),
        .pause_refresh  (1'b0),           // Never pause refresh — boot takes 91ms, exceeds 64ms tREF
        .req            (arb_sdram_req),
        .req_write      (arb_sdram_write),
        .req_addr       (arb_sdram_addr),
        .req_wdata      (arb_sdram_wdata),
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
    // SPI Flash Reader (25 MHz domain)
    // =========================================================================
    // flash_busy declared above (forward ref)
    wire        flash_done;
    wire [7:0]  flash_data_out;
    // flash_data_valid declared above (forward ref)
    wire        flash_data_ready;
    // flash_start declared above (forward ref)
    wire [23:0] flash_start_addr;
    wire [23:0] flash_byte_count;
    wire        flash_sck_pin;

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
        .flash_ncs      (FLASH_nCS),
        .flash_mosi     (FLASH_MOSI),
        .flash_miso     (FLASH_MISO),
        .flash_nwp      (FLASH_nWP),
        .flash_nhold    (FLASH_nHOLD),
        .flash_sck_pin  (flash_sck_pin)
    );

    // =========================================================================
    // Boot Loader (25 MHz domain)
    // =========================================================================
    // boot_done declared above (forward declaration for GPIO)
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
        // Flash reader interface
        .flash_start     (flash_start),
        .flash_addr      (flash_start_addr),
        .flash_count     (flash_byte_count),
        .flash_busy      (flash_busy),
        .flash_data      (flash_data_out),
        .flash_data_valid(flash_data_valid),
        .flash_data_ready(flash_data_ready),
        // SDRAM write interface (routed through arbiter)
        .sdram_req       (boot_sdram_req),
        .sdram_req_write (boot_sdram_write),
        .sdram_req_addr  (boot_sdram_addr),
        .sdram_req_wdata (boot_sdram_wdata),
        .sdram_req_ready (boot_sdram_ready),
        .debug_state     (bl_debug_state)
    );

    // =========================================================================
    // Block Buffer — Dual-port BRAM bridging 25 MHz and 7 MHz domains
    // =========================================================================
    // Port A: 25 MHz (SDRAM arbiter side)
    wire [8:0]  buf_addr_a;
    wire [7:0]  buf_wdata_a;
    wire        buf_we_a;
    wire [7:0]  buf_rdata_a;

    // Port B: 7 MHz (sp_device side)
    wire [8:0]  buf_addr_b;
    wire [7:0]  buf_rdata_b;
    wire [7:0]  buf_wdata_b;
    wire        buf_we_b;

    block_buffer u_block_buffer (
        // Port A (25 MHz - arbiter)
        .clk_a   (CLK_25MHz),
        .addr_a  (buf_addr_a),
        .wdata_a (buf_wdata_a),
        .we_a    (buf_we_a),
        .rdata_a (buf_rdata_a),
        // Port B (7 MHz - device engine)
        .clk_b   (sig_7M),
        .addr_b  (buf_addr_b),
        .wdata_b (buf_wdata_b),
        .we_b    (buf_we_b),
        .rdata_b (buf_rdata_b)
    );

    // =========================================================================
    // SDRAM Arbiter (25 MHz domain)
    // =========================================================================
    // Multiplexes boot_loader (during boot) and sp_device block requests
    // (at runtime). Handles block-level SDRAM <-> buffer transfers.
    // =========================================================================
    wire        dev_block_read_req;
    wire        dev_block_write_req;
    wire [15:0] dev_block_num;
    wire        dev_block_ready;

    sdram_arbiter u_sdram_arbiter (
        .clk                (CLK_25MHz),
        .rst_n              (por_25_n),
        .boot_done          (boot_done),
        // Boot loader port
        .boot_req           (boot_sdram_req),
        .boot_write         (boot_sdram_write),
        .boot_addr          (boot_sdram_addr),
        .boot_wdata         (boot_sdram_wdata),
        .boot_ready         (boot_sdram_ready),
        // Device engine block requests (CDC from 7 MHz)
        .dev_block_read_req (dev_block_read_req),
        .dev_block_write_req(dev_block_write_req),
        .dev_block_num      (dev_block_num),
        .dev_block_ready    (dev_block_ready),
        // Block buffer Port A (25 MHz side)
        .buf_addr_a         (buf_addr_a),
        .buf_wdata_a        (buf_wdata_a),
        .buf_we_a           (buf_we_a),
        .buf_rdata_a        (buf_rdata_a),
        // SDRAM controller interface
        .sdram_req          (arb_sdram_req),
        .sdram_write        (arb_sdram_write),
        .sdram_addr         (arb_sdram_addr),
        .sdram_wdata        (arb_sdram_wdata),
        .sdram_ready        (sdram_req_ready),
        .sdram_rdata        (sdram_req_rdata),
        .sdram_rdata_valid  (sdram_req_rdata_valid)
    );

    // =========================================================================
    // SmartPort Device Engine (7 MHz domain)
    // =========================================================================
    // Replaces the external PicoPort. Receives IWM signals, generates rddata
    // and sense internally. Block buffer Port B provides payload data.
    // =========================================================================
    sp_device u_sp_device (
        .fclk            (sig_7M),
        .rst_n           (por_n),
        .boot_done       (boot_done),
        // IWM drive interface (internal)
        .wrdata          (wrdata),
        .rddata          (sp_rddata),
        .phase           (phase),
        ._wrreq          (_wrreq),
        ._enbl1          (_enbl1),
        ._enbl2          (_enbl2),
        .sense           (sp_sense),
        // Block buffer Port B (7 MHz side)
        .buf_addr        (buf_addr_b),
        .buf_rd_data     (buf_rdata_b),
        .buf_wr_data     (buf_wdata_b),
        .buf_wr_en       (buf_we_b),
        // SDRAM block requests (cross to 25 MHz via arbiter CDC)
        .block_read_req  (dev_block_read_req),
        .block_write_req (dev_block_write_req),
        .block_num       (dev_block_num),
        .block_ready     (dev_block_ready),
        .debug_state     (sp_debug_state),
        .debug_sync      (rx_sync_debug),
        .debug_cmd_code  (sp_debug_cmd),
        .debug_wr_idle   (sp_debug_wr_idle)
    );

endmodule
