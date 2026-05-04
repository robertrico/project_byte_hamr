`timescale 1ns / 1ps
// =============================================================================
// smart_hamr_rev2_top.v — ASIC-faithful Liron card on Byte Hamr Rev 2
// =============================================================================
// Apple II Slot 4 SmartPort controller. 4 KB Liron ROM mapped at
// $C400-$C4FF + $C800-$CFFF. IWM ASIC emulation (iwm_asic.v) drives the
// drive interface (rddata/wrdata/phase/_enbl1/_enbl2/_wrreq) out the
// daughter-board GPIO header to the FujiNet ESP32.
//
// Single clock domain: sig_7M = 7.16 MHz Apple II 7M. No PLL, no SDRAM,
// no flash, no soft core.
//
// Reset network:
//   - POR: 4-bit fclk counter holds reset ~2 µs after FPGA configuration.
//   - nRES_READ (A5): Apple II reset monitor, 2-FF synchronizer in fclk.
//   - Combined reset: por_n & nRES_READ_sync.
//
// Memory map (slot 4):
//   $C0C0-$C0CF  nDEVICE_SELECT   IWM register space
//   $C400-$C4FF  nI_O_SELECT      ROM ($400-$4FF; addr_decoder direct map)
//   $C800-$CFFF  nI_O_STROBE      ROM ($800-$FFF) when expansion active
// =============================================================================

module smart_hamr_rev2_top (
    // ---- Apple II address bus ----
    input wire        A0,  A1,  A2,  A3,  A4,  A5,  A6,  A7,
    input wire        A8,  A9,  A10, A11, A12, A13, A14, A15,

    // ---- Apple II data bus ----
    inout wire        D0, D1, D2, D3, D4, D5, D6, D7,

    // ---- Apple II control ----
    input  wire       sig_7M,
    input  wire       Q3,
    input  wire       R_nW,
    input  wire       nDEVICE_SELECT,
    input  wire       nI_O_SELECT,
    input  wire       nI_O_STROBE,
    output wire       nRES,
    input  wire       nRES_READ,
    input  wire       RDY,
    output wire       nIRQ, nNMI, nDMA, nINH,
    input  wire       DMA_OUT,
    output wire       DMA_IN,
    input  wire       INT_OUT,
    output wire       INT_IN,
    input  wire       PHI0, PHI1,
    input  wire       uSync,

    // ---- Rev 2 level shifter OE ----
    output wire       DATA_OE,

    // ---- 100 MHz osc (Rev 2 LPF match, unused) ----
    input  wire       CLK_100MHz,

    // ---- GPIO header (Rev 2 daughter-board, FujiNet bridge) ----
    output wire       GPIO_1,    // /DEV LA debug
    output wire       GPIO_2,
    output wire       GPIO_3,
    output wire       GPIO_4,
    output wire       GPIO_5,
    output wire       GPIO_6,
    output wire       GPIO_7,
    output wire       GPIO_8,
    output wire       GPIO_9,    // _enbl2
    output wire       GPIO_10,   // wrdata
    output wire       GPIO_11,
    input  wire       GPIO_12,   // rddata (idle-LOW from ESP32)
    input  wire       GPIO_13,   // sense (idle-HIGH)
    output wire       GPIO_14,   // _wrreq
    output wire       GPIO_15,   // phase[1]
    output wire       GPIO_16,   // phase[0]
    output wire       GPIO_17,   // phase[3]
    output wire       GPIO_18,   // phase[2]
    output wire       GPIO_19,
    output wire       GPIO_20    // _enbl1
);

    // =========================================================================
    // Address bus bundle
    // =========================================================================
    wire [11:0] addr = {A11, A10, A9, A8, A7, A6, A5, A4, A3, A2, A1, A0};

    // Suppress unused warnings for inputs not consumed by this design.
    wire _unused = &{1'b0, A12, A13, A14, A15, CLK_100MHz, RDY, PHI0, PHI1, uSync};

    // =========================================================================
    // Power-on reset (sig_7M domain) — ~2 µs hold
    // =========================================================================
    assign nRES = 1'b1;  // open-drain, always released

    reg [3:0] por_counter = 4'd0;
    wire      por_n = &por_counter;

    always @(posedge sig_7M) begin
        if (!por_n)
            por_counter <= por_counter + 4'd1;
    end

    // =========================================================================
    // Apple II reset (nRES_READ) — 2-FF synchronizer, sig_7M domain
    // =========================================================================
    reg [1:0] nres_sync = 2'b00;
    always @(posedge sig_7M)
        nres_sync <= {nres_sync[0], nRES_READ};
    wire nRES_READ_sync = nres_sync[1];

    wire rst_n = por_n & nRES_READ_sync;

    // =========================================================================
    // Pass-through control signals
    // =========================================================================
    assign nIRQ   = 1'bZ;
    assign nNMI   = 1'bZ;
    assign nINH   = 1'bZ;
    assign nDMA   = 1'bZ;
    assign DMA_IN = DMA_OUT;
    assign INT_IN = INT_OUT;

    // =========================================================================
    // Internal signals
    // =========================================================================
    wire [3:0] phase;
    wire       wrdata;
    wire       rddata;
    wire       sense;
    wire       _enbl1;
    wire       _enbl2;
    wire       _wrreq;

    wire [7:0] rom_data;
    wire       rom_oe;
    wire       rom_expansion_active;
    wire [7:0] iwm_data_out;

    // =========================================================================
    // GPIO output mapping (FujiNet bench mapping, do not change)
    // =========================================================================
    // Phase outputs registered to avoid combinatorial glitches on ESP32 pins.
    reg [3:0] phase_gpio = 4'b0000;
    always @(posedge sig_7M or negedge rst_n) begin
        if (!rst_n)
            phase_gpio <= 4'b0000;
        else
            phase_gpio <= phase;
    end

    assign GPIO_1  = nDEVICE_SELECT;       // LA debug
    assign GPIO_9  = _enbl2;
    assign GPIO_10 = wrdata;
    assign GPIO_14 = _wrreq;
    assign GPIO_15 = phase_gpio[1];
    assign GPIO_16 = phase_gpio[0];
    assign GPIO_17 = phase_gpio[3];
    assign GPIO_18 = phase_gpio[2];
    assign GPIO_20 = _enbl1;

    assign GPIO_2  = 1'b0;
    assign GPIO_3  = nRES_READ;            // LA debug — Apple II reset visibility
    assign GPIO_4  = 1'b0;
    assign GPIO_5  = 1'b0;
    assign GPIO_6  = 1'b0;
    assign GPIO_7  = 1'b0;
    assign GPIO_8  = 1'b0;
    assign GPIO_11 = 1'b0;
    assign GPIO_19 = 1'b0;

    // =========================================================================
    // GPIO inputs from ESP32
    // =========================================================================
    // rddata: invert GPIO_12 — ESP32 sends idle-LOW/pulse-HIGH; the IWM
    // expects idle-HIGH/pulse-LOW (drive-style). Sense passes through.
    assign rddata = ~GPIO_12;
    assign sense  = GPIO_13;

    // =========================================================================
    // Level shifter OE (active-low) — Rev 2 dedicated pin B20
    // =========================================================================
    wire lvl_shift_oe = !nDEVICE_SELECT || rom_oe;
    assign DATA_OE = ~lvl_shift_oe;

    // =========================================================================
    // Data bus mux + tri-state (ROM has priority)
    // =========================================================================
    wire [7:0] data_out_mux = rom_oe ? rom_data : iwm_data_out;
    wire       data_oe      = R_nW && (rom_oe || !nDEVICE_SELECT);

    assign D0 = data_oe ? data_out_mux[0] : 1'bZ;
    assign D1 = data_oe ? data_out_mux[1] : 1'bZ;
    assign D2 = data_oe ? data_out_mux[2] : 1'bZ;
    assign D3 = data_oe ? data_out_mux[3] : 1'bZ;
    assign D4 = data_oe ? data_out_mux[4] : 1'bZ;
    assign D5 = data_oe ? data_out_mux[5] : 1'bZ;
    assign D6 = data_oe ? data_out_mux[6] : 1'bZ;
    assign D7 = data_oe ? data_out_mux[7] : 1'bZ;

    // =========================================================================
    // Address decoder — manages rom_oe + expansion ROM flag
    // =========================================================================
    addr_decoder u_addr_decoder (
        .addr                (addr),
        .clk                 (sig_7M),
        .nI_O_STROBE         (nI_O_STROBE),
        .nI_O_SELECT         (nI_O_SELECT),
        .nRES                (rst_n),
        .rom_oe              (rom_oe),
        .rom_expansion_active(rom_expansion_active)
    );

    // =========================================================================
    // IWM ASIC
    // =========================================================================
    iwm_asic u_iwm_asic (
        .addr           (addr[3:0]),
        .nDEVICE_SELECT (nDEVICE_SELECT),
        .fclk           (sig_7M),
        .Q3             (Q3),
        .R_nW           (R_nW),
        .nRES           (rst_n),
        .data_in        ({D7, D6, D5, D4, D3, D2, D1, D0}),
        .data_out       (iwm_data_out),
        .wrdata         (wrdata),
        .phase          (phase),
        ._wrreq         (_wrreq),
        ._enbl1         (_enbl1),
        ._enbl2         (_enbl2),
        .sense          (sense),
        .rddata         (rddata)
    );

    // =========================================================================
    // Boot ROM — 4 KB Liron firmware ($readmemh "liron_rom.mem")
    // =========================================================================
    // Lower 12 bits of slot bus address index ROM directly:
    //   $C400-$C4FF (nI_O_SELECT) → ROM[$400-$4FF]
    //   $C800-$CFFF (nI_O_STROBE) → ROM[$800-$FFF]
    wire [11:0] rom_addr = !nI_O_SELECT ? {4'b0100, addr[7:0]}
                                        : {1'b1,    addr[10:0]};

    boot_rom u_boot_rom (
        .clk  (sig_7M),
        .addr (rom_addr),
        .data (rom_data)
    );

endmodule
