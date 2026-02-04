`timescale 1ns / 1ps
// =============================================================================
// Yellow Hamr - Top Level Module
// =============================================================================
// Apple IIe-compatible Liron disk controller (IWM-based SmartPort interface)
// for the Byte Hamr FPGA card.
//
// Adapted from Steve Chamberlin's Yellowstone project:
//   Original target: Lattice MachXO2-1200HC
//   This target: Lattice ECP5-85F (Byte Hamr)
//
// This design emulates the Apple Liron card, providing SmartPort connectivity
// via the GPIO header for connection to external SmartPort-compatible devices
// (e.g., FujiNet, SD card adapters, etc.)
//
// Memory Map (Slot 4):
//   $C0C0-$C0CF  nDEVICE_SELECT  IWM registers
//   $C400-$C4FF  nI_O_SELECT     Boot ROM ($000-$0FF)
//   $C800-$CFFF  nI_O_STROBE     Boot ROM ($100-$7FF) when expansion active
//
// GPIO Header Pinout (Drive Interface):
//   GPIO1:  phase[0]   Output  Stepper phase 0 / SmartPort command
//   GPIO2:  phase[1]   Output  Stepper phase 1 / SmartPort command
//   GPIO3:  phase[2]   Output  Stepper phase 2 / SmartPort command
//   GPIO4:  phase[3]   Output  Stepper phase 3 / SmartPort command
//   GPIO5:  wrdata     Output  Serial write data
//   GPIO6:  rddata     Input   Serial read data
//   GPIO7:  sense      Input   Write-protect / ACK
//   GPIO8:  _enbl1     Output  Drive 1 enable (active low)
//   GPIO9:  _enbl2     Output  Drive 2 enable (active low)
//   GPIO10: _wrreq     Output  Write request (active low)
//   GPIO11: _en35      Output  3.5" drive enable (active low)
//   GPIO12: (spare)    --      Reserved for future use
// =============================================================================

module yellow_hamr_top (
    // System clock (unused by IWM - uses 7M directly)
    input wire        CLK_25MHz,

    // Apple II bus interface
    input wire [11:0] addr,             // Address bus (directly, as individual pins)
    inout wire [7:0]  data,             // Data bus (directly, as individual pins)
    input wire        sig_7M,           // 7 MHz clock (IWM FCLK)
    input wire        Q3,               // 2 MHz timing reference
    input wire        R_nW,             // Read/Write (1=read, 0=write)
    input wire        nDEVICE_SELECT,   // $C0C0-$C0CF
    input wire        nI_O_SELECT,      // $C400-$C4FF
    input wire        nI_O_STROBE,      // $C800-$CFFF
    output wire       nRES,             // System reset (active low, open-drain)

    // Critical control signals (directly hi-Z to avoid bus contention)
    input  wire        RDY,             // Ready (input only)
    output wire        nIRQ,            // Interrupt request (tri-stated)
    output wire        nNMI,            // Non-maskable interrupt (tri-stated)
    output wire        nDMA,            // DMA (tri-stated)
    output wire        nINH,            // Inhibit (tri-stated)

    // Daisy chain signals - match Logic Hamr direction exactly!
    input  wire        DMA_OUT,         // DMA from higher slots (input)
    output wire        DMA_IN,          // DMA to lower slots (output)
    input  wire        INT_OUT,         // INT from higher slots (input)
    output wire        INT_IN,          // INT to lower slots (output)

    // Clock phases
    input  wire        PHI0,            // Phase 0 clock
    input  wire        PHI1,            // Phase 1 clock
    input  wire        uSync,           // Microsecond sync

    // Drive interface (directly via GPIO header)
    output wire       GPIO1,            // phase[0]
    output wire       GPIO2,            // phase[1]
    output wire       GPIO3,            // phase[2]
    output wire       GPIO4,            // phase[3]
    output wire       GPIO5,            // wrdata
    input wire        GPIO6,            // rddata
    input wire        GPIO7,            // sense
    output wire       GPIO8,            // _enbl1
    output wire       GPIO9,            // _enbl2
    output wire       GPIO10,           // _wrreq
    output wire       GPIO11,           // _en35
    output wire       GPIO12            // spare (unused)
);

    // =========================================================================
    // Internal Signals
    // =========================================================================

    // Heartbeat counter (25MHz / 2^24 ≈ 1.5Hz)
    reg [23:0] heartbeat_cnt;

    // Drive interface signals
    wire [3:0] phase;
    wire       wrdata;
    wire       rddata;
    wire       sense;
    wire       _enbl1;
    wire       _enbl2;
    wire       _wrreq;

    // ROM signals
    wire [7:0] rom_data;
    wire       rom_oe;
    wire       rom_expansion_active;

    // IWM signals
    wire [7:0] iwm_data_out;

    // Data bus control
    wire       data_out_enable;
    wire [7:0] data_out_mux;

    // =========================================================================
    // GPIO Assignments - Drive interface directly from IWM
    // =========================================================================

    assign GPIO1  = phase[0];
    assign GPIO2  = phase[1];
    assign GPIO3  = phase[2];
    assign GPIO4  = phase[3];
    assign GPIO5  = wrdata;
    assign GPIO8  = _enbl1;
    assign GPIO9  = _enbl2;
    assign GPIO10 = _wrreq;
    assign GPIO11 = 1'b1;              // _en35 - always inactive (no 3.5" support yet)
    assign GPIO12 = heartbeat_cnt[9];  // Heartbeat (~24kHz)

    // Input mapping from GPIO
    assign rddata = GPIO6;
    assign sense  = GPIO7;

    // Heartbeat counter (no reset needed - just free-runs)
    always @(posedge CLK_25MHz) begin
        heartbeat_cnt <= heartbeat_cnt + 1'b1;
    end

    // nRES: Release reset line (open-drain output, 1 = hi-Z = released)
    assign nRES = 1'b1;

    // Tri-state control signals (match Logic Hamr)
    assign nIRQ = 1'bZ;
    assign nNMI = 1'bZ;
    assign nINH = 1'bZ;
    assign nDMA = 1'bZ;

    // Daisy chain pass-through (critical for downstream cards like A2DVI!)
    assign DMA_IN = DMA_OUT;
    assign INT_IN = INT_OUT;

    // =========================================================================
    // Data Bus Control
    // =========================================================================

    // IWM active during device select reads
    wire iwm_oe = !nDEVICE_SELECT && R_nW;

    // Data output mux: ROM has priority, then IWM
    assign data_out_mux = rom_oe ? rom_data : iwm_data_out;
    assign data_out_enable = rom_oe || iwm_oe;

    // Directly drive data bus (directly individual bits for proper tristate)
    assign data = data_out_enable ? data_out_mux : 8'bZZZZZZZZ;

    // =========================================================================
    // Address Decoder - manages ROM output enable and expansion ROM flag
    // =========================================================================

    addr_decoder u_addr_decoder (
        .addr                (addr),
        .clk                 (sig_7M),
        .nI_O_STROBE         (nI_O_STROBE),
        .nI_O_SELECT         (nI_O_SELECT),
        .nRES                (nRES),
        .rom_oe              (rom_oe),
        .rom_expansion_active(rom_expansion_active)
    );

    // =========================================================================
    // IWM - Integrated Woz Machine disk controller
    // =========================================================================

    iwm u_iwm (
        .addr           (addr[3:0]),
        .nDEVICE_SELECT (nDEVICE_SELECT),
        .fclk           (sig_7M),
        .Q3             (Q3),
        .nRES           (nRES),
        .data_in        (data),
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
    // Boot ROM - Liron firmware
    // =========================================================================

    // ROM address translation:
    //   $C4xx (nI_O_SELECT) → ROM $000-$0FF (use addr[7:0])
    //   $C8xx-$CFxx (nI_O_STROBE) → ROM $000-$7FF (use addr[10:0])
    wire [10:0] rom_addr = !nI_O_SELECT ? {3'b000, addr[7:0]} : addr[10:0];

    boot_rom u_boot_rom (
        .clk  (sig_7M),
        .addr ({1'b0, rom_addr}),  // Extend to 12 bits for 4KB ROM array
        .data (rom_data)
    );

endmodule
