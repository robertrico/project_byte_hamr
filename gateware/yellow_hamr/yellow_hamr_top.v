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
// GPIO Header Pinout (directly compatible with FujiNet):
//   GPIO1:    phase[0] - SmartPort command bit 0
//   GPIO2:    phase[1] - SmartPort command bit 1
//   GPIO3:    phase[2] - SmartPort command bit 2
//   GPIO4:    phase[3] - SmartPort command bit 3
//   GPIO5:    wrdata   - Serial write data (to device)
//   GPIO6:    rddata   - Serial read data (from device) [INPUT]
//   GPIO7:    sense    - Write-protect / ACK (from device) [INPUT]
//   GPIO8:    _enbl1   - Drive 1 enable (active LOW)
//   GPIO9:    _wrreq   - Write request (active LOW)
//   GPIO10:   _enbl2   - Drive 2 enable (active LOW)
//   GPIO11:   rom_expansion_active - Expansion ROM active (debug)
//   GPIO12:   Level shifter OE (active-low, for Apple II data bus)
// =============================================================================

module yellow_hamr_top (
    // System clock (unused by IWM - uses 7M directly)
    input wire        CLK_25MHz,

    // Apple II bus interface
    input wire [11:0] addr,             // Address bus (directly, as individual pins)
    // Data bus as individual pins (like Logic Hamr)
    inout wire        D0,
    inout wire        D1,
    inout wire        D2,
    inout wire        D3,
    inout wire        D4,
    inout wire        D5,
    inout wire        D6,
    inout wire        D7,
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
    // Additional outputs
    output wire       GPIO9,            // _wrreq
    output wire       GPIO10,           // _enbl2
    output wire       GPIO11,           // rom_expansion_active (debug)
    output wire       GPIO12            // Level shifter OE (directly controls D[7:0] buffer, active-low)
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

    // 2-stage synchronizers for ESP32 signals (filters metastability from async IWM latches)
    reg        _enbl1_s1, _enbl1_s2;
    reg        _enbl2_s1, _enbl2_s2;
    reg        _wrreq_s1, _wrreq_s2;

    // ROM signals
    wire [7:0] rom_data;
    wire       rom_oe;
    wire       rom_expansion_active;

    // IWM signals
    wire [7:0] iwm_data_out;

    // Data bus control
    reg        data_out_enable;
    wire [7:0] data_out_mux;

    // =========================================================================
    // GPIO Assignments - Drive interface directly from IWM
    // =========================================================================

    assign GPIO1  = phase[0];
    assign GPIO2  = phase[1];
    assign GPIO3  = phase[2];
    assign GPIO4  = phase[3];
    assign GPIO5  = wrdata;
    assign GPIO8  = _enbl1_s2;  // 2-stage sync for clean ESP32 signaling

    // Input mapping from GPIO
    assign rddata = GPIO6;
    assign sense  = GPIO7;

    // =========================================================================
    // Additional IWM outputs for FujiNet compatibility
    // =========================================================================

    assign GPIO9  = _wrreq_s2;         // Write request (active low, 2-stage sync)
    assign GPIO10 = _enbl2_s2;         // Drive 2 enable (active low, 2-stage sync)
    assign GPIO11 = rom_expansion_active;  // Expansion ROM active (debug)

    // =========================================================================
    // Level Shifter OE Control (active-low)
    // =========================================================================
    // Enable level shifter output when:
    //   - Reading (R_nW=1) AND
    //   - Either IWM selected (nDEVICE_SELECT=0) OR ROM outputting (rom_oe=1)
    // This matches the original Yellowstone: _en245 = ~(~_devsel || ~_romoe)

    wire lvl_shift_oe = R_nW && (!nDEVICE_SELECT || rom_oe);
    assign GPIO12 = ~lvl_shift_oe;     // Active-low to level shifter

    // Heartbeat counter (no reset needed - just free-runs)
    // 2-stage synchronizer for ESP32 signals (IWM uses async latches, need proper CDC)
    always @(posedge CLK_25MHz) begin
        heartbeat_cnt <= heartbeat_cnt + 1'b1;

        // Stage 1: capture (may be metastable)
        _enbl1_s1 <= _enbl1;
        _enbl2_s1 <= _enbl2;
        _wrreq_s1 <= _wrreq;

        // Stage 2: resolve metastability (~80ns total delay)
        _enbl1_s2 <= _enbl1_s1;
        _enbl2_s2 <= _enbl2_s1;
        _wrreq_s2 <= _wrreq_s1;
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

    // Register the output enable like Logic Hamr does
    // This ensures proper timing alignment with the level shifter
    wire data_oe_next = R_nW && (rom_oe || iwm_oe);
    always @(posedge CLK_25MHz) begin
        data_out_enable <= data_oe_next;
    end

    // Drive data bus as individual bits (like Logic Hamr)
    assign D0 = data_out_enable ? data_out_mux[0] : 1'bZ;
    assign D1 = data_out_enable ? data_out_mux[1] : 1'bZ;
    assign D2 = data_out_enable ? data_out_mux[2] : 1'bZ;
    assign D3 = data_out_enable ? data_out_mux[3] : 1'bZ;
    assign D4 = data_out_enable ? data_out_mux[4] : 1'bZ;
    assign D5 = data_out_enable ? data_out_mux[5] : 1'bZ;
    assign D6 = data_out_enable ? data_out_mux[6] : 1'bZ;
    assign D7 = data_out_enable ? data_out_mux[7] : 1'bZ;

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
    // Boot ROM - Liron firmware
    // =========================================================================

    // ROM address mapping for SLOT 4:
    //   $C4xx (nI_O_SELECT low)  → ROM $400-$4FF (slot 4 boot code)
    //   $C8xx-$CFxx (nI_O_STROBE low) → ROM $800-$FFF (expansion ROM)
    // The Liron ROM has slot-specific boot code at $n00 for each slot n
    wire [11:0] rom_addr = !nI_O_SELECT ? {4'b0100, addr[7:0]}   // Slot 4: ROM $400-$4FF
                                        : {1'b1, addr[10:0]};    // Expansion: ROM $800-$FFF

    boot_rom u_boot_rom (
        .clk  (CLK_25MHz),         // 25MHz for fast response (~40ns latency)
        .addr (rom_addr),  // 12-bit address for 4KB ROM array
        .data (rom_data)
    );

endmodule
