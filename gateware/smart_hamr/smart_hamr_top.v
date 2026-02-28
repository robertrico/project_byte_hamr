`timescale 1ns / 1ps
// =============================================================================
// Yellow Hamr - Top Level Module
// =============================================================================
// Apple IIe-compatible Liron disk controller (IWM-based SmartPort interface)
// for the Byte Hamr FPGA card.
//
// Adapted from Steve Chamberlin's Yellowstone project:
//   Original target: Lattice MachXO2-1200HC (no oscillator, fclk-only design)
//   This target: Lattice ECP5-85F (Byte Hamr)
//
// SINGLE CLOCK DOMAIN DESIGN
// ~~~~~~~~~~~~~~~~~~~~~~~~~~
// This design runs entirely from sig_7M (the Apple II's 7 MHz clock),
// matching Yellowstone's single-clock architecture. The on-board 25 MHz
// oscillator is not connected to the FPGA fabric.
//
// All GPIO outputs to the ESP32 are driven directly from IWM combinatorial
// outputs, exactly as Yellowstone drives its DB-19 connector. The ESP32
// samples at GPIO speeds (~microseconds), well above any glitch concern.
//
// Memory Map (Slot 4):
//   $C0C0-$C0CF  nDEVICE_SELECT  IWM registers
//   $C400-$C4FF  nI_O_SELECT     Boot ROM ($000-$0FF)
//   $C800-$CFFF  nI_O_STROBE     Boot ROM ($100-$7FF) when expansion active
//
// GPIO Header Pinout (FujiNet Rev0-compatible, with FPGA tri-state buffer):
//   GPIO1:    _enbl1    - Drive 1 enable (active LOW)         → ESP32 IO36
//   GPIO2:    phase[2]  - SmartPort command bit 2             → ESP32 IO34
//   GPIO3:    phase[3]  - SmartPort command bit 3             → ESP32 IO35
//   GPIO4:    phase[0]  - SmartPort command bit 0             → ESP32 IO32
//   GPIO5:    phase[1]  - SmartPort command bit 1             → ESP32 IO33
//   GPIO6:    sig_7M    - Apple II 7 MHz clock (debug probe)    [OUTPUT]
//   GPIO7:    _wreq     - Write request (active LOW)          → ESP32 IO26
//   GPIO8:    sense     - Write-protect / ACK (from ESP32)    [INPUT] ← ESP32 IO27
//   GPIO9:    rddata    - Serial read data (from ESP32)       [INPUT] ← ESP32 IO14
//   GPIO10:   wrdata    - Serial write data (to ESP32)        → ESP32 IO22
//   GPIO11:   _enbl2    - Drive 2 enable (active LOW)         → ESP32 IO21
//   GPIO12:   Level shifter OE (active-low, for Apple II data bus)
// =============================================================================

module smart_hamr_top (
    // Apple II bus interface
    input wire [11:0] addr,             // Address bus
    // Data bus as individual pins (like Logic Hamr)
    inout wire        D0,
    inout wire        D1,
    inout wire        D2,
    inout wire        D3,
    inout wire        D4,
    inout wire        D5,
    inout wire        D6,
    inout wire        D7,
    input wire        sig_7M,           // 7 MHz clock (IWM FCLK) - THE clock
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

    // ESP32 FujiNet interface (via GPIO header)
    output wire       GPIO1,            // _enbl1    → ESP32 IO36
    output wire       GPIO2,            // phase[2]  → ESP32 IO34
    output wire       GPIO3,            // phase[3]  → ESP32 IO35
    output wire       GPIO4,            // phase[0]  → ESP32 IO32
    output wire       GPIO5,            // phase[1]  → ESP32 IO33
    output wire       GPIO6,            // DEBUG: sig_7M → LA probe
    output wire       GPIO7,            // _wreq     → ESP32 IO26
    input wire        GPIO8,            // sense/ACK ← ESP32 IO27
    input wire        GPIO9,            // rddata    ← ESP32 IO14
    output wire       GPIO10,           // wrdata    → ESP32 IO22
    output wire       GPIO11,           // _enbl2    → ESP32 IO21
    output wire       GPIO12            // Level shifter OE (active-low)
);

    // =========================================================================
    // Internal Signals
    // =========================================================================

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
    wire       iwm_q7;
    wire       iwm_q7_stable;

    // Data bus output mux
    wire [7:0] data_out_mux;

    // =========================================================================
    // GPIO Output Assignments (FPGA → ESP32)
    // =========================================================================

    // Phase outputs - registered to avoid combinatorial glitches on ESP32 inputs
    reg [3:0]  phase_gpio = 4'b0000;

    always @(posedge sig_7M or negedge por_n) begin
        if (!por_n)
            phase_gpio <= 4'b0000;
        else
            phase_gpio <= phase;
    end

    assign GPIO4  = phase_gpio[0];     // phase[0] → ESP32 IO32 (SP_PHI0/SP_REQ)
    assign GPIO5  = phase_gpio[1];     // phase[1] → ESP32 IO33 (SP_PHI1)
    assign GPIO2  = phase_gpio[2];     // phase[2] → ESP32 IO34 (SP_PHI2)
    assign GPIO3  = phase_gpio[3];     // phase[3] → ESP32 IO35 (SP_PHI3)

    assign GPIO1  = _enbl1;             // _enbl1   → ESP32 IO36 (SP_DRIVE1)
    assign GPIO11 = _enbl2;            // _enbl2   → ESP32 IO21 (SP_DRIVE2)
    assign GPIO7  = _wrreq;            // _wrreq   → ESP32 IO26 (SP_WREQ)
    assign GPIO10 = wrdata;            // wrdata   → ESP32 IO22 (SP_WRDATA)
    assign GPIO6  = iwm_q7_stable;            // DEBUG: iwm_q7_stable → LA probe

    // =========================================================================
    // GPIO Input Assignments (ESP32 → FPGA)
    // =========================================================================

    // rddata: invert GPIO9 (ESP32 sends idle-LOW/pulse-HIGH, IWM expects
    // idle-HIGH/pulse-LOW). No q7_stable gate — the read shift register
    // already self-gates on q7_stable internally (iwm.v line 379).
    // The old gate (iwm_q7_stable ? 1'b1 : ~GPIO9) was blanking valid
    // rddata when q7_stable briefly glitched HIGH during receive,
    // causing the ROM to lose sync and hang.
    assign rddata = ~GPIO9;  // rddata ← ESP32 IO14 (SP_RDDATA)

    assign sense = GPIO8;              // sense/ACK ← ESP32 IO27 (SP_WRPROT/SP_ACK)

    // =========================================================================
    // Level Shifter OE Control (active-low)
    // =========================================================================
    // Enable for both reads and writes when IWM or ROM is selected.
    // R_nW controls the '245 DIR pin (direction), not OE.
    wire lvl_shift_oe = !nDEVICE_SELECT || rom_oe;
    assign GPIO12 = ~lvl_shift_oe;

    // =========================================================================
    // Power-On Reset
    // =========================================================================
    // Byte Hamr nRES cannot be read (hardware limitation). POR only.
    // Holds reset low for 15 fclk cycles (~2 us) after FPGA configuration.
    // ECP5 initializes registers to their declared values on configuration.

    assign nRES = 1'b1;

    reg [3:0] por_counter = 4'd0;
    wire      por_n = &por_counter;   // HIGH (released) when counter saturates

    always @(posedge sig_7M) begin
        if (!por_n)
            por_counter <= por_counter + 4'd1;
    end

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
    // ROM has priority, then IWM. Bus is hi-Z when not being read.

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
    // Address Decoder - manages ROM output enable and expansion ROM flag
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
    // IWM - Integrated Woz Machine disk controller
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
        .sense          (sense),
        .rddata         (rddata),
        .q7_out         (iwm_q7),
        .q7_stable_out  (iwm_q7_stable),
    );

    // =========================================================================
    // Boot ROM - Liron firmware (clocked by sig_7M, matching Yellowstone)
    // =========================================================================

    // ROM address mapping for SLOT 4:
    //   $C4xx (nI_O_SELECT low)  -> ROM $400-$4FF (slot 4 boot code)
    //   $C8xx-$CFxx (nI_O_STROBE low) -> ROM $800-$FFF (expansion ROM)
    // The Liron ROM has slot-specific boot code at $n00 for each slot n
    wire [11:0] rom_addr = !nI_O_SELECT ? {4'b0100, addr[7:0]}   // Slot 4: ROM $400-$4FF
                                        : {1'b1, addr[10:0]};    // Expansion: ROM $800-$FFF

    boot_rom u_boot_rom (
        .clk  (sig_7M),            // Same clock as Yellowstone (fclk)
        .addr (rom_addr),
        .data (rom_data)
    );

endmodule
