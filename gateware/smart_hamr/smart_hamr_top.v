`timescale 1ns / 1ps
// =============================================================================
// Smart Hamr — Top Level Module (Liron Card Emulation)
// =============================================================================
//
// This is the complete Liron disk controller card, implemented in an ECP5.
// A real Liron card contains:
//
//   1. IWM chip (Integrated Woz Machine)   — emulated by iwm_ii.v
//   2. 4K ROM (Liron firmware, 341-0372)   — boot_rom.v + liron_rom.mem
//   3. Address decoder / expansion ROM flag — addr_decoder.v
//   4. 74LS245 bus transceiver              — FPGA tristate + GPIO12 OE
//   5. 74LS21 / 74LS30 / 74LS00 glue       — wiring below
//
// Signal flow:
//   Apple IIe bus → FPGA (IWM + ROM + glue) → GPIO header → ESP32 (FujiNet)
//
// SINGLE CLOCK DOMAIN: sig_7M only. CLK_25MHz is unused.
//
// Memory Map (Slot 4):
//   $C0C0-$C0CF  nDEVICE_SELECT  → IWM registers (16 softswitches)
//   $C400-$C4FF  nI_O_SELECT     → Boot ROM ($400-$4FF, slot 4 page)
//   $C800-$CFFF  nI_O_STROBE     → Expansion ROM ($800-$FFF)
//
// GPIO Header Pinout (FujiNet-compatible):
//   GPIO1:  _enbl1    → ESP32 IO36   (Drive 1 enable, active LOW)
//   GPIO2:  phase[2]  → ESP32 IO34   (SmartPort command bit 2)
//   GPIO3:  phase[3]  → ESP32 IO35   (SmartPort command bit 3)
//   GPIO4:  phase[0]  → ESP32 IO32   (SmartPort command bit 0)
//   GPIO5:  phase[1]  → ESP32 IO33   (SmartPort command bit 1)
//   GPIO6:  rd_buf_en ← ESP32 IO25   (Tri-state buffer enable) [INPUT]
//   GPIO7:  _wreq     → ESP32 IO26   (Write request, active LOW)
//   GPIO8:  sense     ← ESP32 IO27   (Write-protect / ACK)     [INPUT]
//   GPIO9:  rddata    ← ESP32 IO14   (Serial read data)        [INPUT]
//   GPIO10: wrdata    → ESP32 IO22   (Serial write data)
//   GPIO11: _enbl2    → ESP32 IO21   (Drive 2 enable, active LOW)
//   GPIO12: lvl_shift → data bus     (Level shifter OE, active LOW)
// =============================================================================

module smart_hamr_top (
    // System clock (exists on board, deliberately unused)
    input wire        CLK_25MHz,

    // Apple II bus interface
    input wire [11:0] addr,
    inout wire        D0, D1, D2, D3, D4, D5, D6, D7,
    input wire        sig_7M,
    input wire        Q3,
    input wire        R_nW,
    input wire        nDEVICE_SELECT,
    input wire        nI_O_SELECT,
    input wire        nI_O_STROBE,
    output wire       nRES,

    // Control signals (active low, directly hi-Z to avoid bus contention)
    input  wire       RDY,
    output wire       nIRQ,
    output wire       nNMI,
    output wire       nDMA,
    output wire       nINH,

    // Daisy chain
    input  wire       DMA_OUT,
    output wire       DMA_IN,
    input  wire       INT_OUT,
    output wire       INT_IN,

    // Clock phases (unused in single-clock design, but pinned)
    input  wire       PHI0,
    input  wire       PHI1,
    input  wire       uSync,

    // ESP32 FujiNet interface (via GPIO header)
    output wire       GPIO1,
    output wire       GPIO2,
    output wire       GPIO3,
    output wire       GPIO4,
    output wire       GPIO5,
    input  wire       GPIO6,
    output wire       GPIO7,
    input  wire       GPIO8,
    input  wire       GPIO9,
    output wire       GPIO10,
    output wire       GPIO11,
    output wire       GPIO12
);

    // =========================================================================
    // Internal Signals
    // =========================================================================

    // IWM ↔ drive interface
    wire [3:0] phase;
    wire       wrdata;
    wire       _enbl1;
    wire       _enbl2;
    wire       _wrreq;

    // ESP32 → FPGA inputs
    wire       rd_buf_en;
    wire       rddata;
    wire       sense;

    // IWM → data bus
    wire [7:0] iwm_data_out;
    wire       q6, q7;

    // ROM → data bus
    wire [7:0] rom_data;
    wire       rom_oe;
    wire       rom_expansion_active;

    // =========================================================================
    // Power-On Reset
    // =========================================================================
    // Byte Hamr nRES pin cannot be read (hardware limitation). POR only.
    // Holds reset for 15 fclk cycles (~2 us) after FPGA configuration.
    // ECP5 initializes registers to their declared values on configuration.

    assign nRES = 1'b1;

    reg [3:0] por_counter = 4'd0;
    wire      por_n = &por_counter;

    always @(posedge sig_7M) begin
        if (!por_n)
            por_counter <= por_counter + 4'd1;
    end

    // =========================================================================
    // Control Signal Tri-State (match Logic Hamr)
    // =========================================================================
    // These signals are active-low, directly tri-stated to avoid bus contention.

    assign nIRQ = 1'bZ;
    assign nNMI = 1'bZ;
    assign nINH = 1'bZ;
    assign nDMA = 1'bZ;

    // Daisy chain pass-through (critical for downstream cards like A2DVI!)
    assign DMA_IN = DMA_OUT;
    assign INT_IN = INT_OUT;

    // =========================================================================
    // ESP32 GPIO — Input Conditioning
    // =========================================================================

    // Tri-state buffer control from ESP32 (active LOW = enable rddata output).
    // Replaces the external 74LVC125 buffer on real FujiNet hardware.
    // When rd_buf_en is HIGH (disabled), rddata is forced to idle-HIGH
    // (IWM idle state), preventing bus contention when ESP32 is reconfiguring.
    assign rd_buf_en = GPIO6;

    // FujiNet drives rddata as idle-LOW / pulse-HIGH (SPI mode 0, no inversion).
    // The IWM expects idle-HIGH / pulse-LOW (falling-edge = '1' bit), matching
    // original Apple hardware where an inverting amplifier sits between the
    // drive head and the IWM chip. Invert here to restore correct polarity.
    assign rddata = rd_buf_en ? 1'b1 : ~GPIO9;

    assign sense = GPIO8;

    // =========================================================================
    // ESP32 GPIO — Output Assignments (FPGA → ESP32)
    // =========================================================================

    // Phase outputs — registered to suppress combinatorial glitches
    reg [3:0] phase_gpio = 4'b0000;

    always @(posedge sig_7M or negedge por_n) begin
        if (!por_n)
            phase_gpio <= 4'b0000;
        else
            phase_gpio <= phase;
    end

    assign GPIO4  = phase_gpio[0];          // phase[0] → ESP32 IO32
    assign GPIO5  = phase_gpio[1];          // phase[1] → ESP32 IO33
    assign GPIO2  = phase_gpio[2];          // phase[2] → ESP32 IO34
    assign GPIO3  = phase_gpio[3];          // phase[3] → ESP32 IO35

    // Enable and write-request: gated by rd_buf_en to avoid
    // spurious assertions while ESP32 is reconfiguring GPIO direction.
    assign GPIO1  = rd_buf_en ? 1'b1 : _enbl1;   // _enbl1 → ESP32 IO36
    assign GPIO11 = _enbl2;                        // _enbl2 → ESP32 IO21
    assign GPIO7  = rd_buf_en ? 1'b1 : _wrreq;   // _wrreq → ESP32 IO26
    assign GPIO10 = wrdata;                        // wrdata → ESP32 IO22

    // =========================================================================
    // Level Shifter OE (74LVC245 on the Byte Hamr PCB)
    // =========================================================================
    // Active-low. Enable for both reads and writes when IWM or ROM is selected.
    // R_nW controls the '245 DIR pin (direction), not OE.

    wire lvl_shift_oe = !nDEVICE_SELECT || rom_oe;
    assign GPIO12 = ~lvl_shift_oe;

    // =========================================================================
    // Data Bus — Output Mux & Tri-State
    // =========================================================================
    // ROM has priority (it covers both I/O select and expansion ROM),
    // then IWM data. Bus is hi-Z when not being read.

    wire [7:0] data_out_mux = rom_oe ? rom_data : iwm_data_out;
    wire       data_oe = R_nW && (rom_oe || !nDEVICE_SELECT);

    assign D0 = data_oe ? data_out_mux[0] : 1'bZ;
    assign D1 = data_oe ? data_out_mux[1] : 1'bZ;
    assign D2 = data_oe ? data_out_mux[2] : 1'bZ;
    assign D3 = data_oe ? data_out_mux[3] : 1'bZ;
    assign D4 = data_oe ? data_out_mux[4] : 1'bZ;
    assign D5 = data_oe ? data_out_mux[5] : 1'bZ;
    assign D6 = data_oe ? data_out_mux[6] : 1'bZ;
    assign D7 = data_oe ? data_out_mux[7] : 1'bZ;

    // =========================================================================
    // Address Decoder — ROM output enable & expansion ROM flag
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
    // IWM — Integrated Woz Machine (state machine + registers)
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
        .q6_out         (q6),
        .q7_out         (q7)
    );

    // =========================================================================
    // Boot ROM — Liron firmware (4K, combinatorial read)
    // =========================================================================
    // ROM address mapping for SLOT 4:
    //   $C4xx (nI_O_SELECT low)     → ROM $400-$4FF (slot 4 boot code)
    //   $C8xx-$CFxx (nI_O_STROBE)   → ROM $800-$FFF (expansion ROM)
    // The Liron ROM has slot-specific boot code at $n00 for each slot n.

    wire [11:0] rom_addr = !nI_O_SELECT ? {4'b0100, addr[7:0]}
                                        : {1'b1, addr[10:0]};

    boot_rom u_boot_rom (
        .clk  (sig_7M),
        .addr (rom_addr),
        .data (rom_data)
    );

endmodule
