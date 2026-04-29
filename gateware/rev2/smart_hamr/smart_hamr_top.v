`timescale 1ns / 1ps
// =============================================================================
// Smart Hamr — Top Level Module (Liron / SmartPort over ESP32) — Rev 2
// =============================================================================
// Apple IIe-compatible Liron disk controller. IWM emulation + 4 KB Liron boot
// ROM; SmartPort traffic bridged to a FujiNet-class ESP32 over the GPIO
// header.
//
// Rev 2 differences vs Rev 1:
//   - Address bus: bundled addr[11:0]  ->  individual A0..A15 inputs
//                  (Rev 2 base LPF constrains A0..A15 individually)
//   - GPIO header: GPIO1..GPIO12  ->  GPIO_1..GPIO_20 (positional numbering,
//                  *not* the same pad assignments as Rev 1)
//   - DATA_OE: dedicated level-shifter OE pin at B20 (was on GPIO12 in Rev 1)
//   - nRES_READ: new input at A5 (Apple II reset monitor; write side on A8)
//                Now wired into the reset network through a 2-FF synchronizer
//                so Ctrl-Reset propagates into the IWM and addr_decoder
//   - CLK_100MHz: declared so the base LPF's F3 constraint matches; unused by
//                 the design (single-clock 7 MHz, no SDRAM/flash, no PLL)
//
// Memory Map (Slot 4):
//   $C0C0-$C0CF  nDEVICE_SELECT  IWM registers
//   $C400-$C4FF  nI_O_SELECT     Boot ROM ($000-$0FF)
//   $C800-$CFFF  nI_O_STROBE     Boot ROM ($100-$7FF) when expansion active
//
// FujiNet ↔ Rev 2 GPIO Mapping
// ---------------------------------------------------------------------------
// Tuned for the daughter board layout where the ESP32-WROVER-E sits on the
// right of the perfboard (USB up) with the SD card on its left and the
// 2x20 IDC ribbon header at the bottom. FujiNet signals cluster on
// GPIO_9..GPIO_20 so they drop straight down from the ESP32's right edge
// and the two LEFT-edge stragglers (_enbl2 = IO3/RX, wrdata = IO22) land on
// GPIO_9/GPIO_10 — the right end of the ribbon's left half.
//
//   GPIO_1   = nDEVICE_SELECT  out  (LA debug probe; isolated on far end)
//   GPIO_9   = _enbl2          out  → ESP32 IO3/RX  (LEFT pin 34)
//   GPIO_10  = wrdata          out  → ESP32 IO22    (LEFT pin 36)
//   GPIO_12  = rddata          in   ← ESP32 IO14    (RIGHT pin 12, idle-LOW)
//   GPIO_13  = sense/ACK       in   ← ESP32 IO27    (RIGHT pin 11)
//   GPIO_14  = _wrreq          out  → ESP32 IO26    (RIGHT pin 10)
//   GPIO_15  = phase[1]        out  → ESP32 IO33    (RIGHT pin 8)
//   GPIO_16  = phase[0]        out  → ESP32 IO32    (RIGHT pin 7, damped pad)
//   GPIO_17  = phase[3]        out  → ESP32 IO35    (RIGHT pin 6)
//   GPIO_18  = phase[2]        out  → ESP32 IO34    (RIGHT pin 5)
//   GPIO_20  = _enbl1          out  → ESP32 IO36    (RIGHT pin 3)
//
//   Damped (33Ω series) pads: GPIO_1, GPIO_6, GPIO_11, GPIO_16. Used for
//   debug + phase[0] only — fast serial lines (rddata/wrdata) are on
//   un-damped pads (GPIO_12, GPIO_10).
//
//   Unused: GPIO_2..GPIO_8, GPIO_11, GPIO_19. Driven 0 to keep pads defined.
// =============================================================================

module smart_hamr_top (
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
    input  wire       nRES_READ,        // Rev 2: monitor Apple II reset line
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

    // ---- 100 MHz System Clock (Rev 2) — declared for LPF match, unused ----
    input wire        CLK_100MHz,

    // ---- GPIO Header (Rev 2: 20 pins, positional numbering) ----
    output wire       GPIO_1,            // /DEV debug → LA probe
    output wire       GPIO_2,
    output wire       GPIO_3,
    output wire       GPIO_4,
    output wire       GPIO_5,
    output wire       GPIO_6,
    output wire       GPIO_7,
    output wire       GPIO_8,
    output wire       GPIO_9,            // _enbl2
    output wire       GPIO_10,           // wrdata
    output wire       GPIO_11,
    input  wire       GPIO_12,           // rddata from ESP32 (idle-LOW)
    input  wire       GPIO_13,           // sense/ACK from ESP32
    output wire       GPIO_14,           // _wrreq
    output wire       GPIO_15,           // phase[1]
    output wire       GPIO_16,           // phase[0]
    output wire       GPIO_17,           // phase[3]
    output wire       GPIO_18,           // phase[2]
    output wire       GPIO_19,
    output wire       GPIO_20            // _enbl1
);

    // =========================================================================
    // Address Bus Bundling
    // =========================================================================
    // Submodules (addr_decoder, iwm) expect a bundled addr[11:0] representing
    // A0..A11. A12..A15 aren't needed since slot decode is handled by
    // nDEVICE_SELECT / nI_O_SELECT / nI_O_STROBE.
    wire [11:0] addr = {A11, A10, A9, A8, A7, A6, A5, A4, A3, A2, A1, A0};

    // Suppress unused-input warnings for A12..A15 and CLK_100MHz.
    wire _unused = &{1'b0, A12, A13, A14, A15, CLK_100MHz};

    // =========================================================================
    // Internal Signals
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
    wire       iwm_q7;
    wire       iwm_q7_stable;

    // =========================================================================
    // Power-On Reset (7 MHz domain)
    // =========================================================================
    // Holds reset low for 15 fclk cycles (~2 µs) after FPGA configuration.
    // ECP5 initializes registers to their declared values on configuration.
    assign nRES = 1'b1;  // open-drain, always released

    reg [3:0] por_counter = 4'd0;
    wire      por_n = &por_counter;

    always @(posedge sig_7M) begin
        if (!por_n)
            por_counter <= por_counter + 4'd1;
    end

    // =========================================================================
    // Apple II Reset (nRES_READ) — 2-FF synchronizer, sig_7M domain
    // =========================================================================
    // Async source from pad A5; reset to 0 so a floating/bouncing line looks
    // reset-asserted until the synchronizer fills with clean ones. Mirrors
    // Block Hamr Rev 2's pattern (single domain only — Smart Hamr has no
    // 25 MHz clock).
    reg [1:0] nres_sync_7m = 2'b00;
    always @(posedge sig_7M)
        nres_sync_7m <= {nres_sync_7m[0], nRES_READ};
    wire nRES_READ_sync7 = nres_sync_7m[1];

    // Combined reset (active low): POR AND Apple II reset.
    wire rst_7m_n = por_n & nRES_READ_sync7;

    // =========================================================================
    // Tri-state / pass-through control signals
    // =========================================================================
    assign nIRQ = 1'bZ;
    assign nNMI = 1'bZ;
    assign nINH = 1'bZ;
    assign nDMA = 1'bZ;
    assign DMA_IN = DMA_OUT;
    assign INT_IN = INT_OUT;

    // =========================================================================
    // GPIO outputs to ESP32 (FPGA → ESP32)
    // =========================================================================
    // Phase outputs are registered to avoid combinatorial glitches on ESP32
    // inputs. Reset clears them so the ESP32 sees a defined idle state during
    // Apple II reset.
    reg [3:0] phase_gpio = 4'b0000;

    always @(posedge sig_7M or negedge rst_7m_n) begin
        if (!rst_7m_n)
            phase_gpio <= 4'b0000;
        else
            phase_gpio <= phase;
    end

    assign GPIO_1  = nDEVICE_SELECT;     // LA debug
    assign GPIO_9  = _enbl2;
    assign GPIO_10 = wrdata;
    assign GPIO_14 = _wrreq;
    assign GPIO_15 = phase_gpio[1];
    assign GPIO_16 = phase_gpio[0];
    assign GPIO_17 = phase_gpio[3];
    assign GPIO_18 = phase_gpio[2];
    assign GPIO_20 = _enbl1;

    // Unused outputs — drive 0 so floating pads don't leak through inputs.
    assign GPIO_2  = iwm_q7_stable;       // DEBUG: Q7 visibility for LA
    assign GPIO_3  = nRES_READ;           // DEBUG: Apple II reset line for LA
    assign GPIO_4  = 1'b0;
    assign GPIO_5  = 1'b0;
    assign GPIO_6  = 1'b0;
    assign GPIO_7  = 1'b0;
    assign GPIO_8  = 1'b0;
    assign GPIO_11 = 1'b0;
    assign GPIO_19 = 1'b0;

    // =========================================================================
    // GPIO inputs from ESP32 (ESP32 → FPGA)
    // =========================================================================
    // rddata: invert GPIO_12 (ESP32 sends idle-LOW/pulse-HIGH, IWM expects
    // idle-HIGH/pulse-LOW). The read shift register self-gates on q7_stable
    // internally (iwm.v), so no extra gate is needed here.
    assign rddata = ~GPIO_12;
    assign sense  = GPIO_13;

    // Suppress unused-input warning for q7_stable (still exposed by iwm).
    wire _unused_iwm = &{1'b0, iwm_q7, iwm_q7_stable};

    // =========================================================================
    // Level Shifter OE Control (active-low) — Rev 2 dedicated pin at B20
    // =========================================================================
    // Enable U12 whenever our slot is being accessed so the Apple II data bus
    // is gated through to the FPGA. R_nW controls U12's DIR pin externally
    // (driven by U7 inverter), not OE.
    wire lvl_shift_oe = !nDEVICE_SELECT || rom_oe;
    assign DATA_OE = ~lvl_shift_oe;

    // =========================================================================
    // Data Bus — ROM has priority, then IWM
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
    // Address Decoder — manages ROM output enable + expansion ROM flag
    // =========================================================================
    addr_decoder u_addr_decoder (
        .addr                (addr),
        .clk                 (sig_7M),
        .nI_O_STROBE         (nI_O_STROBE),
        .nI_O_SELECT         (nI_O_SELECT),
        .nRES                (rst_7m_n),
        .rom_oe              (rom_oe),
        .rom_expansion_active(rom_expansion_active)
    );

    // =========================================================================
    // IWM — Integrated Woz Machine disk controller
    // =========================================================================
    iwm u_iwm (
        .addr           (addr[3:0]),
        .nDEVICE_SELECT (nDEVICE_SELECT),
        .fclk           (sig_7M),
        .Q3             (Q3),
        .R_nW           (R_nW),
        .nRES           (rst_7m_n),
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
        .q7_stable_out  (iwm_q7_stable)
    );

    // =========================================================================
    // Boot ROM — 4 KB Liron firmware (combinatorial read)
    // =========================================================================
    // Slot 4 mapping:
    //   $C4xx (nI_O_SELECT low)        -> ROM $400-$4FF
    //   $C8xx-$CFxx (nI_O_STROBE low)  -> ROM $800-$FFF (expansion ROM)
    wire [11:0] rom_addr = !nI_O_SELECT ? {4'b0100, addr[7:0]}
                                        : {1'b1,    addr[10:0]};

    boot_rom u_boot_rom (
        .clk  (sig_7M),
        .addr (rom_addr),
        .data (rom_data)
    );

endmodule
