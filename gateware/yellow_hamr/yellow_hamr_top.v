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
// Like Yellowstone, this design runs entirely from sig_7M (the Apple II's
// 7 MHz clock). CLK_25MHz is deliberately unused to avoid introducing a
// second clock domain, which would cause nextpnr to analyze cross-domain
// timing on the purely combinatorial IWM-to-GPIO paths and create false
// timing violations. Yellowstone's LPF uses "BLOCK ASYNCPATHS" to suppress
// this; our approach is simpler — don't create the second domain at all.
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
//   GPIO11:   Q7       - IWM write mode flag (for ESP32 command decoding)
//   GPIO12:   Level shifter OE (active-low, for Apple II data bus)
// =============================================================================

module yellow_hamr_top (
    // System clock - DIRECTLY from Apple II bus, no on-board oscillator needed
    // CLK_25MHz exists on the board but is deliberately unused to keep
    // the design in a single clock domain (matching Yellowstone architecture)
    input wire        CLK_25MHz,

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
    output wire       GPIO11,           // Q7 (IWM write mode flag)
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
    wire       iwm_dbg_buf7;
    wire       iwm_dbg_latch_sync;

    // Data bus output mux
    wire [7:0] data_out_mux;

    // =========================================================================
    // GPIO Phase Output with Bus Reset Debounce
    // =========================================================================
    // Phase outputs to FujiNet GPIO, with debounce on the bus reset pattern.
    //
    // Problem: During Apple II boot scan, the firmware reads $C0C0-$C0CF
    // (IWM soft-switches) for each slot. These accesses toggle phase registers,
    // causing transient phases = 0101 (bus reset pattern) lasting only
    // microseconds. FujiNet's service loop sees this as a real bus reset and
    // clears all device addresses (_devnum = 0). When the ROM later sends
    // STATUS dest=0 (bus-level, handled internally by the ROM), FujiNet
    // matches it to an uninitialized device and tries to send a response
    // that nobody is waiting for — blocking the bus for 30ms.
    //
    // Fix: Debounce the 0101 pattern. Hold previous phase values on GPIO
    // for 3500 fclk cycles (~500µs) when phases = 0101. Real bus resets
    // persist for ~80ms and will propagate after the debounce. Transient
    // boot-scan glitches clear within microseconds and are filtered out.
    // Non-reset phase patterns (0000, 1010, 1011, etc.) pass through
    // immediately with no delay.

    reg [3:0]  phase_gpio = 4'b0000;
    reg [11:0] reset_debounce_ctr = 12'd0;

    always @(posedge sig_7M or negedge por_n) begin
        if (!por_n) begin
            phase_gpio         <= 4'b0000;
            reset_debounce_ctr <= 12'd0;
        end
        else begin
            if (phase == 4'b0101) begin
                // Bus reset pattern detected — start/continue debounce
                if (reset_debounce_ctr != 12'd3500) begin
                    reset_debounce_ctr <= reset_debounce_ctr + 1'b1;
                end
                else begin
                    // Debounce expired — real bus reset, propagate
                    phase_gpio <= phase;
                end
            end
            else begin
                // Non-reset pattern — pass through immediately, reset counter
                reset_debounce_ctr <= 12'd0;
                phase_gpio         <= phase;
            end
        end
    end

    assign GPIO1  = phase_gpio[0];
    assign GPIO2  = phase_gpio[1];
    assign GPIO3  = phase_gpio[2];
    assign GPIO4  = phase_gpio[3];

    assign GPIO5  = wrdata;
    assign GPIO9  = _wrreq;
    assign GPIO11 = iwm_dbg_buf7;      // DEBUG: _underrun (LOW = serializer starved)

    // Drive enable lines: pass through directly to FujiNet.
    // FujiNet's own service loop checks SmartPort phases (ph3 & ph1) before
    // the Disk II motor state machine, so it won't misinterpret enables as
    // Disk II activity when SmartPort phases are active.
    assign GPIO8  = _enbl1;
    assign GPIO10 = _enbl2;

    // Input mapping from GPIO
    // FujiNet drives rddata as idle-LOW / pulse-HIGH (SPI mode 0, no inversion).
    // The IWM expects idle-HIGH / pulse-LOW (falling-edge = '1' bit), matching
    // original Apple hardware where an inverting amplifier sits between the
    // drive head and the IWM chip. Invert here to restore correct polarity.
    assign rddata = ~GPIO6;

    // =========================================================================
    // Sense/ACK Pulse Stretcher
    // =========================================================================
    // The Liron ROM's ACK poll at $C943 runs only 10 iterations (~114µs).
    // FujiNet's ISR decodes the SPI command packet before asserting ACK,
    // which can take ~1100µs. The poll starts ~1130µs after REQ, so the
    // FujiNet ACK and the ROM poll window overlap by only ~18µs. Jitter in
    // FujiNet's SPI decode can push ACK past the last poll read.
    //
    // Fix: when GPIO7 (sense/ACK) goes LOW, hold sense LOW for at least
    // 1400 fclk cycles (~200µs at 7MHz). This guarantees the ROM catches
    // ACK even if FujiNet is a few µs late. The 200µs stretch is harmless:
    // the ROM exits the poll on the first LOW read and proceeds to cleanup
    // at $C949 (~6µs), then enters the receive handler at $C960, which
    // doesn't poll sense until $C97D (another ~30µs of setup). FujiNet's
    // service loop takes milliseconds to reach iwm_ack_set(), so the
    // stretch has long expired before sense needs to go HIGH again.
    reg [10:0] sense_stretch_ctr = 11'd0;
    reg        gpio7_prev = 1'b1;
    wire       sense_stretched;

    always @(posedge sig_7M or negedge por_n) begin
        if (!por_n) begin
            sense_stretch_ctr <= 0;
            gpio7_prev        <= 1'b1;
        end
        else begin
            gpio7_prev <= GPIO7;
            if (gpio7_prev & ~GPIO7) begin
                // Falling edge on GPIO7 — start stretch
                sense_stretch_ctr <= 11'd1400;
            end
            else if (sense_stretch_ctr != 0) begin
                sense_stretch_ctr <= sense_stretch_ctr - 1'b1;
            end
        end
    end

    // Sense is LOW if GPIO7 is LOW OR stretch counter is active
    assign sense_stretched = (GPIO7 == 1'b0) || (sense_stretch_ctr != 0) ? 1'b0 : 1'b1;
    assign sense = sense_stretched;

    // =========================================================================
    // Level Shifter OE Control (active-low)
    // =========================================================================
    // Matches Yellowstone: _en245 = ~(~_devsel || ~_romoe)
    // Enable when reading AND (IWM selected OR ROM outputting)

    // Level shifter must be enabled for BOTH reads and writes when IWM or
    // ROM is selected. R_nW controls the '245 DIR pin (direction), not OE.
    // Previously gated on R_nW, which disabled the shifter during CPU writes
    // — the FPGA saw floating $FF on data_in, corrupting every IWM buffer write.
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
    // Data Bus Control - Purely combinatorial (matches Yellowstone top.v)
    // =========================================================================
    // Yellowstone:
    //   assign data = (rw && !_romoe)  ? romOutput :
    //                 (rw && !_devsel && !addr[0]) ? iwmDataOut :
    //                 8'bZZZZZZZZ;
    //
    // ROM has priority, then IWM. Bus is hi-Z when not being read.
    // NOTE: Yellowstone gated IWM reads on addr[0]==0, which blocked
    // status register reads ($C0CD, Q6=1) needed for sense/ACK.
    // The IWM data_out mux returns valid data for all Q7/Q6 states.

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
        .dbg_buf7       (iwm_dbg_buf7),
        .dbg_latch_sync (iwm_dbg_latch_sync)
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
