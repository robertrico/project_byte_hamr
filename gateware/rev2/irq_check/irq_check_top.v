// =============================================================================
// Byte Hamr Rev 2 - IRQ/NMI Contention Test
// =============================================================================
//
// Purpose: find out whether U14 (SN74LVC8T245, OE/DIR hard-wired, FPGA ->
// Apple) breaks interrupts when another card is fighting it. With OE and DIR
// tied, an "idle" Byte Hamr does not release ~IRQ/~NMI. Its FPGA pin goes
// Hi-Z, the internal pull-up makes U14's B input high, and U14 actively drives
// the 5V bus line HIGH. A second card then has to pull the line low against a
// push-pull driver.
//
// Load this SAME bitstream on both cards. One card (the "tester") fires
// interrupts on command; the other sits idle and acts as the opponent. The
// 6502 itself is the sensor: the card watches the address bus for the vector
// fetch ($FFFE for IRQ/BRK, $FFFA for NMI) and counts how many interrupts the
// CPU actually took and how long each one took.
//
// Register map (DEVICE SELECT, $C080 + slot*$10). Slot 4 = $C0C0.
//   Write +0: command
//       $00  stop / idle, clear all counters
//       $01  IRQ test: 100 shots, ~2 ms apart
//       $02  NMI test: 100 shots, ~2 ms apart
//       $03  scope mode: ~1 us IRQ pulse every ~10 ms, until $00
//   Read  +0: status  bit7 = done, bit6 = busy, bits1:0 = mode
//   Read  +1: interrupts taken (vector fetch seen while we asserted)
//   Read  +2: missed (no vector fetch within 64 cycles)
//   Read  +3: min latency, PHI0 cycles from assert to vector fetch
//   Read  +4: max latency
//   Read  +5: spurious vector fetches (vector fetch we did NOT cause)
//   Read  +6: shots fired
//   Read  +7: $A5 (ID, proves the read path works)
//   Write +8: slot ROM page select (bit 0). Cleared by Ctrl-Reset.
//
// Slot ROM ($Cn00, 512 bytes as 2 pages, irqrom.S -> irq_rom.mem):
// PR#n runs the IRQ then NMI test on this card and prints the counters
// with PASS/FAIL. Works in any slot; finds the other irq_check card by
// ROM signature and clears it first. No typing needed at the bench.
//
// The idle card's +5 counter should match the tester's +1, because it sees
// every vector fetch on the bus. That cross-checks the bus monitor on both.
//
// Monitor program (slot 4; change C0C0 to your tester's slot):
// Manual alternative (monitor):
//   300:A9 01 8D C0 C0 58 AD C0 C0 10 FB 78 60   (IRQ test, CLI, wait, SEI)
//   310:A9 02 8D C0 C0 AD C0 C0 10 FB 60         (NMI test, wait)
//   320:A5 45 40                                 (IRQ handler: LDA $45, RTI)
//   3FB:40                                       (NMI handler: RTI)
//   3FE:20 03                                    (IRQ vector -> $0320)
//   300G  then  C0C0.C0C7      (IRQ)
//   310G  then  C0C0.C0C7      (NMI)
// Expected clean result: +1 = 64 (100 decimal), +2 = 00.
//
// Status pins (GPIO header, for scope triggering):
//   GPIO_1: IRQ asserted by this card
//   GPIO_2: NMI asserted by this card
//   GPIO_3: vector fetch seen (one pulse per fetch)
//   GPIO_4: test done
// =============================================================================

module irq_check_top (
    input  wire        CLK_100MHz,

    // Keep SDRAM idle
    output wire        SDRAM_CKE,
    output wire        SDRAM_nCS,

    // Apple II bus
    input  wire        A0, A1, A2, A3, A4, A5, A6, A7,
    input  wire        A8, A9, A10, A11, A12, A13, A14, A15,
    inout  wire        D0, D1, D2, D3, D4, D5, D6, D7,
    input  wire        PHI0,
    input  wire        R_nW,
    input  wire        nDEVICE_SELECT,
    input  wire        nI_O_SELECT,
    input  wire        nRES_READ,
    input  wire        DMA_OUT,
    input  wire        INT_OUT,

    output wire        nIRQ,
    output wire        nNMI,
    output wire        nINH,
    output wire        nDMA,
    output wire        nRES,            // OPENDRAIN in LPF
    output wire        DMA_IN,
    output wire        INT_IN,
    output wire        DATA_OE,         // U12 OE, active low

    output wire        GPIO_1, GPIO_2, GPIO_3, GPIO_4
);

    wire clk = CLK_100MHz;

    assign SDRAM_CKE = 1'b0;
    assign SDRAM_nCS = 1'b1;

    wire [15:0] addr_raw = {A15, A14, A13, A12, A11, A10, A9, A8,
                            A7, A6, A5, A4, A3, A2, A1, A0};
    wire [7:0]  data_raw = {D7, D6, D5, D4, D3, D2, D1, D0};

    // =========================================================================
    // Apple bus cycle tracking
    // PHI0 rise is synchronized, then the bus is sampled at fixed offsets.
    // Address/R_nW/select: 200 ns into PHI0 high (6502 address long valid).
    // Write data:          380 ns into PHI0 high (6502 data setup ~175 ns).
    // PHI0 high is ~490 ns on a IIe, so both land inside the window.
    // =========================================================================
    reg [2:0] phi0_s = 3'b000;
    always @(posedge clk) phi0_s <= {phi0_s[1:0], PHI0};
    wire phi0_rise = phi0_s[1] & ~phi0_s[2];

    localparam [5:0] T_ADDR = 6'd20;    // 200 ns @ 100 MHz
    localparam [5:0] T_DATA = 6'd38;    // 380 ns

    reg [5:0]  phase     = 6'd63;       // saturates at 63 between cycles
    reg [15:0] bus_addr  = 16'd0;
    reg        bus_rnw   = 1'b1;
    reg        bus_sel   = 1'b0;
    reg        cyc       = 1'b0;        // 1-clk strobe: new bus cycle sampled
    reg        wr_strobe = 1'b0;        // 1-clk strobe: write to our regs
    reg [7:0]  wr_data   = 8'd0;
    reg [3:0]  wr_reg    = 4'd0;

    always @(posedge clk) begin
        cyc       <= 1'b0;
        wr_strobe <= 1'b0;

        if (phi0_rise)
            phase <= 6'd0;
        else if (phase != 6'd63)
            phase <= phase + 1'b1;

        if (phase == T_ADDR) begin
            bus_addr <= addr_raw;
            bus_rnw  <= R_nW;
            bus_sel  <= ~nDEVICE_SELECT;
            cyc      <= 1'b1;
        end

        if (phase == T_DATA && bus_sel && !bus_rnw) begin
            wr_data   <= data_raw;
            wr_reg    <= bus_addr[3:0];
            wr_strobe <= 1'b1;
        end
    end

    wire vec_irq = cyc & bus_rnw & (bus_addr == 16'hFFFE);
    wire vec_nmi = cyc & bus_rnw & (bus_addr == 16'hFFFA);

    // =========================================================================
    // Test engine (advances once per Apple bus cycle)
    // =========================================================================
    localparam [7:0]  SHOTS        = 8'd100;
    localparam [7:0]  TIMEOUT      = 8'd64;
    localparam [15:0] GAP          = 16'd2048;   // ~2 ms between shots
    localparam [15:0] SCOPE_PERIOD = 16'd10230;  // ~10 ms

    localparam [1:0] M_IDLE = 2'd0, M_IRQ = 2'd1, M_NMI = 2'd2, M_SCOPE = 2'd3;
    localparam [1:0] S_IDLE = 2'd0, S_GAP = 2'd1, S_WAIT = 2'd2, S_SCOPE = 2'd3;

    reg [1:0]  mode       = M_IDLE;
    reg [1:0]  state      = S_IDLE;
    reg        busy       = 1'b0;
    reg        done       = 1'b0;
    reg        assert_irq = 1'b0;
    reg        assert_nmi = 1'b0;

    reg [7:0]  shots      = 8'd0;
    reg [7:0]  taken      = 8'd0;
    reg [7:0]  missed     = 8'd0;
    reg [7:0]  lat        = 8'd0;
    reg [7:0]  lat_min    = 8'hFF;
    reg [7:0]  lat_max    = 8'd0;
    reg [7:0]  spurious   = 8'd0;
    reg [15:0] timer      = 16'd0;

    wire asserting = assert_irq | assert_nmi;
    wire hit = (assert_irq & vec_irq) | (assert_nmi & vec_nmi);

    always @(posedge clk) begin
        if (wr_strobe && wr_reg == 4'd0) begin
            // Any command clears counters and drops both lines
            assert_irq <= 1'b0;
            assert_nmi <= 1'b0;
            shots      <= 8'd0;
            taken      <= 8'd0;
            missed     <= 8'd0;
            lat_min    <= 8'hFF;
            lat_max    <= 8'd0;
            spurious   <= 8'd0;
            done       <= 1'b0;
            timer      <= GAP;
            case (wr_data[1:0])
                2'd1:    begin mode <= M_IRQ;   state <= S_GAP;   busy <= 1'b1; end
                2'd2:    begin mode <= M_NMI;   state <= S_GAP;   busy <= 1'b1; end
                2'd3:    begin mode <= M_SCOPE; state <= S_SCOPE; busy <= 1'b1;
                               timer <= SCOPE_PERIOD; end
                default: begin mode <= M_IDLE;  state <= S_IDLE;  busy <= 1'b0; end
            endcase
        end else if (cyc) begin
            // Vector fetches we didn't cause (other cards, or our own after
            // timeout). On the idle card this counts the tester's shots.
            if ((vec_irq | vec_nmi) && !hit && spurious != 8'hFF)
                spurious <= spurious + 1'b1;

            case (state)
                S_GAP: begin
                    if (timer == 16'd0) begin
                        assert_irq <= (mode == M_IRQ);
                        assert_nmi <= (mode == M_NMI);
                        lat        <= 8'd0;
                        shots      <= shots + 1'b1;
                        state      <= S_WAIT;
                    end else
                        timer <= timer - 1'b1;
                end

                S_WAIT: begin
                    if (hit || lat == TIMEOUT) begin
                        assert_irq <= 1'b0;
                        assert_nmi <= 1'b0;
                        if (hit) begin
                            taken <= taken + 1'b1;
                            if (lat < lat_min) lat_min <= lat;
                            if (lat > lat_max) lat_max <= lat;
                        end else
                            missed <= missed + 1'b1;

                        if (shots == SHOTS) begin
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            state <= S_IDLE;
                        end else begin
                            timer <= GAP;
                            state <= S_GAP;
                        end
                    end else
                        lat <= lat + 1'b1;
                end

                S_SCOPE: begin
                    // One-cycle (~1 us) IRQ pulse per period
                    assert_irq <= (timer == 16'd0);
                    timer <= (timer == 16'd0) ? SCOPE_PERIOD : timer - 1'b1;
                end

                default: ;
            endcase
        end
    end

    // =========================================================================
    // Slot ROM: 2 x 256-byte pages at $Cn00, page picked by +8.
    // Ctrl-Reset (nRES_READ low) returns to page 0 so PR#n always finds
    // the entry stub.
    // =========================================================================
    reg [1:0] nres_s = 2'b11;
    always @(posedge clk) nres_s <= {nres_s[0], nRES_READ};

    reg rom_page = 1'b0;
    always @(posedge clk)
        if (!nres_s[1])
            rom_page <= 1'b0;
        else if (wr_strobe && wr_reg == 4'd8)
            rom_page <= wr_data[0];

    reg [7:0] slot_rom [0:511];
    initial $readmemh("irq_rom.mem", slot_rom);
    wire [7:0] rom_data = slot_rom[{rom_page, addr_raw[7:0]}];

    // =========================================================================
    // Register read path (U12 data bus)
    // =========================================================================
    reg [7:0] reg_data;
    always @(*) begin
        case (addr_raw[3:0])
            4'd0:    reg_data = {done, busy, 4'b0000, mode};
            4'd1:    reg_data = taken;
            4'd2:    reg_data = missed;
            4'd3:    reg_data = lat_min;
            4'd4:    reg_data = lat_max;
            4'd5:    reg_data = spurious;
            4'd6:    reg_data = shots;
            4'd7:    reg_data = 8'hA5;
            default: reg_data = 8'hFF;
        endcase
    end
    wire [7:0] read_data = nI_O_SELECT ? reg_data : rom_data;

    // U12 DIR follows R/W in hardware; only drive the B side on reads.
    wire data_drive = R_nW;
    assign {D7, D6, D5, D4, D3, D2, D1, D0} = data_drive ? read_data : 8'bZ;
    // U12 enabled only for our DEVICE SELECT or I/O SELECT (never STROBE:
    // no $C800 ROM here, and STROBE is bus-wide)
    assign DATA_OE = nDEVICE_SELECT & nI_O_SELECT;

    // =========================================================================
    // Bus control lines
    // Released lines are Hi-Z here, which (through U14) still drives HIGH on
    // the Apple side. That is exactly the condition under test.
    // =========================================================================
    assign nIRQ   = assert_irq ? 1'b0 : 1'bZ;
    assign nNMI   = assert_nmi ? 1'b0 : 1'bZ;
    assign nINH   = 1'bZ;
    assign nDMA   = 1'bZ;
    assign nRES   = 1'b1;               // OPENDRAIN: released
    assign DMA_IN = DMA_OUT;
    assign INT_IN = INT_OUT;

    // =========================================================================
    // Scope/debug pins
    // =========================================================================
    reg vec_seen = 1'b0;
    always @(posedge clk) vec_seen <= vec_irq | vec_nmi;

    assign GPIO_1 = assert_irq;
    assign GPIO_2 = assert_nmi;
    assign GPIO_3 = vec_seen;
    assign GPIO_4 = done;

endmodule
