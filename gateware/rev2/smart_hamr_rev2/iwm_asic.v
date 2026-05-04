`timescale 1ns / 1ps
// =============================================================================
// iwm_asic.v — ASIC-faithful Integrated Woz Machine emulation
// =============================================================================
// Targets Apple 344-0041 mask. Spec: IWM Spec Rev 19 (Jan 2 1985); patent
// US 4,326,256 (Wozniak). Section refs in the form "spec pN" / "patent col N".
//
// v1 scope — full register interface + write serializer + read shifter +
// /ENBL gating + x7 data-ready flag. Goal: boots ProDOS from FujiNet
// (matches PR#4 milestone of the prior smart_hamr design).
//
// Clocking model (single FCLK domain):
//   - State register latches use `negedge nDEVICE_SELECT` for level-
//     sensitive semantics matching the real ASIC. addr is stable at this
//     edge by 6502 setup time (~40 ns tas, spec p7).
//   - All FCLK consumers (read/write engines, /ENBL gating, motor timer)
//     read /DEV-domain regs directly. Per spec, no extra synchronizers —
//     state changes are bus-cycle-aligned and slower than fclk reactions.
//   - Bus access strobe (mode/buffer write trigger) fires on the rising
//     edge of (Q3 OR /DEV) — spec p7. Edge-detected on fclk against a
//     single registered prev sample.
//
// Spec-exact constants only. No empirical tuning. Bit-cell thresholds
// from spec p4/p10. /ENBL hold from spec p8/p14. x7 clear delay 14 fclk
// from patent col 8 lines 60-67.
// =============================================================================

module iwm_asic (
    // ---- Bus interface ----
    input  wire [3:0] addr,            // A3..A0 within $C0Cx
    input  wire       nDEVICE_SELECT,  // active-low slot 4 device select
    input  wire       fclk,            // sig_7M (7.16 MHz)
    input  wire       Q3,              // 2 MHz bus access strobe (spec p7)
    input  wire       R_nW,            // 6502 read=1 / write=0
    input  wire       nRES,            // active-low system reset
    input  wire [7:0] data_in,         // 6502 → IWM (write data)
    output reg  [7:0] data_out,        // IWM → 6502 (read data)

    // ---- Drive interface ----
    output reg        wrdata,          // serial write to drive
    output wire [3:0] phase,           // PH0..PH3 / SmartPort cmd lines
    output wire       _wrreq,          // active-low write request
    output wire       _enbl1,          // active-low drive 1 enable
    output wire       _enbl2,          // active-low drive 2 enable
    input  wire       sense,           // status bit 7 input from drive
    input  wire       rddata           // serial read from drive
);

    // =========================================================================
    // /DEV-domain state registers (spec p7, patent Fig. 2 block 39)
    // =========================================================================
    // addr[3:1] selects which bit; addr[0] is the new value.
    reg [3:0] phase_r;
    reg       motorOn;
    reg       driveSel;
    reg       q6, q7;
    reg       rnw_dev;        // R_nW captured at /DEV falling
    reg       a0_dev;         // addr[0] captured at /DEV falling

    assign phase = phase_r;

    always @(negedge nDEVICE_SELECT or negedge nRES) begin
        if (~nRES) begin
            phase_r  <= 4'b0000;
            motorOn  <= 1'b0;
            driveSel <= 1'b0;
            q6       <= 1'b0;
            q7       <= 1'b0;
            rnw_dev  <= 1'b1;
            a0_dev   <= 1'b0;
        end
        else begin
            case (addr[3:1])
                3'h0: phase_r[0] <= addr[0];
                3'h1: phase_r[1] <= addr[0];
                3'h2: phase_r[2] <= addr[0];
                3'h3: phase_r[3] <= addr[0];
                3'h4: motorOn    <= addr[0];
                3'h5: driveSel   <= addr[0];
                3'h6: q6         <= addr[0];
                3'h7: q7         <= addr[0];
            endcase
            rnw_dev <= R_nW;
            a0_dev  <= addr[0];
        end
    end

    // =========================================================================
    // Bus access strobe — rising edge of (Q3 OR /DEV) (spec p7)
    // =========================================================================
    wire dev_or_q3 = nDEVICE_SELECT | Q3;
    reg  dev_or_q3_d;
    wire bus_strobe_rise = dev_or_q3 & ~dev_or_q3_d;

    always @(posedge fclk or negedge nRES) begin
        if (~nRES)
            dev_or_q3_d <= 1'b1;
        else
            dev_or_q3_d <= dev_or_q3;
    end

    // Continuously sample data_in while /DEV is low. The 6502 holds data
    // valid until tDH after PHI0 falls; this guarantees data_held has the
    // final write value when bus_strobe_rise fires.
    reg [7:0] data_held;
    always @(posedge fclk or negedge nRES) begin
        if (~nRES)
            data_held <= 8'd0;
        else if (~nDEVICE_SELECT)
            data_held <= data_in;
    end

    // =========================================================================
    // Mode register (spec p8)
    // =========================================================================
    reg [7:0] modeReg;

    wire modeLatch    = modeReg[0];   // 1 = latch (SmartPort) framing
    wire modeAsync    = modeReg[1];   // 1 = asynchronous handshake
    wire modeTimerOff = modeReg[2];   // 1 = 1-second timer disabled
    wire modeFast     = modeReg[3];   // 1 = 2 µs bit cells
    wire mode8MHz     = modeReg[4];   // 1 = 8 MHz fclk
    wire modeTest     = modeReg[5];   // 1 = test mode
    wire modeMZreset  = modeReg[6];   // 1 = MZ reset

    wire _unused_mode = &{1'b0, modeAsync, modeTest, modeMZreset};

    // =========================================================================
    // Bit-cell timing constants (spec p4 / p10)
    // =========================================================================
    // FCLK-cycle counts per mode. Slow values are doubled because the
    // spec counter runs at FCLK/2 in slow mode, while ours always runs
    // at FCLK.
    reg [5:0] oneThreshold;
    reg [5:0] zeroThreshold;
    reg [5:0] writeBitCell;

    always @(*) begin
        case ({modeFast, mode8MHz})
            2'b00: begin oneThreshold = 6'd14; zeroThreshold = 6'd42; writeBitCell = 6'd28; end
            2'b01: begin oneThreshold = 6'd16; zeroThreshold = 6'd48; writeBitCell = 6'd32; end
            2'b10: begin oneThreshold = 6'd7;  zeroThreshold = 6'd21; writeBitCell = 6'd14; end
            2'b11: begin oneThreshold = 6'd8;  zeroThreshold = 6'd24; writeBitCell = 6'd16; end
        endcase
    end

    // =========================================================================
    // 1-second motor-off timer (spec p8 / p14)
    // =========================================================================
    // motorOn 1→0 starts hold of 2^23 + 100 FCLK cycles when ~modeTimerOff.
    reg [23:0] timerCount;
    reg        timerActive;
    reg        timerDriveSel;
    reg        motorOn_d;

    always @(posedge fclk or negedge nRES) begin
        if (~nRES) begin
            timerCount    <= 24'd0;
            timerActive   <= 1'b0;
            timerDriveSel <= 1'b0;
            motorOn_d     <= 1'b0;
        end
        else begin
            motorOn_d <= motorOn;
            if (motorOn_d & ~motorOn & ~modeTimerOff) begin
                timerActive   <= 1'b1;
                timerDriveSel <= driveSel;
                timerCount    <= 24'd0;
            end
            if (timerActive) begin
                if (timerCount == 24'd8388708)   // 2^23 + 100, spec p14
                    timerActive <= 1'b0;
                else
                    timerCount <= timerCount + 24'd1;
            end
        end
    end

    // =========================================================================
    // Drive enable outputs (spec p7 / p12)
    // =========================================================================
    wire enbl1_motor = motorOn  & ~driveSel;
    wire enbl2_motor = motorOn  &  driveSel;
    wire enbl1_timer = timerActive & ~timerDriveSel;
    wire enbl2_timer = timerActive &  timerDriveSel;

    assign _enbl1 = ~(enbl1_motor | enbl1_timer);
    assign _enbl2 = ~(enbl2_motor | enbl2_timer);

    wire enableActive = ~_enbl1 | ~_enbl2;

    // =========================================================================
    // Read shifter (spec p4 / p9, patent Fig. 3)
    // =========================================================================
    // 1-FF rddata sample in fclk; falling-edge detect (drive idle = HIGH,
    // pulse = LOW). bitTimer counts fclk between events.
    //
    //   edge AND bitTimer ≥ oneThreshold  → shift in 1, reset bitTimer
    //   no edge AND bitTimer ≥ zeroThreshold → shift in 0, reset bitTimer
    //
    // Latch mode (modeLatch=1): byte boundary every 8 bit events after
    // initial sync. Sync is the first $FF (shifter == 8'hFF) — once in
    // sync, count 8 events per byte and load buffer.
    //
    // GCR mode (modeLatch=0): byte boundary on shifter[7]==1 (MSB-based
    // framing). Buffer loaded, x7 set.
    reg [1:0] rddataSync;
    always @(posedge fclk or negedge nRES) begin
        if (~nRES) rddataSync <= 2'b11;
        else       rddataSync <= {rddataSync[0], rddata};
    end
    wire rddata_fall = rddataSync[1] & ~rddataSync[0];

    reg [7:0] shifter;
    reg [5:0] bitTimer;
    reg [2:0] bitCounter;
    reg       latchSynced;

    // =========================================================================
    // Buffer + x7 (patent col 8 lines 60-67)
    // =========================================================================
    // Buffer holds the most recently latched read byte. x7 is the data-
    // ready flop; it overrides D7 of the data register on the bus. x7
    // sets when shifter loads buffer; clears 14 fclk after a valid bus
    // read (/DEV low, x7=1). Spec/patent — no other delays.
    reg [7:0] buffer;
    reg       x7;
    reg [3:0] clrX7Timer;

    // =========================================================================
    // Write serializer (spec p2)
    // =========================================================================
    // writeShifter MSB-first; toggle wrdata at bit-cell midpoint when MSB=1.
    // _underrun fires when shifter empties at byte boundary with no buffered
    // byte to follow.
    reg [7:0] writeShifter;
    reg [5:0] writeBitTimer;
    reg [2:0] writeBitCounter;
    reg       writeBufferEmpty;
    reg       _underrun;
    reg       q7_d;

    // Write request — active when q7=1, _underrun not fired, drive enabled.
    assign _wrreq = ~(q7 & _underrun & enableActive);

    // =========================================================================
    // Main fclk process (read shifter, write serializer, x7, mode/buffer load)
    // =========================================================================
    always @(posedge fclk or negedge nRES) begin
        if (~nRES) begin
            modeReg          <= 8'h07;          // latch=1, async=1, timer-off=1
            shifter          <= 8'd0;
            bitTimer         <= 6'd0;
            bitCounter       <= 3'd0;
            latchSynced      <= 1'b0;
            buffer           <= 8'd0;
            x7               <= 1'b0;
            clrX7Timer       <= 4'd0;
            writeShifter     <= 8'd0;
            writeBitTimer    <= 6'd0;
            writeBitCounter  <= 3'd7;
            writeBufferEmpty <= 1'b1;
            _underrun        <= 1'b1;
            wrdata           <= 1'b1;
            q7_d             <= 1'b0;
        end
        else begin
            q7_d <= q7;

            // ===== x7 clear timer (patent col 8) =====
            if (clrX7Timer != 4'd0) begin
                if (clrX7Timer == 4'd14) begin
                    x7         <= 1'b0;
                    clrX7Timer <= 4'd0;
                end
                else
                    clrX7Timer <= clrX7Timer + 4'd1;
            end
            else if (~nDEVICE_SELECT && x7) begin
                // /DEV asserted while x7=1 → 6502 is reading the data
                // register with x7 latched. Start the 14-fclk countdown.
                clrX7Timer <= 4'd1;
            end

            // ===== Read shifter (q7 = 0) =====
            if (~q7) begin
                if (rddata_fall) begin
                    if (bitTimer >= oneThreshold) begin
                        shifter <= {shifter[6:0], 1'b1};
                        if (modeLatch) begin
                            if (latchSynced) begin
                                if (bitCounter == 3'd7) begin
                                    buffer     <= {shifter[6:0], 1'b1};
                                    x7         <= 1'b1;
                                    shifter    <= 8'd0;
                                    bitCounter <= 3'd0;
                                end
                                else
                                    bitCounter <= bitCounter + 3'd1;
                            end
                            else if ({shifter[6:0], 1'b1} == 8'hFF) begin
                                // First $FF — enter latch-synced framing.
                                buffer      <= 8'hFF;
                                x7          <= 1'b1;
                                shifter     <= 8'd0;
                                bitCounter  <= 3'd0;
                                latchSynced <= 1'b1;
                            end
                        end
                        else if (shifter[6] == 1'b1) begin
                            // GCR: MSB=1 latch (after this shift the new MSB).
                            buffer  <= {shifter[6:0], 1'b1};
                            x7      <= 1'b1;
                            shifter <= 8'd0;
                        end
                        bitTimer <= 6'd0;
                    end
                    // bitTimer < oneThreshold → glitch, ignore (do not reset).
                end
                else if (bitTimer >= zeroThreshold) begin
                    shifter <= {shifter[6:0], 1'b0};
                    if (modeLatch && latchSynced) begin
                        if (bitCounter == 3'd7) begin
                            buffer     <= {shifter[6:0], 1'b0};
                            x7         <= 1'b1;
                            shifter    <= 8'd0;
                            bitCounter <= 3'd0;
                        end
                        else
                            bitCounter <= bitCounter + 3'd1;
                    end
                    bitTimer <= oneThreshold;
                end
                else begin
                    // No edge yet, timer counting. GCR MSB=1 latch path.
                    if (~modeLatch && shifter[7] == 1'b1) begin
                        buffer  <= shifter;
                        x7      <= 1'b1;
                        shifter <= 8'd0;
                    end
                    bitTimer <= bitTimer + 6'd1;
                end
            end
            else begin
                // q7 = 1 (write mode) — reset read framing for next read.
                shifter     <= 8'd0;
                bitTimer    <= 6'd0;
                bitCounter  <= 3'd0;
                latchSynced <= 1'b0;
            end

            // ===== Write serializer (q7 = 1) =====
            if (q7) begin
                if (~q7_d) begin
                    // q7 rising — initialize serializer state for new session.
                    writeBitTimer    <= 6'd0;
                    writeBitCounter  <= 3'd7;
                    writeShifter     <= 8'd0;
                    _underrun        <= 1'b1;
                end
                else if (writeBitTimer == writeBitCell - 6'd1) begin
                    writeBitTimer <= 6'd0;
                    if (writeBitCounter == 3'd7) begin
                        // Byte boundary — load next byte or underrun.
                        writeBitCounter <= 3'd0;
                        if (~writeBufferEmpty) begin
                            writeShifter     <= buffer;
                            writeBufferEmpty <= 1'b1;
                        end
                        else begin
                            _underrun <= 1'b0;
                        end
                    end
                    else begin
                        writeBitCounter <= writeBitCounter + 3'd1;
                        writeShifter    <= {writeShifter[6:0], 1'b0};
                    end
                end
                else
                    writeBitTimer <= writeBitTimer + 6'd1;

                // Bit-cell midpoint toggle on MSB=1 (spec p2).
                if (~_underrun)
                    wrdata <= 1'b1;
                else if (writeBitTimer == (writeBitCell >> 1) && writeShifter[7] == 1'b1)
                    wrdata <= ~wrdata;
            end
            else begin
                // q7 = 0 → idle.
                writeBitTimer    <= 6'd0;
                writeBitCounter  <= 3'd7;
                wrdata           <= 1'b1;
                writeBufferEmpty <= 1'b1;
                _underrun        <= 1'b1;
            end

            // ===== Mode/buffer write — bus_strobe_rise + write to $C0CF =====
            // Single `buffer` reg serves both read latch (q7=0 path) and
            // write source (q7=1 path) — modes are mutually exclusive so
            // there's no conflict, and this matches the real ASIC's
            // single-buffer design.
            if (bus_strobe_rise && ~rnw_dev && q7 && q6 && a0_dev) begin
                if (motorOn) begin
                    // Buffer load — guard against overwrite during a
                    // pending byte (writeBufferEmpty=0 means buffer still
                    // holds data the serializer hasn't picked up yet).
                    if (writeBufferEmpty) begin
                        buffer           <= data_held;
                        writeBufferEmpty <= 1'b0;
                    end
                end
                else begin
                    modeReg <= data_held;
                end
            end
        end
    end

    // =========================================================================
    // Read register mux (spec p7 / p9)
    // =========================================================================
    always @(*) begin
        case ({q7, q6})
            2'b00:   data_out = motorOn ? {x7, buffer[6:0]} : 8'hFF;
            2'b01:   data_out = {sense, 1'b0, enableActive, modeReg[4:0]};
            2'b10:   data_out = {writeBufferEmpty, _underrun, 6'b000000};
            2'b11:   data_out = 8'hFF;
        endcase
    end

endmodule
