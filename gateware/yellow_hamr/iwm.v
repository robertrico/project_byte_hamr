`timescale 1ns / 1ps
// =============================================================================
// IWM (Integrated Waveform Module) Emulation for Yellow Hamr
// =============================================================================
// Adapted from Steve Chamberlin's Yellowstone project
// Original target: Lattice MachXO2-1200HC
// Ported to: Lattice ECP5-85F (Byte Hamr)
//
// The IWM is the disk controller chip used in the Apple IIc, IIgs, and Liron
// card. It handles serial communication with SmartPort-compatible disk drives.
//
// Register Access:
//   Addresses $C0C0-$C0CF (via nDEVICE_SELECT)
//   A3-A1 selects one of 8 state register bits
//   A0 is the new value for that bit (latched on falling edge of nDEVICE_SELECT)
//   A0=0 during a read enables data output
//
// State Register Bits:
//   0: phase[0]    - Stepper phase 0
//   1: phase[1]    - Stepper phase 1
//   2: phase[2]    - Stepper phase 2
//   3: phase[3]    - Stepper phase 3
//   4: motorOn     - Drive motor enable
//   5: driveSelect - Drive select (0=D1, 1=D2)
//   6: Q6          - Mode bit
//   7: Q7          - Mode bit
//
// Read Registers (selected by Q7:Q6):
//   00: Data register (serial input from drive)
//   01: Status register (sense, motorOn, mode)
//   10: Handshake register (buffer empty, underrun)
//   11: (Write mode - no read)
//
// Mode Register: Hardcoded to SCMHL = 00111 for Apple II compatibility
//   S=0: 7 MHz clock
//   C=0: 4 us/bit (slow mode)
//   M=0: 1 sec motor-off delay
//   H=1: Synchronous handshake
//   L=1: Latch mode
// =============================================================================

module iwm (
    // Bus interface
    input wire [3:0]  addr,         // A3-A0: bit select and value
    input wire        nDEVICE_SELECT, // Device enable (active low)
    input wire        fclk,         // 7 MHz clock for serial I/O
    input wire        Q3,           // 2 MHz timing signal
    input wire        nRES,         // System reset (active low)
    input wire [7:0]  data_in,      // Data from Apple II
    output reg [7:0]  data_out,     // Data to Apple II

    // Drive interface
    output reg        wrdata,       // Serial write data (transitions = 1 bits)
    output reg [3:0]  phase,        // Stepper phases / SmartPort command
    output wire       _wrreq,       // Write request (active low)
    output wire       _enbl1,       // Drive 1 enable (active low)
    output wire       _enbl2,       // Drive 2 enable (active low)
    input wire        sense,        // Write protect / ACK from drive
    input wire        rddata        // Serial read data (falling edge = 1 bit)
);

    // Internal state registers
    reg        motorOn;
    reg        driveSelect;
    reg        q6, q7;

    // Serial I/O state
    reg [7:0]  shifter;             // Shift register for serial data
    reg [7:0]  buffer;              // Data buffer (read or write)
    reg        _underrun;           // Write underrun flag (active low)
    reg        writeBufferEmpty;    // Write buffer empty flag

    // Timing counters
    reg [5:0]  bitTimer;            // Bit cell timer (max 42)
    reg [2:0]  bitCounter;          // Bit counter (max 7)
    reg [3:0]  clearBufferTimer;    // Buffer clear delay timer (max 14)

    // rddata synchronizer (2-stage for metastability)
    reg [1:0]  rddataSync;

    // Timing reference
    wire q3orDev = Q3 | nDEVICE_SELECT;

    // =========================================================================
    // State Register - Bit-addressable via A3-A1, A0 is data value
    // =========================================================================
    // Latched on falling edge of nDEVICE_SELECT
    always @(negedge nDEVICE_SELECT or negedge nRES) begin
        if (~nRES) begin
            phase       <= 4'b0000;
            motorOn     <= 1'b0;
            driveSelect <= 1'b0;
            q6          <= 1'b0;
            q7          <= 1'b0;
        end
        else begin
            case (addr[3:1])
                3'h0: phase[0]    <= addr[0];  // $C0C0/C0C1
                3'h1: phase[1]    <= addr[0];  // $C0C2/C0C3
                3'h2: phase[2]    <= addr[0];  // $C0C4/C0C5
                3'h3: phase[3]    <= addr[0];  // $C0C6/C0C7
                3'h4: motorOn     <= addr[0];  // $C0C8/C0C9
                3'h5: driveSelect <= addr[0];  // $C0CA/C0CB
                3'h6: q6          <= addr[0];  // $C0CC/C0CD
                3'h7: q7          <= addr[0];  // $C0CE/C0CF
            endcase
        end
    end

    // =========================================================================
    // Drive Enable Outputs
    // =========================================================================
    // Only one enable may be low at a time
    assign _enbl1 = ~(motorOn & ~driveSelect);  // Drive 1 when motor on, drive 0 selected
    assign _enbl2 = ~(motorOn & driveSelect);   // Drive 2 when motor on, drive 1 selected

    // Write request - active when Q7=1, no underrun, and a drive is enabled
    assign _wrreq = ~(q7 & _underrun & (~_enbl1 | ~_enbl2));

    // =========================================================================
    // Read Registers - Selected by Q7:Q6
    // =========================================================================
    // Register reads occur when A0=0 and nDEVICE_SELECT is low
    always @(*) begin
        case ({q7, q6})
            2'b00:   // Data register (from disk drive)
                     // MSB=1 when data is valid
                data_out = buffer;

            2'b01:   // Status register
                     // {sense, 0, motorOn, mode[4:0]}
                     // mode = 00111 (hardcoded for Apple II)
                data_out = {sense, 1'b0, motorOn, 5'b00111};

            default: // 2'b10: Handshake register
                     // {bufferEmpty, ~underrun, 6'b000000}
                data_out = {writeBufferEmpty, _underrun, 6'b000000};
        endcase
    end

    // =========================================================================
    // rddata Synchronizer
    // =========================================================================
    always @(posedge fclk) begin
        rddataSync <= {rddataSync[0], rddata};
    end

    // =========================================================================
    // Serial I/O and Register Writes
    // =========================================================================
    // Assumes mode register SCMHL = 00111 (Apple II compatible)
    //
    // Timing constants (for 7 MHz clock, 4 us bit cells):
    //   Bit cell:      28 FCLK cycles = 4 us
    //   Half bit cell: 14 FCLK cycles = 2 us
    //   1.5 bit cells: 42 FCLK cycles = 6 us
    //   Buffer clear:  14 FCLK cycles = 2 us
    //
    always @(posedge fclk or negedge nRES) begin
        if (~nRES) begin
            _underrun        <= 1'b1;
            writeBufferEmpty <= 1'b1;
            bitCounter       <= 3'd0;
            buffer           <= 8'd0;
            shifter          <= 8'd0;
            clearBufferTimer <= 4'd0;
            bitTimer         <= 6'd0;
            wrdata           <= 1'b0;
        end
        else begin
            // -----------------------------------------------------------------
            // READ FROM DISK (Q7=0, Q6=0)
            // -----------------------------------------------------------------
            if (q7 == 1'b0 && q6 == 1'b0) begin

                // Buffer clear timer
                // After a valid read from the data buffer (with MSB=1),
                // clear the MSB after ~2 us to indicate data consumed
                if (clearBufferTimer == 4'd0) begin
                    // Start timer if reading valid data
                    if (nDEVICE_SELECT == 1'b0 && addr[0] == 1'b0 && buffer[7] == 1'b1) begin
                        clearBufferTimer <= 4'd1;
                    end
                end
                else begin
                    // After 14 FCLK periods (~2 us), clear valid flag
                    if (clearBufferTimer == 4'd14) begin
                        buffer[7] <= 1'b0;
                        clearBufferTimer <= 4'd0;
                    end
                    else begin
                        clearBufferTimer <= clearBufferTimer + 4'd1;
                    end
                end

                // Falling edge detection on rddata
                if (rddataSync[1] & ~rddataSync[0]) begin
                    // Don't shift if less than half a bit cell has elapsed
                    // (noise rejection)
                    if (bitTimer >= 6'd14) begin
                        shifter <= {shifter[6:0], 1'b1};  // Shift in a 1 bit
                    end
                    bitTimer <= 6'd0;
                end
                else begin
                    // Check for 1.5 bit cells elapsed (no transition = 0 bit)
                    if (bitTimer >= 6'd42) begin
                        shifter <= {shifter[6:0], 1'b0};  // Shift in a 0 bit
                        bitTimer <= 6'd14;  // Reset to half bit cell
                    end
                    else begin
                        // Check for complete byte (MSB = 1)
                        if (shifter[7] == 1'b1) begin
                            buffer <= shifter;    // Latch the byte
                            shifter <= 8'd0;      // Clear shifter
                        end
                        bitTimer <= bitTimer + 6'd1;
                    end
                end
            end

            // -----------------------------------------------------------------
            // WRITE TO DISK (Q7=1)
            // -----------------------------------------------------------------
            if (q7 == 1'b1) begin

                // Time for a new bit?
                if (bitTimer == 6'd28) begin
                    bitTimer <= 6'd0;

                    // Entire byte done?
                    if (bitCounter == 3'd7) begin
                        bitCounter <= 3'd0;

                        // New byte ready in buffer?
                        if (writeBufferEmpty == 1'b0) begin
                            shifter <= buffer;
                            writeBufferEmpty <= 1'b1;
                        end
                        else begin
                            _underrun <= 1'b0;  // Underrun error
                        end
                    end
                    else begin
                        // More bits remaining, shift next bit
                        bitCounter <= bitCounter + 3'd1;
                        shifter <= {shifter[6:0], 1'b0};  // Left shift
                    end
                end
                else begin
                    bitTimer <= bitTimer + 6'd1;
                end

                // wrdata transition at start of bit cell indicates a 1 bit
                if (bitTimer == 6'd1 && shifter[7] == 1'b1) begin
                    wrdata <= ~wrdata;
                end
            end
            else begin
                _underrun <= 1'b1;  // Clear underrun when Q7=0
            end

            // -----------------------------------------------------------------
            // WRITE REGISTERS (Q7=1, Q6=1, A0=1)
            // -----------------------------------------------------------------
            // When Q3 or nDEVICE_SELECT is low, latch write data
            if (q3orDev == 1'b0 && q7 && q6 && addr[0]) begin
                if (motorOn) begin
                    buffer <= data_in;        // Data for disk write
                    writeBufferEmpty <= 1'b0;
                end
                // Note: Mode register writes ignored (mode is hardcoded)
            end
        end
    end

endmodule
