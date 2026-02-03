// =============================================================================
// Decimation + Pack Module for Byte Hamr Logic Analyzer
// =============================================================================
//
// Converts raw 1-bit sample data into Apple II HIRES-compatible 7-bit bytes.
//
// Algorithm:
//   1. On sample_valid, latch sample and load stretch counter
//   2. Each clock while stretch_cnt > 0:
//      - Accumulate bit at current position (LSB-first)
//      - When 7 bits accumulated, emit byte and reset
//   3. Output bytes continuously as samples flow through
//
// Apple II HIRES format:
//   - Bits 0-6: Pixel data (7 pixels per byte)
//   - Bit 7: Palette select (unused, always 0)
//   - Bit 0 = leftmost pixel on screen
//
// =============================================================================

module decimate_pack (
    input  wire       clk,
    input  wire       rst,
    input  wire       sample_in,
    input  wire       sample_valid,
    input  wire       flush,           // Emit partial byte when all samples processed
    input  wire [7:0] stretch_factor,

    output reg  [6:0] byte_out,
    output reg        byte_valid,
    output wire       ready            // High when ready to accept next sample
);

    // =========================================================================
    // Internal State
    // =========================================================================

    reg [2:0] bit_pos;        // Current bit position (0-6)
    reg [6:0] accum;          // Byte accumulator
    reg [7:0] stretch_cnt;    // Pixels remaining for current sample
    reg       sample_latched; // Latched sample value
    reg       active;         // Processing pixels for a sample

    // Ready signal: can accept new sample when not actively processing,
    // or on the last cycle of processing (stretch_cnt will be 0 next cycle)
    assign ready = !active || (stretch_cnt == 8'd1);

    // =========================================================================
    // Initialization
    // =========================================================================

    initial begin
        bit_pos        = 3'd0;
        accum          = 7'd0;
        stretch_cnt    = 8'd0;
        sample_latched = 1'b0;
        active         = 1'b0;
        byte_out       = 7'd0;
        byte_valid     = 1'b0;
    end

    // =========================================================================
    // Decimation State Machine
    // =========================================================================

    always @(posedge clk) begin
        if (rst) begin
            // Reset all state
            bit_pos        <= 3'd0;
            accum          <= 7'd0;
            stretch_cnt    <= 8'd0;
            sample_latched <= 1'b0;
            active         <= 1'b0;
            byte_out       <= 7'd0;
            byte_valid     <= 1'b0;
        end else begin
            // Default: clear byte_valid (one-cycle pulse)
            byte_valid <= 1'b0;

            // Handle new sample arrival
            if (sample_valid) begin
                sample_latched <= sample_in;
                stretch_cnt    <= stretch_factor;
                active         <= 1'b1;
            end

            // Process pixels while active
            // Skip processing if new sample is arriving (let the sample_valid block handle it)
            if (active && stretch_cnt > 0 && !sample_valid) begin
                // Accumulate current sample bit at bit_pos (LSB-first)
                if (sample_latched) begin
                    accum <= accum | (7'd1 << bit_pos);
                end

                // Advance to next bit position
                if (bit_pos == 3'd6) begin
                    // Byte complete - emit it
                    if (sample_latched) begin
                        byte_out <= accum | (7'd1 << bit_pos);
                    end else begin
                        byte_out <= accum;
                    end
                    byte_valid <= 1'b1;
                    accum      <= 7'd0;
                    bit_pos    <= 3'd0;
                end else begin
                    bit_pos <= bit_pos + 1'b1;
                end

                // Decrement stretch counter
                stretch_cnt <= stretch_cnt - 1'b1;

                // Check if this was the last pixel of the sample
                if (stretch_cnt == 8'd1) begin
                    active <= 1'b0;
                end
            end

            // Flush: emit partial byte if there are accumulated bits
            if (flush && !active && bit_pos > 0) begin
                byte_out   <= accum;
                byte_valid <= 1'b1;
                accum      <= 7'd0;
                bit_pos    <= 3'd0;
            end
        end
    end

endmodule
