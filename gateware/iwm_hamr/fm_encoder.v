// ===================================================================
// FM Encoder -- SmartPort Device TX (buffer-driven)
// ===================================================================
// FM-encodes bytes from a buffer for transmission via rddata.
// Reads sequentially from address 0 to (byte_count-1) with zero
// inter-byte gaps, matching PicoPort's PIO autopull behavior.
//
// Architecture (matching PicoPort):
//   1. Caller fills tx_buf[] with complete packet (preamble + data)
//   2. Caller sets byte_count and pulses start
//   3. Encoder reads bytes sequentially from buf_data, drives rddata
//   4. Zero gap between bytes — continuous bit-cell timing
//   5. After last byte, pulses done, returns to idle
//
// FM encoding:
//   - rddata idles HIGH
//   - "1" bit: LOW pulse (PULSE_WIDTH cycles), then HIGH
//   - "0" bit: no edge, HIGH throughout
//   - 28 fclk per bit cell, MSB first
// ===================================================================

`timescale 1ns / 1ps

module fm_encoder (
    input  wire       clk,             // 7 MHz (sig_7M / fclk)
    input  wire       rst_n,
    input  wire       start,           // pulse to begin transmitting
    input  wire [9:0] byte_count,      // number of bytes to transmit
    output reg        rddata = 1'b1,   // FM serial output to IWM
    output reg        busy = 1'b0,
    output reg        done = 1'b0,     // pulse when complete
    // Buffer read interface
    output reg  [9:0] buf_addr = 10'd0, // read address into tx packet buffer
    input  wire [7:0] buf_data          // data from tx packet buffer
);

    // =========================================================================
    // Timing parameters
    // =========================================================================
    localparam BIT_CELL    = 28;       // fclk cycles per bit cell
    localparam PULSE_WIDTH = 4;        // fclk cycles rddata is LOW for a "1"
    localparam PULSE_START = BIT_CELL - PULSE_WIDTH;  // = 24

    // States
    localparam ST_IDLE      = 3'd0;
    localparam ST_ADDR      = 3'd1;    // set address, wait for data
    localparam ST_LOAD      = 3'd2;    // capture data from buffer
    localparam ST_BIT_CELL  = 3'd3;    // run one 28-cycle bit cell
    localparam ST_NEXT_BIT  = 3'd4;    // advance bit index or next byte
    localparam ST_DONE      = 3'd5;

    reg [2:0] state = ST_IDLE;
    reg [7:0] shift_reg = 8'd0;       // current byte being transmitted
    reg [2:0] bit_index = 3'd7;       // 7 down to 0 (MSB first)
    reg [4:0] cell_counter = 5'd0;    // 0..(BIT_CELL-1) within a bit cell
    reg [9:0] bytes_remaining = 10'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= ST_IDLE;
            rddata          <= 1'b1;
            busy            <= 1'b0;
            done            <= 1'b0;
            shift_reg       <= 8'd0;
            bit_index       <= 3'd7;
            cell_counter    <= 5'd0;
            buf_addr        <= 10'd0;
            bytes_remaining <= 10'd0;
        end else begin
            // Default: clear single-cycle pulses
            done <= 1'b0;

            case (state)
                // -------------------------------------------------
                ST_IDLE: begin
                    rddata <= 1'b1;
                    busy   <= 1'b0;
                    if (start) begin
                        buf_addr        <= 10'd0;
                        bytes_remaining <= byte_count;
                        busy            <= 1'b1;
                        state           <= ST_ADDR;
                    end
                end

                // -------------------------------------------------
                // ADDR: set buffer address, wait one cycle for
                // registered read to produce data
                // -------------------------------------------------
                ST_ADDR: begin
                    if (bytes_remaining == 10'd0) begin
                        state <= ST_DONE;
                    end else begin
                        // buf_addr is already set; data appears next cycle
                        state <= ST_LOAD;
                    end
                end

                // -------------------------------------------------
                // LOAD: capture data from buffer (now valid after
                // the 1-cycle registered read latency)
                // -------------------------------------------------
                ST_LOAD: begin
                    shift_reg       <= buf_data;
                    bit_index       <= 3'd7;
                    cell_counter    <= 5'd0;
                    // Don't increment buf_addr here — pre-fetch in
                    // BIT_CELL handles the next address
                    bytes_remaining <= bytes_remaining - 10'd1;
                    state           <= ST_BIT_CELL;
                end

                // -------------------------------------------------
                // BIT_CELL: run one full bit cell (BIT_CELL cycles).
                // For "1" bit: drive rddata LOW during the last
                //   PULSE_WIDTH cycles (creates falling edge).
                // For "0" bit: rddata stays HIGH throughout.
                // -------------------------------------------------
                ST_BIT_CELL: begin
                    if (shift_reg[bit_index] && cell_counter >= PULSE_START[4:0]) begin
                        rddata <= 1'b0;  // pulse LOW for "1" bit
                    end else begin
                        rddata <= 1'b1;  // HIGH otherwise
                    end

                    // Pre-fetch next byte address 2 cycles before byte ends
                    // so buf_data is valid at NEXT_BIT (1-cycle read latency)
                    if (bit_index == 3'd0 && cell_counter == (BIT_CELL - 2)) begin
                        buf_addr <= buf_addr + 10'd1;
                    end

                    if (cell_counter == (BIT_CELL - 1)) begin
                        state <= ST_NEXT_BIT;
                    end else begin
                        cell_counter <= cell_counter + 5'd1;
                    end
                end

                // -------------------------------------------------
                // NEXT_BIT: advance to next bit or load next byte.
                // buf_addr was pre-set during last BIT_CELL cycle
                // (when bit_index==0 and cell_counter==BIT_CELL-2),
                // so buf_data is valid NOW for zero-gap loading.
                // -------------------------------------------------
                ST_NEXT_BIT: begin
                    rddata <= 1'b1;  // return HIGH after pulse
                    if (bit_index == 3'd0) begin
                        // Byte complete — load next from pre-fetched data
                        if (bytes_remaining == 10'd0) begin
                            state <= ST_DONE;
                        end else begin
                            shift_reg       <= buf_data;
                            bit_index       <= 3'd7;
                            cell_counter    <= 5'd0;
                            bytes_remaining <= bytes_remaining - 10'd1;
                            state           <= ST_BIT_CELL;
                        end
                    end else begin
                        bit_index    <= bit_index - 3'd1;
                        cell_counter <= 5'd0;
                        state        <= ST_BIT_CELL;
                    end
                end

                // -------------------------------------------------
                ST_DONE: begin
                    done   <= 1'b1;
                    rddata <= 1'b1;
                    busy   <= 1'b0;
                    state  <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
