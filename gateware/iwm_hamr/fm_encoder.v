// ===================================================================
// FM Encoder -- SmartPort Device TX (buffer-driven)
// ===================================================================
// FM-encodes bytes from a buffer for transmission via rddata.
// Two-state design: BIT_CELL (27 cycles) + NEXT_BIT (1 cycle) = 28 fclk/bit.
// Pulse: 7 fclk (~1µs) centered in bit cell, matching PicoPort/FujiNet.
// ===================================================================

`timescale 1ns / 1ps

module fm_encoder (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       enable,
    input  wire       start,
    input  wire [9:0] byte_count,
    output reg        rddata = 1'b1,
    output reg        busy = 1'b0,
    output reg        done = 1'b0,
    output reg  [9:0] buf_addr = 10'd0,
    input  wire [7:0] buf_data
);

    localparam BIT_CELL    = 28;       // 28 cycles in BIT_CELL + 1 in NEXT_BIT = 29 total
    localparam PULSE_WIDTH = 7;       // ~1µs pulse, matching PicoPort/FujiNet
    localparam PULSE_START = (BIT_CELL - PULSE_WIDTH) / 2;  // = 10
    localparam PULSE_END   = PULSE_START + PULSE_WIDTH;      // = 17

    localparam ST_IDLE      = 3'd0;
    localparam ST_ADDR      = 3'd1;
    localparam ST_LOAD      = 3'd2;
    localparam ST_BIT_CELL  = 3'd3;
    localparam ST_NEXT_BIT  = 3'd4;
    localparam ST_DONE      = 3'd5;

    reg [2:0] state = ST_IDLE;
    reg [7:0] shift_reg = 8'd0;
    reg [2:0] bit_index = 3'd7;
    reg [4:0] cell_counter = 5'd0;
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
        end else if (!enable) begin
            state   <= ST_IDLE;
            rddata  <= 1'b1;
            busy    <= 1'b0;
            done    <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)
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

                ST_ADDR: begin
                    if (bytes_remaining == 10'd0) begin
                        state <= ST_DONE;
                    end else begin
                        state <= ST_LOAD;
                    end
                end

                ST_LOAD: begin
                    shift_reg       <= buf_data;
                    bit_index       <= 3'd7;
                    cell_counter    <= 5'd0;
                    bytes_remaining <= bytes_remaining - 10'd1;
                    state           <= ST_BIT_CELL;
                end

                ST_BIT_CELL: begin
                    if (shift_reg[bit_index] &&
                        cell_counter >= PULSE_START[4:0] &&
                        cell_counter < PULSE_END[4:0]) begin
                        rddata <= 1'b0;
                    end else begin
                        rddata <= 1'b1;
                    end

                    // Pre-fetch next byte 2 cycles before byte ends
                    if (bit_index == 3'd0 && cell_counter == (BIT_CELL - 2)) begin
                        buf_addr <= buf_addr + 10'd1;
                    end

                    if (cell_counter == (BIT_CELL - 1)) begin
                        state <= ST_NEXT_BIT;
                    end else begin
                        cell_counter <= cell_counter + 5'd1;
                    end
                end

                ST_NEXT_BIT: begin
                    rddata <= 1'b1;
                    if (bit_index == 3'd0) begin
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
