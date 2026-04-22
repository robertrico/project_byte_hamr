`timescale 1ns / 1ps
// =============================================================================
// uart_tx.v — UART transmitter (TX only)
// =============================================================================
// 115200 baud, 8N1. Memory-mapped: write DATA to send, poll STATUS for busy.
// ~50 lines. No RX — add later if bootloader path is taken.
// =============================================================================

module uart_tx #(
    parameter CLK_FREQ = 25_000_000,
    parameter BAUD     = 115_200
)(
    input  wire        clk,
    input  wire        rst_n,

    // Register interface (directly memory-mapped)
    input  wire        wr_en,      // write strobe
    input  wire [7:0]  wr_data,    // byte to send

    output wire        busy,       // 1 while transmitting

    // UART pin
    output reg         tx
);

    localparam CLKS_PER_BIT = CLK_FREQ / BAUD;  // 25M/115200 = 217

    reg [7:0]  shift;
    reg [3:0]  bit_idx;    // 0=idle, 1=start, 2-9=data, 10=stop
    reg [7:0]  clk_cnt;

    assign busy = (bit_idx != 4'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx      <= 1'b1;   // idle high
            bit_idx <= 4'd0;
            clk_cnt <= 8'd0;
            shift   <= 8'd0;
        end else if (bit_idx == 4'd0) begin
            tx <= 1'b1;  // idle
            if (wr_en) begin
                shift   <= wr_data;
                bit_idx <= 4'd1;
                clk_cnt <= 8'd0;
                tx      <= 1'b0;   // start bit
            end
        end else begin
            if (clk_cnt == CLKS_PER_BIT[7:0] - 1) begin
                clk_cnt <= 8'd0;
                if (bit_idx <= 4'd8) begin
                    // Data bits (LSB first): bit_idx 1->D0, 2->D1, ... 8->D7
                    tx      <= shift[0];
                    shift   <= {1'b0, shift[7:1]};
                    bit_idx <= bit_idx + 4'd1;
                end else if (bit_idx == 4'd9) begin
                    tx      <= 1'b1;   // stop bit
                    bit_idx <= bit_idx + 4'd1;
                end else begin
                    bit_idx <= 4'd0;   // done
                end
            end else begin
                clk_cnt <= clk_cnt + 8'd1;
            end
        end
    end

endmodule
