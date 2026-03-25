// tx_packet_buf.v — Single-port BRAM for TX response packet
// Written by sp_codec (fast, one byte/clock during TX_FILL)
// Read by FM encoder (one byte every ~224 fclk during TX_SEND)
// Both in 7MHz domain — no CDC needed.

`timescale 1ns / 1ps

module tx_packet_buf (
    input  wire       clk,
    input  wire [9:0] addr,
    input  wire [7:0] wdata,
    input  wire       we,
    output reg  [7:0] rdata = 8'd0
);

    reg [7:0] mem [0:1023];

    always @(posedge clk) begin
        if (we)
            mem[addr] <= wdata;
        rdata <= mem[addr];
    end

endmodule
