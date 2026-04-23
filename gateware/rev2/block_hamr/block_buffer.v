`timescale 1ns / 1ps
// =============================================================================
// block_buffer.v — 512-byte dual-port BRAM with independent clocks
// =============================================================================
// Port A: clk_a (25 MHz) — SDRAM arbiter
// Port B: clk_b (7 MHz)  — bus_interface (CPU side)
// Both ports have synchronous read and write.
// =============================================================================

module block_buffer (
    // Port A (25 MHz - SDRAM arbiter)
    input  wire        clk_a,
    input  wire [8:0]  addr_a,
    input  wire [7:0]  wdata_a,
    input  wire        we_a,
    output reg  [7:0]  rdata_a,

    // Port B (7 MHz - bus_interface)
    input  wire        clk_b,
    input  wire [8:0]  addr_b,
    input  wire [7:0]  wdata_b,
    input  wire        we_b,
    output reg  [7:0]  rdata_b
);

    reg [7:0] mem [0:511];

    // Port A: 25 MHz
    always @(posedge clk_a) begin
        if (we_a)
            mem[addr_a] <= wdata_a;
        rdata_a <= mem[addr_a];
    end

    // Port B: 7 MHz
    always @(posedge clk_b) begin
        if (we_b)
            mem[addr_b] <= wdata_b;
        rdata_b <= mem[addr_b];
    end

endmodule
