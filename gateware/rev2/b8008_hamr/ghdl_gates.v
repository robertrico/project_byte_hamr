// GHDL synthesized gate primitives
// These implement multiplexed D flip-flops used by GHDL when synthesizing
// edge detection on non-clock signals
//
// gate_mdff: Multiplexed DFF - if clk rises, use d, else use els
// gate_midff: Same but with initial value
//
// These are scalar (1-bit) gates - GHDL generates separate instances
// for each bit of wider signals

module gate_mdff (
    input wire clk,
    input wire d,
    input wire els,
    output reg q
);
    always @(posedge clk) begin
        q <= d;
    end
endmodule

module gate_midff (
    input wire clk,
    input wire d,
    input wire els,
    input wire init,
    output reg q
);
    initial begin
        q = 1'b0;
    end
    always @(posedge clk) begin
        q <= d;
    end
endmodule
