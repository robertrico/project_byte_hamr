// block_buffer.v — Dual-port 512-byte BRAM wrapper
// Port A: 25 MHz (SDRAM arbiter side)
// Port B:  7 MHz (device engine side)
// Standard dual-port pattern for ECP5 BRAM inference in Yosys.

module block_buffer (
    // Port A (25 MHz side - SDRAM arbiter)
    input  wire       clk_a,
    input  wire [8:0] addr_a,         // 0-511
    input  wire [7:0] wdata_a,
    input  wire       we_a,           // write enable
    output reg  [7:0] rdata_a,

    // Port B (7 MHz side - device engine)
    input  wire       clk_b,
    input  wire [8:0] addr_b,
    input  wire [7:0] wdata_b,
    input  wire       we_b,
    output reg  [7:0] rdata_b
);

    reg [7:0] mem [0:511];

`ifdef SIMULATION
    // Simulation model: use blocking writes to mem[] so both ports
    // see updates immediately. Port B uses #1 delay to let port A's
    // blocking write settle when both clocks edge simultaneously.
    // On real ECP5 BRAM, dual-port access is handled by the primitive.

    // Port A
    always @(posedge clk_a) begin
        if (we_a)
            mem[addr_a] = wdata_a;     // blocking: visible immediately
        rdata_a <= mem[addr_a];
    end

    // Port B
    always @(posedge clk_b) begin
        if (we_b)
            mem[addr_b] = wdata_b;     // blocking
        #1;
        rdata_b <= mem[addr_b];        // reads after port A's write settles
    end

`else
    // Synthesis model: standard dual-port BRAM pattern for ECP5 inference.
    // Port A
    always @(posedge clk_a) begin
        if (we_a)
            mem[addr_a] <= wdata_a;
        rdata_a <= mem[addr_a];
    end

    // Port B — registered read with 1-cycle latency.
    // sp_codec uses E_LOAD_WAIT between address set and data capture.
    always @(posedge clk_b) begin
        if (we_b)
            mem[addr_b] <= wdata_b;
        rdata_b <= mem[addr_b];
    end
`endif

endmodule
