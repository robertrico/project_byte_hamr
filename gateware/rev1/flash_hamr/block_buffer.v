`timescale 1ns / 1ps
// =============================================================================
// block_buffer.v — 512-byte dual-port BRAM with independent clocks
// =============================================================================
// Port A: clk_a (25 MHz) — SDRAM arbiter / CPU
// Port B: clk_b (7 MHz)  — bus_interface (6502 side)
//
// Uses explicit ECP5 DP16KD instantiation instead of Yosys inference.
// Yosys was misinferring the dual-clock BRAM — DOA was routed to rdata_b
// instead of DOB, causing Port A writes to be invisible from Port B reads.
// =============================================================================

module block_buffer (
    // Port A (25 MHz - SDRAM arbiter / CPU)
    input  wire        clk_a,
    input  wire [8:0]  addr_a,
    input  wire [7:0]  wdata_a,
    input  wire        we_a,
    output wire [7:0]  rdata_a,

    // Port B (7 MHz - bus_interface)
    input  wire        clk_b,
    input  wire [8:0]  addr_b,
    input  wire [7:0]  wdata_b,
    input  wire        we_b,
    output wire [7:0]  rdata_b
);

    // DP16KD in 9-bit mode: 2048 x 9
    // Address: ADA[13:3] = {2'b0, addr[8:0]}, ADA[2:0] = CS = 3'b000
    // Data: DIA[8:0] = {1'b0, data[7:0]}, DOA[7:0] = rdata
    // REGMODE = NOREG: single-cycle registered read (address latched on clock edge)

    DP16KD #(
        .DATA_WIDTH_A(9),
        .DATA_WIDTH_B(9),
        .REGMODE_A("NOREG"),
        .REGMODE_B("NOREG"),
        .CSDECODE_A("0b000"),
        .CSDECODE_B("0b000"),
        .WRITEMODE_A("NORMAL"),
        .WRITEMODE_B("NORMAL"),
        .GSR("DISABLED")
    ) u_bram (
        // ---- Port A: 25 MHz (arbiter / CPU) ----
        .CLKA(clk_a),
        .CEA(1'b1),
        .OCEA(1'b1),
        .WEA(we_a),
        .RSTA(1'b0),
        .CSA2(1'b0), .CSA1(1'b0), .CSA0(1'b0),
        // Address: [13:3] = {2'b0, addr_a[8:0]}, [2:0] = chip select
        .ADA13(1'b0),
        .ADA12(1'b0),
        .ADA11(addr_a[8]),
        .ADA10(addr_a[7]),
        .ADA9(addr_a[6]),
        .ADA8(addr_a[5]),
        .ADA7(addr_a[4]),
        .ADA6(addr_a[3]),
        .ADA5(addr_a[2]),
        .ADA4(addr_a[1]),
        .ADA3(addr_a[0]),
        .ADA2(1'b0), .ADA1(1'b0), .ADA0(1'b0),
        // Data in: [8:0] = {1'b0, wdata_a[7:0]}
        .DIA17(1'b0), .DIA16(1'b0), .DIA15(1'b0), .DIA14(1'b0),
        .DIA13(1'b0), .DIA12(1'b0), .DIA11(1'b0), .DIA10(1'b0),
        .DIA9(1'b0),  .DIA8(1'b0),
        .DIA7(wdata_a[7]), .DIA6(wdata_a[6]),
        .DIA5(wdata_a[5]), .DIA4(wdata_a[4]),
        .DIA3(wdata_a[3]), .DIA2(wdata_a[2]),
        .DIA1(wdata_a[1]), .DIA0(wdata_a[0]),
        // Data out
        .DOA17(), .DOA16(), .DOA15(), .DOA14(),
        .DOA13(), .DOA12(), .DOA11(), .DOA10(),
        .DOA9(),  .DOA8(),
        .DOA7(rdata_a[7]), .DOA6(rdata_a[6]),
        .DOA5(rdata_a[5]), .DOA4(rdata_a[4]),
        .DOA3(rdata_a[3]), .DOA2(rdata_a[2]),
        .DOA1(rdata_a[1]), .DOA0(rdata_a[0]),

        // ---- Port B: 7 MHz (bus_interface / 6502) ----
        .CLKB(clk_b),
        .CEB(1'b1),
        .OCEB(1'b1),
        .WEB(we_b),
        .RSTB(1'b0),
        .CSB2(1'b0), .CSB1(1'b0), .CSB0(1'b0),
        // Address: [13:3] = {2'b0, addr_b[8:0]}, [2:0] = chip select
        .ADB13(1'b0),
        .ADB12(1'b0),
        .ADB11(addr_b[8]),
        .ADB10(addr_b[7]),
        .ADB9(addr_b[6]),
        .ADB8(addr_b[5]),
        .ADB7(addr_b[4]),
        .ADB6(addr_b[3]),
        .ADB5(addr_b[2]),
        .ADB4(addr_b[1]),
        .ADB3(addr_b[0]),
        .ADB2(1'b0), .ADB1(1'b0), .ADB0(1'b0),
        // Data in: [8:0] = {1'b0, wdata_b[7:0]}
        .DIB17(1'b0), .DIB16(1'b0), .DIB15(1'b0), .DIB14(1'b0),
        .DIB13(1'b0), .DIB12(1'b0), .DIB11(1'b0), .DIB10(1'b0),
        .DIB9(1'b0),  .DIB8(1'b0),
        .DIB7(wdata_b[7]), .DIB6(wdata_b[6]),
        .DIB5(wdata_b[5]), .DIB4(wdata_b[4]),
        .DIB3(wdata_b[3]), .DIB2(wdata_b[2]),
        .DIB1(wdata_b[1]), .DIB0(wdata_b[0]),
        // Data out
        .DOB17(), .DOB16(), .DOB15(), .DOB14(),
        .DOB13(), .DOB12(), .DOB11(), .DOB10(),
        .DOB9(),  .DOB8(),
        .DOB7(rdata_b[7]), .DOB6(rdata_b[6]),
        .DOB5(rdata_b[5]), .DOB4(rdata_b[4]),
        .DOB3(rdata_b[3]), .DOB2(rdata_b[2]),
        .DOB1(rdata_b[1]), .DOB0(rdata_b[0])
    );

endmodule
