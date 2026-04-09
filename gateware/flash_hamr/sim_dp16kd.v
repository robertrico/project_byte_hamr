// sim_dp16kd.v — Minimal DP16KD simulation model for testbench use
// Behavioral dual-port RAM matching the ECP5 DP16KD in 9-bit mode

`timescale 1ns / 1ps

module DP16KD #(
    parameter DATA_WIDTH_A = 9,
    parameter DATA_WIDTH_B = 9,
    parameter REGMODE_A = "NOREG",
    parameter REGMODE_B = "NOREG",
    parameter CSDECODE_A = "0b000",
    parameter CSDECODE_B = "0b000",
    parameter WRITEMODE_A = "NORMAL",
    parameter WRITEMODE_B = "NORMAL",
    parameter GSR = "DISABLED"
)(
    input CLKA, CEA, OCEA, WEA, RSTA,
    input CSA2, CSA1, CSA0,
    input ADA13, ADA12, ADA11, ADA10, ADA9, ADA8, ADA7,
    input ADA6, ADA5, ADA4, ADA3, ADA2, ADA1, ADA0,
    input DIA17, DIA16, DIA15, DIA14, DIA13, DIA12, DIA11, DIA10,
    input DIA9, DIA8, DIA7, DIA6, DIA5, DIA4, DIA3, DIA2, DIA1, DIA0,
    output reg DOA17, DOA16, DOA15, DOA14, DOA13, DOA12, DOA11, DOA10,
    output reg DOA9, DOA8, DOA7, DOA6, DOA5, DOA4, DOA3, DOA2, DOA1, DOA0,
    input CLKB, CEB, OCEB, WEB, RSTB,
    input CSB2, CSB1, CSB0,
    input ADB13, ADB12, ADB11, ADB10, ADB9, ADB8, ADB7,
    input ADB6, ADB5, ADB4, ADB3, ADB2, ADB1, ADB0,
    input DIB17, DIB16, DIB15, DIB14, DIB13, DIB12, DIB11, DIB10,
    input DIB9, DIB8, DIB7, DIB6, DIB5, DIB4, DIB3, DIB2, DIB1, DIB0,
    output reg DOB17, DOB16, DOB15, DOB14, DOB13, DOB12, DOB11, DOB10,
    output reg DOB9, DOB8, DOB7, DOB6, DOB5, DOB4, DOB3, DOB2, DOB1, DOB0
);

    // 2048 x 9-bit memory (9-bit mode)
    reg [8:0] mem [0:2047];

    wire [10:0] addr_a = {ADA13, ADA12, ADA11, ADA10, ADA9, ADA8, ADA7, ADA6, ADA5, ADA4, ADA3};
    wire [10:0] addr_b = {ADB13, ADB12, ADB11, ADB10, ADB9, ADB8, ADB7, ADB6, ADB5, ADB4, ADB3};
    wire [8:0]  din_a = {DIA8, DIA7, DIA6, DIA5, DIA4, DIA3, DIA2, DIA1, DIA0};
    wire [8:0]  din_b = {DIB8, DIB7, DIB6, DIB5, DIB4, DIB3, DIB2, DIB1, DIB0};

    integer i;
    initial for (i = 0; i < 2048; i = i + 1) mem[i] = 9'd0;

    // Port A
    always @(posedge CLKA) begin
        if (CEA) begin
            if (WEA) mem[addr_a] <= din_a;
            {DOA8, DOA7, DOA6, DOA5, DOA4, DOA3, DOA2, DOA1, DOA0} <= mem[addr_a];
        end
    end

    // Port B
    always @(posedge CLKB) begin
        if (CEB) begin
            if (WEB) mem[addr_b] <= din_b;
            {DOB8, DOB7, DOB6, DOB5, DOB4, DOB3, DOB2, DOB1, DOB0} <= mem[addr_b];
        end
    end

    // Unused high bits
    always @(*) begin
        {DOA17, DOA16, DOA15, DOA14, DOA13, DOA12, DOA11, DOA10, DOA9} = 9'd0;
        {DOB17, DOB16, DOB15, DOB14, DOB13, DOB12, DOB11, DOB10, DOB9} = 9'd0;
    end

endmodule
