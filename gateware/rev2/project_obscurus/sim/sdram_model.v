`timescale 1ns / 1ps
// =============================================================================
// sdram_model.v — faithful behavioral model for project_obscurus sim
// =============================================================================
// Honors:
//   1. BOUNDED dense array (Verilog-2005, NO -g2012 — this iverilog hangs on
//      SystemVerilog). 64K words (128KB) covers banks 0-1 = all tests touch.
//      Accesses beyond WORDS print an error so a stray high address is caught,
//      not silently aliased.
//   2. Row+bank captured on ACTIVE; column captured on READ/WRITE (NOT ACTIVE).
//   3. Column = A[9:0] only — A10 (auto-precharge) is masked out.
//   4. DQM byte lanes honored on write (only the unmasked lane updates).
// Word index = {ba[1:0], row[12:0], col[9:0]} = 25-bit word address.
// =============================================================================
module sdram_model (
    input              clk,        // SDRAM_CLK
    input  [3:0]       cmd,        // {nCS,nRAS,nCAS,nWE}
    input  [1:0]       ba,
    input  [12:0]      a,
    input              dqm0,
    input              dqm1,
    inout  [15:0]      dq
);
    localparam CMD_ACTIVE = 4'b0011;
    localparam CMD_READ   = 4'b0101;
    localparam CMD_WRITE  = 4'b0100;

    // 2M words covers monitor banks 0..63 (phys byte addr 0..4M-1, widx=phys>>1).
    // Conway's Multiverse lives in banks 16..24; the FARM game world (GBANK=32)
    // sits at exactly the old 1M-word boundary, so the model was doubled.
    localparam WORDS = 2097152;    // 1<<21 words = banks 0..63
    reg [15:0] mem [0:WORDS-1];
    integer ii;
    initial for (ii = 0; ii < WORDS; ii = ii + 1) mem[ii] = 16'h0000;

    reg [1:0]  cur_ba  = 2'd0;
    reg [12:0] cur_row = 13'd0;

    reg [15:0] dq_drive = 16'h0000;
    reg        dq_oe    = 1'b0;
    assign dq = dq_oe ? dq_drive : 16'hZZZZ;

    // CL=2 read pipeline
    reg        rd_pending = 1'b0;
    integer    rd_latency = 0;
    reg [24:0] rd_widx    = 25'd0;

    function [24:0] widx(input [1:0] b, input [12:0] r, input [9:0] c);
        widx = {b, r, c};
    endfunction

    reg [24:0] w;

    always @(posedge clk) begin
        // ---- ACTIVE: latch bank + row ----
        if (cmd == CMD_ACTIVE) begin
            cur_ba  <= ba;
            cur_row <= a;            // full 13-bit row
        end
        // ---- WRITE: column = a[9:0] (A10 masked), DQM lane select ----
        if (cmd == CMD_WRITE) begin
            w = widx(cur_ba, cur_row, a[9:0]);
            if (w >= WORDS) $display("MODEL ERR: write widx %0d out of range", w);
            else begin
                if (!dqm0) mem[w][7:0]  = dq[7:0];   // same-cycle masked write
                if (!dqm1) mem[w][15:8] = dq[15:8];
            end
        end
        // ---- READ: column = a[9:0], schedule CL=2 drive ----
        if (cmd == CMD_READ) begin
            rd_pending <= 1'b1;
            rd_latency <= 2;
            rd_widx    <= widx(cur_ba, cur_row, a[9:0]);
        end else if (rd_pending) begin
            if (rd_latency == 0) begin
                dq_drive   <= (rd_widx < WORDS) ? mem[rd_widx] : 16'h0000;
                dq_oe      <= 1'b1;
                rd_pending <= 1'b0;
            end else begin
                rd_latency <= rd_latency - 1;
                dq_oe      <= 1'b0;
            end
        end else begin
            dq_oe <= 1'b0;
        end
    end
endmodule
