`timescale 1ns / 1ps
// =============================================================================
// Boot ROM Module for Yellow Hamr
// =============================================================================
// 4KB Liron ROM wrapper using ECP5 EBR (Embedded Block RAM)
//
// The ROM contains the Liron disk controller firmware and is accessed via:
//   - $C400-$C4FF (nI_O_SELECT) -> ROM $000-$0FF
//   - $C800-$CFFF (nI_O_STROBE) -> ROM $100-$7FF (when expansion active)
//
// The ROM data is loaded from liron_rom.mem at synthesis time.
// =============================================================================

module boot_rom (
    input wire        clk,          // Clock (7M)
    input wire [11:0] addr,         // 12-bit address (4KB)
    output reg [7:0]  data          // 8-bit data output (registered)
);

    // 4KB ROM array - initialized from hex file
    // Force BRAM inference (required for proper $readmemh initialization on ECP5)
    (* ram_style = "block" *)
    reg [7:0] rom [0:4095];

    // Load ROM contents at synthesis/simulation
    initial begin
        $readmemh("liron_rom.mem", rom);
    end

    // Synchronize address then read - 2 cycle latency but stable sampling
    reg [11:0] addr_sync;

    always @(posedge clk) begin
        addr_sync <= addr;        // Cycle 1: capture address
        data <= rom[addr_sync];   // Cycle 2: read with stable address
    end

endmodule
