`timescale 1ns / 1ps
// =============================================================================
// Boot ROM Module for Yellow Hamr
// =============================================================================
// 4KB Liron ROM wrapper using LUT logic (combinatorial read)
//
// The ROM contains the Liron disk controller firmware and is accessed via:
//   - $C400-$C4FF (nI_O_SELECT) -> ROM $000-$0FF
//   - $C800-$CFFF (nI_O_STROBE) -> ROM $100-$7FF (when expansion active)
//
// The ROM data is loaded from liron_rom.mem at synthesis time.
//
// COMBINATORIAL READ — no clock dependency.
// The previous registered-BRAM approach sampled addr on posedge sig_7M,
// but addr transitions asynchronously relative to sig_7M. This caused
// intermittent reads of partially-transitioned addresses, corrupting
// instruction fetches and crashing the 6502. A combinatorial read
// eliminates the race entirely — data is valid as soon as addr settles.
//
// No ram_style attribute — Yosys finds no BRAM/LUTRAM match for async
// reads and falls through to memory_map, which decomposes to LUT logic.
// =============================================================================

module boot_rom (
    input wire        clk,          // Clock (unused — kept for port compat)
    input wire [11:0] addr,         // 12-bit address (4KB)
    output wire [7:0] data          // 8-bit data output (combinatorial)
);

    // 4KB ROM array - initialized from hex file
    // No ram_style attribute: async read can't use BRAM or LUTRAM on ECP5,
    // so Yosys memory_map decomposes to LUT MUX trees automatically
    reg [7:0] rom [0:4095];

    // Load ROM contents at synthesis/simulation
    initial begin
        $readmemh("liron_rom.mem", rom);
    end

    // Combinatorial read — data valid immediately when addr settles
    assign data = rom[addr];

endmodule
