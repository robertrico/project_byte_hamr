`timescale 1ns / 1ps
// =============================================================================
// Address Decoder for Yellow Hamr
// =============================================================================
// Adapted from Steve Chamberlin's Yellowstone project
// Original target: Lattice MachXO2-1200HC
// Ported to: Lattice ECP5-85F (Byte Hamr)
//
// This module manages the expansion ROM active flag and ROM output enable.
//
// Memory Map (Slot 4):
//   $C0C0-$C0CF  nDEVICE_SELECT  IWM registers
//   $C400-$C4FF  nI_O_SELECT     Boot ROM ($000-$0FF)
//   $C800-$CFFF  nI_O_STROBE     Boot ROM ($100-$7FF) when expansion active
//
// Expansion ROM behavior:
//   - Set active when nI_O_SELECT goes low (slot accessed)
//   - Cleared when $CFFF is accessed (end of shared ROM space)
// =============================================================================

module addr_decoder (
    input wire [11:0] addr,             // Address bus (A0-A11)
    input wire        clk,              // Clock (7M)
    input wire        nI_O_STROBE,      // Shared 2K space ($C800-$CFFF)
    input wire        nI_O_SELECT,      // Card-specific 256 bytes ($C400-$C4FF)
    input wire        nRES,             // System reset (active low)
    output wire       rom_oe,           // ROM output enable (active high)
    output reg        rom_expansion_active  // 1 if this card's ROM is selected
);

    // Detect access to $CFFF (clears expansion ROM)
    // When nI_O_STROBE is low and address is $FFF, we're at $CFFF
    wire clear_expansion = ~nI_O_STROBE & (addr == 12'hFFF);

    // Expansion ROM active flag management
    // - Set when our slot's I/O space is accessed (nI_O_SELECT low)
    // - Cleared when $CFFF is accessed
    // - Cleared on reset
    always @(posedge clk or negedge nRES) begin
        if (~nRES) begin
            rom_expansion_active <= 1'b0;
        end
        else begin
            if (clear_expansion)
                rom_expansion_active <= 1'b0;
            else if (~nI_O_SELECT)
                rom_expansion_active <= 1'b1;
        end
    end

    // ROM output enable:
    // - Enable during I/O select ($C400-$C4FF)
    // - Enable during I/O strobe ($C800-$CFFF) when expansion is active
    assign rom_oe = ~nI_O_SELECT | (rom_expansion_active & ~nI_O_STROBE);

endmodule
