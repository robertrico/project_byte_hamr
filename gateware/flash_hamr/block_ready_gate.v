`timescale 1ns / 1ps
// =============================================================================
// block_ready_gate.v — Persist gating (renamed from write_through.v)
// =============================================================================
// MUST stay in its own module. Yosys synthesis corruption lesson:
// merging gating logic into other modules (sdram_arbiter, bus_interface)
// causes unreachable-state corruption on ECP5.
//
// Job: intercept arb_block_ready, hold gated_block_ready LOW after S4D2
// writes until CPU confirms persist complete via mailbox toggle.
// =============================================================================

module block_ready_gate #(
    parameter UNIT2_OFFSET = 16'd2048
)(
    input  wire        clk,         // 7 MHz
    input  wire        rst_n,
    input  wire        boot_done,

    // From sdram_arbiter
    input  wire        arb_block_ready,

    // To bus_interface
    output wire        gated_block_ready,

    // From bus_interface (write request info)
    input  wire        dev_block_write_req,
    input  wire [15:0] dev_block_num,

    // To mailbox (persist request)
    output reg  [15:0] persist_block,
    output reg         persist_wr,     // pulse: request persist

    // From mailbox (persist complete — synced toggle edge)
    input  wire        persist_done,   // level: toggled by CPU, edge-detected here

    // Control
    input  wire        persist_enabled
);

    // =========================================================================
    // Edge detection on arb_block_ready (25MHz -> 7MHz via existing 2-FF in arbiter)
    // =========================================================================
    reg br_d1, br_prev;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            br_d1   <= 1'b0;
            br_prev <= 1'b0;
        end else begin
            br_d1   <= arb_block_ready;
            br_prev <= br_d1;
        end
    end
    wire br_rise = br_d1 & ~br_prev;

    // Write request sync (already in 7MHz domain from bus_interface)
    reg wr_req_d1, wr_req_d2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_req_d1 <= 1'b0;
            wr_req_d2 <= 1'b0;
        end else begin
            wr_req_d1 <= dev_block_write_req;
            wr_req_d2 <= wr_req_d1;
        end
    end

    // Previously checked UNIT2_OFFSET — now persist all writes when enabled
    // (mount writes to block 0+, not block 2048+)

    // Persist done edge detection (toggle from CPU, synced to 7MHz)
    reg pd_s1, pd_s2, pd_prev;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pd_s1 <= 0; pd_s2 <= 0; pd_prev <= 0;
        end else begin
            pd_s1   <= persist_done;
            pd_s2   <= pd_s1;
            pd_prev <= pd_s2;
        end
    end
    wire pd_edge = (pd_s2 != pd_prev);

    // =========================================================================
    // Gate logic
    // =========================================================================
    reg holding = 1'b0;
    reg persist_wr_d1 = 1'b0;

    // Trigger: rising edge of block_ready after a S4D2 write
    wire trigger = br_rise & wr_req_d2 & boot_done & persist_enabled;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            holding      <= 1'b0;
            persist_block <= 16'd0;
            persist_wr   <= 1'b0;
            persist_wr_d1 <= 1'b0;
        end else begin
            persist_wr    <= 1'b0;  // default: clear pulse
            persist_wr_d1 <= persist_wr;

            if (!holding && trigger) begin
                holding       <= 1'b1;
                persist_block <= dev_block_num;
                persist_wr    <= 1'b1;
            end

            if (holding && pd_edge && !persist_wr && !persist_wr_d1)
                holding <= 1'b0;
        end
    end

    assign gated_block_ready = holding ? 1'b0 : arb_block_ready;

endmodule
