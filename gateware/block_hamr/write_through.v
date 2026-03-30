// write_through.v — Block-ready gating for synchronous flash persistence
//
// Gates arb_block_ready during writes, signals flash_persist to start,
// releases block_ready when flash_persist completes. Reads pass through.
//
// Flash operations are in a SEPARATE MODULE (flash_persist.v) to avoid
// Yosys ECP5 synthesis corruption when flash logic and block_ready gating
// share a state machine.

`timescale 1ns / 1ps

module write_through (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        boot_done,

    // Arbiter block_ready interception
    input  wire        arb_block_ready,
    output wire        gated_block_ready,

    // Write detection (7 MHz domain)
    input  wire        dev_block_write_req,
    input  wire [15:0] dev_block_num,

    // Flash persist interface
    output reg         fp_start = 1'b0,
    output reg  [5:0]  fp_sector = 6'd0,
    input  wire        fp_busy
);

    // CDC for write detection
    reg wr_req_r1 = 1'b0, wr_req_r2 = 1'b0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin wr_req_r1 <= 0; wr_req_r2 <= 0; end
        else begin wr_req_r1 <= dev_block_write_req; wr_req_r2 <= wr_req_r1; end
    end
    wire is_write = wr_req_r2;

    // Rising edge on arb_block_ready
    reg arb_ready_d1 = 1'b0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) arb_ready_d1 <= 0;
        else arb_ready_d1 <= arb_block_ready;
    end
    wire arb_ready_rise = arb_block_ready & ~arb_ready_d1;

    // Gate control
    reg holding = 1'b0;

    wire do_trigger = arb_ready_rise && is_write && boot_done;
    wire gate_active = holding || do_trigger;
    assign gated_block_ready = gate_active ? 1'b0 : arb_block_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            holding   <= 1'b0;
            fp_start  <= 1'b0;
            fp_sector <= 6'd0;
        end else begin
            fp_start <= 1'b0;  // default: single-cycle pulse

            if (!holding && do_trigger) begin
                holding   <= 1'b1;
                `ifndef TIMER_ONLY
                fp_start  <= 1'b1;
                fp_sector <= dev_block_num[8:3];
                `endif
            end

            `ifdef TIMER_ONLY
            // Release after 1 cycle (no flash ops)
            if (holding && !do_trigger)
                holding <= 1'b0;
            `else
            // Release when flash_persist completes
            if (holding && !fp_busy && !fp_start)
                holding <= 1'b0;
            `endif
        end
    end

endmodule
