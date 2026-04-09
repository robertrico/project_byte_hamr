// write_through.v — Block-ready gating for synchronous SD card persistence
//
// Gates arb_block_ready during S4D2 writes, signals sd_persist to start,
// releases block_ready when sd_persist completes. Reads pass through.
// S4D1 writes (block_num < UNIT2_OFFSET) are NOT persisted.

`timescale 1ns / 1ps

module write_through #(
    parameter UNIT2_OFFSET = 16'd2048
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        boot_done,

    input  wire        arb_block_ready,
    output wire        gated_block_ready,

    input  wire        dev_block_write_req,
    input  wire [15:0] dev_block_num,

    output reg         fp_start = 1'b0,
    output reg  [15:0] fp_block_num = 16'd0,
    input  wire        fp_busy,

    input  wire        persist_enabled
);

    // All logic in ONE always block to avoid cross-block NBA timing issues
    reg wr_req_r1 = 1'b0, wr_req_r2 = 1'b0;
    reg arb_ready_d1 = 1'b0;
    reg holding = 1'b0;
    reg fp_start_d1 = 1'b0;

    wire is_s4d2 = (dev_block_num >= UNIT2_OFFSET);

    // gate_active is registered for clean output timing
    reg gate_active_r = 1'b0;
    assign gated_block_ready = gate_active_r ? 1'b0 : arb_block_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_req_r1    <= 1'b0;
            wr_req_r2    <= 1'b0;
            arb_ready_d1 <= 1'b0;
            holding      <= 1'b0;
            fp_start     <= 1'b0;
            fp_start_d1  <= 1'b0;
            fp_block_num <= 16'd0;
            gate_active_r <= 1'b0;
        end else begin
            // CDC for write request
            wr_req_r1    <= dev_block_write_req;
            wr_req_r2    <= wr_req_r1;

            // Rising edge detection using BLOCKING assignment
            // Guaranteed to use OLD arb_ready_d1 before NBA updates it
            begin : edge_detect
                reg rise;
                reg trigger;
                rise    = arb_block_ready & ~arb_ready_d1;
                trigger = rise & wr_req_r2 & boot_done & is_s4d2 & persist_enabled;

                // Update edge detector for next cycle
                arb_ready_d1 <= arb_block_ready;

                // Default: clear start pulse
                fp_start    <= 1'b0;
                fp_start_d1 <= fp_start;

                // Trigger on S4D2 write completion
                if (!holding && trigger) begin
                    holding      <= 1'b1;
                    fp_start     <= 1'b1;
                    fp_block_num <= dev_block_num;
                end

                // Release when persist completes
                if (holding && !fp_busy && !fp_start && !fp_start_d1)
                    holding <= 1'b0;

                // Gate output
                gate_active_r <= holding || trigger;
            end
        end
    end

endmodule
