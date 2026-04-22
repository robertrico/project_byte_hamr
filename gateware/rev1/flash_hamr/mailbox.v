`timescale 1ns / 1ps
// =============================================================================
// mailbox.v — Toggle-CDC mailbox between bus_interface (7MHz) and CPU (25MHz)
// =============================================================================
// Tiny FF-based registers. No BRAM. Toggle-handshake CDC.
//
// CDC Protocol (critical):
//   Sender writes all data fields, then flips toggle.
//   Receiver synchronizes toggle through 2-FF chain.
//   On detected edge, receiver reads data fields combinationally — they are
//   guaranteed stable because sender hasn't touched them since before toggle.
//   Do NOT sync multi-bit fields individually.
// =============================================================================

module mailbox (
    // ---- 7 MHz domain (bus_interface side) ----
    input  wire        clk_bus,
    input  wire        rst_bus_n,

    // Command from 6502 (written by bus_interface)
    input  wire [7:0]  bus_cmd,
    input  wire [7:0]  bus_arg0,
    input  wire        bus_cmd_wr,       // pulse: new command written

    // Status to 6502 (read by bus_interface)
    output wire [7:0]  bus_status_flags,
    output wire [15:0] bus_total_blocks,

    // Persist request from block_ready_gate (7 MHz)
    input  wire [15:0] bus_persist_block,
    input  wire        bus_persist_wr,

    // Persist done toggle (25MHz, for block_ready_gate to sync)
    output wire        bus_persist_done_toggle,

    // ---- 25 MHz domain (CPU side) ----
    input  wire        clk_cpu,
    input  wire        rst_cpu_n,

    // Command to CPU
    output wire [7:0]  cpu_cmd,
    output wire [7:0]  cpu_arg0,
    output wire        cpu_cmd_pending,

    // Status from CPU
    input  wire [7:0]  cpu_status_flags,
    input  wire [15:0] cpu_total_blocks,
    input  wire        cpu_status_wr,

    // Persist to CPU
    output wire [15:0] cpu_persist_block,
    output wire        cpu_persist_pending,

    // CPU acknowledges (active in 25 MHz domain)
    input  wire        cpu_cmd_ack,
    input  wire        cpu_persist_done
);

    // =========================================================================
    // 7 MHz -> 25 MHz: Command path
    // =========================================================================

    // Data registers (written by bus_interface, read by CPU)
    reg [7:0] cmd_reg  = 8'd0;
    reg [7:0] arg0_reg = 8'd0;
    reg       cmd_req_toggle = 1'b0;

    always @(posedge clk_bus or negedge rst_bus_n) begin
        if (!rst_bus_n) begin
            cmd_reg        <= 8'd0;
            arg0_reg       <= 8'd0;
            cmd_req_toggle <= 1'b0;
        end else if (bus_cmd_wr) begin
            cmd_reg        <= bus_cmd;
            arg0_reg       <= bus_arg0;
            cmd_req_toggle <= ~cmd_req_toggle;
        end
    end

    // Sync cmd_req_toggle into 25 MHz domain
    reg cmd_req_s1 = 0, cmd_req_s2 = 0, cmd_req_prev = 0;
    always @(posedge clk_cpu or negedge rst_cpu_n) begin
        if (!rst_cpu_n) begin
            cmd_req_s1 <= 0; cmd_req_s2 <= 0; cmd_req_prev <= 0;
        end else begin
            cmd_req_s1   <= cmd_req_toggle;
            cmd_req_s2   <= cmd_req_s1;
            cmd_req_prev <= cmd_req_s2;
        end
    end

    assign cpu_cmd_pending = (cmd_req_s2 != cmd_req_prev);
    assign cpu_cmd  = cmd_reg;   // stable: written before toggle flip
    assign cpu_arg0 = arg0_reg;

    // =========================================================================
    // 7 MHz -> 25 MHz: Persist request path
    // =========================================================================

    reg [15:0] persist_blk_reg = 16'd0;
    reg        persist_req_toggle = 1'b0;

    always @(posedge clk_bus or negedge rst_bus_n) begin
        if (!rst_bus_n) begin
            persist_blk_reg    <= 16'd0;
            persist_req_toggle <= 1'b0;
        end else if (bus_persist_wr) begin
            persist_blk_reg    <= bus_persist_block;
            persist_req_toggle <= ~persist_req_toggle;
        end
    end

    reg persist_req_s1 = 0, persist_req_s2 = 0, persist_req_prev = 0;
    always @(posedge clk_cpu or negedge rst_cpu_n) begin
        if (!rst_cpu_n) begin
            persist_req_s1 <= 0; persist_req_s2 <= 0; persist_req_prev <= 0;
        end else begin
            persist_req_s1   <= persist_req_toggle;
            persist_req_s2   <= persist_req_s1;
            persist_req_prev <= persist_req_s2;
        end
    end

    assign cpu_persist_pending = (persist_req_s2 != persist_req_prev);
    assign cpu_persist_block   = persist_blk_reg;

    // =========================================================================
    // 25 MHz -> 7 MHz: CPU status path
    // =========================================================================
    // CPU writes flags/total_blocks then pulses cpu_status_wr.
    // These change infrequently — 2-FF sync per bit is safe.

    reg [7:0]  status_reg  = 8'd0;
    reg [15:0] tblk_reg    = 16'd0;

    always @(posedge clk_cpu or negedge rst_cpu_n) begin
        if (!rst_cpu_n) begin
            status_reg <= 8'd0;
            tblk_reg   <= 16'd0;
        end else if (cpu_status_wr) begin
            status_reg <= cpu_status_flags;
            tblk_reg   <= cpu_total_blocks;
        end
    end

    // 2-FF sync into 7 MHz
    reg [7:0]  flags_s1 = 0, flags_s2 = 0;
    reg [15:0] tblk_s1 = 0,  tblk_s2 = 0;

    always @(posedge clk_bus or negedge rst_bus_n) begin
        if (!rst_bus_n) begin
            flags_s1 <= 0; flags_s2 <= 0;
            tblk_s1  <= 0; tblk_s2  <= 0;
        end else begin
            flags_s1 <= status_reg; flags_s2 <= flags_s1;
            tblk_s1  <= tblk_reg;   tblk_s2  <= tblk_s1;
        end
    end

    assign bus_status_flags = flags_s2;
    assign bus_total_blocks = tblk_s2;

    // =========================================================================
    // 25 MHz -> 7 MHz: CPU ack toggles (for bus_interface to detect completion)
    // =========================================================================

    reg cmd_ack_toggle = 1'b0;
    reg persist_done_toggle = 1'b0;

    always @(posedge clk_cpu or negedge rst_cpu_n) begin
        if (!rst_cpu_n) begin
            cmd_ack_toggle      <= 1'b0;
            persist_done_toggle <= 1'b0;
        end else begin
            if (cpu_cmd_ack)
                cmd_ack_toggle <= ~cmd_ack_toggle;
            if (cpu_persist_done)
                persist_done_toggle <= ~persist_done_toggle;
        end
    end

    // Expose persist_done_toggle for block_ready_gate (synced to 7MHz there)
    assign bus_persist_done_toggle = persist_done_toggle;

endmodule
