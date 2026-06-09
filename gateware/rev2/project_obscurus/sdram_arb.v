// =============================================================================
// sdram_arb.v — N-shaped priority arbiter in front of sdram_ctrl.
// C0 (monitor) has priority over C1 (coproc). Each client sends a 1-cycle req
// pulse + we/addr/wdata; the arbiter latches it, serializes onto sdram_ctrl's
// req/busy contract, and returns a PER-CLIENT busy (high from accept to
// completion) + a latched per-client rdata. Adding clients = more pend bits +
// a priority slot; no rewrite.
// =============================================================================
module sdram_arb (
    input  wire        clk,
    input  wire        rst_n,
    output reg         req,
    output reg         we,
    output reg  [25:0] phys_addr,
    output reg  [7:0]  wdata,
    input  wire [7:0]  rdata,
    input  wire        busy,
    input  wire        c0_req,
    input  wire        c0_we,
    input  wire [25:0] c0_addr,
    input  wire [7:0]  c0_wdata,
    output wire        c0_busy,
    output reg  [7:0]  c0_rdata,
    input  wire        c1_req,
    input  wire        c1_we,
    input  wire [25:0] c1_addr,
    input  wire [7:0]  c1_wdata,
    output wire        c1_busy,
    output reg  [7:0]  c1_rdata
);
    reg c0_pend, c1_pend;
    reg servicing;
    reg owner;             // 0 = c0, 1 = c1

    reg busy_d;
    always @(posedge clk) busy_d <= busy;
    wire op_complete = busy_d & ~busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c0_pend<=1'b0; c1_pend<=1'b0; servicing<=1'b0; owner<=1'b0;
            req<=1'b0; we<=1'b0; phys_addr<=26'd0; wdata<=8'd0;
            c0_rdata<=8'd0; c1_rdata<=8'd0;
        end else begin
            req <= 1'b0;
            if (c0_req) c0_pend <= 1'b1;
            if (c1_req) c1_pend <= 1'b1;
            if (!servicing) begin
                if (c0_pend) begin
                    owner<=1'b0; we<=c0_we; phys_addr<=c0_addr; wdata<=c0_wdata;
                    req<=1'b1; servicing<=1'b1; c0_pend<=1'b0;
                end else if (c1_pend) begin
                    owner<=1'b1; we<=c1_we; phys_addr<=c1_addr; wdata<=c1_wdata;
                    req<=1'b1; servicing<=1'b1; c1_pend<=1'b0;
                end
            end else if (op_complete) begin
                if (owner==1'b0) c0_rdata<=rdata; else c1_rdata<=rdata;
                servicing<=1'b0;
            end
        end
    end

    assign c0_busy = c0_pend | (servicing & (owner==1'b0));
    assign c1_busy = c1_pend | (servicing & (owner==1'b1));
endmodule
