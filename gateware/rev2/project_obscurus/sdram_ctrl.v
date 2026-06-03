// =============================================================================
// sdram_ctrl.v — byte-addressed SDRAM controller for project_obscurus
// =============================================================================
// AS4C32M16: {BA[1:0], ROW[12:0], COL[9:0]} = 25-bit word address.
//   phys_word = phys_addr[25:1]
//   COL = phys_addr[10:1], ROW = phys_addr[23:11], BA = phys_addr[25:24]
//   byte_lane = phys_addr[0]   (0 = low byte D[7:0])
// Byte-packed: write masks one lane via DQM; read takes both lanes, selects.
// Refresh: free-running counter, PRIORITY over a pending request when due.
// Request: pulse `req` (with `we`,`phys_addr`,`wdata`) when !busy. `busy` rises
// next cycle, falls when the access completes; `rdata` valid while !busy after
// a read.
// =============================================================================
module sdram_ctrl (
    input  wire        clk,
    input  wire        rst_n,

    // request interface
    input  wire        req,
    input  wire        we,
    input  wire [25:0] phys_addr,
    input  wire [7:0]  wdata,
    output reg  [7:0]  rdata,
    output reg         busy,
    output reg         ready,

    // SDRAM (dq tristate kept in top via dq_out/dq_oe/dq_in)
    output reg         SDRAM_CKE,
    output reg         SDRAM_nCS,
    output reg         SDRAM_nRAS,
    output reg         SDRAM_nCAS,
    output reg         SDRAM_nWE,
    output reg         SDRAM_DQM0,
    output reg         SDRAM_DQM1,
    output reg         SDRAM_BA0,
    output reg         SDRAM_BA1,
    output reg [12:0]  sdram_a,
    output reg [15:0]  dq_out,
    output reg         dq_oe,
    input  wire [15:0] dq_in
);
    localparam [3:0]
        CMD_NOP=4'b0111, CMD_ACTIVE=4'b0011, CMD_READ=4'b0101,
        CMD_WRITE=4'b0100, CMD_PRECHARGE=4'b0010, CMD_REFRESH=4'b0001,
        CMD_LOAD_MODE=4'b0000;

    localparam [3:0]
        ST_INIT=4'd0, ST_PRECH=4'd1, ST_REF1=4'd2, ST_REF2=4'd3, ST_LMR=4'd4,
        ST_IDLE=4'd5, ST_ACT=4'd6, ST_RW=4'd7, ST_WAIT=4'd8, ST_RDLAT=4'd9,
        ST_REFNOW=4'd10;

    reg [3:0]  st = ST_INIT;
    reg [15:0] dly = 16'd5000;
    reg [3:0]  cmd;

    // latched request
    reg        pend = 1'b0;
    reg        pend_we;
    reg [1:0]  pend_ba;
    reg [12:0] pend_row;
    reg [9:0]  pend_col;
    reg        pend_lane;
    reg [7:0]  pend_wdata;

    // free-running refresh
    reg [15:0] ref_cnt = 16'd0;
    reg        ref_due = 1'b0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st<=ST_INIT; dly<=16'd5000; ready<=1'b0; busy<=1'b0;
            SDRAM_CKE<=1'b0; pend<=1'b0; ref_cnt<=16'd0; ref_due<=1'b0;
            dq_oe<=1'b0; rdata<=8'h00;
        end else begin
            cmd        = CMD_NOP;     // default this cycle
            dq_oe     <= 1'b0;
            SDRAM_DQM0<= 1'b0;
            SDRAM_DQM1<= 1'b0;

            // ---- free-running refresh timer (NOT reset by accesses) ----
            if (ref_cnt == 16'd195) begin ref_cnt<=16'd0; ref_due<=1'b1; end
            else ref_cnt <= ref_cnt + 1'b1;

            // ---- latch an incoming request ----
            // NOTE: gate on !pend (no request already outstanding) + ready, but
            // NOT on !busy. busy is also asserted while a PRIORITY refresh is in
            // flight; gating accept on !busy dropped any request that arrived
            // during a refresh (the TB pulses req for a single cycle). pend is
            // independent of refresh servicing, so latching here is safe — the
            // refresh completes first, then the latched request is serviced, and
            // busy stays continuously high across both (matching the TB's
            // wait(busy)/wait(!busy) contract).
            if (req && !pend && ready) begin
                pend       <= 1'b1;
                busy       <= 1'b1;        // request-scoped busy: rises on accept
                pend_we    <= we;
                pend_ba    <= phys_addr[25:24];
                pend_row   <= phys_addr[23:11];
                pend_col   <= phys_addr[10:1];
                pend_lane  <= phys_addr[0];
                pend_wdata <= wdata;
            end

            case (st)
                ST_INIT: begin
                    SDRAM_CKE<=1'b1;
                    if (dly==0) st<=ST_PRECH; else dly<=dly-1'b1;
                end
                ST_PRECH: begin
                    cmd=CMD_PRECHARGE; sdram_a<=13'b0_0100_0000_0000; // A10=1 all
                    dly<=16'd2; st<=ST_REF1;
                end
                ST_REF1: begin
                    if (dly==0) begin cmd=CMD_REFRESH; dly<=16'd8; st<=ST_REF2; end
                    else dly<=dly-1'b1;
                end
                ST_REF2: begin
                    if (dly==0) begin cmd=CMD_REFRESH; dly<=16'd8; st<=ST_LMR; end
                    else dly<=dly-1'b1;
                end
                ST_LMR: begin
                    if (dly==0) begin
                        cmd=CMD_LOAD_MODE;
                        {SDRAM_BA1,SDRAM_BA0}<=2'd0;
                        sdram_a<=13'b000_0_00_010_0_000;  // CL=2, burst=1
                        dly<=16'd2; ready<=1'b1; st<=ST_IDLE;
                    end else dly<=dly-1'b1;
                end

                ST_IDLE: begin
                    // busy is request-scoped (set on accept, cleared at access
                    // completion) and is NOT driven by refresh here. A refresh
                    // takes PRIORITY over a pending access, but a standalone
                    // refresh (no pend) must leave busy low so the TB's
                    // wait(busy) for the NEXT request is not satisfied early by
                    // an unrelated refresh — that race silently dropped requests.
                    if (ref_due) begin
                        ref_due<=1'b0; st<=ST_REFNOW;
                    end else if (pend) begin
                        {SDRAM_BA1,SDRAM_BA0}<=pend_ba;
                        st<=ST_ACT;
                    end
                end

                ST_REFNOW: begin
                    cmd=CMD_REFRESH; dly<=16'd8; st<=ST_IDLE;
                end

                ST_ACT: begin
                    cmd=CMD_ACTIVE; sdram_a<={pend_row};  // 13-bit row
                    dly<=16'd2; st<=ST_RW;
                end
                ST_RW: begin
                    if (dly==0) begin
                        // A10=1 auto-precharge, A[9:0]=col
                        sdram_a <= {2'b00, 1'b1, pend_col};
                        if (pend_we) begin
                            cmd=CMD_WRITE;
                            dq_out    <= {pend_wdata, pend_wdata};
                            dq_oe     <= 1'b1;
                            SDRAM_DQM0<= pend_lane;     // low lane enabled when even
                            SDRAM_DQM1<= ~pend_lane;    // high lane enabled when odd
                            dly<=16'd3; st<=ST_WAIT;
                        end else begin
                            cmd=CMD_READ;
                            SDRAM_DQM0<=1'b0; SDRAM_DQM1<=1'b0;  // read both lanes
                            // TIMING: reference used dly=2 here, which made
                            // ST_RDLAT sample dq_in one cycle BEFORE the model
                            // asserts dq_oe (CL=2 measured from when the model
                            // SEES the registered READ command, which is itself
                            // one cycle after cmd=CMD_READ). Bumping to 3 delays
                            // ST_WAIT->ST_RDLAT by one cycle so the sample lands
                            // on the cycle the model is actually driving dq.
                            dly<=16'd3; st<=ST_WAIT;
                        end
                    end else dly<=dly-1'b1;
                end
                ST_WAIT: begin
                    if (dly==0) begin
                        if (pend_we) begin pend<=1'b0; busy<=1'b0; st<=ST_IDLE; end
                        else st<=ST_RDLAT;
                    end else dly<=dly-1'b1;
                end
                ST_RDLAT: begin
                    rdata <= pend_lane ? dq_in[15:8] : dq_in[7:0];
                    pend<=1'b0; busy<=1'b0; st<=ST_IDLE;
                end

                default: st<=ST_INIT;
            endcase

            {SDRAM_nCS,SDRAM_nRAS,SDRAM_nCAS,SDRAM_nWE} <= cmd;
        end
    end
endmodule
