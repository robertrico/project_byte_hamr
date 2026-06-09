// =============================================================================
// coproc.v — C1 resident-kernel coprocessor.
// Arlet soft-6502 runs a baked, write-protected kernel ($1000-$1FFF) that
// dispatches host-registered tasks from TABLE ($0200-$02FF). Dual-port BRAM:
// port A = Arlet (kernel+tasks execute), port B = host load. Two-axis write
// protection. COUNT is a host-owned register (read-only to Arlet at $E010).
// Generalized $E000-$E003 SDRAM write window (RDY-stall single post; Arlet WE is
// HELD HIGH through the stall, so latches gate on & rdy and the post is one-shot
// via state transition). See spec 2026-06-08.
// =============================================================================
module coproc #(
    parameter [7:0] CORE_ID = 8'd0
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        ready,
    output reg         req,
    output reg         we,
    output reg  [25:0] phys_addr,
    output reg  [7:0]  wdata,
    input  wire        busy,
    input  wire [7:0]  rdata,
    input  wire [12:0] laddr,
    input  wire [7:0]  ldata_in,
    input  wire        lwr,
    output reg  [7:0]  ldata_out,
    input  wire [7:0]  count_in,
    input  wire        count_wr,
    // ---- C4 async-dispatch control plane ----
    input  wire        c4_ring_wr,      // host CP_RING strobe (top: reg_wr & $C0C5 W)
    input  wire        c4_collect_wr,   // host CP_COLLECT strobe (top: reg_wr & $C0CC W)
    input  wire [1:0]  c4_host_slot,    // slot # the host wrote as data (wr_data_latch[1:0])
    input  wire        snapshot_busy,   // top: restore_busy | save_busy
    output wire [3:0]  c4_done,         // host reads $C0C0
    output wire [3:0]  c4_active,       // host reads $C0C1
    output wire [3:0]  c4_timedout,     // host reads $C0C2
    output wire [3:0]  c4_callreq       // (test/visibility)
);
    wire [15:0] AB;
    wire [7:0]  DO;
    wire        WE;
    wire [7:0]  DI;
    reg         rdy;

    // ---- C3 preemptive tick timer state (declared before the cpu instance so
    // the active-high IRQ binding resolves under iverilog -g2005) ----
    wire is_e012 = (AB==16'hE012);   // TICK_CTL: write period (0=disarm); resets cnt + clears pending
    wire is_e013 = (AB==16'hE013);   // TICK_ACK: write clears pending
    reg  [7:0]  tick_period;
    reg  [15:0] tick_cnt;
    reg         irq_pending;

    cpu u_cpu (.clk(clk), .reset(~rst_n), .AB(AB), .DI(DI), .DO(DO), .WE(WE),
               // Arlet IRQ is ACTIVE-HIGH: core requests on (~I & IRQ), so feed
               // irq_pending directly (no inversion).
               .IRQ(irq_pending), .NMI(1'b0), .RDY(rdy));

    reg [7:0] task_count;
    always @(posedge clk or negedge rst_n)
        if (!rst_n)        task_count <= 8'd0;
        else if (count_wr) task_count <= count_in;

    // ---- C3 preemptive tick timer ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tick_period <= 8'd0; tick_cnt <= 16'd0; irq_pending <= 1'b0;
        end else begin
            if (is_e012 & WE & rdy) begin          // arm/disarm: reset counter + clear pending
                tick_period <= DO; tick_cnt <= 16'd0; irq_pending <= 1'b0;
            end else if (is_e013 & WE & rdy) begin  // ack
                irq_pending <= 1'b0;
            end else if (tick_period != 8'd0) begin
                if (tick_cnt >= {tick_period, 8'h00}) begin
                    tick_cnt <= 16'd0; irq_pending <= 1'b1;
                end else tick_cnt <= tick_cnt + 16'd1;
            end
        end
    end

    // ---- C3.1 GO trigger: a NONZERO CP_COUNT write re-bootstraps; $E014 status/ack ----
    // (count_wr = the CP_COUNT write strobe; count_in = the value.) The nonzero gate
    // keeps CP_COUNT=0 a benign park - a zero write would otherwise GO into a 0-task
    // BOOTSTRAP -> RESTORE on an uninitialized TCB -> crash.
    wire is_e014 = (AB==16'hE014);
    reg  go_pending;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                             go_pending <= 1'b0;
        else if (count_wr & (count_in != 8'd0)) go_pending <= 1'b1;   // set wins (before ack)
        else if (is_e014 & WE & rdy)            go_pending <= 1'b0;    // kernel ack
    end

    // ---- C4 control-plane registers (per-bit set/clear flops; index strobes) ----
    reg [3:0] call_req, running, done_r, timedout;
    wire is_callack = (AB==16'hE016) & WE & rdy;
    wire is_runset  = (AB==16'hE017) & WE & rdy;
    wire is_runclr  = (AB==16'hE018) & WE & rdy;
    wire is_doneset = (AB==16'hE019) & WE & rdy;
    wire is_tmoset  = (AB==16'hE01A) & WE & rdy;
    wire [3:0] kbit = (4'b0001 << DO[1:0]);        // kernel slot# (data) -> bit
    wire [3:0] hbit = (4'b0001 << c4_host_slot);   // host slot# -> bit
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin call_req<=0; running<=0; done_r<=0; timedout<=0; end
        else begin
            call_req <= (call_req & ~(is_callack    ? kbit : 4'd0)) | (c4_ring_wr  ? hbit : 4'd0);
            running  <= (running  & ~(is_runclr     ? kbit : 4'd0)) | (is_runset   ? kbit : 4'd0);
            done_r   <= (done_r   & ~(c4_collect_wr ? hbit : 4'd0)) | (is_doneset  ? kbit : 4'd0);
            timedout <= (timedout & ~(c4_collect_wr ? hbit : 4'd0)) | (is_tmoset   ? kbit : 4'd0);
        end
    end
    assign c4_callreq  = call_req;
    assign c4_done     = done_r;
    assign c4_timedout = timedout;
    assign c4_active   = call_req | running | done_r;

    // 8KB resident-kernel BRAM. ECP5 DP16KD has exactly TWO ports, but this
    // yosys (oss-cad-suite) will NOT infer DP16KD for a memory with TWO write
    // ports (each combined with a read) -- it always falls back to FF mapping
    // (verified: 1W+2R maps via $__DP16KD_, any 2W maps to FFs -> 150k LUTs).
    // So we present ONE write port (a priority mux of the two writers) plus two
    // independent read ports. Port B (host load) is a deliberate 6502 setup
    // sequence and takes priority; port A (Arlet) writes only when rdy=1. They
    // never legitimately collide, so the host>Arlet priority is safe and the
    // two-port execute/load semantics are preserved.
    reg [7:0] bram [0:8191];
    initial $readmemh("kernel.mem", bram);

    wire in_bram = (AB[15:13] == 3'b000);
    wire a_wr_ok = WE & in_bram & rdy & ~AB[12] & ~(AB[11:8]==4'h2);
    wire b_wr_ok = lwr & ~laddr[12];

    // single shared write port (host load wins over Arlet)
    wire        wr_en   = a_wr_ok | b_wr_ok;
    wire [12:0] wr_addr = b_wr_ok ? laddr : AB[12:0];
    wire [7:0]  wr_data = b_wr_ok ? ldata_in : DO;
    always @(posedge clk)
        if (wr_en) bram[wr_addr] <= wr_data;

    wire is_rstlo=(AB==16'hFFFC), is_rsthi=(AB==16'hFFFD);
    wire is_irqlo=(AB==16'hFFFE), is_irqhi=(AB==16'hFFFF);
    wire is_nmilo=(AB==16'hFFFA), is_nmihi=(AB==16'hFFFB);
    wire is_count=(AB==16'hE010);
    wire is_coreid=(AB==16'hE011);
    wire is_callreq=(AB==16'hE015);   // C4: kernel reads call_req
    wire is_snapbusy=(AB==16'hE01B);  // C4: kernel reads snapshot_busy (bit0)
    reg [7:0] bram_qa;
    reg in_bram_q, is_rstlo_q,is_rsthi_q,is_irqlo_q,is_irqhi_q,is_nmilo_q,is_nmihi_q,is_count_q,is_coreid_q,is_e014_q;
    reg is_callreq_q, is_snapbusy_q;
    always @(posedge clk) begin
        bram_qa    <= bram[AB[12:0]];   // port A read
        in_bram_q  <= in_bram;
        is_rstlo_q <= is_rstlo; is_rsthi_q <= is_rsthi;
        is_irqlo_q <= is_irqlo; is_irqhi_q <= is_irqhi;
        is_nmilo_q <= is_nmilo; is_nmihi_q <= is_nmihi;
        is_count_q <= is_count;
        is_coreid_q <= is_coreid;
        is_e014_q <= is_e014;
        is_callreq_q <= is_callreq; is_snapbusy_q <= is_snapbusy;
    end
    assign DI = is_rstlo_q ? 8'h00 : is_rsthi_q ? 8'h10
              : is_irqlo_q ? 8'h00 : is_irqhi_q ? 8'h1F
              : is_nmilo_q ? 8'h40 : is_nmihi_q ? 8'h1F
              : is_count_q  ? task_count
              : is_coreid_q ? CORE_ID
              : is_e014_q   ? {7'b0, go_pending}
              : is_callreq_q  ? {4'b0, call_req}
              : is_snapbusy_q ? {7'b0, snapshot_busy}
              : in_bram_q  ? bram_qa
              :              8'h00;

    always @(posedge clk)
        ldata_out <= bram[laddr];        // port B read

    wire is_e000=(AB==16'hE000), is_e001=(AB==16'hE001),
         is_e002=(AB==16'hE002), is_e003=(AB==16'hE003);
    reg [15:0] saddr; reg [7:0] sbank;
    reg busy_d; always @(posedge clk) busy_d <= busy; wire done = busy_d & ~busy;

    localparam ST_BOOT=2'd0, ST_RUN=2'd1, ST_WAIT=2'd2;
    reg [1:0] state;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            req<=0; we<=0; phys_addr<=0; wdata<=0; rdy<=0; state<=ST_BOOT;
            saddr<=0; sbank<=0;
        end else begin
            req <= 1'b0;
            if (is_e000 & WE & rdy) saddr[7:0]  <= DO;
            if (is_e001 & WE & rdy) saddr[15:8] <= DO;
            if (is_e002 & WE & rdy) sbank       <= DO;
            case (state)
                ST_BOOT: if (ready) begin rdy<=1'b1; state<=ST_RUN; end
                ST_RUN:  if (is_e003 & WE) begin
                    we<=1'b1; phys_addr<={2'b00, sbank, saddr}; wdata<=DO;
                    req<=1'b1; rdy<=1'b0; state<=ST_WAIT;
                end
                ST_WAIT: if (done) begin rdy<=1'b1; state<=ST_RUN; end
                default: state<=ST_BOOT;
            endcase
        end
    end
endmodule
