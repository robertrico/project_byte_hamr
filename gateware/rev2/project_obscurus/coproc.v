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
    input  wire        count_wr
);
    wire [15:0] AB;
    wire [7:0]  DO;
    wire        WE;
    wire [7:0]  DI;
    reg         rdy;

    cpu u_cpu (.clk(clk), .reset(~rst_n), .AB(AB), .DI(DI), .DO(DO), .WE(WE),
               // Arlet IRQ is ACTIVE-HIGH: core requests on (~I & IRQ), so feed
               // irq_pending directly (no inversion).
               .IRQ(irq_pending), .NMI(1'b0), .RDY(rdy));

    reg [7:0] task_count;
    always @(posedge clk or negedge rst_n)
        if (!rst_n)        task_count <= 8'd0;
        else if (count_wr) task_count <= count_in;

    // ---- C3 preemptive tick timer ----
    wire is_e012 = (AB==16'hE012);   // TICK_CTL: write period (0=disarm); resets cnt + clears pending
    wire is_e013 = (AB==16'hE013);   // TICK_ACK: write clears pending
    reg  [7:0]  tick_period;
    reg  [15:0] tick_cnt;
    reg         irq_pending;
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
    reg [7:0] bram_qa;
    reg in_bram_q, is_rstlo_q,is_rsthi_q,is_irqlo_q,is_irqhi_q,is_nmilo_q,is_nmihi_q,is_count_q,is_coreid_q;
    always @(posedge clk) begin
        bram_qa    <= bram[AB[12:0]];   // port A read
        in_bram_q  <= in_bram;
        is_rstlo_q <= is_rstlo; is_rsthi_q <= is_rsthi;
        is_irqlo_q <= is_irqlo; is_irqhi_q <= is_irqhi;
        is_nmilo_q <= is_nmilo; is_nmihi_q <= is_nmihi;
        is_count_q <= is_count;
        is_coreid_q <= is_coreid;
    end
    assign DI = is_rstlo_q ? 8'h00 : is_rsthi_q ? 8'h10
              : is_irqlo_q ? 8'h00 : is_irqhi_q ? 8'h1F
              : is_nmilo_q ? 8'h40 : is_nmihi_q ? 8'h1F
              : is_count_q  ? task_count
              : is_coreid_q ? CORE_ID
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
