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
module coproc (
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
               .IRQ(1'b0), .NMI(1'b0), .RDY(rdy));

    reg [7:0] task_count;
    always @(posedge clk or negedge rst_n)
        if (!rst_n)        task_count <= 8'd0;
        else if (count_wr) task_count <= count_in;

    reg [7:0] bram [0:8191];
    integer gi;
    initial begin
        for (gi=0; gi<8192; gi=gi+1) bram[gi] = 8'h00;
        $readmemh("kernel.mem", bram, 13'h1000);
    end

    wire in_bram = (AB[15:13] == 3'b000);
    wire a_wr_ok = WE & in_bram & rdy & ~AB[12] & ~(AB[11:8]==4'h2);

    wire is_rstlo=(AB==16'hFFFC), is_rsthi=(AB==16'hFFFD);
    wire is_irqlo=(AB==16'hFFFE), is_irqhi=(AB==16'hFFFF);
    wire is_nmilo=(AB==16'hFFFA), is_nmihi=(AB==16'hFFFB);
    wire is_count=(AB==16'hE010);
    reg [7:0] bram_qa;
    reg in_bram_q, is_rstlo_q,is_rsthi_q,is_irqlo_q,is_irqhi_q,is_nmilo_q,is_nmihi_q,is_count_q;
    always @(posedge clk) begin
        if (a_wr_ok) bram[AB[12:0]] <= DO;
        bram_qa    <= bram[AB[12:0]];
        in_bram_q  <= in_bram;
        is_rstlo_q <= is_rstlo; is_rsthi_q <= is_rsthi;
        is_irqlo_q <= is_irqlo; is_irqhi_q <= is_irqhi;
        is_nmilo_q <= is_nmilo; is_nmihi_q <= is_nmihi;
        is_count_q <= is_count;
    end
    assign DI = is_rstlo_q ? 8'h00 : is_rsthi_q ? 8'h10
              : is_irqlo_q ? 8'h00 : is_irqhi_q ? 8'h1F
              : is_nmilo_q ? 8'h40 : is_nmihi_q ? 8'h1F
              : is_count_q ? task_count
              : in_bram_q  ? bram_qa
              :              8'h00;

    always @(posedge clk) begin
        if (lwr & ~laddr[12]) bram[laddr] <= ldata_in;
        ldata_out <= bram[laddr];
    end

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
