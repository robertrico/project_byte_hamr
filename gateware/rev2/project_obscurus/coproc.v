// =============================================================================
// coproc.v — Arlet soft-6502 + 8KB BRAM + $E000 SDRAM-write window.
// C0 steel thread: runs coproc_prog.mem (LDA #$42 / STA $E000 / spin). A write
// to $E000 posts ONE SDRAM write (bank0 $0040 = DO) to the arbiter client and
// RDY-stalls the core until the op completes. Reset vector is synthesized
// ($FFFC -> $00, $FFFD -> $02, i.e. PC=$0200). Arlet RDY = global clock-enable;
// reset active-high.
//
// MEMORY TIMING (validated against arlet_cpu.v — see report):
//   Arlet presents AB combinationally (always @* address generator) but expects
//   SYNCHRONOUS memory: DI must reflect mem[AB] ONE clock LATER. The core's
//   internal pipeline (ABL/ABH registering at line ~417, DIHOLD <= DI / assign
//   DIMUX = ~RDY ? DIHOLD : DI at lines ~855/857) is built for a 1-cycle read
//   latency. A purely COMBINATIONAL same-cycle DI feed (DI = mem[AB]) is WRONG:
//   it boots to the wrong PC AND creates a zero-delay AB->mem->DI->AB delta loop
//   that HANGS iverilog at the JMP1 reset state whose AB = {DIMUX, ADD} is
//   self-referential. The fix is a registered (posedge) BRAM read plus
//   registered vector/region selects, so DI[n] = decode(AB[n-1]). With that, the
//   core boots cleanly to $0200 and executes LDA/STA. The DIHOLD path freezes DI
//   during RDY=0 stalls so the in-flight read survives the SDRAM wait.
// =============================================================================
module coproc (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        ready,         // SDRAM init complete — gate core start
    output reg         req,
    output reg         we,
    output reg  [25:0] phys_addr,
    output reg  [7:0]  wdata,
    input  wire        busy,
    input  wire [7:0]  rdata          // unused in C0
);
    wire [15:0] AB;
    wire [7:0]  DO;
    wire        WE;
    wire [7:0]  DI;
    reg         rdy;

    cpu u_cpu (
        .clk(clk), .reset(~rst_n),
        .AB(AB), .DI(DI), .DO(DO), .WE(WE),
        .IRQ(1'b0), .NMI(1'b0), .RDY(rdy)
    );

    reg [7:0] bram [0:8191];
    initial $readmemh("coproc_prog.mem", bram);

    wire in_bram = (AB[15:13] == 3'b000);   // $0000-$1FFF
    wire is_vlo  = (AB == 16'hFFFC);
    wire is_vhi  = (AB == 16'hFFFD);

    // Synchronous BRAM: registered read (1-cycle latency, as Arlet expects) and
    // registered write (gated by RDY so a stalled re-driven WE never double-
    // writes). Vector/region selects registered to match the read latency.
    reg [7:0] bram_q;
    reg       in_bram_q, is_vlo_q, is_vhi_q;
    always @(posedge clk) begin
        if (WE & in_bram & rdy) bram[AB[12:0]] <= DO;
        bram_q    <= bram[AB[12:0]];
        in_bram_q <= in_bram;
        is_vlo_q  <= is_vlo;
        is_vhi_q  <= is_vhi;
    end

    assign DI = is_vlo_q  ? 8'h00       // reset vector lo -> $0200
              : is_vhi_q  ? 8'h02       // reset vector hi
              : in_bram_q ? bram_q
              :             8'h00;

    wire is_e000 = (AB == 16'hE000);
    reg busy_d;
    always @(posedge clk) busy_d <= busy;
    wire done = busy_d & ~busy;

    // ST_BOOT: hold the core frozen (rdy=0) out of reset until SDRAM `ready`.
    // Without this the Arlet core executes STA $E000 within ~10 cycles of reset
    // — long before the controller's ~200us init completes. The arbiter would
    // accept that early request and commit `servicing`, but sdram_ctrl gates
    // accept on `ready` and silently drops it, so the controller never raises
    // busy and the arbiter waits forever for an op_complete that never comes —
    // a permanent deadlock that also starves the monitor (c0). Gating the core
    // start on `ready` guarantees the write is issued into a live controller.
    localparam ST_BOOT=2'd0, ST_RUN=2'd1, ST_WAIT=2'd2;
    reg [1:0] state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            req<=1'b0; we<=1'b0; phys_addr<=26'd0; wdata<=8'd0;
            rdy<=1'b0; state<=ST_BOOT;
        end else begin
            req <= 1'b0;
            case (state)
                ST_BOOT: if (ready) begin
                    rdy<=1'b1; state<=ST_RUN;     // release core once SDRAM live
                end
                ST_RUN: if (is_e000 & WE) begin
                    we<=1'b1; phys_addr<=26'h000_0040; wdata<=DO;
                    req<=1'b1; rdy<=1'b0; state<=ST_WAIT;
                end
                ST_WAIT: if (done) begin
                    rdy<=1'b1; state<=ST_RUN;
                end
                default: state<=ST_BOOT;
            endcase
        end
    end
endmodule
