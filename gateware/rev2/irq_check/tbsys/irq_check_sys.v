// irq_check system sim: Arlet 6502 runs the real slot ROM (irq_rom.mem)
// against two real irq_check cards in slots 4 and 5, with a fake //e
// monitor (tbmon.S). Each card sits behind a model of U12 (OE + DIR).
// Reset runs PR#4 then PR#5; COUT output is printed to the log.
// Run: make irqsyssim
`timescale 1ns/1ps
module irq_check_sys;
    reg clk = 0;
    always #5 clk = ~clk;

    // 6502 clock: one Apple cycle = 978 ns, PHI0 low first, high second.
    reg cpu_clk = 0;
    reg phi0 = 0;
    always begin
        #1  cpu_clk = 1;            // new cycle: AB changes now
        #488 phi0 = 1;
        #489 phi0 = 0; cpu_clk = 0;
    end

    reg reset = 1;
    wire [15:0] AB;
    wire [7:0]  DO;
    wire        WE;
    reg  [7:0]  DI;
    wire rnw = ~WE;

    tri1 nIRQ, nNMI, nINH, nDMA, nRES;
    wire [7:0] D;                   // Apple data bus

    cpu cpu6502 (.clk(cpu_clk), .reset(reset), .AB(AB), .DI(DI), .DO(DO),
                 .WE(WE), .IRQ(~nIRQ), .NMI(~nNMI), .RDY(1'b1));

    // ---- motherboard memory ----
    reg [7:0] ram [0:16'hBFFF];
    reg [7:0] mon [0:2047];
    integer i;
    initial begin
        for (i = 0; i < 16'hC000; i = i + 1) ram[i] = 8'h00;
        $readmemh("tbmon.mem", mon);
    end

    wire io4  = AB[15:4] == 12'hC0C;
    wire io5  = AB[15:4] == 12'hC0D;
    wire rom4 = AB[15:8] == 8'hC4;
    wire rom5 = AB[15:8] == 8'hC5;
    wire card = io4 | io5 | rom4 | rom5;

    reg [7:0] mb_data;
    always @(*) begin
        if (AB < 16'hC000)       mb_data = ram[AB];
        else if (AB >= 16'hF800) mb_data = mon[AB - 16'hF800];
        else                     mb_data = 8'hFF;  // empty slots / I/O
    end

    // Motherboard drives reads it owns, CPU drives writes in PHI0 high
    assign D = (rnw && !card) ? mb_data :
               (!rnw && phi0) ? DO : 8'bZ;

    // Registered read (Arlet wants sync DI): sample D at end of PHI0
    // high, hand it to the CPU just after its clock edge (sync RAM).
    reg [7:0] d_lat;
    always @(negedge phi0) d_lat <= D;
    always @(posedge cpu_clk) DI <= d_lat;

    reg [7:0] line [0:79];
    integer lp = 0;
    always @(negedge phi0) if (!rnw && !reset) begin
        if (AB < 16'hC000) ram[AB] <= DO;
        if (AB == 16'hBFF0) begin
            if (DO == 8'h8D) $write("\n");
            else $write("%c", DO & 8'h7F);
            $fflush;
        end
        if (AB == 16'hBFF1) begin
            $display("\n=== sim end @ %0t us ===", $time / 1000);
            $finish;
        end
    end

    // ---- two cards, each behind a U12 model ----
    wire nds4 = ~(phi0 && io4), nis4 = ~(phi0 && rom4);
    wire nds5 = ~(phi0 && io5), nis5 = ~(phi0 && rom5);
    wire [7:0] d4, d5;
    wire oe4, oe5;
    assign D  = (!oe4 &&  rnw) ? d4 : 8'bZ;
    assign d4 = (!oe4 && !rnw) ? D  : 8'bZ;
    assign D  = (!oe5 &&  rnw) ? d5 : 8'bZ;
    assign d5 = (!oe5 && !rnw) ? D  : 8'bZ;

    wire [7:0] g4, g5;
    wire nres_read = ~reset;

`define CARD(name, dd, nds, nis, oe, g) \
    irq_check_top name ( \
        .CLK_100MHz(clk), .SDRAM_CKE(), .SDRAM_nCS(), \
        .A0(AB[0]), .A1(AB[1]), .A2(AB[2]), .A3(AB[3]), \
        .A4(AB[4]), .A5(AB[5]), .A6(AB[6]), .A7(AB[7]), \
        .A8(AB[8]), .A9(AB[9]), .A10(AB[10]), .A11(AB[11]), \
        .A12(AB[12]), .A13(AB[13]), .A14(AB[14]), .A15(AB[15]), \
        .D0(dd[0]), .D1(dd[1]), .D2(dd[2]), .D3(dd[3]), \
        .D4(dd[4]), .D5(dd[5]), .D6(dd[6]), .D7(dd[7]), \
        .PHI0(phi0), .R_nW(rnw), .nDEVICE_SELECT(nds), \
        .nI_O_SELECT(nis), .nRES_READ(nres_read), \
        .DMA_OUT(1'b1), .INT_OUT(1'b1), \
        .nIRQ(nIRQ), .nNMI(nNMI), .nINH(nINH), .nDMA(nDMA), .nRES(nRES), \
        .DMA_IN(), .INT_IN(), .DATA_OE(oe), \
        .GPIO_1(g[0]), .GPIO_2(g[1]), .GPIO_3(g[2]), .GPIO_4(g[3]) );

    `CARD(card4, d4, nds4, nis4, oe4, g4)
    `CARD(card5, d5, nds5, nis5, oe5, g5)

    // Bus fight check: only one driver on D during a read
    always @(posedge phi0) #400
        if (rnw && ((!oe4 && card && !(io4|rom4)) || (!oe5 && card && !(io5|rom5))))
            $display("FAIL: wrong card enabled @%0t", $time);

    // Heartbeat every 100k cycles (~98 ms) to spot a hung CPU
    integer ncyc = 0;
    always @(posedge cpu_clk) begin
        ncyc = ncyc + 1;
        if ($test$plusargs("trace") && ncyc < 120)
            $display("c%0d AB=%04h WE=%b DO=%02h DI=%02h rst=%b", ncyc, AB, WE, DO, DI, reset);
        if ($test$plusargs("trace") && ncyc == 120) $finish;
        if (ncyc % 100000 == 0) begin
            $display("[hb %0d ms] AB=%04h c4 st=%0d sh=%0d  c5 st=%0d sh=%0d",
                $time / 1000000, AB, card4.state, card4.shots,
                card5.state, card5.shots);
            $fflush;
        end
    end

    initial begin
        #5000 reset = 0;
        #3000000000;
        $display("\nFAIL: timeout");
        $finish;
    end
endmodule
