// irq_check testbench: Apple IIe bus model + simple 6502 interrupt model.
// PHI0 ~1.02 MHz (489 ns high / 489 ns low). Address and R/W change
// ~100 ns into PHI0 low; write data valid ~100 ns into PHI0 high.
// Card in slot 4 ($C0C0).
`timescale 1ns/1ps
module irq_check_tb;
    reg clk = 0;
    always #5 clk = ~clk;

    reg        phi0 = 0;
    reg [15:0] addr = 16'h0300;
    reg        rnw  = 1'b1;
    reg [7:0]  wdata = 8'h00;
    reg        wdrive = 1'b0;

    wire [7:0] D;
    assign D = wdrive ? wdata : 8'bZ;

    wire ndevsel = ~(phi0 && addr[15:4] == 12'hC0C);

    tri1 nIRQ, nNMI, nINH, nDMA, nRES;
    wire DATA_OE, DMA_IN, INT_IN, G1, G2, G3, G4, cke, ncs;

    irq_check_top dut (
        .CLK_100MHz(clk), .SDRAM_CKE(cke), .SDRAM_nCS(ncs),
        .A0(addr[0]), .A1(addr[1]), .A2(addr[2]), .A3(addr[3]),
        .A4(addr[4]), .A5(addr[5]), .A6(addr[6]), .A7(addr[7]),
        .A8(addr[8]), .A9(addr[9]), .A10(addr[10]), .A11(addr[11]),
        .A12(addr[12]), .A13(addr[13]), .A14(addr[14]), .A15(addr[15]),
        .D0(D[0]), .D1(D[1]), .D2(D[2]), .D3(D[3]),
        .D4(D[4]), .D5(D[5]), .D6(D[6]), .D7(D[7]),
        .PHI0(phi0), .R_nW(rnw), .nDEVICE_SELECT(ndevsel),
        .nI_O_SELECT(1'b1), .nRES_READ(1'b1),
        .DMA_OUT(1'b1), .INT_OUT(1'b1),
        .nIRQ(nIRQ), .nNMI(nNMI), .nINH(nINH), .nDMA(nDMA), .nRES(nRES),
        .DMA_IN(DMA_IN), .INT_IN(INT_IN), .DATA_OE(DATA_OE),
        .GPIO_1(G1), .GPIO_2(G2), .GPIO_3(G3), .GPIO_4(G4)
    );

    // One bus cycle. Returns data seen on D late in PHI0 high.
    reg [7:0] rd;
    task bus(input [15:0] a, input r, input [7:0] wd);
        begin
            #100 addr = a; rnw = r; wdata = wd;
            #389 phi0 = 1;
            if (!r) begin #100 wdrive = 1; #350; end else #450;
            rd = D;
            #39 phi0 = 0; wdrive = 0;
        end
    endtask

    // 6502 model. irq_en = I flag clear. Interrupt line sampled at end of
    // each "instruction" (every ilen cycles), then 7-cycle sequence with
    // vector fetch on cycles 6/7. respond=0 ignores the line (miss test).
    reg irq_en = 0;
    reg respond = 1;
    reg nmi_prev = 1;
    reg nmi_pend = 0;
    integer ilen = 4, icnt = 0;

    task cpu_cycle;
        begin
            if (nNMI == 1'b0 && nmi_prev == 1'b1) nmi_pend = 1;
            nmi_prev = nNMI;
            icnt = icnt + 1;
            if (icnt >= ilen && respond && (nmi_pend || (irq_en && !nIRQ))) begin
                icnt = 0;
                bus(16'h0306, 1, 0); bus(16'h0306, 1, 0);
                bus(16'h01FF, 0, 8'h03); bus(16'h01FE, 0, 8'h06);
                bus(16'h01FD, 0, 8'h20);
                if (nmi_pend) begin
                    nmi_pend = 0;
                    bus(16'hFFFA, 1, 0); bus(16'hFFFB, 1, 0);
                end else begin
                    bus(16'hFFFE, 1, 0); bus(16'hFFFF, 1, 0);
                end
                bus(16'h0320, 1, 0);    // handler
            end else begin
                if (icnt >= ilen) icnt = 0;
                bus(16'h0306, 1, 0);
            end
        end
    endtask


    integer errors = 0;
    task check(input [3:0] reg_off, input [7:0] exp, input [8*16-1:0] name);
        begin
            bus(16'hC0C0 | reg_off, 1, 0);
            if (rd !== exp) begin
                $display("FAIL %0s: got %02h exp %02h", name, rd, exp);
                errors = errors + 1;
            end else
                $display("ok   %0s = %02h", name, rd);
        end
    endtask

    task show(input [3:0] reg_off, input [8*16-1:0] name);
        begin
            bus(16'hC0C0 | reg_off, 1, 0);
            $display("     %0s = %02h", name, rd);
        end
    endtask

    task wait_done;
        integer n;
        begin
            n = 0;
            rd = 0;
            while (!rd[7] && n < 300000) begin
                repeat (20) cpu_cycle;
                bus(16'hC0C0, 1, 0);
                n = n + 21;
            end
            if (!rd[7]) begin
                $display("FAIL: done never set");
                errors = errors + 1;
            end
        end
    endtask

    // DATA_OE must only enable during our DEVICE SELECT
    always @(negedge DATA_OE)
        if (ndevsel !== 1'b0) begin
            $display("FAIL: DATA_OE low outside DEVICE SELECT @%0t", $time);
            errors = errors + 1;
        end

    initial begin
        repeat (10) bus(16'h0300, 1, 0);

        $display("--- ID / idle ---");
        check(7, 8'hA5, "ID");
        check(0, 8'h00, "status");

        $display("--- IRQ test, CPU responds ---");
        irq_en = 1; respond = 1;
        bus(16'hC0C0, 0, 8'h01);
        wait_done;
        check(0, 8'h81, "status");
        check(1, 8'd100, "taken");
        check(2, 8'd0, "missed");
        check(6, 8'd100, "shots");
        check(5, 8'd0, "spurious");
        show(3, "lat_min");
        show(4, "lat_max");

        $display("--- IRQ test, I flag set (all miss) ---");
        irq_en = 0;
        bus(16'hC0C0, 0, 8'h01);
        wait_done;
        check(1, 8'd0, "taken");
        check(2, 8'd100, "missed");
        check(6, 8'd100, "shots");

        $display("--- NMI test ---");
        bus(16'hC0C0, 0, 8'h02);
        wait_done;
        check(0, 8'h82, "status");
        check(1, 8'd100, "taken");
        check(2, 8'd0, "missed");
        show(3, "lat_min");
        show(4, "lat_max");

        $display("--- clear ---");
        bus(16'hC0C0, 0, 8'h00);
        check(0, 8'h00, "status");
        check(1, 8'd0, "taken");
        check(3, 8'hFF, "lat_min");

        $display("--- spurious (foreign vector fetch) ---");
        bus(16'hFFFE, 1, 0); bus(16'hFFFF, 1, 0);
        bus(16'hFFFA, 1, 0); bus(16'hFFFB, 1, 0);
        check(5, 8'd2, "spurious");

        $display("--- scope mode: pulse width ---");
        irq_en = 0;
        bus(16'hC0C0, 0, 8'h03);
        fork
            begin : scope
                time t0, t1;
                @(posedge G1) t0 = $time;
                @(negedge G1) t1 = $time;
                $display("     IRQ pulse %0d ns", t1 - t0);
                if (t1 - t0 < 900 || t1 - t0 > 1100) begin
                    $display("FAIL: scope pulse width");
                    errors = errors + 1;
                end
                @(posedge G1) $display("     period %0d us", ($time - t0) / 1000);
            end
            repeat (21000) bus(16'h0306, 1, 0);
        join
        bus(16'hC0C0, 0, 8'h00);

        if (errors == 0) $display("PASS");
        else $display("FAILED: %0d errors", errors);
        $finish;
    end

endmodule
