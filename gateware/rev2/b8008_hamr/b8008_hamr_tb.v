// =============================================================================
// b8008_hamr_tb — gateware verification for the 8008 coprocessor card.
//
// Exercises, in order (test ROM = b8008_rom.mem, see design dir generator):
//   1. POR -> auto-start -> bootstrap RST 0 jam -> bootstrap_done
//   2. Checkpoint latch: OUT 31 x2 -> $C0C3 == $42, $C0C4 == 2
//   3. TX FIFO: ROM emits "8008" via OUT 9 -> pop 4 bytes from $C0C1
//   4. RX path: push 'A' at $C0C2 -> ROM polls IN 1, echoes -> TX 'A',
//      checkpoint 3 ($77), then HLT -> CPU_STAT halted bit
//   5. CONTROL: 'R' (hold reset, counters clear) then 'G' -> full re-boot,
//      checkpoints reach 2 again
//   6. ID register == $B8
//
// 6502 bus timing modeled as ~500 ns nDEVICE_SELECT low (PHI0 high period).
// =============================================================================
`timescale 1ns/1ps

module b8008_hamr_tb;

    reg clk100 = 0;
    always #5 clk100 = ~clk100;   // 100 MHz

    // Apple bus
    reg  [15:0] addr = 16'hC0C0;
    reg         r_nw = 1'b1;
    reg         nds  = 1'b1;
    reg         nios = 1'b1;
    reg         nres_read = 1'b1;
    reg  [7:0]  d_drive = 8'h00;
    reg         d_oe = 1'b0;

    wire D0,D1,D2,D3,D4,D5,D6,D7;
    assign D0 = d_oe ? d_drive[0] : 1'bZ;
    assign D1 = d_oe ? d_drive[1] : 1'bZ;
    assign D2 = d_oe ? d_drive[2] : 1'bZ;
    assign D3 = d_oe ? d_drive[3] : 1'bZ;
    assign D4 = d_oe ? d_drive[4] : 1'bZ;
    assign D5 = d_oe ? d_drive[5] : 1'bZ;
    assign D6 = d_oe ? d_drive[6] : 1'bZ;
    assign D7 = d_oe ? d_drive[7] : 1'bZ;
    wire [7:0] d_bus = {D7,D6,D5,D4,D3,D2,D1,D0};

    b8008_hamr_top #(.AUTO_START_CYCLES(17'd500),
                     .ROM_INIT_FILE("b8008_testrom.mem")) dut (
        .CLK_100MHz(clk100),
        .SDRAM_CLK(), .SDRAM_CKE(), .SDRAM_nCS(), .SDRAM_nRAS(), .SDRAM_nCAS(),
        .SDRAM_nWE(), .SDRAM_DQM0(), .SDRAM_DQM1(), .SDRAM_BA0(), .SDRAM_BA1(),
        .SDRAM_A0(), .SDRAM_A1(), .SDRAM_A2(), .SDRAM_A3(), .SDRAM_A4(),
        .SDRAM_A5(), .SDRAM_A6(), .SDRAM_A7(), .SDRAM_A8(), .SDRAM_A9(),
        .SDRAM_A10(), .SDRAM_A11(), .SDRAM_A12(),
        .SDRAM_D0(), .SDRAM_D1(), .SDRAM_D2(), .SDRAM_D3(), .SDRAM_D4(),
        .SDRAM_D5(), .SDRAM_D6(), .SDRAM_D7(), .SDRAM_D8(), .SDRAM_D9(),
        .SDRAM_D10(), .SDRAM_D11(), .SDRAM_D12(), .SDRAM_D13(), .SDRAM_D14(),
        .SDRAM_D15(),
        .FLASH_nCS(), .FLASH_MOSI(), .FLASH_MISO(1'b0), .FLASH_nWP(), .FLASH_nHOLD(),
        .A0(addr[0]), .A1(addr[1]), .A2(addr[2]), .A3(addr[3]),
        .A4(addr[4]), .A5(addr[5]), .A6(addr[6]), .A7(addr[7]),
        .A8(addr[8]), .A9(addr[9]), .A10(addr[10]), .A11(addr[11]),
        .A12(addr[12]), .A13(addr[13]), .A14(addr[14]), .A15(addr[15]),
        .D0(D0), .D1(D1), .D2(D2), .D3(D3), .D4(D4), .D5(D5), .D6(D6), .D7(D7),
        .PHI0(1'b0), .PHI1(1'b1), .sig_7M(1'b0), .Q3(1'b0), .uSync(1'b0),
        .R_nW(r_nw), .nDEVICE_SELECT(nds),
        .nI_O_SELECT(nios), .nI_O_STROBE(1'b1),
        .DMA_OUT(1'b1), .INT_OUT(1'b1), .RDY(1'b1),
        .nRES_READ(nres_read),
        .nIRQ(), .nNMI(), .nINH(), .nDMA(), .nRES(), .DMA_IN(), .INT_IN(),
        .DATA_OE(),
        .GPIO_1(), .GPIO_2(), .GPIO_3(), .GPIO_4(), .GPIO_5(),
        .GPIO_6(), .GPIO_7(), .GPIO_8(), .GPIO_9(), .GPIO_10(),
        .GPIO_11(), .GPIO_12(), .GPIO_13(), .GPIO_14(), .GPIO_15(),
        .GPIO_16(), .GPIO_17(), .GPIO_18(), .GPIO_19(), .GPIO_20()
    );

    // =========================================================================
    // Second DUT: real b8008_monitor firmware ROM (the synthesis default).
    // Boots the monitor, checks the banner lands in the TX FIFO, then sends
    // "G 1000" — RAM is $00-filled (= HLT), so a parsed command + trampoline
    // shows up as a clean CPU halt.
    // =========================================================================
    reg  [15:0] maddr = 16'hC0C0;
    reg         mr_nw = 1'b1;
    reg         mnds  = 1'b1;
    reg  [7:0]  md_drive = 8'h00;
    reg         md_oe = 1'b0;

    wire MD0,MD1,MD2,MD3,MD4,MD5,MD6,MD7;
    assign MD0 = md_oe ? md_drive[0] : 1'bZ;
    assign MD1 = md_oe ? md_drive[1] : 1'bZ;
    assign MD2 = md_oe ? md_drive[2] : 1'bZ;
    assign MD3 = md_oe ? md_drive[3] : 1'bZ;
    assign MD4 = md_oe ? md_drive[4] : 1'bZ;
    assign MD5 = md_oe ? md_drive[5] : 1'bZ;
    assign MD6 = md_oe ? md_drive[6] : 1'bZ;
    assign MD7 = md_oe ? md_drive[7] : 1'bZ;
    wire [7:0] md_bus = {MD7,MD6,MD5,MD4,MD3,MD2,MD1,MD0};

    b8008_hamr_top #(.AUTO_START_CYCLES(17'd500)) dut_mon (
        .CLK_100MHz(clk100),
        .SDRAM_CLK(), .SDRAM_CKE(), .SDRAM_nCS(), .SDRAM_nRAS(), .SDRAM_nCAS(),
        .SDRAM_nWE(), .SDRAM_DQM0(), .SDRAM_DQM1(), .SDRAM_BA0(), .SDRAM_BA1(),
        .SDRAM_A0(), .SDRAM_A1(), .SDRAM_A2(), .SDRAM_A3(), .SDRAM_A4(),
        .SDRAM_A5(), .SDRAM_A6(), .SDRAM_A7(), .SDRAM_A8(), .SDRAM_A9(),
        .SDRAM_A10(), .SDRAM_A11(), .SDRAM_A12(),
        .SDRAM_D0(), .SDRAM_D1(), .SDRAM_D2(), .SDRAM_D3(), .SDRAM_D4(),
        .SDRAM_D5(), .SDRAM_D6(), .SDRAM_D7(), .SDRAM_D8(), .SDRAM_D9(),
        .SDRAM_D10(), .SDRAM_D11(), .SDRAM_D12(), .SDRAM_D13(), .SDRAM_D14(),
        .SDRAM_D15(),
        .FLASH_nCS(), .FLASH_MOSI(), .FLASH_MISO(1'b0), .FLASH_nWP(), .FLASH_nHOLD(),
        .A0(maddr[0]), .A1(maddr[1]), .A2(maddr[2]), .A3(maddr[3]),
        .A4(maddr[4]), .A5(maddr[5]), .A6(maddr[6]), .A7(maddr[7]),
        .A8(maddr[8]), .A9(maddr[9]), .A10(maddr[10]), .A11(maddr[11]),
        .A12(maddr[12]), .A13(maddr[13]), .A14(maddr[14]), .A15(maddr[15]),
        .D0(MD0), .D1(MD1), .D2(MD2), .D3(MD3), .D4(MD4), .D5(MD5), .D6(MD6), .D7(MD7),
        .PHI0(1'b0), .PHI1(1'b1), .sig_7M(1'b0), .Q3(1'b0), .uSync(1'b0),
        .R_nW(mr_nw), .nDEVICE_SELECT(mnds),
        .nI_O_SELECT(1'b1), .nI_O_STROBE(1'b1),
        .DMA_OUT(1'b1), .INT_OUT(1'b1), .RDY(1'b1),
        .nRES_READ(nres_read),
        .nIRQ(), .nNMI(), .nINH(), .nDMA(), .nRES(), .DMA_IN(), .INT_IN(),
        .DATA_OE(),
        .GPIO_1(), .GPIO_2(), .GPIO_3(), .GPIO_4(), .GPIO_5(),
        .GPIO_6(), .GPIO_7(), .GPIO_8(), .GPIO_9(), .GPIO_10(),
        .GPIO_11(), .GPIO_12(), .GPIO_13(), .GPIO_14(), .GPIO_15(),
        .GPIO_16(), .GPIO_17(), .GPIO_18(), .GPIO_19(), .GPIO_20()
    );

    task mon_read(input [3:0] a, output [7:0] val);
        begin
            maddr = {12'hC0C, a};
            mr_nw = 1'b1;
            #100 mnds = 1'b0;
            #450 val = md_bus;
            #50  mnds = 1'b1;
            #200;
        end
    endtask

    task mon_write(input [3:0] a, input [7:0] val);
        begin
            maddr = {12'hC0C, a};
            mr_nw = 1'b0;
            md_drive = val; md_oe = 1'b1;
            #100 mnds = 1'b0;
            #500 mnds = 1'b1;
            #50  md_oe = 1'b0; mr_nw = 1'b1;
            #200;
        end
    endtask

    // Slot ROM golden copy for the $C4xx read check
    reg [7:0] slot_golden [0:255];
    initial $readmemh("b8008_slot.mem", slot_golden);

    task ios_read(input [7:0] a, output [7:0] val);
        begin
            addr = {8'hC4, a};
            r_nw = 1'b1;
            #100 nios = 1'b0;
            #450 val = d_bus;
            #50  nios = 1'b1;
            #200;
        end
    endtask

    integer errors = 0;

    // ---- 6502-shaped bus cycles: ~500 ns select low ----
    task dev_read(input [3:0] a, output [7:0] val);
        begin
            addr = {12'hC0C, a};
            r_nw = 1'b1;
            #100 nds = 1'b0;
            #450 val = d_bus;          // sample late in the low phase
            #50  nds = 1'b1;
            #200;
        end
    endtask

    task dev_write(input [3:0] a, input [7:0] val);
        begin
            addr = {12'hC0C, a};
            r_nw = 1'b0;
            d_drive = val; d_oe = 1'b1;
            #100 nds = 1'b0;
            #500 nds = 1'b1;           // commit on rising edge
            #50  d_oe = 1'b0; r_nw = 1'b1;
            #200;
        end
    endtask

    task expect8(input [3:0] a, input [7:0] want, input [127:0] name);
        reg [7:0] got;
        begin
            dev_read(a, got);
            if (got !== want) begin
                $display("FAIL %0s: reg $C0C%0h got %02x want %02x (t=%0t)",
                         name, a, got, want, $time);
                errors = errors + 1;
            end else
                $display("  ok %0s = %02x", name, got);
        end
    endtask

    // Wait until checkpoint count reaches N (poll $C0C4) with timeout
    task wait_chkpt(input [7:0] n, input integer timeout_us);
        reg [7:0] got;
        integer waited;
        begin
            waited = 0;
            got = 8'h00;
            while ((got < n) && (waited < timeout_us)) begin
                #1000;                 // 1 us between polls
                waited = waited + 1;
                if (waited % 50 == 0) dev_read(4'h4, got);
            end
            if (got < n) begin
                $display("FAIL: timeout waiting for checkpoint %0d (got %0d, pc=%04x)",
                         n, got, dut.debug_pc);
                errors = errors + 1;
            end
        end
    endtask

    reg [7:0] v;
    reg [7:0] tx0, tx1, tx2, tx3;

    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("b8008_hamr_tb.vcd");
            $dumpvars(0, b8008_hamr_tb);
        end

        // --- 1. Boot: POR, auto-start (500 cycles = 20 us), bootstrap jam ---
        #5000;
        expect8(4'hF, 8'hB8, "ID");

        // --- 1b. Slot ROM: first 4 firmware bytes readable at $C400 ---
        begin : slot_check
            reg [7:0] sv;
            integer si, sbad;
            sbad = 0;
            for (si = 0; si < 4; si = si + 1) begin
                ios_read(si[7:0], sv);
                if (sv !== slot_golden[si]) begin
                    $display("FAIL slot ROM $C4%02x: got %02x want %02x",
                             si, sv, slot_golden[si]);
                    sbad = sbad + 1;
                end
            end
            if (sbad) errors = errors + sbad;
            else $display("  ok slot ROM $C400-3 = firmware bytes");
        end

        // bootstrap_done within 500 us of start
        fork : boot_wait
            begin
                wait (dut.bootstrap_done === 1'b1);
                $display("  ok bootstrap_done at t=%0t", $time);
                disable boot_wait;
            end
            begin
                #500000;
                $display("FAIL: bootstrap never completed");
                errors = errors + 1;
                disable boot_wait;
            end
        join

        // --- 2. Checkpoints 1+2 ---
        wait_chkpt(8'd2, 2000);
        expect8(4'h3, 8'h42, "CHKPT val");
        expect8(4'h4, 8'h02, "CHKPT cnt");

        // --- 3. TX FIFO: "8008" ---
        // ROM emits 4 bytes right after CP2; give it 1 ms of 8008 time
        #1000000;
        dev_read(4'h0, v);
        if (v !== 8'h84) begin        // avail=1, count=4
            $display("FAIL TX_STAT: got %02x want 84", v);
            errors = errors + 1;
        end else $display("  ok TX_STAT = %02x", v);
        dev_read(4'h1, tx0); dev_read(4'h1, tx1);
        dev_read(4'h1, tx2); dev_read(4'h1, tx3);
        if ({tx0,tx1,tx2,tx3} !== 32'h38303038) begin
            $display("FAIL TX data: got %c%c%c%c want 8008", tx0,tx1,tx2,tx3);
            errors = errors + 1;
        end else $display("  ok TX data = %c%c%c%c", tx0,tx1,tx2,tx3);
        expect8(4'h0, 8'h00, "TX empty after pops");

        // --- 4. RX path: push 'A', ROM echoes, CP3, HLT ---
        dev_write(4'h2, 8'h41);
        wait_chkpt(8'd3, 3000);
        expect8(4'h3, 8'h77, "CHKPT3 val");
        #200000;                       // let HLT settle
        dev_read(4'h1, v);
        if (v !== 8'h41) begin
            $display("FAIL echo: got %02x want 41", v);
            errors = errors + 1;
        end else $display("  ok echo = %02x", v);
        dev_read(4'h5, v);
        if (v[7] !== 1'b1 || v[2] !== 1'b1) begin
            $display("FAIL CPU_STAT: got %02x want bit7(boot)=1 bit2(halt)=1", v);
            errors = errors + 1;
        end else $display("  ok CPU_STAT = %02x (halted)", v);

        // --- 5. CONTROL: reset-hold then go -> full reboot ---
        dev_write(4'h5, 8'h52);        // 'R'
        #10000;
        expect8(4'h4, 8'h00, "CHKPT cnt cleared by R");
        expect8(4'h0, 8'h00, "TX flushed by R");
        dev_write(4'h5, 8'h47);        // 'G'
        wait_chkpt(8'd2, 3000);
        expect8(4'h3, 8'h42, "CHKPT val after reboot");
        $display("  ok reboot via R/G");

        // =====================================================================
        // Monitor firmware DUT: banner, prompt, G command -> halt
        // =====================================================================
        $display("--- monitor firmware DUT ---");
        begin : mon_test
            reg [7:0] mv;
            reg [8*16-1:0] banner;
            integer i, waited;

            // Banner "8008 Monitor\r\n> " = 16 bytes; char_delay paces at
            // ~87 us/char of 8008 time, so allow generous wall
            waited = 0;
            mv = 8'h00;
            while (((mv & 8'h7F) < 16) && (waited < 550000)) begin   // count bits only (bit7 = avail flag); startup delay ~330ms
                #1000; waited = waited + 1;
                if (waited % 100 == 0) mon_read(4'h0, mv);   // TX count
            end
            if ((mv & 8'h7F) < 16) begin
                $display("FAIL monitor: banner never arrived (TX count %0d, pc=%04x)",
                         mv, dut_mon.debug_pc);
                errors = errors + 1;
            end else begin
                for (i = 0; i < 16; i = i + 1) begin
                    mon_read(4'h1, mv);
                    banner = {banner[8*15-1:0], mv};
                end
                if (banner !== {"8008 Monitor", 8'h0D, 8'h0A, "> "}) begin
                    $display("FAIL monitor banner: got %s", banner);
                    errors = errors + 1;
                end else
                    $display("  ok monitor banner + prompt");
            end

            // "G 1000\r" -> trampoline into $00-filled RAM -> HLT
            mon_write(4'h2, "G"); mon_write(4'h2, " ");
            mon_write(4'h2, "1"); mon_write(4'h2, "0");
            mon_write(4'h2, "0"); mon_write(4'h2, "0");
            mon_write(4'h2, 8'h0D);
            waited = 0; mv = 8'h00;
            while ((mv[2] !== 1'b1) && (waited < 100000)) begin
                #1000; waited = waited + 1;
                if (waited % 100 == 0) mon_read(4'h5, mv);   // CPU_STAT
            end
            if (mv[2] !== 1'b1) begin
                $display("FAIL monitor G: never halted (CPU_STAT %02x, pc=%04x)",
                         mv, dut_mon.debug_pc);
                errors = errors + 1;
            end else
                $display("  ok monitor G 1000 -> HLT (CPU_STAT %02x)", mv);
        end

        // --- verdict ---
        if (errors == 0) $display("=== b8008_hamr_tb PASS ===");
        else             $display("=== b8008_hamr_tb FAIL (%0d errors) ===", errors);
        $finish;
    end

    // Global watchdog
    initial begin
        #900000000;                    // 900 ms — monitor startup delay is ~330ms of 8008 time
        $display("FAIL: global watchdog expired");
        $finish;
    end

endmodule
