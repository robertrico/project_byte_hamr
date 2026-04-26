`timescale 1ns / 1ps
// =============================================================================
// Smart Hamr Testbench — Rev 2
// =============================================================================
// Same test plan as Rev 1 (T1..T7); only the DUT port mapping changed:
//   - addr[11:0]  ->  individual A0..A15
//   - GPIO1..GPIO12  ->  GPIO_1..GPIO_20 (new pad assignments per top.v)
//   - new ports: DATA_OE, nRES_READ, CLK_100MHz
//
// GPIO mapping driven into the testbench (see smart_hamr_top.v header):
//   GPIO_1  = nDEVICE_SELECT    (out, LA debug)
//   GPIO_9  = _enbl2            (out)
//   GPIO_10 = wrdata            (out)
//   GPIO_12 = rddata            (in, idle LOW from ESP32)
//   GPIO_13 = sense/ACK         (in)
//   GPIO_14 = _wrreq            (out)
//   GPIO_15 = phase[1]          (out)
//   GPIO_16 = phase[0]          (out)
//   GPIO_17 = phase[3]          (out)
//   GPIO_18 = phase[2]          (out)
//   GPIO_20 = _enbl1            (out)
// =============================================================================

module smart_hamr_tb;

    // =========================================================================
    // Parameters
    // =========================================================================
    localparam CLK_7M_PERIOD   = 140;  // 7.16 MHz ≈ 140 ns
    localparam CLK_100M_PERIOD = 10;   // 100 MHz = 10 ns (unused, just driven)
    localparam Q3_PERIOD       = 500;  // ~2 MHz

    // =========================================================================
    // Signals
    // =========================================================================
    reg         sig_7M;
    reg         CLK_100MHz;
    reg         Q3;

    // Apple II address bus — drive as a bundled vector for test convenience,
    // unpack onto A0..A15 at the DUT port.
    reg  [15:0] a_bus;
    wire [15:0] a_drv = a_bus;
    reg  [7:0]  data_out_reg;
    reg         data_drive;
    reg         R_nW;
    reg         nDEVICE_SELECT;
    reg         nI_O_SELECT;
    reg         nI_O_STROBE;
    reg         nRES_READ;             // Apple II reset — drive HIGH normally
    wire        nRES;
    wire        DATA_OE;

    // Individual data bus lines
    wire        D0, D1, D2, D3, D4, D5, D6, D7;
    wire [7:0]  data_read;

    // Unused bus signals
    reg         PHI0, PHI1, uSync, RDY;
    wire        nIRQ, nNMI, nDMA, nINH;
    wire        DMA_IN, INT_IN;
    reg         DMA_OUT, INT_OUT;

    // GPIO header
    wire        gpio_1_devsel_dbg;
    wire        gpio_2,  gpio_3,  gpio_4,  gpio_5,
                gpio_6,  gpio_7,  gpio_8;         // unused
    wire        gpio_9_enbl2;
    wire        gpio_10_wrdata;
    wire        gpio_11;                          // unused damped
    reg         gpio_12_rddata;                   // ESP32 → FPGA (idle LOW)
    reg         gpio_13_sense;                    // ESP32 → FPGA
    wire        gpio_14_wrreq;
    wire        gpio_15_phase1;
    wire        gpio_16_phase0;
    wire        gpio_17_phase3;
    wire        gpio_18_phase2;
    wire        gpio_19;                          // unused
    wire        gpio_20_enbl1;

    // Test control
    reg  [7:0]  read_data;
    integer     test_num;
    integer     errors;

    // =========================================================================
    // Data Bus Tristate Control (individual pins)
    // =========================================================================
    assign D0 = data_drive ? data_out_reg[0] : 1'bZ;
    assign D1 = data_drive ? data_out_reg[1] : 1'bZ;
    assign D2 = data_drive ? data_out_reg[2] : 1'bZ;
    assign D3 = data_drive ? data_out_reg[3] : 1'bZ;
    assign D4 = data_drive ? data_out_reg[4] : 1'bZ;
    assign D5 = data_drive ? data_out_reg[5] : 1'bZ;
    assign D6 = data_drive ? data_out_reg[6] : 1'bZ;
    assign D7 = data_drive ? data_out_reg[7] : 1'bZ;

    assign data_read = {D7, D6, D5, D4, D3, D2, D1, D0};

    // =========================================================================
    // DUT Instantiation
    // =========================================================================
    smart_hamr_top dut (
        // Address bus — unpack the test vector onto A0..A15
        .A0  (a_drv[0]),  .A1  (a_drv[1]),  .A2  (a_drv[2]),  .A3  (a_drv[3]),
        .A4  (a_drv[4]),  .A5  (a_drv[5]),  .A6  (a_drv[6]),  .A7  (a_drv[7]),
        .A8  (a_drv[8]),  .A9  (a_drv[9]),  .A10 (a_drv[10]), .A11 (a_drv[11]),
        .A12 (a_drv[12]), .A13 (a_drv[13]), .A14 (a_drv[14]), .A15 (a_drv[15]),

        // Data bus
        .D0 (D0), .D1 (D1), .D2 (D2), .D3 (D3),
        .D4 (D4), .D5 (D5), .D6 (D6), .D7 (D7),

        // Control
        .sig_7M         (sig_7M),
        .Q3             (Q3),
        .R_nW           (R_nW),
        .nDEVICE_SELECT (nDEVICE_SELECT),
        .nI_O_SELECT    (nI_O_SELECT),
        .nI_O_STROBE    (nI_O_STROBE),
        .nRES           (nRES),
        .nRES_READ      (nRES_READ),
        .RDY            (RDY),
        .nIRQ           (nIRQ),
        .nNMI           (nNMI),
        .nDMA           (nDMA),
        .nINH           (nINH),
        .DMA_OUT        (DMA_OUT),
        .DMA_IN         (DMA_IN),
        .INT_OUT        (INT_OUT),
        .INT_IN         (INT_IN),
        .PHI0           (PHI0),
        .PHI1           (PHI1),
        .uSync          (uSync),
        .DATA_OE        (DATA_OE),
        .CLK_100MHz     (CLK_100MHz),

        // GPIO header
        .GPIO_1  (gpio_1_devsel_dbg),
        .GPIO_2  (gpio_2),
        .GPIO_3  (gpio_3),
        .GPIO_4  (gpio_4),
        .GPIO_5  (gpio_5),
        .GPIO_6  (gpio_6),
        .GPIO_7  (gpio_7),
        .GPIO_8  (gpio_8),
        .GPIO_9  (gpio_9_enbl2),
        .GPIO_10 (gpio_10_wrdata),
        .GPIO_11 (gpio_11),
        .GPIO_12 (gpio_12_rddata),
        .GPIO_13 (gpio_13_sense),
        .GPIO_14 (gpio_14_wrreq),
        .GPIO_15 (gpio_15_phase1),
        .GPIO_16 (gpio_16_phase0),
        .GPIO_17 (gpio_17_phase3),
        .GPIO_18 (gpio_18_phase2),
        .GPIO_19 (gpio_19),
        .GPIO_20 (gpio_20_enbl1)
    );

    // =========================================================================
    // Clock Generation
    // =========================================================================
    initial sig_7M = 0;
    always #(CLK_7M_PERIOD/2) sig_7M = ~sig_7M;

    initial CLK_100MHz = 0;
    always #(CLK_100M_PERIOD/2) CLK_100MHz = ~CLK_100MHz;

    initial Q3 = 0;
    always #(Q3_PERIOD/2) Q3 = ~Q3;

    // =========================================================================
    // Test Tasks
    // =========================================================================
    task init;
    begin
        a_bus          = 16'h0000;
        data_out_reg   = 8'h00;
        data_drive     = 0;
        R_nW           = 1;
        nDEVICE_SELECT = 1;
        nI_O_SELECT    = 1;
        nI_O_STROBE    = 1;
        nRES_READ      = 1;       // Apple II reset released
        gpio_12_rddata = 0;       // idle LOW from ESP32
        gpio_13_sense  = 0;
        PHI0           = 0;
        PHI1           = 0;
        uSync          = 0;
        RDY            = 1;
        DMA_OUT        = 1;
        INT_OUT        = 1;
        test_num       = 0;
        errors         = 0;
    end
    endtask

    // Wait for POR + nRES_READ synchronizer to release.
    task reset;
    begin
        repeat(20) @(posedge sig_7M);
    end
    endtask

    // Write to IWM register ($C0Cx)
    // The IWM uses /DEV-clocked state latches plus a 2-FF sync into fclk
    // domain (~2 fclk latency). Hold /DEV low long enough for the latch to
    // capture, then wait extra cycles after release for the sync chain to
    // propagate before checking outputs.
    task iwm_write;
        input [3:0] reg_addr;
    begin
        a_bus = {12'h000, reg_addr};
        R_nW = 0;
        data_drive = 0;
        @(posedge sig_7M);
        nDEVICE_SELECT = 0;
        repeat(4) @(posedge sig_7M);
        nDEVICE_SELECT = 1;
        repeat(4) @(posedge sig_7M);
    end
    endtask

    // Read from IWM register ($C0Cx)
    task iwm_read;
        input [3:0] reg_addr;
        output [7:0] result;
    begin
        a_bus = {12'h000, reg_addr};
        R_nW = 1;
        data_drive = 0;
        @(posedge sig_7M);
        nDEVICE_SELECT = 0;
        repeat(2) @(posedge sig_7M);
        result = data_read;
        nDEVICE_SELECT = 1;
        repeat(2) @(posedge sig_7M);
    end
    endtask

    // Read from ROM ($C4xx or $C8xx)
    task rom_read;
        input [11:0] rom_addr;
        input        use_strobe;
        output [7:0] result;
    begin
        a_bus = {4'h0, rom_addr};
        R_nW = 1;
        data_drive = 0;
        @(posedge sig_7M);
        if (use_strobe)
            nI_O_STROBE = 0;
        else
            nI_O_SELECT = 0;
        repeat(3) @(posedge sig_7M);
        result = data_read;
        nI_O_STROBE = 1;
        nI_O_SELECT = 1;
        repeat(2) @(posedge sig_7M);
    end
    endtask

    // Pulse rddata (simulate incoming 1 bit from ESP32 — idle LOW, pulse HIGH).
    // The DUT inverts rddata internally, so a HIGH pulse here looks like a
    // LOW pulse to the IWM read shift register.
    task pulse_rddata;
    begin
        gpio_12_rddata = 1;
        repeat(2) @(posedge sig_7M);
        gpio_12_rddata = 0;
        repeat(10) @(posedge sig_7M);
    end
    endtask

    // =========================================================================
    // Test Procedures
    // =========================================================================
    initial begin
        $dumpfile("smart_hamr_tb.vcd");
        $dumpvars(0, smart_hamr_tb);

        $display("");
        $display("===========================================");
        $display("Smart Hamr Rev 2 Testbench");
        $display("===========================================");
        $display("");

        init;
        reset;

        // -----------------------------------------------------------------
        // T1: IWM Register Write — phase[0]
        // -----------------------------------------------------------------
        test_num = 1;
        $display("T1: IWM register write - phase[0]");

        iwm_write(4'b0001);    // phase[0] = 1
        #100;
        if (gpio_16_phase0 == 1'b1)
            $display("    PASS: phase[0] = 1");
        else begin
            $display("    FAIL: phase[0] = %b (expected 1)", gpio_16_phase0);
            errors = errors + 1;
        end

        iwm_write(4'b0000);    // phase[0] = 0
        #100;
        if (gpio_16_phase0 == 1'b0)
            $display("    PASS: phase[0] = 0");
        else begin
            $display("    FAIL: phase[0] = %b (expected 0)", gpio_16_phase0);
            errors = errors + 1;
        end

        // -----------------------------------------------------------------
        // T2: IWM Register Read — status register
        // -----------------------------------------------------------------
        test_num = 2;
        $display("T2: IWM register read - status register");

        iwm_write(4'b1101);    // Q6 = 1 selects status
        #100;
        gpio_13_sense = 1;
        #100;

        iwm_read(4'b0000, read_data);
        if (read_data == 8'h87)
            $display("    PASS: status = $%02X", read_data);
        else begin
            $display("    FAIL: status = $%02X (expected $87)", read_data);
            errors = errors + 1;
        end

        iwm_write(4'b1100);    // Q6 = 0

        // -----------------------------------------------------------------
        // T3: ROM Read — first byte of Liron ROM
        // -----------------------------------------------------------------
        test_num = 3;
        $display("T3: ROM read - first byte");

        // $C400 (nI_O_SELECT low, addr=$00) maps to ROM offset $400, which
        // is the slot-4 boot vector. First byte = $A2 (LDX #imm).
        rom_read(12'h000, 0, read_data);
        if (read_data == 8'hA2)
            $display("    PASS: ROM[$400] (slot-4 boot) = $%02X", read_data);
        else begin
            $display("    FAIL: ROM[$400] = $%02X (expected $A2)", read_data);
            errors = errors + 1;
        end

        // -----------------------------------------------------------------
        // T4: Expansion ROM
        // -----------------------------------------------------------------
        test_num = 4;
        $display("T4: Expansion ROM - access $C800 after $C400");

        rom_read(12'h000, 0, read_data);
        #100;
        rom_read(12'h100, 1, read_data);
        $display("    ROM[$100] = $%02X (expansion ROM)", read_data);

        // -----------------------------------------------------------------
        // T5: Expansion Clear
        // -----------------------------------------------------------------
        test_num = 5;
        $display("T5: Expansion clear - access $CFFF");

        rom_read(12'hFFF, 1, read_data);
        #100;
        if (dut.rom_expansion_active == 1'b0)
            $display("    PASS: expansion cleared");
        else begin
            $display("    FAIL: expansion still active");
            errors = errors + 1;
        end

        // -----------------------------------------------------------------
        // T6: Drive Enable
        // -----------------------------------------------------------------
        test_num = 6;
        $display("T6: Drive enable - motor on, drive select");

        // Clear modeLatch (modeReg[0]) so the IWM's SmartPort drive-gating
        // (iwm.v:248) doesn't force _enbl1/_enbl2 high. Mode register is
        // written by setting Q7=1, Q6=1, motor=1, then reading $C0Cx with
        // the value on the data bus — too involved for this test. Easier
        // path: poke the internal register directly via hierarchical ref.
        // This validates _enbl1/_enbl2 wiring; SmartPort gating itself is
        // exercised by the real Liron ROM during hardware testing.
        force dut.u_iwm.modeReg = 8'h00;
        #50;

        iwm_write(4'b1001);    // motor on
        #100;
        if (gpio_20_enbl1 == 1'b0 && gpio_9_enbl2 == 1'b1)
            $display("    PASS: _enbl1=0, _enbl2=1 (drive 1 selected)");
        else begin
            $display("    FAIL: _enbl1=%b, _enbl2=%b",
                     gpio_20_enbl1, gpio_9_enbl2);
            errors = errors + 1;
        end

        iwm_write(4'b1011);    // driveSelect = 1
        #100;
        if (gpio_20_enbl1 == 1'b1 && gpio_9_enbl2 == 1'b0)
            $display("    PASS: _enbl1=1, _enbl2=0 (drive 2 selected)");
        else begin
            $display("    FAIL: _enbl1=%b, _enbl2=%b",
                     gpio_20_enbl1, gpio_9_enbl2);
            errors = errors + 1;
        end

        iwm_write(4'b1000);    // motor off
        iwm_write(4'b1010);    // driveSelect = 0
        release dut.u_iwm.modeReg;

        // -----------------------------------------------------------------
        // T7: Serial Read (basic pulse detection)
        // -----------------------------------------------------------------
        test_num = 7;
        $display("T7: Serial read - pulse detection");

        repeat(8) begin
            pulse_rddata;
            repeat(30) @(posedge sig_7M);
        end

        iwm_read(4'b0000, read_data);
        $display("    Data buffer = $%02X", read_data);

        // -----------------------------------------------------------------
        // Summary
        // -----------------------------------------------------------------
        $display("");
        $display("===========================================");
        if (errors == 0)
            $display("All tests PASSED");
        else
            $display("Tests completed with %0d errors", errors);
        $display("===========================================");
        $display("");

        #1000;
        $finish;
    end

    // =========================================================================
    // Timeout Watchdog
    // =========================================================================
    initial begin
        #1000000;
        $display("ERROR: Simulation timeout!");
        $finish;
    end

endmodule
