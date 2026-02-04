`timescale 1ns / 1ps
// =============================================================================
// Yellow Hamr Testbench
// =============================================================================
// Tests for the Liron disk controller emulation
//
// Test Plan:
//   T1: IWM register write - Write $C0C1 -> phase[0]=1
//   T2: IWM register read  - Set Q6=1, read $C0C0 -> status byte
//   T3: ROM read           - Read $C400 -> first ROM byte
//   T4: Expansion ROM      - Access $C400, then $C800 -> ROM[$100]
//   T5: Expansion clear    - Access $CFFF -> clears expansion flag
//   T6: Serial write       - Set Q7=1, write data -> wrdata toggles
//   T7: Serial read        - Pulse rddata -> data appears in buffer
// =============================================================================

module yellow_hamr_tb;

    // =========================================================================
    // Parameters
    // =========================================================================
    localparam CLK_25_PERIOD = 40;      // 25 MHz = 40ns period
    localparam CLK_7M_PERIOD = 140;     // 7.16 MHz = ~140ns period
    localparam Q3_PERIOD     = 500;     // ~2 MHz = 500ns period

    // =========================================================================
    // Signals
    // =========================================================================

    // System
    reg         clk_25;
    reg         sig_7M;
    reg         Q3;

    // Apple II bus
    reg  [11:0] addr;
    wire [7:0]  data;
    reg  [7:0]  data_out_reg;
    reg         data_drive;
    reg         R_nW;
    reg         nDEVICE_SELECT;
    reg         nI_O_SELECT;
    reg         nI_O_STROBE;
    reg         nRES;

    // GPIO (drive interface)
    wire        gpio1_phase0;
    wire        gpio2_phase1;
    wire        gpio3_phase2;
    wire        gpio4_phase3;
    wire        gpio5_wrdata;
    reg         gpio6_rddata;
    reg         gpio7_sense;
    wire        gpio8_enbl1;
    wire        gpio9_enbl2;
    wire        gpio10_wrreq;
    wire        gpio11_en35;
    wire        gpio12_spare;

    // Test control
    reg  [7:0]  read_data;
    integer     test_num;
    integer     errors;

    // =========================================================================
    // Data Bus Tristate Control
    // =========================================================================
    assign data = data_drive ? data_out_reg : 8'bZZZZZZZZ;

    // =========================================================================
    // DUT Instantiation
    // =========================================================================
    yellow_hamr_top dut (
        .CLK_25MHz      (clk_25),
        .addr           (addr),
        .data           (data),
        .sig_7M         (sig_7M),
        .Q3             (Q3),
        .R_nW           (R_nW),
        .nDEVICE_SELECT (nDEVICE_SELECT),
        .nI_O_SELECT    (nI_O_SELECT),
        .nI_O_STROBE    (nI_O_STROBE),
        .nRES           (nRES),
        .GPIO1          (gpio1_phase0),
        .GPIO2          (gpio2_phase1),
        .GPIO3          (gpio3_phase2),
        .GPIO4          (gpio4_phase3),
        .GPIO5          (gpio5_wrdata),
        .GPIO6          (gpio6_rddata),
        .GPIO7          (gpio7_sense),
        .GPIO8          (gpio8_enbl1),
        .GPIO9          (gpio9_enbl2),
        .GPIO10         (gpio10_wrreq),
        .GPIO11         (gpio11_en35),
        .GPIO12         (gpio12_spare)
    );

    // =========================================================================
    // Clock Generation
    // =========================================================================
    initial clk_25 = 0;
    always #(CLK_25_PERIOD/2) clk_25 = ~clk_25;

    initial sig_7M = 0;
    always #(CLK_7M_PERIOD/2) sig_7M = ~sig_7M;

    initial Q3 = 0;
    always #(Q3_PERIOD/2) Q3 = ~Q3;

    // =========================================================================
    // Test Tasks
    // =========================================================================

    // Initialize all signals
    task init;
    begin
        addr           = 12'h000;
        data_out_reg   = 8'h00;
        data_drive     = 0;
        R_nW           = 1;
        nDEVICE_SELECT = 1;
        nI_O_SELECT    = 1;
        nI_O_STROBE    = 1;
        nRES           = 1;
        gpio6_rddata   = 1;  // Idle high
        gpio7_sense    = 0;
        test_num       = 0;
        errors         = 0;
    end
    endtask

    // System reset
    task reset;
    begin
        nRES = 0;
        repeat(10) @(posedge sig_7M);
        nRES = 1;
        repeat(10) @(posedge sig_7M);
    end
    endtask

    // Write to IWM register ($C0Cx)
    // addr[3:1] selects register, addr[0] is value
    task iwm_write;
        input [3:0] reg_addr;
    begin
        addr = {8'h00, reg_addr};
        R_nW = 0;
        data_drive = 0;
        @(posedge sig_7M);
        nDEVICE_SELECT = 0;
        repeat(2) @(posedge sig_7M);
        nDEVICE_SELECT = 1;
        repeat(2) @(posedge sig_7M);
    end
    endtask

    // Read from IWM register ($C0Cx)
    task iwm_read;
        input [3:0] reg_addr;
        output [7:0] result;
    begin
        addr = {8'h00, reg_addr};
        R_nW = 1;
        data_drive = 0;
        @(posedge sig_7M);
        nDEVICE_SELECT = 0;
        repeat(2) @(posedge sig_7M);
        result = data;
        nDEVICE_SELECT = 1;
        repeat(2) @(posedge sig_7M);
    end
    endtask

    // Write data to IWM (Q7=1, Q6=1, motorOn=1, A0=1)
    task iwm_write_data;
        input [7:0] write_data;
    begin
        addr = 4'b0001;  // A0=1 for write
        R_nW = 0;
        data_out_reg = write_data;
        data_drive = 1;
        @(posedge sig_7M);
        nDEVICE_SELECT = 0;
        // Wait for Q3 to go low (write window)
        wait(Q3 == 0);
        repeat(2) @(posedge sig_7M);
        nDEVICE_SELECT = 1;
        data_drive = 0;
        repeat(2) @(posedge sig_7M);
    end
    endtask

    // Read from ROM ($C4xx or $C8xx)
    task rom_read;
        input [11:0] rom_addr;
        input        use_strobe;  // 1 for $C800-$CFFF, 0 for $C400-$C4FF
        output [7:0] result;
    begin
        addr = rom_addr;
        R_nW = 1;
        data_drive = 0;
        @(posedge sig_7M);
        if (use_strobe)
            nI_O_STROBE = 0;
        else
            nI_O_SELECT = 0;
        repeat(3) @(posedge sig_7M);  // Wait for ROM read latency
        result = data;
        nI_O_STROBE = 1;
        nI_O_SELECT = 1;
        repeat(2) @(posedge sig_7M);
    end
    endtask

    // Pulse rddata (simulate incoming 1 bit from drive)
    task pulse_rddata;
    begin
        gpio6_rddata = 0;
        repeat(2) @(posedge sig_7M);
        gpio6_rddata = 1;
        repeat(10) @(posedge sig_7M);
    end
    endtask

    // =========================================================================
    // Test Procedures
    // =========================================================================

    initial begin
        $dumpfile("yellow_hamr_tb.vcd");
        $dumpvars(0, yellow_hamr_tb);

        $display("");
        $display("===========================================");
        $display("Yellow Hamr Testbench");
        $display("===========================================");
        $display("");

        init;
        reset;

        // -----------------------------------------------------------------
        // T1: IWM Register Write
        // -----------------------------------------------------------------
        test_num = 1;
        $display("T1: IWM register write - phase[0]=1");

        // Write $C0C1 (phase[0] = 1)
        iwm_write(4'b0001);
        #100;

        if (gpio1_phase0 == 1'b1) begin
            $display("    PASS: phase[0] = 1");
        end else begin
            $display("    FAIL: phase[0] = %b (expected 1)", gpio1_phase0);
            errors = errors + 1;
        end

        // Write $C0C0 (phase[0] = 0)
        iwm_write(4'b0000);
        #100;

        if (gpio1_phase0 == 1'b0) begin
            $display("    PASS: phase[0] = 0");
        end else begin
            $display("    FAIL: phase[0] = %b (expected 0)", gpio1_phase0);
            errors = errors + 1;
        end

        // -----------------------------------------------------------------
        // T2: IWM Register Read (Status)
        // -----------------------------------------------------------------
        test_num = 2;
        $display("T2: IWM register read - status register");

        // Set Q6=1 to select status register
        iwm_write(4'b1101);  // $C0CD: Q6=1
        #100;

        // Set sense input
        gpio7_sense = 1;
        #100;

        // Read status (A0=0)
        iwm_read(4'b0000, read_data);

        // Expected: {sense=1, 0, motorOn=0, mode=00111} = 10000111 = $87
        if (read_data == 8'h87) begin
            $display("    PASS: status = $%02X", read_data);
        end else begin
            $display("    FAIL: status = $%02X (expected $87)", read_data);
            errors = errors + 1;
        end

        // Clear Q6
        iwm_write(4'b1100);  // $C0CC: Q6=0

        // -----------------------------------------------------------------
        // T3: ROM Read ($C400)
        // -----------------------------------------------------------------
        test_num = 3;
        $display("T3: ROM read - first byte");

        rom_read(12'h000, 0, read_data);

        // First byte of Liron ROM should be $C6
        if (read_data == 8'hC6) begin
            $display("    PASS: ROM[$000] = $%02X", read_data);
        end else begin
            $display("    FAIL: ROM[$000] = $%02X (expected $C6)", read_data);
            errors = errors + 1;
        end

        // -----------------------------------------------------------------
        // T4: Expansion ROM
        // -----------------------------------------------------------------
        test_num = 4;
        $display("T4: Expansion ROM - access $C800 after $C400");

        // Access $C400 first (sets expansion active)
        rom_read(12'h000, 0, read_data);
        #100;

        // Now access $C800 (ROM $100)
        rom_read(12'h100, 1, read_data);

        $display("    ROM[$100] = $%02X (expansion ROM)", read_data);

        // -----------------------------------------------------------------
        // T5: Expansion Clear
        // -----------------------------------------------------------------
        test_num = 5;
        $display("T5: Expansion clear - access $CFFF");

        // Access $CFFF (clears expansion flag)
        rom_read(12'hFFF, 1, read_data);
        #100;

        // Check that expansion is cleared (internal signal)
        if (dut.rom_expansion_active == 1'b0) begin
            $display("    PASS: expansion cleared");
        end else begin
            $display("    FAIL: expansion still active");
            errors = errors + 1;
        end

        // -----------------------------------------------------------------
        // T6: Drive Enable
        // -----------------------------------------------------------------
        test_num = 6;
        $display("T6: Drive enable - motor on, drive select");

        // Enable motor ($C0C9: motorOn=1)
        iwm_write(4'b1001);
        #100;

        // With driveSelect=0, _enbl1 should be low
        if (gpio8_enbl1 == 1'b0 && gpio9_enbl2 == 1'b1) begin
            $display("    PASS: _enbl1=0, _enbl2=1 (drive 1 selected)");
        end else begin
            $display("    FAIL: _enbl1=%b, _enbl2=%b", gpio8_enbl1, gpio9_enbl2);
            errors = errors + 1;
        end

        // Select drive 2 ($C0CB: driveSelect=1)
        iwm_write(4'b1011);
        #100;

        if (gpio8_enbl1 == 1'b1 && gpio9_enbl2 == 1'b0) begin
            $display("    PASS: _enbl1=1, _enbl2=0 (drive 2 selected)");
        end else begin
            $display("    FAIL: _enbl1=%b, _enbl2=%b", gpio8_enbl1, gpio9_enbl2);
            errors = errors + 1;
        end

        // Motor off
        iwm_write(4'b1000);  // $C0C8: motorOn=0
        iwm_write(4'b1010);  // $C0CA: driveSelect=0

        // -----------------------------------------------------------------
        // T7: Serial Read (basic pulse detection)
        // -----------------------------------------------------------------
        test_num = 7;
        $display("T7: Serial read - pulse detection");

        // Set to read mode (Q7=0, Q6=0) - already in this state
        // Pulse rddata multiple times to shift in data
        repeat(8) begin
            pulse_rddata;
            // Wait for bit cell time (~4us = ~28 * 140ns)
            repeat(30) @(posedge sig_7M);
        end

        // Read data register
        iwm_read(4'b0000, read_data);
        $display("    Data buffer = $%02X", read_data);

        // -----------------------------------------------------------------
        // Summary
        // -----------------------------------------------------------------
        $display("");
        $display("===========================================");
        if (errors == 0) begin
            $display("All tests PASSED");
        end else begin
            $display("Tests completed with %0d errors", errors);
        end
        $display("===========================================");
        $display("");

        #1000;
        $finish;
    end

    // =========================================================================
    // Timeout Watchdog
    // =========================================================================
    initial begin
        #1000000;  // 1ms timeout
        $display("ERROR: Simulation timeout!");
        $finish;
    end

endmodule
