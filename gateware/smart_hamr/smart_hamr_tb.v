// =============================================================================
// Smart Hamr — Top Level Testbench (Full Liron Card)
// =============================================================================
// Exercises the complete card as seen by the Apple IIe bus and the ESP32:
//
//   1. Power-on reset sequence
//   2. ROM reads — slot 4 boot code ($C400), expansion ROM ($C800)
//   3. Expansion ROM flag — set by $C4xx access, cleared by $CFFF
//   4. IWM softswitch access through the full bus path
//   5. GPIO outputs to ESP32 (phase, enable, wrreq)
//   6. GPIO inputs from ESP32 (rddata, sense, rd_buf_en)
//   7. Level shifter OE control (GPIO12)
//   8. Data bus tri-state (hi-Z when not selected)
//   9. Daisy chain pass-through
//  10. Liron boot sequence replay (first 6 instructions from ROM)
//
// Usage:  make sim DESIGN=smart_hamr
//         make wave DESIGN=smart_hamr
// =============================================================================

`timescale 1ns / 100ps

module smart_hamr_tb;

    // =========================================================================
    // Clock Generation
    // =========================================================================

    // 7.16 MHz — THE clock (139.6 ns period)
    reg sig_7M = 0;
    always #69.8 sig_7M = ~sig_7M;

    // Q3 — ~2 MHz asymmetric
    reg Q3 = 0;
    always #250 Q3 = ~Q3;

    // 25 MHz — unused but pinned
    reg CLK_25MHz = 0;
    always #20 CLK_25MHz = ~CLK_25MHz;

    // PHI0/PHI1 — ~1 MHz (unused but pinned)
    reg PHI0 = 0;
    reg PHI1 = 1;
    always #500 begin PHI0 = ~PHI0; PHI1 = ~PHI1; end

    // =========================================================================
    // Apple II Bus Signals
    // =========================================================================
    reg [11:0]  addr = 12'h000;
    reg         R_nW = 1'b1;            // Default: read
    reg         nDEVICE_SELECT = 1'b1;  // IWM registers
    reg         nI_O_SELECT = 1'b1;     // Slot ROM $C4xx
    reg         nI_O_STROBE = 1'b1;     // Expansion ROM $C8xx-$CFxx
    reg         RDY = 1'b1;
    reg         uSync = 1'b0;

    // Daisy chain inputs
    reg         DMA_OUT = 1'b1;
    reg         INT_OUT = 1'b1;

    // Data bus — directly connected to DUT inout pins
    wire        D0, D1, D2, D3, D4, D5, D6, D7;
    reg  [7:0]  data_drv = 8'h00;
    reg         data_oe = 1'b0;         // Testbench drives bus during writes

    assign D0 = data_oe ? data_drv[0] : 1'bZ;
    assign D1 = data_oe ? data_drv[1] : 1'bZ;
    assign D2 = data_oe ? data_drv[2] : 1'bZ;
    assign D3 = data_oe ? data_drv[3] : 1'bZ;
    assign D4 = data_oe ? data_drv[4] : 1'bZ;
    assign D5 = data_oe ? data_drv[5] : 1'bZ;
    assign D6 = data_oe ? data_drv[6] : 1'bZ;
    assign D7 = data_oe ? data_drv[7] : 1'bZ;

    wire [7:0] data_bus = {D7, D6, D5, D4, D3, D2, D1, D0};

    // =========================================================================
    // ESP32 GPIO Signals (directly wired to DUT)
    // =========================================================================
    reg         esp_rd_buf_en = 1'b1;   // GPIO6: HIGH = buffer disabled (boot)
    reg         esp_sense = 1'b0;       // GPIO8: sense/ACK
    reg         esp_rddata = 1'b0;      // GPIO9: rddata (idle LOW, inverted in top)

    // DUT outputs
    wire        gpio1_enbl1;
    wire        gpio2_ph2;
    wire        gpio3_ph3;
    wire        gpio4_ph0;
    wire        gpio5_ph1;
    wire        gpio7_wrreq;
    wire        gpio10_wrdata;
    wire        gpio11_enbl2;
    wire        gpio12_lvl_oe;

    // DUT control outputs
    wire        nRES, nIRQ, nNMI, nDMA, nINH;
    wire        DMA_IN, INT_IN;

    // =========================================================================
    // DUT Instantiation
    // =========================================================================
    smart_hamr_top dut (
        .CLK_25MHz      (CLK_25MHz),
        .addr           (addr),
        .D0(D0), .D1(D1), .D2(D2), .D3(D3),
        .D4(D4), .D5(D5), .D6(D6), .D7(D7),
        .sig_7M         (sig_7M),
        .Q3             (Q3),
        .R_nW           (R_nW),
        .nDEVICE_SELECT (nDEVICE_SELECT),
        .nI_O_SELECT    (nI_O_SELECT),
        .nI_O_STROBE    (nI_O_STROBE),
        .nRES           (nRES),
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
        .GPIO1          (gpio1_enbl1),
        .GPIO2          (gpio2_ph2),
        .GPIO3          (gpio3_ph3),
        .GPIO4          (gpio4_ph0),
        .GPIO5          (gpio5_ph1),
        .GPIO6          (esp_rd_buf_en),
        .GPIO7          (gpio7_wrreq),
        .GPIO8          (esp_sense),
        .GPIO9          (esp_rddata),
        .GPIO10         (gpio10_wrdata),
        .GPIO11         (gpio11_enbl2),
        .GPIO12         (gpio12_lvl_oe)
    );

    // =========================================================================
    // Test Infrastructure
    // =========================================================================
    integer pass_count = 0;
    integer fail_count = 0;

    task assert_eq;
        input [63:0] actual;
        input [63:0] expected;
        input [255:0] msg;
        begin
            if (actual === expected) begin
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL: %0s — expected %0h, got %0h (t=%0t)",
                         msg, expected, actual, $time);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Bus access tasks — simulate the Apple IIe bus cycle
    // -------------------------------------------------------------------------

    // Read an IWM softswitch ($C0C0 + offset)
    // Latches state bit, then deasserts. For state-setting accesses (LDA).
    task iwm_read;
        input [3:0] sw_addr;
        begin
            R_nW = 1'b1;
            data_oe = 1'b0;
            addr[3:0] = sw_addr;
            @(posedge sig_7M); #1;
            nDEVICE_SELECT = 1'b0;
            repeat (4) @(posedge sig_7M);
            #1;
            nDEVICE_SELECT = 1'b1;
            repeat (2) @(posedge sig_7M);
        end
    endtask

    // Read an IWM softswitch and leave /DEV asserted so caller can sample data_bus.
    // Caller must call iwm_read_end after sampling.
    task iwm_read_sample;
        input [3:0] sw_addr;
        begin
            R_nW = 1'b1;
            data_oe = 1'b0;
            addr[3:0] = sw_addr;
            @(posedge sig_7M); #1;
            nDEVICE_SELECT = 1'b0;
            repeat (3) @(posedge sig_7M);
        end
    endtask

    task iwm_read_end;
        begin
            #1;
            nDEVICE_SELECT = 1'b1;
            repeat (2) @(posedge sig_7M);
        end
    endtask

    // Write to an IWM softswitch ($C0C0 + offset)
    // Data must remain on the bus until the IWM samples on the rising edge
    // of write_clk (Q3 | /DEV). Keep data driven for 2 extra cycles after
    // /DEV deassert so the edge detector inside the IWM catches it.
    task iwm_write;
        input [3:0] sw_addr;
        input [7:0] wdata;
        begin
            R_nW = 1'b0;
            data_oe = 1'b1;
            data_drv = wdata;
            addr[3:0] = sw_addr;
            @(posedge sig_7M); #1;
            nDEVICE_SELECT = 1'b0;
            repeat (4) @(posedge sig_7M);
            #1;
            nDEVICE_SELECT = 1'b1;
            // Hold data on bus for 2 more cycles so IWM samples write_clk_rising
            repeat (2) @(posedge sig_7M);
            data_oe = 1'b0;
            R_nW = 1'b1;
            repeat (1) @(posedge sig_7M);
        end
    endtask

    // Read from slot ROM ($C400 + offset)
    // Sample data while select is still asserted (bus goes hi-Z after deassert)
    task rom_read;
        input [7:0] offset;
        begin
            R_nW = 1'b1;
            data_oe = 1'b0;
            addr[7:0] = offset;
            @(posedge sig_7M); #1;
            nI_O_SELECT = 1'b0;
            repeat (3) @(posedge sig_7M);
            // data_bus is valid here — ROM is combinatorial
            // (assert_eq called by caller while nI_O_SELECT still low)
        end
    endtask

    // Deassert I/O select (call after sampling data)
    task rom_read_end;
        begin
            #1;
            nI_O_SELECT = 1'b1;
            repeat (2) @(posedge sig_7M);
        end
    endtask

    // Read from expansion ROM ($C800 + offset)
    task exp_rom_read;
        input [10:0] offset;
        begin
            R_nW = 1'b1;
            data_oe = 1'b0;
            addr[10:0] = offset;
            @(posedge sig_7M); #1;
            nI_O_STROBE = 1'b0;
            repeat (3) @(posedge sig_7M);
        end
    endtask

    task exp_rom_read_end;
        begin
            #1;
            nI_O_STROBE = 1'b1;
            repeat (2) @(posedge sig_7M);
        end
    endtask

    // Access $CFFF to clear expansion ROM flag
    task clear_expansion;
        begin
            R_nW = 1'b1;
            data_oe = 1'b0;
            addr = 12'hFFF;
            @(posedge sig_7M); #1;
            nI_O_STROBE = 1'b0;
            repeat (4) @(posedge sig_7M);
            #1;
            nI_O_STROBE = 1'b1;
            repeat (2) @(posedge sig_7M);
        end
    endtask

    // -------------------------------------------------------------------------
    // Send a byte on GPIO9 using ESP32-side polarity (idle LOW, pulse HIGH).
    // Each '1' bit = brief HIGH pulse (~1 µs), '0' = stay LOW.
    // Top module inverts: rddata = ~GPIO9. MSB first.
    // -------------------------------------------------------------------------
    task send_byte_esp;
        input [7:0] byte_val;
        integer b;
        begin
            for (b = 7; b >= 0; b = b - 1) begin
                if (byte_val[b]) begin
                    esp_rddata = 1'b1; #1000; esp_rddata = 1'b0; #2900;
                end else begin
                    #3900;
                end
            end
        end
    endtask

    // Softswitch address constants
    localparam PH0_OFF   = 4'h0;
    localparam PH0_ON    = 4'h1;
    localparam PH1_OFF   = 4'h2;
    localparam PH1_ON    = 4'h3;
    localparam PH2_OFF   = 4'h4;
    localparam PH2_ON    = 4'h5;
    localparam PH3_OFF   = 4'h6;
    localparam PH3_ON    = 4'h7;
    localparam MOTOR_OFF = 4'h8;
    localparam MOTOR_ON  = 4'h9;
    localparam SEL_DRV1  = 4'hA;
    localparam SEL_DRV2  = 4'hB;
    localparam Q6L       = 4'hC;
    localparam Q6H       = 4'hD;
    localparam Q7L       = 4'hE;
    localparam Q7H       = 4'hF;

    // =========================================================================
    // Test Sequence
    // =========================================================================
    initial begin
        $dumpfile("smart_hamr_tb.vcd");
        $dumpvars(0, smart_hamr_tb);

        $display("");
        $display("==============================================");
        $display("Smart Hamr — Full Liron Card Testbench");
        $display("==============================================");
        $display("");

        // =====================================================================
        // TEST 1: Power-on reset
        // =====================================================================
        $display("--- Test 1: Power-on reset ---");

        // Wait for POR to complete (15 fclk cycles)
        repeat (20) @(posedge sig_7M);

        // After POR, internal por_n should be high
        assert_eq(dut.por_n, 1'b1, "POR released");

        // GPIO outputs should be in safe state
        assert_eq(gpio1_enbl1, 1'b1, "enbl1 inactive after POR");
        assert_eq(gpio11_enbl2, 1'b1, "enbl2 inactive after POR");
        assert_eq(gpio4_ph0, 1'b0, "phase0 = 0 after POR");
        assert_eq(gpio5_ph1, 1'b0, "phase1 = 0 after POR");
        assert_eq(gpio2_ph2, 1'b0, "phase2 = 0 after POR");
        assert_eq(gpio3_ph3, 1'b0, "phase3 = 0 after POR");

        $display("");

        // =====================================================================
        // TEST 2: Daisy chain pass-through
        // =====================================================================
        $display("--- Test 2: Daisy chain pass-through ---");

        DMA_OUT = 1'b1;
        INT_OUT = 1'b1;
        #1;
        assert_eq(DMA_IN, 1'b1, "DMA pass-through HIGH");
        assert_eq(INT_IN, 1'b1, "INT pass-through HIGH");

        DMA_OUT = 1'b0;
        INT_OUT = 1'b0;
        #1;
        assert_eq(DMA_IN, 1'b0, "DMA pass-through LOW");
        assert_eq(INT_IN, 1'b0, "INT pass-through LOW");

        DMA_OUT = 1'b1;
        INT_OUT = 1'b1;

        $display("");

        // =====================================================================
        // TEST 3: Data bus hi-Z when not selected
        // =====================================================================
        $display("--- Test 3: Data bus hi-Z when idle ---");

        // All selects deasserted, R_nW=read — bus should be hi-Z
        R_nW = 1'b1;
        data_oe = 1'b0;
        nDEVICE_SELECT = 1'b1;
        nI_O_SELECT = 1'b1;
        nI_O_STROBE = 1'b1;
        @(posedge sig_7M);
        assert_eq(D0, 1'bZ, "D0 hi-Z when idle");
        assert_eq(D7, 1'bZ, "D7 hi-Z when idle");

        $display("");

        // =====================================================================
        // TEST 4: Level shifter OE (GPIO12)
        // =====================================================================
        $display("--- Test 4: Level shifter OE ---");

        // Not selected — OE should be HIGH (disabled, active-low)
        assert_eq(gpio12_lvl_oe, 1'b1, "lvl shift OFF when idle");

        // During IWM access — OE should go LOW (enabled)
        addr[3:0] = Q6L;
        R_nW = 1'b1;
        @(posedge sig_7M); #1;
        nDEVICE_SELECT = 1'b0;
        @(posedge sig_7M);
        assert_eq(gpio12_lvl_oe, 1'b0, "lvl shift ON during DEV");
        #1;
        nDEVICE_SELECT = 1'b1;
        repeat (2) @(posedge sig_7M);

        $display("");

        // =====================================================================
        // TEST 5: ROM read — slot 4 boot code ($C400)
        // =====================================================================
        // ROM $400 contains the Liron boot entry:
        //   $400: A2 (LDX #imm)
        //   $401: 20 (#$20)
        //   $402: A2 (LDX #imm)
        //   $403: 00 (#$00)
        //   $404: A2 (LDX #imm)
        //   $405: 03 (#$03)
        $display("--- Test 5: Slot ROM read ($C400) ---");

        rom_read(8'h00);
        assert_eq(data_bus, 8'hA2, "ROM $400 = A2 (LDX)");
        rom_read_end;

        rom_read(8'h01);
        assert_eq(data_bus, 8'h20, "ROM $401 = 20 (#$20)");
        rom_read_end;

        rom_read(8'h02);
        assert_eq(data_bus, 8'hA2, "ROM $402 = A2 (LDX)");
        rom_read_end;

        rom_read(8'h03);
        assert_eq(data_bus, 8'h00, "ROM $403 = 00 (#$00)");
        rom_read_end;

        rom_read(8'h04);
        assert_eq(data_bus, 8'hA2, "ROM $404 = A2 (LDX)");
        rom_read_end;

        rom_read(8'h05);
        assert_eq(data_bus, 8'h03, "ROM $405 = 03 (#$03)");
        rom_read_end;

        $display("");

        // =====================================================================
        // TEST 6: Expansion ROM — set by I/O select, read from $C800
        // =====================================================================
        // After accessing $C4xx, expansion ROM flag should be set.
        // ROM $800 = 20 EE CA (JSR $CAEE)
        $display("--- Test 6: Expansion ROM read ($C800) ---");

        // The rom_read above already set the expansion flag
        assert_eq(dut.rom_expansion_active, 1'b1, "expansion active after C4xx");

        exp_rom_read(11'h000);  // $C800 → ROM $800
        assert_eq(data_bus, 8'h20, "ROM $800 = 20 (JSR)");
        exp_rom_read_end;

        exp_rom_read(11'h001);  // $C801 → ROM $801
        assert_eq(data_bus, 8'hEE, "ROM $801 = EE");
        exp_rom_read_end;

        exp_rom_read(11'h002);  // $C802 → ROM $802
        assert_eq(data_bus, 8'hCA, "ROM $802 = CA");
        exp_rom_read_end;

        $display("");

        // =====================================================================
        // TEST 7: Expansion ROM flag — cleared by $CFFF access
        // =====================================================================
        $display("--- Test 7: Expansion ROM flag clear ($CFFF) ---");

        assert_eq(dut.rom_expansion_active, 1'b1, "expansion still active");

        clear_expansion;

        assert_eq(dut.rom_expansion_active, 1'b0, "expansion cleared by CFFF");

        $display("");

        // =====================================================================
        // TEST 8: IWM through full bus path — phase lines to GPIO
        // =====================================================================
        // smartport_bus_enable: PH1_ON, PH3_ON
        // Then verify GPIO pins reflect the phase state.
        $display("--- Test 8: IWM phases → GPIO outputs ---");

        iwm_read(PH1_ON);
        // Phase GPIO is registered — wait one more cycle
        @(posedge sig_7M);
        assert_eq(gpio5_ph1, 1'b1, "GPIO5 (PH1) = 1");
        assert_eq(gpio4_ph0, 1'b0, "GPIO4 (PH0) = 0");

        iwm_read(PH3_ON);
        @(posedge sig_7M);
        assert_eq(gpio3_ph3, 1'b1, "GPIO3 (PH3) = 1");
        assert_eq(gpio5_ph1, 1'b1, "GPIO5 (PH1) still 1");

        // smartport_bus_disable: clear all
        iwm_read(PH0_OFF);
        iwm_read(PH1_OFF);
        iwm_read(PH2_OFF);
        iwm_read(PH3_OFF);
        @(posedge sig_7M);
        assert_eq({gpio3_ph3, gpio2_ph2, gpio5_ph1, gpio4_ph0},
                  4'b0000, "all phases cleared on GPIO");

        $display("");

        // =====================================================================
        // TEST 9: Drive enable → GPIO with rd_buf_en gating
        // =====================================================================
        $display("--- Test 9: Drive enable → GPIO (rd_buf_en gating) ---");

        // rd_buf_en HIGH (buffer disabled) — _enbl1 should be forced HIGH
        esp_rd_buf_en = 1'b1;
        iwm_read(SEL_DRV1);
        iwm_read(MOTOR_ON);
        @(posedge sig_7M);
        assert_eq(gpio1_enbl1, 1'b1, "enbl1 masked by rd_buf_en=1");

        // rd_buf_en LOW (buffer enabled) — _enbl1 should pass through
        esp_rd_buf_en = 1'b0;
        @(posedge sig_7M);
        assert_eq(gpio1_enbl1, 1'b0, "enbl1 active when rd_buf_en=0");

        // Clean up
        esp_rd_buf_en = 1'b1;
        iwm_read(MOTOR_OFF);

        $display("");

        // =====================================================================
        // TEST 10: ESP32 sense input → IWM status register
        // =====================================================================
        $display("--- Test 10: SENSE input → status register ---");

        iwm_read(MOTOR_ON);
        iwm_read(Q6H);           // L6=1, L7=0 → status register read

        esp_sense = 1'b1;
        @(posedge sig_7M);

        // Read status: Q6=1, access Q7L just to read (L7 already 0)
        iwm_read_sample(Q7L);
        assert_eq(data_bus[7], 1'b1, "status bit7 = SENSE = 1");
        assert_eq(data_bus[5], 1'b1, "status bit5 = motor_on = 1");
        iwm_read_end;

        // Clean up
        esp_sense = 1'b0;
        iwm_read(Q6L);
        iwm_read(MOTOR_OFF);

        $display("");

        // =====================================================================
        // TEST 11: ESP32 rddata inversion
        // =====================================================================
        $display("--- Test 11: RDDATA inversion (GPIO9) ---");

        // rd_buf_en HIGH → rddata forced to idle HIGH regardless of GPIO9
        esp_rd_buf_en = 1'b1;
        esp_rddata = 1'b0;
        #1;
        assert_eq(dut.rddata, 1'b1, "rddata=1 when buf disabled");

        esp_rddata = 1'b1;
        #1;
        assert_eq(dut.rddata, 1'b1, "rddata=1 when buf disabled (2)");

        // rd_buf_en LOW → rddata = ~GPIO9
        esp_rd_buf_en = 1'b0;
        esp_rddata = 1'b0;
        #1;
        assert_eq(dut.rddata, 1'b1, "rddata=~0=1 (idle)");

        esp_rddata = 1'b1;
        #1;
        assert_eq(dut.rddata, 1'b0, "rddata=~1=0 (pulse)");

        // Clean up
        esp_rd_buf_en = 1'b1;
        esp_rddata = 1'b0;

        $display("");

        // =====================================================================
        // TEST 12: Write to IWM data bus path
        // =====================================================================
        // Verify data written by testbench reaches the IWM data_in port.
        // Set up mode register write: motor_off, Q6H, then STA Q7H with data.
        $display("--- Test 12: Write path (STA to IWM) ---");

        iwm_read(MOTOR_OFF);
        iwm_read(Q6H);
        iwm_write(Q7H, 8'h07);    // Write mode reg = 0x07

        // Verify via status read (Q6=1, Q7=0)
        iwm_read(Q7L);            // L7=0, L6 still 1 → status
        iwm_read_sample(Q7L);     // Read status with /DEV held
        assert_eq(data_bus[4:0], 5'h07, "mode reg = 07 via status");
        iwm_read_end;

        iwm_read(Q6L);

        $display("");

        // =====================================================================
        // TEST 13: Read path end-to-end (ESP32 GPIO9 → data bus)
        // =====================================================================
        // ESP32 sends byte $C3 on GPIO9 (idle LOW, pulse HIGH for '1').
        // Top module inverts: rddata = ~GPIO9 (idle HIGH, pulse LOW).
        // IWM shift register assembles the byte and presents it on data bus.
        $display("--- Test 13: Read path end-to-end ($C3 via GPIO9) ---");

        // Set mode = $07 (latch, async, timer disable)
        iwm_read(MOTOR_OFF);
        iwm_read(Q6H);
        iwm_write(Q7H, 8'h07);
        iwm_read(Q7L);
        iwm_read(Q6L);

        // Enter read mode: motor ON, Q6=0, Q7=0
        iwm_read(MOTOR_ON);

        // Enable rd_buf_en (LOW = buffer active, rddata passes through)
        esp_rd_buf_en = 1'b0;
        repeat (4) @(posedge sig_7M);

        // Send byte $C3 = 11000011 via GPIO9 (ESP32 convention: pulse HIGH = '1')
        // Each bit cell ≈ 4 µs. '1' = HIGH pulse ~1 µs then LOW. '0' = stay LOW.
        // Bit 7 = 1 (pulse HIGH)
        esp_rddata = 1'b1; #1000; esp_rddata = 1'b0; #2900;
        // Bit 6 = 1 (pulse HIGH)
        esp_rddata = 1'b1; #1000; esp_rddata = 1'b0; #2900;
        // Bit 5 = 0 (stay LOW)
        #3900;
        // Bit 4 = 0
        #3900;
        // Bit 3 = 0
        #3900;
        // Bit 2 = 0
        #3900;
        // Bit 1 = 1 (pulse HIGH)
        esp_rddata = 1'b1; #1000; esp_rddata = 1'b0; #2900;
        // Bit 0 = 1 (pulse HIGH)
        esp_rddata = 1'b1; #1000; esp_rddata = 1'b0; #2900;

        // Wait for byte to propagate through shift register
        repeat (20) @(posedge sig_7M);

        // Read data register via bus: LDA Q6L with /DEV asserted
        iwm_read_sample(Q6L);
        assert_eq(data_bus[7], 1'b1, "read path: D7=1 (valid byte)");
        assert_eq(data_bus, 8'hC3, "read path: data bus = $C3");

        // Level shifter should be enabled during this read
        assert_eq(gpio12_lvl_oe, 1'b0, "lvl shift ON during read");
        iwm_read_end;

        // After deassert, bus should return to hi-Z
        @(posedge sig_7M);
        assert_eq(D0, 1'bZ, "D0 hi-Z after read");

        $display("");

        // =====================================================================
        // TEST 14: Read auto-clear visible on bus
        // =====================================================================
        // After a valid read (D7=1 latched), the register clears after 14 FCLK.
        $display("--- Test 14: Read auto-clear on bus ---");

        // The read_sample above triggered x7_latch + clear countdown.
        // Wait for auto-clear to complete (14 FCLK + margin)
        repeat (20) @(posedge sig_7M);

        // Read again — should see cleared register (D7=0)
        iwm_read_sample(Q6L);
        assert_eq(data_bus[7], 1'b0, "auto-clear: D7=0 on bus");
        iwm_read_end;

        $display("");

        // =====================================================================
        // TEST 15: rd_buf_en gating prevents data reception
        // =====================================================================
        // With rd_buf_en HIGH, rddata is forced to idle HIGH (no edges).
        // A byte sent on GPIO9 should NOT be received.
        $display("--- Test 15: rd_buf_en gating blocks read ---");

        esp_rd_buf_en = 1'b1;      // Buffer disabled
        repeat (4) @(posedge sig_7M);

        // Send a byte — should be ignored
        esp_rddata = 1'b1; #1000; esp_rddata = 1'b0; #2900;
        esp_rddata = 1'b1; #1000; esp_rddata = 1'b0; #2900;
        esp_rddata = 1'b1; #1000; esp_rddata = 1'b0; #2900;
        esp_rddata = 1'b1; #1000; esp_rddata = 1'b0; #2900;
        esp_rddata = 1'b1; #1000; esp_rddata = 1'b0; #2900;
        esp_rddata = 1'b1; #1000; esp_rddata = 1'b0; #2900;
        esp_rddata = 1'b1; #1000; esp_rddata = 1'b0; #2900;
        esp_rddata = 1'b1; #1000; esp_rddata = 1'b0; #2900;

        repeat (20) @(posedge sig_7M);

        // Data register should still be empty
        iwm_read_sample(Q6L);
        assert_eq(data_bus[7], 1'b0, "rd_buf_en: D7=0 (no byte)");
        iwm_read_end;

        // Clean up
        esp_rd_buf_en = 1'b0;

        $display("");

        // =====================================================================
        // TEST 16: Write path end-to-end (CPU → GPIO10 WRDATA)
        // =====================================================================
        // CPU writes $C3 to IWM. WRDATA toggles should appear on GPIO10.
        $display("--- Test 16: Write path end-to-end ($C3 → GPIO10) ---");

        // Enter write mode: motor on, L6=1, L7=1
        iwm_read(Q6H);             // L6 = 1
        iwm_read(Q7H);             // L7 = 1 → write_mode_entry

        // GPIO10 (wrdata) should start HIGH
        repeat (2) @(posedge sig_7M);
        assert_eq(gpio10_wrdata, 1'b1, "GPIO10 starts HIGH in write");

        // Write data byte: L7=1, L6=1, motor_on=1 → sel_write_data
        iwm_write(Q6H, 8'hC3);

        // Count edges on GPIO10 over 300 FCLK (~42 µs, covers one byte)
        // Also check /WRREQ midway through (after initial delay completes)
        begin : top_count_edges
            integer edge_count;
            reg prev_wr;
            reg wrreq_checked;
            integer i;
            edge_count = 0;
            wrreq_checked = 0;
            prev_wr = gpio10_wrdata;
            for (i = 0; i < 300; i = i + 1) begin
                @(posedge sig_7M);
                if (gpio10_wrdata !== prev_wr) begin
                    edge_count = edge_count + 1;
                    prev_wr = gpio10_wrdata;
                end
                // Check /WRREQ once after the initial delay (~20 FCLK)
                if (i == 30 && !wrreq_checked) begin
                    wrreq_checked = 1;
                    assert_eq(gpio7_wrreq, 1'b0, "GPIO7 /WRREQ active in write");
                end
            end
            // $C3 = 11000011 has 4 one-bits → 4 toggles on GPIO10
            assert_eq(edge_count, 4, "GPIO10 edges = 4 for $C3");
        end

        // Clean up: exit write mode
        iwm_read(Q7L);
        iwm_read(Q6L);
        iwm_read(MOTOR_OFF);

        repeat (4) @(posedge sig_7M);
        assert_eq(gpio10_wrdata, 1'b0, "GPIO10 idle LOW after exit");

        $display("");

        // =====================================================================
        // TEST 17: Write→Read transition (full GPIO path)
        // =====================================================================
        // Write a byte via GPIO10, wait for underrun, transition to read mode,
        // receive a byte via GPIO9. This is the critical SmartPort turnaround.
        $display("--- Test 17: Write->Read transition (GPIO) ---");

        // Enter write mode and write one byte
        iwm_read(MOTOR_ON);
        iwm_read(Q6H);
        iwm_read(Q7H);             // write_mode_entry
        iwm_write(Q6H, 8'hC8);    // Write PEND marker

        // Wait for byte to shift out + underrun
        repeat (280) @(posedge sig_7M);

        // Verify underrun: handshake bit 6 = 0
        iwm_read(Q6L);
        iwm_read_sample(Q6L);      // L6=0, L7=1 → handshake
        assert_eq(data_bus[6], 1'b0, "W→R: underrun on bus");
        iwm_read_end;

        // Transition: exit write mode (Liron ROM sequence)
        iwm_read(Q6H);             // STA Q6H
        iwm_read(Q7L);             // LDA Q7L → exit write mode
        assert_eq(gpio10_wrdata, 1'b0, "W→R: GPIO10 idle after exit");

        iwm_read(Q6L);             // LDA Q6L → read data register

        // Wait for counter to clear runt zone
        repeat (16) @(posedge sig_7M);

        // Send $C3 from "ESP32" on GPIO9
        esp_rddata = 1'b1; #1000; esp_rddata = 1'b0; #2900;  // bit 7
        esp_rddata = 1'b1; #1000; esp_rddata = 1'b0; #2900;  // bit 6
        #3900;                                                  // bit 5 = 0
        #3900;                                                  // bit 4 = 0
        #3900;                                                  // bit 3 = 0
        #3900;                                                  // bit 2 = 0
        esp_rddata = 1'b1; #1000; esp_rddata = 1'b0; #2900;  // bit 1
        esp_rddata = 1'b1; #1000; esp_rddata = 1'b0; #2900;  // bit 0

        repeat (20) @(posedge sig_7M);

        // Read the byte from the data bus
        iwm_read_sample(Q6L);
        assert_eq(data_bus[7], 1'b1, "W→R: read D7=1 on bus");
        assert_eq(data_bus, 8'hC3, "W→R: read $C3 on bus");
        iwm_read_end;

        // Clean up
        repeat (20) @(posedge sig_7M);
        iwm_read(MOTOR_OFF);

        $display("");

        // =====================================================================
        // TEST 18: Read→Write transition (full GPIO path)
        // =====================================================================
        $display("--- Test 18: Read->Write transition (GPIO) ---");

        // Receive a byte
        iwm_read(MOTOR_ON);
        // L6=0, L7=0, motor=1 → read data
        repeat (16) @(posedge sig_7M);

        esp_rddata = 1'b1; #1000; esp_rddata = 1'b0; #2900;  // 8 pulses = $FF
        esp_rddata = 1'b1; #1000; esp_rddata = 1'b0; #2900;
        esp_rddata = 1'b1; #1000; esp_rddata = 1'b0; #2900;
        esp_rddata = 1'b1; #1000; esp_rddata = 1'b0; #2900;
        esp_rddata = 1'b1; #1000; esp_rddata = 1'b0; #2900;
        esp_rddata = 1'b1; #1000; esp_rddata = 1'b0; #2900;
        esp_rddata = 1'b1; #1000; esp_rddata = 1'b0; #2900;
        esp_rddata = 1'b1; #1000; esp_rddata = 1'b0; #2900;

        repeat (20) @(posedge sig_7M);

        iwm_read_sample(Q6L);
        assert_eq(data_bus, 8'hFF, "R→W: read $FF on bus");
        iwm_read_end;

        // Switch to write mode
        iwm_read(Q6H);
        iwm_read(Q7H);             // write_mode_entry
        assert_eq(gpio10_wrdata, 1'b1, "R→W: GPIO10 HIGH on entry");

        // Write $AA (4 one-bits → 4 edges)
        iwm_write(Q6H, 8'hAA);

        begin : top_rw_edges
            integer edge_count;
            reg prev_wr;
            integer i;
            edge_count = 0;
            prev_wr = gpio10_wrdata;
            for (i = 0; i < 300; i = i + 1) begin
                @(posedge sig_7M);
                if (gpio10_wrdata !== prev_wr) begin
                    edge_count = edge_count + 1;
                    prev_wr = gpio10_wrdata;
                end
            end
            assert_eq(edge_count, 4, "R→W: GPIO10 edges = 4 for $AA");
        end

        // Clean up
        iwm_read(Q7L);
        iwm_read(Q6L);
        iwm_read(MOTOR_OFF);

        $display("");

        // =====================================================================
        // TEST 19: /WRREQ gating by rd_buf_en
        // =====================================================================
        // smart_hamr_top.v:185 gates /WRREQ with rd_buf_en:
        //   GPIO7 = rd_buf_en ? 1'b1 : _wrreq
        // When rd_buf_en is HIGH, GPIO7 should stay HIGH regardless of write state.
        $display("--- Test 19: /WRREQ gating by rd_buf_en ---");

        esp_rd_buf_en = 1'b1;         // Buffer disabled
        iwm_read(MOTOR_ON);
        iwm_read(Q6H);                // L6 = 1
        iwm_read(Q7H);                // L7 = 1, write_mode_entry
        iwm_write(Q6H, 8'hFF);       // Write data (enters write state)

        // /WRREQ should be active internally, but GPIO7 is gated
        repeat (20) @(posedge sig_7M);
        assert_eq(gpio7_wrreq, 1'b1, "WRREQ masked by rd_buf_en=1");

        // Now enable buffer — /WRREQ should pass through
        esp_rd_buf_en = 1'b0;
        repeat (2) @(posedge sig_7M);
        assert_eq(gpio7_wrreq, 1'b0, "WRREQ active when rd_buf_en=0");

        // Clean up
        esp_rd_buf_en = 1'b1;
        iwm_read(Q7L);
        iwm_read(Q6L);
        iwm_read(MOTOR_OFF);

        $display("");

        // =====================================================================
        // TEST 20: _enbl2 NOT gated by rd_buf_en (documenting asymmetry)
        // =====================================================================
        // smart_hamr_top.v:183 gates _enbl1: GPIO1 = rd_buf_en ? 1'b1 : _enbl1
        // smart_hamr_top.v:184 does NOT gate _enbl2: GPIO11 = _enbl2
        // This means _enbl2 asserts on GPIO11 even when rd_buf_en is HIGH.
        // Documenting current behavior — may need gating if ESP32 is confused.
        $display("--- Test 20: _enbl2 NOT gated by rd_buf_en ---");

        esp_rd_buf_en = 1'b1;         // Buffer disabled
        iwm_read(SEL_DRV2);           // Select drive 2
        iwm_read(MOTOR_ON);           // Motor on → _enbl2 should go LOW

        @(posedge sig_7M);
        // _enbl1 is gated — should stay HIGH
        assert_eq(gpio1_enbl1, 1'b1, "enbl1 masked (rd_buf_en=1)");
        // _enbl2 is NOT gated — should be LOW (active)
        assert_eq(gpio11_enbl2, 1'b0, "enbl2 NOT masked (asymmetry)");

        // Clean up
        iwm_read(MOTOR_OFF);
        esp_rd_buf_en = 1'b0;

        $display("");

        // =====================================================================
        // TEST 21: Back-to-back 3-byte read through GPIO (firehose)
        // =====================================================================
        // ESP32 streams PBEGIN($C3) + DEST($80) + SRC($85) on GPIO9.
        // Simulates the Liron ROM's LDA Q6L / BPL polling loop reading each.
        $display("--- Test 21: Back-to-back 3-byte read (GPIO) ---");

        // Set mode = $07
        iwm_read(MOTOR_OFF);
        iwm_read(Q6H);
        iwm_write(Q7H, 8'h07);
        iwm_read(Q7L);
        iwm_read(Q6L);

        iwm_read(MOTOR_ON);
        esp_rd_buf_en = 1'b0;
        repeat (16) @(posedge sig_7M);

        // --- Byte 1: $C3 ---
        send_byte_esp(8'hC3);
        repeat (20) @(posedge sig_7M);

        iwm_read_sample(Q6L);
        assert_eq(data_bus[7], 1'b1, "firehose[0]: D7=1");
        assert_eq(data_bus, 8'hC3, "firehose[0]: $C3");
        iwm_read_end;

        // Auto-clear fires after the read above
        repeat (20) @(posedge sig_7M);

        // Verify cleared
        iwm_read_sample(Q6L);
        assert_eq(data_bus[7], 1'b0, "firehose[0]: cleared");
        iwm_read_end;

        // --- Byte 2: $80 ---
        send_byte_esp(8'h80);
        repeat (20) @(posedge sig_7M);

        iwm_read_sample(Q6L);
        assert_eq(data_bus[7], 1'b1, "firehose[1]: D7=1");
        assert_eq(data_bus, 8'h80, "firehose[1]: $80");
        iwm_read_end;

        repeat (20) @(posedge sig_7M);

        // --- Byte 3: $85 ---
        send_byte_esp(8'h85);
        repeat (20) @(posedge sig_7M);

        iwm_read_sample(Q6L);
        assert_eq(data_bus[7], 1'b1, "firehose[2]: D7=1");
        assert_eq(data_bus, 8'h85, "firehose[2]: $85");
        iwm_read_end;

        // Clean up
        repeat (20) @(posedge sig_7M);
        esp_rd_buf_en = 1'b1;
        iwm_read(MOTOR_OFF);

        $display("");

        // =====================================================================
        // TEST 22: REQ/ACK handshake — Liron ROM post-write ACK poll
        // =====================================================================
        // Reproduces the exact sequence that causes "REQ timeout in command
        // processing" on the ESP32. After writing a packet, the ROM:
        //   1. Polls Q6L bit 6 until write underrun (bit 6 = 0)
        //   2. STA Q6H (sets L6=1, also writes $00 to write data reg)
        //   3. LDA Q7L (clears L7 → L6=1,L7=0 = status read)
        //      BMI loop — branches if bit 7 = 1 (ACK not asserted)
        //      Falls through if bit 7 = 0 (ACK asserted by ESP32)
        //   4. LDA PH0_OFF (deasserts REQ)
        //
        // The ESP32 asserts ACK by pulling GPIO8 (sense) LOW.
        // If the status register doesn't expose sense on bit 7, the ROM
        // loops forever, never deasserts REQ, and ESP32 reports "REQ timeout".
        $display("--- Test 22: REQ/ACK handshake (post-write) ---");

        // Setup: mode $07, bus enable, motor on, drive 2
        iwm_read(MOTOR_OFF);
        iwm_read(Q6H);
        iwm_write(Q7H, 8'h07);
        iwm_read(Q7L);
        iwm_read(Q6L);

        iwm_read(PH1_ON);             // Bus enable
        iwm_read(PH3_ON);
        iwm_read(SEL_DRV2);
        iwm_read(MOTOR_ON);
        esp_rd_buf_en = 1'b0;

        // Pre-check: ROM polls status for ACK HIGH before asserting REQ.
        // ACK should be deasserted (GPIO8 = HIGH by default).
        esp_sense = 1'b0;  // ACK deasserted = sense LOW? No...
        // ESP32 ACK deasserted = GPIO HIGH (hi-Z with pullup)
        // sense = GPIO8 directly. ACK deasserted → GPIO8 HIGH → sense=1
        // status bit 7 = sense = 1 → BMI taken → keep polling
        // Wait — the pre-check loop exits when bit 7 = 1 (BMI branches on N=1).
        // Actually re-reading the ROM:
        //   Lc813: lda iwm_q7l,x ; read status
        //          bmi Lc81f      ; branch if bit7=1 (sense HIGH = ACK deasserted)
        // So bit7=1 means ACK is NOT asserted = good, proceed.
        // bit7=0 means ACK IS asserted = still busy, keep waiting.
        esp_sense = 1'b1;  // ACK deasserted = sense HIGH

        // Enter write mode
        iwm_read(Q6H);                // L6=1
        iwm_read(Q7H);                // L7=1, write_mode_entry

        // Assert REQ
        iwm_read(PH0_ON);
        @(posedge sig_7M);
        assert_eq(gpio4_ph0, 1'b1, "ACK: REQ asserted (GPIO4=1)");

        // Write byte: the ROM combines L7-set + data write in one STA Q7H.
        // Our test did iwm_read(Q7H) (no data) then iwm_write(Q6H, data).
        // This means write_data_reg=$00 was loaded as the first byte on
        // write_mode_entry. $C8 goes into the buffer as the second byte.
        // We must wait for BOTH bytes to shift out before underrun.
        iwm_write(Q6H, 8'hC8);

        // Wait for two bytes: 7 CLK delay + 2 × (8 × 14 CLK) = 231 CLK = 462 FCLK.
        // Plus margin for task overhead.
        repeat (550) @(posedge sig_7M);

        // Step 1: Poll Q6L bit 6 for underrun (L6=0, L7=1 → handshake)
        iwm_read(Q6L);                // L6=0
        iwm_read_sample(Q6L);
        assert_eq(data_bus[6], 1'b0, "ACK: underrun detected");
        iwm_read_end;

        // Step 2: STA Q6H (sets L6=1)
        // The ROM has A=$00 after AND #$40 with bit6=0
        iwm_write(Q6H, 8'h00);

        // Step 3: LDA Q7L (clears L7, L6=1,L7=0 → status register)
        // ACK is still deasserted (sense=1), so bit 7 should be 1
        iwm_read_sample(Q7L);
        assert_eq(data_bus[7], 1'b1, "ACK: sense=1 before ACK assert");
        iwm_read_end;

        // Now simulate ESP32 asserting ACK (pulls GPIO8 LOW)
        esp_sense = 1'b0;
        repeat (2) @(posedge sig_7M);

        // Read status again — bit 7 should now be 0 (ACK asserted)
        iwm_read_sample(Q7L);
        assert_eq(data_bus[7], 1'b0, "ACK: sense=0 after ACK assert");
        iwm_read_end;

        // Step 4: ROM sees ACK, deasserts REQ
        iwm_read(PH0_OFF);
        @(posedge sig_7M);
        assert_eq(gpio4_ph0, 1'b0, "ACK: REQ deasserted (GPIO4=0)");

        // ESP32 would now deassert ACK
        esp_sense = 1'b1;
        repeat (2) @(posedge sig_7M);

        // Verify sense returns to 1 on status register
        iwm_read_sample(Q7L);
        assert_eq(data_bus[7], 1'b1, "ACK: sense=1 after ACK deassert");
        iwm_read_end;

        // Clean up
        iwm_read(Q6L);
        iwm_read(Q7L);
        iwm_read(PH1_OFF);
        iwm_read(PH3_OFF);
        iwm_read(MOTOR_OFF);
        esp_rd_buf_en = 1'b1;

        $display("");

        // =====================================================================
        // Results
        // =====================================================================
        $display("==============================================");
        $display("Results: %0d passed, %0d failed", pass_count, fail_count);
        $display("==============================================");

        if (fail_count == 0)
            $display("*** ALL TESTS PASSED ***");
        else
            $display("*** %0d TEST(S) FAILED ***", fail_count);

        $display("");
        $finish;
    end

    // =========================================================================
    // Timeout watchdog
    // =========================================================================
    initial begin
        #15_000_000;  // 15 ms — REQ/ACK handshake test needs more time
        $display("ERROR: Testbench timed out!");
        $finish;
    end

endmodule
