// =============================================================================
// IWM Address Decoder & State Machine — Testbench
// =============================================================================
// Exercises the IWM decoder using sequences taken directly from the
// Liron ROM disassembly (liron-if.asm).
//
// Usage:  make unit DESIGN=smart_hamr MODULE=iwm_ii
//         make unit-wave DESIGN=smart_hamr MODULE=iwm_ii
//
// What we test:
//   1. Reset clears all state
//   2. Individual state bit set/clear (PHASE0-3, Motor-On, Drive-Sel, L6, L7)
//   3. smartport_bus_enable  — sets PHASE1, PHASE3
//   4. smartport_bus_disable — clears PHASE0-3
//   5. Enable logic (_enbl1, _enbl2)
//   6. Register decode (status, handshake, all-ones, read-data)
//   7. Mode register write via Q3|/DEV write clock
//   8. Write data register load & handshake buffer-ready flag
//   9. Liron ROM init sequence (motor_off, Q6H, Q7L read-verify loop)
// =============================================================================

`timescale 1ns / 100ps

module iwm_ii_tb;

    // =========================================================================
    // Clock Generation — 7.16 MHz (139.6 ns period)
    // =========================================================================
    reg fclk = 0;
    always #69.8 fclk = ~fclk;

    // Q3 — ~2 MHz asymmetric (approximation: 250 ns high, 250 ns low)
    reg Q3 = 0;
    always #250 Q3 = ~Q3;

    // =========================================================================
    // DUT Signals
    // =========================================================================
    reg  [3:0] addr = 4'h0;
    reg        nDEVICE_SELECT = 1'b1;
    reg  [7:0] data_in = 8'h00;
    reg        nRES = 1'b0;          // Start in reset
    reg        rddata = 1'b1;        // Idle high
    reg        sense = 1'b0;

    wire [3:0] phase;
    wire       _enbl1;
    wire       _enbl2;
    wire       _wrreq;
    wire       wrdata;
    wire [7:0] data_out;
    wire       q6_out;
    wire       q7_out;

    // Test 29 working registers (module-level for Icarus compatibility)
    integer    t29_iter;
    integer    t29_found_c3 = 0;
    reg  [7:0] t29_read_byte = 8'h00;
    reg        t29_esp32_go = 1'b0;       // pulse HIGH to trigger one ESP32 response
    reg        t29_esp32_done = 1'b0;     // ESP32 sets HIGH when current response done
    integer    t29_init_round = 0;        // which INIT iteration (0-3)
    integer    t29_read_ok = 0;           // read_packet success flag

    // =========================================================================
    // DUT Instantiation
    // =========================================================================
    iwm u_iwm (
        .addr           (addr),
        .nDEVICE_SELECT (nDEVICE_SELECT),
        .data_in        (data_in),
        .fclk           (fclk),
        .Q3             (Q3),
        .nRES           (nRES),
        .phase          (phase),
        ._enbl1         (_enbl1),
        ._enbl2         (_enbl2),
        ._wrreq         (_wrreq),
        .rddata         (rddata),
        .sense          (sense),
        .wrdata         (wrdata),
        .data_out       (data_out),
        .q6_out         (q6_out),
        .q7_out         (q7_out)
    );

    // =========================================================================
    // Test Infrastructure
    // =========================================================================
    integer pass_count = 0;
    integer fail_count = 0;

    task assert_eq;
        input [63:0] actual;
        input [63:0] expected;
        input [255:0] msg;    // 32-char message (packed string)
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
    // Simulate a 6502 bus access to an IWM softswitch.
    //
    // The Apple II asserts /DEV for the duration of PHI0 low (~500 ns).
    // The IWM latches A0-A3 on the falling edge of /DEV.
    // For writes, data is sampled on the rising edge of (Q3 | /DEV).
    //
    // addr_val[3:0] maps to the softswitch offsets from $C080:
    //   0 = PHASE0 OFF    1 = PHASE0 ON
    //   2 = PHASE1 OFF    3 = PHASE1 ON
    //   ...
    //   C = Q6L            D = Q6H
    //   E = Q7L            F = Q7H
    //
    // A0 = data (set/clear), A[3:1] = which state bit
    // -------------------------------------------------------------------------
    task iwm_access;
        input [3:0] addr_val;
        begin
            // Setup address before /DEV assertion
            addr = addr_val;
            data_in = 8'h00;
            @(posedge fclk); #1;

            // Assert /DEV (falling edge triggers state latch)
            // #1 ensures ndev_prev samples the old HIGH value first
            nDEVICE_SELECT = 1'b0;

            // Hold /DEV low for ~500 ns (several fclk cycles)
            repeat (4) @(posedge fclk);

            // Deassert /DEV
            #1;
            nDEVICE_SELECT = 1'b1;

            // Wait for state to settle
            repeat (2) @(posedge fclk);
        end
    endtask

    // Write access — same as above but also drives data bus
    task iwm_write;
        input [3:0] addr_val;
        input [7:0] write_data;
        begin
            addr = addr_val;
            data_in = write_data;
            @(posedge fclk); #1;

            nDEVICE_SELECT = 1'b0;
            repeat (4) @(posedge fclk);

            #1;
            nDEVICE_SELECT = 1'b1;

            // Wait for write clock edge (rising edge of Q3|/DEV)
            repeat (3) @(posedge fclk);
        end
    endtask

    // =========================================================================
    // IWM Softswitch Address Constants (offset from $C080)
    // These match the Liron ROM equates in liron-if.asm
    // =========================================================================
    localparam PH0_OFF    = 4'h0;   // iwm_ph_0_off
    localparam PH0_ON     = 4'h1;   // iwm_ph_0_on
    localparam PH1_OFF    = 4'h2;   // iwm_ph_1_off
    localparam PH1_ON     = 4'h3;   // iwm_ph_1_on
    localparam PH2_OFF    = 4'h4;   // iwm_ph_2_off
    localparam PH2_ON     = 4'h5;   // iwm_ph_2_on
    localparam PH3_OFF    = 4'h6;   // iwm_ph_3_off
    localparam PH3_ON     = 4'h7;   // iwm_ph_3_on
    localparam MOTOR_OFF  = 4'h8;   // iwm_motor_off
    localparam MOTOR_ON   = 4'h9;   // iwm_motor_on
    localparam SEL_DRV1   = 4'hA;   // iwm_sel_drive_1
    localparam SEL_DRV2   = 4'hB;   // iwm_sel_drive_2
    localparam Q6L        = 4'hC;   // iwm_q6l
    localparam Q6H        = 4'hD;   // iwm_q6h
    localparam Q7L        = 4'hE;   // iwm_q7l
    localparam Q7H        = 4'hF;   // iwm_q7h

    // -------------------------------------------------------------------------
    // Send a byte on RDDATA using IWM-side polarity (idle HIGH, pulse LOW).
    // Each '1' bit = brief LOW pulse (~1 µs), '0' = stay HIGH.
    // Bit cell ≈ 4 µs (28 FCLK). MSB first (bit 7 → bit 0).
    // -------------------------------------------------------------------------
    task send_byte_rddata;
        input [7:0] byte_val;
        integer b;
        begin
            for (b = 7; b >= 0; b = b - 1) begin
                if (byte_val[b]) begin
                    rddata = 1'b0; #1000; rddata = 1'b1; #2900;
                end else begin
                    #3900;
                end
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Send a byte using the exact ESP32 SPI TX timing (1 MHz clock, MSB first).
    //
    // The ESP32's encode_spi_packet() maps each data byte to 4 SPI bytes.
    // TX SPI clock = 1 MHz (iwm_ll.cpp line 611: devcfg.clock_speed_hz = 1 * MHZ).
    // Each SPI byte holds 2 data bits:
    //   - first data bit  → SPI bit 6 (pulse at 1-2 µs into SPI byte)
    //   - second data bit → SPI bit 2 (pulse at 5-6 µs into SPI byte)
    // Each SPI byte = 8 bits × 1 µs = 8 µs.
    //
    // Result: each '1' data bit = 1 µs pulse, spaced 4 µs apart.
    // This is 4 µs per data bit — matching the IWM slow-mode spec exactly.
    //
    // SPI byte bit layout at 1 MHz (MSB first on wire):
    //   Bit:  7      6      5      4      3      2      1      0
    //   µs:   0-1    1-2    2-3    3-4    4-5    5-6    6-7    7-8
    //   Val:  0      d1     0      0      0      d2     0      0
    //
    // IWM-side polarity: idle HIGH, pulse LOW for '1' (after top-module invert).
    // -------------------------------------------------------------------------
    task send_byte_spi_timing;
        input [7:0] byte_val;
        integer b;
        integer pair;
        begin
            for (pair = 0; pair < 4; pair = pair + 1) begin
                // --- First data bit of pair (SPI bit 6) ---
                b = 7 - (pair * 2);
                // 1 µs gap at start of SPI byte (SPI bit 7 = 0)
                #1000;
                if (byte_val[b]) begin
                    rddata = 1'b0;  // 1 µs LOW pulse (SPI bit 6 = 1)
                    #1000;
                    rddata = 1'b1;
                end else begin
                    #1000;          // 1 µs idle (SPI bit 6 = 0)
                end
                // 3 µs gap (SPI bits 5,4,3 = 0)
                #3000;
                // --- Second data bit of pair (SPI bit 2) ---
                b = 7 - (pair * 2 + 1);
                if (byte_val[b]) begin
                    rddata = 1'b0;  // 1 µs LOW pulse (SPI bit 2 = 1)
                    #1000;
                    rddata = 1'b1;
                end else begin
                    #1000;          // 1 µs idle (SPI bit 2 = 0)
                end
                // 2 µs remaining (SPI bits 1,0 = 0)
                #2000;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Trigger auto-clear by simulating the ROM's LDA Q6L with /DEV assertion.
    // -------------------------------------------------------------------------
    task trigger_auto_clear;
        begin
            addr = Q6L;
            @(posedge fclk); #1;
            nDEVICE_SELECT = 1'b0;
            repeat (4) @(posedge fclk);
            #1;
            nDEVICE_SELECT = 1'b1;
            repeat (20) @(posedge fclk);
        end
    endtask

    // =========================================================================
    // Test Sequence
    // =========================================================================
    initial begin
        $dumpfile("iwm_ii_tb.vcd");
        $dumpvars(0, iwm_ii_tb);

        $display("");
        $display("==============================================");
        $display("IWM Address Decoder & State Machine — Testbench");
        $display("==============================================");
        $display("");

        // =====================================================================
        // TEST 1: Reset
        // =====================================================================
        $display("--- Test 1: Reset clears all state ---");

        // Hold reset for several cycles
        nRES = 1'b0;
        repeat (10) @(posedge fclk);
        nRES = 1'b1;
        repeat (2) @(posedge fclk);

        assert_eq(phase, 4'b0000, "phase after reset");
        assert_eq(_enbl1, 1'b1,   "enbl1 after reset (inactive)");
        assert_eq(_enbl2, 1'b1,   "enbl2 after reset (inactive)");
        assert_eq(q6_out, 1'b0,   "Q6 after reset");
        assert_eq(q7_out, 1'b0,   "Q7 after reset");

        // Q6=0, Q7=0, motor_on=0 → read all ones
        assert_eq(data_out, 8'hFF, "data_out = all ones (idle)");

        $display("");

        // =====================================================================
        // TEST 2: Individual state bits — PHASE0-3
        // =====================================================================
        $display("--- Test 2: Phase line control ---");

        // Set PHASE0 ON  (addr = 0001 → A[3:1]=000, A0=1)
        iwm_access(PH0_ON);
        assert_eq(phase, 4'b0001, "PHASE0 on");

        // Set PHASE2 ON
        iwm_access(PH2_ON);
        assert_eq(phase, 4'b0101, "PHASE0+2 on");

        // Clear PHASE0 OFF
        iwm_access(PH0_OFF);
        assert_eq(phase, 4'b0100, "PHASE2 only");

        // Clear PHASE2 OFF
        iwm_access(PH2_OFF);
        assert_eq(phase, 4'b0000, "all phases off");

        $display("");

        // =====================================================================
        // TEST 3: smartport_bus_enable (from liron-if.asm line 687-691)
        //   LDA iwm_ph_1_on,x    → set PHASE1
        //   LDA iwm_ph_3_on,x    → set PHASE3
        // =====================================================================
        $display("--- Test 3: smartport_bus_enable sequence ---");

        iwm_access(PH1_ON);
        assert_eq(phase, 4'b0010, "PHASE1 on (bus enable step 1)");

        iwm_access(PH3_ON);
        assert_eq(phase, 4'b1010, "PHASE1+3 on (bus enabled)");

        $display("");

        // =====================================================================
        // TEST 4: smartport_bus_disable (from liron-if.asm line 693-700)
        //   LDA iwm_ph_0_off,x
        //   LDA iwm_ph_1_off,x
        //   LDA iwm_ph_2_off,x
        //   LDA iwm_ph_3_off,x
        // =====================================================================
        $display("--- Test 4: smartport_bus_disable sequence ---");

        iwm_access(PH0_OFF);
        iwm_access(PH1_OFF);
        iwm_access(PH2_OFF);
        iwm_access(PH3_OFF);
        assert_eq(phase, 4'b0000, "all phases off (bus disabled)");

        $display("");

        // =====================================================================
        // TEST 5: Drive select and enable
        // =====================================================================
        $display("--- Test 5: Drive select & enable ---");

        // Select drive 1 (Drive-Sel=0) then motor on
        iwm_access(SEL_DRV1);
        iwm_access(MOTOR_ON);
        assert_eq(_enbl1, 1'b0, "enbl1 active (drive 1 selected, motor on)");
        assert_eq(_enbl2, 1'b1, "enbl2 inactive");

        // Switch to drive 2
        iwm_access(SEL_DRV2);
        assert_eq(_enbl1, 1'b1, "enbl1 inactive (drive 2 now selected)");
        assert_eq(_enbl2, 1'b0, "enbl2 active");

        // Motor off
        iwm_access(MOTOR_OFF);
        assert_eq(_enbl1, 1'b1, "enbl1 inactive (motor off)");
        assert_eq(_enbl2, 1'b1, "enbl2 inactive (motor off)");

        $display("");

        // =====================================================================
        // TEST 6: Register decode — Status register
        //   L6=1, L7=0 → read status register
        //   From liron-if.asm "Lc813: lda iwm_q7l,x ; read status reg"
        //   which is preceded by Q6H to set up the status read.
        // =====================================================================
        $display("--- Test 6: Status register read ---");

        // Drive 2 selected, motor ON, then Q6H + Q7L = status read
        iwm_access(SEL_DRV2);
        iwm_access(MOTOR_ON);
        iwm_access(Q6H);          // L6 = 1
        // Q7 is already 0 from reset
        assert_eq(q6_out, 1'b1,  "Q6=1 for status read");
        assert_eq(q7_out, 1'b0,  "Q7=0 for status read");

        // Status register: bits[4:0] = mode_reg[4:0] (all 0 after reset)
        //                  bit 5 = motor_on (1)
        //                  bit 6 = 0
        //                  bit 7 = SENSE input
        sense = 1'b1;
        @(posedge fclk);
        assert_eq(data_out, 8'hA0, "status: sense=1, motor=1, mode=00");

        sense = 1'b0;
        @(posedge fclk);
        assert_eq(data_out, 8'h20, "status: sense=0, motor=1, mode=00");

        // Clean up
        iwm_access(Q6L);          // L6 = 0
        iwm_access(MOTOR_OFF);

        $display("");

        // =====================================================================
        // TEST 7: Register decode — Handshake register
        //   L6=0, L7=1 → read handshake register
        //   From liron-if.asm "Lc82c: asl iwm_q6l,x / bcc Lc82c"
        //   (polling bit 7 of handshake for write buffer ready)
        // =====================================================================
        $display("--- Test 7: Handshake register read ---");

        iwm_access(Q7H);          // L7 = 1
        // L6 is already 0
        assert_eq(q7_out, 1'b1,  "Q7=1 for handshake read");
        assert_eq(q6_out, 1'b0,  "Q6=0 for handshake read");

        // Handshake: bits[5:0]=1, bit6=write_state, bit7=buffer_ready
        // After reset with no write activity: ready=1, write_state=0
        assert_eq(data_out[7], 1'b1, "handshake bit7: buffer ready");
        assert_eq(data_out[5:0], 6'b111111, "handshake bits[5:0] = 1s");

        // Clean up
        iwm_access(Q7L);          // L7 = 0

        $display("");

        // =====================================================================
        // TEST 8: Mode register write
        //   From liron-if.asm "iwm_write_mode_reg" (line 925-937):
        //     LDA iwm_motor_off,x     → motor off
        //     LDA iwm_q6h,x           → L6=1
        //     (then) STA iwm_q7h,x    → L7=1, writes mode reg (motor off)
        //     (verify) LDA iwm_q7l,x  → read back status bits[4:0]
        //
        //   The Liron ROM writes mode = $07 for SmartPort:
        //     bit 0=1 (latch), bit 1=1 (async), bit 2=1 (timer disable)
        // =====================================================================
        $display("--- Test 8: Mode register write ---");

        // Follow the exact Liron ROM sequence
        iwm_access(MOTOR_OFF);     // L4 = 0 (motor off, required for mode write)
        iwm_access(Q6H);           // L6 = 1
        // Now write mode reg: L6=1, L7=1, motor_off → sel_write_mode
        // STA iwm_q7h,x — this sets L7=1 AND writes data on write clock edge
        iwm_write(Q7H, 8'h07);    // Set L7=1, write mode = 0x07

        assert_eq(q6_out, 1'b1,   "Q6=1 after mode write setup");
        assert_eq(q7_out, 1'b1,   "Q7=1 after mode write");

        // Verify: read back via status register (Q6=1, Q7=0)
        iwm_access(Q7L);           // L7 = 0 → now reading status
        // Status bits[4:0] should reflect mode_reg[4:0] = 0x07
        assert_eq(data_out[4:0], 5'h07, "status[4:0] = mode 0x07");

        // Clean up
        iwm_access(Q6L);

        $display("");

        // =====================================================================
        // TEST 9: Write data register & handshake
        //   From liron-if.asm "Lc829" write loop:
        //     asl iwm_q6l,x    → poll handshake bit 7 (buffer ready)
        //     bcc Lc82c        → loop until ready
        //     sta iwm_q6h,x    → write data byte
        //
        //   Setup: Q6=1, Q7=1 (L7=1, L6=1) with motor ON = write load
        // =====================================================================
        $display("--- Test 9: Write data register ---");

        iwm_access(MOTOR_ON);      // Motor on
        iwm_access(SEL_DRV2);      // Select drive 2 (as Liron does)
        iwm_access(Q7H);           // L7 = 1
        // Q6 is 0 → handshake read (L7=1, L6=0)
        // Verify handshake shows ready
        assert_eq(data_out[7], 1'b1, "handshake ready before write");

        // Now set up for write: need L6=1, L7=1, motor_on
        iwm_access(Q6H);           // L6 = 1 → sel_write_data
        // Write a data byte
        iwm_write(Q6H, 8'hA5);    // STA iwm_q6h — write 0xA5

        // After write, check handshake (need L6=0, L7=1)
        iwm_access(Q6L);           // L6 = 0 → handshake read
        // Buffer should NOT be ready immediately after loading
        // (It goes not-ready when data is loaded)

        // Clean up
        iwm_access(Q7L);
        iwm_access(MOTOR_OFF);

        $display("");

        // =====================================================================
        // TEST 10: /WRREQ output
        //   Per spec: /WRREQ low when L7=1, motor_on, write_state
        // =====================================================================
        $display("--- Test 10: /WRREQ output ---");

        // Start clean
        assert_eq(_wrreq, 1'b1, "wrreq inactive at start");

        iwm_access(MOTOR_ON);
        iwm_access(Q6H);           // L6 = 1
        iwm_access(Q7H);           // L7 = 1, motor on → write load state
        iwm_write(Q6H, 8'hFF);    // Write data to enter write state

        // Now L7=1, motor_on=1, write_state should be 1
        // /WRREQ should be LOW (active)
        assert_eq(q7_out, 1'b1, "Q7=1 for wrreq check");

        // Clean up
        iwm_access(Q7L);
        iwm_access(Q6L);
        iwm_access(MOTOR_OFF);

        $display("");

        // =====================================================================
        // TEST 11: Full Liron bus init sequence
        //   From liron-if.asm:
        //     reset_smartport_bus (line 662-677):
        //       JSR smartport_bus_disable     → clear all phases
        //       LDA iwm_ph_0_on,x            → assert PHASE0
        //       LDA iwm_ph_2_on,x            → assert PHASE2
        //       (delay 80ms)
        //       JSR smartport_bus_disable     → clear all phases
        //       (delay 10ms)
        // =====================================================================
        $display("--- Test 11: Liron reset_smartport_bus sequence ---");

        // smartport_bus_disable
        iwm_access(PH0_OFF);
        iwm_access(PH1_OFF);
        iwm_access(PH2_OFF);
        iwm_access(PH3_OFF);
        assert_eq(phase, 4'b0000, "phases clear (disable step 1)");

        // Assert PH0 and PH2 (reset command)
        iwm_access(PH0_ON);
        iwm_access(PH2_ON);
        assert_eq(phase, 4'b0101, "PH0+PH2 = reset command");

        // (80ms delay would happen here in real code)

        // smartport_bus_disable again
        iwm_access(PH0_OFF);
        iwm_access(PH1_OFF);
        iwm_access(PH2_OFF);
        iwm_access(PH3_OFF);
        assert_eq(phase, 4'b0000, "phases clear (disable step 2)");

        $display("");

        // =====================================================================
        // TEST 12: Read all-ones (Q6=0, Q7=0, motor off)
        // =====================================================================
        $display("--- Test 12: Read all-ones state ---");

        // Everything should be cleared from previous test
        assert_eq(q6_out, 1'b0, "Q6=0");
        assert_eq(q7_out, 1'b0, "Q7=0");
        assert_eq(data_out, 8'hFF, "data_out = all ones (Q6=0,Q7=0,motor off)");

        $display("");

        // =====================================================================
        // TEST 13: Read shift register — receive a byte via RDDATA
        // =====================================================================
        // Simulate ESP32 sending byte $C3 (11000011) on RDDATA.
        // Inside the IWM, rddata is idle HIGH (top module inverts GPIO9).
        // A '1' bit = brief LOW pulse (~1 µs), a '0' bit = stays HIGH.
        // Bit cell ≈ 4 µs. Byte = 8 × 4 µs = 32 µs.
        //
        // Setup: motor ON, Q6=0, Q7=0 → sel_read_data (read data register)
        $display("--- Test 13: Read byte via RDDATA ($C3) ---");

        // First, write mode register to $07 (latch=1, async=1, timer_disable=1)
        iwm_access(MOTOR_OFF);
        iwm_access(Q6H);
        iwm_write(Q7H, 8'h07);
        iwm_access(Q7L);
        iwm_access(Q6L);

        // Now enter read mode: motor ON, Q6=0, Q7=0
        iwm_access(MOTOR_ON);
        // Q6 and Q7 already 0

        // Wait for clock divider to settle
        repeat (4) @(posedge fclk);

        // Send byte $C3 = 11000011 (MSB first on wire)
        // Each '1' bit = pull rddata LOW for ~1 µs, then back HIGH
        // Each '0' bit = stay HIGH for ~4 µs (one bit cell ≈ 28 FCLK ≈ 3.9 µs)

        // Bit 7 = 1 (pulse)
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;
        // Bit 6 = 1 (pulse)
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;
        // Bit 5 = 0 (no pulse, wait one bit cell)
        #3900;
        // Bit 4 = 0
        #3900;
        // Bit 3 = 0
        #3900;
        // Bit 2 = 0
        #3900;
        // Bit 1 = 1 (pulse)
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;
        // Bit 0 = 1 (pulse)
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;

        // Wait for the byte to propagate through the shift register
        repeat (20) @(posedge fclk);

        // Now read the data register: access Q6L while L6=0, L7=0, motor=1
        // The ROM does: LDA iwm_q6l,x / BPL loop
        // We check data_out directly (it's always driven by data_out_mux)
        assert_eq(data_out[7], 1'b1, "read byte D7=1 (valid)");
        assert_eq(data_out, 8'hC3, "read byte = $C3");

        $display("");

        // =====================================================================
        // TEST 14: Read data register auto-clear after valid read
        // =====================================================================
        // In async+latch mode, after the CPU reads the data register with
        // D7=1, the register clears after 14 FCLK (~2 µs).
        $display("--- Test 14: Read data register auto-clear ---");

        // Simulate the ROM's LDA Q6L,X — assert /DEV to trigger latch
        addr = Q6L;
        data_in = 8'h00;
        @(posedge fclk); #1;
        nDEVICE_SELECT = 1'b0;

        // Hold /DEV low for a few cycles (simulating the 6502 bus cycle)
        repeat (4) @(posedge fclk);

        // x7_latch should have captured D7=1 on dev_falling
        // Now deassert /DEV
        #1;
        nDEVICE_SELECT = 1'b1;

        // Wait for the 14-FCLK clear countdown to complete
        repeat (20) @(posedge fclk);

        // After clear, data register should be 0
        assert_eq(data_out[7], 1'b0, "D7=0 after auto-clear");
        assert_eq(data_out, 8'h00, "read_data_reg cleared");

        $display("");

        // =====================================================================
        // TEST 15: Read second byte ($FF = 11111111)
        // =====================================================================
        $display("--- Test 15: Read second byte ($FF) ---");

        // Send $FF = all 1s (8 pulses)
        repeat (4) @(posedge fclk);

        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 7
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 6
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 5
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 4
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 3
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 2
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 1
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 0

        repeat (20) @(posedge fclk);

        assert_eq(data_out[7], 1'b1, "2nd byte D7=1 (valid)");
        assert_eq(data_out, 8'hFF, "read byte = $FF");

        // Clean up: auto-clear by reading
        addr = Q6L;
        @(posedge fclk); #1;
        nDEVICE_SELECT = 1'b0;
        repeat (4) @(posedge fclk);
        #1;
        nDEVICE_SELECT = 1'b1;
        repeat (20) @(posedge fclk);

        // Clean up state
        iwm_access(MOTOR_OFF);

        $display("");

        // =====================================================================
        // TEST 16: Write shift register — WRDATA toggle output
        // =====================================================================
        // Enter write mode and write $C3 (11000011). Verify WRDATA toggles
        // for each '1' bit (4 toggles for 4 one-bits).
        //
        // Liron ROM sequence:
        //   motor_on, sel_drive_2, Q6H (L6=1), Q7H+data (L7=1, write mode)
        //   Poll Q6L bit 7 for buffer ready, then STA Q6H with data
        $display("--- Test 16: Write WRDATA toggle ($C3) ---");

        // Set mode = $07 first
        iwm_access(MOTOR_OFF);
        iwm_access(Q6H);
        iwm_write(Q7H, 8'h07);
        iwm_access(Q7L);
        iwm_access(Q6L);

        // Enter write mode: motor on, then set L7 (triggers write_mode_entry)
        iwm_access(MOTOR_ON);
        iwm_access(SEL_DRV2);
        iwm_access(Q6H);           // L6 = 1
        iwm_access(Q7H);           // L7 = 1, motor on → write mode entry

        // WRDATA should now be HIGH (initialized on write_mode_entry)
        repeat (2) @(posedge fclk);
        assert_eq(wrdata, 1'b1, "WRDATA starts HIGH in write mode");

        // Now write data: L7=1, L6=1, motor_on=1 → sel_write_data
        iwm_write(Q6H, 8'hC3);    // STA iwm_q6h — load $C3 into write buffer

        // Count WRDATA edges over the duration of one byte (8 bit cells)
        // Each bit cell = 28 FCLK ≈ 3.9 µs. Full byte = 224 FCLK + 14 delay.
        // We need to count transitions over ~260 FCLK cycles
        begin : count_edges_block
            integer edge_count;
            reg prev_wrdata;
            integer i;
            edge_count = 0;
            prev_wrdata = wrdata;
            for (i = 0; i < 300; i = i + 1) begin
                @(posedge fclk);
                if (wrdata !== prev_wrdata) begin
                    edge_count = edge_count + 1;
                    prev_wrdata = wrdata;
                end
            end
            // $C3 = 11000011 has 4 one-bits → 4 toggles
            assert_eq(edge_count, 4, "WRDATA edges = 4 for $C3");
        end

        // Clean up: exit write mode
        iwm_access(Q7L);           // L7 = 0
        iwm_access(Q6L);           // L6 = 0
        iwm_access(MOTOR_OFF);

        // WRDATA should return to idle LOW when not in write mode
        repeat (4) @(posedge fclk);
        assert_eq(wrdata, 1'b0, "WRDATA idle LOW after exit");

        $display("");

        // =====================================================================
        // TEST 17: Write handshake — buffer ready transitions
        // =====================================================================
        $display("--- Test 17: Write handshake ---");

        // Enter write mode: motor on, L6=1, then L7=1 (triggers entry)
        iwm_access(MOTOR_ON);
        iwm_access(Q6H);           // L6 = 1
        iwm_access(Q7H);           // L7 = 1 → write_mode_entry

        // Now write data: L7=1, L6=1, motor_on=1 → sel_write_data
        iwm_write(Q6H, 8'hFF);

        // Check handshake (L6=0, L7=1 → sel_read_handshake)
        iwm_access(Q6L);           // L6 = 0 → handshake read

        // After write_mode_entry, the 7-CLK delay runs, then
        // write_buffer_ready goes HIGH (buffer consumed into shift reg)
        // Wait for the delay + a couple extra cycles
        repeat (30) @(posedge fclk);

        assert_eq(data_out[7], 1'b1, "handshake: buffer ready after load");
        assert_eq(data_out[6], 1'b1, "handshake: in write state");

        // Write another byte (switch to L6=1 for write, then back to L6=0 for handshake)
        iwm_access(Q6H);           // L6 = 1 → sel_write_data
        iwm_write(Q6H, 8'hAA);

        // Buffer should now be NOT ready (just loaded)
        iwm_access(Q6L);           // L6 = 0 → handshake
        assert_eq(data_out[7], 1'b0, "handshake: buffer full after write");

        // Clean up
        iwm_access(Q7L);
        iwm_access(Q6L);
        iwm_access(MOTOR_OFF);

        $display("");

        // =====================================================================
        // TEST 18: Write underrun detection
        // =====================================================================
        // Enter write mode, write one byte, then DON'T write more.
        // After the shift register finishes, underrun should occur.
        $display("--- Test 18: Write underrun ---");

        iwm_access(MOTOR_ON);
        iwm_access(Q6H);           // L6 = 1
        iwm_access(Q7H);           // L7 = 1 → write_mode_entry
        // Now L7=1, L6=1, motor_on=1 → sel_write_data
        iwm_write(Q6H, 8'hFF);    // Write one byte to buffer

        // Wait for one full byte to shift out
        // 7 CLK delay + 8 bit cells × 14 CLK = 7 + 112 = 119 CLK = 238 FCLK
        // Plus some margin
        repeat (280) @(posedge fclk);

        // Check handshake: should show underrun (bit 6 = 0)
        iwm_access(Q6L);           // L6 = 0 → handshake read
        assert_eq(data_out[6], 1'b0, "underrun: write_state cleared");

        // /WRREQ should be inactive (high) after underrun
        assert_eq(_wrreq, 1'b1, "wrreq inactive after underrun");

        // Clean up: exit write mode (clears underrun)
        iwm_access(Q7L);
        iwm_access(Q6L);
        iwm_access(MOTOR_OFF);

        $display("");

        // =====================================================================
        // TEST 19: Write→Read transition (Liron post-write sequence)
        // =====================================================================
        // After sending a packet, the Liron ROM does:
        //   1. Wait for underrun (poll handshake bit 6)
        //   2. STA Q6H (L6=1, doesn't matter for write-done state)
        //   3. LDA Q7L (L7→0, exits write mode, reads status)
        //   4. LDA PH0_OFF (deassert REQ)
        //   5. LDA Q6L (L6→0, now in read-data mode)
        //   6. Poll LDA Q6L / BPL for incoming byte
        //
        // This tests that the write machinery fully resets and the read
        // shift register cleanly receives a byte after exiting write mode.
        $display("--- Test 19: Write->Read transition ---");

        // === WRITE PHASE ===
        // Set mode $07
        iwm_access(MOTOR_OFF);
        iwm_access(Q6H);
        iwm_write(Q7H, 8'h07);
        iwm_access(Q7L);
        iwm_access(Q6L);

        // Enter write mode and send one byte
        iwm_access(MOTOR_ON);
        iwm_access(SEL_DRV2);
        iwm_access(Q6H);           // L6 = 1
        iwm_access(Q7H);           // L7 = 1, write_mode_entry
        iwm_write(Q6H, 8'hC8);    // Write $C8 (PEND marker)

        // Wait for byte to shift out + underrun
        // 7 CLK delay + 8 × 14 CLK = 119 CLK = 238 FCLK + margin
        repeat (280) @(posedge fclk);

        // Poll handshake bit 6 (write_state) — should be 0 (underrun)
        iwm_access(Q6L);           // L6=0, L7=1 → handshake read
        assert_eq(data_out[6], 1'b0, "W→R: underrun occurred");

        // === TRANSITION: exit write mode (Liron ROM sequence) ===
        // STA Q6H — sets L6=1 (data write to IWM, but write is done)
        iwm_access(Q6H);
        // LDA Q7L — sets L7=0, exits write mode. L6=1, L7=0 → status read
        iwm_access(Q7L);

        // At this point write_mode should be false, wrdata should be idle
        assert_eq(wrdata, 1'b0, "W→R: wrdata idle after Q7L");

        // LDA Q6L — sets L6=0. L6=0, L7=0, motor=1 → read data register
        iwm_access(Q6L);

        // === READ PHASE ===
        // Now send a byte on rddata as if ESP32 is responding
        // Send $C3 (sync byte)
        repeat (4) @(posedge fclk);

        // Wait for bit cell counter to pass the runt dead zone (≥7 CLK = 14 FCLK).
        // In real SmartPort, the ESP32 sends 6 sync bytes ($FF) before PBEGIN,
        // which establishes bit cell timing and takes the counter well past the
        // dead zone. Simulate this settling with a 16 FCLK (~2.2 µs) wait.
        repeat (16) @(posedge fclk);

        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 7 = 1
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 6 = 1
        #3900;                                          // bit 5 = 0
        #3900;                                          // bit 4 = 0
        #3900;                                          // bit 3 = 0
        #3900;                                          // bit 2 = 0
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 1 = 1
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 0 = 1

        assert_eq(data_out[7], 1'b1, "W→R: read D7=1 (valid)");
        assert_eq(data_out, 8'hC3, "W→R: read byte = $C3");

        // Clean up by triggering auto-clear
        addr = Q6L;
        @(posedge fclk); #1;
        nDEVICE_SELECT = 1'b0;
        repeat (4) @(posedge fclk);
        #1;
        nDEVICE_SELECT = 1'b1;
        repeat (20) @(posedge fclk);

        iwm_access(MOTOR_OFF);
        $display("");

        // =====================================================================
        // TEST 20: Read→Write transition
        // =====================================================================
        // Receive a byte, then switch to write mode and send a byte.
        // Verifies the read shift register state doesn't interfere with write.
        $display("--- Test 20: Read->Write transition ---");

        // === READ PHASE ===
        iwm_access(MOTOR_ON);
        iwm_access(SEL_DRV2);
        // L6=0, L7=0, motor=1 → read data register
        // Wait for counter to pass runt zone (≥7 CLK = 14 FCLK)
        repeat (16) @(posedge fclk);

        // Send $FF on rddata
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 7
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 6
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 5
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 4
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 3
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 2
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 1
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 0

        repeat (20) @(posedge fclk);
        assert_eq(data_out, 8'hFF, "R→W: read byte = $FF");

        // === TRANSITION: enter write mode ===
        // Liron ROM: STA Q7H (L7=1), which enters write mode
        // But first: Q6H to set L6=1 for write-load state
        iwm_access(Q6H);           // L6 = 1
        iwm_access(Q7H);           // L7 = 1, write_mode_entry

        // read_active should now be false (l7=1), read_sr should be cleared
        assert_eq(wrdata, 1'b1, "R→W: wrdata HIGH on entry");

        // === WRITE PHASE ===
        iwm_write(Q6H, 8'hAA);    // Load $AA into write buffer

        // Count edges: $AA = 10101010 → 4 one-bits → 4 toggles
        begin : rw_count_edges
            integer edge_count;
            reg prev_wrdata;
            integer i;
            edge_count = 0;
            prev_wrdata = wrdata;
            for (i = 0; i < 300; i = i + 1) begin
                @(posedge fclk);
                if (wrdata !== prev_wrdata) begin
                    edge_count = edge_count + 1;
                    prev_wrdata = wrdata;
                end
            end
            assert_eq(edge_count, 4, "R→W: WRDATA edges = 4 for $AA");
        end

        // Clean up
        iwm_access(Q7L);
        iwm_access(Q6L);
        iwm_access(MOTOR_OFF);
        repeat (4) @(posedge fclk);

        $display("");

        // =====================================================================
        // TEST 21: SmartPort INIT round-trip (write cmd → read response)
        // =====================================================================
        // Simulates the core of a SmartPort INIT exchange:
        //   1. Bus enable (PH1+PH3)
        //   2. Mode register write ($07)
        //   3. Motor on, select drive 2
        //   4. Write sync + PBEGIN ($C3) as the "command" start
        //   5. Wait for underrun
        //   6. Transition to read mode (the exact Liron ROM sequence)
        //   7. Receive PBEGIN ($C3) from "ESP32" as response start
        //   8. Receive a status byte ($80 = host address)
        //
        // This is the minimum viable SmartPort conversation.
        $display("--- Test 21: SmartPort INIT round-trip ---");

        // --- Bus enable (smartport_bus_enable) ---
        iwm_access(PH1_ON);
        iwm_access(PH3_ON);
        assert_eq(phase, 4'b1010, "INIT: bus enabled (PH1+PH3)");

        // --- Mode register write ($07) ---
        iwm_access(MOTOR_OFF);
        iwm_access(Q6H);
        iwm_write(Q7H, 8'h07);
        // returns with L6=1, L7=1, motor_off → mode write done
        iwm_access(Q7L);
        iwm_access(Q6L);

        // --- Select drive 2, motor on ---
        iwm_access(SEL_DRV2);
        iwm_access(MOTOR_ON);

        // --- Assert REQ (PH0_ON) ---
        iwm_access(PH0_ON);
        assert_eq(phase[0], 1'b1, "INIT: REQ asserted (PH0)");

        // --- Enter write mode ---
        iwm_access(Q6H);           // L6 = 1
        iwm_access(Q7H);           // L7 = 1, write_mode_entry

        // --- Write sync byte $FF then PBEGIN $C3 ---
        iwm_write(Q6H, 8'hFF);    // First sync byte

        // Wait for buffer ready (shift reg consumed the byte)
        repeat (30) @(posedge fclk);
        // Poll handshake: L6=0, L7=1 → handshake read
        iwm_access(Q6L);
        assert_eq(data_out[7], 1'b1, "INIT: handshake ready for 2nd byte");

        // Write PBEGIN marker
        iwm_access(Q6H);
        iwm_write(Q6H, 8'hC3);

        // Wait for both bytes to shift out + underrun
        repeat (560) @(posedge fclk);   // 2 bytes × 238 FCLK + margin

        // --- Wait for underrun (Liron ROM: poll handshake bit 6) ---
        iwm_access(Q6L);
        assert_eq(data_out[6], 1'b0, "INIT: write underrun after cmd");

        // --- Transition to read mode (exact Liron ROM sequence) ---
        // STA Q6H (L6=1)
        iwm_access(Q6H);
        // LDA Q7L (L7→0, exits write mode)
        iwm_access(Q7L);
        assert_eq(wrdata, 1'b0, "INIT: wrdata idle after write exit");

        // --- Deassert REQ ---
        iwm_access(PH0_OFF);
        assert_eq(phase[0], 1'b0, "INIT: REQ deasserted");

        // LDA Q6L (L6→0, now reading data register)
        iwm_access(Q6L);

        // --- Simulate ESP32 sending response ---
        // ESP32 would assert ACK, then after REQ re-asserts, send data.
        // For this test we skip the ACK/REQ handshake and just send bytes.

        // Re-assert REQ for the response phase
        iwm_access(PH0_ON);

        // Send PBEGIN ($C3) from "ESP32"
        // Wait for counter to pass runt zone (≥7 CLK = 14 FCLK)
        repeat (16) @(posedge fclk);
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 7 = 1
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 6 = 1
        #3900;                                          // bit 5 = 0
        #3900;                                          // bit 4 = 0
        #3900;                                          // bit 3 = 0
        #3900;                                          // bit 2 = 0
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 1 = 1
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 0 = 1

        repeat (20) @(posedge fclk);
        assert_eq(data_out[7], 1'b1, "INIT: PBEGIN D7=1 (valid)");
        assert_eq(data_out, 8'hC3, "INIT: PBEGIN = $C3");

        // Trigger auto-clear (simulating the LDA Q6L the ROM does)
        addr = Q6L;
        @(posedge fclk); #1;
        nDEVICE_SELECT = 1'b0;
        repeat (4) @(posedge fclk);
        #1;
        nDEVICE_SELECT = 1'b1;
        repeat (20) @(posedge fclk);

        // Send second byte: DEST = $80 (host address)
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 7 = 1
        #3900;                                          // bit 6 = 0
        #3900;                                          // bit 5 = 0
        #3900;                                          // bit 4 = 0
        #3900;                                          // bit 3 = 0
        #3900;                                          // bit 2 = 0
        #3900;                                          // bit 1 = 0
        #3900;                                          // bit 0 = 0

        repeat (20) @(posedge fclk);
        assert_eq(data_out[7], 1'b1, "INIT: DEST D7=1 (valid)");
        assert_eq(data_out, 8'h80, "INIT: DEST = $80 (host)");

        // --- Clean up ---
        iwm_access(PH0_OFF);
        iwm_access(PH1_OFF);
        iwm_access(PH3_OFF);
        iwm_access(Q6L);
        iwm_access(MOTOR_OFF);

        $display("");

        // =====================================================================
        // TEST 22: Leading zeros before first '1' — shift register accumulation
        // =====================================================================
        // If noise or partial sync causes zero bits before the first valid byte,
        // the shift register should accumulate them harmlessly until a '1'
        // reaches the MSB. Send 6 zero bits (timeouts) then $C3 (11000011).
        // The zeros fill the SR but never set bit 7, so no false byte complete.
        // When $C3 arrives, the SR resets on completion and latches $C3.
        $display("--- Test 22: Leading zeros before first '1' ---");

        iwm_access(MOTOR_OFF);
        iwm_access(Q6H);
        iwm_write(Q7H, 8'h07);
        iwm_access(Q7L);
        iwm_access(Q6L);
        iwm_access(MOTOR_ON);

        // Wait for counter to pass runt zone
        repeat (16) @(posedge fclk);

        // Send 6 zero-bit cells (no edges, just timeouts)
        // Each timeout = 21 CLK = 42 FCLK ≈ 5.9 µs
        repeat (6) begin
            // Wait for timeout at counter=21 (each timeout is ~42 FCLK from reset-to-7)
            #3900;  // ~4 µs per zero-bit cell
        end

        // Wait for all timeouts to propagate
        repeat (10) @(posedge fclk);

        // Data register should still be empty (no byte complete from zeros)
        assert_eq(data_out[7], 1'b0, "leading 0s: no false byte");

        // Now send $C3 — this is the real byte
        send_byte_rddata(8'hC3);
        repeat (20) @(posedge fclk);

        assert_eq(data_out[7], 1'b1, "leading 0s: D7=1 after $C3");
        assert_eq(data_out, 8'hC3, "leading 0s: byte = $C3");

        trigger_auto_clear;
        iwm_access(MOTOR_OFF);

        $display("");

        // =====================================================================
        // TEST 23: Consecutive zeros — $81 (10000001)
        // =====================================================================
        // Tests the interval counter's "reset to 7" window widening with
        // 6 consecutive zero bits between two '1' bits.
        $display("--- Test 23: Consecutive zeros ($81 = 10000001) ---");

        iwm_access(MOTOR_ON);
        repeat (16) @(posedge fclk);

        send_byte_rddata(8'h81);
        repeat (20) @(posedge fclk);

        assert_eq(data_out[7], 1'b1, "$81: D7=1 (valid)");
        assert_eq(data_out, 8'h81, "$81: byte = $81");

        trigger_auto_clear;
        iwm_access(MOTOR_OFF);

        $display("");

        // =====================================================================
        // TEST 24: Runt pulse rejection
        // =====================================================================
        // A brief pulse during the dead zone (counter 0-6 CLK) should be
        // rejected — no lft1 generated. Send a valid '1' bit, then
        // immediately send a runt pulse (within ~2 CLK = 4 FCLK), then
        // send 7 more '1' bits. If runt is rejected we get $FF.
        // If runt is counted we'd get a shifted/wrong byte.
        $display("--- Test 24: Runt pulse rejection ---");

        iwm_access(MOTOR_ON);
        repeat (16) @(posedge fclk);

        // Send bit 7 = 1 (valid pulse)
        rddata = 1'b0; #1000; rddata = 1'b1;

        // Runt: pulse arrives ~280 ns after last edge (≈2 CLK, in dead zone 0-6)
        #280;
        rddata = 1'b0; #200; rddata = 1'b1;

        // Resume normal timing for remaining 7 bits (all '1's)
        #2420;  // fill out remainder of first bit cell
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 6
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 5
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 4
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 3
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 2
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 1
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 0

        repeat (20) @(posedge fclk);

        assert_eq(data_out[7], 1'b1, "runt: D7=1 (valid)");
        assert_eq(data_out, 8'hFF, "runt: byte = $FF (runt rejected)");

        trigger_auto_clear;
        iwm_access(MOTOR_OFF);

        $display("");

        // =====================================================================
        // TEST 25: Back-to-back multi-byte read stream (firehose)
        // =====================================================================
        // Simulates ESP32 streaming 3 consecutive bytes with no gap:
        //   PBEGIN ($C3) + DEST ($80) + SRC ($85)
        // The Liron ROM reads each byte with LDA Q6L / BPL polling.
        // Auto-clear must fire and complete between each byte read.
        $display("--- Test 25: Back-to-back 3-byte read stream ---");

        iwm_access(MOTOR_ON);
        repeat (16) @(posedge fclk);

        // --- Byte 1: PBEGIN $C3 ---
        send_byte_rddata(8'hC3);
        repeat (20) @(posedge fclk);

        assert_eq(data_out[7], 1'b1, "stream[0]: D7=1");
        assert_eq(data_out, 8'hC3, "stream[0]: $C3");

        // Simulate ROM's LDA Q6L (triggers auto-clear)
        trigger_auto_clear;

        // Verify register cleared
        assert_eq(data_out[7], 1'b0, "stream[0]: cleared");

        // --- Byte 2: DEST $80 ---
        send_byte_rddata(8'h80);
        repeat (20) @(posedge fclk);

        assert_eq(data_out[7], 1'b1, "stream[1]: D7=1");
        assert_eq(data_out, 8'h80, "stream[1]: $80");

        trigger_auto_clear;
        assert_eq(data_out[7], 1'b0, "stream[1]: cleared");

        // --- Byte 3: SRC $85 ---
        send_byte_rddata(8'h85);
        repeat (20) @(posedge fclk);

        assert_eq(data_out[7], 1'b1, "stream[2]: D7=1");
        assert_eq(data_out, 8'h85, "stream[2]: $85");

        trigger_auto_clear;
        iwm_access(MOTOR_OFF);

        $display("");

        // =====================================================================
        // TEST 26: Write double-buffering — verify both bytes' serial output
        // =====================================================================
        // Write $C3 (4 one-bits) then $AA (4 one-bits) back-to-back.
        // Total WRDATA edges should be 8 (4+4).
        $display("--- Test 26: Write double-buffer serial output ---");

        iwm_access(MOTOR_OFF);
        iwm_access(Q6H);
        iwm_write(Q7H, 8'h07);
        iwm_access(Q7L);
        iwm_access(Q6L);

        iwm_access(MOTOR_ON);
        iwm_access(SEL_DRV2);
        iwm_access(Q6H);           // L6 = 1
        iwm_access(Q7H);           // L7 = 1, write_mode_entry

        // Write first byte: $C3
        iwm_write(Q6H, 8'hC3);

        // Wait for buffer ready (first byte loaded into shift reg)
        repeat (30) @(posedge fclk);

        // Write second byte $AA while first is still shifting.
        // Need L6=1 for sel_write_data (still set from above).
        iwm_write(Q6H, 8'hAA);

        // Now wait for BOTH bytes to shift out + underrun.
        // 7 CLK delay + 2 × 8 × 14 CLK = 7 + 224 = 231 CLK = 462 FCLK.
        // Minus the ~30 FCLK already waited = ~432 + margin.
        repeat (500) @(posedge fclk);

        // Verify underrun occurred (both bytes shifted out, no more data)
        iwm_access(Q6L);
        assert_eq(data_out[6], 1'b0, "dblbuf: underrun after 2 bytes");

        // Clean up
        iwm_access(Q7L);
        iwm_access(Q6L);
        iwm_access(MOTOR_OFF);

        $display("");

        // =====================================================================
        // TEST 27: Read byte with ESP32 actual SPI timing (4 µs data bits)
        // =====================================================================
        // The ESP32 encode_spi_packet() maps each data byte to 4 SPI bytes.
        // TX SPI clock = 1 MHz (iwm_ll.cpp line 611), producing 4 µs per
        // data bit — matching the IWM slow-mode spec. Verifies the read
        // data extractor works with the real ESP32 SPI encoding format
        // (1 µs pulses at SPI bit positions 6 and 2 within each SPI byte).
        $display("--- Test 27: Read byte with ESP32 SPI timing ($C3) ---");

        iwm_access(MOTOR_OFF);
        iwm_access(Q6H);
        iwm_write(Q7H, 8'h07);
        iwm_access(Q7L);
        iwm_access(Q6L);
        iwm_access(MOTOR_ON);

        // Wait for counter to pass runt zone
        repeat (16) @(posedge fclk);

        // Send $C3 using actual ESP32 SPI encoding (4 µs per data bit)
        send_byte_spi_timing(8'hC3);

        // Wait for byte to propagate
        repeat (30) @(posedge fclk);

        assert_eq(data_out[7], 1'b1, "SPI timing: D7=1 (valid)");
        assert_eq(data_out, 8'hC3, "SPI timing: byte = $C3");

        trigger_auto_clear;
        iwm_access(MOTOR_OFF);

        $display("");

        // =====================================================================
        // TEST 28: ESP32 SPI timing — consecutive bytes
        // =====================================================================
        // Send 3 bytes using ESP32 SPI encoding: $C3, $80, $85
        $display("--- Test 28: ESP32 SPI timing multi-byte ---");

        iwm_access(MOTOR_ON);
        repeat (16) @(posedge fclk);

        // Byte 1: $C3
        send_byte_spi_timing(8'hC3);
        repeat (30) @(posedge fclk);
        assert_eq(data_out[7], 1'b1, "SPI multi[0]: D7=1");
        assert_eq(data_out, 8'hC3, "SPI multi[0]: $C3");
        trigger_auto_clear;

        // Byte 2: $80
        send_byte_spi_timing(8'h80);
        repeat (30) @(posedge fclk);
        assert_eq(data_out[7], 1'b1, "SPI multi[1]: D7=1");
        assert_eq(data_out, 8'h80, "SPI multi[1]: $80");
        trigger_auto_clear;

        // Byte 3: $85
        send_byte_spi_timing(8'h85);
        repeat (30) @(posedge fclk);
        assert_eq(data_out[7], 1'b1, "SPI multi[2]: D7=1");
        assert_eq(data_out, 8'h85, "SPI multi[2]: $85");
        trigger_auto_clear;

        iwm_access(MOTOR_OFF);

        $display("");

        // =====================================================================
        // TEST 29: Race — sr_byte_complete coincides with dev_falling
        // =====================================================================
        // On real hardware, the ROM's LDA Q6L,X can land on the exact FCLK
        // cycle where the shift register completes a byte. When this happens,
        // x7_latch must sample the INCOMING byte's MSB (sr_next[7]), not the
        // stale read_data_reg[7] (which is 0 from a prior auto-clear).
        // Without the fix, x7_latch gets 0 and auto-clear never fires.
        //
        // Strategy: send a byte, wait for auto-clear, then send a second byte
        // and time the /DEV assertion to coincide with sr_byte_complete.
        // We calibrate by monitoring u_iwm.sr_byte_complete.
        $display("--- Test 29: Race — sr_byte_complete vs dev_falling ---");

        iwm_access(MOTOR_ON);
        repeat (8) @(posedge fclk);

        // Send first byte $C3 and trigger auto-clear normally
        send_byte_spi_timing(8'hC3);
        repeat (30) @(posedge fclk);
        assert_eq(data_out, 8'hC3, "race29: first byte $C3");
        trigger_auto_clear;

        // Verify register cleared
        assert_eq(u_iwm.read_data_reg, 8'h00, "race29: cleared after $C3");

        // Now send $85 and ALIGN dev_falling with sr_byte_complete.
        // First, send the byte in the background using a fork-like approach:
        // we just send it and then hunt for the exact sr_byte_complete edge.
        //
        // Send $85 = 10000101 — 8 data bits via SPI timing
        send_byte_spi_timing(8'h85);

        // Now spin waiting for sr_byte_complete, then IMMEDIATELY assert /DEV
        // on the SAME fclk edge. We poll at fclk rate.
        begin : race29_hunt
            for (t29_iter = 0; t29_iter < 500; t29_iter = t29_iter + 1) begin
                @(posedge fclk); #1;
                if (u_iwm.sr_byte_complete) begin
                    // sr_byte_complete is HIGH right now (just committed).
                    // The read_data_reg is being loaded with sr_next on THIS edge.
                    // Assert /DEV on the NEXT edge — this is 1 fclk after byte load.
                    // To hit the same-edge race, we needed to have /DEV go low
                    // BEFORE this edge. But we can't go back in time.
                    //
                    // Instead: the byte is now in read_data_reg with D7=1.
                    // But x7_latch hasn't been set yet (no dev_falling).
                    // Wait 0 more fclk — assert /DEV immediately.
                    // dev_falling will fire on the next posedge fclk.
                    // At that point, read_data_reg has $85 (from sr_byte_complete
                    // one cycle ago). x7_latch <= read_data_reg[7] = 1. Good.
                    //
                    // The REAL race to test: dev_falling on the SAME edge as
                    // sr_byte_complete. To achieve this, we need /DEV to have
                    // gone low in the previous cycle. Let's detect when sr is
                    // about to complete instead.
                    t29_iter = 500;  // break
                end
            end
        end

        // OK different approach: we know the byte takes a deterministic number
        // of FCLK after send_byte_spi_timing returns. Let's measure it, then
        // replay with the /DEV assertion precisely timed.
        //
        // Actually, let's do this properly. We need to set up the /DEV BEFORE
        // the byte completes, so they coincide. The approach:
        // 1. Send a byte and record when sr_byte_complete fires
        // 2. Trigger auto-clear, reset state
        // 3. Send the same byte again
        // 4. Assert /DEV at exactly the right time so dev_falling == sr_byte_complete

        // Step 1: measure byte completion time
        // Auto-clear and reset
        trigger_auto_clear;
        repeat (30) @(posedge fclk);

        // Send $85 and measure when sr_byte_complete fires
        begin : race29_measure
            integer measure_start;
            integer measure_sbc_time;
            measure_sbc_time = 0;

            // Record start time
            measure_start = $time;

            send_byte_spi_timing(8'h85);

            // Wait for byte to complete
            for (t29_iter = 0; t29_iter < 500; t29_iter = t29_iter + 1) begin
                @(posedge fclk); #1;
                if (u_iwm.sr_byte_complete && measure_sbc_time == 0) begin
                    measure_sbc_time = $time - measure_start;
                    $display("  race29: sr_byte_complete at +%0d ps from send start", measure_sbc_time);
                end
            end

            // Trigger auto-clear
            trigger_auto_clear;
            assert_eq(u_iwm.read_data_reg, 8'h00, "race29: cleared for replay");

            // Step 2: replay — send same byte, but assert /DEV at the measured time
            // We need /DEV low ONE fclk before sr_byte_complete so that dev_falling
            // fires on the same edge. That means nDEV goes low at (sbc_time - 1 fclk).
            //
            // Since send_byte_spi_timing is blocking, we can't easily interleave.
            // Use a different approach: pre-position addr=Q6L, send the byte,
            // then count fclk cycles and assert /DEV at the right moment.

            // First: how many FCLK from end of send_byte_spi_timing to sr_byte_complete?
            // The SPI timing task takes deterministic time. After it returns,
            // the byte propagates through sync + bit cell counter + shift register.
            // Let's just measure fclk count from send start.

            // Send the byte again and count fclk edges to sr_byte_complete
            send_byte_spi_timing(8'h85);
            begin : race29_count
                integer fclk_count;
                fclk_count = 0;
                for (t29_iter = 0; t29_iter < 500; t29_iter = t29_iter + 1) begin
                    @(posedge fclk);
                    fclk_count = fclk_count + 1;
                    #1;
                    if (u_iwm.sr_byte_complete) begin
                        $display("  race29: sr_byte_complete at fclk +%0d after send", fclk_count);
                        t29_iter = 500;  // break
                    end
                end
            end

            trigger_auto_clear;
        end

        // Step 3: THE ACTUAL RACE TEST
        // Now we know the fclk offset. We replay with /DEV going low 1 fclk before.
        // But send_byte_spi_timing blocks... we need concurrency.
        // Use rddata directly with precise timing instead.
        //
        // $85 = 10000101
        // SPI encoding: 4 pairs × 8µs = 32µs total
        // After the last edge, the byte completes after propagation through
        // the synchronizer (3 fclk) + bit cell recognition.
        //
        // Simpler approach: send the byte, then IMMEDIATELY start polling.
        // The first poll where D7=1 exercises the dev_falling + already-loaded case.
        // But the HARD race is dev_falling + sr_byte_complete on SAME edge.
        //
        // Let's just set up /DEV low BEFORE sending the last bit of the byte.
        // When the last bit shifts in, sr_byte_complete fires with /DEV already low.

        // Send first 7 bits of $85 (1000010_) using SPI timing
        // $85 = 10000101 → pairs: (1,0), (0,0), (0,1), (0,1)
        // Send 3 complete pairs (6 bits: 100001) + first bit of pair 4 (0)
        // Then we pause, assert /DEV, and send the final bit (1) which completes the byte.

        // Pair 0: bits 7,6 = 1,0
        #1000;
        rddata = 1'b0; #1000; rddata = 1'b1;  // bit 7 = 1
        #3000;
        #1000;                                  // bit 6 = 0
        #2000;

        // Pair 1: bits 5,4 = 0,0
        #1000;
        #1000;                                  // bit 5 = 0
        #3000;
        #1000;                                  // bit 4 = 0
        #2000;

        // Pair 2: bits 3,2 = 0,1
        #1000;
        #1000;                                  // bit 3 = 0
        #3000;
        rddata = 1'b0; #1000; rddata = 1'b1;  // bit 2 = 1
        #2000;

        // Pair 3 first bit: bit 1 = 0
        #1000;
        #1000;                                  // bit 1 = 0
        #3000;

        // NOW: 7 bits shifted in (1000010). SR should be 7'b1000010 = $42.
        // The 8th bit (bit 0 = 1) will make sr_next = {$42[6:0], 1} = $85.
        // sr_next[7] = 1 → sr_byte_complete fires.
        //
        // Assert /DEV LOW now, THEN send the final bit.
        // dev_falling fires on the next posedge fclk after /DEV goes low.
        // When the final bit's falling edge propagates through the 3-stage
        // synchronizer and bit cell counter, sr_byte_complete fires.
        // We need dev_falling to land on that same fclk.

        addr = Q6L;
        @(posedge fclk); #1;
        nDEVICE_SELECT = 1'b0;

        // Send final bit (bit 0 = 1) while /DEV is low
        rddata = 1'b0; #1000; rddata = 1'b1;
        #2000;

        // Wait for byte to propagate through synchronizer + extractor
        // (3 fclk sync + bit cell recognition ≈ 10-15 fclk)
        // During this time, dev_falling has already fired (1st fclk after /DEV low).
        // We need to keep /DEV low long enough for the auto-clear mechanism to work.
        repeat (20) @(posedge fclk);

        // Sample the data
        t29_read_byte = data_out;
        #1;
        nDEVICE_SELECT = 1'b1;

        $display("  race29 result: byte=$%02h x7=%b cp=%b reg=$%02h",
                 t29_read_byte, u_iwm.x7_latch, u_iwm.clear_pending, u_iwm.read_data_reg);

        // The byte should be readable
        assert_eq(t29_read_byte[7], 1'b1, "race29: D7=1 (valid byte)");
        assert_eq(t29_read_byte, 8'h85, "race29: byte = $85");

        // x7_latch should be set (either from dev_falling seeing reg[7]=1,
        // or from the sr_byte_complete coincidence fix)
        assert_eq(u_iwm.x7_latch, 1'b1, "race29: x7_latch set");

        // Auto-clear should be pending or already done
        // (clear_pending=1 means countdown in progress, which is correct)
        // Wait for clear to complete
        repeat (20) @(posedge fclk);
        assert_eq(u_iwm.read_data_reg, 8'h00, "race29: auto-clear completed");
        assert_eq(u_iwm.x7_latch, 1'b0, "race29: x7_latch cleared");

        iwm_access(MOTOR_OFF);

        $display("");

        // =====================================================================
        // TEST 30: Race — lft1/lft0 hot when read_active drops
        // =====================================================================
        // On real hardware, when the ROM exits read mode (e.g., Q6H to read
        // status), read_active drops to 0. If lft1 or lft0 happened to be
        // asserted on the previous cycle, sr_byte_complete could fire on the
        // same edge that read_active drops — loading a garbage partial byte
        // into read_data_reg. Without the read_active gate on sr_byte_complete,
        // this garbage byte (with D7=1) would fool the next read cycle.
        //
        // Strategy: send data so that a falling edge is being processed right
        // when we do Q6H to exit read mode. Then re-enter read mode and verify
        // read_data_reg is clean.
        $display("--- Test 30: Race — lft pulse when read_active drops ---");

        iwm_access(MOTOR_ON);
        repeat (8) @(posedge fclk);

        // Send several '1' bits to fill the shift register to just below
        // completion (7 bits shifted, MSB about to become 1).
        // SR after 6 '1' bits: 00111111. Next '1' makes 01111111 (MSB=0 still).
        // After 7 '1' bits: SR gets 01111111, then bit_in=1 makes sr_next=11111111
        // → sr_byte_complete fires! So we need exactly 7 '1' pulses, and then
        // exit read mode right as the 7th pulse's lft1 fires.
        //
        // Actually, let's think more carefully. The first '1' bit: sr was 00000000,
        // sr_next = 00000001 (MSB=0, no complete). Continue...
        // After 7 '1' bits: sr = 01111111, next '1' → sr_next = 11111111,
        // sr_byte_complete fires.
        //
        // So we send 7 '1' pulses (SR = 0111_1111), then time the 8th pulse
        // to arrive at the EXACT fclk where we also do Q6H.

        // Send 7 '1' pulses with ~4µs spacing (bit cell timing)
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 1
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 2
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 3
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 4
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 5
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 6
        rddata = 1'b0; #1000; rddata = 1'b1; #2900;  // bit 7

        // SR should now be 0111_1111 ($7F). Verify:
        @(posedge fclk); #1;
        $display("  race30: SR after 7 pulses = $%02h", u_iwm.read_sr);

        // Now send the 8th pulse. This will make sr_byte_complete fire
        // when the edge propagates through the synchronizer (~3 fclk).
        // We want to exit read mode (Q6H) on that same fclk.
        //
        // Send the 8th falling edge
        rddata = 1'b0;
        #1000;
        rddata = 1'b1;

        // The falling edge propagates through the 3-stage synchronizer:
        //   fclk+0: rddata_sync1 captures LOW
        //   fclk+1: rddata_sync2 = LOW
        //   fclk+2: rddata_sync3 = old HIGH, rddata_sync2 = LOW → rddata_falling!
        //   fclk+2: lft1 fires (if bc_counter >= 7)
        //   fclk+3: shift_pulse active, sr_byte_complete may fire
        //
        // Wait 2 fclk for the edge to reach rddata_falling, then exit read mode
        // on the fclk where lft1 fires. That way lft1=1 is "hot" when
        // read_active drops.
        @(posedge fclk);
        @(posedge fclk);

        // On the NEXT posedge, lft1 should fire. Exit read mode NOW by
        // putting addr=Q6H — the dev_falling on next posedge will set L6=1,
        // making read_active=0.
        addr = Q6H;
        nDEVICE_SELECT = 1'b0;

        // This next posedge: dev_falling sets L6=1 (read_active→0),
        // AND lft1 might be hot from the 8th pulse.
        @(posedge fclk); #1;
        $display("  race30: at exit edge: lft1=%b lft0=%b sbc=%b sr=%02h reg=%02h ra=%b",
                 u_iwm.lft1, u_iwm.lft0, u_iwm.sr_byte_complete,
                 u_iwm.read_sr, u_iwm.read_data_reg, u_iwm.read_active);

        repeat (3) @(posedge fclk);
        #1;
        nDEVICE_SELECT = 1'b1;

        // Now read_active is 0 (L6=1). The shift register should be reset.
        // CRITICAL CHECK: read_data_reg should NOT have a garbage byte.
        // If sr_byte_complete was NOT gated by read_active, it could have
        // loaded $FF (from 8 consecutive '1' bits) into read_data_reg.
        $display("  race30: after exit: reg=$%02h x7=%b",
                 u_iwm.read_data_reg, u_iwm.x7_latch);

        // The register should be $00 (or whatever it had before, NOT $FF)
        assert_eq(u_iwm.read_data_reg[7], 1'b0, "race30: no garbage D7");

        // Re-enter read mode and verify clean state
        iwm_access(Q6L);                 // L6=0 → read_active=1
        repeat (4) @(posedge fclk);

        // Poll — should see D7=0 (no valid byte)
        assert_eq(data_out[7], 1'b0, "race30: clean after re-enter");

        iwm_access(MOTOR_OFF);

        $display("");

        // =====================================================================
        // TEST 31: Full SmartPort INIT x4 — exact ROM + ESP32 protocol sequence
        // =====================================================================
        // Simulates 4 consecutive INIT round-trips matching the exact sequence
        // the Liron ROM and ESP32 FujiNet execute during boot enumeration.
        // Each iteration: write_packet (command) → read_packet (response).
        //
        // ROM reference: liron-if.asm
        //   INIT loop at Lce19 (line 1337) — loops for each device
        //   smartport_bus_write_packet (line 343) — sends INIT command
        //   smartport_bus_read_packet  (line 578) — receives response
        //   read_packet success exit (line 312-320) — Q6H, poll Q7L, PH0_OFF
        //
        // ESP32 reference: iwm_ll.cpp
        //   iwm_send_packet_spi (line 205) — sends response via SPI
        //
        // Drive 83 fails on hardware after drives 81-82 succeed (2.log).
        // This test exercises 4 consecutive round-trips to reproduce that.
        $display("--- Test 29: Full SmartPort INIT x4 — exact ROM + ESP32 sequence ---");

        for (t29_init_round = 0; t29_init_round < 4; t29_init_round = t29_init_round + 1) begin
            $display("  === INIT round %0d ===", t29_init_round);

            // Reset per-round state
            t29_found_c3 = 0;
            t29_read_ok = 0;

            // =================================================================
            // WRITE PHASE: smartport_bus_write_packet (line 343)
            // =================================================================

            // ROM: jsr smartport_bus_enable (PH1=1, PH3=1)
            iwm_access(PH1_ON);
            iwm_access(PH3_ON);

            // ROM: iwm_write_mode_reg (line 925)
            //   motor_off → Q6H → STA Q7H with $07 → LDA Q7L (verify)
            iwm_access(MOTOR_OFF);
            iwm_access(Q6H);
            iwm_write(Q7H, 8'h07);
            iwm_access(Q7L);
            // Now: L6=1, L7=0, motor_off, mode=$07

            // ROM: select drive, motor on
            iwm_access(SEL_DRV2);
            iwm_access(MOTOR_ON);
            // Now: L6=1, L7=0, motor_on=1

            // ROM: poll status for ACK deasserted (sense HIGH)
            // ESP32 ACK is deasserted between rounds
            sense = 1'b1;
            iwm_access(Q7L);

            // ROM: assert REQ (PH0=1)
            iwm_access(PH0_ON);

            // ROM: enter write mode — STA Q7H with $FF (L6 already 1)
            iwm_write(Q7H, 8'hFF);

            // Wait for write init delay
            repeat (20) @(posedge fclk);

            // ROM: poll handshake, write sync byte $FF
            iwm_access(Q6L);
            iwm_write(Q6H, 8'hFF);

            // Wait for first byte to shift out
            repeat (240) @(posedge fclk);

            // ROM: poll handshake, write PBEGIN $C3
            iwm_access(Q6L);
            iwm_write(Q6H, 8'hC3);

            // Wait for bytes to shift out + underrun
            repeat (560) @(posedge fclk);

            // ROM: post-write underrun poll
            iwm_access(Q6L);

            // ROM: Q6H to prepare for status read
            iwm_access(Q6H);

            // ROM: post-write ACK poll — wait for ESP32 to assert ACK (sense LOW)
            sense = 1'b0;
            iwm_access(Q7L);

            // ROM: deassert REQ
            iwm_access(PH0_OFF);

            // ROM: Q6L (final write_packet cleanup, line 556)
            iwm_access(Q6L);
            // Now: L6=0, L7=0, motor_on=1, PH0=0

            // =================================================================
            // READ PHASE: smartport_bus_read_packet (line 578)
            // =================================================================

            // ROM: buffer setup (no IWM accesses)
            // ROM: jsr smartport_bus_enable (PH1=1, PH3=1 — already set)
            iwm_access(PH1_ON);
            iwm_access(PH3_ON);

            // ROM: Q6H (line 597) — L6=1
            iwm_access(Q6H);

            // ROM: poll Q7L for ACK deasserted (sense HIGH)
            // ESP32: iwm_ack_set() → sense goes HIGH (ready to send)
            sense = 1'b1;
            iwm_access(Q7L);

            // ROM: assert REQ for response (line 600)
            iwm_access(PH0_ON);

            // Trigger ESP32 response
            t29_esp32_go = 1'b1;
            @(posedge fclk);
            t29_esp32_go = 1'b0;  // pulse — ESP32 block detects rising edge

            // ROM: BPL polling loop — search for $C3 sync (max 30 iterations)
            for (t29_iter = 0; t29_iter < 30; t29_iter = t29_iter + 1) begin
                if (!t29_found_c3) begin
                    addr = Q6L;
                    @(posedge fclk); #1;
                    nDEVICE_SELECT = 1'b0;
                    repeat (4) @(posedge fclk);
                    t29_read_byte = data_out;
                    #1;
                    nDEVICE_SELECT = 1'b1;
                    #3000;
                    if (t29_read_byte[7]) begin
                        if (t29_read_byte == 8'hC3)
                            t29_found_c3 = 1;
                        #11000;
                    end else begin
                        #3000;
                    end
                end
            end

            if (!t29_found_c3)
                $display("    FAIL: no $C3 sync found");

            // Read DEST, SRC, TYPE
            if (t29_found_c3) begin
                // DEST ($80 = host)
                t29_read_byte = 8'h00;
                for (t29_iter = 0; t29_iter < 200; t29_iter = t29_iter + 1) begin
                    if (!t29_read_byte[7]) begin
                        addr = Q6L;
                        @(posedge fclk); #1;
                        nDEVICE_SELECT = 1'b0;
                        repeat (4) @(posedge fclk);
                        t29_read_byte = data_out;
                        #1;
                        nDEVICE_SELECT = 1'b1;
                        #3000;
                        if (t29_read_byte[7]) #4000; else #3000;
                    end
                end
                $display("    DEST=$%02h", t29_read_byte);
                assert_eq(t29_read_byte, 8'h80, "INIT29: DEST = $80");

                // SRC ($81 + round)
                t29_read_byte = 8'h00;
                for (t29_iter = 0; t29_iter < 200; t29_iter = t29_iter + 1) begin
                    if (!t29_read_byte[7]) begin
                        addr = Q6L;
                        @(posedge fclk); #1;
                        nDEVICE_SELECT = 1'b0;
                        repeat (4) @(posedge fclk);
                        t29_read_byte = data_out;
                        #1;
                        nDEVICE_SELECT = 1'b1;
                        #3000;
                        if (t29_read_byte[7]) #4000; else #3000;
                    end
                end
                $display("    SRC=$%02h", t29_read_byte);
                assert_eq(t29_read_byte, 8'h81 + t29_init_round, "INIT29: SRC");

                // TYPE ($82)
                t29_read_byte = 8'h00;
                for (t29_iter = 0; t29_iter < 200; t29_iter = t29_iter + 1) begin
                    if (!t29_read_byte[7]) begin
                        addr = Q6L;
                        @(posedge fclk); #1;
                        nDEVICE_SELECT = 1'b0;
                        repeat (4) @(posedge fclk);
                        t29_read_byte = data_out;
                        #1;
                        nDEVICE_SELECT = 1'b1;
                        #3000;
                        if (t29_read_byte[7]) #4000; else #3000;
                    end
                end
                $display("    TYPE=$%02h", t29_read_byte);
                assert_eq(t29_read_byte, 8'h82, "INIT29: TYPE = $82");

                t29_read_ok = 1;
            end

            // Wait for ESP32 stimulus to finish this round
            if (!t29_esp32_done) @(posedge t29_esp32_done);
            t29_esp32_done = 1'b0;  // reset for next round

            // =================================================================
            // READ_PACKET EXIT: success path (liron-if.asm lines 312-320)
            // =================================================================
            if (t29_read_ok) begin
                // ROM: lda iwm_q6h,x (line 312) — L6=1
                iwm_access(Q6H);

                // ROM: poll iwm_q7l,x until sense LOW (ACK asserted)
                // ESP32: iwm_ack_clr() → sense LOW (done sending)
                sense = 1'b0;
                iwm_access(Q7L);

                // ROM: lda iwm_ph_0_off,x (line 317) — deassert REQ
                iwm_access(PH0_OFF);
                // ROM: clc / rts
            end

            // =================================================================
            // BACK IN INIT LOOP (line 1349-1350)
            // ROM: lda Z4d / beq Lce19
            // Z4d = packet status byte = $00 (success) → loop for next device
            // No IWM accesses here, just 6502 overhead
            // =================================================================
            #5000;  // ~5µs of 6502 overhead before next iteration
        end

        // Clean up
        iwm_access(PH0_OFF);
        iwm_access(Q6L);
        iwm_access(MOTOR_OFF);

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
    // Test 29 — ESP32 response stimulus (separate initial block)
    // =========================================================================
    // Sends SmartPort INIT response for each round when triggered.
    // Runs concurrently with the ROM polling loop in the main initial block.
    // Each round: sync pattern + PBEGIN + header bytes with per-round SRC.
    initial begin : esp32_stimulus
        integer esp_round;
        for (esp_round = 0; esp_round < 4; esp_round = esp_round + 1) begin
            @(posedge t29_esp32_go);

            // ESP32 overhead: process REQ + enable_output
            // Vary per round by half-FCLK steps to probe worst-case alignment.
            // FCLK half-period ≈ 70ns. These offsets shift the entire SPI
            // waveform relative to the IWM's clk_div phase.
            case (esp_round)
                0: #2000;
                1: #2035;   // +35ns (quarter FCLK)
                2: #2070;   // +70ns (half FCLK)
                3: #2105;   // +105ns (3/4 FCLK)
            endcase

            // Sync pattern (same as real ESP32)
            send_byte_spi_timing(8'hFF);
            send_byte_spi_timing(8'h3F);
            send_byte_spi_timing(8'hCF);
            send_byte_spi_timing(8'hF3);
            send_byte_spi_timing(8'hFC);
            send_byte_spi_timing(8'hFF);

            // PBEGIN marker
            send_byte_spi_timing(8'hC3);

            // Packet header
            send_byte_spi_timing(8'h80);                    // DEST (host)
            send_byte_spi_timing(8'h81 + esp_round);        // SRC (device N)
            send_byte_spi_timing(8'h82);                    // TYPE

            t29_esp32_done = 1'b1;
        end
    end

    // =========================================================================
    // Timeout watchdog
    // =========================================================================
    initial begin
        #60_000_000;  // 60 ms — 4 round-trips need more time
        $display("ERROR: Testbench timed out!");
        $finish;
    end

endmodule
