`timescale 1ns / 1ps
// =============================================================================
// IWM Write Serializer Testbench
// =============================================================================
// Simulates the Liron ROM's exact write sequence and verifies that the IWM
// write serializer produces correct FM-encoded wrdata output.
//
// The ROM's write handshake loop:
//   1. ASL iwm_q6l,x  — polls handshake register bit 7 (writeBufferEmpty)
//   2. BCC loop        — loops until writeBufferEmpty=1
//   3. STA iwm_q6h,x  — writes data byte (Q7=1, Q6=1, A0=1, motorOn=1)
//
// FM encoding (IWM spec):
//   Each byte is serialized MSB-first. Each bit cell is 100 wr_clk (25MHz)
//   = 4.000µs, exactly matching ESP32's SPI sample rate (APB_CLK/40).
//   A '1' bit produces a wrdata transition at the bit-cell midpoint.
//   A '0' bit produces no transition.
//   The ESP32 detects transitions (edges) to decode '1' bits.
//
// SmartPort sync sequence (sent first): $3F, $CF, $F3, $FC, $FF, $C3
// Then command bytes follow.
// =============================================================================

module iwm_write_tb;

    // =========================================================================
    // Parameters
    // =========================================================================
    localparam CLK_7M_PERIOD = 140;     // 7.16 MHz ~ 140ns period
    localparam FCLK_HALF     = CLK_7M_PERIOD / 2;
    localparam CLK_25M_PERIOD = 40;     // 25 MHz = 40ns period
    localparam CLK_25M_HALF   = CLK_25M_PERIOD / 2;

    // IWM register addresses (addr[3:0] as seen by the IWM)
    // addr[3:1] selects register, addr[0] is value
    localparam PH0_OFF  = 4'b0000;  // phase[0] = 0
    localparam PH0_ON   = 4'b0001;  // phase[0] = 1
    localparam PH1_OFF  = 4'b0010;  // phase[1] = 0
    localparam PH1_ON   = 4'b0011;  // phase[1] = 1
    localparam PH2_OFF  = 4'b0100;  // phase[2] = 0
    localparam PH2_ON   = 4'b0101;  // phase[2] = 1
    localparam PH3_OFF  = 4'b0110;  // phase[3] = 0
    localparam PH3_ON   = 4'b0111;  // phase[3] = 1
    localparam MOTOR_OFF= 4'b1000;  // motorOn = 0
    localparam MOTOR_ON = 4'b1001;  // motorOn = 1
    localparam DRV1     = 4'b1010;  // driveSelect = 0
    localparam DRV2     = 4'b1011;  // driveSelect = 1
    localparam Q6L      = 4'b1100;  // Q6 = 0
    localparam Q6H      = 4'b1101;  // Q6 = 1
    localparam Q7L      = 4'b1110;  // Q7 = 0
    localparam Q7H      = 4'b1111;  // Q7 = 1

    // =========================================================================
    // Signals
    // =========================================================================
    reg         fclk;
    reg         wr_clk;
    reg         Q3;
    reg         nRES;
    reg  [3:0]  addr;
    reg         nDEVICE_SELECT;
    reg  [7:0]  data_in;
    wire [7:0]  data_out;
    wire        wrdata;
    wire [3:0]  phase;
    wire        _wrreq;
    wire        _enbl1;
    wire        _enbl2;
    reg         sense;
    reg         rddata;
    wire        q7_out;

    // FM decode state
    integer     wrdata_edge_count;
    integer     bit_cell_count;
    reg         wrdata_prev;
    reg  [7:0]  decoded_byte;
    integer     decoded_bit_idx;
    integer     decoded_byte_count;
    reg  [7:0]  expected_bytes [0:31];
    integer     expected_count;
    integer     errors;

    // Timing capture
    integer     last_edge_time;
    integer     current_time;

    // =========================================================================
    // DUT Instantiation
    // =========================================================================
    iwm dut (
        .addr           (addr),
        .nDEVICE_SELECT (nDEVICE_SELECT),
        .fclk           (fclk),
        .Q3             (Q3),
        .nRES           (nRES),
        .data_in        (data_in),
        .data_out       (data_out),
        .wrdata         (wrdata),
        .phase          (phase),
        ._wrreq         (_wrreq),
        ._enbl1         (_enbl1),
        ._enbl2         (_enbl2),
        .sense          (sense),
        .rddata         (rddata),
        .wr_clk         (wr_clk),
        .q7_out         (q7_out)
    );

    // =========================================================================
    // Clock Generation
    // =========================================================================
    initial fclk = 0;
    always #(FCLK_HALF) fclk = ~fclk;

    initial wr_clk = 0;
    always #(CLK_25M_HALF) wr_clk = ~wr_clk;  // 25 MHz

    initial Q3 = 0;
    always #250 Q3 = ~Q3;  // ~2 MHz

    // =========================================================================
    // Bus Access Tasks
    // =========================================================================
    // These simulate the 6502's bus timing for IWM register accesses.
    // A real bus access takes ~7 fclk cycles with nDEVICE_SELECT low.

    // IWM state register access (level-detect for phases/motor/drive,
    // edge-detect for Q6/Q7). This simulates LDA $C0Cx,x
    task iwm_access;
        input [3:0] reg_addr;
    begin
        // Setup address before nDEV falls
        addr = reg_addr;
        data_in = 8'h00;
        @(posedge fclk);
        // nDEVICE_SELECT goes low (6502 address decode complete)
        nDEVICE_SELECT = 0;
        // Hold for ~7 fclk cycles (like real bus access)
        repeat(7) @(posedge fclk);
        nDEVICE_SELECT = 1;
        // Bus turnaround
        repeat(3) @(posedge fclk);
    end
    endtask

    // IWM data write access (STA $C0CF,x — Q7=1 Q6=1 A0=1 motorOn=1)
    // This writes a byte to the IWM's write data register.
    task iwm_write_byte;
        input [7:0] byte_val;
    begin
        addr = 4'b1111;  // Q7H with A0=1 — but Q6/Q7 are edge-detect,
                         // the actual write condition is:
                         // ~nDEVICE_SELECT & q7 & q6 & addr[0]
                         // Q7 and Q6 must already be set.
        data_in = byte_val;
        @(posedge fclk);
        nDEVICE_SELECT = 0;
        repeat(7) @(posedge fclk);
        nDEVICE_SELECT = 1;
        repeat(3) @(posedge fclk);
    end
    endtask

    // Read IWM register and return value
    task iwm_read_reg;
        input [3:0] reg_addr;
        output [7:0] result;
    begin
        addr = reg_addr;
        data_in = 8'h00;
        @(posedge fclk);
        nDEVICE_SELECT = 0;
        repeat(4) @(posedge fclk);
        result = data_out;
        repeat(3) @(posedge fclk);
        nDEVICE_SELECT = 1;
        repeat(3) @(posedge fclk);
    end
    endtask

    // Poll handshake register until writeBufferEmpty=1, then write byte.
    // This simulates the ROM's exact handshake loop:
    //   ASL iwm_q6l,x   (read handshake, bit 7 = writeBufferEmpty)
    //   BCC loop         (loop while bit 7 = 0)
    //   STA iwm_q6h,x   (write data)
    task rom_handshake_write;
        input [7:0] byte_val;
        reg [7:0] handshake;
        integer poll_count;
    begin
        poll_count = 0;
        // Poll handshake register (Q7=1, Q6=0, A0=0 → addr=1100)
        // The ROM reads with ASL which is a read-modify-write, but
        // for the IWM it's just a read of the handshake register.
        forever begin
            iwm_read_reg(Q6L, handshake);  // Read handshake (Q7=1, Q6=0)
            poll_count = poll_count + 1;
            if (handshake[7] == 1'b1) begin
                // writeBufferEmpty — write the byte
                // ROM does: STA iwm_q6h,x (addr = Q6H with A0=1 = 1101...
                // but actually the write goes to $C0CF = addr[3:0] = 1111)
                // Wait — the ROM's STA goes to iwm_q6h which is $C08D+slot.
                // That's addr[3:1]=110, addr[0]=1 → 4'b1101.
                // But writeCondNow = ~nDEVICE_SELECT & q7 & q6 & addr[0]
                // q7=1 (already set), q6... the STA to $C08D sets q6=1 on
                // the falling edge, and addr[0]=1. So writeCondNow fires.
                //
                // Actually: the ROM accesses $C08D (iwm_q6h) with the STA.
                // That address has addr[3:1]=110 (Q6), addr[0]=1.
                // The edge-detect sets q6=1. Then writeCondNow checks q7&q6&addr[0].
                // q7 is already 1 from earlier. q6 just went to 1. addr[0]=1.
                // So writeCondNow goes TRUE.
                //
                // But wait — the handshake read was at Q6L (addr=1100), which
                // set q6=0 on the falling edge of nDEVICE_SELECT. Then the
                // write is at Q6H (addr=1101), which sets q6=1.
                //
                // Let's just simulate what the ROM does:
                // Read at $C08C (Q6L) then write at $C08D (Q6H)
                iwm_write_byte_via_q6h(byte_val);
                disable rom_handshake_write;
            end
            if (poll_count > 1000) begin
                $display("    ERROR: handshake poll timeout for byte $%02X", byte_val);
                errors = errors + 1;
                disable rom_handshake_write;
            end
        end
    end
    endtask

    // Write data byte via STA iwm_q6h,x ($C08D+slot → addr[3:0] = 1101)
    // This sets Q6=1 on the edge-detect AND writes the data byte.
    task iwm_write_byte_via_q6h;
        input [7:0] byte_val;
    begin
        addr = Q6H;  // 4'b1101 — Q6=1, A0=1
        data_in = byte_val;
        @(posedge fclk);
        nDEVICE_SELECT = 0;
        repeat(7) @(posedge fclk);
        nDEVICE_SELECT = 1;
        repeat(3) @(posedge fclk);
    end
    endtask

    // =========================================================================
    // FM Decoder — monitors wrdata and decodes bytes
    // =========================================================================
    // SmartPort FM encoding:
    //   Bit cell = 100 wr_clk @ 25MHz = 4.000µs (exact ESP32 SPI match)
    //   '1' bit = wrdata transition (toggle) at bit-cell midpoint
    //   '0' bit = no transition for one bit cell
    //
    // We detect transitions on wrdata and measure the interval between them.
    // Interval ~4000ns = '1' followed by '1' (back-to-back transitions)
    // Interval ~8000ns = '1' followed by '0' then '1'
    // Interval ~12000ns = '1' followed by '00' then '1'
    // etc.

    reg  [7:0] fm_decoded_bytes [0:63];
    integer    fm_byte_count;
    integer    fm_bit_count;
    reg  [7:0] fm_shift_reg;
    integer    fm_last_edge_ns;
    integer    fm_idle_count;
    reg        fm_wrdata_prev;
    reg        fm_active;
    integer    fm_edge_num;

    initial begin
        fm_byte_count = 0;
        fm_bit_count = 0;
        fm_shift_reg = 8'h00;
        fm_last_edge_ns = 0;
        fm_idle_count = 0;
        fm_wrdata_prev = 1'b1;
        fm_active = 0;
        fm_edge_num = 0;
    end

    always @(posedge wr_clk) begin
        if (fm_active) begin
            // Stop decoding when underrun fires — the serializer forces
            // wrdata=0 on underrun, which isn't a data transition.
            if (dut._underrun == 1'b0) begin
                fm_active <= 0;
            end
            else if (wrdata !== fm_wrdata_prev) begin
                // Transition detected — every transition is a '1' bit.
                // The interval since the last edge tells us how many '0'
                // bits precede this '1'.
                fm_edge_num = fm_edge_num + 1;
                if (fm_last_edge_ns == 0) begin
                    // Very first transition — no reference point, so we
                    // can't know how many zeros preceded it. For SmartPort
                    // the first byte is $FF (all ones), so just shift in
                    // the '1' bit with no preceding zeros.
                    $display("  FM edge %0d: t=%0dns (first, wrdata=%b→%b)", fm_edge_num, $time, fm_wrdata_prev, wrdata);
                    fm_shift_reg = {fm_shift_reg[6:0], 1'b1};
                    fm_bit_count = fm_bit_count + 1;
                    if (fm_bit_count == 8) begin
                        fm_decoded_bytes[fm_byte_count] = fm_shift_reg;
                        $display("  FM decoded byte %0d: $%02X", fm_byte_count, fm_shift_reg);
                        fm_byte_count = fm_byte_count + 1;
                        fm_bit_count = 0;
                    end
                end
                else begin
                    // Calculate interval since last edge
                    current_time = $time;
                    begin : calc_bits
                        integer interval_ns;
                        integer bit_cells;
                        integer i;
                        interval_ns = current_time - fm_last_edge_ns;
                        // Round to nearest bit cell count
                        // 100 wr_clk * 40ns = 4000ns per bit cell
                        bit_cells = (interval_ns + 2000) / 4000;
                        if (bit_cells < 1) bit_cells = 1;

                        $display("  FM edge %0d: t=%0dns interval=%0dns cells=%0d (wrdata=%b→%b)",
                                 fm_edge_num, $time, interval_ns, bit_cells, fm_wrdata_prev, wrdata);

                        // Shift in (bit_cells-1) zeros then one 1
                        for (i = 0; i < bit_cells - 1; i = i + 1) begin
                            fm_shift_reg = {fm_shift_reg[6:0], 1'b0};
                            fm_bit_count = fm_bit_count + 1;
                            if (fm_bit_count == 8) begin
                                fm_decoded_bytes[fm_byte_count] = fm_shift_reg;
                                $display("  FM decoded byte %0d: $%02X", fm_byte_count, fm_shift_reg);
                                fm_byte_count = fm_byte_count + 1;
                                fm_bit_count = 0;
                            end
                        end
                        // The '1' bit
                        fm_shift_reg = {fm_shift_reg[6:0], 1'b1};
                        fm_bit_count = fm_bit_count + 1;
                        if (fm_bit_count == 8) begin
                            fm_decoded_bytes[fm_byte_count] = fm_shift_reg;
                            $display("  FM decoded byte %0d: $%02X", fm_byte_count, fm_shift_reg);
                            fm_byte_count = fm_byte_count + 1;
                            fm_bit_count = 0;
                        end
                    end
                end
                fm_last_edge_ns = $time;
                fm_idle_count = 0;
            end
            else begin
                fm_idle_count = fm_idle_count + 1;
            end
            fm_wrdata_prev = wrdata;
        end
    end

    // =========================================================================
    // Test Sequence
    // =========================================================================
    reg [7:0] handshake_val;
    integer i;

    // SmartPort INIT command packet bytes (after sync):
    // dest=$81 (device 1), src=$80 (host), type=$85 (INIT cmd type)
    // Then: count, odd-byte-count, groups...
    // For a minimal INIT, let's use the exact bytes from the log:
    // packet: FCFFFFFFFFC381808080808281808582
    // That's the raw IWM-level bytes. The sync is $FF,$FC... and $C3 is the sync mark.
    // After $C3: $81, $80, $80, $80, $80, $82, $81, $80, $85, $82
    //
    // But the ROM sends sync + data differently. Let me use the exact sync sequence
    // and a simple command packet.

    // Expected bytes the serializer should output:
    // Sync: $3F, $CF, $F3, $FC, $FF, $C3
    // Then: $81 (dest=1|$80), $80 (src=host)
    // For this test, we'll send exactly these 8 bytes and verify.

    initial begin
        $dumpfile("iwm_write_tb.vcd");
        $dumpvars(0, iwm_write_tb);

        // Initialize
        addr = 4'b0000;
        data_in = 8'h00;
        nDEVICE_SELECT = 1;
        nRES = 0;
        sense = 1;       // ACK deasserted (HIGH)
        rddata = 1;      // Idle HIGH
        errors = 0;

        $display("");
        $display("===========================================");
        $display("IWM Write Serializer Testbench");
        $display("===========================================");
        $display("");

        // =====================================================================
        // RESET
        // =====================================================================
        repeat(20) @(posedge fclk);
        nRES = 1;
        repeat(10) @(posedge fclk);

        $display("After reset:");
        $display("  wrdata = %b (expect 1 = idle HIGH)", wrdata);
        $display("  _wrreq = %b (expect 1 = deasserted)", _wrreq);

        // =====================================================================
        // SET UP IWM FOR SMARTPORT WRITE
        // Exact ROM sequence from smartport_bus_write_packet (liron-if.asm)
        // =====================================================================
        $display("");
        $display("--- Setting up IWM for SmartPort write ---");

        // Step 1: smartport_bus_enable — set phase[1]=1, phase[3]=1
        $display("  Setting phase[1]=1, phase[3]=1 (bus enable)");
        iwm_access(PH1_ON);    // LDA iwm_ph_1_on,x
        iwm_access(PH3_ON);    // LDA iwm_ph_3_on,x

        // Step 2: iwm_write_mode_reg — ROM sequence:
        //   LDA iwm_motor_off,x  → motorOn=0
        //   LDA iwm_q6h,x        → Q6=1
        //   TYA; STA iwm_q7h,x   → Q7=1, writes modeReg (motorOn=0)
        //   TYA; EOR iwm_q7l,x   → Q7=0, reads status, verify mode
        //   (loop if mismatch)
        $display("  Writing mode register $07");
        iwm_access(MOTOR_OFF);  // LDA iwm_motor_off,x
        iwm_access(Q6H);       // LDA iwm_q6h,x → Q6=1

        // STA iwm_q7h,x with data=$07 → sets Q7=1, writes modeReg=$07
        addr = Q7H;
        data_in = 8'h07;
        @(posedge fclk);
        nDEVICE_SELECT = 0;
        repeat(7) @(posedge fclk);
        nDEVICE_SELECT = 1;
        repeat(3) @(posedge fclk);

        // EOR iwm_q7l,x → read status (Q7L with A0=0), sets Q7=0
        iwm_access(Q7L);       // Q7=0

        // Read status to verify mode wrote correctly
        // Status at Q7=0, Q6=1: {sense, 0, enableActive, modeReg[4:0]}
        iwm_read_reg(4'b0000, handshake_val);
        $display("  Status register = $%02X (mode[4:0] should be $07)", handshake_val);

        // IMPORTANT: Wait for q7_stable to fall. The mode reg write set
        // Q7=1 briefly, triggering q7_stable. In real hardware the ~236
        // fclk of 6502 instructions between mode reg write and write-mode
        // entry is enough for Q7_FALL_THRESH (100 fclk). Our compressed
        // testbench only takes ~66 fclk. Wait here to match real timing.
        $display("  Waiting for q7_stable to clear (Q7_FALL_THRESH=100)...");
        repeat(120) @(posedge fclk);
        $display("    q7_stable = %b (expect 0)", dut.q7_stable);

        // Step 3: Select drive 2 — LDA iwm_sel_drive_2,x
        $display("  Selecting drive 2");
        iwm_access(DRV2);

        // Step 4: Motor on — LDA iwm_motor_on,x
        $display("  Motor ON");
        iwm_access(MOTOR_ON);

        $display("  _enbl1 = %b, _enbl2 = %b", _enbl1, _enbl2);
        $display("  _wrreq = %b", _wrreq);

        // Step 5: ACK poll — LDA iwm_q7l,x; BMI (wait for sense HIGH)
        // ROM reads status at Q7L (Q7=0), Q6 is still 1 from step 2.
        // Status bit 7 = sense. In our TB sense=1, so immediate pass.
        // Note: The ROM reads iwm_q7l,x which has addr[3:1]=7, addr[0]=0.
        // Edge-detect: q7 <= 0 (already 0, no change). Level-detect: no effect.
        iwm_read_reg(4'b1110, handshake_val);  // addr=1110 = Q7L, A0=0
        $display("  Status (ACK check) = $%02X (bit7 = sense = %b)", handshake_val, handshake_val[7]);

        // Step 6: Assert REQ — LDA iwm_ph_0_on,x
        $display("  Asserting REQ (phase[0]=1)");
        iwm_access(PH0_ON);
        $display("  phases = %b", phase);

        // Step 7: LDA #$ff; STA iwm_q7h,x
        // Sets Q7=1 on edge detect AND writes $FF to buffer via
        // writeCondNow (q7=1, q6=1, addr[0]=1, motorOn=1).
        // The serializer init (q7_stable 0→1) loads writeShifter=$FF
        // for the first partial cell, then loads buffer=$FF at byte boundary.
        $display("  STA iwm_q7h with A=$FF (enter write mode + first buffer load)");
        addr = Q7H;     // 4'b1111 — Q7=1, A0=1
        data_in = 8'hFF; // The ROM has A=$FF from LDA #$ff
        @(posedge fclk);
        nDEVICE_SELECT = 0;
        repeat(7) @(posedge fclk);
        nDEVICE_SELECT = 1;
        repeat(3) @(posedge fclk);

        // =====================================================================
        // START FM DECODER — must start BEFORE q7_stable rises so we
        // don't miss the first wrdata toggles. The serializer init fires
        // when q7_stable goes 0→1 (8 fclk after Q7 goes HIGH), and the
        // first wrdata toggle is ~15 fclk after that.
        // =====================================================================
        fm_active = 1;
        fm_last_edge_ns = 0;
        fm_byte_count = 0;
        fm_bit_count = 0;
        fm_edge_num = 0;

        // Debug: check state right after the Q7H write
        repeat(3) @(posedge fclk);
        $display("  After Q7H write (+3 fclk):");
        $display("    q7=%b q6=%b motorOn=%b driveSelect=%b", dut.q7, dut.q6, dut.motorOn, dut.driveSelect);
        $display("    writeCondNow=%b writeCondPrev1=%b writeCondPrev2=%b", dut.writeCondNow, dut.writeCondPrev1, dut.writeCondPrev2);
        $display("    writeBuffer=$%02X writeBufferEmpty=%b _underrun=%b", dut.writeBuffer, dut.writeBufferEmpty, dut._underrun);
        $display("    q7_stable=%b q7_rise_ctr=%0d", dut.q7_stable, dut.q7_rise_ctr);
        $display("    sp_seen_req=%b sp_gate=%b", dut.sp_seen_req, dut.sp_gate);
        $display("    _enbl1=%b _enbl2=%b", _enbl1, _enbl2);

        // Wait for q7_stable to go HIGH (Q7_RISE_THRESH = 8 fclk)
        $display("  Waiting for q7_stable...");
        repeat(15) @(posedge fclk);
        $display("  After q7_stable wait (+15 more fclk):");
        $display("    q7_stable = %b", dut.q7_stable);
        $display("    _wrreq = %b (expect 0 = asserted)", _wrreq);
        $display("    wrdata = %b", wrdata);
        $display("    writeBufferEmpty=%b _underrun=%b", dut.writeBufferEmpty, dut._underrun);
        $display("    writeBitTimer=%0d writeBitCounter=%0d", dut.writeBitTimer, dut.writeBitCounter);
        $display("    writeShifter=$%02X writeBuffer=$%02X", dut.writeShifter, dut.writeBuffer);

        // =====================================================================
        // WRITE SYNC SEQUENCE + COMMAND PACKET
        // =====================================================================
        // The serializer was initialized with writeShifter=$FF.
        // It will serialize $FF (all 1s = 8 transitions) while we load
        // the first sync byte.
        //
        // The ROM's handshake loop:
        //   Lc82c: ASL iwm_q6l,x   — read handshake, check bit 7
        //          BCC Lc82c        — loop while writeBufferEmpty=0
        //          STA iwm_q6h,x   — write byte
        //
        // Sync bytes in send order: $3F, $CF, $F3, $FC, $FF, $C3

        $display("");
        $display("--- Sending sync sequence ---");

        // The $FF loaded on init gets serialized first.
        // Now feed the sync bytes through the handshake loop.
        rom_handshake_write(8'h3F);
        $display("  Wrote sync byte $3F");

        rom_handshake_write(8'hCF);
        $display("  Wrote sync byte $CF");

        rom_handshake_write(8'hF3);
        $display("  Wrote sync byte $F3");

        rom_handshake_write(8'hFC);
        $display("  Wrote sync byte $FC");

        rom_handshake_write(8'hFF);
        $display("  Wrote sync byte $FF");

        rom_handshake_write(8'hC3);
        $display("  Wrote sync byte $C3 (sync mark)");

        // Command bytes
        $display("");
        $display("--- Sending command packet ---");

        rom_handshake_write(8'h81);  // dest = device 1 | $80
        $display("  Wrote dest $81");

        rom_handshake_write(8'h80);  // src = host ($80)
        $display("  Wrote src $80");

        rom_handshake_write(8'h80);  // packet type (INIT = $85... using $80 for simplicity)
        $display("  Wrote type $80");

        rom_handshake_write(8'hA5);  // arbitrary test byte with mixed bits
        $display("  Wrote data $A5");

        // =====================================================================
        // WAIT FOR UNDERRUN AND SERIALIZER TO FINISH
        // =====================================================================
        $display("");
        $display("--- Waiting for underrun ---");

        // Don't write any more bytes — let the serializer drain
        // Wait until _wrreq goes HIGH (underrun)
        repeat(500) @(posedge fclk);  // ~70µs — enough for last byte + underrun

        // Stop FM decoder
        fm_active = 0;

        $display("  _wrreq = %b (expect 1 = deasserted after underrun)", _wrreq);
        $display("  wrdata = %b", wrdata);

        // =====================================================================
        // VERIFY DECODED BYTES
        // =====================================================================
        $display("");
        $display("===========================================");
        $display("FM Decoded Output (%0d bytes):", fm_byte_count);
        $display("===========================================");

        for (i = 0; i < fm_byte_count && i < 64; i = i + 1) begin
            $display("  Byte %0d: $%02X", i, fm_decoded_bytes[i]);
        end

        // Expected sequence:
        // Byte 0: $FF (initial writeShifter value, serialized on init)
        // Byte 1: $3F (first sync byte)
        // Byte 2: $CF
        // Byte 3: $F3
        // Byte 4: $FC
        // Byte 5: $FF
        // Byte 6: $C3 (sync mark)
        // Byte 7: $81 (dest)
        // Byte 8: $80 (src)
        // Byte 9: $80 (type)
        // Byte 10: $A5 (data)

        expected_bytes[0]  = 8'hFF;
        expected_bytes[1]  = 8'h3F;
        expected_bytes[2]  = 8'hCF;
        expected_bytes[3]  = 8'hF3;
        expected_bytes[4]  = 8'hFC;
        expected_bytes[5]  = 8'hFF;
        expected_bytes[6]  = 8'hC3;
        expected_bytes[7]  = 8'h81;
        expected_bytes[8]  = 8'h80;
        expected_bytes[9]  = 8'h80;
        expected_bytes[10] = 8'hA5;
        expected_count = 11;

        $display("");
        $display("--- Verification ---");
        if (fm_byte_count < expected_count) begin
            $display("  FAIL: Only decoded %0d bytes, expected %0d", fm_byte_count, expected_count);
            errors = errors + 1;
        end
        else begin
            for (i = 0; i < expected_count; i = i + 1) begin
                if (fm_decoded_bytes[i] === expected_bytes[i]) begin
                    $display("  Byte %0d: $%02X = $%02X PASS", i, fm_decoded_bytes[i], expected_bytes[i]);
                end
                else begin
                    $display("  Byte %0d: $%02X != $%02X FAIL", i, fm_decoded_bytes[i], expected_bytes[i]);
                    errors = errors + 1;
                end
            end
        end

        // =====================================================================
        // SUMMARY
        // =====================================================================
        $display("");
        $display("===========================================");
        if (errors == 0)
            $display("ALL TESTS PASSED");
        else
            $display("FAILED: %0d errors", errors);
        $display("===========================================");
        $display("");

        #1000;
        $finish;
    end

    // =========================================================================
    // Timeout Watchdog
    // =========================================================================
    initial begin
        #5000000;  // 5ms timeout
        $display("ERROR: Simulation timeout!");
        $display("  fm_byte_count = %0d", fm_byte_count);
        $display("  q7_stable = %b", dut.q7_stable);
        $display("  _wrreq = %b", _wrreq);
        $display("  _underrun = %b", dut._underrun);
        $finish;
    end

endmodule
