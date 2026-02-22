// =============================================================================
// IWM — Integrated Woz Machine
// =============================================================================
// Complete IWM emulation as described in IWM Spec Rev 19 (September 24, 1982)
// and US Patent 4,742,448.
//
// This module implements:
//   1. 8-bit state register (PHASE0-3, Motor-On, Drive-Sel, L6, L7)
//   2. Register select decode (from L7, L6, Motor-On)
//   3. Mode register (write-only, 8 bits)
//   4. Status register (read-only, mirrors mode + sense + motor)
//   5. Read data extractor (RDDATA falling edge → bit cell windowing)
//   6. Read shift register (serial-to-parallel, 8 bits LSB→MSB)
//   7. Read data register (parallel latch with async clear)
//   8. Write shift register (parallel-to-serial, MSB-first)
//   9. Write toggle (WRDATA output, edge per '1' bit)
//  10. Write handshake / underrun detection
//
// IWM Spec Rev 19, page 7 — State Register:
//   "This is an 8-bit write-only pseudo-register. The bits in this register
//    are individually addressed by A3,A2,A1. The data on A0 is latched into
//    the addressed state bit by /DEV low."
//
//   Address  Name       Function
//   -------  ---------  --------------------------------------------------
//   0        PHASE0     A 1 in this bit will drive PHASE0 to a high state
//   1        PHASE1     "
//   2        PHASE2     "
//   3        PHASE3     "
//   4        LMotor-On  A 1 on LMotor-On sets the enable selected below low
//   5        Drive-Sel  A 1 on this bit selects /ENBL2; a 0 selects /ENBL1
//   6        L6         (see register decode below)
//   7        L7         (see register decode below)
//
// IWM Spec Rev 19, page 7 — Register Decode:
//   L7 L6 Motor-On  Read Operation             Write Operation    State Name
//   -- -- --------  -------------------------  -----------------  ----------
//   0  0  0         read all ones              -                  -
//   0  0  1         read data register         -                  Read
//   0  1  x         read status register       -                  Write-Protect Sense
//   1  0  x         read write-handshake reg   -                  Write
//   1  1  0         write mode register        -                  Mode Set
//   1  1  1         write data register        -                  Write Load
//
// IWM Patent col 4 lines 25-35 — State Latch Table:
//   A3 A2 A1 | Latch
//   0  0  0  | P0 (PHASE0)
//   0  0  1  | P1 (PHASE1)
//   0  1  0  | P2 (PHASE2)
//   0  1  1  | P3 (PHASE3)
//   1  0  0  | L4 (Motor-On)
//   1  0  1  | L5 (Drive-Sel)
//   1  1  0  | L6
//   1  1  1  | L7
// =============================================================================

module iwm (
    // Apple II bus interface (active during /DEV assertion)
    input  wire [3:0] addr,            // A0-A3 from Apple II bus
    input  wire       nDEVICE_SELECT,  // Active-low device select (/DEV)
    input  wire [7:0] data_in,         // D0-D7 from Apple II bus (active on writes)

    // Clocks and reset
    input  wire       fclk,            // 7 MHz reference clock (FCLK)
    input  wire       Q3,              // ~2 MHz bus timing (qualifies writes)
    input  wire       nRES,            // Active-low reset

    // Drive interface outputs
    output wire [3:0] phase,           // PHASE0-PHASE3 stepper/command lines
    output wire       _enbl1,          // Drive 1 enable (active low)
    output wire       _enbl2,          // Drive 2 enable (active low)
    output wire       _wrreq,          // Write request (directly active low)

    // Drive interface inputs
    input  wire       rddata,          // Serial read data from drive
    input  wire       sense,           // Write-protect / ACK from drive

    // Serial data output to drive
    output wire       wrdata,          // Serial write data to drive

    // Register interface to top module
    output wire [7:0] data_out,        // Data bus output (active during reads)

    // State bits exposed for top module use
    output wire       q6_out,          // L6 state bit (active-high)
    output wire       q7_out           // L7 state bit (active-high)
);

    // =========================================================================
    // State Register — 8 individually-addressed latches
    // =========================================================================
    // Per IWM Spec: "The falling edge of /DEV latches information on A0-A3."
    // A1-A3 select which of 8 state bits to update. A0 is the data.
    // All 8 state bits cleared on /RESET.

    reg [7:0] state = 8'h00;

    // Detect falling edge of /DEV using FCLK
    reg ndev_prev = 1'b1;
    always @(posedge fclk) begin
        ndev_prev <= nDEVICE_SELECT;
    end
    wire dev_falling = ndev_prev & ~nDEVICE_SELECT;

    // Latch state bits on falling edge of /DEV
    always @(posedge fclk) begin
        if (!nRES) begin
            state <= 8'h00;
        end else if (dev_falling) begin
            state[addr[3:1]] <= addr[0];
        end
    end

    // Named state bit aliases
    wire motor_on = state[4];
    wire drive_sel = state[5];
    wire l6 = state[6];
    wire l7 = state[7];

    // =========================================================================
    // Drive Interface Outputs
    // =========================================================================
    // Per IWM Spec page 6:
    //   PHASE0-3: "Programmable output lines."
    //   /ENBL1, /ENBL2: "Programmable buffered output lines.
    //     No more than one enable may be low at any time."
    //   Drive-Sel: "A 1 selects /ENBL2; a 0 selects /ENBL1"
    //   Motor-On + Drive-Sel together determine which enable is active.

    assign phase = state[3:0];

    // Enable logic: only active when motor_on is set
    // Per spec: "If an enable is low then Motor-On is true"
    assign _enbl1 = ~(motor_on & ~drive_sel);
    assign _enbl2 = ~(motor_on &  drive_sel);

    // =========================================================================
    // Write Clock — "rising edge of (Q3 OR /DEV)"
    // =========================================================================
    // Per IWM Spec page 5:
    //   "Data written to the IWM is sampled by the zero to one transition of
    //    the logical OR of Q3 and /DEV."
    // This is the clock that qualifies register writes (mode reg, data reg).

    wire write_clk = Q3 | nDEVICE_SELECT;
    reg  write_clk_prev = 1'b1;
    always @(posedge fclk) begin
        write_clk_prev <= write_clk;
    end
    wire write_clk_rising = ~write_clk_prev & write_clk;

    // =========================================================================
    // Register Select Decode
    // =========================================================================
    // Per IWM Spec page 7 and Patent col 6 lines 15-65:

    wire sel_read_all_ones  = ~l7 & ~l6 & ~motor_on;
    wire sel_read_data      = ~l7 & ~l6 &  motor_on;
    wire sel_read_status    = ~l7 &  l6;
    wire sel_read_handshake =  l7 & ~l6;
    wire sel_write_mode     =  l7 &  l6 & ~motor_on;
    wire sel_write_data     =  l7 &  l6 &  motor_on;

    // Post-update register select: predicts sel_read_data AFTER the current
    // dev_falling state update takes effect. Needed because the x7_latch
    // sampling happens on the same clock edge as the state bit change.
    // When the ROM does "LDA Q6L,X", L6 transitions 1→0 on dev_falling,
    // but sel_read_data still sees the OLD L6=1, missing the latch.
    wire new_l6       = (addr[3:1] == 3'd6) ? addr[0] : l6;
    wire new_l7       = (addr[3:1] == 3'd7) ? addr[0] : l7;
    wire new_motor_on = (addr[3:1] == 3'd4) ? addr[0] : motor_on;
    wire will_sel_read_data = ~new_l7 & ~new_l6 & new_motor_on;

    // =========================================================================
    // Mode Register (write-only, 8 bits)
    // =========================================================================
    // Per IWM Spec page 8:
    //   "All eight mode bits are reset to 0 by /RESET low."
    //   Written when L6=1, L7=1, Motor-On=0 on rising edge of write clock.
    //
    //   Bit  Function
    //   ---  --------------------------------------------------
    //   0    1 = latch mode (should be set in asynchronous mode)
    //   1    0 = synchronous handshake; 1 = asynchronous
    //   2    0 = 1-second on-board timer enable; 1 = timer disable
    //   3    0 = slow mode; 1 = fast mode (2 uS bit cell timing)
    //   4    0 = 7MHz; 1 = 8MHz (clock descriptor)
    //   5    1 = test mode; 0 = normal operation
    //   6    1 = MZ-reset
    //   7    reserved

    reg [7:0] mode_reg = 8'h00;

    always @(posedge fclk) begin
        if (!nRES) begin
            mode_reg <= 8'h00;
        end else if (write_clk_rising && sel_write_mode) begin
            mode_reg <= data_in;
        end
    end

    // Named mode bits
    wire mode_latch = mode_reg[0];
    wire mode_async = mode_reg[1];

    // =========================================================================
    // Status Register (read-only)
    // =========================================================================
    // Per IWM Spec page 9:
    //   Bit  Function
    //   ---  --------------------------------------------------
    //   0-4  same as mode register
    //   5    1 = either /ENBL1 or /ENBL2 is currently active (low)
    //   6    1 = MZ (reserved, should always read 0)
    //   7    1 = SENSE input high; 0 = SENSE input low

    wire [7:0] status_reg;
    assign status_reg[4:0] = mode_reg[4:0];
    assign status_reg[5]   = motor_on;          // either enable is active when motor_on
    assign status_reg[6]   = 1'b0;              // MZ bit — always 0 per spec
    assign status_reg[7]   = sense;             // SENSE input directly

    // =========================================================================
    // Block 1: Internal Clock Enable — CLK rate (FCLK/2 for slow mode)
    // =========================================================================
    // IWM Spec Rev 19, page 2:
    //   "In slow mode CLK is equivalent to the clock input on FCLK divided
    //    by two. Therefore, in 7M and slow mode the bit cell time will be
    //    28 FCLK clock input periods."
    //
    // Liron sets mode = $07 → slow mode, 7MHz clock descriptor.
    // CLK period = 2 × FCLK ≈ 280 ns.
    // Bit cell = 14 CLK periods = 28 FCLK periods ≈ 3.91 µs.

    reg clk_div = 1'b0;

    always @(posedge fclk) begin
        if (!nRES)
            clk_div <= 1'b0;
        else
            clk_div <= ~clk_div;
    end

    wire clk_en = clk_div;

    // =========================================================================
    // Block 2: RDDATA Synchronizer & Falling Edge Detector
    // =========================================================================
    // IWM Spec page 5:
    //   "RDDATA: The serial data input. The falling transition of each pulse
    //    is synchronized by the IWM."
    //
    // Inside this module: rddata is idle HIGH (top module inverts GPIO9).
    // A '1' bit = brief LOW pulse (~1 µs ≈ 7 FCLK cycles).
    // A '0' bit = stays HIGH for the entire bit cell (~4 µs).
    //
    // Two-stage synchronizer prevents metastability from the asynchronous
    // RDDATA input. Third stage holds previous value for edge detection.

    reg rddata_sync1 = 1'b1;
    reg rddata_sync2 = 1'b1;
    reg rddata_sync3 = 1'b1;

    always @(posedge fclk) begin
        if (!nRES) begin
            rddata_sync1 <= 1'b1;
            rddata_sync2 <= 1'b1;
            rddata_sync3 <= 1'b1;
        end else begin
            rddata_sync1 <= rddata;
            rddata_sync2 <= rddata_sync1;
            rddata_sync3 <= rddata_sync2;
        end
    end

    // Falling edge: sync3 was HIGH (previous), sync2 is LOW (current)
    wire rddata_falling = rddata_sync3 & ~rddata_sync2;

    // =========================================================================
    // Block 3: Read Data Extractor — Bit Cell Counter & LFT1/LFT0
    // =========================================================================
    // IWM Patent col 7, lines 25-65:
    //   "Each time a negative transition of RDDATA occurs, it resets an
    //    interval counter. When 8/7 is reset, the interval is 14 CLKs."
    //
    //   "A '1' is a negative transition at the expected time."
    //   "A '0' is no transition at the expected time. The expected time is
    //    widened by approximately one-half an interval before and after."
    //
    // IWM Spec Rev 19, page 4 — Bit cell windows (slow, 7M, CLK = FCLK/2):
    //   Nclks 0-6:   dead zone (runt rejection, no bit generated)
    //   Nclks 7-20:  '1' window (falling edge here → lft1)
    //   Nclks 21:    timeout (no edge → lft0, counter resets to 7)
    //
    // Window widening (patent col 7, lines 45-65):
    //   After timeout at 21, counter resets to 7 so next timeout occurs
    //   at 21 again (14 CLK later from timeout). An edge in 7-20 after
    //   timeout produces lft1. This matches the spec table:
    //     7-20 → 1, 21-34 → 01, 35-48 → 001 (counted from last edge)
    //
    // FujiNet ESP32: TX SPI at 1 MHz, 2 data bits per SPI byte (bit 6
    // and bit 2), producing 4 µs per data bit — matching the IWM spec.
    // At FCLK/2 rate: 4 µs / 280 ns ≈ 14.3 CLK between consecutive
    // edges, safely within the valid window (7-20 CLK).

    wire read_active = ~l7 & ~l6;

    reg [5:0] bc_counter = 6'd0;
    reg       lft1 = 1'b0;
    reg       lft0 = 1'b0;

    always @(posedge fclk) begin
        if (!nRES || !read_active) begin
            bc_counter <= 6'd0;
            lft1 <= 1'b0;
            lft0 <= 1'b0;
        end else begin
            // Default: clear pulses each cycle (one-shot)
            lft1 <= 1'b0;
            lft0 <= 1'b0;

            if (rddata_falling) begin
                // Falling edge detected on RDDATA
                if (bc_counter >= 6'd7) begin
                    // Edge within valid window (counter >= 7) → '1' bit
                    lft1 <= 1'b1;
                end
                // Any edge resets counter (re-sync), even runts in 0-6
                bc_counter <= 6'd0;
            end else if (clk_en) begin
                // No edge this cycle — advance counter at CLK rate
                if (bc_counter == 6'd21) begin
                    // Timeout — no edge arrived → '0' bit
                    lft0 <= 1'b1;
                    // Reset to 7 so next timeout is 14 CLK later
                    bc_counter <= 6'd7;
                end else begin
                    bc_counter <= bc_counter + 6'd1;
                end
            end
        end
    end

    // =========================================================================
    // Block 4: Read Shift Register
    // =========================================================================
    // IWM Spec Rev 19, page 3:
    //   "In the read state the data is shifted into the LSB of the shift
    //    register, and the shift register shifts data from LSB to MSB."
    //   "When a full data nibble is shifted into the internal shift register,
    //    the data will be latched by the read data register and the shift
    //    register will be cleared to all zeros."
    //
    // IWM Patent col 8, lines 1-6:
    //   Shift register data logic: line 55 = LFT1 | (LFT0 & ~SR7)
    //   Shift clock fires on each LFT1/LFT0 pulse, inhibited when SR7=1.
    //
    // Shift direction: left shift, new bit enters at bit 0.
    // Serial input: lft1 → '1'; lft0 → '0'.
    // Byte complete: when sr_next[7] == 1 (a 1 reaches the MSB).

    reg [7:0] read_sr = 8'h00;

    wire shift_pulse = lft1 | lft0;
    wire bit_in      = lft1;

    // What the shift register will contain after this shift
    wire [7:0] sr_next = {read_sr[6:0], bit_in};

    // Byte is complete when this shift puts a 1 into the MSB.
    wire sr_byte_complete = shift_pulse & sr_next[7] & ~read_sr[7];

    always @(posedge fclk) begin
        if (!nRES || !read_active) begin
            read_sr <= 8'h00;
        end else if (shift_pulse && !read_sr[7]) begin
            if (sr_next[7]) begin
                // Byte complete — clear shift register immediately
                read_sr <= 8'h00;
            end else begin
                read_sr <= sr_next;
            end
        end
    end

    // =========================================================================
    // Block 5: Read Data Register — Parallel Latch with Async Clear
    // =========================================================================
    // IWM Spec Rev 19, page 3:
    //   "In asynchronous mode the data register will latch the shift register
    //    when a one is shifted into the MSB and will be cleared 14 FCLK
    //    periods (about 2 µS) after a valid data read takes place."
    //
    // IWM Spec page 8 (latch mode):
    //   "In latch mode the msb of the read data is latched internally during
    //    /DEV low (this internally latched msb is then used for the
    //    determination of a valid data read)."
    //
    // A "valid data read" = /DEV low AND sel_read_data AND x7_latch == 1.
    //
    // NOTE: Non-latch mode (mode_reg[0]=0) is not fully implemented.
    // In non-latch mode, D7 should be sampled live and auto-clear should
    // still trigger on a valid read. Our x7_latch is only set in latch mode,
    // so auto-clear won't fire when mode_latch=0. This is fine for Liron
    // (always sets mode=$07, latch=1) but would need work for other uses.
    //
    // Similarly, synchronous mode (mode_reg[1]=0) is not implemented.
    // Sync mode loads the data register on every shift, not just on SR7=1.
    // Liron always uses async mode, so this path is not needed.

    reg [7:0] read_data_reg = 8'h00;

    reg       x7_latch = 1'b0;
    reg [3:0] clear_counter = 4'd0;
    reg       clear_pending = 1'b0;

    wire reading_data_reg = sel_read_data & ~nDEVICE_SELECT;

    always @(posedge fclk) begin
        if (!nRES) begin
            read_data_reg <= 8'h00;
            x7_latch      <= 1'b0;
            clear_counter  <= 4'd0;
            clear_pending  <= 1'b0;
        end else begin

            // --- Latch MSB on falling edge of /DEV (latch mode) ---
            // Two race conditions resolved here:
            // 1. Use will_sel_read_data (post-update prediction) because on
            //    dev_falling the state bits haven't committed yet. Without this,
            //    "LDA Q6L,X" from L6=1 would miss the latch entirely.
            // 2. If sr_byte_complete fires on the same edge as dev_falling,
            //    read_data_reg still holds the old (pre-load) value. Use
            //    sr_next[7] instead so the new byte's MSB is captured.
            if (dev_falling && will_sel_read_data && mode_latch) begin
                x7_latch <= sr_byte_complete ? sr_next[7] : read_data_reg[7];
            end

            // --- Detect valid read and start clear countdown ---
            if (reading_data_reg && x7_latch && !clear_pending) begin
                clear_pending <= 1'b1;
                clear_counter <= 4'd0;
            end

            // --- Clear countdown (14 FCLK periods) ---
            if (clear_pending) begin
                if (clear_counter == 4'd13) begin
                    // 14 FCLK elapsed — clear the register
                    read_data_reg <= 8'h00;
                    x7_latch      <= 1'b0;
                    clear_pending  <= 1'b0;
                    clear_counter  <= 4'd0;
                end else begin
                    clear_counter <= clear_counter + 4'd1;
                end
            end

            // --- Parallel load from shift register (byte complete) ---
            // HIGHEST PRIORITY: new byte wins over pending clear.
            if (sr_byte_complete) begin
                read_data_reg <= sr_next;
                clear_pending  <= 1'b0;
                clear_counter  <= 4'd0;
            end

        end
    end

    // =========================================================================
    // Block 6-7: Write Data Register, Shift Register, Toggle, & Handshake
    // =========================================================================
    // IWM Spec Rev 19, page 2:
    //   "In asynchronous mode CLK is used to generate the bit cell timings.
    //    ...the write shift register is loaded every 8 bit cell times
    //    starting seven CLK periods after the write state begins."
    //
    // IWM Spec page 3:
    //   "An underrun occurs when data has not been written to the buffer
    //    register between the time the write-handshake bit indicates an
    //    empty buffer and the time the buffer is transferred to the write
    //    shift-register."
    //
    // IWM Spec page 6:
    //   "WRDATA: The serial data output. A transition occurs on this output
    //    for each one bit."
    //
    // IWM Patent col 9, lines 55-65 (async write):
    //   "After shift register 83 has been parallel loaded with the data from
    //    write data register 81, the most significant bit in shift register 83
    //    will be shifted onto line 95 and after eight more CLK periods, toggle
    //    85 will cause WRDATA to toggle from '1' to '0'."
    //   When 8/7=0: "toggles occur 6 CLKs after shifts, and shifts occur
    //    8 CLKs after toggles."
    //
    // Double buffered: write_data_reg (CPU buffer) → write_sr (shift reg)
    // MSB shifted out first. Toggle flip-flop produces edges on WRDATA.

    // --- Write mode detection ---
    wire write_mode = l7 & motor_on;

    reg l7_prev = 1'b0;
    always @(posedge fclk) begin
        if (!nRES)
            l7_prev <= 1'b0;
        else
            l7_prev <= l7;
    end

    wire write_mode_entry = l7 & ~l7_prev & motor_on;

    // --- Write registers ---
    reg [7:0] write_data_reg    = 8'h00;
    reg       write_buffer_ready = 1'b1;
    reg       write_state       = 1'b0;
    reg       wr_underrun       = 1'b0;

    reg [7:0] write_sr          = 8'h00;
    reg       wrdata_toggle     = 1'b1;
    reg [3:0] wr_cell_cnt       = 4'd0;
    reg [2:0] wr_bit_cnt        = 3'd0;
    reg       wr_running        = 1'b0;
    reg [2:0] wr_delay_cnt      = 3'd0;
    reg       wr_in_delay       = 1'b0;

    always @(posedge fclk) begin
        if (!nRES) begin
            // --- Global reset ---
            write_data_reg     <= 8'h00;
            write_buffer_ready <= 1'b1;
            write_state        <= 1'b0;
            wr_underrun        <= 1'b0;
            write_sr           <= 8'h00;
            wrdata_toggle      <= 1'b1;
            wr_cell_cnt        <= 4'd0;
            wr_bit_cnt         <= 3'd0;
            wr_running         <= 1'b0;
            wr_delay_cnt       <= 3'd0;
            wr_in_delay        <= 1'b0;
        end else if (!write_mode) begin
            // --- Not in write mode: reset write machinery ---
            // IWM Spec page 3: "Clearing state bit L7 will reset the
            // /underrun flag."
            write_state        <= 1'b0;
            wr_underrun        <= 1'b0;
            write_sr           <= 8'h00;
            wrdata_toggle      <= 1'b1;
            wr_cell_cnt        <= 4'd0;
            wr_bit_cnt         <= 3'd0;
            wr_running         <= 1'b0;
            wr_in_delay        <= 1'b0;
            wr_delay_cnt       <= 3'd0;
            write_buffer_ready <= 1'b1;
        end else begin
            // === Write mode active (L7=1, motor_on=1) ===

            // --- CPU writes to data register ---
            if (write_clk_rising && sel_write_data) begin
                write_data_reg     <= data_in;
                write_buffer_ready <= 1'b0;   // Buffer now full
                write_state        <= 1'b1;
            end

            // --- Write mode entry: start initial delay ---
            if (write_mode_entry) begin
                wrdata_toggle  <= 1'b1;       // WRDATA starts HIGH
                wr_in_delay    <= 1'b1;
                wr_delay_cnt   <= 3'd0;
                wr_running     <= 1'b0;
                write_state    <= 1'b1;
            end

            // --- Initial 7-CLK delay before first shift register load ---
            if (wr_in_delay && clk_en) begin
                if (wr_delay_cnt == 3'd6) begin
                    // 7 CLK periods elapsed — load shift register
                    write_sr           <= write_data_reg;
                    write_buffer_ready <= 1'b1;
                    wr_in_delay        <= 1'b0;
                    wr_running         <= 1'b1;
                    wr_cell_cnt        <= 4'd0;
                    wr_bit_cnt         <= 3'd0;
                end else begin
                    wr_delay_cnt <= wr_delay_cnt + 3'd1;
                end
            end

            // --- Main shift register operation ---
            if (wr_running && clk_en && !wr_underrun) begin
                wr_cell_cnt <= wr_cell_cnt + 4'd1;

                // Toggle point: 7th CLK period of each bit cell (count 6)
                if (wr_cell_cnt == 4'd6) begin
                    if (write_sr[7]) begin
                        wrdata_toggle <= ~wrdata_toggle;
                    end
                end

                // End of bit cell: 14th CLK period (count 13)
                if (wr_cell_cnt == 4'd13) begin
                    wr_cell_cnt <= 4'd0;

                    if (wr_bit_cnt == 3'd7) begin
                        // All 8 bits shifted out — need next byte
                        wr_bit_cnt <= 3'd0;

                        if (!write_buffer_ready) begin
                            // Buffer has new data — parallel load
                            write_sr           <= write_data_reg;
                            write_buffer_ready <= 1'b1;
                        end else begin
                            // Buffer empty — UNDERRUN
                            wr_underrun <= 1'b1;
                            write_state <= 1'b0;
                            wr_running  <= 1'b0;
                        end
                    end else begin
                        // Shift next bit (MSB first → left shift)
                        write_sr   <= {write_sr[6:0], 1'b0};
                        wr_bit_cnt <= wr_bit_cnt + 3'd1;
                    end
                end
            end
        end
    end

    // --- Handshake register (read-only) ---
    // IWM Spec page 9:
    //   bit 0-5: reserved (read as 1s)
    //   bit 6: write state (cleared to 0 if underrun)
    //   bit 7: write data buffer ready for data
    wire [7:0] handshake_reg;
    assign handshake_reg[5:0] = 6'b111111;
    assign handshake_reg[6]   = write_state;
    assign handshake_reg[7]   = write_buffer_ready;

    // --- WRDATA output ---
    // Active only in write mode; idle LOW otherwise.
    assign wrdata = write_mode ? wrdata_toggle : 1'b0;

    // --- /WRREQ output ---
    // IWM Spec page 7: "The combination of L7 and Motor-On and /underrun
    //   enables /WRREQ low."
    assign _wrreq = ~(l7 & motor_on & write_state);

    // =========================================================================
    // Data Bus Output Mux
    // =========================================================================
    // Per the register decode table, select what goes on the bus during reads.

    reg [7:0] data_out_mux;
    always @(*) begin
        case (1'b1)
            sel_read_all_ones:  data_out_mux = 8'hFF;
            sel_read_data:      data_out_mux = read_data_reg;
            sel_read_status:    data_out_mux = status_reg;
            sel_read_handshake: data_out_mux = handshake_reg;
            default:            data_out_mux = 8'hFF;
        endcase
    end

    assign data_out = data_out_mux;

    // =========================================================================
    // Expose state for top module
    // =========================================================================

    assign q6_out = l6;
    assign q7_out = l7;

endmodule
