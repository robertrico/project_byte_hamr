`timescale 1ns / 1ps
module iwm (
	// Bus interface
	input wire [3:0]  addr,           // A3-A1 selects state register bit, A0 is new value
	input wire        nDEVICE_SELECT, // Device enable (active low)
	input wire        fclk,           // 7 or 8 MHz clock for serial I/O
	input wire        Q3,             // 2 MHz timing signal
	input wire        R_nW,           // 6502 read(1)/write(0) — gates buffer/mode writes
	input wire        nRES,           // System reset (active low)
	input wire [7:0]  data_in,        // Data from Apple II
	output reg [7:0]  data_out,       // Data to Apple II

	// Drive interface
	output reg        wrdata,         // Serial data output (transition = 1 bit)
	output reg [3:0]  phase,          // Programmable output / SmartPort command
	output wire       _wrreq,         // Write request (active low)
	output wire       _enbl1,         // Drive 1 enable (active low)
	output wire       _enbl2,         // Drive 2 enable (active low)
	input wire        sense,          // Write protect / ACK from drive
	input wire        rddata,         // Serial data input (falling edge = 1 bit)

	// Q7 output for ESP32 command decoding
	output wire       q7_out,

	// Write mode indicator (synchronized Q7, glitch-free)
	output wire       q7_stable_out
);

	// =========================================================================
	// Internal state
	// =========================================================================
	reg [7:0] shifter;            // Read shift register
	reg [7:0] writeShifter;       // Write shift register (separate from read)
	reg [7:0] buffer;
	reg [7:0] modeReg;            // Mode register (write-only, spec p8)
	reg       motorOn;
	reg       driveSelect;
	reg       q6, q7;
	reg       _underrun;
	reg       writeBufferEmpty;
	// /DEV-rising-edge sampler for register writes (spec: latch on rising
	// edge of (Q3 OR /DEV)).
	//
	// /DEV synced into fclk domain (2-FF) → rising-edge detect on dev_sync
	// triggers the buffer/mode load. R_nW + addr[0] are latched at the
	// /DEV-FALLING edge alongside q7/q6 (see negedge block below) — they
	// must NOT be captured in fclk domain because R_nW transitions at the
	// same PHI0 edge that drives /DEV rising, so an fclk capture late in
	// the cycle could race and pick up the next cycle's polarity. data_in
	// IS captured in fclk domain because the 6502 only puts data on the
	// bus late in PHI0 high; we sample continuously while dev_meta is low
	// (1-FF, 1 fclk shorter than dev_sync) to cap the window before the
	// next cycle starts driving.
	reg       dev_meta,   dev_sync,   dev_sync_prev;
	reg [7:0] data_held;
	// /DEV-domain latches for R_nW and addr[0]; fclk-synced versions below.
	reg       rnw_dev,    a0_dev;
	reg       rnw_meta,   rnw_sync;
	reg       a0_meta,    a0_sync;

	// Forward declarations for the /DEV-clocked sync registers (defined
	// further down). iverilog requires declaration before use; Yosys is
	// permissive. Keeping the actual flip-flop logic in its original
	// section below, this just brings the symbol names into scope early.
	reg [3:0] phase_meta,   phase_sync;
	reg       motorOn_meta, motorOn_sync;
	reg       driveSel_meta,driveSel_sync;
	reg       q6_meta,      q6_sync;
	reg       q7_meta,      q7_sync;

	// q7_stable is now just q7_sync — the /DEV-clocked state latch
	// eliminates glitches at the source, so no hysteresis filter needed.
	wire q7_stable = q7_sync;

	// Drain delay: after underrun fires, keep the handshake register's
	// bit 6 showing "no underrun" for DRAIN_DELAY fclk cycles. This
	// extends the Liron ROM's drain wait at $C92C, pushing the ACK
	// poll start later and giving FujiNet more time to decode the
	// command packet and assert ACK. Without this, the ROM's ~120µs
	// ACK poll window starts immediately after the last byte serializes
	// out (~840µs), but FujiNet's SPI capture alone takes ~928µs.
	//
	// 1430 cycles @ 7.16MHz ≈ 200µs extra drain time.
	localparam DRAIN_DELAY = 11'd1430;
	reg [10:0] drain_delay_ctr;
	reg        _underrun_prev;
	wire       _underrun_delayed = (drain_delay_ctr != 0) ? 1'b1 : _underrun;

	// Mode register bit aliases (spec p8)
	wire modeLatch   = modeReg[0]; // 1 = latch mode
	wire modeAsync   = modeReg[1]; // 1 = asynchronous handshake
	wire modeTimerOff= modeReg[2]; // 1 = 1-second timer disabled
	wire modeFast    = modeReg[3]; // 1 = fast mode (2 uS bit cells)
	wire mode8MHz    = modeReg[4]; // 1 = 8 MHz clock
	wire modeTest    = modeReg[5]; // 1 = test mode
	wire modeMZreset = modeReg[6]; // 1 = MZ-reset

	// =========================================================================
	// Mode-dependent timing parameters (all in FCLK cycles)
	// =========================================================================
	// Read bit cell windows (spec p4/p10):
	//   The spec gives Nclks relative to the effective clock (FCLK in fast,
	//   FCLK/2 in slow). Since our counter runs at FCLK, slow mode values
	//   are doubled.
	//
	//   Mode       | oneThreshold | zeroThreshold | writeBitCell
	//   slow, 7M   |   14 (7*2)   |   42 (21*2)   |     28
	//   slow, 8M   |   16 (8*2)   |   48 (24*2)   |     32
	//   fast, 7M   |    7         |   21           |     14
	//   fast, 8M   |    8         |   24           |     16
	reg [5:0] oneThreshold;
	reg [5:0] zeroThreshold;
	reg [5:0] writeBitCell;

	always @(*) begin
		case ({modeFast, mode8MHz})
			// Slow/7M: writeBitCell = 27 → 28 fclk = 3.91µs (matches real
			// Apple II IWM hardware spec). Liron firmware timing loops are
			// tuned for this. We bumped to 28 (4.05µs) for FujiNet RX and
			// it broke Liron's "drop Q7 after N µs" timing → truncated
			// last byte. Reverted to 27 + drain mode in serializer to
			// hide any remaining FujiNet-side timing edge cases.
			2'b00: begin oneThreshold = 14; zeroThreshold = 42; writeBitCell = 27; end
			2'b01: begin oneThreshold = 16; zeroThreshold = 48; writeBitCell = 31; end
			2'b10: begin oneThreshold =  7; zeroThreshold = 21; writeBitCell = 13; end
			2'b11: begin oneThreshold =  8; zeroThreshold = 24; writeBitCell = 15; end
		endcase
	end

	// =========================================================================
	// State latches (patent Fig. 2, block 39; spec p7)
	//
	// The real IWM latches state on the falling edge of /DEV — addr is
	// guaranteed stable by the 6502's 40ns setup time (tas). Using /DEV
	// as the clock eliminates metastability from fclk-sampling addr bus
	// glitches, which was causing Q7 spikes during the write loop.
	//
	// These registers are in the /DEV clock domain. They are synchronized
	// into the fclk domain below via 2-FF synchronizers.
	// =========================================================================

	always @(negedge nDEVICE_SELECT or negedge nRES) begin
		if (~nRES) begin
			phase       <= 4'b0000;
			motorOn     <= 1'b0;
			driveSelect <= 1'b0;
			q6          <= 1'b0;
			q7          <= 1'b0;
			rnw_dev     <= 1'b1;
			a0_dev      <= 1'b0;
		end
		else begin
			case (addr[3:1])
				3'h0: phase[0]    <= addr[0];
				3'h1: phase[1]    <= addr[0];
				3'h2: phase[2]    <= addr[0];
				3'h3: phase[3]    <= addr[0];
				3'h4: motorOn     <= addr[0];
				3'h5: driveSelect <= addr[0];
				3'h6: q6          <= addr[0];
				3'h7: q7          <= addr[0];
			endcase
			// Capture R_nW and addr[0] of THIS bus cycle. /DEV-falling
			// edge guarantees both are stable (6502 setup time, addr is
			// what triggered the slot decode). Synced to fclk below.
			rnw_dev <= R_nW;
			a0_dev  <= addr[0];
		end
	end

	// =========================================================================
	// 2-FF synchronizers: /DEV domain → fclk domain
	//
	// State bits change only on /DEV falling edges (~1µs apart minimum).
	// The 2-FF sync adds ~280ns latency (2 fclk @ 7MHz), well within
	// the 6502 bus cycle. Metastability resolves before the synced value
	// is used by any fclk-domain logic.
	// =========================================================================
	always @(posedge fclk or negedge nRES) begin
		if (~nRES) begin
			phase_meta    <= 4'b0000; phase_sync    <= 4'b0000;
			motorOn_meta  <= 1'b0;    motorOn_sync  <= 1'b0;
			driveSel_meta <= 1'b0;    driveSel_sync <= 1'b0;
			q6_meta       <= 1'b0;    q6_sync       <= 1'b0;
			q7_meta       <= 1'b0;    q7_sync       <= 1'b0;
			dev_meta      <= 1'b1;    dev_sync      <= 1'b1;
			dev_sync_prev <= 1'b1;
			data_held     <= 8'd0;
			rnw_meta      <= 1'b1;    rnw_sync      <= 1'b1;
			a0_meta       <= 1'b0;    a0_sync       <= 1'b0;
		end
		else begin
			phase_meta    <= phase;       phase_sync    <= phase_meta;
			motorOn_meta  <= motorOn;     motorOn_sync  <= motorOn_meta;
			driveSel_meta <= driveSelect; driveSel_sync <= driveSel_meta;
			q6_meta       <= q6;          q6_sync       <= q6_meta;
			q7_meta       <= q7;          q7_sync       <= q7_meta;
			rnw_meta      <= rnw_dev;     rnw_sync      <= rnw_meta;
			a0_meta       <= a0_dev;      a0_sync       <= a0_meta;

			// /DEV synchronizer + rising-edge tracking.
			dev_meta      <= nDEVICE_SELECT;
			dev_sync      <= dev_meta;
			dev_sync_prev <= dev_sync;

			// data_in needs LATE capture (6502 drives data only late in
			// PHI0 high). Sample continuously while dev_meta is low; the
			// last value before dev_meta sees high is what fire uses.
			if (~dev_meta) begin
				data_held <= data_in;
			end
		end
	end

	// =========================================================================
	// 1-second motor-off timer (spec p8/p14)
	// When timer is enabled (modeReg[2]==0), the selected /ENBLx stays low
	// for 2^23 + 100 FCLK periods (~1.2s at 7MHz) after Motor-On goes 1->0.
	// =========================================================================
	reg [23:0] timerCount;
	reg        timerActive;
	reg        timerDriveSel;  // which drive was selected when timer started
	reg        motorOn_prev;

	always @(posedge fclk or negedge nRES) begin
		if (~nRES) begin
			timerCount   <= 0;
			timerActive  <= 0;
			timerDriveSel<= 0;
			motorOn_prev <= 0;
		end
		else begin
			motorOn_prev <= motorOn_sync;
			if (motorOn_prev & ~motorOn_sync) begin
				// Motor-On falling edge
				if (~modeTimerOff) begin
					// Timer enabled — start countdown
					timerActive   <= 1;
					timerDriveSel <= driveSel_sync;
					timerCount    <= 0;
				end
			end
			if (timerActive) begin
				if (timerCount == 24'd8388708) begin // 2^23 + 100
					timerActive <= 0;
				end
				else
					timerCount <= timerCount + 1'b1;
			end
		end
	end

	// =========================================================================
	// Drive enable outputs (spec p7/p12)
	// /ENBLx is active (low) when Motor-On=1 OR timer is still running
	// for that drive.
	// =========================================================================
	wire enbl1_motor = motorOn_sync & ~driveSel_sync;
	wire enbl2_motor = motorOn_sync &  driveSel_sync;
	wire enbl1_timer = timerActive & ~timerDriveSel;
	wire enbl2_timer = timerActive &  timerDriveSel;

	// SmartPort drive gating: force /ENBL1 and /ENBL2 inactive (HIGH)
	// for the ENTIRE SmartPort session — both during the initial phase
	// setup AND between commands. Without this, the ESP32 sees active
	// drives in the inter-command gaps (when phases briefly go idle)
	// and starts Disk II emulation, burning CPU and blocking STATUS.
	//
	// In SmartPort mode (modeLatch=1), /ENBLx lines serve no purpose —
	// the ESP32 uses phases for SP command detection, not drive enables.
	// So whenever modeLatch is set and motorOn is high, we suppress
	// both drive enables unconditionally. This covers:
	//   - Pre-REQ window (ROM setting up phases, ~16µs)
	//   - Active SP command (phases cycling through 1011/1010/etc)
	//   - Inter-command gaps (phases briefly at 0000)
	//
	// modeLatch=0 (Disk II mode) is unaffected — drives pass through.
	wire sp_drive_gate = modeLatch & motorOn_sync;

	assign _enbl1 = sp_drive_gate ? 1'b1 : ~(enbl1_motor | enbl1_timer);
	assign _enbl2 = sp_drive_gate ? 1'b1 : ~(enbl2_motor | enbl2_timer);

	// Enable active flag for status register (spec p9 bit 5)
	wire enableActive = ~_enbl1 | ~_enbl2;

	// Write request: uses q7_sync (glitch-free from /DEV-clocked latch).
	assign _wrreq = ~(q7_stable & _underrun & (~_enbl1 | ~_enbl2));

	assign q7_out = q7_sync;
	assign q7_stable_out = q7_sync;

	// =========================================================================
	// Read registers (spec p7/p9)
	// Q7 Q6  Motor-On  Operation
	//  0  0     0      Read all ones
	//  0  0     1      Read data register
	//  0  1     x      Read status register
	//  1  0     x      Read write-handshake register
	//  1  1     0      (write mode register — no read defined)
	//  1  1     1      (write data register — no read defined)
	// =========================================================================
	// Patent Fig. 3: D7 comes from RR7 (controlled by x7 via hold read
	// data register logic, block 67), D0-D6 come from the buffer.
	// x7 is the data-ready flag — it overrides buffer[7] on the bus.
	// Forward-declared here for iverilog (actual reg lives further down).
	reg       x7;
	reg [3:0] clrX7Timer;
	always @(*) begin
		case ({q7_sync, q6_sync})
			2'b00:   data_out = {x7, buffer[6:0]};
			2'b01:   data_out = {sense, 1'b0, enableActive, modeReg[4:0]};
			2'b10:   data_out = {writeBufferEmpty, _underrun_delayed, 6'b000000};
			2'b11:   data_out = 8'hFF;
		endcase
	end

	// =========================================================================
	// Write data sampling (spec p4)
	// The real IWM samples on the rising edge of (Q3 OR /DEV). In our FPGA,
	// we use falling-edge detection of nDEVICE_SELECT to latch data writes
	// exactly once per bus access. Q3 is not used — data_in is stable for
	// the entire nDEVICE_SELECT LOW window.
	// =========================================================================

	// =========================================================================
	// Serial I/O
	// =========================================================================
	//
	// LATCH MODE (modeReg[0]=1) BYTE FRAMING
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// In GCR mode (latch=0), bytes latch when shifter[7]==1 (MSB-based
	// framing). SmartPort latch mode needs different framing because the
	// sync pattern contains bytes with MSB=0 (e.g. $3F = 00111111).
	//
	// Approach: sync on the first $FF (via shifter[7]==1), then switch to
	// an 8-bit counter that latches every 8 bits unconditionally. This
	// matches the real IWM's latch-mode behavior (IWM spec).

	reg [1:0] rddataSync;
	always @(posedge fclk) begin
		rddataSync <= {rddataSync[0], rddata};
	end

	reg [5:0] bitTimer;
	reg [2:0] bitCounter;
	reg [5:0] writeBitTimer;     // Separate write bit timer (avoids read↔write crosstalk)
	reg [2:0] writeBitCounter;   // Separate write bit counter
	reg       latchSynced;       // 1 = synced, latch every 8 bits
	reg       q7_prev;           // For detecting Q7 0→1 transition
	reg       q6_prev;           // For detecting Q6 transitions
	// Drain mode: keeps wrdata serializer running after Q7 falls so the
	// last byte (or part of it) currently in shifter/buffer finishes
	// transmitting on the wire. Liron drops Q7 on a fixed timing schedule
	// tuned for real Apple II IWM 3.91 µs cells; with our 4.05 µs cells
	// the schedule expires before the last byte clears, so without drain
	// the FujiNet sees a truncated packet. Set on Q7 falling, cleared on
	// natural _underrun (shifter empty + buffer empty at byte boundary).
	reg       in_drain;

	always @(posedge fclk or negedge nRES) begin
		if (~nRES) begin
			modeReg          <= 8'h07; // Default: latch=1, async=1, timer-off=1 (matches Liron init)
			_underrun        <= 1'b1;
			writeBufferEmpty <= 1'b1;
			bitCounter       <= 3'd0;
			bitTimer         <= 6'd0;
			writeBitCounter  <= 3'd7;
			writeBitTimer    <= 6'd0;
			buffer           <= 8'd0;
			shifter          <= 8'd0;
			writeShifter     <= 8'd0;
			x7               <= 1'b0;
			clrX7Timer       <= 4'd0;
			wrdata           <= 1'b1; // Idle HIGH matches FujiNet's static prev_level=true
			latchSynced      <= 1'b0;
			drain_delay_ctr  <= 11'd0;
			_underrun_prev   <= 1'b1;
			q7_prev          <= 1'b0;
			q6_prev          <= 1'b0;
			in_drain         <= 1'b0;
		end
		else begin

			// =============================================================
			// X7 — data-ready flag (patent Fig. 3, blocks 67/69)
			//
			// X7 is a separate flip-flop that controls D7 on the bus.
			// It is SET when the shift register loads a complete byte
			// into the read data register (buffer).
			// It is CLEARED 14 FCLK after a valid data read, defined
			// by the spec as: /DEV low AND D7 outputting a 1.
			// (Spec Rev 19 p3; Patent col 8, lines 60-67)
			//
			// The clear condition uses /DEV and x7 directly — NOT the
			// latched q7/q6 state bits. The real IWM's clear X7 logic
			// (block 69) has inputs: FCLK, /DEV, SYNCH, X7.
			// =============================================================
			if (clrX7Timer != 0) begin
				if (clrX7Timer == 4'd14) begin
					x7 <= 1'b0;
					clrX7Timer <= 0;
				end
				else
					clrX7Timer <= clrX7Timer + 1'b1;
			end
			else if (nDEVICE_SELECT == 0 && x7 == 1'b1) begin
				// Valid data read: /DEV low and D7=1 (x7 is set)
				// Start the 14-FCLK clear countdown
				clrX7Timer <= 4'd1;
			end

			// =============================================================
			// READ FROM DISK — shift register runs when Q7=0 (read mode).
			// Q6 must NOT gate this: the real IWM shift register runs
			// continuously, and SmartPort data arrives while Q6=1 (status
			// read mode) during the ACK/REQ handshake.
			//
			// FLUSH ON Q6 FALLING EDGE: When the ROM transitions from
			// status-read (Q6=1) to data-read (Q6=0), any noise-induced
			// shift register state from the handshake phase is flushed.
			// Without this, rddata noise during ACK/REQ polling could
			// set latchSynced prematurely (false $FF detection), causing
			// all subsequent bytes to be mis-framed. The ESP32 sends
			// 5+ $FF sync bytes, so there's plenty of time to re-sync.
			// =============================================================
			q6_prev <= q6_sync;
			if (~q6_sync & q6_prev & ~q7_stable) begin
				// Q6 falling edge while in read mode — flush read state
				latchSynced <= 1'b0;
				bitTimer    <= 6'd0;
				bitCounter  <= 3'd0;
				shifter     <= 8'd0;
			end

			if (q7_stable == 0) begin
				if (rddataSync[1] & ~rddataSync[0]) begin
					// Falling edge on rddata
					if (bitTimer >= oneThreshold) begin
						shifter <= {shifter[6:0], 1'b1};
						if (latchSynced && modeLatch) begin
							if (bitCounter == 7) begin
								buffer <= {shifter[6:0], 1'b1};
								x7 <= 1'b1;
								shifter <= 0;
								bitCounter <= 0;
							end
							else
								bitCounter <= bitCounter + 1'b1;
						end
						else if ({shifter[6:0], 1'b1} == 8'hFF && modeLatch) begin
							// First $FF sync byte — enter latch-synced mode
							buffer <= 8'hFF;
							x7 <= 1'b1;
							shifter <= 0;
							bitCounter <= 0;
							latchSynced <= 1'b1;
						end
						else if (shifter[6] == 1) begin
							// MSB=1 latch (standard GCR behavior)
							buffer <= {shifter[6:0], 1'b1};
							x7 <= 1'b1;
							shifter <= 0;
						end
						bitTimer <= 0;
					end
					// NOTE: If bitTimer < oneThreshold, this is a noise
					// glitch — do NOT reset bitTimer. Let it keep counting
					// so the next real edge sees the correct accumulated
					// gap. This mirrors the fm_decode noise filter logic.
				end
				else begin
					if (bitTimer >= zeroThreshold) begin
						// No edge for 1.5 bit cells — shift in a 0
						shifter <= {shifter[6:0], 1'b0};
						if (latchSynced && modeLatch) begin
							if (bitCounter == 7) begin
								buffer <= {shifter[6:0], 1'b0};
								x7 <= 1'b1;
								shifter <= 0;
								bitCounter <= 0;
							end
							else
								bitCounter <= bitCounter + 1'b1;
						end
						bitTimer <= oneThreshold;
					end
					else begin
						// No edge, timer still counting — check for
						// MSB=1 latch (only when not in latch-synced mode)
						if (!(latchSynced && modeLatch) && shifter[7] == 1) begin
							buffer <= shifter;
							x7 <= 1'b1;
							shifter <= 0;
						end
						bitTimer <= bitTimer + 1'b1;
					end
				end
			end
			else begin
				// q7_stable=1 (write mode) — reset latch sync so next
				// read re-syncs from the leading $FF.
				latchSynced <= 1'b0;
				bitTimer    <= 6'd0;
				bitCounter  <= 3'd0;
				shifter     <= 8'd0;
			end

			// =============================================================
			// WRITE TO DISK (spec p2)
			// Uses separate writeShifter/writeBitTimer/writeBitCounter
			// to avoid read-path state corrupting write output.
			// On q7_sync 0→1 transition, write state is initialized
			// so the first byte loads immediately with no garbage on
			// wrdata. The /DEV-clocked state latch ensures Q7 never
			// glitches.
			// =============================================================
			q7_prev <= q7_stable;

			if (q7_stable && ~q7_prev) begin
				// Q7 RISING — start new write session, init state.
				// Pre-seed buffer with $FF so the first byte boundary
				// loads a sync byte instead of underrunning (Liron's
				// LDA→STA setup takes longer than 14-fclk first-byte
				// deadline). FujiNet treats it as extra sync FF.
				writeBitTimer    <= 6'd14;
				writeBitCounter  <= 3'd7;
				writeShifter     <= 8'hFF;
				buffer           <= 8'hFF;
				writeBufferEmpty <= 1'b0;
				_underrun        <= 1'b1;
				in_drain         <= 1'b0;
			end
			else begin
				// Q7 FALLING (q7_prev=1, q7_stable=0) → enter drain.
				// Serializer keeps running until _underrun fires
				// (shifter + buffer both empty at byte boundary).
				if (q7_prev && ~q7_stable) begin
					in_drain <= 1'b1;
				end

				// Gate uses q7_prev so the Q7-fall fclk itself runs the
				// serializer (in_drain commits non-blocking, won't be 1
				// until next fclk; q7_prev is still 1 in this fclk).
				// Without this, wrdata idles HIGH for 1 fclk = 140ns
				// glitch on the wire = FujiNet sees false edge.
				if (q7_stable || q7_prev || in_drain) begin
					// SERIALIZER (active during write OR drain)
					if (writeBitTimer == writeBitCell) begin
						writeBitTimer <= 0;
						if (writeBitCounter == 7) begin
							writeBitCounter <= 0;
							if (writeBufferEmpty == 0) begin
								writeShifter <= buffer;
								writeBufferEmpty <= 1'b1;
							end
							else begin
								_underrun <= 0;
								in_drain  <= 1'b0; // drain done
							end
						end
						else begin
							writeBitCounter <= writeBitCounter + 1'b1;
							writeShifter <= {writeShifter[6:0], 1'b0};
						end
					end
					else
						writeBitTimer <= writeBitTimer + 1'b1;

					// Toggle wrdata at bit-cell midpoint when MSB=1.
					if (~_underrun)
						wrdata <= 1'b1; // Idle HIGH on underrun
					else if (writeBitTimer == 1 && writeShifter[7] == 1)
						wrdata <= ~wrdata;
				end
				else begin
					// q7_stable=0 AND not draining → idle.
					wrdata <= 1'b1;
				end
			end

			// =============================================================
			// DRAIN DELAY COUNTER
			// After the real underrun fires, keep _underrun_delayed HIGH
			// for DRAIN_DELAY fclk cycles. This extends the ROM's drain
			// wait ($C92C) giving FujiNet more time to decode and ACK.
			// =============================================================
			_underrun_prev <= _underrun;
			if (q7_stable == 1'b0) begin
				drain_delay_ctr <= 11'd0;  // Q7 off → reset
			end
			else if (_underrun_prev == 1'b1 && _underrun == 1'b0) begin
				// Falling edge of _underrun — start delay
				drain_delay_ctr <= DRAIN_DELAY;
			end
			else if (drain_delay_ctr != 11'd0) begin
				drain_delay_ctr <= drain_delay_ctr - 1'b1;
			end

			// =============================================================
			// WRITE REGISTERS (spec p4/p7)
			// Q7=1, Q6=1, A0=1: Motor-On=1 writes data, Motor-On=0 writes mode
			//
			// Fire on the synced rising edge of /DEV — i.e. the END of the
			// bus cycle, after the 6502 has held data_in valid through PHI0.
			// All gating signals (R_nW, addr[0], q7, q6) come from holding
			// registers captured WHILE /DEV was low, so they reflect the
			// just-completed bus cycle, not whatever is on the wire now.
			// =============================================================
			if (dev_sync & ~dev_sync_prev) begin
				if (~rnw_sync & q7_sync & q6_sync & a0_sync) begin
					if (motorOn_sync) begin
						// Guard: reject buffer writes after underrun.
						if (_underrun) begin
							buffer <= data_held;
							writeBufferEmpty <= 0;
						end
					end
					else begin
						modeReg <= data_held;
					end
				end
			end
		end
	end

endmodule
