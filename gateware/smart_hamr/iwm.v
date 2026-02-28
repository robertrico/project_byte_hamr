`timescale 1ns / 1ps
module iwm (
	// Bus interface
	input wire [3:0]  addr,           // A3-A1 selects state register bit, A0 is new value
	input wire        nDEVICE_SELECT, // Device enable (active low)
	input wire        fclk,           // 7 or 8 MHz clock for serial I/O
	input wire        Q3,             // 2 MHz timing signal
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

	// Write mode indicator (filtered Q7 with hysteresis)
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

	// Write-register one-shot: the real IWM latches data once per bus
	// access on the rising edge of (Q3 OR /DEV). Our level-detect fires
	// every fclk while nDEVICE_SELECT is LOW (~7 cycles), which causes
	// the serializer to replay bytes when it consumes the buffer mid-access.
	// This edge-detect on the combined write condition fires exactly once
	// per bus access, matching real IWM behavior.
	//
	// Two-stage pipeline: writeCondPrev1 delays one fclk so data_in
	// has settled before the latch fires. writeCondPrev2 blocks all
	// subsequent fclk cycles in the same bus access.
	reg       writeCondPrev1;
	reg       writeCondPrev2;
	wire      writeCondNow = ~nDEVICE_SELECT & q7 & q6 & addr[0];

	// Q7 stability filter with hysteresis.
	//
	// The write serializer needs a HIGH falling threshold to ride through
	// bus glitches during Q6 toggling (~6-8µs bursts). The ROM NEVER
	// intentionally sets Q7=0 during writing — the write handshake
	// toggles Q6 only ($C08C/$C08D). Q7 transitions are permanent:
	// 0→1 at STA $C08F, 1→0 at LDA $C08E. But hardware glitches on
	// the address bus during Q6 toggles can briefly make addr[3:1]
	// look like 111, causing Q7 to glitch via edge-detect. These
	// glitches can last up to one full Q6 cycle (~6-8µs).
	//
	// Rising: q7_stable HIGH after Q7 continuously HIGH for 8 fclk (~1.1µs)
	// Falling: q7_stable LOW after Q7 continuously LOW for 100 fclk (~14µs)
	localparam Q7_RISE_THRESH = 4'd8;    // ~1.1µs
	localparam Q7_FALL_THRESH = 7'd100;  // ~14µs
	reg [3:0]  q7_rise_ctr;
	reg [6:0]  q7_fall_ctr;
	reg        q7_stable;

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
			// writeBitCell is period-1 because the timer counts 0..N inclusive
			// (N+1 fclk cycles). IWM spec: 28 FCLK for slow/7M = 3.91µs bit cell.
			2'b00: begin oneThreshold = 14; zeroThreshold = 42; writeBitCell = 27; end
			2'b01: begin oneThreshold = 16; zeroThreshold = 48; writeBitCell = 31; end
			2'b10: begin oneThreshold =  7; zeroThreshold = 21; writeBitCell = 13; end
			2'b11: begin oneThreshold =  8; zeroThreshold = 24; writeBitCell = 15; end
		endcase
	end

	// =========================================================================
	// State pseudo-register (spec p7)
	// Bits are individually addressed by A3-A1, data on A0.
	//
	// Level-detect for phases/motor/drive (safe — different addr[3:1]
	// values can't cross-contaminate). Edge-detect (falling nDEVICE_SELECT)
	// for Q6/Q7 because $C08C/$C08D and $C08E/$C08F differ only in addr[0].
	//
	// The downstream Q7 STABILITY FILTER (q7_stable) handles hardware bus
	// glitches that make Q7 appear to transition during writing.
	// =========================================================================

	reg nDEVICE_SELECT_prev;

	always @(posedge fclk or negedge nRES) begin
		if (~nRES) begin
			phase                <= 4'b0000;
			motorOn              <= 1'b0;
			driveSelect          <= 1'b0;
			q6                   <= 1'b0;
			q7                   <= 1'b0;
			nDEVICE_SELECT_prev  <= 1'b1;
		end
		else begin
			nDEVICE_SELECT_prev <= nDEVICE_SELECT;

			if (~nDEVICE_SELECT) begin
				// Level-detect: phases, motor, drive.
				// Samples every fclk while /DEV is LOW, which acts as
				// a self-correcting latch — even if the first fclk
				// catches an address bus glitch, subsequent cycles
				// overwrite with the correct stable value.
				case (addr[3:1])
					3'h0: phase[0]    <= addr[0];
					3'h1: phase[1]    <= addr[0];
					3'h2: phase[2]    <= addr[0];
					3'h3: phase[3]    <= addr[0];
					3'h4: motorOn     <= addr[0];
					3'h5: driveSelect <= addr[0];
					default: ; // Q6, Q7 use edge-detect below
				endcase
			end

			// Edge-detect: Q6, Q7 only.
			// These share addr[3:1]=11x and differ only in addr[0],
			// so level-detect could cross-contaminate $C08C↔$C08D
			// or $C08E↔$C08F. Edge-detect fires once per bus access
			// when the address is stable.
			if (nDEVICE_SELECT_prev & ~nDEVICE_SELECT) begin
				case (addr[3:1])
					3'h6: q6 <= addr[0];
					3'h7: q7 <= addr[0];
					default: ;
				endcase
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
			motorOn_prev <= motorOn;
			if (motorOn_prev & ~motorOn) begin
				// Motor-On falling edge
				if (~modeTimerOff) begin
					// Timer enabled — start countdown
					timerActive   <= 1;
					timerDriveSel <= driveSelect;
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
	wire enbl1_motor = motorOn & ~driveSelect;
	wire enbl2_motor = motorOn &  driveSelect;
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
	wire sp_drive_gate = modeLatch & motorOn;

	assign _enbl1 = sp_drive_gate ? 1'b1 : ~(enbl1_motor | enbl1_timer);
	assign _enbl2 = sp_drive_gate ? 1'b1 : ~(enbl2_motor | enbl2_timer);

	// Enable active flag for status register (spec p9 bit 5)
	wire enableActive = ~_enbl1 | ~_enbl2;

	// Write request: q7_stable (not raw q7) so glitches don't pull
	// _wrreq LOW and trigger FujiNet's wrdata sampling during receive.
	assign _wrreq = ~(q7_stable & _underrun & (~_enbl1 | ~_enbl2));

	assign q7_out = q7;
	assign q7_stable_out = q7_stable;

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
	always @(*) begin
		case ({q7, q6})
			2'b00:   data_out = buffer;
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
	reg [3:0] clearBufferTimer;
	reg       latchSynced;       // 1 = synced, latch every 8 bits
	reg       q7_prev;           // For detecting Q7 0→1 transition

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
			clearBufferTimer <= 4'd0;
			wrdata           <= 1'b1; // Idle HIGH matches FujiNet's static prev_level=true
			latchSynced      <= 1'b0;
			drain_delay_ctr  <= 11'd0;
			_underrun_prev   <= 1'b1;
			q7_prev          <= 1'b0;
			q7_rise_ctr      <= 4'd0;
			q7_fall_ctr      <= 7'd0;
			q7_stable        <= 1'b0;
			writeCondPrev1   <= 1'b0;
			writeCondPrev2   <= 1'b0;
		end
		else begin

			// Clear buffer MSB when CPU reads data register (Q7=0, Q6=0)
			// Spec p3: after a valid data read, MSB is cleared (14 FCLK
			// periods in async mode).
			if (q7 == 0 && q6 == 0) begin
				if (clearBufferTimer == 0) begin
					if (nDEVICE_SELECT == 0 && addr[0] == 0 && buffer[7] == 1'b1)
						clearBufferTimer <= 1;
				end
				else begin
					if (clearBufferTimer == 4'hE) begin
						buffer[7] <= 0;
						clearBufferTimer <= 0;
					end
					else
						clearBufferTimer <= clearBufferTimer + 1'b1;
				end
			end

			// =============================================================
			// READ FROM DISK — shift register runs when q7_stable=0.
			// Uses q7_stable (FALL=100, ~14µs) so bus glitches during
			// writing don't let the shift register run and corrupt the
			// buffer via the MSB=1 latch path.
			// Q6 must NOT gate this: the real IWM shift register runs
			// continuously, and SmartPort data arrives while Q6=1 (status
			// read mode) during the ACK/REQ handshake.
			// =============================================================
			if (q7_stable == 0) begin
				if (rddataSync[1] & ~rddataSync[0]) begin
					// Falling edge on rddata
					if (bitTimer >= oneThreshold) begin
						shifter <= {shifter[6:0], 1'b1};
						if (latchSynced && modeLatch) begin
							if (bitCounter == 7) begin
								buffer <= {shifter[6:0], 1'b1};
								shifter <= 0;
								bitCounter <= 0;
							end
							else
								bitCounter <= bitCounter + 1'b1;
						end
						else if ({shifter[6:0], 1'b1} == 8'hFF && modeLatch) begin
							// First $FF sync byte — enter latch-synced mode
							buffer <= 8'hFF;
							shifter <= 0;
							bitCounter <= 0;
							latchSynced <= 1'b1;
						end
						else if (shifter[6] == 1) begin
							// MSB=1 latch (standard GCR behavior)
							buffer <= {shifter[6:0], 1'b1};
							shifter <= 0;
						end
					end
					bitTimer <= 0;
				end
				else begin
					if (bitTimer >= zeroThreshold) begin
						// No edge for 1.5 bit cells — shift in a 0
						shifter <= {shifter[6:0], 1'b0};
						if (latchSynced && modeLatch) begin
							if (bitCounter == 7) begin
								buffer <= {shifter[6:0], 1'b0};
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
			// Q7 STABILITY FILTER
			// Rising: q7_stable HIGH after Q7 continuously HIGH for 8 fclk.
			// Falling: q7_stable LOW after Q7 continuously LOW for 100 fclk.
			// =============================================================
			if (q7) begin
				q7_fall_ctr <= 7'd0;
				if (!q7_stable) begin
					if (q7_rise_ctr < Q7_RISE_THRESH)
						q7_rise_ctr <= q7_rise_ctr + 1'b1;
					else
						q7_stable <= 1'b1;
				end
			end
			else begin
				q7_rise_ctr <= 4'd0;
				if (q7_stable) begin
					if (q7_fall_ctr < Q7_FALL_THRESH)
						q7_fall_ctr <= q7_fall_ctr + 1'b1;
					else
						q7_stable <= 1'b0;
				end
			end

			// =============================================================
			// WRITE TO DISK (spec p2)
			// Uses separate writeShifter/writeBitTimer/writeBitCounter
			// to avoid read-path state corrupting write output.
			// On q7_stable 0→1 transition, write state is initialized
			// so the first byte loads immediately with no garbage on
			// wrdata. The stability filter ensures Q7 glitches never
			// reach the serializer.
			// =============================================================
			q7_prev <= q7_stable;

			if (q7_stable) begin
				if (~q7_prev) begin
					// q7_stable just went high — initialize write state.
					// Load writeShifter with $FF (all-ones) so wrdata
					// starts toggling immediately as sync preamble.
					// Counter=7 so the first byte-boundary loads from
					// buffer on the next writeBitCell rollover.
					//
					// IWM spec: "the write shift register is loaded
					// every 8 bit cell times starting seven CLK periods
					// after the write state begins." In slow mode,
					// CLK = fclk/2, so 7 CLK = 14 fclk. Timer starts
					// at 14 so it rolls over after 14 cycles (27-14+1),
					// triggering the first byte load at the spec-correct
					// time. No wrdata toggle in this initial partial cell
					// (toggle is at timer==1, already passed).
					writeBitTimer   <= 6'd14;
					writeBitCounter <= 3'd7;
					writeShifter    <= 8'hFF;
				end
				else begin
					// Normal write serializer operation
					if (writeBitTimer == writeBitCell) begin
						writeBitTimer <= 0;
						if (writeBitCounter == 7) begin
							writeBitCounter <= 0;
							if (writeBufferEmpty == 0) begin
								writeShifter <= buffer;
								writeBufferEmpty <= 1'b1;
							end
							else
								_underrun <= 0;
						end
						else begin
							writeBitCounter <= writeBitCounter + 1'b1;
							writeShifter <= {writeShifter[6:0], 1'b0};
						end
					end
					else
						writeBitTimer <= writeBitTimer + 1'b1;

					// Toggle wrdata at bit-cell midpoint when MSB=1.
					// Gate on _underrun: once underrun fires, wrdata
					// must stop immediately. Without this, the serializer
					// keeps toggling for up to Q7_FALL_THRESH cycles
					// (~14µs) while _wrreq is already HIGH — producing
					// trailing wrdata transitions that corrupt FujiNet's
					// FM decoder byte alignment.
					if (~_underrun)
						wrdata <= 1'b1; // Idle HIGH immediately on underrun
					else if (writeBitTimer == 1 && writeShifter[7] == 1)
						wrdata <= ~wrdata;
				end
			end
			else begin
				_underrun <= 1;
				wrdata    <= 1'b1; // Idle HIGH matches FujiNet's static prev_level=true.
				                   // FujiNet's iwm_decode_byte initializes prev_level=true.
				                   // If wrdata is LOW at SPI capture start, prev_level≠current
				                   // fires a spurious edge at sample 0, resyncing idx to
				                   // half_samples and shifting the entire decode window by 2µs.
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
			// One-shot write pulse with 1-fclk settle delay:
			// - fclk N:   writeCondNow goes TRUE (q6 just updated)
			// - fclk N+1: writeCondPrev1=1, writeCondPrev2=0 → FIRE
			// - fclk N+2: writeCondPrev2=1 → blocked for rest of access
			//
			// The 1-fclk delay ensures data_in has settled after the
			// address decode propagates through the FPGA fabric.
			// writeCondPrev2 prevents re-fire for the remaining ~5 fclk
			// of the bus access, eliminating the double-load race.
			// =============================================================
			writeCondPrev1 <= writeCondNow;
			writeCondPrev2 <= writeCondPrev1;
			if (writeCondPrev1 & ~writeCondPrev2) begin
				if (motorOn) begin
					// Guard: reject buffer writes after underrun.
					if (_underrun) begin
						buffer <= data_in;
						writeBufferEmpty <= 0;
					end
				end
				else begin
					modeReg <= data_in;
				end
			end
		end
	end

endmodule
