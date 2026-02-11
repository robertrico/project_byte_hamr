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
	output wire       q7_out
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
			2'b00: begin oneThreshold = 14; zeroThreshold = 42; writeBitCell = 28; end
			2'b01: begin oneThreshold = 16; zeroThreshold = 48; writeBitCell = 32; end
			2'b10: begin oneThreshold =  7; zeroThreshold = 21; writeBitCell = 14; end
			2'b11: begin oneThreshold =  8; zeroThreshold = 24; writeBitCell = 16; end
		endcase
	end

	// =========================================================================
	// State pseudo-register (spec p7)
	// Bits are individually addressed by A3-A1, data on A0.
	// Level detection: re-latch on every fclk while nDEVICE_SELECT is LOW.
	// This is idempotent (same bus cycle = same addr/value) and ensures
	// Q7/Q6 are available to the write register path within the same
	// nDEVICE_SELECT LOW window. ~3-4 fclk edges occur per bus cycle.
	// =========================================================================

	// SmartPort bus reset detect: phases = 0101 (ph0+ph2 ON, ph1+ph3 OFF)
	// The Liron ROM asserts this pattern for ~80ms before every INIT.
	// Use it to clean up motorOn/driveSelect/Q6/Q7 since the ROM's own
	// cleanup at $C949 never turns off motorOn (leaving _enbl2 stuck LOW).
	wire busReset = (phase == 4'b0101);

	always @(posedge fclk or negedge nRES) begin
		if (~nRES) begin
			phase            <= 4'b0000;
			motorOn          <= 1'b0;
			driveSelect      <= 1'b0;
			q6               <= 1'b0;
			q7               <= 1'b0;
		end
		else begin
			if (busReset) begin
				// SmartPort bus reset — scrub non-phase state
				motorOn     <= 1'b0;
				driveSelect <= 1'b0;
				q6          <= 1'b0;
				q7          <= 1'b0;
			end

			if (~nDEVICE_SELECT) begin
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
	//
	// SMARTPORT FIX: When SmartPort phases are active (ph1+ph3 = 1010),
	// suppress _enbl2 going LOW until phase[0] is also set (1011 = REQ).
	// This prevents FujiNet's ISR from entering the Disk II head-stepping
	// path during the ~12µs window between motorOn=1 and REQ assertion.
	// Without this, the ISR gets stuck processing a Disk II move_head()
	// call and misses the 1011 SmartPort command window entirely.
	// =========================================================================
	wire enbl1_motor = motorOn & ~driveSelect;
	wire enbl2_motor = motorOn &  driveSelect;
	wire enbl1_timer = timerActive & ~timerDriveSel;
	wire enbl2_timer = timerActive &  timerDriveSel;

	// Suppress _enbl2 when SmartPort enable is active but REQ not yet set
	wire sp_enable_no_req = phase[3] & phase[1] & ~phase[0];
	assign _enbl1 = ~(enbl1_motor | enbl1_timer);
	assign _enbl2 = sp_enable_no_req ? 1'b1 : ~(enbl2_motor | enbl2_timer);

	// Enable active flag for status register (spec p9 bit 5)
	wire enableActive = ~_enbl1 | ~_enbl2;

	// Write request: Q7 and no underrun and a drive is enabled (spec p6)
	assign _wrreq = ~(q7 & _underrun & (~_enbl1 | ~_enbl2));

	assign q7_out = q7;

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
			2'b10:   data_out = {writeBufferEmpty, _underrun, 6'b000000};
			2'b11:   data_out = 8'hFF;
		endcase
	end

	// =========================================================================
	// Write data sampling (spec p4)
	// The real IWM samples on the rising edge of (Q3 OR /DEV). In our FPGA,
	// data_in is stable for the entire nDEVICE_SELECT LOW window, so we only
	// need nDEVICE_SELECT. Dropping Q3 avoids a timing dependency where Q3
	// must be LOW simultaneously with nDEVICE_SELECT during a fclk posedge —
	// a narrow window that can systematically miss all buffer writes.
	// =========================================================================
	wire q3orDev = nDEVICE_SELECT;

	// =========================================================================
	// Serial I/O
	// =========================================================================
	//
	// LATCH MODE (modeReg[0]=1) BYTE FRAMING
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// The original code latched to buffer only when shifter[7]==1. This
	// is correct for GCR floppy data (latch=0) where every valid byte has
	// MSB=1. But SmartPort mode uses latch mode (modeReg[0]=1), and the
	// sync pattern contains bytes WITHOUT MSB=1:
	//   FF 3F CF F3 FC FF C3 ...
	//        ^^
	//   $3F = 00111111  -  bit 7 is 0!
	//
	// With the old logic, $3F never latches. Its bits merge with the
	// next byte, destroying byte alignment. The ROM scans for PBEGIN
	// ($C3) and never finds it.
	//
	// Fix: once the first byte syncs via shifter[7]==1 (from the leading
	// $FF sync), switch to an 8-bit counter that latches every 8 bits
	// unconditionally. This matches the real IWM's latch-mode behavior
	// described in the IWM specification.

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
			wrdata           <= 1'b0;
			latchSynced      <= 1'b0;
			q7_prev          <= 1'b0;
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
			// READ FROM DISK — shift register runs in all read modes (Q7=0)
			// Q6 must NOT gate this: the real IWM shift register runs
			// continuously, and SmartPort data arrives while Q6=1 (status
			// read mode) during the ACK/REQ handshake.
			// =============================================================
			if (q7 == 0) begin
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
				// Q7=1 (write mode) — reset latch sync so next read
				// re-syncs from the leading $FF.
				latchSynced <= 1'b0;
			end

			// =============================================================
			// WRITE TO DISK (spec p2)
			// Uses separate writeShifter/writeBitTimer/writeBitCounter
			// to avoid read-path state corrupting write output.
			// On Q7 0→1 transition, write state is initialized so the
			// first byte loads immediately with no garbage on wrdata.
			// =============================================================
			q7_prev <= q7;

			if (q7 == 1'b1) begin
				if (~q7_prev) begin
					// Q7 just went high — initialize write state.
					// Load writeShifter with $FF (all-ones) so wrdata
					// starts toggling immediately as sync preamble.
					// Counter=7 so the first byte-boundary loads from
					// buffer on the next writeBitCell rollover.
					//
					// Timer starts at 2 (not 0) so the first toggle at
					// writeBitTimer==1 occurs after a full bit cell
					// (writeBitCell - 2 + writeBitCell + 1 = 2*wBC - 1
					// fclk cycles ≈ 55 cycles ≈ 7.9µs for slow/7M).
					// Starting at 0 would produce a runt 280ns first
					// pulse that corrupts the FM decoder's sync phase.
					writeBitTimer   <= 6'd2;
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

					if (writeBitTimer == 1 && writeShifter[7] == 1)
						wrdata <= ~wrdata;
				end
			end
			else
				_underrun <= 1;

			// =============================================================
			// WRITE REGISTERS (spec p4/p7)
			// Q7=1, Q6=1, A0=1: Motor-On=1 writes data, Motor-On=0 writes mode
			// Spec says data sampled on rising edge of (Q3 OR /DEV), but
			// level-detect works because data/addr are stable while /DEV
			// is low. Writing the same value on multiple fclk cycles is
			// harmless and avoids edge-detection timing sensitivity.
			// =============================================================
			if (~q3orDev & q7 & q6 & addr[0]) begin
				if (motorOn) begin
					buffer <= data_in;
					writeBufferEmpty <= 0;
				end
				else begin
					modeReg <= data_in;
				end
			end
		end
	end

endmodule
