// =============================================================================
// Regeneration Engine for Logic Hamr v2
// =============================================================================
//
// Core redesign: Processes all 8 channels in PARALLEL from a single SDRAM pass.
// This is 8x faster than v1's serial approach.
//
// Architecture:
// - 8 parallel decimate_pack instances (one per channel)
// - Read raw samples from capture buffer once
// - Feed all 8 bits to the 8 decimate_pack instances simultaneously
// - Store output bytes in register arrays
// - Write display buffer to SDRAM when complete
//
// Supports re-zoom: Can regenerate with different window preset without
// recapturing, since raw samples stay in capture buffer.
//
// =============================================================================

module regen_engine (
    input  wire        clk,             // 25 MHz system clock
    input  wire        rst_n,           // Active-low reset

    // Control interface
    input  wire        start,           // Pulse to start regeneration
    input  wire        soft_reset,      // Abort and return to idle

    // Configuration
    input  wire [1:0]  window_preset,   // Window preset (0-3)

    // Status outputs
    output reg         busy,            // Regeneration in progress
    output reg         done,            // Regeneration complete

    // SDRAM read interface (for capture buffer)
    output reg         sdram_rd_req,
    output reg  [12:0] sdram_rd_addr,
    input  wire        sdram_rd_ready,
    input  wire [7:0]  sdram_rd_data,
    input  wire        sdram_rd_valid,

    // SDRAM write interface (for display buffer)
    output reg         sdram_wr_req,
    output reg  [12:0] sdram_wr_addr,
    output reg  [7:0]  sdram_wr_data,
    input  wire        sdram_wr_ready
);

    // =========================================================================
    // Window Preset Configuration
    // =========================================================================

    reg [8:0]  cfg_total_samples;
    reg [7:0]  cfg_stretch;

    always @(*) begin
        case (window_preset)
            2'd0: begin cfg_total_samples = 9'd38;  cfg_stretch = 8'd7; end
            2'd1: begin cfg_total_samples = 9'd88;  cfg_stretch = 8'd3; end
            2'd2: begin cfg_total_samples = 9'd133; cfg_stretch = 8'd2; end
            2'd3: begin cfg_total_samples = 9'd266; cfg_stretch = 8'd1; end
        endcase
    end

    // =========================================================================
    // State Machine
    // =========================================================================

    localparam [2:0]
        ST_IDLE      = 3'd0,   // Waiting for start
        ST_READ_REQ  = 3'd1,   // Issue SDRAM read request
        ST_READ_WAIT = 3'd2,   // Wait for read data
        ST_PROCESS   = 3'd3,   // Feed sample to decimate_pack instances
        ST_FLUSH     = 3'd4,   // Flush final bytes
        ST_WRITE     = 3'd5,   // Write display buffer to SDRAM
        ST_DONE      = 3'd6;   // Complete

    reg [2:0] state;

    // =========================================================================
    // Decimate Pack Instances (8 parallel)
    // =========================================================================

    // Decimate pack control signals
    reg        dec_rst;
    reg        dec_sample_valid;
    reg        dec_flush;
    wire [7:0] dec_sample_in;       // One bit per channel from SDRAM
    wire [6:0] dec_byte_out [0:7];  // Output from each channel
    wire [7:0] dec_byte_valid;
    wire [7:0] dec_ready;

    // Current sample from SDRAM
    reg [7:0] current_sample;

    assign dec_sample_in = current_sample;

    // Instantiate 8 decimate_pack modules
    genvar ch;
    generate
        for (ch = 0; ch < 8; ch = ch + 1) begin : dec_gen
            decimate_pack u_decimate_pack (
                .clk(clk),
                .rst(dec_rst),
                .sample_in(dec_sample_in[ch]),
                .sample_valid(dec_sample_valid),
                .flush(dec_flush),
                .stretch_factor(cfg_stretch),
                .byte_out(dec_byte_out[ch]),
                .byte_valid(dec_byte_valid[ch]),
                .ready(dec_ready[ch])
            );
        end
    endgenerate

    // =========================================================================
    // Display Buffer Storage (8 channels x 38 bytes)
    // =========================================================================

    reg [6:0] display_buf [0:7][0:37];  // 8 channels x 38 bytes
    reg [5:0] byte_idx [0:7];            // Write index per channel

    // =========================================================================
    // Processing Counters
    // =========================================================================

    reg [8:0]  sample_idx;           // Current sample index (reading)
    reg [2:0]  write_channel;        // Current channel for SDRAM write
    reg [5:0]  write_byte_idx;       // Current byte within channel
    reg        read_pending;         // Read request is pending
    reg        process_done;         // All samples processed
    reg [3:0]  flush_countdown;      // Countdown for flush completion

    // =========================================================================
    // SDRAM Address Calculation Helper (for display buffer)
    // =========================================================================
    // Display buffer at 0x200, each channel has 38 bytes
    // Address = 0x200 + channel * 38 + byte_offset
    // Combinational calculation: channel * 38 = channel * 32 + channel * 4 + channel * 2
    wire [8:0] channel_offset = ({6'b0, write_channel} << 5) +
                                ({6'b0, write_channel} << 2) +
                                ({6'b0, write_channel} << 1);

    // =========================================================================
    // State Machine (includes byte capture to avoid multiple drivers)
    // =========================================================================

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            busy <= 1'b0;
            done <= 1'b0;
            dec_rst <= 1'b1;
            dec_sample_valid <= 1'b0;
            dec_flush <= 1'b0;
            sdram_rd_req <= 1'b0;
            sdram_rd_addr <= 13'd0;
            sdram_wr_req <= 1'b0;
            sdram_wr_addr <= 13'd0;
            sdram_wr_data <= 8'd0;
            sample_idx <= 9'd0;
            write_channel <= 3'd0;
            write_byte_idx <= 6'd0;
            current_sample <= 8'd0;
            read_pending <= 1'b0;
            process_done <= 1'b0;
            flush_countdown <= 4'd0;
            for (i = 0; i < 8; i = i + 1)
                byte_idx[i] <= 6'd0;
        end else if (soft_reset) begin
            state <= ST_IDLE;
            busy <= 1'b0;
            done <= 1'b0;
            dec_rst <= 1'b1;
            dec_sample_valid <= 1'b0;
            dec_flush <= 1'b0;
            sdram_rd_req <= 1'b0;
            sdram_wr_req <= 1'b0;
            read_pending <= 1'b0;
            process_done <= 1'b0;
        end else begin
            // Default: clear request pulses
            dec_sample_valid <= 1'b0;
            dec_flush <= 1'b0;

            // Clear SDRAM write requests when acknowledged
            if (sdram_wr_ready && sdram_wr_req) begin
                sdram_wr_req <= 1'b0;
            end

            // Capture output bytes from decimate_pack instances
            for (i = 0; i < 8; i = i + 1) begin
                if (dec_byte_valid[i] && byte_idx[i] < 38) begin
                    display_buf[i][byte_idx[i]] <= dec_byte_out[i];
                    byte_idx[i] <= byte_idx[i] + 1'b1;
                end
            end

            case (state)
                ST_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        state <= ST_READ_REQ;
                        busy <= 1'b1;
                        dec_rst <= 1'b0;
                        sample_idx <= 9'd0;
                        process_done <= 1'b0;
                        for (i = 0; i < 8; i = i + 1)
                            byte_idx[i] <= 6'd0;
                    end
                end

                ST_READ_REQ: begin
                    // Issue read for next sample from capture buffer
                    if (!sdram_rd_req && sdram_rd_ready) begin
                        sdram_rd_req <= 1'b1;
                        sdram_rd_addr <= {4'b0, sample_idx};
                        read_pending <= 1'b1;
                        state <= ST_READ_WAIT;
                    end
                end

                ST_READ_WAIT: begin
                    // Clear read request (controller has accepted it)
                    sdram_rd_req <= 1'b0;
                    // Wait for read data
                    if (sdram_rd_valid) begin
                        current_sample <= sdram_rd_data;
                        read_pending <= 1'b0;
                        state <= ST_PROCESS;
                    end
                end

                ST_PROCESS: begin
                    // Feed sample to decimate_pack instances
                    // Wait for all instances to be ready
                    if (&dec_ready) begin
                        dec_sample_valid <= 1'b1;
                        sample_idx <= sample_idx + 1'b1;

                        if (sample_idx >= cfg_total_samples - 1) begin
                            // All samples processed
                            process_done <= 1'b1;
                            state <= ST_FLUSH;
                            flush_countdown <= 4'd15;  // Allow time for bytes to complete
                        end else begin
                            state <= ST_READ_REQ;
                        end
                    end
                end

                ST_FLUSH: begin
                    // Flush decimate_pack instances to emit partial bytes
                    if (flush_countdown > 0) begin
                        if (flush_countdown == 4'd15)
                            dec_flush <= 1'b1;
                        flush_countdown <= flush_countdown - 1'b1;
                    end else begin
                        // Start writing display buffer to SDRAM
                        state <= ST_WRITE;
                        write_channel <= 3'd0;
                        write_byte_idx <= 6'd0;
                    end
                end

                ST_WRITE: begin
                    // Write display buffer to SDRAM
                    if (!sdram_wr_req && sdram_wr_ready) begin
                        // Calculate SDRAM address for display buffer
                        // Display buffer starts at 0x200 (512)
                        // Address = 0x200 + channel * 38 + byte_idx
                        sdram_wr_req <= 1'b1;
                        sdram_wr_addr <= 13'h200 + channel_offset + {7'b0, write_byte_idx};
                        sdram_wr_data <= {1'b0, display_buf[write_channel][write_byte_idx]};

                        if (write_byte_idx == 6'd37) begin
                            // Done with this channel
                            write_byte_idx <= 6'd0;
                            if (write_channel == 3'd7) begin
                                // All channels done
                                state <= ST_DONE;
                            end else begin
                                write_channel <= write_channel + 1'b1;
                            end
                        end else begin
                            write_byte_idx <= write_byte_idx + 1'b1;
                        end
                    end
                end

                ST_DONE: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    dec_rst <= 1'b1;
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

    // =========================================================================
    // Initial Values (for simulation when rst_n is tied high)
    // =========================================================================

    integer init_i;
    initial begin
        state = ST_IDLE;
        busy = 1'b0;
        done = 1'b0;
        dec_rst = 1'b1;
        dec_sample_valid = 1'b0;
        dec_flush = 1'b0;
        sdram_rd_req = 1'b0;
        sdram_rd_addr = 13'd0;
        sdram_wr_req = 1'b0;
        sdram_wr_addr = 13'd0;
        sdram_wr_data = 8'd0;
        sample_idx = 9'd0;
        write_channel = 3'd0;
        write_byte_idx = 6'd0;
        current_sample = 8'd0;
        read_pending = 1'b0;
        process_done = 1'b0;
        flush_countdown = 4'd0;
        for (init_i = 0; init_i < 8; init_i = init_i + 1)
            byte_idx[init_i] = 6'd0;
    end

endmodule
