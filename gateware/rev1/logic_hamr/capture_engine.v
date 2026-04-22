// =============================================================================
// Capture Engine for Logic Hamr
// =============================================================================
//
// Manages the complete capture lifecycle:
// - Arming and waiting for trigger
// - Pre-trigger buffering in BRAM circular buffer
// - Trigger detection (rising/falling edge on selected channel)
// - Raw sample storage to SDRAM via sdram_controller
//
// Uses the sdram_controller's simple write interface instead of raw SDRAM
// commands. The pause_refresh signal is used to prevent refresh interruptions
// during time-critical capture.
//
// =============================================================================

module capture_engine (
    input  wire        clk,             // 25 MHz system clock
    input  wire        rst_n,           // Active-low reset

    // Control interface
    input  wire        arm,             // Pulse to start capture
    input  wire        soft_reset,      // Force return to idle

    // Configuration
    input  wire [7:0]  probe_input,     // Synchronized probe data (8 channels)
    input  wire        sample_strobe,   // 1 MHz sample clock pulse
    input  wire [2:0]  trig_ch,         // Trigger channel (0-7)
    input  wire        trig_mode,       // 0 = rising edge, 1 = falling edge
    input  wire [1:0]  window_preset,   // Window preset (0-3)

    // Status outputs
    output reg         armed,           // Currently armed, waiting for trigger
    output reg         captured,        // Capture complete, raw data in SDRAM
    output wire [8:0]  total_samples,   // Number of samples captured (for regen_engine)
    output wire        pause_refresh,   // Assert to defer SDRAM refresh

    // Debug outputs
    output wire [2:0]  dbg_state,       // Current state machine state
    output wire [3:0]  dbg_bram_count,  // Pre-trigger sample count
    output wire        dbg_trigger_edge, // trigger_valid (full trigger condition)
    output wire        dbg_pretrig_ready, // bram_count >= cfg_pre_samples

    // SDRAM write interface (directly connected to sdram_controller)
    output reg         sdram_wr_req,
    output reg  [12:0] sdram_wr_addr,
    output reg  [7:0]  sdram_wr_data,
    input  wire        sdram_wr_ready
);

    // =========================================================================
    // State Machine
    // =========================================================================

    localparam [2:0]
        ST_IDLE        = 3'd0,   // Waiting for ARM command
        ST_ARMED       = 3'd1,   // Sampling to BRAM, awaiting trigger
        ST_BRAM_XFER   = 3'd2,   // Transferring pre-trigger from BRAM to SDRAM
        ST_POST_TRIG   = 3'd3,   // Capturing post-trigger samples
        ST_WRITE       = 3'd4,   // Writing sample to SDRAM
        ST_CAPTURED    = 3'd5;   // Capture complete

    reg [2:0] state;

    // =========================================================================
    // Window Preset Configuration
    // =========================================================================

    reg [8:0]  cfg_total_samples;
    reg [3:0]  cfg_pre_samples;
    reg [8:0]  cfg_post_samples;

    always @(*) begin
        case (window_preset)
            2'd0: begin  // Preset 0: 38us window (max zoom)
                cfg_total_samples = 9'd38;
                cfg_pre_samples   = 4'd2;
                cfg_post_samples  = 9'd36;
            end
            2'd1: begin  // Preset 1: 88us window (default)
                cfg_total_samples = 9'd88;
                cfg_pre_samples   = 4'd4;
                cfg_post_samples  = 9'd84;
            end
            2'd2: begin  // Preset 2: 133us window
                cfg_total_samples = 9'd133;
                cfg_pre_samples   = 4'd7;
                cfg_post_samples  = 9'd126;
            end
            2'd3: begin  // Preset 3: 266us window (min zoom)
                cfg_total_samples = 9'd266;
                cfg_pre_samples   = 4'd13;
                cfg_post_samples  = 9'd253;
            end
        endcase
    end

    assign total_samples = cfg_total_samples;

    // =========================================================================
    // Pre-Trigger BRAM Circular Buffer
    // =========================================================================
    // Small circular buffer to store samples before trigger event.
    // Max 13 bytes for preset 3 (266 samples with 13 pre-trigger).

    reg [7:0] pretrig_bram [0:15];   // 16-byte buffer (max 13 used)
    reg [3:0] bram_wr_ptr;           // Write pointer (circular)
    reg [3:0] bram_rd_ptr;           // Read pointer (for transfer)
    reg [3:0] bram_count;            // Number of valid samples in buffer

    // =========================================================================
    // Edge Detection for Trigger
    // =========================================================================

    reg [7:0] probe_prev;

    always @(posedge clk) begin
        if (sample_strobe)
            probe_prev <= probe_input;
    end

    // Edge detection on all channels
    wire [7:0] probe_rising  = probe_input & ~probe_prev;
    wire [7:0] probe_falling = ~probe_input & probe_prev;

    // Trigger detection on selected channel
    wire trig_rising  = probe_rising[trig_ch];
    wire trig_falling = probe_falling[trig_ch];
    wire trigger_edge = trig_mode ? trig_falling : trig_rising;

    // Trigger is valid only when armed and we have enough pre-trigger samples
    wire trigger_valid = (state == ST_ARMED) && trigger_edge && sample_strobe &&
                         (bram_count >= cfg_pre_samples);

    // Debug outputs
    assign dbg_state = state;
    assign dbg_bram_count = bram_count;
    assign dbg_trigger_edge = trigger_valid;  // Changed to show trigger_valid, not just edge
    assign dbg_pretrig_ready = (bram_count >= cfg_pre_samples);  // Pre-trigger buffer ready

    // =========================================================================
    // Capture State Registers
    // =========================================================================

    reg [8:0]  capture_wr_idx;      // SDRAM write index during capture
    reg [8:0]  post_trigger_cnt;    // Countdown of remaining post-trigger samples
    reg [3:0]  bram_xfer_idx;       // Index during BRAM to SDRAM transfer
    reg [7:0]  current_sample;      // Latched sample for SDRAM write
    reg [7:0]  trigger_sample;      // Preserved trigger sample
    reg        write_pending;       // Write request is pending

    // =========================================================================
    // Pause Refresh During Active Capture
    // =========================================================================

    assign pause_refresh = (state == ST_ARMED) || (state == ST_BRAM_XFER) ||
                           (state == ST_POST_TRIG) || (state == ST_WRITE);

    // =========================================================================
    // State Machine
    // =========================================================================

    integer k;
    initial begin
        state = ST_IDLE;
        armed = 1'b0;
        captured = 1'b0;
        bram_wr_ptr = 4'd0;
        bram_rd_ptr = 4'd0;
        bram_count = 4'd0;
        capture_wr_idx = 9'd0;
        post_trigger_cnt = 9'd0;
        bram_xfer_idx = 4'd0;
        current_sample = 8'd0;
        trigger_sample = 8'd0;
        write_pending = 1'b0;
        probe_prev = 8'd0;
        sdram_wr_req = 1'b0;
        sdram_wr_addr = 13'd0;
        sdram_wr_data = 8'd0;
        for (k = 0; k < 16; k = k + 1)
            pretrig_bram[k] = 8'd0;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            armed <= 1'b0;
            captured <= 1'b0;
            bram_wr_ptr <= 4'd0;
            bram_rd_ptr <= 4'd0;
            bram_count <= 4'd0;
            capture_wr_idx <= 9'd0;
            post_trigger_cnt <= 9'd0;
            bram_xfer_idx <= 4'd0;
            current_sample <= 8'd0;
            trigger_sample <= 8'd0;
            write_pending <= 1'b0;
            sdram_wr_req <= 1'b0;
        end else if (soft_reset) begin
            state <= ST_IDLE;
            armed <= 1'b0;
            captured <= 1'b0;
            write_pending <= 1'b0;
            sdram_wr_req <= 1'b0;
        end else begin
            // Default: clear write request after it's accepted
            if (sdram_wr_ready && sdram_wr_req) begin
                sdram_wr_req <= 1'b0;
                write_pending <= 1'b0;
            end

            case (state)
                ST_IDLE: begin
                    if (arm) begin
                        // ARM the capture engine
                        state <= ST_ARMED;
                        armed <= 1'b1;
                        captured <= 1'b0;
                        bram_wr_ptr <= 4'd0;
                        bram_count <= 4'd0;
                        capture_wr_idx <= 9'd0;
                    end
                end

                ST_ARMED: begin
                    // Sample to circular BRAM buffer on each sample_strobe
                    if (sample_strobe && !trigger_valid) begin
                        // Only write to buffer if trigger hasn't fired
                        pretrig_bram[bram_wr_ptr] <= probe_input;

                        // Update circular buffer pointers
                        if (bram_count < cfg_pre_samples) begin
                            bram_count <= bram_count + 1'b1;
                        end
                        // Advance write pointer with wrap
                        if (bram_wr_ptr == cfg_pre_samples - 1)
                            bram_wr_ptr <= 4'd0;
                        else
                            bram_wr_ptr <= bram_wr_ptr + 1'b1;
                    end

                    // Check for trigger (separate from buffer write)
                    if (trigger_valid) begin
                        // Trigger detected with enough pre-trigger samples
                        trigger_sample <= probe_input;
                        post_trigger_cnt <= cfg_post_samples - 1'b1; // -1 because trigger counts

                        // Read pointer: oldest sample in circular buffer
                        // bram_wr_ptr points to where we WOULD write next (oldest slot)
                        bram_rd_ptr <= bram_wr_ptr;
                        bram_xfer_idx <= 4'd0;

                        state <= ST_BRAM_XFER;
                    end
                end

                ST_BRAM_XFER: begin
                    // Transfer pre-trigger samples from BRAM to SDRAM
                    if (!write_pending && sdram_wr_ready) begin
                        if (bram_xfer_idx < cfg_pre_samples) begin
                            // Issue write for current BRAM sample
                            sdram_wr_req <= 1'b1;
                            sdram_wr_addr <= {4'b0, capture_wr_idx};
                            sdram_wr_data <= pretrig_bram[bram_rd_ptr];
                            write_pending <= 1'b1;

                            // Advance pointers with wrap at cfg_pre_samples
                            if (bram_rd_ptr == cfg_pre_samples - 1)
                                bram_rd_ptr <= 4'd0;
                            else
                                bram_rd_ptr <= bram_rd_ptr + 1'b1;
                            bram_xfer_idx <= bram_xfer_idx + 1'b1;
                            capture_wr_idx <= capture_wr_idx + 1'b1;
                        end else begin
                            // All pre-trigger samples transferred
                            // Now write the trigger sample and continue with post-trigger
                            sdram_wr_req <= 1'b1;
                            sdram_wr_addr <= {4'b0, capture_wr_idx};
                            sdram_wr_data <= trigger_sample;
                            write_pending <= 1'b1;
                            capture_wr_idx <= capture_wr_idx + 1'b1;
                            state <= ST_POST_TRIG;
                        end
                    end
                end

                ST_POST_TRIG: begin
                    // Capture post-trigger samples
                    if (post_trigger_cnt == 0) begin
                        // All samples captured, wait for final write to complete
                        if (!write_pending) begin
                            state <= ST_CAPTURED;
                            armed <= 1'b0;
                            captured <= 1'b1;
                        end
                    end else if (sample_strobe && !write_pending) begin
                        current_sample <= probe_input;
                        state <= ST_WRITE;
                    end
                end

                ST_WRITE: begin
                    // Write post-trigger sample to SDRAM
                    if (!write_pending && sdram_wr_ready) begin
                        sdram_wr_req <= 1'b1;
                        sdram_wr_addr <= {4'b0, capture_wr_idx};
                        sdram_wr_data <= current_sample;
                        write_pending <= 1'b1;
                        capture_wr_idx <= capture_wr_idx + 1'b1;
                        post_trigger_cnt <= post_trigger_cnt - 1'b1;
                        state <= ST_POST_TRIG;
                    end
                end

                ST_CAPTURED: begin
                    // Allow re-arming from captured state
                    if (arm) begin
                        state <= ST_ARMED;
                        armed <= 1'b1;
                        captured <= 1'b0;
                        bram_wr_ptr <= 4'd0;
                        bram_count <= 4'd0;
                        capture_wr_idx <= 9'd0;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
