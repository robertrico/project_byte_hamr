// =============================================================================
// Byte Hamr Logic Hamr - Capture Engine
// =============================================================================
//
// Real-time 8-channel logic analyzer with trigger detection and pre-trigger
// buffering. Captures signals at 1 MHz sample rate and generates Apple II
// HIRES-compatible display data through the decimate_pack module.
//
// Capture Flow:
//   1. Configure trigger channel, edge mode, and window preset
//   2. ARM the capture engine (write to $C0C9)
//   3. Pre-trigger samples continuously fill BRAM circular buffer
//   4. On trigger, BRAM contents + post-trigger samples written to SDRAM
//   5. Regenerate command processes captured data through decimate_pack
//   6. Display buffer ready for Apple II to read
//
// Register Map (offsets from $C0C0):
//   $00 CHANNEL  R/W  Channel select (0-7) for display buffer read
//   $01 ADDR     R/W  Byte index within channel (0-37)
//   $02 DATA     R    Read result from SDRAM display buffer
//   $03 CMD      W    $02=Read byte, $10=Regenerate from captured data
//   $04 STATUS   R    [0]=busy [1]=ready [2]=armed [3]=captured
//   $05 STRETCH  R    Stretch factor (from window preset, read-only)
//   $06 TRIG_CH  R/W  Trigger channel (0-7)
//   $07 TRIG_MODE R/W Trigger mode (0=rising edge, 1=falling edge)
//   $08 WINDOW   R/W  Window preset (0=38us, 1=88us, 2=133us, 3=266us)
//   $09 ARM      W    Write any value to ARM capture engine
//   $0A DEBUG_EN R/W  Debug pattern enable (0=real probes, 1=test pattern)
//
// Window Presets:
//   Preset 0: 38 samples,  2 pre + 36 post,  stretch 7 (max zoom)
//   Preset 1: 88 samples,  4 pre + 84 post,  stretch 3 (default)
//   Preset 2: 133 samples, 7 pre + 126 post, stretch 2
//   Preset 3: 266 samples, 13 pre + 253 post, stretch 1 (min zoom)
//
// SDRAM Memory Map:
//   0x0000-0x010F  Capture buffer (max 266 samples × 8 bits)
//   0x1000-0x112F  Display buffer (8 channels × 38 bytes)
//
// GPIO Pins:
//   GPIO1:     Heartbeat output (250kHz)
//   GPIO2:     Ready output (init complete + display loaded)
//   GPIO3:     Armed output (waiting for trigger)
//   GPIO4:     Captured output (capture complete)
//   GPIO5-12:  Probe inputs [0-7] for signal capture
//
// =============================================================================

module logic_hamr_v1_top (
    // System clock
    input  wire        CLK_25MHz,

    // ----- SDRAM Interface -----
    output wire        SDRAM_CLK,
    output wire        SDRAM_CKE,
    output wire        SDRAM_nCS,
    output wire        SDRAM_nRAS,
    output wire        SDRAM_nCAS,
    output wire        SDRAM_nWE,
    output wire        SDRAM_DQM0,
    output wire        SDRAM_DQM1,
    output wire        SDRAM_BA0,
    output wire        SDRAM_BA1,
    output wire        SDRAM_A0,
    output wire        SDRAM_A1,
    output wire        SDRAM_A2,
    output wire        SDRAM_A3,
    output wire        SDRAM_A4,
    output wire        SDRAM_A5,
    output wire        SDRAM_A6,
    output wire        SDRAM_A7,
    output wire        SDRAM_A8,
    output wire        SDRAM_A9,
    output wire        SDRAM_A10,
    output wire        SDRAM_A11,
    output wire        SDRAM_A12,
    inout  wire        SDRAM_D0,
    inout  wire        SDRAM_D1,
    inout  wire        SDRAM_D2,
    inout  wire        SDRAM_D3,
    inout  wire        SDRAM_D4,
    inout  wire        SDRAM_D5,
    inout  wire        SDRAM_D6,
    inout  wire        SDRAM_D7,
    inout  wire        SDRAM_D8,
    inout  wire        SDRAM_D9,
    inout  wire        SDRAM_D10,
    inout  wire        SDRAM_D11,
    inout  wire        SDRAM_D12,
    inout  wire        SDRAM_D13,
    inout  wire        SDRAM_D14,
    inout  wire        SDRAM_D15,

    // ----- Apple II Bus -----
    input  wire        A0,
    input  wire        A1,
    input  wire        A2,
    input  wire        A3,
    input  wire        A4,
    input  wire        A5,
    input  wire        A6,
    input  wire        A7,
    input  wire        A8,
    input  wire        A9,
    input  wire        A10,
    input  wire        A11,
    input  wire        A12,
    input  wire        A13,
    input  wire        A14,
    input  wire        A15,

    inout  wire        D0,
    inout  wire        D1,
    inout  wire        D2,
    inout  wire        D3,
    inout  wire        D4,
    inout  wire        D5,
    inout  wire        D6,
    inout  wire        D7,

    input  wire        PHI0,
    input  wire        PHI1,
    input  wire        sig_7M,
    input  wire        Q3,
    input  wire        uSync,

    input  wire        R_nW,
    output wire        nRES,
    input  wire        nDEVICE_SELECT,
    input  wire        nI_O_SELECT,
    input  wire        nI_O_STROBE,
    input  wire        DMA_OUT,
    input  wire        INT_OUT,

    output wire        nIRQ,
    output wire        nNMI,
    input  wire        RDY,
    output wire        nINH,
    output wire        nDMA,
    output wire        DMA_IN,
    output wire        INT_IN,

    // ----- GPIO Breakout -----
    // GPIO1-4: Debug outputs
    output wire        GPIO1,   // Heartbeat
    output wire        GPIO2,   // Ready
    output wire        GPIO3,   // Armed
    output wire        GPIO4,   // Captured
    // GPIO5-12: Probe inputs for signal capture
    input  wire        GPIO5,   // PROBE[0]
    input  wire        GPIO6,   // PROBE[1]
    input  wire        GPIO7,   // PROBE[2]
    input  wire        GPIO8,   // PROBE[3]
    input  wire        GPIO9,   // PROBE[4]
    input  wire        GPIO10,  // PROBE[5]
    input  wire        GPIO11,  // PROBE[6]
    input  wire        GPIO12   // PROBE[7]
);

    // =========================================================================
    // Internal Signal Bundles
    // =========================================================================

    wire [15:0] apple_addr = {A15, A14, A13, A12, A11, A10, A9, A8,
                              A7, A6, A5, A4, A3, A2, A1, A0};
    wire [7:0]  apple_data_in = {D7, D6, D5, D4, D3, D2, D1, D0};

    wire [15:0] sdram_dq_in = {SDRAM_D15, SDRAM_D14, SDRAM_D13, SDRAM_D12,
                               SDRAM_D11, SDRAM_D10, SDRAM_D9, SDRAM_D8,
                               SDRAM_D7, SDRAM_D6, SDRAM_D5, SDRAM_D4,
                               SDRAM_D3, SDRAM_D2, SDRAM_D1, SDRAM_D0};

    // Combine GPIO5-12 inputs into PROBE bus for capture
    wire [7:0] PROBE = {GPIO12, GPIO11, GPIO10, GPIO9, GPIO8, GPIO7, GPIO6, GPIO5};

    // =========================================================================
    // Clock
    // =========================================================================

    wire clk = CLK_25MHz;
    wire rst_n = 1'b1;

    // =========================================================================
    // Heartbeat - 250kHz square wave
    // =========================================================================

    reg [5:0] heartbeat_cnt = 6'd0;
    reg       heartbeat_led = 1'b0;

    always @(posedge clk) begin
        if (heartbeat_cnt == 6'd49) begin
            heartbeat_cnt <= 6'd0;
            heartbeat_led <= ~heartbeat_led;
        end else begin
            heartbeat_cnt <= heartbeat_cnt + 1'b1;
        end
    end

    assign GPIO1 = heartbeat_led;

    // =========================================================================
    // SDRAM Controller
    // =========================================================================

    // SDRAM Command encoding: {nCS, nRAS, nCAS, nWE}
    localparam CMD_NOP        = 4'b0111;
    localparam CMD_ACTIVE     = 4'b0011;
    localparam CMD_READ       = 4'b0101;
    localparam CMD_WRITE      = 4'b0100;
    localparam CMD_PRECHARGE  = 4'b0010;
    localparam CMD_REFRESH    = 4'b0001;
    localparam CMD_LOAD_MODE  = 4'b0000;

    // State machine states
    localparam [4:0]
        // SDRAM initialization states (keep)
        ST_INIT_WAIT       = 5'd0,
        ST_PRECHARGE       = 5'd1,
        ST_REFRESH1        = 5'd2,
        ST_REFRESH2        = 5'd3,
        ST_LOAD_MODE       = 5'd4,

        // Capture engine states (new - replace ST_GEN_*)
        ST_CAP_IDLE        = 5'd5,   // Waiting for ARM command
        ST_CAP_ARMED       = 5'd6,   // Sampling to BRAM, awaiting trigger
        ST_CAP_WRITE       = 5'd7,   // Writing post-trigger sample to SDRAM
        ST_CAP_CAPTURED    = 5'd8,   // Capture complete
        ST_BRAM_XFER       = 5'd9,   // Transferring pre-trigger from BRAM to SDRAM

        // Regenerate states (process captured data through decimate_pack)
        ST_REGEN_INIT      = 5'd10,  // Initialize regeneration
        ST_REGEN_READ      = 5'd11,  // Read sample from capture buffer
        ST_REGEN_WAIT      = 5'd12,  // Wait for SDRAM read
        ST_REGEN_PROCESS   = 5'd13,  // Feed sample to decimate_pack
        ST_REGEN_WRITE     = 5'd14,  // Write display byte to SDRAM
        ST_REGEN_NEXT      = 5'd15,  // Next sample/channel

        // Apple II read states (keep)
        ST_ACTIVATE        = 5'd16,
        ST_READ_CMD        = 5'd17,
        ST_READ_WAIT       = 5'd18,
        ST_READ_CAPTURE    = 5'd19,
        ST_DONE            = 5'd20,

        // Refresh states (keep)
        ST_REFRESH         = 5'd21,
        ST_REFRESH_WAIT    = 5'd22;

    // Display buffer configuration
    localparam BYTES_PER_CHANNEL = 6'd38;
    localparam NUM_CHANNELS = 4'd8;
    localparam DISPLAY_BUFFER_BASE = 13'h0200;  // SDRAM address for display buffer (512, past capture buffer)

    // SDRAM hardware control
    reg [4:0]  sdram_state;
    reg [15:0] sdram_delay;
    reg [3:0]  sdram_cmd;
    reg        sdram_cke;
    reg [12:0] sdram_addr;
    reg [1:0]  sdram_ba;
    reg [15:0] sdram_dq_out;
    reg        sdram_dq_oe;

    // Status flags
    reg        sdram_busy;
    reg        sdram_init_done;
    reg        pattern_loaded;

    // Command interface (Apple II read requests)
    reg        cmd_pending;

    // Latched command parameters (linear address)
    reg [9:0]  latched_linear_addr;  // 10 bits for capture (0-266) + display (512-815)

    // Pattern generation state
    reg [2:0]  gen_channel;       // Current channel (0-7)
    reg [5:0]  gen_byte_idx;      // Current byte within channel (0-37)
    reg [6:0]  gen_sample_cnt;    // Samples generated for current channel
    reg        gen_sample_phase;  // Current sample value (alternating)
    reg        pattern_loading;
    reg [7:0]  gen_wait_cnt;      // Wait counter for sample processing

    // Refresh counter
    localparam REFRESH_INTERVAL = 8'd190;
    reg [7:0]  refresh_cnt;
    reg        refresh_needed;

    // =========================================================================
    // Decimate Pack Module Interface
    // =========================================================================

    reg        dec_rst;
    reg        dec_sample_in;
    reg        dec_sample_valid;
    reg        dec_flush;          // Flush partial byte at end of channel
    wire [6:0] dec_byte_out;
    wire       dec_byte_valid;

    // Byte capture register - captures bytes from decimate_pack
    reg [6:0]  captured_byte;
    reg        byte_pending;
    reg        byte_captured_during_write;  // Flag: new byte arrived during SDRAM write delay

    // =========================================================================
    // Register File
    // =========================================================================

    reg [7:0] reg_channel;   // $00: CHANNEL (0-7)
    reg [7:0] reg_addr;      // $01: ADDR (byte index 0-37)
    reg [7:0] reg_data;      // $02: DATA (read result)
    reg [7:0] reg_stretch;   // $05: STRETCH factor (1-15, default 3)

    // Scratch registers ($06-$0F) - partially repurposed for capture
    reg [7:0] slot_reg [0:15];

    // =========================================================================
    // Capture Engine Configuration Registers
    // =========================================================================

    reg [2:0]  reg_trig_ch;      // $C0C6: Trigger channel (0-7)
    reg        reg_trig_mode;    // $C0C7: 0=rising edge, 1=falling edge
    reg [1:0]  reg_window;       // $C0C8: Window preset (0-3 internal, maps to 1-4)
    reg        reg_debug_en;     // $C0CA: Debug test pattern enable

    // Capture state flags
    reg        armed;            // Capture armed, waiting for trigger
    reg        captured;         // Capture complete, data available

    // Window preset lookup table (active values based on reg_window)
    reg [8:0]  cfg_total_samples;   // Total samples to capture
    reg [3:0]  cfg_pre_samples;     // Pre-trigger sample count
    reg [8:0]  cfg_post_samples;    // Post-trigger sample count
    reg [7:0]  cfg_stretch;         // Stretch factor for display

    always @(*) begin
        case (reg_window)
            2'd0: begin  // Preset 1: 38us window (max zoom)
                cfg_total_samples = 9'd38;
                cfg_pre_samples   = 4'd2;
                cfg_post_samples  = 9'd36;
                cfg_stretch       = 8'd7;
            end
            2'd1: begin  // Preset 2: 88us window (default)
                cfg_total_samples = 9'd88;   // 88 * 3 = 264 pixels (2 short, but no overflow)
                cfg_pre_samples   = 4'd4;
                cfg_post_samples  = 9'd84;
                cfg_stretch       = 8'd3;
            end
            2'd2: begin  // Preset 3: 133us window
                cfg_total_samples = 9'd133;
                cfg_pre_samples   = 4'd7;
                cfg_post_samples  = 9'd126;
                cfg_stretch       = 8'd2;
            end
            2'd3: begin  // Preset 4: 266us window (min zoom)
                cfg_total_samples = 9'd266;
                cfg_pre_samples   = 4'd13;
                cfg_post_samples  = 9'd253;
                cfg_stretch       = 8'd1;
            end
        endcase
    end

    // =========================================================================
    // Apple II Bus Synchronization
    // =========================================================================

    reg [2:0] phi0_sync;

    always @(posedge clk) begin
        phi0_sync <= {phi0_sync[1:0], PHI0};
    end

    wire phi0_falling = phi0_sync[2] && !phi0_sync[1];

    reg        sampled_device_sel;
    reg        sampled_write;
    reg [3:0]  sampled_addr;
    reg [7:0]  sampled_data;

    always @(posedge clk) begin
        if (PHI0) begin
            sampled_device_sel <= ~nDEVICE_SELECT;
            sampled_write      <= ~R_nW;
            sampled_addr       <= apple_addr[3:0];
            sampled_data       <= apple_data_in;
        end
    end

    wire write_strobe = phi0_falling && sampled_device_sel && sampled_write;
    wire device_selected = ~nDEVICE_SELECT;

    // =========================================================================
    // Initialization
    // =========================================================================

    integer i;
    initial begin
        // Register file
        reg_channel = 8'h00;
        reg_addr    = 8'h00;
        reg_data    = 8'hFF;
        reg_stretch = 8'h03;  // Default stretch factor = 3
        for (i = 0; i < 16; i = i + 1)
            slot_reg[i] = 8'hFF;

        // SDRAM state machine
        sdram_state  = ST_INIT_WAIT;
        sdram_delay  = 16'd5000;
        sdram_cmd    = CMD_NOP;
        sdram_addr   = 13'd0;
        sdram_ba     = 2'd0;
        sdram_dq_out = 16'd0;
        sdram_dq_oe  = 1'b0;
        sdram_cke    = 1'b0;

        // Status
        sdram_busy      = 1'b0;
        sdram_init_done = 1'b0;
        pattern_loaded  = 1'b0;

        // Command interface
        cmd_pending         = 1'b0;
        latched_linear_addr = 10'd0;

        // Pattern generation
        gen_channel      = 3'd0;
        gen_byte_idx     = 6'd0;
        gen_sample_cnt   = 7'd0;
        gen_sample_phase = 1'b0;
        pattern_loading  = 1'b0;
        gen_wait_cnt     = 8'd0;

        // Decimate pack control
        dec_rst          = 1'b1;
        dec_sample_in    = 1'b0;
        dec_sample_valid = 1'b0;
        dec_flush        = 1'b0;
        captured_byte    = 7'd0;
        byte_pending     = 1'b0;
        byte_captured_during_write = 1'b0;

        // Refresh
        refresh_cnt    = REFRESH_INTERVAL;
        refresh_needed = 1'b0;

        // Bus synchronization
        phi0_sync          = 3'b0;
        sampled_device_sel = 1'b0;
        sampled_write      = 1'b0;
        sampled_addr       = 4'b0;
        sampled_data       = 8'b0;

        // Capture engine configuration
        reg_trig_ch   = 3'd0;       // Default: trigger on channel 0
        reg_trig_mode = 1'b0;       // Default: rising edge
        reg_window    = 2'd1;       // Default: preset 2 (88us window)
        reg_debug_en  = 1'b0;       // Default: use real probes

        // Capture state
        armed    = 1'b0;
        captured = 1'b0;
    end

    // =========================================================================
    // Decimate Pack Module Instance
    // =========================================================================

    wire dec_ready;  // decimate_pack ready for next sample

    decimate_pack u_decimate_pack (
        .clk(clk),
        .rst(dec_rst),
        .sample_in(dec_sample_in),
        .sample_valid(dec_sample_valid),
        .flush(dec_flush),             // Emit partial byte at end of channel
        .stretch_factor(reg_stretch),  // Full 8 bits (1-255)
        .byte_out(dec_byte_out),
        .byte_valid(dec_byte_valid),
        .ready(dec_ready)
    );

    // =========================================================================
    // Sample Clock Generation (1 MHz from 25 MHz)
    // =========================================================================

    reg [4:0] sample_div;
    wire sample_strobe = (sample_div == 5'd24);

    always @(posedge clk) begin
        if (sample_div == 5'd24)
            sample_div <= 5'd0;
        else
            sample_div <= sample_div + 1'b1;
    end

    // =========================================================================
    // Probe Input Synchronization and Edge Detection
    // =========================================================================

    reg [7:0] probe_sync1;      // First sync stage
    reg [7:0] probe_sync2;      // Second sync stage (use this for sampling)

    always @(posedge clk) begin
        probe_sync1 <= PROBE;
        probe_sync2 <= probe_sync1;
    end

    // =========================================================================
    // Debug Test Pattern Generator
    // =========================================================================
    // When reg_debug_en is set, generates different frequency square waves
    // on each channel for testing without external signals.

    reg [15:0] debug_counter;

    always @(posedge clk) begin
        if (sample_strobe)
            debug_counter <= debug_counter + 1'b1;
    end

    // Each bit divides by different amount (higher bit = slower frequency)
    // PROBE[0] = slowest (~3.9 kHz), PROBE[7] = fastest (~500 kHz)
    wire [7:0] debug_pattern = {
        debug_counter[0],   // PROBE[7]: 500 kHz
        debug_counter[1],   // PROBE[6]: 250 kHz
        debug_counter[2],   // PROBE[5]: 125 kHz
        debug_counter[3],   // PROBE[4]: 62.5 kHz
        debug_counter[4],   // PROBE[3]: 31.25 kHz
        debug_counter[5],   // PROBE[2]: 15.6 kHz
        debug_counter[6],   // PROBE[1]: 7.8 kHz
        debug_counter[7]    // PROBE[0]: 3.9 kHz
    };

    // Mux between real probe inputs and debug pattern
    wire [7:0] probe_input = reg_debug_en ? debug_pattern : probe_sync2;

    // =========================================================================
    // Edge Detection (on probe_input, works with both debug and real probes)
    // =========================================================================

    reg [7:0] probe_prev;       // Previous sample for edge detection

    always @(posedge clk) begin
        if (sample_strobe)
            probe_prev <= probe_input;
    end

    // Edge detection (computed continuously, checked at sample_strobe)
    wire [7:0] probe_rising  = probe_input & ~probe_prev;
    wire [7:0] probe_falling = ~probe_input & probe_prev;

    // =========================================================================
    // Trigger Detection
    // =========================================================================

    // Extract trigger channel edge based on mode
    wire trig_rising  = probe_rising[reg_trig_ch];
    wire trig_falling = probe_falling[reg_trig_ch];
    wire trigger_edge = reg_trig_mode ? trig_falling : trig_rising;

    // Trigger is valid only when armed
    wire trigger_detected = armed && trigger_edge && sample_strobe;

    // =========================================================================
    // Pre-Trigger BRAM Circular Buffer
    // =========================================================================
    // Small circular buffer to store samples before trigger event.
    // Max 13 bytes for preset 4 (266 samples with 13 pre-trigger).

    reg [7:0] pretrig_bram [0:15];  // 16-byte buffer (max 13 used)
    reg [3:0] bram_wr_ptr;          // Write pointer (circular)
    reg [3:0] bram_rd_ptr;          // Read pointer (for transfer)
    reg [3:0] bram_count;           // Number of valid samples in buffer

    // Capture state registers
    reg [8:0]  capture_wr_idx;      // SDRAM write index during capture
    reg [8:0]  post_trigger_cnt;    // Countdown of remaining post-trigger samples
    reg [3:0]  bram_xfer_idx;       // Index during BRAM to SDRAM transfer
    reg [7:0]  current_sample;      // Latched sample for SDRAM write
    reg [7:0]  trigger_sample;      // Preserved trigger sample (not overwritten during BRAM xfer)
    reg        post_trigger_mode;   // Set after trigger, prevents re-triggering

    // Regeneration state registers
    reg [2:0]  regen_channel;       // Current channel being regenerated
    reg [8:0]  regen_sample_idx;    // Current sample index in capture buffer
    reg [5:0]  regen_byte_idx;      // Current byte index in display buffer

    // Initialize sampling and capture state registers
    integer k;
    initial begin
        sample_div        = 5'd0;
        probe_sync1       = 8'd0;
        probe_sync2       = 8'd0;
        probe_prev        = 8'd0;
        debug_counter     = 16'd0;
        bram_wr_ptr       = 4'd0;
        bram_rd_ptr       = 4'd0;
        bram_count        = 4'd0;
        capture_wr_idx    = 9'd0;
        post_trigger_cnt  = 9'd0;
        bram_xfer_idx     = 4'd0;
        current_sample    = 8'd0;
        trigger_sample    = 8'd0;
        post_trigger_mode = 1'b0;
        regen_channel     = 3'd0;
        regen_sample_idx  = 9'd0;
        regen_byte_idx    = 6'd0;
        for (k = 0; k < 16; k = k + 1)
            pretrig_bram[k] = 8'd0;
    end

    // =========================================================================
    // SDRAM Pin Assignments
    // =========================================================================

    assign SDRAM_CLK = clk;
    assign SDRAM_CKE = sdram_cke;
    assign {SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} = sdram_cmd;

    assign SDRAM_BA0 = sdram_ba[0];
    assign SDRAM_BA1 = sdram_ba[1];

    assign SDRAM_A0  = sdram_addr[0];
    assign SDRAM_A1  = sdram_addr[1];
    assign SDRAM_A2  = sdram_addr[2];
    assign SDRAM_A3  = sdram_addr[3];
    assign SDRAM_A4  = sdram_addr[4];
    assign SDRAM_A5  = sdram_addr[5];
    assign SDRAM_A6  = sdram_addr[6];
    assign SDRAM_A7  = sdram_addr[7];
    assign SDRAM_A8  = sdram_addr[8];
    assign SDRAM_A9  = sdram_addr[9];
    assign SDRAM_A10 = sdram_addr[10];
    assign SDRAM_A11 = sdram_addr[11];
    assign SDRAM_A12 = sdram_addr[12];

    assign SDRAM_DQM0 = 1'b0;
    assign SDRAM_DQM1 = 1'b1;

    assign SDRAM_D0  = sdram_dq_oe ? sdram_dq_out[0]  : 1'bZ;
    assign SDRAM_D1  = sdram_dq_oe ? sdram_dq_out[1]  : 1'bZ;
    assign SDRAM_D2  = sdram_dq_oe ? sdram_dq_out[2]  : 1'bZ;
    assign SDRAM_D3  = sdram_dq_oe ? sdram_dq_out[3]  : 1'bZ;
    assign SDRAM_D4  = sdram_dq_oe ? sdram_dq_out[4]  : 1'bZ;
    assign SDRAM_D5  = sdram_dq_oe ? sdram_dq_out[5]  : 1'bZ;
    assign SDRAM_D6  = sdram_dq_oe ? sdram_dq_out[6]  : 1'bZ;
    assign SDRAM_D7  = sdram_dq_oe ? sdram_dq_out[7]  : 1'bZ;
    assign SDRAM_D8  = sdram_dq_oe ? sdram_dq_out[8]  : 1'bZ;
    assign SDRAM_D9  = sdram_dq_oe ? sdram_dq_out[9]  : 1'bZ;
    assign SDRAM_D10 = sdram_dq_oe ? sdram_dq_out[10] : 1'bZ;
    assign SDRAM_D11 = sdram_dq_oe ? sdram_dq_out[11] : 1'bZ;
    assign SDRAM_D12 = sdram_dq_oe ? sdram_dq_out[12] : 1'bZ;
    assign SDRAM_D13 = sdram_dq_oe ? sdram_dq_out[13] : 1'bZ;
    assign SDRAM_D14 = sdram_dq_oe ? sdram_dq_out[14] : 1'bZ;
    assign SDRAM_D15 = sdram_dq_oe ? sdram_dq_out[15] : 1'bZ;

    reg [15:0] sdram_dq_sample = 16'd0;
    always @(posedge clk) begin
        sdram_dq_sample <= sdram_dq_in;
    end

    // =========================================================================
    // Refresh Counter
    // =========================================================================

    always @(posedge clk) begin
        if (sdram_state == ST_INIT_WAIT || sdram_state == ST_REFRESH) begin
            refresh_cnt <= REFRESH_INTERVAL;
            refresh_needed <= 1'b0;
        end else if (refresh_cnt == 0) begin
            refresh_needed <= 1'b1;
        end else begin
            refresh_cnt <= refresh_cnt - 1'b1;
        end
    end

    // =========================================================================
    // Linear Address Calculation
    // =========================================================================

    // For read commands: linear_addr = (reg_channel[2:0] * 38) + reg_addr[5:0]
    wire [8:0] channel_times_38 = ({6'b0, reg_channel[2:0]} << 5) +
                                   ({6'b0, reg_channel[2:0]} << 2) +
                                   ({6'b0, reg_channel[2:0]} << 1);
    wire [8:0] computed_linear_addr = channel_times_38 + {3'b0, reg_addr[5:0]};

    // For pattern generation: gen_linear_addr = (gen_channel * 38) + gen_byte_idx
    wire [8:0] gen_channel_times_38 = ({6'b0, gen_channel} << 5) +
                                       ({6'b0, gen_channel} << 2) +
                                       ({6'b0, gen_channel} << 1);
    wire [8:0] gen_linear_addr = gen_channel_times_38 + {3'b0, gen_byte_idx};

    // =========================================================================
    // Combined Register Write + SDRAM State Machine
    // =========================================================================

    always @(posedge clk) begin

        // =================================================================
        // Register Write Handling
        // =================================================================

        if (write_strobe) begin
            case (sampled_addr)
                4'h0: reg_channel <= sampled_data;
                4'h1: reg_addr <= sampled_data;
                4'h2: ;  // DATA is read-only
                4'h3: begin  // CMD register
                    if (sampled_data == 8'hFF) begin
                        // $FF = SOFT RESET - force state machine back to idle
                        sdram_state <= ST_CAP_IDLE;
                        sdram_busy <= 1'b0;
                        armed <= 1'b0;
                        captured <= 1'b0;
                        pattern_loaded <= 1'b0;
                        cmd_pending <= 1'b0;
                        dec_rst <= 1'b1;  // Reset decimate_pack
                        byte_pending <= 1'b0;
                        byte_captured_during_write <= 1'b0;
                        post_trigger_mode <= 1'b0;
                    end else if (!sdram_busy) begin
                        if (sampled_data == 8'h02 && pattern_loaded) begin
                            // $02 = Read byte from SDRAM display buffer
                            cmd_pending <= 1'b1;
                            sdram_busy <= 1'b1;
                            latched_linear_addr <= DISPLAY_BUFFER_BASE[9:0] + {1'b0, computed_linear_addr};
                        end else if (sampled_data == 8'h10 && captured) begin
                            // $10 = Regenerate display from captured data
                            pattern_loaded <= 1'b0;
                            sdram_busy <= 1'b1;
                            sdram_state <= ST_REGEN_INIT;
                            sdram_delay <= 16'd2;
                        end
                    end
                end
                4'h4: ;  // STATUS is read-only
                4'h5: reg_stretch <= sampled_data;  // STRETCH factor (manual override)
                4'h6: reg_trig_ch <= sampled_data[2:0];  // Trigger channel (0-7)
                4'h7: reg_trig_mode <= sampled_data[0];  // Trigger mode (0=rising, 1=falling)
                4'h8: reg_window <= sampled_data[1:0];   // Window preset (0-3)
                4'h9: begin  // ARM command
                    if (!armed && !sdram_busy) begin
                        // ARM the capture engine
                        armed <= 1'b1;
                        captured <= 1'b0;
                        pattern_loaded <= 1'b0;
                        sdram_busy <= 1'b1;
                        bram_wr_ptr <= 4'd0;
                        bram_count <= 4'd0;
                        post_trigger_mode <= 1'b0;  // Reset post-trigger mode
                        sdram_state <= ST_CAP_ARMED;
                    end
                end
                4'hA: reg_debug_en <= sampled_data[0];   // Debug pattern enable
                default: slot_reg[sampled_addr] <= sampled_data;
            endcase
        end

        // =================================================================
        // SDRAM State Machine
        // =================================================================

        sdram_cmd <= CMD_NOP;
        sdram_dq_oe <= 1'b0;
        dec_sample_valid <= 1'b0;  // Default: no sample pulse

        case (sdram_state)
            // ---- INITIALIZATION ----

            ST_INIT_WAIT: begin
                sdram_cke <= 1'b1;
                if (sdram_delay == 0) begin
                    sdram_state <= ST_PRECHARGE;
                end else begin
                    sdram_delay <= sdram_delay - 1'b1;
                end
            end

            ST_PRECHARGE: begin
                sdram_cmd <= CMD_PRECHARGE;
                sdram_addr <= 13'b0_0100_0000_0000;  // A10=1 all banks
                sdram_delay <= 16'd2;
                sdram_state <= ST_REFRESH1;
            end

            ST_REFRESH1: begin
                if (sdram_delay == 0) begin
                    sdram_cmd <= CMD_REFRESH;
                    sdram_delay <= 16'd8;
                    sdram_state <= ST_REFRESH2;
                end else begin
                    sdram_delay <= sdram_delay - 1'b1;
                end
            end

            ST_REFRESH2: begin
                if (sdram_delay == 0) begin
                    sdram_cmd <= CMD_REFRESH;
                    sdram_delay <= 16'd8;
                    sdram_state <= ST_LOAD_MODE;
                end else begin
                    sdram_delay <= sdram_delay - 1'b1;
                end
            end

            ST_LOAD_MODE: begin
                if (sdram_delay == 0) begin
                    sdram_cmd <= CMD_LOAD_MODE;
                    sdram_ba <= 2'd0;
                    sdram_addr <= 13'b000_0_00_010_0_000;  // CAS=2, burst=1
                    sdram_delay <= 16'd2;
                    sdram_init_done <= 1'b1;
                    sdram_state <= ST_CAP_IDLE;  // Go to capture idle, not pattern gen
                end else begin
                    sdram_delay <= sdram_delay - 1'b1;
                end
            end

            // ---- CAPTURE ENGINE ----

            ST_CAP_IDLE: begin
                // Waiting for ARM command (handled in register write section)
                // Handle refresh while idle
                if (refresh_needed) begin
                    sdram_state <= ST_REFRESH;
                end else if (cmd_pending) begin
                    // Handle read commands while in capture idle
                    cmd_pending <= 1'b0;
                    sdram_state <= ST_ACTIVATE;
                end
            end

            ST_CAP_ARMED: begin
                // Handle refresh even when armed
                if (refresh_needed && !sample_strobe) begin
                    sdram_state <= ST_REFRESH;
                end else if (sample_strobe) begin
                    // Write current sample to circular BRAM buffer
                    pretrig_bram[bram_wr_ptr] <= probe_input;

                    // Update circular buffer pointers
                    if (bram_count < cfg_pre_samples) begin
                        // Buffer not yet full
                        bram_count <= bram_count + 1'b1;
                        bram_wr_ptr <= bram_wr_ptr + 1'b1;
                    end else begin
                        // Buffer full, wrap around
                        bram_wr_ptr <= (bram_wr_ptr == cfg_pre_samples - 1) ? 4'd0 : bram_wr_ptr + 1'b1;
                    end

                    // Check for trigger (only if not already in post-trigger mode)
                    if (trigger_detected && !post_trigger_mode && (bram_count >= cfg_pre_samples)) begin
                        // Trigger detected and we have enough pre-trigger samples
                        post_trigger_mode <= 1'b1;  // Prevent re-triggering
                        // Latch the current sample for first post-trigger write
                        current_sample <= probe_input;
                        trigger_sample <= probe_input;  // Preserve trigger sample (survives BRAM xfer)
                        // Calculate read pointer: oldest sample in circular buffer
                        bram_rd_ptr <= (bram_wr_ptr >= cfg_pre_samples) ?
                                       (bram_wr_ptr - cfg_pre_samples + 1) :
                                       (cfg_pre_samples[3:0] - bram_wr_ptr);
                        bram_xfer_idx <= 4'd0;
                        capture_wr_idx <= 9'd0;
                        post_trigger_cnt <= cfg_post_samples;
                        sdram_state <= ST_BRAM_XFER;
                    end else if (post_trigger_mode) begin
                        // In post-trigger mode, sample and go to write
                        current_sample <= probe_input;
                        sdram_cmd <= CMD_ACTIVE;
                        sdram_ba <= 2'd0;
                        sdram_addr <= 13'd0;
                        sdram_delay <= 16'd2;
                        sdram_state <= ST_CAP_WRITE;
                    end
                end
            end

            ST_BRAM_XFER: begin
                // Transfer pre-trigger samples from BRAM to SDRAM capture buffer
                if (sdram_delay == 0) begin
                    if (bram_xfer_idx < cfg_pre_samples) begin
                        // Activate row for write
                        sdram_cmd <= CMD_ACTIVE;
                        sdram_ba <= 2'd0;
                        sdram_addr <= 13'd0;  // Row 0 for capture buffer
                        current_sample <= pretrig_bram[bram_rd_ptr];
                        bram_rd_ptr <= (bram_rd_ptr == cfg_pre_samples - 1) ? 4'd0 : bram_rd_ptr + 1'b1;
                        sdram_delay <= 16'd2;
                        sdram_state <= ST_CAP_WRITE;
                    end else begin
                        // All pre-trigger samples transferred, now capture post-trigger
                        // Restore the preserved trigger sample for first post-trigger write
                        current_sample <= trigger_sample;
                        sdram_cmd <= CMD_ACTIVE;
                        sdram_ba <= 2'd0;
                        sdram_addr <= 13'd0;
                        sdram_delay <= 16'd2;
                        sdram_state <= ST_CAP_WRITE;
                    end
                end else begin
                    sdram_delay <= sdram_delay - 1'b1;
                end
            end

            ST_CAP_WRITE: begin
                if (sdram_delay == 0) begin
                    // Write current sample to SDRAM capture buffer
                    sdram_cmd <= CMD_WRITE;
                    sdram_addr <= {2'b01, 1'b0, capture_wr_idx, 1'b0};  // Capture buffer at base 0
                    sdram_dq_out <= {8'h00, current_sample};
                    sdram_dq_oe <= 1'b1;
                    capture_wr_idx <= capture_wr_idx + 1'b1;
                    sdram_delay <= 16'd4;  // tWR + tRP

                    // Determine next state
                    if (bram_xfer_idx < cfg_pre_samples) begin
                        // Still transferring pre-trigger samples
                        bram_xfer_idx <= bram_xfer_idx + 1'b1;
                        sdram_state <= ST_BRAM_XFER;
                    end else if (post_trigger_cnt > 0) begin
                        // More post-trigger samples to capture
                        post_trigger_cnt <= post_trigger_cnt - 1'b1;
                        // Wait for next sample strobe
                        sdram_state <= ST_CAP_ARMED;  // Re-use armed state for sampling
                    end else begin
                        // Capture complete
                        armed <= 1'b0;
                        captured <= 1'b1;
                        sdram_busy <= 1'b0;
                        sdram_state <= ST_CAP_CAPTURED;
                    end
                end else begin
                    // While waiting, capture next sample on strobe
                    if (sample_strobe && (bram_xfer_idx >= cfg_pre_samples) && (post_trigger_cnt > 0)) begin
                        current_sample <= probe_input;
                    end
                    sdram_delay <= sdram_delay - 1'b1;
                end
            end

            ST_CAP_CAPTURED: begin
                // Capture complete, waiting for regenerate or new ARM
                if (refresh_needed) begin
                    sdram_state <= ST_REFRESH;
                end else if (cmd_pending) begin
                    cmd_pending <= 1'b0;
                    sdram_state <= ST_ACTIVATE;
                end
            end

            // ---- REGENERATION (process captured data through decimate_pack) ----

            ST_REGEN_INIT: begin
                if (sdram_delay == 0) begin
                    // Initialize regeneration for first channel
                    dec_rst <= 1'b1;  // Reset decimate_pack
                    dec_flush <= 1'b0;
                    regen_channel <= 3'd0;
                    gen_channel <= 3'd0;  // MUST reset for correct display buffer addressing
                    regen_sample_idx <= 9'd0;
                    regen_byte_idx <= 6'd0;
                    gen_byte_idx <= 6'd0;  // Reuse for display buffer byte tracking
                    byte_pending <= 1'b0;
                    byte_captured_during_write <= 1'b0;
                    pattern_loading <= 1'b1;
                    reg_stretch <= cfg_stretch;  // Use window preset stretch factor
                    sdram_delay <= 16'd2;
                    sdram_state <= ST_REGEN_READ;
                end else begin
                    sdram_delay <= sdram_delay - 1'b1;
                end
            end

            ST_REGEN_READ: begin
                // Capture any pending byte from decimate_pack
                if (dec_byte_valid && !byte_pending) begin
                    captured_byte <= dec_byte_out;
                    byte_pending <= 1'b1;
                end

                // Release reset after first cycle
                if (sdram_delay == 1) begin
                    dec_rst <= 1'b0;
                end

                if (sdram_delay == 0) begin
                    // Check if we have a byte to write first
                    if (byte_pending || dec_byte_valid) begin
                        sdram_cmd <= CMD_ACTIVE;
                        sdram_ba <= 2'd0;
                        sdram_addr <= 13'd0;  // Row 0
                        sdram_delay <= 16'd2;
                        sdram_state <= ST_REGEN_WRITE;
                    end else if (regen_sample_idx < cfg_total_samples) begin
                        // Read next sample from capture buffer
                        sdram_cmd <= CMD_ACTIVE;
                        sdram_ba <= 2'd0;
                        sdram_addr <= 13'd0;
                        sdram_delay <= 16'd2;
                        latched_linear_addr <= regen_sample_idx;
                        sdram_state <= ST_REGEN_WAIT;
                    end else begin
                        // All samples processed for this channel
                        // Add delay to let decimate_pack finish processing last sample
                        sdram_delay <= 16'd4;
                        sdram_state <= ST_REGEN_NEXT;
                    end
                end else begin
                    sdram_delay <= sdram_delay - 1'b1;
                end
            end

            ST_REGEN_WAIT: begin
                // Wait for SDRAM read
                if (sdram_delay == 0) begin
                    sdram_cmd <= CMD_READ;
                    sdram_addr <= {2'b01, latched_linear_addr, 1'b0};
                    sdram_delay <= 16'd3;  // CAS latency
                    sdram_state <= ST_REGEN_PROCESS;
                end else begin
                    sdram_delay <= sdram_delay - 1'b1;
                end
            end

            ST_REGEN_PROCESS: begin
                // Capture bytes from decimate_pack
                if (dec_byte_valid && !byte_pending) begin
                    captured_byte <= dec_byte_out;
                    byte_pending <= 1'b1;
                end

                if (sdram_delay == 0) begin
                    // Wait for decimate_pack to be ready before sending next sample
                    // This prevents overrunning when stretch_factor > 2
                    if (dec_ready) begin
                        // Extract the bit for current channel from captured sample
                        dec_sample_in <= sdram_dq_sample[regen_channel];
                        dec_sample_valid <= 1'b1;
                        regen_sample_idx <= regen_sample_idx + 1'b1;
                        sdram_delay <= 16'd2;
                        sdram_state <= ST_REGEN_READ;
                    end
                    // If not ready, stay in this state and keep checking
                end else begin
                    sdram_delay <= sdram_delay - 1'b1;
                end
            end

            ST_REGEN_WRITE: begin
                // Write display byte to SDRAM
                if (sdram_delay == 0) begin
                    // Only write if we haven't filled all 38 bytes yet
                    if (gen_byte_idx < BYTES_PER_CHANNEL) begin
                        sdram_cmd <= CMD_WRITE;
                        // Display buffer address: 0x200 + (channel * 38) + byte_idx
                        sdram_addr <= {2'b01, DISPLAY_BUFFER_BASE[9:0] + {1'b0, gen_channel_times_38} + {4'b0, gen_byte_idx}, 1'b0};
                        sdram_dq_out <= {8'h00, 1'b0, captured_byte};  // Bit 7 = 0 (palette)
                        sdram_dq_oe <= 1'b1;
                        gen_byte_idx <= gen_byte_idx + 1'b1;
                    end
                    sdram_delay <= 16'd4;

                    // Check if we captured a new byte during the delay OR right now
                    if (dec_byte_valid) begin
                        captured_byte <= dec_byte_out;
                        // byte_pending stays 1 - new byte just arrived
                    end else if (byte_captured_during_write) begin
                        // Byte was captured during delay, byte_pending stays 1
                        // captured_byte already has the right value
                    end else begin
                        // No new byte, clear pending
                        byte_pending <= 1'b0;
                    end
                    byte_captured_during_write <= 1'b0;  // Reset flag for next write

                    sdram_state <= ST_REGEN_READ;
                end else begin
                    // Capture any new byte while waiting to issue write command
                    // The byte we're about to write is already committed (captured_byte),
                    // so we can safely overwrite it with a new value for the NEXT write
                    if (dec_byte_valid) begin
                        captured_byte <= dec_byte_out;
                        byte_captured_during_write <= 1'b1;  // Remember we got one
                    end
                    sdram_delay <= sdram_delay - 1'b1;
                end
            end

            ST_REGEN_NEXT: begin
                // Assert flush to emit any partial byte
                dec_flush <= 1'b1;

                // Check for remaining bytes to write
                if (dec_byte_valid && !byte_pending) begin
                    captured_byte <= dec_byte_out;
                    byte_pending <= 1'b1;
                end

                if (sdram_delay == 0) begin
                    if (byte_pending || dec_byte_valid) begin
                        // Write the pending byte (check dec_byte_valid too in case flush just emitted)
                        if (dec_byte_valid && !byte_pending) begin
                            captured_byte <= dec_byte_out;
                            byte_pending <= 1'b1;
                        end
                        sdram_cmd <= CMD_ACTIVE;
                        sdram_ba <= 2'd0;
                        sdram_addr <= 13'd0;
                        sdram_delay <= 16'd2;
                        sdram_state <= ST_REGEN_WRITE;
                    end else if (regen_channel < NUM_CHANNELS - 1) begin
                        // Move to next channel (don't wait for exactly 38 bytes)
                        dec_flush <= 1'b0;
                        regen_channel <= regen_channel + 1'b1;
                        gen_channel <= regen_channel + 1'b1;  // Update for address calc
                        regen_sample_idx <= 9'd0;
                        gen_byte_idx <= 6'd0;
                        dec_rst <= 1'b1;  // Reset decimate_pack for new channel
                        sdram_delay <= 16'd2;
                        sdram_state <= ST_REGEN_READ;
                    end else begin
                        // All channels done
                        dec_flush <= 1'b0;
                        pattern_loading <= 1'b0;
                        pattern_loaded <= 1'b1;
                        sdram_busy <= 1'b0;
                        sdram_state <= ST_CAP_IDLE;
                    end
                end else begin
                    sdram_delay <= sdram_delay - 1'b1;
                end
            end

            // ---- SDRAM READ OPERATIONS ----

            ST_ACTIVATE: begin
                sdram_cmd <= CMD_ACTIVE;
                sdram_ba <= 2'd0;
                sdram_addr <= 13'd0;
                sdram_delay <= 16'd2;
                sdram_state <= ST_READ_CMD;
            end

            ST_READ_CMD: begin
                if (sdram_delay == 0) begin
                    sdram_cmd <= CMD_READ;
                    sdram_addr <= {2'b01, latched_linear_addr, 1'b0};
                    sdram_delay <= 16'd2;
                    sdram_state <= ST_READ_WAIT;
                end else begin
                    sdram_delay <= sdram_delay - 1'b1;
                end
            end

            ST_READ_WAIT: begin
                if (sdram_delay == 0) begin
                    sdram_state <= ST_READ_CAPTURE;
                end else begin
                    sdram_delay <= sdram_delay - 1'b1;
                end
            end

            ST_READ_CAPTURE: begin
                reg_data <= sdram_dq_sample[7:0];
                sdram_delay <= 16'd2;
                sdram_state <= ST_DONE;
            end

            ST_DONE: begin
                if (sdram_delay == 0) begin
                    sdram_busy <= 1'b0;
                    // Return to appropriate capture state
                    sdram_state <= captured ? ST_CAP_CAPTURED : ST_CAP_IDLE;
                end else begin
                    sdram_delay <= sdram_delay - 1'b1;
                end
            end

            // ---- PERIODIC REFRESH ----

            ST_REFRESH: begin
                sdram_cmd <= CMD_REFRESH;
                sdram_delay <= 16'd8;
                sdram_state <= ST_REFRESH_WAIT;
            end

            ST_REFRESH_WAIT: begin
                if (sdram_delay == 0) begin
                    // Return to appropriate capture state
                    if (armed)
                        sdram_state <= ST_CAP_ARMED;
                    else if (captured)
                        sdram_state <= ST_CAP_CAPTURED;
                    else
                        sdram_state <= ST_CAP_IDLE;
                end else begin
                    sdram_delay <= sdram_delay - 1'b1;
                end
            end

            default: sdram_state <= ST_INIT_WAIT;
        endcase
    end

    // =========================================================================
    // Register Read Mux
    // =========================================================================

    wire ready = sdram_init_done && pattern_loaded;
    wire [7:0] status_reg = {4'b0, captured, armed, ready, sdram_busy};

    reg [7:0] read_data;
    always @(*) begin
        read_data = 8'hFF;
        case (apple_addr[3:0])
            4'h0: read_data = reg_channel;
            4'h1: read_data = reg_addr;
            4'h2: read_data = reg_data;
            4'h3: read_data = 8'h00;  // CMD write-only
            4'h4: read_data = status_reg;  // [0]=busy [1]=ready [2]=armed [3]=captured
            4'h5: read_data = cfg_stretch;  // Current stretch (from window preset)
            4'h6: read_data = {5'b0, reg_trig_ch};     // Trigger channel
            4'h7: read_data = {7'b0, reg_trig_mode};   // Trigger mode
            4'h8: read_data = {6'b0, reg_window};      // Window preset (0-3)
            4'h9: read_data = 8'h00;  // ARM write-only
            4'hA: read_data = {7'b0, reg_debug_en};    // Debug enable
            4'hB: read_data = slot_reg[11];
            4'hC: read_data = slot_reg[12];
            4'hD: read_data = slot_reg[13];
            4'hE: read_data = slot_reg[14];
            4'hF: read_data = slot_reg[15];
        endcase
    end

    wire data_drive = device_selected && R_nW;

    assign D0 = data_drive ? read_data[0] : 1'bZ;
    assign D1 = data_drive ? read_data[1] : 1'bZ;
    assign D2 = data_drive ? read_data[2] : 1'bZ;
    assign D3 = data_drive ? read_data[3] : 1'bZ;
    assign D4 = data_drive ? read_data[4] : 1'bZ;
    assign D5 = data_drive ? read_data[5] : 1'bZ;
    assign D6 = data_drive ? read_data[6] : 1'bZ;
    assign D7 = data_drive ? read_data[7] : 1'bZ;

    // =========================================================================
    // Control Lines - Passive
    // =========================================================================

    assign nIRQ = 1'bZ;
    assign nNMI = 1'bZ;
    assign nINH = 1'bZ;
    assign nDMA = 1'bZ;
    assign nRES = 1'b1;

    assign DMA_IN = DMA_OUT;
    assign INT_IN = INT_OUT;

    // =========================================================================
    // GPIO Debug Outputs
    // =========================================================================
    // GPIO1: Heartbeat (assigned above)
    // GPIO2: Ready (system initialized)
    // GPIO3: Armed (waiting for trigger)
    // GPIO4: Captured (capture complete)
    // GPIO5-12: Now inputs for probe capture

    assign GPIO2 = ready;
    assign GPIO3 = armed;
    assign GPIO4 = captured;

endmodule
