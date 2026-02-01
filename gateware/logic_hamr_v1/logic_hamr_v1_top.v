// =============================================================================
// Byte Hamr Logic Hamr - Decimated Sample Pattern Generator
// =============================================================================
//
// Generates waveform patterns using the decimate_pack module to convert
// sample streams into Apple II HIRES-compatible 7-bit bytes.
//
// Pattern Generation:
//   - Generates square wave samples (1010101...)
//   - Processes through decimate_pack with configurable stretch factor
//   - Stores 8 channels × 38 bytes = 304 bytes in SDRAM
//   - Each channel has phase offset (even=start low, odd=start high)
//
// Register Map (offsets from $C0C0):
//   $00 CHANNEL  R/W  Channel select (0-7)
//   $01 ADDR     R/W  Byte index within channel (0-37)
//   $02 DATA     R    Read result from SDRAM
//   $03 CMD      W    $02=Read byte, $10=Regenerate pattern
//   $04 STATUS   R    [0]=busy [1]=ready (init + pattern loaded)
//   $05 STRETCH  R/W  Stretch factor (1-15, default 3)
//   $06-$0F      R/W  Scratch registers (loopback testing)
//
// SDRAM Address Mapping:
//   Linear address = (CHANNEL * 38) + ADDR
//   BA[1:0]     = 2'd0 (bank 0 only)
//   Row[12:0]   = 13'd0 (fixed)
//   Column[9:0] = linear_address[9:0] (max 303 fits in 10 bits)
//
// GPIO outputs:
//   GPIO1:  Heartbeat (250kHz)
//   GPIO2:  Ready (init_done && pattern_loaded)
//   GPIO3:  Pattern load in progress
//   GPIO4:  SDRAM busy
//   GPIO5-12: Debug signals
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
    output wire        GPIO1,
    output wire        GPIO2,
    output wire        GPIO3,
    output wire        GPIO4,
    output wire        GPIO5,
    output wire        GPIO6,
    output wire        GPIO7,
    output wire        GPIO8,
    output wire        GPIO9,
    output wire        GPIO10,
    output wire        GPIO11,
    output wire        GPIO12
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
        ST_INIT_WAIT       = 5'd0,
        ST_PRECHARGE       = 5'd1,
        ST_REFRESH1        = 5'd2,
        ST_REFRESH2        = 5'd3,
        ST_LOAD_MODE       = 5'd4,
        ST_GEN_INIT        = 5'd5,   // Initialize pattern generator
        ST_GEN_SAMPLE      = 5'd6,   // Generate next sample
        ST_GEN_WAIT        = 5'd7,   // Wait for decimate_pack
        ST_GEN_WRITE       = 5'd8,   // Write byte to SDRAM
        ST_GEN_NEXT        = 5'd9,   // Next byte or channel
        ST_IDLE            = 5'd10,
        ST_ACTIVATE        = 5'd11,
        ST_READ_CMD        = 5'd12,
        ST_READ_WAIT       = 5'd13,
        ST_READ_CAPTURE    = 5'd14,
        ST_DONE            = 5'd15,
        ST_REFRESH         = 5'd16,
        ST_REFRESH_WAIT    = 5'd17;

    // Pattern configuration: 8 channels × 38 bytes = 304 total
    localparam BYTES_PER_CHANNEL = 6'd38;
    localparam NUM_CHANNELS = 4'd8;
    // We need 266 pixels (38 × 7) to fill the display
    // Generate square wave samples until we have 38 bytes

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
    reg [8:0]  latched_linear_addr;

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
    wire [6:0] dec_byte_out;
    wire       dec_byte_valid;

    // Byte capture register - captures bytes from decimate_pack
    reg [6:0]  captured_byte;
    reg        byte_pending;

    // =========================================================================
    // Register File
    // =========================================================================

    reg [7:0] reg_channel;   // $00: CHANNEL (0-7)
    reg [7:0] reg_addr;      // $01: ADDR (byte index 0-37)
    reg [7:0] reg_data;      // $02: DATA (read result)
    reg [7:0] reg_stretch;   // $05: STRETCH factor (1-15, default 3)

    // Scratch registers ($06-$0F)
    reg [7:0] slot_reg [0:15];

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
        latched_linear_addr = 9'd0;

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
        captured_byte    = 7'd0;
        byte_pending     = 1'b0;

        // Refresh
        refresh_cnt    = REFRESH_INTERVAL;
        refresh_needed = 1'b0;

        // Bus synchronization
        phi0_sync          = 3'b0;
        sampled_device_sel = 1'b0;
        sampled_write      = 1'b0;
        sampled_addr       = 4'b0;
        sampled_data       = 8'b0;
    end

    // =========================================================================
    // Decimate Pack Module Instance
    // =========================================================================

    decimate_pack u_decimate_pack (
        .clk(clk),
        .rst(dec_rst),
        .sample_in(dec_sample_in),
        .sample_valid(dec_sample_valid),
        .stretch_factor(reg_stretch),  // Full 8 bits (1-255)
        .byte_out(dec_byte_out),
        .byte_valid(dec_byte_valid)
    );

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
                    if (!sdram_busy) begin
                        if (sampled_data == 8'h02 && pattern_loaded) begin
                            // $02 = Read byte from SDRAM
                            cmd_pending <= 1'b1;
                            sdram_busy <= 1'b1;
                            latched_linear_addr <= computed_linear_addr;
                        end else if (sampled_data == 8'h10) begin
                            // $10 = Regenerate pattern with current stretch factor
                            pattern_loaded <= 1'b0;
                            sdram_busy <= 1'b1;
                            sdram_state <= ST_GEN_INIT;
                            sdram_delay <= 16'd2;
                        end
                    end
                end
                4'h4: ;  // STATUS is read-only
                4'h5: reg_stretch <= sampled_data;  // STRETCH factor
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
                    sdram_state <= ST_GEN_INIT;
                end else begin
                    sdram_delay <= sdram_delay - 1'b1;
                end
            end

            // ---- PATTERN GENERATION ----

            ST_GEN_INIT: begin
                if (sdram_delay == 0) begin
                    // Initialize pattern generator for first channel
                    dec_rst <= 1'b1;  // Reset decimate_pack
                    gen_channel <= 3'd0;
                    gen_byte_idx <= 6'd0;
                    gen_sample_cnt <= 7'd0;
                    gen_sample_phase <= 1'b0;  // Channel 0 starts with 0
                    byte_pending <= 1'b0;  // Clear any pending byte from previous run
                    pattern_loading <= 1'b1;
                    sdram_delay <= 16'd2;
                    sdram_state <= ST_GEN_SAMPLE;
                end else begin
                    sdram_delay <= sdram_delay - 1'b1;
                end
            end

            ST_GEN_SAMPLE: begin
                // Capture any byte that becomes valid (continuously)
                if (dec_byte_valid && !byte_pending) begin
                    captured_byte <= dec_byte_out;
                    byte_pending <= 1'b1;
                end

                // Release reset one cycle before generating samples
                if (sdram_delay == 1) begin
                    dec_rst <= 1'b0;
                end

                if (sdram_delay == 0) begin
                    // If a byte just arrived OR was already pending, write it first
                    // (Check dec_byte_valid directly because byte_pending update is non-blocking)
                    if (byte_pending || dec_byte_valid) begin
                        sdram_cmd <= CMD_ACTIVE;
                        sdram_ba <= 2'd0;
                        sdram_addr <= 13'd0;  // Row 0
                        sdram_delay <= 16'd2;
                        sdram_state <= ST_GEN_WRITE;
                    end else if (gen_byte_idx < BYTES_PER_CHANNEL) begin
                        // Generate next sample (square wave until we have 38 bytes)
                        dec_sample_in <= gen_sample_phase;
                        gen_sample_phase <= ~gen_sample_phase;  // Toggle for square wave
                        dec_sample_valid <= 1'b1;
                        gen_sample_cnt <= gen_sample_cnt + 1'b1;
                        gen_wait_cnt <= reg_stretch;  // Wait for stretch cycles (1-255)
                        sdram_state <= ST_GEN_WAIT;
                    end else begin
                        // All 38 bytes written, move to next channel
                        sdram_state <= ST_GEN_NEXT;
                    end
                end else begin
                    sdram_delay <= sdram_delay - 1'b1;
                end
            end

            ST_GEN_WAIT: begin
                // Capture any byte that becomes valid
                if (dec_byte_valid && !byte_pending) begin
                    captured_byte <= dec_byte_out;
                    byte_pending <= 1'b1;
                end

                // If we have a byte to write, go write it immediately
                // (don't wait for sample to finish - we might get another byte!)
                if (byte_pending || dec_byte_valid) begin
                    sdram_cmd <= CMD_ACTIVE;
                    sdram_ba <= 2'd0;
                    sdram_addr <= 13'd0;  // Row 0
                    sdram_delay <= 16'd2;
                    sdram_state <= ST_GEN_WRITE;
                end else if (gen_wait_cnt > 0) begin
                    // Wait for decimate_pack to process stretch_factor pixels
                    gen_wait_cnt <= gen_wait_cnt - 1'b1;
                end else begin
                    // Processing complete, go back to sample generation
                    sdram_state <= ST_GEN_SAMPLE;
                end
            end

            ST_GEN_WRITE: begin
                if (sdram_delay == 0) begin
                    sdram_cmd <= CMD_WRITE;
                    sdram_addr <= {3'b010, gen_linear_addr, 1'b0};  // A10=1 auto-precharge
                    sdram_dq_out <= {8'h00, 1'b0, captured_byte};  // Bit 7 = 0 (palette)
                    sdram_dq_oe <= 1'b1;
                    gen_byte_idx <= gen_byte_idx + 1'b1;
                    sdram_delay <= 16'd4;  // tWR + tRP
                    // Return to appropriate state
                    if (gen_wait_cnt > 0) begin
                        sdram_state <= ST_GEN_WAIT;  // Continue waiting for sample to finish
                    end else begin
                        sdram_state <= ST_GEN_SAMPLE;  // Sample done, get next
                    end
                    // Capture new byte if one arrives, otherwise clear pending
                    if (dec_byte_valid) begin
                        captured_byte <= dec_byte_out;
                        // byte_pending stays 1
                    end else begin
                        byte_pending <= 1'b0;
                    end
                end else begin
                    // Continue capturing bytes while waiting for SDRAM
                    if (dec_byte_valid && !byte_pending) begin
                        captured_byte <= dec_byte_out;
                        byte_pending <= 1'b1;
                    end
                    // Keep counting down sample wait time during SDRAM operations
                    if (gen_wait_cnt > 0) begin
                        gen_wait_cnt <= gen_wait_cnt - 1'b1;
                    end
                    sdram_delay <= sdram_delay - 1'b1;
                end
            end

            ST_GEN_NEXT: begin
                // Capture any remaining bytes
                if (dec_byte_valid && !byte_pending) begin
                    captured_byte <= dec_byte_out;
                    byte_pending <= 1'b1;
                end

                if (sdram_delay == 0) begin
                    // Check if we need to write any pending byte
                    if (byte_pending) begin
                        sdram_cmd <= CMD_ACTIVE;
                        sdram_ba <= 2'd0;
                        sdram_addr <= 13'd0;
                        sdram_delay <= 16'd2;
                        sdram_state <= ST_GEN_WRITE;
                    end else if (gen_byte_idx < BYTES_PER_CHANNEL) begin
                        // Still waiting for bytes, keep waiting
                        sdram_delay <= 16'd3;
                    end else if (gen_channel < NUM_CHANNELS - 1) begin
                        // Move to next channel
                        gen_channel <= gen_channel + 1'b1;
                        gen_byte_idx <= 6'd0;
                        gen_sample_cnt <= 7'd0;
                        gen_sample_phase <= ~gen_channel[0];  // Odd channels start high
                        dec_rst <= 1'b1;  // Reset decimate_pack for new channel
                        sdram_delay <= 16'd2;
                        sdram_state <= ST_GEN_SAMPLE;
                    end else begin
                        // All channels done
                        pattern_loading <= 1'b0;
                        pattern_loaded <= 1'b1;
                        sdram_busy <= 1'b0;  // Clear busy so reads/regenerates work
                        sdram_state <= ST_IDLE;
                    end
                end else begin
                    sdram_delay <= sdram_delay - 1'b1;
                end
            end

            // ---- OPERATIONAL ----

            ST_IDLE: begin
                if (sdram_delay != 0) begin
                    sdram_delay <= sdram_delay - 1'b1;
                end else if (refresh_needed) begin
                    sdram_state <= ST_REFRESH;
                end else if (cmd_pending) begin
                    cmd_pending <= 1'b0;
                    sdram_state <= ST_ACTIVATE;
                end
            end

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
                    sdram_addr <= {3'b010, latched_linear_addr, 1'b0};
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
                    sdram_state <= ST_IDLE;
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
                    sdram_state <= ST_IDLE;
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
    wire [7:0] status_reg = {6'b0, ready, sdram_busy};

    reg [7:0] read_data;
    always @(*) begin
        read_data = 8'hFF;
        case (apple_addr[3:0])
            4'h0: read_data = reg_channel;
            4'h1: read_data = reg_addr;
            4'h2: read_data = reg_data;
            4'h3: read_data = 8'h00;
            4'h4: read_data = status_reg;
            4'h5: read_data = reg_stretch;
            4'h6: read_data = slot_reg[6];
            4'h7: read_data = slot_reg[7];
            4'h8: read_data = slot_reg[8];
            4'h9: read_data = slot_reg[9];
            4'hA: read_data = slot_reg[10];
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
    // GPIO Outputs
    // =========================================================================

    assign GPIO2  = ready;
    assign GPIO3  = pattern_loading;
    assign GPIO4  = sdram_busy;
    assign GPIO5  = PHI0;
    assign GPIO6  = ~nDEVICE_SELECT;
    assign GPIO7  = R_nW;
    assign GPIO8  = dec_byte_valid;
    assign GPIO9  = dec_sample_valid;
    assign GPIO10 = PHI1;
    assign GPIO11 = write_strobe;
    assign GPIO12 = sampled_device_sel;

endmodule
