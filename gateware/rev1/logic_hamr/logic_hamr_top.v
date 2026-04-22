// =============================================================================
// Logic Hamr - Top Level Integration
// =============================================================================
//
// Modular architecture with parallel regeneration.
// All 8 channels processed simultaneously from single SDRAM pass (8x faster).
//
// Module Structure:
//   - sdram_controller:  Central SDRAM interface
//   - capture_engine:    ARM, trigger, raw sample capture
//   - regen_engine:      8x parallel decimate_pack, display buffer generation
//   - bus_interface:     Apple II register reads/writes
//
// Phase State Machine:
//   PHASE_INIT → PHASE_IDLE → PHASE_CAPTURE → PHASE_CAPTURED → PHASE_REGEN → PHASE_READY
//                    ↑                                               ↑
//                    └─────────────── cmd_arm ─────────────────────┘
//                                                                    ↑
//                                cmd_regen (re-zoom!) ─────────────┘
//
// =============================================================================

module logic_hamr_top (
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
    output wire        GPIO1,   // Heartbeat
    output wire        GPIO2,   // Ready
    output wire        GPIO3,   // Armed
    input  wire        GPIO4,   // PROBE[0]
    input  wire        GPIO5,   // PROBE[1]
    input  wire        GPIO6,   // PROBE[2]
    input  wire        GPIO7,   // PROBE[3]
    input  wire        GPIO8,   // PROBE[4]
    input  wire        GPIO9,   // PROBE[5]
    input  wire        GPIO10,  // PROBE[6]
    input  wire        GPIO11,  // (unused)
    output wire        GPIO12   // Directly drives D[7:0] level shifter active-low output enable
);

    // =========================================================================
    // Internal Signal Bundles
    // =========================================================================

    wire [7:0]  apple_data_in = {D7, D6, D5, D4, D3, D2, D1, D0};
    wire [7:0] PROBE = {1'b0, GPIO10, GPIO9, GPIO8, GPIO7, GPIO6, GPIO5, GPIO4};

    // Pass through nDEVICE_SELECT to level shifter OE (active-low)
    assign GPIO12 = nDEVICE_SELECT;

    // =========================================================================
    // Clock and Reset
    // =========================================================================

    wire clk = CLK_25MHz;

    // Power-on reset generator: hold reset low for first 256 clocks (~10us)
    reg [7:0] por_cnt = 8'd0;
    reg       por_done = 1'b0;

    always @(posedge clk) begin
        if (!por_done) begin
            if (por_cnt == 8'hFF)
                por_done <= 1'b1;
            else
                por_cnt <= por_cnt + 1'b1;
        end
    end

    wire rst_n = por_done;

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
    // Sample Clock Generation (1 MHz from 25 MHz)
    // =========================================================================

    reg [4:0] sample_div = 5'd0;
    wire sample_strobe = (sample_div == 5'd24);

    always @(posedge clk) begin
        if (sample_div == 5'd24)
            sample_div <= 5'd0;
        else
            sample_div <= sample_div + 1'b1;
    end

    // =========================================================================
    // Probe Input Synchronization
    // =========================================================================

    reg [7:0] probe_sync1 = 8'd0;
    reg [7:0] probe_sync2 = 8'd0;

    always @(posedge clk) begin
        probe_sync1 <= PROBE;
        probe_sync2 <= probe_sync1;
    end

    // =========================================================================
    // Debug Test Pattern Generator
    // =========================================================================

    reg [15:0] debug_counter = 16'd0;

    always @(posedge clk) begin
        if (sample_strobe)
            debug_counter <= debug_counter + 1'b1;
    end

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

    // =========================================================================
    // Bus Interface Outputs
    // =========================================================================

    wire [2:0]  reg_channel;
    wire [5:0]  reg_addr;
    wire [2:0]  reg_trig_ch;
    wire        reg_trig_mode;
    wire [1:0]  reg_window;
    wire        reg_debug_en;
    wire [7:0]  reg_stretch;
    wire        cmd_read;
    wire        cmd_regen;
    wire        cmd_arm;
    wire        cmd_reset;

    // Mux probe input
    wire [7:0] probe_input = reg_debug_en ? debug_pattern : probe_sync2;

    // =========================================================================
    // Phase State Machine
    // =========================================================================

    localparam [2:0]
        PHASE_INIT     = 3'd0,
        PHASE_IDLE     = 3'd1,
        PHASE_CAPTURE  = 3'd2,
        PHASE_CAPTURED = 3'd3,
        PHASE_REGEN    = 3'd4,
        PHASE_READY    = 3'd5,
        PHASE_READ     = 3'd6;

    reg [2:0] phase = PHASE_INIT;

    // =========================================================================
    // SDRAM Controller
    // =========================================================================

    wire        sdram_init_done;
    wire        sdram_pause_refresh;
    wire        sdram_req;
    wire        sdram_req_write;
    wire [12:0] sdram_req_addr;
    wire [7:0]  sdram_req_wdata;
    wire        sdram_req_ready;
    wire [7:0]  sdram_req_rdata;
    wire        sdram_req_rdata_valid;

    // SDRAM address and data bus
    wire [12:0] sdram_a;
    wire [15:0] sdram_dq;

    sdram_controller u_sdram_controller (
        .clk(clk),
        .rst_n(rst_n),
        .init_done(sdram_init_done),
        .pause_refresh(sdram_pause_refresh),
        .req(sdram_req),
        .req_write(sdram_req_write),
        .req_addr(sdram_req_addr),
        .req_wdata(sdram_req_wdata),
        .req_ready(sdram_req_ready),
        .req_rdata(sdram_req_rdata),
        .req_rdata_valid(sdram_req_rdata_valid),
        .SDRAM_CLK(SDRAM_CLK),
        .SDRAM_CKE(SDRAM_CKE),
        .SDRAM_nCS(SDRAM_nCS),
        .SDRAM_nRAS(SDRAM_nRAS),
        .SDRAM_nCAS(SDRAM_nCAS),
        .SDRAM_nWE(SDRAM_nWE),
        .SDRAM_DQM0(SDRAM_DQM0),
        .SDRAM_DQM1(SDRAM_DQM1),
        .SDRAM_BA0(SDRAM_BA0),
        .SDRAM_BA1(SDRAM_BA1),
        .SDRAM_A(sdram_a),
        .SDRAM_DQ(sdram_dq)
    );

    // SDRAM address pin assignments
    assign SDRAM_A0  = sdram_a[0];
    assign SDRAM_A1  = sdram_a[1];
    assign SDRAM_A2  = sdram_a[2];
    assign SDRAM_A3  = sdram_a[3];
    assign SDRAM_A4  = sdram_a[4];
    assign SDRAM_A5  = sdram_a[5];
    assign SDRAM_A6  = sdram_a[6];
    assign SDRAM_A7  = sdram_a[7];
    assign SDRAM_A8  = sdram_a[8];
    assign SDRAM_A9  = sdram_a[9];
    assign SDRAM_A10 = sdram_a[10];
    assign SDRAM_A11 = sdram_a[11];
    assign SDRAM_A12 = sdram_a[12];

    // SDRAM data pin assignments
    assign SDRAM_D0  = sdram_dq[0];
    assign SDRAM_D1  = sdram_dq[1];
    assign SDRAM_D2  = sdram_dq[2];
    assign SDRAM_D3  = sdram_dq[3];
    assign SDRAM_D4  = sdram_dq[4];
    assign SDRAM_D5  = sdram_dq[5];
    assign SDRAM_D6  = sdram_dq[6];
    assign SDRAM_D7  = sdram_dq[7];
    assign SDRAM_D8  = sdram_dq[8];
    assign SDRAM_D9  = sdram_dq[9];
    assign SDRAM_D10 = sdram_dq[10];
    assign SDRAM_D11 = sdram_dq[11];
    assign SDRAM_D12 = sdram_dq[12];
    assign SDRAM_D13 = sdram_dq[13];
    assign SDRAM_D14 = sdram_dq[14];
    assign SDRAM_D15 = sdram_dq[15];

    // =========================================================================
    // Capture Engine
    // =========================================================================

    wire        cap_armed;
    wire        cap_captured;
    wire [8:0]  cap_total_samples;
    wire        cap_pause_refresh;
    wire        cap_sdram_wr_req;
    wire [12:0] cap_sdram_wr_addr;
    wire [7:0]  cap_sdram_wr_data;

    // Debug signals from capture engine
    wire [2:0]  cap_dbg_state;
    wire [3:0]  cap_dbg_bram_count;
    wire        cap_dbg_trigger_edge;
    wire        cap_dbg_pretrig_ready;

    capture_engine u_capture_engine (
        .clk(clk),
        .rst_n(rst_n),
        .arm(cmd_arm),
        .soft_reset(cmd_reset),
        .probe_input(probe_input),
        .sample_strobe(sample_strobe),
        .trig_ch(reg_trig_ch),
        .trig_mode(reg_trig_mode),
        .window_preset(reg_window),
        .armed(cap_armed),
        .captured(cap_captured),
        .total_samples(cap_total_samples),
        .pause_refresh(cap_pause_refresh),
        .sdram_wr_req(cap_sdram_wr_req),
        .sdram_wr_addr(cap_sdram_wr_addr),
        .sdram_wr_data(cap_sdram_wr_data),
        .sdram_wr_ready(sdram_req_ready),
        .dbg_state(cap_dbg_state),
        .dbg_bram_count(cap_dbg_bram_count),
        .dbg_trigger_edge(cap_dbg_trigger_edge),
        .dbg_pretrig_ready(cap_dbg_pretrig_ready)
    );

    // =========================================================================
    // Regeneration Engine
    // =========================================================================

    wire        regen_busy;
    wire        regen_done;
    wire        regen_sdram_rd_req;
    wire [12:0] regen_sdram_rd_addr;
    wire        regen_sdram_wr_req;
    wire [12:0] regen_sdram_wr_addr;
    wire [7:0]  regen_sdram_wr_data;

    regen_engine u_regen_engine (
        .clk(clk),
        .rst_n(rst_n),
        .start(cmd_regen),
        .soft_reset(cmd_reset),
        .window_preset(reg_window),
        .busy(regen_busy),
        .done(regen_done),
        .sdram_rd_req(regen_sdram_rd_req),
        .sdram_rd_addr(regen_sdram_rd_addr),
        .sdram_rd_ready(sdram_req_ready),
        .sdram_rd_data(sdram_req_rdata),
        .sdram_rd_valid(sdram_req_rdata_valid),
        .sdram_wr_req(regen_sdram_wr_req),
        .sdram_wr_addr(regen_sdram_wr_addr),
        .sdram_wr_data(regen_sdram_wr_data),
        .sdram_wr_ready(sdram_req_ready)
    );

    // =========================================================================
    // Apple II Read Interface
    // =========================================================================

    // Display buffer read state machine
    reg        read_pending = 1'b0;
    reg        read_active = 1'b0;
    reg [12:0] read_addr;
    reg [7:0]  read_data = 8'd0;

    localparam DISPLAY_BUFFER_BASE = 13'h0200;

    // Calculate display buffer address: base + channel * 38 + byte_offset
    wire [8:0] channel_offset = ({6'b0, reg_channel} << 5) +
                                ({6'b0, reg_channel} << 2) +
                                ({6'b0, reg_channel} << 1);  // channel * 38

    always @(posedge clk) begin
        if (cmd_read && !read_pending && !read_active) begin
            read_pending <= 1'b1;
            read_addr <= DISPLAY_BUFFER_BASE + {4'b0, channel_offset} + {7'b0, reg_addr};
        end

        if (read_pending && sdram_req_ready && phase == PHASE_READ) begin
            read_pending <= 1'b0;
            read_active <= 1'b1;
        end

        if (read_active && sdram_req_rdata_valid) begin
            read_data <= sdram_req_rdata;
            read_active <= 1'b0;
        end
    end

    // =========================================================================
    // SDRAM Request Mux
    // =========================================================================

    // Route SDRAM requests based on current phase
    assign sdram_req = (phase == PHASE_CAPTURE) ? cap_sdram_wr_req :
                       (phase == PHASE_REGEN) ? (regen_sdram_rd_req | regen_sdram_wr_req) :
                       (phase == PHASE_READ) ? read_pending :
                       1'b0;

    assign sdram_req_write = (phase == PHASE_CAPTURE) ? 1'b1 :
                             (phase == PHASE_REGEN) ? regen_sdram_wr_req :
                             1'b0;  // Read for PHASE_READ

    assign sdram_req_addr = (phase == PHASE_CAPTURE) ? cap_sdram_wr_addr :
                            (phase == PHASE_REGEN) ? (regen_sdram_wr_req ? regen_sdram_wr_addr : regen_sdram_rd_addr) :
                            read_addr;

    assign sdram_req_wdata = (phase == PHASE_CAPTURE) ? cap_sdram_wr_data :
                             (phase == PHASE_REGEN) ? regen_sdram_wr_data :
                             8'h00;

    assign sdram_pause_refresh = cap_pause_refresh;

    // =========================================================================
    // Status Signals
    // =========================================================================

    wire busy = (phase == PHASE_CAPTURE) || (phase == PHASE_REGEN) ||
                read_pending || read_active || regen_busy;
    wire ready = (phase == PHASE_READY);
    wire armed = cap_armed;
    wire captured = cap_captured;

    // =========================================================================
    // Bus Interface
    // =========================================================================

    // Data bus signals
    wire [7:0] data_in = {D7, D6, D5, D4, D3, D2, D1, D0};
    wire [7:0] data_out;
    wire       data_oe;

    bus_interface u_bus_interface (
        .clk(clk),
        .rst_n(rst_n),
        .phi0(PHI0),
        .addr({A3, A2, A1, A0}),
        .data_in(data_in),
        .data_out(data_out),
        .data_oe(data_oe),
        .ndevice_select(nDEVICE_SELECT),
        .r_nw(R_nW),
        .init_done(sdram_init_done),
        .busy(busy),
        .armed(armed),
        .captured(captured),
        .regen_done(ready),
        .read_data(read_data),
        .dbg_cap_state(cap_dbg_state),
        .dbg_bram_count(cap_dbg_bram_count),
        .dbg_trigger_edge(cap_dbg_trigger_edge),
        .reg_channel(reg_channel),
        .reg_addr(reg_addr),
        .reg_trig_ch(reg_trig_ch),
        .reg_trig_mode(reg_trig_mode),
        .reg_window(reg_window),
        .reg_debug_en(reg_debug_en),
        .cmd_read(cmd_read),
        .cmd_regen(cmd_regen),
        .cmd_arm(cmd_arm),
        .cmd_reset(cmd_reset),
        .reg_stretch(reg_stretch)
    );

    // Apple II data bus tristate drivers
    assign D0 = data_oe ? data_out[0] : 1'bZ;
    assign D1 = data_oe ? data_out[1] : 1'bZ;
    assign D2 = data_oe ? data_out[2] : 1'bZ;
    assign D3 = data_oe ? data_out[3] : 1'bZ;
    assign D4 = data_oe ? data_out[4] : 1'bZ;
    assign D5 = data_oe ? data_out[5] : 1'bZ;
    assign D6 = data_oe ? data_out[6] : 1'bZ;
    assign D7 = data_oe ? data_out[7] : 1'bZ;

    // =========================================================================
    // Phase State Machine
    // =========================================================================

    always @(posedge clk) begin
        if (cmd_reset) begin
            phase <= PHASE_IDLE;
        end else begin
            case (phase)
                PHASE_INIT: begin
                    if (sdram_init_done)
                        phase <= PHASE_IDLE;
                end

                PHASE_IDLE: begin
                    if (cmd_arm)
                        phase <= PHASE_CAPTURE;
                    else if (read_pending)
                        phase <= PHASE_READ;
                end

                PHASE_CAPTURE: begin
                    if (cap_captured)
                        phase <= PHASE_CAPTURED;
                end

                PHASE_CAPTURED: begin
                    if (cmd_regen)
                        phase <= PHASE_REGEN;
                    else if (cmd_arm)
                        phase <= PHASE_CAPTURE;
                end

                PHASE_REGEN: begin
                    if (regen_done)
                        phase <= PHASE_READY;
                end

                PHASE_READY: begin
                    if (cmd_arm)
                        phase <= PHASE_CAPTURE;
                    else if (cmd_regen)
                        phase <= PHASE_REGEN;
                    else if (read_pending)
                        phase <= PHASE_READ;
                end

                PHASE_READ: begin
                    if (!read_pending && !read_active)
                        phase <= PHASE_READY;
                end

                default: phase <= PHASE_INIT;
            endcase
        end
    end

    // =========================================================================
    // Unused Apple II Bus Signals
    // =========================================================================

    assign nIRQ = 1'bZ;
    assign nNMI = 1'bZ;
    assign nINH = 1'bZ;
    assign nDMA = 1'bZ;
    assign nRES = 1'b1;

    assign DMA_IN = DMA_OUT;
    assign INT_IN = INT_OUT;

    // =========================================================================
    // GPIO Status Outputs
    // =========================================================================

    assign GPIO2 = ready;     // Display buffer ready for reading
    assign GPIO3 = armed;     // Capture engine armed, waiting for trigger

endmodule
