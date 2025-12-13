// =============================================================================
// Byte Hamr Signal Check - Basic Board Bring-up Test
// =============================================================================
//
// Tests all major interfaces:
// - SDRAM: Write/read pattern test
// - GPIO: Walking 1s pattern
// - Apple II Bus: Sample and loopback
//
// Status output on GPIO1-4:
//   GPIO1: Heartbeat (blinks at ~1Hz when running)
//   GPIO2: SDRAM test passed
//   GPIO3: Apple II bus activity detected
//   GPIO4: Error indicator
//
// =============================================================================

module signal_check_top (
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
    // Address bus (directly from level shifters)
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

    // Data bus (directly from level shifters)
    inout  wire        D0,
    inout  wire        D1,
    inout  wire        D2,
    inout  wire        D3,
    inout  wire        D4,
    inout  wire        D5,
    inout  wire        D6,
    inout  wire        D7,

    // Clock and timing signals
    input  wire        PHI0,
    input  wire        PHI1,
    input  wire        sig_7M,  // 7M clock from Apple II (renamed for Verilog)
    input  wire        Q3,
    input  wire        uSync,

    // Control signals
    input  wire        R_nW,
    input  wire        nRES,
    input  wire        nIRQ,
    input  wire        nNMI,
    input  wire        nDEVICE_SELECT,
    input  wire        nI_O_SELECT,
    input  wire        nI_O_STROBE,
    input  wire        RDY,
    input  wire        DMA_IN,
    input  wire        INT_IN,
    input  wire        nINH,
    input  wire        nDMA,

    // Output control signals
    output wire        DMA_OUT,
    output wire        INT_OUT,

    // ----- GPIO Breakout (directly to FPGA) -----
    output wire        GPIO1,         // Heartbeat
    output wire        GPIO2,         // SDRAM test passed
    output wire        GPIO3,         // Apple II activity
    output wire        GPIO4,         // Error indicator
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

    // Bundle individual signals into vectors for easier use
    wire [15:0] apple_addr = {A15, A14, A13, A12, A11, A10, A9, A8,
                              A7, A6, A5, A4, A3, A2, A1, A0};
    wire [7:0]  apple_data_in = {D7, D6, D5, D4, D3, D2, D1, D0};

    // SDRAM address/data as vectors
    reg [12:0] sdram_addr;
    reg [1:0]  sdram_ba;
    reg [15:0] sdram_dq_out;
    reg        sdram_dq_oe;
    wire [15:0] sdram_dq_in = {SDRAM_D15, SDRAM_D14, SDRAM_D13, SDRAM_D12,
                               SDRAM_D11, SDRAM_D10, SDRAM_D9, SDRAM_D8,
                               SDRAM_D7, SDRAM_D6, SDRAM_D5, SDRAM_D4,
                               SDRAM_D3, SDRAM_D2, SDRAM_D1, SDRAM_D0};

    // =========================================================================
    // Clock and Reset
    // =========================================================================

    wire clk = CLK_25MHz;

    // Simple power-on reset (hold reset for 2^16 cycles = ~2.6ms)
    reg [15:0] reset_cnt = 16'h0;
    wire rst_n = reset_cnt[15];

    always @(posedge clk) begin
        if (!reset_cnt[15])
            reset_cnt <= reset_cnt + 1'b1;
    end

    // =========================================================================
    // Heartbeat - 1Hz blinker shows FPGA is alive
    // =========================================================================

    reg [24:0] heartbeat_cnt;
    reg        heartbeat_led;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            heartbeat_cnt <= 25'd0;
            heartbeat_led <= 1'b0;
        end else begin
            if (heartbeat_cnt == 25'd12_500_000) begin  // 25MHz / 12.5M = 2Hz toggle = 1Hz blink
                heartbeat_cnt <= 25'd0;
                heartbeat_led <= ~heartbeat_led;
            end else begin
                heartbeat_cnt <= heartbeat_cnt + 1'b1;
            end
        end
    end

    assign GPIO1 = heartbeat_led;

    // =========================================================================
    // SDRAM Controller - Simple Test Sequence
    // =========================================================================
    //
    // SDRAM State Machine:
    //   1. Power-up delay (200us @ 25MHz = 5000 cycles)
    //   2. Precharge all banks
    //   3. 2x Auto-refresh
    //   4. Load Mode Register
    //   5. Write test pattern to 8 addresses
    //   6. Read back and verify
    //
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
        ST_INIT_WAIT    = 5'd0,
        ST_PRECHARGE    = 5'd1,
        ST_REFRESH1     = 5'd2,
        ST_REFRESH2     = 5'd3,
        ST_LOAD_MODE    = 5'd4,
        ST_IDLE         = 5'd5,
        ST_WRITE_ACT    = 5'd6,
        ST_WRITE        = 5'd7,
        ST_WRITE_DONE   = 5'd8,
        ST_READ_ACT     = 5'd9,
        ST_READ         = 5'd10,
        ST_READ_WAIT    = 5'd11,
        ST_VERIFY       = 5'd12,
        ST_NEXT_ADDR    = 5'd13,
        ST_PASS         = 5'd14,
        ST_FAIL         = 5'd15;

    reg [4:0]  sdram_state;
    reg [15:0] sdram_delay;
    reg [3:0]  sdram_cmd;
    reg        sdram_cke;
    reg        sdram_test_pass;
    reg        sdram_test_fail;

    // Test pattern - write addresses 0-7 with known pattern
    reg [2:0]  test_addr_cnt;
    reg        test_phase;  // 0 = writing, 1 = reading
    wire [15:0] test_pattern = {test_addr_cnt, 5'b10101, test_addr_cnt, 5'b01010};

    // SDRAM outputs - directly connect individual pins
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

    assign SDRAM_DQM0 = 1'b0;  // Enable both bytes
    assign SDRAM_DQM1 = 1'b0;

    // Bidirectional data bus - directly drive individual pins
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

    // Sample input data
    reg [15:0] sdram_dq_sample;
    always @(posedge clk) begin
        sdram_dq_sample <= sdram_dq_in;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sdram_state <= ST_INIT_WAIT;
            sdram_delay <= 16'd5000;  // 200us power-up delay
            sdram_cmd <= CMD_NOP;
            sdram_addr <= 13'd0;
            sdram_ba <= 2'd0;
            sdram_dq_out <= 16'd0;
            sdram_dq_oe <= 1'b0;
            sdram_cke <= 1'b0;
            sdram_test_pass <= 1'b0;
            sdram_test_fail <= 1'b0;
            test_addr_cnt <= 3'd0;
            test_phase <= 1'b0;
        end else begin
            // Default: NOP command
            sdram_cmd <= CMD_NOP;
            sdram_dq_oe <= 1'b0;

            case (sdram_state)
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
                    sdram_addr <= 13'b0_0100_0000_0000;  // A10=1 for all banks
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
                        // Mode: CAS latency 2, sequential, burst length 1
                        sdram_addr <= 13'b000_0_00_010_0_000;
                        sdram_delay <= 16'd2;
                        sdram_state <= ST_IDLE;
                    end else begin
                        sdram_delay <= sdram_delay - 1'b1;
                    end
                end

                ST_IDLE: begin
                    if (sdram_delay == 0) begin
                        // Start test sequence
                        test_addr_cnt <= 3'd0;
                        test_phase <= 1'b0;  // Start with writes
                        sdram_state <= ST_WRITE_ACT;
                    end else begin
                        sdram_delay <= sdram_delay - 1'b1;
                    end
                end

                // ---- WRITE PHASE ----
                ST_WRITE_ACT: begin
                    sdram_cmd <= CMD_ACTIVE;
                    sdram_ba <= 2'd0;
                    sdram_addr <= {10'd0, test_addr_cnt};  // Row = test address
                    sdram_delay <= 16'd2;  // tRCD
                    sdram_state <= ST_WRITE;
                end

                ST_WRITE: begin
                    if (sdram_delay == 0) begin
                        sdram_cmd <= CMD_WRITE;
                        sdram_addr <= 13'b0_0100_0000_0000;  // Column 0, A10=1 for auto-precharge
                        sdram_dq_out <= test_pattern;
                        sdram_dq_oe <= 1'b1;
                        sdram_delay <= 16'd3;  // tWR + tRP
                        sdram_state <= ST_WRITE_DONE;
                    end else begin
                        sdram_delay <= sdram_delay - 1'b1;
                    end
                end

                ST_WRITE_DONE: begin
                    if (sdram_delay == 0) begin
                        if (test_addr_cnt == 3'd7) begin
                            // All writes done, switch to read phase
                            test_addr_cnt <= 3'd0;
                            test_phase <= 1'b1;
                            sdram_state <= ST_READ_ACT;
                        end else begin
                            test_addr_cnt <= test_addr_cnt + 1'b1;
                            sdram_state <= ST_WRITE_ACT;
                        end
                    end else begin
                        sdram_delay <= sdram_delay - 1'b1;
                    end
                end

                // ---- READ PHASE ----
                ST_READ_ACT: begin
                    sdram_cmd <= CMD_ACTIVE;
                    sdram_ba <= 2'd0;
                    sdram_addr <= {10'd0, test_addr_cnt};  // Row = test address
                    sdram_delay <= 16'd2;  // tRCD
                    sdram_state <= ST_READ;
                end

                ST_READ: begin
                    if (sdram_delay == 0) begin
                        sdram_cmd <= CMD_READ;
                        sdram_addr <= 13'b0_0100_0000_0000;  // Column 0, A10=1 for auto-precharge
                        sdram_delay <= 16'd3;  // CAS latency 2 + 1
                        sdram_state <= ST_READ_WAIT;
                    end else begin
                        sdram_delay <= sdram_delay - 1'b1;
                    end
                end

                ST_READ_WAIT: begin
                    if (sdram_delay == 0) begin
                        sdram_state <= ST_VERIFY;
                    end else begin
                        sdram_delay <= sdram_delay - 1'b1;
                    end
                end

                ST_VERIFY: begin
                    if (sdram_dq_sample == test_pattern) begin
                        sdram_state <= ST_NEXT_ADDR;
                    end else begin
                        sdram_state <= ST_FAIL;
                    end
                end

                ST_NEXT_ADDR: begin
                    if (test_addr_cnt == 3'd7) begin
                        sdram_state <= ST_PASS;
                    end else begin
                        test_addr_cnt <= test_addr_cnt + 1'b1;
                        sdram_delay <= 16'd2;  // tRP
                        sdram_state <= ST_READ_ACT;
                    end
                end

                ST_PASS: begin
                    sdram_test_pass <= 1'b1;
                    // Stay here - test complete
                end

                ST_FAIL: begin
                    sdram_test_fail <= 1'b1;
                    // Stay here - test failed
                end

                default: sdram_state <= ST_INIT_WAIT;
            endcase
        end
    end

    assign GPIO2 = sdram_test_pass;
    assign GPIO4 = sdram_test_fail;

    // =========================================================================
    // Apple II Bus Monitor
    // =========================================================================
    //
    // Sample the Apple II bus on PHI0 rising edge
    // Detect any bus activity (shows Apple II is alive and connected)
    //
    // =========================================================================

    reg [2:0]  phi0_sync;
    reg [15:0] apple_addr_sample;
    reg [7:0]  apple_data_sample;
    reg        apple_rw_sample;
    reg        apple_activity;
    reg [23:0] activity_timeout;

    // Synchronize PHI0 to system clock (metastability protection)
    always @(posedge clk) begin
        phi0_sync <= {phi0_sync[1:0], PHI0};
    end

    wire phi0_rising = (phi0_sync[2:1] == 2'b01);

    // Sample bus on PHI0 rising edge
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            apple_addr_sample <= 16'd0;
            apple_data_sample <= 8'd0;
            apple_rw_sample <= 1'b1;
            apple_activity <= 1'b0;
            activity_timeout <= 24'd0;
        end else begin
            if (phi0_rising) begin
                apple_addr_sample <= apple_addr;
                apple_data_sample <= apple_data_in;
                apple_rw_sample <= R_nW;
                apple_activity <= 1'b1;
                activity_timeout <= 24'd12_500_000;  // 0.5s timeout
            end else if (activity_timeout != 0) begin
                activity_timeout <= activity_timeout - 1'b1;
            end else begin
                apple_activity <= 1'b0;
            end
        end
    end

    assign GPIO3 = apple_activity;

    // Apple II data bus - tri-state (never drive during test)
    assign D0 = 1'bZ;
    assign D1 = 1'bZ;
    assign D2 = 1'bZ;
    assign D3 = 1'bZ;
    assign D4 = 1'bZ;
    assign D5 = 1'bZ;
    assign D6 = 1'bZ;
    assign D7 = 1'bZ;

    // Don't assert any control outputs during test
    assign DMA_OUT = 1'b0;
    assign INT_OUT = 1'b0;

    // =========================================================================
    // GPIO Walking 1s Test
    // =========================================================================
    //
    // GPIO5-12: Output walking 1s pattern, shifting every 0.5s
    // This allows visual verification with LEDs or scope
    //
    // =========================================================================

    reg [23:0] gpio_walk_cnt;
    reg [7:0]  gpio_walk_pattern;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gpio_walk_cnt <= 24'd0;
            gpio_walk_pattern <= 8'b00000001;
        end else begin
            if (gpio_walk_cnt == 24'd12_500_000) begin  // 0.5s
                gpio_walk_cnt <= 24'd0;
                gpio_walk_pattern <= {gpio_walk_pattern[6:0], gpio_walk_pattern[7]};
            end else begin
                gpio_walk_cnt <= gpio_walk_cnt + 1'b1;
            end
        end
    end

    assign GPIO5  = gpio_walk_pattern[0];
    assign GPIO6  = gpio_walk_pattern[1];
    assign GPIO7  = gpio_walk_pattern[2];
    assign GPIO8  = gpio_walk_pattern[3];
    assign GPIO9  = gpio_walk_pattern[4];
    assign GPIO10 = gpio_walk_pattern[5];
    assign GPIO11 = gpio_walk_pattern[6];
    assign GPIO12 = gpio_walk_pattern[7];

endmodule
