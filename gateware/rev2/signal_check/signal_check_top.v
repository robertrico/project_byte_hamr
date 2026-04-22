// =============================================================================
// Byte Hamr Rev 2 - Signal Check (Board Bring-up Test)
// =============================================================================
//
// Tests all major interfaces:
// - SDRAM: write/read pattern test
// - GPIO: walking 1s across GPIO_9..GPIO_20 (12 new Rev 2 pins)
// - Apple II bus: sample + $C0C0-$C0CF loopback register file
// - Data bus transceiver (U12) + R/W inverter (U7) via loopback
// - nRES_READ: new Rev 2 bidirectional reset monitoring
//
// Status output:
//   GPIO_1: Heartbeat (~250 kHz)
//   GPIO_2: SDRAM test passed
//   GPIO_3: Apple II I/O SELECT or STROBE active
//   GPIO_4: nDEVICE_SELECT active
//   GPIO_5: R_nW debug (HIGH = read)
//   GPIO_6: write_pending debug (HIGH when write captured)
//   GPIO_7: D0 input debug (raw)
//   GPIO_8: nRES_READ (HIGH = Apple II not in reset)  ← Rev 2 only
//   GPIO_9..GPIO_20: Walking 1s (12-bit rotation, exercises new pins)
//
// Clock: 100 MHz from F3 oscillator, divided to 25 MHz internally.
// Counter-bit clock is adequate here — all loads run at 25 MHz or slower.
// =============================================================================

module signal_check_top (
    // System clock (Rev 2: 100 MHz)
    input  wire        CLK_100MHz,

    // ----- SDRAM -----
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
    output wire        SDRAM_A0, SDRAM_A1, SDRAM_A2, SDRAM_A3,
    output wire        SDRAM_A4, SDRAM_A5, SDRAM_A6, SDRAM_A7,
    output wire        SDRAM_A8, SDRAM_A9, SDRAM_A10, SDRAM_A11, SDRAM_A12,
    inout  wire        SDRAM_D0, SDRAM_D1, SDRAM_D2, SDRAM_D3,
    inout  wire        SDRAM_D4, SDRAM_D5, SDRAM_D6, SDRAM_D7,
    inout  wire        SDRAM_D8, SDRAM_D9, SDRAM_D10, SDRAM_D11,
    inout  wire        SDRAM_D12, SDRAM_D13, SDRAM_D14, SDRAM_D15,

    // ----- Apple II Bus -----
    input  wire        A0, A1, A2, A3, A4, A5, A6, A7,
    input  wire        A8, A9, A10, A11, A12, A13, A14, A15,
    inout  wire        D0, D1, D2, D3, D4, D5, D6, D7,

    // Timing
    input  wire        PHI0,
    input  wire        PHI1,
    input  wire        sig_7M,
    input  wire        Q3,
    input  wire        uSync,

    // Control
    input  wire        R_nW,
    input  wire        nDEVICE_SELECT,
    input  wire        nI_O_SELECT,
    input  wire        nI_O_STROBE,
    input  wire        DMA_OUT,
    input  wire        INT_OUT,
    input  wire        RDY,
    input  wire        nRES_READ,       // Rev 2: monitor Apple II reset line

    output wire        nIRQ,
    output wire        nNMI,
    output wire        nINH,
    output wire        nDMA,
    output wire        nRES,            // OPENDRAIN write-side (hold released)
    output wire        DMA_IN,
    output wire        INT_IN,

    // Rev 2: data bus transceiver (U12) OE. Active-low.
    output wire        DATA_OE,

    // ----- GPIO Header (Rev 2: 20 pins) -----
    output wire        GPIO_1,  GPIO_2,  GPIO_3,  GPIO_4,  GPIO_5,
    output wire        GPIO_6,  GPIO_7,  GPIO_8,  GPIO_9,  GPIO_10,
    output wire        GPIO_11, GPIO_12, GPIO_13, GPIO_14, GPIO_15,
    output wire        GPIO_16, GPIO_17, GPIO_18, GPIO_19, GPIO_20
);

    // =========================================================================
    // Signal bundles
    // =========================================================================
    wire [15:0] apple_addr = {A15, A14, A13, A12, A11, A10, A9, A8,
                              A7, A6, A5, A4, A3, A2, A1, A0};
    wire [7:0]  apple_data_in = {D7, D6, D5, D4, D3, D2, D1, D0};

    reg [12:0] sdram_addr;
    reg [1:0]  sdram_ba;
    reg [15:0] sdram_dq_out;
    reg        sdram_dq_oe;
    wire [15:0] sdram_dq_in = {SDRAM_D15, SDRAM_D14, SDRAM_D13, SDRAM_D12,
                               SDRAM_D11, SDRAM_D10, SDRAM_D9, SDRAM_D8,
                               SDRAM_D7, SDRAM_D6, SDRAM_D5, SDRAM_D4,
                               SDRAM_D3, SDRAM_D2, SDRAM_D1, SDRAM_D0};

    // =========================================================================
    // Clock: 100 MHz -> 25 MHz via /4 counter
    // =========================================================================
    // Using a counter bit as a clock is acceptable here because all logic runs
    // at 25 MHz with plenty of margin. Swap for EHXPLLL in production designs.
    (* keep = "true" *) reg [1:0] clk_div = 2'd0;
    always @(posedge CLK_100MHz) clk_div <= clk_div + 1'b1;
    (* keep = "true" *) wire clk = clk_div[1];

    wire rst_n = 1'b1;

    // =========================================================================
    // Heartbeat: ~250 kHz square wave on GPIO_1
    // =========================================================================
    reg [5:0] heartbeat_cnt  = 6'd0;
    reg       heartbeat_led = 1'b0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            heartbeat_cnt <= 6'd0;
            heartbeat_led <= 1'b0;
        end else begin
            if (heartbeat_cnt == 6'd49) begin  // 25M / 50 = 500k toggle = 250k sq
                heartbeat_cnt <= 6'd0;
                heartbeat_led <= ~heartbeat_led;
            end else begin
                heartbeat_cnt <= heartbeat_cnt + 1'b1;
            end
        end
    end
    assign GPIO_1 = heartbeat_led;

    // =========================================================================
    // SDRAM init + write/read pattern test
    // =========================================================================
    localparam CMD_NOP       = 4'b0111;
    localparam CMD_ACTIVE    = 4'b0011;
    localparam CMD_READ      = 4'b0101;
    localparam CMD_WRITE     = 4'b0100;
    localparam CMD_PRECHARGE = 4'b0010;
    localparam CMD_REFRESH   = 4'b0001;
    localparam CMD_LOAD_MODE = 4'b0000;

    localparam [4:0]
        ST_INIT_WAIT  = 5'd0,  ST_PRECHARGE  = 5'd1,
        ST_REFRESH1   = 5'd2,  ST_REFRESH2   = 5'd3,
        ST_LOAD_MODE  = 5'd4,  ST_IDLE       = 5'd5,
        ST_WRITE_ACT  = 5'd6,  ST_WRITE      = 5'd7,  ST_WRITE_DONE = 5'd8,
        ST_READ_ACT   = 5'd9,  ST_READ       = 5'd10, ST_READ_WAIT  = 5'd11,
        ST_VERIFY     = 5'd12, ST_NEXT_ADDR  = 5'd13,
        ST_PASS       = 5'd14, ST_FAIL       = 5'd15;

    reg [4:0]  sdram_state     = 5'd0;
    reg [15:0] sdram_delay     = 16'd5000;
    reg [3:0]  sdram_cmd       = 4'b0111;  // CMD_NOP
    reg        sdram_cke       = 1'b0;
    reg        sdram_test_pass = 1'b0;
    reg        sdram_test_fail = 1'b0;

    reg [2:0]  test_addr_cnt = 3'd0;
    reg        test_phase    = 1'b0;
    wire [15:0] test_pattern = {test_addr_cnt, 5'b10101, test_addr_cnt, 5'b01010};

    assign SDRAM_CLK = clk;
    assign SDRAM_CKE = sdram_cke;
    assign {SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} = sdram_cmd;
    assign SDRAM_BA0 = sdram_ba[0];
    assign SDRAM_BA1 = sdram_ba[1];

    assign SDRAM_A0  = sdram_addr[0];   assign SDRAM_A1  = sdram_addr[1];
    assign SDRAM_A2  = sdram_addr[2];   assign SDRAM_A3  = sdram_addr[3];
    assign SDRAM_A4  = sdram_addr[4];   assign SDRAM_A5  = sdram_addr[5];
    assign SDRAM_A6  = sdram_addr[6];   assign SDRAM_A7  = sdram_addr[7];
    assign SDRAM_A8  = sdram_addr[8];   assign SDRAM_A9  = sdram_addr[9];
    assign SDRAM_A10 = sdram_addr[10];  assign SDRAM_A11 = sdram_addr[11];
    assign SDRAM_A12 = sdram_addr[12];

    assign SDRAM_DQM0 = 1'b0;
    assign SDRAM_DQM1 = 1'b0;

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

    reg [15:0] sdram_dq_sample;
    always @(posedge clk) sdram_dq_sample <= sdram_dq_in;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sdram_state     <= ST_INIT_WAIT;
            sdram_delay     <= 16'd5000;   // 200us @ 25MHz
            sdram_cmd       <= CMD_NOP;
            sdram_addr      <= 13'd0;
            sdram_ba        <= 2'd0;
            sdram_dq_out    <= 16'd0;
            sdram_dq_oe     <= 1'b0;
            sdram_cke       <= 1'b0;
            sdram_test_pass <= 1'b0;
            sdram_test_fail <= 1'b0;
            test_addr_cnt   <= 3'd0;
            test_phase      <= 1'b0;
        end else begin
            sdram_cmd   <= CMD_NOP;
            sdram_dq_oe <= 1'b0;

            case (sdram_state)
                ST_INIT_WAIT: begin
                    sdram_cke <= 1'b1;
                    if (sdram_delay == 0) sdram_state <= ST_PRECHARGE;
                    else                  sdram_delay <= sdram_delay - 1'b1;
                end
                ST_PRECHARGE: begin
                    sdram_cmd   <= CMD_PRECHARGE;
                    sdram_addr  <= 13'b0_0100_0000_0000;
                    sdram_delay <= 16'd2;
                    sdram_state <= ST_REFRESH1;
                end
                ST_REFRESH1: begin
                    if (sdram_delay == 0) begin
                        sdram_cmd   <= CMD_REFRESH;
                        sdram_delay <= 16'd8;
                        sdram_state <= ST_REFRESH2;
                    end else sdram_delay <= sdram_delay - 1'b1;
                end
                ST_REFRESH2: begin
                    if (sdram_delay == 0) begin
                        sdram_cmd   <= CMD_REFRESH;
                        sdram_delay <= 16'd8;
                        sdram_state <= ST_LOAD_MODE;
                    end else sdram_delay <= sdram_delay - 1'b1;
                end
                ST_LOAD_MODE: begin
                    if (sdram_delay == 0) begin
                        sdram_cmd   <= CMD_LOAD_MODE;
                        sdram_ba    <= 2'd0;
                        sdram_addr  <= 13'b000_0_00_010_0_000;  // CL=2, burst=1
                        sdram_delay <= 16'd2;
                        sdram_state <= ST_IDLE;
                    end else sdram_delay <= sdram_delay - 1'b1;
                end
                ST_IDLE: begin
                    if (sdram_delay == 0) begin
                        test_addr_cnt <= 3'd0;
                        test_phase    <= 1'b0;
                        sdram_state   <= ST_WRITE_ACT;
                    end else sdram_delay <= sdram_delay - 1'b1;
                end
                ST_WRITE_ACT: begin
                    sdram_cmd   <= CMD_ACTIVE;
                    sdram_ba    <= 2'd0;
                    sdram_addr  <= {10'd0, test_addr_cnt};
                    sdram_delay <= 16'd2;
                    sdram_state <= ST_WRITE;
                end
                ST_WRITE: begin
                    if (sdram_delay == 0) begin
                        sdram_cmd    <= CMD_WRITE;
                        sdram_addr   <= 13'b0_0100_0000_0000;
                        sdram_dq_out <= test_pattern;
                        sdram_dq_oe  <= 1'b1;
                        sdram_delay  <= 16'd3;
                        sdram_state  <= ST_WRITE_DONE;
                    end else sdram_delay <= sdram_delay - 1'b1;
                end
                ST_WRITE_DONE: begin
                    if (sdram_delay == 0) begin
                        if (test_addr_cnt == 3'd7) begin
                            test_addr_cnt <= 3'd0;
                            test_phase    <= 1'b1;
                            sdram_state   <= ST_READ_ACT;
                        end else begin
                            test_addr_cnt <= test_addr_cnt + 1'b1;
                            sdram_state   <= ST_WRITE_ACT;
                        end
                    end else sdram_delay <= sdram_delay - 1'b1;
                end
                ST_READ_ACT: begin
                    sdram_cmd   <= CMD_ACTIVE;
                    sdram_ba    <= 2'd0;
                    sdram_addr  <= {10'd0, test_addr_cnt};
                    sdram_delay <= 16'd2;
                    sdram_state <= ST_READ;
                end
                ST_READ: begin
                    if (sdram_delay == 0) begin
                        sdram_cmd   <= CMD_READ;
                        sdram_addr  <= 13'b0_0100_0000_0000;
                        sdram_delay <= 16'd3;
                        sdram_state <= ST_READ_WAIT;
                    end else sdram_delay <= sdram_delay - 1'b1;
                end
                ST_READ_WAIT: begin
                    if (sdram_delay == 0) sdram_state <= ST_VERIFY;
                    else                  sdram_delay <= sdram_delay - 1'b1;
                end
                ST_VERIFY: begin
                    if (sdram_dq_sample == test_pattern) sdram_state <= ST_NEXT_ADDR;
                    else                                 sdram_state <= ST_FAIL;
                end
                ST_NEXT_ADDR: begin
                    if (test_addr_cnt == 3'd7) sdram_state <= ST_PASS;
                    else begin
                        test_addr_cnt <= test_addr_cnt + 1'b1;
                        sdram_delay   <= 16'd2;
                        sdram_state   <= ST_READ_ACT;
                    end
                end
                ST_PASS: sdram_test_pass <= 1'b1;
                ST_FAIL: sdram_test_fail <= 1'b1;
                default: sdram_state <= ST_INIT_WAIT;
            endcase
        end
    end

    assign GPIO_2 = sdram_test_pass;

    // =========================================================================
    // Apple II bus status
    // =========================================================================
    assign GPIO_3 = ~nI_O_SELECT | ~nI_O_STROBE;
    assign GPIO_4 = ~nDEVICE_SELECT;

    // =========================================================================
    // Device Select register file ($C0C0-$C0CF, slot 4)
    //
    // This is the R/W inverter (U7) + data transceiver (U12) loopback.
    // From the Apple II monitor:
    //   C0C0:01   ...   C0C0   → must read back 01
    //   C0C0:02   ...   C0C0   → must read back 02   (walk bits through D0-D7)
    // Failure = U7 bad, wrong DIR polarity, or bad level shifter line.
    // =========================================================================
    reg [7:0] slot_reg [0:15];
    integer i;
    initial for (i = 0; i < 16; i = i + 1) slot_reg[i] = 8'hFF;

    wire device_selected = ~nDEVICE_SELECT;
    reg  write_pending = 1'b0;

    always @(posedge clk) begin
        if (device_selected && !R_nW) begin
            slot_reg[apple_addr[3:0]] <= apple_data_in;
            write_pending <= 1'b1;
        end else begin
            write_pending <= 1'b0;
        end
    end

    wire [7:0] read_data  = slot_reg[apple_addr[3:0]];
    wire       data_drive = R_nW;  // drive bus when CPU is reading from us

    assign D0 = data_drive ? read_data[0] : 1'bZ;
    assign D1 = data_drive ? read_data[1] : 1'bZ;
    assign D2 = data_drive ? read_data[2] : 1'bZ;
    assign D3 = data_drive ? read_data[3] : 1'bZ;
    assign D4 = data_drive ? read_data[4] : 1'bZ;
    assign D5 = data_drive ? read_data[5] : 1'bZ;
    assign D6 = data_drive ? read_data[6] : 1'bZ;
    assign D7 = data_drive ? read_data[7] : 1'bZ;

    // Pass daisy chain through
    assign DMA_IN = DMA_OUT;
    assign INT_IN = INT_OUT;

    // Passive control lines
    assign nIRQ = 1'bZ;
    assign nNMI = 1'bZ;
    assign nINH = 1'bZ;
    assign nDMA = 1'bZ;
    assign nRES = 1'b1;  // OPENDRAIN: always released

    // =========================================================================
    // Data bus transceiver OE (Rev 2)
    // Active-low. Enable whenever our slot is addressed so U12 passes the data
    // bus through. Disabled otherwise to stay off shared-bus cycles.
    // =========================================================================
    wire slot_active = ~nDEVICE_SELECT | ~nI_O_SELECT | ~nI_O_STROBE;
    assign DATA_OE = ~slot_active;

    // =========================================================================
    // Walking 1s on GPIO_9..GPIO_20 (12 pins, shift every 0.5s)
    // =========================================================================
    reg [23:0] gpio_walk_cnt     = 24'd0;
    reg [11:0] gpio_walk_pattern = 12'b0000_0000_0001;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gpio_walk_cnt     <= 24'd0;
            gpio_walk_pattern <= 12'b0000_0000_0001;
        end else begin
            if (gpio_walk_cnt == 24'd12_500_000) begin  // 0.5s @ 25MHz
                gpio_walk_cnt     <= 24'd0;
                gpio_walk_pattern <= {gpio_walk_pattern[10:0], gpio_walk_pattern[11]};
            end else begin
                gpio_walk_cnt <= gpio_walk_cnt + 1'b1;
            end
        end
    end

    // =========================================================================
    // GPIO assignments
    // =========================================================================
    assign GPIO_5  = R_nW;               // read cycle
    assign GPIO_6  = write_pending;      // write captured
    assign GPIO_7  = apple_data_in[0];   // D0 raw
    assign GPIO_8  = nRES_READ;          // Rev 2: Apple II reset line (HIGH = not in reset)

    assign GPIO_9  = gpio_walk_pattern[0];
    assign GPIO_10 = gpio_walk_pattern[1];
    assign GPIO_11 = gpio_walk_pattern[2];
    assign GPIO_12 = gpio_walk_pattern[3];
    assign GPIO_13 = gpio_walk_pattern[4];
    assign GPIO_14 = gpio_walk_pattern[5];
    assign GPIO_15 = gpio_walk_pattern[6];
    assign GPIO_16 = gpio_walk_pattern[7];
    assign GPIO_17 = gpio_walk_pattern[8];
    assign GPIO_18 = gpio_walk_pattern[9];
    assign GPIO_19 = gpio_walk_pattern[10];
    assign GPIO_20 = gpio_walk_pattern[11];

endmodule
