// =============================================================================
// Byte Hamr SDRAM Bank Switcher
// =============================================================================
//
// Minimal bank-switching test. The Apple II selects a bank (0-3), writes/reads
// individual bytes at addresses within that bank. Proves banks hold
// independent data.
//
// Register Map (offsets from $C0C0):
//   $00 BANK    R/W  Active bank (0-3, maps to SDRAM BA[1:0])
//   $01 ADDR    R/W  Address within bank (0-255)
//   $02 DATA    R/W  Write: byte to store. Read: last SDRAM read result
//   $03 CMD     W    $01=Write byte, $02=Read byte
//   $04 STATUS  R    [0]=busy [1]=init_done [2]=last_cmd (0=wr,1=rd)
//   $05-$0F     R/W  Scratch registers (loopback testing)
//
// SDRAM Address Mapping:
//   BA[1:0]     = BANK[1:0]
//   Row[12:0]   = 13'd0 (fixed)
//   Column[9:0] = {2'b00, ADDR[7:0]}
//
// Byte access: DQM0=0 (low byte enabled), DQM1=1 (high byte masked)
//   Writes: SDRAM_D[7:0] = DATA register
//   Reads:  DATA register = SDRAM_D[7:0]
//
// 4 banks x 256 locations = 1KB total addressable space.
//
// GPIO outputs:
//   GPIO1:  Heartbeat (250kHz)
//   GPIO2:  SDRAM init done
//   GPIO3:  bank_reg[0]
//   GPIO4:  bank_reg[1]
//   GPIO5:  SDRAM busy
//   GPIO6:  R_nW debug
//   GPIO7:  Command pending debug
//   GPIO8-12: Walking 1s pattern
//
// =============================================================================

module sdram_bank_top (
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
    // SDRAM Controller - Bank-Switched Byte Access
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
    localparam [3:0]
        ST_INIT_WAIT    = 4'd0,
        ST_PRECHARGE    = 4'd1,
        ST_REFRESH1     = 4'd2,
        ST_REFRESH2     = 4'd3,
        ST_LOAD_MODE    = 4'd4,
        ST_IDLE         = 4'd5,
        ST_ACTIVATE     = 4'd6,
        ST_WRITE_CMD    = 4'd7,
        ST_WRITE_RECOV  = 4'd8,
        ST_READ_CMD     = 4'd9,
        ST_READ_WAIT    = 4'd10,
        ST_READ_CAPTURE = 4'd11,
        ST_DONE         = 4'd12,
        ST_REFRESH      = 4'd13,
        ST_REFRESH_WAIT = 4'd14;

    // SDRAM hardware control
    reg [3:0]  sdram_state;
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
    reg        last_cmd_type;    // 0=write, 1=read

    // Command interface
    reg        cmd_pending;
    reg        cmd_is_read;

    // Latched command parameters
    reg [9:0]  latched_col;
    reg [1:0]  latched_bank;
    reg [7:0]  latched_write_data;

    // Refresh counter (8192 refreshes per 64ms @ 25MHz)
    localparam REFRESH_INTERVAL = 8'd190;
    reg [7:0]  refresh_cnt;
    reg        refresh_needed;

    // ---- Register file ----

    reg [7:0] reg_bank;     // $00: BANK
    reg [7:0] reg_addr;     // $01: ADDR
    reg [7:0] reg_data;     // $02: DATA (write staging / read result)

    // Scratch registers ($05-$0F)
    reg [7:0] slot_reg [0:15];

    // Write pending flag (for debug)
    reg write_pending;

    // Device select decode
    wire device_selected = ~nDEVICE_SELECT;

    // Initialize all registers. ECP5 FPGAs power-on to initial values.
    // iverilog also needs these for correct simulation.
    // Scratch regs init to $FF (Yosys tri-state optimization workaround).
    integer i;
    initial begin
        // Register file
        reg_bank = 8'hFF;
        reg_addr = 8'hFF;
        reg_data = 8'hFF;
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
        last_cmd_type   = 1'b0;

        // Command interface
        cmd_pending        = 1'b0;
        cmd_is_read        = 1'b0;
        latched_col        = 10'd0;
        latched_bank       = 2'd0;
        latched_write_data = 8'd0;
        write_pending      = 1'b0;

        // Refresh
        refresh_cnt    = REFRESH_INTERVAL;
        refresh_needed = 1'b0;
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

    // Byte-lane masking: only use low byte (D[7:0])
    assign SDRAM_DQM0 = 1'b0;  // Low byte enabled
    assign SDRAM_DQM1 = 1'b1;  // High byte masked

    // Bidirectional SDRAM data bus
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

    // Sample SDRAM data input (registered for timing)
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
    // Combined Register Write + SDRAM State Machine
    // =========================================================================

    always @(posedge clk) begin

        // =================================================================
        // Register Write Handling
        // =================================================================

        write_pending <= 1'b0;

        if (device_selected && !R_nW) begin
            write_pending <= 1'b1;
            case (apple_addr[3:0])
                4'h0: reg_bank <= apple_data_in;
                4'h1: reg_addr <= apple_data_in;
                4'h2: reg_data <= apple_data_in;
                4'h3: begin  // CMD register
                    if (!sdram_busy && sdram_init_done) begin
                        if (apple_data_in == 8'h01 || apple_data_in == 8'h02) begin
                            cmd_pending <= 1'b1;
                            cmd_is_read <= (apple_data_in == 8'h02);
                            sdram_busy <= 1'b1;
                            last_cmd_type <= (apple_data_in == 8'h02);
                            // Latch bank, column, and write data
                            latched_bank <= reg_bank[1:0];
                            latched_col  <= {2'b00, reg_addr};
                            latched_write_data <= reg_data;
                        end
                    end
                end
                4'h4: ;  // STATUS is read-only
                default: slot_reg[apple_addr[3:0]] <= apple_data_in;
            endcase
        end

        // =================================================================
        // SDRAM State Machine
        // =================================================================

        // Default: NOP, release data bus
        sdram_cmd <= CMD_NOP;
        sdram_dq_oe <= 1'b0;

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
                    // CAS latency 2, sequential, burst length 1
                    sdram_addr <= 13'b000_0_00_010_0_000;
                    sdram_delay <= 16'd2;
                    sdram_init_done <= 1'b1;
                    sdram_state <= ST_IDLE;
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
                sdram_ba <= latched_bank;
                sdram_addr <= 13'd0;  // Row always 0
                sdram_delay <= 16'd2;  // tRCD
                if (cmd_is_read) begin
                    sdram_state <= ST_READ_CMD;
                end else begin
                    sdram_state <= ST_WRITE_CMD;
                end
            end

            // ---- WRITE PATH ----

            ST_WRITE_CMD: begin
                if (sdram_delay == 0) begin
                    sdram_cmd <= CMD_WRITE;
                    sdram_addr <= {3'b010, latched_col};  // A10=1 auto-precharge
                    sdram_dq_out <= {8'h00, latched_write_data};  // Low byte only
                    sdram_dq_oe <= 1'b1;
                    sdram_delay <= 16'd3;  // tWR + tRP
                    sdram_state <= ST_WRITE_RECOV;
                end else begin
                    sdram_delay <= sdram_delay - 1'b1;
                end
            end

            ST_WRITE_RECOV: begin
                if (sdram_delay == 0) begin
                    sdram_state <= ST_DONE;
                end else begin
                    sdram_delay <= sdram_delay - 1'b1;
                end
            end

            // ---- READ PATH ----

            ST_READ_CMD: begin
                if (sdram_delay == 0) begin
                    sdram_cmd <= CMD_READ;
                    sdram_addr <= {3'b010, latched_col};  // A10=1 auto-precharge
                    sdram_delay <= 16'd2;  // CAS latency 2
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
                // Capture low byte only (DQM1 masks upper byte)
                reg_data <= sdram_dq_sample[7:0];
                sdram_delay <= 16'd2;  // tRP recovery
                sdram_state <= ST_DONE;
            end

            // ---- COMPLETION ----

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
                sdram_delay <= 16'd8;  // tRC
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

    wire [7:0] status_reg = {5'b0, last_cmd_type, sdram_init_done, sdram_busy};

    reg [7:0] read_data;
    always @(*) begin
        case (apple_addr[3:0])
            4'h0: read_data = reg_bank;
            4'h1: read_data = reg_addr;
            4'h2: read_data = reg_data;
            4'h3: read_data = 8'h00;       // CMD is write-only
            4'h4: read_data = status_reg;
            default: read_data = slot_reg[apple_addr[3:0]];
        endcase
    end

    // Drive Apple II data bus (OE gated externally by level shifter)
    wire data_drive = R_nW;

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

    assign GPIO2 = sdram_init_done;
    assign GPIO3 = reg_bank[0];
    assign GPIO4 = reg_bank[1];
    assign GPIO5 = sdram_busy;
    assign GPIO6 = R_nW;
    assign GPIO7 = cmd_pending;

    // Walking 1s pattern on GPIO8-12
    reg [23:0] gpio_walk_cnt = 24'd0;
    reg [7:0]  gpio_walk_pattern = 8'b00000001;

    always @(posedge clk) begin
        if (gpio_walk_cnt == 24'd12_500_000) begin  // 0.5s
            gpio_walk_cnt <= 24'd0;
            gpio_walk_pattern <= {gpio_walk_pattern[6:0], gpio_walk_pattern[7]};
        end else begin
            gpio_walk_cnt <= gpio_walk_cnt + 1'b1;
        end
    end

    assign GPIO8  = gpio_walk_pattern[3];
    assign GPIO9  = gpio_walk_pattern[4];
    assign GPIO10 = gpio_walk_pattern[5];
    assign GPIO11 = gpio_walk_pattern[6];
    assign GPIO12 = gpio_walk_pattern[7];

endmodule
