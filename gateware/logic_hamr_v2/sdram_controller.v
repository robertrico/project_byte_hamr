// =============================================================================
// SDRAM Controller for Logic Hamr v2
// =============================================================================
//
// Provides a simple SRAM-like interface to the SDRAM, abstracting all timing
// details including initialization, refresh, row activation, precharge, and
// CAS latency. Single point of contact with SDRAM - no other module touches
// SDRAM pins directly.
//
// Interface:
//   - Single request port (read or write)
//   - pause_refresh: Defer refresh during time-critical capture
//   - init_done: High when SDRAM is ready for use
//
// SDRAM: MT48LC16M16A2 at 25 MHz
//   - tRCD: 20ns (1 clock)
//   - tRP: 20ns (1 clock)
//   - tRFC: 66ns (2 clocks)
//   - CAS Latency: 2
//   - Burst Length: 1
//
// =============================================================================

module sdram_controller (
    input  wire        clk,           // 25 MHz system clock
    input  wire        rst_n,         // Active-low reset

    // Initialization status
    output reg         init_done,     // High when SDRAM is ready for use

    // Refresh control
    input  wire        pause_refresh, // Defer refresh while high

    // Request interface (single port)
    input  wire        req,           // Request (read or write)
    input  wire        req_write,     // 1 = write, 0 = read
    input  wire [12:0] req_addr,      // Linear byte address (13 bits = 8K locations)
    input  wire [7:0]  req_wdata,     // Write data (ignored for reads)
    output reg         req_ready,     // Ready to accept request
    output reg  [7:0]  req_rdata,     // Read data
    output reg         req_rdata_valid, // Read data valid (one cycle pulse)

    // SDRAM physical pins
    output wire        SDRAM_CLK,
    output reg         SDRAM_CKE,
    output wire        SDRAM_nCS,
    output wire        SDRAM_nRAS,
    output wire        SDRAM_nCAS,
    output wire        SDRAM_nWE,
    output wire        SDRAM_DQM0,
    output wire        SDRAM_DQM1,
    output wire        SDRAM_BA0,
    output wire        SDRAM_BA1,
    output reg  [12:0] SDRAM_A,
    inout  wire [15:0] SDRAM_DQ
);

    // =========================================================================
    // SDRAM Command Encoding: {nCS, nRAS, nCAS, nWE}
    // =========================================================================

    localparam [3:0] CMD_NOP       = 4'b0111;
    localparam [3:0] CMD_ACTIVE    = 4'b0011;
    localparam [3:0] CMD_READ      = 4'b0101;
    localparam [3:0] CMD_WRITE     = 4'b0100;
    localparam [3:0] CMD_PRECHARGE = 4'b0010;
    localparam [3:0] CMD_REFRESH   = 4'b0001;
    localparam [3:0] CMD_LOAD_MODE = 4'b0000;

    // =========================================================================
    // State Machine States
    // =========================================================================

    localparam [3:0]
        ST_INIT_WAIT   = 4'd0,   // Power-up wait (200us)
        ST_PRECHARGE   = 4'd1,   // Precharge all banks
        ST_REFRESH1    = 4'd2,   // First auto-refresh
        ST_REFRESH2    = 4'd3,   // Second auto-refresh
        ST_LOAD_MODE   = 4'd4,   // Load mode register
        ST_IDLE        = 4'd5,   // Ready for requests
        ST_ACTIVATE    = 4'd6,   // Activate row
        ST_READ_CMD    = 4'd7,   // Issue read command
        ST_READ_WAIT   = 4'd8,   // Wait for CAS latency
        ST_WRITE_CMD   = 4'd9,   // Issue write command
        ST_WRITE_WAIT  = 4'd10,  // Wait for tWR
        ST_REFRESH     = 4'd11,  // Periodic refresh
        ST_REFRESH_WAIT= 4'd12;  // Wait for refresh complete

    reg [3:0]  state;
    reg [15:0] delay_cnt;

    // =========================================================================
    // SDRAM Control Signals
    // =========================================================================

    reg [3:0]  sdram_cmd;
    reg [1:0]  sdram_ba;
    reg [15:0] sdram_dq_out;
    reg        sdram_dq_oe;

    // Assign command pins
    assign {SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} = sdram_cmd;
    assign SDRAM_CLK = clk;
    assign SDRAM_BA0 = sdram_ba[0];
    assign SDRAM_BA1 = sdram_ba[1];

    // DQM: Enable lower byte, disable upper byte (we only use 8-bit data)
    assign SDRAM_DQM0 = 1'b0;  // Lower byte enabled
    assign SDRAM_DQM1 = 1'b1;  // Upper byte masked

    // Bidirectional data bus
    assign SDRAM_DQ = sdram_dq_oe ? sdram_dq_out : 16'hZZZZ;

    // Sample input data
    reg [15:0] sdram_dq_sample;
    always @(posedge clk) begin
        sdram_dq_sample <= SDRAM_DQ;
    end

    // =========================================================================
    // Refresh Counter
    // =========================================================================

    // Refresh interval: 7.8us = 195 clocks at 25MHz
    localparam [7:0] REFRESH_INTERVAL = 8'd190;

    reg [7:0]  refresh_cnt;
    reg        refresh_needed;
    reg        refresh_pending;  // Tracks if refresh was needed but paused

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            refresh_cnt <= REFRESH_INTERVAL;
            refresh_needed <= 1'b0;
            refresh_pending <= 1'b0;
        end else if (state == ST_INIT_WAIT || state == ST_REFRESH) begin
            refresh_cnt <= REFRESH_INTERVAL;
            refresh_needed <= 1'b0;
            refresh_pending <= 1'b0;
        end else if (refresh_cnt == 0) begin
            if (pause_refresh) begin
                // Defer refresh, mark as pending
                refresh_pending <= 1'b1;
            end else begin
                refresh_needed <= 1'b1;
            end
        end else begin
            refresh_cnt <= refresh_cnt - 1'b1;
            // If pause_refresh drops and we have a pending refresh
            if (!pause_refresh && refresh_pending) begin
                refresh_needed <= 1'b1;
                refresh_pending <= 1'b0;
            end
        end
    end

    // =========================================================================
    // Request Latching
    // =========================================================================

    reg        latched_write;
    reg [12:0] latched_addr;
    reg [7:0]  latched_wdata;

    // =========================================================================
    // State Machine
    // =========================================================================

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_INIT_WAIT;
            delay_cnt <= 16'd5000;  // ~200us at 25MHz
            init_done <= 1'b0;
            req_ready <= 1'b0;
            req_rdata <= 8'h00;
            req_rdata_valid <= 1'b0;
            sdram_cmd <= CMD_NOP;
            sdram_ba <= 2'b00;
            SDRAM_A <= 13'd0;
            SDRAM_CKE <= 1'b0;
            sdram_dq_out <= 16'd0;
            sdram_dq_oe <= 1'b0;
            latched_write <= 1'b0;
            latched_addr <= 13'd0;
            latched_wdata <= 8'd0;
        end else begin
            // Default: NOP command, no output enable, clear valid
            sdram_cmd <= CMD_NOP;
            sdram_dq_oe <= 1'b0;
            req_rdata_valid <= 1'b0;

            case (state)
                // ---------------------------------------------------------
                // Initialization Sequence
                // ---------------------------------------------------------

                ST_INIT_WAIT: begin
                    SDRAM_CKE <= 1'b1;
                    req_ready <= 1'b0;
                    if (delay_cnt == 0) begin
                        state <= ST_PRECHARGE;
                    end else begin
                        delay_cnt <= delay_cnt - 1'b1;
                    end
                end

                ST_PRECHARGE: begin
                    sdram_cmd <= CMD_PRECHARGE;
                    SDRAM_A <= 13'b0_0100_0000_0000;  // A10=1: precharge all banks
                    delay_cnt <= 16'd2;  // tRP
                    state <= ST_REFRESH1;
                end

                ST_REFRESH1: begin
                    if (delay_cnt == 0) begin
                        sdram_cmd <= CMD_REFRESH;
                        delay_cnt <= 16'd8;  // tRFC
                        state <= ST_REFRESH2;
                    end else begin
                        delay_cnt <= delay_cnt - 1'b1;
                    end
                end

                ST_REFRESH2: begin
                    if (delay_cnt == 0) begin
                        sdram_cmd <= CMD_REFRESH;
                        delay_cnt <= 16'd8;  // tRFC
                        state <= ST_LOAD_MODE;
                    end else begin
                        delay_cnt <= delay_cnt - 1'b1;
                    end
                end

                ST_LOAD_MODE: begin
                    if (delay_cnt == 0) begin
                        sdram_cmd <= CMD_LOAD_MODE;
                        sdram_ba <= 2'b00;
                        // Mode register: CAS=2, burst=1, sequential
                        // A[2:0] = burst length (000 = 1)
                        // A[3] = burst type (0 = sequential)
                        // A[6:4] = CAS latency (010 = 2)
                        // A[8:7] = operating mode (00 = standard)
                        // A[9] = write burst mode (0 = programmed burst length)
                        SDRAM_A <= 13'b000_0_00_010_0_000;
                        delay_cnt <= 16'd2;  // tMRD
                        init_done <= 1'b1;
                        state <= ST_IDLE;
                    end else begin
                        delay_cnt <= delay_cnt - 1'b1;
                    end
                end

                // ---------------------------------------------------------
                // Idle - Ready for Requests
                // ---------------------------------------------------------

                ST_IDLE: begin
                    if (delay_cnt != 0) begin
                        delay_cnt <= delay_cnt - 1'b1;
                        req_ready <= 1'b0;
                    end else begin
                        req_ready <= 1'b1;

                        // Priority: new request > refresh (avoid dropping requests)
                        if (req && req_ready) begin
                            // Latch request
                            latched_write <= req_write;
                            latched_addr <= req_addr;
                            latched_wdata <= req_wdata;
                            req_ready <= 1'b0;

                            // Activate row (row address = upper bits)
                            sdram_cmd <= CMD_ACTIVE;
                            sdram_ba <= 2'b00;
                            SDRAM_A <= 13'd0;  // Row 0 (all our addresses fit in one row)
                            delay_cnt <= 16'd2;  // tRCD
                            state <= ST_ACTIVATE;
                        end else if (refresh_needed) begin
                            req_ready <= 1'b0;
                            sdram_cmd <= CMD_PRECHARGE;
                            SDRAM_A <= 13'b0_0100_0000_0000;  // A10=1: all banks
                            delay_cnt <= 16'd2;  // tRP
                            state <= ST_REFRESH;
                        end
                    end
                end

                // ---------------------------------------------------------
                // Activate Row
                // ---------------------------------------------------------

                ST_ACTIVATE: begin
                    if (delay_cnt == 0) begin
                        if (latched_write) begin
                            state <= ST_WRITE_CMD;
                        end else begin
                            state <= ST_READ_CMD;
                        end
                    end else begin
                        delay_cnt <= delay_cnt - 1'b1;
                    end
                end

                // ---------------------------------------------------------
                // Read Operation
                // ---------------------------------------------------------

                ST_READ_CMD: begin
                    sdram_cmd <= CMD_READ;
                    sdram_ba <= 2'b00;
                    // Column address with auto-precharge (A10=1)
                    // Format: {2'b01, addr[12:0], 1'b0} -> extract column from addr
                    SDRAM_A <= {2'b01, latched_addr[9:0], 1'b0};
                    delay_cnt <= 16'd3;  // CAS latency (match SDRAM model)
                    state <= ST_READ_WAIT;
                end

                ST_READ_WAIT: begin
                    if (delay_cnt == 0) begin
                        // Capture read data
                        req_rdata <= sdram_dq_sample[7:0];
                        req_rdata_valid <= 1'b1;
                        delay_cnt <= 16'd2;  // tRP after auto-precharge
                        state <= ST_IDLE;
                    end else begin
                        delay_cnt <= delay_cnt - 1'b1;
                    end
                end

                // ---------------------------------------------------------
                // Write Operation
                // ---------------------------------------------------------

                ST_WRITE_CMD: begin
                    sdram_cmd <= CMD_WRITE;
                    sdram_ba <= 2'b00;
                    // Column address with auto-precharge (A10=1)
                    SDRAM_A <= {2'b01, latched_addr[9:0], 1'b0};
                    sdram_dq_out <= {8'h00, latched_wdata};
                    sdram_dq_oe <= 1'b1;
                    delay_cnt <= 16'd4;  // tWR + tRP
                    state <= ST_WRITE_WAIT;
                end

                ST_WRITE_WAIT: begin
                    if (delay_cnt == 0) begin
                        state <= ST_IDLE;
                    end else begin
                        delay_cnt <= delay_cnt - 1'b1;
                    end
                end

                // ---------------------------------------------------------
                // Refresh Operation
                // ---------------------------------------------------------

                ST_REFRESH: begin
                    if (delay_cnt == 0) begin
                        sdram_cmd <= CMD_REFRESH;
                        delay_cnt <= 16'd8;  // tRFC
                        state <= ST_REFRESH_WAIT;
                    end else begin
                        delay_cnt <= delay_cnt - 1'b1;
                    end
                end

                ST_REFRESH_WAIT: begin
                    if (delay_cnt == 0) begin
                        state <= ST_IDLE;
                    end else begin
                        delay_cnt <= delay_cnt - 1'b1;
                    end
                end

                default: state <= ST_INIT_WAIT;
            endcase
        end
    end

    // =========================================================================
    // Initial Values (for simulation when rst_n is tied high)
    // =========================================================================

    initial begin
        state = ST_INIT_WAIT;
        delay_cnt = 16'd5000;
        init_done = 1'b0;
        req_ready = 1'b0;
        req_rdata = 8'h00;
        req_rdata_valid = 1'b0;
        sdram_cmd = CMD_NOP;
        sdram_ba = 2'b00;
        SDRAM_A = 13'd0;
        SDRAM_CKE = 1'b0;
        sdram_dq_out = 16'd0;
        sdram_dq_oe = 1'b0;
        latched_write = 1'b0;
        latched_addr = 13'd0;
        latched_wdata = 8'd0;
        refresh_cnt = REFRESH_INTERVAL;
        refresh_needed = 1'b0;
        refresh_pending = 1'b0;
    end

endmodule
