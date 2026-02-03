// =============================================================================
// Bus Interface for Logic Hamr
// =============================================================================
//
// Handles the Apple II register interface:
// - Synchronizes PHI0 clock
// - Decodes register addresses $C0C0-$C0CF
// - Maintains register state
// - Issues commands to other modules
//
// Register map is identical to v1 for software compatibility.
//
// =============================================================================

module bus_interface (
    input  wire        clk,             // 25 MHz system clock
    input  wire        rst_n,           // Active-low reset

    // Apple II bus signals
    input  wire        phi0,            // Apple II phase 0 clock
    input  wire [3:0]  addr,            // Lower 4 bits of address (offset from $C0C0)
    input  wire [7:0]  data_in,         // Data from Apple II (directly connected to pins)
    output wire [7:0]  data_out,        // Data to Apple II
    output reg         data_oe,         // Data output enable
    input  wire        ndevice_select,  // Active-low device select
    input  wire        r_nw,            // Read/!Write

    // Status inputs
    input  wire        init_done,       // SDRAM initialized
    input  wire        busy,            // Operation in progress
    input  wire        armed,           // Capture engine armed
    input  wire        captured,        // Capture complete
    input  wire        regen_done,      // Regeneration complete
    input  wire [7:0]  read_data,       // Data from SDRAM read

    // Debug inputs
    input  wire [2:0]  dbg_cap_state,   // Capture engine state
    input  wire [3:0]  dbg_bram_count,  // Pre-trigger buffer count
    input  wire        dbg_trigger_edge,// Trigger edge detected

    // Configuration outputs
    output reg  [2:0]  reg_channel,     // Channel select (0-7)
    output reg  [5:0]  reg_addr,        // Byte address (0-37)
    output reg  [2:0]  reg_trig_ch,     // Trigger channel (0-7)
    output reg         reg_trig_mode,   // 0=rising, 1=falling
    output reg  [1:0]  reg_window,      // Window preset (0-3)
    output reg         reg_debug_en,    // Debug pattern enable

    // Command outputs (active for one clock)
    output reg         cmd_read,        // Issue display buffer read
    output reg         cmd_regen,       // Start regeneration
    output reg         cmd_arm,         // ARM capture engine
    output reg         cmd_reset,       // Soft reset

    // Stretch factor (from window preset lookup)
    output reg  [7:0]  reg_stretch
);

    // =========================================================================
    // Register Addresses
    // =========================================================================

    localparam [3:0]
        REG_CHANNEL   = 4'h0,   // R/W - Channel select (0-7)
        REG_ADDR      = 4'h1,   // R/W - Byte address (0-37)
        REG_DATA      = 4'h2,   // R   - Read result
        REG_CMD       = 4'h3,   // W   - Command register
        REG_STATUS    = 4'h4,   // R   - Status bits
        REG_STRETCH   = 4'h5,   // R   - Stretch factor
        REG_TRIG_CH   = 4'h6,   // R/W - Trigger channel
        REG_TRIG_MODE = 4'h7,   // R/W - Trigger mode
        REG_WINDOW    = 4'h8,   // R/W - Window preset
        REG_ARM       = 4'h9,   // W   - ARM command
        REG_DEBUG_EN  = 4'hA,   // R/W - Debug enable
        REG_CAP_DBG   = 4'hB;   // R   - Capture debug {edge, bram_count[3:0], state[2:0]}

    // Command values
    localparam [7:0]
        CMD_READ   = 8'h02,
        CMD_REGEN  = 8'h10,
        CMD_RESET  = 8'hFF;

    // =========================================================================
    // PHI0, Device Select, R/W, and Data Synchronization
    // =========================================================================
    // ALL bus signals must be synchronized through matching pipelines so they
    // align on the same clock cycle when we detect phi0_falling.
    // When phi0_sync[2:1] == 2'b10, we're detecting PHI0 was HIGH 2 clocks ago.
    // So we check [2] of other sync registers to see their state at that time.

    reg [2:0] phi0_sync;
    reg [2:0] ndevsel_sync;
    reg [2:0] r_nw_sync;
    reg [7:0] data_sync_0, data_sync_1, data_sync_2;  // 3-stage data pipeline
    reg [3:0] addr_sync_0, addr_sync_1, addr_sync_2;  // 3-stage addr pipeline

    wire phi0_rising  = (phi0_sync[2:1] == 2'b01);
    wire phi0_falling = (phi0_sync[2:1] == 2'b10);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phi0_sync <= 3'b000;
            ndevsel_sync <= 3'b111;  // Default to not selected (high)
            r_nw_sync <= 3'b111;     // Default to read (high)
            data_sync_0 <= 8'h00;
            data_sync_1 <= 8'h00;
            data_sync_2 <= 8'h00;
            addr_sync_0 <= 4'h0;
            addr_sync_1 <= 4'h0;
            addr_sync_2 <= 4'h0;
        end else begin
            phi0_sync <= {phi0_sync[1:0], phi0};
            ndevsel_sync <= {ndevsel_sync[1:0], ndevice_select};
            r_nw_sync <= {r_nw_sync[1:0], r_nw};
            // Synchronize data and address through same pipeline
            data_sync_0 <= data_in;
            data_sync_1 <= data_sync_0;
            data_sync_2 <= data_sync_1;
            addr_sync_0 <= addr;
            addr_sync_1 <= addr_sync_0;
            addr_sync_2 <= addr_sync_1;
        end
    end

    // =========================================================================
    // Bus Transaction Detection
    // =========================================================================

    reg [3:0]  latched_addr;
    reg [7:0]  latched_wdata;
    reg        latched_r_nw;
    reg        device_select_sync;
    reg        device_select_prev;

    // CRITICAL FIX: Use synchronized ndevice_select from 2 clocks ago
    // This matches the timing of phi0_falling detection for WRITE path
    wire bus_active = !ndevsel_sync[2];

    // For READ path: use current (unsynchronized) signals for immediate response
    // The Apple II only samples data on PHI0 rising edge, so driving early is fine
    wire bus_read_now = !ndevice_select && r_nw;

    // Latch address and data on PHI0 falling edge (when bus is stable)
    // CRITICAL: Use synchronized versions of ALL bus signals from 2 clocks ago
    // to match phi0_falling timing (which detects PHI0 was high 2 clocks ago)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            latched_addr <= 4'h0;
            latched_wdata <= 8'h00;
            latched_r_nw <= 1'b1;
            device_select_sync <= 1'b0;
            device_select_prev <= 1'b0;
        end else begin
            device_select_prev <= device_select_sync;

            if (phi0_falling && bus_active) begin
                latched_addr <= addr_sync_2;   // Address from 2 clocks ago
                latched_wdata <= data_sync_2;  // Data from 2 clocks ago
                latched_r_nw <= r_nw_sync[2];  // R/W from 2 clocks ago
                device_select_sync <= 1'b1;
            end else if (phi0_rising) begin
                device_select_sync <= 1'b0;
            end
        end
    end

    // Write strobe: active for one clock when write is latched
    wire write_strobe = device_select_sync && !device_select_prev && !latched_r_nw;


    // =========================================================================
    // Read Data Mux
    // =========================================================================

    reg [7:0] read_data_mux;

    // Status register bits
    wire [7:0] status_reg = {4'b0000, captured, armed, regen_done & init_done, busy};

    // Use current address directly for reads - fast response path
    // The Apple II holds address valid during entire bus cycle
    always @(*) begin
        case (addr)  // Use current address for immediate response
            REG_CHANNEL:   read_data_mux = {5'b0, reg_channel};
            REG_ADDR:      read_data_mux = {2'b0, reg_addr};
            REG_DATA:      read_data_mux = read_data;
            REG_CMD:       read_data_mux = 8'h00;  // Write-only
            REG_STATUS:    read_data_mux = status_reg;
            REG_STRETCH:   read_data_mux = reg_stretch;
            REG_TRIG_CH:   read_data_mux = {5'b0, reg_trig_ch};
            REG_TRIG_MODE: read_data_mux = {7'b0, reg_trig_mode};
            REG_WINDOW:    read_data_mux = {6'b0, reg_window};
            REG_ARM:       read_data_mux = 8'h00;  // Write-only
            REG_DEBUG_EN:  read_data_mux = {7'b0, reg_debug_en};
            REG_CAP_DBG:   read_data_mux = {dbg_trigger_edge, dbg_bram_count, dbg_cap_state};
            default:       read_data_mux = 8'h00;
        endcase
    end

    // Drive data bus during reads - directly from input signals for fast response
    // No synchronization needed here since Apple II samples on PHI0 rising edge
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_oe <= 1'b0;
        end else begin
            // Drive whenever device selected for read (regardless of PHI0 phase)
            data_oe <= bus_read_now;
        end
    end

    assign data_out = read_data_mux;

    // =========================================================================
    // Window Preset Lookup
    // =========================================================================

    always @(*) begin
        case (reg_window)
            2'd0: reg_stretch = 8'd7;   // Preset 0: stretch=7
            2'd1: reg_stretch = 8'd3;   // Preset 1: stretch=3
            2'd2: reg_stretch = 8'd2;   // Preset 2: stretch=2
            2'd3: reg_stretch = 8'd1;   // Preset 3: stretch=1
        endcase
    end

    // =========================================================================
    // Register Write Handling
    // =========================================================================

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_channel <= 3'd0;
            reg_addr <= 6'd0;
            reg_trig_ch <= 3'd0;
            reg_trig_mode <= 1'b0;
            reg_window <= 2'd1;  // Default to preset 1
            reg_debug_en <= 1'b0;
            cmd_read <= 1'b0;
            cmd_regen <= 1'b0;
            cmd_arm <= 1'b0;
            cmd_reset <= 1'b0;
        end else begin
            // Clear command pulses
            cmd_read <= 1'b0;
            cmd_regen <= 1'b0;
            cmd_arm <= 1'b0;
            cmd_reset <= 1'b0;

            if (write_strobe) begin
                case (latched_addr)
                    REG_CHANNEL:   reg_channel <= latched_wdata[2:0];
                    REG_ADDR:      reg_addr <= latched_wdata[5:0];
                    REG_TRIG_CH:   reg_trig_ch <= latched_wdata[2:0];
                    REG_TRIG_MODE: reg_trig_mode <= latched_wdata[0];
                    REG_WINDOW:    reg_window <= latched_wdata[1:0];
                    REG_DEBUG_EN:  reg_debug_en <= latched_wdata[0];

                    REG_CMD: begin
                        case (latched_wdata)
                            CMD_READ:  cmd_read <= 1'b1;
                            CMD_REGEN: cmd_regen <= 1'b1;
                            CMD_RESET: cmd_reset <= 1'b1;
                        endcase
                    end

                    REG_ARM: cmd_arm <= 1'b1;
                endcase
            end
        end
    end

    // =========================================================================
    // Initial Values (for simulation when rst_n is tied high)
    // =========================================================================

    initial begin
        phi0_sync = 3'b000;
        ndevsel_sync = 3'b111;  // Not selected
        r_nw_sync = 3'b111;     // Default to read
        data_sync_0 = 8'h00;
        data_sync_1 = 8'h00;
        data_sync_2 = 8'h00;
        addr_sync_0 = 4'h0;
        addr_sync_1 = 4'h0;
        addr_sync_2 = 4'h0;
        latched_addr = 4'h0;
        latched_wdata = 8'h00;
        latched_r_nw = 1'b1;
        device_select_sync = 1'b0;
        device_select_prev = 1'b0;
        data_oe = 1'b0;
        reg_channel = 3'd0;
        reg_addr = 6'd0;
        reg_trig_ch = 3'd0;
        reg_trig_mode = 1'b0;
        reg_window = 2'd1;
        reg_debug_en = 1'b0;
        cmd_read = 1'b0;
        cmd_regen = 1'b0;
        cmd_arm = 1'b0;
        cmd_reset = 1'b0;
    end

endmodule
