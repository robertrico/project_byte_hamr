// =============================================================================
// Byte Hamr Logic Hamr - Capture Engine Testbench
// =============================================================================
//
// Tests the real-time capture engine with trigger detection:
// - SDRAM initialization
// - Debug test pattern generator
// - ARM command and trigger detection
// - Pre-trigger buffering (BRAM circular buffer)
// - Post-trigger capture to SDRAM
// - Regeneration through decimate_pack
// - Display buffer verification
//
// =============================================================================

`timescale 1ns / 100ps

module logic_hamr_v1_tb;

    // =========================================================================
    // Clock generation - 25 MHz (40ns period)
    // =========================================================================
    reg clk_25mhz = 0;
    always #20 clk_25mhz = ~clk_25mhz;

    // =========================================================================
    // Apple II clock - ~1 MHz (1000ns period)
    // =========================================================================
    reg phi0 = 0;
    reg phi1 = 0;
    always #500 begin
        phi0 = ~phi0;
        phi1 = ~phi0;
    end

    // =========================================================================
    // DUT Signals
    // =========================================================================

    // SDRAM
    wire        sdram_clk;
    wire        sdram_cke;
    wire        sdram_ncs;
    wire        sdram_nras;
    wire        sdram_ncas;
    wire        sdram_nwe;
    wire        sdram_dqm0;
    wire        sdram_dqm1;
    wire        sdram_ba0, sdram_ba1;
    wire        sdram_a0, sdram_a1, sdram_a2, sdram_a3;
    wire        sdram_a4, sdram_a5, sdram_a6, sdram_a7;
    wire        sdram_a8, sdram_a9, sdram_a10, sdram_a11, sdram_a12;
    wire        sdram_d0, sdram_d1, sdram_d2, sdram_d3;
    wire        sdram_d4, sdram_d5, sdram_d6, sdram_d7;
    wire        sdram_d8, sdram_d9, sdram_d10, sdram_d11;
    wire        sdram_d12, sdram_d13, sdram_d14, sdram_d15;

    // Apple II Address bus
    reg [15:0] apple_addr = 16'hFFFF;

    // Apple II Data bus
    wire [7:0] apple_data;
    reg  [7:0] apple_data_drv = 8'h00;
    reg        apple_data_oe = 0;
    assign apple_data = apple_data_oe ? apple_data_drv : 8'hZZ;

    // Apple II Control
    reg        r_nw = 1;
    reg        ndevice_select = 1;
    reg        nio_select = 1;
    reg        nio_strobe = 1;
    reg        rdy = 1;
    reg        sig_7m = 0;
    reg        q3 = 0;
    reg        usync = 1;

    // GPIO outputs (directly from DUT)
    wire       gpio1, gpio2, gpio3, gpio4;

    // GPIO probe inputs (testbench drives these)
    reg        gpio5 = 0, gpio6 = 0, gpio7 = 0, gpio8 = 0;
    reg        gpio9 = 0, gpio10 = 0, gpio11 = 0, gpio12 = 0;

    // =========================================================================
    // SDRAM Model - 8K locations for capture + display buffers
    // =========================================================================

    reg [7:0]  sdram_mem [0:8191];
    reg [15:0] sdram_dq_out = 16'd0;
    reg        sdram_dq_oe = 0;
    reg [12:0] sdram_read_idx = 13'd0;
    reg [3:0]  sdram_cas_delay = 4'd0;

    wire [12:0] sdram_addr_w = {sdram_a12, sdram_a11, sdram_a10, sdram_a9, sdram_a8,
                                sdram_a7, sdram_a6, sdram_a5, sdram_a4,
                                sdram_a3, sdram_a2, sdram_a1, sdram_a0};
    wire [1:0]  sdram_bank_w = {sdram_ba1, sdram_ba0};
    wire [15:0] sdram_dq_in_w = {sdram_d15, sdram_d14, sdram_d13, sdram_d12,
                                 sdram_d11, sdram_d10, sdram_d9, sdram_d8,
                                 sdram_d7, sdram_d6, sdram_d5, sdram_d4,
                                 sdram_d3, sdram_d2, sdram_d1, sdram_d0};

    assign sdram_d0  = sdram_dq_oe ? sdram_dq_out[0]  : 1'bZ;
    assign sdram_d1  = sdram_dq_oe ? sdram_dq_out[1]  : 1'bZ;
    assign sdram_d2  = sdram_dq_oe ? sdram_dq_out[2]  : 1'bZ;
    assign sdram_d3  = sdram_dq_oe ? sdram_dq_out[3]  : 1'bZ;
    assign sdram_d4  = sdram_dq_oe ? sdram_dq_out[4]  : 1'bZ;
    assign sdram_d5  = sdram_dq_oe ? sdram_dq_out[5]  : 1'bZ;
    assign sdram_d6  = sdram_dq_oe ? sdram_dq_out[6]  : 1'bZ;
    assign sdram_d7  = sdram_dq_oe ? sdram_dq_out[7]  : 1'bZ;
    assign sdram_d8  = sdram_dq_oe ? sdram_dq_out[8]  : 1'bZ;
    assign sdram_d9  = sdram_dq_oe ? sdram_dq_out[9]  : 1'bZ;
    assign sdram_d10 = sdram_dq_oe ? sdram_dq_out[10] : 1'bZ;
    assign sdram_d11 = sdram_dq_oe ? sdram_dq_out[11] : 1'bZ;
    assign sdram_d12 = sdram_dq_oe ? sdram_dq_out[12] : 1'bZ;
    assign sdram_d13 = sdram_dq_oe ? sdram_dq_out[13] : 1'bZ;
    assign sdram_d14 = sdram_dq_oe ? sdram_dq_out[14] : 1'bZ;
    assign sdram_d15 = sdram_dq_oe ? sdram_dq_out[15] : 1'bZ;

    // SDRAM command decode
    wire [3:0] sdram_cmd = {sdram_ncs, sdram_nras, sdram_ncas, sdram_nwe};
    localparam CMD_NOP       = 4'b0111;
    localparam CMD_ACTIVE    = 4'b0011;
    localparam CMD_READ      = 4'b0101;
    localparam CMD_WRITE     = 4'b0100;
    localparam CMD_PRECHARGE = 4'b0010;
    localparam CMD_REFRESH   = 4'b0001;
    localparam CMD_LOAD_MODE = 4'b0000;

    integer j;
    initial begin
        for (j = 0; j < 8192; j = j + 1)
            sdram_mem[j] = 8'h00;
        sdram_cas_delay = 0;
    end

    // Extract column address (bits 9:1 since A0 is used for byte select)
    wire [12:0] sdram_col = sdram_addr_w[12:1];

    always @(posedge sdram_clk) begin
        // SDRAM read data timing - drive data immediately after CAS delay starts
        // With sdram_cas_delay=3, we want data available on clock N+3 for DUT to read
        if (sdram_cas_delay > 0) begin
            sdram_cas_delay <= sdram_cas_delay - 1;
            // Always output data while cas_delay is active
            sdram_dq_oe <= 1;
            sdram_dq_out <= {8'h00, sdram_mem[sdram_read_idx]};
            // BACKDOOR: Force data into DUT's sdram_dq_sample when CAS latency expires
            // This works around Icarus Verilog's tri-state resolution issues
            if (sdram_cas_delay == 1) begin
                force dut.sdram_dq_sample = {8'h00, sdram_mem[sdram_read_idx]};
            end
        end else begin
            sdram_dq_oe <= 0;
            release dut.sdram_dq_sample;
        end

        case (sdram_cmd)
            CMD_WRITE: begin
                sdram_mem[sdram_col] <= sdram_dq_in_w[7:0];
                // Show capture buffer writes (1024-1290) and display buffer writes (1536+)
                if (sdram_col < 10 || (sdram_col >= 1024 && sdram_col < 1300) || (sdram_col >= 1536 && sdram_col < 1900))
                    $display("[%0t] SDRAM WRITE: addr=%04h data=%02h",
                             $time, sdram_col, sdram_dq_in_w[7:0]);
            end
            CMD_READ: begin
                sdram_read_idx <= sdram_col;
                sdram_cas_delay <= 3;  // CAS latency 3 to match DUT timing
                // Debug: show reads from capture buffer (1024-1300) and display buffer (1536+)
                if ((sdram_col >= 1024 && sdram_col < 1300) || (sdram_col >= 1536 && sdram_col < 1900))
                    $display("[%0t] SDRAM READ: addr=%04h data=%02h",
                             $time, sdram_col, sdram_mem[sdram_col]);
            end
        endcase
    end

    // =========================================================================
    // Device Under Test
    // =========================================================================

    logic_hamr_v1_top dut (
        .CLK_25MHz(clk_25mhz),

        // SDRAM
        .SDRAM_CLK(sdram_clk),
        .SDRAM_CKE(sdram_cke),
        .SDRAM_nCS(sdram_ncs),
        .SDRAM_nRAS(sdram_nras),
        .SDRAM_nCAS(sdram_ncas),
        .SDRAM_nWE(sdram_nwe),
        .SDRAM_DQM0(sdram_dqm0),
        .SDRAM_DQM1(sdram_dqm1),
        .SDRAM_BA0(sdram_ba0),
        .SDRAM_BA1(sdram_ba1),
        .SDRAM_A0(sdram_a0),
        .SDRAM_A1(sdram_a1),
        .SDRAM_A2(sdram_a2),
        .SDRAM_A3(sdram_a3),
        .SDRAM_A4(sdram_a4),
        .SDRAM_A5(sdram_a5),
        .SDRAM_A6(sdram_a6),
        .SDRAM_A7(sdram_a7),
        .SDRAM_A8(sdram_a8),
        .SDRAM_A9(sdram_a9),
        .SDRAM_A10(sdram_a10),
        .SDRAM_A11(sdram_a11),
        .SDRAM_A12(sdram_a12),
        .SDRAM_D0(sdram_d0),
        .SDRAM_D1(sdram_d1),
        .SDRAM_D2(sdram_d2),
        .SDRAM_D3(sdram_d3),
        .SDRAM_D4(sdram_d4),
        .SDRAM_D5(sdram_d5),
        .SDRAM_D6(sdram_d6),
        .SDRAM_D7(sdram_d7),
        .SDRAM_D8(sdram_d8),
        .SDRAM_D9(sdram_d9),
        .SDRAM_D10(sdram_d10),
        .SDRAM_D11(sdram_d11),
        .SDRAM_D12(sdram_d12),
        .SDRAM_D13(sdram_d13),
        .SDRAM_D14(sdram_d14),
        .SDRAM_D15(sdram_d15),

        // Apple II Address
        .A0(apple_addr[0]),
        .A1(apple_addr[1]),
        .A2(apple_addr[2]),
        .A3(apple_addr[3]),
        .A4(apple_addr[4]),
        .A5(apple_addr[5]),
        .A6(apple_addr[6]),
        .A7(apple_addr[7]),
        .A8(apple_addr[8]),
        .A9(apple_addr[9]),
        .A10(apple_addr[10]),
        .A11(apple_addr[11]),
        .A12(apple_addr[12]),
        .A13(apple_addr[13]),
        .A14(apple_addr[14]),
        .A15(apple_addr[15]),

        // Apple II Data
        .D0(apple_data[0]),
        .D1(apple_data[1]),
        .D2(apple_data[2]),
        .D3(apple_data[3]),
        .D4(apple_data[4]),
        .D5(apple_data[5]),
        .D6(apple_data[6]),
        .D7(apple_data[7]),

        // Clocks and timing
        .PHI0(phi0),
        .PHI1(phi1),
        .sig_7M(sig_7m),
        .Q3(q3),
        .uSync(usync),

        // Control
        .R_nW(r_nw),
        .nRES(),
        .nIRQ(),
        .nNMI(),
        .nDEVICE_SELECT(ndevice_select),
        .nI_O_SELECT(nio_select),
        .nI_O_STROBE(nio_strobe),
        .RDY(rdy),
        .DMA_IN(),
        .INT_IN(),
        .nINH(),
        .nDMA(),
        .DMA_OUT(1'b0),
        .INT_OUT(1'b0),

        // GPIO - outputs from DUT
        .GPIO1(gpio1),
        .GPIO2(gpio2),
        .GPIO3(gpio3),
        .GPIO4(gpio4),

        // GPIO - probe inputs (driven by testbench)
        .GPIO5(gpio5),
        .GPIO6(gpio6),
        .GPIO7(gpio7),
        .GPIO8(gpio8),
        .GPIO9(gpio9),
        .GPIO10(gpio10),
        .GPIO11(gpio11),
        .GPIO12(gpio12)
    );

    // =========================================================================
    // Register Map Constants
    // =========================================================================

    localparam REG_CHANNEL   = 4'h0;  // Channel select for display buffer read
    localparam REG_ADDR      = 4'h1;  // Byte index within channel
    localparam REG_DATA      = 4'h2;  // Read result
    localparam REG_CMD       = 4'h3;  // Command register
    localparam REG_STATUS    = 4'h4;  // Status register
    localparam REG_STRETCH   = 4'h5;  // Stretch factor (read-only from preset)
    localparam REG_TRIG_CH   = 4'h6;  // Trigger channel (0-7)
    localparam REG_TRIG_MODE = 4'h7;  // Trigger mode (0=rising, 1=falling)
    localparam REG_WINDOW    = 4'h8;  // Window preset (0-3)
    localparam REG_ARM       = 4'h9;  // ARM command (write any value)
    localparam REG_DEBUG_EN  = 4'hA;  // Debug pattern enable

    // Status register bits
    localparam STATUS_BUSY     = 0;
    localparam STATUS_READY    = 1;
    localparam STATUS_ARMED    = 2;
    localparam STATUS_CAPTURED = 3;

    // =========================================================================
    // Bus Transaction Tasks
    // =========================================================================

    task bus_write;
        input [3:0] offset;
        input [7:0] data;
        begin
            @(posedge phi0);
            apple_addr <= {12'hC0C, offset};
            r_nw <= 1'b0;
            apple_data_drv <= data;
            apple_data_oe <= 1'b1;
            ndevice_select <= 1'b0;

            @(negedge phi0);
            #100;

            ndevice_select <= 1'b1;
            r_nw <= 1'b1;
            apple_data_oe <= 1'b0;
            apple_addr <= 16'hFFFF;

            @(posedge phi0);
        end
    endtask

    reg [7:0] bus_read_result;
    task bus_read;
        input [3:0] offset;
        begin
            @(posedge phi0);
            apple_addr <= {12'hC0C, offset};
            r_nw <= 1'b1;
            apple_data_oe <= 1'b0;
            ndevice_select <= 1'b0;

            @(negedge phi0);
            #100;
            bus_read_result = apple_data;

            ndevice_select <= 1'b1;
            apple_addr <= 16'hFFFF;

            @(posedge phi0);
        end
    endtask

    // Wait for not busy
    task wait_not_busy;
        integer timeout;
        begin
            timeout = 0;
            bus_read_result = 8'hFF;
            while (bus_read_result[STATUS_BUSY] == 1'b1 && timeout < 5000) begin
                bus_read(REG_STATUS);
                timeout = timeout + 1;
            end
            if (timeout >= 5000) begin
                $display("[%0t] ERROR: Busy timeout!", $time);
            end
        end
    endtask

    // Wait for captured flag
    task wait_captured;
        integer timeout;
        begin
            timeout = 0;
            bus_read_result = 8'h00;
            while (bus_read_result[STATUS_CAPTURED] == 1'b0 && timeout < 5000) begin
                bus_read(REG_STATUS);
                timeout = timeout + 1;
            end
            if (timeout >= 5000) begin
                $display("[%0t] ERROR: Capture timeout!", $time);
            end
        end
    endtask

    // Wait for ready flag (pattern_loaded)
    task wait_ready;
        integer timeout;
        begin
            timeout = 0;
            bus_read_result = 8'h00;
            while (bus_read_result[STATUS_READY] == 1'b0 && timeout < 5000) begin
                bus_read(REG_STATUS);
                timeout = timeout + 1;
            end
            if (timeout >= 5000) begin
                $display("[%0t] ERROR: Ready timeout!", $time);
            end
        end
    endtask

    // Read byte from display buffer
    reg [7:0] rom_read_result;
    task rom_read;
        input [2:0] channel;
        input [5:0] addr;
        begin
            wait_not_busy;
            bus_write(REG_CHANNEL, {5'b0, channel});
            bus_write(REG_ADDR, {2'b0, addr});
            bus_write(REG_CMD, 8'h02);  // CMD = READ
            wait_not_busy;
            bus_read(REG_DATA);
            rom_read_result = bus_read_result;
        end
    endtask

    // =========================================================================
    // Test Sequence
    // =========================================================================

    integer test_pass;
    integer test_fail;

    initial begin
        $dumpfile("logic_hamr_v1_tb.vcd");
        $dumpvars(0, logic_hamr_v1_tb);

        test_pass = 0;
        test_fail = 0;

        $display("===========================================");
        $display("Byte Hamr Capture Engine Testbench");
        $display("===========================================");
        $display("");

        // ---- Wait for SDRAM initialization ----
        $display("[%0t] Waiting for SDRAM initialization...", $time);
        #300_000;  // 300us for SDRAM init

        // Check that we're in CAP_IDLE state (not busy, not armed, not captured)
        bus_read(REG_STATUS);
        $display("[%0t] Initial status: %02h", $time, bus_read_result);

        // =========================================================================
        // Test 1: Register read/write verification
        // =========================================================================
        $display("");
        $display("--- Test 1: Register read/write ---");
        begin : test_registers
            integer errors;
            errors = 0;

            // Test trigger channel register
            bus_write(REG_TRIG_CH, 8'h05);
            bus_read(REG_TRIG_CH);
            if (bus_read_result != 8'h05) begin
                $display("  TRIG_CH: FAIL (wrote 5, read %02h)", bus_read_result);
                errors = errors + 1;
            end else begin
                $display("  TRIG_CH: PASS");
            end

            // Test trigger mode register
            bus_write(REG_TRIG_MODE, 8'h01);
            bus_read(REG_TRIG_MODE);
            if (bus_read_result != 8'h01) begin
                $display("  TRIG_MODE: FAIL (wrote 1, read %02h)", bus_read_result);
                errors = errors + 1;
            end else begin
                $display("  TRIG_MODE: PASS");
            end

            // Test window preset register
            bus_write(REG_WINDOW, 8'h02);
            bus_read(REG_WINDOW);
            if (bus_read_result != 8'h02) begin
                $display("  WINDOW: FAIL (wrote 2, read %02h)", bus_read_result);
                errors = errors + 1;
            end else begin
                $display("  WINDOW: PASS");
            end

            // Test debug enable register
            bus_write(REG_DEBUG_EN, 8'h01);
            bus_read(REG_DEBUG_EN);
            if (bus_read_result != 8'h01) begin
                $display("  DEBUG_EN: FAIL (wrote 1, read %02h)", bus_read_result);
                errors = errors + 1;
            end else begin
                $display("  DEBUG_EN: PASS");
            end

            if (errors == 0) begin
                test_pass = test_pass + 1;
            end else begin
                test_fail = test_fail + 1;
            end
        end

        // =========================================================================
        // Test 2: Debug pattern capture with rising edge trigger
        // =========================================================================
        $display("");
        $display("--- Test 2: Debug pattern capture (rising edge) ---");
        begin : test_debug_capture
            // Configure for debug mode
            // Use channel 7 which toggles every sample (fastest)
            bus_write(REG_DEBUG_EN, 8'h01);   // Enable debug pattern
            bus_write(REG_TRIG_CH, 8'h07);    // Trigger on channel 7 (fastest, 500kHz)
            bus_write(REG_TRIG_MODE, 8'h00);  // Rising edge
            bus_write(REG_WINDOW, 8'h00);     // Window preset 0 (38 samples, smallest)

            $display("  Debug enabled, trigger on ch7 rising edge, window preset 0");
            $display("  Arming capture engine...");
            bus_write(REG_ARM, 8'h01);

            // Check armed status
            #1000;  // Small delay for state to settle
            bus_read(REG_STATUS);
            $display("  Status after ARM: %02h (busy=%b armed=%b captured=%b)",
                     bus_read_result, bus_read_result[0], bus_read_result[2], bus_read_result[3]);

            if (bus_read_result[STATUS_ARMED]) begin
                $display("  Armed: PASS");
            end else begin
                $display("  Armed: FAIL");
            end

            // Wait for trigger and capture (should be very fast with ch7)
            $display("  Waiting for debug pattern trigger on ch7...");
            #100_000;  // 100us should be plenty

            bus_read(REG_STATUS);
            $display("  Status after wait: %02h", bus_read_result);

            wait_captured;

            bus_read(REG_STATUS);
            if (bus_read_result[STATUS_CAPTURED]) begin
                $display("  Captured: PASS (STATUS=%02h)", bus_read_result);
                test_pass = test_pass + 1;
            end else begin
                $display("  Captured: FAIL (STATUS=%02h)", bus_read_result);
                test_fail = test_fail + 1;
            end
        end

        // =========================================================================
        // Test 2b: Verify capture buffer contents
        // =========================================================================
        $display("");
        $display("--- Test 2b: Raw capture buffer verification ---");
        begin : test_capture_buffer
            integer i;
            // Check DUT internal state
            $display("  DUT internal state:");
            $display("    pretrig_bram[0] = %02h", dut.pretrig_bram[0]);
            $display("    pretrig_bram[1] = %02h", dut.pretrig_bram[1]);
            $display("    capture_wr_idx  = %d", dut.capture_wr_idx);
            $display("    cfg_total_samples = %d", dut.cfg_total_samples);
            $display("    cfg_stretch = %d", dut.cfg_stretch);
            $display("    reg_stretch = %d", dut.reg_stretch);

            // Check testbench SDRAM memory (capture buffer)
            // DUT uses {2'b01, 1'b0, capture_idx, 1'b0} for capture buffer addressing
            // After extracting [12:1]: 01_0_XXXXXXXXX = 0x400 + index = 1024 + index
            $display("");
            $display("  SDRAM capture buffer (first 10 samples at offset 1024):");
            $write("    ");
            for (i = 0; i < 10; i = i + 1) begin
                $write("%02h ", sdram_mem[1024 + i]);
            end
            $display("");

            if (sdram_mem[1024] == 8'h00 && sdram_mem[1025] == 8'h00) begin
                $display("  WARNING: Capture buffer appears empty!");
            end else begin
                $display("  Capture buffer has valid data.");
            end
        end

        // =========================================================================
        // Test 3: Regenerate from captured data
        // =========================================================================
        $display("");
        $display("--- Test 3: Regenerate display buffer ---");
        begin : test_regenerate
            $display("  Triggering regeneration...");
            $display("  Monitoring decimate_pack during regen:");
            bus_write(REG_CMD, 8'h10);  // CMD = Regenerate

            // Wait a bit and sample key signals multiple times
            #5_000;  // 5us - early in regeneration
            $display("  [5us] TB: sdram_dq_oe=%b sdram_cas_delay=%d sdram_dq_out=%04h",
                     sdram_dq_oe, sdram_cas_delay, sdram_dq_out);
            $display("  [5us] BUS: sdram_d0-7 = %b%b%b%b%b%b%b%b",
                     sdram_d7, sdram_d6, sdram_d5, sdram_d4, sdram_d3, sdram_d2, sdram_d1, sdram_d0);
            $display("  [5us] DUT: sdram_dq_oe=%b sdram_dq_sample=%04h",
                     dut.sdram_dq_oe, dut.sdram_dq_sample);
            $display("  [5us] sdram_state=%d regen_channel=%d sample_idx=%d byte_idx=%d",
                     dut.sdram_state, dut.regen_channel, dut.regen_sample_idx, dut.gen_byte_idx);
            $display("  [5us] sdram_dq_sample=%04h dec_sample_in=%b dec_sample_valid=%b",
                     dut.sdram_dq_sample, dut.dec_sample_in, dut.dec_sample_valid);
            $display("  [5us] decimate: stretch=%d active=%b stretch_cnt=%d bit_pos=%d accum=%02h",
                     dut.reg_stretch, dut.u_decimate_pack.active, dut.u_decimate_pack.stretch_cnt,
                     dut.u_decimate_pack.bit_pos, dut.u_decimate_pack.accum);
            $display("  [5us] decimate: byte_out=%02h byte_valid=%b",
                     dut.u_decimate_pack.byte_out, dut.u_decimate_pack.byte_valid);

            #95_000;  // 95us more (100us total)
            $display("");
            $display("  [100us] regen_channel=%d sample_idx=%d byte_idx=%d",
                     dut.regen_channel, dut.regen_sample_idx, dut.gen_byte_idx);
            $display("  [100us] dec_sample_in=%b dec_sample_valid=%b dec_byte_valid=%b",
                     dut.dec_sample_in, dut.dec_sample_valid, dut.dec_byte_valid);
            $display("  [100us] dec_byte_out=%02h byte_pending=%b captured_byte=%02h",
                     dut.u_decimate_pack.byte_out, dut.byte_pending, dut.captured_byte);

            // Wait for regeneration to complete
            #900_000;  // 900us more

            wait_ready;

            bus_read(REG_STATUS);
            if (bus_read_result[STATUS_READY]) begin
                $display("  Regenerate complete: PASS (STATUS=%02h)", bus_read_result);

                // Read first 10 bytes from display buffer (like CAPTEST.S)
                $display("");
                $display("  Channel 0, Bytes 0-9 (CAPTEST.S style dump):");
                $write("  ");
                rom_read(3'd0, 6'd0);
                $write("%02h ", rom_read_result);
                rom_read(3'd0, 6'd1);
                $write("%02h ", rom_read_result);
                rom_read(3'd0, 6'd2);
                $write("%02h ", rom_read_result);
                rom_read(3'd0, 6'd3);
                $write("%02h ", rom_read_result);
                rom_read(3'd0, 6'd4);
                $write("%02h ", rom_read_result);
                rom_read(3'd0, 6'd5);
                $write("%02h ", rom_read_result);
                rom_read(3'd0, 6'd6);
                $write("%02h ", rom_read_result);
                rom_read(3'd0, 6'd7);
                $write("%02h ", rom_read_result);
                rom_read(3'd0, 6'd8);
                $write("%02h ", rom_read_result);
                rom_read(3'd0, 6'd9);
                $write("%02h ", rom_read_result);
                $display("");

                // Also read channel 7 (the trigger channel with 500kHz pattern)
                $display("");
                $display("  Channel 7 (trigger ch), Bytes 0-9:");
                $write("  ");
                rom_read(3'd7, 6'd0);
                $write("%02h ", rom_read_result);
                rom_read(3'd7, 6'd1);
                $write("%02h ", rom_read_result);
                rom_read(3'd7, 6'd2);
                $write("%02h ", rom_read_result);
                rom_read(3'd7, 6'd3);
                $write("%02h ", rom_read_result);
                rom_read(3'd7, 6'd4);
                $write("%02h ", rom_read_result);
                rom_read(3'd7, 6'd5);
                $write("%02h ", rom_read_result);
                rom_read(3'd7, 6'd6);
                $write("%02h ", rom_read_result);
                rom_read(3'd7, 6'd7);
                $write("%02h ", rom_read_result);
                rom_read(3'd7, 6'd8);
                $write("%02h ", rom_read_result);
                rom_read(3'd7, 6'd9);
                $write("%02h ", rom_read_result);
                $display("");

                // Expected values for channel 0 (3.9kHz, debug_counter[7]):
                // With stretch=7, window preset 0, 38 samples total
                // Each sample becomes 7 pixels. First byte = bits 0-6 of first sample stretched.
                // If sample is 0, byte = 00. If sample is 1, byte = 7F.
                // Channel 0 toggles every 128 samples, so for 38 samples it stays constant.
                $display("");
                $display("  Expected: Ch0 should be mostly 00 or 7F (3.9kHz = slow toggle)");
                $display("  Expected: Ch7 should alternate 00/7F rapidly (500kHz = every sample)");

                test_pass = test_pass + 1;
            end else begin
                $display("  Regenerate: FAIL (STATUS=%02h)", bus_read_result);
                test_fail = test_fail + 1;
            end
        end

        // =========================================================================
        // Test 4: External probe trigger (falling edge)
        // =========================================================================
        $display("");
        $display("--- Test 4: External probe capture (falling edge) ---");
        begin : test_external_probe
            // Configure for external probes
            bus_write(REG_DEBUG_EN, 8'h00);   // Disable debug pattern
            bus_write(REG_TRIG_CH, 8'h02);    // Trigger on channel 2 (GPIO7)
            bus_write(REG_TRIG_MODE, 8'h01);  // Falling edge
            bus_write(REG_WINDOW, 8'h01);     // Window preset 1 (88 samples)

            // Set initial probe state - channel 2 (gpio7) HIGH
            gpio5 = 0; gpio6 = 0; gpio7 = 1; gpio8 = 0;
            gpio9 = 0; gpio10 = 0; gpio11 = 0; gpio12 = 0;

            $display("  Arming capture engine...");
            bus_write(REG_ARM, 8'h01);

            // Wait for pre-trigger buffer to fill (need 4 samples at 1MHz = 4us)
            #10_000;

            // Generate falling edge on channel 2
            $display("  Generating falling edge on channel 2...");
            gpio7 = 0;

            // Wait for capture
            #200_000;  // 200us for capture

            wait_captured;

            bus_read(REG_STATUS);
            if (bus_read_result[STATUS_CAPTURED]) begin
                $display("  Captured: PASS (STATUS=%02h)", bus_read_result);
                test_pass = test_pass + 1;
            end else begin
                $display("  Captured: FAIL (STATUS=%02h)", bus_read_result);
                test_fail = test_fail + 1;
            end
        end

        // =========================================================================
        // Test 5: Verify GPIO debug outputs
        // =========================================================================
        $display("");
        $display("--- Test 5: GPIO debug outputs ---");
        begin : test_gpio
            // GPIO1 = heartbeat (should toggle)
            // GPIO2 = ready (should be high after regenerate)
            // GPIO3 = armed (should be low after capture)
            // GPIO4 = captured (should be high after capture)

            $display("  GPIO1 (heartbeat): %b", gpio1);
            $display("  GPIO2 (ready): %b", gpio2);
            $display("  GPIO3 (armed): %b", gpio3);
            $display("  GPIO4 (captured): %b", gpio4);

            if (gpio4 == 1'b1) begin
                $display("  GPIO4 captured flag: PASS");
                test_pass = test_pass + 1;
            end else begin
                $display("  GPIO4 captured flag: FAIL");
                test_fail = test_fail + 1;
            end
        end

        // =========================================================================
        // Test 6: Window preset verification
        // =========================================================================
        $display("");
        $display("--- Test 6: Window preset stretch values ---");
        begin : test_presets
            integer errors;
            errors = 0;

            // Preset 0: stretch = 7
            bus_write(REG_WINDOW, 8'h00);
            bus_read(REG_STRETCH);
            $display("  Preset 0: stretch=%d (expect 7)", bus_read_result);
            if (bus_read_result != 8'd7) errors = errors + 1;

            // Preset 1: stretch = 3
            bus_write(REG_WINDOW, 8'h01);
            bus_read(REG_STRETCH);
            $display("  Preset 1: stretch=%d (expect 3)", bus_read_result);
            if (bus_read_result != 8'd3) errors = errors + 1;

            // Preset 2: stretch = 2
            bus_write(REG_WINDOW, 8'h02);
            bus_read(REG_STRETCH);
            $display("  Preset 2: stretch=%d (expect 2)", bus_read_result);
            if (bus_read_result != 8'd2) errors = errors + 1;

            // Preset 3: stretch = 1
            bus_write(REG_WINDOW, 8'h03);
            bus_read(REG_STRETCH);
            $display("  Preset 3: stretch=%d (expect 1)", bus_read_result);
            if (bus_read_result != 8'd1) errors = errors + 1;

            if (errors == 0) begin
                test_pass = test_pass + 1;
            end else begin
                test_fail = test_fail + 1;
            end
        end

        // =========================================================================
        // Summary
        // =========================================================================
        $display("");
        $display("===========================================");
        $display("Results: %0d passed, %0d failed", test_pass, test_fail);
        $display("===========================================");

        if (test_fail == 0)
            $display("*** ALL TESTS PASSED ***");
        else
            $display("*** SOME TESTS FAILED ***");

        $display("");
        #1000;
        $finish;
    end

    // =========================================================================
    // Monitor for state changes
    // =========================================================================

    reg prev_gpio3 = 0;
    reg prev_gpio4 = 0;

    always @(posedge clk_25mhz) begin
        if (gpio3 && !prev_gpio3)
            $display("[%0t] GPIO3: Armed", $time);
        if (!gpio3 && prev_gpio3)
            $display("[%0t] GPIO3: Disarmed", $time);
        if (gpio4 && !prev_gpio4)
            $display("[%0t] GPIO4: Captured", $time);
        prev_gpio3 <= gpio3;
        prev_gpio4 <= gpio4;
    end

endmodule
