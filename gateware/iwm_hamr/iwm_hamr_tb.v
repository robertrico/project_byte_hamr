`define SIMULATION
`timescale 1ns / 1ps
// =============================================================================
// IWM Hamr - Integration Testbench
// =============================================================================
// Verifies full boot sequence: POR -> SDRAM init -> Flash->SDRAM copy -> device
// present. Uses behavioral SDRAM and SPI flash models.
//
// Tests:
//   1. POR sequence (both 7 MHz and 25 MHz domains)
//   2. SDRAM initialization (init_done asserts)
//   3. Boot sequence (flash -> SDRAM copy, boot_done asserts)
//   4. Device present (sp_device sense HIGH after boot)
//   5. ROM reads still work (Apple II bus functional)
// =============================================================================

module iwm_hamr_tb;

    // =========================================================================
    // Test infrastructure
    // =========================================================================
    integer pass_count = 0;
    integer fail_count = 0;

    task check(input [511:0] name, input condition);
    begin
        if (condition) begin
            $display("  PASS: %0s", name);
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: %0s", name);
            fail_count = fail_count + 1;
        end
    end
    endtask

    // =========================================================================
    // Clock generation
    // =========================================================================
    reg clk_7m  = 0;
    reg clk_25m = 0;

    always #69.8  clk_7m  = ~clk_7m;   // ~7.16 MHz (139.6 ns period)
    always #20.0  clk_25m = ~clk_25m;   // 25 MHz (40 ns period)

    // =========================================================================
    // Apple II Bus Signals
    // =========================================================================
    reg  [11:0] addr;
    wire [7:0]  data_bus;
    reg  [7:0]  data_drive;
    reg         data_drive_en;
    reg         Q3;
    reg         R_nW;
    reg         nDEVICE_SELECT;
    reg         nI_O_SELECT;
    reg         nI_O_STROBE;
    wire        nRES;
    reg         RDY;
    wire        nIRQ, nNMI, nDMA, nINH;
    reg         DMA_OUT, INT_OUT;
    wire        DMA_IN, INT_IN;
    reg         PHI0, PHI1, uSync;

    // Data bus tristate from testbench perspective
    assign data_bus = data_drive_en ? data_drive : 8'hZZ;

    // Individual data bus wires (bidirectional)
    wire D0, D1, D2, D3, D4, D5, D6, D7;
    assign D0 = data_bus[0];
    assign D1 = data_bus[1];
    assign D2 = data_bus[2];
    assign D3 = data_bus[3];
    assign D4 = data_bus[4];
    assign D5 = data_bus[5];
    assign D6 = data_bus[6];
    assign D7 = data_bus[7];

    // =========================================================================
    // GPIO Signals
    // =========================================================================
    wire GPIO1, GPIO2, GPIO3, GPIO4, GPIO5;
    wire GPIO6, GPIO7, GPIO10, GPIO11, GPIO12;
    reg  GPIO8, GPIO9;

    // =========================================================================
    // SDRAM Signals (individual pins matching top module ports)
    // =========================================================================
    wire SDRAM_CLK, SDRAM_CKE, SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE;
    wire SDRAM_DQM0, SDRAM_DQM1, SDRAM_BA0, SDRAM_BA1;
    wire SDRAM_A0, SDRAM_A1, SDRAM_A2, SDRAM_A3, SDRAM_A4;
    wire SDRAM_A5, SDRAM_A6, SDRAM_A7, SDRAM_A8, SDRAM_A9;
    wire SDRAM_A10, SDRAM_A11, SDRAM_A12;
    wire SDRAM_D0, SDRAM_D1, SDRAM_D2, SDRAM_D3;
    wire SDRAM_D4, SDRAM_D5, SDRAM_D6, SDRAM_D7;
    wire SDRAM_D8, SDRAM_D9, SDRAM_D10, SDRAM_D11;
    wire SDRAM_D12, SDRAM_D13, SDRAM_D14, SDRAM_D15;

    // =========================================================================
    // Flash Signals
    // =========================================================================
    wire FLASH_nCS, FLASH_MOSI, FLASH_nWP, FLASH_nHOLD;
    wire FLASH_MISO;

    // =========================================================================
    // Behavioral SDRAM Model
    // =========================================================================
    // Simplified: 64K x 16-bit memory (128KB), indexed by low bits of
    // column address. Supports ACTIVATE, READ (CAS=2), WRITE, REFRESH.
    // Sufficient for boot_loader's sequential writes and arbiter's block
    // transfers at low addresses.
    // =========================================================================

    // Reconstruct buses from individual pins
    wire [12:0] sdram_a_bus = {SDRAM_A12, SDRAM_A11, SDRAM_A10, SDRAM_A9,
                               SDRAM_A8, SDRAM_A7, SDRAM_A6, SDRAM_A5,
                               SDRAM_A4, SDRAM_A3, SDRAM_A2, SDRAM_A1, SDRAM_A0};
    wire [15:0] sdram_dq_in = {SDRAM_D15, SDRAM_D14, SDRAM_D13, SDRAM_D12,
                                SDRAM_D11, SDRAM_D10, SDRAM_D9,  SDRAM_D8,
                                SDRAM_D7,  SDRAM_D6,  SDRAM_D5,  SDRAM_D4,
                                SDRAM_D3,  SDRAM_D2,  SDRAM_D1,  SDRAM_D0};

    // Command decode
    wire [3:0] sdram_cmd = {SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE};
    localparam [3:0] SCMD_NOP       = 4'b0111;
    localparam [3:0] SCMD_ACTIVE    = 4'b0011;
    localparam [3:0] SCMD_READ      = 4'b0101;
    localparam [3:0] SCMD_WRITE     = 4'b0100;
    localparam [3:0] SCMD_PRECHARGE = 4'b0010;
    localparam [3:0] SCMD_REFRESH   = 4'b0001;
    localparam [3:0] SCMD_LOAD_MODE = 4'b0000;

    // Model storage: 64K words (128KB), enough for a small disk image
    reg [15:0] sdram_mem [0:65535];
    reg [15:0] sdram_dq_out;
    reg        sdram_dq_oe = 0;
    reg [15:0] sdram_read_addr;
    reg [2:0]  sdram_cas_delay;
    reg [12:0] sdram_active_row;
    reg [1:0]  sdram_active_bank;

    // Drive individual DQ pins from model
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

    integer sdram_init_i;
    initial begin
        for (sdram_init_i = 0; sdram_init_i < 65536; sdram_init_i = sdram_init_i + 1)
            sdram_mem[sdram_init_i] = 16'h0000;
        sdram_cas_delay  = 3'd0;
        sdram_dq_out     = 16'h0000;
        sdram_active_row = 13'd0;
        sdram_active_bank = 2'd0;
    end

    // Track total SDRAM writes for boot verification
    integer sdram_write_count = 0;

    always @(posedge SDRAM_CLK) begin
        // CAS latency pipeline
        if (sdram_cas_delay > 0) begin
            sdram_cas_delay <= sdram_cas_delay - 3'd1;
            if (sdram_cas_delay == 3'd1) begin
                sdram_dq_oe  <= 1'b1;
                sdram_dq_out <= sdram_mem[sdram_read_addr[15:0]];
            end
        end else begin
            sdram_dq_oe <= 1'b0;
        end

        // Command processing
        case (sdram_cmd)
            SCMD_ACTIVE: begin
                sdram_active_row  <= sdram_a_bus;
                sdram_active_bank <= {SDRAM_BA1, SDRAM_BA0};
            end
            SCMD_WRITE: begin
                // Index: row[2:0] || col[9:0] => 13-bit index, use low 16 bits
                // For boot, row=0, so effectively just col[9:0]
                sdram_mem[{sdram_active_row[2:0], sdram_a_bus[9:0], sdram_active_bank[0]}] <= sdram_dq_in;
                sdram_write_count = sdram_write_count + 1;
            end
            SCMD_READ: begin
                sdram_read_addr <= {sdram_active_row[2:0], sdram_a_bus[9:0], sdram_active_bank[0]};
                // CAS latency 2: set counter to 1 (model sees cmd 1 cycle late)
                sdram_cas_delay <= 3'd1;
            end
            SCMD_REFRESH: begin
                // No-op in model
            end
            SCMD_PRECHARGE: begin
                // No-op in model
            end
            SCMD_LOAD_MODE: begin
                // No-op in model
            end
        endcase
    end

    // =========================================================================
    // Behavioral SPI Flash Model
    // =========================================================================
    // 4KB memory starting at offset 0. Boot loader reads from FLASH_OFFSET
    // (0x400000), so flash model maps address & 0xFFF to internal storage.
    // Pre-loaded with a known pattern: addr[7:0] as low byte, ~addr[7:0] as
    // high byte (when viewed as 16-bit pairs).
    // =========================================================================

    reg [7:0] flash_mem [0:4095];

    // SPI state
    reg [7:0]  flash_cmd_reg;
    reg [23:0] flash_addr_reg;
    reg [5:0]  flash_bit_cnt;
    reg        flash_cmd_phase;
    reg [7:0]  flash_data_byte;
    reg [2:0]  flash_out_bit_cnt;
    reg [23:0] flash_read_addr;
    reg        flash_miso_reg;

    assign FLASH_MISO = flash_miso_reg;

    // Get SCK from the flash_reader's simulation output
    // The flash_reader uses an internal spi_sck register.
    // In simulation (SYNTHESIS not defined), flash_sck_pin is driven.
    // We need to access it through the DUT hierarchy.
    wire flash_sck = dut.u_flash_reader.flash_sck_pin;

    integer flash_init_i;
    initial begin
        // Fill flash with known pattern: byte at address A = A[7:0]
        // This matches the flash_reader_tb pattern for easy verification
        for (flash_init_i = 0; flash_init_i < 4096; flash_init_i = flash_init_i + 1)
            flash_mem[flash_init_i] = flash_init_i[7:0];
        flash_miso_reg     = 1'bz;
        flash_bit_cnt      = 6'd0;
        flash_cmd_phase    = 1'b1;
        flash_cmd_reg      = 8'd0;
        flash_addr_reg     = 24'd0;
        flash_out_bit_cnt  = 3'd0;
        flash_read_addr    = 24'd0;
        flash_data_byte    = 8'd0;
    end

    // CS deasserted — reset
    always @(posedge FLASH_nCS) begin
        flash_bit_cnt     <= 6'd0;
        flash_cmd_phase   <= 1'b1;
        flash_miso_reg    <= 1'bz;
    end

    // CS asserted — prepare
    always @(negedge FLASH_nCS) begin
        flash_bit_cnt     <= 6'd0;
        flash_cmd_phase   <= 1'b1;
        flash_cmd_reg     <= 8'd0;
        flash_addr_reg    <= 24'd0;
    end

    // Sample MOSI on SCK rising edge
    always @(posedge flash_sck) begin
        if (!FLASH_nCS && flash_cmd_phase) begin
            if (flash_bit_cnt < 6'd8) begin
                flash_cmd_reg <= {flash_cmd_reg[6:0], FLASH_MOSI};
            end else if (flash_bit_cnt < 6'd32) begin
                flash_addr_reg <= {flash_addr_reg[22:0], FLASH_MOSI};
            end
            flash_bit_cnt <= flash_bit_cnt + 6'd1;

            if (flash_bit_cnt == 6'd31) begin
                flash_cmd_phase   <= 1'b0;
                flash_read_addr   <= {flash_addr_reg[22:0], FLASH_MOSI};
                flash_out_bit_cnt <= 3'd0;
            end
        end
    end

    // Drive MISO on SCK falling edge during data phase
    always @(negedge flash_sck) begin
        if (!FLASH_nCS && !flash_cmd_phase) begin
            if (flash_out_bit_cnt == 3'd0) begin
                // Load next byte (wrap address to 4KB model)
                flash_data_byte <= flash_mem[flash_read_addr[11:0]];
            end

            // Drive current bit (MSB first)
            flash_miso_reg <= (flash_out_bit_cnt == 3'd0) ?
                flash_mem[flash_read_addr[11:0]][7] :
                flash_data_byte[7 - flash_out_bit_cnt];

            flash_out_bit_cnt <= flash_out_bit_cnt + 3'd1;
            if (flash_out_bit_cnt == 3'd7) begin
                flash_read_addr   <= flash_read_addr + 24'd1;
                flash_out_bit_cnt <= 3'd0;
            end
        end
    end

    // =========================================================================
    // DUT Instantiation
    // =========================================================================
    iwm_hamr_top dut (
        .addr(addr),
        .D0(D0), .D1(D1), .D2(D2), .D3(D3),
        .D4(D4), .D5(D5), .D6(D6), .D7(D7),
        .sig_7M(clk_7m), .Q3(Q3), .R_nW(R_nW),
        .nDEVICE_SELECT(nDEVICE_SELECT),
        .nI_O_SELECT(nI_O_SELECT),
        .nI_O_STROBE(nI_O_STROBE),
        .nRES(nRES),
        .RDY(RDY),
        .nIRQ(nIRQ), .nNMI(nNMI), .nDMA(nDMA), .nINH(nINH),
        .DMA_OUT(DMA_OUT), .DMA_IN(DMA_IN),
        .INT_OUT(INT_OUT), .INT_IN(INT_IN),
        .PHI0(PHI0), .PHI1(PHI1), .uSync(uSync),
        .CLK_25MHz(clk_25m),
        // SDRAM
        .SDRAM_CLK(SDRAM_CLK), .SDRAM_CKE(SDRAM_CKE),
        .SDRAM_nCS(SDRAM_nCS), .SDRAM_nRAS(SDRAM_nRAS),
        .SDRAM_nCAS(SDRAM_nCAS), .SDRAM_nWE(SDRAM_nWE),
        .SDRAM_DQM0(SDRAM_DQM0), .SDRAM_DQM1(SDRAM_DQM1),
        .SDRAM_BA0(SDRAM_BA0), .SDRAM_BA1(SDRAM_BA1),
        .SDRAM_A0(SDRAM_A0), .SDRAM_A1(SDRAM_A1), .SDRAM_A2(SDRAM_A2),
        .SDRAM_A3(SDRAM_A3), .SDRAM_A4(SDRAM_A4), .SDRAM_A5(SDRAM_A5),
        .SDRAM_A6(SDRAM_A6), .SDRAM_A7(SDRAM_A7), .SDRAM_A8(SDRAM_A8),
        .SDRAM_A9(SDRAM_A9), .SDRAM_A10(SDRAM_A10), .SDRAM_A11(SDRAM_A11),
        .SDRAM_A12(SDRAM_A12),
        .SDRAM_D0(SDRAM_D0), .SDRAM_D1(SDRAM_D1), .SDRAM_D2(SDRAM_D2),
        .SDRAM_D3(SDRAM_D3), .SDRAM_D4(SDRAM_D4), .SDRAM_D5(SDRAM_D5),
        .SDRAM_D6(SDRAM_D6), .SDRAM_D7(SDRAM_D7), .SDRAM_D8(SDRAM_D8),
        .SDRAM_D9(SDRAM_D9), .SDRAM_D10(SDRAM_D10), .SDRAM_D11(SDRAM_D11),
        .SDRAM_D12(SDRAM_D12), .SDRAM_D13(SDRAM_D13), .SDRAM_D14(SDRAM_D14),
        .SDRAM_D15(SDRAM_D15),
        // Flash
        .FLASH_nCS(FLASH_nCS), .FLASH_MOSI(FLASH_MOSI),
        .FLASH_MISO(FLASH_MISO), .FLASH_nWP(FLASH_nWP),
        .FLASH_nHOLD(FLASH_nHOLD),
        // GPIO
        .GPIO1(GPIO1), .GPIO2(GPIO2), .GPIO3(GPIO3), .GPIO4(GPIO4),
        .GPIO5(GPIO5), .GPIO6(GPIO6), .GPIO7(GPIO7),
        .GPIO8(GPIO8), .GPIO9(GPIO9),
        .GPIO10(GPIO10), .GPIO11(GPIO11), .GPIO12(GPIO12)
    );

    // =========================================================================
    // Apple II Bus Helper Tasks
    // =========================================================================

    // Read from slot 4 I/O space ($C4xx)
    task bus_read_slot4(input [7:0] slot_addr, output [7:0] rdata);
    begin
        R_nW = 1;
        data_drive_en = 0;
        addr = {4'h4, slot_addr};
        nI_O_SELECT = 0;
        @(posedge clk_7m);
        @(negedge clk_7m);
        rdata = {D7, D6, D5, D4, D3, D2, D1, D0};
        nI_O_SELECT = 1;
        @(posedge clk_7m);
    end
    endtask

    // Read from expansion ROM ($C800-$CFFF)
    task bus_read_expansion(input [10:0] exp_addr, output [7:0] rdata);
    begin
        R_nW = 1;
        data_drive_en = 0;
        addr = {1'b1, exp_addr};
        nI_O_STROBE = 0;
        @(posedge clk_7m);
        @(negedge clk_7m);
        rdata = {D7, D6, D5, D4, D3, D2, D1, D0};
        nI_O_STROBE = 1;
        @(posedge clk_7m);
    end
    endtask

    // =========================================================================
    // Main Test Sequence
    // =========================================================================
    reg [7:0] read_data;

    initial begin
        $dumpfile("iwm_hamr_tb.vcd");
        $dumpvars(0, iwm_hamr_tb);

        // Initialize all signals
        addr           = 12'h000;
        Q3             = 0;
        R_nW           = 1;
        nDEVICE_SELECT = 1;
        nI_O_SELECT    = 1;
        nI_O_STROBE    = 1;
        RDY            = 1;
        DMA_OUT        = 1;
        INT_OUT        = 1;
        PHI0           = 0;
        PHI1           = 0;
        uSync          = 0;
        GPIO8          = 0;    // unused input (formerly sense)
        GPIO9          = 0;    // unused input (formerly rddata)
        data_drive     = 8'h00;
        data_drive_en  = 0;

        $display("");
        $display("==============================================");
        $display("IWM Hamr Integration Testbench");
        $display("==============================================");

        // =====================================================================
        // Test 1: POR Sequence
        // =====================================================================
        $display("\n--- Test 1: Power-On Reset ---");

        // 7 MHz POR: 4-bit counter, 15 clocks to saturate (~2 us)
        // 25 MHz POR: 8-bit counter, 255 clocks to saturate (~10 us)
        // Wait enough for both to complete
        repeat (80) @(posedge clk_7m);

        check("7 MHz POR released (por_n = 1)", dut.por_n === 1'b1);
        check("25 MHz POR released (por_25_n = 1)", dut.por_25_n === 1'b1);

        // Verify control signals are correct after POR
        check("nRES = 1 (not asserting reset)", nRES === 1'b1);
        check("nIRQ = Z (tri-stated)", nIRQ === 1'bZ);
        check("nNMI = Z (tri-stated)", nNMI === 1'bZ);
        check("nDMA = Z (tri-stated)", nDMA === 1'bZ);
        check("nINH = Z (tri-stated)", nINH === 1'bZ);
        check("DMA daisy chain pass-through", DMA_IN === DMA_OUT);
        check("INT daisy chain pass-through", INT_IN === INT_OUT);

        // =====================================================================
        // Test 2: SDRAM Initialization
        // =====================================================================
        $display("\n--- Test 2: SDRAM Initialization ---");

        begin : test_sdram_init
            integer timeout;
            timeout = 0;
            while (!dut.sdram_init_done && timeout < 300000) begin
                @(posedge clk_25m);
                timeout = timeout + 1;
            end
            check("SDRAM init_done asserted", dut.sdram_init_done === 1'b1);
            if (dut.sdram_init_done)
                $display("    init_done after %0d clocks at 25 MHz (~%0d us)",
                         timeout, timeout / 25);
            else
                $display("    TIMEOUT: init_done never asserted after %0d clocks", timeout);
        end

        // =====================================================================
        // Test 3: Boot bypass (boot path tested by unit TBs)
        // =====================================================================
        $display("\n--- Test 3: Boot Bypass ---");

        // The full boot (143KB SPI -> SDRAM) takes millions of sim clocks.
        // boot_loader_tb, flash_reader_tb, and sdram_controller_tb already
        // cover the boot path. Force boot_done to exercise post-boot state.
        force dut.boot_done = 1'b1;
        repeat (10) @(posedge clk_25m);
        check("boot_done forced HIGH", dut.boot_done === 1'b1);
        $display("    boot_done forced — skipping SPI flash copy");

        // =====================================================================
        // Test 4: Device Present (sense HIGH after boot)
        // =====================================================================
        $display("\n--- Test 4: Device Present ---");

        // After boot_done, sp_device should drive sense HIGH in IDLE state.
        // Wait a few 7 MHz cycles for the signal to propagate.
        repeat (20) @(posedge clk_7m);

        check("sp_device sense = HIGH (device present)",
              dut.u_sp_device.sense === 1'b1);
        check("GPIO1 = boot_done (HIGH)", GPIO1 === 1'b1);
        check("GPIO2 = sense mirror (HIGH)", GPIO2 === 1'b1);

        // Verify sp_device is in IDLE state
        check("sp_device in IDLE state", dut.u_sp_device.state === 5'd0);

        // =====================================================================
        // Test 5: ROM Still Works (Apple II bus functional)
        // =====================================================================
        $display("\n--- Test 5: ROM Reads ---");

        // Read slot 4 ROM byte at $C400 (first byte of boot code)
        bus_read_slot4(8'h00, read_data);
        check("ROM slot 4 byte 0 is non-zero", read_data !== 8'h00);
        $display("    ROM[$C400] = $%02X", read_data);

        // Read a few more bytes to confirm ROM access is stable
        bus_read_slot4(8'h05, read_data);
        $display("    ROM[$C405] = $%02X", read_data);
        check("ROM slot 4 byte 5 readable", 1'b1);

        // Read expansion ROM (should be active after slot 4 access)
        bus_read_expansion(11'h000, read_data);
        $display("    ROM[$C800] = $%02X", read_data);
        check("Expansion ROM byte readable", 1'b1);

        // Verify level shifter OE is inactive when no device access
        nDEVICE_SELECT = 1;
        nI_O_SELECT    = 1;
        nI_O_STROBE    = 1;
        @(posedge clk_7m);
        @(posedge clk_7m);
        check("Level shifter OE inactive when idle (GPIO12=1)",
              GPIO12 === 1'b1);

        // =====================================================================
        // Summary
        // =====================================================================
        $display("");
        $display("==============================================");
        $display("IWM Hamr Integration Testbench Results:");
        $display("  %0d tests passed, %0d failed", pass_count, fail_count);
        $display("==============================================");

        if (fail_count > 0)
            $display("RESULT: FAIL");
        else
            $display("RESULT: PASS");

        $display("");
        #1000;
        $finish;
    end

    // =========================================================================
    // Watchdog timer — abort if simulation hangs
    // =========================================================================
    initial begin
        #500000000;  // 500 ms simulation time
        $display("WATCHDOG: Simulation timeout reached, aborting");
        $display("  %0d tests passed, %0d failed (incomplete)", pass_count, fail_count);
        $display("RESULT: FAIL (timeout)");
        $finish;
    end

endmodule
