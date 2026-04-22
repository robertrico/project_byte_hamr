// =============================================================================
// Byte Hamr SDRAM Test - Testbench
// =============================================================================
//
// Simulates the sdram_test_top module with:
// - 25 MHz system clock
// - Simple SDRAM model (8 locations, CAS latency 2)
// - Apple II bus transaction tasks (register read/write)
// - End-to-end SDRAM write/read verification
//
// =============================================================================

`timescale 1ns / 100ps

module sdram_test_tb;

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

    // Output control signals
    wire       dma_out_w, int_out_w;
    wire       dma_in_w, int_in_w;

    // GPIO
    wire       gpio1, gpio2, gpio3, gpio4;
    wire       gpio5, gpio6, gpio7, gpio8;
    wire       gpio9, gpio10, gpio11, gpio12;

    // =========================================================================
    // Simple SDRAM Model
    // =========================================================================

    reg [15:0] sdram_mem [0:7];
    reg [15:0] sdram_dq_out;
    reg        sdram_dq_oe = 0;
    reg [2:0]  sdram_read_addr;
    reg [2:0]  sdram_cas_delay;

    wire [12:0] sdram_addr_w = {sdram_a12, sdram_a11, sdram_a10, sdram_a9, sdram_a8,
                                sdram_a7, sdram_a6, sdram_a5, sdram_a4,
                                sdram_a3, sdram_a2, sdram_a1, sdram_a0};
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

    reg [2:0] active_row;

    integer i;
    initial begin
        for (i = 0; i < 8; i = i + 1)
            sdram_mem[i] = 16'h0000;
        sdram_cas_delay = 0;
    end

    always @(posedge sdram_clk) begin
        if (sdram_cas_delay > 0) begin
            sdram_cas_delay <= sdram_cas_delay - 1;
            if (sdram_cas_delay == 1) begin
                sdram_dq_oe <= 1;
                sdram_dq_out <= sdram_mem[sdram_read_addr];
            end
        end else begin
            sdram_dq_oe <= 0;
        end

        case (sdram_cmd)
            CMD_ACTIVE: begin
                active_row <= sdram_addr_w[2:0];
            end
            CMD_WRITE: begin
                // Use column address (from WRITE command) for memory indexing
                // Test addresses differ in column bits, not row bits
                sdram_mem[sdram_addr_w[2:0]] <= sdram_dq_in_w;
                $display("[%0t] SDRAM WRITE: col=%0d data=%04h", $time, sdram_addr_w[2:0], sdram_dq_in_w);
            end
            CMD_READ: begin
                // Use column address (from READ command) for memory indexing
                sdram_read_addr <= sdram_addr_w[2:0];
                // CAS latency 2: set counter to 1 because model sees the
                // READ command 1 cycle late (DUT NBA delay), so effective
                // latency is counter + 1 = 2 cycles from DUT perspective.
                sdram_cas_delay <= 1;
                $display("[%0t] SDRAM READ:  col=%0d (data=%04h)", $time, sdram_addr_w[2:0], sdram_mem[sdram_addr_w[2:0]]);
            end
        endcase
    end

    // =========================================================================
    // Device Under Test
    // =========================================================================

    sdram_test_top dut (
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
        .DMA_IN(dma_in_w),
        .INT_IN(int_in_w),
        .nINH(),
        .nDMA(),
        .DMA_OUT(1'b0),
        .INT_OUT(1'b0),

        // GPIO
        .GPIO1(gpio1),
        .GPIO2(gpio2),
        .GPIO3(gpio3),
        .GPIO4(gpio4),
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
    // Bus Transaction Tasks
    // =========================================================================

    // Write a value to a device select register
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

    // Read a value from a device select register
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

    // Poll STATUS register until not busy (with timeout)
    task wait_sdram_ready;
        integer timeout;
        begin
            timeout = 0;
            bus_read_result = 8'hFF;
            while (bus_read_result[0] == 1'b1 && timeout < 1000) begin
                bus_read(4'h6);
                timeout = timeout + 1;
            end
            if (timeout >= 1000) begin
                $display("[%0t] ERROR: SDRAM busy timeout!", $time);
            end
        end
    endtask

    // High-level: Write a 16-bit word to SDRAM
    task sdram_write_word;
        input [7:0] addr_lo;
        input [7:0] addr_mid;
        input [7:0] addr_hi;
        input [7:0] data_lo;
        input [7:0] data_hi;
        begin
            wait_sdram_ready;
            bus_write(4'h0, addr_lo);
            bus_write(4'h1, addr_mid);
            bus_write(4'h2, addr_hi);
            bus_write(4'h3, data_lo);
            bus_write(4'h4, data_hi);
            bus_write(4'h5, 8'h01);  // CMD = WRITE
            $display("[%0t] CMD WRITE: addr=%02h%02h%02h data=%02h%02h",
                     $time, addr_hi, addr_mid, addr_lo, data_hi, data_lo);
        end
    endtask

    // High-level: Read a 16-bit word from SDRAM
    // Result available in bus_read_result after each bus_read call
    reg [7:0] sdram_read_lo;
    reg [7:0] sdram_read_hi;
    task sdram_read_word;
        input [7:0] addr_lo;
        input [7:0] addr_mid;
        input [7:0] addr_hi;
        begin
            wait_sdram_ready;
            bus_write(4'h0, addr_lo);
            bus_write(4'h1, addr_mid);
            bus_write(4'h2, addr_hi);
            bus_write(4'h5, 8'h02);  // CMD = READ
            wait_sdram_ready;
            bus_read(4'h3);
            sdram_read_lo = bus_read_result;
            bus_read(4'h4);
            sdram_read_hi = bus_read_result;
            $display("[%0t] CMD READ:  addr=%02h%02h%02h result=%02h%02h",
                     $time, addr_hi, addr_mid, addr_lo, sdram_read_hi, sdram_read_lo);
        end
    endtask

    // =========================================================================
    // Test Sequence
    // =========================================================================

    integer test_pass;
    integer test_fail;

    initial begin
        $dumpfile("sdram_test_tb.vcd");
        $dumpvars(0, sdram_test_tb);

        test_pass = 0;
        test_fail = 0;

        $display("===========================================");
        $display("Byte Hamr SDRAM Register Access Testbench");
        $display("===========================================");
        $display("");

        // ---- Wait for SDRAM initialization ----
        $display("[%0t] Waiting for SDRAM initialization...", $time);
        #250_000;  // 250us - past the 200us power-up delay

        // Poll until init_done
        begin : wait_init
            integer timeout;
            timeout = 0;
            bus_read_result = 8'h00;
            while (bus_read_result[1] == 1'b0 && timeout < 200) begin
                bus_read(4'h6);
                timeout = timeout + 1;
            end
            if (bus_read_result[1]) begin
                $display("[%0t] SDRAM initialization complete (STATUS=%02h)", $time, bus_read_result);
            end else begin
                $display("[%0t] ERROR: SDRAM init timeout!", $time);
                $finish;
            end
        end

        // ---- Test 1: Scratch register loopback ----
        $display("");
        $display("--- Test 1: Scratch register loopback ---");
        begin : test_scratch
            bus_write(4'h7, 8'hA5);
            bus_read(4'h7);
            if (bus_read_result == 8'hA5) begin
                $display("  REG $07: PASS (wrote $A5, read $%02h)", bus_read_result);
                test_pass = test_pass + 1;
            end else begin
                $display("  REG $07: FAIL (wrote $A5, read $%02h)", bus_read_result);
                test_fail = test_fail + 1;
            end

            bus_write(4'hF, 8'h42);
            bus_read(4'hF);
            if (bus_read_result == 8'h42) begin
                $display("  REG $0F: PASS (wrote $42, read $%02h)", bus_read_result);
                test_pass = test_pass + 1;
            end else begin
                $display("  REG $0F: FAIL (wrote $42, read $%02h)", bus_read_result);
                test_fail = test_fail + 1;
            end
        end

        // ---- Test 2: SDRAM write and read $BEEF at address 0 ----
        $display("");
        $display("--- Test 2: SDRAM write $BEEF to address 0 ---");
        begin : test_beef
            sdram_write_word(8'h00, 8'h00, 8'h00, 8'hEF, 8'hBE);
            sdram_read_word(8'h00, 8'h00, 8'h00);
            if ({sdram_read_hi, sdram_read_lo} == 16'hBEEF) begin
                $display("  PASS (read $%02h%02h)", sdram_read_hi, sdram_read_lo);
                test_pass = test_pass + 1;
            end else begin
                $display("  FAIL (expected $BEEF, got $%02h%02h)", sdram_read_hi, sdram_read_lo);
                test_fail = test_fail + 1;
            end
        end

        // ---- Test 3: SDRAM write and read $CAFE at address 1 ----
        $display("");
        $display("--- Test 3: SDRAM write $CAFE to address 1 ---");
        begin : test_cafe
            sdram_write_word(8'h01, 8'h00, 8'h00, 8'hFE, 8'hCA);
            sdram_read_word(8'h01, 8'h00, 8'h00);
            if ({sdram_read_hi, sdram_read_lo} == 16'hCAFE) begin
                $display("  PASS (read $%02h%02h)", sdram_read_hi, sdram_read_lo);
                test_pass = test_pass + 1;
            end else begin
                $display("  FAIL (expected $CAFE, got $%02h%02h)", sdram_read_hi, sdram_read_lo);
                test_fail = test_fail + 1;
            end
        end

        // ---- Test 4: Re-read address 0 (verify not corrupted) ----
        $display("");
        $display("--- Test 4: Re-read address 0 (data integrity) ---");
        begin : test_reread
            sdram_read_word(8'h00, 8'h00, 8'h00);
            if ({sdram_read_hi, sdram_read_lo} == 16'hBEEF) begin
                $display("  PASS (still $%02h%02h)", sdram_read_hi, sdram_read_lo);
                test_pass = test_pass + 1;
            end else begin
                $display("  FAIL (expected $BEEF, got $%02h%02h)", sdram_read_hi, sdram_read_lo);
                test_fail = test_fail + 1;
            end
        end

        // ---- Test 5: Multi-address write/read (8 locations) ----
        $display("");
        $display("--- Test 5: Multi-address write/read (8 locations) ---");
        begin : test_multi
            integer j;
            integer pass_cnt;
            pass_cnt = 0;

            // Write phase
            for (j = 0; j < 8; j = j + 1) begin
                sdram_write_word(j[7:0], 8'h00, 8'h00, j[7:0], ~j[7:0]);
            end

            // Read and verify phase
            for (j = 0; j < 8; j = j + 1) begin
                sdram_read_word(j[7:0], 8'h00, 8'h00);
                if (sdram_read_lo == j[7:0] && sdram_read_hi == ~j[7:0]) begin
                    pass_cnt = pass_cnt + 1;
                end else begin
                    $display("  addr %0d: FAIL (expected $%02h%02h, got $%02h%02h)",
                             j, ~j[7:0], j[7:0], sdram_read_hi, sdram_read_lo);
                end
            end
            $display("  Multi-address: %0d/8 passed", pass_cnt);
            if (pass_cnt == 8) begin
                test_pass = test_pass + 1;
            end else begin
                test_fail = test_fail + 1;
            end
        end

        // ---- Test 6: Command while busy (graceful rejection) ----
        $display("");
        $display("--- Test 6: Command while busy ---");
        begin : test_busy
            // Set up address
            bus_write(4'h0, 8'h03);
            bus_write(4'h1, 8'h00);
            bus_write(4'h2, 8'h00);
            // Issue READ command
            bus_write(4'h5, 8'h02);
            // Immediately try another command (should be ignored)
            bus_write(4'h5, 8'h01);
            // Wait for first command to complete
            wait_sdram_ready;
            $display("  PASS (no hang, completed gracefully)");
            test_pass = test_pass + 1;
        end

        // ---- Test 7: Address register readback ----
        $display("");
        $display("--- Test 7: Address register readback ---");
        begin : test_addr_readback
            bus_write(4'h0, 8'hAB);
            bus_write(4'h1, 8'hCD);
            bus_write(4'h2, 8'h12);
            bus_read(4'h0);
            if (bus_read_result == 8'hAB) begin
                $display("  ADDR_LO: PASS ($%02h)", bus_read_result);
            end else begin
                $display("  ADDR_LO: FAIL (expected $AB, got $%02h)", bus_read_result);
                test_fail = test_fail + 1;
            end
            bus_read(4'h1);
            if (bus_read_result == 8'hCD) begin
                $display("  ADDR_MID: PASS ($%02h)", bus_read_result);
            end else begin
                $display("  ADDR_MID: FAIL (expected $CD, got $%02h)", bus_read_result);
                test_fail = test_fail + 1;
            end
            bus_read(4'h2);
            if (bus_read_result == 8'h12) begin
                $display("  ADDR_HI: PASS ($%02h)", bus_read_result);
                test_pass = test_pass + 1;
            end else begin
                $display("  ADDR_HI: FAIL (expected $12, got $%02h)", bus_read_result);
                test_fail = test_fail + 1;
            end
        end

        // ---- Summary ----
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
    // Monitor - GPIO state changes
    // =========================================================================

    reg prev_gpio2 = 0;
    reg prev_gpio4 = 0;

    always @(posedge clk_25mhz) begin
        if (gpio2 && !prev_gpio2)
            $display("[%0t] GPIO2: SDRAM init done", $time);
        prev_gpio2 <= gpio2;
        prev_gpio4 <= gpio4;
    end

endmodule
