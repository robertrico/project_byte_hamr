// =============================================================================
// Byte Hamr SDRAM Bank Switcher - Testbench
// =============================================================================
//
// Tests bank-switching SDRAM access:
// - SDRAM init detection
// - Scratch register loopback
// - Single bank write/read with staging trash
// - Bank independence (same address, different banks)
// - Address independence within a bank
// - Cross-bank address test
//
// =============================================================================

`timescale 1ns / 100ps

module sdram_bank_tb;

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

    // GPIO
    wire       gpio1, gpio2, gpio3, gpio4;
    wire       gpio5, gpio6, gpio7, gpio8;
    wire       gpio9, gpio10, gpio11, gpio12;

    // =========================================================================
    // SDRAM Model - 4 banks, 8 locations per bank
    // =========================================================================

    reg [7:0]  sdram_mem [0:31];   // {bank[1:0], col[2:0]} = 32 entries
    reg [15:0] sdram_dq_out;
    reg        sdram_dq_oe = 0;
    reg [4:0]  sdram_read_idx;
    reg [2:0]  sdram_cas_delay;

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

    reg [1:0] active_bank;

    integer j;
    initial begin
        for (j = 0; j < 32; j = j + 1)
            sdram_mem[j] = 8'h00;
        sdram_cas_delay = 0;
    end

    always @(posedge sdram_clk) begin
        if (sdram_cas_delay > 0) begin
            sdram_cas_delay <= sdram_cas_delay - 1;
            if (sdram_cas_delay == 1) begin
                sdram_dq_oe <= 1;
                // Return low byte from model, upper byte 0
                sdram_dq_out <= {8'h00, sdram_mem[sdram_read_idx]};
            end
        end else begin
            sdram_dq_oe <= 0;
        end

        case (sdram_cmd)
            CMD_ACTIVE: begin
                active_bank <= sdram_bank_w;
            end
            CMD_WRITE: begin
                // Index: {bank, col[2:0]}
                sdram_mem[{active_bank, sdram_addr_w[2:0]}] <= sdram_dq_in_w[7:0];
                $display("[%0t] SDRAM WRITE: bank=%0d col=%0d data=%02h",
                         $time, active_bank, sdram_addr_w[2:0], sdram_dq_in_w[7:0]);
            end
            CMD_READ: begin
                sdram_read_idx <= {active_bank, sdram_addr_w[2:0]};
                // CAS latency 2: counter=1 (NBA compensation)
                sdram_cas_delay <= 1;
                $display("[%0t] SDRAM READ:  bank=%0d col=%0d (data=%02h)",
                         $time, active_bank, sdram_addr_w[2:0],
                         sdram_mem[{active_bank, sdram_addr_w[2:0]}]);
            end
        endcase
    end

    // =========================================================================
    // Device Under Test
    // =========================================================================

    sdram_bank_top dut (
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

    // Poll STATUS until not busy
    task wait_sdram_ready;
        integer timeout;
        begin
            timeout = 0;
            bus_read_result = 8'hFF;
            while (bus_read_result[0] == 1'b1 && timeout < 1000) begin
                bus_read(4'h4);
                timeout = timeout + 1;
            end
            if (timeout >= 1000) begin
                $display("[%0t] ERROR: SDRAM busy timeout!", $time);
            end
        end
    endtask

    // High-level: Write byte to bank/addr
    task bank_write;
        input [7:0] bank;
        input [7:0] addr;
        input [7:0] data;
        begin
            wait_sdram_ready;
            bus_write(4'h0, bank);   // BANK
            bus_write(4'h1, addr);   // ADDR
            bus_write(4'h2, data);   // DATA
            bus_write(4'h3, 8'h01);  // CMD = WRITE
            $display("[%0t] CMD WRITE: bank=%0d addr=%02h data=%02h",
                     $time, bank, addr, data);
        end
    endtask

    // High-level: Read byte from bank/addr
    reg [7:0] bank_read_result;
    task bank_read;
        input [7:0] bank;
        input [7:0] addr;
        begin
            wait_sdram_ready;
            bus_write(4'h0, bank);   // BANK
            bus_write(4'h1, addr);   // ADDR
            bus_write(4'h3, 8'h02);  // CMD = READ
            wait_sdram_ready;
            bus_read(4'h2);          // DATA
            bank_read_result = bus_read_result;
            $display("[%0t] CMD READ:  bank=%0d addr=%02h result=%02h",
                     $time, bank, addr, bank_read_result);
        end
    endtask

    // =========================================================================
    // Test Sequence
    // =========================================================================

    integer test_pass;
    integer test_fail;

    initial begin
        $dumpfile("sdram_bank_tb.vcd");
        $dumpvars(0, sdram_bank_tb);

        test_pass = 0;
        test_fail = 0;

        $display("===========================================");
        $display("Byte Hamr SDRAM Bank Switcher Testbench");
        $display("===========================================");
        $display("");

        // ---- Wait for SDRAM init ----
        $display("[%0t] Waiting for SDRAM initialization...", $time);
        #250_000;

        begin : wait_init
            integer timeout;
            timeout = 0;
            bus_read_result = 8'h00;
            while (bus_read_result[1] == 1'b0 && timeout < 200) begin
                bus_read(4'h4);
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
            bus_write(4'h5, 8'hA5);
            bus_read(4'h5);
            if (bus_read_result == 8'hA5) begin
                $display("  REG $05: PASS (wrote $A5, read $%02h)", bus_read_result);
                test_pass = test_pass + 1;
            end else begin
                $display("  REG $05: FAIL (wrote $A5, read $%02h)", bus_read_result);
                test_fail = test_fail + 1;
            end
        end

        // ---- Test 2: Single bank write/read with staging trash ----
        $display("");
        $display("--- Test 2: Single bank write/read (bank 0, addr 0) ---");
        begin : test_single
            bank_write(8'd0, 8'h00, 8'hAA);
            // Trash the DATA register to prove read comes from SDRAM
            wait_sdram_ready;
            bus_write(4'h2, 8'h00);  // Overwrite DATA with $00
            bank_read(8'd0, 8'h00);
            if (bank_read_result == 8'hAA) begin
                $display("  PASS (read $%02h after trashing staging)", bank_read_result);
                test_pass = test_pass + 1;
            end else begin
                $display("  FAIL (expected $AA, got $%02h)", bank_read_result);
                test_fail = test_fail + 1;
            end
        end

        // ---- Test 3: Bank independence ----
        $display("");
        $display("--- Test 3: Bank independence (addr 0 across 4 banks) ---");
        begin : test_bank_ind
            integer k;
            integer pass_cnt;
            reg [7:0] expected;
            pass_cnt = 0;

            // Write different values to same address in each bank
            bank_write(8'd0, 8'h00, 8'hAA);
            bank_write(8'd1, 8'h00, 8'hBB);
            bank_write(8'd2, 8'h00, 8'hCC);
            bank_write(8'd3, 8'h00, 8'hDD);

            // Read back and verify each bank
            for (k = 0; k < 4; k = k + 1) begin
                bank_read(k[7:0], 8'h00);
                case (k)
                    0: expected = 8'hAA;
                    1: expected = 8'hBB;
                    2: expected = 8'hCC;
                    3: expected = 8'hDD;
                endcase
                if (bank_read_result == expected) begin
                    $display("  Bank %0d: PASS ($%02h)", k, bank_read_result);
                    pass_cnt = pass_cnt + 1;
                end else begin
                    $display("  Bank %0d: FAIL (expected $%02h, got $%02h)",
                             k, expected, bank_read_result);
                end
            end
            if (pass_cnt == 4) begin
                test_pass = test_pass + 1;
            end else begin
                test_fail = test_fail + 1;
            end
        end

        // ---- Test 4: Address independence within bank 0 ----
        $display("");
        $display("--- Test 4: Address independence (8 addrs in bank 0) ---");
        begin : test_addr_ind
            integer k;
            integer pass_cnt;
            pass_cnt = 0;

            // Write distinct values to 8 addresses in bank 0
            for (k = 0; k < 8; k = k + 1) begin
                bank_write(8'd0, k[7:0], k[7:0] + 8'h10);
            end

            // Read back and verify
            for (k = 0; k < 8; k = k + 1) begin
                bank_read(8'd0, k[7:0]);
                if (bank_read_result == k[7:0] + 8'h10) begin
                    pass_cnt = pass_cnt + 1;
                end else begin
                    $display("  Addr %0d: FAIL (expected $%02h, got $%02h)",
                             k, k[7:0] + 8'h10, bank_read_result);
                end
            end
            $display("  Address independence: %0d/8 passed", pass_cnt);
            if (pass_cnt == 8) begin
                test_pass = test_pass + 1;
            end else begin
                test_fail = test_fail + 1;
            end
        end

        // ---- Test 5: Cross-bank address test ----
        $display("");
        $display("--- Test 5: Cross-bank address test ---");
        begin : test_cross
            integer pass_cnt;
            pass_cnt = 0;

            bank_write(8'd0, 8'h05, 8'h11);
            bank_write(8'd2, 8'h05, 8'h22);

            bank_read(8'd0, 8'h05);
            if (bank_read_result == 8'h11) begin
                $display("  Bank 0 addr 5: PASS ($%02h)", bank_read_result);
                pass_cnt = pass_cnt + 1;
            end else begin
                $display("  Bank 0 addr 5: FAIL (expected $11, got $%02h)", bank_read_result);
            end

            bank_read(8'd2, 8'h05);
            if (bank_read_result == 8'h22) begin
                $display("  Bank 2 addr 5: PASS ($%02h)", bank_read_result);
                pass_cnt = pass_cnt + 1;
            end else begin
                $display("  Bank 2 addr 5: FAIL (expected $22, got $%02h)", bank_read_result);
            end

            if (pass_cnt == 2) begin
                test_pass = test_pass + 1;
            end else begin
                test_fail = test_fail + 1;
            end
        end

        // ---- Test 6: BANK register readback ----
        $display("");
        $display("--- Test 6: Register readback ---");
        begin : test_readback
            bus_write(4'h0, 8'h02);  // BANK = 2
            bus_write(4'h1, 8'h37);  // ADDR = $37
            bus_read(4'h0);
            if (bus_read_result == 8'h02) begin
                $display("  BANK: PASS ($%02h)", bus_read_result);
            end else begin
                $display("  BANK: FAIL (expected $02, got $%02h)", bus_read_result);
                test_fail = test_fail + 1;
            end
            bus_read(4'h1);
            if (bus_read_result == 8'h37) begin
                $display("  ADDR: PASS ($%02h)", bus_read_result);
                test_pass = test_pass + 1;
            end else begin
                $display("  ADDR: FAIL (expected $37, got $%02h)", bus_read_result);
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
    // Monitor
    // =========================================================================

    reg prev_gpio2 = 0;

    always @(posedge clk_25mhz) begin
        if (gpio2 && !prev_gpio2)
            $display("[%0t] GPIO2: SDRAM init done", $time);
        prev_gpio2 <= gpio2;
    end

endmodule
