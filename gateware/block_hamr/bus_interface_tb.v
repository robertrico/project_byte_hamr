`timescale 1ns / 1ps
// =============================================================================
// bus_interface_tb.v — TDD testbench for the Apple II bus register interface
// =============================================================================
// Tests the register file that replaces the SmartPort protocol stack.
// Red/Green: write tests first, then make them pass.
//
// Register Map ($C0n0 - $C0nF):
//   addr 0: STATUS (R) / COMMAND (W)
//     Read:  {ready, error, 5'b0, boot_done}
//     Write: $01=READ_BLOCK, $02=WRITE_BLOCK
//   addr 1: DATA (R/W, auto-increments buf_addr)
//   addr 2: BLOCK_LO (R/W)
//   addr 3: BLOCK_HI (R/W)
// =============================================================================

module bus_interface_tb;

    // Clock: 7.16 MHz → ~140ns period
    reg clk = 0;
    always #70 clk = ~clk;

    // DUT signals
    reg         rst_n;
    reg  [3:0]  addr;
    reg  [7:0]  data_in;
    wire [7:0]  data_out;
    reg         nDEVICE_SELECT;
    reg         R_nW;
    reg         boot_done;

    // Block buffer Port B (directly modeled here for unit testing)
    wire [8:0]  buf_addr;
    wire [7:0]  buf_wdata;
    wire        buf_we;
    reg  [7:0]  buf_rdata;

    // Block request interface
    wire        block_read_req;
    wire        block_write_req;
    wire [15:0] block_num;
    reg         block_ready;

    // Test tracking
    integer tests_run = 0;
    integer tests_passed = 0;
    integer tests_failed = 0;

    // Dirty tracking
    wire [279:0] dirty_bitmap;
    wire         has_dirty;

    // ---- DUT ----
    bus_interface dut (
        .clk              (clk),
        .rst_n            (rst_n),
        .addr             (addr),
        .data_in          (data_in),
        .data_out         (data_out),
        .nDEVICE_SELECT   (nDEVICE_SELECT),
        .R_nW             (R_nW),
        .boot_done        (boot_done),
        .buf_addr         (buf_addr),
        .buf_rdata        (buf_rdata),
        .buf_wdata        (buf_wdata),
        .buf_we           (buf_we),
        .block_read_req   (block_read_req),
        .block_write_req  (block_write_req),
        .block_num        (block_num),
        .block_ready      (block_ready),
        .dirty_bitmap     (dirty_bitmap),
        .has_dirty        (has_dirty),
        .dirty_clear_sector(5'd0),
        .dirty_clear_en   (1'b0)
    );

    // ---- Simple block buffer model (512 bytes, sync read like BRAM) ----
    reg [7:0] mock_buffer [0:511];
    integer i;

    // Synchronous read (matches real BRAM behavior — 1-cycle latency)
    always @(posedge clk) begin
        buf_rdata <= mock_buffer[buf_addr];
        if (buf_we)
            mock_buffer[buf_addr] <= buf_wdata;
    end

    // =========================================================================
    // Bus cycle tasks — simulate Apple II bus timing
    // =========================================================================
    // nDEVICE_SELECT LOW for ~4 fclk cycles (~560ns at 7MHz)
    // Address and R/W stable before nDEVICE_SELECT falls
    // Write data stable during LOW window

    task bus_write(input [3:0] a, input [7:0] d);
        begin
            @(posedge clk);
            addr = a;
            R_nW = 1'b0;
            data_in = d;
            @(posedge clk);
            nDEVICE_SELECT = 1'b0;
            repeat(4) @(posedge clk);
            nDEVICE_SELECT = 1'b1;
            repeat(2) @(posedge clk);  // inter-cycle gap
        end
    endtask

    task bus_read(input [3:0] a, output [7:0] d);
        begin
            @(posedge clk);
            addr = a;
            R_nW = 1'b1;
            @(posedge clk);
            nDEVICE_SELECT = 1'b0;
            repeat(3) @(posedge clk);  // let data settle
            d = data_out;
            @(posedge clk);
            nDEVICE_SELECT = 1'b1;
            repeat(2) @(posedge clk);  // inter-cycle gap
        end
    endtask

    // =========================================================================
    // Assertion helper
    // =========================================================================
    task check(input [255:0] name, input [7:0] actual, input [7:0] expected);
        begin
            tests_run = tests_run + 1;
            if (actual === expected) begin
                tests_passed = tests_passed + 1;
                $display("  PASS: %0s = $%02X", name, actual);
            end else begin
                tests_failed = tests_failed + 1;
                $display("  FAIL: %0s = $%02X, expected $%02X", name, actual, expected);
            end
        end
    endtask

    task check16(input [255:0] name, input [15:0] actual, input [15:0] expected);
        begin
            tests_run = tests_run + 1;
            if (actual === expected) begin
                tests_passed = tests_passed + 1;
                $display("  PASS: %0s = $%04X", name, actual);
            end else begin
                tests_failed = tests_failed + 1;
                $display("  FAIL: %0s = $%04X, expected $%04X", name, actual, expected);
            end
        end
    endtask

    task check1(input [255:0] name, input actual, input expected);
        begin
            tests_run = tests_run + 1;
            if (actual === expected) begin
                tests_passed = tests_passed + 1;
                $display("  PASS: %0s = %0b", name, actual);
            end else begin
                tests_failed = tests_failed + 1;
                $display("  FAIL: %0s = %0b, expected %0b", name, actual, expected);
            end
        end
    endtask

    // =========================================================================
    // Test sequence
    // =========================================================================
    reg [7:0] rd;

    initial begin
        $dumpfile("bus_interface_tb.vcd");
        $dumpvars(0, bus_interface_tb);

        // Initialize
        rst_n = 0;
        nDEVICE_SELECT = 1;
        R_nW = 1;
        addr = 0;
        data_in = 0;
        boot_done = 0;
        block_ready = 0;

        // Fill mock buffer with known pattern
        for (i = 0; i < 512; i = i + 1)
            mock_buffer[i] = i[7:0];

        // Reset
        repeat(4) @(posedge clk);
        rst_n = 1;
        repeat(2) @(posedge clk);

        // =================================================================
        // TEST 1: After reset, before boot, STATUS = {0,0,5'b0,0} = $00
        //   ready=0 (not ready until boot done), boot_done=0
        // =================================================================
        $display("\n--- Test 1: Reset state, before boot ---");
        bus_read(4'h0, rd);
        check("STATUS before boot", rd, 8'h00);

        // =================================================================
        // TEST 2: After boot_done, STATUS = {1,0,5'b0,1} = $81
        //   ready=1 (idle), boot_done=1
        // =================================================================
        $display("\n--- Test 2: After boot_done ---");
        boot_done = 1;
        repeat(4) @(posedge clk);
        bus_read(4'h0, rd);
        check("STATUS after boot", rd, 8'h81);

        // =================================================================
        // TEST 3: Write and verify block number
        // =================================================================
        $display("\n--- Test 3: Write block number $0042 ---");
        bus_write(4'h2, 8'h42);  // BLOCK_LO = $42
        bus_write(4'h3, 8'h00);  // BLOCK_HI = $00
        check16("block_num", block_num, 16'h0042);

        // Read back block registers
        bus_read(4'h2, rd);
        check("BLOCK_LO readback", rd, 8'h42);
        bus_read(4'h3, rd);
        check("BLOCK_HI readback", rd, 8'h00);

        // =================================================================
        // TEST 4: Issue READ command → block_read_req rises, STATUS not ready
        // =================================================================
        $display("\n--- Test 4: READ command ---");
        bus_write(4'h0, 8'h01);  // COMMAND = READ
        repeat(2) @(posedge clk);
        check1("block_read_req", block_read_req, 1'b1);
        check1("block_write_req", block_write_req, 1'b0);
        bus_read(4'h0, rd);
        check("STATUS busy", rd & 8'h80, 8'h00);  // ready=0

        // =================================================================
        // TEST 5: block_ready → STATUS returns to ready, req deasserts
        // =================================================================
        $display("\n--- Test 5: Block ready ---");
        block_ready = 1;
        repeat(4) @(posedge clk);
        block_ready = 0;
        repeat(2) @(posedge clk);
        bus_read(4'h0, rd);
        check("STATUS ready after block_ready", rd, 8'h81);
        check1("block_read_req deasserted", block_read_req, 1'b0);

        // =================================================================
        // TEST 6: Read DATA port — auto-increments buf_addr
        //   Mock buffer has pattern: buf[n] = n & $FF
        // =================================================================
        $display("\n--- Test 6: DATA read auto-increment ---");
        bus_read(4'h1, rd);
        check("DATA read byte 0", rd, 8'h00);
        bus_read(4'h1, rd);
        check("DATA read byte 1", rd, 8'h01);
        bus_read(4'h1, rd);
        check("DATA read byte 2", rd, 8'h02);
        bus_read(4'h1, rd);
        check("DATA read byte 3", rd, 8'h03);

        // =================================================================
        // TEST 7: Write DATA port — buf_we pulses, auto-increments
        // =================================================================
        $display("\n--- Test 7: DATA write ---");
        // First set block and issue read to reset buf_addr
        bus_write(4'h2, 8'h00);
        bus_write(4'h3, 8'h00);
        bus_write(4'h0, 8'h01);  // READ to reset buf_addr
        block_ready = 1;
        repeat(4) @(posedge clk);
        block_ready = 0;
        repeat(2) @(posedge clk);

        bus_write(4'h5, 8'hDE);  // Write $DE to buf[0] (addr 5 = DATA_WRITE)
        bus_write(4'h5, 8'hAD);  // Write $AD to buf[1]
        bus_write(4'h5, 8'hBE);  // Write $BE to buf[2]
        bus_write(4'h5, 8'hEF);  // Write $EF to buf[3]
        @(posedge clk);           // Let last BRAM write settle

        check("mock_buffer[0]", mock_buffer[0], 8'hDE);
        check("mock_buffer[1]", mock_buffer[1], 8'hAD);
        check("mock_buffer[2]", mock_buffer[2], 8'hBE);
        check("mock_buffer[3]", mock_buffer[3], 8'hEF);

        // =================================================================
        // TEST 8: WRITE command → block_write_req rises
        // =================================================================
        $display("\n--- Test 8: WRITE command ---");
        bus_write(4'h2, 8'h05);  // BLOCK_LO = $05
        bus_write(4'h3, 8'h01);  // BLOCK_HI = $01
        bus_write(4'h0, 8'h02);  // COMMAND = WRITE
        repeat(2) @(posedge clk);
        check1("block_write_req", block_write_req, 1'b1);
        check1("block_read_req", block_read_req, 1'b0);
        check16("block_num for write", block_num, 16'h0105);

        block_ready = 1;
        repeat(4) @(posedge clk);
        block_ready = 0;
        repeat(2) @(posedge clk);
        check1("block_write_req deasserted", block_write_req, 1'b0);

        // =================================================================
        // TEST 9: Reading non-DATA registers does NOT auto-increment
        // =================================================================
        $display("\n--- Test 9: Non-DATA reads don't auto-increment ---");
        // Issue a read to reset buf_addr
        bus_write(4'h0, 8'h01);
        block_ready = 1;
        repeat(4) @(posedge clk);
        block_ready = 0;
        repeat(2) @(posedge clk);

        bus_read(4'h0, rd);  // Read STATUS — should NOT increment
        bus_read(4'h0, rd);  // Read STATUS again
        bus_read(4'h1, rd);  // NOW read DATA — should be byte 0
        check("DATA after STATUS reads", rd, mock_buffer[0]);

        // =================================================================
        // Summary
        // =================================================================
        $display("\n========================================");
        $display("  Tests: %0d  Passed: %0d  Failed: %0d",
                 tests_run, tests_passed, tests_failed);
        $display("========================================\n");

        if (tests_failed > 0)
            $display("*** SOME TESTS FAILED ***");
        else
            $display("*** ALL TESTS PASSED ***");

        $finish;
    end

endmodule
