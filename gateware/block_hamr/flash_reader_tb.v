`define SIMULATION
`timescale 1ns / 1ps

// ===================================================================
// SPI Flash Behavioral Model (IS25LP128F subset)
// ===================================================================
// - Responds to READ command (0x03) only
// - 1KB internal memory initialized with address == data pattern
// - Samples MOSI on SPI clock rising edge
// - Drives MISO on SPI clock falling edge
// ===================================================================
module spi_flash_model (
    input  wire ncs,
    input  wire sck,
    input  wire mosi,
    output reg  miso
);

    reg [7:0] flash_mem [0:1023];

    // Command/address reception
    reg [7:0]  cmd_reg;
    reg [23:0] addr_reg;
    reg [5:0]  bit_cnt;      // counts bits received (0..31 for cmd+addr)
    reg        cmd_phase;    // 1 = receiving cmd+addr, 0 = outputting data
    reg [7:0]  data_byte;    // current byte being shifted out
    reg [2:0]  out_bit_cnt;  // which bit of data_byte to output next
    reg [23:0] read_addr;    // current read address

    integer i;

    initial begin
        for (i = 0; i < 1024; i = i + 1)
            flash_mem[i] = i[7:0];
        miso    = 1'bz;
        bit_cnt = 6'd0;
        cmd_phase = 1'b1;
        cmd_reg   = 8'd0;
        addr_reg  = 24'd0;
    end

    // Detect CS assertion / deassertion
    always @(posedge ncs) begin
        // CS deasserted — reset state
        bit_cnt   <= 6'd0;
        cmd_phase <= 1'b1;
        miso      <= 1'bz;
    end

    always @(negedge ncs) begin
        bit_cnt   <= 6'd0;
        cmd_phase <= 1'b1;
        cmd_reg   <= 8'd0;
        addr_reg  <= 24'd0;
    end

    // Sample MOSI on SCK rising edge
    always @(posedge sck) begin
        if (!ncs && cmd_phase) begin
            if (bit_cnt < 6'd8) begin
                cmd_reg <= {cmd_reg[6:0], mosi};
            end else if (bit_cnt < 6'd32) begin
                addr_reg <= {addr_reg[22:0], mosi};
            end
            bit_cnt <= bit_cnt + 6'd1;

            if (bit_cnt == 6'd31) begin
                // Command + address fully received
                cmd_phase <= 1'b0;
                read_addr <= {addr_reg[22:0], mosi}; // include this last bit
                out_bit_cnt <= 3'd0;
            end
        end
    end

    // Drive MISO on SCK falling edge during data phase
    always @(negedge sck) begin
        if (!ncs && !cmd_phase) begin
            if (out_bit_cnt == 3'd0) begin
                // Load next byte
                data_byte <= flash_mem[read_addr[9:0]]; // wrap at 1KB
            end

            // Drive current bit (MSB first)
            miso <= (out_bit_cnt == 3'd0) ?
                    flash_mem[read_addr[9:0]][7] :
                    data_byte[7 - out_bit_cnt];

            out_bit_cnt <= out_bit_cnt + 3'd1;
            if (out_bit_cnt == 3'd7) begin
                read_addr   <= read_addr + 24'd1;
                out_bit_cnt <= 3'd0;
            end
        end
    end

endmodule


// ===================================================================
// Testbench
// ===================================================================
module flash_reader_tb;

    // ---------------------------------------------------------------
    // Clock generation: 25 MHz = 40 ns period
    // ---------------------------------------------------------------
    reg clk;
    initial clk = 0;
    always #20 clk = ~clk;

    // ---------------------------------------------------------------
    // DUT signals
    // ---------------------------------------------------------------
    reg         rst_n;
    reg         start;
    reg  [23:0] start_addr;
    reg  [23:0] byte_count;
    wire        busy;
    wire        done;
    wire [7:0]  data_out;
    wire        data_valid;
    reg         data_ready;
    wire        flash_ncs;
    wire        flash_mosi;
    wire        flash_miso;
    wire        flash_nwp;
    wire        flash_nhold;
    wire        flash_sck_pin;

    // ---------------------------------------------------------------
    // DUT instantiation
    // ---------------------------------------------------------------
    flash_reader dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .start         (start),
        .start_addr    (start_addr),
        .byte_count    (byte_count),
        .busy          (busy),
        .done          (done),
        .data_out      (data_out),
        .data_valid    (data_valid),
        .data_ready    (data_ready),
        .flash_ncs     (flash_ncs),
        .flash_mosi    (flash_mosi),
        .flash_miso    (flash_miso),
        .flash_nwp     (flash_nwp),
        .flash_nhold   (flash_nhold),
        .flash_sck_pin (flash_sck_pin)
    );

    // ---------------------------------------------------------------
    // SPI flash model
    // ---------------------------------------------------------------
    spi_flash_model flash (
        .ncs  (flash_ncs),
        .sck  (flash_sck_pin),
        .mosi (flash_mosi),
        .miso (flash_miso)
    );

    // ---------------------------------------------------------------
    // Test infrastructure
    // ---------------------------------------------------------------
    integer pass_count;
    integer fail_count;
    integer test_num;
    integer byte_idx;
    reg [7:0] captured_bytes [0:511];
    integer capture_idx;
    integer timeout;

    // Task: wait for done with timeout
    task wait_for_done;
        input integer max_cycles;
        integer cyc;
        begin
            cyc = 0;
            while (!done && cyc < max_cycles) begin
                @(posedge clk);
                cyc = cyc + 1;
            end
            if (cyc >= max_cycles) begin
                $display("FAIL: Timeout waiting for done (after %0d cycles)", max_cycles);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // Capture data_valid bytes into captured_bytes array
    always @(posedge clk) begin
        if (data_valid) begin
            captured_bytes[capture_idx] = data_out;
            capture_idx = capture_idx + 1;
        end
    end

    // ---------------------------------------------------------------
    // Main test sequence
    // ---------------------------------------------------------------
    initial begin
        $dumpfile("flash_reader_tb.vcd");
        $dumpvars(0, flash_reader_tb);

        pass_count = 0;
        fail_count = 0;
        capture_idx = 0;

        // Reset
        rst_n      = 0;
        start      = 0;
        start_addr = 24'd0;
        byte_count = 24'd0;
        data_ready = 1;

        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (2) @(posedge clk);

        // =============================================================
        // Test 1: Read 1 byte from address 0
        // =============================================================
        $display("\n--- Test 1: Read 1 byte from addr 0 ---");
        test_num    = 1;
        capture_idx = 0;
        start_addr  = 24'h000000;
        byte_count  = 24'd1;
        data_ready  = 1;

        @(posedge clk);
        start = 1;
        @(posedge clk);
        start = 0;

        wait_for_done(5000);

        if (capture_idx == 1 && captured_bytes[0] == 8'h00) begin
            $display("PASS: Test 1 — read byte 0x%02x from addr 0 (expected 0x00)",
                     captured_bytes[0]);
            pass_count = pass_count + 1;
        end else begin
            $display("FAIL: Test 1 — got %0d bytes, first=0x%02x (expected 1 byte, 0x00)",
                     capture_idx, captured_bytes[0]);
            fail_count = fail_count + 1;
        end

        repeat (10) @(posedge clk);

        // =============================================================
        // Test 2: Read 4 bytes from address 0x10
        // =============================================================
        $display("\n--- Test 2: Read 4 bytes from addr 0x10 ---");
        test_num    = 2;
        capture_idx = 0;
        start_addr  = 24'h000010;
        byte_count  = 24'd4;
        data_ready  = 1;

        @(posedge clk);
        start = 1;
        @(posedge clk);
        start = 0;

        wait_for_done(5000);

        begin : test2_check
            reg test2_pass;
            test2_pass = 1;
            if (capture_idx != 4) begin
                test2_pass = 0;
                $display("FAIL: Test 2 — got %0d bytes, expected 4", capture_idx);
                fail_count = fail_count + 1;
            end else begin
                for (byte_idx = 0; byte_idx < 4; byte_idx = byte_idx + 1) begin
                    if (captured_bytes[byte_idx] !== (8'h10 + byte_idx[7:0])) begin
                        test2_pass = 0;
                        $display("FAIL: Test 2 — byte[%0d]=0x%02x, expected 0x%02x",
                                 byte_idx, captured_bytes[byte_idx],
                                 8'h10 + byte_idx[7:0]);
                        fail_count = fail_count + 1;
                    end
                end
                if (test2_pass) begin
                    $display("PASS: Test 2 — read 4 bytes: 0x%02x 0x%02x 0x%02x 0x%02x",
                             captured_bytes[0], captured_bytes[1],
                             captured_bytes[2], captured_bytes[3]);
                    pass_count = pass_count + 1;
                end
            end
        end

        repeat (10) @(posedge clk);

        // =============================================================
        // Test 3: Read 512 bytes from address 0
        // =============================================================
        $display("\n--- Test 3: Read 512 bytes from addr 0 ---");
        test_num    = 3;
        capture_idx = 0;
        start_addr  = 24'h000000;
        byte_count  = 24'd512;
        data_ready  = 1;

        @(posedge clk);
        start = 1;
        @(posedge clk);
        start = 0;

        wait_for_done(100000);

        begin : test3_check
            reg test3_pass;
            test3_pass = 1;
            if (capture_idx != 512) begin
                test3_pass = 0;
                $display("FAIL: Test 3 — got %0d bytes, expected 512", capture_idx);
                fail_count = fail_count + 1;
            end else begin
                // Check first byte
                if (captured_bytes[0] !== 8'h00) begin
                    test3_pass = 0;
                    $display("FAIL: Test 3 — first byte=0x%02x, expected 0x00",
                             captured_bytes[0]);
                    fail_count = fail_count + 1;
                end
                // Check last byte: addr 511 = 0xFF (wraps in 8 bits)
                if (captured_bytes[511] !== 8'hFF) begin
                    test3_pass = 0;
                    $display("FAIL: Test 3 — last byte=0x%02x, expected 0xFF",
                             captured_bytes[511]);
                    fail_count = fail_count + 1;
                end
                // Spot check middle
                if (captured_bytes[128] !== 8'h80) begin
                    test3_pass = 0;
                    $display("FAIL: Test 3 — byte[128]=0x%02x, expected 0x80",
                             captured_bytes[128]);
                    fail_count = fail_count + 1;
                end
                if (test3_pass) begin
                    $display("PASS: Test 3 — read 512 bytes, first=0x%02x mid=0x%02x last=0x%02x",
                             captured_bytes[0], captured_bytes[128], captured_bytes[511]);
                    pass_count = pass_count + 1;
                end
            end
        end

        repeat (10) @(posedge clk);

        // =============================================================
        // Test 4: Backpressure — hold data_ready low mid-transfer
        // =============================================================
        $display("\n--- Test 4: Backpressure test ---");
        test_num    = 4;
        capture_idx = 0;
        start_addr  = 24'h000000;
        byte_count  = 24'd8;
        data_ready  = 1;

        @(posedge clk);
        start = 1;
        @(posedge clk);
        start = 0;

        // Wait for 2 bytes to come through, then stall
        begin : test4_wait
            integer t4_cyc;
            t4_cyc = 0;
            while (capture_idx < 2 && t4_cyc < 5000) begin
                @(posedge clk);
                t4_cyc = t4_cyc + 1;
            end
        end

        // Deassert data_ready for 10 clocks
        data_ready = 0;
        repeat (10) @(posedge clk);
        data_ready = 1;

        wait_for_done(50000);

        begin : test4_check
            reg test4_pass;
            test4_pass = 1;
            if (capture_idx != 8) begin
                test4_pass = 0;
                $display("FAIL: Test 4 — got %0d bytes, expected 8", capture_idx);
                fail_count = fail_count + 1;
            end else begin
                for (byte_idx = 0; byte_idx < 8; byte_idx = byte_idx + 1) begin
                    if (captured_bytes[byte_idx] !== byte_idx[7:0]) begin
                        test4_pass = 0;
                        $display("FAIL: Test 4 — byte[%0d]=0x%02x, expected 0x%02x",
                                 byte_idx, captured_bytes[byte_idx], byte_idx[7:0]);
                        fail_count = fail_count + 1;
                    end
                end
                if (test4_pass) begin
                    $display("PASS: Test 4 — backpressure: all 8 bytes correct after stall");
                    pass_count = pass_count + 1;
                end
            end
        end

        repeat (10) @(posedge clk);

        // =============================================================
        // Test 5: Busy/done signal behavior
        // =============================================================
        $display("\n--- Test 5: Busy/done signals ---");
        test_num    = 5;
        capture_idx = 0;
        start_addr  = 24'h000000;
        byte_count  = 24'd2;
        data_ready  = 1;

        // Verify idle state
        begin : test5_block
            reg test5_pass;
            test5_pass = 1;

            if (busy !== 1'b0) begin
                test5_pass = 0;
                $display("FAIL: Test 5 — busy should be 0 before start");
                fail_count = fail_count + 1;
            end

            @(posedge clk);
            start = 1;
            @(posedge clk);
            start = 0;

            // busy should be high now
            @(posedge clk);
            if (busy !== 1'b1) begin
                test5_pass = 0;
                $display("FAIL: Test 5 — busy should be 1 after start");
                fail_count = fail_count + 1;
            end

            // Wait for done
            begin : test5_wait_done
                integer t5_cyc;
                t5_cyc = 0;
                while (!done && t5_cyc < 5000) begin
                    @(posedge clk);
                    t5_cyc = t5_cyc + 1;
                end
                if (t5_cyc >= 5000) begin
                    test5_pass = 0;
                    $display("FAIL: Test 5 — timeout waiting for done");
                    fail_count = fail_count + 1;
                end
            end

            // On the cycle done is asserted, busy should go low
            if (done && busy !== 1'b0) begin
                test5_pass = 0;
                $display("FAIL: Test 5 — busy should be 0 when done asserts");
                fail_count = fail_count + 1;
            end

            @(posedge clk);
            // done should be deasserted after one cycle
            if (done !== 1'b0) begin
                test5_pass = 0;
                $display("FAIL: Test 5 — done should be single-cycle pulse");
                fail_count = fail_count + 1;
            end

            if (test5_pass) begin
                $display("PASS: Test 5 — busy/done signals behave correctly");
                pass_count = pass_count + 1;
            end
        end

        // =============================================================
        // Summary
        // =============================================================
        repeat (10) @(posedge clk);
        $display("\n====================================");
        $display("  Results: %0d PASSED, %0d FAILED", pass_count, fail_count);
        $display("====================================\n");

        if (fail_count > 0)
            $display("*** SOME TESTS FAILED ***");
        else
            $display("*** ALL TESTS PASSED ***");

        $finish;
    end

endmodule
