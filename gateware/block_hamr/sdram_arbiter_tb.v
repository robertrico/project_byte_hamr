// sdram_arbiter_tb.v — Testbench for sdram_arbiter + block_buffer
// Tests: boot passthrough, block read, block write, CDC timing
// SDRAM model: 512-entry x 16-bit, 3-clock CAS latency

`timescale 1ns / 1ps

module sdram_arbiter_tb;

    // ----------------------------------------------------------------
    // Clock: 25 MHz -> 40 ns period
    // ----------------------------------------------------------------
    reg clk;
    initial clk = 0;
    always #20 clk = ~clk;

    reg rst_n;

    // ----------------------------------------------------------------
    // Scoreboard
    // ----------------------------------------------------------------
    integer pass_count;
    integer fail_count;

    task check;
        input [40*8-1:0] label;
        input            condition;
        begin
            if (condition) begin
                $display("  PASS: %0s", label);
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL: %0s", label);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // ----------------------------------------------------------------
    // DUT signals
    // ----------------------------------------------------------------
    reg         boot_done;
    reg         boot_req;
    reg         boot_write;
    reg  [25:0] boot_addr;
    reg  [15:0] boot_wdata;
    wire        boot_ready;

    reg         dev_block_read_req;
    reg         dev_block_write_req;
    reg  [15:0] dev_block_num;
    wire        dev_block_ready;

    wire [8:0]  buf_addr_a;
    wire [7:0]  buf_wdata_a;
    wire        buf_we_a;
    wire [7:0]  buf_rdata_a;

    wire        sdram_req;
    wire        sdram_write;
    wire [25:0] sdram_addr;
    wire [15:0] sdram_wdata;
    reg         sdram_ready;
    reg  [15:0] sdram_rdata;
    reg         sdram_rdata_valid;

    // Block buffer Port B — directly driven by testbench
    reg  [8:0]  buf_addr_b;
    reg  [7:0]  buf_wdata_b;
    reg         buf_we_b;
    wire [7:0]  buf_rdata_b;

    // ----------------------------------------------------------------
    // Instantiate block buffer (both ports on same 25 MHz clk for sim)
    // ----------------------------------------------------------------
    block_buffer u_buf (
        .clk_a   (clk),
        .addr_a  (buf_addr_a),
        .wdata_a (buf_wdata_a),
        .we_a    (buf_we_a),
        .rdata_a (buf_rdata_a),

        .clk_b   (clk),
        .addr_b  (buf_addr_b),
        .wdata_b (buf_wdata_b),
        .we_b    (buf_we_b),
        .rdata_b (buf_rdata_b)
    );

    // ----------------------------------------------------------------
    // Instantiate DUT
    // ----------------------------------------------------------------
    sdram_arbiter u_arb (
        .clk                (clk),
        .rst_n              (rst_n),
        .boot_done          (boot_done),

        .boot_req           (boot_req),
        .boot_write         (boot_write),
        .boot_addr          (boot_addr),
        .boot_wdata         (boot_wdata),
        .boot_ready         (boot_ready),

        .dev_block_read_req (dev_block_read_req),
        .dev_block_write_req(dev_block_write_req),
        .dev_block_num      (dev_block_num),
        .dev_block_ready    (dev_block_ready),

        .buf_addr_a         (buf_addr_a),
        .buf_wdata_a        (buf_wdata_a),
        .buf_we_a           (buf_we_a),
        .buf_rdata_a        (buf_rdata_a),

        .sdram_req          (sdram_req),
        .sdram_write        (sdram_write),
        .sdram_addr         (sdram_addr),
        .sdram_wdata        (sdram_wdata),
        .sdram_ready        (sdram_ready),
        .sdram_rdata        (sdram_rdata),
        .sdram_rdata_valid  (sdram_rdata_valid)
    );

    // ----------------------------------------------------------------
    // Simple SDRAM model — 512 x 16 bit, 3-clock CAS latency
    // ----------------------------------------------------------------
    reg [15:0] sdram_mem [0:511];

    // Pipeline for read latency
    reg        rd_pipe_valid [0:2];
    reg [15:0] rd_pipe_data  [0:2];

    integer si;
    initial begin
        for (si = 0; si < 512; si = si + 1)
            sdram_mem[si] = 16'd0;
        // Pre-fill block 0 (words 0-255) with a known pattern
        for (si = 0; si < 256; si = si + 1)
            sdram_mem[si] = {si[7:0] ^ 8'hFF, si[7:0]};  // hi=~i, lo=i
    end

    // SDRAM controller behaviour
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sdram_ready       <= 1'b0;
            sdram_rdata       <= 16'd0;
            sdram_rdata_valid <= 1'b0;
            rd_pipe_valid[0]  <= 1'b0;
            rd_pipe_valid[1]  <= 1'b0;
            rd_pipe_valid[2]  <= 1'b0;
            rd_pipe_data[0]   <= 16'd0;
            rd_pipe_data[1]   <= 16'd0;
            rd_pipe_data[2]   <= 16'd0;
        end else begin
            // Shift pipeline
            rd_pipe_valid[2] <= rd_pipe_valid[1];
            rd_pipe_data[2]  <= rd_pipe_data[1];
            rd_pipe_valid[1] <= rd_pipe_valid[0];
            rd_pipe_data[1]  <= rd_pipe_data[0];
            rd_pipe_valid[0] <= 1'b0;
            rd_pipe_data[0]  <= 16'd0;

            // Output from end of pipeline
            sdram_rdata_valid <= rd_pipe_valid[2];
            sdram_rdata       <= rd_pipe_data[2];

            // Accept request
            sdram_ready <= 1'b0;
            if (sdram_req) begin
                sdram_ready <= 1'b1;
                if (sdram_write) begin
                    // Write — use lower 9 bits of word address
                    sdram_mem[sdram_addr[9:1]] <= sdram_wdata;
                end else begin
                    // Read — inject into pipeline stage 0
                    rd_pipe_valid[0] <= 1'b1;
                    rd_pipe_data[0]  <= sdram_mem[sdram_addr[9:1]];
                end
            end
        end
    end

    // ----------------------------------------------------------------
    // VCD dump
    // ----------------------------------------------------------------
    initial begin
        $dumpfile("sdram_arbiter_tb.vcd");
        $dumpvars(0, sdram_arbiter_tb);
    end

    // ----------------------------------------------------------------
    // Helper tasks
    // ----------------------------------------------------------------
    task reset;
        begin
            rst_n               <= 1'b0;
            boot_done           <= 1'b0;
            boot_req            <= 1'b0;
            boot_write          <= 1'b0;
            boot_addr           <= 26'd0;
            boot_wdata          <= 16'd0;
            dev_block_read_req  <= 1'b0;
            dev_block_write_req <= 1'b0;
            dev_block_num       <= 16'd0;
            buf_addr_b          <= 9'd0;
            buf_wdata_b         <= 8'd0;
            buf_we_b            <= 1'b0;
            repeat (4) @(posedge clk);
            rst_n <= 1'b1;
            repeat (2) @(posedge clk);
        end
    endtask

    // Boot-phase write: one 16-bit word
    task boot_write_word;
        input [25:0] addr;
        input [15:0] data;
        begin
            @(posedge clk);
            boot_addr  <= addr;
            boot_wdata <= data;
            boot_write <= 1'b1;
            boot_req   <= 1'b1;
            @(posedge clk);
            // Wait for ready
            while (!boot_ready) @(posedge clk);
            boot_req <= 1'b0;
            @(posedge clk);
        end
    endtask

    // Read buffer Port B at given address (returns after BRAM latency)
    task read_buf_b;
        input  [8:0] addr;
        output [7:0] data;
        begin
            @(posedge clk);
            buf_addr_b <= addr;
            buf_we_b   <= 1'b0;
            @(posedge clk);  // BRAM registered read latency
            @(posedge clk);
            data = buf_rdata_b;
        end
    endtask

    // Write buffer Port B
    task write_buf_b;
        input [8:0] addr;
        input [7:0] data;
        begin
            @(posedge clk);
            buf_addr_b  <= addr;
            buf_wdata_b <= data;
            buf_we_b    <= 1'b1;
            @(posedge clk);
            buf_we_b <= 1'b0;
        end
    endtask

    // ----------------------------------------------------------------
    // Main test sequence
    // ----------------------------------------------------------------
    integer i;
    reg [7:0] rd_byte;
    reg       test_ok;

    initial begin
        pass_count = 0;
        fail_count = 0;

        $display("");
        $display("==============================================");
        $display(" sdram_arbiter testbench");
        $display("==============================================");

        // ===========================================================
        // TEST 1: Boot passthrough
        // ===========================================================
        $display("");
        $display("--- Test 1: Boot passthrough ---");
        reset;

        // Write 4 words via boot port into SDRAM model (block 1 region: addr 512+)
        boot_write_word(26'd512, 16'hCAFE);
        boot_write_word(26'd514, 16'hBEEF);
        boot_write_word(26'd516, 16'hDEAD);
        boot_write_word(26'd518, 16'hF00D);

        // Verify SDRAM model contents (word addresses)
        check("boot wr word 0 = 0xCAFE", sdram_mem[256] == 16'hCAFE);
        check("boot wr word 1 = 0xBEEF", sdram_mem[257] == 16'hBEEF);
        check("boot wr word 2 = 0xDEAD", sdram_mem[258] == 16'hDEAD);
        check("boot wr word 3 = 0xF00D", sdram_mem[259] == 16'hF00D);

        // ===========================================================
        // TEST 2: Block read (SDRAM block 0 -> buffer)
        // ===========================================================
        $display("");
        $display("--- Test 2: Block read ---");

        // Switch to runtime
        boot_done <= 1'b1;
        repeat (4) @(posedge clk);

        // Request block 0 read
        dev_block_num      <= 16'd0;
        dev_block_read_req <= 1'b1;

        // Wait for completion
        repeat (20000) begin
            @(posedge clk);
            if (dev_block_ready) disable block_read_wait;
        end
        begin : block_read_wait end

        check("block read completed", dev_block_ready == 1'b1);

        // Deassert request (handshake)
        dev_block_read_req <= 1'b0;
        repeat (10) @(posedge clk);

        // Verify buffer contents — check first 8 bytes and last 2 bytes
        test_ok = 1;
        for (i = 0; i < 4; i = i + 1) begin
            // Expected: lo byte = i, hi byte = ~i
            read_buf_b(i * 2, rd_byte);
            if (rd_byte !== i[7:0]) test_ok = 0;
            read_buf_b(i * 2 + 1, rd_byte);
            if (rd_byte !== (i[7:0] ^ 8'hFF)) test_ok = 0;
        end
        check("block read: first 8 bytes correct", test_ok);

        // Check bytes 510,511 (word 255: lo=255, hi=0x00)
        read_buf_b(9'd510, rd_byte);
        test_ok = (rd_byte == 8'hFF);
        read_buf_b(9'd511, rd_byte);
        test_ok = test_ok && (rd_byte == 8'h00);
        check("block read: last 2 bytes correct", test_ok);

        // Verify dev_block_ready clears after req deasserted
        repeat (20) @(posedge clk);
        check("block_ready cleared after deassert", dev_block_ready == 1'b0);

        // ===========================================================
        // TEST 3: Block write (buffer -> SDRAM block 1)
        // ===========================================================
        $display("");
        $display("--- Test 3: Block write ---");

        // Fill buffer with pattern via Port B: byte i = i ^ 0xA5
        for (i = 0; i < 512; i = i + 1) begin
            write_buf_b(i[8:0], i[7:0] ^ 8'hA5);
        end
        repeat (4) @(posedge clk);

        // Request block 1 write
        dev_block_num       <= 16'd1;
        dev_block_write_req <= 1'b1;

        // Wait for completion
        repeat (20000) begin
            @(posedge clk);
            if (dev_block_ready) disable block_write_wait;
        end
        begin : block_write_wait end

        check("block write completed", dev_block_ready == 1'b1);

        // Deassert request
        dev_block_write_req <= 1'b0;
        repeat (10) @(posedge clk);

        // Verify SDRAM model — block 1 starts at word address 256
        // Word i should be { (i*2+1)^0xA5, (i*2)^0xA5 }
        test_ok = 1;
        begin : blk_wr_check
            reg [7:0] exp_lo, exp_hi;
            for (i = 0; i < 256; i = i + 1) begin
                exp_lo = (i[7:0] * 2)     ^ 8'hA5;
                exp_hi = (i[7:0] * 2 + 1) ^ 8'hA5;
                if (sdram_mem[256 + i][7:0] !== exp_lo)
                    test_ok = 0;
                if (sdram_mem[256 + i][15:8] !== exp_hi)
                    test_ok = 0;
            end
        end
        check("block write: all 256 words correct", test_ok);

        // ===========================================================
        // TEST 4: CDC timing — verify request crosses domains
        // ===========================================================
        $display("");
        $display("--- Test 4: CDC timing ---");

        // Re-initialize block 0 in SDRAM with a different pattern
        for (i = 0; i < 256; i = i + 1)
            sdram_mem[i] = {8'hAA, i[7:0]};

        repeat (10) @(posedge clk);

        // Pulse dev_block_read_req briefly (simulating 7 MHz assertion)
        dev_block_num      <= 16'd0;
        dev_block_read_req <= 1'b0;
        repeat (2) @(posedge clk);

        // Assert for just a few 25 MHz cycles — CDC must still catch it
        dev_block_read_req <= 1'b1;
        repeat (6) @(posedge clk);  // held for ~240 ns (> 2x 25 MHz period for safe CDC)

        // Keep it asserted and wait for ready
        repeat (20000) begin
            @(posedge clk);
            if (dev_block_ready) disable cdc_wait;
        end
        begin : cdc_wait end

        check("CDC block read completed", dev_block_ready == 1'b1);

        // Verify first two bytes
        read_buf_b(9'd0, rd_byte);
        check("CDC read byte 0 correct", rd_byte == 8'h00);
        read_buf_b(9'd1, rd_byte);
        check("CDC read byte 1 correct", rd_byte == 8'hAA);

        dev_block_read_req <= 1'b0;
        repeat (20) @(posedge clk);

        // ===========================================================
        // Summary
        // ===========================================================
        $display("");
        $display("==============================================");
        $display(" Results: %0d PASS, %0d FAIL", pass_count, fail_count);
        if (fail_count == 0)
            $display(" ALL TESTS PASSED");
        else
            $display(" SOME TESTS FAILED");
        $display("==============================================");
        $display("");

        $finish;
    end

    // ----------------------------------------------------------------
    // Watchdog
    // ----------------------------------------------------------------
    initial begin
        #5_000_000;
        $display("TIMEOUT: simulation exceeded 5 ms");
        $finish;
    end

endmodule
