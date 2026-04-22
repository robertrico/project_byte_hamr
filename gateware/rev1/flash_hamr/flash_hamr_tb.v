// flash_hamr_tb.v — Integration testbench for on-demand cache architecture
//
// Tests the CDC intercept path: 6502 CMD_READ → magic toggle → firmware
// fills buffer → MAGIC_DONE → 6502 sees block_ready.
//
// Also tests non-intercepted reads (arbiter path) and the cache_enabled
// gating between the two modes.

`timescale 1ns / 1ps

module flash_hamr_tb;

    // =========================================================================
    // Clocks: 25 MHz system, ~7 MHz bus
    // =========================================================================
    reg clk_25 = 0;
    always #20 clk_25 = ~clk_25;   // 25 MHz

    reg sig_7m = 0;
    always #70 sig_7m = ~sig_7m;    // ~7.14 MHz

    // =========================================================================
    // Bus interface signals (mimics 6502 bus cycles)
    // =========================================================================
    reg [3:0]  bus_addr = 0;
    reg [7:0]  bus_wdata = 0;
    reg        bus_rnw = 1;         // 1=read, 0=write
    reg        nDEVICE_SELECT = 1;
    wire [7:0] bus_rdata;

    // =========================================================================
    // DUT: bus_interface (7 MHz domain)
    // =========================================================================
    reg por_7m_n = 0;
    reg por_25m_n = 0;

    wire [8:0]  buf_addr_b;
    wire [7:0]  buf_rdata_b, buf_wdata_b;
    wire        buf_we_b;
    wire        dev_block_read_req_raw, dev_block_write_req;
    wire [15:0] dev_block_num;
    wire        dev_block_ready;
    wire [7:0]  bus_dbg_wr_count, bus_dbg_rd_count;

    bus_interface u_bus (
        .clk(sig_7m), .rst_n(por_7m_n),
        .addr(bus_addr), .data_in(bus_wdata), .data_out(bus_rdata),
        .nDEVICE_SELECT(nDEVICE_SELECT), .R_nW(bus_rnw),
        .boot_done(1'b1), .total_blocks(16'd280),
        .buf_addr(buf_addr_b), .buf_rdata(buf_rdata_b),
        .buf_wdata(buf_wdata_b), .buf_we(buf_we_b),
        .block_read_req(dev_block_read_req_raw),
        .block_write_req(dev_block_write_req),
        .block_num(dev_block_num), .block_ready(dev_block_ready),
        .block_num_out(),
        .dbg_wr_count_out(bus_dbg_wr_count),
        .dbg_rd_count_out(bus_dbg_rd_count),
        .sd_ready(1'b1), .sd_error_in(1'b0),
        .s4d2_mounted(1'b0), .s4d2_loading(1'b0),
        .img_count(4'd0),
        .cat_rd_addr(), .cat_rd_data(8'd0),
        .img_select(), .img_name_idx(),
        .mount_request(), .sd_init_request(),
        .sd_cmd_data(), .sd_cmd_wr(),
        .s4d2_block_count(16'd0), .s4d2_is_2mg(1'b0),
        .dbg_dir_byte0(8'd0), .dbg_dir_byte8(8'd0),
        .dbg_is_fat16(1'b0),
        .mount_dbg_sectors(16'd0), .mount_dbg_state(5'd0),
        .mount_dbg_fat_entry(16'd0)
    );

    // =========================================================================
    // Cache intercept logic (mirrors flash_hamr_top.v)
    // =========================================================================
    reg cache_enabled_25 = 0;   // set by "firmware"
    reg cache_en_s1 = 0, cache_en_s2 = 0;
    always @(posedge sig_7m or negedge por_7m_n) begin
        if (!por_7m_n) begin cache_en_s1 <= 0; cache_en_s2 <= 0; end
        else begin cache_en_s1 <= cache_enabled_25; cache_en_s2 <= cache_en_s1; end
    end

    wire is_magic_block = (dev_block_num[15:1] == 15'h7FFF);
    wire is_intercepted = is_magic_block | cache_en_s2;
    wire dev_block_read_req = dev_block_read_req_raw & ~is_intercepted;
    wire magic_block_req    = dev_block_read_req_raw & is_intercepted;

    // Toggle CDC (7→25 MHz)
    reg magic_req_toggle = 0, magic_req_prev = 0;
    always @(posedge sig_7m or negedge por_7m_n) begin
        if (!por_7m_n) begin magic_req_toggle <= 0; magic_req_prev <= 0; end
        else begin
            magic_req_prev <= magic_block_req;
            if (magic_block_req & ~magic_req_prev)
                magic_req_toggle <= ~magic_req_toggle;
        end
    end

    // magic_active flag
    reg magic_active = 0;
    always @(posedge sig_7m or negedge por_7m_n) begin
        if (!por_7m_n) magic_active <= 0;
        else begin
            if (magic_block_req & ~magic_req_prev) magic_active <= 1;
            if (magic_active & ~dev_block_read_req_raw) magic_active <= 0;
        end
    end

    // MAGIC_DONE stretch (simulates cpu_soc)
    reg [6:0] mbr_stretch = 0;
    wire cpu_block_ready_25 = (mbr_stretch != 0);
    always @(posedge clk_25) begin
        if (mbr_stretch != 0) mbr_stretch <= mbr_stretch - 1;
    end

    // CDC stretch → 7 MHz
    reg cpu_br_s1 = 0, cpu_br_s2 = 0;
    always @(posedge sig_7m or negedge por_7m_n) begin
        if (!por_7m_n) begin cpu_br_s1 <= 0; cpu_br_s2 <= 0; end
        else begin cpu_br_s1 <= cpu_block_ready_25; cpu_br_s2 <= cpu_br_s1; end
    end

    // Simulated arbiter: pulse block_ready HIGH after a short delay
    // when a non-intercepted read arrives.
    reg arb_block_ready = 0;
    reg dev_read_prev = 0;
    always @(posedge clk_25) begin
        dev_read_prev <= dev_block_read_req;
        if (dev_block_read_req & ~dev_read_prev)
            arb_block_ready <= 0;  // go LOW on new request
        else if (!dev_block_read_req)
            arb_block_ready <= 0;
    end
    // Arbiter completes after ~20 clk_25 cycles
    reg [4:0] arb_delay = 0;
    always @(posedge clk_25) begin
        if (dev_block_read_req & ~dev_read_prev)
            arb_delay <= 5'd20;
        else if (arb_delay != 0)
            arb_delay <= arb_delay - 1;
        if (arb_delay == 1)
            arb_block_ready <= 1;
    end

    // Block ready mux (mirrors flash_hamr_top.v)
    assign dev_block_ready = magic_active ? cpu_br_s2 : arb_block_ready;

    // =========================================================================
    // Block buffer (dual-port BRAM)
    // =========================================================================
    wire [8:0]  buf_addr_a;
    wire [7:0]  buf_wdata_a, buf_rdata_a;
    wire        buf_we_a;

    // CPU (firmware) port A access
    reg        cpu_buf_claim = 0;
    reg [8:0]  cpu_buf_addr = 0;
    reg [7:0]  cpu_buf_wdata = 0;
    reg        cpu_buf_we = 0;

    wire [8:0]  mux_addr_a  = cpu_buf_claim ? cpu_buf_addr  : 9'd0;
    wire [7:0]  mux_wdata_a = cpu_buf_claim ? cpu_buf_wdata : 8'd0;
    wire        mux_we_a    = cpu_buf_claim ? cpu_buf_we    : 1'b0;

    block_buffer u_buf (
        .clk_a(clk_25), .addr_a(mux_addr_a), .wdata_a(mux_wdata_a),
        .we_a(mux_we_a), .rdata_a(buf_rdata_a),
        .clk_b(clk_25), .addr_b(buf_addr_b), .wdata_b(buf_wdata_b),
        .we_b(buf_we_b), .rdata_b(buf_rdata_b)
    );

    // =========================================================================
    // Bus cycle tasks
    // =========================================================================
    task bus_write(input [3:0] addr, input [7:0] data);
        begin
            @(posedge sig_7m);
            bus_addr <= addr;
            bus_wdata <= data;
            bus_rnw <= 0;
            nDEVICE_SELECT <= 0;
            @(posedge sig_7m);
            @(posedge sig_7m);
            nDEVICE_SELECT <= 1;
            @(posedge sig_7m);
            @(posedge sig_7m);
            bus_rnw <= 1;
        end
    endtask

    task bus_read(input [3:0] addr, output [7:0] data);
        begin
            @(posedge sig_7m);
            bus_addr <= addr;
            bus_rnw <= 1;
            nDEVICE_SELECT <= 0;
            @(posedge sig_7m);
            @(posedge sig_7m);
            data = bus_rdata;
            nDEVICE_SELECT <= 1;
            @(posedge sig_7m);
        end
    endtask

    task cmd_read_block(input [15:0] blk);
        begin
            bus_write(4'h2, blk[7:0]);    // BLK_LO
            bus_write(4'h3, blk[15:8]);   // BLK_HI
            bus_write(4'h0, 8'h01);       // CMD_READ
        end
    endtask

    task wait_ready(output success);
        reg [7:0] status;
        integer timeout;
        begin
            success = 0;
            for (timeout = 0; timeout < 5000; timeout = timeout + 1) begin
                bus_read(4'h0, status);
                if (status[7]) begin
                    success = 1;
                    timeout = 5000; // break
                end
            end
        end
    endtask

    // Simulate firmware: detect toggle, fill buffer, fire MAGIC_DONE
    task firmware_serve_block(input [7:0] fill_byte);
        integer i;
        begin
            // Fill buffer via Port A
            cpu_buf_claim <= 1;
            cpu_buf_addr <= 0;
            @(posedge clk_25);
            for (i = 0; i < 512; i = i + 1) begin
                cpu_buf_wdata <= fill_byte + i[7:0];
                cpu_buf_we <= 1;
                @(posedge clk_25);
                cpu_buf_we <= 0;
                cpu_buf_addr <= cpu_buf_addr + 1;
                @(posedge clk_25);
            end
            cpu_buf_claim <= 0;
            @(posedge clk_25);

            // Fire MAGIC_DONE
            mbr_stretch <= 7'd127;
            @(posedge clk_25);
        end
    endtask

    // =========================================================================
    // Test sequence
    // =========================================================================
    integer pass_count = 0;
    integer fail_count = 0;
    reg [7:0] rdata;
    reg ok;

    initial begin
        $dumpfile("flash_hamr_tb.vcd");
        $dumpvars(0, flash_hamr_tb);

        // Reset
        #200;
        por_7m_n <= 1;
        por_25m_n <= 1;
        #500;

        // ---- Test 1: Non-intercepted read (arbiter path) ----
        $display("\n--- Test 1: Non-intercepted block read ---");
        cache_enabled_25 <= 0;
        #500;

        cmd_read_block(16'd5);
        wait_ready(ok);
        if (ok) begin
            $display("PASS: Non-intercepted read completed (ready=1)");
            pass_count = pass_count + 1;
        end else begin
            $display("FAIL: Non-intercepted read timed out");
            fail_count = fail_count + 1;
        end

        // ---- Test 2: Intercepted read (cache path) ----
        $display("\n--- Test 2: Intercepted block read (cache enabled) ---");
        cache_enabled_25 <= 1;
        #500; // wait for CDC

        cmd_read_block(16'd2);

        // Wait for toggle to propagate (firmware side)
        #2000;

        // Check magic_active is set
        if (magic_active) begin
            $display("PASS: magic_active asserted after intercepted CMD_READ");
            pass_count = pass_count + 1;
        end else begin
            $display("FAIL: magic_active not set");
            fail_count = fail_count + 1;
        end

        // Firmware serves the block
        firmware_serve_block(8'hA0);

        // Wait for MAGIC_DONE to propagate
        #2000;

        // Poll ready
        wait_ready(ok);
        if (ok) begin
            $display("PASS: Intercepted read completed after MAGIC_DONE");
            pass_count = pass_count + 1;
        end else begin
            $display("FAIL: Intercepted read timed out after MAGIC_DONE");
            fail_count = fail_count + 1;
        end

        // Read first data byte — should be fill_byte (0xA0)
        bus_read(4'h1, rdata);
        if (rdata == 8'hA0) begin
            $display("PASS: First data byte = 0x%02X (expected 0xA0)", rdata);
            pass_count = pass_count + 1;
        end else begin
            $display("FAIL: First data byte = 0x%02X (expected 0xA0)", rdata);
            fail_count = fail_count + 1;
        end

        // ---- Test 3: Second intercepted read (toggle edge detection) ----
        $display("\n--- Test 3: Second intercepted read (toggle re-arms) ---");
        cmd_read_block(16'd3);
        #2000;

        if (magic_active) begin
            $display("PASS: magic_active re-armed for second request");
            pass_count = pass_count + 1;
        end else begin
            $display("FAIL: magic_active did not re-arm");
            fail_count = fail_count + 1;
        end

        firmware_serve_block(8'hB0);
        #2000;

        wait_ready(ok);
        if (ok) begin
            $display("PASS: Second intercepted read completed");
            pass_count = pass_count + 1;
        end else begin
            $display("FAIL: Second intercepted read timed out");
            fail_count = fail_count + 1;
        end

        bus_read(4'h1, rdata);
        if (rdata == 8'hB0) begin
            $display("PASS: Second block data byte = 0x%02X (expected 0xB0)", rdata);
            pass_count = pass_count + 1;
        end else begin
            $display("FAIL: Second block data byte = 0x%02X (expected 0xB0)", rdata);
            fail_count = fail_count + 1;
        end

        // ---- Test 4: Magic block ($FFFF) intercepted even without cache_en ----
        $display("\n--- Test 4: Magic block $FFFF always intercepted ---");
        cache_enabled_25 <= 0;
        #500;

        cmd_read_block(16'hFFFF);
        #2000;

        if (magic_active) begin
            $display("PASS: magic_active set for block $FFFF (cache off)");
            pass_count = pass_count + 1;
        end else begin
            $display("FAIL: magic_active not set for block $FFFF");
            fail_count = fail_count + 1;
        end

        firmware_serve_block(8'hCC);
        #2000;
        wait_ready(ok);
        if (ok) begin
            $display("PASS: Magic block $FFFF read completed");
            pass_count = pass_count + 1;
        end else begin
            $display("FAIL: Magic block $FFFF timed out");
            fail_count = fail_count + 1;
        end

        // ---- Summary ----
        $display("\n========================================");
        $display("flash_hamr_tb: %0d tests, %0d passed, %0d failed",
                 pass_count + fail_count, pass_count, fail_count);
        $display("========================================\n");

        #1000;
        $finish;
    end

endmodule
