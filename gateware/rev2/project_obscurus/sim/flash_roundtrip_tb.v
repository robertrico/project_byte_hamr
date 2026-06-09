`timescale 1ns/1ps
// =============================================================================
// flash_roundtrip_tb.v — Round-trip proof: erase → program → read via the
// real flash_writer + flash_reader engines sharing one spi_flash_model.
//
// Both engines share the same SPI bus wires via a simple OR/AND mux:
//   model_sck  = fw_sck  | fr_sck    (only one active at a time, other 0)
//   model_ncs  = fw_ncs  & fr_ncs    (active-low: both idle = 1&1 = 1)
//   model_mosi = fw_mosi | fr_mosi   (only one active at a time, other 0)
//   MISO fanned out to both engines.
//
// Test sequence:
//   1. Erase  sector at $400000 (maps to model addr[11:0] = $000)
//   2. Program 256 bytes at $400000 with pattern idx ^ $A5
//   3. Read back 8 bytes from $400000 via flash_reader
//   4. Verify bytes match the programmed pattern
// =============================================================================

module flash_roundtrip_tb;

    // -------------------------------------------------------------------------
    // Clock — 25 MHz (40 ns period)
    // -------------------------------------------------------------------------
    reg clk;
    initial clk = 0;
    always #20 clk = ~clk;

    // -------------------------------------------------------------------------
    // flash_writer signals
    // -------------------------------------------------------------------------
    reg         fw_rst_n       = 0;
    reg         fw_start_erase = 0;
    reg         fw_start_prog  = 0;
    reg  [23:0] fw_flash_addr  = 24'h400000;
    reg  [7:0]  fw_prog_data   = 8'd0;
    reg  [7:0]  fw_byte_idx    = 8'd0;
    wire        fw_prog_data_req;
    wire        fw_busy;
    wire        fw_done;
    wire        fw_sck, fw_ncs, fw_mosi;

    // -------------------------------------------------------------------------
    // flash_reader signals
    // -------------------------------------------------------------------------
    reg         fr_rst_n      = 0;
    reg         fr_start      = 0;
    reg  [23:0] fr_start_addr = 24'h400000;
    reg  [23:0] fr_byte_count = 24'd8;
    wire        fr_busy;
    wire        fr_done;
    wire [7:0]  fr_data_out;
    wire        fr_data_valid;
    reg         fr_data_ready = 1;
    wire        fr_sck, fr_ncs, fr_mosi;
    wire        fr_nwp, fr_nhold;

    // -------------------------------------------------------------------------
    // Shared SPI bus — OR/AND mux (safe because only one engine active at once)
    // -------------------------------------------------------------------------
    wire spi_sck  = fw_sck  | fr_sck;
    wire spi_ncs  = fw_ncs  & fr_ncs;
    wire spi_mosi = fw_mosi | fr_mosi;
    wire spi_miso;

    // -------------------------------------------------------------------------
    // DUTs
    // -------------------------------------------------------------------------
    flash_writer writer (
        .clk            (clk),
        .rst_n          (fw_rst_n),
        .start_erase    (fw_start_erase),
        .start_program  (fw_start_prog),
        .flash_addr     (fw_flash_addr),
        .prog_data      (fw_prog_data),
        .prog_data_valid(1'b1),
        .prog_data_req  (fw_prog_data_req),
        .busy           (fw_busy),
        .done           (fw_done),
        .spi_sck        (fw_sck),
        .spi_ncs        (fw_ncs),
        .spi_mosi       (fw_mosi),
        .spi_miso       (spi_miso)
    );

    flash_reader reader (
        .clk           (clk),
        .rst_n         (fr_rst_n),
        .start         (fr_start),
        .start_addr    (fr_start_addr),
        .byte_count    (fr_byte_count),
        .busy          (fr_busy),
        .done          (fr_done),
        .data_out      (fr_data_out),
        .data_valid    (fr_data_valid),
        .data_ready    (fr_data_ready),
        .flash_ncs     (fr_ncs),
        .flash_mosi    (fr_mosi),
        .flash_miso    (spi_miso),
        .flash_nwp     (fr_nwp),
        .flash_nhold   (fr_nhold),
        .flash_sck_pin (fr_sck)
    );

    spi_flash_model flash (
        .sck  (spi_sck),
        .ncs  (spi_ncs),
        .mosi (spi_mosi),
        .miso (spi_miso)
    );

    // -------------------------------------------------------------------------
    // Auto-feed prog_data: advance on prog_data_req pulse
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (fw_prog_data_req)
            fw_byte_idx <= fw_byte_idx + 8'd1;
        fw_prog_data <= fw_byte_idx ^ 8'hA5;
    end

    // -------------------------------------------------------------------------
    // Capture reader output
    // -------------------------------------------------------------------------
    reg [7:0] captured [0:7];
    integer   cap_idx = 0;
    always @(posedge clk) begin
        if (fr_data_valid && cap_idx < 8) begin
            captured[cap_idx] = fr_data_out;
            cap_idx = cap_idx + 1;
        end
    end

    // -------------------------------------------------------------------------
    // Wait-for-done helper (timeout 500 000 cycles)
    // -------------------------------------------------------------------------
    task wait_done;
        input [0:0] which;   // 0=writer, 1=reader
        integer cyc;
        begin
            cyc = 0;
            if (which == 0) begin
                while (!fw_done && cyc < 500000) begin @(posedge clk); cyc = cyc + 1; end
            end else begin
                while (!fr_done && cyc < 500000) begin @(posedge clk); cyc = cyc + 1; end
            end
            if (cyc >= 500000) begin
                $display("TIMEOUT waiting for %s done", which ? "reader" : "writer");
                $finish;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Test sequence
    // -------------------------------------------------------------------------
    integer pass_count;
    integer fail_count;
    integer k;
    reg [7:0] expected;

    initial begin
        $dumpfile("flash_roundtrip_tb.vcd");
        $dumpvars(0, flash_roundtrip_tb);

        pass_count = 0;
        fail_count = 0;

        // Reset both engines
        fw_rst_n = 0;
        fr_rst_n = 0;
        repeat (5) @(posedge clk);
        fw_rst_n = 1;
        fr_rst_n = 1;
        repeat (2) @(posedge clk);

        // =================================================================
        // Step 1: Sector erase at $400000
        // =================================================================
        $display("\n--- Step 1: Sector erase at $400000 ---");
        fw_flash_addr = 24'h400000;
        @(posedge clk); fw_start_erase = 1;
        @(posedge clk); fw_start_erase = 0;
        wait_done(0);
        $display("  Erase done.  mem[0]=$%02X (expect $FF)", flash.mem[0]);
        if (flash.mem[0] !== 8'hFF) begin
            $display("  FAIL: mem[0] not $FF after erase"); fail_count = fail_count + 1;
        end else begin
            pass_count = pass_count + 1;
        end
        repeat (5) @(posedge clk);

        // =================================================================
        // Step 2: Page program 256 bytes at $400000, pattern idx ^ $A5
        // =================================================================
        $display("--- Step 2: Page program at $400000, pattern idx^$A5 ---");
        fw_byte_idx   = 8'd0;
        fw_flash_addr = 24'h400000;
        @(posedge clk); fw_start_prog = 1;
        @(posedge clk); fw_start_prog = 0;
        wait_done(0);
        $display("  Program done. mem[0]=$%02X (expect $%02X)", flash.mem[0], 8'h00 ^ 8'hA5);
        $display("                mem[1]=$%02X (expect $%02X)", flash.mem[1], 8'h01 ^ 8'hA5);
        if (flash.mem[0] !== (8'h00 ^ 8'hA5) || flash.mem[1] !== (8'h01 ^ 8'hA5)) begin
            $display("  FAIL: backing store mismatch after program"); fail_count = fail_count + 1;
        end else begin
            pass_count = pass_count + 1;
        end
        repeat (5) @(posedge clk);

        // =================================================================
        // Step 3: Read back 8 bytes from $400000 via flash_reader
        // =================================================================
        $display("--- Step 3: Read 8 bytes from $400000 via flash_reader ---");
        cap_idx       = 0;
        fr_start_addr = 24'h400000;
        fr_byte_count = 24'd8;
        fr_data_ready = 1;
        @(posedge clk); fr_start = 1;
        @(posedge clk); fr_start = 0;
        wait_done(1);
        repeat (5) @(posedge clk);

        // =================================================================
        // Step 4: Verify
        // =================================================================
        $display("--- Step 4: Verify 8 bytes ---");
        if (cap_idx !== 8) begin
            $display("  FAIL: captured %0d bytes, expected 8", cap_idx);
            fail_count = fail_count + 1;
        end else begin
            for (k = 0; k < 8; k = k + 1) begin
                expected = k[7:0] ^ 8'hA5;
                if (captured[k] !== expected) begin
                    $display("  FAIL: byte[%0d]=$%02X, expected=$%02X", k, captured[k], expected);
                    fail_count = fail_count + 1;
                end else begin
                    pass_count = pass_count + 1;
                    $display("  PASS: byte[%0d]=$%02X", k, captured[k]);
                end
            end
        end

        // =================================================================
        // Summary
        // =================================================================
        $display("\n========================================");
        $display("  PASS=%0d  FAIL=%0d", pass_count, fail_count);
        $display("========================================");
        if (fail_count == 0)
            $display("*** ALL TESTS PASSED ***");
        else
            $display("*** FAILURES DETECTED ***");

        $finish;
    end

    // Watchdog
    initial begin
        #100_000_000;
        $display("WATCHDOG TIMEOUT");
        $finish;
    end

endmodule
