`define SIMULATION
`timescale 1ns / 1ps
// =============================================================================
// IWM Read Path Testbench — FM rddata → x7 → buffer → data_out
// =============================================================================
// Verifies that iwm.v's latch-mode byte framing correctly decodes FM-encoded
// rddata from fm_encoder, asserting x7 for every byte without sync loss.
//
// Architecture: iwm (DUT) ← rddata ← fm_encoder ← tx_packet_buf (preloaded)
//               Apple II bus reads simulated by testbench tasks.
//
// Tests:
//   T1: INIT response (~10 wire bytes, header only)
//   T2: READBLOCK response (512-byte payload, ~596 post-PBEGIN wire bytes)
//   T3: 5× back-to-back READBLOCK (Q6 flush + REQ toggle between each)
//   T4: Worst-case data patterns ($00, $80, alternating $00/$FF)
// =============================================================================

module iwm_rddata_tb;

    // =========================================================================
    // Test Infrastructure
    // =========================================================================
    integer pass_count = 0;
    integer fail_count = 0;
    integer test_num   = 0;
    integer total_x7_pulses = 0;

    // =========================================================================
    // Clock Generation — 7.16 MHz
    // =========================================================================
    reg fclk = 0;
    always #69.8 fclk = ~fclk;

    // =========================================================================
    // IWM Bus Signals
    // =========================================================================
    reg  [3:0] addr;
    reg        nDEVICE_SELECT;
    reg        Q3;
    reg        nRES;
    reg  [7:0] data_in;
    wire [7:0] data_out;
    wire       wrdata;
    wire [3:0] phase;
    wire       _wrreq, _enbl1, _enbl2;
    reg        sense;
    wire       q7_out, q7_stable_out;
    wire       debug_x7, debug_latch_synced;

    // =========================================================================
    // FM Encoder + TX Packet Buffer
    // =========================================================================
    reg        buf_write_mode = 1;  // 1 = TB writes buffer, 0 = encoder reads
    reg  [9:0] tb_buf_addr = 0;
    reg  [7:0] tb_buf_wdata = 0;
    reg        tb_buf_we = 0;

    wire [9:0] enc_buf_addr;
    wire [7:0] buf_rdata;
    wire [9:0] buf_addr_mux = buf_write_mode ? tb_buf_addr : enc_buf_addr;
    wire       buf_we_mux   = buf_write_mode ? tb_buf_we   : 1'b0;

    reg        enc_enable    = 0;
    reg        enc_start     = 0;
    reg  [9:0] enc_byte_count = 0;
    wire       enc_busy, enc_done;
    wire       rddata_wire;

    // =========================================================================
    // DUT: IWM
    // =========================================================================
    iwm dut (
        .addr           (addr),
        .nDEVICE_SELECT (nDEVICE_SELECT),
        .fclk           (fclk),
        .Q3             (Q3),
        .nRES           (nRES),
        .data_in        (data_in),
        .data_out       (data_out),
        .wrdata         (wrdata),
        .phase          (phase),
        ._wrreq         (_wrreq),
        ._enbl1         (_enbl1),
        ._enbl2         (_enbl2),
        .sense          (sense),
        .rddata         (rddata_wire),
        .q7_out         (q7_out),
        .q7_stable_out  (q7_stable_out),
        .debug_x7       (debug_x7),
        .debug_latch_synced (debug_latch_synced)
    );

    tx_packet_buf u_tx_buf (
        .clk   (fclk),
        .addr  (buf_addr_mux),
        .wdata (tb_buf_wdata),
        .we    (buf_we_mux),
        .rdata (buf_rdata)
    );

    fm_encoder u_fm_enc (
        .clk        (fclk),
        .rst_n      (nRES),
        .enable     (enc_enable),
        .start      (enc_start),
        .byte_count (enc_byte_count),
        .rddata     (rddata_wire),
        .busy       (enc_busy),
        .done       (enc_done),
        .buf_addr   (enc_buf_addr),
        .buf_data   (buf_rdata)
    );

    // =========================================================================
    // VCD Dump
    // =========================================================================
    initial begin
        $dumpfile("iwm_rddata_tb.vcd");
        $dumpvars(0, iwm_rddata_tb);
    end

    // =========================================================================
    // IWM Register Address Constants
    // =========================================================================
    localparam [3:0] IWM_PH0_OFF   = 4'h0, IWM_PH0_ON    = 4'h1;
    localparam [3:0] IWM_PH1_OFF   = 4'h2, IWM_PH1_ON    = 4'h3;
    localparam [3:0] IWM_PH2_OFF   = 4'h4, IWM_PH2_ON    = 4'h5;
    localparam [3:0] IWM_PH3_OFF   = 4'h6, IWM_PH3_ON    = 4'h7;
    localparam [3:0] IWM_MOTOR_OFF = 4'h8, IWM_MOTOR_ON   = 4'h9;
    localparam [3:0] IWM_DRIVE_1   = 4'hA, IWM_DRIVE_2    = 4'hB;
    localparam [3:0] IWM_Q6L       = 4'hC, IWM_Q6H        = 4'hD;
    localparam [3:0] IWM_Q7L       = 4'hE, IWM_Q7H        = 4'hF;

    // =========================================================================
    // Shared Packet Arrays
    // =========================================================================
    reg [7:0]  wire_bytes [0:1023];     // Full wire bytes (preamble + packet)
    integer    wire_count;
    reg [7:0]  expected_bytes [0:700];  // Post-PBEGIN bytes (what ROM reads)
    integer    expected_count;
    reg [7:0]  rb_payload [0:511];      // Raw payload for READBLOCK build

    // =========================================================================
    // Bus Access Tasks (matching sp_validate_tb / Liron ROM timing)
    // =========================================================================

    task iwm_bus_read;
        input  [3:0] iwm_addr;
        output [7:0] rdata;
    begin
        addr = iwm_addr;
        data_in = 8'h00;
        @(posedge fclk);
        nDEVICE_SELECT = 0;
        repeat (3) @(posedge fclk);
        rdata = data_out;
        nDEVICE_SELECT = 1;
        repeat (3) @(posedge fclk);
    end
    endtask

    task iwm_bus_write;
        input [3:0] iwm_addr;
        input [7:0] wdata;
    begin
        addr = iwm_addr;
        data_in = wdata;
        @(posedge fclk);
        nDEVICE_SELECT = 0;
        repeat (4) @(posedge fclk);
        nDEVICE_SELECT = 1;
        data_in = 8'h00;
        repeat (3) @(posedge fclk);
    end
    endtask

    task iwm_touch;
        input [3:0] iwm_addr;
    begin : iwm_touch_blk
        reg [7:0] dummy;
        iwm_bus_read(iwm_addr, dummy);
    end
    endtask

    // =========================================================================
    // Mode Register Write (Liron: motor OFF, Q6H, STA Q7H, verify via Q7L)
    // =========================================================================
    task iwm_write_mode_reg;
        input [7:0] mode_val;
    begin : iwm_write_mode_blk
        reg [7:0] rdata;
        iwm_touch(IWM_MOTOR_OFF);
        iwm_touch(IWM_Q6H);
        iwm_bus_write(IWM_Q7H, mode_val);
        iwm_bus_read(IWM_Q7L, rdata);
        if ((rdata & 8'h1F) !== (mode_val & 8'h1F))
            $display("  WARN: mode reg verify: got %02h, expected %02h",
                     rdata & 8'h1F, mode_val & 8'h1F);
    end
    endtask

    // =========================================================================
    // IWM Read Data Byte — Poll Q6L for x7=1 (data ready)
    // =========================================================================
    task iwm_read_data_byte;
        output [7:0] byte_val;
        output       valid;
        input integer max_polls;
    begin : iwm_read_data_byte_blk
        reg [7:0] rdata;
        integer timeout;
        valid = 0;
        byte_val = 8'h00;
        for (timeout = 0; timeout < max_polls; timeout = timeout + 1) begin
            iwm_bus_read(IWM_Q6L, rdata);
            if (rdata[7]) begin
                byte_val = rdata;
                valid = 1;
                // Wait for x7 to clear before returning (prevents re-read)
                begin : wait_x7_clear
                    integer w;
                    for (w = 0; w < 300; w = w + 1) begin
                        @(posedge fclk);
                        if (!dut.x7)
                            disable wait_x7_clear;
                    end
                end
                disable iwm_read_data_byte_blk;
            end
        end
    end
    endtask

    // =========================================================================
    // Buffer Load Helpers
    // =========================================================================
    task buf_load_byte;
        input [9:0] address;
        input [7:0] data;
    begin
        @(posedge fclk);
        tb_buf_addr = address;
        tb_buf_wdata = data;
        tb_buf_we = 1;
        @(posedge fclk);
        tb_buf_we = 0;
    end
    endtask

    task load_wire_bytes_to_buf;
    begin : load_wb
        integer i;
        buf_write_mode = 1;
        for (i = 0; i < wire_count; i = i + 1)
            buf_load_byte(i[9:0], wire_bytes[i]);
    end
    endtask

    // =========================================================================
    // Reset
    // =========================================================================
    task reset_all;
    begin
        nRES = 0;
        enc_enable = 0;
        enc_start = 0;
        buf_write_mode = 1;
        nDEVICE_SELECT = 1;
        repeat (10) @(posedge fclk);
        nRES = 1;
        repeat (5) @(posedge fclk);
    end
    endtask

    // =========================================================================
    // Liron ROM Setup Sequence ($C800)
    // =========================================================================
    task liron_setup;
    begin : liron_setup_blk
        reg [7:0] rdata;
        integer timeout, ack_ok;

        // 1. Bus enable: PH1 ON, PH3 ON
        iwm_touch(IWM_PH1_ON);
        iwm_touch(IWM_PH3_ON);

        // 2. Write mode register = $07
        iwm_write_mode_reg(8'h07);

        // 3. Select drive 2, motor ON
        iwm_touch(IWM_DRIVE_2);
        iwm_touch(IWM_MOTOR_ON);

        // 4. Sense poll (Q6H first so Q7L reads status register)
        iwm_touch(IWM_Q6H);
        ack_ok = 0;
        for (timeout = 0; timeout < 50; timeout = timeout + 1) begin
            iwm_bus_read(IWM_Q7L, rdata);
            if (rdata[7]) begin
                ack_ok = 1;
                timeout = 50;
            end
        end
        if (!ack_ok)
            $display("  ERROR: sense poll timeout");

        // 5. Assert REQ (PH0 ON)
        iwm_touch(IWM_PH0_ON);
    end
    endtask

    // =========================================================================
    // Q6 Flush — Reset IWM read shift register for clean re-sync
    // Sets Q6=1 then reads Q6L to trigger Q6 HIGH→LOW transition.
    // =========================================================================
    task q6_flush;
    begin : q6_flush_blk
        reg [7:0] dummy;
        iwm_touch(IWM_Q6H);               // Ensure Q6=1
        iwm_bus_read(IWM_Q7L, dummy);      // Ensure Q7=0 (read mode)
        iwm_bus_read(IWM_Q6L, dummy);      // Q6: 1→0, triggers flush
    end
    endtask

    // =========================================================================
    // Encoder Control
    // =========================================================================
    task start_encoder;
        input [9:0] count;
    begin
        buf_write_mode = 0;
        enc_enable = 1;
        enc_byte_count = count;
        @(posedge fclk);
        enc_start = 1;
        @(posedge fclk);
        enc_start = 0;
    end
    endtask

    task stop_encoder;
    begin
        enc_enable = 0;
        @(posedge fclk);
    end
    endtask

    // =========================================================================
    // Packet Construction — Preamble (12 bytes, matching sp_codec E_SYNC)
    // =========================================================================
    task build_preamble;
    begin
        wire_bytes[0]  = 8'hFF; wire_bytes[1]  = 8'hFF;
        wire_bytes[2]  = 8'hFF; wire_bytes[3]  = 8'hFF;
        wire_bytes[4]  = 8'hFF; wire_bytes[5]  = 8'hFF;
        wire_bytes[6]  = 8'h3F; wire_bytes[7]  = 8'hCF;
        wire_bytes[8]  = 8'hF3; wire_bytes[9]  = 8'hFC;
        wire_bytes[10] = 8'hFF;
        wire_bytes[11] = 8'hC3;  // PBEGIN
        wire_count = 12;
    end
    endtask

    // =========================================================================
    // Build INIT Response (no payload)
    // dest=1, source=0, ptype=1(status), aux=0, stat=0(OK)
    // =========================================================================
    task build_init_response;
    begin : build_init
        reg [7:0] checksum;
        integer idx, eidx;

        build_preamble;
        idx = wire_count;
        eidx = 0;
        checksum = 8'h00;

        // Header: DEST SRC PTYPE AUX STAT (all |$80)
        wire_bytes[idx] = 8'h81; checksum = checksum ^ 8'h81; expected_bytes[eidx] = 8'h81; eidx = eidx+1; idx = idx+1;
        wire_bytes[idx] = 8'h80; checksum = checksum ^ 8'h80; expected_bytes[eidx] = 8'h80; eidx = eidx+1; idx = idx+1;
        wire_bytes[idx] = 8'h81; checksum = checksum ^ 8'h81; expected_bytes[eidx] = 8'h81; eidx = eidx+1; idx = idx+1;
        wire_bytes[idx] = 8'h80; checksum = checksum ^ 8'h80; expected_bytes[eidx] = 8'h80; eidx = eidx+1; idx = idx+1;
        wire_bytes[idx] = 8'h80; checksum = checksum ^ 8'h80; expected_bytes[eidx] = 8'h80; eidx = eidx+1; idx = idx+1;

        // Counts: ODDCNT=0, GRP7CNT=0
        wire_bytes[idx] = 8'h80; checksum = checksum ^ 8'h80; expected_bytes[eidx] = 8'h80; eidx = eidx+1; idx = idx+1;
        wire_bytes[idx] = 8'h80; checksum = checksum ^ 8'h80; expected_bytes[eidx] = 8'h80; eidx = eidx+1; idx = idx+1;

        // Checksum
        wire_bytes[idx] = checksum | 8'hAA;        expected_bytes[eidx] = checksum | 8'hAA;        eidx = eidx+1; idx = idx+1;
        wire_bytes[idx] = (checksum>>1) | 8'hAA;   expected_bytes[eidx] = (checksum>>1) | 8'hAA;   eidx = eidx+1; idx = idx+1;

        // PEND
        wire_bytes[idx] = 8'hC8; expected_bytes[eidx] = 8'hC8; eidx = eidx+1; idx = idx+1;

        wire_count = idx;
        expected_count = eidx;
    end
    endtask

    // =========================================================================
    // Build READBLOCK Response from rb_payload[0:511]
    // dest=1, source=0, ptype=2(data), aux=0, stat=0(OK), 512-byte payload
    // Caller must fill rb_payload before calling.
    // =========================================================================
    task build_readblock_from_payload;
    begin : build_rb
        reg [7:0] checksum, oddmsb, grpmsb;
        integer idx, eidx, i, j, base;

        build_preamble;
        idx = wire_count;
        eidx = 0;
        checksum = 8'h00;

        // Header: DEST=$81 SRC=$80 PTYPE=$82 AUX=$80 STAT=$80
        wire_bytes[idx]=8'h81; checksum=checksum^8'h81; expected_bytes[eidx]=8'h81; eidx=eidx+1; idx=idx+1;
        wire_bytes[idx]=8'h80; checksum=checksum^8'h80; expected_bytes[eidx]=8'h80; eidx=eidx+1; idx=idx+1;
        wire_bytes[idx]=8'h82; checksum=checksum^8'h82; expected_bytes[eidx]=8'h82; eidx=eidx+1; idx=idx+1;
        wire_bytes[idx]=8'h80; checksum=checksum^8'h80; expected_bytes[eidx]=8'h80; eidx=eidx+1; idx=idx+1;
        wire_bytes[idx]=8'h80; checksum=checksum^8'h80; expected_bytes[eidx]=8'h80; eidx=eidx+1; idx=idx+1;

        // Counts: ODDCNT=1 (512%7), GRP7CNT=73 (512/7)
        wire_bytes[idx]=8'h81; checksum=checksum^8'h81; expected_bytes[eidx]=8'h81; eidx=eidx+1; idx=idx+1;
        wire_bytes[idx]=8'hC9; checksum=checksum^8'hC9; expected_bytes[eidx]=8'hC9; eidx=eidx+1; idx=idx+1;

        // Odd section: 1 byte (rb_payload[0])
        oddmsb = 8'h80 | ((rb_payload[0] >> 1) & 8'h40);
        wire_bytes[idx] = oddmsb;                expected_bytes[eidx] = oddmsb;                eidx=eidx+1; idx=idx+1;
        wire_bytes[idx] = rb_payload[0] | 8'h80; expected_bytes[eidx] = rb_payload[0] | 8'h80; eidx=eidx+1; idx=idx+1;
        checksum = checksum ^ rb_payload[0];

        // 73 groups of 7
        for (i = 0; i < 73; i = i + 1) begin : grp_loop
            base = 1 + i * 7;
            grpmsb = 8'h80;
            for (j = 0; j < 7; j = j + 1)
                grpmsb = grpmsb | ((rb_payload[base+j] >> (1+j)) & (8'h80 >> (1+j)));
            wire_bytes[idx] = grpmsb; expected_bytes[eidx] = grpmsb; eidx=eidx+1; idx=idx+1;
            for (j = 0; j < 7; j = j + 1) begin
                wire_bytes[idx] = rb_payload[base+j] | 8'h80;
                expected_bytes[eidx] = rb_payload[base+j] | 8'h80;
                checksum = checksum ^ rb_payload[base+j];
                eidx=eidx+1; idx=idx+1;
            end
        end

        // Checksum
        wire_bytes[idx] = checksum | 8'hAA;        expected_bytes[eidx] = checksum | 8'hAA;        eidx=eidx+1; idx=idx+1;
        wire_bytes[idx] = (checksum>>1) | 8'hAA;   expected_bytes[eidx] = (checksum>>1) | 8'hAA;   eidx=eidx+1; idx=idx+1;

        // PEND
        wire_bytes[idx] = 8'hC8; expected_bytes[eidx] = 8'hC8; eidx=eidx+1; idx=idx+1;

        wire_count = idx;
        expected_count = eidx;
    end
    endtask

    // =========================================================================
    // Fill rb_payload with constant value and build READBLOCK
    // =========================================================================
    task build_readblock_response;
        input [7:0] fill_value;
    begin : build_rb_fill
        integer i;
        for (i = 0; i < 512; i = i + 1)
            rb_payload[i] = fill_value;
        build_readblock_from_payload;
    end
    endtask

    // =========================================================================
    // Read Response and Verify Against Expected Bytes
    // =========================================================================
    task read_and_verify_response;
        input [255:0] test_name;
        output integer mismatches;
    begin : read_verify
        reg [7:0] rdata;
        reg       valid;
        integer   i, sync_found, sync_reads;

        mismatches = 0;
        total_x7_pulses = 0;

        // Sync hunt: read bytes until $C3 (PBEGIN) found
        sync_found = 0;
        sync_reads = 0;
        for (i = 0; i < 60; i = i + 1) begin
            iwm_read_data_byte(rdata, valid, 50000);
            if (!valid) begin
                $display("  ERROR [%0s]: sync hunt timeout at read %0d", test_name, i);
                $display("    latchSynced=%b bitCounter=%0d bitTimer=%0d x7=%b",
                         dut.latchSynced, dut.bitCounter, dut.bitTimer, dut.x7);
                mismatches = mismatches + 1;
                disable read_verify;
            end
            total_x7_pulses = total_x7_pulses + 1;
            sync_reads = sync_reads + 1;
            if (rdata == 8'hC3) begin
                sync_found = 1;
                i = 60;
            end
        end
        if (!sync_found) begin
            $display("  ERROR [%0s]: $C3 PBEGIN not found in 60 reads", test_name);
            mismatches = mismatches + 1;
            disable read_verify;
        end

        // Read expected_count bytes and compare
        for (i = 0; i < expected_count; i = i + 1) begin
            iwm_read_data_byte(rdata, valid, 5000);
            if (!valid) begin
                $display("  ERROR [%0s]: byte %0d/%0d x7 timeout", test_name, i, expected_count);
                $display("    latchSynced=%b bitCounter=%0d bitTimer=%0d buffer=%02h",
                         dut.latchSynced, dut.bitCounter, dut.bitTimer, dut.buffer);
                mismatches = mismatches + 1;
                // Continue trying remaining bytes instead of aborting
                if (mismatches > 10) begin
                    $display("  ERROR [%0s]: too many failures, aborting", test_name);
                    disable read_verify;
                end
            end
            else begin
                total_x7_pulses = total_x7_pulses + 1;
                if (rdata !== expected_bytes[i]) begin
                    $display("  MISMATCH [%0s]: byte[%0d] got=%02h exp=%02h buf=%02h bc=%0d",
                             test_name, i, rdata, expected_bytes[i], dut.buffer, dut.bitCounter);
                    mismatches = mismatches + 1;
                    if (mismatches > 10) begin
                        $display("  ERROR [%0s]: too many mismatches, aborting", test_name);
                        disable read_verify;
                    end
                end
            end
        end

        $display("  [%0s] sync_reads=%0d data_bytes=%0d x7_total=%0d mismatches=%0d",
                 test_name, sync_reads, expected_count, total_x7_pulses, mismatches);
    end
    endtask

    // =========================================================================
    // Main Test Sequence
    // =========================================================================
    initial begin
        // Initialize all signals
        addr = 4'h0;
        nDEVICE_SELECT = 1;
        Q3 = 0;
        nRES = 0;
        data_in = 8'h00;
        sense = 1;          // Device always present
        enc_enable = 0;
        enc_start = 0;
        buf_write_mode = 1;
        tb_buf_we = 0;

        repeat (20) @(posedge fclk);
        nRES = 1;
        repeat (10) @(posedge fclk);

        $display("=============================================================");
        $display("IWM Read Path Testbench");
        $display("  FM rddata -> IWM shift register -> x7 -> buffer -> data_out");
        $display("=============================================================");

        // =================================================================
        // T1: Single INIT response (no payload, ~10 post-PBEGIN bytes)
        // =================================================================
        begin : test1
            integer mm;
            test_num = 1;
            $display("\nT1: INIT response (header only, no payload)");
            $display("--------------------------------------------");

            reset_all;
            build_init_response;
            $display("  Wire bytes=%0d, expected post-PBEGIN=%0d", wire_count, expected_count);
            load_wire_bytes_to_buf;

            liron_setup;
            q6_flush;
            start_encoder(wire_count[9:0]);

            read_and_verify_response("T1_INIT", mm);

            stop_encoder;
            buf_write_mode = 1;

            if (mm == 0) begin
                $display("  PASS: T1");
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL: T1 -- %0d mismatches", mm);
                fail_count = fail_count + 1;
            end
        end

        // =================================================================
        // T2: READBLOCK response (512 bytes, fill=$DE)
        // =================================================================
        begin : test2
            integer mm;
            test_num = 2;
            $display("\nT2: READBLOCK response (512 bytes, fill=$DE)");
            $display("--------------------------------------------");

            reset_all;
            build_readblock_response(8'hDE);
            $display("  Wire bytes=%0d, expected post-PBEGIN=%0d", wire_count, expected_count);
            load_wire_bytes_to_buf;

            liron_setup;
            q6_flush;
            start_encoder(wire_count[9:0]);

            read_and_verify_response("T2_RB", mm);

            stop_encoder;
            buf_write_mode = 1;

            if (mm == 0) begin
                $display("  PASS: T2");
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL: T2 -- %0d mismatches", mm);
                fail_count = fail_count + 1;
            end
        end

        // =================================================================
        // T3: Back-to-back 5x READBLOCK (Q6 flush + REQ between each)
        // No full reset — tests persistent IWM state across reads.
        // =================================================================
        begin : test3
            integer mm, total_mm, resp_num;
            test_num = 3;
            $display("\nT3: 5x back-to-back READBLOCK (fill=$AB)");
            $display("--------------------------------------------");

            total_mm = 0;

            reset_all;
            build_readblock_response(8'hAB);
            $display("  Wire bytes=%0d, expected post-PBEGIN=%0d (per response)", wire_count, expected_count);
            load_wire_bytes_to_buf;

            liron_setup;

            for (resp_num = 0; resp_num < 5; resp_num = resp_num + 1) begin
                $display("  --- Response %0d/5 ---", resp_num + 1);

                // Between responses: deassert/reassert REQ
                if (resp_num > 0) begin
                    iwm_touch(IWM_PH0_OFF);
                    repeat (100) @(posedge fclk);  // ~14us inter-response gap
                    iwm_touch(IWM_PH0_ON);
                end

                q6_flush;
                start_encoder(wire_count[9:0]);

                read_and_verify_response("T3_B2B", mm);

                stop_encoder;

                if (mm != 0) begin
                    $display("  FAIL: T3 response %0d -- %0d mismatches", resp_num + 1, mm);
                end
                total_mm = total_mm + mm;
            end

            buf_write_mode = 1;

            if (total_mm == 0) begin
                $display("  PASS: T3 -- all 5 responses verified");
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL: T3 -- %0d total mismatches across 5 responses", total_mm);
                fail_count = fail_count + 1;
            end
        end

        // =================================================================
        // T4: Worst-case data patterns
        // =================================================================
        begin : test4
            integer mm, total_mm;
            integer i;
            test_num = 4;
            $display("\nT4: Worst-case data patterns");
            $display("--------------------------------------------");

            total_mm = 0;

            // T4a: $00 fill (7 consecutive zero bits per wire byte)
            $display("  --- T4a: fill=$00 ---");
            reset_all;
            build_readblock_response(8'h00);
            load_wire_bytes_to_buf;
            liron_setup;
            q6_flush;
            start_encoder(wire_count[9:0]);
            read_and_verify_response("T4a_$00", mm);
            stop_encoder;
            buf_write_mode = 1;
            if (mm == 0)
                $display("  PASS: T4a");
            else begin
                $display("  FAIL: T4a -- %0d mismatches", mm);
                total_mm = total_mm + mm;
            end

            // T4b: $80 fill (MSB=1 then 7 zeros — worst-case zero-insertion)
            $display("  --- T4b: fill=$80 ---");
            reset_all;
            build_readblock_response(8'h80);
            load_wire_bytes_to_buf;
            liron_setup;
            q6_flush;
            start_encoder(wire_count[9:0]);
            read_and_verify_response("T4b_$80", mm);
            stop_encoder;
            buf_write_mode = 1;
            if (mm == 0)
                $display("  PASS: T4b");
            else begin
                $display("  FAIL: T4b -- %0d mismatches", mm);
                total_mm = total_mm + mm;
            end

            // T4c: Alternating $00/$FF (maximum bit-pattern diversity)
            $display("  --- T4c: alternating $00/$FF ---");
            reset_all;
            for (i = 0; i < 512; i = i + 1)
                rb_payload[i] = (i[0]) ? 8'hFF : 8'h00;
            build_readblock_from_payload;
            load_wire_bytes_to_buf;
            liron_setup;
            q6_flush;
            start_encoder(wire_count[9:0]);
            read_and_verify_response("T4c_alt", mm);
            stop_encoder;
            buf_write_mode = 1;
            if (mm == 0)
                $display("  PASS: T4c");
            else begin
                $display("  FAIL: T4c -- %0d mismatches", mm);
                total_mm = total_mm + mm;
            end

            if (total_mm == 0) begin
                $display("  PASS: T4 -- all worst-case patterns verified");
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL: T4 -- %0d total mismatches", total_mm);
                fail_count = fail_count + 1;
            end
        end

        // =================================================================
        // T5: Gap sweep — realistic delay between Q6 flush and rddata
        // Simulates the real hardware gap (~64 fclk) and sweeps ±10 fclk
        // around it to find which gap values produce framing errors.
        // This is the test that exposes the garbage zeros bug.
        // =================================================================
        begin : test5
            integer mm, total_mm, gap, gap_fail_count;
            test_num = 5;
            $display("\nT5: Q6-flush-to-rddata gap sweep (40..90 fclk)");
            $display("--------------------------------------------");

            total_mm = 0;
            gap_fail_count = 0;

            // Build a READBLOCK with known data
            reset_all;
            build_readblock_response(8'hDE);
            load_wire_bytes_to_buf;

            // Sweep gap from 40 to 90 fclk (real hardware is ~60-70)
            for (gap = 40; gap <= 90; gap = gap + 1) begin

                // Re-setup IWM for each gap value (clean state)
                reset_all;
                load_wire_bytes_to_buf;
                liron_setup;

                // Q6 flush
                q6_flush;

                // INSERT THE GAP — this is what real hardware has
                repeat (gap) @(posedge fclk);

                // Start encoder AFTER the gap
                start_encoder(wire_count[9:0]);

                // Read and verify
                read_and_verify_response("T5_GAP", mm);

                stop_encoder;
                buf_write_mode = 1;

                if (mm != 0) begin
                    $display("  FAIL: gap=%0d fclk — %0d mismatches", gap, mm);
                    gap_fail_count = gap_fail_count + 1;
                    total_mm = total_mm + mm;
                end
            end

            $display("  Gap sweep: %0d/51 gaps passed, %0d failed",
                     51 - gap_fail_count, gap_fail_count);

            if (gap_fail_count == 0) begin
                $display("  PASS: T5 -- all gap values produce correct framing");
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL: T5 -- %0d gap values produce framing errors", gap_fail_count);
                fail_count = fail_count + 1;
            end
        end

        // =================================================================
        // T6: Gap sweep with back-to-back responses
        // Same as T5 but sends 3 responses per gap value to catch
        // state that persists across responses.
        // =================================================================
        begin : test6
            integer mm, total_mm, gap, resp_num, gap_fail_count;
            test_num = 6;
            $display("\nT6: Gap sweep + back-to-back (3 responses per gap)");
            $display("--------------------------------------------");

            total_mm = 0;
            gap_fail_count = 0;

            build_readblock_response(8'h42);

            // Test a few critical gap values
            for (gap = 55; gap <= 75; gap = gap + 5) begin

                reset_all;
                load_wire_bytes_to_buf;
                liron_setup;

                begin : gap_b2b
                    integer resp_mm;
                    resp_mm = 0;

                    for (resp_num = 0; resp_num < 3; resp_num = resp_num + 1) begin
                        if (resp_num > 0) begin
                            iwm_touch(IWM_PH0_OFF);
                            repeat (100) @(posedge fclk);
                            iwm_touch(IWM_PH0_ON);
                        end

                        q6_flush;
                        repeat (gap) @(posedge fclk);
                        start_encoder(wire_count[9:0]);

                        read_and_verify_response("T6_B2B", mm);

                        stop_encoder;

                        if (mm != 0) begin
                            $display("  FAIL: gap=%0d resp=%0d — %0d mismatches",
                                     gap, resp_num + 1, mm);
                            resp_mm = resp_mm + mm;
                        end
                    end

                    if (resp_mm != 0) begin
                        gap_fail_count = gap_fail_count + 1;
                        total_mm = total_mm + resp_mm;
                    end else begin
                        $display("  gap=%0d: all 3 responses OK", gap);
                    end
                end

                buf_write_mode = 1;
            end

            if (gap_fail_count == 0) begin
                $display("  PASS: T6 -- all gap/response combos verified");
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL: T6 -- %0d gap values had failures", gap_fail_count);
                fail_count = fail_count + 1;
            end
        end

        // =================================================================
        // Summary
        // =================================================================
        $display("\n=============================================================");
        $display("RESULTS: %0d PASS, %0d FAIL out of %0d tests",
                 pass_count, fail_count, pass_count + fail_count);
        if (fail_count == 0)
            $display("ALL TESTS PASSED");
        else
            $display("*** FAILURES DETECTED ***");
        $display("=============================================================");

        #1000;
        $finish;
    end

    // =========================================================================
    // Watchdog — 500ms timeout
    // =========================================================================
    initial begin
        #5_000_000_000;
        $display("\nERROR: Simulation watchdog timeout at %0t", $time);
        $display("  test_num=%0d latchSynced=%b bitCounter=%0d x7=%b",
                 test_num, dut.latchSynced, dut.bitCounter, dut.x7);
        $finish;
    end

endmodule
