`timescale 1ns/1ps
// cflash_restore_tb.v - unit test for cflash_restore FSM.
// The reader-model below mimics flash_reader.v's REAL streaming handshake:
//   - on fr_start (1-cycle pulse) it latches addr+count, asserts busy
//   - it emits fr_data_valid for exactly ONE cycle per byte (gated by fr_ready)
//   - fr_done pulses exactly ONE cycle AFTER the final fr_data_valid
module cflash_restore_tb;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0, boot_done=0;
    wire fr_start; wire [23:0] fr_addr, fr_count; wire fr_ready;
    reg fr_busy=0, fr_done=0, fr_dvalid=0; reg [7:0] fr_data=0;
    wire [12:0] r_laddr; wire [7:0] r_ldata; wire r_lwr;
    wire restore_done, restore_valid; wire [7:0] svc_count_o;
    integer errors=0; reg [7:0] cap0200; reg wrote=0;
    integer wd=0;

    cflash_restore dut(.clk(clk), .rst_n(rst_n), .boot_done(boot_done),
        .fr_start(fr_start), .fr_start_addr(fr_addr), .fr_byte_count(fr_count),
        .fr_data_ready(fr_ready), .fr_busy(fr_busy), .fr_done(fr_done),
        .fr_data_out(fr_data), .fr_data_valid(fr_dvalid),
        .laddr(r_laddr), .ldata(r_ldata), .lwr(r_lwr),
        .restore_done(restore_done), .restore_valid(restore_valid), .svc_count(svc_count_o));

    // capture BRAM port-B writes
    always @(posedge clk) if (r_lwr) begin
        wrote<=1; if (r_laddr==13'h0200) cap0200<=r_ldata;
    end

    // ---- reader model state ----
    reg good_magic;
    localparam M_IDLE=0, M_GAP=1, M_STREAM=2, M_FIN=3;
    reg [1:0] mst; reg [23:0] m_cnt, m_idx, m_addr; reg [3:0] gap;

    function [7:0] byteval;
        input [23:0] addr; input [23:0] idx;
        begin
            if (addr == 24'h400000) begin // HEADER page
                case (idx)
                    24'd0: byteval = good_magic ? 8'h43 : 8'h58; // 'C' or 'X'
                    24'd1: byteval = 8'h52;                      // 'R'
                    24'd2: byteval = 8'h01;                      // version
                    24'd3: byteval = 8'h07;                      // svc_count
                    default: byteval = 8'h00;
                endcase
            end else begin                  // DATA region 0x400100+
                byteval = (24'h200 + idx) & 24'hFF;
            end
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fr_busy<=0; fr_done<=0; fr_dvalid<=0; fr_data<=0;
            mst<=M_IDLE; m_cnt<=0; m_idx<=0; m_addr<=0; gap<=0;
        end else begin
            fr_done<=0; fr_dvalid<=0;
            case (mst)
                M_IDLE: if (fr_start) begin
                    m_addr<=fr_addr; m_cnt<=fr_count; m_idx<=0;
                    fr_busy<=1; gap<=3; mst<=M_GAP;       // model cmd/addr latency
                end
                M_GAP: if (gap==0) mst<=M_STREAM; else gap<=gap-1;
                M_STREAM: if (fr_ready) begin
                    fr_data   <= byteval(m_addr, m_idx);
                    fr_dvalid <= 1'b1;
                    if (m_idx == m_cnt-1) mst<=M_FIN;
                    else begin m_idx<=m_idx+1; gap<=1; mst<=M_GAP; end
                end
                M_FIN: begin fr_done<=1; fr_busy<=0; mst<=M_IDLE; end
                default: mst<=M_IDLE;
            endcase
        end
    end

    // watchdog
    always @(posedge clk) begin
        wd<=wd+1;
        if (wd>500000) begin $display("FAIL watchdog: FSM hung"); $finish; end
    end

    initial begin
        rst_n=0; good_magic=1; #40; rst_n=1; #20;
        @(posedge clk); boot_done=1;
        wait(restore_done);
        if (!restore_valid) begin errors=errors+1; $display("FAIL valid magic not restored"); end
        else if (!wrote) begin errors=errors+1; $display("FAIL no BRAM writes on valid"); end
        else $display("PASS restore valid-magic -> BRAM written, valid=1 (svc_count=%0d cap0200=%02h)", svc_count_o, cap0200);
        // bad magic:
        rst_n=0; good_magic=0; wrote=0; #40; rst_n=1; #20;
        @(posedge clk); boot_done=1; wait(restore_done);
        if (restore_valid) begin errors=errors+1; $display("FAIL bad magic restored"); end
        else if (wrote) begin errors=errors+1; $display("FAIL bad magic wrote BRAM"); end
        else $display("PASS restore bad-magic -> no write, valid=0");
        if (errors==0) $display("PASS cflash_restore"); else $display("FAIL cflash_restore %0d",errors);
        $finish;
    end
endmodule
