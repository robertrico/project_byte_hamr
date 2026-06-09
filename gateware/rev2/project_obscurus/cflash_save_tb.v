`timescale 1ns/1ps
// cflash_save_tb.v - unit test for the SAVE FSM (BRAM -> SPI flash registry sector).
// Models flash_writer's REAL handshake (read flash_writer.v):
//   - busy high from start until done; done is a 1-cycle pulse.
//   - PROGRAM requires prog_data_valid HIGH at each byte start (else it stalls);
//     it pulses prog_data_req once per byte (256x) as a "byte consumed, advance" ACK.
//   - Each start_program programs EXACTLY one 256-byte page, terminated by its own
//     done. Caller must wait for done before the next start_program.
// Load-bearing asserts (DO NOT WEAKEN):
//   (1) no emitted flash_addr < 0x400000  (min_addr >= 0x400000)
//   (2) the header page at 0x400000 gets magic "CR" ($43,$52)
//   plus: an erase happened.
module cflash_save_tb;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0, save_start=0;
    reg [7:0] svc_count=8'd2;
    wire [12:0] laddr; reg [7:0] ldata;
    always @(posedge clk) ldata <= laddr[7:0];   // stub BRAM: byte == low 8 bits of addr
    wire fw_erase, fw_prog; wire [23:0] fw_addr; wire [7:0] fw_data; wire fw_dvalid;
    reg fw_req=0, fw_busy=0, fw_done=0;
    wire save_busy;
    integer errors=0; reg [23:0] min_addr=24'hFFFFFF; reg saw_erase=0; reg [7:0] hdr0,hdr1;

    cflash_save dut(.clk(clk), .rst_n(rst_n), .save_start(save_start), .svc_count(svc_count),
        .laddr(laddr), .ldata(ldata),
        .fw_start_erase(fw_erase), .fw_start_program(fw_prog), .fw_flash_addr(fw_addr),
        .fw_prog_data(fw_data), .fw_prog_data_valid(fw_dvalid), .fw_prog_data_req(fw_req),
        .fw_busy(fw_busy), .fw_done(fw_done), .save_busy(save_busy));

    // ---- address monitor (load-bearing brick-bound + erase observed) ----
    always @(posedge clk) begin
        if ((fw_erase||fw_prog) && fw_addr<min_addr) min_addr<=fw_addr;
        if (fw_erase) saw_erase<=1;
    end

    // ---- MODEL of flash_writer, matching the real per-page handshake ----
    // mstate: 0 idle, 1 erasing, 2 programming
    reg [1:0] mstate=0;
    integer   mcyc=0;
    reg [8:0] mbyte=0;
    reg [23:0] maddr=0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fw_req<=0; fw_busy<=0; fw_done<=0; mstate<=0; mcyc<=0; mbyte<=0; maddr<=0;
        end else begin
            fw_done <= 1'b0;
            fw_req  <= 1'b0;
            case (mstate)
                2'd0: begin                       // IDLE: latch a new command
                    if (fw_erase) begin
                        fw_busy<=1; maddr<=fw_addr; mcyc<=0; mstate<=1;
                    end else if (fw_prog) begin
                        fw_busy<=1; maddr<=fw_addr; mcyc<=0; mbyte<=0; mstate<=2;
                    end
                end
                2'd1: begin                       // ERASE: spin a few cycles, then done
                    mcyc <= mcyc + 1;
                    if (mcyc==8) begin fw_done<=1; fw_busy<=0; mstate<=0; end
                end
                2'd2: begin                       // PROGRAM: ACK 256 bytes (req every 4 cyc)
                    mcyc <= mcyc + 1;
                    if (mcyc[1:0]==2'd0 && mbyte<9'd256) begin
                        // writer requires valid high at byte start; assert it does.
                        if (!fw_dvalid) begin
                            errors<=errors+1;
                            $display("FAIL writer stalled: prog_data_valid low at byte %0d (deadlock)", mbyte);
                        end
                        fw_req <= 1'b1;            // ACK: byte consumed, advance
                        // capture header magic bytes (header page is at 0x400000)
                        if (maddr==24'h400000) begin
                            if (mbyte==9'd0) hdr0 <= fw_data;
                            if (mbyte==9'd1) hdr1 <= fw_data;
                        end
                        mbyte <= mbyte + 9'd1;
                    end
                    if (mbyte==9'd256) begin fw_done<=1; fw_busy<=0; mstate<=0; end
                end
                default: mstate<=0;
            endcase
        end
    end

    // ---- watchdog so a broken FSM doesn't hang the run ----
    integer wd=0;
    always @(posedge clk) begin
        wd<=wd+1;
        if (wd>200000) begin $display("FAIL timeout (FSM hung)"); $finish; end
    end

    initial begin
        rst_n=0; #40; rst_n=1; #20;
        @(posedge clk); save_start=1; @(posedge clk); save_start=0;
        wait(save_busy); wait(!save_busy);
        #100;
        if (!saw_erase) begin errors=errors+1; $display("FAIL no erase"); end
        if (min_addr < 24'h400000) begin errors=errors+1; $display("FAIL brick-bound: emitted %06X",min_addr); end
        if (hdr0!==8'h43 || hdr1!==8'h52) begin errors=errors+1; $display("FAIL magic %02X%02X want 4352 (CR)",hdr0,hdr1); end
        if (errors==0) $display("PASS cflash_save (erase@>=400000, magic CR, no addr<400000)");
        else $display("FAIL cflash_save %0d",errors);
        $finish;
    end
endmodule
