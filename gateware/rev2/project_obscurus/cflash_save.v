// cflash_save.v - SAVE FSM: persist the coproc registry to SPI flash.
//
// Erases the registry sector, then programs:
//   - 14 DATA pages  (coproc BRAM $0200-$0FFF) FIRST, at flash 0x400100 + P*256
//   - the HEADER page (magic "CR", version, svc_count) LAST, at flash 0x400000
// Header LAST is the torn-save fail-safe: an interrupted save leaves the header
// erased ($FF != "CR" = invalid), so the boot restore skips a partial image.
//
// Every flash address = FLASH_BASE | offset with offset < 0x1000, so nothing
// below 0x400000 (one sector) can ever be emitted even with a logic bug.
//
// flash_writer handshake (see flash_writer.v):
//   - start_erase / start_program are 1-cycle pulses; flash_addr must be valid
//     on the cycle the pulse is HIGH.
//   - PROGRAM streams EXACTLY 256 bytes per start_program. The writer requires
//     prog_data_valid HIGH at each byte start or it STALLS; it pulses
//     prog_data_req once per byte as an ACK ("byte latched, advance").
//   - Each page completes with its own 1-cycle done (after WIP poll). The caller
//     MUST wait for done before issuing the next start_program. Hence the
//     per-page done-wait built into S_DPROG / S_HPROG (NOT a per-byte advance).
//
// We hold prog_data_valid HIGH for the whole program command and present the
// current byte combinationally (data = registered BRAM ldata; header = mux on
// hidx). BRAM read latency is 1 cycle; the writer requests a byte only every
// ~16 cycles, so ldata has long settled by the time prog_data_req fires.
module cflash_save (
    input  wire clk, input wire rst_n,
    input  wire save_start,            // 1-cycle pulse
    input  wire [7:0] svc_count,
    output wire [12:0] laddr,          // BRAM addr to read
    input  wire [7:0]  ldata,          // BRAM[laddr], registered (1-cyc latency)
    output reg  fw_start_erase, output reg fw_start_program,
    output reg  [23:0] fw_flash_addr,
    output wire [7:0]  fw_prog_data, output wire fw_prog_data_valid,
    input  wire fw_prog_data_req, input wire fw_busy, input wire fw_done,
    output wire save_busy);

    localparam [23:0] FLASH_BASE = 24'h400000;
    localparam [12:0] BRAM_BASE  = 13'h0200;
    localparam        DATA_PAGES = 14;

    localparam [3:0]
        S_IDLE   = 4'd0,
        S_ERASE  = 4'd1,   // erase pulse cycle
        S_ERWAIT = 4'd2,   // wait erase done
        S_DPAGE  = 4'd3,   // issue program pulse for a data page
        S_DPROG  = 4'd4,   // stream 256 data bytes, wait page done
        S_HPAGE  = 4'd5,   // issue program pulse for the header page
        S_HPROG  = 4'd6,   // stream 256 header bytes, wait page done
        S_DONE   = 4'd7;

    reg [3:0] st;
    reg [3:0] page;        // 0..13 data page index
    reg [7:0] boff;        // 0..255 byte offset within data page
    reg [7:0] hidx;        // 0..255 byte offset within header page

    assign save_busy = (st != S_IDLE);

    // BRAM read address for the current data byte.
    assign laddr = BRAM_BASE + {page, 8'd0} + {5'd0, boff};

    // Header page bytes: "CR" magic, version 0x01, svc_count, then 0xFF pad.
    wire [7:0] hdr_byte = (hidx==8'd0) ? 8'h43 :          // 'C'
                          (hidx==8'd1) ? 8'h52 :          // 'R'
                          (hidx==8'd2) ? 8'h01 :          // version
                          (hidx==8'd3) ? svc_count :
                                         8'hFF;

    // Data presented to the writer: header bytes during header program,
    // otherwise the registered BRAM byte. Valid held high across both programs.
    assign fw_prog_data       = (st==S_HPROG) ? hdr_byte : ldata;
    assign fw_prog_data_valid = (st==S_DPROG) || (st==S_HPROG);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st<=S_IDLE; page<=0; boff<=0; hidx<=0;
            fw_start_erase<=0; fw_start_program<=0; fw_flash_addr<=24'd0;
        end else begin
            fw_start_erase   <= 1'b0;
            fw_start_program <= 1'b0;
            case (st)
                S_IDLE: if (save_start) begin
                    fw_flash_addr <= FLASH_BASE;       // erase the registry sector
                    fw_start_erase <= 1'b1;
                    st <= S_ERASE;
                end
                S_ERASE: st <= S_ERWAIT;               // let erase pulse clear
                S_ERWAIT: if (fw_done) begin
                    page <= 0; boff <= 0;
                    st <= S_DPAGE;
                end
                S_DPAGE: begin
                    // offset = 0x100 + page*256, max page 13 -> 0xE00 < 0x1000
                    fw_flash_addr <= FLASH_BASE | (24'h000100 + {12'd0, page, 8'd0});
                    boff <= 0;
                    fw_start_program <= 1'b1;
                    st <= S_DPROG;
                end
                S_DPROG: begin
                    if (fw_done) begin
                        if (page == DATA_PAGES-1) st <= S_HPAGE;
                        else begin page <= page + 4'd1; st <= S_DPAGE; end
                    end else if (fw_prog_data_req) begin
                        boff <= boff + 8'd1;           // advance to next data byte
                    end
                end
                S_HPAGE: begin
                    fw_flash_addr <= FLASH_BASE;       // header page LAST (offset 0)
                    hidx <= 0;
                    fw_start_program <= 1'b1;
                    st <= S_HPROG;
                end
                S_HPROG: begin
                    if (fw_done) st <= S_DONE;
                    else if (fw_prog_data_req) hidx <= hidx + 8'd1;
                end
                S_DONE: if (!fw_busy) st <= S_IDLE;
                default: st <= S_IDLE;
            endcase
        end
    end
endmodule
