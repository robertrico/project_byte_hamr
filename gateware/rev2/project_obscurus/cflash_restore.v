// cflash_restore.v - auto at boot_done: read header @ 0x400000, check magic "CR"; if valid,
// read 14 data pages @ 0x400100 and write coproc BRAM $0200-$0FFF via port B (coproc's own
// $1000+ write-protect stays in force). Bad/empty magic -> no write, valid=0 (fail-safe).
//
// flash_reader handshake (verified against flash_reader.v):
//   - fr_start is a 1-cycle pulse, consumed in flash_reader's S_IDLE.
//   - fr_data_valid pulses ONCE per byte (gated by fr_data_ready; held high here, no stall).
//   - fr_done pulses exactly ONE cycle AFTER the final fr_data_valid -> safe to act on done
//     after the last byte has been counted/written.
module cflash_restore (
    input  wire clk, input wire rst_n,
    input  wire boot_done,
    output reg  fr_start, output reg [23:0] fr_start_addr, output reg [23:0] fr_byte_count,
    output reg  fr_data_ready,
    input  wire fr_busy, input wire fr_done, input wire [7:0] fr_data_out, input wire fr_data_valid,
    output reg  [12:0] laddr, output reg [7:0] ldata, output reg lwr,
    output reg  restore_done, output reg restore_valid, output reg [7:0] svc_count);

    localparam [23:0] FLASH_BASE = 24'h400000;
    localparam [3:0] R_IDLE=0,R_HSTART=1,R_HREAD=2,R_DSTART=3,R_DREAD=4,R_DONE=5;
    reg [3:0] st; reg [7:0] hidx; reg [12:0] widx; reg started; reg magic_ok;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st<=R_IDLE; hidx<=0; widx<=0; started<=0; magic_ok<=0;
            fr_start<=0; fr_start_addr<=0; fr_byte_count<=0; fr_data_ready<=1'b1;
            laddr<=0; ldata<=0; lwr<=0; restore_done<=0; restore_valid<=0; svc_count<=0;
        end else begin
            fr_start<=0; lwr<=0;
            case (st)
                R_IDLE: if (boot_done && !started) begin
                    started<=1; magic_ok<=1'b1;
                    fr_start_addr<=FLASH_BASE; fr_byte_count<=24'd4; fr_start<=1; hidx<=0; st<=R_HSTART;
                end
                R_HSTART: st<=R_HREAD;
                R_HREAD: begin
                    if (fr_data_valid) begin
                        if (hidx==8'd0 && fr_data_out!=8'h43) magic_ok<=1'b0;  // not 'C'
                        if (hidx==8'd1 && fr_data_out!=8'h52) magic_ok<=1'b0;  // not 'R'
                        if (hidx==8'd3) svc_count<=fr_data_out;
                        hidx<=hidx+1;
                    end
                    if (fr_done) begin
                        if (magic_ok) begin widx<=0; st<=R_DSTART; end
                        else begin restore_done<=1; restore_valid<=1'b0; st<=R_DONE; end
                    end
                end
                R_DSTART: begin
                    fr_start_addr<=FLASH_BASE | 24'h100; fr_byte_count<=24'd3584; fr_start<=1; st<=R_DREAD;
                end
                R_DREAD: begin
                    if (fr_data_valid) begin
                        laddr<=13'h0200 + widx; ldata<=fr_data_out; lwr<=1'b1; widx<=widx+1;
                    end
                    if (fr_done) begin restore_valid<=1'b1; restore_done<=1; st<=R_DONE; end
                end
                R_DONE: ;   // latch; stay
                default: st<=R_IDLE;
            endcase
        end
    end
endmodule
