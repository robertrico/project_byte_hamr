`timescale 1ns/1ps
// =============================================================================
// spi_flash_model.v — Combined IS25LP128F behavioral model for C-flash tests
// =============================================================================
// Supports: WREN(06), SECTOR_ERASE(20), PAGE_PROGRAM(02), READ(03),
//           READ_STATUS(05).
//
// Sector-relative addressing: all addresses are masked to [11:0] (4096-byte
// backing store).  The real registry sector at $400000 maps in via its low
// bits without requiring a 4 MB array.
//
// Backing array is NONVOLATILE w.r.t. DUT resets: initialised to $FF at
// simulation power-on only (initial block).  There is intentionally no reset
// port — the DUT's rst_n must not clobber flash contents.
//
// Protocol timing matches flash_writer.v + flash_reader.v exactly:
//   MOSI sampled on posedge SCK  (SPI mode 0, master drives on falling edge)
//   MISO driven  on negedge SCK  (slave drives on falling edge)
//
// Merged from:
//   spi_flash_write_model  (gateware/rev2/block_hamr/flash_writer_tb.v)
//   spi_flash_model        (gateware/rev2/block_hamr/flash_reader_tb.v)
// =============================================================================

module spi_flash_model (
    input  wire sck,
    input  wire ncs,
    input  wire mosi,
    output reg  miso
);

    // -------------------------------------------------------------------------
    // 4 KB backing store — persists across DUT resets
    // -------------------------------------------------------------------------
    reg [7:0] mem [0:4095];

    // -------------------------------------------------------------------------
    // Internal state
    // -------------------------------------------------------------------------
    reg        wel;             // Write Enable Latch
    reg        wip;             // Write In Progress
    reg [7:0]  cmd_reg;
    reg [23:0] addr_reg;
    reg [5:0]  bit_cnt;
    reg        cmd_phase;       // 1 = receiving cmd+addr, 0 = data phase
    reg [7:0]  data_byte;       // byte being shifted in or out
    reg [2:0]  out_bit_cnt;     // which bit to drive next (MISO)
    reg [23:0] current_addr;    // running address during data phase
    reg [7:0]  status_reg;      // scratch for status byte
    reg        in_status_read;  // streaming READ_STATUS response
    reg [15:0] wip_counter;     // fast-sim WIP countdown (decrements on posedge SCK)
    reg [8:0]  pp_byte_cnt;     // page-program byte counter

    integer i;

    // -------------------------------------------------------------------------
    // Power-on initialisation — never runs again on DUT rst_n
    // -------------------------------------------------------------------------
    initial begin
        for (i = 0; i < 4096; i = i + 1)
            mem[i] = 8'hFF;
        miso           = 1'b1;
        wel            = 1'b0;
        wip            = 1'b0;
        bit_cnt        = 6'd0;
        cmd_phase      = 1'b1;
        cmd_reg        = 8'd0;
        addr_reg       = 24'd0;
        in_status_read = 1'b0;
        wip_counter    = 16'd0;
        pp_byte_cnt    = 9'd0;
    end

    // -------------------------------------------------------------------------
    // WIP countdown — decrement on every SCK edge (sim-only fast timeout)
    // -------------------------------------------------------------------------
    always @(posedge sck) begin
        if (wip_counter > 0) begin
            wip_counter <= wip_counter - 1;
            if (wip_counter == 1)
                wip <= 1'b0;
        end
    end

    // -------------------------------------------------------------------------
    // CS deassert (posedge ncs) — complete pending write commands
    // -------------------------------------------------------------------------
    always @(posedge ncs) begin
        bit_cnt        <= 6'd0;
        cmd_phase      <= 1'b1;
        out_bit_cnt    <= 3'd0;
        miso           <= 1'b1;
        in_status_read <= 1'b0;

        case (cmd_reg)
            8'h06: begin  // WREN
                wel <= 1'b1;
            end
            8'h20: begin  // SECTOR ERASE — entire 4 KB backing store
                if (wel && !wip) begin
                    for (i = 0; i < 4096; i = i + 1)
                        mem[i] = 8'hFF;
                    wip         <= 1'b1;
                    wip_counter <= 16'd4;   // fast sim: clears in 4 SCK edges
                    wel         <= 1'b0;
                end
            end
            8'h02: begin  // PAGE PROGRAM — data was written during clocking
                if (wel && !wip) begin
                    wip         <= 1'b1;
                    wip_counter <= 16'd2;   // fast sim: clears in 2 SCK edges
                    wel         <= 1'b0;
                end
            end
            // READ (03) and READ_STATUS (05) have no CS-edge action
        endcase
        cmd_reg <= 8'd0;
    end

    // -------------------------------------------------------------------------
    // CS assert (negedge ncs) — reset reception state
    // -------------------------------------------------------------------------
    always @(negedge ncs) begin
        bit_cnt        <= 6'd0;
        cmd_phase      <= 1'b1;
        cmd_reg        <= 8'd0;
        addr_reg       <= 24'd0;
        pp_byte_cnt    <= 9'd0;
        in_status_read <= 1'b0;
    end

    // -------------------------------------------------------------------------
    // Sample MOSI on SCK rising edge (SPI mode 0: slave latches on rising SCK)
    // -------------------------------------------------------------------------
    always @(posedge sck) begin
        if (!ncs) begin
            if (cmd_phase) begin
                // Accumulate command byte (bits 0-7) then address (bits 8-31)
                if (bit_cnt < 6'd8)
                    cmd_reg <= {cmd_reg[6:0], mosi};
                else if (bit_cnt < 6'd32)
                    addr_reg <= {addr_reg[22:0], mosi};

                bit_cnt <= bit_cnt + 6'd1;

                // After 8 bits — check for address-less commands
                if (bit_cnt == 6'd7) begin
                    case ({cmd_reg[6:0], mosi})
                        8'h06: begin  // WREN: no address, CS will deassert
                            cmd_reg <= {cmd_reg[6:0], mosi};
                        end
                        8'h05: begin  // READ_STATUS: no address, stream data
                            cmd_reg        <= {cmd_reg[6:0], mosi};
                            cmd_phase      <= 1'b0;
                            in_status_read <= 1'b1;
                            out_bit_cnt    <= 3'd0;
                        end
                    endcase
                end

                // After 32 bits (8 cmd + 24 addr): enter data phase
                if (bit_cnt == 6'd31) begin
                    cmd_phase    <= 1'b0;
                    current_addr <= {addr_reg[22:0], mosi};  // include last addr bit
                    out_bit_cnt  <= 3'd0;
                end

            end else if (cmd_reg == 8'h02 && !in_status_read) begin
                // PAGE PROGRAM data: accumulate bits, write byte on bit 7
                data_byte   <= {data_byte[6:0], mosi};
                out_bit_cnt <= out_bit_cnt + 3'd1;
                if (out_bit_cnt == 3'd7) begin
                    // Flash semantics: can only clear bits (NOR cells)
                    mem[current_addr[11:0]] <= mem[current_addr[11:0]] & {data_byte[6:0], mosi};
                    current_addr            <= current_addr + 24'd1;
                    pp_byte_cnt             <= pp_byte_cnt + 9'd1;
                    out_bit_cnt             <= 3'd0;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Drive MISO on SCK falling edge (slave drives on falling, master samples
    // on rising — correct timing for flash_reader.v's S_READ_DATA capture)
    // -------------------------------------------------------------------------
    always @(negedge sck) begin
        if (!ncs && !cmd_phase) begin
            if (in_status_read) begin
                // Status register: bit1=WEL, bit0=WIP (all other bits 0)
                status_reg  = {6'b0, wel, wip};
                miso        <= status_reg[7 - out_bit_cnt];
                out_bit_cnt <= out_bit_cnt + 3'd1;
                // Status is read indefinitely; out_bit_cnt wraps naturally

            end else if (cmd_reg == 8'h03) begin
                // READ (03): stream bytes MSB-first
                // Load next byte on bit 0 of each byte
                if (out_bit_cnt == 3'd0)
                    data_byte <= mem[current_addr[11:0]];
                miso <= (out_bit_cnt == 3'd0) ?
                        mem[current_addr[11:0]][7] :
                        data_byte[7 - out_bit_cnt];
                out_bit_cnt <= out_bit_cnt + 3'd1;
                if (out_bit_cnt == 3'd7) begin
                    current_addr <= current_addr + 24'd1;
                    out_bit_cnt  <= 3'd0;
                end
            end
        end
    end

endmodule
