`timescale 1ns / 1ps
// =============================================================================
// bus_interface.v — Apple II bus register interface for block device
// =============================================================================
// Simple register file for 6502 block device access.
// The slot ROM reads/writes these registers directly.
//
// Register Map ($C0n0 - $C0nF, via addr[3:0]):
//   0: STATUS (R) / COMMAND (W)
//      Read:  {ready, error, 5'b0, boot_done}
//      Write: $01 = READ_BLOCK, $02 = WRITE_BLOCK
//   1: DATA_READ (R)
//      Read with auto-increment. Used by LDA abs,X (4-cycle, 1 pulse).
//   2: BLOCK_LO (R/W)  — block number low byte
//   3: BLOCK_HI (R/W)  — block number high byte
//   4: SOFT RESET (W)  — writing any value resets all state
//   5: DATA_WRITE (W)
//      Write with auto-increment. Separate address avoids the 6502
//      STA abs,X dummy-read bug: STA is 5 cycles and cycle 4 is a
//      dummy READ at the effective address. If writes used addr 1,
//      the dummy read would trigger a spurious auto-increment.
//   6-F: Reserved (reads $00)
//
// BRAM pipeline: block_buffer has 1-cycle registered reads. A pre-fetch
// register (prefetch) always holds the byte at buf_addr. When the CPU
// reads DATA, it gets prefetch, then buf_addr increments and the BRAM
// delivers the next byte into prefetch on the following clock.
// =============================================================================

module bus_interface (
    input  wire        clk,              // 7 MHz (sig_7M)
    input  wire        rst_n,

    // Apple II bus
    input  wire [3:0]  addr,             // A3-A0 register select
    input  wire [7:0]  data_in,          // Data from Apple II
    output reg  [7:0]  data_out,         // Data to Apple II
    input  wire        nDEVICE_SELECT,   // Active low
    input  wire        R_nW,             // 1=read, 0=write

    // System
    input  wire        boot_done,
    input  wire [15:0] total_blocks,       // auto-detected volume size from boot_loader

    // Block buffer Port B (7 MHz side) — BRAM with 1-cycle read latency
    output reg  [8:0]  buf_addr,
    input  wire [7:0]  buf_rdata,        // Registered read (1-cycle latency)
    output reg  [7:0]  buf_wdata,
    output reg         buf_we,

    // SDRAM block request interface (directly to sdram_arbiter)
    output reg         block_read_req,
    output reg         block_write_req,
    output reg  [15:0] block_num,
    input  wire        block_ready,

    // Block number exposed for write-through controller
    output wire [15:0] block_num_out
);

    // =========================================================================
    // Command codes
    // =========================================================================
    localparam CMD_READ_BLOCK  = 8'h01;
    localparam CMD_WRITE_BLOCK = 8'h02;

    // =========================================================================
    // State machine
    // =========================================================================
    localparam [1:0] S_IDLE = 2'd0;
    localparam [1:0] S_BUSY = 2'd1;

    reg [1:0] state;
    reg error;
    reg auto_inc;

    assign block_num_out = block_num;

    wire ready = (state == S_IDLE) & boot_done;

    // =========================================================================
    // Pre-fetch register for BRAM read pipeline
    // =========================================================================
    // buf_rdata arrives 1 cycle after buf_addr changes. We continuously
    // capture it into prefetch. When the CPU reads DATA, it gets prefetch
    // (which reflects the current buf_addr). Then buf_addr increments,
    // and prefetch updates on the next clock with the new byte.
    reg [7:0] prefetch;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            prefetch <= 8'd0;
        else
            prefetch <= buf_rdata;
    end

    // =========================================================================
    // Bus signal synchronizers (2-FF, 7 MHz domain)
    // =========================================================================
    reg nds_d1, nds_d2;
    reg rw_d1,  rw_d2;
    reg [3:0] addr_d1, addr_d2;
    reg [7:0] data_d1, data_d2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            nds_d1  <= 1'b1; nds_d2  <= 1'b1;
            rw_d1   <= 1'b1; rw_d2   <= 1'b1;
            addr_d1 <= 4'd0; addr_d2 <= 4'd0;
            data_d1 <= 8'd0; data_d2 <= 8'd0;
        end else begin
            nds_d1  <= nDEVICE_SELECT; nds_d2  <= nds_d1;
            rw_d1   <= R_nW;           rw_d2   <= rw_d1;
            addr_d1 <= addr;           addr_d2 <= addr_d1;
            data_d1 <= data_in;        data_d2 <= data_d1;
        end
    end

    wire nds_fall = nds_d2 & ~nds_d1;
    wire nds_rise = ~nds_d2 & nds_d1;

    // =========================================================================
    // Bus cycle capture — continuous sampling during active cycle
    // =========================================================================
    // Rev 2 has a U12 level-shifter in the data path which adds ~5-10 ns of
    // propagation delay. The original posedge-nDEVICE_SELECT capture is a
    // single-point sample right at cycle end — too fragile against that
    // added delay, and it was freezing the Apple II on PR#4 while writes
    // to $C0C0 silently got bogus data.
    //
    // Instead, continuously sample data/addr/R_nW while the synchronized
    // nDEVICE_SELECT is low. The last sample captured before nds_rise fires
    // is guaranteed to be inside the valid data window (the 2-FF sync
    // naturally excludes samples taken after the real bus cycle ended).
    // This matches signal_check's loopback register, which is the proven
    // working pattern for this board revision.
    reg [7:0] wr_data_latch;
    reg [3:0] wr_addr_latch;
    reg       wr_rw_latch;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_data_latch <= 8'd0;
            wr_addr_latch <= 4'd0;
            wr_rw_latch   <= 1'b1;
        end else if (!nds_d2) begin
            wr_data_latch <= data_d2;
            wr_addr_latch <= addr_d2;
            wr_rw_latch   <= rw_d2;
        end
    end

    // =========================================================================
    // Block ready synchronizer (block_ready comes from 25 MHz domain)
    // =========================================================================
    reg br_d1, br_d2, br_prev;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            br_d1   <= 1'b0;
            br_d2   <= 1'b0;
            br_prev <= 1'b0;
        end else begin
            br_d1   <= block_ready;
            br_d2   <= br_d1;
            br_prev <= br_d2;
        end
    end

    wire br_rise = br_d2 & ~br_prev;

    // =========================================================================
    // Read mux (combinational — data valid entire nDEVICE_SELECT window)
    // =========================================================================
    always @(*) begin
        case (addr)
            4'h0: data_out = {ready, error, 5'b0, boot_done};
            4'h1: data_out = prefetch;
            4'h2: data_out = block_num[7:0];
            4'h3: data_out = block_num[15:8];
            4'h6: data_out = total_blocks[7:0];
            4'h7: data_out = total_blocks[15:8];
            default: data_out = 8'h00;
        endcase
    end

    // =========================================================================
    // Main logic
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= S_IDLE;
            error           <= 1'b0;
            block_num       <= 16'd0;
            buf_addr        <= 9'd0;
            buf_wdata       <= 8'd0;
            buf_we          <= 1'b0;
            block_read_req  <= 1'b0;
            block_write_req <= 1'b0;
            auto_inc        <= 1'b0;
        end else begin
            // Defaults
            buf_we  <= 1'b0;
            // Auto-increment: READ of addr 1, or WRITE of addr 5.
            // Separate addresses prevent 6502 STA abs,X dummy-read
            // (cycle 4) from causing a spurious increment.
            auto_inc <= nds_rise && (
                (wr_addr_latch == 4'h1 &&  wr_rw_latch) ||  // LDA DATA_READ
                (wr_addr_latch == 4'h5 && ~wr_rw_latch)     // STA DATA_WRITE
            );

            // ---- State transitions ----
            case (state)
                S_IDLE: begin
                    if (br_rise) begin
                        block_read_req  <= 1'b0;
                        block_write_req <= 1'b0;
                    end
                end

                S_BUSY: begin
                    if (br_rise) begin
                        block_read_req  <= 1'b0;
                        block_write_req <= 1'b0;
                        buf_addr        <= 9'd0;
                        state           <= S_IDLE;
                    end
                end
            endcase

            // ---- Write register handling (uses bus-cycle-captured values) ----
            if (nds_rise & ~wr_rw_latch) begin
                case (wr_addr_latch)
                    4'h0: begin  // COMMAND register
                        if (boot_done && state == S_IDLE) begin
                            error <= 1'b0;
                            buf_addr <= 9'd0;
                            case (wr_data_latch)
                                CMD_READ_BLOCK: begin
                                    block_read_req <= 1'b1;
                                    state <= S_BUSY;
                                end
                                CMD_WRITE_BLOCK: begin
                                    block_write_req <= 1'b1;
                                    state <= S_BUSY;
                                end
                            endcase
                        end
                    end

                    4'h5: begin  // DATA_WRITE register
                        buf_wdata <= wr_data_latch;
                        buf_we    <= 1'b1;
                    end

                    4'h2: begin
                        block_num[7:0] <= wr_data_latch;
                        buf_addr       <= 9'd0;
                    end
                    4'h3: block_num[15:8] <= wr_data_latch;

                    4'h4: begin  // SOFT RESET register
                        state           <= S_IDLE;
                        error           <= 1'b0;
                        buf_addr        <= 9'd0;
                        block_read_req  <= 1'b0;
                        block_write_req <= 1'b0;
                        block_num       <= 16'd0;
                    end
                endcase
            end

            // Auto-increment buf_addr 1 cycle after nds_rise — delayed so
            // BRAM write (buf_we) completes at the correct address first
            if (auto_inc) begin
                buf_addr <= buf_addr + 9'd1;
            end
        end
    end

endmodule
