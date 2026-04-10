`timescale 1ns / 1ps
// =============================================================================
// bus_interface.v — Apple II bus register interface (multi-drive)
// =============================================================================
// Register Map ($C0n0 - $C0nF, via addr[3:0]):
//   0: STATUS (R) / COMMAND (W)
//   1: DATA_READ (R) — auto-increment
//   2: BLOCK_LO (R/W) — relative block number
//   3: BLOCK_HI (R/W) — relative block number
//   4: SOFT RESET (W)
//   5: DATA_WRITE (W) — auto-increment (separate from DATA_READ)
//   6: TOTAL_BLOCKS_LO (R) — S4D1 volume size (backward compat)
//   7: TOTAL_BLOCKS_HI (R)
//   8: SD_STATUS (R) / SD_CMD (W)
//   9: IMG_COUNT (R) / IMG_SELECT (W) — {mount_slot[1:0], 2'b0, img_idx[3:0]}
//   A: CAT_NAME_CHAR (R) / CAT_NAME_IDX (W)
//   B: UNIT_BLK_LO (R) — block count indexed by active_unit
//   C: UNIT_BLK_HI (R) — block count indexed by active_unit
//   D: BOOT_UNIT (R) / ACTIVE_UNIT (W) — ROM writes before I/O
//   E: — / BOOT_UNIT (W) — menu sets before reboot
//   F: DEBUG (R)
//
// Hardware offset addition:
//   block_num_out = block_num + unit_offset[active_unit]
//   The arbiter sees absolute SDRAM block addresses.
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
    input  wire [15:0] total_blocks,       // S4D1 volume size from boot_loader

    // Block buffer Port B (7 MHz side)
    output reg  [8:0]  buf_addr,
    input  wire [7:0]  buf_rdata,
    output reg  [7:0]  buf_wdata,
    output reg         buf_we,

    // SDRAM block request interface
    output reg         block_read_req,
    output reg         block_write_req,
    output reg  [15:0] block_num,          // relative block (within unit)
    input  wire        block_ready,

    // Block number with unit offset applied (to arbiter + persist)
    output wire [15:0] block_num_out,

    // Debug counters
    output wire [7:0] dbg_wr_count_out,
    output wire [7:0] dbg_rd_count_out,

    // ---- SD card management (registers 8-F) ----
    input  wire        sd_ready,
    input  wire        sd_error_in,
    input  wire        s4d2_mounted,
    input  wire        s4d2_loading,
    input  wire [3:0]  img_count,

    // Catalog BRAM read port
    output wire [8:0]  cat_rd_addr,
    input  wire [7:0]  cat_rd_data,

    // Mount command output
    output reg  [3:0]  img_select,
    output reg  [4:0]  img_name_idx,
    output reg         mount_request,
    output reg         sd_init_request,

    // Mailbox command output
    output reg  [7:0]  sd_cmd_data,
    output reg         sd_cmd_wr,

    // Mount slot (high bits of arg0 for multi-drive mount)
    output reg  [1:0]  mount_slot,

    // ---- Per-unit data (CDC'd from 25MHz in top-level) ----
    input  wire [15:0] unit_blkcnt_0,     // S4D1 block count
    input  wire [15:0] unit_blkcnt_1,     // S4D2 block count
    input  wire [15:0] unit_blkcnt_2,     // S4D3 block count
    input  wire [15:0] unit_blkcnt_3,     // S4D4 block count
    input  wire [15:0] unit_offset_1,     // S4D2 SDRAM block offset
    input  wire [15:0] unit_offset_2,     // S4D3 SDRAM block offset
    input  wire [15:0] unit_offset_3      // S4D4 SDRAM block offset
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
    reg [7:0] dbg_write_count;
    reg [7:0] dbg_read_count;

    // =========================================================================
    // Multi-drive registers
    // =========================================================================
    reg [1:0] active_unit;    // ROM writes before each I/O (0=S4D1, 1=S4D2, ...)
    reg [1:0] boot_unit_reg;  // Menu program writes before reboot

    // Per-unit offset mux (S4D1 offset is always 0)
    wire [15:0] active_offset = (active_unit == 2'd0) ? 16'd0 :
                                (active_unit == 2'd1) ? unit_offset_1 :
                                (active_unit == 2'd2) ? unit_offset_2 :
                                                        unit_offset_3;

    // Block number with unit offset applied — sent to arbiter
    assign block_num_out = block_num + active_offset;

    // Per-unit block count mux
    wire [15:0] active_blkcnt = (active_unit == 2'd0) ? unit_blkcnt_0 :
                                (active_unit == 2'd1) ? unit_blkcnt_1 :
                                (active_unit == 2'd2) ? unit_blkcnt_2 :
                                                        unit_blkcnt_3;

    assign dbg_wr_count_out = dbg_write_count;
    assign dbg_rd_count_out = dbg_read_count;

    wire ready = (state == S_IDLE) & boot_done;

    // =========================================================================
    // CDC synchronizers for 25MHz → 7MHz domain crossing
    // =========================================================================
    reg        sd_ready_s1,     sd_ready_s2;
    reg        sd_error_s1,     sd_error_s2;
    reg        s4d2_mounted_s1, s4d2_mounted_s2;
    reg        s4d2_loading_s1, s4d2_loading_s2;
    reg [3:0]  img_count_s1,    img_count_s2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            {sd_ready_s1, sd_ready_s2}         <= 2'b0;
            {sd_error_s1, sd_error_s2}         <= 2'b0;
            {s4d2_mounted_s1, s4d2_mounted_s2} <= 2'b0;
            {s4d2_loading_s1, s4d2_loading_s2} <= 2'b0;
            {img_count_s1, img_count_s2}       <= 8'b0;
        end else begin
            sd_ready_s1     <= sd_ready;        sd_ready_s2     <= sd_ready_s1;
            sd_error_s1     <= sd_error_in;     sd_error_s2     <= sd_error_s1;
            s4d2_mounted_s1 <= s4d2_mounted;    s4d2_mounted_s2 <= s4d2_mounted_s1;
            s4d2_loading_s1 <= s4d2_loading;    s4d2_loading_s2 <= s4d2_loading_s1;
            img_count_s1    <= img_count;       img_count_s2    <= img_count_s1;
        end
    end

    // =========================================================================
    // Pre-fetch register for BRAM read pipeline
    // =========================================================================
    reg [7:0] prefetch;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            prefetch <= 8'd0;
        else
            prefetch <= buf_rdata;
    end

    // =========================================================================
    // nDEVICE_SELECT edge detection
    // =========================================================================
    reg nds_d1, nds_d2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            nds_d1 <= 1'b1;
            nds_d2 <= 1'b1;
        end else begin
            nds_d1 <= nDEVICE_SELECT;
            nds_d2 <= nds_d1;
        end
    end

    wire nds_fall = nds_d2 & ~nds_d1;
    wire nds_rise = ~nds_d2 & nds_d1;

    // =========================================================================
    // Bus cycle capture — latch data/addr on posedge nDEVICE_SELECT
    // =========================================================================
    reg [7:0] wr_data_latch;
    reg [3:0] wr_addr_latch;
    reg       wr_rw_latch;

    always @(posedge nDEVICE_SELECT) begin
        wr_data_latch <= data_in;
        wr_addr_latch <= addr;
        wr_rw_latch   <= R_nW;
    end

    // =========================================================================
    // Block ready synchronizer (25 MHz → 7 MHz)
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
    // Catalog BRAM addressing
    // =========================================================================
    wire [8:0] cat_entry_base = {img_select, 5'd0};
    assign cat_rd_addr = cat_entry_base + {4'd0, img_name_idx};

    // =========================================================================
    // Read mux (combinational)
    // =========================================================================
    always @(*) begin
        case (addr)
            4'h0: data_out = {ready, error, 5'b0, boot_done};
            4'h1: data_out = prefetch;
            4'h2: data_out = block_num[7:0];
            4'h3: data_out = block_num[15:8];
            4'h6: data_out = total_blocks[7:0];
            4'h7: data_out = total_blocks[15:8];
            // SD management
            4'h8: data_out = {sd_ready_s2, sd_error_s2, s4d2_mounted_s2, s4d2_loading_s2, 4'b0};
            4'h9: data_out = {4'b0, img_count_s2};
            4'hA: data_out = cat_rd_data;
            // Multi-drive indexed registers
            4'hB: data_out = active_blkcnt[7:0];
            4'hC: data_out = active_blkcnt[15:8];
            4'hD: data_out = {6'b0, boot_unit_reg};
            4'hE: data_out = 8'h00;
            4'hF: data_out = {block_ready, block_write_req, block_read_req, state, 3'b0};
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
            img_select      <= 4'd0;
            img_name_idx    <= 5'd0;
            mount_request   <= 1'b0;
            sd_init_request <= 1'b0;
            sd_cmd_data     <= 8'd0;
            sd_cmd_wr       <= 1'b0;
            mount_slot      <= 2'd0;
            active_unit     <= 2'd0;
            boot_unit_reg   <= 2'd0;
            dbg_write_count <= 8'd0;
            dbg_read_count  <= 8'd0;
        end else begin
            mount_request   <= 1'b0;
            sd_cmd_wr       <= 1'b0;
            buf_we  <= 1'b0;

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

            // ---- Write register handling ----
            if (nds_rise & ~wr_rw_latch) begin
                case (wr_addr_latch)
                    4'h0: begin  // COMMAND
                        if (boot_done && state == S_IDLE) begin
                            error <= 1'b0;
                            buf_addr <= 9'd0;
                            case (wr_data_latch)
                                CMD_READ_BLOCK: begin
                                    block_read_req <= 1'b1;
                                    state <= S_BUSY;
                                    dbg_read_count <= dbg_read_count + 8'd1;
                                end
                                CMD_WRITE_BLOCK: begin
                                    block_write_req <= 1'b1;
                                    state <= S_BUSY;
                                    dbg_write_count <= dbg_write_count + 8'd1;
                                end
                            endcase
                        end
                    end

                    4'h5: begin  // DATA_WRITE
                        buf_wdata <= wr_data_latch;
                        buf_we    <= 1'b1;
                    end

                    4'h2: begin  // BLOCK_LO
                        block_num[7:0] <= wr_data_latch;
                        buf_addr       <= 9'd0;
                    end
                    4'h3: block_num[15:8] <= wr_data_latch;  // BLOCK_HI

                    4'h4: begin  // SOFT RESET
                        state           <= S_IDLE;
                        error           <= 1'b0;
                        buf_addr        <= 9'd0;
                        block_read_req  <= 1'b0;
                        block_write_req <= 1'b0;
                        block_num       <= 16'd0;
                        active_unit     <= 2'd0;
                    end

                    // SD management registers
                    4'h8: begin  // SD_CMD
                        if (wr_data_latch == 8'h01)
                            mount_request <= 1'b1;
                        if (wr_data_latch == 8'h02)
                            sd_init_request <= 1'b1;
                        if (wr_data_latch == 8'h00)
                            sd_init_request <= 1'b0;
                        sd_cmd_data <= wr_data_latch;
                        sd_cmd_wr   <= 1'b1;
                    end
                    4'h9: begin  // IMG_SELECT + MOUNT_SLOT
                        img_select <= wr_data_latch[3:0];
                        mount_slot <= wr_data_latch[7:6];
                    end
                    4'hA: begin  // IMG_NAME_IDX
                        img_name_idx <= wr_data_latch[4:0];
                    end

                    // Multi-drive registers
                    4'hD: active_unit   <= wr_data_latch[1:0];  // ACTIVE_UNIT
                    4'hE: boot_unit_reg <= wr_data_latch[1:0];  // BOOT_UNIT
                endcase
            end

            // Auto-increment
            if (auto_inc) begin
                buf_addr <= buf_addr + 9'd1;
            end
        end
    end

endmodule
