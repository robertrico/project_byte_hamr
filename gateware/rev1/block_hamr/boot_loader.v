// boot_loader.v — Copies disk image from SPI flash into SDRAM at power-on
//
// Auto-detects volume size from the ProDOS volume header (block 2,
// total_blocks field at entry offset $25-$26). Caps at MAX_BLOCKS to
// prevent overrunning flash. Exposes detected total_blocks for the
// bus_interface STATUS response.
//
// Reads bytes from flash_reader, packs into 16-bit words, writes to SDRAM
// starting at address 0. Handles odd byte counts by zero-padding high byte.

`timescale 1ns / 1ps

module boot_loader #(
    parameter FLASH_OFFSET = 24'h400000,   // 4MB offset in flash (after bitstream)
    parameter MAX_BLOCKS   = 16'd24576     // 12MB / 512 — hard flash limit
)(
    input  wire        clk,            // 25 MHz
    input  wire        rst_n,
    input  wire        sdram_init_done,
    output reg         boot_done = 1'b0,
    output reg  [15:0] total_blocks = 16'd0,  // auto-detected from ProDOS header

    // Flash reader interface
    output reg         flash_start,
    output reg  [23:0] flash_addr,
    output reg  [23:0] flash_count,
    input  wire        flash_busy,
    input  wire [7:0]  flash_data,
    input  wire        flash_data_valid,
    output reg         flash_data_ready,

    // SDRAM write interface
    output reg         sdram_req,
    output reg         sdram_req_write,
    output reg  [25:0] sdram_req_addr,
    output reg  [15:0] sdram_req_wdata,
    input  wire        sdram_req_ready,

    // Debug
    output wire [2:0]  debug_state
);

    // =========================================================================
    // ProDOS volume header parsing
    // =========================================================================
    // Block 2 starts at byte 1024. Volume header entry at byte 1028.
    // total_blocks at entry offset $25-$26 = absolute bytes 1065 (lo), 1066 (hi).
    localparam [23:0] HDR_BLKCNT_LO = 24'd1065;
    localparam [23:0] HDR_BLKCNT_HI = 24'd1066;

    // Maximum image size in bytes (MAX_BLOCKS * 512)
    localparam [23:0] MAX_IMAGE_BYTES = {MAX_BLOCKS[14:0], 9'd0};

    // =========================================================================
    // State encoding
    // =========================================================================
    localparam [2:0] ST_WAIT_INIT  = 3'd0,
                     ST_START_FLASH = 3'd1,
                     ST_COPY_LO    = 3'd2,
                     ST_COPY_HI    = 3'd3,
                     ST_WRITE_SDRAM = 3'd4,
                     ST_DONE       = 3'd5;

    reg [2:0]  state = ST_WAIT_INIT;
    assign debug_state = state;
    reg [7:0]  lo_byte = 8'd0;
    reg [23:0] byte_count = 24'd0;       // bytes consumed from flash
    reg [25:0] write_addr = 26'd0;
    reg [23:0] image_size = 24'd0;       // actual bytes to copy (set after header parse)
    reg [7:0]  blkcnt_lo = 8'd0;         // captured total_blocks lo byte
    reg        size_valid = 1'b0;         // header parsed, image_size is set

    // =========================================================================
    // State Machine
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= ST_WAIT_INIT;
            boot_done       <= 1'b0;
            total_blocks    <= 16'd0;
            flash_start     <= 1'b0;
            flash_addr      <= 24'd0;
            flash_count     <= 24'd0;
            flash_data_ready <= 1'b0;
            sdram_req       <= 1'b0;
            sdram_req_write <= 1'b0;
            sdram_req_addr  <= 26'd0;
            sdram_req_wdata <= 16'd0;
            lo_byte         <= 8'd0;
            byte_count      <= 24'd0;
            write_addr      <= 26'd0;
            image_size      <= 24'd0;
            blkcnt_lo       <= 8'd0;
            size_valid      <= 1'b0;
        end else begin
            // Defaults: deassert single-cycle pulses
            flash_start <= 1'b0;

            case (state)
                // ---------------------------------------------------------
                // Wait for SDRAM initialization to complete
                // ---------------------------------------------------------
                ST_WAIT_INIT: begin
                    boot_done        <= 1'b0;
                    flash_data_ready <= 1'b0;
                    sdram_req        <= 1'b0;
                    if (sdram_init_done) begin
                        state <= ST_START_FLASH;
                    end
                end

                // ---------------------------------------------------------
                // Kick off flash read — request MAX, stop early once
                // we parse total_blocks from the ProDOS header
                // ---------------------------------------------------------
                ST_START_FLASH: begin
                    flash_start      <= 1'b1;
                    flash_addr       <= FLASH_OFFSET;
                    flash_count      <= MAX_IMAGE_BYTES;
                    flash_data_ready <= 1'b1;
                    byte_count       <= 24'd0;
                    write_addr       <= 26'd0;
                    image_size       <= MAX_IMAGE_BYTES;
                    size_valid       <= 1'b0;
                    state            <= ST_COPY_LO;
                end

                // ---------------------------------------------------------
                // Capture low byte from flash
                // ---------------------------------------------------------
                ST_COPY_LO: begin
                    sdram_req <= 1'b0;
                    if (flash_data_valid) begin
                        lo_byte          <= flash_data;
                        flash_data_ready <= 1'b0;

                        // Capture total_blocks hi byte (byte 1066 is a lo byte in the stream)
                        if (byte_count == HDR_BLKCNT_HI && !size_valid) begin
                            total_blocks <= {flash_data, blkcnt_lo};
                            if ({flash_data, blkcnt_lo} == 16'd0 || {flash_data, blkcnt_lo} > MAX_BLOCKS)
                                image_size <= MAX_IMAGE_BYTES;
                            else
                                image_size <= {flash_data[6:0], blkcnt_lo, 9'd0};
                            size_valid <= 1'b1;
                        end

                        byte_count <= byte_count + 24'd1;

                        if (size_valid && byte_count + 24'd1 >= image_size) begin
                            // Last byte, odd count — pad hi byte
                            sdram_req_wdata <= {8'h00, flash_data};
                            sdram_req_addr  <= write_addr;
                            sdram_req       <= 1'b1;
                            sdram_req_write <= 1'b1;
                            state           <= ST_WRITE_SDRAM;
                        end else begin
                            state <= ST_COPY_HI;
                        end
                    end
                end

                // ---------------------------------------------------------
                // Capture high byte, form 16-bit word, initiate SDRAM write
                // ---------------------------------------------------------
                ST_COPY_HI: begin
                    flash_data_ready <= 1'b1;
                    if (flash_data_valid) begin
                        sdram_req_wdata  <= {flash_data, lo_byte};
                        sdram_req_addr   <= write_addr;
                        sdram_req        <= 1'b1;
                        sdram_req_write  <= 1'b1;
                        flash_data_ready <= 1'b0;

                        // Capture total_blocks lo byte (byte 1065 is a hi byte in the stream)
                        if (byte_count == HDR_BLKCNT_LO)
                            blkcnt_lo <= flash_data;

                        byte_count <= byte_count + 24'd1;
                        state      <= ST_WRITE_SDRAM;
                    end
                end

                // ---------------------------------------------------------
                // Wait for SDRAM to accept the write
                // ---------------------------------------------------------
                ST_WRITE_SDRAM: begin
                    if (sdram_req_ready) begin
                        sdram_req  <= 1'b0;
                        write_addr <= write_addr + 26'd2;
                        if (size_valid && byte_count >= image_size) begin
                            state <= ST_DONE;
                        end else begin
                            flash_data_ready <= 1'b1;
                            state            <= ST_COPY_LO;
                        end
                    end
                end

                // ---------------------------------------------------------
                // Boot complete — stay here forever
                // ---------------------------------------------------------
                ST_DONE: begin
                    boot_done        <= 1'b1;
                    sdram_req        <= 1'b0;
                    flash_data_ready <= 1'b0;
                end

                default: begin
                    state <= ST_WAIT_INIT;
                end
            endcase
        end
    end

endmodule
