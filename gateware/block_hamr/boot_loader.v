// boot_loader.v — Copies disk image from SPI flash into SDRAM at power-on
// Reads bytes from flash_reader, packs into 16-bit words, writes to SDRAM
// starting at address 0. Handles odd byte counts by zero-padding high byte.

`timescale 1ns / 1ps

module boot_loader #(
    parameter FLASH_OFFSET = 24'h400000,   // 4MB offset in flash (after bitstream)
    parameter IMAGE_SIZE   = 24'h800000    // 8,388,608 bytes = 8MB ProDOS volume
)(
    input  wire        clk,            // 25 MHz
    input  wire        rst_n,
    input  wire        sdram_init_done,
    output reg         boot_done = 1'b0,
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
    reg [23:0] bytes_remaining = 24'd0;
    reg [25:0] write_addr = 26'd0;

    // =========================================================================
    // State Machine
    // =========================================================================

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= ST_WAIT_INIT;
            boot_done       <= 1'b0;
            flash_start     <= 1'b0;
            flash_addr      <= 24'd0;
            flash_count     <= 24'd0;
            flash_data_ready <= 1'b0;
            sdram_req       <= 1'b0;
            sdram_req_write <= 1'b0;
            sdram_req_addr  <= 26'd0;
            sdram_req_wdata <= 16'd0;
            lo_byte         <= 8'd0;
            bytes_remaining <= 24'd0;
            write_addr      <= 26'd0;
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
                // Kick off the flash read
                // ---------------------------------------------------------
                ST_START_FLASH: begin
                    flash_start      <= 1'b1;
                    flash_addr       <= FLASH_OFFSET;
                    flash_count      <= IMAGE_SIZE;
                    flash_data_ready <= 1'b1;
                    bytes_remaining  <= IMAGE_SIZE;
                    write_addr       <= 26'd0;
                    state            <= ST_COPY_LO;
                end

                // ---------------------------------------------------------
                // Capture low byte from flash
                // ---------------------------------------------------------
                ST_COPY_LO: begin
                    sdram_req <= 1'b0;
                    if (flash_data_valid) begin
                        lo_byte          <= flash_data;
                        bytes_remaining  <= bytes_remaining - 24'd1;
                        flash_data_ready <= 1'b0;  // Stall flash after capture
                        if (bytes_remaining == 24'd1) begin
                            // This was the last byte and it's unpaired (odd count)
                            // Pad high byte with 0x00
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
                    // Assert data_ready to request the high byte
                    flash_data_ready <= 1'b1;
                    if (flash_data_valid) begin
                        sdram_req_wdata  <= {flash_data, lo_byte};
                        sdram_req_addr   <= write_addr;
                        sdram_req        <= 1'b1;
                        sdram_req_write  <= 1'b1;
                        flash_data_ready <= 1'b0;  // Stall flash during SDRAM write
                        bytes_remaining  <= bytes_remaining - 24'd1;
                        state            <= ST_WRITE_SDRAM;
                    end
                end

                // ---------------------------------------------------------
                // Wait for SDRAM to accept the write
                // ---------------------------------------------------------
                ST_WRITE_SDRAM: begin
                    if (sdram_req_ready) begin
                        sdram_req  <= 1'b0;
                        write_addr <= write_addr + 26'd2;
                        if (bytes_remaining == 24'd0) begin
                            state <= ST_DONE;
                        end else begin
                            flash_data_ready <= 1'b1;  // Resume flash reads
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
