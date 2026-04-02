// boot_loader.v — Flash Hamr boot loader (flash copy ONLY)
//
// Copies menu volume from SPI flash to SDRAM at power-on.
// Same as Block Hamr's proven boot_loader — no SD card involvement.
// SD init is triggered later via register write from the 6502.

`timescale 1ns / 1ps

module boot_loader #(
    parameter FLASH_OFFSET = 24'h400000,
    parameter MAX_BLOCKS   = 16'd280
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sdram_init_done,
    output reg         boot_done,
    output reg  [15:0] total_blocks,

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
    input  wire        sdram_req_ready
);

    localparam [23:0] HDR_BLKCNT_LO = 24'd1065;
    localparam [23:0] HDR_BLKCNT_HI = 24'd1066;
    localparam [23:0] MAX_IMAGE_BYTES = {MAX_BLOCKS[14:0], 9'd0};

    localparam [2:0]
        ST_WAIT_INIT   = 3'd0,
        ST_START_FLASH = 3'd1,
        ST_COPY_LO     = 3'd2,
        ST_COPY_HI     = 3'd3,
        ST_WRITE_SDRAM = 3'd4,
        ST_DONE        = 3'd5;

    reg [2:0]  state = ST_WAIT_INIT;
    reg [7:0]  lo_byte;
    reg [23:0] byte_count;
    reg [25:0] write_addr;
    reg [23:0] image_size;
    reg [7:0]  blkcnt_lo;
    reg        size_valid;

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
            flash_start <= 1'b0;

            case (state)
                ST_WAIT_INIT: begin
                    boot_done        <= 1'b0;
                    flash_data_ready <= 1'b0;
                    sdram_req        <= 1'b0;
                    if (sdram_init_done)
                        state <= ST_START_FLASH;
                end

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

                ST_COPY_LO: begin
                    sdram_req <= 1'b0;
                    if (flash_data_valid) begin
                        lo_byte          <= flash_data;
                        flash_data_ready <= 1'b0;

                        if (byte_count == HDR_BLKCNT_HI && !size_valid) begin
                            total_blocks <= {flash_data, blkcnt_lo};
                            if ({flash_data, blkcnt_lo} == 16'd0 ||
                                {flash_data, blkcnt_lo} > MAX_BLOCKS)
                                image_size <= MAX_IMAGE_BYTES;
                            else
                                image_size <= {flash_data[6:0], blkcnt_lo, 9'd0};
                            size_valid <= 1'b1;
                        end

                        byte_count <= byte_count + 24'd1;

                        if (size_valid && byte_count + 24'd1 >= image_size) begin
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

                ST_COPY_HI: begin
                    flash_data_ready <= 1'b1;
                    if (flash_data_valid) begin
                        sdram_req_wdata  <= {flash_data, lo_byte};
                        sdram_req_addr   <= write_addr;
                        sdram_req        <= 1'b1;
                        sdram_req_write  <= 1'b1;
                        flash_data_ready <= 1'b0;

                        if (byte_count == HDR_BLKCNT_LO)
                            blkcnt_lo <= flash_data;

                        byte_count <= byte_count + 24'd1;
                        state      <= ST_WRITE_SDRAM;
                    end
                end

                ST_WRITE_SDRAM: begin
                    if (sdram_req_ready) begin
                        sdram_req  <= 1'b0;
                        write_addr <= write_addr + 26'd2;
                        if (size_valid && byte_count >= image_size)
                            state <= ST_DONE;
                        else begin
                            flash_data_ready <= 1'b1;
                            state            <= ST_COPY_LO;
                        end
                    end
                end

                ST_DONE: begin
                    boot_done        <= 1'b1;
                    sdram_req        <= 1'b0;
                    flash_data_ready <= 1'b0;
                end

                default: state <= ST_WAIT_INIT;
            endcase
        end
    end

endmodule
