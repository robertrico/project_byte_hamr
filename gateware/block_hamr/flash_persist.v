// flash_persist.v — Flash sector erase + page program from SDRAM
//
// Separate module for flash operations to avoid Yosys ECP5 synthesis
// corruption when flash logic is combined with block_ready gating in
// a single state machine (same class of bug as sdram_arbiter corruption).
//
// Interface: pulse `start` with `sector_num` to erase+program one 4KB sector.
// Asserts `busy` while working. Claims SDRAM via `sdram_claim`.

`timescale 1ns / 1ps

module flash_persist (
    input  wire        clk,
    input  wire        rst_n,

    // Command interface
    input  wire        start,          // pulse: begin erase+program
    input  wire [5:0]  sector_num,     // which sector (0-34, 6 bits for 280-block disk)
    output reg         busy = 1'b0,

    // SDRAM read interface
    output reg         sdram_req = 1'b0,
    output reg  [25:0] sdram_addr = 26'd0,
    output wire [15:0] sdram_wdata,
    input  wire        sdram_ready,
    input  wire [15:0] sdram_rdata,
    input  wire        sdram_rdata_valid,

    // Flash writer interface
    output reg         fw_start_erase = 1'b0,
    output reg         fw_start_program = 1'b0,
    output reg  [23:0] fw_flash_addr = 24'd0,
    output wire [7:0]  fw_prog_data,
    output wire        fw_prog_data_valid,
    input  wire        fw_prog_data_req,
    input  wire        fw_busy,

    // SDRAM claim
    output reg         sdram_claim = 1'b0
);

    localparam [23:0] FLASH_OFFSET = 24'h400000;

    // State machine
    localparam [3:0]
        FP_IDLE         = 4'd0,
        FP_ERASE        = 4'd1,
        FP_ERASE_WAIT   = 4'd2,
        FP_PAGE_SETUP   = 4'd3,
        FP_SDRAM_SETUP  = 4'd4,
        FP_SDRAM_ISSUE  = 4'd5,
        FP_SDRAM_WAIT   = 4'd6,
        FP_STREAM_WAIT  = 4'd7,
        FP_NEXT_PAGE    = 4'd8,
        FP_DONE         = 4'd9;

    reg [3:0]   state = FP_IDLE;
    reg [5:0]   sector = 6'd0;
    reg [15:0]  rd_data_latch = 16'd0;
    reg [4:0]   page = 5'd0;
    reg [7:0]   word = 8'd0;
    reg         feed_hi = 1'b0;

    // Address computation
    wire [23:0] sector_flash_addr = FLASH_OFFSET + {6'd0, sector, 12'd0};
    wire [23:0] page_flash_addr   = FLASH_OFFSET + {6'd0, sector, page[3:0], 8'd0};
    wire [25:0] sdram_word_addr   = {8'd0, sector, page[3:0], word[6:0], 1'b0};

    // Data streaming
    assign fw_prog_data       = feed_hi ? rd_data_latch[15:8] : rd_data_latch[7:0];
    assign fw_prog_data_valid = (state == FP_STREAM_WAIT);
    assign sdram_wdata        = 16'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= FP_IDLE;
            busy           <= 1'b0;
            sdram_claim    <= 1'b0;
            sector         <= 6'd0;
            rd_data_latch  <= 16'd0;
            page           <= 5'd0;
            word           <= 8'd0;
            feed_hi        <= 1'b0;
            sdram_req      <= 1'b0;
            sdram_addr     <= 26'd0;
            fw_start_erase <= 1'b0;
            fw_start_program <= 1'b0;
            fw_flash_addr  <= 24'd0;
        end else begin
            sdram_req        <= 1'b0;
            fw_start_erase   <= 1'b0;
            fw_start_program <= 1'b0;

            case (state)
                FP_IDLE: begin
                    busy        <= 1'b0;
                    sdram_claim <= 1'b0;
                    if (start) begin
                        busy           <= 1'b1;
                        sdram_claim    <= 1'b1;
                        sector         <= sector_num;
                        fw_flash_addr  <= FLASH_OFFSET + {6'd0, sector_num, 12'd0};
                        fw_start_erase <= 1'b1;
                        state          <= FP_ERASE;
                    end
                end

                FP_ERASE: begin
                    if (fw_busy)
                        state <= FP_ERASE_WAIT;
                end

                FP_ERASE_WAIT: begin
                    if (!fw_busy) begin
                        page  <= 5'd0;
                        state <= FP_PAGE_SETUP;
                    end
                end

                FP_PAGE_SETUP: begin
                    if (page == 5'd16) begin
                        state <= FP_DONE;
                    end else begin
                        fw_flash_addr    <= page_flash_addr;
                        fw_start_program <= 1'b1;
                        word             <= 8'd0;
                        feed_hi          <= 1'b0;
                        state            <= FP_SDRAM_SETUP;
                    end
                end

                FP_SDRAM_SETUP: begin
                    sdram_addr <= sdram_word_addr;
                    sdram_req  <= 1'b1;
                    state      <= FP_SDRAM_ISSUE;
                end

                FP_SDRAM_ISSUE: begin
                    sdram_req <= 1'b1;
                    if (sdram_ready) begin
                        sdram_req <= 1'b0;
                        state     <= FP_SDRAM_WAIT;
                    end
                end

                FP_SDRAM_WAIT: begin
                    if (sdram_rdata_valid) begin
                        rd_data_latch <= sdram_rdata;
                        feed_hi       <= 1'b0;
                        state         <= FP_STREAM_WAIT;
                    end
                end

                FP_STREAM_WAIT: begin
                    if (fw_prog_data_req) begin
                        if (!feed_hi) begin
                            feed_hi <= 1'b1;
                        end else begin
                            feed_hi <= 1'b0;
                            if (word == 8'd127) begin
                                page  <= page + 5'd1;
                                state <= FP_NEXT_PAGE;
                            end else begin
                                word  <= word + 8'd1;
                                state <= FP_SDRAM_SETUP;
                            end
                        end
                    end
                end

                FP_NEXT_PAGE: begin
                    if (!fw_busy)
                        state <= FP_PAGE_SETUP;
                end

                FP_DONE: begin
                    busy        <= 1'b0;
                    sdram_claim <= 1'b0;
                    state       <= FP_IDLE;
                end

                default: state <= FP_IDLE;
            endcase
        end
    end

endmodule
