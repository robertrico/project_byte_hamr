// flash_writer.v — SPI flash write controller for IS25LP128F
// Supports SECTOR_ERASE (0x20) and PAGE_PROGRAM (0x02) with
// automatic WRITE_ENABLE (0x06) and WIP status polling (0x05).
// SPI clock = CLK/2 = 12.5 MHz from 25 MHz system clock.
//
// SPI bit protocol (same as flash_reader):
//   Phase 0: drive MOSI, SCK low (setup)
//   Phase 1: SCK high (flash samples MOSI / we sample MISO)
// Each byte = 16 CLK cycles = 640ns at 25 MHz.
//
// Usage:
//   Erase: assert start_erase for 1 cycle, provide flash_addr
//   Program: assert start_program for 1 cycle, provide flash_addr,
//            then feed 256 bytes via prog_data when prog_data_req pulses

`timescale 1ns / 1ps

module flash_writer (
    input  wire        clk,            // 25 MHz
    input  wire        rst_n,

    // Command interface
    input  wire        start_erase,    // pulse: begin sector erase
    input  wire        start_program,  // pulse: begin page program (256 bytes)
    input  wire [23:0] flash_addr,     // sector or page address

    // Page program data stream
    input  wire [7:0]  prog_data,      // byte to program (valid when prog_data_valid=1)
    input  wire        prog_data_valid,// HIGH when prog_data is ready to latch
    output reg         prog_data_req,  // pulse: request next byte

    // Status
    output reg         busy,
    output reg         done,           // single-cycle pulse on completion

    // SPI pins (active after boot_done via top-level mux)
    output reg         spi_sck,
    output reg         spi_ncs,
    output reg         spi_mosi,
    input  wire        spi_miso
);

    // =========================================================================
    // SPI flash command bytes
    // =========================================================================
    localparam CMD_WREN          = 8'h06;
    localparam CMD_SECTOR_ERASE  = 8'h20;
    localparam CMD_PAGE_PROGRAM  = 8'h02;
    localparam CMD_READ_STATUS   = 8'h05;

    // =========================================================================
    // State machine
    // =========================================================================
    localparam [3:0]
        FW_IDLE       = 4'd0,
        FW_WREN       = 4'd1,   // shift out WREN command (8 bits)
        FW_WREN_GAP   = 4'd2,   // nCS high pause
        FW_CMD        = 4'd3,   // shift out erase/program command (8 bits)
        FW_ADDR       = 4'd4,   // shift out 24-bit address
        FW_DATA       = 4'd5,   // shift out 256 data bytes (page program only)
        FW_END_CMD    = 4'd6,   // lower SCK after last bit, then deselect
        FW_CMD_GAP    = 4'd7,   // nCS high, prepare status poll
        FW_POLL_CMD   = 4'd8,   // shift out READ_STATUS command (8 bits)
        FW_POLL_DATA  = 4'd9,   // shift in status byte
        FW_POLL_CHECK = 4'd10,  // check WIP bit
        FW_DONE       = 4'd11;

    reg [3:0]   state;
    reg         spi_phase;          // 0=setup(SCK low), 1=sample(SCK high)
    reg [4:0]   bit_cnt;            // bit counter
    reg [31:0]  shift_out;          // outgoing shift register
    reg [7:0]   shift_in;           // incoming shift register (status)
    reg [8:0]   byte_cnt;           // byte counter for page program
    reg         is_program;         // 1=page program, 0=sector erase
    reg [23:0]  latched_addr;
    reg [7:0]   tx_byte;            // current byte being shifted out
    reg [3:0]   next_after_end;     // state to go to after FW_END_CMD

    // =========================================================================
    // Main state machine
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= FW_IDLE;
            spi_sck        <= 1'b0;
            spi_ncs        <= 1'b1;
            spi_mosi       <= 1'b0;
            spi_phase      <= 1'b0;
            bit_cnt        <= 5'd0;
            shift_out      <= 32'd0;
            shift_in       <= 8'd0;
            byte_cnt       <= 9'd0;
            busy           <= 1'b0;
            done           <= 1'b0;
            is_program     <= 1'b0;
            latched_addr   <= 24'd0;
            prog_data_req  <= 1'b0;
            tx_byte        <= 8'd0;
            next_after_end <= FW_IDLE;
        end else begin
            // Defaults
            done          <= 1'b0;
            prog_data_req <= 1'b0;

            case (state)
                // =============================================================
                // IDLE
                // =============================================================
                FW_IDLE: begin
                    spi_ncs   <= 1'b1;
                    spi_sck   <= 1'b0;
                    spi_phase <= 1'b0;
                    busy      <= 1'b0;

                    if (start_erase || start_program) begin
                        busy         <= 1'b1;
                        is_program   <= start_program;
                        latched_addr <= flash_addr;
                        spi_ncs      <= 1'b0;
                        shift_out    <= {CMD_WREN, 24'd0};
                        bit_cnt      <= 5'd0;
                        spi_phase    <= 1'b0;
                        state        <= FW_WREN;
                    end
                end

                // =============================================================
                // WREN — shift out 8-bit Write Enable command
                // =============================================================
                FW_WREN: begin
                    if (!spi_phase) begin
                        spi_mosi  <= shift_out[31];
                        spi_sck   <= 1'b0;
                        spi_phase <= 1'b1;
                    end else begin
                        spi_sck   <= 1'b1;
                        spi_phase <= 1'b0;
                        shift_out <= {shift_out[30:0], 1'b0};
                        bit_cnt   <= bit_cnt + 5'd1;
                        if (bit_cnt == 5'd7) begin
                            // Last bit clocked — go to END_CMD to deselect
                            next_after_end <= FW_WREN_GAP;
                            state          <= FW_END_CMD;
                        end
                    end
                end

                // =============================================================
                // WREN_GAP — nCS was just raised by END_CMD, re-select for next cmd
                // =============================================================
                FW_WREN_GAP: begin
                    spi_ncs   <= 1'b0;  // re-select
                    spi_phase <= 1'b0;
                    bit_cnt   <= 5'd0;
                    shift_out <= {(is_program ? CMD_PAGE_PROGRAM : CMD_SECTOR_ERASE), 24'd0};
                    state     <= FW_CMD;
                end

                // =============================================================
                // CMD — shift out 8-bit command (0x20 or 0x02)
                // =============================================================
                FW_CMD: begin
                    if (!spi_phase) begin
                        spi_mosi  <= shift_out[31];
                        spi_sck   <= 1'b0;
                        spi_phase <= 1'b1;
                    end else begin
                        spi_sck   <= 1'b1;
                        spi_phase <= 1'b0;
                        shift_out <= {shift_out[30:0], 1'b0};
                        bit_cnt   <= bit_cnt + 5'd1;
                        if (bit_cnt == 5'd7) begin
                            bit_cnt   <= 5'd0;
                            shift_out <= {latched_addr, 8'd0};
                            state     <= FW_ADDR;
                        end
                    end
                end

                // =============================================================
                // ADDR — shift out 24-bit address
                // =============================================================
                FW_ADDR: begin
                    if (!spi_phase) begin
                        spi_mosi  <= shift_out[31];
                        spi_sck   <= 1'b0;
                        spi_phase <= 1'b1;
                    end else begin
                        spi_sck   <= 1'b1;
                        spi_phase <= 1'b0;
                        shift_out <= {shift_out[30:0], 1'b0};
                        bit_cnt   <= bit_cnt + 5'd1;
                        if (bit_cnt == 5'd23) begin
                            bit_cnt <= 5'd0;
                            if (is_program) begin
                                // Don't fire prog_data_req here — FW_DATA
                                // fires it at byte 0 start after latching.
                                byte_cnt      <= 9'd0;
                                state         <= FW_DATA;
                            end else begin
                                // Erase: deselect after address
                                next_after_end <= FW_CMD_GAP;
                                state          <= FW_END_CMD;
                            end
                        end
                    end
                end

                // =============================================================
                // DATA — shift out 256 bytes for page program
                // =============================================================
                FW_DATA: begin
                    if (!spi_phase) begin
                        if (bit_cnt == 5'd0 && !prog_data_valid) begin
                            // Stall: SDRAM read not done yet.
                            // Hold SCK low — flash doesn't care about pauses.
                        end else begin
                            if (bit_cnt == 5'd0) begin
                                tx_byte <= prog_data;
                                // Signal "I latched this byte, advance."
                                // For bytes 0-254: caller prepares next byte.
                                // For byte 255: caller transitions to NEXT_PAGE.
                                prog_data_req <= 1'b1;
                            end
                            spi_mosi  <= (bit_cnt == 5'd0) ? prog_data[7] : tx_byte[7 - bit_cnt[2:0]];
                            spi_sck   <= 1'b0;
                            spi_phase <= 1'b1;
                        end
                    end else begin
                        spi_sck   <= 1'b1;
                        spi_phase <= 1'b0;
                        bit_cnt   <= bit_cnt + 5'd1;
                        if (bit_cnt[2:0] == 3'd7) begin
                            bit_cnt  <= 5'd0;
                            byte_cnt <= byte_cnt + 9'd1;
                            if (byte_cnt == 9'd255) begin
                                next_after_end <= FW_CMD_GAP;
                                state          <= FW_END_CMD;
                            end
                        end
                    end
                end

                // =============================================================
                // END_CMD — SCK low, then deselect nCS. Bridges shift states
                // to gap states without losing the last SCK rising edge.
                // =============================================================
                FW_END_CMD: begin
                    spi_sck <= 1'b0;
                    spi_ncs <= 1'b1;
                    state   <= next_after_end;
                end

                // =============================================================
                // CMD_GAP — nCS high after erase/program, prepare status poll
                // =============================================================
                FW_CMD_GAP: begin
                    spi_sck   <= 1'b0;
                    spi_phase <= 1'b0;
                    bit_cnt   <= 5'd0;
                    spi_ncs   <= 1'b0;
                    shift_out <= {CMD_READ_STATUS, 24'd0};
                    state     <= FW_POLL_CMD;
                end

                // =============================================================
                // POLL_CMD — shift out READ_STATUS command (0x05)
                // =============================================================
                FW_POLL_CMD: begin
                    if (!spi_phase) begin
                        spi_mosi  <= shift_out[31];
                        spi_sck   <= 1'b0;
                        spi_phase <= 1'b1;
                    end else begin
                        spi_sck   <= 1'b1;
                        spi_phase <= 1'b0;
                        shift_out <= {shift_out[30:0], 1'b0};
                        bit_cnt   <= bit_cnt + 5'd1;
                        if (bit_cnt == 5'd7) begin
                            bit_cnt  <= 5'd0;
                            shift_in <= 8'd0;
                            state    <= FW_POLL_DATA;
                        end
                    end
                end

                // =============================================================
                // POLL_DATA — shift in 8-bit status register
                // =============================================================
                FW_POLL_DATA: begin
                    if (!spi_phase) begin
                        spi_sck   <= 1'b0;
                        spi_mosi  <= 1'b0;
                        spi_phase <= 1'b1;
                    end else begin
                        spi_sck   <= 1'b1;
                        spi_phase <= 1'b0;
                        shift_in  <= {shift_in[6:0], spi_miso};
                        bit_cnt   <= bit_cnt + 5'd1;
                        if (bit_cnt == 5'd7) begin
                            state <= FW_POLL_CHECK;
                        end
                    end
                end

                // =============================================================
                // POLL_CHECK — check WIP bit (bit 0 of status register)
                // =============================================================
                FW_POLL_CHECK: begin
                    spi_sck <= 1'b0;
                    spi_ncs <= 1'b1;

                    if (shift_in[0]) begin
                        // WIP still set — poll again
                        spi_phase <= 1'b0;
                        bit_cnt   <= 5'd0;
                        spi_ncs   <= 1'b0;
                        shift_out <= {CMD_READ_STATUS, 24'd0};
                        state     <= FW_POLL_CMD;
                    end else begin
                        state <= FW_DONE;
                    end
                end

                // =============================================================
                // DONE
                // =============================================================
                FW_DONE: begin
                    spi_ncs <= 1'b1;
                    spi_sck <= 1'b0;
                    done    <= 1'b1;
                    busy    <= 1'b0;
                    state   <= FW_IDLE;
                end

                default: state <= FW_IDLE;
            endcase
        end
    end

endmodule
