// flash_reader.v — SPI flash reader for IS25LP128F
// Reads data using standard READ command (0x03) + 24-bit address
// SPI clock = CLK/2 = 12.5 MHz from 25 MHz system clock
// Targets ECP5 USRMCLK for post-configuration flash access

module flash_reader (
    input  wire        clk,            // 25 MHz
    input  wire        rst_n,
    input  wire        start,          // pulse to begin reading
    input  wire [23:0] start_addr,     // flash byte address
    input  wire [23:0] byte_count,     // number of bytes to read
    output reg         busy,
    output reg         done,
    output reg  [7:0]  data_out,
    output reg         data_valid,     // pulse per byte
    input  wire        data_ready,     // backpressure (stall if low)
    // SPI flash pins
    output reg         flash_ncs,
    output reg         flash_mosi,
    input  wire        flash_miso,
    output wire        flash_nwp,      // tied HIGH
    output wire        flash_nhold,    // tied HIGH
    output wire        flash_sck_pin   // for simulation visibility
);

    // ---------------------------------------------------------------
    // Write-protect and hold disabled
    // ---------------------------------------------------------------
    assign flash_nwp   = 1'b1;
    assign flash_nhold = 1'b1;

    // ---------------------------------------------------------------
    // SPI clock — directly driven by state machine logic
    // ---------------------------------------------------------------
    reg spi_sck;

    // USRMCLK now lives at top level (singleton — shared with flash_writer)
    assign flash_sck_pin = spi_sck;

    // ---------------------------------------------------------------
    // State encoding
    // ---------------------------------------------------------------
    localparam [2:0] S_IDLE      = 3'd0,
                     S_SEND_CMD  = 3'd1,
                     S_SEND_ADDR = 3'd2,
                     S_READ_DATA = 3'd3,
                     S_DONE      = 3'd4;

    reg [2:0]  state         = S_IDLE;
    reg [4:0]  bit_cnt       = 5'd0;
    reg [23:0] bytes_left    = 24'd0;
    reg [31:0] cmd_addr      = 32'd0;
    reg [7:0]  shift_in      = 8'd0;
    reg [2:0]  rx_bit_cnt    = 3'd0;
    reg        spi_clk_phase = 1'b0;
    reg        waiting_ack;   // stalled waiting for data_ready

    // ---------------------------------------------------------------
    // Main state machine
    // ---------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_IDLE;
            spi_sck       <= 1'b0;
            spi_clk_phase <= 1'b0;
            flash_ncs     <= 1'b1;
            flash_mosi    <= 1'b0;
            busy          <= 1'b0;
            done          <= 1'b0;
            data_out      <= 8'd0;
            data_valid    <= 1'b0;
            bit_cnt       <= 5'd0;
            bytes_left    <= 24'd0;
            cmd_addr      <= 32'd0;
            shift_in      <= 8'd0;
            rx_bit_cnt    <= 3'd0;
            waiting_ack   <= 1'b0;
        end else begin
            // Default: clear single-cycle pulses
            done       <= 1'b0;
            data_valid <= 1'b0;

            case (state)
                // ---------------------------------------------------
                // IDLE — wait for start pulse
                // ---------------------------------------------------
                S_IDLE: begin
                    flash_ncs     <= 1'b1;
                    spi_sck       <= 1'b0;
                    spi_clk_phase <= 1'b0;
                    flash_mosi    <= 1'b0;
                    busy          <= 1'b0;
                    waiting_ack   <= 1'b0;
                    if (start) begin
                        busy          <= 1'b1;
                        flash_ncs     <= 1'b0;
                        bytes_left    <= byte_count;
                        // Pack command + address into 32-bit shift register
                        cmd_addr      <= {8'h03, start_addr};
                        bit_cnt       <= 5'd0;
                        spi_clk_phase <= 1'b0;
                        state         <= S_SEND_CMD;
                    end
                end

                // ---------------------------------------------------
                // SEND_CMD — shift out 8-bit READ command (0x03)
                //   Uses cmd_addr[31:24], shifts left each bit
                // ---------------------------------------------------
                S_SEND_CMD: begin
                    if (spi_clk_phase == 1'b0) begin
                        // Setup phase: drive MOSI with MSB, SCK low
                        flash_mosi    <= cmd_addr[31];
                        spi_sck       <= 1'b0;
                        spi_clk_phase <= 1'b1;
                    end else begin
                        // Sample phase: raise SCK (slave latches MOSI)
                        spi_sck       <= 1'b1;
                        spi_clk_phase <= 1'b0;
                        cmd_addr      <= {cmd_addr[30:0], 1'b0};
                        bit_cnt       <= bit_cnt + 5'd1;
                        if (bit_cnt == 5'd7) begin
                            // Command byte done; address is now in
                            // cmd_addr[31:8] after the shifts
                            bit_cnt <= 5'd0;
                            state   <= S_SEND_ADDR;
                        end
                    end
                end

                // ---------------------------------------------------
                // SEND_ADDR — shift out 24-bit address MSB first
                //   cmd_addr has been shifted so address MSB is at [31]
                // ---------------------------------------------------
                S_SEND_ADDR: begin
                    if (spi_clk_phase == 1'b0) begin
                        flash_mosi    <= cmd_addr[31];
                        spi_sck       <= 1'b0;
                        spi_clk_phase <= 1'b1;
                    end else begin
                        spi_sck       <= 1'b1;
                        spi_clk_phase <= 1'b0;
                        cmd_addr      <= {cmd_addr[30:0], 1'b0};
                        bit_cnt       <= bit_cnt + 5'd1;
                        if (bit_cnt == 5'd23) begin
                            // All 24 address bits sent
                            bit_cnt    <= 5'd0;
                            rx_bit_cnt <= 3'd0;
                            shift_in   <= 8'd0;
                            state      <= S_READ_DATA;
                        end
                    end
                end

                // ---------------------------------------------------
                // READ_DATA — clock in bytes from MISO
                // ---------------------------------------------------
                S_READ_DATA: begin
                    if (waiting_ack) begin
                        // Byte complete but consumer not ready — stall
                        if (data_ready) begin
                            waiting_ack <= 1'b0;
                            data_valid  <= 1'b1;
                            if (bytes_left == 24'd0) begin
                                state <= S_DONE;
                            end
                        end
                        // else: hold — SPI clock frozen
                    end else begin
                        if (spi_clk_phase == 1'b0) begin
                            // Setup phase: SCK low
                            spi_sck       <= 1'b0;
                            flash_mosi    <= 1'b0;
                            spi_clk_phase <= 1'b1;
                        end else begin
                            // Sample phase: SCK high, capture MISO
                            spi_sck       <= 1'b1;
                            spi_clk_phase <= 1'b0;
                            shift_in      <= {shift_in[6:0], flash_miso};
                            rx_bit_cnt    <= rx_bit_cnt + 3'd1;
                            if (rx_bit_cnt == 3'd7) begin
                                // Byte complete
                                data_out   <= {shift_in[6:0], flash_miso};
                                bytes_left <= bytes_left - 24'd1;
                                rx_bit_cnt <= 3'd0;
                                shift_in   <= 8'd0;
                                if (data_ready) begin
                                    data_valid <= 1'b1;
                                    if (bytes_left == 24'd1) begin
                                        state <= S_DONE;
                                    end
                                end else begin
                                    waiting_ack <= 1'b1;
                                end
                            end
                        end
                    end
                end

                // ---------------------------------------------------
                // DONE — deassert nCS, pulse done, return to idle
                // ---------------------------------------------------
                S_DONE: begin
                    flash_ncs <= 1'b1;
                    spi_sck   <= 1'b0;
                    done      <= 1'b1;
                    busy      <= 1'b0;
                    state     <= S_IDLE;
                end

                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule
