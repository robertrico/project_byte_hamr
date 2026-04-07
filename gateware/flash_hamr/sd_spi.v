// sd_spi.v — Byte-level SPI transceiver for SD card
//
// SPI Mode 0 (CPOL=0, CPHA=0):
//   - MOSI driven on SCK falling edge (setup phase)
//   - MISO sampled on SCK rising edge (sample phase)
//   - SCK idles LOW
//   - MSB first
//
// Dual clock speed:
//   slow_mode=1: SCK = CLK/128 = ~195kHz at 25MHz (SD init requires ≤400kHz)
//   slow_mode=0: SCK = CLK/2   = 12.5MHz at 25MHz (SD data transfer)
//
// Interface: pulse start, provide tx_byte. When done pulses, rx_byte is valid.
// Back-to-back: pulse start on the same cycle done fires.

`timescale 1ns / 1ps

module sd_spi (
    input  wire        clk,        // 25 MHz
    input  wire        rst_n,

    // Byte transfer interface
    input  wire        start,      // pulse: begin 8-bit transfer
    input  wire [7:0]  tx_byte,    // data to send (latched on start)
    output reg  [7:0]  rx_byte,    // data received (valid when done pulses)
    output reg         done,       // single-cycle pulse: transfer complete
    output wire        busy,       // HIGH during transfer

    // Clock speed control
    input  wire        slow_mode,  // 1 = ~195kHz, 0 = 12.5MHz

    // SPI pins (directly to GPIO)
    output reg         spi_sck,
    output reg         spi_mosi,
    input  wire        spi_miso
);

    // =========================================================================
    // State
    // =========================================================================
    localparam S_IDLE = 1'b0,
               S_XFER = 1'b1;

    reg        state = S_IDLE;
    reg [7:0]  shift_out;       // TX shift register (MSB first)
    reg [7:0]  shift_in;        // RX shift register
    reg [2:0]  bit_cnt;         // 0..7
    reg        spi_phase;       // 0 = setup (SCK low, drive MOSI)
                                // 1 = sample (SCK high, capture MISO)
    reg [6:0]  clk_div;         // clock divider counter (0..63 in slow mode)
    reg        clk_tick;        // fires when divider reaches threshold

    assign busy = (state == S_XFER);

    // =========================================================================
    // Clock divider
    // =========================================================================
    // slow_mode: divide by 64 per half-period → SCK period = 128 CLK cycles
    //   25MHz / 128 = ~195kHz
    // fast_mode: divide by 4 per half-period → SCK period = 8 CLK cycles
    //   25MHz / 8 = 3.125MHz
    wire [6:0] div_threshold = slow_mode ? 7'd63 : 7'd3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_div  <= 7'd0;
            clk_tick <= 1'b0;
        end else if (state == S_XFER) begin
            if (clk_div == div_threshold) begin
                clk_div  <= 7'd0;
                clk_tick <= 1'b1;
            end else begin
                clk_div  <= clk_div + 7'd1;
                clk_tick <= 1'b0;
            end
        end else begin
            clk_div  <= 7'd0;
            clk_tick <= 1'b0;
        end
    end

    // =========================================================================
    // Main state machine
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            spi_sck   <= 1'b0;
            spi_mosi  <= 1'b1;     // MOSI idles high (SD convention)
            rx_byte   <= 8'd0;
            done      <= 1'b0;
            shift_out <= 8'd0;
            shift_in  <= 8'd0;
            bit_cnt   <= 3'd0;
            spi_phase <= 1'b0;
        end else begin
            done <= 1'b0;   // default: clear pulse

            case (state)
                S_IDLE: begin
                    spi_sck <= 1'b0;
                    if (start) begin
                        shift_out <= tx_byte;
                        shift_in  <= 8'd0;
                        bit_cnt   <= 3'd0;
                        spi_phase <= 1'b0;
                        state     <= S_XFER;
                        // Drive first MOSI bit immediately (setup phase)
                        spi_mosi  <= tx_byte[7];
                    end
                end

                S_XFER: begin
                    if (clk_tick) begin
                        if (!spi_phase) begin
                            // === Setup phase complete → raise SCK (sample) ===
                            spi_sck   <= 1'b1;
                            spi_phase <= 1'b1;
                        end else begin
                            // === Sample phase complete → capture MISO, lower SCK ===
                            spi_sck  <= 1'b0;
                            shift_in <= {shift_in[6:0], spi_miso};

                            if (bit_cnt == 3'd7) begin
                                // Last bit done
                                rx_byte <= {shift_in[6:0], spi_miso};
                                done    <= 1'b1;

                                // Check for back-to-back start
                                if (start) begin
                                    shift_out <= tx_byte;
                                    shift_in  <= 8'd0;
                                    bit_cnt   <= 3'd0;
                                    spi_phase <= 1'b0;
                                    spi_mosi  <= tx_byte[7];
                                end else begin
                                    spi_mosi <= 1'b1;
                                    state    <= S_IDLE;
                                end
                            end else begin
                                // Advance to next bit
                                bit_cnt   <= bit_cnt + 3'd1;
                                shift_out <= {shift_out[6:0], 1'b0};
                                spi_mosi  <= shift_out[6]; // next MSB
                                spi_phase <= 1'b0;
                            end
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
