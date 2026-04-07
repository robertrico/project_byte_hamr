`timescale 1ns / 1ps
// =============================================================================
// spi_master.v — Memory-mapped SPI byte transceiver for SD card
// =============================================================================
// SPI Mode 0 (CPOL=0, CPHA=0): SCK idles LOW, MOSI driven on SCK falling
// edge (setup phase), MISO sampled on SCK rising edge (sample phase).
// Modeled after flash_reader.v phase pattern.
//
// Dual speed via clock divider:
//   slow_mode=1: SCK = CLK/(2*(DIV_SLOW+1)) ≈ 195 kHz at 25 MHz (SD init)
//   slow_mode=0: SCK = CLK/(2*(DIV_FAST+1)) ≈ 3.125 MHz at 25 MHz (SD data)
//
// Register interface (active when active=1):
//   Offset 0x00  SPI_DATA    W: TX byte (starts transfer)  R: RX byte
//   Offset 0x04  SPI_STATUS  R: {30'b0, done, busy}
//   Offset 0x08  SPI_CTRL    W: {29'b0, slow_mode, cs_manual, cs_val}
//                                bit 0: cs_val (directly drives CS pin)
//                                bit 1: cs_manual (1=firmware controls CS)
//                                bit 2: slow_mode (1=~195kHz, 0=~3MHz)
// =============================================================================

module spi_master #(
    parameter DIV_SLOW = 63,   // 25MHz / (2*64) ≈ 195 kHz
    parameter DIV_FAST = 3     // 25MHz / (2*4)  ≈ 3.125 MHz
)(
    input  wire        clk,        // 25 MHz
    input  wire        rst_n,

    // CPU bus interface
    input  wire        active,     // address decode hit
    input  wire [3:0]  addr,       // register offset (word-aligned, [3:2] used)
    input  wire [31:0] wdata,
    input  wire [3:0]  wstrb,      // non-zero = write
    output reg  [31:0] rdata,
    output reg         ready,

    // SPI pins
    output reg         spi_sck,
    output reg         spi_mosi,
    input  wire        spi_miso,
    output wire        spi_cs
);

    // =========================================================================
    // Control register state
    // =========================================================================
    reg        cs_val    = 1'b1;   // CS pin value (1 = deasserted)
    reg        slow_mode = 1'b1;   // Start in slow mode for SD init

    assign spi_cs = cs_val;

    // =========================================================================
    // Transfer state
    // =========================================================================
    localparam S_IDLE = 1'b0,
               S_XFER = 1'b1;

    reg        state = S_IDLE;
    reg [7:0]  shift_out;          // TX shift register (MSB first)
    reg [7:0]  shift_in;           // RX shift register
    reg [7:0]  rx_byte;            // Latched RX result
    reg [2:0]  bit_cnt;            // 0..7
    reg        spi_phase;          // 0=setup (SCK low), 1=sample (SCK high)
    reg [6:0]  clk_div;            // Clock divider counter
    reg        done_flag;          // Set for 1 cycle when transfer completes

    wire       busy = (state == S_XFER);

    // Clock divider threshold
    wire [6:0] div_threshold = slow_mode ? DIV_SLOW[6:0] : DIV_FAST[6:0];

    // =========================================================================
    // Clock divider — generates tick when counter reaches threshold
    // =========================================================================
    reg clk_tick;

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
    // SPI state machine (flash_reader.v phase model)
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            spi_sck   <= 1'b0;
            spi_mosi  <= 1'b1;     // MOSI idles high (SD convention)
            rx_byte   <= 8'hFF;
            shift_out <= 8'hFF;
            shift_in  <= 8'd0;
            bit_cnt   <= 3'd0;
            spi_phase <= 1'b0;
            done_flag <= 1'b0;
        end else begin
            done_flag <= 1'b0;     // Default: clear pulse

            case (state)
                S_IDLE: begin
                    spi_sck <= 1'b0;
                    // Transfer starts when CPU writes to SPI_DATA
                    if (active && (addr[3:2] == 2'b00) && (wstrb != 4'h0)) begin
                        shift_out <= wdata[7:0];
                        shift_in  <= 8'd0;
                        bit_cnt   <= 3'd0;
                        spi_phase <= 1'b0;
                        state     <= S_XFER;
                        // Drive first MOSI bit immediately (setup phase)
                        spi_mosi  <= wdata[7];
                    end
                end

                S_XFER: begin
                    if (clk_tick) begin
                        if (!spi_phase) begin
                            // === Setup complete -> raise SCK (sample) ===
                            spi_sck   <= 1'b1;
                            spi_phase <= 1'b1;
                        end else begin
                            // === Sample complete -> capture MISO, lower SCK ===
                            spi_sck  <= 1'b0;
                            shift_in <= {shift_in[6:0], spi_miso};

                            if (bit_cnt == 3'd7) begin
                                // Last bit done
                                rx_byte   <= {shift_in[6:0], spi_miso};
                                done_flag <= 1'b1;
                                spi_mosi  <= 1'b1;  // Idle high
                                state     <= S_IDLE;
                            end else begin
                                // Advance to next bit
                                bit_cnt   <= bit_cnt + 3'd1;
                                shift_out <= {shift_out[6:0], 1'b0};
                                spi_mosi  <= shift_out[6];  // Next MSB
                                spi_phase <= 1'b0;
                            end
                        end
                    end
                end
            endcase
        end
    end

    // =========================================================================
    // Register read/write interface
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cs_val    <= 1'b1;
            slow_mode <= 1'b1;
            ready     <= 1'b0;
        end else begin
            ready <= 1'b0;

            if (active && !ready) begin
                ready <= 1'b1;

                case (addr[3:2])
                    2'b00: begin  // SPI_DATA
                        rdata <= {24'd0, rx_byte};
                        // Write handled in state machine above
                    end
                    2'b01: begin  // SPI_STATUS
                        rdata <= {30'd0, done_flag, busy};
                    end
                    2'b10: begin  // SPI_CTRL
                        rdata <= {29'd0, slow_mode, 1'b0, cs_val};
                        if (wstrb != 4'h0) begin
                            cs_val    <= wdata[0];
                            slow_mode <= wdata[2];
                        end
                    end
                    default: rdata <= 32'd0;
                endcase
            end
        end
    end

endmodule
