// sdram_arbiter.v — Multiplexes SDRAM access between boot_loader (boot)
// and bus_interface (runtime).  Handles block-level transfers between SDRAM
// and the dual-port block buffer.

module sdram_arbiter (
    input  wire        clk,            // 25 MHz
    input  wire        rst_n,
    input  wire        boot_done,      // from boot_loader

    // Boot loader port (active during boot only)
    input  wire        boot_req,
    input  wire        boot_write,
    input  wire [25:0] boot_addr,
    input  wire [15:0] boot_wdata,
    output wire        boot_ready,

    // Device engine port (block-level requests, CDC from 7 MHz)
    input  wire        dev_block_read_req,   // 7 MHz domain — synchronized here
    input  wire        dev_block_write_req,  // 7 MHz domain — synchronized here
    input  wire [15:0] dev_block_num,        // stable before req asserted
    output reg         dev_block_ready = 1'b0,      // synchronized to 7 MHz by consumer

    // Block buffer Port A (25 MHz side)
    output reg  [8:0]  buf_addr_a = 9'd0,
    output reg  [7:0]  buf_wdata_a = 8'd0,
    output reg         buf_we_a = 1'b0,
    input  wire [7:0]  buf_rdata_a,

    // SDRAM controller interface
    output wire        sdram_req,
    output wire        sdram_write,
    output wire [25:0] sdram_addr,
    output wire [15:0] sdram_wdata,
    input  wire        sdram_ready,
    input  wire [15:0] sdram_rdata,
    input  wire        sdram_rdata_valid
);

    // -----------------------------------------------------------------
    // CDC: 2-FF synchronizers for 7 MHz → 25 MHz crossing
    // -----------------------------------------------------------------
    reg dev_read_r1 = 1'b0, dev_read_r2 = 1'b0;
    reg dev_write_r1 = 1'b0, dev_write_r2 = 1'b0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dev_read_r1  <= 1'b0;
            dev_read_r2  <= 1'b0;
            dev_write_r1 <= 1'b0;
            dev_write_r2 <= 1'b0;
        end else begin
            dev_read_r1  <= dev_block_read_req;
            dev_read_r2  <= dev_read_r1;
            dev_write_r1 <= dev_block_write_req;
            dev_write_r2 <= dev_write_r1;
        end
    end

    wire dev_read_sync  = dev_read_r2;
    wire dev_write_sync = dev_write_r2;

    // Rising-edge detectors
    reg dev_read_prev = 1'b0, dev_write_prev = 1'b0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dev_read_prev  <= 1'b0;
            dev_write_prev <= 1'b0;
        end else begin
            dev_read_prev  <= dev_read_sync;
            dev_write_prev <= dev_write_sync;
        end
    end

    wire dev_read_rise  = dev_read_sync  & ~dev_read_prev;
    wire dev_write_rise = dev_write_sync & ~dev_write_prev;

    // -----------------------------------------------------------------
    // State machine
    // -----------------------------------------------------------------
    localparam S_IDLE                = 4'd0;
    localparam S_BLOCK_READ_SETUP   = 4'd1;
    localparam S_BLOCK_READ_ISSUE   = 4'd2;
    localparam S_BLOCK_READ_WAIT    = 4'd3;
    localparam S_BLOCK_READ_STORE   = 4'd4;
    localparam S_BLOCK_READ_DONE    = 4'd5;
    localparam S_BLOCK_WRITE_LOAD_LO  = 4'd6;
    localparam S_BLOCK_WRITE_WAIT_LO  = 4'd7;  // BRAM latency for lo byte
    localparam S_BLOCK_WRITE_LOAD_HI  = 4'd8;  // latch lo, present hi addr
    localparam S_BLOCK_WRITE_WAIT_HI  = 4'd9;  // BRAM latency for hi byte
    localparam S_BLOCK_WRITE_ISSUE  = 4'd10;
    localparam S_BLOCK_WRITE_WAIT   = 4'd11;
    localparam S_BLOCK_WRITE_DONE   = 4'd12;

    localparam [8:0] WRITE_LIMIT = 9'd255;

    reg [3:0]   state = S_IDLE;
    reg [8:0]   xfer_cnt = 9'd0;        // 0-255 word index (256 words = 512 bytes)
    reg [25:0]  base_addr = 26'd0;
    reg [7:0]   wr_lo_byte = 8'd0;      // latched low byte during block write
    reg [15:0]  rd_data_latch = 16'd0;   // latched SDRAM read data

    // -----------------------------------------------------------------
    // Boot passthrough — combinational mux to SDRAM controller.
    // Both forward (req/addr/data) and backward (ready) paths are
    // combinational during boot, eliminating the timing asymmetry
    // that caused dropped writes when refresh coincided with the
    // 1-cycle registered forward delay.
    // -----------------------------------------------------------------
    reg         rt_sdram_req = 1'b0;
    reg         rt_sdram_write = 1'b0;
    reg  [25:0] rt_sdram_addr = 26'd0;
    reg  [15:0] rt_sdram_wdata = 16'd0;

    assign sdram_req          = (!boot_done) ? boot_req   : rt_sdram_req;
    assign sdram_write        = (!boot_done) ? boot_write : rt_sdram_write;
    assign sdram_addr         = (!boot_done) ? boot_addr  : rt_sdram_addr;
    assign sdram_wdata        = (!boot_done) ? boot_wdata : rt_sdram_wdata;
    assign boot_ready  = (!boot_done) ? sdram_ready : 1'b0;

    // -----------------------------------------------------------------
    // Main logic
    // -----------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= S_IDLE;
            xfer_cnt        <= 9'd0;
            base_addr       <= 26'd0;
            wr_lo_byte      <= 8'd0;
            rd_data_latch   <= 16'd0;
            dev_block_ready <= 1'b0;
            rt_sdram_req    <= 1'b0;
            rt_sdram_write  <= 1'b0;
            rt_sdram_addr   <= 26'd0;
            rt_sdram_wdata  <= 16'd0;
            buf_addr_a      <= 9'd0;
            buf_wdata_a     <= 8'd0;
            buf_we_a        <= 1'b0;
        end else if (!boot_done) begin
            // ---- Boot phase: combinational mux handles SDRAM signals ----
            buf_addr_a      <= 9'd0;
            buf_wdata_a     <= 8'd0;
            buf_we_a        <= 1'b0;
            dev_block_ready <= 1'b0;
            rt_sdram_req    <= 1'b0;
            rt_sdram_write  <= 1'b0;
            state           <= S_IDLE;
        end else begin
            // ---- Runtime phase ----
            // Default: no buffer write, no SDRAM request
            buf_we_a      <= 1'b0;
            rt_sdram_req  <= 1'b0;

            case (state)
                // =====================================================
                // IDLE
                // =====================================================
                S_IDLE: begin
                    if (dev_read_rise) begin
                        dev_block_ready <= 1'b0;
                        base_addr <= {1'b0, dev_block_num, 9'd0};
                        xfer_cnt  <= 9'd0;
                        state     <= S_BLOCK_READ_SETUP;
                    end else if (dev_write_rise) begin
                        dev_block_ready <= 1'b0;
                        base_addr <= {1'b0, dev_block_num, 9'd0};
                        xfer_cnt  <= 9'd0;
                        state     <= S_BLOCK_WRITE_LOAD_LO;
                    end
                end

                // =====================================================
                // BLOCK READ: SDRAM → block buffer
                // =====================================================
                S_BLOCK_READ_SETUP: begin
                    // Present address to SDRAM — byte address = base + xfer_cnt*2
                    rt_sdram_addr  <= base_addr + {17'd0, xfer_cnt[7:0], 1'b0};
                    rt_sdram_write <= 1'b0;
                    rt_sdram_req   <= 1'b1;
                    state          <= S_BLOCK_READ_ISSUE;
                end

                S_BLOCK_READ_ISSUE: begin
                    // Hold rt_sdram_req HIGH until controller accepts.
                    rt_sdram_req <= 1'b1;
                    if (sdram_ready) begin
                        rt_sdram_req <= 1'b0;
                        state        <= S_BLOCK_READ_WAIT;
                    end
                end

                S_BLOCK_READ_WAIT: begin
                    // Wait for read data valid, latch it immediately
                    if (sdram_rdata_valid) begin
                        rd_data_latch <= sdram_rdata;
                        // Write low byte to buffer at (xfer_cnt * 2)
                        buf_addr_a  <= {xfer_cnt[7:0], 1'b0};
                        buf_wdata_a <= sdram_rdata[7:0];
                        buf_we_a    <= 1'b1;
                        state       <= S_BLOCK_READ_STORE;
                    end
                end

                S_BLOCK_READ_STORE: begin
                    // Write high byte to buffer at (xfer_cnt * 2 + 1)
                    // Use latched data, not live sdram_rdata
                    buf_addr_a  <= {xfer_cnt[7:0], 1'b1};
                    buf_wdata_a <= rd_data_latch[15:8];
                    buf_we_a    <= 1'b1;

                    if (xfer_cnt == 9'd255) begin
                        state <= S_BLOCK_READ_DONE;
                    end else begin
                        xfer_cnt <= xfer_cnt + 9'd1;
                        state    <= S_BLOCK_READ_SETUP;
                    end
                end

                S_BLOCK_READ_DONE: begin
                    buf_we_a        <= 1'b0;
                    dev_block_ready <= 1'b1;
                    state           <= S_IDLE;
                end

                // =====================================================
                // BLOCK WRITE: block buffer → SDRAM
                //
                // BRAM has 1-cycle registered read latency.  After
                // presenting addr on cycle N, rdata is valid for
                // sampling on cycle N+2 (NBA semantics: BRAM
                // registers at N+1 posedge, arbiter reads old value
                // at N+1, new value available at N+2).
                //
                //   LOAD_LO:  present lo addr
                //   WAIT_LO:  BRAM latency
                //   LOAD_HI:  latch lo byte, present hi addr
                //   WAIT_HI:  BRAM latency
                //   ISSUE:    read hi byte, combine, issue SDRAM wr
                //   WAIT:     wait for sdram_ready
                // =====================================================
                S_BLOCK_WRITE_LOAD_LO: begin
                    // Present low-byte address to buffer
                    buf_addr_a <= {xfer_cnt[7:0], 1'b0};
                    state      <= S_BLOCK_WRITE_WAIT_LO;
                end

                S_BLOCK_WRITE_WAIT_LO: begin
                    // BRAM latency cycle — rdata_a not yet valid
                    state <= S_BLOCK_WRITE_LOAD_HI;
                end

                S_BLOCK_WRITE_LOAD_HI: begin
                    // rdata_a now has lo byte — latch it
                    wr_lo_byte <= buf_rdata_a;
                    // Present high-byte address
                    buf_addr_a <= {xfer_cnt[7:0], 1'b1};
                    state      <= S_BLOCK_WRITE_WAIT_HI;
                end

                S_BLOCK_WRITE_WAIT_HI: begin
                    // BRAM latency cycle for hi byte
                    state <= S_BLOCK_WRITE_ISSUE;
                end

                S_BLOCK_WRITE_ISSUE: begin
                    rt_sdram_addr  <= base_addr + {17'd0, xfer_cnt[7:0], 1'b0};
                    rt_sdram_wdata <= {buf_rdata_a, wr_lo_byte};
                    rt_sdram_write <= 1'b1;
                    rt_sdram_req   <= 1'b1;
                    state          <= S_BLOCK_WRITE_WAIT;
                end

                S_BLOCK_WRITE_WAIT: begin
                    // Hold req until controller accepts
                    rt_sdram_req   <= 1'b1;
                    rt_sdram_write <= 1'b1;
                    if (sdram_ready) begin
                        rt_sdram_req   <= 1'b0;
                        rt_sdram_write <= 1'b0;

                        if (xfer_cnt == WRITE_LIMIT) begin
                            state <= S_BLOCK_WRITE_DONE;
                        end else begin
                            xfer_cnt <= xfer_cnt + 9'd1;
                            state <= S_BLOCK_WRITE_LOAD_LO;
                        end
                    end
                end

                S_BLOCK_WRITE_DONE: begin
                    dev_block_ready <= 1'b1;
                    state           <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
