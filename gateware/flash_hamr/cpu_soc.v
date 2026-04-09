`timescale 1ns / 1ps
// =============================================================================
// cpu_soc.v — PicoRV32 SoC wrapper for Flash Hamr
// =============================================================================
// Phase 2: IMEM + DMEM + UART TX + GPIO + SPI Master
// Later phases add: Mailbox, SDRAM port, Block buffer port
//
// Memory Map (active peripherals marked with *):
//   0x00000000 - 0x00007FFF  *IMEM (32 KB, $readmemh at synthesis)
//   0x10000000 - 0x10003FFF  *DMEM (16 KB, stack/heap)
//   0x20000000              *SPI Master (SD card)
//   0x30000000               Mailbox (Phase 5)
//   0x40000000               SDRAM port (Phase 6)
//   0x50000000              *UART TX
//   0x60000000               Block buffer port (Phase 5)
//   0x70000000              *GPIO output
// =============================================================================

module cpu_soc (
    input  wire        clk,        // 25 MHz
    input  wire        rst_n,

    // GPIO
    output reg  [7:0]  gpio_out,

    // UART TX pin
    output wire        uart_txd,

    // SPI Master pins (SD card)
    output wire        spi_sck,
    output wire        spi_mosi,
    input  wire        spi_miso,
    output wire        spi_cs,

    // Mailbox interface (directly to/from mailbox module in top)
    input  wire [7:0]  mbox_cmd,
    input  wire [7:0]  mbox_arg0,
    input  wire        mbox_cmd_pending,
    output reg  [7:0]  mbox_status_flags,
    output reg  [15:0] mbox_total_blocks,
    output reg         mbox_status_wr,
    output reg         mbox_cmd_ack,
    input  wire [15:0] mbox_persist_block,
    input  wire        mbox_persist_pending,
    output reg         mbox_persist_done,

    // Block buffer Port A (directly to top-level mux)
    output reg         buf_claim,
    output reg         cache_enabled,
    output wire        cache_release,  // stretched pulse: release current request to arbiter
    output reg  [8:0]  buf_addr,
    output reg  [7:0]  buf_wdata,
    output reg         buf_we,
    input  wire [7:0]  buf_rdata,    // read data from block buffer (1-cycle latency)

    // Magic block request (CDC'd from 7MHz)
    input  wire        magic_block_req,
    input  wire [15:0] magic_block_num,
    output wire        magic_block_ready,

    // Debug inputs (CDC'd from 7MHz in top-level)
    input  wire [31:0] dbg_bus_state,

    // SDRAM port (directly to top-level mux)
    output reg         sdram_claim,
    output reg         sdram_req,
    output reg         sdram_write,
    output reg  [25:0] sdram_addr,
    output reg  [15:0] sdram_wdata,
    input  wire        sdram_ready,
    input  wire [15:0] sdram_rdata,
    input  wire        sdram_rdata_valid
);

    // =========================================================================
    // PicoRV32 native memory interface
    // =========================================================================
    wire        mem_valid;
    wire        mem_instr;
    wire        mem_ready;
    wire [31:0] mem_addr;
    wire [31:0] mem_wdata;
    wire [ 3:0] mem_wstrb;
    wire [31:0] mem_rdata;

    // =========================================================================
    // Address decode — top nibble selects peripheral
    // =========================================================================
    wire [3:0] addr_sel = mem_addr[31:28];

    wire sel_imem = (addr_sel == 4'h0);
    wire sel_dmem = (addr_sel == 4'h1);
    wire sel_spi  = (addr_sel == 4'h2);  // Phase 2
    wire sel_mbox = (addr_sel == 4'h3);  // Phase 5
    wire sel_sdram = (addr_sel == 4'h4); // Phase 6
    wire sel_uart = (addr_sel == 4'h5);
    wire sel_bbuf = (addr_sel == 4'h6);  // Phase 5
    wire sel_gpio = (addr_sel == 4'h7);

    // =========================================================================
    // IMEM — 32 KB instruction memory (read-only from CPU perspective)
    // =========================================================================
    // 8192 x 32-bit words, initialized via $readmemh
    reg [31:0] imem [0:8191];
    initial $readmemh("firmware.mem", imem);

    wire [12:0] imem_word_addr = mem_addr[14:2];  // word-aligned
    reg  [31:0] imem_rdata;
    reg         imem_ready;

    always @(posedge clk) begin
        imem_ready <= 1'b0;
        if (mem_valid && sel_imem && !imem_ready) begin
            imem_rdata <= imem[imem_word_addr];
            imem_ready <= 1'b1;
        end
    end

    // =========================================================================
    // DMEM — 16 KB data memory (read/write)
    // =========================================================================
    // 4096 x 32-bit words
    reg [31:0] dmem [0:4095];

    wire [11:0] dmem_word_addr = mem_addr[13:2];
    reg  [31:0] dmem_rdata;
    reg         dmem_ready;

    always @(posedge clk) begin
        dmem_ready <= 1'b0;
        if (mem_valid && sel_dmem && !dmem_ready) begin
            dmem_rdata <= dmem[dmem_word_addr];
            // Byte-granular writes
            if (mem_wstrb[0]) dmem[dmem_word_addr][ 7: 0] <= mem_wdata[ 7: 0];
            if (mem_wstrb[1]) dmem[dmem_word_addr][15: 8] <= mem_wdata[15: 8];
            if (mem_wstrb[2]) dmem[dmem_word_addr][23:16] <= mem_wdata[23:16];
            if (mem_wstrb[3]) dmem[dmem_word_addr][31:24] <= mem_wdata[31:24];
            dmem_ready <= 1'b1;
        end
    end

    // =========================================================================
    // UART TX
    // =========================================================================
    wire uart_busy;
    wire uart_wr = mem_valid && sel_uart && (mem_addr[3:0] == 4'h0)
                   && (mem_wstrb != 4'h0) && !uart_ready_r;

    reg uart_ready_r;
    always @(posedge clk) begin
        uart_ready_r <= 1'b0;
        if (mem_valid && sel_uart && !uart_ready_r)
            uart_ready_r <= 1'b1;
    end

    wire [31:0] uart_rdata = {31'd0, uart_busy};  // STATUS at any read

    uart_tx u_uart_tx (
        .clk(clk), .rst_n(rst_n),
        .wr_en(uart_wr), .wr_data(mem_wdata[7:0]),
        .busy(uart_busy), .tx(uart_txd)
    );

    // =========================================================================
    // SPI Master (SD card)
    // =========================================================================
    wire [31:0] spi_rdata;
    wire        spi_ready;

    spi_master u_spi_master (
        .clk(clk), .rst_n(rst_n),
        .active(mem_valid && sel_spi),
        .addr(mem_addr[3:0]),
        .wdata(mem_wdata), .wstrb(mem_wstrb),
        .rdata(spi_rdata), .ready(spi_ready),
        .spi_sck(spi_sck), .spi_mosi(spi_mosi),
        .spi_miso(spi_miso), .spi_cs(spi_cs)
    );

    // =========================================================================
    // Mailbox peripheral (0x30000000)
    // CPU reads: CMD(+0), ARG0(+4), STATUS(+8), PERSIST_BLK(+10), PERSIST_PEND(+14)
    // CPU writes: STATUS_FLAGS(+8), TOTAL_BLOCKS(+C), CMD_ACK(+18), PERSIST_DONE(+1C)
    // =========================================================================
    reg [31:0] mbox_rdata;
    reg        mbox_ready;

    // Latch persist_pending (1-cycle pulse from mailbox) until CPU acks
    reg persist_pending_latch;
    reg persist_pend_prev;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mbox_ready           <= 1'b0;
            mbox_status_flags    <= 8'd0;
            mbox_total_blocks    <= 16'd0;
            mbox_status_wr       <= 1'b0;
            mbox_cmd_ack         <= 1'b0;
            mbox_persist_done    <= 1'b0;
            persist_pending_latch <= 1'b0;
            persist_pend_prev    <= 1'b0;
        end else begin
            mbox_ready        <= 1'b0;
            mbox_status_wr    <= 1'b0;
            mbox_cmd_ack      <= 1'b0;
            mbox_persist_done <= 1'b0;

            // Latch on rising edge of mbox_persist_pending
            persist_pend_prev <= mbox_persist_pending;
            if (mbox_persist_pending && !persist_pend_prev)
                persist_pending_latch <= 1'b1;

            if (mem_valid && sel_mbox && !mbox_ready) begin
                mbox_ready <= 1'b1;

                case (mem_addr[4:2])
                    3'd0: mbox_rdata <= {24'd0, mbox_cmd};           // CMD
                    3'd1: mbox_rdata <= {24'd0, mbox_arg0};          // ARG0
                    3'd2: begin                                       // STATUS_FLAGS
                        mbox_rdata <= {24'd0, mbox_status_flags};
                        if (mem_wstrb != 4'h0) begin
                            mbox_status_flags <= mem_wdata[7:0];
                            mbox_status_wr    <= 1'b1;
                        end
                    end
                    3'd3: begin                                       // TOTAL_BLOCKS
                        mbox_rdata <= {16'd0, mbox_total_blocks};
                        if (mem_wstrb != 4'h0) begin
                            mbox_total_blocks <= mem_wdata[15:0];
                            mbox_status_wr    <= 1'b1;
                        end
                    end
                    3'd4: mbox_rdata <= {16'd0, mbox_persist_block}; // PERSIST_BLK
                    3'd5: mbox_rdata <= {31'd0, persist_pending_latch}; // PERSIST_PEND (latched)
                    3'd6: begin                                       // CMD_ACK (write-only)
                        mbox_rdata <= 32'd0;
                        if (mem_wstrb != 4'h0)
                            mbox_cmd_ack <= 1'b1;
                    end
                    3'd7: begin                                       // PERSIST_DONE (write-only)
                        mbox_rdata <= 32'd0;
                        if (mem_wstrb != 4'h0) begin
                            mbox_persist_done     <= 1'b1;
                            persist_pending_latch <= 1'b0;  // clear latch
                        end
                    end
                endcase
            end
        end
    end

    // =========================================================================
    // Block Buffer Port (0x60000000)
    // CPU writes: ADDR(+0), WDATA(+4), CTRL(+C bit0=claim)
    // CPU reads:  RDATA(+8)
    // =========================================================================
    reg [31:0] bbuf_rdata;
    reg        bbuf_ready;

    // Magic block request: toggle CDC from 7MHz.
    // Stretch must span enough 7MHz cycles for bus_interface's 3-stage
    // edge detector (br_d1→br_d2→br_prev). 127 cycles @ 25MHz = 5.08μs
    // = ~35 cycles @ 7MHz — plenty of margin.
    reg [6:0] mbr_stretch;
    assign magic_block_ready = (mbr_stretch != 7'd0);

    // Cache release: firmware signals "data is in SDRAM, let arbiter serve it"
    // Stretched like mbr_stretch — 15 cycles = 600ns, spans ~4 7MHz cycles.
    reg [3:0] crel_stretch;
    assign cache_release = (crel_stretch != 4'd0);

    reg bbuf_post_inc;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bbuf_ready        <= 1'b0;
            buf_claim         <= 1'b0;
            cache_enabled     <= 1'b0;
            crel_stretch      <= 4'd0;
            buf_addr          <= 9'd0;
            buf_wdata         <= 8'd0;
            buf_we            <= 1'b0;
            mbr_stretch       <= 7'd0;
            bbuf_post_inc     <= 1'b0;
        end else begin
            bbuf_ready        <= 1'b0;
            buf_we            <= 1'b0;

            // Stretch counter for cache_release CDC (25MHz → 7MHz)
            if (crel_stretch != 4'd0)
                crel_stretch <= crel_stretch - 4'd1;

            // Post-increment: advance buf_addr AFTER the BRAM write cycle
            if (bbuf_post_inc) begin
                buf_addr      <= buf_addr + 9'd1;
                bbuf_post_inc <= 1'b0;
            end

            // Stretch counter countdown for magic_block_ready CDC
            if (mbr_stretch != 7'd0)
                mbr_stretch <= mbr_stretch - 7'd1;

            if (mem_valid && sel_bbuf && !bbuf_ready) begin
                bbuf_ready <= 1'b1;

                case (mem_addr[4:2])
                    3'd0: begin  // ADDR (+0x00)
                        bbuf_rdata <= {23'd0, buf_addr};
                        if (mem_wstrb != 4'h0)
                            buf_addr <= mem_wdata[8:0];
                    end
                    3'd1: begin  // WDATA (+0x04, write byte then auto-increment)
                        bbuf_rdata <= 32'd0;
                        if (mem_wstrb != 4'h0) begin
                            buf_wdata <= mem_wdata[7:0];
                            buf_we    <= 1'b1;
                            // Don't increment yet — BRAM needs current addr for this write.
                            // Increment happens on the NEXT cycle via bbuf_post_inc.
                            bbuf_post_inc <= 1'b1;
                        end
                    end
                    3'd2: begin  // RDATA (+0x08) — read with 1-cycle BRAM latency
                        bbuf_rdata    <= {24'd0, buf_rdata};
                        bbuf_post_inc <= 1'b1;  // increment after read
                    end
                    3'd3: begin  // CTRL (+0x0C) bit0=claim, bit1=cache_enabled
                        bbuf_rdata <= {30'd0, cache_enabled, buf_claim};
                        if (mem_wstrb != 4'h0) begin
                            buf_claim     <= mem_wdata[0];
                            cache_enabled <= mem_wdata[1];
                        end
                    end
                    3'd4: begin  // MAGIC_STATUS (+0x10) R: {toggle, block_num}
                        // Bit 17 = raw toggle from 7MHz CDC. Firmware compares
                        // with its local copy to detect new requests. No
                        // hardware edge detector — avoids SET/CLEAR races.
                        bbuf_rdata <= {14'd0, magic_block_req, 1'b0, magic_block_num};
                    end
                    3'd5: begin  // MAGIC_DONE (+0x14) W: signal block ready (CPU fills buffer)
                        bbuf_rdata <= 32'd0;
                        if (mem_wstrb != 4'h0)
                            mbr_stretch <= 7'd127;  // 5.08μs — ~35 cycles at 7MHz
                    end
                    3'd6: begin  // CACHE_RELEASE (+0x18) W: release to arbiter (SDRAM→buffer)
                        bbuf_rdata <= 32'd0;
                        if (mem_wstrb != 4'h0)
                            crel_stretch <= 4'd15;  // 600ns stretched pulse
                    end
                    default: bbuf_rdata <= 32'd0;
                endcase
            end
        end
    end

    // =========================================================================
    // SDRAM Port (0x40000000)
    // +0x00  ADDR     W: set 26-bit SDRAM byte address
    // +0x04  WDATA    W: write 16-bit word (triggers SDRAM write, stalls CPU until accepted)
    // +0x08  RDATA    R: triggers SDRAM read, stalls CPU until data valid
    // +0x0C  CTRL     R/W: bit0=claim, R: bit1=sdram_ready
    //
    // State machine handles the SDRAM req/ready/valid handshake transparently.
    // The CPU bus stalls (mem_ready stays low) until the SDRAM op completes.
    // =========================================================================
    reg [31:0] sdram_port_rdata;
    reg        sdram_port_ready;

    localparam SP_IDLE     = 2'd0,
               SP_WRITE    = 2'd1,
               SP_READ_REQ = 2'd2,
               SP_READ_WAIT = 2'd3;
    reg [1:0] sp_state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sdram_port_ready <= 1'b0;
            sdram_claim      <= 1'b0;
            sdram_req        <= 1'b0;
            sdram_write      <= 1'b0;
            sdram_addr       <= 26'd0;
            sdram_wdata      <= 16'd0;
            sp_state         <= SP_IDLE;
        end else begin
            sdram_port_ready <= 1'b0;

            case (sp_state)
                SP_IDLE: begin
                    sdram_req <= 1'b0;
                    if (mem_valid && sel_sdram && !sdram_port_ready) begin
                        case (mem_addr[3:2])
                            2'd0: begin  // ADDR
                                sdram_port_rdata <= {6'd0, sdram_addr};
                                if (mem_wstrb != 4'h0)
                                    sdram_addr <= mem_wdata[25:0];
                                sdram_port_ready <= 1'b1;
                            end
                            2'd1: begin  // WDATA — start write
                                if (mem_wstrb != 4'h0) begin
                                    sdram_wdata <= mem_wdata[15:0];
                                    sdram_write <= 1'b1;
                                    sdram_req   <= 1'b1;
                                    sp_state    <= SP_WRITE;
                                end else begin
                                    sdram_port_rdata <= {16'd0, sdram_wdata};
                                    sdram_port_ready <= 1'b1;
                                end
                            end
                            2'd2: begin  // RDATA — start read
                                sdram_write <= 1'b0;
                                sdram_req   <= 1'b1;
                                sp_state    <= SP_READ_REQ;
                            end
                            2'd3: begin  // CTRL
                                sdram_port_rdata <= {30'd0, sdram_ready, sdram_claim};
                                if (mem_wstrb != 4'h0)
                                    sdram_claim <= mem_wdata[0];
                                sdram_port_ready <= 1'b1;
                            end
                        endcase
                    end
                end

                SP_WRITE: begin
                    // Wait for SDRAM to accept write request
                    if (sdram_ready) begin
                        sdram_req        <= 1'b0;
                        sdram_port_rdata <= 32'd0;
                        sdram_port_ready <= 1'b1;
                        sp_state         <= SP_IDLE;
                    end
                end

                SP_READ_REQ: begin
                    // Wait for request to be accepted
                    if (sdram_ready) begin
                        sdram_req <= 1'b0;
                        sp_state  <= SP_READ_WAIT;
                    end
                end

                SP_READ_WAIT: begin
                    // Wait for read data valid
                    if (sdram_rdata_valid) begin
                        sdram_port_rdata <= {16'd0, sdram_rdata};
                        sdram_port_ready <= 1'b1;
                        sp_state         <= SP_IDLE;
                    end
                end
            endcase
        end
    end

    // =========================================================================
    // GPIO output register
    // =========================================================================
    reg gpio_ready;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gpio_out   <= 8'd0;
            gpio_ready <= 1'b0;
        end else begin
            gpio_ready <= 1'b0;
            if (mem_valid && sel_gpio && !gpio_ready) begin
                if (mem_wstrb != 4'h0)
                    gpio_out <= mem_wdata[7:0];
                gpio_ready <= 1'b1;
            end
        end
    end

    // GPIO reads: offset 0 = gpio_out, offset 4 = debug bus state
    wire [31:0] gpio_rdata = (mem_addr[2]) ? dbg_bus_state : {24'd0, gpio_out};

    // =========================================================================
    // Bus mux — read data and ready
    // =========================================================================
    assign mem_rdata = sel_imem  ? imem_rdata       :
                       sel_dmem  ? dmem_rdata       :
                       sel_spi   ? spi_rdata        :
                       sel_mbox  ? mbox_rdata       :
                       sel_sdram ? sdram_port_rdata  :
                       sel_uart  ? uart_rdata       :
                       sel_bbuf  ? bbuf_rdata       :
                       sel_gpio  ? gpio_rdata       :
                       32'hDEADBEEF;

    assign mem_ready = imem_ready | dmem_ready | spi_ready | mbox_ready |
                       sdram_port_ready | uart_ready_r | bbuf_ready | gpio_ready;

    // =========================================================================
    // PicoRV32 CPU
    // =========================================================================
    picorv32 #(
        .ENABLE_COUNTERS(0),
        .ENABLE_COUNTERS64(0),
        .ENABLE_REGS_16_31(1),
        .ENABLE_REGS_DUALPORT(1),
        .BARREL_SHIFTER(1),
        .TWO_CYCLE_COMPARE(0),
        .TWO_CYCLE_ALU(0),
        .COMPRESSED_ISA(0),
        .CATCH_MISALIGN(0),
        .CATCH_ILLINSN(0),
        .ENABLE_PCPI(0),
        .ENABLE_MUL(0),
        .ENABLE_FAST_MUL(0),
        .ENABLE_DIV(0),
        .ENABLE_IRQ(0),
        .ENABLE_IRQ_QREGS(0),
        .ENABLE_IRQ_TIMER(0),
        .ENABLE_TRACE(0),
        .STACKADDR(32'h10004000)  // Top of 16 KB DMEM
    ) u_cpu (
        .clk(clk),
        .resetn(rst_n),

        .mem_valid(mem_valid),
        .mem_instr(mem_instr),
        .mem_ready(mem_ready),
        .mem_addr(mem_addr),
        .mem_wdata(mem_wdata),
        .mem_wstrb(mem_wstrb),
        .mem_rdata(mem_rdata),

        // Unused interfaces
        .mem_la_read(),
        .mem_la_write(),
        .mem_la_addr(),
        .mem_la_wdata(),
        .mem_la_wstrb(),
        .pcpi_valid(),
        .pcpi_insn(),
        .pcpi_rs1(),
        .pcpi_rs2(),
        .pcpi_wr(1'b0),
        .pcpi_rd(32'd0),
        .pcpi_wait(1'b0),
        .pcpi_ready(1'b0),
        .irq(32'd0),
        .eoi(),
        .trace_valid(),
        .trace_data()
    );

endmodule
