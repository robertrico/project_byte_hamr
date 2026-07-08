// =============================================================================
// Byte Hamr Rev 2 - b8008_hamr
// =============================================================================
// Intel 8008 softcore (b8008, cycle-exact) as an Apple II slot coprocessor.
// The //e is the 8008's terminal: the b8008_monitor firmware's UART ports
// (IN 1 = RX, OUT 9 = TX) are backed by FIFOs on the $C0Cx register window
// instead of a physical UART. Load programs through the monitor's L command,
// run with G — same silicon-proven firmware as the Versa build.
//
// Core: b8008_core.v — GENERATED from ~/Development/intel-8008-vhdl (see file
// header for commit + regen command). Personality: ROM 4KB @ $0000 (monitor),
// RAM 12KB @ $1000. ROM contents from b8008_rom.mem ($00-filled: $00 = HLT,
// wild jumps freeze diagnosably; also avoids the documented GHDL/yosys
// $FF-fill BRAM corruption).
//
// Apple II side (slot 4, register window only — no slot/expansion ROM):
//   $C0C0  R  TX_STAT   bit7 = byte available, bits[6:0] = TX FIFO count
//   $C0C1  R  TX_DATA   8008 -> host FIFO; pops on read (absolute LDA only)
//   $C0C2  R  RX_STAT   bit7 = space free,  bits[6:0] = RX FIFO free count
//          W  RX_PUSH   host -> 8008 "keyboard" byte (7-bit ASCII + CR/DEL)
//   $C0C3  R  CHKPT     last byte the 8008 wrote to OUT 31 (checkpoint port)
//   $C0C4  R  CHKPT_CNT count of OUT 31 writes since reset/clear
//   $C0C5  R  CPU_STAT  {bootstrap_done, running, s2, s1, s0, halted,
//                        tx_avail, rx_space}
//          W  CONTROL   $52 ('R') = hold 8008 in reset, flush FIFOs + chkpt
//                       $47 ('G') = release: auto-start + RST0 jam re-run
//   $C0C6  R  PC_LO     debug_pc[7:0]  (live — diagnose hangs from the //e)
//   $C0C7  R  PC_HI     debug_pc[13:8]
//   $C0C8  R  LED       last byte the 8008 wrote to OUT 8 (monitor LED port)
//   $C0C9  W  INT_REQ   bits[2:0] = RST vector; latched until T1I ack.
//                       Armed only after bootstrap (can't race the RST0 jam).
//   $C0CF  R  ID        constant $B8 — card-present check
//
// 8008 boot ceremony (ported from b8008_monitor_top.vhdl, incl. the latched
// jam vector — a combinational mux raced the bootstrap's own T1I):
//   reset release -> 2 ms auto-start -> run -> bootstrap INT jams RST 0 at
//   T1I (status 110 + sync) -> monitor prompt lands in the TX FIFO.
//
// SDRAM + SPI flash pins are parked inactive (unused in this design).
// =============================================================================

module b8008_hamr_top #(
    // 2 ms at 25 MHz on silicon; testbench overrides to shorten boot
    parameter AUTO_START_CYCLES = 17'd50000,
    // Synthesis default = b8008_monitor firmware; tb overrides with test ROM
    parameter ROM_INIT_FILE = "b8008_rom.mem"
) (
    input  wire        CLK_100MHz,

    // ----- SDRAM (unused — parked) -----
    output wire        SDRAM_CLK,
    output wire        SDRAM_CKE,
    output wire        SDRAM_nCS,
    output wire        SDRAM_nRAS,
    output wire        SDRAM_nCAS,
    output wire        SDRAM_nWE,
    output wire        SDRAM_DQM0,
    output wire        SDRAM_DQM1,
    output wire        SDRAM_BA0,
    output wire        SDRAM_BA1,
    output wire        SDRAM_A0, SDRAM_A1, SDRAM_A2, SDRAM_A3,
    output wire        SDRAM_A4, SDRAM_A5, SDRAM_A6, SDRAM_A7,
    output wire        SDRAM_A8, SDRAM_A9, SDRAM_A10, SDRAM_A11, SDRAM_A12,
    inout  wire        SDRAM_D0, SDRAM_D1, SDRAM_D2, SDRAM_D3,
    inout  wire        SDRAM_D4, SDRAM_D5, SDRAM_D6, SDRAM_D7,
    inout  wire        SDRAM_D8, SDRAM_D9, SDRAM_D10, SDRAM_D11,
    inout  wire        SDRAM_D12, SDRAM_D13, SDRAM_D14, SDRAM_D15,

    // ----- SPI Flash (unused — parked) -----
    output wire        FLASH_nCS,
    output wire        FLASH_MOSI,
    input  wire        FLASH_MISO,
    output wire        FLASH_nWP,
    output wire        FLASH_nHOLD,

    // ----- Apple II Bus -----
    input  wire        A0, A1, A2, A3, A4, A5, A6, A7,
    input  wire        A8, A9, A10, A11, A12, A13, A14, A15,
    inout  wire        D0, D1, D2, D3, D4, D5, D6, D7,

    input  wire        PHI0,
    input  wire        PHI1,
    input  wire        sig_7M,
    input  wire        Q3,
    input  wire        uSync,

    input  wire        R_nW,
    input  wire        nDEVICE_SELECT,
    input  wire        nI_O_SELECT,
    input  wire        nI_O_STROBE,
    input  wire        DMA_OUT,
    input  wire        INT_OUT,
    input  wire        RDY,
    input  wire        nRES_READ,

    output wire        nIRQ,
    output wire        nNMI,
    output wire        nINH,
    output wire        nDMA,
    output wire        nRES,
    output wire        DMA_IN,
    output wire        INT_IN,

    output wire        DATA_OE,

    output wire        GPIO_1,  GPIO_2,  GPIO_3,  GPIO_4,  GPIO_5,
    output wire        GPIO_6,  GPIO_7,  GPIO_8,  GPIO_9,  GPIO_10,
    output wire        GPIO_11, GPIO_12, GPIO_13, GPIO_14, GPIO_15,
    output wire        GPIO_16, GPIO_17, GPIO_18, GPIO_19, GPIO_20
);

    // =========================================================================
    // Bus bundles
    // =========================================================================
    wire [15:0] apple_addr = {A15, A14, A13, A12, A11, A10, A9, A8,
                              A7, A6, A5, A4, A3, A2, A1, A0};
    wire [7:0]  apple_data_in = {D7, D6, D5, D4, D3, D2, D1, D0};

    // =========================================================================
    // Clock: 100 MHz -> 25 MHz via /4 counter (same pattern as project_obscurus)
    // =========================================================================
    (* keep = "true" *) reg [1:0] clk_div = 2'd0;
    always @(posedge CLK_100MHz) clk_div <= clk_div + 1'b1;
    (* keep = "true" *) wire clk = clk_div[1];

    // POR + nRES_READ sync: hold reset ~16 clk cycles after config, then OR
    // with Apple II reset so Ctrl-Reset also reboots the 8008.
    reg [3:0] por_cnt = 4'd0;
    reg       por_n   = 1'b0;
    always @(posedge clk) begin
        if (por_cnt != 4'hF) por_cnt <= por_cnt + 1'b1;
        por_n <= (por_cnt == 4'hF);
    end
    reg nres_s1, nres_s2;
    always @(posedge clk) begin
        nres_s1 <= nRES_READ;
        nres_s2 <= nres_s1;
    end
    wire rst_n = por_n & nres_s2;

    // =========================================================================
    // Apple II bus interface (obscurus idiom: 2-FF sync, continuous capture,
    // commit on rising edge of nDEVICE_SELECT)
    // =========================================================================
    reg nds_d1, nds_d2;
    reg rw_d1,  rw_d2;
    reg [3:0] addr_d1, addr_d2;
    reg [7:0] data_d1, data_d2;

    always @(posedge clk) begin
        nds_d1  <= nDEVICE_SELECT; nds_d2  <= nds_d1;
        rw_d1   <= R_nW;           rw_d2   <= rw_d1;
        addr_d1 <= apple_addr[3:0]; addr_d2 <= addr_d1;
        data_d1 <= apple_data_in;   data_d2 <= data_d1;
    end

    wire nds_rise = ~nds_d2 & nds_d1;

    reg [7:0] wr_data_latch;
    reg [3:0] wr_addr_latch;
    reg       wr_rw_latch;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_data_latch <= 8'd0;
            wr_addr_latch <= 4'd0;
            wr_rw_latch   <= 1'b1;
        end else if (!nds_d2) begin
            wr_data_latch <= data_d2;
            wr_addr_latch <= addr_d2;
            wr_rw_latch   <= rw_d2;
        end
    end

    wire reg_wr = nds_rise & ~wr_rw_latch;
    wire reg_rd = nds_rise &  wr_rw_latch;

    // =========================================================================
    // 8008 subsystem reset + auto-start
    // CONTROL 'R' holds the CPU in reset (and flushes FIFOs/checkpoint);
    // 'G' releases it. POR / Ctrl-Reset do a full release cycle too.
    // 2 ms after release, run_enable goes high; the bootstrap FSM then jams
    // RST 0. (Monitor top used debug_clock_control + auto_start_pulse; here
    // run is a plain level — no front-panel step/stop on this board.)
    // =========================================================================
    reg ctl_hold = 1'b0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) ctl_hold <= 1'b0;
        else if (reg_wr & (wr_addr_latch == 4'h5)) begin
            if      (wr_data_latch == 8'h52) ctl_hold <= 1'b1;   // 'R'
            else if (wr_data_latch == 8'h47) ctl_hold <= 1'b0;   // 'G'
        end
    end

    wire cpu_rst = ~rst_n | ctl_hold;

    reg [16:0] auto_start_cnt = 17'd0;
    reg        run_flag = 1'b0;
    always @(posedge clk) begin
        if (cpu_rst) begin
            auto_start_cnt <= 17'd0;
            run_flag       <= 1'b0;
        end else if (!run_flag) begin
            auto_start_cnt <= auto_start_cnt + 1'b1;
            if (auto_start_cnt == AUTO_START_CYCLES)
                run_flag <= 1'b1;
        end
    end

    // =========================================================================
    // b8008 core
    // =========================================================================
    wire        phi2;
    wire        cpu_sync;
    wire        s0, s1, s2;
    wire [13:0] rom_a;
    wire [7:0]  rom_q;
    wire [13:0] debug_pc;
    wire [7:0]  io_port_out;
    wire [4:0]  io_port_num_out;
    wire        io_port_write;
    wire        io_port_read;
    wire [7:0]  rx_out_reg;
    wire        bootstrap_int;
    reg         host_int_req = 1'b0;
    reg  [2:0]  host_int_vec = 3'b000;
    reg  [2:0]  cpu_int_vec  = 3'b000;

    b8008_top u_b8008 (
        .clk_in(clk),
        .reset(cpu_rst),
        .run_enable(run_flag),
        .interrupt(bootstrap_int | host_int_req),
        .int_vector(cpu_int_vec),
        .ready_in(1'b1),
        .phi1_out(),
        .phi2_out(phi2),
        .sync_out(cpu_sync),
        .s0_out(s0), .s1_out(s1), .s2_out(s2),
        .address_out(),
        .data_out(),
        .ram_byte_0(),
        .debug_reg_a(), .debug_reg_b(), .debug_reg_c(), .debug_reg_d(),
        .debug_reg_e(), .debug_reg_h(), .debug_reg_l(),
        .debug_cycle(),
        .debug_pc(debug_pc),
        .debug_ir(),
        .debug_needs_address(),
        .debug_int_pending(),
        .debug_flag_carry(), .debug_flag_zero(),
        .debug_flag_sign(),  .debug_flag_parity(),
        .debug_io_port_8(), .debug_io_port_9(), .debug_io_port_10(),
        .debug_state_half(),
        .io_port_in(rx_out_reg),
        .io_port_in_select(3'b001),   // port 1 = terminal RX (matches monitor)
        .io_port_in_enable(1'b1),
        .io_port_out(io_port_out),
        .io_port_num_out(io_port_num_out),
        .io_port_write(io_port_write),
        .io_port_read(io_port_read),
        .rom_a(rom_a),
        .rom_d(rom_q),
        .rom_ce_n(),
        .rom_oe_n()
    );

    // =========================================================================
    // Monitor ROM: 4KB sync-read BRAM @ 8008 $0000-$0FFF. $00 fill (= HLT).
    // =========================================================================
    reg [7:0] rom_mem [0:4095];
    initial $readmemh(ROM_INIT_FILE, rom_mem);
    reg [7:0] rom_q_r;
    always @(posedge clk) rom_q_r <= rom_mem[rom_a[11:0]];
    assign rom_q = rom_q_r;

    // =========================================================================
    // Bootstrap interrupt FSM — verbatim port of b8008_monitor_top.vhdl.
    // clk domain, advances on phi2 rising-edge enable. Jams RST 0 at T1I
    // (status 110 + sync), >=16 phi2 cycles grace to exit STOPPED.
    // =========================================================================
    reg bs_phi2_prev  = 1'b0;
    reg bs_int        = 1'b0;
    reg bootstrap_done = 1'b0;
    reg [7:0] bs_counter = 8'd0;

    always @(posedge clk) begin
        bs_phi2_prev <= phi2;
        if (cpu_rst) begin
            bs_int         <= 1'b0;
            bootstrap_done <= 1'b0;
            bs_counter     <= 8'd0;
        end else if (phi2 & ~bs_phi2_prev) begin
            if (!bootstrap_done) begin
                bs_int     <= 1'b1;
                bs_counter <= bs_counter + 1'b1;
                if (bs_counter >= 8'd16) begin
                    if (s2 & s1 & ~s0 & cpu_sync) begin
                        bs_int         <= 1'b0;
                        bootstrap_done <= 1'b1;
                    end
                end
            end
        end
    end
    assign bootstrap_int = bs_int;

    // T1I acknowledge (same decode as int_button.vhdl — no sync term)
    wire t1i_ack = s2 & s1 & ~s0;

    // Host interrupt request: write $C0C9 with vector in [2:0]. Armed only
    // after bootstrap so it can never race the RST 0 jam. Held until T1I.
    always @(posedge clk) begin
        if (cpu_rst | ~bootstrap_done) begin
            host_int_req <= 1'b0;
            host_int_vec <= 3'b000;
        end else if (reg_wr & (wr_addr_latch == 4'h9)) begin
            host_int_req <= 1'b1;
            host_int_vec <= wr_data_latch[2:0];
        end else if (host_int_req & t1i_ack) begin
            host_int_req <= 1'b0;
        end
    end

    // Jam vector latched at request time (vec_latch race fix from monitor top)
    always @(posedge clk) begin
        if (cpu_rst | ~bootstrap_done) cpu_int_vec <= 3'b000;   // bootstrap = RST 0
        else if (host_int_req)         cpu_int_vec <= host_int_vec;
    end

    // =========================================================================
    // TX FIFO: 8008 OUT 9 -> host. 64 deep, combinational head, drop on full.
    // =========================================================================
    reg [7:0] tx_fifo [0:63];
    reg [5:0] tx_wp = 6'd0, tx_rp = 6'd0;
    reg [6:0] tx_count = 7'd0;

    wire tx_pop  = reg_rd & (wr_addr_latch == 4'h1) & (tx_count != 7'd0);
    wire [7:0] tx_head = tx_fifo[tx_rp];
    wire tx_avail = (tx_count != 7'd0);

    // io_port_write asserts for one phi2 period (~1.4 us = many clk cycles),
    // so edge-detect it: one push/latch per OUT instruction.
    reg iow_d1 = 1'b0;
    always @(posedge clk) iow_d1 <= io_port_write;
    wire iow_rise = io_port_write & ~iow_d1;

    always @(posedge clk) begin
        if (cpu_rst) begin
            tx_wp <= 6'd0; tx_rp <= 6'd0; tx_count <= 7'd0;
        end else begin
            case ({iow_rise & (io_port_num_out == 5'd9) & (tx_count != 7'd64), tx_pop})
                2'b10: begin tx_fifo[tx_wp] <= io_port_out; tx_wp <= tx_wp + 1'b1; tx_count <= tx_count + 1'b1; end
                2'b01: begin tx_rp <= tx_rp + 1'b1; tx_count <= tx_count - 1'b1; end
                2'b11: begin tx_fifo[tx_wp] <= io_port_out; tx_wp <= tx_wp + 1'b1; tx_rp <= tx_rp + 1'b1; end
                default: ;
            endcase
        end
    end

    // =========================================================================
    // RX FIFO: host $C0C2 write -> 8008 IN 1. Snapshot-and-pop semantics
    // ported from b8008_usart.vhdl: on the INP read strobe's rising edge,
    // freeze {ready, data[6:0]} for the CPU and pop atomically. A byte
    // arriving the same cycle waits for the next poll.
    // =========================================================================
    reg [7:0] rx_fifo [0:63];
    reg [5:0] rx_wp = 6'd0, rx_rp = 6'd0;
    reg [6:0] rx_count = 7'd0;
    reg [7:0] rx_snap = 8'd0;

    wire rx_space = (rx_count != 7'd64);
    wire [6:0] rx_free = 7'd64 - rx_count;
    wire rx_host_push = reg_wr & (wr_addr_latch == 4'h2) & rx_space;

    reg ior_d1 = 1'b0;
    always @(posedge clk) ior_d1 <= io_port_read;
    wire ior_rise = io_port_read & ~ior_d1;
    wire rx_cpu_pop = ior_rise & (io_port_num_out[2:0] == 3'b001);

    always @(posedge clk) begin
        if (cpu_rst) begin
            rx_wp <= 6'd0; rx_rp <= 6'd0; rx_count <= 7'd0; rx_snap <= 8'd0;
        end else begin
            case ({rx_host_push, rx_cpu_pop & (rx_count != 7'd0)})
                2'b10: begin rx_fifo[rx_wp] <= wr_data_latch; rx_wp <= rx_wp + 1'b1; rx_count <= rx_count + 1'b1; end
                2'b01: begin rx_rp <= rx_rp + 1'b1; rx_count <= rx_count - 1'b1; end
                2'b11: begin rx_fifo[rx_wp] <= wr_data_latch; rx_wp <= rx_wp + 1'b1; rx_rp <= rx_rp + 1'b1; end
                default: ;
            endcase
            if (rx_cpu_pop)
                rx_snap <= {(rx_count != 7'd0), rx_fifo[rx_rp][6:0]};
        end
    end
    assign rx_out_reg = rx_snap;

    // =========================================================================
    // Checkpoint latch (OUT 31) + LED latch (OUT 8)
    // =========================================================================
    reg [7:0] chkpt_val = 8'd0;
    reg [7:0] chkpt_cnt = 8'd0;
    reg [7:0] led_val   = 8'd0;

    always @(posedge clk) begin
        if (cpu_rst) begin
            chkpt_val <= 8'd0; chkpt_cnt <= 8'd0; led_val <= 8'd0;
        end else if (iow_rise) begin
            if (io_port_num_out == 5'd31) begin
                chkpt_val <= io_port_out;
                chkpt_cnt <= chkpt_cnt + 1'b1;
            end
            if (io_port_num_out == 5'd8)
                led_val <= io_port_out;
        end
    end

    // =========================================================================
    // CPU status
    // =========================================================================
    wire halted = (({s2, s1, s0}) == 3'b011);   // STOPPED state code
    wire [7:0] cpu_stat = {bootstrap_done, run_flag & ~cpu_rst,
                           s2, s1, s0, halted, tx_avail, rx_space};

    // =========================================================================
    // Slot ROM ($C400-$C4FF): PR#4 terminal firmware (b8fw.S -> b8008_slot.mem)
    // Combinational LUT read, obscurus pattern. $00 fill.
    // =========================================================================
    reg [7:0] slot_rom_mem [0:255];
    initial $readmemh("b8008_slot.mem", slot_rom_mem);
    wire [7:0] slot_rom_data = slot_rom_mem[apple_addr[7:0]];

    // =========================================================================
    // Register read mux
    // =========================================================================
    reg [7:0] reg_data_out;
    always @(*) begin
        case (apple_addr[3:0])
            4'h0: reg_data_out = {tx_avail, tx_count};
            4'h1: reg_data_out = tx_head;
            4'h2: reg_data_out = {rx_space, rx_free};
            4'h3: reg_data_out = chkpt_val;
            4'h4: reg_data_out = chkpt_cnt;
            4'h5: reg_data_out = cpu_stat;
            4'h6: reg_data_out = debug_pc[7:0];
            4'h7: reg_data_out = {2'b00, debug_pc[13:8]};
            4'h8: reg_data_out = led_val;
            4'hF: reg_data_out = 8'hB8;         // ID
            default: reg_data_out = 8'h00;
        endcase
    end

    // -------- Drive D bus: registers ($C0Cx) + slot ROM ($C4xx) --------------
    wire device_read = ~nDEVICE_SELECT & R_nW;
    wire rom_read    = ~nI_O_SELECT    & R_nW;

    wire [7:0] d_out = rom_read    ? slot_rom_data :
                       device_read ? reg_data_out  : 8'h00;
    wire       d_oe  = rom_read | device_read;

    assign D0 = d_oe ? d_out[0] : 1'bZ;
    assign D1 = d_oe ? d_out[1] : 1'bZ;
    assign D2 = d_oe ? d_out[2] : 1'bZ;
    assign D3 = d_oe ? d_out[3] : 1'bZ;
    assign D4 = d_oe ? d_out[4] : 1'bZ;
    assign D5 = d_oe ? d_out[5] : 1'bZ;
    assign D6 = d_oe ? d_out[6] : 1'bZ;
    assign D7 = d_oe ? d_out[7] : 1'bZ;

    // U12 level shifter OE (active low), live on read AND write
    wire slot_active = ~nDEVICE_SELECT | ~nI_O_SELECT;
    assign DATA_OE = ~slot_active;

    // =========================================================================
    // Daisy chain + passive control lines
    // =========================================================================
    assign DMA_IN = DMA_OUT;
    assign INT_IN = INT_OUT;
    assign nIRQ = 1'bZ;
    assign nNMI = 1'bZ;
    assign nINH = 1'bZ;
    assign nDMA = 1'bZ;
    assign nRES = 1'b1;   // OPENDRAIN: released

    // =========================================================================
    // Unused peripherals parked
    // =========================================================================
    assign SDRAM_CLK = 1'b0;  assign SDRAM_CKE = 1'b0;
    assign SDRAM_nCS = 1'b1;  assign SDRAM_nRAS = 1'b1;
    assign SDRAM_nCAS = 1'b1; assign SDRAM_nWE = 1'b1;
    assign SDRAM_DQM0 = 1'b1; assign SDRAM_DQM1 = 1'b1;
    assign SDRAM_BA0 = 1'b0;  assign SDRAM_BA1 = 1'b0;
    assign SDRAM_A0 = 1'b0;  assign SDRAM_A1 = 1'b0;  assign SDRAM_A2 = 1'b0;
    assign SDRAM_A3 = 1'b0;  assign SDRAM_A4 = 1'b0;  assign SDRAM_A5 = 1'b0;
    assign SDRAM_A6 = 1'b0;  assign SDRAM_A7 = 1'b0;  assign SDRAM_A8 = 1'b0;
    assign SDRAM_A9 = 1'b0;  assign SDRAM_A10 = 1'b0; assign SDRAM_A11 = 1'b0;
    assign SDRAM_A12 = 1'b0;
    assign SDRAM_D0 = 1'bZ;  assign SDRAM_D1 = 1'bZ;  assign SDRAM_D2 = 1'bZ;
    assign SDRAM_D3 = 1'bZ;  assign SDRAM_D4 = 1'bZ;  assign SDRAM_D5 = 1'bZ;
    assign SDRAM_D6 = 1'bZ;  assign SDRAM_D7 = 1'bZ;  assign SDRAM_D8 = 1'bZ;
    assign SDRAM_D9 = 1'bZ;  assign SDRAM_D10 = 1'bZ; assign SDRAM_D11 = 1'bZ;
    assign SDRAM_D12 = 1'bZ; assign SDRAM_D13 = 1'bZ; assign SDRAM_D14 = 1'bZ;
    assign SDRAM_D15 = 1'bZ;
    assign FLASH_nCS = 1'b1; assign FLASH_MOSI = 1'b0;
    assign FLASH_nWP = 1'b1; assign FLASH_nHOLD = 1'b1;

    // =========================================================================
    // GPIO debug taps + heartbeat
    // =========================================================================
    reg [5:0] hb_cnt = 6'd0;
    reg       hb_led = 1'b0;
    always @(posedge clk) begin
        if (hb_cnt == 6'd49) begin
            hb_cnt <= 6'd0;
            hb_led <= ~hb_led;
        end else hb_cnt <= hb_cnt + 1'b1;
    end
    assign GPIO_1  = hb_led;
    assign GPIO_2  = phi2;
    assign GPIO_3  = cpu_sync;
    assign GPIO_4  = s0;
    assign GPIO_5  = s1;
    assign GPIO_6  = s2;
    assign GPIO_7  = bootstrap_done;
    assign GPIO_8  = bs_int;
    assign GPIO_9  = run_flag;
    assign GPIO_10 = cpu_rst;
    assign GPIO_11 = tx_avail;
    assign GPIO_12 = io_port_write;
    assign GPIO_13 = io_port_read;
    assign GPIO_14 = halted;
    assign GPIO_15 = ~nDEVICE_SELECT;
    assign GPIO_16 = R_nW;
    assign GPIO_17 = debug_pc[0];
    assign GPIO_18 = debug_pc[1];
    assign GPIO_19 = debug_pc[2];
    assign GPIO_20 = debug_pc[3];

endmodule
