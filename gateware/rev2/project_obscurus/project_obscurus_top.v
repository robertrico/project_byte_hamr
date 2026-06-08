// =============================================================================
// Byte Hamr Rev 2 - project_obscurus
// =============================================================================
// Minimal Apple II coprocessor playground.
//
// SDRAM monitor register port. The sdram_ctrl controller owns init + access;
// the 6502 drives byte-addressed SDRAM through the $C0Cx register window.
//
// Apple II side (slot 4 hardcoded):
//   $C400-$C4FF  slot ROM     6502 driver (slot_rom.mem)
//   $C800-$CFFF  expansion ROM (monitor.mem) — enabled by nI_O_SELECT, released
//                on $CFFF access
//   $C0C0    W   ADDR_LO      low byte of 16-bit offset within bank
//   $C0C1    W   ADDR_HI      high byte of offset
//   $C0C2    W   BANK_LO      low byte of 10-bit bank
//   $C0C3    W   BANK_HI      high 2 bits of bank
//   $C0C4    W   TRIG_RD      write any byte -> SDRAM read at {bank,addr}
//   $C0C5    R   STATUS       bit7=busy, bit6=ready
//   $C0C6    W   DATA         write -> SDRAM write at {bank,addr}
//            R   DATA         read  -> last SDRAM read result
//   $C0C9    W   CP_LADDR_LO  low byte of coproc BRAM load address
//   $C0CA    W   CP_LADDR_HI  high 5 bits of 13-bit load address
//   $C0CB    W   CP_WDATA     write -> coproc BRAM[laddr], laddr++
//   $C0CC    R   CP_RDATA     read  -> coproc BRAM[laddr], laddr++
//   $C0CD    W   CP_COUNT     write -> coproc task count
//   $C0CE-$C0CF  SCRATCH      R/W loopback registers
//
// addr auto-increments on completion of each SDRAM read/write (busy falling
// edge from the controller). busy is sticky from the access strobe until the
// first STATUS read after completion.
//
// GPIO (not routed on this board — compile-only debug taps).
// =============================================================================

module project_obscurus_top (
    input  wire        CLK_100MHz,

    // ----- SDRAM -----
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
    // Clock: 100 MHz -> 25 MHz via /4 counter (same pattern as signal_check)
    // =========================================================================
    (* keep = "true" *) reg [1:0] clk_div = 2'd0;
    always @(posedge CLK_100MHz) clk_div <= clk_div + 1'b1;
    (* keep = "true" *) wire clk = clk_div[1];

    // POR + nRES_READ sync: hold reset for ~16 clk cycles after config, then
    // OR with Apple II reset line so Ctrl-Reset re-inits SDRAM.
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
    // Heartbeat
    // =========================================================================
    reg [5:0] hb_cnt = 6'd0;
    reg       hb_led = 1'b0;
    always @(posedge clk) begin
        if (hb_cnt == 6'd49) begin
            hb_cnt <= 6'd0;
            hb_led <= ~hb_led;
        end else hb_cnt <= hb_cnt + 1'b1;
    end
    assign GPIO_1 = hb_led;

    // =========================================================================
    // SDRAM controller
    // =========================================================================
    wire        sdram_req;
    wire        sdram_we;
    wire [25:0] sdram_phys_addr;
    wire [7:0]  sdram_wdata;
    wire [7:0]  sdram_rdata;
    wire        sdram_busy;
    wire        ready;              // SDRAM init complete

    wire [12:0] ctrl_a;
    wire [15:0] ctrl_dq_out;
    wire        ctrl_dq_oe;

    wire [15:0] sdram_dq_in = {SDRAM_D15, SDRAM_D14, SDRAM_D13, SDRAM_D12,
                               SDRAM_D11, SDRAM_D10, SDRAM_D9, SDRAM_D8,
                               SDRAM_D7, SDRAM_D6, SDRAM_D5, SDRAM_D4,
                               SDRAM_D3, SDRAM_D2, SDRAM_D1, SDRAM_D0};

    sdram_ctrl u_sdram (
        .clk(clk), .rst_n(rst_n),
        .req(sdram_req), .we(sdram_we), .phys_addr(sdram_phys_addr),
        .wdata(sdram_wdata), .rdata(sdram_rdata),
        .busy(sdram_busy), .ready(ready),
        .SDRAM_CKE(SDRAM_CKE),
        .SDRAM_nCS(SDRAM_nCS), .SDRAM_nRAS(SDRAM_nRAS),
        .SDRAM_nCAS(SDRAM_nCAS), .SDRAM_nWE(SDRAM_nWE),
        .SDRAM_DQM0(SDRAM_DQM0), .SDRAM_DQM1(SDRAM_DQM1),
        .SDRAM_BA0(SDRAM_BA0), .SDRAM_BA1(SDRAM_BA1),
        .sdram_a(ctrl_a),
        .dq_out(ctrl_dq_out), .dq_oe(ctrl_dq_oe), .dq_in(sdram_dq_in)
    );

    assign SDRAM_CLK = clk;
    assign SDRAM_A0=ctrl_a[0];  assign SDRAM_A1=ctrl_a[1];  assign SDRAM_A2=ctrl_a[2];
    assign SDRAM_A3=ctrl_a[3];  assign SDRAM_A4=ctrl_a[4];  assign SDRAM_A5=ctrl_a[5];
    assign SDRAM_A6=ctrl_a[6];  assign SDRAM_A7=ctrl_a[7];  assign SDRAM_A8=ctrl_a[8];
    assign SDRAM_A9=ctrl_a[9];  assign SDRAM_A10=ctrl_a[10]; assign SDRAM_A11=ctrl_a[11];
    assign SDRAM_A12=ctrl_a[12];

    assign SDRAM_D0  = ctrl_dq_oe ? ctrl_dq_out[0]  : 1'bZ;
    assign SDRAM_D1  = ctrl_dq_oe ? ctrl_dq_out[1]  : 1'bZ;
    assign SDRAM_D2  = ctrl_dq_oe ? ctrl_dq_out[2]  : 1'bZ;
    assign SDRAM_D3  = ctrl_dq_oe ? ctrl_dq_out[3]  : 1'bZ;
    assign SDRAM_D4  = ctrl_dq_oe ? ctrl_dq_out[4]  : 1'bZ;
    assign SDRAM_D5  = ctrl_dq_oe ? ctrl_dq_out[5]  : 1'bZ;
    assign SDRAM_D6  = ctrl_dq_oe ? ctrl_dq_out[6]  : 1'bZ;
    assign SDRAM_D7  = ctrl_dq_oe ? ctrl_dq_out[7]  : 1'bZ;
    assign SDRAM_D8  = ctrl_dq_oe ? ctrl_dq_out[8]  : 1'bZ;
    assign SDRAM_D9  = ctrl_dq_oe ? ctrl_dq_out[9]  : 1'bZ;
    assign SDRAM_D10 = ctrl_dq_oe ? ctrl_dq_out[10] : 1'bZ;
    assign SDRAM_D11 = ctrl_dq_oe ? ctrl_dq_out[11] : 1'bZ;
    assign SDRAM_D12 = ctrl_dq_oe ? ctrl_dq_out[12] : 1'bZ;
    assign SDRAM_D13 = ctrl_dq_oe ? ctrl_dq_out[13] : 1'bZ;
    assign SDRAM_D14 = ctrl_dq_oe ? ctrl_dq_out[14] : 1'bZ;
    assign SDRAM_D15 = ctrl_dq_oe ? ctrl_dq_out[15] : 1'bZ;

    // =========================================================================
    // Slot ROM (combinational LUT read of $C400-$C4FF)
    // =========================================================================
    reg [7:0] slot_rom_mem [0:255];
    initial $readmemh("slot_rom.mem", slot_rom_mem);

    wire [7:0] slot_rom_data = slot_rom_mem[apple_addr[7:0]];

    // =========================================================================
    // Apple II bus interface
    // 2-FF sync nDEVICE_SELECT/R_nW/addr/data, continuous capture, commit on
    // rising edge of nDEVICE_SELECT. (Matches block_hamr/bus_interface.v.)
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

    // SCRATCH loopback registers ($C0C3..$C0CF except mapped ones)
    reg [7:0] scratch [0:15];
    integer ki;
    initial for (ki = 0; ki < 16; ki = ki + 1) scratch[ki] = 8'h00;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (ki = 0; ki < 16; ki = ki + 1) scratch[ki] <= 8'h00;
        end else if (nds_rise & ~wr_rw_latch & (wr_addr_latch >= 4'hE)) begin
            scratch[wr_addr_latch] <= wr_data_latch;
        end
    end

    // =========================================================================
    // Monitor register port
    // =========================================================================
    reg [15:0] m_addr = 16'd0;    // 16-bit offset within bank
    reg [9:0]  m_bank = 10'd0;
    reg [7:0]  m_wdata = 8'd0;
    reg        m_we = 1'b0;
    reg        m_busy = 1'b0;      // STATUS bit7 — latched at strobe
    reg        m_req = 1'b0;       // 1-cycle pulse to sdram_ctrl

    // ---- monitor is arbiter client c0 (priority); coproc is c1 ----
    wire        mon_busy;
    wire [7:0]  mon_rdata;
    wire        cop_req, cop_we;
    wire [25:0] cop_addr;
    wire [7:0]  cop_wdata;
    wire        cop_busy;
    wire [7:0]  cop_rdata;

    sdram_arb u_arb (
        .clk(clk), .rst_n(rst_n),
        .req(sdram_req), .we(sdram_we), .phys_addr(sdram_phys_addr),
        .wdata(sdram_wdata), .rdata(sdram_rdata), .busy(sdram_busy),
        .c0_req(m_req), .c0_we(m_we), .c0_addr({m_bank, m_addr}),
        .c0_wdata(m_wdata), .c0_busy(mon_busy), .c0_rdata(mon_rdata),
        .c1_req(cop_req), .c1_we(cop_we), .c1_addr(cop_addr),
        .c1_wdata(cop_wdata), .c1_busy(cop_busy), .c1_rdata(cop_rdata)
    );

    wire reg_wr = nds_rise & ~wr_rw_latch;   // register write commit
    // STATUS register read commit ($C0C5)
    wire status_rd = nds_rise & wr_rw_latch & (wr_addr_latch == 4'h5);

    // ---- C1 coproc load port ($C0C9-CD) ----
    reg  [12:0] m_laddr = 13'd0;
    wire        cp_wdata_wr = reg_wr & (wr_addr_latch == 4'hB);   // CP_WDATA write
    wire        cp_rdata_rd = nds_rise & wr_rw_latch & (wr_addr_latch == 4'hC); // CP_RDATA read
    wire        cp_count_wr = reg_wr & (wr_addr_latch == 4'hD);   // CP_COUNT write
    wire [7:0]  cp_ldata_out;

    coproc #(.CORE_ID(8'd0)) u_coproc (
        .clk(clk), .rst_n(rst_n), .ready(ready),
        .req(cop_req), .we(cop_we), .phys_addr(cop_addr), .wdata(cop_wdata),
        .busy(cop_busy), .rdata(cop_rdata),
        .laddr(m_laddr), .ldata_in(wr_data_latch), .lwr(cp_wdata_wr),
        .ldata_out(cp_ldata_out),
        .count_in(wr_data_latch), .count_wr(cp_count_wr)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) m_laddr <= 13'd0;
        else begin
            if (reg_wr & (wr_addr_latch == 4'h9)) m_laddr[7:0]  <= wr_data_latch;       // CP_LADDR_LO
            if (reg_wr & (wr_addr_latch == 4'hA)) m_laddr[12:8] <= wr_data_latch[4:0];  // CP_LADDR_HI
            if (cp_wdata_wr) m_laddr <= m_laddr + 1'b1;   // write-autoinc
            if (cp_rdata_rd) m_laddr <= m_laddr + 1'b1;   // read-autoinc (separate addr)
        end
    end

    // busy falling edge for the MONITOR's own op (via arbiter c0) = op complete
    reg mon_busy_d;
    always @(posedge clk) mon_busy_d <= mon_busy;
    wire op_done = mon_busy_d & ~mon_busy;

    // op_complete: set when the controller finishes the access, cleared when the
    // 6502 reads STATUS. m_busy stays asserted (sticky) from the strobe until the
    // FIRST STATUS read after completion clears it. This guarantees the monitor's
    // first poll always observes busy=1 even when the SDRAM op finishes faster
    // than the 6502 can issue its next read cycle — a single-cycle op_done pulse
    // would otherwise be missed by the polling loop.
    reg op_complete;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_addr<=16'd0; m_bank<=10'd0; m_wdata<=8'd0;
            m_we<=1'b0; m_busy<=1'b0; m_req<=1'b0; op_complete<=1'b0;
        end else begin
            m_req <= 1'b0;                       // default: pulse
            if (reg_wr) begin
                case (wr_addr_latch)
                    4'h0: m_addr[7:0]  <= wr_data_latch;       // ADDR_LO
                    4'h1: m_addr[15:8] <= wr_data_latch;       // ADDR_HI
                    4'h2: m_bank[7:0]  <= wr_data_latch;       // BANK_LO
                    4'h3: m_bank[9:8]  <= wr_data_latch[1:0];  // BANK_HI
                    4'h4: begin                                // TRIG_RD
                        m_we<=1'b0; m_req<=1'b1;
                        m_busy<=1'b1; op_complete<=1'b0;
                    end
                    4'h6: begin                                // DATA write
                        m_wdata<=wr_data_latch; m_we<=1'b1;
                        m_req<=1'b1; m_busy<=1'b1; op_complete<=1'b0;
                    end
                    default: ;
                endcase
            end
            if (op_done) begin
                op_complete <= 1'b1;
                m_addr <= m_addr + 1'b1;          // auto-inc on completion
            end
            // clear sticky busy on the STATUS read that follows completion
            if (status_rd && (op_complete || op_done))
                m_busy <= 1'b0;
        end
    end

    wire [7:0] status_byte = {m_busy, ready, 6'b0};  // bit7=busy, bit6=ready

    // -------- Read mux (combinational) --------
    reg [7:0] reg_data_out;
    always @(*) begin
        case (apple_addr[3:0])
            4'h5: reg_data_out = status_byte;   // STATUS
            4'h6: reg_data_out = mon_rdata;     // DATA (monitor's latched read)
            4'hC: reg_data_out = cp_ldata_out;  // CP_RDATA (coproc BRAM read-back)
            default: reg_data_out = scratch[apple_addr[3:0]];
        endcase
    end

    // =========================================================================
    // Expansion ROM ($C800-$CFFF)
    // =========================================================================
    reg [7:0] monitor_mem [0:2047];
    initial $readmemh("monitor.mem", monitor_mem);

    // nI_O_SELECT/STROBE sampled unsynced here: this is a slow ROM-enable latch,
    // so any metastable glitch is benign and self-corrects on the next access;
    // the ROM data path itself (exp_rom_data/exp_read below) is combinational.
    reg rom_en = 1'b0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rom_en <= 1'b0;
        else begin
            if (~nI_O_SELECT) rom_en <= 1'b1;
            if (~nI_O_STROBE && apple_addr[10:0]==11'h7FF) rom_en <= 1'b0; // $CFFF
        end
    end

    // Expansion-ROM ARM soft-switch (default 0 = card silent on shared $C800 bus).
    // Arm: write $AA to $C0C7.  Disarm: write $AA to $C0C8.  Reset/POR -> disarm.
    // Symmetric magic guard: only the exact byte to the exact address flips state,
    // so stray/rogue writes can't arm us. Gates ALL $C800-$CFFF drive below.
    reg rom_armed = 1'b0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rom_armed <= 1'b0;
        else if (nds_rise & ~wr_rw_latch & (wr_data_latch == 8'hAA)) begin
            if      (wr_addr_latch == 4'h7) rom_armed <= 1'b1;   // ROM_ARM
            else if (wr_addr_latch == 4'h8) rom_armed <= 1'b0;   // ROM_DISARM
        end
    end

    wire [7:0] exp_rom_data = monitor_mem[apple_addr[10:0]];
    wire       exp_read = rom_en & rom_armed & ~nI_O_STROBE & R_nW;

    // -------- Drive D bus --------
    wire device_read = ~nDEVICE_SELECT & R_nW;
    wire rom_read    = ~nI_O_SELECT    & R_nW;

    wire [7:0] d_out = rom_read    ? slot_rom_data :
                       exp_read    ? exp_rom_data  :
                       device_read ? reg_data_out  : 8'h00;
    wire       d_oe  = rom_read | exp_read | device_read;

    assign D0 = d_oe ? d_out[0] : 1'bZ;
    assign D1 = d_oe ? d_out[1] : 1'bZ;
    assign D2 = d_oe ? d_out[2] : 1'bZ;
    assign D3 = d_oe ? d_out[3] : 1'bZ;
    assign D4 = d_oe ? d_out[4] : 1'bZ;
    assign D5 = d_oe ? d_out[5] : 1'bZ;
    assign D6 = d_oe ? d_out[6] : 1'bZ;
    assign D7 = d_oe ? d_out[7] : 1'bZ;

    // =========================================================================
    // U12 level shifter OE (active low). Enable when our slot space is hit.
    // Asserts on BOTH read and write (it is the '245 buffer OE, live both ways).
    // =========================================================================
    wire slot_active = ~nDEVICE_SELECT | ~nI_O_SELECT | (rom_en & rom_armed & ~nI_O_STROBE);
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
    assign nRES = 1'b1;  // OPENDRAIN: released

    // =========================================================================
    // GPIO debug (not routed on this board — compile-only)
    // =========================================================================
    assign GPIO_2  = ready;
    assign GPIO_3  = sdram_busy;
    assign GPIO_4  = m_busy;
    assign GPIO_5  = ~nDEVICE_SELECT;
    assign GPIO_6  = ~nI_O_SELECT;
    assign GPIO_7  = rom_en;
    assign GPIO_8  = R_nW;
    assign GPIO_9  = nRES_READ;
    assign GPIO_10 = ~nI_O_STROBE;
    assign GPIO_11 = m_req;
    assign GPIO_12 = op_done;
    assign GPIO_13 = m_addr[0];
    assign GPIO_14 = m_addr[1];
    assign GPIO_15 = m_addr[2];
    assign GPIO_16 = m_addr[3];
    assign GPIO_17 = sdram_rdata[0];
    assign GPIO_18 = sdram_rdata[1];
    assign GPIO_19 = sdram_rdata[2];
    assign GPIO_20 = sdram_rdata[3];

endmodule
