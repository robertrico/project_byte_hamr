`define SIMULATION
`timescale 1ns / 1ps
// =============================================================================
// SmartPort Validation Testbench — Liron ROM Bit-Exact Protocol Tests
// =============================================================================
// Drives the Apple II bus EXACTLY as the Liron ROM would, through the IWM
// model, verifying the full signal path:
//   Bus writes → IWM → wrdata → fm_decoder → sp_codec → sp_device
//   sp_device → sp_codec → tx_pkt_buf → fm_encoder → rddata → IWM → Bus reads
//
// Stimulus derived from: liron-if.asm, liron.asm, PicoPort (sp_encode.c),
// FujiNet (iwm_ll.cpp). All test vectors use Liron-accurate packet format
// (STAT=$80 always, command code in payload[0]).
//
// Tests:
//   T0: IWM bus infrastructure (register access, write/read mode)
//   T1: INIT command full round-trip
//   T2: STATUS command (4-byte payload response)
//   T3: READBLOCK (512-byte payload response)
// =============================================================================

module sp_validate_tb;

    // =========================================================================
    // Test infrastructure
    // =========================================================================
    integer pass_count = 0;
    integer fail_count = 0;
    integer test_num   = 0;

    task check(input [511:0] name, input condition);
    begin
        if (condition) begin
            $display("  PASS: %0s", name);
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: %0s", name);
            fail_count = fail_count + 1;
        end
    end
    endtask

    // =========================================================================
    // Clock generation
    // =========================================================================
    reg clk_7m  = 0;
    reg clk_25m = 0;
    always #69.8  clk_7m  = ~clk_7m;   // ~7.16 MHz
    always #20.0  clk_25m = ~clk_25m;   // 25 MHz

    // =========================================================================
    // Apple II Bus Signals
    // =========================================================================
    reg  [11:0] addr;
    wire [7:0]  data_bus;
    reg  [7:0]  data_drive;
    reg         data_drive_en;
    reg         Q3, R_nW;
    reg         nDEVICE_SELECT, nI_O_SELECT, nI_O_STROBE;
    wire        nRES;
    reg         RDY;
    wire        nIRQ, nNMI, nDMA, nINH;
    reg         DMA_OUT, INT_OUT;
    wire        DMA_IN, INT_IN;
    reg         PHI0, PHI1, uSync;

    assign data_bus = data_drive_en ? data_drive : 8'hZZ;
    wire D0 = data_bus[0], D1 = data_bus[1], D2 = data_bus[2], D3 = data_bus[3];
    wire D4 = data_bus[4], D5 = data_bus[5], D6 = data_bus[6], D7 = data_bus[7];

    // =========================================================================
    // GPIO
    // =========================================================================
    wire GPIO1, GPIO2, GPIO3, GPIO4, GPIO5;
    wire GPIO6, GPIO7, GPIO10, GPIO11, GPIO12;
    reg  GPIO8, GPIO9;

    // =========================================================================
    // SDRAM Signals
    // =========================================================================
    wire SDRAM_CLK, SDRAM_CKE, SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE;
    wire SDRAM_DQM0, SDRAM_DQM1, SDRAM_BA0, SDRAM_BA1;
    wire SDRAM_A0, SDRAM_A1, SDRAM_A2, SDRAM_A3, SDRAM_A4;
    wire SDRAM_A5, SDRAM_A6, SDRAM_A7, SDRAM_A8, SDRAM_A9;
    wire SDRAM_A10, SDRAM_A11, SDRAM_A12;
    wire SDRAM_D0, SDRAM_D1, SDRAM_D2, SDRAM_D3;
    wire SDRAM_D4, SDRAM_D5, SDRAM_D6, SDRAM_D7;
    wire SDRAM_D8, SDRAM_D9, SDRAM_D10, SDRAM_D11;
    wire SDRAM_D12, SDRAM_D13, SDRAM_D14, SDRAM_D15;

    // Flash
    wire FLASH_nCS, FLASH_MOSI, FLASH_nWP, FLASH_nHOLD, FLASH_MISO;

    // =========================================================================
    // Behavioral SDRAM Model (from iwm_hamr_tb)
    // =========================================================================
    wire [12:0] sdram_a_bus = {SDRAM_A12, SDRAM_A11, SDRAM_A10, SDRAM_A9,
                               SDRAM_A8, SDRAM_A7, SDRAM_A6, SDRAM_A5,
                               SDRAM_A4, SDRAM_A3, SDRAM_A2, SDRAM_A1, SDRAM_A0};
    wire [15:0] sdram_dq_in = {SDRAM_D15, SDRAM_D14, SDRAM_D13, SDRAM_D12,
                                SDRAM_D11, SDRAM_D10, SDRAM_D9,  SDRAM_D8,
                                SDRAM_D7,  SDRAM_D6,  SDRAM_D5,  SDRAM_D4,
                                SDRAM_D3,  SDRAM_D2,  SDRAM_D1,  SDRAM_D0};

    wire [3:0] sdram_cmd = {SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE};
    localparam [3:0] SCMD_NOP       = 4'b0111;
    localparam [3:0] SCMD_ACTIVE    = 4'b0011;
    localparam [3:0] SCMD_READ      = 4'b0101;
    localparam [3:0] SCMD_WRITE     = 4'b0100;
    localparam [3:0] SCMD_PRECHARGE = 4'b0010;
    localparam [3:0] SCMD_REFRESH   = 4'b0001;
    localparam [3:0] SCMD_LOAD_MODE = 4'b0000;

    reg [15:0] sdram_mem [0:65535];
    reg [15:0] sdram_read_addr;
    reg [2:0]  sdram_cas_delay;
    reg [12:0] sdram_active_row;
    reg [1:0]  sdram_active_bank;

    // DQ drive uses blocking assignments for immediate visibility.
    // sdram_driving is set/cleared with blocking assigns so the
    // combinatorial DQ output is visible in the same time step
    // that the controller's sdram_dq_sample NBA evaluates.
    reg        sdram_driving = 0;
    reg [15:0] sdram_drive_data = 16'd0;
    reg        sdram_wr_pending = 0;       // 1-cycle delayed write
    reg [15:0] sdram_wr_addr_pending = 0;  // address for delayed write

    assign SDRAM_D0  = sdram_driving ? sdram_drive_data[0]  : 1'bZ;
    assign SDRAM_D1  = sdram_driving ? sdram_drive_data[1]  : 1'bZ;
    assign SDRAM_D2  = sdram_driving ? sdram_drive_data[2]  : 1'bZ;
    assign SDRAM_D3  = sdram_driving ? sdram_drive_data[3]  : 1'bZ;
    assign SDRAM_D4  = sdram_driving ? sdram_drive_data[4]  : 1'bZ;
    assign SDRAM_D5  = sdram_driving ? sdram_drive_data[5]  : 1'bZ;
    assign SDRAM_D6  = sdram_driving ? sdram_drive_data[6]  : 1'bZ;
    assign SDRAM_D7  = sdram_driving ? sdram_drive_data[7]  : 1'bZ;
    assign SDRAM_D8  = sdram_driving ? sdram_drive_data[8]  : 1'bZ;
    assign SDRAM_D9  = sdram_driving ? sdram_drive_data[9]  : 1'bZ;
    assign SDRAM_D10 = sdram_driving ? sdram_drive_data[10] : 1'bZ;
    assign SDRAM_D11 = sdram_driving ? sdram_drive_data[11] : 1'bZ;
    assign SDRAM_D12 = sdram_driving ? sdram_drive_data[12] : 1'bZ;
    assign SDRAM_D13 = sdram_driving ? sdram_drive_data[13] : 1'bZ;
    assign SDRAM_D14 = sdram_driving ? sdram_drive_data[14] : 1'bZ;
    assign SDRAM_D15 = sdram_driving ? sdram_drive_data[15] : 1'bZ;

    integer sdram_init_i;
    initial begin
        for (sdram_init_i = 0; sdram_init_i < 65536; sdram_init_i = sdram_init_i + 1)
            sdram_mem[sdram_init_i] = 16'h0000;
        sdram_cas_delay   = 3'd0;
        sdram_drive_data  = 16'h0000;
        sdram_active_row  = 13'd0;
        sdram_active_bank = 2'd0;
    end

    // ---- SDRAM Behavioral Read Pipeline ----
    //
    // Critical: the DQ drive must use BLOCKING assignments so the
    // controller's sdram_dq_sample (also at posedge) sees the data
    // in the same simulation time step. With NBA, the model's DQ
    // update isn't visible until after ALL posedge blocks execute,
    // but sdram_dq_sample has already captured the old (Z) value.
    //
    // Using blocking assignments for the drive control makes the
    // combinatorial assign on SDRAM_Dx update immediately, before
    // the controller's NBA captures sdram_dq_sample.
    //
    // Controller timeline:
    //   T+0: CMD_READ, delay_cnt<=3
    //   T+1: delay_cnt 3→2   T+2: 2→1   T+3: 1→0
    //   T+4: delay_cnt==0, req_rdata <= sdram_dq_sample (from T+3)
    // Model must drive DQ stable at T+2 posedge so T+3 sample is valid.
    // With cas_delay=2: counts T+0→T+1(=1)→T+2(=0, fire). At T+2
    // posedge, blocking assign sets sdram_driving=1. The assign on
    // SDRAM_Dx updates immediately. Controller's NBA `sdram_dq_sample
    // <= SDRAM_DQ` sees the driven value.
    reg [2:0] sdram_hold_cnt = 3'd0;

    always @(posedge SDRAM_CLK) begin
        // Execute delayed write: read controller's registered output directly.
        // sdram_dq_out was set via NBA on the WRITE cmd cycle, now stable.
        if (sdram_wr_pending) begin
            sdram_mem[sdram_wr_addr_pending] = dut.u_sdram_controller.sdram_dq_out;
            sdram_wr_pending = 0;
        end

        // Command processing
        case (sdram_cmd)
            SCMD_ACTIVE: begin
                sdram_active_row  = sdram_a_bus;
                sdram_active_bank = {SDRAM_BA1, SDRAM_BA0};
            end
            SCMD_WRITE: begin
                // Controller's sdram_dq_out is set via NBA in the same
                // cycle as WRITE cmd. Use hierarchical ref to get the
                // latched write data directly — avoids DQ bus NBA race.
                sdram_wr_pending = 1;
                sdram_wr_addr_pending = {sdram_active_row[2:0], sdram_a_bus[9:0], sdram_active_bank[0]};
            end
            SCMD_READ: begin
                sdram_read_addr = {sdram_active_row[2:0], sdram_a_bus[9:0], sdram_active_bank[0]};
                sdram_cas_delay = 3'd2;
            end
            default: ;
        endcase

        // CAS countdown: drive DQ when CAS completes.
        if (sdram_cas_delay > 3'd0) begin
            sdram_cas_delay = sdram_cas_delay - 3'd1;
            if (sdram_cas_delay == 3'd0) begin
                sdram_driving    = 1;
                sdram_drive_data = sdram_mem[sdram_read_addr[15:0]];
            end
        end
        if (sdram_cmd == SCMD_ACTIVE || sdram_cmd == SCMD_PRECHARGE)
            sdram_driving = 0;

        // Trace controller's captured data around word 36
        if (test_num >= 3 && dut.u_sdram_controller.req_rdata_valid &&
            dut.u_sdram_arbiter.xfer_cnt >= 9'd34 && dut.u_sdram_arbiter.xfer_cnt <= 9'd38)
            $display("  [CTRL] rdata_valid: xfer=%0d rdata=%04h dq_sample=%04h driving=%b DQ=%04h",
                     dut.u_sdram_arbiter.xfer_cnt,
                     dut.u_sdram_controller.req_rdata,
                     dut.u_sdram_controller.sdram_dq_sample,
                     sdram_driving,
                     {SDRAM_D15,SDRAM_D14,SDRAM_D13,SDRAM_D12,SDRAM_D11,SDRAM_D10,SDRAM_D9,SDRAM_D8,
                      SDRAM_D7,SDRAM_D6,SDRAM_D5,SDRAM_D4,SDRAM_D3,SDRAM_D2,SDRAM_D1,SDRAM_D0});
    end

    // =========================================================================
    // Behavioral SPI Flash Model (from iwm_hamr_tb)
    // =========================================================================
    reg [7:0] flash_mem [0:4095];
    reg [7:0]  flash_cmd_reg;
    reg [23:0] flash_addr_reg;
    reg [5:0]  flash_bit_cnt;
    reg        flash_cmd_phase;
    reg [7:0]  flash_data_byte;
    reg [2:0]  flash_out_bit_cnt;
    reg [23:0] flash_read_addr;
    reg        flash_miso_reg;
    assign FLASH_MISO = flash_miso_reg;
    wire flash_sck = dut.u_flash_reader.flash_sck_pin;

    integer flash_init_i;
    initial begin
        for (flash_init_i = 0; flash_init_i < 4096; flash_init_i = flash_init_i + 1)
            flash_mem[flash_init_i] = flash_init_i[7:0];
        flash_miso_reg    = 1'bz;
        flash_bit_cnt     = 6'd0;
        flash_cmd_phase   = 1'b1;
        flash_cmd_reg     = 8'd0;
        flash_addr_reg    = 24'd0;
        flash_out_bit_cnt = 3'd0;
        flash_read_addr   = 24'd0;
        flash_data_byte   = 8'd0;
    end

    always @(posedge FLASH_nCS) begin
        flash_bit_cnt   <= 6'd0;
        flash_cmd_phase <= 1'b1;
        flash_miso_reg  <= 1'bz;
    end
    always @(negedge FLASH_nCS) begin
        flash_bit_cnt   <= 6'd0;
        flash_cmd_phase <= 1'b1;
        flash_cmd_reg   <= 8'd0;
        flash_addr_reg  <= 24'd0;
    end
    always @(posedge flash_sck) begin
        if (!FLASH_nCS && flash_cmd_phase) begin
            if (flash_bit_cnt < 6'd8)
                flash_cmd_reg <= {flash_cmd_reg[6:0], FLASH_MOSI};
            else if (flash_bit_cnt < 6'd32)
                flash_addr_reg <= {flash_addr_reg[22:0], FLASH_MOSI};
            flash_bit_cnt <= flash_bit_cnt + 6'd1;
            if (flash_bit_cnt == 6'd31) begin
                flash_cmd_phase   <= 1'b0;
                flash_read_addr   <= {flash_addr_reg[22:0], FLASH_MOSI};
                flash_out_bit_cnt <= 3'd0;
            end
        end
    end
    always @(negedge flash_sck) begin
        if (!FLASH_nCS && !flash_cmd_phase) begin
            if (flash_out_bit_cnt == 3'd0)
                flash_data_byte <= flash_mem[flash_read_addr[11:0]];
            flash_miso_reg <= (flash_out_bit_cnt == 3'd0) ?
                flash_mem[flash_read_addr[11:0]][7] :
                flash_data_byte[7 - flash_out_bit_cnt];
            flash_out_bit_cnt <= flash_out_bit_cnt + 3'd1;
            if (flash_out_bit_cnt == 3'd7) begin
                flash_read_addr   <= flash_read_addr + 24'd1;
                flash_out_bit_cnt <= 3'd0;
            end
        end
    end

    // =========================================================================
    // DUT Instantiation
    // =========================================================================
    iwm_hamr_top dut (
        .addr(addr),
        .D0(D0), .D1(D1), .D2(D2), .D3(D3),
        .D4(D4), .D5(D5), .D6(D6), .D7(D7),
        .sig_7M(clk_7m), .Q3(Q3), .R_nW(R_nW),
        .nDEVICE_SELECT(nDEVICE_SELECT),
        .nI_O_SELECT(nI_O_SELECT),
        .nI_O_STROBE(nI_O_STROBE),
        .nRES(nRES), .RDY(RDY),
        .nIRQ(nIRQ), .nNMI(nNMI), .nDMA(nDMA), .nINH(nINH),
        .DMA_OUT(DMA_OUT), .DMA_IN(DMA_IN),
        .INT_OUT(INT_OUT), .INT_IN(INT_IN),
        .PHI0(PHI0), .PHI1(PHI1), .uSync(uSync),
        .CLK_25MHz(clk_25m),
        .SDRAM_CLK(SDRAM_CLK), .SDRAM_CKE(SDRAM_CKE),
        .SDRAM_nCS(SDRAM_nCS), .SDRAM_nRAS(SDRAM_nRAS),
        .SDRAM_nCAS(SDRAM_nCAS), .SDRAM_nWE(SDRAM_nWE),
        .SDRAM_DQM0(SDRAM_DQM0), .SDRAM_DQM1(SDRAM_DQM1),
        .SDRAM_BA0(SDRAM_BA0), .SDRAM_BA1(SDRAM_BA1),
        .SDRAM_A0(SDRAM_A0), .SDRAM_A1(SDRAM_A1), .SDRAM_A2(SDRAM_A2),
        .SDRAM_A3(SDRAM_A3), .SDRAM_A4(SDRAM_A4), .SDRAM_A5(SDRAM_A5),
        .SDRAM_A6(SDRAM_A6), .SDRAM_A7(SDRAM_A7), .SDRAM_A8(SDRAM_A8),
        .SDRAM_A9(SDRAM_A9), .SDRAM_A10(SDRAM_A10), .SDRAM_A11(SDRAM_A11),
        .SDRAM_A12(SDRAM_A12),
        .SDRAM_D0(SDRAM_D0), .SDRAM_D1(SDRAM_D1), .SDRAM_D2(SDRAM_D2),
        .SDRAM_D3(SDRAM_D3), .SDRAM_D4(SDRAM_D4), .SDRAM_D5(SDRAM_D5),
        .SDRAM_D6(SDRAM_D6), .SDRAM_D7(SDRAM_D7), .SDRAM_D8(SDRAM_D8),
        .SDRAM_D9(SDRAM_D9), .SDRAM_D10(SDRAM_D10), .SDRAM_D11(SDRAM_D11),
        .SDRAM_D12(SDRAM_D12), .SDRAM_D13(SDRAM_D13), .SDRAM_D14(SDRAM_D14),
        .SDRAM_D15(SDRAM_D15),
        .FLASH_nCS(FLASH_nCS), .FLASH_MOSI(FLASH_MOSI),
        .FLASH_MISO(FLASH_MISO), .FLASH_nWP(FLASH_nWP),
        .FLASH_nHOLD(FLASH_nHOLD),
        .GPIO1(GPIO1), .GPIO2(GPIO2), .GPIO3(GPIO3), .GPIO4(GPIO4),
        .GPIO5(GPIO5), .GPIO6(GPIO6), .GPIO7(GPIO7),
        .GPIO8(GPIO8), .GPIO9(GPIO9),
        .GPIO10(GPIO10), .GPIO11(GPIO11), .GPIO12(GPIO12)
    );

    // =========================================================================
    // VCD dump
    // =========================================================================
    initial begin
        $dumpfile("sp_validate_tb.vcd");
        $dumpvars(0, sp_validate_tb);
    end

    // =========================================================================
    // IWM Register Address Constants
    // =========================================================================
    localparam [3:0] IWM_PH0_OFF   = 4'h0;   // REQ off
    localparam [3:0] IWM_PH0_ON    = 4'h1;   // REQ on
    localparam [3:0] IWM_PH1_OFF   = 4'h2;
    localparam [3:0] IWM_PH1_ON    = 4'h3;
    localparam [3:0] IWM_PH2_OFF   = 4'h4;
    localparam [3:0] IWM_PH2_ON    = 4'h5;
    localparam [3:0] IWM_PH3_OFF   = 4'h6;
    localparam [3:0] IWM_PH3_ON    = 4'h7;
    localparam [3:0] IWM_MOTOR_OFF = 4'h8;
    localparam [3:0] IWM_MOTOR_ON  = 4'h9;
    localparam [3:0] IWM_DRIVE_1   = 4'hA;
    localparam [3:0] IWM_DRIVE_2   = 4'hB;
    localparam [3:0] IWM_Q6L       = 4'hC;   // Q6 OFF: data/handshake register
    localparam [3:0] IWM_Q6H       = 4'hD;   // Q6 ON: status read / data write
    localparam [3:0] IWM_Q7L       = 4'hE;   // Q7 OFF: read mode
    localparam [3:0] IWM_Q7H       = 4'hF;   // Q7 ON: write mode / mode reg

    // =========================================================================
    // Bus Access Tasks — Simulates 6502 LDA/STA to IWM registers
    // =========================================================================
    // Each access: ~7 fclk total (~1us, matching 1 MHz 6502 bus cycle).
    // nDEVICE_SELECT LOW for ~4 fclk (falling edge latches IWM state).
    // =========================================================================

    task iwm_bus_read;
        input  [3:0] iwm_addr;
        output [7:0] rdata;
    begin
        addr = {8'h00, iwm_addr};
        R_nW = 1;
        data_drive_en = 0;
        @(posedge clk_7m);
        nDEVICE_SELECT = 0;
        repeat (3) @(posedge clk_7m);
        rdata = {D7, D6, D5, D4, D3, D2, D1, D0};
        nDEVICE_SELECT = 1;
        repeat (3) @(posedge clk_7m);
    end
    endtask

    task iwm_bus_write;
        input [3:0] iwm_addr;
        input [7:0] wdata;
    begin
        addr = {8'h00, iwm_addr};
        R_nW = 0;
        data_drive = wdata;
        data_drive_en = 1;
        @(posedge clk_7m);
        nDEVICE_SELECT = 0;
        repeat (4) @(posedge clk_7m);
        nDEVICE_SELECT = 1;
        data_drive_en = 0;
        R_nW = 1;
        repeat (3) @(posedge clk_7m);
    end
    endtask

    // Side-effect read: used for phase/motor/drive selects where we
    // don't care about read data, just the register select side-effect.
    task iwm_touch;
        input [3:0] iwm_addr;
    begin : iwm_touch_blk
        reg [7:0] dummy;
        iwm_bus_read(iwm_addr, dummy);
    end
    endtask

    // =========================================================================
    // SmartPort Bus Control Tasks (matching Liron ROM sequences)
    // =========================================================================

    // smartport_bus_enable: PH1 ON, PH3 ON
    task sp_bus_enable;
    begin
        iwm_touch(IWM_PH1_ON);
        iwm_touch(IWM_PH3_ON);
    end
    endtask

    // smartport_bus_disable: all phases OFF
    task sp_bus_disable;
    begin
        iwm_touch(IWM_PH0_OFF);
        iwm_touch(IWM_PH1_OFF);
        iwm_touch(IWM_PH2_OFF);
        iwm_touch(IWM_PH3_OFF);
    end
    endtask

    // IWM mode register write (Liron: motor OFF, Q6H, STA Q7H)
    task iwm_write_mode_reg;
        input [7:0] mode_val;
    begin : iwm_write_mode_blk
        reg [7:0] rdata;
        iwm_touch(IWM_MOTOR_OFF);       // Motor off (required for mode write)
        iwm_touch(IWM_Q6H);             // Assert Q6H
        iwm_bus_write(IWM_Q7H, mode_val); // Write mode register
        // Verify: read status, low 5 bits should match
        iwm_bus_read(IWM_Q7L, rdata);
        if ((rdata & 8'h1F) !== (mode_val & 8'h1F))
            $display("  WARN: mode reg verify: got %02h, expected %02h",
                     rdata & 8'h1F, mode_val & 8'h1F);
    end
    endtask

    // =========================================================================
    // IWM Write Path Tasks
    // =========================================================================

    // Write a single byte through IWM handshake (Liron: poll Q6L bit 7, STA Q6H)
    task iwm_write_byte;
        input [7:0] byte_val;
    begin : iwm_write_byte_blk
        reg [7:0] rdata;
        integer timeout;
        for (timeout = 0; timeout < 500; timeout = timeout + 1) begin
            iwm_bus_read(IWM_Q6L, rdata);
            if (rdata[7]) begin
                iwm_bus_write(IWM_Q6H, byte_val);
                disable iwm_write_byte_blk;
            end
        end
        $display("  ERROR: iwm_write_byte(%02h) handshake timeout", byte_val);
    end
    endtask

    // Wait for write underrun (Liron $C92C: poll Q6L bit 6)
    task iwm_wait_underrun;
    begin : iwm_wait_underrun_blk
        reg [7:0] rdata;
        integer timeout;
        for (timeout = 0; timeout < 5000; timeout = timeout + 1) begin
            iwm_bus_read(IWM_Q6L, rdata);
            if (!rdata[6])
                disable iwm_wait_underrun_blk;
        end
        $display("  ERROR: iwm_wait_underrun timeout");
    end
    endtask

    // =========================================================================
    // IWM Read Path Tasks
    // =========================================================================

    // Read one data byte from IWM (poll Q6L bit 7 = x7 = data ready)
    task iwm_read_data_byte;
        output [7:0] byte_val;
        output       valid;
        input integer max_polls;
    begin : iwm_read_data_byte_blk
        reg [7:0] rdata;
        integer timeout;
        valid = 0;
        byte_val = 8'h00;
        for (timeout = 0; timeout < max_polls; timeout = timeout + 1) begin
            iwm_bus_read(IWM_Q6L, rdata);
            if (rdata[7]) begin
                byte_val = rdata;
                valid = 1;
                // IWM x7 clears 14 fclk after a valid read, but can
                // re-set if the shift register re-frames the same data.
                // Wait until x7 goes LOW to guarantee we won't re-read.
                // Then return — next poll will wait for the NEW x7 rise.
                begin : wait_x7_clear
                    integer w;
                    for (w = 0; w < 300; w = w + 1) begin
                        @(posedge clk_7m);
                        if (!dut.u_iwm.x7)
                            disable wait_x7_clear;
                    end
                end
                disable iwm_read_data_byte_blk;
            end
        end
    end
    endtask

    // =========================================================================
    // SmartPort Packet Builder
    // =========================================================================
    // Builds Liron-accurate command packets. Returns wire bytes (post-sync,
    // starting from DEST through PEND). Sync is sent separately.
    //
    // Payload format (decoded bytes):
    //   [0] = command code (INIT=5, STATUS=0, READBLOCK=1, WRITEBLOCK=2)
    //   [1] = param count
    //   [2] = unit number
    //   [3] = buffer ptr low
    //   [4] = block_lo / status_code
    //   [5] = block_mid
    //   [6] = block_hi
    //   [7..8] = padding (0x00)
    // =========================================================================

    // Packet buffer for command TX (max ~30 bytes for commands)
    reg [7:0] cmd_pkt [0:63];
    integer   cmd_pkt_len;

    // Build a SmartPort command packet
    task build_command_packet;
        input [6:0] dest;           // destination device ID
        input [7:0] cmd_code;       // command code (in payload[0])
        input [7:0] param_count;    // param count
        input [7:0] unit_num;       // unit number
        input [7:0] status_or_block_lo;
        input [7:0] block_mid;
        input [7:0] block_hi;
    begin : build_cmd
        reg [7:0] payload [0:8];
        reg [7:0] oddmsb;
        reg [7:0] grpmsb;
        reg [7:0] checksum;
        integer   numodds, numgrps;
        integer   i, idx;

        // Build 9-byte payload (Liron always sends 9 bytes for commands)
        payload[0] = cmd_code;
        payload[1] = param_count;
        payload[2] = unit_num;
        payload[3] = 8'h00;         // buffer ptr
        payload[4] = status_or_block_lo;
        payload[5] = block_mid;
        payload[6] = block_hi;
        payload[7] = 8'h00;
        payload[8] = 8'h00;

        numodds = 9 % 7;   // = 2
        numgrps = 9 / 7;   // = 1

        idx = 0;
        checksum = 8'h00;

        // Header
        cmd_pkt[idx] = {1'b1, dest};           checksum = checksum ^ cmd_pkt[idx]; idx = idx + 1;
        cmd_pkt[idx] = 8'h80;                  checksum = checksum ^ cmd_pkt[idx]; idx = idx + 1; // source = host
        cmd_pkt[idx] = 8'h80;                  checksum = checksum ^ cmd_pkt[idx]; idx = idx + 1; // ptype = command
        cmd_pkt[idx] = 8'h80;                  checksum = checksum ^ cmd_pkt[idx]; idx = idx + 1; // aux
        cmd_pkt[idx] = 8'h80;                  checksum = checksum ^ cmd_pkt[idx]; idx = idx + 1; // stat (always $80!)
        cmd_pkt[idx] = numodds[7:0] | 8'h80;   checksum = checksum ^ cmd_pkt[idx]; idx = idx + 1; // oddcnt
        cmd_pkt[idx] = numgrps[7:0] | 8'h80;   checksum = checksum ^ cmd_pkt[idx]; idx = idx + 1; // grp7cnt

        // Odd bytes
        if (numodds > 0) begin
            // ODDMSB: bit (6-i) = bit 7 of payload[i]
            oddmsb = 8'h80;
            for (i = 0; i < numodds; i = i + 1)
                oddmsb = oddmsb | ((payload[i] >> (1+i)) & (8'h80 >> (1+i)));
            cmd_pkt[idx] = oddmsb; idx = idx + 1;
            for (i = 0; i < numodds; i = i + 1) begin
                cmd_pkt[idx] = payload[i] | 8'h80;
                idx = idx + 1;
            end
        end

        // Groups of 7
        for (i = 0; i < numgrps; i = i + 1) begin : grp_loop
            integer j, base;
            base = numodds + i * 7;
            grpmsb = 8'h80;
            for (j = 0; j < 7; j = j + 1)
                grpmsb = grpmsb | ((payload[base + j] >> (1+j)) & (8'h80 >> (1+j)));
            cmd_pkt[idx] = grpmsb; idx = idx + 1;
            for (j = 0; j < 7; j = j + 1) begin
                cmd_pkt[idx] = payload[base + j] | 8'h80;
                idx = idx + 1;
            end
        end

        // XOR raw payload bytes into checksum
        for (i = 0; i < 9; i = i + 1)
            checksum = checksum ^ payload[i];

        // Checksum bytes
        cmd_pkt[idx] = checksum | 8'hAA;                 idx = idx + 1;
        cmd_pkt[idx] = (checksum >> 1) | 8'hAA;          idx = idx + 1;

        // PEND
        cmd_pkt[idx] = 8'hC8; idx = idx + 1;

        cmd_pkt_len = idx;
    end
    endtask

    // =========================================================================
    // DATA Packet Builder (for WRITEBLOCK phase 2)
    // =========================================================================
    // Builds a Liron-accurate DATA packet with 512-byte payload.
    // Wire format: DEST SRC PTYPE AUX STAT ODDCNT GRP7CNT [odd] [groups] CHK CHK PEND
    // =========================================================================
    reg [7:0] data_pkt [0:700];
    integer   data_pkt_len;
    reg [7:0] wb_data [0:511];    // WRITEBLOCK payload source

    task build_data_packet;
        input [6:0] dest;
    begin : build_data_blk
        reg [7:0] oddmsb, grpmsb, checksum;
        integer numodds, numgrps, idx, i, j, base;

        numodds = 512 % 7;  // = 1
        numgrps = 512 / 7;  // = 73

        idx = 0;
        checksum = 8'h00;

        // Header
        data_pkt[idx] = {1'b1, dest};   checksum = checksum ^ data_pkt[idx]; idx = idx + 1;
        data_pkt[idx] = 8'h80;          checksum = checksum ^ data_pkt[idx]; idx = idx + 1; // source=host
        data_pkt[idx] = 8'h82;          checksum = checksum ^ data_pkt[idx]; idx = idx + 1; // ptype=DATA
        data_pkt[idx] = 8'h80;          checksum = checksum ^ data_pkt[idx]; idx = idx + 1; // aux
        data_pkt[idx] = 8'h80;          checksum = checksum ^ data_pkt[idx]; idx = idx + 1; // stat
        data_pkt[idx] = numodds[7:0] | 8'h80; checksum = checksum ^ data_pkt[idx]; idx = idx + 1;
        data_pkt[idx] = numgrps[7:0] | 8'h80; checksum = checksum ^ data_pkt[idx]; idx = idx + 1;

        // Odd bytes (1 byte for 512 % 7 = 1)
        if (numodds > 0) begin
            oddmsb = 8'h80;
            for (i = 0; i < numodds; i = i + 1)
                oddmsb = oddmsb | ((wb_data[i] >> (1+i)) & (8'h80 >> (1+i)));
            data_pkt[idx] = oddmsb; idx = idx + 1;
            for (i = 0; i < numodds; i = i + 1) begin
                data_pkt[idx] = wb_data[i] | 8'h80;
                idx = idx + 1;
            end
        end

        // Groups of 7 (73 groups)
        for (i = 0; i < numgrps; i = i + 1) begin : data_grp_loop
            base = numodds + i * 7;
            grpmsb = 8'h80;
            for (j = 0; j < 7; j = j + 1)
                grpmsb = grpmsb | ((wb_data[base + j] >> (1+j)) & (8'h80 >> (1+j)));
            data_pkt[idx] = grpmsb; idx = idx + 1;
            for (j = 0; j < 7; j = j + 1) begin
                data_pkt[idx] = wb_data[base + j] | 8'h80;
                idx = idx + 1;
            end
        end

        // XOR raw payload bytes into checksum
        for (i = 0; i < 512; i = i + 1)
            checksum = checksum ^ wb_data[i];

        // Checksum
        data_pkt[idx] = checksum | 8'hAA;        idx = idx + 1;
        data_pkt[idx] = (checksum >> 1) | 8'hAA; idx = idx + 1;
        // PEND
        data_pkt[idx] = 8'hC8; idx = idx + 1;

        data_pkt_len = idx;
    end
    endtask

    // =========================================================================
    // Send DATA packet via IWM (WRITEBLOCK phase 2)
    // After command ACK, the host re-enters write mode and sends 512 bytes.
    // =========================================================================
    task liron_send_data_packet;
        input [6:0] dest;
        output      error;
    begin : liron_data_send
        reg [7:0] rdata;
        integer   i, timeout, ack_ok;

        error = 0;
        build_data_packet(dest);

        // 1. Bus enable (already enabled from command)
        sp_bus_enable;

        // 2. Mode register (already set)
        // 3. Motor ON (already on)

        // 4. Sense poll — device should be in WB_WAIT_WR with sense HIGH
        //    Must read Q6H first to set Q6=1, so Q7L reads status register
        //    (sense in bit 7). Without Q6H, Q7=0 Q6=0 reads data register.
        iwm_touch(IWM_Q6H);
        ack_ok = 0;
        for (timeout = 0; timeout < 50; timeout = timeout + 1) begin
            iwm_bus_read(IWM_Q7L, rdata);
            if (rdata[7]) begin
                ack_ok = 1;
                timeout = 50;
            end
        end
        if (!ack_ok) begin
            $display("  ERROR: DATA sense poll timeout (sense=%b state=%0d)",
                     dut.u_sp_device.sense, dut.u_sp_device.state);
            error = 1;
            disable liron_data_send;
        end

        // 5. Enter write mode (no REQ for DATA phase — device detects
        //    wrdata activity in ST_WB_WAIT_WR)
        iwm_bus_write(IWM_Q7H, 8'hFF);

        // 7. Write sync + packet
        iwm_write_byte(8'h3F);
        iwm_write_byte(8'hCF);
        iwm_write_byte(8'hF3);
        iwm_write_byte(8'hFC);
        iwm_write_byte(8'hFF);
        iwm_write_byte(8'hC3);

        for (i = 0; i < data_pkt_len; i = i + 1)
            iwm_write_byte(data_pkt[i]);

        // 8. Wait for underrun
        iwm_wait_underrun;

        // No ACK for DATA phase — the Liron ROM does not poll for ACK
        // after the DATA packet. It goes directly to reading the STATUS
        // response. The device receives the DATA silently.

        // 9. Cleanup: exit write mode, deassert REQ
        iwm_touch(IWM_PH0_OFF);
        iwm_touch(IWM_Q6L);
    end
    endtask

    // =========================================================================
    // Liron ROM Command Sequence — Full Bus-Level Replay
    // =========================================================================
    // Reproduces the exact register access sequence from liron-if.asm
    // =========================================================================

    // Send a SmartPort command packet via IWM and get ACK.
    // Reproduces exact Liron ROM register access sequence.
    task liron_send_and_ack;
        input [6:0] dest;
        input [7:0] cmd_code;
        input [7:0] param_count;
        input [7:0] unit_num;
        input [7:0] status_or_block_lo;
        input [7:0] block_mid;
        input [7:0] block_hi;
        output      error;
    begin : liron_full_send
        reg [7:0] rdata;
        integer   i, timeout;
        integer   ack_ok;

        error = 0;

        // Build the command packet
        build_command_packet(dest, cmd_code, param_count, unit_num,
                            status_or_block_lo, block_mid, block_hi);

        // 1. Bus enable
        sp_bus_enable;

        // 2. Write mode register = $07
        iwm_write_mode_reg(8'h07);

        // 3. Select drive 2, motor ON
        iwm_touch(IWM_DRIVE_2);
        iwm_touch(IWM_MOTOR_ON);

        // 4. Sense poll (50 iterations)
        ack_ok = 0;
        for (timeout = 0; timeout < 50; timeout = timeout + 1) begin
            iwm_bus_read(IWM_Q7L, rdata);
            if (rdata[7]) begin
                ack_ok = 1;
                timeout = 50; // break
            end
        end
        if (!ack_ok) begin
            $display("  ERROR: sense poll timeout — no device");
            error = 1;
            disable liron_full_send;
        end

        // 5. Assert REQ (PH0 ON)
        iwm_touch(IWM_PH0_ON);

        // 6. Enter write mode: STA $FF to Q7H
        iwm_bus_write(IWM_Q7H, 8'hFF);

        // 7. Write sync bytes: $3F $CF $F3 $FC $FF $C3
        iwm_write_byte(8'h3F);
        iwm_write_byte(8'hCF);
        iwm_write_byte(8'hF3);
        iwm_write_byte(8'hFC);
        iwm_write_byte(8'hFF);
        iwm_write_byte(8'hC3);

        // 8. Write packet bytes (header + data + checksum + PEND)
        for (i = 0; i < cmd_pkt_len; i = i + 1)
            iwm_write_byte(cmd_pkt[i]);

        // 9. Wait for underrun (Liron $C92C)
        iwm_wait_underrun;

        // 10. Switch to status read (STA Q6H to prepare for status reads)
        iwm_touch(IWM_Q6H);

        // 11. ACK poll (10 iterations, Liron $C938-$C943)
        // Sense must go LOW = device ACKed
        ack_ok = 0;
        for (timeout = 0; timeout < 200; timeout = timeout + 1) begin
            iwm_bus_read(IWM_Q7L, rdata);
            if (!rdata[7]) begin
                ack_ok = 1;
                timeout = 200; // break
            end
        end

        // 12. Deassert REQ (Liron $C949)
        iwm_touch(IWM_PH0_OFF);
        iwm_touch(IWM_Q6L);     // cleanup read

        if (!ack_ok) begin
            $display("  ERROR: ACK timeout — sense never went LOW");
            error = 1;
        end
    end
    endtask

    // =========================================================================
    // Liron ROM Response Read — Full Bus-Level Replay
    // =========================================================================

    // Response capture buffer
    reg [7:0] resp_bytes [0:700];
    integer   resp_len;
    integer   resp_error;

    task liron_read_response;
    begin : liron_read
        reg [7:0] rdata;
        integer   timeout, ack_ok;
        reg       valid;

        resp_len = 0;
        resp_error = 0;

        // --- Liron ROM: smartport_bus_read_packet ---

        // 1. Bus enable (PH1+PH3 already on, but no harm)
        sp_bus_enable;

        // 2. Q6H read = FLUSH read shift register
        iwm_touch(IWM_Q6H);

        // 3. Wait for SENSE HIGH (device ready, Liron $C97D)
        ack_ok = 0;
        for (timeout = 0; timeout < 200; timeout = timeout + 1) begin
            iwm_bus_read(IWM_Q7L, rdata);
            if (rdata[7]) begin
                ack_ok = 1;
                timeout = 200;
            end
        end
        if (!ack_ok) begin
            $display("  ERROR: response sense poll timeout");
            resp_error = 1;
            disable liron_read;
        end

        // 4. Assert REQ (PH0 ON)
        iwm_touch(IWM_PH0_ON);

        // 5. Sync hunt: poll Q6L for valid bytes, look for $C3
        //    Liron: 30 valid nibbles max.
        //    For READBLOCK the device needs time (SDRAM load + encode + FM TX
        //    preamble). Use generous per-byte poll limit.
        ack_ok = 0;
        for (timeout = 0; timeout < 60; timeout = timeout + 1) begin
            iwm_read_data_byte(rdata, valid, 50000);
            if (!valid) begin
                $display("  ERROR: sync hunt — data read timeout (poll exhausted)");
                $display("         sp_device state = %0d", dut.u_sp_device.state);
                resp_error = 1;
                disable liron_read;
            end
            if (rdata == 8'hC3) begin
                ack_ok = 1;
                timeout = 60;
            end
        end
        if (!ack_ok) begin
            $display("  ERROR: sync hunt — $C3 not found in 60 reads");
            resp_error = 1;
            disable liron_read;
        end

        // 6. Read response bytes: first read 7 header bytes, then compute
        //    expected remaining count from ODDCNT/GRP7CNT, then read all
        //    data + checksum + PEND. This avoids false termination on $C8
        //    appearing as a data byte in group-of-7 encoded payload.
        begin : read_response_bytes
            integer hdr_i, oddcnt_val, grpcnt_val, expected_total;
            // Read 7 header bytes
            for (hdr_i = 0; hdr_i < 7; hdr_i = hdr_i + 1) begin
                iwm_read_data_byte(rdata, valid, 5000);
                if (!valid) begin
                    $display("  ERROR: header byte %0d read timeout", hdr_i);
                    resp_error = 1;
                    disable liron_read;
                end
                resp_bytes[resp_len] = rdata;
                resp_len = resp_len + 1;
            end
            // Compute expected remaining bytes from ODDCNT/GRP7CNT
            oddcnt_val = resp_bytes[5] & 8'h7F;
            grpcnt_val = resp_bytes[6] & 8'h7F;
            // Remaining: odd section (1 MSB + oddcnt data if oddcnt>0)
            //          + groups (1 MSB + 7 data each)
            //          + 2 checksum + 1 PEND
            expected_total = (oddcnt_val > 0 ? 1 + oddcnt_val : 0)
                           + grpcnt_val * 8
                           + 3;   // 2 checksum + 1 PEND
            for (hdr_i = 0; hdr_i < expected_total; hdr_i = hdr_i + 1) begin
                iwm_read_data_byte(rdata, valid, 5000);
                if (!valid) begin
                    $display("  ERROR: data byte %0d/%0d read timeout", hdr_i, expected_total);
                    resp_error = 1;
                    disable liron_read;
                end
                resp_bytes[resp_len] = rdata;
                resp_len = resp_len + 1;
            end
        end

        // 7. Post-TX: wait for SENSE LOW (device done, Liron $Lcne8)
        iwm_touch(IWM_Q6H);  // Switch to status read
        for (timeout = 0; timeout < 200; timeout = timeout + 1) begin
            iwm_bus_read(IWM_Q7L, rdata);
            if (!rdata[7]) timeout = 200;
        end

        // 8. Deassert REQ
        iwm_touch(IWM_PH0_OFF);
    end
    endtask

    // =========================================================================
    // Response Verification Helpers
    // =========================================================================

    task verify_response_header;
        input [6:0] exp_dest;
        input [6:0] exp_source;
        input [6:0] exp_ptype;
        input [6:0] exp_stat;
    begin : verify_hdr_blk
        if (resp_len < 7) begin
            $display("  ERROR: response too short (%0d bytes, need >= 7)", resp_len);
            disable verify_hdr_blk;
        end
        check("DEST byte",    (resp_bytes[0] & 8'h7F) == {1'b0, exp_dest});
        check("SOURCE byte",  (resp_bytes[1] & 8'h7F) == {1'b0, exp_source});
        check("PTYPE byte",   (resp_bytes[2] & 8'h7F) == {1'b0, exp_ptype});
        // AUX is always $80 (= 0)
        check("AUX byte",     resp_bytes[3] == 8'h80);
        check("STAT byte",    (resp_bytes[4] & 8'h7F) == {1'b0, exp_stat});
    end
    endtask

    task verify_response_checksum;
    begin : verify_cksum
        reg [7:0] checksum;
        integer   oddcnt, grpcnt, i, j, idx;
        reg [7:0] oddmsb, grpmsb, decoded_byte;

        if (resp_len < 9) begin
            $display("  ERROR: response too short for checksum verify");
            disable verify_cksum;
        end

        checksum = 8'h00;

        // XOR header wire bytes (indices 0..6 = DEST through GRP7CNT)
        for (i = 0; i < 7; i = i + 1)
            checksum = checksum ^ resp_bytes[i];

        oddcnt = resp_bytes[5] & 8'h7F;   // ODDCNT
        grpcnt = resp_bytes[6] & 8'h7F;   // GRP7CNT

        idx = 7;   // points to first data byte (ODDMSB or GRPMSB)

        // Decode and XOR odd bytes
        if (oddcnt > 0) begin
            oddmsb = resp_bytes[idx]; idx = idx + 1;
            for (i = 0; i < oddcnt; i = i + 1) begin
                decoded_byte = ((oddmsb << (i+1)) & 8'h80) | (resp_bytes[idx] & 8'h7F);
                checksum = checksum ^ decoded_byte;
                idx = idx + 1;
            end
        end

        // Decode and XOR group bytes
        for (i = 0; i < grpcnt; i = i + 1) begin
            grpmsb = resp_bytes[idx]; idx = idx + 1;
            for (j = 0; j < 7; j = j + 1) begin
                decoded_byte = ((grpmsb << (j+1)) & 8'h80) | (resp_bytes[idx] & 8'h7F);
                checksum = checksum ^ decoded_byte;
                idx = idx + 1;
            end
        end

        // Reconstruct transmitted checksum (Liron ROM Lcnb8 algorithm)
        begin : cksum_reconstruct
            reg [7:0] chk_even, chk_odd, reconstructed;
            chk_even = resp_bytes[idx];
            chk_odd  = resp_bytes[idx + 1];
            // SEC; ROL chk_odd → (chk_odd << 1) | 1
            // AND chk_even
            reconstructed = ((chk_odd << 1) | 8'h01) & chk_even;
            checksum = checksum ^ reconstructed;
        end

        check("checksum valid (XOR = 0)", checksum == 8'h00);

        // Verify PEND
        check("PEND marker ($C8)", resp_bytes[resp_len - 1] == 8'hC8);
    end
    endtask

    // =========================================================================
    // Main Test Sequence
    // =========================================================================
    reg send_error;

    initial begin
        // Initialize
        addr           = 12'h000;
        Q3             = 0;
        R_nW           = 1;
        nDEVICE_SELECT = 1;
        nI_O_SELECT    = 1;
        nI_O_STROBE    = 1;
        RDY            = 1;
        DMA_OUT        = 1;
        INT_OUT        = 1;
        PHI0           = 0;
        PHI1           = 0;
        uSync          = 0;
        GPIO8          = 0;
        GPIO9          = 0;
        data_drive     = 8'h00;
        data_drive_en  = 0;

        $display("");
        $display("========================================================");
        $display("SmartPort Validation Testbench — Liron ROM Protocol Tests");
        $display("========================================================");

        // Wait for POR + SDRAM init
        repeat (80) @(posedge clk_7m);
        begin : wait_sdram
            integer t;
            for (t = 0; t < 300000; t = t + 1) begin
                @(posedge clk_25m);
                if (dut.sdram_init_done) t = 300000;
            end
        end

        // Force boot_done to skip flash copy
        force dut.boot_done = 1'b1;
        repeat (20) @(posedge clk_7m);

        // =================================================================
        // T0: IWM Bus Infrastructure
        // =================================================================
        $display("\n--- T0: IWM Bus Infrastructure ---");
        test_num = 0;

        // T0.1: Mode register write/read
        begin : t0_1
            reg [7:0] rdata;
            iwm_touch(IWM_MOTOR_OFF);
            iwm_touch(IWM_Q6H);
            iwm_bus_write(IWM_Q7H, 8'h07);  // Write mode reg
            iwm_bus_read(IWM_Q7L, rdata);    // Read status
            check("T0.1 mode register = $07", (rdata & 8'h1F) == 5'h07);
        end

        // T0.2: Phase line control
        begin : t0_2
            iwm_touch(IWM_PH1_ON);
            iwm_touch(IWM_PH3_ON);
            repeat (5) @(posedge clk_7m);
            check("T0.2a PH1+PH3 asserted", dut.u_iwm.phase == 4'b1010);

            iwm_touch(IWM_PH0_ON);
            repeat (5) @(posedge clk_7m);
            check("T0.2b PH0 (REQ) asserted", dut.u_iwm.phase == 4'b1011);

            iwm_touch(IWM_PH0_OFF);
            repeat (5) @(posedge clk_7m);
            check("T0.2c PH0 deasserted", dut.u_iwm.phase == 4'b1010);

            sp_bus_disable;
            repeat (5) @(posedge clk_7m);
            check("T0.2d all phases off", dut.u_iwm.phase == 4'b0000);
        end

        // T0.3: Sense readback
        begin : t0_3
            reg [7:0] rdata;
            // sp_device should be in IDLE with sense=HIGH
            check("T0.3a sp_device IDLE", dut.u_sp_device.state == 5'd0);
            iwm_touch(IWM_Q6H);
            iwm_bus_read(IWM_Q7L, rdata);
            check("T0.3b sense HIGH via status reg", rdata[7] == 1'b1);
        end

        $display("  T0 complete");

        // =================================================================
        // T1: INIT Command — Full Round-Trip
        // =================================================================
        $display("\n--- T1: INIT Command (Liron-accurate) ---");
        $display("  [diag] enabling IWM byte latch trace");
        test_num = 1;

        // Reset bus state
        sp_bus_disable;
        repeat (50) @(posedge clk_7m);

        // Send INIT command to device 1
        // Payload: cmd=INIT($05), params=2, unit=1, rest=0
        liron_send_and_ack(
            7'd1,          // dest
            8'h05,         // cmd = INIT
            8'h02,         // param count
            8'h01,         // unit number
            8'h00,         // status/block_lo
            8'h00,         // block_mid
            8'h00,         // block_hi
            send_error
        );

        check("T1.1 command sent + ACK received", send_error == 0);

        if (!send_error) begin
            // Read response
            liron_read_response;

            $display("  Response: %0d bytes received", resp_len);
            if (resp_len > 0) begin
                begin : t1_dump
                    integer d;
                    $write("  Wire: ");
                    for (d = 0; d < resp_len && d < 30; d = d + 1)
                        $write("%02h ", resp_bytes[d]);
                    $write("\n");
                end
            end

            check("T1.2 response received", resp_len >= 9);
            check("T1.3 no read error", resp_error == 0);

            if (resp_len >= 9 && !resp_error) begin
                // Verify header: dest=0(host), source=1, ptype=1(status), stat=0x7F
                verify_response_header(7'd0, 7'd1, 7'd1, 7'h7F);

                // Verify counts: oddcnt=0, grp7cnt=0
                check("T1.4 ODDCNT = 0", (resp_bytes[5] & 8'h7F) == 7'd0);
                check("T1.5 GRP7CNT = 0", (resp_bytes[6] & 8'h7F) == 7'd0);

                // Verify checksum
                verify_response_checksum;

                // Verify unit_id was assigned
                check("T1.6 unit_id assigned", dut.u_sp_device.unit_id == 7'd1);
            end
        end

        $display("  T1 complete");

        // =================================================================
        // T2: STATUS Command — 4-byte payload response
        // =================================================================
        $display("\n--- T2: STATUS Command (4-byte payload) ---");
        test_num = 2;

        // Bus cleanup between commands
        sp_bus_disable;
        repeat (100) @(posedge clk_7m);

        // Send STATUS command to device 1
        // Payload: cmd=STATUS($00), params=3, unit=1, buf=0, status_code=0
        liron_send_and_ack(
            7'd1,          // dest (unit_id assigned by INIT)
            8'h00,         // cmd = STATUS
            8'h03,         // param count
            8'h01,         // unit number
            8'h00,         // status_code = general status
            8'h00,         // block_mid (unused)
            8'h00,         // block_hi (unused)
            send_error
        );

        check("T2.1 STATUS sent + ACK received", send_error == 0);

        if (!send_error) begin
            liron_read_response;

            $display("  Response: %0d bytes received", resp_len);
            if (resp_len > 0) begin
                begin : t2_dump
                    integer d;
                    $write("  Wire: ");
                    for (d = 0; d < resp_len && d < 40; d = d + 1)
                        $write("%02h ", resp_bytes[d]);
                    $write("\n");
                end
            end

            check("T2.2 response received", resp_len >= 9);
            check("T2.3 no read error", resp_error == 0);

            if (resp_len >= 9 && !resp_error) begin
                // Header: dest=0(host), source=1, ptype=1(status), stat=0(OK)
                verify_response_header(7'd0, 7'd1, 7'd1, 7'd0);

                // Counts: oddcnt=4, grp7cnt=0 (4 byte payload)
                check("T2.4 ODDCNT = 4", (resp_bytes[5] & 8'h7F) == 7'd4);
                check("T2.5 GRP7CNT = 0", (resp_bytes[6] & 8'h7F) == 7'd0);

                // Decode payload: ODDMSB + 4 odd bytes
                if (resp_len >= 12) begin : t2_payload
                    reg [7:0] oddmsb;
                    reg [7:0] p0, p1, p2, p3;
                    oddmsb = resp_bytes[7];
                    p0 = ((oddmsb << 1) & 8'h80) | (resp_bytes[8]  & 8'h7F);
                    p1 = ((oddmsb << 2) & 8'h80) | (resp_bytes[9]  & 8'h7F);
                    p2 = ((oddmsb << 3) & 8'h80) | (resp_bytes[10] & 8'h7F);
                    p3 = ((oddmsb << 4) & 8'h80) | (resp_bytes[11] & 8'h7F);

                    $display("  Decoded payload: %02h %02h %02h %02h", p0, p1, p2, p3);

                    // Bug 4 fix: byte 0 should be $F8 (block device, online, R/W)
                    check("T2.6 status byte = $F8 (online)", p0 == 8'hF8);

                    // Block count: BLOCK_COUNT=280 = $0118
                    check("T2.7 block_count_lo = $18", p1 == 8'h18);
                    check("T2.8 block_count_hi = $01", p2 == 8'h01);
                    check("T2.9 reserved = $00", p3 == 8'h00);
                end

                // Verify checksum
                verify_response_checksum;
            end
        end

        $display("  T2 complete");

        // =================================================================
        // T3: READBLOCK Command — 512-byte payload response
        // =================================================================
        $display("\n--- T3: READBLOCK (512-byte payload, block 0) ---");
        test_num = 3;

        // Pre-fill SDRAM block 0 with known pattern: byte[i] = i & 0xFF
        // SDRAM word address = {1'b0, block_num[15:0], 9'd0} + word_idx*2
        // For block 0: base = 0, each word at address W stores {byte[2W+1], byte[2W]}
        begin : prefill_sdram
            integer w;
            // Arbiter reads word W at byte-addr W*2. Controller maps to
            // model index {row[2:0], col[9:0], bank[0]}. For block 0:
            // row=0, bank=0, col=W → index = W*2.
            for (w = 0; w < 256; w = w + 1) begin
                sdram_mem[w * 2][7:0]  = (w * 2) & 16'hFF;
                sdram_mem[w * 2][15:8] = (w * 2 + 1) & 16'hFF;
            end
            $display("  SDRAM pre-filled: mem[0]=%04h mem[2]=%04h mem[4]=%04h",
                     sdram_mem[0], sdram_mem[2], sdram_mem[4]);
            // Also dump block buffer contents before test (should be empty)
            $display("  block_buf before: [0]=%02h [1]=%02h [2]=%02h [3]=%02h",
                     dut.u_block_buffer.mem[0], dut.u_block_buffer.mem[1],
                     dut.u_block_buffer.mem[2], dut.u_block_buffer.mem[3]);
        end

        // Disable SDRAM refresh during block read test
        force dut.u_sdram_controller.refresh_needed = 1'b0;

        // Wait for SDRAM to settle
        sp_bus_disable;
        repeat (100) @(posedge clk_7m);

        // Monitor sp_device state transitions during READBLOCK
        $display("  [diag] sp_device state before send: %0d", dut.u_sp_device.state);

        // Send READBLOCK for block 0
        // Payload: cmd=READBLOCK($01), params=3, unit=1, buf=0, block=0
        liron_send_and_ack(
            7'd1,          // dest
            8'h01,         // cmd = READBLOCK
            8'h03,         // param count
            8'h01,         // unit number
            8'h00,         // block_lo = 0
            8'h00,         // block_mid = 0
            8'h00,         // block_hi = 0
            send_error
        );

        check("T3.1 READBLOCK sent + ACK received", send_error == 0);
        // Verify block buffer at the known failure point
        $display("  [diag] block_buf[168]=%02h [169]=%02h [170]=%02h [171]=%02h [172]=%02h",
                 dut.u_block_buffer.mem[168], dut.u_block_buffer.mem[169],
                 dut.u_block_buffer.mem[170], dut.u_block_buffer.mem[171],
                 dut.u_block_buffer.mem[172]);
        // Verify block buffer and SDRAM model at the failing boundary
        $display("  sdram_mem[72]=%04h [74]=%04h [76]=%04h",
                 sdram_mem[72], sdram_mem[74], sdram_mem[76]);
        $display("  block_buf[0]=%02h [1]=%02h [72]=%02h [73]=%02h [511]=%02h",
                 dut.u_block_buffer.mem[0], dut.u_block_buffer.mem[1],
                 dut.u_block_buffer.mem[72], dut.u_block_buffer.mem[73],
                 dut.u_block_buffer.mem[511]);

        // Dump block buffer after SDRAM transfer
        $display("  block_buf after: [0]=%02h [1]=%02h [2]=%02h [3]=%02h [4]=%02h [5]=%02h",
                 dut.u_block_buffer.mem[0], dut.u_block_buffer.mem[1],
                 dut.u_block_buffer.mem[2], dut.u_block_buffer.mem[3],
                 dut.u_block_buffer.mem[4], dut.u_block_buffer.mem[5]);
        $display("  sdram rdata_latch in arbiter: %04h", dut.u_sdram_arbiter.rd_data_latch);

        if (!send_error) begin
            liron_read_response;

            $display("  Response: %0d bytes received", resp_len);

            check("T3.2 response received", resp_len >= 9);
            check("T3.3 no read error", resp_error == 0);

            if (resp_len >= 9 && !resp_error) begin
                // Header: dest=0, source=1, ptype=2(DATA), stat=0
                verify_response_header(7'd0, 7'd1, 7'd2, 7'd0);

                // Counts: oddcnt = 512%7 = 1, grp7cnt = 512/7 = 73
                check("T3.4 ODDCNT = 1", (resp_bytes[5] & 8'h7F) == 7'd1);
                check("T3.5 GRP7CNT = 73", (resp_bytes[6] & 8'h7F) == 7'd73);

                // Decode all 512 payload bytes and verify against pattern
                begin : t3_verify
                    reg [7:0] decoded [0:511];
                    integer oddcnt, grpcnt, idx, di, gi, gj;
                    reg [7:0] msb_byte;
                    integer mismatches;

                    oddcnt = resp_bytes[5] & 8'h7F;
                    grpcnt = resp_bytes[6] & 8'h7F;
                    idx = 7;
                    di = 0;

                    // Odd bytes
                    if (oddcnt > 0) begin
                        msb_byte = resp_bytes[idx]; idx = idx + 1;
                        for (gi = 0; gi < oddcnt; gi = gi + 1) begin
                            decoded[di] = ((msb_byte << (gi+1)) & 8'h80) |
                                          (resp_bytes[idx] & 8'h7F);
                            di = di + 1;
                            idx = idx + 1;
                        end
                    end

                    // Groups
                    for (gi = 0; gi < grpcnt; gi = gi + 1) begin
                        msb_byte = resp_bytes[idx]; idx = idx + 1;
                        for (gj = 0; gj < 7; gj = gj + 1) begin
                            decoded[di] = ((msb_byte << (gj+1)) & 8'h80) |
                                          (resp_bytes[idx] & 8'h7F);
                            di = di + 1;
                            idx = idx + 1;
                        end
                    end

                    check("T3.6 decoded 512 bytes", di == 512);

                    // Dump raw wire bytes around the failure point
                    // Byte 73 is in group 10 (73 = 1 odd + 10*7 + 2).
                    // Group 10 wire starts at: 7 hdr + 1+1 odd + 10*8 = 89
                    $display("  resp_bytes[87..96]: %02h %02h %02h %02h %02h %02h %02h %02h %02h %02h",
                             resp_bytes[87], resp_bytes[88], resp_bytes[89],
                             resp_bytes[90], resp_bytes[91], resp_bytes[92],
                             resp_bytes[93], resp_bytes[94], resp_bytes[95],
                             resp_bytes[96]);

                    // Verify pattern: byte[i] = i & 0xFF
                    mismatches = 0;
                    for (gi = 0; gi < 512; gi = gi + 1) begin
                        if (decoded[gi] !== (gi & 8'hFF)) begin
                            if (mismatches < 5)
                                $display("  MISMATCH: byte[%0d] = %02h, expected %02h",
                                         gi, decoded[gi], gi & 8'hFF);
                            mismatches = mismatches + 1;
                        end
                    end
                    if (mismatches > 5)
                        $display("  ... and %0d more mismatches", mismatches - 5);
                    check("T3.7 all 512 bytes match pattern", mismatches == 0);
                end

                // Verify checksum
                verify_response_checksum;
            end
        end

        $display("  T3 complete");

        // =================================================================
        // T4: WRITEBLOCK — Three-Phase Protocol
        // =================================================================
        // Phase 1: CMD packet (type $80) → ACK
        // Phase 2: DATA packet (type $82, 512 bytes) → ACK
        // Phase 3: Read STATUS response (no payload)
        // Verify: SDRAM gets correct data, STATUS response OK
        // =================================================================
        $display("\n--- T4: WRITEBLOCK (512-byte write, block 1) ---");
        test_num = 4;

        sp_bus_disable;
        repeat (100) @(posedge clk_7m);

        // Phase 1: Send WRITEBLOCK command for block 1
        liron_send_and_ack(
            7'd1, 8'h02, 8'h03, 8'h01,  // dest=1, cmd=WRITEBLOCK, params=3, unit=1
            8'h01, 8'h00, 8'h00,         // block_lo=1, mid=0, hi=0
            send_error
        );
        check("T4.1 WRITEBLOCK CMD sent + ACK", send_error == 0);
        $display("  [diag] after CMD ACK: sp_state=%0d sense=%b",
                 dut.u_sp_device.state, dut.u_sp_device.sense);

        if (!send_error) begin
            // Phase 2: Send DATA packet with pattern byte[i] = ~i
            begin : t4_fill_wb
                integer fi;
                for (fi = 0; fi < 512; fi = fi + 1)
                    wb_data[fi] = (~fi) & 8'hFF;
            end
            $display("  [diag] before DATA send: wrdata=%b sp_state=%0d q7=%b",
                     dut.u_iwm.wrdata, dut.u_sp_device.state, dut.u_iwm.q7_sync);
            liron_send_data_packet(7'd1, send_error);
            check("T4.2 DATA packet sent", send_error == 0);
            $display("  [diag] after DATA send: sp_state=%0d sense=%b",
                     dut.u_sp_device.state, dut.u_sp_device.sense);

            if (!send_error) begin
                // Phase 3: Read STATUS response
                liron_read_response;
                check("T4.3 STATUS response received", resp_len >= 9);
                check("T4.4 no read error", resp_error == 0);

                if (resp_len >= 9 && !resp_error) begin
                    verify_response_header(7'd0, 7'd1, 7'd1, 7'd0);
                    check("T4.5 ODDCNT = 0 (no payload)", (resp_bytes[5] & 8'h7F) == 7'd0);
                    verify_response_checksum;
                end

                // Verify SDRAM got the data: block 1 starts at model addr 512
                // Controller maps: block_num=1 → base_addr={1'b0, 16'd1, 9'd0}=512
                // Word W → sdram addr = 512 + W*2, model index = (256+W)*2
                $display("  SDRAM[512]=%04h [514]=%04h [516]=%04h (expect ~i pattern)",
                         sdram_mem[512], sdram_mem[514], sdram_mem[516]);
                begin : t4_verify_sdram
                    integer vi;
                    integer sdram_mismatches;
                    reg [15:0] expected_word;
                    sdram_mismatches = 0;
                    for (vi = 0; vi < 256; vi = vi + 1) begin
                        expected_word[7:0]  = (~(vi * 2)) & 8'hFF;
                        expected_word[15:8] = (~(vi * 2 + 1)) & 8'hFF;
                        if (sdram_mem[(256 + vi) * 2] !== expected_word) begin
                            if (sdram_mismatches < 3)
                                $display("  SDRAM MISMATCH: word[%0d] at addr %0d = %04h, expected %04h",
                                         vi, (256 + vi) * 2, sdram_mem[(256 + vi) * 2], expected_word);
                            sdram_mismatches = sdram_mismatches + 1;
                        end
                    end
                    check("T4.6 SDRAM block 1 data correct", sdram_mismatches == 0);
                end
            end
        end

        $display("  T4 complete");

        // =================================================================
        // T5: Sense/ACK Timing
        // =================================================================
        $display("\n--- T5: Sense/ACK Timing ---");
        test_num = 5;

        sp_bus_disable;
        repeat (100) @(posedge clk_7m);

        // T5.1: Measure ACK latency (sense HIGH→LOW after command)
        begin : t5_timing
            reg [7:0] rdata;
            integer ack_ok, timeout;
            integer t_start, t_ack;

            // Send STATUS command and measure time to ACK
            build_command_packet(7'd1, 8'h00, 8'h03, 8'h01, 8'h00, 8'h00, 8'h00);
            sp_bus_enable;
            iwm_write_mode_reg(8'h07);
            iwm_touch(IWM_DRIVE_2);
            iwm_touch(IWM_MOTOR_ON);

            // Sense poll
            ack_ok = 0;
            for (timeout = 0; timeout < 50; timeout = timeout + 1) begin
                iwm_bus_read(IWM_Q7L, rdata);
                if (rdata[7]) begin ack_ok = 1; timeout = 50; end
            end

            if (ack_ok) begin
                iwm_touch(IWM_PH0_ON);
                iwm_bus_write(IWM_Q7H, 8'hFF);
                // Send sync + packet
                iwm_write_byte(8'h3F); iwm_write_byte(8'hCF);
                iwm_write_byte(8'hF3); iwm_write_byte(8'hFC);
                iwm_write_byte(8'hFF); iwm_write_byte(8'hC3);
                begin : t5_send_pkt
                    integer si;
                    for (si = 0; si < cmd_pkt_len; si = si + 1)
                        iwm_write_byte(cmd_pkt[si]);
                end
                iwm_wait_underrun;
                iwm_touch(IWM_Q6H);

                // Record time and poll for ACK
                t_start = $time;
                ack_ok = 0;
                for (timeout = 0; timeout < 200; timeout = timeout + 1) begin
                    iwm_bus_read(IWM_Q7L, rdata);
                    if (!rdata[7]) begin
                        t_ack = $time;
                        ack_ok = 1;
                        timeout = 200;
                    end
                end

                iwm_touch(IWM_PH0_OFF);
                iwm_touch(IWM_Q6L);

                if (ack_ok) begin
                    $display("  ACK latency: %0d ns (~%0d us)", t_ack - t_start, (t_ack - t_start) / 1000);
                    // Liron window: ~108us (772 fclk @ 7.16MHz = 107,800ns)
                    check("T5.1 ACK within 108us", (t_ack - t_start) < 108000000);
                end else begin
                    check("T5.1 ACK within 108us", 0);
                end

                // Read and discard the response
                liron_read_response;
            end
        end

        // T5.2: Verify sense starts HIGH in IDLE
        sp_bus_disable;
        repeat (200) @(posedge clk_7m);
        check("T5.2 sense HIGH in IDLE", dut.u_sp_device.sense == 1'b1);

        $display("  T5 complete");

        // =================================================================
        // T8: Error Conditions
        // =================================================================
        $display("\n--- T8: Error Conditions ---");
        test_num = 8;

        // T8.1: Command not for us (dest=2, but unit_id=1)
        sp_bus_disable;
        repeat (500) @(posedge clk_7m);  // generous settle time
        $display("  [diag] sp_device state=%0d sense=%b unit_id=%0d before T8",
                 dut.u_sp_device.state, dut.u_sp_device.sense, dut.u_sp_device.unit_id);

        begin : t8_wrong_dest
            reg [7:0] rdata;
            integer ack_ok, timeout;

            build_command_packet(7'd2, 8'h00, 8'h03, 8'h02, 8'h00, 8'h00, 8'h00);
            sp_bus_enable;
            iwm_write_mode_reg(8'h07);
            iwm_touch(IWM_DRIVE_2);
            iwm_touch(IWM_MOTOR_ON);

            ack_ok = 0;
            for (timeout = 0; timeout < 50; timeout = timeout + 1) begin
                iwm_bus_read(IWM_Q7L, rdata);
                if (rdata[7]) begin ack_ok = 1; timeout = 50; end
            end

            if (ack_ok) begin
                iwm_touch(IWM_PH0_ON);
                iwm_bus_write(IWM_Q7H, 8'hFF);
                iwm_write_byte(8'h3F); iwm_write_byte(8'hCF);
                iwm_write_byte(8'hF3); iwm_write_byte(8'hFC);
                iwm_write_byte(8'hFF); iwm_write_byte(8'hC3);
                begin : t8_send_wrong
                    integer si;
                    for (si = 0; si < cmd_pkt_len; si = si + 1)
                        iwm_write_byte(cmd_pkt[si]);
                end
                iwm_wait_underrun;
                iwm_touch(IWM_Q6H);

                // ACK poll — should NOT get ACK (device ignores wrong dest)
                ack_ok = 0;
                for (timeout = 0; timeout < 30; timeout = timeout + 1) begin
                    iwm_bus_read(IWM_Q7L, rdata);
                    if (!rdata[7]) begin ack_ok = 1; timeout = 30; end
                end

                iwm_touch(IWM_PH0_OFF);
                iwm_touch(IWM_Q6L);

                check("T8.1 no ACK for wrong dest (device ignores)", ack_ok == 0);
            end
        end

        // T8.2: Device returns to IDLE after wrong dest
        // The device enters RX path, decodes the command, finds dest doesn't
        // match, and returns to IDLE. The full decode takes ~10000+ fclk.
        repeat (20000) @(posedge clk_7m);
        check("T8.2 device back to IDLE", dut.u_sp_device.state == 5'd0);
        check("T8.3 sense still HIGH", dut.u_sp_device.sense == 1'b1);

        $display("  T8 complete");

        // =================================================================
        // T10: Multi-Command Sequences
        // =================================================================
        $display("\n--- T10: Multi-Command Sequence ---");
        test_num = 10;

        sp_bus_disable;
        repeat (100) @(posedge clk_7m);

        // T10.1: STATUS after previous tests — verify clean state re-entry
        liron_send_and_ack(
            7'd1, 8'h00, 8'h03, 8'h01, 8'h00, 8'h00, 8'h00,
            send_error
        );
        check("T10.1 STATUS cmd OK after prior tests", send_error == 0);
        if (!send_error) begin
            liron_read_response;
            check("T10.1 response OK", resp_len >= 9 && !resp_error);
            if (resp_len >= 9) begin
                verify_response_header(7'd0, 7'd1, 7'd1, 7'd0);
                verify_response_checksum;
            end
        end

        // T10.2: READBLOCK of block 1 — should return the data written by T4
        sp_bus_disable;
        repeat (100) @(posedge clk_7m);

        liron_send_and_ack(
            7'd1, 8'h01, 8'h03, 8'h01, 8'h01, 8'h00, 8'h00,  // READBLOCK block 1
            send_error
        );
        check("T10.2 READBLOCK block 1 cmd OK", send_error == 0);

        if (!send_error) begin
            liron_read_response;
            check("T10.2 response received", resp_len >= 9 && !resp_error);

            if (resp_len >= 9 && !resp_error) begin
                verify_response_header(7'd0, 7'd1, 7'd2, 7'd0);

                // Decode and verify: should be ~i pattern from T4 WRITEBLOCK
                begin : t10_verify
                    integer oddcnt, grpcnt, idx, di, gi, gj, mismatches;
                    reg [7:0] msb_byte;
                    reg [7:0] decoded [0:511];

                    oddcnt = resp_bytes[5] & 8'h7F;
                    grpcnt = resp_bytes[6] & 8'h7F;
                    idx = 7;
                    di = 0;

                    if (oddcnt > 0) begin
                        msb_byte = resp_bytes[idx]; idx = idx + 1;
                        for (gi = 0; gi < oddcnt; gi = gi + 1) begin
                            decoded[di] = ((msb_byte << (gi+1)) & 8'h80) |
                                          (resp_bytes[idx] & 8'h7F);
                            di = di + 1; idx = idx + 1;
                        end
                    end
                    for (gi = 0; gi < grpcnt; gi = gi + 1) begin
                        msb_byte = resp_bytes[idx]; idx = idx + 1;
                        for (gj = 0; gj < 7; gj = gj + 1) begin
                            decoded[di] = ((msb_byte << (gj+1)) & 8'h80) |
                                          (resp_bytes[idx] & 8'h7F);
                            di = di + 1; idx = idx + 1;
                        end
                    end

                    mismatches = 0;
                    for (gi = 0; gi < 512; gi = gi + 1) begin
                        if (decoded[gi] !== ((~gi) & 8'hFF)) begin
                            if (mismatches < 3)
                                $display("  MISMATCH: byte[%0d] = %02h, expected %02h",
                                         gi, decoded[gi], (~gi) & 8'hFF);
                            mismatches = mismatches + 1;
                        end
                    end
                    check("T10.3 READBLOCK block 1 matches T4 write data", mismatches == 0);
                end

                verify_response_checksum;
            end
        end

        // T10.3: Another STATUS to confirm clean state
        sp_bus_disable;
        repeat (100) @(posedge clk_7m);

        liron_send_and_ack(
            7'd1, 8'h00, 8'h03, 8'h01, 8'h00, 8'h00, 8'h00,
            send_error
        );
        check("T10.4 final STATUS OK", send_error == 0);
        if (!send_error) begin
            liron_read_response;
            check("T10.4 response OK", resp_len >= 9 && !resp_error);
            if (resp_len >= 9)
                verify_response_checksum;
        end

        $display("  T10 complete");

        // =================================================================
        // Summary
        // =================================================================
        $display("");
        $display("========================================================");
        $display("Validation Testbench Results:");
        $display("  %0d PASSED, %0d FAILED", pass_count, fail_count);
        $display("========================================================");
        if (fail_count == 0)
            $display("RESULT: ALL TESTS PASSED");
        else
            $display("RESULT: FAIL");
        $display("");

        #5000;
        $finish;
    end

    // =========================================================================
    // Diagnostic: block buffer port A write trace (during T3)
    // =========================================================================
    always @(posedge clk_25m) begin
        if (test_num >= 3 && dut.u_block_buffer.we_a) begin
            if (dut.u_block_buffer.addr_a >= 9'd70 && dut.u_block_buffer.addr_a <= 9'd77)
                $display("  [BUF_A] t=%0t write addr=%0d data=%02h",
                         $time, dut.u_block_buffer.addr_a, dut.u_block_buffer.wdata_a);
        end
    end
    // Monitor port B writes during T3
    always @(posedge clk_7m) begin
        if (test_num >= 3 && dut.u_block_buffer.we_b) begin
            if (dut.u_block_buffer.addr_b >= 9'd70 && dut.u_block_buffer.addr_b <= 9'd77)
                $display("  [BUF_B] t=%0t WRITE addr=%0d data=%02h *** UNEXPECTED ***",
                         $time, dut.u_block_buffer.addr_b, dut.u_block_buffer.wdata_b);
        end
    end

    // =========================================================================
    // Diagnostic: sp_device state transition trace (during T3)
    // =========================================================================
    reg [4:0] prev_sp_state = 5'd0;
    always @(posedge clk_7m) begin
        if (dut.u_sp_device.state !== prev_sp_state && test_num >= 3) begin
            $display("  [sp] t=%0t state %0d -> %0d  (block_ready=%b block_read_req=%b block_num=%0d)",
                     $time, prev_sp_state, dut.u_sp_device.state,
                     dut.u_sp_device.block_ready,
                     dut.u_sp_device.block_read_req,
                     dut.u_sp_device.block_num);
        end
        prev_sp_state <= dut.u_sp_device.state;
    end

    // Trace arbiter state during T3
    reg [3:0] prev_arb_state = 4'd0;
    reg [3:0] prev_sdram_state = 4'd0;
    always @(posedge clk_25m) begin
        if (dut.u_sdram_arbiter.state !== prev_arb_state && test_num >= 3) begin
            $display("  [arb] t=%0t state %0d -> %0d  (rdata_valid=%b sdram_ready=%b xfer=%0d)",
                     $time, prev_arb_state, dut.u_sdram_arbiter.state,
                     dut.u_sdram_controller.req_rdata_valid,
                     dut.u_sdram_controller.req_ready,
                     dut.u_sdram_arbiter.xfer_cnt);
        end
        prev_arb_state <= dut.u_sdram_arbiter.state;
        // Also trace SDRAM controller state changes
        if (dut.u_sdram_controller.state !== prev_sdram_state && test_num >= 3 &&
            dut.u_sdram_arbiter.state >= 4'd1) begin
            $display("  [sdram] t=%0t state %0d -> %0d  (delay=%0d rdata_valid=%b)",
                     $time, prev_sdram_state, dut.u_sdram_controller.state,
                     dut.u_sdram_controller.delay_cnt,
                     dut.u_sdram_controller.req_rdata_valid);
        end
        prev_sdram_state <= dut.u_sdram_controller.state;
    end

    // =========================================================================
    // Diagnostic: trace IWM byte latches (x7 rising edge)
    // Shows every byte the IWM produces from rddata during read mode
    // =========================================================================
    reg prev_x7 = 0;
    always @(posedge clk_7m) begin
        prev_x7 <= dut.u_iwm.x7;
        if (dut.u_iwm.x7 && !prev_x7 && test_num >= 1) begin
            $display("  [IWM] t=%0t x7 RISE: buffer=%02h (latchSynced=%b bitCnt=%0d)",
                     $time, dut.u_iwm.buffer, dut.u_iwm.latchSynced, dut.u_iwm.bitCounter);
        end
    end

    // =========================================================================
    // Watchdog
    // =========================================================================
    initial begin
        #2000000000;   // 2 seconds (T4 WRITEBLOCK + T10 multi-cmd)
        $display("WATCHDOG: Simulation timeout");
        $display("  %0d passed, %0d failed (incomplete)", pass_count, fail_count);
        $display("RESULT: FAIL (timeout)");
        $finish;
    end

endmodule
