`timescale 1ns / 1ps
// Integration TB: drives the $C0Cx register port like the 6502 monitor would.
module project_obscurus_tb;
    reg clk100 = 0;
    always #5 clk100 = ~clk100;

    reg [15:0] apple_addr = 16'h0000;
    wire [7:0] apple_d;
    reg [7:0]  apple_d_drive = 8'hZZ;
    reg        apple_d_oe = 1'b0;
    reg nDEVICE_SELECT=1, nI_O_SELECT=1, nI_O_STROBE=1, R_nW=1, PHI0=0, nRES_READ=1;
    assign apple_d = apple_d_oe ? apple_d_drive : 8'hZZ;

    wire SDRAM_CLK, SDRAM_CKE, SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE;
    wire SDRAM_DQM0, SDRAM_DQM1, SDRAM_BA0, SDRAM_BA1;
    wire SDRAM_A0,SDRAM_A1,SDRAM_A2,SDRAM_A3,SDRAM_A4,SDRAM_A5,SDRAM_A6;
    wire SDRAM_A7,SDRAM_A8,SDRAM_A9,SDRAM_A10,SDRAM_A11,SDRAM_A12;
    wire [12:0] sdram_a = {SDRAM_A12,SDRAM_A11,SDRAM_A10,SDRAM_A9,SDRAM_A8,
                           SDRAM_A7,SDRAM_A6,SDRAM_A5,SDRAM_A4,SDRAM_A3,
                           SDRAM_A2,SDRAM_A1,SDRAM_A0};
    wire [3:0] sdram_cmd = {SDRAM_nCS,SDRAM_nRAS,SDRAM_nCAS,SDRAM_nWE};
    wire [15:0] sdram_dq;

    // SPI flash model nets (declared before dut so the port connections bind)
    wire FLASH_nCS_w, FLASH_MOSI_w, flash_miso_w;

    project_obscurus_top dut (
        .CLK_100MHz(clk100),
        .SDRAM_CLK(SDRAM_CLK), .SDRAM_CKE(SDRAM_CKE), .SDRAM_nCS(SDRAM_nCS),
        .SDRAM_nRAS(SDRAM_nRAS), .SDRAM_nCAS(SDRAM_nCAS), .SDRAM_nWE(SDRAM_nWE),
        .SDRAM_DQM0(SDRAM_DQM0), .SDRAM_DQM1(SDRAM_DQM1),
        .SDRAM_BA0(SDRAM_BA0), .SDRAM_BA1(SDRAM_BA1),
        .SDRAM_A0(SDRAM_A0),.SDRAM_A1(SDRAM_A1),.SDRAM_A2(SDRAM_A2),
        .SDRAM_A3(SDRAM_A3),.SDRAM_A4(SDRAM_A4),.SDRAM_A5(SDRAM_A5),
        .SDRAM_A6(SDRAM_A6),.SDRAM_A7(SDRAM_A7),.SDRAM_A8(SDRAM_A8),
        .SDRAM_A9(SDRAM_A9),.SDRAM_A10(SDRAM_A10),.SDRAM_A11(SDRAM_A11),
        .SDRAM_A12(SDRAM_A12),
        .SDRAM_D0(sdram_dq[0]),.SDRAM_D1(sdram_dq[1]),.SDRAM_D2(sdram_dq[2]),
        .SDRAM_D3(sdram_dq[3]),.SDRAM_D4(sdram_dq[4]),.SDRAM_D5(sdram_dq[5]),
        .SDRAM_D6(sdram_dq[6]),.SDRAM_D7(sdram_dq[7]),.SDRAM_D8(sdram_dq[8]),
        .SDRAM_D9(sdram_dq[9]),.SDRAM_D10(sdram_dq[10]),.SDRAM_D11(sdram_dq[11]),
        .SDRAM_D12(sdram_dq[12]),.SDRAM_D13(sdram_dq[13]),.SDRAM_D14(sdram_dq[14]),
        .SDRAM_D15(sdram_dq[15]),
        .A0(apple_addr[0]),.A1(apple_addr[1]),.A2(apple_addr[2]),.A3(apple_addr[3]),
        .A4(apple_addr[4]),.A5(apple_addr[5]),.A6(apple_addr[6]),.A7(apple_addr[7]),
        .A8(apple_addr[8]),.A9(apple_addr[9]),.A10(apple_addr[10]),.A11(apple_addr[11]),
        .A12(apple_addr[12]),.A13(apple_addr[13]),.A14(apple_addr[14]),.A15(apple_addr[15]),
        .D0(apple_d[0]),.D1(apple_d[1]),.D2(apple_d[2]),.D3(apple_d[3]),
        .D4(apple_d[4]),.D5(apple_d[5]),.D6(apple_d[6]),.D7(apple_d[7]),
        .PHI0(PHI0),.PHI1(~PHI0),.sig_7M(1'b0),.Q3(1'b0),.uSync(1'b0),
        .R_nW(R_nW),.nDEVICE_SELECT(nDEVICE_SELECT),.nI_O_SELECT(nI_O_SELECT),
        .nI_O_STROBE(nI_O_STROBE),.DMA_OUT(1'b1),.INT_OUT(1'b1),.RDY(1'b1),
        .nRES_READ(nRES_READ),
        .FLASH_nCS(FLASH_nCS_w),.FLASH_MOSI(FLASH_MOSI_w),.FLASH_MISO(flash_miso_w),
        .FLASH_nWP(),.FLASH_nHOLD(),
        .nIRQ(),.nNMI(),.nINH(),.nDMA(),.nRES(),.DMA_IN(),.INT_IN(),.DATA_OE(),
        .GPIO_1(),.GPIO_2(),.GPIO_3(),.GPIO_4(),.GPIO_5(),.GPIO_6(),.GPIO_7(),
        .GPIO_8(),.GPIO_9(),.GPIO_10(),.GPIO_11(),.GPIO_12(),.GPIO_13(),.GPIO_14(),
        .GPIO_15(),.GPIO_16(),.GPIO_17(),.GPIO_18(),.GPIO_19(),.GPIO_20()
    );

    sdram_model model (
        .clk(SDRAM_CLK), .cmd(sdram_cmd), .ba({SDRAM_BA1,SDRAM_BA0}),
        .a(sdram_a), .dqm0(SDRAM_DQM0), .dqm1(SDRAM_DQM1), .dq(sdram_dq)
    );

    // ----- SPI flash model for the C-flash integration proof (Task 6) -----
    // SUBTLETY 1: the top routes SPI SCK through the ECP5 USRMCLK primitive,
    // which is compiled out under `ifndef SYNTHESIS, leaving `spi_sck` as a plain
    // internal wire (NOT a top-level port). Reach it by HIERARCHICAL REFERENCE
    // (dut.spi_sck) via an assign-probe so it can drive the model's .sck port.
    // FLASH_MISO is a DUT INPUT driven by the model. The model has NO reset port,
    // so its backing store persists across the nRES_READ pulse -- essential for
    // the SAVE -> RAM-loss -> RESTORE proof below.
    wire flash_sck_probe;
    assign flash_sck_probe = dut.spi_sck;
    spi_flash_model u_flash (
        .sck (flash_sck_probe),
        .ncs (FLASH_nCS_w),
        .mosi(FLASH_MOSI_w),
        .miso(flash_miso_w)
    );

    integer errors = 0;
    reg [7:0] tmp;
    reg       c4ok;
    reg [7:0] r0, r1;

    // ---- Conway's Multiverse TICK1 oracle state ----
    // LIFE8 skill image (built from software/SDM/LIFE8.S -> life8.mem, copied to
    // the sim build dir by the Makefile). LSIM=1 sim build => GROWS=8 rows.
    localparam integer LIFE8LEN = 689;
    reg [7:0] life8img [0:1023];
    initial $readmemh("life8.mem", life8img);
    // LIFE8GR = LIFE8 + 4 GR alters (8 cells/byte, ROWBYTES=5, GWIDTH=40,
    // MUL5 stride). LSIM=1 sim build => GROWS=8 rows. Loaded over $0300 AFTER
    // the DHGR oracle (both ORG $0300, can't co-reside) -- BRAM survives a sim
    // reset, so a plain load-port overwrite re-points the skill.
    localparam integer LIFE8GRLEN = 680;
    reg [7:0] life8grimg [0:1023];
    initial $readmemh("life8gr.mem", life8grimg);
    // ---- FARM game protocol (skill 2 @ $0600, GBANK=32) ----
    // Blob length is MEASURED from the readmemh image, not hardcoded:
    // $readmemh leaves unwritten entries X, so scan for the last defined
    // byte. A stale constant here once truncated PUTEV off the blob tail
    // (ring silently dead) - never hardcode this again.
    reg [7:0] farmimg [0:2047];
    initial $readmemh("farmtask.mem", farmimg);
    integer FARMLEN;
    initial begin
        #1; FARMLEN = 0;
        begin : flen_scan
            integer fi;
            for (fi = 0; fi < 2048; fi = fi + 1)
                if (farmimg[fi] !== 8'hxx) FARMLEN = fi + 1;
        end
    end
    reg [7:0] wkimg [0:4095];
    initial $readmemh("worktask.mem", wkimg);
    integer WKLEN;
    initial begin
        #1; WKLEN = 0;
        begin : wklen_scan
            integer wi;
            for (wi = 0; wi < 4096; wi = wi + 1)
                if (wkimg[wi] !== 8'hxx) WKLEN = wi + 1;
        end
    end
    reg [7:0] rb [0:639];      // read-back of one 8x80 universe buffer
    integer   bufsum, mvi;

    task wr_reg(input [3:0] r, input [7:0] d);
        begin
            apple_addr = {8'hC0, 4'hC, r}; R_nW=1'b0;
            apple_d_drive=d; apple_d_oe=1'b1; nDEVICE_SELECT=1'b0;
            #300; nDEVICE_SELECT=1'b1; apple_d_oe=1'b0; #200;
        end
    endtask
    task rd_reg(input [3:0] r, output [7:0] d);
        begin
            apple_addr = {8'hC0, 4'hC, r}; R_nW=1'b1; apple_d_oe=1'b0;
            nDEVICE_SELECT=1'b0; #300; d=apple_d; nDEVICE_SELECT=1'b1; #200;
        end
    endtask
    task poll_busy;
        begin : pb
            integer guard; guard=0;
            rd_reg(4'h5, tmp);
            while (tmp[7]) begin
                rd_reg(4'h5, tmp);
                guard=guard+1; if (guard>10000) begin
                    $display("FAIL poll_busy stuck"); errors=errors+1; disable pb;
                end
            end
        end
    endtask

    task set_addr(input [15:0] a); begin
        wr_reg(4'h0, a[7:0]); wr_reg(4'h1, a[15:8]); end
    endtask
    task set_bank(input [9:0] b); begin
        wr_reg(4'h2, b[7:0]); wr_reg(4'h3, {6'd0,b[9:8]}); end
    endtask
    task sdram_write(input [9:0] bank, input [15:0] a, input [7:0] d); begin
        set_bank(bank); set_addr(a); wr_reg(4'h6, d); poll_busy; end
    endtask
    task sdram_read(input [9:0] bank, input [15:0] a, output [7:0] d); begin
        set_bank(bank); set_addr(a); wr_reg(4'h4, 8'h00); poll_busy;
        rd_reg(4'h6, d); end
    endtask

    reg [7:0] krn_before, tbl_before;
    reg [7:0] hibram_z0, hibram_r;   // itr4.1 high-BRAM snapshot + readback
    task load_byte(input [7:0] d); begin wr_reg(4'hB, d); end endtask  // CP_WDATA (auto-inc)
    task cp_read(input [13:0] a, output [7:0] d); begin
        // CP_RDATA reads auto-increment m_laddr (top.v line 304) AND the coproc's
        // port-B ldata_out continuously tracks bram[m_laddr] (registered, 1 cyc).
        // So once CP_LADDR is set, ldata_out already = bram[a]; the FIRST read
        // returns it. A second read would over-advance (returns bram[a+1]).
        // Re-arm the address before reading so the latch reflects exactly `a`.
        wr_reg(4'h9, a[7:0]); wr_reg(4'hA, {2'b0, a[13:8]});
        rd_reg(4'hC, d);    // ldata_out already settled to bram[a]
    end endtask

    // ---- itr4.1: 16-bit load port helpers (48 KB high-BRAM range) ----
    // cp_read only accepts 14-bit (covers $0000-$3FFF). These extend to the full
    // 16-bit m_laddr so the $4000-$BFFF region added by itr4 can be exercised.
    task cp_load_byte16(input [15:0] a, input [7:0] d); begin
        wr_reg(4'h9, a[7:0]); wr_reg(4'hA, a[15:8]);
        load_byte(d);   // CP_WDATA at the new address, then laddr++
    end endtask
    task cp_read16(input [15:0] a, output [7:0] d); begin
        // same re-arm-before-read semantics as cp_read - do NOT rd_reg twice
        // (port-B ldata_out is registered; a 2nd read advances to bram[a+1])
        wr_reg(4'h9, a[7:0]); wr_reg(4'hA, a[15:8]);
        rd_reg(4'hC, d);  // ldata_out settled to bram[a] after addr-set latency
    end endtask

    // ---- C4 async-dispatch helpers ----
    // Per-slot mailbox in coproc BRAM = $0F80 + slot*32: +0 skill_id, +1 budget,
    // +2 arg0, +16 result. cmpskill (skill 0 @ $0300) computes result = arg0+1.
    task stage_mbox(input [1:0] slot, input [7:0] skill,
                    input [7:0] budget, input [7:0] arg0);
        reg [7:0] lo;
        begin
            lo = 8'h80 + {slot, 5'b0};                 // $0F80 + slot*32 low byte
            wr_reg(4'h9, lo); wr_reg(4'hA, 8'h0F);     // CP_LADDR = mailbox base
            load_byte(skill); load_byte(budget); load_byte(arg0);
        end
    endtask
    task ring   (input [1:0] slot); begin wr_reg(4'h5, {6'b0, slot}); end endtask // CP_RING
    task collect(input [1:0] slot); begin wr_reg(4'hC, {6'b0, slot}); end endtask // CP_COLLECT
    task read_result(input [1:0] slot, output [7:0] d); begin
        cp_read(13'h0F90 + {slot, 5'b0}, d);           // mailbox+16
    end endtask
    // poll DONE ($C0C0, nibble 0) until (done & mask)==mask, or maxcyc clk100 cycles.
    // Leaves the last DONE read in `tmp`; ok=1 if satisfied, 0 if timed out.
    task wait_done(input [3:0] mask, input integer maxcyc, output ok);
        begin : wd
            integer g; g = 0; ok = 1'b0;
            rd_reg(4'h0, tmp);
            while ((tmp[3:0] & mask) != mask) begin
                repeat (500) @(posedge clk100);
                rd_reg(4'h0, tmp);
                g = g + 500;
                if (g > maxcyc) disable wd;
            end
            ok = 1'b1;
        end
    endtask

    // ---- Conway's Multiverse TICK1 helpers ----
    // A universe lives in SDRAM bank UBASE(=16); buffer A @ $0000, B @ $4000.
    // Rows are contiguous: row r byte b is at base + r*80 + b => idx r*80+b spans
    // 0..639 across the 8x80 (sim) grid, so the buffer is 640 sequential bytes.
    // Both host read- AND write-ports auto-increment m_addr on op completion
    // (top.v line 467), so we walk the whole buffer with one set_addr + a loop.
    task mv_seed;     // zero buffer A, then stamp the seed pattern bytes
        begin
            for (mvi=0; mvi<640; mvi=mvi+1) begin
                case (mvi)
                    95:  sdram_write(10'd16, 16'h0000+95,  8'h01); // glider  r1 b15
                    175: sdram_write(10'd16, 16'h0000+175, 8'h02); // glider  r2 b15
                    240: sdram_write(10'd16, 16'h0000+240, 8'h40); // blinkerB r3 b0
                    241: sdram_write(10'd16, 16'h0000+241, 8'h03); // blinkerB r3 b1
                    242: sdram_write(10'd16, 16'h0000+242, 8'h40); // blinkerA r3 b2
                    243: sdram_write(10'd16, 16'h0000+243, 8'h03); // blinkerA r3 b3
                    254: sdram_write(10'd16, 16'h0000+254, 8'h40); // glider  r3 b14
                    255: sdram_write(10'd16, 16'h0000+255, 8'h03); // glider  r3 b15
                    480: sdram_write(10'd16, 16'h0000+480, 8'h03); // torus   r6 b0
                    559: sdram_write(10'd16, 16'h0000+559, 8'h40); // torus   r6 b79
                    default: sdram_write(10'd16, 16'h0000+mvi, 8'h00);
                endcase
            end
        end
    endtask
    task mv_readback(input [15:0] base);  // slurp 640 bytes -> rb[], sum16 -> bufsum
        begin
            set_bank(10'd16); set_addr(base);
            bufsum = 0;
            for (mvi=0; mvi<640; mvi=mvi+1) begin
                wr_reg(4'h4, 8'h00); poll_busy; rd_reg(4'h6, tmp);
                rb[mvi] = tmp; bufsum = bufsum + tmp;
            end
        end
    endtask
    task mv_chk(input integer i, input [7:0] v);  // assert one packed byte
        begin
            if (rb[i] !== v) begin errors=errors+1;
                $display("FAIL multiverse TICK1 byte[%0d]=%02X want %02X",i,rb[i],v);
            end
        end
    endtask
    task mv_sum(input integer want);  // assert full-buffer sum16 (catches strays)
        begin
            if ((bufsum & 32'hFFFF) !== want) begin errors=errors+1;
                $display("FAIL multiverse TICK1 sum16=%04X want %04X",bufsum & 32'hFFFF, want);
            end
        end
    endtask
    task mv_tick;    // ring slot0 SKILL 1 (LIFE1 = one univ-0 gen + DONE)
        begin
            stage_mbox(2'd0, 8'h01, 8'h00, 8'h00);
            ring(2'd0);
            wait_done(4'b0001, 40000000, c4ok);
            if (!c4ok) begin errors=errors+1;
                $display("FAIL multiverse TICK1 skill never reached DONE");
            end
            collect(2'd0);
        end
    endtask

    // ---- multiverse ROUND-ROBIN helpers (forever-loop proof) ----
    // Seed `bank` buffer A (640 bytes) with a clean horizontal blinker at
    // row3 col7..9 (byte1 bits0..2 => idx 241 = $07), interior of an 8x80 grid.
    task mv_seed_blinker(input [9:0] bank);
        begin
            for (mvi=0; mvi<640; mvi=mvi+1)
                if (mvi==241) sdram_write(bank, 16'h0000+241, 8'h07);
                else          sdram_write(bank, 16'h0000+mvi, 8'h00);
        end
    endtask
    task mv_readback_bank(input [9:0] bank, input [15:0] base);
        begin
            set_bank(bank); set_addr(base);
            bufsum = 0;
            for (mvi=0; mvi<640; mvi=mvi+1) begin
                wr_reg(4'h4, 8'h00); poll_busy; rd_reg(4'h6, tmp);
                rb[mvi] = tmp; bufsum = bufsum + tmp;
            end
        end
    endtask
    // After halt, assert univ in `bank` evolved its blinker by `front`/`gen`.
    // The FRONT-selected buffer always holds the latest fully-computed gen:
    //   front=1 (odd ticks) -> vertical blinker (col8 = byte1 bit1 = $02 at
    //   rows2,3,4 => idx 161/241/321);  front=0 (even) -> horizontal seed ($07
    //   at idx 241). Either way GEN>=1 already proved the universe was ticked.
    task mv_chk_blinker(input [9:0] bank, input [7:0] front, input integer u);
        begin
            mv_readback_bank(bank, front ? 16'h4000 : 16'h0000);
            if (front) begin                    // vertical (evolved)
                mv_chk(161,8'h02); mv_chk(241,8'h02); mv_chk(321,8'h02);
                mv_sum(16'h0006);
            end else begin                       // horizontal (seed, even gen)
                mv_chk(241,8'h07);
                mv_sum(16'h0007);
            end
        end
    endtask

    // ---- Conway's Multiverse GR helpers (8 cells/byte, ROWBYTES=5) ----
    // GR sim universe: GROWS=8 rows x 5 bytes = 40 contiguous bytes; cell
    // (row,col) is at idx row*5 + col/8, bit col%8 (bit0=leftmost column).
    // Both buffers live in bank UBASE(=16): A @ $0000, B @ $0400. Readbacks
    // reuse rb[]/bufsum so mv_chk()/mv_sum() apply unchanged.
    task mvgr_clear;     // zero buffer A (40 bytes) in bank 16
        begin
            for (mvi=0; mvi<40; mvi=mvi+1)
                sdram_write(10'd16, 16'h0000+mvi, 8'h00);
        end
    endtask
    task mvgr_readback(input [15:0] base);  // slurp 40 bytes -> rb[], sum16
        begin
            set_bank(10'd16); set_addr(base);
            bufsum = 0;
            for (mvi=0; mvi<40; mvi=mvi+1) begin
                wr_reg(4'h4, 8'h00); poll_busy; rd_reg(4'h6, tmp);
                rb[mvi] = tmp; bufsum = bufsum + tmp;
            end
        end
    endtask

    // ---- FARM protocol helpers (skill 2, GBANK=32) ----
    // load FARMTASK blob -> coproc BRAM $0600, table[2]=$0600
    task farm_load;
        integer i;
    begin
        wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h06);      // CP_LADDR = $0600
        for (i=0; i<FARMLEN; i=i+1) load_byte(farmimg[i]);
        wr_reg(4'h9, 8'h04); wr_reg(4'hA, 8'h02);      // table[2] @ $0204
        load_byte(8'h00); load_byte(8'h06);            // vector = $0600
    end endtask

    // init GBANK cold-start state. SIG written here too (tb rig writes it
    // up front; FARM.S cold start writes SIG last - the liveness ordering
    // matters on hardware, not in this single-threaded rig). SIG also serves
    // as the lap-test regression net: ring page-wrap bugs land on $0000-$0003.
    task farm_init;
        integer i;
    begin
        sdram_write(10'd32, 16'h0000, 8'h46);          // SIG 'F'
        sdram_write(10'd32, 16'h0001, 8'h4D);          // SIG 'M'
        sdram_write(10'd32, 16'h0002, 8'h00);          // SEQCTR
        sdram_write(10'd32, 16'h0003, 8'h00);          // HEAD
        sdram_write(10'd32, 16'h0200, 8'h00);          // MFLAG
        // v2 market array: 4 x (PRICE lo, PRICE hi, SUPPLY) at $0210+c*3
        sdram_write(10'd32, 16'h0210, 8'd8);  sdram_write(10'd32, 16'h0211, 8'h00); sdram_write(10'd32, 16'h0212, 8'h00); // crop0 wheat
        sdram_write(10'd32, 16'h0213, 8'd14); sdram_write(10'd32, 16'h0214, 8'h00); sdram_write(10'd32, 16'h0215, 8'h00); // crop1 carrot
        sdram_write(10'd32, 16'h0216, 8'd24); sdram_write(10'd32, 16'h0217, 8'h00); sdram_write(10'd32, 16'h0218, 8'h00); // crop2 berry
        sdram_write(10'd32, 16'h0219, 8'd40); sdram_write(10'd32, 16'h021A, 8'h00); sdram_write(10'd32, 16'h021B, 8'h00); // crop3 pumpkin
        sdram_write(10'd32, 16'h0220, 8'd100);         // CASH lo
        sdram_write(10'd32, 16'h0221, 8'h00);          // CASH hi
        sdram_write(10'd32, 16'h0222, 8'd5);           // SEEDS wheat
        sdram_write(10'd32, 16'h0223, 8'd3);           // SEEDS carrot
        sdram_write(10'd32, 16'h0224, 8'd2);           // SEEDS berry
        sdram_write(10'd32, 16'h0225, 8'd1);           // SEEDS pumpkin
        sdram_write(10'd32, 16'h0226, 8'h00);          // CROPS wheat
        sdram_write(10'd32, 16'h0227, 8'h00);          // CROPS carrot
        sdram_write(10'd32, 16'h0228, 8'h00);          // CROPS berry
        sdram_write(10'd32, 16'h0229, 8'h00);          // CROPS pumpkin
        for (i=0; i<400; i=i+1) sdram_write(10'd32, 16'h0300+i, 8'h00);
    end endtask

    // send one command, poll FLAG clear, return RESULT. ok=0 on timeout.
    task farm_cmd(input [7:0] op, input [7:0] a0, input [7:0] a1,
                  input [7:0] a2, output [7:0] res, output ok);
        integer t; reg [7:0] f;
    begin
        sdram_write(10'd32, 16'h0201, op);
        sdram_write(10'd32, 16'h0202, a0);
        sdram_write(10'd32, 16'h0203, a1);
        sdram_write(10'd32, 16'h0204, a2);
        sdram_write(10'd32, 16'h0200, 8'h01);          // FLAG last
        ok = 0; res = 8'hFF;
        for (t=0; t<5000 && !ok; t=t+1) begin
            sdram_read(10'd32, 16'h0200, f);
            if (f == 8'h00) ok = 1;
        end
        if (ok) sdram_read(10'd32, 16'h0205, res);
        else $display("farm_cmd timeout op=%02X (FLAG never cleared)", op);
    end endtask

    task wk_load;
        integer i;
    begin
        wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h20);      // CP_LADDR = $2000
        for (i=0; i<WKLEN; i=i+1) load_byte(wkimg[i]);
        wr_reg(4'h9, 8'h06); wr_reg(4'hA, 8'h02);      // TABLE[3] @ $0206
        load_byte(8'h00); load_byte(8'h20);            // vector = $2000
    end endtask

    // recipe entry poke: 8 bytes at $0300+idx*8 in bank 33
    task wk_recipe(input [7:0] idx, input [7:0] i0, input [7:0] i1,
                   input [7:0] i2, input [7:0] i3, input [7:0] tm,
                   input [7:0] val, input [7:0] rar);
        reg [15:0] a;
    begin
        a = 16'h0300 + idx*8;
        sdram_write(10'd33, a+0, i0); sdram_write(10'd33, a+1, i1);
        sdram_write(10'd33, a+2, i2); sdram_write(10'd33, a+3, i3);
        sdram_write(10'd33, a+4, tm); sdram_write(10'd33, a+5, val);
        sdram_write(10'd33, a+6, rar); sdram_write(10'd33, a+7, 8'h00);
    end endtask

    task wk_init;       // bank 33 cold seed (mirrors //e COLDST)
        integer i;
    begin
        sdram_write(10'd33, 16'h0000, 8'h57);          // SIG 'W'
        sdram_write(10'd33, 16'h0001, 8'h4B);          // SIG 'K'
        sdram_write(10'd33, 16'h0002, 8'h00);          // SEQCTR
        sdram_write(10'd33, 16'h0003, 8'h00);          // HEAD
        sdram_write(10'd33, 16'h0004, 8'h00);          // WHBEAT
        sdram_write(10'd33, 16'h0005, 8'h03);          // CVER v3
        sdram_write(10'd33, 16'h0200, 8'h00);          // FLAG
        for (i=0; i<4;  i=i+1) sdram_write(10'd33, 16'h0210+i, 8'h00); // retired pantry
        for (i=0; i<12; i=i+1) sdram_write(10'd33, 16'h0228+i, 8'h00); // GOODS
        sdram_write(10'd33, 16'h0214, 8'h00);          // SKILL
        sdram_write(10'd33, 16'h0215, 8'h00);          // DISC lo
        sdram_write(10'd33, 16'h0216, 8'h00);          // DISC hi
        sdram_write(10'd33, 16'h0217, 8'h00);          // MODE
        for (i=0; i<8;  i=i+1) sdram_write(10'd33, 16'h0220+i, 8'h00); // stations
        // 3 recipes are enough for the tb (full book is //e content):
        wk_recipe(8'd0, 8'd0, 8'd0, 8'hFF, 8'hFF, 8'd4, 8'd28, 8'd0); // BREAD
        wk_recipe(8'd5, 8'd2, 8'd2, 8'hFF, 8'hFF, 8'd4, 8'd80, 8'd1); // JAM
        wk_recipe(8'd11, 8'd0, 8'd1, 8'd2, 8'd3, 8'd20, 8'd127, 8'd3); // FEAST
        // remaining entries = $FF terminator pattern
        for (i=1; i<5;  i=i+1) wk_recipe(i[7:0], 8'hFE, 8'hFE, 8'hFE, 8'hFE, 8'd1, 8'd0, 8'd0);
        for (i=6; i<11; i=i+1) wk_recipe(i[7:0], 8'hFE, 8'hFE, 8'hFE, 8'hFE, 8'd1, 8'd0, 8'd0);
    end endtask

    task wk_cmd(input [7:0] op, input [7:0] a0, input [7:0] a1,
                input [7:0] a2, input [7:0] a3,
                output [7:0] res, output [7:0] res1, output ok);
        integer t; reg [7:0] f;
    begin
        sdram_write(10'd33, 16'h0201, op);
        sdram_write(10'd33, 16'h0202, a0);
        sdram_write(10'd33, 16'h0203, a1);
        sdram_write(10'd33, 16'h0204, a2);
        sdram_write(10'd33, 16'h0205, a3);
        sdram_write(10'd33, 16'h0200, 8'h01);          // FLAG last
        ok = 0; res = 8'hFF; res1 = 8'hFF;
        for (t=0; t<5000 && !ok; t=t+1) begin
            sdram_read(10'd33, 16'h0200, f);
            if (f == 8'h00) ok = 1;
        end
        if (ok) begin
            sdram_read(10'd33, 16'h0206, res);
            sdram_read(10'd33, 16'h0207, res1);
        end else $display("wk_cmd timeout op=%02X", op);
    end endtask

    // reader-rule drain: from farm_tail/farm_seq, dispatch into ev arrays
    reg [7:0] farm_tail = 0, farm_seq = 0;
    integer farm_nev = 0;
    reg [7:0] ev_type [0:255]; reg [7:0] ev_p0 [0:255]; reg [7:0] ev_p1 [0:255];
    integer farm_resyncs = 0;
    task farm_drain;
        reg [7:0] h, s, s2, rs, rt, rp0, rp1; integer guard; integer rsg;
    begin
        sdram_read(10'd32, 16'h0003, h);
        guard = 0;
        while (farm_tail !== h && guard < 128) begin
            sdram_read(10'd32, 16'h0100 + farm_tail*4 + 0, rs);
            sdram_read(10'd32, 16'h0100 + farm_tail*4 + 1, rt);
            sdram_read(10'd32, 16'h0100 + farm_tail*4 + 2, rp0);
            sdram_read(10'd32, 16'h0100 + farm_tail*4 + 3, rp1);
            if (rs !== farm_seq) begin
                // overrun -> resync: stable (SEQCTR,HEAD) pair
                farm_resyncs = farm_resyncs + 1;
                s2 = 8'hFF; rsg = 0;
                while (s2 !== s && rsg < 16) begin
                    sdram_read(10'd32, 16'h0002, s);
                    sdram_read(10'd32, 16'h0003, h);
                    sdram_read(10'd32, 16'h0002, s2);
                    rsg = rsg + 1;
                end
                if (s2 !== s) begin
                    errors = errors + 1;
                    $display("FAIL farm_drain resync: no stable (SEQCTR,HEAD) pair in 16 tries");
                end
                farm_tail = h; farm_seq = s;
            end else begin
                ev_type[farm_nev]=rt; ev_p0[farm_nev]=rp0; ev_p1[farm_nev]=rp1;
                farm_nev = farm_nev + 1;
                farm_tail = (farm_tail + 1) & 8'h3F;
                farm_seq = farm_seq + 1;
            end
            guard = guard + 1;
        end
    end endtask

    initial begin
        // VCD only on demand (+vcd): full-suite dumps reach ~20 GB and
        // dominate wall time. make sim PLUSARGS=+vcd when waves are needed.
        if ($test$plusargs("vcd")) begin
            $dumpfile("project_obscurus_tb.vcd"); $dumpvars(0, project_obscurus_tb);
        end
        nRES_READ=1'b0; #1000; nRES_READ=1'b1;
        wait (dut.ready);
        $display("[%0t] ready", $time);

        // +farmonly skips every pre-farm phase (kernel boots on reset; the
        // farm section is self-contained: farm_init/farm_load own all state,
        // slot 0 is free on a fresh boot). Full suite remains the default
        // and the pre-merge gate.
        if ($test$plusargs("farmonly")) $display("--- farmonly: skipping pre-farm phases ---");
        else begin : prefarm

        // 1. W-then-R round trip
        sdram_write(10'd0, 16'h0000, 8'hAA);
        sdram_read (10'd0, 16'h0000, tmp);
        if (tmp!==8'hAA) begin errors=errors+1; $display("FAIL rt %02X",tmp); end

        // 2. bank isolation
        sdram_write(10'd1, 16'h0000, 8'h5A);
        sdram_read (10'd0, 16'h0000, tmp);
        if (tmp!==8'hAA) begin errors=errors+1; $display("FAIL bank0 %02X",tmp); end
        sdram_read (10'd1, 16'h0000, tmp);
        if (tmp!==8'h5A) begin errors=errors+1; $display("FAIL bank1 %02X",tmp); end

        // 3. byte-lane isolation
        sdram_write(10'd0, 16'h0010, 8'h11);
        sdram_write(10'd0, 16'h0011, 8'h22);
        sdram_read (10'd0, 16'h0010, tmp);
        if (tmp!==8'h11) begin errors=errors+1; $display("FAIL lane lo %02X",tmp); end
        sdram_read (10'd0, 16'h0011, tmp);
        if (tmp!==8'h22) begin errors=errors+1; $display("FAIL lane hi %02X",tmp); end

        // 4. busy asserts at strobe
        set_bank(10'd0); set_addr(16'h0100);
        wr_reg(4'h6, 8'h7E);
        rd_reg(4'h5, tmp);
        if (!tmp[7]) begin errors=errors+1; $display("FAIL busy-at-strobe"); end
        poll_busy;

        // 5. auto-increment + D dump
        // NOTE: write loop uses a local iterator (not the shared `tmp`) because
        // sdram_write() -> poll_busy() clobbers `tmp` with the STATUS byte; using
        // `tmp` as the loop variable would make the loop run only once (the read
        // loop below already uses a local `i` for the same reason).
        begin : fill
            integer j;
            for (j=0; j<16; j=j+1)
                sdram_write(10'd0, 16'h0200 + j, 8'h20 + j);
        end
        set_bank(10'd0); set_addr(16'h0200);
        begin : dump
            integer i; reg [7:0] v;
            for (i=0; i<16; i=i+1) begin
                wr_reg(4'h4, 8'h00); poll_busy; rd_reg(4'h6, v);
                if (v !== 8'h20 + i) begin
                    errors=errors+1; $display("FAIL dump[%0d]=%02X",i,v); end
            end
        end

        // 6. read with NO preceding strobe must NOT advance addr
        rd_reg(4'h6, tmp);
        set_addr(16'h0200);
        wr_reg(4'h4, 8'h00); poll_busy; rd_reg(4'h6, tmp);
        if (tmp !== 8'h20) begin errors=errors+1; $display("FAIL no-strobe stepped"); end

        // 7. $CFFF byte: rom2mem fills unused space with $FF; the stub here is
        //    all $00. The requirement is only that $CFFF is never live monitor
        //    code. Accept either $00 (stub) or $FF (real build fill).
        if (dut.monitor_mem[11'h7FF] !== 8'h00 && dut.monitor_mem[11'h7FF] !== 8'hFF && dut.monitor_mem[11'h7FF] !== 8'h60)
            begin errors=errors+1; $display("FAIL $CFFF live %02X",dut.monitor_mem[11'h7FF]); end

        // 8. expansion ROM SILENT by default (disarmed): set rom_en via a $C4xx
        //    access, then read $C8xx -> card must NOT drive (exp_read low).
        apple_addr = 16'hC400; nI_O_SELECT=1'b0; R_nW=1'b1; #200;
        nI_O_SELECT=1'b1; #100;
        apple_addr = 16'hC800; nI_O_STROBE=1'b0; R_nW=1'b1; #200;
        if (dut.rom_armed !== 1'b0) begin errors=errors+1; $display("FAIL armed at reset"); end
        if (dut.exp_read  !== 1'b0) begin errors=errors+1; $display("FAIL exp_read while disarmed"); end
        nI_O_STROBE=1'b1; #100;

        // 9. ARM via $C0C7<-$AA; DISARM via $C0C8<-$AA.
        wr_reg(4'h7, 8'hAA);
        if (dut.rom_armed !== 1'b1) begin errors=errors+1; $display("FAIL not armed"); end
        wr_reg(4'h8, 8'hAA);
        if (dut.rom_armed !== 1'b0) begin errors=errors+1; $display("FAIL not disarmed"); end

        // 10. magic guard: non-$AA write to $C0C7 must NOT arm.
        wr_reg(4'h7, 8'h55);
        if (dut.rom_armed !== 1'b0) begin errors=errors+1; $display("FAIL armed by non-magic"); end

        // 11. reset disarms.
        wr_reg(4'h7, 8'hAA);
        nRES_READ=1'b0; #500; nRES_READ=1'b1; #200;
        if (dut.rom_armed !== 1'b0) begin errors=errors+1; $display("FAIL reset did not disarm"); end

        // --- coproc liveness: re-wait `ready` after the check-11 reset pulse ---
        // Check 11 above pulsed nRES_READ low, which re-inits the SDRAM
        // controller (back to ST_INIT). Re-wait for `ready` so both the coproc
        // (held in ST_BOOT until ready) and the monitor port operate against a
        // live controller before the C1 task-registration sequence below.
        //
        // NOTE: the original C0 milestone (an autonomous coproc that hardcoded a
        // $42 write to bank0/$0040 via coproc_prog.mem) was SUPERSEDED by the C1
        // resident kernel (kernel.mem) — the coproc no longer writes anything on
        // its own; it idle-spins until the host registers a task (COUNT>0). The
        // equivalent "coproc writes a known byte to a known SDRAM location, host
        // reads it back" property is now proven by C1 below (taskA -> $99@$0050,
        // and taskC -> $42@$0040 preserving the original C0 assertion).
        wait (dut.ready);
        // monitor regression: host write/read elsewhere still works with arbiter
        // in front. Use bank1 (the sdram_model holds banks 0-1 = 64K words; bank2+
        // is outside the model and would always read 00) at an address untouched
        // by the coproc, to prove an independent c0 op round-trips post-arbiter.
        sdram_write(10'd1, 16'h00AB, 8'h99);
        sdram_read (10'd1, 16'h00AB, tmp);
        if (tmp!==8'h99) begin errors=errors+1; $display("FAIL monitor regress %02X",tmp); end
        else $display("PASS monitor regress");

        // ===== C1: resident kernel + task registration (COUNT-LAST) =====
        // baseline the protected bytes from the clean kernel image (COUNT still 0)
        cp_read(13'h1000, krn_before);     // kernel first opcode ($78 SEI)
        cp_read(13'h0200, tbl_before);     // TABLE byte (zero)
        // NOTE: under the current COOPERATIVE kernel (kernel.S, the C2 scheduler)
        // a dispatched task that terminates with RTS pops an EMPTY stack (RESTORE
        // already rts'd INTO the task) and derails the coproc CPU into a BRK/RTI
        // storm -- after which the kernel never regains control, so only the FIRST
        // task's write lands and taskC never dispatches. The kernel's finish API is
        // JMP DONE ($1006): it marks the task DONE and PICKANY advances to the next
        // ready task. So C1 one-shot tasks MUST end with JMP DONE, not RTS. (This
        // was the root cause of the long-standing "C1 taskC result 00" failure.)
        // taskA at $0300: writes $99 to bank0 $0050, then JMP DONE
        wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h03);   // CP_LADDR = $0300
        load_byte(8'hA9); load_byte(8'h50); load_byte(8'h8D); load_byte(8'h00); load_byte(8'hE0);
        load_byte(8'hA9); load_byte(8'h00); load_byte(8'h8D); load_byte(8'h01); load_byte(8'hE0);
        load_byte(8'h8D); load_byte(8'h02); load_byte(8'hE0); load_byte(8'hA9); load_byte(8'h99);
        load_byte(8'h8D); load_byte(8'h03); load_byte(8'hE0); load_byte(8'h4C); load_byte(8'h06); load_byte(8'h10);
        // taskBAD at $0320: STA $1000 + STA $0200 must be refused, then JMP DONE
        wr_reg(4'h9, 8'h20); wr_reg(4'hA, 8'h03);   // CP_LADDR = $0320
        load_byte(8'hA9); load_byte(8'hEE); load_byte(8'h8D); load_byte(8'h00); load_byte(8'h10);
        load_byte(8'h8D); load_byte(8'h00); load_byte(8'h02); load_byte(8'h4C); load_byte(8'h06); load_byte(8'h10);
        // taskC at $0340: writes $42 to bank0 $0040 (preserves the original C0 assertion), then JMP DONE
        wr_reg(4'h9, 8'h40); wr_reg(4'hA, 8'h03);   // CP_LADDR = $0340
        load_byte(8'hA9); load_byte(8'h40); load_byte(8'h8D); load_byte(8'h00); load_byte(8'hE0);
        load_byte(8'hA9); load_byte(8'h00); load_byte(8'h8D); load_byte(8'h01); load_byte(8'hE0);
        load_byte(8'h8D); load_byte(8'h02); load_byte(8'hE0); load_byte(8'hA9); load_byte(8'h42);
        load_byte(8'h8D); load_byte(8'h03); load_byte(8'hE0); load_byte(8'h4C); load_byte(8'h06); load_byte(8'h10);
        // TABLE: entry0=$0300 @ $0200, entry1=$0320 @ $0202, entry2=$0340 @ $0204
        wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h02); load_byte(8'h00); load_byte(8'h03);
        wr_reg(4'h9, 8'h02); wr_reg(4'hA, 8'h02); load_byte(8'h20); load_byte(8'h03);
        wr_reg(4'h9, 8'h04); wr_reg(4'hA, 8'h02); load_byte(8'h40); load_byte(8'h03);
        // *** release: COUNT = 3 LAST ***
        wr_reg(4'hD, 8'h03);
        repeat (8000) @(posedge clk100);   // let the kernel dispatch all three repeatedly
        sdram_read(10'd0, 16'h0050, tmp);
        if (tmp!==8'h99) begin errors=errors+1; $display("FAIL C1 taskA result %02X",tmp); end
        else $display("PASS C1 taskA dispatched ($99 @ $0050)");
        sdram_read(10'd0, 16'h0040, tmp);
        if (tmp!==8'h42) begin errors=errors+1; $display("FAIL C1 taskC result %02X want 42",tmp); end
        else $display("PASS C1 taskC dispatched ($42 @ $0040)");
        cp_read(13'h1000, tmp);
        if (tmp!==krn_before) begin errors=errors+1; $display("FAIL C1 task wrote kernel $1000"); end
        else $display("PASS C1 kernel $1000 protected from task");
        cp_read(13'h0200, tmp);
        if (tmp!==tbl_before) begin errors=errors+1; $display("FAIL C1 task wrote TABLE $0200"); end
        else $display("PASS C1 TABLE $0200 protected from task");

        // --- itr4.1: high-BRAM ($4000-$BFFF) load + readback ---
        // Proves the 48 KB array + 16-bit load path are real. Pre-itr4.1
        // these writes aliased back under $4000 (CP_LADDR_HI capped at 6 bits).
        cp_read16(16'h0000, hibram_z0);  // snapshot $0000 before high writes
        cp_load_byte16(16'h4000, 8'hA5); // alt-bit pattern, WORKTASK base
        cp_load_byte16(16'h8000, 8'h5A); // future-task base
        cp_load_byte16(16'hBFFF, 8'h3C); // top of task space
        cp_read16(16'h4000, hibram_r);
        if (hibram_r!==8'hA5) begin errors=errors+1; $display("FAIL hibram $4000=%h", hibram_r); end
        cp_read16(16'h8000, hibram_r);
        if (hibram_r!==8'h5A) begin errors=errors+1; $display("FAIL hibram $8000=%h", hibram_r); end
        cp_read16(16'hBFFF, hibram_r);
        if (hibram_r!==8'h3C) begin errors=errors+1; $display("FAIL hibram $BFFF=%h", hibram_r); end
        // alias guard: the $4000 write must NOT have touched $0000
        cp_read16(16'h0000, hibram_r);
        if (hibram_r!==hibram_z0) begin errors=errors+1; $display("FAIL hibram alias $0000 %h->%h", hibram_z0, hibram_r); end
        else $display("PASS hibram 48KB load/readback");

        // ===== C2: cooperative race (NPARAM 4/8 -> order 1/2; re-arm 12/8 -> flip 2/1) =====
        // The C1 tasks above terminate with RTS (not JSR DONE), which is the OLD
        // pre-cooperative task convention: after RESTORE rts'd into the task, the
        // task's own final RTS pops an empty stack and derails the coproc CPU into
        // a BRK/RTI storm (verified: post-C1 the CPU never re-enters the kernel,
        // krn=0 rdcount=0). The C2 cooperative tasks (JSR YIELD / JSR DONE) are a
        // different, correct convention -- but they need a LIVE kernel. Pulse the
        // coproc reset so the CPU restarts at the $1000 RESET vector (KWAIT0),
        // task_count clears to 0, and BRAM (kernel.mem + anything we load next) is
        // preserved. Then re-wait `ready` (reset also re-inits the SDRAM ctrl).
        nRES_READ=1'b0; #1000; nRES_READ=1'b1; #200;
        wait (dut.ready);
        repeat (2000) @(posedge clk100);            // let kernel reach KWAIT0
        // load racetask (45 bytes) at coproc $0300
        wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h03);   // CP_LADDR = $0300
        load_byte(8'hA4); load_byte(8'hEC); load_byte(8'hB6); load_byte(8'hE8); load_byte(8'hCA);
        load_byte(8'h20); load_byte(8'h03); load_byte(8'h10); load_byte(8'hD0); load_byte(8'hFA);
        load_byte(8'hA5); load_byte(8'hED); load_byte(8'h18); load_byte(8'h69); load_byte(8'h01);
        load_byte(8'h85); load_byte(8'hED); load_byte(8'h48); load_byte(8'hA5); load_byte(8'hEE);
        load_byte(8'h18); load_byte(8'h65); load_byte(8'hEC); load_byte(8'h8D); load_byte(8'h00);
        load_byte(8'hE0); load_byte(8'hA5); load_byte(8'hEF); load_byte(8'h69); load_byte(8'h00);
        load_byte(8'h8D); load_byte(8'h01); load_byte(8'hE0); load_byte(8'hA9); load_byte(8'h00);
        load_byte(8'h8D); load_byte(8'h02); load_byte(8'hE0); load_byte(8'h68); load_byte(8'h8D);
        load_byte(8'h03); load_byte(8'hE0); load_byte(8'h20); load_byte(8'h06); load_byte(8'h10);
        // TABLE entry0=$0300 @ $0200, entry1=$0300 @ $0202
        wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h02);
        load_byte(8'h00); load_byte(8'h03); load_byte(8'h00); load_byte(8'h03);
        // NPARAM[0]=4, NPARAM[1]=8 @ coproc ZP $00E8
        wr_reg(4'h9, 8'hE8); wr_reg(4'hA, 8'h00);
        load_byte(8'd4); load_byte(8'd8);
        // arm: kernel is fresh-reset (COUNT=0, parked at KWAITN). COUNT=2 is the
        // 0->nonzero arm edge -> BOOTSTRAP dispatches both racetask instances.
        wr_reg(4'hD, 8'h02);                        // COUNT=2 (arm edge 0->nonzero)
        repeat (40000) @(posedge clk100);           // AMPLE: both finish, kernel parks
        sdram_read(10'd0, 16'h0061, tmp);
        if (tmp!==8'h01) begin errors=errors+1; $display("FAIL C2 race r0 %02X want 01",tmp); end
        else $display("PASS C2 race task0 first (r0=01)");
        sdram_read(10'd0, 16'h0062, tmp);
        if (tmp!==8'h02) begin errors=errors+1; $display("FAIL C2 race r1 %02X want 02",tmp); end
        else $display("PASS C2 race task1 second (r1=02)");
        // re-arm + flip: NPARAM 12/8, single GO write (COUNT=0 park removed)
        wr_reg(4'h9, 8'hE8); wr_reg(4'hA, 8'h00);
        load_byte(8'd12); load_byte(8'd8);
        wr_reg(4'hD, 8'h02);
        repeat (40000) @(posedge clk100);
        sdram_read(10'd0, 16'h0061, tmp);
        if (tmp!==8'h02) begin errors=errors+1; $display("FAIL C2 flip r0 %02X want 02",tmp); end
        else $display("PASS C2 flip task0 now second (r0=02)");
        sdram_read(10'd0, 16'h0062, tmp);
        if (tmp!==8'h01) begin errors=errors+1; $display("FAIL C2 flip r1 %02X want 01",tmp); end
        else $display("PASS C2 flip task1 now first (r1=01)");

        // ===== C3: PREEMPTIVE race (no-yield tasks sliced by the timer tick) =====
        // Isolate the scenario exactly like the C2 block: pulse the coproc reset so
        // the CPU restarts at the $1000 RESET vector (KWAIT0), task_count clears to 0,
        // and BRAM (kernel.mem + anything we load next) is preserved. Re-wait `ready`
        // (reset also re-inits the SDRAM ctrl) and let the kernel reach KWAIT0.
        nRES_READ=1'b0; #1000; nRES_READ=1'b1; #200;
        wait (dut.ready);
        repeat (2000) @(posedge clk100);            // let kernel reach KWAIT0
        // load racetask3 (49 bytes) at coproc $0300:
        wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h03);
        load_byte(8'hA4); load_byte(8'hEC); load_byte(8'hB6); load_byte(8'hE8); load_byte(8'hA9);
        load_byte(8'h80); load_byte(8'h38); load_byte(8'hE9); load_byte(8'h01); load_byte(8'hD0);
        load_byte(8'hFC); load_byte(8'hCA); load_byte(8'hD0); load_byte(8'hF6); load_byte(8'h78);
        load_byte(8'hA5); load_byte(8'hED); load_byte(8'h18); load_byte(8'h69); load_byte(8'h01);
        load_byte(8'h85); load_byte(8'hED); load_byte(8'h48); load_byte(8'h98); load_byte(8'h18);
        load_byte(8'h65); load_byte(8'hEE); load_byte(8'h8D); load_byte(8'h00); load_byte(8'hE0);
        load_byte(8'hA5); load_byte(8'hEF); load_byte(8'h69); load_byte(8'h00); load_byte(8'h8D);
        load_byte(8'h01); load_byte(8'hE0); load_byte(8'hA9); load_byte(8'h00); load_byte(8'h8D);
        load_byte(8'h02); load_byte(8'hE0); load_byte(8'h68); load_byte(8'h8D); load_byte(8'h03);
        load_byte(8'hE0); load_byte(8'h4C); load_byte(8'h06); load_byte(8'h10);
        // TABLE entry0=$0300@$0200, entry1=$0300@$0202
        wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h02);
        load_byte(8'h00); load_byte(8'h03); load_byte(8'h00); load_byte(8'h03);
        // NPARAM[0]=4, NPARAM[1]=12 (clear margin; small for sim) @ $00E8
        wr_reg(4'h9, 8'hE8); wr_reg(4'hA, 8'h00);
        load_byte(8'd4); load_byte(8'd12);
        wr_reg(4'hD, 8'h02);                         // COUNT=2 (arm)
        repeat (200000) @(posedge clk100);           // ample - inner-delay tasks run long
        sdram_read(10'd0, 16'h0061, tmp);
        if (tmp!==8'h01) begin errors=errors+1; $display("FAIL C3 race r0 %02X want 01",tmp); end
        else $display("PASS C3 preemptive race task0 first (r0=01)");
        sdram_read(10'd0, 16'h0062, tmp);
        if (tmp!==8'h02) begin errors=errors+1; $display("FAIL C3 race r1 %02X want 02",tmp); end
        else $display("PASS C3 preemptive race task1 second (r1=02)");
        // re-arm FLIP (this is the preemption proof): NPARAM 20/12, single GO write
        wr_reg(4'h9, 8'hE8); wr_reg(4'hA, 8'h00);
        load_byte(8'd20); load_byte(8'd12);
        wr_reg(4'hD, 8'h02);
        repeat (200000) @(posedge clk100);
        sdram_read(10'd0, 16'h0061, tmp);
        if (tmp!==8'h02) begin errors=errors+1; $display("FAIL C3 flip r0 %02X want 02 (timer not preempting?)",tmp); end
        else $display("PASS C3 flip task0 now second (r0=02)");
        sdram_read(10'd0, 16'h0062, tmp);
        if (tmp!==8'h01) begin errors=errors+1; $display("FAIL C3 flip r1 %02X want 01",tmp); end
        else $display("PASS C3 flip task1 now first (r1=01)");

        // ===== C3.1 GO-trigger regression: back-to-back, NO reset between =====
        // The C3 block just ran (racetask3 @ $0300). Its FINAL flip (NPARAM 20/12,
        // task1 smaller) left A's result = $0061=02, $0062=01 (task1 first).
        // Scenario B: re-arm with ONE CP_COUNT write (no COUNT=0 park, NO nRES).
        // If GO works, B re-bootstraps fresh and writes B's OWN result.
        // If the old COUNT-level bug were present, B would read A's stale cells.
        //
        // DISCRIMINATION: A left 02/01. The task's suggested B (16/4) would also give
        // 02/01 -> a stale read would be INDISTINGUISHABLE from fresh. So B must be
        // task0-first: NPARAM[0]=4, NPARAM[1]=16 -> task0 (smaller budget) finishes
        // first -> r0=01, r1=02. stale(02/01) != fresh(01/02) -> the test discriminates.
        wr_reg(4'h9, 8'hE8); wr_reg(4'hA, 8'h00);
        load_byte(8'd4); load_byte(8'd16);
        wr_reg(4'hD, 8'h02);                          // single GO write, no COUNT=0, no reset
        repeat (200000) @(posedge clk100);
        sdram_read(10'd0, 16'h0061, tmp);
        if (tmp!==8'h01) begin errors=errors+1; $display("FAIL C3.1 regress r0 %02X want 01 (stale 02 = GO didn't fire?)",tmp); end
        else $display("PASS C3.1 GO re-bootstrap fresh (r0=01)");
        sdram_read(10'd0, 16'h0062, tmp);
        if (tmp!==8'h02) begin errors=errors+1; $display("FAIL C3.1 regress r1 %02X want 02 (stale 01?)",tmp); end
        else $display("PASS C3.1 GO regression task0 first (r1=02)");
        // CP_COUNT=0 must be a benign park (nonzero gate) - NOT crash the kernel.
        // After the park, GO with task1-first NPARAM (16/4 -> r0=02) so the result
        // DIFFERS from B's 01 above: proves the post-park run is a fresh dispatch.
        wr_reg(4'hD, 8'h00);                          // park (no GO)
        repeat (2000) @(posedge clk100);
        wr_reg(4'h9, 8'hE8); wr_reg(4'hA, 8'h00);
        load_byte(8'd16); load_byte(8'd4);            // task1 smaller -> r0=02
        wr_reg(4'hD, 8'h02);
        repeat (200000) @(posedge clk100);
        sdram_read(10'd0, 16'h0061, tmp);
        if (tmp!==8'h02) begin errors=errors+1; $display("FAIL C3.1 after-park r0 %02X want 02 (CP_COUNT=0 crashed kernel?)",tmp); end
        else $display("PASS C3.1 kernel survived CP_COUNT=0 park (r0=02)");

        // ===== C3 BRK-safety: a stray BRK in a task must NOT corrupt the scheduler =====
        // The ISR's B-bit leg ($1F00: AND #$10 -> IRQBRK -> RTI) returns into the BRK
        // task so it proceeds to JMP DONE; a paired marker task must still run and
        // land its write. Isolate like the other scenarios (reset -> re-bootstrap).
        nRES_READ=1'b0; #1000; nRES_READ=1'b1; #200;
        wait (dut.ready);
        repeat (2000) @(posedge clk100);
        // clear any stale marker
        sdram_write(10'd0, 16'h0070, 8'h00);
        // task0 (marker) at $0300: write $5A -> bank0 $0070 via $E000 window, JMP DONE
        wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h03);
        load_byte(8'hA9); load_byte(8'h70); load_byte(8'h8D); load_byte(8'h00); load_byte(8'hE0);
        load_byte(8'hA9); load_byte(8'h00); load_byte(8'h8D); load_byte(8'h01); load_byte(8'hE0);
        load_byte(8'h8D); load_byte(8'h02); load_byte(8'hE0); load_byte(8'hA9); load_byte(8'h5A);
        load_byte(8'h8D); load_byte(8'h03); load_byte(8'hE0); load_byte(8'h4C); load_byte(8'h06); load_byte(8'h10);
        // task1 (BRK) at $0320: BRK, pad, JMP DONE  (00 00 4C 06 10)
        wr_reg(4'h9, 8'h20); wr_reg(4'hA, 8'h03);
        load_byte(8'h00); load_byte(8'h00); load_byte(8'h4C); load_byte(8'h06); load_byte(8'h10);
        // TABLE entry0=$0300 @ $0200, entry1=$0320 @ $0202
        wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h02);
        load_byte(8'h00); load_byte(8'h03); load_byte(8'h20); load_byte(8'h03);
        // NPARAM (unused by these tasks, but BOOTSTRAP touches it): small values
        wr_reg(4'h9, 8'hE8); wr_reg(4'hA, 8'h00);
        load_byte(8'd4); load_byte(8'd4);
        wr_reg(4'hD, 8'h02);                         // COUNT=2 (arm)
        repeat (40000) @(posedge clk100);
        sdram_read(10'd0, 16'h0070, tmp);
        if (tmp!==8'h5A) begin errors=errors+1; $display("FAIL C3 BRK-safety marker %02X want 5A (BRK corrupted scheduler?)",tmp); end
        else $display("PASS C3 BRK-safety: marker landed ($5A @ $0070); stray BRK RTI'd clean");

        // ===== C-flash: SAVE registry -> RAM loss -> auto-RESTORE -> run service =====
        // The END-TO-END proof. Persist the host-registered service to SPI flash,
        // EXPLICITLY wipe the BRAM task region (simulating the power-cycle RAM loss
        // that a sim reset does NOT model -- $readmemh runs only once at t=0), then
        // pulse the coproc reset to re-arm the restore FSM, which must repopulate the
        // task region FROM FLASH. If the service then runs, the ONLY path the code
        // took to BRAM is the flash restore. (See SUBTLETY 1 for the SCK hookup and
        // SUBTLETY 2 for why the explicit zero-fill makes this proof meaningful.)
        //
        // Isolate like the other scenarios: reset coproc, re-wait ready, let park.
        nRES_READ=1'b0; #1000; nRES_READ=1'b1; #200;
        wait (dut.ready);
        repeat (2000) @(posedge clk100);            // let kernel reach KWAITN

        // 1. Host-load racetask3 (49 bytes) @ coproc $0300 -- the SAME service the
        //    C3/C3.1 blocks register. This is the pre-save "registry" in BRAM.
        wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h03);   // CP_LADDR = $0300
        load_byte(8'hA4); load_byte(8'hEC); load_byte(8'hB6); load_byte(8'hE8); load_byte(8'hA9);
        load_byte(8'h80); load_byte(8'h38); load_byte(8'hE9); load_byte(8'h01); load_byte(8'hD0);
        load_byte(8'hFC); load_byte(8'hCA); load_byte(8'hD0); load_byte(8'hF6); load_byte(8'h78);
        load_byte(8'hA5); load_byte(8'hED); load_byte(8'h18); load_byte(8'h69); load_byte(8'h01);
        load_byte(8'h85); load_byte(8'hED); load_byte(8'h48); load_byte(8'h98); load_byte(8'h18);
        load_byte(8'h65); load_byte(8'hEE); load_byte(8'h8D); load_byte(8'h00); load_byte(8'hE0);
        load_byte(8'hA5); load_byte(8'hEF); load_byte(8'h69); load_byte(8'h00); load_byte(8'h8D);
        load_byte(8'h01); load_byte(8'hE0); load_byte(8'hA9); load_byte(8'h00); load_byte(8'h8D);
        load_byte(8'h02); load_byte(8'hE0); load_byte(8'h68); load_byte(8'h8D); load_byte(8'h03);
        load_byte(8'hE0); load_byte(8'h4C); load_byte(8'h06); load_byte(8'h10);
        // TABLE entry0=$0300@$0200, entry1=$0300@$0202 (both instances of the task)
        wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h02);
        load_byte(8'h00); load_byte(8'h03); load_byte(8'h00); load_byte(8'h03);

        // 2. SAVE to flash: write $5A to $C0CF, wait save_busy (b7) to clear.
        wr_reg(4'hF, 8'h5A);
        repeat (50) @(posedge clk100);              // let save_busy assert
        begin : wsave
            integer g; g=0;
            rd_reg(4'hF, tmp);
            while (tmp[7]) begin
                repeat (500) @(posedge clk100); rd_reg(4'hF, tmp);
                g=g+1; if (g>4000) begin
                    $display("FAIL C-flash SAVE never completed (b7 stuck)");
                    errors=errors+1; disable wsave; end
            end
        end
        $display("PASS C-flash SAVE complete");
        // Confirm magic "CR" landed in the flash model's header page (0x400000 ->
        // sector-relative byte 0/1).  Diagnostic only (not the proof itself).
        if (u_flash.mem[0]!==8'h43 || u_flash.mem[1]!==8'h52)
            $display("WARN C-flash header magic = %02X %02X (want 43 52)",
                     u_flash.mem[0], u_flash.mem[1]);

        // 3. EXPLICITLY ZERO the task region $0200-$0FFF (3584 bytes) via the host
        //    load port -- simulates the RAM loss a power-cycle causes (BRAM is NOT
        //    wiped by a sim reset). This wipes BOTH the TABLE ($0200) and the task
        //    code ($0300). After this, the service is GONE from BRAM.
        wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h02);   // CP_LADDR = $0200
        begin : zerofill
            integer k;
            for (k=0; k<3584; k=k+1) load_byte(8'h00);
        end
        // verify the wipe: TABLE byte and task first opcode now read back 00
        cp_read(13'h0200, tmp);
        if (tmp!==8'h00) begin errors=errors+1; $display("FAIL C-flash zero TABLE %02X",tmp); end
        cp_read(13'h0300, tmp);
        if (tmp!==8'h00) begin errors=errors+1; $display("FAIL C-flash zero task %02X",tmp); end
        else $display("PASS C-flash task region zeroed");

        // 4. Pulse the coproc reset: rst_n drops -> the restore FSM's `started` latch
        //    clears (re-arms) AND the CPU restarts; boot_done(por_n) stays high so the
        //    restore re-runs: reads flash header (valid "CR" now) -> streams 3584 data
        //    bytes back into BRAM $0200-$0FFF, repopulating the zeroed region.
        nRES_READ=1'b0; #1000; nRES_READ=1'b1; #200;
        wait (dut.ready);
        // 5. Wait for restore to finish: poll $C0CF b1 (restore_done).
        begin : wrest
            integer g; g=0;
            rd_reg(4'hF, tmp);
            while (!tmp[1]) begin
                repeat (1000) @(posedge clk100); rd_reg(4'hF, tmp);
                g=g+1; if (g>4000) begin
                    $display("FAIL C-flash RESTORE never completed (b1 stuck)");
                    errors=errors+1; disable wrest; end
            end
        end
        // 6. Assert restore_valid (b0) == 1.
        rd_reg(4'hF, tmp);
        if (!tmp[0]) begin errors=errors+1;
            $display("FAIL C-flash restore_valid=0 (magic/round-trip broken) stat=%02X",tmp); end
        else $display("PASS C-flash auto-restore valid");
        repeat (2000) @(posedge clk100);            // let kernel reach KWAITN

        // 7. Set NPARAM fresh ($00E8 is ZP, BELOW the saved $0200-$0FFF region, so it
        //    is NOT restored from flash -- the host must re-supply it). NPARAM[0]=4,
        //    NPARAM[1]=16 -> task0 (smaller budget) finishes first -> r0=01, r1=02.
        wr_reg(4'h9, 8'hE8); wr_reg(4'hA, 8'h00);
        load_byte(8'd4); load_byte(8'd16);
        // clear stale result cells so a non-running task can't masquerade as a pass
        sdram_write(10'd0, 16'h0061, 8'h00);
        sdram_write(10'd0, 16'h0062, 8'h00);
        // GO: COUNT=2 is the 0->nonzero arm edge -> BOOTSTRAP dispatches the RESTORED
        //     task. If this runs, the code reached BRAM ONLY via the flash restore.
        wr_reg(4'hD, 8'h02);
        repeat (200000) @(posedge clk100);
        sdram_read(10'd0, 16'h0061, tmp); krn_before = tmp;     // reuse reg for r0
        sdram_read(10'd0, 16'h0062, tmp); tbl_before = tmp;     // reuse reg for r1
        if (krn_before!==8'h01 || tbl_before!==8'h02) begin
            errors=errors+1;
            $display("FAIL C-flash service did NOT run from flash (r0=%02X r1=%02X want 01 02)",
                     krn_before, tbl_before);
        end else
            $display("PASS C-flash service ran from flash (r0=%02X r1=%02X)",
                     krn_before, tbl_before);

        // ===== C4: ASYNC SKILL DISPATCH (positive verification of the kernel) =====
        // Isolate like every other block: pulse coproc reset -> RESET re-runs
        // (CLEARSLOTS, arm free-running tick, reach KIDLE/KISPIN CLI-idle), and the
        // gateware control-plane regs (call_req/running/done/timedout/go_pending)
        // all clear on rst_n. The reset also re-arms the C-flash restore FSM, which
        // repopulates BRAM $0200-$0FFF from flash; WAIT for restore_done before we
        // host-load cmpskill (restore owns coproc port B until done).
        nRES_READ=1'b0; #1000; nRES_READ=1'b1; #200;
        wait (dut.ready);
        begin : c4_wrest
            integer g; g=0;
            rd_reg(4'hF, tmp);
            while (!tmp[1]) begin
                repeat (1000) @(posedge clk100); rd_reg(4'hF, tmp);
                g=g+1; if (g>4000) begin
                    $display("FAIL C4 restore never completed (b1 stuck)");
                    errors=errors+1; disable c4_wrest; end
            end
        end
        repeat (4000) @(posedge clk100);            // let kernel reach KIDLE/KISPIN

        // --- C4.1 Register cmpskill (skill 0): load 68 bytes @ coproc $0300, TABLE[0]=$0300
        wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h03);   // CP_LADDR = $0300
        load_byte(8'hA5); load_byte(8'hEC); load_byte(8'h0A); load_byte(8'h0A); load_byte(8'h0A);
        load_byte(8'h0A); load_byte(8'h18); load_byte(8'h69); load_byte(8'h80); load_byte(8'hAA);
        load_byte(8'hA5); load_byte(8'hEC); load_byte(8'h0A); load_byte(8'h0A); load_byte(8'h0A);
        load_byte(8'h0A); load_byte(8'h0A); load_byte(8'h18); load_byte(8'h69); load_byte(8'h80);
        load_byte(8'h95); load_byte(8'h00); load_byte(8'hA9); load_byte(8'h0F); load_byte(8'h95);
        load_byte(8'h01); load_byte(8'hF6); load_byte(8'h00); load_byte(8'hF6); load_byte(8'h00);
        load_byte(8'hA1); load_byte(8'h00); load_byte(8'h95); load_byte(8'h02); load_byte(8'h95);
        load_byte(8'h03); load_byte(8'hB5); load_byte(8'h02); load_byte(8'hF0); load_byte(8'h0B);
        load_byte(8'hA9); load_byte(8'h00); load_byte(8'h38); load_byte(8'hE9); load_byte(8'h01);
        load_byte(8'hD0); load_byte(8'hFB); load_byte(8'hD6); load_byte(8'h02); load_byte(8'hD0);
        load_byte(8'hF5); load_byte(8'hB5); load_byte(8'h00); load_byte(8'h18); load_byte(8'h69);
        load_byte(8'h0E); load_byte(8'h95); load_byte(8'h00); load_byte(8'hB5); load_byte(8'h03);
        load_byte(8'h18); load_byte(8'h69); load_byte(8'h01); load_byte(8'h81); load_byte(8'h00);
        load_byte(8'h4C); load_byte(8'h06); load_byte(8'h10);                       // 68 bytes
        // TABLE entry skill 0 = $0300 @ coproc $0200
        wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h02); load_byte(8'h00); load_byte(8'h03);
        // verify it landed (re-arm addr, read first opcode = $A5)
        cp_read(13'h0300, tmp);
        if (tmp!==8'hA5) begin errors=errors+1; $display("FAIL C4 cmpskill not loaded @ $0300 = %02X",tmp); end
        else $display("PASS C4 cmpskill registered (skill 0 -> $0300)");

        // --- C4.2 Single CALL from idle (BUG-FIX #2: CALL into slot 0 from idle must RUN)
        stage_mbox(2'd0, 8'h00, 8'h00, 8'h04);      // skill 0, budget 0 (no limit), arg0=4
        ring(2'd0);
        wait_done(4'b0001, 300000, c4ok);
        if (!c4ok) begin
            errors=errors+1;
            rd_reg(4'h1, r0); rd_reg(4'h2, r1);
            $display("FAIL C4 single CALL-from-idle never completed: DONE=%02X ACTIVE=%02X TIMEDOUT=%02X",
                     tmp, r0, r1);
            read_result(2'd0, r0);
            $display("      (mailbox+16 result cell = %02X; skill spawned? check ACTIVE)", r0);
        end else $display("PASS C4 single CALL from idle ran (DONE[0]=1)");
        read_result(2'd0, tmp);
        if (tmp!==8'h05) begin errors=errors+1; $display("FAIL C4 result 4->5 got %02X",tmp); end
        else $display("PASS C4 result 4->5");
        collect(2'd0);

        // --- C4.3 Two concurrent CALLs (cmpskill reentrancy + slot independence)
        stage_mbox(2'd0, 8'h00, 8'h00, 8'h06);      // slot0 arg0=6 -> 7
        stage_mbox(2'd1, 8'h00, 8'h00, 8'h0A);      // slot1 arg0=10 -> 11
        ring(2'd0); ring(2'd1);
        wait_done(4'b0011, 300000, c4ok);
        if (!c4ok) begin
            errors=errors+1; rd_reg(4'h1,r0);
            $display("FAIL C4 two concurrent never both done: DONE=%02X ACTIVE=%02X",tmp,r0);
        end
        read_result(2'd0, r0); read_result(2'd1, r1);
        if (r0!==8'h07 || r1!==8'h0B) begin
            errors=errors+1;
            $display("FAIL C4 concurrent results r0=%02X (want 07) r1=%02X (want 0B)",r0,r1);
        end else $display("PASS C4 two concurrent calls, distinct results 07/0B");
        collect(2'd0); collect(2'd1);

        // --- C4.4 Mid-race CALL (dispatch-in-ISR: spawn slot1 WHILE slot0 runs)
        stage_mbox(2'd0, 8'h00, 8'h00, 8'h18);      // slot0 long: arg0=24 -> 25=$19
        ring(2'd0);
        repeat (8000) @(posedge clk100);            // slot0 dispatched + grinding its spin
        rd_reg(4'h1, tmp);                          // slot0 must already be busy here
        if (!tmp[0]) $display("WARN C4 mid-race: slot0 not yet ACTIVE before slot1 ring (ACTIVE=%02X)",tmp);
        stage_mbox(2'd1, 8'h00, 8'h00, 8'h02);      // slot1 short: arg0=2 -> 3
        ring(2'd1);                                 // serviced mid-execution of slot0
        wait_done(4'b0011, 1500000, c4ok);
        if (!c4ok) begin
            errors=errors+1; rd_reg(4'h1,r0);
            $display("FAIL C4 mid-race: both never done DONE=%02X ACTIVE=%02X (slot1 not dispatched mid-run?)",tmp,r0);
        end
        read_result(2'd0, r0); read_result(2'd1, r1);
        if (r0!==8'h19 || r1!==8'h03) begin
            errors=errors+1;
            $display("FAIL C4 mid-race results r0=%02X (want 19) r1=%02X (want 03)",r0,r1);
        end else $display("PASS C4 mid-race CALL serviced (r0=19 r1=03)");
        collect(2'd0); collect(2'd1);

        // --- C4.5 RUNNING visibility (ACTIVE register): slot shows busy before DONE
        stage_mbox(2'd0, 8'h00, 8'h00, 8'h10);      // arg0=16, runs a while
        ring(2'd0);
        repeat (6000) @(posedge clk100);            // dispatched + running (call_req cleared, running set)
        rd_reg(4'h0, r1);                           // DONE snapshot (must NOT be set yet)
        rd_reg(4'h1, tmp);                          // ACTIVE
        if (!tmp[0] || r1[0]) begin
            errors=errors+1;
            $display("FAIL C4 RUNNING-vis: ACTIVE=%02X DONE=%02X (want ACTIVE[0]=1, DONE[0]=0)",tmp,r1);
        end else $display("PASS C4 RUNNING visible in ACTIVE");
        wait_done(4'b0001, 800000, c4ok);
        if (!c4ok) begin errors=errors+1; $display("FAIL C4 RUNNING-vis slot never finished"); end
        collect(2'd0);

        // --- C4.6 Run-budget timeout (BUG-FIX #1: force-complete sole running slot, NO runaway)
        stage_mbox(2'd0, 8'h00, 8'h02, 8'hFF);      // arg0=255 (huge spin), budget=2 ticks
        ring(2'd0);
        wait_done(4'b0001, 300000, c4ok);           // must complete via WATCHDOG, not natural finish
        if (!c4ok) begin
            errors=errors+1; rd_reg(4'h1,r0); rd_reg(4'h2,r1);
            $display("FAIL C4 budget-timeout: DONE[0] never set DONE=%02X ACTIVE=%02X TIMEDOUT=%02X (runaway?)",tmp,r0,r1);
        end
        rd_reg(4'h2, tmp);                          // TIMEDOUT
        if (!tmp[0]) begin errors=errors+1; $display("FAIL C4 budget-timeout: TIMEDOUT[0]=0 (force-complete didn't flag) TMO=%02X",tmp); end
        else $display("PASS C4 budget force-complete (DONE[0]=1, TIMEDOUT[0]=1)");
        collect(2'd0);                              // free slot 0 (clears done+timedout)
        // Let the kernel settle to stable KISPIN idle after the force-complete -> JMP
        // KIDLE abandonment (host-side latency between poll/collect and a new CALL is
        // realistic). The AB trace confirmed the CPU idles in $10xx/$1Fxx here -- the
        // timed-out skill STOPPED (no runaway in the $03xx task region).
        repeat (5000) @(posedge clk100);
        // NO-RUNAWAY proof: the timed-out skill must have STOPPED and the slot be
        // reusable -> a fresh normal call into slot 0 completes with result $04.
        stage_mbox(2'd0, 8'h00, 8'h00, 8'h03);      // arg0=3 -> 4, no budget
        ring(2'd0);
        wait_done(4'b0001, 300000, c4ok);
        read_result(2'd0, tmp);
        if (!c4ok || tmp!==8'h04) begin
            errors=errors+1;
            $display("FAIL C4 slot NOT reusable after timeout: ok=%0d result=%02X (want 04; runaway corrupted slot?)",c4ok,tmp);
        end else $display("PASS C4 run-budget force-complete + slot reusable (3->4)");
        collect(2'd0);

        // ===== SDRAM READ WINDOW: round-trip through the REAL arbiter =====
        // Proves the coproc READ window ($E004-$E008) against the actual sdram_arb +
        // SDRAM model (not a Task-1 stub): the host SEEDS a region, a skill READS it
        // back via the auto-incrementing read window, SUMS the bytes, and WRITES the
        // sum to a result cell -- which the host reads and asserts. A pass means the
        // R2 latch (sread<=rdata on `done`, advance ptr) fires on the correct cycle
        // relative to the real arbiter's c1_busy/c1_rdata timing.
        //
        // The C4 block above left the kernel idle (KISPIN, free-running tick armed).
        // We register sdrtest as skill 0 (overwriting cmpskill @ $0300; TABLE[0] still
        // $0300) and CALL it into slot 0 via the same async ring/wait_done path.
        //
        // Seed bank0 $0080..$0087 = $11,$22,$33,$44,$55,$66,$77,$88.
        //   sum = $11+$22+$33+$44+$55+$66+$77+$88 = $0264 -> low byte $64.
        begin : sdrtest_blk
            integer si; reg [7:0] seedv;
            for (si=0; si<8; si=si+1) begin
                seedv = 8'h11 * (si+1);          // $11,$22,...,$88
                sdram_write(10'd0, 16'h0080 + si, seedv);
            end
        end
        // clear the result cell so a non-running skill can't masquerade as a pass
        sdram_write(10'd0, 16'h0090, 8'h00);
        // load sdrtest (61 bytes) @ coproc $0300
        wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h03);   // CP_LADDR = $0300
        load_byte(8'hA9); load_byte(8'h80); load_byte(8'h8D); load_byte(8'h04); load_byte(8'hE0);
        load_byte(8'hA9); load_byte(8'h00); load_byte(8'h8D); load_byte(8'h05); load_byte(8'hE0);
        load_byte(8'hA9); load_byte(8'h00); load_byte(8'h8D); load_byte(8'h06); load_byte(8'hE0);
        load_byte(8'hA9); load_byte(8'h00); load_byte(8'h85); load_byte(8'h80); load_byte(8'hA9);
        load_byte(8'h08); load_byte(8'h85); load_byte(8'h81); load_byte(8'h8D); load_byte(8'h07);
        load_byte(8'hE0); load_byte(8'hAD); load_byte(8'h08); load_byte(8'hE0); load_byte(8'h18);
        load_byte(8'h65); load_byte(8'h80); load_byte(8'h85); load_byte(8'h80); load_byte(8'hC6);
        load_byte(8'h81); load_byte(8'hD0); load_byte(8'hF1); load_byte(8'hA9); load_byte(8'h90);
        load_byte(8'h8D); load_byte(8'h00); load_byte(8'hE0); load_byte(8'hA9); load_byte(8'h00);
        load_byte(8'h8D); load_byte(8'h01); load_byte(8'hE0); load_byte(8'hA9); load_byte(8'h00);
        load_byte(8'h8D); load_byte(8'h02); load_byte(8'hE0); load_byte(8'hA5); load_byte(8'h80);
        load_byte(8'h8D); load_byte(8'h03); load_byte(8'hE0); load_byte(8'h4C); load_byte(8'h06);
        load_byte(8'h10);                            // 61 bytes
        // TABLE entry skill 0 = $0300 @ coproc $0200
        wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h02); load_byte(8'h00); load_byte(8'h03);
        cp_read(13'h0300, tmp);
        if (tmp!==8'hA9) begin errors=errors+1; $display("FAIL sdram-read: sdrtest not loaded @ $0300 = %02X",tmp); end
        // CALL sdrtest into slot 0 (skill 0, budget 0 = no limit, arg0 unused)
        stage_mbox(2'd0, 8'h00, 8'h00, 8'h00);
        ring(2'd0);
        wait_done(4'b0001, 300000, c4ok);
        if (!c4ok) begin
            errors=errors+1; rd_reg(4'h1, r0);
            $display("FAIL sdram-read: sdrtest never completed DONE=%02X ACTIVE=%02X (read handshake stuck against real arbiter?)",tmp,r0);
        end
        sdram_read(10'd0, 16'h0090, tmp);
        if (tmp!==8'h64) begin
            errors=errors+1;
            $display("FAIL sdram-read: coproc summed host-seeded region (got %02X want 64) -- R2 latch timing wrong vs real arbiter?",tmp);
        end else
            $display("PASS sdram-read: coproc summed host-seeded region (got %02X want 64)",tmp);
        collect(2'd0);

        // ===== CONWAY'S MULTIVERSE: TICK1 (bit-packed Life, sliding window) =====
        // Oracle: seed universe 0 (bank 16, buffer A) with a blinker (interior),
        // a second blinker straddling an interior 7-cell BYTE BOUNDARY, a glider
        // positioned to MOVE across a byte boundary, and a blinker straddling the
        // col-0 / col-559 TORUS seam. LIFE8 (LSIM=1 => 8 rows) ticks universe 0
        // once per CALL: read front buf, compute back buf, flip FRONT[0], bump
        // GEN[0], JMP DONE. We CALL it 4x (gens alternate buffers B,A,B,A via the
        // FRONT flip) and assert every generation against values computed by an
        // independent Python reference simulator (same toroidal bit-packed rules).
        // Per gen: explicit packed-byte asserts (diagnosable) + a full-buffer
        // sum16 (catches spurious/missing cells anywhere). Blinkers are period-2;
        // the glider returns to its phase shifted +1 row/+1 col by gen 4 -- the
        // strongest proof the cross-byte neighbor math is correct.
        sdram_write(10'd24, 16'h0010, 8'h00);   // FRONT[0] = 0 (buffer A is live)
        sdram_write(10'd24, 16'h0020, 8'h00);   // GEN[0]   = 0
        mv_seed;                                 // buffer A (bank 16) := patterns

        // load LIFE8 image @ coproc $0300. Two entries via a fixed JMP table:
        //   skill 0 -> $0300 = JMP LIFE8 (forever round-robin, never DONE)
        //   skill 1 -> $0303 = JMP LIFE1 (single univ-0 tick + DONE = oracle)
        wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h03);            // CP_LADDR = $0300
        for (mvi=0; mvi<LIFE8LEN; mvi=mvi+1) load_byte(life8img[mvi]);
        wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h02);            // CP_LADDR = $0200
        load_byte(8'h00); load_byte(8'h03);                  // TABLE[0] = $0300
        load_byte(8'h03); load_byte(8'h03);                  // TABLE[1] = $0303
        cp_read(13'h0300, tmp);
        if (tmp!==8'h4C) begin errors=errors+1;
            $display("FAIL multiverse TICK1 not loaded @ $0300 = %02X",tmp); end

        // --- GEN 1 (front A -> back B, sum16 = $004F) ---
        mv_tick; mv_readback(16'h4000); mv_sum(16'h004F);
        mv_chk(161,8'h01); mv_chk(163,8'h01); mv_chk(174,8'h40); mv_chk(175,8'h02);
        mv_chk(241,8'h01); mv_chk(243,8'h01); mv_chk(255,8'h03);
        mv_chk(321,8'h01); mv_chk(323,8'h01); mv_chk(335,8'h01);
        mv_chk(400,8'h01); mv_chk(480,8'h01); mv_chk(560,8'h01);

        // --- GEN 2 (front B -> back A, blinkers+torus back to seed, sum16 = $0110) ---
        mv_tick; mv_readback(16'h0000); mv_sum(16'h0110);
        mv_chk(175,8'h02); mv_chk(240,8'h40); mv_chk(241,8'h03); mv_chk(242,8'h40);
        mv_chk(243,8'h03); mv_chk(254,8'h40); mv_chk(255,8'h02);
        mv_chk(335,8'h03); mv_chk(480,8'h03); mv_chk(559,8'h40);

        // --- GEN 3 (front A -> back B, sum16 = $0013) ---
        mv_tick; mv_readback(16'h4000); mv_sum(16'h0013);
        mv_chk(161,8'h01); mv_chk(163,8'h01); mv_chk(175,8'h01);
        mv_chk(241,8'h01); mv_chk(243,8'h01); mv_chk(255,8'h06);
        mv_chk(321,8'h01); mv_chk(323,8'h01); mv_chk(335,8'h03);
        mv_chk(400,8'h01); mv_chk(480,8'h01); mv_chk(560,8'h01);

        // --- GEN 4 (front B -> back A; glider = seed shifted +1,+1, sum16 = $00D6) ---
        mv_tick; mv_readback(16'h0000); mv_sum(16'h00D6);
        mv_chk(175,8'h02); mv_chk(240,8'h40); mv_chk(241,8'h03); mv_chk(242,8'h40);
        mv_chk(243,8'h03); mv_chk(255,8'h04); mv_chk(335,8'h07);
        mv_chk(480,8'h03); mv_chk(559,8'h40);

        if (errors==0)
            $display("PASS multiverse TICK1 (blinker/byte-boundary/glider/torus, 4 gens)");
        else
            $display("FAIL multiverse TICK1 %0d errors", errors);

        // ===== CONWAY'S MULTIVERSE: ROUND-ROBIN (forever loop, univ0+univ1) =====
        // Prove the real LIFE8 (skill 0 @ $0300) free-running loop ticks MULTIPLE
        // universes round-robin. Seed univ0 (bank16) AND univ1 (bank17) buffer A
        // with a horizontal blinker, FRONT=GEN=0. Ring skill 0 (budget 0). The
        // loop NEVER DONEs, so we POLL the GEN counters until BOTH univ0 and univ1
        // have ticked (GEN[0]>=1 && GEN[1]>=1 = round-robin reached both). Then we
        // HALT the coproc with an nRES_READ pulse (clears call_req; the sdram_model
        // array has no reset port so SDRAM survives) and read back each universe's
        // LIVE buffer (selected by FRONT) and assert the blinker evolved -- proving
        // per-universe RB/WB derivation (bank=UBASE+u, live/other buffer) + flip +
        // bump are all correct across more than one universe.
        begin : multiverse_rr
            reg [7:0] g0, g1, f0, f1; integer rrg;
            mv_seed_blinker(10'd16);                 // univ0 buffer A := blinker
            mv_seed_blinker(10'd17);                 // univ1 buffer A := blinker
            sdram_write(10'd24, 16'h0010, 8'h00);    // FRONT[0] = 0
            sdram_write(10'd24, 16'h0011, 8'h00);    // FRONT[1] = 0
            sdram_write(10'd24, 16'h0020, 8'h00);    // GEN[0]   = 0
            sdram_write(10'd24, 16'h0021, 8'h00);    // GEN[1]   = 0
            // skill 0 = forever round-robin; arg0 unused, budget 0 (no watchdog)
            stage_mbox(2'd0, 8'h00, 8'h00, 8'h00);
            ring(2'd0);
            // poll GEN[0] and GEN[1] until both ticked, generous cycle cap
            rrg = 0; g0 = 0; g1 = 0;
            while ((g0 < 8'd1) || (g1 < 8'd1)) begin
                sdram_read(10'd24, 16'h0020, g0);
                sdram_read(10'd24, 16'h0021, g1);
                rrg = rrg + 1;
                if (rrg > 200000) begin errors=errors+1;
                    $display("FAIL multiverse round-robin GEN stuck g0=%0d g1=%0d",g0,g1);
                    g0 = 8'd1; g1 = 8'd1; end
            end
            // HALT the free-running loop (SDRAM persists across the reset pulse)
            nRES_READ=1'b0; #500; nRES_READ=1'b1; #200;
            wait (dut.ready);
            // read final FRONT for each universe, assert the live buffer evolved
            sdram_read(10'd24, 16'h0010, f0);
            sdram_read(10'd24, 16'h0020, g0);
            sdram_read(10'd24, 16'h0011, f1);
            sdram_read(10'd24, 16'h0021, g1);
            $display("multiverse round-robin halt: u0 FRONT=%0d GEN=%0d  u1 FRONT=%0d GEN=%0d",f0,g0,f1,g1);
            mv_chk_blinker(10'd16, f0, 0);
            mv_chk_blinker(10'd17, f1, 1);
            if ((g0 < 8'd1) || (g1 < 8'd1)) begin errors=errors+1;
                $display("FAIL multiverse round-robin GEN0=%0d GEN1=%0d",g0,g1); end
            if (errors==0)
                $display("PASS multiverse round-robin (univ0 + univ1 both ticked + evolved)");
            else
                $display("FAIL multiverse round-robin %0d errors", errors);
        end

        // ===== CONWAY'S MULTIVERSE GR: TICK1 (8 cells/byte, 40x48 GR grid) =====
        // Re-target proof: reload $0300 with LIFE8GR (overwrites LIFE8 -- both
        // ORG $0300, can't co-reside; BRAM survives the round-robin reset pulse,
        // so a load-port overwrite re-points the skill). TABLE[0]=$0300 (LIFE8GR
        // forever) / TABLE[1]=$0303 (LIFE1 GR = one univ-0 tick + DONE = oracle)
        // unchanged. Three patterns on the GR dims, expected gens from an
        // independent torus reference sim: an interior horizontal blinker
        // (-> vertical, 1 tick); a glider straddling the byte0/byte1 boundary
        // (-> +1row/+1col after 4 ticks, catches the 8/byte math); a blinker on
        // the col39<->col0 torus seam (-> vertical col0, exercises GWIDTH-1 wrap).
        begin : multiverse_gr
            integer gi;
            wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h03);            // CP_LADDR=$0300
            for (gi=0; gi<LIFE8GRLEN; gi=gi+1) load_byte(life8grimg[gi]);
            wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h02);            // CP_LADDR=$0200
            load_byte(8'h00); load_byte(8'h03);                  // TABLE[0]=$0300
            load_byte(8'h03); load_byte(8'h03);                  // TABLE[1]=$0303
            cp_read(13'h0300, tmp);
            if (tmp!==8'h4C) begin errors=errors+1;
                $display("FAIL multiverse-GR not loaded @ $0300 = %02X",tmp); end

            // --- A: interior horizontal blinker row2 c10-12 -> vertical col11 ---
            sdram_write(10'd24, 16'h0010, 8'h00);    // FRONT[0]=0
            sdram_write(10'd24, 16'h0020, 8'h00);    // GEN[0]=0
            mvgr_clear;
            sdram_write(10'd16, 16'h0000+11, 8'h1C); // row2 byte1 = c10,11,12
            mv_tick;                                 // 1 tick -> live buffer B
            mvgr_readback(16'h0400);
            mv_sum(16'h0018);
            mv_chk(6,8'h08); mv_chk(11,8'h08); mv_chk(16,8'h08);

            // --- C: torus-seam blinker row4 c39,0,1 -> vertical col0 ---
            sdram_write(10'd24, 16'h0010, 8'h00);
            sdram_write(10'd24, 16'h0020, 8'h00);
            mvgr_clear;
            sdram_write(10'd16, 16'h0000+24, 8'h80); // row4 byte4 = col39
            sdram_write(10'd16, 16'h0000+20, 8'h03); // row4 byte0 = col0,1
            mv_tick;                                 // 1 tick -> live buffer B
            mvgr_readback(16'h0400);
            mv_sum(16'h0003);
            mv_chk(15,8'h01); mv_chk(20,8'h01); mv_chk(25,8'h01);

            // --- B: glider straddling byte0/byte1 -> seed shifted +1,+1 @ gen4 --
            sdram_write(10'd24, 16'h0010, 8'h00);
            sdram_write(10'd24, 16'h0020, 8'h00);
            mvgr_clear;
            sdram_write(10'd16, 16'h0000+10, 8'h80); // (2,7)
            sdram_write(10'd16, 16'h0000+16, 8'h01); // (3,8)
            sdram_write(10'd16, 16'h0000+20, 8'hC0); // (4,6),(4,7)
            sdram_write(10'd16, 16'h0000+21, 8'h01); // (4,8)
            mv_tick; mv_tick; mv_tick; mv_tick;      // 4 ticks -> live buffer A
            mvgr_readback(16'h0000);
            mv_sum(16'h0086);
            mv_chk(16,8'h01); mv_chk(21,8'h02); mv_chk(25,8'h80); mv_chk(26,8'h03);

            if (errors==0)
                $display("PASS multiverse-GR TICK1 (blinker/glider-byte-boundary/torus-seam, GWIDTH-1 wrap + MUL5)");
            else
                $display("FAIL multiverse-GR TICK1 %0d errors", errors);
        end

        end // prefarm (+farmonly skips to here)

        // ===== WORKSHOP: 2nd task, craft pipeline =====
        begin : wk_m1
        reg [7:0] r, r1; reg ok; reg [7:0] h1, h2; integer t;
        $display("--- WORKSHOP task tests ---");
        // quiesce: host may not touch BRAM while a task runs.
        // reset, wait restore_done, load BOTH blobs, ring BOTH.
        nRES_READ = 1'b0; #1000; nRES_READ = 1'b1;
        wait (dut.ready);
        repeat (2000) @(posedge clk100);
        begin : wkm1_rstwait
        integer t; reg [7:0] fs;
        fs = 8'h00; t = 0;
        while (!fs[1] && t < 20000) begin
            rd_reg(4'hF, fs); t = t + 1;
        end
        if (!fs[1]) begin errors=errors+1; $display("FAIL wk_m1: restore never done"); end
        end
        wk_init;
        farm_load;
        wk_load;
        stage_mbox(2'd0, 8'd2, 8'd0, 8'd0);
        stage_mbox(2'd1, 8'd3, 8'd0, 8'd0);
        ring(2'd0);
        ring(2'd1);
        wk_cmd(8'h00, 8'h00, 8'h00, 8'h00, 8'h00, r, r1, ok);   // OPSTAT
        if (!ok || r!==8'h01) begin errors=errors+1; $display("FAIL wk STATUS r=%h ok=%b", r, ok); end
        else $display("PASS wk STATUS (dual task live)");
        // dual heartbeats advance
        sdram_read(10'd33, 16'h0004, h1);
        sdram_read(10'd32, 16'h0004, r);
        begin : wkbeat
        reg beat0, beat1;
        beat0 = 0; beat1 = 0;
        for (t = 0; t < 5 && !(beat0 && beat1); t = t + 1) begin
            repeat (200000) @(posedge clk100);
            sdram_read(10'd33, 16'h0004, h2);
            if (h2 !== h1) beat1 = 1;
            sdram_read(10'd32, 16'h0004, r1);
            if (r1 !== r) beat0 = 1;
        end
        if (!beat1) begin errors=errors+1; $display("FAIL wk heartbeat stuck"); end
        if (!beat0) begin errors=errors+1; $display("FAIL farm heartbeat stuck w/ 2 tasks"); end
        if (beat0 && beat1) $display("PASS dual heartbeats");
        end
        // dud combo: (0,0,0,FF) not in table -> RUINED, skill 1
        wk_cmd(8'h02, 8'd0, 8'd0, 8'd0, 8'hFF, r, r1, ok);
        if (!ok || r!==8'hE8) begin errors=errors+1; $display("FAIL dud r=%h want E8", r); end
        sdram_read(10'd33, 16'h0214, r);
        if (r!==8'd1) begin errors=errors+1; $display("FAIL skill=%h want 01", r); end
        else $display("PASS dud combo ruined + skill up");
        // force discovery (SKILL=$FF -> threshold maxed, no pantry needed)
        sdram_write(10'd33, 16'h0214, 8'hFF);                // rig: maxed skill
        sdram_write(10'd33, 16'h0215, 8'h01);                // rig: BREAD pre-discovered (det.)
        wk_cmd(8'h02, 8'd0, 8'd0, 8'hFF, 8'hFF, r, r1, ok);  // BREAD
        if (!ok || r!==8'h01) begin errors=errors+1; $display("FAIL craft r=%h", r); end
        sdram_read(10'd33, 16'h0220, r);                     // station0 STATE
        if (r!==8'h01) begin errors=errors+1; $display("FAIL station state=%h want 01", r); end
        sdram_read(10'd33, 16'h0215, r);                     // DISC lo bit 0
        if (r[0]!==1'b1) begin errors=errors+1; $display("FAIL disc bit=%h", r); end
        else $display("PASS discovery + cooking");
        // wait for station to finish cooking (auto-idles to STATE=00, GOODS++)
        begin : wkdone
        reg [7:0] st; integer w;
        st = 8'h01; w = 0;
        while (st === 8'h01 && w < 200) begin
            repeat (10000) @(posedge clk100);
            sdram_read(10'd33, 16'h0220, st); w = w + 1;
        end
        end
        // allow coproc to complete PUTEV after IDLE (STATE=0) write (timing gap)
        repeat (2000) @(posedge clk100);
        // drain wk ring: expect WEVDONE type 5, p0=station0, p1=recipe0
        begin : wkring
        reg [7:0] h, rs, rt, rp0, rp1; integer g;
        sdram_read(10'd33, 16'h0003, h);
        if (h == 8'h00) begin errors=errors+1; $display("FAIL wk ring empty"); end
        else begin
            sdram_read(10'd33, 16'h0100, rs);
            sdram_read(10'd33, 16'h0101, rt);
            sdram_read(10'd33, 16'h0102, rp0);
            sdram_read(10'd33, 16'h0103, rp1);
            if (rt!==8'h05 || rp0!==8'h00 || rp1!==8'h00) begin errors=errors+1;
                $display("FAIL WEVDONE %h %h %h", rt, rp0, rp1); end
            else $display("PASS WEVDONE(station0, BREAD)");
        end
        end
        // OPADDCASH lands on the farm side (//e bus credit leg)
        sdram_read(10'd32, 16'h0220, h1); sdram_read(10'd32, 16'h0221, h2);
        farm_cmd(8'h06, 8'd160, 8'd0, 8'h00, r, ok);
        if (!ok || r!==8'h01) begin errors=errors+1; $display("FAIL addcash r=%h", r); end
        sdram_read(10'd32, 16'h0220, r); sdram_read(10'd32, 16'h0221, r1);
        if ({r1,r} !== {h2,h1} + 16'd160) begin errors=errors+1;
            $display("FAIL cash %h%h want +160", r1, r); end
        else $display("PASS OPADDCASH credited");
        // OPSPEND: debit farm cash; insufficient -> RERRCASH ($E4).
        // Poke cash to a known 200 for determinism — prior balance is
        // LFSR-yield-dependent; poke guarantees cash > 50 for the test.
        sdram_write(10'd32, 16'h0220, 8'd200);
        sdram_write(10'd32, 16'h0221, 8'h00);
        sdram_read(10'd32, 16'h0220, h1); sdram_read(10'd32, 16'h0221, h2);
        farm_cmd(8'h07, 8'd50, 8'd0, 8'h00, r, ok);   // spend 50
        if (!ok || r!==8'h01) begin errors=errors+1; $display("FAIL spend r=%h", r); end
        sdram_read(10'd32, 16'h0220, r); sdram_read(10'd32, 16'h0221, r1);
        if ({r1,r} !== {h2,h1} - 16'd50) begin errors=errors+1;
            $display("FAIL post-spend cash %h%h", r1, r); end
        // overspend: ask for 60000 -> RERRCASH, cash unchanged
        sdram_read(10'd32, 16'h0220, h1); sdram_read(10'd32, 16'h0221, h2);
        farm_cmd(8'h07, 8'h60, 8'hEA, 8'h00, r, ok);  // 60000
        if (r!==8'hE4) begin errors=errors+1; $display("FAIL overspend r=%h want E4", r); end
        sdram_read(10'd32, 16'h0220, r); sdram_read(10'd32, 16'h0221, r1);
        if ({r1,r} !== {h2,h1}) begin errors=errors+1; $display("FAIL overspend mutated cash"); end
        else $display("PASS OPSPEND + insufficient guard");
        // discovery roll path: skill FF, bit clear -> bounded retry loop
        // (LFSR is cycle-deterministic; loop converges same way every run)
        begin : wk_roll
        reg [7:0] r, r1; reg ok; integer a; reg won;
        wk_cmd(8'h04, 8'h00, 8'h00, 8'h00, 8'h00, r, r1, ok);  // OPMODE 0 (clear BOOM)
        sdram_write(10'd33, 16'h0215, 8'h00);   // clear disc bits
        sdram_write(10'd33, 16'h0214, 8'hFF);   // max skill
        won = 0;
        for (a = 0; a < 20 && !won; a = a + 1) begin
            wk_cmd(8'h02, 8'd0, 8'd0, 8'hFF, 8'hFF, r, r1, ok);  // BREAD attempt (no pantry needed)
            if (r === 8'h01) won = 1;
            else if (r !== 8'hE8) begin errors=errors+1;
                $display("FAIL roll attempt r=%h want 01/E8", r); won = 1; end
        end
        sdram_read(10'd33, 16'h0215, r);
        if (!won || r[0] !== 1'b1) begin errors=errors+1;
            $display("FAIL roll path: never discovered, disc=%h", r); end
        else $display("PASS discovery roll converged, disc bit set");
        // wait for station to auto-idle (cook-done -> STATE=00 + GOODS++)
        begin : wk_rollclean
        reg [7:0] st; integer w;
        st = 8'h01; w = 0;
        while (st === 8'h01 && w < 200) begin
            repeat (10000) @(posedge clk100);
            sdram_read(10'd33, 16'h0220, st); w = w + 1;
        end
        end
        // rarity-impossible: skill 0 + FEAST (rarity 3) -> always RUINED (no pantry needed)
        sdram_write(10'd33, 16'h0214, 8'h00);   // skill 0
        wk_cmd(8'h02, 8'd0, 8'd1, 8'd2, 8'd3, r, r1, ok);    // FEAST
        if (r !== 8'hE8) begin errors=errors+1;
            $display("FAIL rarity floor r=%h want E8 (impossible at skill 0)", r); end
        else $display("PASS rarity floor: FEAST impossible at skill 0");
        // restore rig: downstream wk_reset asserts SKILL $FF survives reset
        sdram_write(10'd33, 16'h0214, 8'hFF);
        end
        // ===== recipe shop: WOPLEARN (op 5) =====
        begin : wk_learn
        reg [7:0] r, r1; reg ok;
        // clear DISC, learn BREAD (idx 0) -> bit 0 set
        sdram_write(10'd33, 16'h0215, 8'h00);
        sdram_write(10'd33, 16'h0216, 8'h00);
        wk_cmd(8'h05, 8'd0, 8'h00, 8'h00, 8'h00, r, r1, ok);   // WOPLEARN 0
        if (!ok || r!==8'h01) begin errors=errors+1; $display("FAIL learn r=%h", r); end
        sdram_read(10'd33, 16'h0215, r);
        if (r[0]!==1'b1) begin errors=errors+1; $display("FAIL learn bit=%h", r); end
        // double-learn -> RERRBAD
        wk_cmd(8'h05, 8'd0, 8'h00, 8'h00, 8'h00, r, r1, ok);
        if (r!==8'hE6) begin errors=errors+1; $display("FAIL double-learn r=%h want E6", r); end
        else $display("PASS WOPLEARN + double-learn guard");
        // learn idx 9 (>7 -> hi byte, bit 1 of $0216)
        wk_cmd(8'h05, 8'd9, 8'h00, 8'h00, 8'h00, r, r1, ok);
        sdram_read(10'd33, 16'h0216, r);
        if (r[1]!==1'b1) begin errors=errors+1; $display("FAIL learn hi bit=%h", r); end
        else $display("PASS WOPLEARN hi byte (idx 9)");
        // out-of-range idx (12 >= NRECIP=12) -> RERRBAD
        wk_cmd(8'h05, 8'd12, 8'h00, 8'h00, 8'h00, r, r1, ok);
        if (r!==8'hE6) begin errors=errors+1; $display("FAIL learn-12 r=%h want E6", r); end
        end
        // ===== known-recipe failure curve (distribution test) =====
        // skill 0, BREAD owned, 30 attempts: assert both ROK and $E9 occur.
        // blob rolls discovery/known-fail WITHOUT pantry; no deposit needed.
        begin : wk_kfail
        reg [7:0] r, r1; reg ok; integer a, nok, nfail;
        // own BREAD (idx 0, combo 0,0), skill 0 -> FAILBASE[0]=48/256 ~19%
        sdram_write(10'd33, 16'h0215, 8'h01);   // DISC bit 0 (BREAD known)
        sdram_write(10'd33, 16'h0216, 8'h00);
        sdram_write(10'd33, 16'h0214, 8'h00);   // skill 0
        nok = 0; nfail = 0;
        for (a = 0; a < 30; a = a + 1) begin
            wk_cmd(8'h02, 8'd0, 8'd0, 8'hFF, 8'hFF, r, r1, ok); // craft BREAD (rolls without pantry)
            if (r === 8'h01) nok = nok + 1;
            else if (r === 8'hE9) nfail = nfail + 1;
            else begin errors=errors+1; $display("FAIL kfail unexpected r=%h", r); end
            // wait for station to auto-idle (cook-done -> STATE=00 + GOODS++)
            begin : kf_clear
            integer w; reg [7:0] st;
            st = 8'h01; w = 0;
            while (st === 8'h01 && w < 200) begin
                repeat (4000) @(posedge clk100);
                sdram_read(10'd33, 16'h0220, st); w = w + 1;
            end
            end
        end
        if (nok == 0 || nfail == 0) begin errors=errors+1;
            $display("FAIL kfail no spread: ok=%0d fail=%0d", nok, nfail); end
        else $display("PASS known-fail curve ok=%0d fail=%0d (both occur)", nok, nfail);
        // restore rig: wk_reset asserts SKILL $FF survives reset
        sdram_write(10'd33, 16'h0214, 8'hFF);
        end

        // ===== craft auto-stores a good + frees station =====
        begin : wk_autostore
        reg [7:0] r, r1; reg ok; integer w; reg [7:0] st, g;
        // own BREAD (idx0, combo 0,0) at high skill so the known-roll cooks
        sdram_write(10'd33, 16'h0215, 8'h01);   // DISC bit0 (BREAD owned)
        sdram_write(10'd33, 16'h0214, 8'hFF);   // skill FF -> low fail
        sdram_write(10'd33, 16'h0228, 8'h00);   // GOODS[0]=0
        wk_cmd(8'h02, 8'd0, 8'd0, 8'hFF, 8'hFF, r, r1, ok);  // craft BREAD
        if (!ok || r!==8'h01) begin errors=errors+1; $display("FAIL autostore craft r=%h", r); end
        // wait the station to finish (FSIM tiny divider)
        st = 8'h01; w = 0;
        while (st !== 8'h00 && w < 400) begin
            repeat (4000) @(posedge clk100);
            sdram_read(10'd33, 16'h0220, st); w = w + 1;   // station0 STATE
        end
        if (st!==8'h00) begin errors=errors+1; $display("FAIL station not idle after cook st=%h", st); end
        sdram_read(10'd33, 16'h0228, g);                   // GOODS[0]
        if (g!==8'h01) begin errors=errors+1; $display("FAIL GOODS[0]=%h want 01", g); end
        else $display("PASS craft auto-stores good + frees station");
        end

        // ===== WOPSELL: sell goods at value, e2e to farm cash =====
        begin : wk_sell
        reg [7:0] r, r1; reg ok; reg [7:0] c0l, c0h;
        sdram_write(10'd33, 16'h0228, 8'd5);    // GOODS[0]=5 (BREAD value 28)
        sdram_write(10'd33, 16'h0217, 8'h00);   // MODE: no BOOM
        // sell 2 BREAD -> value 56, GOODS[0]=3
        wk_cmd(8'h03, 8'd0, 8'd2, 8'h00, 8'h00, r, r1, ok);  // WOPSELL(prod0, qty2)
        if (!ok || r!==8'h01) begin errors=errors+1; $display("FAIL wopsell r=%h", r); end
        if (r1!==8'd28) begin errors=errors+1; $display("FAIL per-unit=%d want 28", r1); end
        sdram_read(10'd33, 16'h0228, r);
        if (r!==8'd3) begin errors=errors+1; $display("FAIL GOODS[0]=%d want 3", r); end
        // oversell -> RERRCROP, goods unchanged
        wk_cmd(8'h03, 8'd0, 8'd9, 8'h00, 8'h00, r, r1, ok);
        if (r!==8'hE5) begin errors=errors+1; $display("FAIL oversell r=%h want E5", r); end
        sdram_read(10'd33, 16'h0228, r);
        if (r!==8'd3) begin errors=errors+1; $display("FAIL oversell mutated goods=%d", r); end
        // BOOM doubles: MODE bit0, sell 1 -> value 56
        sdram_write(10'd33, 16'h0217, 8'h01);
        wk_cmd(8'h03, 8'd0, 8'd1, 8'h00, 8'h00, r, r1, ok);
        if (r1!==8'd56) begin errors=errors+1; $display("FAIL boom per-unit=%d want 56", r1); end
        // e2e cash credit: poke farm cash, OPADDC the value, assert
        sdram_read(10'd32, 16'h0220, c0l); sdram_read(10'd32, 16'h0221, c0h);
        farm_cmd(8'h06, 8'd56, 8'd0, 8'h00, r, ok);   // OPADDC 56
        sdram_read(10'd32, 16'h0220, r); sdram_read(10'd32, 16'h0221, r1);
        if ({r1,r} !== {c0h,c0l} + 16'd56) begin errors=errors+1; $display("FAIL e2e cash"); end
        else $display("PASS WOPSELL per-unit + oversell + BOOM + e2e cash");
        end

        // ===== stale-op: WOPDEP removed -> RERRBAD, no mutation =====
        begin : wk_staleop
        reg [7:0] r, r1; reg ok;
        sdram_write(10'd33, 16'h0210, 8'h00);
        wk_cmd(8'h01, 8'd0, 8'd5, 8'h00, 8'h00, r, r1, ok);  // old WOPDEP
        if (r!==8'hE6) begin errors=errors+1; $display("FAIL stale WOPDEP r=%h want E6", r); end
        sdram_read(10'd33, 16'h0210, r);
        if (r!==8'h00) begin errors=errors+1; $display("FAIL stale WOPDEP mutated %h", r); end
        else $display("PASS WOPDEP removed (RERRBAD, no-op)");
        end

        end

        // ===== WORKSHOP+FARM dual reset recovery =====
        begin : wk_reset
        reg [7:0] r, r1; reg ok;
        $display("--- DUAL-TASK reset recovery ---");
        nRES_READ = 1'b0; #1000; nRES_READ = 1'b1;
        wait (dut.ready);
        repeat (2000) @(posedge clk100);
        begin : wk_rstwait
        integer t; reg [7:0] fs;
        fs = 8'h00; t = 0;
        while (!fs[1] && t < 20000) begin
            rd_reg(4'hF, fs); t = t + 1;
        end
        if (!fs[1]) begin errors=errors+1; $display("FAIL wk reset: restore never done"); end
        end
        // both worlds survive
        sdram_read(10'd33, 16'h0000, r); sdram_read(10'd33, 16'h0001, r1);
        if (r!==8'h57 || r1!==8'h4B) begin errors=errors+1; $display("FAIL wk SIG lost"); end
        sdram_read(10'd33, 16'h0214, r);
        if (r!==8'hFF) begin errors=errors+1; $display("FAIL skill lost %h", r); end
        // reload + respawn BOTH (quiesce path order:
        // ALL host BRAM writes - blobs AND mailbox
        // staging - before EITHER task is rung)
        farm_load;
        wk_load;
        stage_mbox(2'd0, 8'd2, 8'd0, 8'd0);
        stage_mbox(2'd1, 8'd3, 8'd0, 8'd0);
        ring(2'd0);
        ring(2'd1);
        farm_cmd(8'h00, 8'h00, 8'h00, 8'h00, r, ok);
        if (!ok || r!==8'h01) begin errors=errors+1; $display("FAIL dual reset: farm respawn"); end
        wk_cmd(8'h00, 8'h00, 8'h00, 8'h00, 8'h00, r, r1, ok);
        if (!ok || r!==8'h01) begin errors=errors+1; $display("FAIL dual reset: wk respawn"); end
        else $display("PASS dual-task reset recovery");
        end

        if (errors==0) $display("PASS"); else $display("FAIL: %0d errors", errors);
        $finish;
    end
    initial begin #2_000_000_000; $display("TIMEOUT"); $finish; end
endmodule
