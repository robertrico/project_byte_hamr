# Coprocessor C-flash — Persistent Registry Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Persist the coproc's registered services (table + code) to SPI flash and auto-restore them at boot, so a registered service survives a power-cycle with no host re-load.

**Architecture:** Port block_hamr's proven `flash_writer`/`flash_reader` SPI engines. Add `cflash_save` (host-triggered: erase sector → read coproc BRAM `$0200–$0FFF` via port B → program flash `0x400000`) and `cflash_restore` (auto at `boot_done`: read flash → magic-validate → write BRAM via port B). The top muxes port B (host/save/restore, time-exclusive), decodes host status `$C0CE/$C0CF`, and does the `boot_done` SPI mux + `USRMCLK`. `coproc.v` is unchanged.

**Tech Stack:** Verilog-2005 (Icarus `-g2005`), Yosys/nextpnr ECP5 (`USRMCLK` primitive, sim-bypassed), Merlin32. Build `make DESIGN=project_obscurus REV=rev2`; sim `make sim DESIGN=project_obscurus REV=rev2`. Branch `coproc`. Spec: `docs/superpowers/specs/2026-06-08-coproc-cflash-registry-design.md`.

**Fixed constants:** `FLASH_BASE = 24'h400000`. Registry sector: header page `0x400000` (`magic "CR"` + ver + svc_count), data `0x400100`+ = BRAM `$0200–$0FFF` (3584 B = 14 pages). Host: `$C0CF` W=`CP_SAVE` ($5A) / R=status `{b7 save_busy, b1 restore_done, b0 restore_valid}`; `$C0CE` R=`svc_count`. Write-bound: `flash_addr = FLASH_BASE | offset`, `offset < 0x1000`.

---

## File Structure
- Port (copy from block_hamr): `flash_writer.v`, `flash_reader.v`.
- New: `cflash_save.v`, `cflash_restore.v` (+ `_tb.v`), `sim/spi_flash_model.v` (combined r/w).
- Modify: `project_obscurus_top.v` (instantiate + SPI mux + USRMCLK + boot_done + port-B mux + `$C0CE/CF` decode + `FLASH_*` ports), `project_obscurus_tb.v` (integration), `Makefile` (new `.v` auto-globbed; host targets).
- New host: `software/SDM/CPSAVE.S`, `CPBOOT.S`.

`coproc.v`, `sdram_*`, `arlet_*`, `kernel.S` unchanged.

---

## Task 1: Port the SPI engines + combined flash sim model

**Files:** Create `gateware/rev2/project_obscurus/flash_writer.v`, `flash_reader.v`, `sim/spi_flash_model.v`.

- [ ] **Step 1: Copy the proven engines verbatim**
```bash
cd /Users/hambook/Development/project_byte_hamr/gateware/rev2/project_obscurus
cp ../block_hamr/flash_writer.v ./flash_writer.v
cp ../block_hamr/flash_reader.v ./flash_reader.v
```
These are proven IS25LP128F engines — do NOT edit the logic. (`flash_writer`: `start_erase`/`start_program`/`flash_addr[23:0]` + `prog_data`/`prog_data_valid`→`prog_data_req` stream, `busy`/`done`, SPI pins. `flash_reader`: `start`/`start_addr[23:0]`/`byte_count[23:0]` → `data_out`/`data_valid` stream w/ `data_ready` backpressure, `busy`/`done`, SPI pins.)

- [ ] **Step 2: Syntax-check they parse**
```bash
$(echo ${HOME})/oss-cad-suite/bin/iverilog -g2005 -t null flash_writer.v flash_reader.v
```
Expected: clean (no errors).

- [ ] **Step 3: Write a combined flash sim model `sim/spi_flash_model.v`**

The two block_hamr models are split (write-only / read-only). C-flash needs ONE model where SAVE writes and RESTORE reads the same backing store. Addresses mask to `[12:0]` (sector-relative) so `0x400000` works without a 4 MB array. The backing array PERSISTS across the coproc `rst_n` (the model has no reset — only power-on init to `$FF`).
```verilog
`timescale 1ns/1ps
// Combined IS25LP128F-ish model: WREN(06)/SECTOR_ERASE(20)/PAGE_PROGRAM(02)/READ(03).
// Sector-relative: addr masked to [12:0] (one 4KB sector backing store), so the real
// $400000 registry maps in via its low bits. Backing array persists across DUT reset.
module spi_flash_model (
    input  wire sck, input wire ncs, input wire mosi, output reg miso
);
    reg [7:0] mem [0:4095];
    integer i;
    initial begin miso=1'b1; for (i=0;i<4096;i=i+1) mem[i]=8'hFF; end

    reg [7:0]  cmd; reg [23:0] addr; reg [7:0] shin, shout;
    reg [4:0]  bit; reg [31:0] phase; reg wel;
    // sample MOSI on SCK rising; drive MISO on SCK falling. ncs high = deselect/reset frame.
    always @(posedge ncs) begin phase<=0; bit<=0; end
    always @(posedge sck) begin
        if (!ncs) begin
            shin <= {shin[6:0], mosi};
            bit <= bit + 1;
            if (bit==7) begin
                bit<=0;
                case (phase)
                    0: begin cmd <= {shin[6:0],mosi}; phase<=1;
                             if ({shin[6:0],mosi}==8'h06) wel<=1'b1; end
                    1: begin addr[23:16]<={shin[6:0],mosi}; phase<=2; end
                    2: begin addr[15:8] <={shin[6:0],mosi}; phase<=3; end
                    3: begin addr[7:0]  <={shin[6:0],mosi}; phase<=4;
                             if (cmd==8'h20 && wel) begin // SECTOR ERASE
                                 for (i=0;i<4096;i=i+1) mem[i]<=8'hFF; wel<=1'b0; end
                       end
                    default: begin // data phase: program (02) writes, read (03) shifts out
                             if (cmd==8'h02 && wel) begin
                                 mem[addr[11:0]] <= mem[addr[11:0]] & {shin[6:0],mosi};
                                 addr <= addr + 1; end
                       end
                endcase
            end
        end
    end
    always @(negedge sck) begin
        if (!ncs && cmd==8'h03 && phase>=4) begin
            miso <= mem[addr[11:0]][7-bit[2:0]];  // MSB-first read (approx)
            if (bit==7) addr <= addr + 1;
        end
    end
endmodule
```
NOTE: this is a *behavioral approximation* sufficient for the FSM tests (erase→program→read round-trip + magic). If the read-bit timing doesn't match `flash_reader`'s expectations in sim, align it to block_hamr's `spi_flash_model` read path (copy that read logic; keep the shared `mem` + the program/erase from the write model). The goal: SAVE then RESTORE see the same bytes. Verify with the Task 6 round-trip; adjust the model (NOT the engines) if needed.

- [ ] **Step 4: Commit**
```bash
git add gateware/rev2/project_obscurus/flash_writer.v gateware/rev2/project_obscurus/flash_reader.v gateware/rev2/project_obscurus/sim/spi_flash_model.v
git commit -m "vendor(coproc-cflash): port flash_writer/reader + combined SPI flash sim model"
```
Trailer on every commit: `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`

---

## Task 2: `cflash_save.v` — SAVE FSM (BRAM→flash) + unit test

**Files:** Create `cflash_save.v`, `cflash_save_tb.v`.

- [ ] **Step 1: Write the failing unit test `cflash_save_tb.v`**

Models the flash writer's contract + asserts: erase@`0x400000`, then 14 data pages, then the header page LAST with magic "CR", and NO emitted `flash_addr < 0x400000`. Provides BRAM bytes via a stub (`ldata` = `laddr[7:0]` so the data is checkable).
```verilog
`timescale 1ns/1ps
module cflash_save_tb;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0, save_start=0;
    reg [7:0] svc_count=8'd2;
    // port-B read stub: ldata = low byte of laddr
    wire [12:0] laddr; reg [7:0] ldata;
    always @(posedge clk) ldata <= laddr[7:0];
    // flash_writer iface
    wire fw_erase, fw_prog; wire [23:0] fw_addr; wire [7:0] fw_data; wire fw_dvalid;
    reg fw_req=0, fw_busy=0, fw_done=0;
    wire save_busy;
    integer errors=0; reg [23:0] min_addr=24'hFFFFFF; reg saw_erase=0; reg [7:0] hdr0,hdr1;

    cflash_save dut(.clk(clk), .rst_n(rst_n), .save_start(save_start), .svc_count(svc_count),
        .laddr(laddr), .ldata(ldata),
        .fw_start_erase(fw_erase), .fw_start_program(fw_prog), .fw_flash_addr(fw_addr),
        .fw_prog_data(fw_data), .fw_prog_data_valid(fw_dvalid), .fw_prog_data_req(fw_req),
        .fw_busy(fw_busy), .fw_done(fw_done), .save_busy(save_busy));

    // track min flash addr emitted (brick-bound check)
    always @(posedge clk) begin
        if (fw_erase && fw_addr<min_addr) min_addr<=fw_addr;
        if (fw_prog  && fw_addr<min_addr) min_addr<=fw_addr;
        if (fw_erase) saw_erase<=1;
    end
    // model the writer: on erase/prog pulse -> busy 4 cyc; on program, pulse req 256x then done.
    integer pcnt; reg [23:0] cur_addr;
    always @(posedge clk) begin
        fw_done<=0; fw_req<=0;
        if (fw_erase) begin fw_busy<=1; cur_addr<=fw_addr; pcnt<=4; end
        else if (fw_prog) begin fw_busy<=1; cur_addr<=fw_addr; pcnt<=256; fw_req<=1; end
        else if (fw_busy) begin
            if (fw_prog==0 && pcnt>0 && cur_addr[11:0]<24'h1000 && fw_dvalid) begin
                if (cur_addr==24'h400000) begin if(pcnt==256) hdr0<=fw_data; if(pcnt==255) hdr1<=fw_data; end
                pcnt<=pcnt-1; if (pcnt>1) fw_req<=1; else begin fw_busy<=0; fw_done<=1; end
            end else if (!fw_dvalid && pcnt>0 && pcnt<=256 && cur_addr!=24'h0) fw_req<=1;
            else begin pcnt<=pcnt-1; if (pcnt<=1) begin fw_busy<=0; fw_done<=1; end end
        end
    end

    initial begin
        rst_n=0; #40; rst_n=1; #20;
        @(posedge clk); save_start=1; @(posedge clk); save_start=0;
        wait(save_busy); wait(!save_busy);   // save completes
        #100;
        if (!saw_erase) begin errors=errors+1; $display("FAIL no erase"); end
        if (min_addr < 24'h400000) begin errors=errors+1; $display("FAIL brick-bound: emitted %06X",min_addr); end
        if (hdr0!==8'h43 || hdr1!==8'h52) begin errors=errors+1; $display("FAIL magic %02X%02X want 4352 (CR)",hdr0,hdr1); end
        if (errors==0) $display("PASS cflash_save (erase@>=400000, magic CR, no addr<400000)");
        else $display("FAIL cflash_save %0d",errors);
        $finish;
    end
endmodule
```
(The tb's writer-model is approximate; the load-bearing assertions are **min_addr ≥ 0x400000** and **magic "CR" at 0x400000**. Adjust the model handshake to make the FSM progress, but do NOT weaken those two asserts.)

- [ ] **Step 2: Run — expect failure (no module)**
```bash
cd /Users/hambook/Development/project_byte_hamr/gateware/rev2/project_obscurus
$(echo ${HOME})/oss-cad-suite/bin/iverilog -g2005 -o /tmp/cs.out -s cflash_save_tb cflash_save.v cflash_save_tb.v
```
Expected: error (no `cflash_save`).

- [ ] **Step 3: Implement `cflash_save.v`**
```verilog
// cflash_save.v - SAVE FSM: erase registry sector, program 14 DATA pages from coproc
// BRAM $0200-$0FFF (via port-B reads), then the HEADER page (magic "CR") LAST (torn-save
// fail-safe). All flash addresses hard-bounded to FLASH_BASE | offset (offset<0x1000) so
// nothing below 0x400000 can ever be written.
module cflash_save (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        save_start,        // 1-cycle pulse
    input  wire [7:0]  svc_count,         // stored in header
    // coproc port-B read (top muxes this onto coproc when save_busy)
    output wire [12:0] laddr,             // BRAM addr to read
    input  wire [7:0]  ldata,             // BRAM[laddr], registered (1-cyc latency)
    // flash_writer
    output reg         fw_start_erase,
    output reg         fw_start_program,
    output reg  [23:0] fw_flash_addr,
    output reg  [7:0]  fw_prog_data,
    output reg         fw_prog_data_valid,
    input  wire        fw_prog_data_req,
    input  wire        fw_busy,
    input  wire        fw_done,
    // status
    output wire        save_busy
);
    localparam [23:0] FLASH_BASE = 24'h400000;
    localparam BRAM_BASE = 13'h0200;       // registry start in BRAM
    localparam DATA_PAGES = 14;            // $0200-$0FFF = 3584B = 14 pages

    localparam [3:0] S_IDLE=0,S_ERASE=1,S_ERWAIT=2,S_DPAGE=3,S_DPROG=4,S_HPAGE=5,S_HPROG=6,S_DONE=7;
    reg [3:0] st;
    reg [3:0] page;       // 0..13 data page index
    reg [7:0] boff;       // byte offset within page 0..255
    reg [7:0] hidx;       // header byte index

    assign save_busy = (st != S_IDLE);
    // BRAM addr for the current data byte (only meaningful in data phase)
    assign laddr = BRAM_BASE + {page, 8'd0} + {5'd0, boff};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st<=S_IDLE; page<=0; boff<=0; hidx<=0;
            fw_start_erase<=0; fw_start_program<=0; fw_flash_addr<=0;
            fw_prog_data<=0; fw_prog_data_valid<=0;
        end else begin
            fw_start_erase<=0; fw_start_program<=0; fw_prog_data_valid<=0;
            case (st)
                S_IDLE: if (save_start) begin
                    fw_flash_addr <= FLASH_BASE;           // erase sector @ 0x400000
                    fw_start_erase<=1; st<=S_ERASE; end
                S_ERASE: st<=S_ERWAIT;
                S_ERWAIT: if (fw_done) begin page<=0; boff<=0; st<=S_DPAGE; end
                // ---- DATA pages first (so the header/magic is programmed LAST) ----
                S_DPAGE: begin
                    fw_flash_addr <= FLASH_BASE | (24'h100 + {page,8'd0}); // 0x400100 + page*256
                    boff<=0; fw_start_program<=1; st<=S_DPROG;
                end
                S_DPROG: if (fw_prog_data_req) begin
                    fw_prog_data <= ldata;                 // BRAM[laddr] (laddr tracks page,boff)
                    fw_prog_data_valid<=1;
                    if (boff==8'd255) begin
                        if (page==DATA_PAGES-1) st<=S_HPAGE;
                        else begin page<=page+1; st<=S_DPAGE; end
                    end else boff<=boff+1;
                end
                // ---- HEADER page LAST ----
                S_HPAGE: begin
                    fw_flash_addr <= FLASH_BASE;           // 0x400000 header
                    hidx<=0; fw_start_program<=1; st<=S_HPROG;
                end
                S_HPROG: if (fw_prog_data_req) begin
                    case (hidx)
                        8'd0: fw_prog_data<=8'h43;          // 'C'
                        8'd1: fw_prog_data<=8'h52;          // 'R'
                        8'd2: fw_prog_data<=8'h01;          // version
                        8'd3: fw_prog_data<=svc_count;
                        default: fw_prog_data<=8'hFF;       // pad
                    endcase
                    fw_prog_data_valid<=1;
                    if (hidx==8'd255) st<=S_DONE; else hidx<=hidx+1;
                end
                S_DONE: if (!fw_busy) st<=S_IDLE;
                default: st<=S_IDLE;
            endcase
        end
    end
endmodule
```

- [ ] **Step 4: Run — expect PASS**
```bash
$(echo ${HOME})/oss-cad-suite/bin/iverilog -g2005 -o /tmp/cs.out -s cflash_save_tb cflash_save.v cflash_save_tb.v && $(echo ${HOME})/oss-cad-suite/bin/vvp /tmp/cs.out
```
Expected: `PASS cflash_save`. The two load-bearing asserts (min_addr ≥ 0x400000, magic "CR") must hold. If the tb's writer-model handshake stalls the FSM, fix the MODEL's req/done timing — not the FSM, not the asserts.

- [ ] **Step 5: Commit**
```bash
git add gateware/rev2/project_obscurus/cflash_save.v gateware/rev2/project_obscurus/cflash_save_tb.v
git commit -m "feat(coproc-cflash): cflash_save FSM (BRAM->flash, magic-last, addr>=0x400000 bound)"
```

---

## Task 3: `cflash_restore.v` — RESTORE FSM (flash→BRAM) + unit test

**Files:** Create `cflash_restore.v`, `cflash_restore_tb.v`.

- [ ] **Step 1: Write the failing unit test `cflash_restore_tb.v`**

Models the flash reader streaming a header (magic) + data; asserts: valid magic → BRAM writes at `$0200+` match the data + `restore_valid`; bad magic → no BRAM writes + `restore_valid=0`.
```verilog
`timescale 1ns/1ps
module cflash_restore_tb;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0, boot_done=0;
    // flash_reader iface (tb models the reader)
    wire fr_start; wire [23:0] fr_addr, fr_count; wire fr_ready;
    reg fr_busy=0, fr_done=0, fr_dvalid=0; reg [7:0] fr_data=0;
    // port-B write (tb captures)
    wire [12:0] r_laddr; wire [7:0] r_ldata; wire r_lwr;
    wire restore_done, restore_valid; wire [7:0] svc_count_o;
    integer errors=0; reg [7:0] cap0200, cap0201; reg wrote=0;

    cflash_restore dut(.clk(clk), .rst_n(rst_n), .boot_done(boot_done),
        .fr_start(fr_start), .fr_start_addr(fr_addr), .fr_byte_count(fr_count),
        .fr_data_ready(fr_ready), .fr_busy(fr_busy), .fr_done(fr_done),
        .fr_data_out(fr_data), .fr_data_valid(fr_dvalid),
        .laddr(r_laddr), .ldata(r_ldata), .lwr(r_lwr),
        .restore_done(restore_done), .restore_valid(restore_valid), .svc_count(svc_count_o));

    always @(posedge clk) if (r_lwr) begin
        wrote<=1;
        if (r_laddr==13'h0200) cap0200<=r_ldata;
        if (r_laddr==13'h0201) cap0201<=r_ldata;
    end

    // reader model: on fr_start, stream fr_count bytes. HEADER read first (addr 0x400000):
    // produce magic per `good_magic`. Then DATA read (0x400100): produce 0x200+i pattern.
    reg good_magic; integer idx; integer total; reg in_hdr;
    task run_reader; begin
        idx=0; total=fr_count; in_hdr=(fr_addr==24'h400000);
        fr_busy=1; @(posedge clk);
        while (idx<total) begin
            if (in_hdr) begin
                case (idx) 0: fr_data=good_magic?8'h43:8'h00; 1: fr_data=8'h52;
                           2: fr_data=8'h01; 3: fr_data=8'd2; default: fr_data=8'hFF; endcase
            end else fr_data = idx[7:0];   // data pattern
            fr_dvalid=1; @(posedge clk); fr_dvalid=0; idx=idx+1; @(posedge clk);
        end
        fr_busy=0; fr_done=1; @(posedge clk); fr_done=0;
    end endtask
    always @(posedge fr_start) run_reader;

    initial begin
        // --- valid magic ---
        rst_n=0; good_magic=1; #40; rst_n=1; #20;
        @(posedge clk); boot_done=1;
        wait(restore_done);
        if (!restore_valid) begin errors=errors+1; $display("FAIL valid magic not restored"); end
        else if (!wrote) begin errors=errors+1; $display("FAIL no BRAM writes on valid"); end
        else $display("PASS restore valid-magic -> BRAM written, valid=1");
        // --- bad magic ---
        rst_n=0; good_magic=0; wrote=0; #40; rst_n=1; #20;
        @(posedge clk); boot_done=1; wait(restore_done);
        if (restore_valid) begin errors=errors+1; $display("FAIL bad magic restored"); end
        else if (wrote) begin errors=errors+1; $display("FAIL bad magic wrote BRAM"); end
        else $display("PASS restore bad-magic -> no write, valid=0");
        if (errors==0) $display("PASS cflash_restore"); else $display("FAIL cflash_restore %0d",errors);
        $finish;
    end
endmodule
```

- [ ] **Step 2: Run — expect failure**
```bash
cd /Users/hambook/Development/project_byte_hamr/gateware/rev2/project_obscurus
$(echo ${HOME})/oss-cad-suite/bin/iverilog -g2005 -o /tmp/cr.out -s cflash_restore_tb cflash_restore.v cflash_restore_tb.v
```
Expected: error (no `cflash_restore`).

- [ ] **Step 3: Implement `cflash_restore.v`**
```verilog
// cflash_restore.v - RESTORE FSM (auto at boot_done): read flash header @ 0x400000, check
// magic "CR"; if valid, read 14 data pages @ 0x400100 and write coproc BRAM $0200-$0FFF via
// port B (write-protect on $1000+ stays in force in coproc). Bad/empty magic -> no write.
module cflash_restore (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        boot_done,         // level: high after FPGA config (POR-derived)
    // flash_reader
    output reg         fr_start,
    output reg  [23:0] fr_start_addr,
    output reg  [23:0] fr_byte_count,
    output reg         fr_data_ready,
    input  wire        fr_busy,
    input  wire        fr_done,
    input  wire [7:0]  fr_data_out,
    input  wire        fr_data_valid,
    // coproc port-B write (top muxes onto coproc when restore_busy)
    output reg  [12:0] laddr,
    output reg  [7:0]  ldata,
    output reg         lwr,
    // status
    output reg         restore_done,
    output reg         restore_valid,
    output reg  [7:0]  svc_count
);
    localparam [23:0] FLASH_BASE = 24'h400000;
    localparam [3:0] R_IDLE=0,R_HSTART=1,R_HREAD=2,R_DSTART=3,R_DREAD=4,R_DONE=5;
    reg [3:0] st;
    reg [7:0] hidx;
    reg [12:0] widx;          // 0..3583 data byte index -> BRAM $0200+widx
    reg started;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st<=R_IDLE; hidx<=0; widx<=0; started<=0;
            fr_start<=0; fr_start_addr<=0; fr_byte_count<=0; fr_data_ready<=1'b1;
            laddr<=0; ldata<=0; lwr<=0;
            restore_done<=0; restore_valid<=0; svc_count<=0;
        end else begin
            fr_start<=0; lwr<=0;
            case (st)
                R_IDLE: if (boot_done && !started) begin
                    started<=1;
                    fr_start_addr<=FLASH_BASE; fr_byte_count<=24'd4;  // header: magic+ver+count
                    fr_start<=1; hidx<=0; st<=R_HSTART;
                end
                R_HSTART: st<=R_HREAD;
                R_HREAD: begin
                    if (fr_data_valid) begin
                        case (hidx)
                            8'd0: if (fr_data_out!==8'h43) begin restore_done<=1; st<=R_DONE; end // not 'C'
                            8'd1: if (fr_data_out!==8'h52) begin restore_done<=1; st<=R_DONE; end // not 'R'
                            8'd3: svc_count<=fr_data_out;
                            default: ;
                        endcase
                        hidx<=hidx+1;
                    end
                    if (fr_done && st==R_HREAD) begin           // header read finished, magic ok
                        widx<=0; st<=R_DSTART;
                    end
                end
                R_DSTART: begin
                    fr_start_addr<=FLASH_BASE | 24'h100; fr_byte_count<=24'd3584; // 14 pages
                    fr_start<=1; st<=R_DREAD;
                end
                R_DREAD: begin
                    if (fr_data_valid) begin
                        laddr<=13'h0200 + widx; ldata<=fr_data_out; lwr<=1'b1;  // write BRAM
                        widx<=widx+1;
                    end
                    if (fr_done) begin restore_valid<=1; restore_done<=1; st<=R_DONE; end
                end
                R_DONE: ;   // latch done/valid; stay
                default: st<=R_IDLE;
            endcase
        end
    end
endmodule
```
NOTE: the magic-fail in `R_HREAD` jumps to `R_DONE` with `restore_valid=0` (default), no data read, no BRAM write — fail-safe. The `fr_data_ready` is held high (no backpressure needed; the FSM keeps up with the SPI byte rate).

- [ ] **Step 4: Run — expect PASS**
```bash
$(echo ${HOME})/oss-cad-suite/bin/iverilog -g2005 -o /tmp/cr.out -s cflash_restore_tb cflash_restore.v cflash_restore_tb.v && $(echo ${HOME})/oss-cad-suite/bin/vvp /tmp/cr.out
```
Expected: `PASS cflash_restore` (valid→write+valid; bad magic→no-write+invalid).

- [ ] **Step 5: Commit**
```bash
git add gateware/rev2/project_obscurus/cflash_restore.v gateware/rev2/project_obscurus/cflash_restore_tb.v
git commit -m "feat(coproc-cflash): cflash_restore FSM (flash->BRAM, magic-validate, fail-safe)"
```

---

## Task 4: Top integration — instantiate, SPI mux, port-B mux, host status

**Files:** Modify `project_obscurus_top.v`.

- [ ] **Step 1: Add the `FLASH_*` ports**
Add to the module port list (pins already in `byte_hamr.lpf`):
```verilog
    output wire        FLASH_nCS,
    output wire        FLASH_MOSI,
    input  wire        FLASH_MISO,
    output wire        FLASH_nWP,
    output wire        FLASH_nHOLD,
```

- [ ] **Step 2: `boot_done` (POR-derived) + instantiate the engines + FSMs**
Add after the coproc/arbiter instantiation. `boot_done` = `por_n` (config completes before the POR counter finishes, so `por_n` rising is a safe post-config boot signal):
```verilog
    // ---- C-flash: SPI engines + save/restore FSMs ----
    wire boot_done = por_n;                 // post-config (POR-derived)
    // flash_writer signals
    wire fw_erase, fw_prog; wire [23:0] fw_addr; wire [7:0] fw_data; wire fw_dvalid;
    wire fw_req, fw_busy, fw_wdone;
    wire writer_sck, writer_ncs, writer_mosi;
    // flash_reader signals
    wire fr_start; wire [23:0] fr_addr, fr_count; wire fr_ready;
    wire fr_busy, fr_rdone; wire [7:0] fr_data; wire fr_dvalid;
    wire reader_sck, reader_ncs, reader_mosi;
    // save / restore
    wire        save_start = reg_wr & (wr_addr_latch==4'hF) & (wr_data_latch==8'h5A); // CP_SAVE $C0CF=$5A
    wire        save_busy;
    wire [12:0] save_laddr; wire restore_busy;
    wire [12:0] rst_laddr;  wire [7:0] rst_ldata; wire rst_lwr;
    wire        restore_done, restore_valid; wire [7:0] svc_count;

    flash_writer u_fw (
        .clk(clk), .rst_n(rst_n),
        .start_erase(fw_erase), .start_program(fw_prog), .flash_addr(fw_addr),
        .prog_data(fw_data), .prog_data_valid(fw_dvalid), .prog_data_req(fw_req),
        .busy(fw_busy), .done(fw_wdone),
        .spi_sck(writer_sck), .spi_ncs(writer_ncs), .spi_mosi(writer_mosi), .spi_miso(FLASH_MISO));

    flash_reader u_fr (
        .clk(clk), .rst_n(rst_n), .start(fr_start), .start_addr(fr_addr), .byte_count(fr_count),
        .busy(fr_busy), .done(fr_rdone), .data_out(fr_data), .data_valid(fr_dvalid),
        .data_ready(fr_ready),
        .flash_ncs(reader_ncs), .flash_mosi(reader_mosi), .flash_miso(FLASH_MISO),
        .flash_nwp(), .flash_nhold(), .flash_sck_pin(reader_sck));

    cflash_save u_save (
        .clk(clk), .rst_n(rst_n), .save_start(save_start), .svc_count(8'd0),
        .laddr(save_laddr), .ldata(cp_ldata_out),
        .fw_start_erase(fw_erase), .fw_start_program(fw_prog), .fw_flash_addr(fw_addr),
        .fw_prog_data(fw_data), .fw_prog_data_valid(fw_dvalid), .fw_prog_data_req(fw_req),
        .fw_busy(fw_busy), .fw_done(fw_wdone), .save_busy(save_busy));

    cflash_restore u_rst (
        .clk(clk), .rst_n(rst_n), .boot_done(boot_done),
        .fr_start(fr_start), .fr_start_addr(fr_addr), .fr_byte_count(fr_count),
        .fr_data_ready(fr_ready), .fr_busy(fr_busy), .fr_done(fr_rdone),
        .fr_data_out(fr_data), .fr_data_valid(fr_dvalid),
        .laddr(rst_laddr), .ldata(rst_ldata), .lwr(rst_lwr),
        .restore_done(restore_done), .restore_valid(restore_valid), .svc_count(svc_count));
    assign restore_busy = ~restore_done;    // restore active from boot until done
```
NOTE: confirm `reg_wr`/`wr_addr_latch`/`wr_data_latch`/`cp_ldata_out`/`por_n`/`clk`/`rst_n` exist (they do from C1/C3.1). `flash_reader`'s `flash_nwp`/`flash_nhold` are tied high inside it; leave unconnected or tie at top.

- [ ] **Step 3: SPI pin mux + `USRMCLK`**
```verilog
    // reader owns SPI during boot/restore; writer during save. They never overlap.
    wire spi_sck  = save_busy ? writer_sck  : reader_sck;
    wire spi_ncs  = save_busy ? writer_ncs  : reader_ncs;
    wire spi_mosi = save_busy ? writer_mosi : reader_mosi;
    assign FLASH_nCS  = spi_ncs;
    assign FLASH_MOSI = spi_mosi;
    assign FLASH_nWP   = 1'b1;
    assign FLASH_nHOLD = 1'b1;
`ifdef SYNTHESIS
    USRMCLK u_usrmclk (.USRMCLKI(spi_sck), .USRMCLKTS(1'b0));
`endif
```
(In sim, `spi_sck` is a plain wire to the flash model; under synth, `USRMCLK` drives the config-flash clock. Match block_hamr's `\`ifdef`/`\`else` guard exactly — copy its form.)

- [ ] **Step 4: Port-B mux (host / save / restore) into the coproc**
The coproc port-B inputs are currently driven by host regs (`m_laddr`/`wr_data_latch`/`cp_wdata_wr`). Mux them with save/restore. Find the `coproc u_coproc(...)` instantiation and change the `.laddr/.ldata_in/.lwr` connections to muxed wires:
```verilog
    wire [12:0] cp_laddr_mux = restore_busy ? rst_laddr : save_busy ? save_laddr : m_laddr;
    wire [7:0]  cp_ldata_mux = restore_busy ? rst_ldata : wr_data_latch;
    wire        cp_lwr_mux   = restore_busy ? rst_lwr   : cp_wdata_wr;  // save only READS (lwr=0)
    // ... in coproc u_coproc: .laddr(cp_laddr_mux), .ldata_in(cp_ldata_mux), .lwr(cp_lwr_mux),
```
(SAVE reads BRAM — it drives `save_laddr` and consumes `cp_ldata_out`; `lwr` stays host/restore. RESTORE writes — drives `rst_laddr/ldata/lwr`. Host owns port B when neither busy. Time-exclusive.)

- [ ] **Step 5: Host status read mux — `$C0CE`/`$C0CF`**
In the register read mux (`case (apple_addr[3:0])`), add (and the scratch default now only covers what's left):
```verilog
            4'hE: reg_data_out = svc_count;                                   // CP_SVCNT
            4'hF: reg_data_out = {save_busy, 5'b0, restore_done, restore_valid}; // CP_FSTAT
```
And narrow the scratch write decode so `$C0CE/$C0CF` aren't scratch (the C3.1 SAVE used `$C0CF` write; keep `$C0CF` write = `save_start` via the decode in Step 2; `$C0CE` is read-only).

- [ ] **Step 6: Build (USRMCLK, EBR, timing)**
Run: `make clean && make DESIGN=project_obscurus REV=rev2`
Expected: clean bitstream. The `*.v` glob picks up `flash_writer/reader/cflash_*`. Confirm `USRMCLK` maps (synth), DP16KD still 8 (flash FSMs are logic), timing PASS at 25 MHz. Report Fmax + DP16KD + LUT delta. Also add `FLASH_*` to the design's `.lpf` use if not already (pins exist in `byte_hamr.lpf`). Do NOT flash.

- [ ] **Step 7: Commit**
```bash
git add gateware/rev2/project_obscurus/project_obscurus_top.v
git commit -m "feat(coproc-cflash): top integrate flash engines + save/restore FSMs + SPI mux + \$C0CE/CF status"
```

---

## Task 5: Host loaders `CPSAVE.S` + `CPBOOT.S` + disk

**Files:** Create `software/SDM/CPSAVE.S`, `CPBOOT.S`; modify `Makefile`.

- [ ] **Step 1: `CPSAVE.S`** (register a service, then SAVE to flash)
Mirror `CPRACE3.S`'s load/register, then trigger save + poll. ORG `$6000`:
```
* CPSAVE.S - load+register racetask3, then persist the registry to flash.
 TYP $06
 DSK CPSAVE
 ORG $6000
CPLADDRLO = $C0C9
CPLADDRHI = $C0CA
CPWDATA   = $C0CB
CPSAVE    = $C0CF
CPFSTAT   = $C0CF
 JMP MAIN
 PUT SDRAMLIB
* (LOADBLK + the RT3 DFB table: copy from CPRACE3.S verbatim)
MAIN JSR SDM_READY
* --- load racetask3 -> $0300, TABLE entries, (NPARAM optional) ---
 ... (copy CPRACE3 load+TABLE block) ...
* --- SAVE to flash: write $5A, poll busy(b7) clear ---
 LDA #$5A
 STA CPSAVE
:wb LDA CPFSTAT
 AND #$80
 BNE :wb            ; wait save_busy clear
 RTS
```
Copy `LOADBLK` + the racetask3 `DFB` bytes from `CPRACE3.S`. (No GO here — just register + persist.)

- [ ] **Step 2: `CPBOOT.S`** (after power-cycle: confirm restore, then run from flash)
```
* CPBOOT.S - assumes the registry was auto-restored from flash at boot. Confirm valid,
* set NPARAM + COUNT(GO), read results. NO task load (it came from flash).
 TYP $06
 DSK CPBOOT
 ORG $6000
CPLADDRLO = $C0C9
CPLADDRHI = $C0CA
CPWDATA   = $C0CB
CPCOUNT   = $C0CD
CPFSTAT   = $C0CF
 JMP MAIN
 PUT SDRAMLIB
* (WAITRACE + SHOWRESULTS + ZEROCELLS: copy from CPRACE3.S)
MAIN JSR SDM_READY
* --- confirm restore valid (b0) ---
 LDA CPFSTAT
 AND #$01
 BNE :ok
 LDA #<MNOREG
 STA $06
 LDA #>MNOREG
 STA $07
 JSR PRSTR          ; "NO REGISTRY"
 RTS
:ok
* --- NPARAM 50/200 @ $00E8 (host-provided runtime params) ---
 LDA #$E8 / STA CPLADDRLO / LDA #$00 / STA CPLADDRHI
 LDA #50 / STA CPWDATA / LDA #200 / STA CPWDATA
* --- GO (re-bootstraps; tasks already in BRAM from flash restore) ---
 LDA #$02 / STA CPCOUNT
 JSR WAITRACE
 JSR SHOWRESULTS    ; expect 01 02
 RTS
MNOREG ASC "NO REGISTRY"
 DFB $00
```
(Reuse `WAITRACE`/`SHOWRESULTS`/`ZEROCELLS`/`PRSTR` from `CPRACE3.S`. The key: CPBOOT does NOT load racetask3 — it relies on the flash restore having put it in BRAM.)

- [ ] **Step 3: Makefile + disk**
Add `cpsave`/`cpboot` targets (assemble), add to `.PHONY` + `sdmdisk` deps + pack lines (`$(AC_CLASSIC) -p $(SDM_PO) CPSAVE BIN 0x6000 < $(SDM_DIR)/CPSAVE`, same for CPBOOT). `make cpsave cpboot sdmdisk`; confirm both in the catalog.

- [ ] **Step 4: Commit**
```bash
git add software/SDM/CPSAVE.S software/SDM/CPBOOT.S Makefile
git commit -m "feat(coproc-cflash): CPSAVE (register+persist) + CPBOOT (confirm restore+run)"
```

---

## Task 6: Integration sim — save → reset → restore → run

**Files:** Modify `project_obscurus_tb.v`.

- [ ] **Step 1: Wire the flash model + FLASH pins in the tb**
READ `project_obscurus_tb.v`. Add the `FLASH_*` DUT connections + instantiate `spi_flash_model` on them (`spi_flash_model flash(.sck(FLASH... ), ...)` — the SPI pins from the DUT). The model's backing array must NOT reset with the coproc (it models nonvolatile flash). Confirm `clk100`/`sdram_read`/`load_byte`/`wr_reg`/`tmp`/`errors`.

- [ ] **Step 2: Add the persistence integration block**
After the C3.1 block, before the final summary:
```verilog
        // ===== C-flash: SAVE -> reset -> auto-RESTORE -> run from flash =====
        // load racetask3 @ $0300 + TABLE (NOT via flash - this is the pre-save register)
        wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h03);
        // <<< load_byte() the 49 racetask3 bytes (from C3.1) >>>
        wr_reg(4'h9, 8'h00); wr_reg(4'hA, 8'h02);
        load_byte(8'h00); load_byte(8'h03); load_byte(8'h00); load_byte(8'h03); // TABLE
        // SAVE to flash:
        wr_reg(4'hF, 8'h5A);                       // CP_SAVE
        // wait save_busy (CP_FSTAT b7) clear
        begin : savewait integer g; g=0;
          rd_reg(4'hF, tmp);
          while (tmp[7] && g<200000) begin rd_reg(4'hF, tmp); @(posedge clk100); g=g+1; end
        end
        $display("PASS C-flash SAVE complete");
        // --- simulate power-cycle: reset the COPROC (BRAM re-inits, flash model RETAINS) ---
        // pulse the coproc reset path (nRES_READ or the design's reset) WITHOUT clearing the
        // flash model; restore runs at boot_done.
        nRES_READ=1'b0; repeat (40) @(posedge clk100); nRES_READ=1'b1;
        repeat (300000) @(posedge clk100);          // let restore (flash->BRAM) finish
        // confirm restore valid:
        rd_reg(4'hF, tmp);
        if (!tmp[0]) begin errors=errors+1; $display("FAIL C-flash restore_valid=0"); end
        else $display("PASS C-flash auto-restore valid");
        // run from FLASH-restored tasks (NO re-load): NPARAM + GO
        wr_reg(4'h9, 8'hE8); wr_reg(4'hA, 8'h00); load_byte(8'd4); load_byte(8'd12);
        wr_reg(4'hD, 8'h02);                        // GO
        repeat (200000) @(posedge clk100);
        sdram_read(10'd0, 16'h0061, tmp);
        if (tmp!==8'h01) begin errors=errors+1; $display("FAIL C-flash run r0 %02X want 01 (restored task ran?)",tmp); end
        else $display("PASS C-flash service ran from flash (r0=01, no re-load)");
```
DEBUG: if restore_valid=0 → magic not written/read (check the flash model round-trip + cflash_save magic-last + cflash_restore header read). If r0 != 01 → the restored BRAM doesn't match (flash model data path, or restore BRAM-write addr). The proof: the task ran WITHOUT the post-reset `load_byte` of racetask3 — it came from flash.

- [ ] **Step 3: Run the sim**
Run: `make sim DESIGN=project_obscurus REV=rev2`
Expected: `PASS C-flash SAVE complete`, `PASS C-flash auto-restore valid`, `PASS C-flash service ran from flash` + ALL prior (C1/C2/C3/C3.1/monitor) still PASS, final 0 errors. iverilog `-g2005`.
If the flash model's read-bit timing mismatches `flash_reader`, align the model's read path to block_hamr's `spi_flash_model` (Task 1 note) — root-cause the SPI handshake, don't fake the assertion.

- [ ] **Step 4: Commit**
```bash
git add gateware/rev2/project_obscurus/project_obscurus_tb.v gateware/rev2/project_obscurus/sim/spi_flash_model.v
git commit -m "test(coproc-cflash): save -> reset -> auto-restore -> run-from-flash integration"
```

---

## Task 7: Bench verification (user-run)

**Files:** none. The **user** flashes.

- [ ] **Step 1: Build** — `make clean && make DESIGN=project_obscurus REV=rev2`; clean, USRMCLK mapped, DP16KD 8, timing. Report "ready".
- [ ] **Step 2: Hand the procedure to the user**
```
1. (you) flash build/project_obscurus.bit
2. Boot /SDRAM/.  BRUN CPSAVE      -> registers racetask3 + persists to flash (no visible output, or "SAVED")
3. POWER-CYCLE the board (or re-config) - do NOT re-flash, do NOT reload the disk task
4. Boot /SDRAM/.  BRUN CPBOOT      -> if restore worked: prints 01 02 (the service ran from FLASH, never re-loaded)
                                       if "NO REGISTRY": restore didn't validate
```
Pass = after a power-cycle, `CPBOOT` runs the service **without loading it** → it came from flash. Persistence across boot, on silicon — "register once, there on the next boot."
If "NO REGISTRY" after a clean SAVE+power-cycle: the restore didn't find valid magic — check the SPI mux drives the flash after boot_done, USRMCLK, and that the power-cycle didn't erase 0x400000 (it shouldn't — only re-config).

- [ ] **Step 3: Record** — update `project_coproc_c0.md` + `MEMORY.md`: C-flash persistent registry; the flash region (0x400000), the SPI-mux/USRMCLK/boot_done pattern, host $C0CE/CF status. Multi-core = the remaining rung.

---

## Self-Review

**Spec coverage:** flash_writer/reader port → Task 1. Combined flash sim model (sector-relative, persists across reset) → Task 1. cflash_save (BRAM→flash, magic-last, addr≥0x400000 bound) → Task 2. cflash_restore (flash→BRAM, magic-validate, fail-safe) → Task 3. SPI mux + USRMCLK + boot_done(POR) + port-B mux + `$C0CE/CF` host status + CP_SAVE trigger → Task 4. Host CPSAVE/CPBOOT → Task 5. save→reset→restore→run integration → Task 6. Bench power-cycle proof → Task 7. Write-bound (`FLASH_BASE | offset`) → Task 2 + asserted in Task 2 tb (min_addr≥0x400000). Torn-save magic-last → Task 2 FSM (data pages then header). coproc.v unchanged (top muxes) → noted.

**Placeholder scan:** the flash sim model + the tb's writer-model are flagged "behavioral approximation, align to block_hamr's read path if SPI timing mismatches" — a real adjustment step with a concrete fallback (copy block_hamr's read logic), not a TODO. CPSAVE/CPBOOT "copy LOADBLK/DFB/WAITRACE from CPRACE3" — concrete source. No bare TODOs.

**Type/label consistency:** `FLASH_BASE 0x400000` consistent across save/restore/model. `$C0CF` W=CP_SAVE($5A)/R=fstat, `$C0CE` R=svc_count consistent top↔host. cflash_save ports (`fw_*`, `laddr`/`ldata`, `save_busy`) match the top instantiation + flash_writer. cflash_restore ports (`fr_*`, `laddr/ldata/lwr`, `restore_*`) match. Port-B mux (`restore_busy`>`save_busy`>host) consistent. Magic "CR" = `$43$52` consistent (save writes, restore checks, tb asserts). `boot_done=por_n`. Registry `$0200-$0FFF` = 14 data pages consistent.

**Executor notes:** iverilog `-g2005`; `make ... REV=rev2`. Do NOT edit the ported flash_writer/reader logic. The `USRMCLK` is `\`ifdef SYNTHESIS`-guarded (sim sees a plain `spi_sck` wire to the model) — copy block_hamr's guard form exactly. The write-bound (`FLASH_BASE | offset`) + min_addr≥0x400000 assert are the brick-hygiene; keep them. SAVE programs data pages then the header LAST (torn-save fail-safe). Flash model persists across the coproc reset (models nonvolatile). Don't modify coproc.v/kernel.S/sdram_*/arlet_*.
