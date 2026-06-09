# project_obscurus Armed Expansion ROM — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop the card from driving the shared `$C800–$CFFF` expansion-ROM bus unless software explicitly arms it, eliminating the contention that crashes ProDOS/CopyUtils ($CEF3) — while keeping `$C800` usable on demand for the `PR#4` monitor.

**Architecture:** Add a `rom_armed` flip-flop (default 0) gating all `$C800` drive; arm via `$C0C7←$AA`, disarm via `$C0C8←$AA`, hard-disarm on reset/POR. Fill unused expansion ROM with `$60` (RTS) so stray fetches return harmlessly. The `$C400` slot ROM arms on `PR#4` and hosts a `DISARM` routine (always-live) the monitor jumps to on exit. SDRAM `$C0Cx` access stays always-live/unarmed.

**Tech Stack:** Verilog (iverilog `-g2005` sim — this build hangs on `-g2009`/`-g2012`), Merlin32, `scripts/rom2mem.py`, yosys/nextpnr/ecppack (ECP5).

**Spec:** `docs/superpowers/specs/2026-06-04-obscurus-armed-rom-design.md`

---

## Background for the implementer

- The card drives the Apple D bus in `project_obscurus_top.v` via `d_oe`/`DATA_OE`. Three terms: `device_read` (`$C0Cx`, slot-gated, safe), `rom_read` (`$C400`, slot-gated, safe), and `exp_read = rom_en & ~nI_O_STROBE & R_nW` (`$C800–$CFFF`, the unsafe one). This task gates `exp_read` and the `DATA_OE` expansion term with a new `rom_armed`.
- `rom_en` (existing): sets on `~nI_O_SELECT` (`$C4xx`), clears on `$CFFF`. Keep it; just AND with `rom_armed`.
- Register writes commit on `nds_rise` via `wr_addr_latch`/`wr_data_latch`/`wr_rw_latch` (already in the file). The `reg_wr = nds_rise & ~wr_rw_latch` case statement handles `$C0C0–$C0C6`; scratch is `wr_addr_latch >= 4'h7`.
- iverilog here HANGS on `-g2009`/`-g2012`. Use default `-g2005` (no flag). Sims auto-background; use a hang guard.
- No 6502 simulator: `.S` files are verified by assemble-clean + greps; runtime is bench.
- **Disarm-from-ROM rule:** once `rom_armed`=0, the `$C800` monitor stops responding mid-fetch. So the disarm sequence MUST execute from the always-live `$C400` slot ROM (gated only by `nI_O_SELECT`, never by `rom_armed`). The monitor's `Q` does `JMP $C420` into the slot ROM's `DISARM` routine.

## File Structure

| File | Responsibility | Action |
|------|----------------|--------|
| `scripts/rom2mem.py` | add optional fill-byte arg (default `0xFF`) | Modify |
| `Makefile` | monitor.mem recipe → fill `0x60` | Modify |
| `gateware/rev2/project_obscurus/project_obscurus_top.v` | `rom_armed` FF, gate `$C800` drive, scratch shrink, arm/disarm decode | Modify |
| `gateware/rev2/project_obscurus/project_obscurus_tb.v` | sim: armed/disarmed/magic/reset assertions | Modify |
| `gateware/rev2/project_obscurus/slot_rom.S` | arm on `PR#4` entry + fixed `DISARM` at `$C420` | Modify |
| `gateware/rev2/project_obscurus/monitor.S` | `T` command (inline sweep) + `Q`→`JMP $C420` | Modify |
| `software/SDM/SDRAMLIB.S` | add `SDM_ROMON`/`SDM_ROMOFF` helpers | Modify |

---

## Task 1: rom2mem fill byte + Makefile RTS fill

**Files:** Modify `scripts/rom2mem.py`, `Makefile`

- [ ] **Step 1: Add optional fill-byte arg to rom2mem.py**

Replace the function signature + fill line. Current:
```python
def rom2mem(bin_path, mem_path, base=0xC400, size=4096):
    with open(bin_path, "rb") as f:
        data = f.read()

    rom = bytearray([0xFF] * size)
```
New:
```python
def rom2mem(bin_path, mem_path, base=0xC400, size=4096, fill=0xFF):
    with open(bin_path, "rb") as f:
        data = f.read()

    rom = bytearray([fill] * size)
```
And the arg parsing at the bottom. Current:
```python
    size = int(sys.argv[4], 0) if len(sys.argv) > 4 else 4096
    rom2mem(bin_path, mem_path, base, size)
```
New:
```python
    size = int(sys.argv[4], 0) if len(sys.argv) > 4 else 4096
    fill = int(sys.argv[5], 0) if len(sys.argv) > 5 else 0xFF
    rom2mem(bin_path, mem_path, base, size, fill)
```
(Existing callers with 4 args are unaffected — fill defaults to `0xFF`.)

- [ ] **Step 2: Point the monitor.mem recipe at fill `0x60` (RTS)**

In `Makefile`, the `$(OBSCURUS_MON_MEM)` recipe's `rom2mem.py` line currently reads:
```make
	python3 scripts/rom2mem.py $(GATEWARE_DIR)/project_obscurus/monitor.bin $@ 0xC000 2048
```
Change to add `0x60`:
```make
	python3 scripts/rom2mem.py $(GATEWARE_DIR)/project_obscurus/monitor.bin $@ 0xC000 2048 0x60
```

- [ ] **Step 3: Regenerate monitor.mem and verify unused region is `60`**

Run:
```bash
cd /Users/hambook/Development/project_byte_hamr
python3 scripts/rom2mem.py gateware/rev2/project_obscurus/monitor.bin gateware/rev2/project_obscurus/monitor.mem 0xC000 2048 0x60
sed -n '2048p' gateware/rev2/project_obscurus/monitor.mem    # $CFFF byte
sed -n '1780p' gateware/rev2/project_obscurus/monitor.mem    # $CEF3 region byte
```
Expected: both print `60` (RTS fill), while the early lines (monitor code) are unchanged.

- [ ] **Step 4: Commit**

```bash
git add scripts/rom2mem.py Makefile gateware/rev2/project_obscurus/monitor.mem
git commit -m "build: rom2mem fill-byte arg; fill obscurus monitor unused ROM with \$60 (RTS)"
```

---

## Task 2: Gateware `rom_armed` soft-switch + drive gate (+ sim)

**Files:** Modify `gateware/rev2/project_obscurus/project_obscurus_top.v`, `gateware/rev2/project_obscurus/project_obscurus_tb.v`

- [ ] **Step 1: Add the failing sim scenario (red)**

In `project_obscurus_tb.v`, after the existing scenario 7 (the `$CFFF` check) and before the final PASS/FAIL print, add an armed-ROM block. (The TB already has `wr_reg(r,d)` and `rd_reg(r,d)` tasks driving `$C0Cx`, and access to `dut.` internals.)

```verilog
        // 8. expansion ROM is SILENT by default (disarmed): reading $C8xx must
        //    not drive the bus. Drive a $C4xx access first to set rom_en, then
        //    read $C800 — DATA_OE must stay deasserted and our card must not own D.
        apple_addr = 16'hC400; nI_O_SELECT=1'b0; R_nW=1'b1; #200;
        nI_O_SELECT=1'b1; #100;                       // rom_en now set
        apple_addr = 16'hC800; nI_O_STROBE=1'b0; R_nW=1'b1; #200;
        if (dut.rom_armed !== 1'b0) begin errors=errors+1; $display("FAIL armed at reset"); end
        if (dut.exp_read !== 1'b0)  begin errors=errors+1; $display("FAIL exp_read while disarmed"); end
        nI_O_STROBE=1'b1; #100;

        // 9. ARM via $C0C7<-$AA, then $C800 read drives; DISARM via $C0C8<-$AA stops it.
        wr_reg(4'h7, 8'hAA);                          // arm
        if (dut.rom_armed !== 1'b1) begin errors=errors+1; $display("FAIL not armed"); end
        wr_reg(4'h8, 8'hAA);                          // disarm
        if (dut.rom_armed !== 1'b0) begin errors=errors+1; $display("FAIL not disarmed"); end

        // 10. magic guard: wrong value does NOT arm.
        wr_reg(4'h7, 8'h55);
        if (dut.rom_armed !== 1'b0) begin errors=errors+1; $display("FAIL armed by non-magic"); end

        // 11. reset disarms: arm, assert reset, must clear.
        wr_reg(4'h7, 8'hAA);
        nRES_READ=1'b0; #500; nRES_READ=1'b1; #200;
        if (dut.rom_armed !== 1'b0) begin errors=errors+1; $display("FAIL reset did not disarm"); end
        @(posedge dut.ready);                          // let SDRAM re-init after reset (optional wait)
```
> Note: scenario 11 asserts reset, which re-inits SDRAM; the `@(posedge dut.ready)` is a courtesy wait. If it stalls the sim, replace with `#2000;`.

- [ ] **Step 2: Run sim — expect FAIL (red)**

Run:
```bash
make sim DESIGN=project_obscurus REV=rev2 2>&1 & P=$!; for i in $(seq 1 60); do kill -0 $P 2>/dev/null && sleep 1 || break; done; kill -0 $P 2>/dev/null && { echo HANG; pkill -9 -f "make sim"; pkill -9 ivl; pkill -9 vvp; } || echo "make-exited-${i}s"
```
Expected: compile error or FAIL — `dut.rom_armed`/`dut.exp_read` don't exist yet (or `exp_read` is always-on). This is the red state.

- [ ] **Step 3: Add `rom_armed` FF + gate the `$C800` drive**

In `project_obscurus_top.v`, the `rom_en` block currently reads:
```verilog
    reg rom_en = 1'b0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rom_en <= 1'b0;
        else begin
            if (~nI_O_SELECT) rom_en <= 1'b1;
            if (~nI_O_STROBE && apple_addr[10:0]==11'h7FF) rom_en <= 1'b0; // $CFFF
        end
    end
```
Immediately AFTER that block, add the `rom_armed` FF (arm `$C0C7←$AA`, disarm `$C0C8←$AA`; `rst_n` covers nRES + POR):
```verilog
    // Expansion-ROM ARM soft-switch (default 0 = card silent on shared $C800 bus).
    // Arm:  write $AA to $C0C7.  Disarm: write $AA to $C0C8.  Reset/POR -> disarm.
    // Symmetric magic guard: only the exact byte to the exact address changes state,
    // so stray/rogue writes can't arm us. Gates ALL $C800-$CFFF drive below.
    reg rom_armed = 1'b0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rom_armed <= 1'b0;
        else if (nds_rise & ~wr_rw_latch & (wr_data_latch == 8'hAA)) begin
            if      (wr_addr_latch == 4'h7) rom_armed <= 1'b1;   // ROM_ARM
            else if (wr_addr_latch == 4'h8) rom_armed <= 1'b0;   // ROM_DISARM
        end
    end
```
Then gate the expansion drive. Current:
```verilog
    wire       exp_read = rom_en & ~nI_O_STROBE & R_nW;
```
New:
```verilog
    wire       exp_read = rom_en & rom_armed & ~nI_O_STROBE & R_nW;
```
And the `DATA_OE` term. Current:
```verilog
    wire slot_active = ~nDEVICE_SELECT | ~nI_O_SELECT | (rom_en & ~nI_O_STROBE);
```
New:
```verilog
    wire slot_active = ~nDEVICE_SELECT | ~nI_O_SELECT | (rom_en & rom_armed & ~nI_O_STROBE);
```

- [ ] **Step 4: Shrink scratch + ignore `$C7`/`$C8` as scratch**

The scratch loopback currently captures `wr_addr_latch >= 4'h7`. That now overlaps the arm/disarm registers. Change it to `>= 4'h9`. Current:
```verilog
        end else if (nds_rise & ~wr_rw_latch & (wr_addr_latch >= 4'h7)) begin
            scratch[wr_addr_latch] <= wr_data_latch;
        end
```
New:
```verilog
        end else if (nds_rise & ~wr_rw_latch & (wr_addr_latch >= 4'h9)) begin
            scratch[wr_addr_latch] <= wr_data_latch;
        end
```
(The `reg_wr` case for `$C0C0–$C0C6` is unchanged; `$C7`/`$C8` are handled by the `rom_armed` block above and need no entry there. The read mux still returns `scratch[apple_addr[3:0]]` for the default case — reads of `$C7`/`$C8` return stale scratch, harmless.)

- [ ] **Step 5: Run sim — expect PASS (green)**

Run the same hang-guarded `make sim` as Step 2.
Expected: `PASS`, `make-exited`. All prior scenarios (1–7) plus the new 8–11 pass: disarmed = no `$C800` drive, arm/disarm via magic works, non-magic ignored, reset disarms. If a scenario FAILs, fix the RTL (not the TB).

- [ ] **Step 6: Commit**

```bash
git add gateware/rev2/project_obscurus/project_obscurus_top.v gateware/rev2/project_obscurus/project_obscurus_tb.v
git commit -m "feat: rom_armed soft-switch gates \$C800 drive (default silent); sim asserts arm/disarm/magic/reset"
```

---

## Task 3: `$C400` slot ROM — arm on entry + `DISARM` routine

**Files:** Modify `gateware/rev2/project_obscurus/slot_rom.S`

- [ ] **Step 1: Rewrite the stub to arm + add a fixed `DISARM` at `$C420`**

Replace `slot_rom.S` entirely with:
```
* =============================================================================
* slot_rom.S — project_obscurus $C400 slot ROM (Merlin32)
* =============================================================================
* PR#4 does JSR $C400 -> ENTRY: ARM the $C800 expansion ROM ($C0C7<-$AA),
* restore CSW=COUT1 so output reaches the screen, then JMP $C800 (monitor).
*
* DISARM (fixed at $C420) lives in the ALWAYS-LIVE slot ROM (gated only by
* nI_O_SELECT, never by rom_armed), so it keeps executing after it disarms the
* $C800 ROM. The monitor's Q command JMPs here: it disarms ($C0C8<-$AA) and
* RTSes back to BASIC over PR#'s JSR return address.
* =============================================================================
            TYP   $06
            DSK   slot_rom.bin
            ORG   $C400

CSWL   =    $36
CSWH   =    $37
COUT1  =    $FDF0
MONITOR =   $C800
ROMARM  =    $C0C7
ROMDIS  =    $C0C8
MAGIC  =    $AA

ENTRY       LDA   #MAGIC
            STA   ROMARM          ; arm $C800
            LDA   #<COUT1
            STA   CSWL
            LDA   #>COUT1
            STA   CSWH
            JMP   MONITOR

            DS    $C420-*         ; pad so DISARM lands at a fixed address
DISARM      LDA   #MAGIC
            STA   ROMDIS          ; disarm $C800 (executes from always-live slot ROM)
            RTS                   ; back to BASIC
```

- [ ] **Step 2: Assemble + verify ENTRY and DISARM placement**

Run:
```bash
cd /Users/hambook/Development/project_byte_hamr
make assemble ASM_SRC=gateware/rev2/project_obscurus/slot_rom.S
xxd gateware/rev2/project_obscurus/slot_rom.bin | head -1     # first byte a9 (LDA #MAGIC)
python3 -c "d=open('gateware/rev2/project_obscurus/slot_rom.bin','rb').read(); print('len',len(d)); print('byte@\$C420 (off 0x20):', hex(d[0x20]) if len(d)>0x20 else 'short')"
```
Expected: first byte `a9`; length ≥ `0x24`; byte at offset `0x20` = `a9` (the `LDA #MAGIC` of `DISARM`). This confirms `DISARM` is exactly at `$C420`.

- [ ] **Step 3: Regenerate slot_rom.mem**

Run:
```bash
python3 scripts/rom2mem.py gateware/rev2/project_obscurus/slot_rom.bin gateware/rev2/project_obscurus/slot_rom.mem 0xC000 256
```
(Slot ROM keeps `$FF` fill — it's slot-gated/always-live, only entered at known addresses, never executed past its routines.)

- [ ] **Step 4: Commit**

```bash
git add gateware/rev2/project_obscurus/slot_rom.S gateware/rev2/project_obscurus/slot_rom.mem
git commit -m "feat: \$C400 stub arms \$C800 on PR#4; DISARM routine at \$C420 (always-live)"
```

---

## Task 4: Monitor `T` command + `Q` disarm

**Files:** Modify `gateware/rev2/project_obscurus/monitor.S`

The monitor already has: register equates `RADDRLO=$C0C0 … RDATA=$C0C6`, routines `SETADDR`, `POLLB` (`LDA RSTATUS`/`BMI`), `PRBYTE`/`COUT`/`CROUT`/`PRSTR`, scratch `ADDR=$0300`/`BANK=$0303`/`DBUF=$0305`, dispatch chain (`CMP #"R"` … `CMP #"D"` … `CMP #"Q"`), and string-print via `PRSTR` (STRPTR = `$06`).

- [ ] **Step 1: Add the `T` dispatch + change `Q` to JMP DISARM**

In the dispatch chain, the `:nD` / `Q` section currently reads:
```
:nD         CMP   #"Q"
            BNE   :nErr
            RTS                    ; back to BASIC
:nErr       JSR   PRERR
            JMP   PROMPT
```
Replace with (insert `T` before `Q`, route `Q` through DISARM):
```
:nD         CMP   #"T"
            BNE   :nT
            JMP   DOTEST
:nT         CMP   #"Q"
            BNE   :nErr
            JMP   DISARM           ; $C420 slot ROM: disarm $C800 + RTS to BASIC
:nErr       JSR   PRERR
            JMP   PROMPT
```
Add the equate near the other equates at the top of `monitor.S`:
```
DISARM  =   $C420                  ; slot ROM disarm routine (always-live)
```

- [ ] **Step 2: Add the `T` bank-sweep handler**

Add these scratch equates near the monitor's other `$0300`-page equates (they sit above the monitor's `$0300–$0314` usage, no overlap):
```
TBANK   =   $0315                  ; 2 bytes: sweep bank counter
TFAIL   =   $0317                  ; 2 bytes: failure counter
TGOT    =   $0319                  ; 1 byte: last byte read
```
Add the handler (place it with the other `DOxxxx` handlers; it uses the monitor's own `SETADDR`/`POLLB` + register equates, value scheme `lo^hi^off^$5A`, two-phase write-all/read-all over offsets 0–9 of banks 0–1023):
```
* ---- T: 1024-bank two-phase sweep ----
DOTEST      LDA   #0
            STA   TFAIL
            STA   TFAIL+1
* phase 1: write all
            LDA   #0
            STA   TBANK
            STA   TBANK+1
:p1         LDA   TBANK
            STA   BANK
            LDA   TBANK+1
            STA   BANK+1
            LDA   #0
            STA   ADDR
            STA   ADDR+1
            JSR   SETADDR
            LDX   #0
:w1         TXA
            EOR   TBANK
            EOR   TBANK+1
            EOR   #$5A
            STA   RDATA            ; A=expected -> DATA write strobe (auto-inc)
            JSR   POLLB
            INX
            CPX   #10
            BNE   :w1
            INC   TBANK
            BNE   :n1
            INC   TBANK+1
:n1         LDA   TBANK+1
            CMP   #$04             ; 1024 = $0400
            BCC   :p1
* phase 2: read all + compare
            LDA   #0
            STA   TBANK
            STA   TBANK+1
:p2         LDA   TBANK
            STA   BANK
            LDA   TBANK+1
            STA   BANK+1
            LDA   #0
            STA   ADDR
            STA   ADDR+1
            JSR   SETADDR
            LDX   #0
:r1         STA   RTRIG            ; trigger read (A don't-care, auto-inc)
            JSR   POLLB
            LDA   RDATA
            STA   TGOT
            TXA
            EOR   TBANK
            EOR   TBANK+1
            EOR   #$5A
            CMP   TGOT
            BEQ   :rok
            INC   TFAIL
            BNE   :rok
            INC   TFAIL+1
:rok        INX
            CPX   #10
            BNE   :r1
            INC   TBANK
            BNE   :n2
            INC   TBANK+1
:n2         LDA   TBANK+1
            CMP   #$04
            BCC   :p2
* result
            JSR   CROUT
            LDA   TFAIL
            ORA   TFAIL+1
            BNE   :tfail
            LDA   #<MTPASS
            STA   STRPTR
            LDA   #>MTPASS
            STA   STRPTR+1
            JSR   PRSTR
            JSR   CROUT
            JMP   PROMPT
:tfail      LDA   TFAIL+1
            JSR   PRBYTE
            LDA   TFAIL
            JSR   PRBYTE
            LDA   #<MTFAIL
            STA   STRPTR
            LDA   #>MTFAIL
            STA   STRPTR+1
            JSR   PRSTR
            JSR   CROUT
            JMP   PROMPT
```
Add the two strings near the monitor's other `ASC` strings:
```
MTPASS      ASC   "TEST PASS"
            DFB   $00
MTFAIL      ASC   " TEST FAIL"
            DFB   $00
```
> `RTRIG` must already be equated in monitor.S (`RTRIG = $C0C4`). If the monitor uses a different name for `$C0C4`, use that name. (Confirm against the file's equate block.)

- [ ] **Step 3: Assemble + verify size leaves `$CFFF` free**

Run:
```bash
cd /Users/hambook/Development/project_byte_hamr
make assemble ASM_SRC=gateware/rev2/project_obscurus/monitor.S
xxd gateware/rev2/project_obscurus/monitor.bin | head -1     # first byte 20 (JSR)
wc -c < gateware/rev2/project_obscurus/monitor.bin           # must be < 2046
```
Expected: first byte `20`; size well under 2046. If Merlin32 errors, fix syntax only (preserve behavior) and report.

- [ ] **Step 4: Rebuild monitor.mem (RTS fill) + confirm**

Run:
```bash
python3 scripts/rom2mem.py gateware/rev2/project_obscurus/monitor.bin gateware/rev2/project_obscurus/monitor.mem 0xC000 2048 0x60
sed -n '2048p' gateware/rev2/project_obscurus/monitor.mem    # expect 60
```

- [ ] **Step 5: Integration sim still passes (monitor.mem changed)**

Run the hang-guarded `make sim DESIGN=project_obscurus REV=rev2`.
Expected: `PASS` (the TB `$readmemh`s monitor.mem; scenario 7 now accepts `$60` too — if scenario 7 asserts `!== 8'hFF`, update it to also allow `8'h60`: change to `!== 8'hFF && dut.monitor_mem[11'h7FF] !== 8'h60`).

- [ ] **Step 6: Commit**

```bash
git add gateware/rev2/project_obscurus/monitor.S gateware/rev2/project_obscurus/monitor.mem gateware/rev2/project_obscurus/project_obscurus_tb.v
git commit -m "feat: monitor T command (1024-bank sweep) + Q disarms via \$C420"
```

---

## Task 5: `SDRAMLIB` arm/disarm helpers

**Files:** Modify `software/SDM/SDRAMLIB.S`

- [ ] **Step 1: Add `SDM_ROMON`/`SDM_ROMOFF` + register equates**

In `SDRAMLIB.S`, after the existing `R_DATA = SDMBASE+6` equate, add:
```
R_ARM     =     SDMBASE+7         ; $C0C7 — write $AA to arm $C800 ROM
R_DISARM  =     SDMBASE+8         ; $C0C8 — write $AA to disarm
```
After the existing entry-point routines (e.g. after `SDM_RDNEXT`), add:
```
* ---- $C800 expansion-ROM arm/disarm (ONLY needed to run ROM-resident code;
*      SDRAM data access via SDM_READ/WRITE NEVER needs this) ----
SDM_ROMON LDA   #$AA
          STA   R_ARM
          RTS
SDM_ROMOFF
          LDA   #$AA
          STA   R_DISARM
          RTS
```

- [ ] **Step 2: Assemble-check (via the standalone test that PUTs it) + invariant grep**

Run:
```bash
cd /Users/hambook/Development/project_byte_hamr
make sdmtest 2>&1 | tail -2
echo "== lib still has no indexed $C0Cx (empty) =="; grep -nE "R_(ADRLO|ADRHI|BNKLO|BNKHI|TRIG|STAT|DATA|ARM|DISARM),[XY]" software/SDM/SDRAMLIB.S
```
Expected: `make sdmtest` assembles clean (SDMTEST is unchanged and still PUTs SDRAMLIB); grep prints nothing. `SDMTEST` behavior is unchanged (it never calls the new helpers).

- [ ] **Step 3: Commit**

```bash
git add software/SDM/SDRAMLIB.S
git commit -m "feat: SDRAMLIB SDM_ROMON/SDM_ROMOFF helpers (\$C800 arm/disarm)"
```

---

## Task 6: Bitstream build + docs + bench

**Files:** Modify `gateware/rev2/project_obscurus/README.md`; memory

- [ ] **Step 1: Full clean bitstream build (no flash)**

Run:
```bash
cd /Users/hambook/Development/project_byte_hamr
make clean && make DESIGN=project_obscurus REV=rev2 2>&1 | tail -5
ls -la build/project_obscurus.bit
```
Expected: `Bitstream ready: build/project_obscurus.bit`, exit 0, timing closes. (Per CLAUDE.md: do NOT `prog-flash` — the user flashes.)

- [ ] **Step 2: Update the README**

In `gateware/rev2/project_obscurus/README.md`, add an "Expansion ROM arming" section: the card is silent on `$C800–$CFFF` by default; arm with `$C0C7←$AA`, disarm with `$C0C8←$AA`; reset/POR auto-disarm; `PR#4` arms→monitor (R/W/B/D + **`T`**=bank test)→`Q` disarms; SDRAM `$C0Cx` access is always-live and never arms. Note the `rom_en` requirement for `$C800` and the "no disk I/O / IRQ-`$C800` while armed" rule.

- [ ] **Step 3: Bench checklist (user runs on hardware after flashing)**

Document + report:
1. Card installed, **no `PR#4`**: `CATALOG`, then **copy a file with CopyUtils** → no crash, no `$CEF3`, disks intact. (The regression.)
2. `PR#4` → monitor banner; `R 0`/`W 0 AA`/`R 0`/`B 1`/`D 0` behave; **`T`** → `TEST PASS`.
3. `Q` → back to `]`; then `CATALOG`/copy again → still clean.
4. Ctrl-Reset mid-monitor → back to BASIC, card silent, disks fine.
Capture via `obs-screenshot`.

- [ ] **Step 4: Update memory + commit**

Append to `project_obscurus_monitor.md` (or a new note): the armed-ROM fix — `rom_armed` default 0, `$C0C7`/`$C0C8` magic `$AA` arm/disarm, reset-disarm, RTS fill, `DISARM` at `$C420`, monitor `T`. Add one-line MEMORY.md update. Then:
```bash
git add gateware/rev2/project_obscurus/README.md
git commit -m "docs: armed expansion ROM — arm/disarm protocol + bench checklist"
```

---

## Self-Review (addressed)

- **Spec coverage:** `rom_armed` FF + gate (Task 2), magic `$AA` symmetric arm/disarm + reset/POR disarm (Task 2), RTS fill (Task 1), register map `$C0C7`/`$C0C8` + scratch shrink (Task 2), `$C400` arm + `DISARM` always-live (Task 3), monitor `T` + `Q` disarm (Task 4), `SDM_ROMON/OFF` (Task 5), SDRAM stays unarmed (untouched register path), sim arm/disarm/magic/reset (Task 2), bench CATALOG/copy regression (Task 6). Software-contract/timing is documentation (in spec/README), no code beyond the above.
- **Deliberate deviation from spec's DRY `SDMSWEEP.S`:** the monitor's `T` reuses the monitor's own proven register routines inline rather than refactoring the bench-verified `SDRAMLIB.S`/`SDMTEST.S` (which use DS vars unsuitable for a `$C800`-ROM context). Lower risk; ~35 lines of sweep duplicated. The standalone `SDMTEST` is untouched and still the unarmed proof.
- **Name consistency:** `rom_armed`, `R_ARM=$C0C7`/`R_DISARM=$C0C8`, magic `$AA`, `DISARM=$C420`, `TBANK/TFAIL/TGOT` used consistently across RTL, slot_rom.S, monitor.S, SDRAMLIB.S. `RTRIG=$C0C4` reused in the monitor sweep (verify the file's equate name in Task 4 Step 2).
- **Placeholder scan:** complete code/commands in every step; the only conditional ("if scenario 7 asserts `!== 8'hFF`") gives the exact edit.
