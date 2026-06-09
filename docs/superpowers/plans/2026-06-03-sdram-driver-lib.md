# SDRAM Driver Library + Bank Test Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A reusable Merlin `PUT`-include driver library (`SDRAMLIB.S`) for the project_obscurus SDRAM register port, plus a test program (`SDMTEST.S`) that imports the same library and verifies the first 10 offsets of all 1024 banks, packaged on a bootable ProDOS floppy.

**Architecture:** The library talks to the always-live `$C0Cx` register port with plain absolute `STA`/`LDA` only (no zero page, no `PR#4`, no `$C800`/`$C400` ROM). It exposes a param-block API (`SDM_BANK`/`SDM_ADDR`/`SDM_VAL` + `SDM_READY/SETBANK/SETADDR/WRITE/READ/WRNEXT/RDNEXT`). The test `PUT`s the library and runs a two-phase write-all/read-all sweep, reporting `PASS`/`FAILURES`. Disk built with acx (AppleCommander) from the bootable ProDOS 2.4.1 base.

**Tech Stack:** Merlin32 (`$(MERLIN32)`), AppleCommander acx (`java -jar /Users/hambook/Development/AppleCommander-ac-13.0.jar`), ProDOS 8 + BASIC.SYSTEM, ADTPro for floppy delivery.

**Spec:** `docs/superpowers/specs/2026-06-03-sdram-driver-lib-design.md`

---

## Background the implementer needs

- **No hardware/sim here.** There is no 6502 simulator in this repo. "Tests" for the asm are: (a) it **assembles clean** under Merlin32, (b) **source-invariant greps** pass (no ZP in the lib, no `$C4`/`$C8`/`PR#`, no indexed access to `$C0Cx`), (c) the **disk builds** and `acx list` shows the right files/types. Runtime correctness is verified on the user's Apple II (bench checklist).
- **Merlin32 idioms** (match `gateware/rev2/project_obscurus/monitor.S`): labels in column 1, `:local` labels, equates with `=`, `DS n` reserves n bytes, `ASC "..."` stores text **high-bit-set** (confirmed on this toolchain — so no `ORA #$80` needed before `COUT`), `DFB` for raw bytes, `PUT NAME` includes `NAME.S` from the same directory.
- **`PUT` + ProDOS naming:** the library file is `SDRAMLIB.S`; `PUT SDRAMLIB` resolves it. ProDOS filenames disallow underscores, so on disk it is `SDRAMLIB` (TXT) and on the Apple `PUT SDRAMLIB` works.
- **Register port (slot 4):** `$C0C0` ADDR_LO(W), `$C0C1` ADDR_HI(W), `$C0C2` BANK_LO(W), `$C0C3` BANK_HI(W), `$C0C4` TRIG_RD(W strobe), `$C0C5` STATUS(R: bit7 busy, bit6 ready), `$C0C6` DATA(R last read / W write). Auto-increment fires only on a completed access (TRIG_RD strobe or DATA write); a DATA read does NOT advance. busy is sticky from strobe until the first STATUS read after completion.
- **acx invocation** (verified): `java -jar /Users/hambook/Development/AppleCommander-ac-13.0.jar <cmd> ...`. `list -d <img>`; `import -d <img> --raw -t BIN -a 0x2000 -n SDMTEST <file>`; `import -d <img> --text -t TXT -n SDRAMLIB <file>` (`--text` sets high bit + `$8D` newlines = Merlin-on-Apple format); `rm -d <img> <NAME>` (acx `delete` alias) to free space.

## File Structure

| File | Responsibility | Action |
|------|----------------|--------|
| `software/SDM/SDRAMLIB.S` | The driver library: equates, param vars, entry points. `PUT`-included; no ORG, no ZP. | Create |
| `software/SDM/SDMTEST.S` | Test program: `ORG $2000`, `PUT SDRAMLIB`, two-phase 1024-bank sweep, output. | Create |
| `software/SDM/README.md` | How to build/run, the API, on-Apple `PUT` usage. | Create |
| `Makefile` | `sdmtest` (assemble) + `sdmdisk` (build bootable .po) targets. | Modify |

---

## Task 1: The driver library `SDRAMLIB.S`

**Files:**
- Create: `software/SDM/SDRAMLIB.S`

- [ ] **Step 1: Write the library**

```
* =============================================================================
* SDRAMLIB.S — project_obscurus SDRAM register-port driver (Merlin32)
* =============================================================================
* PUT-include library. NO ORG, NO zero page, NO PR#4 / $C800 / $C400 ROM.
* All access is plain absolute STA/LDA to $C0Cx (single nDEVICE_SELECT pulse;
* never indexed, to avoid the STA abs,X dummy-read double-count).
*
* Public params (set, then JSR an entry point):
*   SDM_BANK  (2 bytes)  10-bit bank (high 6 bits ignored by hardware)
*   SDM_ADDR  (2 bytes)  16-bit offset within bank
*   SDM_VAL   (1 byte)   value to write / last value read
*
* Entry points:
*   SDM_READY   bounded wait for ready. RETURNS C=0 ready, C=1 timeout.
*   SDM_SETBANK push SDM_BANK -> bank registers
*   SDM_SETADDR push SDM_ADDR -> addr registers
*   SDM_WRITE   setbank+setaddr, write SDM_VAL, wait busy clear
*   SDM_READ    setbank+setaddr, trigger read, wait, load -> A and SDM_VAL
*   SDM_WRNEXT  write SDM_VAL at hardware auto-inc addr (no setaddr), wait
*   SDM_RDNEXT  trigger read at auto-inc addr, wait, load -> A and SDM_VAL
* WRNEXT/RDNEXT require a prior SETADDR (or WRITE/READ) to set the start; the
* hardware then advances by one per completed access. No bank carry on wrap.
*
* Slot configurable by the single SLOT equate.
* =============================================================================

SLOT      =     4
SDMBASE   =     $C080+SLOT*16     ; $C0C0
R_ADRLO   =     SDMBASE+0
R_ADRHI   =     SDMBASE+1
R_BNKLO   =     SDMBASE+2
R_BNKHI   =     SDMBASE+3
R_TRIG    =     SDMBASE+4
R_STAT    =     SDMBASE+5
R_DATA    =     SDMBASE+6

SDM_BANK  DS    2
SDM_ADDR  DS    2
SDM_VAL   DS    1

* ---- bounded ready wait: C=0 ready, C=1 timeout (~65536 polls) ----
SDM_READY LDX   #0
          LDY   #0
:lp       BIT   R_STAT
          BVS   :rdy            ; bit6 (ready) -> V set
          INY
          BNE   :lp
          INX
          BNE   :lp
          SEC                   ; timed out
          RTS
:rdy      CLC
          RTS

SDM_SETBANK
          LDA   SDM_BANK
          STA   R_BNKLO
          LDA   SDM_BANK+1
          STA   R_BNKHI
          RTS

SDM_SETADDR
          LDA   SDM_ADDR
          STA   R_ADRLO
          LDA   SDM_ADDR+1
          STA   R_ADRHI
          RTS

* ---- wait busy clear (bit7). ops always complete; no timeout ----
SDM_POLL  LDA   R_STAT
          BMI   SDM_POLL        ; bit7 (busy) -> N set -> loop
          RTS

SDM_WRITE JSR   SDM_SETBANK
          JSR   SDM_SETADDR
          LDA   SDM_VAL
          STA   R_DATA          ; latch + trigger write
          JMP   SDM_POLL        ; tail call (RTS from POLL)

SDM_READ  JSR   SDM_SETBANK
          JSR   SDM_SETADDR
          STA   R_TRIG          ; any write triggers a read (value ignored)
          JSR   SDM_POLL
          LDA   R_DATA
          STA   SDM_VAL
          RTS

SDM_WRNEXT
          LDA   SDM_VAL
          STA   R_DATA
          JMP   SDM_POLL

SDM_RDNEXT
          STA   R_TRIG
          JSR   SDM_POLL
          LDA   R_DATA
          STA   SDM_VAL
          RTS
```

- [ ] **Step 2: Invariant grep — library must be ZP-free and ROM-free**

Run:
```bash
cd /Users/hambook/Development/project_byte_hamr
echo "== indexed $C0Cx (must be empty) =="; grep -nE "R_(ADRLO|ADRHI|BNKLO|BNKHI|TRIG|STAT|DATA),[XY]" software/SDM/SDRAMLIB.S
echo "== forbidden ROM/PR refs (must be empty) =="; grep -nE "\\\$C8|\\\$C4|PR#|FDED|FD6A|C800|C400" software/SDM/SDRAMLIB.S
echo "== done =="
```
Expected: both greps print nothing (only the `== ... ==` lines). The library references only `$C0Cx` (via `SDMBASE`) and uses no indexed register access, no monitor ROM, no PR#.

- [ ] **Step 3: Commit**

```bash
git add software/SDM/SDRAMLIB.S
git commit -m "feat: SDRAMLIB.S — PUT-include SDRAM register-port driver (param-block API)"
```

---

## Task 2: The bank test `SDMTEST.S`

**Files:**
- Create: `software/SDM/SDMTEST.S`

- [ ] **Step 1: Write the test program**

```
* =============================================================================
* SDMTEST.S — project_obscurus SDRAM bank test (Merlin32)
* =============================================================================
* BRUN at $2000. PUTs SDRAMLIB. Two-phase write-all/read-all over offsets
* $00-$09 of all 1024 banks. expected = lo(bank) EOR hi(bank) EOR off EOR $5A.
* Output: banner, '.' per 64 banks, first 16 mismatches, then PASS / nnnn FAILURES.
* NO CARD + quit if the card isn't ready (SDM_READY timeout).
* =============================================================================
            TYP   $06
            DSK   SDMTEST
            ORG   $2000

            JMP   MAIN
            PUT   SDRAMLIB        ; lib code + its DS vars live here

COUT      =       $FDED
CROUT     =       $FD8E
PRBYTE    =       $FDDA
STRPTR    =       $06             ; test-local ZP for string printing (app, not lib)

* ---- test state ----
CURBANK   DS      2               ; 0..1023
FAILS     DS      2               ; 16-bit mismatch counter
PRINTED   DS      1               ; mismatch lines printed (cap 16)
GOT       DS      1               ; byte read back

* =============================================================================
MAIN        LDA   #<MBANNER
            STA   STRPTR
            LDA   #>MBANNER
            STA   STRPTR+1
            JSR   PRSTR
            JSR   CROUT

            JSR   SDM_READY
            BCC   :ready
            LDA   #<MNOCARD
            STA   STRPTR
            LDA   #>MNOCARD
            STA   STRPTR+1
            JSR   PRSTR
            JSR   CROUT
            RTS                    ; back to BASIC
:ready
            LDA   #0
            STA   FAILS
            STA   FAILS+1
            STA   PRINTED

* ---------- PHASE 1: write all ----------
            LDA   #0
            STA   CURBANK
            STA   CURBANK+1
P1          JSR   DOTCHK
            LDA   CURBANK
            STA   SDM_BANK
            LDA   CURBANK+1
            STA   SDM_BANK+1
            JSR   SDM_SETBANK
            LDA   #0
            STA   SDM_ADDR
            STA   SDM_ADDR+1
            JSR   SDM_SETADDR
            LDX   #0
:wl         JSR   CALCEXP          ; A = expected for offset X
            STA   SDM_VAL
            JSR   SDM_WRNEXT
            INX
            CPX   #10
            BNE   :wl
            JSR   BANKINC
            BCC   P1               ; carry clear = more banks

* ---------- PHASE 2: read all ----------
            LDA   #0
            STA   CURBANK
            STA   CURBANK+1
P2          JSR   DOTCHK
            LDA   CURBANK
            STA   SDM_BANK
            LDA   CURBANK+1
            STA   SDM_BANK+1
            JSR   SDM_SETBANK
            LDA   #0
            STA   SDM_ADDR
            STA   SDM_ADDR+1
            JSR   SDM_SETADDR
            LDX   #0
:rl         JSR   SDM_RDNEXT       ; A = byte read
            STA   GOT
            JSR   CALCEXP          ; A = expected
            CMP   GOT
            BEQ   :ok
            JSR   ONFAIL
:ok         INX
            CPX   #10
            BNE   :rl
            JSR   BANKINC
            BCC   P2

* ---------- result ----------
            JSR   CROUT
            LDA   FAILS
            ORA   FAILS+1
            BNE   :fail
            LDA   #<MPASS
            STA   STRPTR
            LDA   #>MPASS
            STA   STRPTR+1
            JSR   PRSTR
            JSR   CROUT
            RTS
:fail       LDA   FAILS+1
            JSR   PRBYTE
            LDA   FAILS
            JSR   PRBYTE
            LDA   #<MFAIL
            STA   STRPTR
            LDA   #>MFAIL
            STA   STRPTR+1
            JSR   PRSTR
            JSR   CROUT
            RTS

* ---- expected = X(offset) EOR lo(bank) EOR hi(bank) EOR $5A ; clobbers A ----
CALCEXP     TXA
            EOR   CURBANK
            EOR   CURBANK+1
            EOR   #$5A
            RTS

* ---- CURBANK++ ; returns C=0 if more (CURBANK<1024), C=1 if done ----
BANKINC     INC   CURBANK
            BNE   :nc
            INC   CURBANK+1
:nc         LDA   CURBANK+1
            CMP   #$04             ; 1024 = $0400
            RTS                    ; C set when hi>=4 -> done

* ---- print '.' when (CURBANK & $3F)==0 ----
DOTCHK      LDA   CURBANK
            AND   #$3F
            BNE   :skip
            LDA   #"."
            JSR   COUT
:skip       RTS

* ---- on mismatch: FAILS++ ; print line if under cap (16) ----
ONFAIL      INC   FAILS
            BNE   :nf
            INC   FAILS+1
:nf         LDA   PRINTED
            CMP   #16
            BCS   :done            ; already printed 16 -> suppress
            INC   PRINTED
            LDA   CURBANK+1
            JSR   PRBYTE
            LDA   CURBANK
            JSR   PRBYTE
            LDA   #"/"
            JSR   COUT
            TXA                    ; offset
            JSR   PRBYTE
            LDA   #" "
            JSR   COUT
            JSR   CALCEXP          ; expected
            JSR   PRBYTE
            LDA   #" "
            JSR   COUT
            LDA   GOT
            JSR   PRBYTE
            JSR   CROUT
:done       RTS

* ---- print high-bit-set, $00-terminated string at (STRPTR) ----
PRSTR       LDY   #0
:l          LDA   (STRPTR),Y
            BEQ   :x
            JSR   COUT
            INY
            BNE   :l
:x          RTS

MBANNER     ASC   "SDM BANK TEST"
            DFB   $00
MNOCARD     ASC   "NO CARD"
            DFB   $00
MPASS       ASC   "PASS"
            DFB   $00
MFAIL       ASC   " FAILURES"
            DFB   $00
```

- [ ] **Step 2: Add the assemble target to the Makefile**

Insert near the other Merlin32 recipes (after the `assemble:` rule around line 455-462). Add:
```make
# project_obscurus SDRAM driver test (BRUN BIN at $2000)
SDM_DIR  := software/SDM
sdmtest:
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) SDMTEST.S
```
(`SDMTEST.S` does `PUT SDRAMLIB`, so `SDRAMLIB.S` must sit beside it — it does.)

- [ ] **Step 3: Assemble both files together (the build "test")**

Run: `make sdmtest`
Expected: Merlin32 prints success, no errors; `software/SDM/SDMTEST` (the BIN) and `software/SDM/_FileInformation.txt` are produced. Then verify the entry + load address:
```bash
cd /Users/hambook/Development/project_byte_hamr
xxd software/SDM/SDMTEST | head -1            # expect first byte 4c (JMP MAIN)
grep SDMTEST software/SDM/_FileInformation.txt # expect AuxType(2000)
```
Expected: first byte `4c`; `_FileInformation.txt` shows `AuxType(2000)`. If Merlin32 errors on a `PUT`/syntax issue, fix the source (preserve behavior) and re-run.

- [ ] **Step 4: Invariant grep on the test (no indexed register access)**

Run:
```bash
grep -nE "R_(ADRLO|ADRHI|BNKLO|BNKHI|TRIG|STAT|DATA),[XY]" software/SDM/SDMTEST.S
```
Expected: nothing. (The test drives the port only through the library entry points; it must not touch `$C0Cx` with indexed addressing.)

- [ ] **Step 5: Commit**

```bash
git add software/SDM/SDMTEST.S Makefile
git commit -m "feat: SDMTEST.S — two-phase 1024-bank sweep + sdmtest assemble target"
```

---

## Task 3: Bootable disk build (`sdmdisk` target)

**Files:**
- Modify: `Makefile`

- [ ] **Step 1: Add the `sdmdisk` target**

Insert right after the `sdmtest:` rule:
```make
# Build a bootable ProDOS floppy with SDMTEST (BIN) + SDRAMLIB (TXT source).
# Starts from the bootable 2.4.1 base (has PRODOS + BASIC.SYSTEM), frees space
# by deleting unused bulk files, then imports our two files.
SDM_PO   := $(SDM_DIR)/SDMTEST.po
sdmdisk: sdmtest
	cp $(PRODOS_SRC) $(SDM_PO)
	-$(AC) rm -f -d $(SDM_PO) COPYIIPLUS.7.2
	-$(AC) rm -f -d $(SDM_PO) ADTPRO
	-$(AC) rm -f -d $(SDM_PO) ADTPRO.BIN
	-$(AC) rm -f -d $(SDM_PO) BITSY.BOOT
	-$(AC) rm -f -d $(SDM_PO) QUIT.SYSTEM
	$(AC) import -d $(SDM_PO) -f --raw -t BIN -a 0x2000 -n SDMTEST $(SDM_DIR)/SDMTEST
	$(AC) import -d $(SDM_PO) -f --text -t TXT -n SDRAMLIB $(SDM_DIR)/SDRAMLIB.S
	$(AC) list -d $(SDM_PO)
	@echo "Disk ready: $(SDM_PO) — copy to ADTPro disks and send to floppy."
```
Note: `$(AC)` is already defined in the Makefile as `java -jar $(AC_JAR)` with `AC_JAR := /Users/hambook/Development/AppleCommander-ac-13.0.jar` (the acx interface). The `rm` lines are prefixed `-` so the build continues if a name is already absent. `--text` import sets the high bit + `$8D` newlines, which is exactly what Merlin-on-Apple expects for `PUT SDRAMLIB`.

- [ ] **Step 2: Build the disk and verify contents**

Run: `make sdmdisk`
Expected: `acx list` output shows a bootable volume containing at least `PRODOS` (SYS), `BASIC.SYSTEM` (SYS), `SDMTEST` (BIN, `A=$2000`), and `SDRAMLIB` (TXT), with free space > 0. Confirm:
```bash
java -jar /Users/hambook/Development/AppleCommander-ac-13.0.jar list -d software/SDM/SDMTEST.po
```
Expected: `SDMTEST   BIN ... A=$2000` and `SDRAMLIB  TXT ...` both present; `BASIC.SYSTEM SYS` and `PRODOS SYS` still present (bootable).

- [ ] **Step 3: Verify the imported library is Merlin-loadable text (high-bit)**

Export it back and confirm the high bit is set (so on-Apple `PUT` will read it):
```bash
java -jar /Users/hambook/Development/AppleCommander-ac-13.0.jar export -d software/SDM/SDMTEST.po --raw SDRAMLIB /tmp/sdramlib.out 2>/dev/null
xxd /tmp/sdramlib.out | head -2
```
Expected: bytes have the high bit set (e.g. `*`=`$AA`, letters `$C1`+), and line endings are `$8D` — i.e. values in the `$80`–`$FF` range, not plain ASCII `$20`–`$7E`. (If they're low-ASCII, the `--text` flag didn't apply; fix the import line.)

- [ ] **Step 4: Commit**

```bash
git add Makefile
git commit -m "build: sdmdisk — bootable ProDOS floppy with SDMTEST + SDRAMLIB source"
```

---

## Task 4: Docs, memory, bench checklist

**Files:**
- Create: `software/SDM/README.md`
- Memory: `~/.claude/projects/.../memory/`

- [ ] **Step 1: Write `software/SDM/README.md`**

Document: the `SDM_` API (param vars + entry points + the `SDM_READY` carry contract + the WRNEXT/RDNEXT auto-inc-needs-prior-SETADDR rule + no-bank-carry-on-wrap), the `SLOT` equate, how to `PUT SDRAMLIB` from your own Merlin source (on-Apple example), the build commands (`make sdmtest`, `make sdmdisk`), and the delivery path (copy `SDMTEST.po` to the ADTPro disks folder, send to a floppy, boot, `BRUN SDMTEST`). Include the value scheme `lo^hi^off^$5A` and its single-bit-decode-fault detection guarantee/limit.

- [ ] **Step 2: Bench verification checklist (user runs on hardware)**

Add to the README and report to the user:
- Boot `SDMTEST.po` → `]` prompt → `BRUN SDMTEST`.
- Expect: `SDM BANK TEST`, a row of 16 `.` per phase, then `PASS`.
- Negative test (proves the test checks): temporarily change phase-2's `CALCEXP` constant (e.g. `$5A`→`$5B`) so every read mismatches → rebuild → expect `FAILURES` (count `2800` hex = 10240) with the first 16 mismatch lines, then revert.
- Card absent at boot → `NO CARD` (not a hang, not a fail count).
- Capture via `obs-screenshot`.

- [ ] **Step 3: Update project memory**

Append a memory file noting the SDM driver library + test exist (API, files in `software/SDM/`, `make sdmdisk`, bench status) and add the one-line index entry to `MEMORY.md`. Cross-link `[[project_obscurus_monitor]]` and `[[feedback_use_merlin32]]`.

- [ ] **Step 4: Commit**

```bash
git add software/SDM/README.md
git commit -m "docs: SDM driver library README + bench checklist"
```

---

## Self-Review Notes (addressed)

- **Spec coverage:** library API + zero-ZP/no-ROM invariants (Task 1), two-phase sweep + value scheme + print-cap + NO CARD + auto-inc usage (Task 2), ProDOS BRUN BIN + library-as-TXT + bootable base + space-freeing + high-bit verify (Task 3), on-Apple `PUT` usage + negative test + bench (Task 4). The `SDM_READY` bounded/carry contract (Task 1) matches the spec's revised requirement. Print flood capped at 16 (Task 2 `ONFAIL`). Hex count via two `PRBYTE` (Task 2 `:fail`). No-bank-carry documented (Task 4).
- **Naming consistency:** `SDM_BANK/SDM_ADDR/SDM_VAL` and `SDM_READY/SETBANK/SETADDR/WRITE/READ/WRNEXT/RDNEXT/POLL` identical between `SDRAMLIB.S` (defs) and `SDMTEST.S` (calls). Register equates `R_ADRLO..R_DATA` used only inside the library. `PUT SDRAMLIB` ↔ file `SDRAMLIB.S` ↔ disk `SDRAMLIB`.
- **Placeholder scan:** every code step is complete source; every command has expected output. The only deferred items are the spec's explicit YAGNI list (full-chip sweep, march patterns, BASIC front-end, runtime-config, slot auto-detect).
- **Known environment note:** `ASC` stores high-bit-set on this Merlin32 (verified via the monitor banner), so `COUT` is called directly without `ORA #$80`; `PRSTR` prints the bytes as-is.
