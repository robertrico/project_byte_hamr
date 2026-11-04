# Open items — stock-take 2026-10-11

Sources: repo docs/code sweep, Claude memory sweep, git branch state,
and static code review this session. Each item names where it came from.
"Stale" = a doc or memory says open but other evidence says done.
Nothing here is verified on the bench today.

## 1. Git state

| Branch | vs main | Remote | Note |
|--------|---------|--------|------|
| `slot-parameterization` (current) | 0 | pushed | Fast-forwarded into main 2026-11-03. Deletable. |
| `obscurus-sdram-monitor` | +51 unmerged | **local only** | Hypervisor 5a–5c-2b bench-verified work. Not backed up. |
| `smartport-host` | +19 unmerged | **local only** | Phase 4 done, Phase 5 built, never benched. Not backed up. |
| `write` | +1 unmerged | pushed | PicoPort WRITEBLOCK (2026-03-10). Never merged. |
| `PicoRV32FatFs`, `conway-multiverse`, `conway-multiverse-gr`, `coproc`, `coproc-c4`, `coproc-sdram-read`, `farm-game`, `farm-v2`, `flash_rtl` | merged | mixed | Deletable. |

Stashes: `stash@{0}` "dino - wip" (on slot-parameterization), `stash@{1}`
farm-v2 recipe shop WIP, `stash@{2}` conway-multiverse WIP. Decide keep/drop.

## 2. Per design

### Rev 2 hardware
- **E1 (bench 2026-11-03):** U14 drives ~IRQ/~NMI/~INH/~RES/~DMA push-pull;
  an idle Byte Hamr blocks every other card's interrupts. Possible repairs
  + Rev 3 notes: `hardware/byte_hamr/rev2_errata.md`. None done. Rev 3
  draft spec: `hardware/byte_hamr/rev3_spec.md`.

### smart_hamr_rev2 (FujiNet; Byte Hamr as Liron) — active
Bench plan: `gateware/rev2/smart_hamr_rev2/HANDOFF.md`. **Step 0 (A2M2053 known-good device) before FujiNet.**
- 2026-10-12: `iwm_asic.v` read-path fixes (MSB framing, latch-mode D7 hold) pass the `vtb/` oracle tb. Uncommitted, not flashed.
- Step 0 hookup: ECP5 GPIO not 5 V tolerant vs A2M2053 U1/U3 at VCCB 5 V. Pick 3.3 V VCCB or a translator first.
- Phase 5 (WRITE_BLOCK, FORMAT, 2nd drive) not started.
- Large images (8 / 32 MB) unverified.
- Rig: GPIO_9 `_enbl2` -> ESP32 IO3 (doc) vs IO21 (firmware). Resolve.
- FujiNet side (external repo): timer-paced RX decode (`iwm_ll.cpp:427-487`); SD fast-seek off, 4 MHz.
- `docs/bringup_log.md` Phases 0–3 still say "pending" (README says PASS); duplicate "Phase 5" heading.

### smart_hamr (rev2 legacy) — superseded
- Last commits are WIP (DRAIN_DELAY 700 µs, Q7 rework). Move to `deprecated/`?

### smartport_host (branch only)
- Phase 5 SP passthrough built, never flashed or benched. Open risks: BBUF_RDATA discard read, 8000-spin vs FujiNet 1.1 ms, `$CnFE`.
- Phase 6 UniDisk 3.5 + level shifters pending hardware. INIT receive intermittent.
- Overlaps smart_hamr_rev2's goal. Decide: keep, park or drop.

### block_hamr (rev2)
- **Bug (static, this session):** `boot_loader.v:138-143` clamps the copy to 12 MB but leaves `total_blocks` at the header value. A >12 MB image reports its full size to ProDOS while blocks >24575 are uninitialized SDRAM. Fix: clamp `total_blocks` too.
- `hamr_rom.asm:333` dummy block-0 read "optimize later"; `:189` STATUS param count ignored.
- README is a byte-identical rev1 copy (25 MHz, GPIO1..12, no Rev 2 status). Rev 2 bench pass only in commit `4e2a58a`.

### flash_hamr (rev1 only)
- Not ported to Rev 2.
- v7 known issue: CREATE on a mounted volume corrupted the .PO; boot from D2 not fully validated. No v8 closure.
- 16-bit block number caps drives at 32 MB.
- `$C800` contention fix deferred (v5). Return-to-picker option B deferred (v8).
- Test plan (60+ tb, 9 hw tests) has no results.

### b8008_hamr / ASM8 / A8
- QA two-card: T3 (FARM slot 2), T4 (coproc slot 2), T7 (two designs at once, `$C800` contention) have no results.
- Slot ROMs must be rebuilt per slot (standing constraint).
- Queued: slot ROM pushes CR into RX on PR#4 (needs reflash).
- Future: SCELBAL, direct load port, self-hosted assembler, 8008 SDRAM client, cflash persist.
- A8 P5 polish optional. VCF A8 blog post draft missing 6 screenshots.
- No README in `gateware/rev2/b8008_hamr`.

### signal_check (rev2)
- DATA_OE fix (stop driving shared `$C800` on nI_O_STROBE) never tested on hardware.

### project_obscurus / coproc / farm / SDM
- Bench pending: `CPDEMO` (C4), `CPSDRD` (SDRAM read window), `GRVERSE` (Conway GR).
- Coproc: multi-core (draft spec), live task registration while kernel runs, CALL-during-force-complete hardening, svc_count `$C0CE` stub.
- Hypervisor (obscurus-sdram-monitor branch): Task 7 language card, 5c-3 hyperkey, 5d/5e guest suspend/resume, supervisor RTOS spec, dirty bitmap to BRAM, T5C2/T5C3 readback bug, prune diag BINs. True DMA needs Rev 3.
- Farm: itr 4 world events spec not written; FARM.bin near 13.8 KB ceiling; FARMTASK blob ~10 B headroom; zombie-slot leak (accepted); FARMTEST gaps (AUTOSTORE tick, dual-task reset); quiesce reload must poll CP_FSTAT (not confirmed in code).
- Project needs a real name ("obscurus" placeholder). README says "slot 4 hardcoded" (now SLOT param) and omits coproc/kernel/cflash.

### software/SMARTPORT (PicoPort)
- Current tree: only INIT/STATUS/READ; WRITEBLOCK lives on unmerged `write` branch, which writes to SD without checking the data-packet checksum.
- RX decode: `fm_decode.S:150` floors cell count (boundaries at nominal edges); last ~5 bytes corrupt; checksum logged not enforced.
- ACK after 123 µs idle detect only fit rev1 `iwm.v` DRAIN_DELAY; untested against spec-pure rev2.
- Dead build knob: `make DISK=` unused since SD-backed.
- README claims WRITE/FORMAT/CONTROL.

### LOGICHAMR / ASM
- "TODO FIX CORNERS" (`LOGIC.S:255`, `LOGIC4.S:187`, `LOGIC5.S:272`); static page numbers.

## 3. Repo-wide
- **Makefile hides synth/PnR failures:** yosys/nextpnr piped to `tee` without pipefail, and `--timing-allow-fail`. Build can print "complete" after a failure.
- Not ported to Rev 2: logic_hamr, sdram_test, sdram_bank, pattern_rom, flash_hamr.
- Stale docs: top README (says rev2 has only signal_check), VERSION.md (Rev 1, 16 MB flash, 13-pin GPIO), software/README (missing B8008, SDM), SDM README (SLOT now in SLOTDEF.S), Makefile comment about stub monitor.S.
- ~40 plans/specs moved to the local-only workspace in `a2506ae`; most now exist only in git history (`git show a2506ae^:<path>`).
- No status doc: `rev2/b8008_hamr`, `rev2/smart_hamr`, `rev2/signal_check`, `rev2/constraints`, `hardware/byte_hamr`, `hardware/fujinet_adapter` (also untracked), `scripts/`.
- Rev 2 LPF regen script lived in `/tmp/gen_rev2_lpf.py` (likely gone).

## 4. Claude memory hygiene (stale or contradictory)
Stale: A8 (shipped, still "bench-pending"), b8008 "still pending" seam (done),
obscurus monitor / SDM lib descriptions ("bench pending", body says verified),
PicoRV32 coproc direction (superseded by Arlet coproc), coproc C1–C3 "next"
(done), hypervisor index "not yet built" (5a–5c-2b verified), farm "M2–M5
pending" (shipped), smartport_host "Phase 4 NEXT", Rev 2 U7 inverter
"must validate" (validated 2026-04-29), flash_hamr Phase 10 "next" (done in
v5), old IWM Hamr bug files (superseded by v2 register pivot),
bootstrap_mem.md (Rev 1 Yellow Hamr era, still marked "read first"),
sdram_architecture.md (Pico boot plan, superseded), Quick Build paths
(pre rev1/rev2 split).

Contradictions: ACK/sense polarity (bootstrap_mem vs smartport_protocol
"CORRECTED"), flash_hamr v8 single-drive plan vs shipped Duo, Merlin line
limit (<=40 vs <=50; CLAUDE.md says 40).

## 5. Decisions needed before cleanup
1. Push or archive the two local-only branches (`obscurus-sdram-monitor`, `smartport-host`)?
2. Delete the 9 merged branches?
3. Stashes: keep or drop each?
4. `write` branch: merge into PicoPort line, or park PicoPort?
5. smart_hamr legacy -> `deprecated/`?
6. smartport_host vs smart_hamr_rev2: one FujiNet path or both?
7. OK to fix block_hamr `total_blocks` clamp and Makefile pipefail?
8. OK to prune/refresh the stale Claude memories listed above?
