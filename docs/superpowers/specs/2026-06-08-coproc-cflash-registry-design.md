# Coprocessor C-flash — Persistent Registry (design)

**Date:** 2026-06-08
**Branch:** `coproc` (continues from C3.1, bench-verified)
**Board:** Byte Hamr Rev 2, `gateware/rev2/project_obscurus/` + `software/SDM/`
**Status:** approved-in-discussion, pending spec review
**Builds on:** [[project_coproc_c0]] C3.1 — preemptive scheduler + GO trigger.

## Why this exists

The day-one ask: "register a service, have it there on a secondary boot." Today the host
loads services into coproc BRAM each session; a power-cycle loses them. C-flash **persists
the registered services to the onboard SPI flash and auto-restores them at boot** — register
once, power-cycle, the services are back without re-loading from disk.

## Goal

A host `SAVE` command writes the coproc's registry (table + service code) to flash; on the
next FPGA configuration the gateware auto-restores it into BRAM and reports valid. Success =
load a service, register it, `SAVE`, power-cycle, then trigger it (`GO`) with **no re-load**
and it runs — on the bench. (A SAVE bug can't clobber the bitstream — bounded `>= 0x400000`
for free — but that's hygiene, not a hard mandate; the bitstream is re-flashable anyway.)

## Architecture

Port the proven block_hamr SPI engine, wrap it with save/restore + a magic + a hard
write-bound. The persisted set is the **unprotected registry region only**; the baked kernel
(`$1000–$1FFF`) is never persisted (always fresh from `kernel.mem` at synth).

### Flash registry layout (one 4 KB sector @ `0x400000`)
```
$400000  HEADER page (256B): magic "CR" (2) | version (1) | svc_count (1) | 252 reserved
$400100  DATA: coproc BRAM $0200-$0FFF image = 3584 B = 14 pages (TABLE $0200-$02FF + task code $0300-$0FFF)
```
`0x400000` is 4 MB into the 16 MB IS25LP128F, safely above the ~2 MB bitstream
(block_hamr's `flash_persist` proved user data survives there across re-config). `NPARAM`
(per-task budgets) and `COUNT` are **runtime params the host provides at `GO` time**, not
persisted — the registry holds the *services* (code + registration vectors), the host
parametrizes per run.

### Components
- **`flash_writer.v` + `flash_reader.v`** — ported from `gateware/rev2/block_hamr/` (verbatim
  IS25LP128F SPI: WREN/sector-erase `0x20`/page-program `0x02`/WIP-poll; read for restore;
  12.5 MHz SPI). Proven engines — do not rewrite.
- **`cflash_save.v`** (host-triggered SAVE FSM): erase the registry sector → read coproc BRAM
  `$0200–$0FFF` (via port B reads) → page-program the header + 14 data pages to flash. Drives
  `busy`/`done`.
- **`cflash_restore.v`** (auto at boot RESTORE FSM): read the header → check **magic** → if
  valid, read the 14 data pages → write coproc BRAM `$0200–$0FFF` (via port B writes, which
  already write-protect `$1000+`). Sets a `restore_done`/`restore_valid` status.
- **Port-B arbitration**: port B is shared — host load (normal), SAVE reads (during save),
  RESTORE writes (during boot restore). They are **mutually exclusive in time** (the host
  waits during save; restore runs at boot before any host load), muxed by `save_busy` /
  `restore_busy`. The write-protect (`laddr[12]` → no `$1000+`) stays in force for restore.
- **Host interface** (`$C0Cx` / `$E0xx`):
  - `CP_SAVE` (`$C0CF`, host write magic `$5A`) → pulse `save_start`. Host polls `busy`.
  - `$E015` (coproc/host read) → bit0 `restore_valid`, bit1 `restore_done`, bit7 `save_busy`.
  - `$E016` (read) → restored `svc_count` (so a generic host "run services" program knows how
    many were registered).
- **SPI mux + `USRMCLK`** (port block_hamr's top pattern): after `boot_done`, user logic
  drives the config-flash SPI; on the ECP5 the config-SCK is reached through the `USRMCLK`
  primitive (not a normal IO). The top muxes `flash_writer`/`flash_reader` onto
  `FLASH_nCS/MOSI/MISO` + `USRMCLK` for SCK.

### Bitstream-safety write-bound (sensible hygiene, free)
A SAVE bug that wrote below `0x400000` would clobber the bitstream — **recoverable** (just
re-flash, which we do routinely), but an annoying mid-session detour. So bound it for free:
every SAVE flash address is `flash_addr = 24'h400000 | offset` with `offset` masked to the
sector, so erase/program can't land below `0x400000` even with a logic bug. Structural, not
a runtime check — costs nothing, avoids the detour. (Restore only *reads* → can't corrupt
regardless.)

### Robustness
- **Magic validation** on restore: header magic != "CR" → no restore; BRAM stays
  synth-initialized (zeroed task region) → kernel idles safe in `KWAITGO`, no garbage
  dispatched. Empty/first-boot flash = no restore, clean.
- **Erase-before-program**: the SAVE FSM erases the sector before programming (flash bits
  only 1→0 on program; erase sets 1s).
- **Restore-before-GO**: restore runs at boot while the kernel idles in `KWAITGO`; the host
  polls `$E015` `restore_done` before issuing `GO`. The host load port stays inert until
  restore completes (restore owns port B).

## Data flow
```
SAVE:  host: load+register services ; STA CP_SAVE=$5A
       gateware: save_start -> erase sector@0x400000 -> port-B read BRAM[$0200-$0FFF]
                 -> program header(magic,count) + 14 data pages ; busy clears
BOOT:  FPGA config -> boot_done -> cflash_restore: read flash header
                 magic=="CR"? -> read 14 data pages -> port-B write BRAM[$0200-$0FFF]
                 -> restore_done=1, restore_valid=1, $E016=svc_count
       coproc kernel boots -> KWAITGO (idle).  host: poll $E015 valid -> set NPARAM/COUNT -> GO
       -> kernel dispatches the FLASH-RESTORED services (no re-load)
```

## Components / files (branch `coproc`)
- Port: `gateware/rev2/project_obscurus/flash_writer.v`, `flash_reader.v` (copy from
  `block_hamr`, adjust only the module header comment; keep the proven logic).
- Create: `cflash_save.v`, `cflash_restore.v` (+ `_tb.v` each).
- `coproc.v` — expose a port-B mux input (so SAVE/RESTORE can drive `laddr`/data/we and read
  `ldata_out`) + the `$E015`/`$E016` reads + the restore-write path; pass `boot_done`.
- `project_obscurus_top.v` — instantiate `flash_writer`/`flash_reader`/`cflash_save`/
  `cflash_restore`; the SPI pin mux + `USRMCLK` + `boot_done` (GSR/`SEDGA` or a POR-derived
  boot_done); decode `CP_SAVE` ($C0CF) → `save_start`; route the FSMs' port-B access into the
  coproc; add `FLASH_*` ports + the `byte_hamr.lpf` already has the pins.
- `software/SDM/CPSAVE.S` — host: ensure services registered, `STA CP_SAVE`, poll `$E015`
  busy→done. `CPBOOT.S` (or extend a loader) — poll `$E015` valid, set NPARAM/COUNT, GO.
- `project_obscurus_tb.v` + a flash sim model (`sim/spi_flash_model.v`, port from block_hamr
  if present, else a minimal IS25LP128F model: WREN/erase/program/read + a backing array that
  PERSISTS across the coproc reset).

## Error handling / edge cases
- **Brick prevention** (the big one): write-bound `>= 0x400000`, structural. Covered above.
- **Power loss mid-SAVE**: the sector may be partially programmed → magic may be written last
  (program the DATA pages first, the HEADER/magic page LAST) so a half-save leaves an invalid
  magic → restore skips it (fail-safe: a torn save = "no registry", not corrupt-load).
- **Restore vs kernel boot race**: restore writes `$0200–$0FFF` while the kernel is in
  `KWAITGO` (reads nothing there until GO). The host must not `GO` before `$E015` done — the
  host polls. (Restore is ~ms; done well before a human-driven GO.)
- **Magic invalid / first boot**: no restore; clean empty registry.
- **SPI mux before boot_done**: user logic must NOT drive the flash until `boot_done` (the
  config controller owns it during configuration). Gate all flash drive on `boot_done`.

## Testing
- **Sim — SAVE FSM unit** (`cflash_save_tb`): trigger save, assert the flash model receives an
  erase@`0x400000` then the header (magic "CR") + the BRAM image; assert **no emitted address
  < 0x400000** (the brick-bound).
- **Sim — RESTORE FSM unit** (`cflash_restore_tb`): pre-load the flash model with a valid
  registry, run restore, assert BRAM `$0200–$0FFF` matches + `restore_valid`. Then a model
  with bad magic → `restore_valid=0`, BRAM untouched.
- **Sim — integration (the proof):** load racetask + register, SAVE (flash model captures it),
  **reset the coproc** (BRAM re-inits to `kernel.mem`, task region zeroed; the flash model
  RETAINS its contents), let restore run, then `GO` (no re-load) and assert the race runs from
  the restored tasks. This is persistence-across-boot in sim.
- **Build:** port adds `flash_*`/`cflash_*` modules; DP16KD still 8 (the flash FSMs are logic,
  not BRAM); timing PASS; confirm `USRMCLK`/SPI pins map.
- **Bench (the real proof):** flash; load+register a service; `SAVE`; **power-cycle** the
  board; on reboot the host polls valid + `GO`s with no re-load → the service runs.

## Success criteria
A registered service survives a power-cycle: SAVE → reconfigure → auto-restore → run, with no
host re-load. (The SAVE path is bounded `>= 0x400000` so it won't clobber the bitstream — a
free guard against an avoidable re-flash, not a hard safety mandate.) The persistent registry
— "register once, it's there on the next boot."

## Non-goals
- No SDRAM-backed service heaps / >8 KB services (the "less RAM-hindered" model — deferred;
  the SDRAM API is there when we cross that bridge).
- No multiple registry slots / versioning / wear-leveling (one sector, overwrite).
- No persistence of `NPARAM`/`COUNT` (runtime params, host-provided).
- No host-driven restore (auto-at-boot only) and no partial/incremental save (whole sector).
- Multi-core — separate rung.
