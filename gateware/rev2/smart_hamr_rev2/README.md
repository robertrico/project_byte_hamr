# smart_hamr_rev2

ASIC-faithful Apple Liron card recreation on Byte Hamr Rev 2.

Stock FujiNet, stock ProDOS, stock Apple II. Card presents as Slot 4 SmartPort
controller; bridges Apple II IWM register space to FujiNet ESP32 over GPIO
header.

## Spec references

- Apple Integrated Woz Machine Specification, Revision 19 (Jan 2 1985)
- US Patent 4,326,256 (Wozniak, IWM bit-cell decoder)
- Apple SmartPort Bus Specification
- Liron schematic 670-9107 / 820-0117

## Build

```
make clean
make DESIGN=smart_hamr_rev2 REV=rev2
```

## Flash (only on user authorization)

```
make DESIGN=smart_hamr_rev2 REV=rev2 prog-flash
```

## Phase status

| Phase | Scope | Status |
|-------|-------|--------|
| 0 | scaffolding (top, ROM, idle iwm) | bench PASS (2026-05-03) |
| 1 | state regs + mode reg + data_out mux + /ENBL timer | bench PASS (2026-05-03) |
| 2 | write path (TX serializer) | bench PASS (2026-05-03) |
| 3 | read path (RX shifter + x7 + buffer) | bench PASS (2026-05-03) |
| 4 | full ProDOS session | bench PASS (2026-05-03) — full 10-device enum + ProDOS boot |
| 5 | second drive + WRITE/FORMAT | not started |

v1 spec-pure (no drain, no Q6-flush, no DRAIN_DELAY) reaches the
PR#4 functional milestone of the prior `smart_hamr`. Empirical
hacks from PR#4 era are NOT required for Liron protocol fidelity.

`data_chksum=1` errors observed during early bring-up traced to
FujiNet ESP32 debug-print flooding stalling its ISR — not a
gateware fault. Reduce ESP32 print rate when troubleshooting
similar errors.

See `PLAN.md` for full scope, `docs/bringup_log.md` for bench results.

## Files

| File | Purpose |
|------|---------|
| `smart_hamr_rev2_top.v` | Slot interface, GPIO map, tri-state, POR, reset |
| `iwm_asic.v` | IWM ASIC emulation (single FCLK domain) |
| `addr_decoder.v` | nI_O_SELECT / nI_O_STROBE expansion ROM mgmt (verbatim) |
| `boot_rom.v` | 4 KB ROM container, combinatorial read (verbatim) |
| `liron_rom.mem` | Liron firmware data (verbatim) |
| `smart_hamr_rev2.lpf` | FPGA pin constraints (Rev 2) |
