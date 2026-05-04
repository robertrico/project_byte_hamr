# Bring-up log — smart_hamr_rev2

Per-phase bench validation history. Each entry: date, design SHA, observed
behavior, pass/fail, notes.

## Phase 0 — scaffolding

### Pass criteria

- Apple II boots, no freeze, no smoke.
- Slot 4 visible to scanning.
- ProDOS does not panic.
- No driver claim acceptable; slot inert is acceptable.

### Bench log

| Date | SHA | Result | Notes |
|------|-----|--------|-------|
| 2026-05-02 | (uncommitted v1) | RTL written, build clean. Bench validation pending user authorization to flash. | |

## Phase 1 — state registers + register read mux + /ENBL

### Pass criteria

- Liron ROM init runs without crash.
- `_enbl` + phase outputs visibly cycle on FujiNet bench (ESP32 reads
  pattern changes, logs activity).

### Bench log

| Date | SHA | Result | Notes |
|------|-----|--------|-------|
| 2026-05-02 | (uncommitted v1) | RTL written, build clean. Bench validation pending. | Phase 1 RTL is in same iwm_asic.v as Phase 0 — both gated by single `make`. |

## Phase 2 — write path (TX)

### Pass criteria

- FujiNet receives valid SmartPort command packets.
- Capture confirms PBEGIN $C3, dest, src, type, aux, status, count, data,
  checksum, PEND $C8 byte stream.

### Bench log

| Date | SHA | Result | Notes |
|------|-----|--------|-------|
| 2026-05-02 | (uncommitted v1) | RTL written: writeShifter, writeBitTimer/Counter, _underrun, _wrreq, buffer load on bus_strobe_rise. Spec-exact constants (writeBitCell = 28 / 32 / 14 / 16). Build clean. Bench validation pending. | Midpoint toggle = writeBitCell>>1 (spec p2). No DRAIN_DELAY hack. |

## Phase 3 — read path (RX)

### Pass criteria

- Apple II runs Liron STATUS command.
- Receives valid status block from FujiNet.
- ProDOS sees mounted SmartPort device.

### Bench log

| Date | SHA | Result | Notes |
|------|-----|--------|-------|
| 2026-05-02 | (uncommitted v1) | RTL written: 2-FF rddata sample, bitTimer thresholds, latch-mode framing (sync on first $FF, then 8 bits/byte), GCR fallback (MSB=1 latch), x7 with 14-fclk clear timer (patent col 8). Build clean. | No Q6-falling-edge flush, no Q7-rise pre-seed buffer. Spec only. |

## Phase 4 — full ProDOS session

### Pass criteria

- ProDOS catalogs disk image.
- Sequential file load.
- 50+ cold + warm reboot cycles without freeze.

### Bench log

| Date | SHA | Result | Notes |
|------|-----|--------|-------|
| 2026-05-02 | (uncommitted v1) | v1 RTL = Phases 0-3 superset. Pending flash + bench. v1 target = match prior smart_hamr PR#4 milestone (boots ProDOS from FujiNet). | |
| 2026-05-03 | (uncommitted v1, spec-pure) | **PASS.** Boots ProDOS from FujiNet. Full 10-device SmartPort enumeration (drives 0x81–0x8a). READ_BLOCK / STATUS / CONTROL all work. | Spec-pure RTL: no drain, no Q6-flush, no DRAIN_DELAY. Proves the empirical hacks in prior `smart_hamr` were not load-bearing for Liron protocol. |
| 2026-05-03 | same | False alarm: `data_chksum=1` errors and `Send REQ timeout dest=0x83` traced to **FujiNet ESP32 print-flooding stalling its own ISR window**, not an IWM-emulation bug. Disabled debug printf reduces error rate. | Important lesson: ESP32 serial debug output competes with IWM bit-cell capture for CPU. Always evaluate FujiNet errors against current debug-print level before assuming gateware fault. |

## Phase 5 — second drive + WRITE/FORMAT

Pending.

## Phase 5 — second drive + WRITE/FORMAT

Pending.
