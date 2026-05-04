# iwm_asic.v — spec → RTL signal mapping

Cross-reference for every IWM signal in `iwm_asic.v`. Citation form
`spec pN` = IWM Specification Rev 19 (Jan 2 1985); `patent col N` = US
Patent 4,326,256 (Wozniak).

## Bus interface

| RTL signal | Source | Spec ref | Notes |
|-----------|--------|----------|-------|
| `addr[3:0]` | A0..A3 of slot bus | spec p7 | A3..A1 selects state bit, A0 is value |
| `nDEVICE_SELECT` | slot decode | spec p7 | active-low, drives state-reg latches |
| `fclk` | sig_7M | spec p2 | 7.16 MHz Apple II 7M |
| `Q3` | Apple II Q3 | spec p7 | 2 MHz strobe, OR'd with /DEV for bus access |
| `R_nW` | 6502 R/W | spec p7 | gates buffer / mode writes |
| `nRES` | combined POR + Apple reset | — | async clear |
| `data_in/data_out` | D0..D7 mux | spec p9 | tri-stated externally by top |

## State registers (/DEV-clocked)

Per spec p7 + patent Fig. 2 block 39. addr[3:1] decode:

| addr[3:1] | Register | addr[0]=0 | addr[0]=1 |
|-----------|----------|-----------|-----------|
| 000 | `phase_r[0]` | clear | set |
| 001 | `phase_r[1]` | clear | set |
| 010 | `phase_r[2]` | clear | set |
| 011 | `phase_r[3]` | clear | set |
| 100 | `motorOn` | clear | set |
| 101 | `driveSel` | clear | set |
| 110 | `q6` | clear | set |
| 111 | `q7` | clear | set |

Latched on `negedge nDEVICE_SELECT`. Async clear on `~nRES`. No FCLK
synchronizer downstream — real ASIC has none, state changes are bus-
cycle-aligned and slower than any FCLK consumer's reaction window.

`rnw_dev` and `a0_dev` also latched on /DEV falling, used by mode/buffer
write event.

## Bus access strobe

Per spec p7. `dev_or_q3 = nDEVICE_SELECT | Q3`. `bus_strobe_rise` =
posedge of `dev_or_q3` in fclk domain. Used to commit data_held into
`modeReg` when q7=1, q6=1, a0_dev=1, rnw_dev=0, motorOn=0.

`data_held` continuously samples `data_in` while `~nDEVICE_SELECT`,
so the latched value is the last byte the 6502 drove during PHI0 high.

## Mode register (spec p8)

| Bit | Name | Function |
|-----|------|----------|
| 0 | modeLatch | 1 = latch (SmartPort) framing |
| 1 | modeAsync | 1 = async handshake |
| 2 | modeTimerOff | 1 = disable 1-second motor timer |
| 3 | modeFast | 1 = 2 µs bit cells |
| 4 | mode8MHz | 1 = 8 MHz fclk (unused on Apple II) |
| 5 | modeTest | 1 = test mode |
| 6 | modeMZreset | 1 = MZ reset |
| 7 | (unused) | — |

Reset value: `8'h07` (latch=1, async=1, timer-off=1) — Liron ROM init
expects this.

## Bit-cell timing constants (spec p4 / p10)

Not yet instantiated (Phase 2/3). Reference values for FCLK cycle counts:

| Mode | oneThreshold | zeroThreshold | writeBitCell |
|------|--------------|---------------|--------------|
| slow / 7 MHz | 14 | 42 | 28 |
| slow / 8 MHz | 16 | 48 | 32 |
| fast / 7 MHz | 7 | 21 | 14 |
| fast / 8 MHz | 8 | 24 | 16 |

## /ENBL gating (spec p7 / p12)

`_enbl1 = ~((motorOn & ~driveSel) | (timerActive & ~timerDriveSel))`
`_enbl2 = ~((motorOn &  driveSel) | (timerActive &  timerDriveSel))`

`timerActive` set on `motorOn` falling when `~modeTimerOff`. Holds
2^23 + 100 FCLK = 8388708 cycles (~1.17 s @ 7.16 MHz).

`timerDriveSel` snapshots `driveSel` at timer start so the correct
/ENBL is held even if `driveSel` toggles during the wind-down.

## Read register mux (spec p7 / p9)

| q7 | q6 | motorOn | data_out |
|----|----|---------|----------|
| 0 | 0 | 0 | `8'hFF` |
| 0 | 0 | 1 | `{x7, buffer[6:0]}` |
| 0 | 1 | x | `{sense, 1'b0, enableActive, modeReg[4:0]}` (status) |
| 1 | 0 | x | `{writeBufferEmpty, _underrun, 6'b0}` (handshake) |
| 1 | 1 | x | `8'hFF` (write-only registers) |

`enableActive = ~_enbl1 | ~_enbl2`. Per spec p9 bit 5.

## Phase 1 stubs

Not yet implemented; held at idle:

- `x7 = 0`, `buffer = 0`
- `writeBufferEmpty = 1`, `_underrun = 1`
- `wrdata = 1` (idle HIGH)
- `_wrreq = 1` (inactive)
- `rddata` consumed by `_unused_rd` ribbon

Phase 2 brings `wrdata`, `writeShifter`, `_underrun`, `_wrreq` under
the serializer. Phase 3 brings `rddata` sampling, `bitTimer`,
`shifter`, `buffer`, `x7`.
