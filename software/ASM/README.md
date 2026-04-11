# ASM

Apple II 6502 assembly programs for graphics, hardware testing, and logic analyzer display prototypes.

## Programs

| File | Purpose |
|------|---------|
| `LOGIC.S` | Basic GUI: renders a single shape to HI-RES display |
| `LOGIC3.S` | Logic analyzer display with transition detection (up to 8 channels) |
| `LOGIC4.S` | Enhanced logic analyzer with macro-based multi-channel processing |
| `LOGIC5.S` | Full logic analyzer with live FPGA capture via `$C0C0`-`$C0C9` registers |
| `PERIPHERAL.S` | Low-level FPGA register exerciser (read/write `$C0C0`-`$C0CF`) |
| `BH.UTILS.S` | Utility menu with keyboard-driven dispatch for 5 options |
| `PLANE.S` | Multi-byte shape rendering demo |
| `STARSHIP.S` | Game sprite demo (small ship graphic) |

## Shared Includes

| File | Purpose |
|------|---------|
| `INCLUDE.S` | Apple II memory constants, HI-RES clear screen subroutine |
| `LINEADDR.S` | HI-RES line address lookup tables (HI/LO bytes for 192 scanlines) |

## How It Works

Programs load at `$8000` and render to Apple II HI-RES video memory (`$2000`-`$3FFF`). The LOGIC series communicates with the Byte Hamr FPGA through the `$C0C0`-`$C0C9` I/O window to read captured logic analyzer samples and display them as waveforms.

## Build

```bash
merlin32 LOGIC5.S    # Produces LOGIC5 binary at $8000
merlin32 LOGIC.S     # Produces LOGIC binary at $8000
```

Transfer the resulting binaries to an Apple II via ADTPro or package into a `.dsk` image using `../utils/create_dsk.sh`.
