# Logic Hamr (Apple II Software)

Apple II logic analyzer display program for the Logic Hamr FPGA design. Renders captured digital waveforms on the HI-RES screen with trigger control and zoom.

## Programs

| File | Purpose |
|------|---------|
| `LOGIC.S` | Main logic analyzer: captures, triggers, and renders 8-channel waveforms |
| `CAPTEST.S` | Capture engine test: validates trigger and capture flow without display |

## How It Works

Communicates with the Logic Hamr FPGA via `$C0C0`-`$C0C9`:

| Register | Purpose |
|----------|---------|
| `$C0C0` | Channel select (0-7) |
| `$C0C1` | Sample address |
| `$C0C2` | Data read port |
| `$C0C3` | Command register |
| `$C0C4` | Status register |
| `$C0C5` | Stretch/zoom factor |
| `$C0C6`-`$C0C9` | Trigger configuration |

The program arms the FPGA capture engine, waits for a trigger event, then reads back decimated sample data and renders each channel as a waveform trace on the Apple II HI-RES screen. Supports programmable trigger modes (rising/falling edge) and multiple window presets.

## Build

```bash
merlin32 LOGIC.S      # Produces LOGIC executable at $8000
merlin32 CAPTEST.S    # Produces CAPTEST executable at $8000
```

## Shared Includes

- `INCLUDE.S` -- Apple II memory constants and HI-RES utilities
- `LINEADDR.S` -- HI-RES line address lookup tables
