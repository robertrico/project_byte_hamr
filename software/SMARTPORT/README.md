# PicoPort (SmartPort Disk Emulator)

SmartPort disk emulator running on a Raspberry Pi Pico W. Receives SmartPort commands from the Apple II via the Byte Hamr FPGA, decodes FM-encoded packets, and serves ProDOS block storage from disk images on an SD card.

## What It Does

1. **Receives** SmartPort commands from Apple II via FM-encoded serial protocol
2. **Decodes** packets using oversampled PIO sampling and edge detection
3. **Processes** commands: STATUS, READBLOCK, WRITEBLOCK, FORMAT, CONTROL, INIT
4. **Reads** disk images from SD card (`.2mg` / `.po` format, block-based)
5. **Encodes** responses as SmartPort packets with proper framing and checksums
6. **Transmits** FM-encoded data back to Apple II via PIO output
7. **Manages** handshaking with ACK (open-drain), REQ, and phase signals

## Key Files

| File | Purpose |
|------|---------|
| `main.c` | Entry point: GPIO init, PIO state machines, SmartPort bus loop |
| `picoport.h` | Pin definitions mapping Pico W GPIO to FPGA/SmartPort signals |
| `sp_proto.h` | Protocol definitions (commands, packet types, buffer sizes) |
| `sp_cmd.c` | Bus state machine, packet routing, command handlers |
| `sp_asm.S` | Timing-critical ARM assembly (ACK, phase reads, REQ waits) |
| `sp_encode.c` | Packet encoding (bit 7 masking, checksums, group-of-7) |
| `sp_decode_pkt.c` | Packet decoding (extract commands from received packets) |
| `sp_data.c` | Static data (DIB blocks, status responses, sync bytes) |
| `sd_block.c/h` | SD card interface: reads `.2mg` disk images, serves blocks |
| `fm_decode.S` | ARM Thumb-2 assembly: decodes FM samples from PIO into bytes |
| `fm_rx.pio` | PIO state machine for receiving FM-encoded data |
| `fm_tx.pio` | PIO state machine for transmitting FM-encoded data |
| `hw_config.c` | SPI clock and pin setup for SD card |
| `PLAN.md` | Comprehensive design document (protocol, PIO, wire-level details) |

## SmartPort Wire Protocol

- **FM encoding**: 4us bit cells; transition = 1-bit, steady = 0-bit
- **Sync**: `$FF $3F $CF $F3 $FC $FF $C3` (PBEGIN)
- **Packet**: SYNC + PBEGIN + DEST + SRC + TYPE + AUX + STAT + ODDCNT + GRP7CNT + data + CHKSUM + PEND
- **Group-of-7 encoding**: Preserves bit 7 by packing MSBs into a header byte
- **Bus phases**: 4 pins (PH0-PH3) indicate command/idle/reset state
- **Block size**: 512 bytes

## Pin Mapping

| Pico GPIO | Signal | Direction |
|-----------|--------|-----------|
| GP11 | SP_WRDATA | Output (to Apple II) |
| GP21 | SP_RDDATA | Input (from Apple II) |
| GP9 | SP_ACK | Output (open-drain) |
| GP2-5 | SP_PHx | Input (phase bits) |

## Build

Requires Raspberry Pi Pico SDK, ARM GCC cross-compiler, and CMake 3.13+.

```bash
make                        # Build with default ProDOS_2_4_1.po
make DISK=Merlin.po         # Build with different disk image
make flash                  # Flash via OpenOCD/SWD
make monitor                # Serial output at 115200 baud
make test                   # Build test firmware
make clean                  # Remove build directory
```

## Disk Utilities

```bash
python3 dsk2po.py game.dsk game.po         # Convert DOS 3.3 to ProDOS
python3 create_prodos_vol.py out.po 500000  # Create empty ProDOS volume
```
