# Block Hamr

ProDOS block storage device for the Apple II, implemented on the Byte Hamr FPGA card. Stores a disk image in SDRAM backed by SPI flash persistence, allowing the Apple II to boot and run from a virtual hard drive.

## What It Does

Block Hamr acts as a ProDOS-compatible block device (similar to a SmartPort drive):

1. **Boot** - On power-up, copies a disk image from SPI flash (at offset `0x400000`) into SDRAM
2. **Auto-detect** - Reads the ProDOS volume header to determine total block count
3. **Block I/O** - The 6502 reads and writes 512-byte blocks through a register interface at `$C0C0-$C0CF`
4. **Write-through persistence** - Modified blocks are asynchronously flushed back to SPI flash

Max disk image size: 12 MB (24,576 blocks of 512 bytes), limited by flash space after the bitstream.

## Register Map

Base address: `$C0C0` (slot 4)

| Offset | Name | R/W | Description |
|--------|------|-----|-------------|
| `$00` | STATUS / CMD | R/W | Read: `{ready, error, 5'b0, boot_done}` / Write: `$01` = READ, `$02` = WRITE |
| `$01` | DATA_READ | R | Read block data (auto-increment) |
| `$02` | BLOCK_LO | R/W | Block number low byte |
| `$03` | BLOCK_HI | R/W | Block number high byte |
| `$04` | SOFT_RESET | W | Pulse to reset state machine |
| `$05` | DATA_WRITE | W | Write block data (auto-increment) |
| `$06` | BLKCNT_LO | R | Total block count low (from boot loader) |
| `$07` | BLKCNT_HI | R | Total block count high |

## Architecture

```
Apple II Bus (7 MHz)           FPGA System (25 MHz)

  boot_rom.v                   boot_loader.v
  (Slot $Cn ROM)               Flash -> SDRAM copy
       |                       Auto-detect blocks
  bus_interface.v                    |
  (Register file)              sdram_arbiter.v
       |                       (Mux boot + block requests)
  block_buffer.v                     |
  (512B dual-port BRAM)        sdram_controller.v
       |                       (64MB SDRAM interface)
       +------ CDC sync ------+
                               flash_persist.v
                               (Write-through to flash)
```

**Clock domains**: 7.16 MHz (Apple II bus) and 25 MHz (SDRAM/flash), with 2-FF synchronizers at boundaries.

**Key design decisions**:
- Driver fits entirely in `$Cn` (256 bytes) - no shared `$C800` ROM needed
- Separate DATA_READ/DATA_WRITE addresses to avoid 6502 STA dummy-read bug
- Prefetch pipeline hides BRAM read latency

## Boot Sequence

1. SDRAM initializes (~200us)
2. Boot loader reads disk image from flash at `0x400000`
3. ProDOS volume header parsed for block count (bytes 1065-1066)
4. Image copied to SDRAM address 0
5. `boot_done` asserted, SPI mux switches from reader to writer
6. Apple II boots from block 0

## Makefile Commands

All commands run from the project root (`project_byte_hamr/`).

```bash
# Build
make DESIGN=block_hamr         # Build complete bitstream
make DESIGN=block_hamr synth   # Synthesis only
make DESIGN=block_hamr pnr     # Place & route only
make DESIGN=block_hamr bit     # Generate bitstream

# Program
make DESIGN=block_hamr prog              # JTAG (volatile)
make DESIGN=block_hamr prog-flash        # SPI flash (persistent)
make DESIGN=block_hamr prog-flash-with-image  # Bitstream + disk image

# Disk image management
make DESIGN=block_hamr prog-flash-with-image DISK_IMAGE=path/to/image.po

# Swap disk images without re-flashing the bitstream
make read-flash                          # Dump current image to disk_dump.po
make read-flash DUMP_FILE=backup.po      # Dump to a specific file
make read-flash DUMP_SIZE=4194304        # Read only 4MB (default: 12MB)
make write-flash DISK_IMAGE=new_disk.po  # Write new image to flash at 0x400000
make write-flash DISK_IMAGE=new_disk.po FLASH_IMG_OFFSET=0x400000

# Simulate
make DESIGN=block_hamr sim         # Run testbench
make DESIGN=block_hamr wave        # Simulation + GTKWave
make DESIGN=block_hamr unit MODULE=bus_interface

# Reports
make DESIGN=block_hamr report      # View build summary
make clean                         # Remove build artifacts
```

## Files

| File | Description |
|------|-------------|
| `block_hamr_top.v` | Top-level - SPI mux, reset, clock domains |
| `boot_rom.v` | 4KB slot ROM (256B boot + shared expansion) |
| `bus_interface.v` | Register file, command/status, prefetch pipeline |
| `block_buffer.v` | 512-byte dual-port BRAM (7 MHz / 25 MHz) |
| `boot_loader.v` | Flash-to-SDRAM copy with ProDOS auto-detect |
| `sdram_controller.v` | SDRAM init, refresh, read/write |
| `sdram_arbiter.v` | Mux boot loader and block I/O requests |
| `flash_reader.v` | SPI flash reader (boot phase) |
| `flash_writer.v` | SPI flash programmer (runtime phase) |
| `flash_persist.v` | Sector erase/program controller |
| `write_through.v` | Gates block_ready during flash writes |
| `addr_decoder.v` | Apple II address space decoder |
| `hamr_rom.asm` | 6502 ProDOS driver source (Merlin32) |
| `hamr_rom.mem` | Generated ROM content (hex) |
| `block_hamr.lpf` | Pin constraints (design-specific) |
