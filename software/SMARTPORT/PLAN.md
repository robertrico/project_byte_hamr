# PicoPort — SmartPort Device on RP2350 with PIO

A bare-metal SmartPort device built from scratch on a Raspberry Pi Pico W (RP2350),
communicating through the Byte Hamr FPGA Liron card. No frameworks, no RTOS, no C —
just PIO state machines, ARM Thumb-2 assembly, and the SmartPort protocol. All of it.

## Goals

1. **Learn PIO** by implementing a real-time serial protocol
2. **Isolate signal integrity issues** — remove FujiNet's complexity, test the bus directly
3. **Build toward a standalone SmartPort disk emulator** — SD card, maybe WiFi later
4. **Understand SmartPort deeply** — every bit, every handshake, every timing constraint

---

## Hardware

- **Pico W** — RP2350, dual-core ARM Cortex-M33, 150 MHz, 520KB SRAM
- **Byte Hamr** — ECP5 FPGA running Smart Hamr gateware (IWM + Liron ROM)
- **Connection**: Pico GPIO ↔ FPGA 3.3V I/O (same as current ESP32 DevKitC setup)

### Pin Assignments (adapt from ESP32 mapping)

| Signal    | Direction (Pico perspective) | Function                          |
|-----------|------------------------------|-----------------------------------|
| WRDATA    | INPUT                        | Data from Apple II (via IWM write serializer) |
| RDDATA    | OUTPUT                       | Data to Apple II (via IWM read shift register) |
| ACK       | OUTPUT (open-drain)          | Device acknowledgment (active LOW, hi-Z = deasserted) |
| REQ/PHI0  | INPUT                        | Host request (active HIGH)        |
| PHI1      | INPUT                        | Phase 1 (bus enable detect)       |
| PHI2      | INPUT                        | Phase 2 (bus reset detect)        |
| PHI3      | INPUT                        | Phase 3 (bus enable detect)       |

### Bus State Detection

| PH3 | PH2 | PH1 | PH0 | Meaning            |
|-----|-----|-----|-----|--------------------|
|  1  |  0  |  1  |  1  | SmartPort COMMAND (REQ asserted, bus enabled) |
|  1  |  0  |  1  |  0  | SmartPort IDLE (bus enabled, no REQ) |
|  0  |  1  |  0  |  1  | Bus RESET          |
|  *  |  *  |  *  |  *  | Other — ignore      |

Trigger on phases == `0b1011` (PH3=1, PH2=0, PH1=1, PH0=1) to start packet capture.

---

## SmartPort Wire Protocol Reference

### FM Encoding

SmartPort uses FM-like encoding where:
- **Data bit `1`** = a transition (level change) on WRDATA/RDDATA within a 4µs bit cell
- **Data bit `0`** = no transition within the bit cell
- **Byte time** = 8 bits × 4µs = 32µs per byte
- **WRDATA idles HIGH**

This is NOT standard FM (no separate clock bits). The IWM's latch mode synchronizes on
8 consecutive transitions (`$FF` = all 1s), then frames bytes with a free-running 8-bit
counter.

```
  Bit cell timing (4µs each):

  Data:    1     0     1     1     0     0     1     0
        ┌──┐  ┌─────┐┌──┐  ┌──┐  ┌──────────┐┌──┐  ┌─────┐
  Wire: │  └──┘     ││  └──┘  └──┘          ││  └──┘     │
        ←4µs→ ←4µs→  ←4µs→ ←4µs→ ←4µs→←4µs→ ←4µs→ ←4µs→
        edge   no     edge   edge  no    no    edge   no
               edge                edge  edge         edge
```

### PIO Implications

For **receiving WRDATA** (PIO RX state machine):
- Sample the pin at ~2MHz (every ~500ns), look for transitions
- Each 4µs cell = ~8 samples
- Transition detected → shift in `1`, resync to mid-cell
- No transition for full cell → shift in `0`
- After 8 bits accumulated → push byte to FIFO

For **transmitting RDDATA** (PIO TX state machine):
- Pull byte from FIFO
- For each bit (MSB first):
  - If `1`: toggle output halfway through the 4µs cell
  - If `0`: hold output steady for 4µs
- 4µs = 600 PIO cycles at 150MHz — plenty of precision

### Sync Sequence

Before every packet, 5 sync bytes plus a start marker:

```
Wire order: $FF  $3F  $CF  $F3  $FC  $FF  $C3
            sync sync sync sync sync sync PBEGIN (start marker)
```

**Receiver**: After detecting `$FF` (8 transitions = latch sync), scan subsequent bytes
for `$C3`. Allow up to 30 bytes of searching. Anything before `$C3` is discarded.

**Transmitter**: Send all 7 bytes before the packet header.

### Packet Structure

```
Offset  Field      Description
------  ---------  -------------------------------------------
 0-5    SYNC       $FF $3F $CF $F3 $FC $FF
 6      PBEGIN     $C3 — start of packet
 7      DEST       destination unit | $80
 8      SRC        source unit ($80 = host)
 9      TYPE       packet type | $80 ($80=cmd, $81=status, $82=data)
10      AUX        auxiliary type | $80
11      STAT       status byte | $80
12      ODDCNT     count of "odd" bytes (data_len % 7) | $80
13      GRP7CNT    count of 7-byte groups (data_len / 7) | $80
14      ODDMSB     MSBs of odd bytes, packed | $80
15..    ODD BYTES  odd data bytes, each | $80
        GRP7MSB    MSBs of 7-byte group, packed | $80
        G7BYTE1-7  7 data bytes, each | $80
        ...        (repeat for each group)
 N      CHKSUM1    checksum even bits | $AA
 N+1    CHKSUM2    checksum odd bits >> 1 | $AA
 N+2    PEND       $C8 — end of packet
```

**Rule**: Every data byte on the wire has bit 7 forced to 1 (`| $80`). Checksum bytes
use `| $AA` instead. The receiver must strip these masks.

### Group-of-7 Encoding

SmartPort can't transmit raw 8-bit values (bit 7 is reserved for framing). Solution:
every 7 data bytes become 8 wire bytes.

```
Original 7 bytes:  D0  D1  D2  D3  D4  D5  D6
                   Each has bit 7 that needs preserving

Wire encoding (8 bytes):
  Byte 0 (GRP7MSB): bit6=D0.7, bit5=D1.7, bit4=D2.7, bit3=D3.7,
                     bit2=D4.7, bit1=D5.7, bit0=D6.7  | $80
  Byte 1: D0 | $80  (bit 7 replaced, bits 6-0 preserved)
  Byte 2: D1 | $80
  ...
  Byte 7: D6 | $80
```

Decoding (ARM pseudocode):
```asm
@ R0 = pointer to 8 wire bytes (GRP7MSB + 7 data bytes)
@ R1 = pointer to 7 decoded output bytes
@ R2 = GRP7MSB byte (wire[0] & 0x7F)
    LDRB    R2, [R0]            @ load MSB byte
    BIC     R2, R2, #0x80       @ strip $80 mask
    MOV     R3, #7              @ loop counter
    MOV     R4, #1              @ shift amount (starts at 1)
decode_loop:
    LDRB    R5, [R0, R4]        @ load wire byte
    BIC     R5, R5, #0x80       @ strip $80 mask → bits 6:0
    LSL     R6, R2, R4          @ shift MSB byte left by (i+1)
    AND     R6, R6, #0x80       @ extract bit 7 for this byte
    ORR     R5, R5, R6          @ combine: bit7 | bits 6:0
    STRB    R5, [R1], #1        @ store decoded byte, advance ptr
    ADD     R4, R4, #1          @ next shift amount
    SUBS    R3, R3, #1
    BNE     decode_loop
```

**Packet size examples:**

| Payload   | Odd bytes | Groups | Wire data bytes | Total packet bytes |
|-----------|-----------|--------|-----------------|--------------------|
| 9 bytes   | 2         | 1      | 11              | 22 (command)       |
| 512 bytes | 1         | 73     | 586             | 603 (block read)   |

### Checksum

XOR all **decoded** data bytes, XOR all header bytes (offsets 7-13):

```asm
@ Computing checksum:
@ R0 = pointer to packet header (offset 7)
@ R1 = pointer to decoded data
@ R2 = data length
@ Returns checksum in R0
    MOV     R3, #0              @ checksum = 0
    MOV     R4, #7              @ XOR 7 header bytes (offsets 7-13)
hdr_loop:
    LDRB    R5, [R0], #1
    EOR     R3, R3, R5
    SUBS    R4, R4, #1
    BNE     hdr_loop
data_loop:
    LDRB    R5, [R1], #1        @ XOR all data bytes
    EOR     R3, R3, R5
    SUBS    R2, R2, #1
    BNE     data_loop

@ Encode to wire:
    ORR     R4, R3, #0xAA       @ chksum1 = checksum | $AA (even bits)
    LSR     R5, R3, #1
    ORR     R5, R5, #0xAA       @ chksum2 = (checksum >> 1) | $AA (odd bits)

@ Decode from wire:
    AND     R4, R4, #0x55       @ even bits
    AND     R5, R5, #0x55
    LSL     R5, R5, #1          @ odd bits shifted back
    ORR     R3, R4, R5          @ reconstructed checksum
```

### ACK/REQ Handshake

#### Command Packet (host → device)

```
Host (Apple II / IWM)              Device (Pico)
─────────────────────              ─────────────
                                   ACK = hi-Z (deasserted/HIGH)
1. Assert REQ (PH0=1)
   Enter write mode
   Send sync + packet on WRDATA
                                   2. Detect phases==0b1011
                                      Start capturing WRDATA
                                      Decode packet, verify checksum
                                      Assert ACK (drive LOW)
3. See ACK low (poll, 10 loops!)
   Deassert REQ (PH0=0)
                                   4. Deassert ACK (hi-Z)
```

**CRITICAL**: The ROM polls for ACK only **10 iterations** after write underrun +
~200µs drain delay. Your device MUST assert ACK within ~320µs of the last byte.

#### Commands with Data Packet (WriteBlock, Control)

After the command packet handshake above:

```
                                   5. Wait for REQ to fall (550µs timeout)
                                      Deassert ACK (hi-Z)
                                      [ready for data packet]
6. Assert REQ
   Send data packet on WRDATA
                                   7. Capture data packet
                                      Verify checksum
                                      Assert ACK (drive LOW)
8. See ACK, deassert REQ
                                   9. Process command
                                      [then send reply — see below]
```

#### Reply Packet (device → host)

```
                                   1. Deassert ACK (hi-Z) — "ready to send"
2. Poll status reg, see ACK
   deasserted (bit 7 = HIGH)
   Assert REQ (PH0=1)
                                   3. See REQ rise (30ms timeout)
                                      Start sending FM data on RDDATA
                                      (sync + reply packet)
4. Scan for $C3 (max 30 bytes)
   Read header + data
   Verify checksum, read $C8 PEND
   Deassert REQ
                                   5. See REQ fall
                                      Assert ACK (drive LOW)
                                      [handshake complete]
                                   6. Deassert ACK (hi-Z)
```

### SmartPort Commands

| Code | Command    | Data Packet? | Description                    |
|------|------------|--------------|--------------------------------|
| $00  | STATUS     | No           | Read device status / DIB       |
| $01  | READBLOCK  | No           | Read 512-byte block            |
| $02  | WRITEBLOCK | Yes (512B)   | Write 512-byte block           |
| $03  | FORMAT     | No           | Format device                  |
| $04  | CONTROL    | Yes          | Send control data to device    |
| $05  | INIT       | No           | Initialize / enumerate device  |

### INIT / Bus Enumeration

On power-up, the Liron ROM enumerates devices:

1. ROM sends bus RESET (PH0+PH2 high for 80ms, then 10ms settle)
2. ROM sends INIT command to destination=1, unit_num=2
3. First un-addressed device responds, gets assigned unit 1
4. ROM increments destination, sends next INIT
5. Repeat until no ACK received (3000 retries) or device reports "last"
6. ROM stores total unit count

Your Pico must respond to INIT when it has no address yet (destination matches next
available), then remember its assigned unit number for all subsequent commands.

### Timing Constants

| Parameter                        | Value               |
|----------------------------------|----------------------|
| Bit cell period                  | 4µs (28 FCLK @ 7MHz)|
| Byte time on wire                | 32µs                |
| Command packet (~22 bytes)       | ~700µs wire time    |
| Block packet (~603 bytes)        | ~19.3ms wire time   |
| ROM ACK poll after write         | 10 iterations (TIGHT)|
| ROM sync byte search             | max 30 bytes        |
| Reply REQ timeout                | 30ms                |
| Data packet REQ fall timeout     | 550µs               |
| Post-send REQ fall timeout       | 500µs               |
| Bus reset hold                   | 80ms                |
| Bus reset settle                 | 10ms                |
| Read retry count (ROM)           | 5 retries           |

---

## Phase 1: Hello World ProDOS Boot

**Goal**: Pico serves a minimal ProDOS volume from RAM. Apple II boots to `HELLO`
BASIC program. Proves the entire chain works: PIO → FPGA → Apple II.

### What You Need

1. **PIO RX state machine** — FM-decode WRDATA, push bytes to FIFO
2. **PIO TX state machine** — FM-encode bytes from FIFO to RDDATA
3. **ACK/REQ GPIO handling** — bit-bang, no PIO needed (handshake is slow enough)
4. **Packet parser** — decode command packets, extract command/unit/block number
5. **Packet builder** — encode reply packets with group-of-7 + checksum
6. **Block storage** — 280 blocks × 512 bytes = 140KB in SRAM (fits easily in 520KB)
7. **A ProDOS .po disk image** — with just a boot block, volume directory, and HELLO

### Minimal Command Support

For Phase 1, you only need to handle:

| Command  | What to do                                           |
|----------|------------------------------------------------------|
| INIT     | Respond with status byte (device present)            |
| STATUS   | Return DIB (device info block) or device status      |
| READBLOCK| Return 512 bytes from your block array               |

That's it. No writes, no control codes. Just serve blocks.

### Device Info Block (DIB) — STATUS code $03

The Liron ROM requests this during enumeration. You must respond with:

```asm
dib_data:
    .byte 0x00                              @ general status (no errors)
    .byte 0x00, 0x01, 0x18                  @ block count: 280 (0x000118) for 140K
    .ascii "PICOPORT"                       @ device name (8 bytes)
    .ascii "        "                       @ name padding (8 more, 16 total)
    .byte 0x02                              @ device type: ProFile-type hard disk
    .byte 0x00                              @ device subtype
    .byte 0x00, 0x01                        @ firmware version 1.0
dib_data_end:
    .equ DIB_LEN, dib_data_end - dib_data   @ = 25 bytes
```

### ProDOS Disk Image

Build a minimal 140K ProDOS `.po` image:
- Block 0-1: Boot blocks (can be empty if not booting from this device)
- Block 2: Volume directory header
- Block 3-5: Volume directory
- Block 6: Volume bitmap
- Remaining: HELLO file data

Or just use an existing ProDOS image. Convert to a raw binary blob and embed in flash:

```
# Convert .po to a linkable object:
arm-none-eabi-objcopy -I binary -O elf32-littlearm \
    --rename-section .data=.rodata \
    prodos_hello.po disk_image.o

# In your assembly, reference the linker symbols:
.extern _binary_prodos_hello_po_start
.extern _binary_prodos_hello_po_size

# Read block N: base address + (N * 512)
```

### Suggested Build Order

Everything is ARM Thumb-2 assembly from the start. The Pico SDK can still be used
for its linker scripts, boot2, and register definitions — you're just writing `.S`
files instead of `.c` files. The PIO programs are their own assembly language
regardless.

```
Step 1: Blink an LED (ARM asm)
        Bare-metal startup: set up clocks, configure GPIO, toggle a pin in a
        delay loop. Confirms your toolchain, linker script, and flash programming
        all work. This is your "hello world" — no SDK runtime, just you and the
        silicon.

        Key RP2350 registers to learn:
          - IO_BANK0_BASE ($40028000) — GPIO function select
          - SIO_BASE ($D0000000) — GPIO output set/clear/toggle
          - PADS_BANK0_BASE ($40038000) — pad configuration (drive, pull-up)
          - RESETS_BASE ($40020000) — peripheral reset control

        You'll write these register setup routines once and reuse them forever.

Step 2: UART output (ARM asm)
        Get UART TX working so you can printf-debug over USB-serial. Write a
        minimal uart_putc / uart_puts / uart_hex routine. You'll lean on this
        for every subsequent step.

        RP2350 UART is standard ARM PL011:
          - UART0_BASE ($40070000)
          - Set baud rate divisor, enable TX, write to UARTDR

Step 3: PIO TX — generate a steady 4µs square wave on RDDATA
        Write a PIO program that toggles a pin every 2µs (every 300 PIO cycles
        at 150MHz). Verify with scope/LA that the period is exactly 4µs.
        This is your "transmit all 1s" test — the IWM should sync on this.

        PIO TX concept (pseudocode):
        ┌──────────────────────────────────────────┐
        │  .wrap_target                            │
        │      pull block          ; get byte      │
        │      set x, 7           ; 8 bits         │
        │  bitloop:                                │
        │      out pins, 1 [N]    ; shift MSB out  │
        │                         ; delay for half │
        │                         ;   bit cell     │
        │      ??? toggle logic   ; if bit was 1,  │
        │                         ;   toggle pin   │
        │      jmp x-- bitloop                     │
        │  .wrap                                   │
        └──────────────────────────────────────────┘

        The exact PIO implementation is the puzzle to solve. The tricky part:
        FM encoding means you toggle on 1, hold on 0. The PIO needs to track
        the CURRENT pin level and XOR with the data bit to decide the next level.

        ARM side: write a pio_init routine that:
          1. Unreset PIO0 block
          2. Write PIO instruction memory
          3. Configure state machine (clock divider, pin mapping, shift direction)
          4. Enable the state machine
        All done via direct register writes to PIO0_BASE ($50200000).

Step 4: PIO RX — capture WRDATA transitions
        Write a PIO program that detects edges and measures inter-edge timing.
        Push raw edge timestamps or decoded bytes to the FIFO.
        Test by having the Apple II write known data through the IWM.

        PIO RX concept:
        ┌──────────────────────────────────────────┐
        │  wait_for_edge:                          │
        │      wait 1 pin, 0      ; wait for HIGH  │
        │      [measure time]                      │
        │      wait 0 pin, 0      ; wait for LOW   │
        │      [measure time]                      │
        │  ; if time < threshold → bit 1           │
        │  ; if time > threshold → bit 0           │
        │  ; after 8 bits → push to FIFO           │
        └──────────────────────────────────────────┘

        Edge detection is natural for PIO — the `wait` instruction blocks
        until the pin changes. Count cycles between edges to distinguish
        1-bits (4µs between edges) from 0-bits (no edge for 4µs+).

Step 5: ACK/REQ handshake — direct register access
        Write subroutines (BL-callable):
          - ack_assert:    set ACK GPIO to output, drive LOW
          - ack_deassert:  set ACK GPIO to input (hi-Z, external pull-up)
          - req_poll_rise: spin on GPIO input register, timeout via cycle counter
          - req_poll_fall: same, opposite polarity

        These are all SIO register reads/writes:
          - SIO_GPIO_OUT_CLR to drive low
          - SIO_GPIO_OE_SET / SIO_GPIO_OE_CLR for output enable/disable
          - SIO_GPIO_IN to read pin state

        Test: have the Apple II send a command, watch REQ go high on your
        UART output, read a few raw bytes off the PIO RX FIFO.

Step 6: Parse a command packet
        Combine PIO RX + ACK/REQ handling:
        1. Poll phase pins, wait for pattern == 0b1011
        2. Start PIO RX state machine
        3. Pull bytes from FIFO, scan for $C3 sync
        4. Read header + data bytes
        5. Decode group-of-7 (bit manipulation in ARM — shifts and ORs)
        6. Compute checksum (XOR loop)
        7. Assert ACK
        8. Dump command info over UART

        At this point you're a SmartPort bus sniffer. You can see every
        command the Apple II sends without responding. Invaluable for
        debugging — and a useful tool on its own.

Step 7: Respond to INIT
        When you see INIT (cmd=$05) addressed to the next unit:
        1. Assert ACK (you accept the address)
        2. Wait for REQ to fall
        3. Deassert ACK
        4. Build a status reply packet in a RAM buffer
        5. Encode group-of-7 + checksum
        6. FM-encode via PIO TX
        7. Wait for REQ rise, start PIO TX
        8. After transmit, assert ACK, wait for REQ fall, deassert ACK

        If enumeration works, the Liron ROM now knows your device exists.
        UART shows: "INIT: assigned unit 1"

Step 8: Respond to STATUS
        Handle STATUS command ($00):
        - Status code $00: return general status (4 bytes)
        - Status code $03: return DIB (25 bytes)

        After this, ProDOS can see your device in the slot scan.
        This is the first time you'll see your device name on the Apple II
        screen — "PICOPORT" in a catalog listing.

Step 9: Respond to READBLOCK
        Handle READBLOCK ($01):
        1. Extract 3-byte block number from command packet
        2. Compute offset into your RAM block array
        3. Build a data reply packet (TYPE=$82, 512 bytes payload)
        4. Encode group-of-7 + checksum into a TX buffer
        5. FM-encode and transmit via PIO TX

        This is the big one — 603 wire bytes at 4µs each = 19.3ms of
        continuous FM transmission. If your PIO TX timing is off by even
        a fraction, the IWM won't decode it.

Step 10: Boot ProDOS
         Embed a 140K ProDOS image in flash (or load over UART into SRAM).
         Handle INIT + STATUS + READBLOCK. Power on the Apple II. Watch it boot.

         When you see the ProDOS splash and a BASIC prompt — you're done
         with Phase 1. Everything after this is refinement.
```

### Debugging Strategy

- **UART output** — printf over USB-serial for every command received/sent
- **Logic analyzer** — monitor WRDATA, RDDATA, ACK, REQ simultaneously
- **Error counters** — track checksum failures, timeout counts, retry counts
- **LED indicators** — blink patterns for state (idle, receiving, transmitting, error)

---

## Phase 2: Test Harness

**Goal**: Expand the Phase 1 firmware into a comprehensive diagnostic tool that
exercises every SmartPort command and edge case. Add WRITEBLOCK support, error
injection, timing measurement. Both sides — a 6502 test program on the Apple II
and the Pico firmware working in concert.

### Test Modes

Build selectable test modes that the Pico cycles through or that you select via UART:

```
Mode 1: Echo Test
        Respond to every READBLOCK with block data = { block_num repeated 512 times }
        Apple II side: 6502 program reads blocks, verifies pattern
        Catches: bit errors, byte framing errors, group-of-7 encode/decode bugs

Mode 2: Stress Test — Sequential Reads
        Apple II reads blocks 0, 1, 2, ... N sequentially as fast as possible
        Pico counts requests, measures timing between commands
        Catches: handshake timing issues, IWM state machine problems

Mode 3: Stress Test — Random Reads
        Apple II reads random block numbers
        Pico verifies it can handle any block number
        Catches: block number encoding/decoding bugs in group-of-7

Mode 4: Large Volume Test
        Serve a 32MB image (65,536 blocks) — maximum ProDOS volume
        Tests block number range up to $FFFF

Mode 5: Write Test
        Apple II writes blocks, Pico stores in RAM, reads back to verify
        Tests the full WRITEBLOCK handshake (command + data + reply)

Mode 6: Error Injection
        Pico deliberately introduces errors:
        - Corrupt one byte in a reply packet
        - Delay ACK assertion by varying amounts
        - Send a short packet (missing bytes)
        - Send with slightly wrong bit cell timing (3.8µs, 4.2µs)
        Measures: how the ROM/ProDOS handles each error, retry behavior

Mode 7: Timing Analysis
        Pico measures and reports over UART:
        - Exact time between REQ rise and first WRDATA transition
        - Exact time between last WRDATA byte and REQ fall
        - Inter-command gap timing
        - Histogram of bit cell periods seen on WRDATA
```

### 6502 Test Program

Write a small 6502 program that exercises the SmartPort interface directly, bypassing
ProDOS. Load it as a `.SYSTEM` file on the ProDOS boot disk from Phase 1.

```asm
; SmartPort test — call the SmartPort entry point directly
; The entry point is at Cn00+3 (where n = slot number)
;
; SmartPort call convention:
;   JSR $Cn00       ; the ROM entry point
;   .byte cmd       ; command number
;   .word param_ptr ; pointer to parameter list
;
; Parameter list for READBLOCK:
;   .byte 3         ; parameter count
;   .byte unit      ; unit number
;   .word buffer    ; pointer to 512-byte buffer
;   .byte blk_lo    ; block number low
;   .byte blk_mid   ; block number mid
;   .byte blk_hi    ; block number high

; Read block 0 into $2000:
            JSR sp_entry    ; SmartPort entry point (patched at load)
            .byte $01       ; READBLOCK
            .word params
            BCS error       ; carry set = error, A = error code
            ; success — 512 bytes at $2000
            ...

params:     .byte 3         ; param count
            .byte 1         ; unit 1
            .word $2000     ; buffer address
            .byte 0,0,0     ; block 0
```

Consider stealing/adapting FujiNet's SmartPort test firmware — it already exercises
the protocol and reports results on screen. Or write your own from scratch — you're
writing 6502 asm anyway, and you know the protocol now.

---

## Phase 3: SD Card Integration

**Goal**: Read .po / .2mg disk images from a micro SD card. Serve blocks on demand.
This turns the Pico into a standalone SmartPort disk emulator.

### SD Card Interface

The Pico has SPI available. Use a micro SD breakout board wired to the Pico's SPI
peripheral (not PIO — save PIO for SmartPort). SD card SPI is well-documented and
there are minimal bare-metal SD/FAT libraries available.

### .po Image Format

A `.po` (ProDOS order) image is just raw blocks concatenated:
- Block 0 starts at file offset 0
- Block N starts at file offset N × 512
- No header, no metadata

Read block: seek to `block_num * 512`, read 512 bytes. That's it.

### .2mg Image Format

A `.2mg` image has a 64-byte header followed by raw block data:

```
Offset  Size  Field
0       4     Magic: "2IMG"
4       4     Creator ID
8       2     Header size (usually 64)
10      2     Version
12      4     Image format (0=DOS 3.3, 1=ProDOS, 2=NIB)
16      4     Flags
20      4     ProDOS blocks count
24      4     Data offset (usually 64)
28      4     Data length
32-63   -     Comment/creator offsets, reserved
```

Read block: seek to `data_offset + (block_num * 512)`, read 512 bytes.

At startup, read the 64-byte header, stash `data_offset` and `block_count`
in registers or RAM, and you're done — the rest is just `.po` with an offset.

### Architecture

```
                    ┌─────────────────────────────┐
                    │         Pico RP2350          │
                    │                              │
  WRDATA ──────────→│ PIO SM0 (RX) → FIFO → Core0 │
  RDDATA ←──────────│ PIO SM1 (TX) ← FIFO ← Core0 │
  ACK    ←──────────│ GPIO (bit-bang)         │    │
  REQ    ──────────→│ GPIO (polling)          │    │
                    │                         │    │
                    │              Core1 ─────┘    │
                    │              (SD card I/O,   │
                    │               prefetch,      │
                    │               UART debug)    │
                    │                              │
                    │         SPI (hardware)       │
                    │            │                 │
                    └────────────│─────────────────┘
                                 │
                           ┌─────┴─────┐
                           │  SD Card  │
                           └───────────┘
```

- **Core 0**: SmartPort protocol handler (PIO + packet encode/decode)
- **Core 1**: SD card reads, block prefetching, UART debug output
- **Shared**: Block buffer in SRAM with simple mutex or mailbox

### Block Prefetching

Sequential reads are the common case (ProDOS reads blocks 0, 1, 2, 3...). When
Core 0 serves block N, signal Core 1 to prefetch block N+1 from SD. If the next
request is indeed N+1, it's already in RAM — zero latency. SD card seek + read for
a single 512-byte block takes ~1-2ms at SPI speeds, well within the inter-command
gap.

### Configuration

How does the user select which image to mount? Options:
- **Hardcoded** — one image, compile-time path. Simplest.
- **First .po file** — scan root directory, mount first image found.
- **UART menu** — list files over serial, user types a number.
- **Button + LED** — cycle through files with a button, LED blinks file index.
- **Config file** — `PICOPORT.CFG` on SD root, lists image path.

Start with hardcoded, evolve from there.

---

## Phase 4 (Future): WiFi Integration

**Goal**: Use the Pico W's WiFi for TNFS, HTTP image download, or a web-based
management interface.

Not designed yet. Get Phases 1-3 solid first. But the Pico W has the hardware —
CYW43439 WiFi chip, lwIP stack available. A minimal TNFS client would let you
browse and mount images from the same servers FujiNet uses.

Could also serve a tiny web page for image management — upload .po files from a
browser, select mounted image, view error counters. No Apple II configuration
app needed.

Another option: skip TNFS entirely. Simple UART or USB serial protocol to push
images from a laptop. `cat game.po > /dev/tty.pico` and done.

---

## Reference: ESP32 FujiNet Values for Comparison

When debugging, compare your Pico measurements against these known-working values
from the ESP32 FujiNet implementation:

| Parameter                    | ESP32 Value              | Notes                    |
|------------------------------|--------------------------|--------------------------|
| WRDATA sample rate           | 2.05 MHz (SPI)           | ~8 samples per bit cell  |
| RDDATA output rate           | 1 MHz (SPI)              | 4 SPI bytes per data byte|
| ACK assert → REQ fall        | < 550µs expected         | ISR measured              |
| REQ rise → first TX byte     | ~10µs                    | SPI setup time           |
| Full block TX time           | ~19.3ms                  | 603 bytes × 32µs        |
| Inter-command gap (typical)  | 24-33ms                  | From a5.log analysis     |
| Block read success rate      | 99.97% (with caps)       | 1 fail in 3488 reads     |
| REQ timeout rate             | ~0.26% (large packets)   | 9 in ~3500 operations    |

---

## What We Already Know (From Smart Hamr Debugging)

These are proven facts from the FPGA + ESP32 debugging sessions that apply directly:

1. **Large packets fail more than small packets.** REQ timeouts are 100% correlated
   with spi_len=2411 (512-byte blocks). Short DIB replies (183 bytes) almost never fail.
   The longer the FM stream, the more chance of a bit error.

2. **22pF caps on RDDATA and WRDATA dramatically improve reliability.** Kills
   high-frequency ringing on the GPIO lines. Keep using them on the Pico too until
   Rev 2 board with proper level shifters.

3. **The IWM's latch sync is fragile.** Noise during handshake gaps can cause the
   shift register to false-sync on garbage. The FPGA has a Q6-falling-edge flush to
   mitigate this. Your PIO RX should similarly reset its state between packets.

4. **ACK timing is critical.** The Liron ROM only polls 10 times after write
   completion. If your device takes too long to assert ACK, the ROM gives up.
   This is the tightest constraint in the protocol.

5. **The ROM retries.** Read has 5 retries, write has 3000 (!). A single failed
   packet is not fatal — the ROM will try again. But each retry costs 3.5-8 seconds
   of wall time (ProDOS timeout), which kills the user experience.

---

## Context for Future Sessions

### What this project IS
This is a learning project. The user builds things to understand them — the destination
is secondary to the journey. They built an IWM from a 1982 patent, debugged it with
a logic analyzer, got FujiNet working, and now want to build the same protocol on
different hardware to understand it deeper. Expect tangents into 6809, TTL hardware
parsers, and other "absurd but educational" directions. Roll with it.

### How we work together
- The user is the hands, eyes, and taste — they build, flash, test, refine
- Claude writes large chunks of code, analyzes logs, provides protocol knowledge
- The user reads and understands every line — don't generate code they won't learn from
- The user knows C well, prefers assembly, enjoys the difficulty
- **NEVER build the bitstream** — user does all hardware flashing themselves

### Key references in this repo
- `gateware/smart_hamr/liron-if.asm` — THE definitive reference for SmartPort handshake
  timing. Every register access the Liron ROM makes is documented here. When in doubt
  about protocol behavior, read this file.
- `gateware/smart_hamr/iwm.v` — The working IWM implementation. FM bit cell timing,
  shift register behavior, write serializer — all proven against real hardware.
- `gateware/smart_hamr/a1.log` through `a5.log` — Debug logs from ESP32 FujiNet sessions.
  a1=before caps, a2=with caps (dramatic improvement), a3/a5=stability testing.
  Pattern: grep for "sending block packet" followed by "Reset" to find rddata failures.

### Traps to avoid
- The ESP32 FujiNet uses SPI peripheral for FM encode/decode. The Pico should use PIO
  instead — that's the whole point. Don't cargo-cult the ESP32's SPI approach.
- The `liron.asm` file is the gate-array version of the ROM, NOT the IWM version.
  Always use `liron-if.asm` for our hardware.
- SmartPort FM encoding is NOT standard FM. No clock bits. A `1` = transition within
  a 4µs cell, a `0` = no transition. The IWM detects edges, not levels.
- ACK is active LOW, open-drain. Deasserted = hi-Z (pulled HIGH). Many people get
  this backwards.
- The ACK timing window after command packet receive is brutally tight — the ROM
  polls only 10 times. This was the source of many early bugs.

### The user's hardware
- Apple IIe with A2DVI (HDMI output, can screenshot)
- Byte Hamr FPGA card (ECP5) in a slot, running Smart Hamr gateware
- ESP32 DevKitC WROVER currently wired to FPGA GPIO (will be replaced by Pico)
- DuoDisk for physical floppy access
- Logic analyzer for signal debugging
- Pico W (RP2350) for the PicoPort project — not yet wired up

