# Flash Hamr — Next Steps

## Context
Branch: `PicoRV32FatFs`. PicoRV32 + FatFS SD card controller.
Full plan: `/Users/hambook/.claude/plans/merry-wandering-mccarthy.md`

## What Works (hardware verified, commit f6eb302)
- Streaming mount: SD → SDRAM, ProDOS boots, CATALOG, CopyII+ (53-block sapling)
- Persist: CREATE DEV + SAVE TEST = 8 persists, all survive cold reboot
- Picker UI, .PO and .2MG support, 12.5MHz SPI, SDSC byte-addressing card

## Current: On-Demand Cache — Direct Buffer Fill Fix

### Goal
Phase 10: replace streaming mount with on-demand cache. Mount is instant (just
f_open). Reads are intercepted by CPU, fetched from SD on miss, served from
SDRAM on hit. SDRAM acts as block cache.

### Previous Bug
After mount + reboot, 6502 read stale data from block buffer Port B.
Two approaches failed: (1) CPU direct write to buffer, (2) CACHE_RELEASE
to let arbiter do SDRAM→buffer. Both returned stale picker data at $0800.

### Fix Applied (this session)
**Key insight**: `fill_catalog_buffer()` writes to Port A and the 6502
reads correct data from Port B — proven working. Persist reads also work
(Port B write → Port A read). Both BRAM directions are functional.

The CACHE_RELEASE approach had too many CDC handshake opportunities for
failure (7MHz cache_released → combinational is_intercepted → arbiter
25MHz CDC edge detection → arbiter transfer → firmware timed delay).
The original "Approach 1" failure was likely a firmware bug (possibly
toggling cache_enabled OFF during buffer fill via `BBUF_CTRL = BBUF_CTRL_CLAIM`
without `| BBUF_CTRL_CACHE_EN`), misdiagnosed as a BRAM port mapping issue.

**New approach**: CPU fills block buffer directly for ALL cache reads,
using the exact same pattern as catalog fill:
- **Cache miss**: SD → sector_buf → SDRAM (cache) + buffer (immediate read)
- **Cache hit**: SDRAM → buffer (CPU reads SDRAM, writes to buffer)
- **Out of range**: fill buffer with zeros
- **CACHE_RELEASE eliminated** — firmware never writes BBUF_CACHE_RELEASE

The buffer fill sequence is identical to catalog fill:
```c
BBUF_CTRL = BBUF_CTRL_CLAIM | BBUF_CTRL_CACHE_EN;  // claim + keep cache on
BBUF_ADDR = 0;
for (i = 0; i < 512; i++) BBUF_WDATA = data[i];
BBUF_CTRL = BBUF_CTRL_CACHE_EN;                      // release claim
BBUF_MAGIC_DONE = 1;                                  // signal 6502
```

### Performance
- Cache miss: ~5-20ms (SD read dominates, buffer fill adds ~40μs)
- Cache hit: ~200μs (CPU reads 256 SDRAM words + writes 512 buffer bytes)
- Boot (20 blocks, all miss): ~200ms. Subsequent: all hits, ~4ms

### Architecture Overview

```
6502 CMD_READ → bus_interface (7MHz) → block_read_req
  → is_intercepted? (cache_en_s2 = 1)
    YES: magic_block_req → toggle CDC → firmware sees request
         firmware: SD → sector_buf → SDRAM (cache_miss)
         then: CACHE_RELEASE → arbiter reads SDRAM → buffer (Port A)
         then: MAGIC_DONE → cpu_br_s2 → dev_block_ready → bus S_IDLE
    NO:  arbiter reads SDRAM → buffer → gated_block_ready → bus S_IDLE
  → 6502 reads 512 bytes from DATA register (Port B)
```

### Architecture Overview (updated)

```
6502 CMD_READ → bus_interface (7MHz) → block_read_req
  → is_intercepted? (cache_en_s2 = 1)
    YES: magic_block_req → toggle CDC → firmware sees request
         MISS: SD → sector_buf → SDRAM + block buffer (CPU fills both)
         HIT:  SDRAM → block buffer (CPU reads SDRAM, writes buffer)
         then: MAGIC_DONE → cpu_br_s2 → dev_block_ready → bus S_IDLE
    NO:  arbiter reads SDRAM → buffer → gated_block_ready → bus S_IDLE
  → 6502 reads 512 bytes from DATA register (Port B)
```

### Key Registers
- BBUF_CTRL (+0x0C): bit 0 = buf_claim, bit 1 = cache_enabled
- BBUF_MAGIC_STATUS (+0x10): bit 17 = toggle, bits 15:0 = block_num
- BBUF_MAGIC_DONE (+0x14): write = signal block ready (stretch pulse)
- BBUF_CACHE_RELEASE (+0x18): UNUSED (RTL exists but firmware never writes it)

### Key Signals (7MHz domain)
- cache_en_s2: CDC'd cache_enabled from CPU
- is_intercepted: is_magic_block | (cache_en_s2 & ~cache_released)
- magic_active: set on magic_block_req rise, cleared when block_read_req drops
- dev_block_ready: magic_active ? cpu_br_s2 : gated_block_ready

## File Map

### RTL (gateware/flash_hamr/)
- `flash_hamr_top.v` — top level, cache intercept, CDC, SDRAM mux
- `cpu_soc.v` — PicoRV32 + peripherals + BBUF/SDRAM/mailbox registers
- `block_ready_gate.v` — persist gating (arb_ready_delayed fix)
- `block_buffer.v` — 512B dual-port BRAM (Port A/B both work — catalog proves it)
- `bus_interface.v` — Apple II register interface + debug counters
- `sdram_arbiter.v` — DO NOT MODIFY
- `mailbox.v` — toggle-CDC between bus_interface and CPU

### Firmware (gateware/flash_hamr/firmware/)
- `main.c` — on-demand cache: CPU fills buffer directly for hit/miss
- `spi_sd.c` — SD SPI driver (200ms timeout)
- `hal.h` — register defines (BBUF_CACHE_RELEASE still defined but unused)

### Key Constants
- Slot 4, SPI 12.5MHz, IMEM 32KB, DMEM 16KB
- Cache bitmap: 8KB in DMEM (supports up to 32MB / 65536 blocks)
- wait_ready timeout: ~1.28s, sd_wait_response: 200ms
- MAGIC_DONE stretch: 15 cycles (600ns)
