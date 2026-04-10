/* main.c — PicoRV32 firmware for Flash Hamr
 *
 * Phase 7: Full mount — stream SD card image to SDRAM
 */

#include "hal.h"
#include "spi_sd.h"
#include "ff.h"

/* Mailbox registers */
#define MBOX_REG_CMD       (*(volatile uint32_t *)0x30000000)
#define MBOX_REG_ARG0      (*(volatile uint32_t *)0x30000004)
#define MBOX_REG_STATUS    (*(volatile uint32_t *)0x30000008)
#define MBOX_REG_TOTAL_BLK (*(volatile uint32_t *)0x3000000C)

/* Status flags */
#define FL_SD_READY     (1 << 0)
#define FL_SD_ERROR     (1 << 1)
#define FL_S4D2_MOUNTED (1 << 2)
#define FL_S4D2_LOADING (1 << 3)
#define FL_CPU_BUSY     (1 << 4)

/* Mailbox commands */
#define MCMD_MOUNT   0x01
#define MCMD_SD_INIT 0x02

/* SDRAM layout: S4D1 at offset 0, mountable units start at block 1024 */
#define S4D1_BLOCKS         280
#define MOUNT_BASE_BLOCK    1024

/* ================================================================ */

static FATFS fs;
static uint8_t card_type;

#define MAX_IMAGES 15

typedef struct {
    char     fname[13];
    uint32_t fsize;
    uint8_t  flags;       /* bit 0: is_2mg */
} image_entry_t;

static image_entry_t catalog[MAX_IMAGES];
static uint8_t num_images;
static uint8_t status_flags;
static uint8_t file_is_open;

static void delay(volatile uint32_t count) { while (count--) ; }

static int has_ext(const char *fname, const char *ext) {
    const char *p = fname;
    while (*p && *p != '.') p++;
    if (*p != '.') return 0;
    p++;
    for (int i = 0; i < 3; i++) {
        char a = p[i], b = ext[i];
        if (a >= 'a' && a <= 'z') a -= 32;
        if (b >= 'a' && b <= 'z') b -= 32;
        if (a != b) return 0;
    }
    return 1;
}

static void set_status(uint8_t flags) {
    status_flags = flags;
    MBOX_REG_STATUS = flags | ((num_images & 0x0F) << 4);
}

/* ================================================================
 * Fill catalog block buffer ($FFFF)
 * ================================================================ */
static void fill_catalog_buffer(void) {
    /* Always preserve cache_enabled — clearing it after mount
     * would de-intercept subsequent block reads. */
    uint32_t ctrl_base = file_is_open ? BBUF_CTRL_CACHE_EN : 0;
    BBUF_CTRL = BBUF_CTRL_CLAIM | ctrl_base;
    BBUF_ADDR = 0;

    /* Header */
    BBUF_WDATA = num_images;
    BBUF_WDATA = 0;  /* page 0 */
    BBUF_WDATA = 1;  /* 1 page total */
    for (int i = 3; i < 32; i++) BBUF_WDATA = 0;

    /* Entries */
    for (int e = 0; e < MAX_IMAGES; e++) {
        if (e < num_images) {
            const char *fn = catalog[e].fname;
            int fi = 0;
            for (int c = 0; c < 8; c++) {
                if (fn[fi] && fn[fi] != '.') BBUF_WDATA = fn[fi++];
                else BBUF_WDATA = ' ';
            }
            if (fn[fi] == '.') fi++;
            for (int c = 0; c < 3; c++) {
                if (fn[fi]) BBUF_WDATA = fn[fi++];
                else BBUF_WDATA = ' ';
            }
            BBUF_WDATA = catalog[e].flags;
            uint16_t blocks = (uint16_t)(catalog[e].fsize >> 9);
            BBUF_WDATA = blocks & 0xFF;
            BBUF_WDATA = (blocks >> 8) & 0xFF;
            uint32_t sz = catalog[e].fsize;
            BBUF_WDATA = sz & 0xFF;
            BBUF_WDATA = (sz >> 8) & 0xFF;
            BBUF_WDATA = (sz >> 16) & 0xFF;
            BBUF_WDATA = (sz >> 24) & 0xFF;
            for (int i = 18; i < 32; i++) BBUF_WDATA = 0;
        } else {
            for (int i = 0; i < 32; i++) BBUF_WDATA = 0;
        }
    }
    BBUF_CTRL = ctrl_base;
}

/* ================================================================
 * SD init + catalog scan
 * ================================================================ */
static int do_sd_init(void) {
    FRESULT fr;
    DIR dir;
    FILINFO fno;
    num_images = 0;

    fr = f_mount(&fs, "", 1);
    if (fr != FR_OK) {
        uart_puts("f_mount FAIL: ");
        uart_put_hex8(fr);
        uart_puts("\r\n");
        return -1;
    }

    fr = f_opendir(&dir, "/");
    if (fr != FR_OK) return -1;

    while (num_images < MAX_IMAGES) {
        fr = f_readdir(&dir, &fno);
        if (fr != FR_OK || fno.fname[0] == 0) break;
        if (fno.fattrib & (AM_DIR | AM_HID | AM_SYS)) continue;

        int is_po  = has_ext(fno.fname, "PO");
        int is_2mg = has_ext(fno.fname, "2MG");
        if (!is_po && !is_2mg) continue;

        int j;
        for (j = 0; fno.fname[j] && j < 12; j++)
            catalog[num_images].fname[j] = fno.fname[j];
        catalog[num_images].fname[j] = 0;
        catalog[num_images].fsize = fno.fsize;
        catalog[num_images].flags = is_2mg ? 1 : 0;

        uart_puts("  ");
        uart_puts(catalog[num_images].fname);
        uart_puts("\r\n");
        num_images++;
    }
    f_closedir(&dir);

    uart_puts("Found ");
    uart_putc('0' + num_images);
    uart_puts(" images\r\n");
    return 0;
}

/* ================================================================
 * On-demand block reads with SDRAM cache
 * ================================================================
 * Mount just opens the file (instant). Reads are intercepted:
 *   MISS: SD → sector_buf → SDRAM + block buffer
 *   HIT:  SDRAM → block buffer
 * Writes go through the arbiter → SDRAM, then persist to SD.
 * ================================================================ */
static uint8_t sector_buf[512];
static FIL mounted_fil;
static uint8_t mounted_is_2mg;
static uint32_t mounted_data_offset;
static uint16_t mounted_block_count;
static uint16_t mounted_sdram_base;   /* absolute SDRAM block offset for mounted vol */

/* Cache bitmap: 1 bit per block. 8KB = 65536 blocks = 32MB max. */
#define CACHE_BITMAP_SIZE 8192
static uint8_t cache_bitmap[CACHE_BITMAP_SIZE];

static inline int block_is_cached(uint16_t blk) {
    return (cache_bitmap[blk >> 3] >> (blk & 7)) & 1;
}
static inline void block_set_cached(uint16_t blk) {
    cache_bitmap[blk >> 3] |= (1 << (blk & 7));
}

/* Fill block buffer from SDRAM (cache hit).
 * CPU reads 256 words from SDRAM, writes 512 bytes to buffer Port A. */
static void serve_from_sdram(uint16_t blk) {
    uint32_t sdram_base = (uint32_t)blk << 9;
    BBUF_CTRL = BBUF_CTRL_CLAIM | BBUF_CTRL_CACHE_EN;
    BBUF_ADDR = 0;
    sdram_claim();
    for (int w = 0; w < 256; w++) {
        uint16_t word = sdram_read_word(sdram_base + w * 2);
        BBUF_WDATA = word & 0xFF;
        BBUF_WDATA = (word >> 8) & 0xFF;
    }
    sdram_release();
    BBUF_CTRL = BBUF_CTRL_CACHE_EN;
}

/* Read block from SD, store in SDRAM + block buffer (cache miss).
 * blk = absolute SDRAM block number (with unit offset applied). */
static int serve_from_sd(uint16_t blk) {
    uint16_t local_blk = blk - mounted_sdram_base;
    uint32_t file_off = mounted_data_offset + ((uint32_t)local_blk * 512);
    UINT br;

    FRESULT fr = f_lseek(&mounted_fil, file_off);
    if (fr == FR_OK)
        fr = f_read(&mounted_fil, sector_buf, 512, &br);

    if (fr != FR_OK) {
        BBUF_CTRL = BBUF_CTRL_CLAIM | BBUF_CTRL_CACHE_EN;
        BBUF_ADDR = 0;
        for (int i = 0; i < 512; i++) BBUF_WDATA = 0;
        BBUF_CTRL = BBUF_CTRL_CACHE_EN;
        return -1;
    }
    for (UINT i = br; i < 512; i++) sector_buf[i] = 0;

    /* Write to SDRAM cache */
    uint32_t sdram_base = (uint32_t)blk << 9;
    sdram_claim();
    for (int w = 0; w < 256; w++) {
        uint16_t word = sector_buf[w * 2] | (sector_buf[w * 2 + 1] << 8);
        sdram_write_word(sdram_base + w * 2, word);
    }
    sdram_release();

    /* Write to block buffer (same pattern as catalog fill) */
    BBUF_CTRL = BBUF_CTRL_CLAIM | BBUF_CTRL_CACHE_EN;
    BBUF_ADDR = 0;
    for (int i = 0; i < 512; i++) BBUF_WDATA = sector_buf[i];
    BBUF_CTRL = BBUF_CTRL_CACHE_EN;

    block_set_cached(blk);
    return 0;
}

/* ================================================================
 * Mount image — on-demand (no streaming)
 * ================================================================ */
static int do_mount(uint8_t idx, uint8_t slot) {
    if (idx >= num_images) {
        uart_puts("Mount: bad index\r\n");
        return -1;
    }

    uart_puts("Mounting: ");
    uart_puts(catalog[idx].fname);
    uart_puts(" -> unit ");
    uart_putc('1' + slot);
    uart_puts("\r\n");

    set_status(FL_SD_READY | FL_S4D2_LOADING | FL_CPU_BUSY);

    /* Disable cache during mount setup */
    BBUF_CTRL = 0;

    /* Build path: "/" + filename */
    char path[16] = "/";
    int pi = 1;
    for (int i = 0; catalog[idx].fname[i] && pi < 14; i++)
        path[pi++] = catalog[idx].fname[i];
    path[pi] = 0;

    /* Close any previously mounted file */
    if (file_is_open) {
        f_close(&mounted_fil);
        file_is_open = 0;
    }

    FRESULT fr = f_open(&mounted_fil, path, FA_READ | FA_WRITE);
    if (fr != FR_OK) {
        uart_puts("f_open FAIL: ");
        uart_put_hex8(fr);
        uart_puts("\r\n");
        set_status(FL_SD_READY | FL_SD_ERROR);
        return -1;
    }

    uint32_t file_size = f_size(&mounted_fil);
    mounted_data_offset = 0;
    mounted_is_2mg = catalog[idx].flags & 1;

    /* .2mg header detection */
    if (mounted_is_2mg) {
        UINT br;
        fr = f_read(&mounted_fil, sector_buf, 64, &br);
        if (fr != FR_OK || br < 64) {
            uart_puts("2mg header read fail\r\n");
            f_close(&mounted_fil);
            set_status(FL_SD_READY | FL_SD_ERROR);
            return -1;
        }

        if (sector_buf[0] == '2' && sector_buf[1] == 'I' &&
            sector_buf[2] == 'M' && sector_buf[3] == 'G') {
            mounted_data_offset = sector_buf[24] | (sector_buf[25] << 8) |
                                  (sector_buf[26] << 16) | (sector_buf[27] << 24);
            uint32_t data_len = sector_buf[28] | (sector_buf[29] << 8) |
                                (sector_buf[30] << 16) | (sector_buf[31] << 24);
            mounted_block_count = (uint16_t)(data_len >> 9);
            uart_puts("2MG: offset=");
            uart_put_hex32(mounted_data_offset);
            uart_puts(" blocks=");
            uart_put_hex32(mounted_block_count);
            uart_puts("\r\n");
        } else {
            uart_puts("2mg: bad magic\r\n");
            mounted_is_2mg = 0;
            mounted_data_offset = 0;
            mounted_block_count = (uint16_t)(file_size >> 9);
        }
    } else {
        mounted_block_count = (uint16_t)(file_size >> 9);
    }

    /* SDRAM base for this unit (slot 0=S4D2 → unit 1) */
    mounted_sdram_base = MOUNT_BASE_BLOCK;

    /* Clear cache bitmap, then protect S4D1 blocks */
    for (int i = 0; i < CACHE_BITMAP_SIZE; i++)
        cache_bitmap[i] = 0;
    for (uint16_t b = 0; b < S4D1_BLOCKS; b++)
        block_set_cached(b);

    file_is_open = 1;

    uart_puts("Mounted: ");
    uart_put_hex32(mounted_block_count);
    uart_puts(" blocks @ SDRAM ");
    uart_put_hex32(mounted_sdram_base);
    uart_puts("\r\n");

    /* Write per-unit registers (CDC'd to bus_interface) */
    uint8_t unit = slot + 1;  /* slot 0 = S4D2 = unit 1 */
    MBOX_UNIT_BLKCNT(unit) = mounted_block_count;
    if (unit >= 1)
        MBOX_UNIT_OFFSET(unit) = mounted_sdram_base;

    MBOX_REG_TOTAL_BLK = mounted_block_count;

    /* Enable cache intercept — all block reads now go through CPU */
    BBUF_CTRL = BBUF_CTRL_CACHE_EN;

    set_status(FL_SD_READY | FL_S4D2_MOUNTED);
    return 0;
}

/* ================================================================
 * Main
 * ================================================================ */
int main(void) {
#ifndef BUILD_TS
#define BUILD_TS "unknown"
#endif
    uart_puts("\r\n=== Flash Hamr - " BUILD_TS " ===\r\n");

    /* Auto-init SD */
    int rc = sd_init(&card_type);
    if (rc != SD_OK) {
        uart_puts("SD init failed\r\n");
        set_status(FL_SD_ERROR);
        goto loop;
    }

    rc = do_sd_init();
    if (rc != 0) {
        set_status(FL_SD_ERROR);
        goto loop;
    }

    set_status(FL_SD_READY);

    /* SDRAM quick test */
    sdram_claim();
    sdram_write_word(0x100000, 0xCAFE);
    uint16_t v = sdram_read_word(0x100000);
    sdram_release();
    uart_puts("SDRAM: ");
    uart_put_hex32(v);
    uart_puts(v == 0xCAFE ? " OK\r\n" : " FAIL\r\n");

loop:
    uart_puts("Command loop\r\n");

    uint16_t persist_count = 0;
    uint32_t loop_count = 0;
    uint16_t magic_count = 0;
    uint8_t last_toggle = 0;  /* firmware tracks toggle for edge detection */

    while (1) {
        loop_count++;

        /* Debug heartbeat: compare FW vs HW counts + intercept state */
        if ((loop_count & 0xFFFFF) == 0 && loop_count > 0) {
            uint32_t dbg = (*(volatile uint32_t *)0x70000004);
            uint8_t hw_magic = (dbg >> 24) & 0xFF;  // [31:24] magic rises
            uint8_t hw_reads = (dbg >> 16) & 0xFF;  // [23:16] CMD_READ count
            uint8_t flags    = dbg & 0xFF;           // [7:0] flags
            uart_puts("\r\n~M");
            uart_put_hex8(magic_count & 0xFF);
            uart_putc('/');
            uart_put_hex8(hw_magic);
            uart_puts(" RD=");
            uart_put_hex8(hw_reads);
            uart_puts(" F=");
            uart_put_hex8(flags);
            // F bits: [7]cache_en [6]intercepted [5]mactive [4]rd_req
            //         [3]gated_rdy [2]holding [1]persist_en [0]boot_done
            uart_puts("\r\n");
        }

        /* Check for intercepted block read (magic blocks + cache reads) */
        uint32_t ms = BBUF_MAGIC_STATUS;
        uint8_t cur_toggle = (ms >> 17) & 1;
        if (cur_toggle != last_toggle) {
            last_toggle = cur_toggle;
            magic_count++;
            uint16_t blk = ms & 0xFFFF;

            uint8_t was_hit = 0;
            if (blk == 0xFFFF) {
                /* Catalog */
                fill_catalog_buffer();
            } else if (block_is_cached(blk)) {
                /* Cache hit — serve from SDRAM (covers S4D1 + previously read blocks) */
                serve_from_sdram(blk);
                was_hit = 1;
            } else if (file_is_open &&
                       blk >= mounted_sdram_base &&
                       blk < mounted_sdram_base + mounted_block_count) {
                /* Cache miss — serve from SD */
                serve_from_sd(blk);
            } else {
                /* Out of range — zeros */
                BBUF_CTRL = BBUF_CTRL_CLAIM | BBUF_CTRL_CACHE_EN;
                BBUF_ADDR = 0;
                for (int i = 0; i < 512; i++) BBUF_WDATA = 0;
                BBUF_CTRL = BBUF_CTRL_CACHE_EN;
            }

            BBUF_MAGIC_DONE = 1;

            /* Compact log: C=catalog, *=miss, .=hit, ?=out-of-range */
            if (blk == 0xFFFF) uart_putc('C');
            else if (blk >= mounted_block_count) uart_putc('?');
            else uart_putc(was_hit ? '.' : '*');
            uart_put_hex8((blk >> 8) & 0xFF);
            uart_put_hex8(blk & 0xFF);
            uart_putc(' ');
        }

        /* Check for persist request from block_ready_gate */
        uint32_t ps = (*(volatile uint32_t *)0x30000014);  /* PERSIST_PEND */
        if (ps & 1) {
            persist_count++;
            uint16_t pblk = (*(volatile uint32_t *)0x30000010) & 0xFFFF;  /* PERSIST_BLK */
            FRESULT pfr = FR_INT_ERR;

            if (file_is_open) {
                /* HOT PATH — no UART until after PERSIST_DONE.
                 * Every microsecond here counts: ProDOS wait_ready
                 * times out at ~1.28 seconds. */

                /* Read 512 bytes from block buffer */
                BBUF_CTRL = BBUF_CTRL_CLAIM | BBUF_CTRL_CACHE_EN;
                BBUF_ADDR = 0;
                for (int i = 0; i < 512; i++)
                    sector_buf[i] = (uint8_t)BBUF_RDATA;
                BBUF_CTRL = BBUF_CTRL_CACHE_EN;

                /* Write to SD via FatFS (use local block for file seek) */
                uint16_t local_pblk = pblk - mounted_sdram_base;
                uint32_t file_off = mounted_data_offset + ((uint32_t)local_pblk * 512);
                UINT bw;
                pfr = f_lseek(&mounted_fil, file_off);
                if (pfr == FR_OK)
                    pfr = f_write(&mounted_fil, sector_buf, 512, &bw);
                if (pfr == FR_OK)
                    pfr = f_sync(&mounted_fil);
            }

            /* Release bus FIRST, then print debug */
            (*(volatile uint32_t *)0x3000001C) = 1;  /* PERSIST_DONE */

            /* Mark block as cached — arbiter wrote it to SDRAM */
            if (pfr == FR_OK)
                block_set_cached(pblk);

            /* Now safe to print — bus is released, ProDOS can continue */
            uart_puts("P#");
            uart_put_hex8(persist_count & 0xFF);
            uart_puts(" b=");
            uart_put_hex32(pblk);
            if (pfr == FR_OK)
                uart_puts(" OK\r\n");
            else {
                uart_puts(" E=");
                uart_put_hex8(pfr);
                uart_puts("\r\n");
            }
        }

        /* Check for mailbox command (toggle-based: bit 31 = new command pending) */
        uint32_t cmd_status = (*(volatile uint32_t *)0x30000000);
        uint8_t cmd = cmd_status & 0xFF;
        uint8_t cmd_new = (cmd_status >> 31) & 1;

        if (cmd_new && cmd != 0) {
            /* ACK to clear pending latch */
            (*(volatile uint32_t *)0x30000018) = 1;

            uart_puts("CMD: ");
            uart_put_hex8(cmd);
            uart_puts("\r\n");

            if (cmd == MCMD_SD_INIT) {
                f_mount(0, "", 0);
                rc = sd_init(&card_type);
                if (rc == SD_OK) {
                    do_sd_init();
                    set_status(FL_SD_READY);
                } else {
                    set_status(FL_SD_ERROR);
                }
            } else if (cmd == MCMD_MOUNT) {
                uint8_t raw = (*(volatile uint32_t *)0x30000004) & 0xFF;
                uint8_t idx = raw & 0x0F;
                uint8_t slot = (raw >> 6) & 0x03;
                uart_puts("Mount idx=");
                uart_put_hex8(idx);
                uart_puts(" slot=");
                uart_put_hex8(slot);
                uart_puts("\r\n");
                do_mount(idx, slot);
            }
        }

        /* No delay — tight polling for on-demand cache responsiveness */
    }

    return 0;
}
