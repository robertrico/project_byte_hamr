#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "sd_block.h"
#include "picoport.h"
#include "f_util.h"
#include "ff.h"

#define IMAGE_FILENAME "pico_boot.2mg"

// .2mg header offsets
#define HDR_MAGIC       0   // 4 bytes: "2IMG"
#define HDR_BLOCKS      20  // uint32_t LE: number of 512-byte blocks
#define HDR_DATA_OFF    24  // uint32_t LE: byte offset to block data
#define HDR_DATA_LEN    28  // uint32_t LE: data length in bytes
#define HDR_SIZE        64  // minimum header size

static FATFS g_fs;
static FIL   g_fil;
static bool  g_mounted;
static uint32_t g_block_count;
static uint32_t g_data_offset;

static uint32_t read_le32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

bool sd_init(void) {
    g_mounted = false;

    // Mount FAT filesystem
    FRESULT fr = f_mount(&g_fs, "0:", 1);
    if (fr != FR_OK) {
        printf("SD: f_mount failed: %s (%d)\n", FRESULT_str(fr), fr);
        return false;
    }
    printf("SD: filesystem mounted\n");

    // Open disk image
    fr = f_open(&g_fil, IMAGE_FILENAME, FA_READ | FA_WRITE);
    if (fr != FR_OK) {
        printf("SD: cannot open %s: %s (%d)\n",
               IMAGE_FILENAME, FRESULT_str(fr), fr);
        return false;
    }

    // Read and parse .2mg header
    uint8_t hdr[HDR_SIZE];
    UINT br;
    fr = f_read(&g_fil, hdr, HDR_SIZE, &br);
    if (fr != FR_OK || br < HDR_SIZE) {
        printf("SD: header read failed\n");
        f_close(&g_fil);
        return false;
    }

    // Verify magic "2IMG"
    if (hdr[0] != '2' || hdr[1] != 'I' || hdr[2] != 'M' || hdr[3] != 'G') {
        printf("SD: bad .2mg magic: %02X %02X %02X %02X\n",
               hdr[0], hdr[1], hdr[2], hdr[3]);
        f_close(&g_fil);
        return false;
    }

    g_block_count = read_le32(&hdr[HDR_BLOCKS]);
    g_data_offset = read_le32(&hdr[HDR_DATA_OFF]);
    uint32_t data_len = read_le32(&hdr[HDR_DATA_LEN]);

    printf("SD: %s — %u blocks, data@%u (%u bytes)\n",
           IMAGE_FILENAME, g_block_count, g_data_offset, data_len);

    g_mounted = true;
    return true;
}

bool sd_read_block(uint32_t block_num, uint8_t *buf) {
    if (!g_mounted || block_num >= g_block_count)
        return false;

    FSIZE_t offset = (FSIZE_t)g_data_offset + (FSIZE_t)block_num * 512;

    // Retry on transient SD busy — card may still be programming
    // after a write.  5 attempts with escalating back-off.
    for (int try = 0; try < 5; try++) {
        FRESULT fr = f_lseek(&g_fil, offset);
        if (fr == FR_OK) {
            UINT br;
            fr = f_read(&g_fil, buf, 512, &br);
            if (fr == FR_OK && br == 512)
                return true;
        }
        sleep_ms(20 + try * 40);  // 20, 60, 100, 140, 180ms
    }
    return false;
}

bool sd_write_block(uint32_t block_num, const uint8_t *buf) {
    if (!g_mounted || block_num >= g_block_count)
        return false;

    FSIZE_t offset = (FSIZE_t)g_data_offset + (FSIZE_t)block_num * 512;

    for (int try = 0; try < 5; try++) {
        FRESULT fr = f_lseek(&g_fil, offset);
        if (fr != FR_OK) {
            sleep_ms(20 + try * 40);
            continue;
        }
        UINT bw;
        fr = f_write(&g_fil, buf, 512, &bw);
        if (fr != FR_OK || bw != 512) {
            sleep_ms(20 + try * 40);
            continue;
        }
        fr = f_sync(&g_fil);
        if (fr == FR_OK) {
            // Give card time to finish internal programming before
            // the next bus operation hits.  Without this, the very next
            // READ/WRITE finds the card still busy and fails.
            sleep_ms(10);
            return true;
        }
        sleep_ms(20 + try * 40);
    }
    return false;
}

uint32_t sd_get_block_count(void) {
    return g_block_count;
}
