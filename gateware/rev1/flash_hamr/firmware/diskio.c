/* diskio.c — FatFS low-level disk I/O for Flash Hamr SD card
 *
 * Hooks FatFS to our spi_sd driver. Single drive (pdrv=0).
 */

#include "ff.h"
#include "diskio.h"
#include "spi_sd.h"
#include "hal.h"

static DSTATUS sd_status = STA_NOINIT;
static uint8_t sd_card_type = CT_NONE;

DSTATUS disk_initialize(BYTE pdrv) {
    if (pdrv != 0) return STA_NOINIT;

    int rc = sd_init(&sd_card_type);
    if (rc == SD_OK) {
        sd_status = 0;
    } else {
        sd_status = STA_NOINIT;
    }
    return sd_status;
}

DSTATUS disk_status(BYTE pdrv) {
    if (pdrv != 0) return STA_NOINIT;
    return sd_status;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count) {
    if (pdrv != 0 || !count) return RES_PARERR;
    if (sd_status & STA_NOINIT) return RES_NOTRDY;

    /* SDSC cards need byte address; SDHC uses sector number directly */
    for (UINT i = 0; i < count; i++) {
        uint32_t addr = sector + i;
        if (!(sd_card_type & CT_BLOCK))
            addr *= 512;

        int rc = sd_read_sector(addr, buff + i * 512);
        if (rc != 0) {
            uart_puts("disk_read FAIL sec=");
            uart_put_hex32(sector + i);
            uart_puts(" addr=");
            uart_put_hex32(addr);
            uart_puts(" rc=");
            uart_put_hex8(rc & 0xFF);
            uart_puts("\r\n");
            return RES_ERROR;
        }
    }
    return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count) {
    if (pdrv != 0 || !count) return RES_PARERR;
    if (sd_status & STA_NOINIT) return RES_NOTRDY;

    for (UINT i = 0; i < count; i++) {
        uint32_t addr = sector + i;
        if (!(sd_card_type & CT_BLOCK))
            addr *= 512;

        int rc = sd_write_sector(addr, buff + i * 512);
        if (rc != 0) {
            uart_puts("disk_write FAIL sec=");
            uart_put_hex32(sector + i);
            uart_puts(" addr=");
            uart_put_hex32(addr);
            uart_puts(" rc=");
            uart_put_hex8(rc & 0xFF);
            uart_puts("\r\n");
            return RES_ERROR;
        }
    }
    return RES_OK;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff) {
    if (pdrv != 0) return RES_PARERR;
    if (sd_status & STA_NOINIT) return RES_NOTRDY;

    switch (cmd) {
        case CTRL_SYNC:
            /* SD SPI writes are synchronous — nothing to flush */
            return RES_OK;

        default:
            return RES_PARERR;
    }
}
