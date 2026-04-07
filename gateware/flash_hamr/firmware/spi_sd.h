#ifndef SPI_SD_H
#define SPI_SD_H

#include <stdint.h>

/* Card type flags (set after successful init) */
#define CT_NONE   0x00
#define CT_MMC    0x01   /* MMC v3 */
#define CT_SD1    0x02   /* SD v1 */
#define CT_SD2    0x04   /* SD v2 */
#define CT_BLOCK  0x08   /* Block addressing (SDHC/SDXC) */

/* Error codes */
#define SD_OK          0
#define SD_ERR_TIMEOUT 1
#define SD_ERR_CMD8    2
#define SD_ERR_ACMD41  3
#define SD_ERR_CMD58   4

/* Initialize SD card. Returns SD_OK on success.
 * Sets *card_type to CT_* flags. */
int sd_init(uint8_t *card_type);

/* Read a 512-byte sector. addr is sector number (block-addressed). */
int sd_read_sector(uint32_t addr, uint8_t *buf);

/* Write a 512-byte sector. addr is sector number (block-addressed). */
int sd_write_sector(uint32_t addr, const uint8_t *buf);

/* Low-level SPI byte transfer */
uint8_t spi_xfer(uint8_t tx);

/* CS control */
void sd_cs_assert(void);
void sd_cs_deassert(void);

#endif /* SPI_SD_H */
