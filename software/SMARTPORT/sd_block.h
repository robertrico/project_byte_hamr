#ifndef SD_BLOCK_H
#define SD_BLOCK_H

#include <stdint.h>
#include <stdbool.h>

// Initialize SD card, mount FAT filesystem, open pico_boot.2mg,
// and parse the .2mg header. Returns true on success.
bool sd_init(void);

// Read a 512-byte block from the mounted disk image.
// Returns true on success.
bool sd_read_block(uint32_t block_num, uint8_t *buf);

// Total block count from the .2mg header.
uint32_t sd_get_block_count(void);

#endif // SD_BLOCK_H
