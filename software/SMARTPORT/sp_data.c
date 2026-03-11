#include <stdint.h>

// Sync sequence — prepended to every outgoing packet
const uint8_t sync_bytes[] = { 0xFF, 0x3F, 0xCF, 0xF3, 0xFC, 0xFF };
const uint8_t sync_bytes_len = 6;

// Device Info Block (DIB) — 25 bytes
// Returned for STATUS command with status_code = $03
// Block count bytes [1..3] are patched at runtime by sp_data_set_blocks().
uint8_t dib_data[] = {
    0xF8,                           // general status: online, read/write
    0x00, 0x00, 0x00,               // block count LE (set at runtime)
    0x08,                           // device name length
    'P','I','C','O','P','O','R','T',// device name (8 chars)
    ' ',' ',' ',' ',' ',' ',' ',' ',// padding to 16 chars total
    0x02, 0x00,                     // device type: hard disk, subtype 0
    0x00, 0x01                      // firmware version 1.0
};
const uint8_t dib_data_len = sizeof(dib_data);

// General status — 4 bytes
// Returned for STATUS command with status_code = $00
// Block count bytes [1..3] are patched at runtime by sp_data_set_blocks().
uint8_t gen_status_data[] = {
    0xF8,                           // general status: online, read/write
    0x00, 0x00, 0x00                // block count LE (set at runtime)
};
const uint8_t gen_status_len = sizeof(gen_status_data);

// Patch block count into DIB and general status arrays
void sp_data_set_blocks(uint32_t blocks) {
    dib_data[1] = blocks & 0xFF;
    dib_data[2] = (blocks >> 8) & 0xFF;
    dib_data[3] = (blocks >> 16) & 0xFF;
    gen_status_data[1] = blocks & 0xFF;
    gen_status_data[2] = (blocks >> 8) & 0xFF;
    gen_status_data[3] = (blocks >> 16) & 0xFF;
}
