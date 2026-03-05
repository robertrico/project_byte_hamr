#include <stdint.h>
#include "sp_proto.h"

// Build a complete SmartPort response packet with group-of-7 encoding.
//
// Wire format of response packet:
//   [0-5]   sync: FF 3F CF F3 FC FF
//   [6]     C3 (PBEGIN)
//   [7]     80 (DEST = host)
//   [8]     source | 80
//   [9]     ptype ($81=status, $82=data)
//   [10]    80 (AUX)
//   [11]    status | 80
//   [12]    numodds | 80
//   [13]    numgrps | 80
//   [14]    oddmsb | 80  (if numodds > 0)
//   [15..]  odd bytes | 80
//   then    groups: grpmsb|80, 7 data bytes|80 each
//   [N]     checksum | $AA
//   [N+1]   (checksum >> 1) | $AA
//   [N+2]   C8 (PEND)
//   [N+3]   00 (end marker)
//
// Returns: total packet length

int build_packet(uint8_t *tx_buf, uint8_t source, uint8_t ptype,
                 uint8_t status, const uint8_t *data, int data_len)
{
    if (data_len < 0) data_len = 0;
    int numgrps = data_len / 7;
    int numodds = data_len % 7;

    // Compute data checksum (XOR all data bytes)
    uint8_t checksum = 0;
    for (int i = 0; i < data_len; i++)
        checksum ^= data[i];

    // Sync bytes (standard SmartPort sync pattern)
    int pos = 0;
    tx_buf[pos++] = 0xFF;
    tx_buf[pos++] = 0x3F;
    tx_buf[pos++] = 0xCF;
    tx_buf[pos++] = 0xF3;
    tx_buf[pos++] = 0xFC;
    tx_buf[pos++] = 0xFF;

    // Header
    int hdr_start = pos;
    tx_buf[pos++] = 0xC3;              // PBEGIN
    tx_buf[pos++] = 0x80;              // DEST = host
    tx_buf[pos++] = source | 0x80;     // SOURCE
    tx_buf[pos++] = ptype;             // PTYPE (already has $80 bit)
    tx_buf[pos++] = 0x80;              // AUX
    tx_buf[pos++] = status | 0x80;     // STAT
    tx_buf[pos++] = numodds | 0x80;    // ODDCNT
    tx_buf[pos++] = numgrps | 0x80;    // GRP7CNT

    // XOR header bytes (dest through grp7cnt) into checksum
    for (int i = hdr_start + 1; i < pos; i++)
        checksum ^= tx_buf[i];

    // Encode odd bytes (if any)
    if (numodds > 0) {
        uint8_t oddmsb = 0x80;
        for (int i = 0; i < numodds; i++) {
            // oddmsb |= (data[i] & 0x80) >> (1+i)
            oddmsb |= (data[i] & 0x80) >> (1 + i);
            tx_buf[pos + 1 + i] = data[i] | 0x80;
        }
        tx_buf[pos] = oddmsb;
        pos += 1 + numodds;
    }

    // Encode groups of 7
    const uint8_t *grp_data = data + numodds;
    for (int g = 0; g < numgrps; g++) {
        // Compute grpmsb
        uint8_t grpmsb = 0;
        for (int i = 0; i < 7; i++) {
            // grpmsb |= (data[i] >> (i+1)) & (0x80 >> (i+1))
            grpmsb |= (grp_data[i] >> (i + 1)) & (0x80 >> (i + 1));
        }
        tx_buf[pos] = grpmsb | 0x80;
        for (int i = 0; i < 7; i++)
            tx_buf[pos + 1 + i] = grp_data[i] | 0x80;
        pos += 8;
        grp_data += 7;
    }

    // Checksum
    tx_buf[pos++] = checksum | 0xAA;
    tx_buf[pos++] = (checksum >> 1) | 0xAA;

    // End
    tx_buf[pos++] = 0xC8;  // PEND
    tx_buf[pos++] = 0x00;  // end marker

    return pos;
}
