#include <stdint.h>
#include "sp_proto.h"

// Decode SmartPort command packet fields from raw wire bytes.
//
// Takes raw wire bytes from fm_decode (starting at C3 PBEGIN)
// and extracts command fields into a cmd_struct.
//
// Wire packet layout (pkt_bytes from fm_decode):
//   [0]  = C3 PBEGIN
//   [1]  = DEST | $80
//   [2]  = SRC  | $80
//   [3]  = PTYPE | $80
//   [4]  = AUX  | $80
//   [5]  = STAT | $80
//   [6]  = ODDCNT | $80
//   [7]  = GRP7CNT | $80
//   [8]  = ODDMSB | $80
//   [9..] = odd bytes (each | $80)
//   then  = group MSB + 7 data bytes per group (each | $80)
//   then  = checksum bytes
//   last  = C8 PEND
//
// Returns: 0 on success, -1 on error

int decode_cmd(const uint8_t *pkt, int pkt_len, cmd_struct_t *cmd)
{
    if (pkt_len < 10)
        return -1;

    // Extract header fields (strip $80 mask)
    cmd->dest       = pkt[1] & 0x7F;
    cmd->source     = pkt[2] & 0x7F;
    cmd->ptype      = pkt[3] & 0x7F;

    int numodds = pkt[6] & 0x7F;
    int numgrps = pkt[7] & 0x7F;
    int oddmsb  = pkt[8] & 0x7F;

    // Sanity check: SmartPort commands have at most 2 odd bytes and 1 group.
    // Garbled FM decode can produce wild values (e.g. ODDCNT=66 from
    // a bit-6 flip of 0x82→0xC2), causing out-of-bounds reads.
    if (numodds > 6 || numgrps > 4)
        return -1;

    // Verify packet is long enough for the declared payload
    int expected = 9 + numodds + numgrps * 8 + 2;  // hdr + odds + groups + chksum
    if (pkt_len < expected)
        return -1;

    // Decode payload into temp buffer (odd bytes + group bytes)
    uint8_t decoded[16];

    // Decode odd bytes
    for (int i = 0; i < numodds && i < 6; i++) {
        uint8_t b = pkt[9 + i] & 0x7F;
        if ((oddmsb >> (6 - i)) & 1)
            b |= 0x80;
        decoded[i] = b;
    }

    // Decode first group of 7
    if (numgrps > 0) {
        int grp_off = 9 + numodds;
        int grpmsb = pkt[grp_off] & 0x7F;

        for (int i = 0; i < 7; i++) {
            uint8_t b = pkt[grp_off + 1 + i] & 0x7F;
            if ((grpmsb >> (6 - i)) & 1)
                b |= 0x80;
            decoded[numodds + i] = b;
        }
    }

    // Extract command fields from decoded data:
    //   [0] = command code
    //   [1] = param count
    //   [2] = params[0] = unit number
    //   [3] = params[1] = buffer pointer low
    //   [4] = params[2] = status_code / block_lo
    //   [5] = params[3] = block_mid
    //   [6] = params[4] = block_hi
    cmd->cmd         = decoded[0];
    cmd->status_code = decoded[4];
    cmd->block_lo    = decoded[4];
    cmd->block_mid   = decoded[5];
    cmd->block_hi    = decoded[6];

    return 0;
}
