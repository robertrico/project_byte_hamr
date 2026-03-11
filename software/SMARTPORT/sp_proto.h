#ifndef SP_PROTO_H
#define SP_PROTO_H

#include <stdint.h>

// SmartPort command codes
#define SP_CMD_STATUS       0x00
#define SP_CMD_READBLOCK    0x01
#define SP_CMD_WRITEBLOCK   0x02
#define SP_CMD_FORMAT       0x03
#define SP_CMD_CONTROL      0x04
#define SP_CMD_INIT         0x05

// Packet types
#define SP_PTYPE_CMD        0x80
#define SP_PTYPE_STATUS     0x81
#define SP_PTYPE_DATA       0x82

// Bus phase patterns (logical, after LUT unscramble)
#define SP_BUS_COMMAND      0x0B
#define SP_BUS_IDLE         0x0A
#define SP_BUS_RESET        0x05

// Buffer sizes
#define SAMPLE_BUF_SIZE     2048
#define PKT_MAX             64
#define TX_BUF_MAX          700

// Decoded command struct
typedef struct {
    uint8_t dest;
    uint8_t source;
    uint8_t ptype;
    uint8_t cmd;
    uint8_t status_code;
    uint8_t block_lo;
    uint8_t block_mid;
    uint8_t block_hi;
} cmd_struct_t;

// --- Assembly functions (sp_asm.S) ---

// ACK open-drain control
void ack_assert(void);
void ack_deassert(void);

// Phase/REQ helpers
uint32_t read_phases(void);
int req_wait_rise(uint32_t timeout_us);
int req_wait_fall(uint32_t timeout_us);

// PIO capture — returns sample count, data in sample_buf[]
int capture_samples(void);

// --- C functions ---

// FM decoder (fm_decode.S — stays in assembly)
int fm_decode(const uint32_t *samples, int count,
              uint8_t *pkt_out, int max_pkt);

// Packet encoder (sp_encode.c)
int build_packet(uint8_t *tx_buf, uint8_t source, uint8_t ptype,
                 uint8_t status, const uint8_t *data, int data_len);

// Packet decoder (sp_decode_pkt.c)
int decode_cmd(const uint8_t *pkt, int pkt_len, cmd_struct_t *cmd);

// Data payload decoder — inverse of build_packet() group-of-7 encoding.
// Returns decoded byte count (e.g. 512 for WRITEBLOCK), or -1 on error.
int decode_data(const uint8_t *pkt, int pkt_len, uint8_t *data_out, int max_out);

// PIO restart (main.c)
void pio_rx_restart(void);

// --- External data (sp_data.c) ---
extern const uint8_t sync_bytes[];
extern uint8_t dib_data[];
extern const uint8_t dib_data_len;
extern uint8_t gen_status_data[];
extern const uint8_t gen_status_len;

// Patch block count into DIB and general status (call after .2mg header parse)
void sp_data_set_blocks(uint32_t blocks);

// --- Shared buffers (sp_asm.S) ---
extern uint32_t sample_buf[];
extern uint8_t pkt_bytes[];
extern uint8_t tx_buf[];

#endif // SP_PROTO_H
