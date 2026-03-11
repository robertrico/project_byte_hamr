#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/structs/pio.h"

#include "picoport.h"
#include "sp_proto.h"
#include "sd_block.h"

// PIO RX base — defined in sp_asm.S, shared with capture_samples
extern pio_hw_t *g_pio_rx_hw;

// PIO TX base — local to this file
static pio_hw_t *g_pio_tx_hw;

// Our assigned SmartPort unit number (0 = unassigned)
static uint32_t g_unit;

// Last TX packet length (set by handlers, used by verify)
static int g_tx_len;

// ============================================================
// Lock-free log ring: Core 0 writes, Core 1 reads
// ============================================================
#define LOG_MAX 64
typedef struct {
    uint8_t cmd;
    uint8_t dest;
    uint8_t status;
    uint8_t flags;      // bit0=handled, bit1=timeout, bit2=chksum fail
    int8_t chkval;      // verify_packet return value
    uint16_t samples;
    uint16_t pkt_len;
    uint16_t tx_len;    // TX packet length sent
    uint16_t block;     // block number (for READ/WRITE)
    uint8_t raw[24];    // first 24 raw RX packet bytes
    uint8_t tx_hdr[24]; // first 24 TX buffer bytes (sync+header+start of data)
} log_entry_t;

static volatile log_entry_t g_log[LOG_MAX];
static volatile uint32_t g_log_wr;
static volatile uint32_t g_log_rd;

static void log_event(uint8_t cmd, uint8_t dest, uint8_t status,
                      uint8_t flags, int8_t chkval, int samples, int pkt_len,
                      int tx_len, uint16_t block)
{
    uint32_t wr = g_log_wr;
    uint32_t next = (wr + 1) & (LOG_MAX - 1);
    if (next == g_log_rd)
        return;
    g_log[wr].cmd = cmd;
    g_log[wr].dest = dest;
    g_log[wr].status = status;
    g_log[wr].flags = flags;
    g_log[wr].chkval = chkval;
    g_log[wr].samples = (uint16_t)samples;
    g_log[wr].pkt_len = (uint16_t)pkt_len;
    g_log[wr].tx_len = (uint16_t)tx_len;
    g_log[wr].block = block;
    int copy = pkt_len < 24 ? pkt_len : 24;
    for (int i = 0; i < copy; i++)
        g_log[wr].raw[i] = pkt_bytes[i];
    for (int i = copy; i < 24; i++)
        g_log[wr].raw[i] = 0;
    int tcopy = tx_len < 24 ? tx_len : 24;
    for (int i = 0; i < tcopy; i++)
        g_log[wr].tx_hdr[i] = tx_buf[i];
    for (int i = tcopy; i < 24; i++)
        g_log[wr].tx_hdr[i] = 0;
    __dmb();
    g_log_wr = next;
}

static void log_drain(void)
{
    while (g_log_rd != g_log_wr) {
        uint32_t rd = g_log_rd;
        volatile log_entry_t *e = &g_log[rd];

        const char *name = "???";
        switch (e->cmd) {
        case 0x00: name = "STATUS"; break;
        case 0x01: name = "READ"; break;
        case 0x02: name = "WRITE"; break;
        case 0x03: name = "FORMAT"; break;
        case 0x04: name = "CONTROL"; break;
        case 0x05: name = "INIT"; break;
        }
        printf("  %s dest=%d st=$%02X %s%s [%d samp, %d pkt, tx=%d",
               name, e->dest, e->status,
               (e->flags & 8) ? "RX_CHK" :
               (e->flags & 2) ? "TIMEOUT" :
               (e->flags & 1) ? "OK" : "SKIP",
               (e->flags & 4) ? " TX_CHK" : "",
               e->samples, e->pkt_len, e->tx_len);
        if (e->cmd == 0x01)
            printf(" blk=%d", e->block);
        printf("]");
        if (e->flags & (4 | 8))
            printf(" chk=%d", e->chkval);
        printf("\n    rx:");
        int n = e->pkt_len < 24 ? e->pkt_len : 24;
        for (int i = 0; i < n; i++)
            printf(" %02X", e->raw[i]);
        if (e->tx_len > 0) {
            printf("\n    tx:");
            int tn = e->tx_len < 24 ? e->tx_len : 24;
            for (int i = 0; i < tn; i++)
                printf(" %02X", e->tx_hdr[i]);
        }
        printf("\n");

        __dmb();
        g_log_rd = (rd + 1) & (LOG_MAX - 1);
    }
}

// ============================================================
// verify_packet — Simulate ROM's decode + checksum check
// ============================================================
// Walks the encoded tx_buf exactly as the ROM would.
// Returns 0 if checksum OK, nonzero if fail.
static int verify_packet(const uint8_t *buf, int len)
{
    // Find PBEGIN ($C3) — skip sync bytes
    int i = 0;
    while (i < len && buf[i] != 0xC3)
        i++;
    if (i >= len) return -1;
    i++;  // skip C3

    // Read 7 header bytes, XOR (byte | $80) into checksum
    // ROM does: AND $7F, EOR $80 = effectively ORA $80
    if (i + 7 > len) return -2;
    uint8_t checksum = 0;
    uint8_t dest   = buf[i++] & 0x7F;  (void)dest;
    uint8_t source = buf[i-1]; checksum ^= (buf[i-1] | 0x80);
    source = buf[i++] & 0x7F; checksum ^= (buf[i-1] | 0x80);
    uint8_t ptype  = buf[i++] & 0x7F; checksum ^= (buf[i-1] | 0x80);
    uint8_t aux    = buf[i++] & 0x7F; checksum ^= (buf[i-1] | 0x80); (void)aux;
    uint8_t stat   = buf[i++] & 0x7F; checksum ^= (buf[i-1] | 0x80); (void)stat;
    int oddcnt     = buf[i++] & 0x7F; checksum ^= (buf[i-1] | 0x80);
    int grpcnt     = buf[i++] & 0x7F; checksum ^= (buf[i-1] | 0x80);

    // Decode odd bytes first (wire order: odds before groups)
    if (oddcnt > 0) {
        if (i + 1 + oddcnt > len) return -3;
        uint8_t oddmsb = buf[i++] & 0x7F;
        for (int j = 0; j < oddcnt; j++) {
            uint8_t wire = buf[i++];
            uint8_t bit7 = (oddmsb >> (6 - j)) & 1;
            uint8_t decoded = (wire & 0x7F) | (bit7 << 7);
            checksum ^= decoded;
        }
    }

    // Decode groups of 7
    for (int g = 0; g < grpcnt; g++) {
        if (i + 8 > len) return -4;
        uint8_t grpmsb = buf[i++] & 0x7F;
        for (int j = 0; j < 7; j++) {
            uint8_t wire = buf[i++];
            uint8_t bit7 = (grpmsb >> (6 - j)) & 1;
            uint8_t decoded = (wire & 0x7F) | (bit7 << 7);
            checksum ^= decoded;
        }
    }

    // Read checksum bytes from packet
    if (i + 2 > len) return -5;
    uint8_t ckbyte1 = buf[i++];   // even bits | $AA
    uint8_t ckbyte2 = buf[i++];   // odd bits >> 1 | $AA

    // Reconstruct: SEC ROL ckbyte2, AND ckbyte1
    uint8_t shifted = (ckbyte2 << 1) | 1;
    uint8_t pkt_cksum = shifted & ckbyte1;

    // Final check: checksum XOR pkt_cksum should be 0
    return checksum ^ pkt_cksum;
}

// ============================================================
// Core 1 entry: USB serial log consumer
// ============================================================
static void core1_main(void)
{
    for (;;) {
        log_drain();
        sleep_ms(1);
    }
}

// ============================================================
// send_packet_pio — Feed TX buffer to PIO TX FIFO (SM1)
// ============================================================
extern void pio_tx_restart(void);

static void send_packet_pio(const uint8_t *buf, int len)
{
    // Reset TX SM to ensure clean state (no leftover OSR bits)
    pio_tx_restart();

    for (int i = 0; i < len; i++) {
        while (g_pio_tx_hw->fstat & (1u << 17))
            tight_loop_contents();
        g_pio_tx_hw->txf[1] = (uint32_t)buf[i] << 24;
    }
}

// ============================================================
// decode_samples — FM decode captured samples + trim
// ============================================================
static int decode_samples(int sample_count)
{
    if (sample_count == 0)
        return 0;
    int pkt_len = fm_decode(sample_buf, sample_count, pkt_bytes, PKT_MAX);
    while (pkt_len > 0 && pkt_bytes[pkt_len - 1] == 0)
        pkt_len--;
    return pkt_len;
}

// ============================================================
// flush_rx_fifo — Drain stale PIO RX data
// ============================================================
static void flush_rx_fifo(void)
{
    while (!(g_pio_rx_hw->fstat & (1u << 8)))
        (void)g_pio_rx_hw->rxf[0];
}

static inline bool rx_fifo_has_data(void)
{
    return !(g_pio_rx_hw->fstat & (1u << 8));
}

// ============================================================
// sp_delay_us — Hardware timer busy-wait delay
// ============================================================
static void sp_delay_us(uint32_t us)
{
    busy_wait_us_32(us);
}

// ============================================================
// do_handshake_send — Common reply send sequence
// ============================================================
static int do_handshake_send(int pkt_len)
{
    ack_deassert();

    if (req_wait_rise(30000))
        return -1;

    // Wait for ROM's Q6 flush before sending.
    // The Liron ROM asserts REQ ($C982: LDA $C081,X) then immediately
    // reads Q6 OFF ($C987: LDA $C08C,X) just 6 cycles (6µs) later.
    // That Q6 falling edge flushes the IWM's read shift register.
    // If our rddata edges arrive before the flush, the flush destroys
    // the in-progress sync and the IWM re-syncs with a random byte
    // framing offset — the ROM never finds $C3 (PBEGIN).
    // A 15µs delay ensures the flush completes before our first edge.
    sp_delay_us(15);

    // Enable TX output (set RDDATA pin to output)
    pio_sm_set_consecutive_pindirs(pio0, 1, SP_RDDATA, 1, true);

    send_packet_pio(tx_buf, pkt_len);

    // Wait for PIO TX SM1 FIFO to drain (TXEMPTY = FSTAT bit 25)
    while (!(g_pio_tx_hw->fstat & (1u << 25)))
        tight_loop_contents();
    sp_delay_us(40);  // last byte still shifting out in OSR

    // Disable TX output (make RDDATA pin hi-Z to avoid crosstalk)
    pio_sm_set_consecutive_pindirs(pio0, 1, SP_RDDATA, 1, false);

    // Signal "packet complete" — ROM polls SENSE, won't drop REQ until it sees ACK
    ack_assert();

    // Now wait for ROM to deassert REQ
    if (req_wait_fall(30000)) {
        ack_deassert();
        return -1;
    }

    sp_delay_us(1);
    ack_deassert();

    // Flush stale RX FIFO — our own TX leaked noise into PIO RX
    flush_rx_fifo();
    pio_rx_restart();

    return 0;
}

// ============================================================
// Handlers
// ============================================================
static int do_init(cmd_struct_t *cmd)
{
    g_unit = cmd->dest;
    g_tx_len = build_packet(tx_buf, g_unit, SP_PTYPE_STATUS, 0xFF, NULL, 0);
    return do_handshake_send(g_tx_len);
}

static int do_status(cmd_struct_t *cmd)
{
    const uint8_t *data;
    int data_len;
    if (cmd->status_code == 0x03) {
        data = dib_data;
        data_len = dib_data_len;
    } else {
        data = gen_status_data;
        data_len = gen_status_len;
    }
    g_tx_len = build_packet(tx_buf, g_unit, SP_PTYPE_STATUS, 0x00, data, data_len);
    return do_handshake_send(g_tx_len);
}

// Block read buffer — filled from SD card per request
static uint8_t g_block_buf[512];

static int do_readblock(cmd_struct_t *cmd)
{
    uint32_t block = cmd->block_lo | ((uint32_t)cmd->block_mid << 8) | ((uint32_t)cmd->block_hi << 16);
    if (block >= sd_get_block_count()) {
        g_tx_len = build_packet(tx_buf, g_unit, SP_PTYPE_STATUS, 0x27, NULL, 0);
        return do_handshake_send(g_tx_len);
    }
    if (!sd_read_block(block, g_block_buf)) {
        // SD read failed — return I/O error
        g_tx_len = build_packet(tx_buf, g_unit, SP_PTYPE_STATUS, 0x27, NULL, 0);
        return do_handshake_send(g_tx_len);
    }
    g_tx_len = build_packet(tx_buf, g_unit, SP_PTYPE_DATA, 0x00, g_block_buf, 512);
    return do_handshake_send(g_tx_len);
}

// ============================================================
// sp_main — SmartPort bus loop on Core 0
// ============================================================
void sp_main(PIO pio_rx, uint sm_rx, PIO pio_tx, uint sm_tx)
{
    (void)sm_rx;
    (void)sm_tx;

    g_pio_rx_hw = (pio_hw_t *)pio_rx;
    g_pio_tx_hw = (pio_hw_t *)pio_tx;
    g_unit = 0;
    g_log_wr = 0;
    g_log_rd = 0;

    multicore_launch_core1(core1_main);

    // Flush stale RX data from boot delay (Apple II sends INITs while we print diagnostics)
    flush_rx_fifo();
    pio_rx_restart();
    pio_tx_restart();

    printf("sp_main: core0=bus, core1=serial\n");
    sleep_ms(10);

    for (;;) {
        uint32_t phases = read_phases();

        if (phases == SP_BUS_RESET) {
            pio_rx_restart();
            continue;
        }

        if (phases != SP_BUS_COMMAND || !rx_fifo_has_data()) {
            if (phases != SP_BUS_COMMAND && phases != SP_BUS_RESET)
                flush_rx_fifo();
            continue;
        }

        int sample_count = capture_samples();
        if (sample_count == 0) {
            pio_rx_restart();
            continue;
        }

        // ACK must happen fast — Liron's $C943 ACK poll is only
        // ~108µs (10 iterations).  Check REQ and ACK immediately,
        // then decode after the handshake is secured.
        {
            uint32_t gpio_in = *((volatile uint32_t *)0xD0000004);
            if (!((gpio_in >> SP_REQ) & 1)) {
                pio_rx_restart();
                continue;
            }
        }

        ack_assert();

        if (req_wait_fall(5500)) {
            ack_deassert();
            pio_rx_restart();
            continue;
        }

        int pkt_len = decode_samples(sample_count);
        if (pkt_len == 0) {
            // FM decode produced nothing — send error so Liron retries
            // instead of hanging waiting for RDDATA that never comes.
            if (g_unit != 0) {
                g_tx_len = build_packet(tx_buf, g_unit, SP_PTYPE_STATUS, 0x27, NULL, 0);
                do_handshake_send(g_tx_len);
            } else {
                ack_deassert();
                pio_rx_restart();
            }
            continue;
        }

        // Fix 1-bit insertion from Liron inter-byte pauses.
        // The 6502's timing gaps between IWM writes can stretch FM
        // bit cells, causing the PIO decoder to sample a spurious 0-bit.
        // This shifts all subsequent bytes right by 1 bit, clearing bit 7.
        // All SmartPort wire bytes after PBEGIN must have bit 7 = 1.
        // Scan the entire packet (not just byte 9+) and allow up to 2
        // corrections, since insertions can occur at bytes 3, 5, or 9.
        {
            int corrections = 0;
            for (int fix = 1; fix < pkt_len && corrections < 2; fix++) {
                if (!(pkt_bytes[fix] & 0x80)) {
                    for (int j = fix; j < pkt_len - 1; j++)
                        pkt_bytes[j] = (pkt_bytes[j] << 1) | (pkt_bytes[j + 1] >> 7);
                    pkt_bytes[pkt_len - 1] <<= 1;
                    corrections++;
                    fix--;  // re-check this position after shift
                }
            }
        }

        cmd_struct_t cmd;
        memset(&cmd, 0, sizeof(cmd));
        if (decode_cmd(pkt_bytes, pkt_len, &cmd) < 0) {
            // Garbled packet — send error so Liron retries immediately.
            if (g_unit != 0) {
                g_tx_len = build_packet(tx_buf, g_unit, SP_PTYPE_STATUS, 0x27, NULL, 0);
                do_handshake_send(g_tx_len);
            } else {
                ack_deassert();
                pio_rx_restart();
            }
            continue;
        }

        // Verify RX packet checksum — log for diagnostics, don't reject.
        // FM decode has consistent tail corruption (last ~5 bytes) but
        // command fields in early bytes are reliable.  Host retries if needed.
        int rx_chk = verify_packet(pkt_bytes, pkt_len);

        int rc = -99;
        int handled = 1;
        if (g_unit == 0) {
            if (cmd.cmd == SP_CMD_INIT)
                rc = do_init(&cmd);
            else {
                ack_deassert();
                handled = 0;
            }
        } else if (cmd.dest != g_unit) {
            ack_deassert();
            handled = 0;
        } else {
            switch (cmd.cmd) {
            case SP_CMD_INIT:      rc = do_init(&cmd);      break;
            case SP_CMD_STATUS:    rc = do_status(&cmd);     break;
            case SP_CMD_READBLOCK: rc = do_readblock(&cmd);  break;
            default:
                ack_deassert();
                handled = 0;
                break;
            }
        }

        // Verify our own packet checksum (fast, no I/O)
        int chk = 0;
        if (handled && rc == 0 && g_tx_len > 0) {
            chk = verify_packet(tx_buf, g_tx_len);
        }

        // Only restart PIO RX if do_handshake_send didn't already do it.
        // do_handshake_send calls pio_rx_restart() on success (rc==0).
        if (!handled || rc != 0)
            pio_rx_restart();

        uint8_t flags = 0;
        if (handled) flags |= 1;
        if (rc == -1) flags |= 2;
        if (chk != 0) flags |= 4;
        if (rx_chk != 0) flags |= 8;
        uint16_t blknum = cmd.block_lo | ((uint16_t)cmd.block_mid << 8);
        log_event(cmd.cmd, cmd.dest, cmd.status_code, flags,
                  (int8_t)(chk != 0 ? chk : rx_chk),
                  sample_count, pkt_len, handled ? g_tx_len : 0,
                  blknum);
    }
}
