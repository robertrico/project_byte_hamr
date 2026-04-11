/* spi_sd.c — SD card SPI driver for PicoRV32
 *
 * Full init sequence: power-up clocks, CMD0, CMD8, ACMD41, CMD58.
 * Handles SD v1, SD v2 (SDSC), and SDHC/SDXC.
 * Block read (CMD17) and block write (CMD24).
 */

#include "spi_sd.h"
#include "hal.h"

/* ================================================================
 * Low-level SPI helpers
 * ================================================================ */

uint8_t spi_xfer(uint8_t tx) {
    SPI_DATA = tx;
    while (SPI_STATUS & SPI_STATUS_BUSY)
        ;
    return (uint8_t)SPI_DATA;
}

void sd_cs_assert(void) {
    uint32_t ctrl = SPI_CTRL;
    SPI_CTRL = ctrl & ~1u;  /* cs_val = 0 */
}

void sd_cs_deassert(void) {
    uint32_t ctrl = SPI_CTRL;
    SPI_CTRL = ctrl | 1u;   /* cs_val = 1 */
    spi_xfer(0xFF);          /* extra clocks after CS deassert */
}

/* Flush SPI bus — reset card state after errors.
 * CS HIGH + 80 dummy clocks lets card finish any in-progress op
 * and resync its command parser. */
static void sd_spi_flush(void) {
    sd_cs_deassert();
    for (int i = 0; i < 10; i++)
        spi_xfer(0xFF);
}

static void spi_slow(void) {
    SPI_CTRL = SPI_CTRL | 0x04;   /* slow_mode = 1 */
}

static void spi_fast(void) {
    SPI_CTRL = SPI_CTRL & ~0x04u; /* slow_mode = 0 */
}

/* Wait for card to go non-busy (response != 0xFF means data, 0xFF means busy/idle).
 * SD spec TAAC allows up to 100ms read latency. At 12.5MHz SPI (~1μs/byte
 * with CPU overhead), 200000 iterations ≈ 200ms — generous margin for cards
 * that are slow after idle periods (internal GC, wear leveling). */
static uint8_t sd_wait_response(void) {
    for (int i = 0; i < 200000; i++) {
        uint8_t r = spi_xfer(0xFF);
        if (r != 0xFF)
            return r;
    }
    return 0xFF;  /* timeout */
}

/* ================================================================
 * SD command helpers
 * ================================================================ */

/* Send a 6-byte SD command, return R1 response byte */
static uint8_t sd_cmd(uint8_t cmd, uint32_t arg) {
    uint8_t r;

    /* CS already asserted by caller */

    /* Send command byte: 01_cmd[5:0] */
    spi_xfer(0x40 | cmd);

    /* Send 32-bit argument, big-endian */
    spi_xfer((arg >> 24) & 0xFF);
    spi_xfer((arg >> 16) & 0xFF);
    spi_xfer((arg >>  8) & 0xFF);
    spi_xfer((arg >>  0) & 0xFF);

    /* CRC7 — only matters for CMD0 and CMD8 (card checks CRC in SPI mode
     * only for these two commands before CRC is disabled) */
    if (cmd == 0)
        spi_xfer(0x95);       /* Valid CRC for CMD0 with arg=0 */
    else if (cmd == 8)
        spi_xfer(0x87);       /* Valid CRC for CMD8 with arg=0x1AA */
    else
        spi_xfer(0x01);       /* Dummy CRC + stop bit */

    /* Wait for response (R1 byte, bit 7 = 0) */
    for (int i = 0; i < 10; i++) {
        r = spi_xfer(0xFF);
        if (!(r & 0x80))
            return r;
    }
    return 0xFF;  /* timeout */
}

/* Send ACMD: CMD55 prefix + the actual command */
static uint8_t sd_acmd(uint8_t cmd, uint32_t arg) {
    sd_cmd(55, 0);   /* APP_CMD prefix */
    return sd_cmd(cmd, arg);
}

/* ================================================================
 * SD card initialization
 * ================================================================ */

int sd_init(uint8_t *card_type) {
    uint8_t r;
    int i;

    *card_type = CT_NONE;

    /* Slow clock for init (≤400 kHz required by spec) */
    spi_slow();
    sd_cs_deassert();

    /* Step 1: 74+ dummy clocks with CS HIGH (power-up sequence)
     * SD spec requires this before first command */
    uart_puts("SD: 80 dummy clocks...\r\n");
    for (i = 0; i < 10; i++)
        spi_xfer(0xFF);

    /* Step 2: CMD0 — GO_IDLE_STATE (software reset)
     * Expected R1 = 0x01 (in idle state) */
    uart_puts("SD: CMD0...");
    sd_cs_assert();
    r = sd_cmd(0, 0);
    sd_cs_deassert();

    uart_puts(" R1=");
    uart_put_hex8(r);
    uart_puts("\r\n");

    if (r != 0x01) {
        uart_puts("SD: CMD0 failed\r\n");
        return SD_ERR_TIMEOUT;
    }

    /* Step 3: CMD8 — SEND_IF_COND (SD v2 detection)
     * Arg = 0x1AA: voltage range 2.7-3.6V, check pattern 0xAA
     * R1 = 0x01 + 4 bytes R7 → SD v2
     * R1 = 0x05 (illegal command) → SD v1 or MMC */
    uart_puts("SD: CMD8...");
    sd_cs_assert();
    r = sd_cmd(8, 0x000001AA);

    uint8_t cmd8_ok = 0;
    if (r == 0x01) {
        /* Read 4-byte R7 response */
        uint8_t r7[4];
        r7[0] = spi_xfer(0xFF);
        r7[1] = spi_xfer(0xFF);
        r7[2] = spi_xfer(0xFF);
        r7[3] = spi_xfer(0xFF);

        uart_puts(" R7=");
        uart_put_hex8(r7[0]);
        uart_put_hex8(r7[1]);
        uart_put_hex8(r7[2]);
        uart_put_hex8(r7[3]);
        uart_puts("\r\n");

        /* Check voltage accepted and echo pattern */
        if ((r7[2] & 0x0F) == 0x01 && r7[3] == 0xAA) {
            cmd8_ok = 1;
            uart_puts("SD: v2 card detected\r\n");
        } else {
            sd_cs_deassert();
            uart_puts("SD: CMD8 bad echo\r\n");
            return SD_ERR_CMD8;
        }
    } else {
        uart_puts(" R1=");
        uart_put_hex8(r);
        uart_puts(" (v1 card)\r\n");
    }
    sd_cs_deassert();

    /* Step 4: ACMD41 — SD_SEND_OP_COND (initialize card)
     * Must loop until card exits idle (R1 bit 0 = 0).
     * For v2 cards, set HCS bit (bit 30) to enable SDHC.
     * Some cards take 500ms+, so we use a generous timeout. */
    uart_puts("SD: ACMD41 loop");
    uint32_t acmd41_arg = cmd8_ok ? 0x40000000 : 0;  /* HCS bit for v2 */

    for (i = 0; i < 10000; i++) {
        sd_cs_assert();
        r = sd_acmd(41, acmd41_arg);
        sd_cs_deassert();

        if (r == 0x00)
            break;
        if (r != 0x01) {
            uart_puts(" err=");
            uart_put_hex8(r);
            uart_puts("\r\n");
            return SD_ERR_ACMD41;
        }

        /* Print a dot every 512 iterations for progress (power of 2, no divide) */
        if ((i & 0x1FF) == 0)
            uart_putc('.');
    }

    if (r != 0x00) {
        uart_puts(" TIMEOUT\r\n");
        return SD_ERR_ACMD41;
    }

    uart_puts(" OK (");
    uart_put_hex32(i);
    uart_puts(" tries)\r\n");

    /* Step 5: CMD58 — READ_OCR (check CCS bit for block addressing)
     * CCS = bit 30 of OCR. If set, card uses block addresses (SDHC/SDXC).
     * If clear, card uses byte addresses (SDSC). */
    uart_puts("SD: CMD58...");
    sd_cs_assert();
    r = sd_cmd(58, 0);

    if (r != 0x00) {
        sd_cs_deassert();
        uart_puts(" R1=");
        uart_put_hex8(r);
        uart_puts(" FAIL\r\n");
        return SD_ERR_CMD58;
    }

    uint8_t ocr[4];
    ocr[0] = spi_xfer(0xFF);
    ocr[1] = spi_xfer(0xFF);
    ocr[2] = spi_xfer(0xFF);
    ocr[3] = spi_xfer(0xFF);
    sd_cs_deassert();

    uart_puts(" OCR=");
    uart_put_hex8(ocr[0]);
    uart_put_hex8(ocr[1]);
    uart_put_hex8(ocr[2]);
    uart_put_hex8(ocr[3]);
    uart_puts("\r\n");

    /* Determine card type */
    if (cmd8_ok) {
        *card_type = CT_SD2;
        if (ocr[0] & 0x40) {
            *card_type |= CT_BLOCK;
            uart_puts("SD: SDHC/SDXC (block addressing)\r\n");
        } else {
            uart_puts("SD: SD v2 (byte addressing)\r\n");
        }
    } else {
        *card_type = CT_SD1;
        uart_puts("SD: SD v1\r\n");
    }

    /* Switch to fast clock for data transfers */
    spi_fast();

    uart_puts("SD: Init OK\r\n");
    return SD_OK;
}

/* ================================================================
 * Block read (CMD17 — READ_SINGLE_BLOCK)
 * ================================================================ */

int sd_read_sector(uint32_t sector, uint8_t *buf) {
    uint8_t r;

    for (int attempt = 0; attempt < 3; attempt++) {
        sd_cs_assert();
        r = sd_cmd(17, sector);

        if (r != 0x00) {
            uart_puts("\r\n!R17 R1=");
            uart_put_hex8(r);
            uart_puts("\r\n");
            sd_spi_flush();
            continue;
        }

        /* Wait for data token 0xFE */
        r = sd_wait_response();
        if (r != 0xFE) {
            uart_puts("\r\n!R17 tok=");
            uart_put_hex8(r);
            uart_puts("\r\n");
            sd_spi_flush();
            continue;
        }

        /* Read 512 bytes */
        for (int i = 0; i < 512; i++)
            buf[i] = spi_xfer(0xFF);

        /* Discard 2-byte CRC */
        spi_xfer(0xFF);
        spi_xfer(0xFF);

        sd_cs_deassert();
        return 0;
    }

    uart_puts("!R17 GIVEUP\r\n");
    return -1;
}

/* ================================================================
 * Block write (CMD24 — WRITE_SINGLE_BLOCK)
 * ================================================================ */

int sd_write_sector(uint32_t sector, const uint8_t *buf) {
    uint8_t r;

    for (int attempt = 0; attempt < 3; attempt++) {
        sd_cs_assert();

        /* Wait for card not busy before sending command.
         * Previous write may still be programming.
         * 2M iterations ≈ 2-4 seconds — covers slow SDSC cards
         * doing internal GC after multi-file seeks. */
        int busy = 1;
        for (int32_t i = 0; i < 2000000; i++) {
            if (spi_xfer(0xFF) == 0xFF) { busy = 0; break; }
        }
        if (busy) {
            uart_puts("\r\n!W24 pre-busy\r\n");
            sd_spi_flush();
            continue;
        }

        r = sd_cmd(24, sector);

        if (r != 0x00) {
            uart_puts("\r\n!W24 R1=");
            uart_put_hex8(r);
            uart_puts("\r\n");
            sd_spi_flush();
            continue;
        }

        /* Send data token */
        spi_xfer(0xFE);

        /* Send 512 bytes */
        for (int i = 0; i < 512; i++)
            spi_xfer(buf[i]);

        /* Send dummy CRC */
        spi_xfer(0xFF);
        spi_xfer(0xFF);

        /* Read data response token: xxx0_sss1
         * sss = 010 → accepted, 101 → CRC error, 110 → write error */
        r = sd_wait_response();
        if ((r & 0x1F) != 0x05) {
            uart_puts("\r\n!W24 resp=");
            uart_put_hex8(r);
            uart_puts("\r\n");
            sd_spi_flush();
            continue;
        }

        /* Card accepted the data — now wait for programming to finish.
         * Do NOT retry after this point: the data is committed, we just
         * need to wait for the card's internal write/erase to complete.
         * 5M iterations ≈ 5-10s — covers worst-case SDSC GC storms. */
        for (int32_t i = 0; i < 5000000; i++) {
            if (spi_xfer(0xFF) != 0x00) {
                sd_cs_deassert();
                return 0;
            }
        }

        /* Programming truly stuck — this is a hardware failure, not retry-able */
        uart_puts("\r\n!W24 prog-timeout\r\n");
        sd_cs_deassert();
        return -2;
    }

    uart_puts("!W24 GIVEUP\r\n");
    return -1;
}
