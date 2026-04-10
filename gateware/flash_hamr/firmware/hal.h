#ifndef HAL_H
#define HAL_H

#include <stdint.h>

/* ============================================================
 * PicoRV32 Memory-Mapped Peripheral Addresses
 * ============================================================ */

/* SPI Master (SD card) */
#define SPI_BASE        0x20000000
#define SPI_DATA        (*(volatile uint32_t *)(SPI_BASE + 0x00))
#define SPI_STATUS      (*(volatile uint32_t *)(SPI_BASE + 0x04))
#define SPI_CTRL        (*(volatile uint32_t *)(SPI_BASE + 0x08))

#define SPI_STATUS_BUSY  (1 << 0)
#define SPI_STATUS_DONE  (1 << 1)
#define SPI_CTRL_CS_LOW  0   /* Assert CS (active low) */
#define SPI_CTRL_CS_HIGH 1   /* Deassert CS */
#define SPI_CTRL_SLOW    2   /* Slow clock (~200 kHz for SD init) */

/* Mailbox (CPU <-> bus_interface) */
#define MBOX_BASE       0x30000000
#define MBOX_CMD        (*(volatile uint32_t *)(MBOX_BASE + 0x00))
#define MBOX_ARG0       (*(volatile uint32_t *)(MBOX_BASE + 0x04))
#define MBOX_STATUS     (*(volatile uint32_t *)(MBOX_BASE + 0x08))
#define MBOX_FLAGS      (*(volatile uint32_t *)(MBOX_BASE + 0x0C))
#define MBOX_PERSIST_BLK (*(volatile uint32_t *)(MBOX_BASE + 0x10))
#define MBOX_TOTAL_BLK  (*(volatile uint32_t *)(MBOX_BASE + 0x14))

/* Multi-drive registers */
#define MBOX_BOOT_UNIT  (*(volatile uint32_t *)(MBOX_BASE + 0x20))
#define MBOX_UNIT_BLKCNT(n) (*(volatile uint32_t *)(MBOX_BASE + 0x24 + (n)*4))
#define MBOX_UNIT_OFFSET(n) (*(volatile uint32_t *)(MBOX_BASE + 0x34 + ((n)-1)*4))

/* Mailbox flag bits */
#define FLAG_SD_READY    (1 << 0)
#define FLAG_SD_ERROR    (1 << 1)
#define FLAG_S4D2_MOUNTED (1 << 2)
#define FLAG_S4D2_LOADING (1 << 3)
#define FLAG_CPU_BUSY    (1 << 4)

/* Mailbox commands */
#define CMD_NONE         0x00
#define CMD_MOUNT        0x01
#define CMD_SD_INIT      0x02

/* SDRAM Port */
#define SDRAM_BASE      0x40000000
#define SDRAM_ADDR      (*(volatile uint32_t *)(SDRAM_BASE + 0x00))
#define SDRAM_WDATA     (*(volatile uint32_t *)(SDRAM_BASE + 0x04))
#define SDRAM_RDATA     (*(volatile uint32_t *)(SDRAM_BASE + 0x08))
#define SDRAM_CTRL      (*(volatile uint32_t *)(SDRAM_BASE + 0x0C))

#define SDRAM_CTRL_CLAIM  (1 << 0)
#define SDRAM_CTRL_READY  (1 << 1)

/* SDRAM helpers */
static inline void sdram_claim(void) { SDRAM_CTRL = SDRAM_CTRL_CLAIM; }
static inline void sdram_release(void) { SDRAM_CTRL = 0; }

static inline void sdram_write_word(uint32_t byte_addr, uint16_t data) {
    SDRAM_ADDR = byte_addr;
    SDRAM_WDATA = data;  /* triggers write request */
    /* Wait for acceptance — the peripheral stalls the CPU bus */
}

static inline uint16_t sdram_read_word(uint32_t byte_addr) {
    SDRAM_ADDR = byte_addr;
    return (uint16_t)SDRAM_RDATA;  /* triggers read, blocks until valid */
}

/* UART TX */
#define UART_BASE       0x50000000
#define UART_DATA       (*(volatile uint32_t *)(UART_BASE + 0x00))
#define UART_STATUS     (*(volatile uint32_t *)(UART_BASE + 0x04))

#define UART_STATUS_BUSY (1 << 0)

/* Block Buffer Port */
#define BBUF_BASE         0x60000000
#define BBUF_ADDR         (*(volatile uint32_t *)(BBUF_BASE + 0x00))
#define BBUF_WDATA        (*(volatile uint32_t *)(BBUF_BASE + 0x04))
#define BBUF_RDATA        (*(volatile uint32_t *)(BBUF_BASE + 0x08))
#define BBUF_CTRL         (*(volatile uint32_t *)(BBUF_BASE + 0x0C))
#define BBUF_MAGIC_STATUS (*(volatile uint32_t *)(BBUF_BASE + 0x10))
#define BBUF_MAGIC_DONE   (*(volatile uint32_t *)(BBUF_BASE + 0x14))

#define BBUF_CTRL_CLAIM    (1 << 0)
#define BBUF_CTRL_CACHE_EN (1 << 1)  /* enable on-demand cache intercept */
#define BBUF_CACHE_RELEASE (*(volatile uint32_t *)(BBUF_BASE + 0x18))  /* release to arbiter */
#define BBUF_MAGIC_PENDING (1 << 16)  /* bit 16 of MAGIC_STATUS (legacy) */
#define BBUF_MAGIC_TOGGLE  (1 << 17)  /* bit 17: raw toggle from 7MHz CDC */

/* GPIO (directly memory-mapped) */
#define GPIO_BASE       0x70000000
#define GPIO_OUT        (*(volatile uint32_t *)(GPIO_BASE + 0x00))

/* ============================================================
 * UART printf support
 * ============================================================ */

static inline void uart_putc(char c) {
    while (UART_STATUS & UART_STATUS_BUSY)
        ;
    UART_DATA = (uint32_t)c;
}

static inline void uart_puts(const char *s) {
    while (*s)
        uart_putc(*s++);
}

/* Minimal printf-like hex output for debugging */
static inline void uart_put_hex8(uint8_t v) {
    const char hex[] = "0123456789ABCDEF";
    uart_putc(hex[v >> 4]);
    uart_putc(hex[v & 0xF]);
}

static inline void uart_put_hex32(uint32_t v) {
    uart_put_hex8((v >> 24) & 0xFF);
    uart_put_hex8((v >> 16) & 0xFF);
    uart_put_hex8((v >>  8) & 0xFF);
    uart_put_hex8((v >>  0) & 0xFF);
}

#endif /* HAL_H */
