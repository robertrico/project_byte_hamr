#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "picoport.h"
#include "fm_rx.pio.h"
#include "fm_tx.pio.h"

#define SAMPLE_BUF_SIZE 2048
#define PKT_MAX 64
#define SAMPLES_PER_CELL 8

static uint32_t sample_buf[SAMPLE_BUF_SIZE];
static uint8_t pkt_bytes[PKT_MAX];

extern int fm_decode(const uint32_t *samples, int count,
                     uint8_t *pkt_out, int max_pkt);

// ============================================================
// Test infrastructure
// ============================================================
static int tests_run = 0;
static int tests_passed = 0;
static int asserts_run = 0;
static int asserts_failed = 0;
static int current_test_failed = 0;

#define TEST(name) do { \
    tests_run++; \
    current_test_failed = 0; \
    printf("TEST %d: %s ... ", tests_run, name); \
} while(0)

#define PASS() do { \
    if (!current_test_failed) { tests_passed++; printf("PASS\n"); } \
} while(0)

#define FAIL(msg) do { \
    current_test_failed = 1; \
    printf("FAIL: %s\n", msg); \
} while(0)

#define ASSERT_EQ(actual, expected, label) do { \
    asserts_run++; \
    if ((actual) != (expected)) { \
        asserts_failed++; \
        current_test_failed = 1; \
        printf("FAIL: %s: got 0x%02X expected 0x%02X\n", \
               label, (unsigned)(actual), (unsigned)(expected)); \
    } \
} while(0)

#define ASSERT_GE(actual, min_val, label) do { \
    asserts_run++; \
    if ((actual) < (min_val)) { \
        asserts_failed++; \
        current_test_failed = 1; \
        printf("FAIL: %s: got %d, expected >= %d\n", \
               label, (int)(actual), (int)(min_val)); \
    } \
} while(0)

// ============================================================
// Synthetic FM signal generator
// ============================================================
// Generates oversampled pin data as if the PIO captured it.
// FM encoding: bit=1 → toggle, bit=0 → hold.
// Each bit cell = SAMPLES_PER_CELL samples at the current level.

typedef struct {
    uint32_t buf[SAMPLE_BUF_SIZE];
    int bit_pos;        // current bit position in buf
    int pin_level;      // current simulated pin state
} fm_gen_t;

static void fm_gen_init(fm_gen_t *g) {
    memset(g->buf, 0, sizeof(g->buf));
    g->bit_pos = 0;
    g->pin_level = 0;
}

// Write N samples at current pin level
static void fm_gen_samples(fm_gen_t *g, int n) {
    for (int i = 0; i < n; i++) {
        int word_idx = g->bit_pos / 32;
        int bit_idx = 31 - (g->bit_pos % 32);
        if (word_idx < SAMPLE_BUF_SIZE && g->pin_level)
            g->buf[word_idx] |= (1u << bit_idx);
        g->bit_pos++;
    }
}

// Emit one FM bit cell
static void fm_gen_bit(fm_gen_t *g, int bit) {
    if (bit) g->pin_level ^= 1;  // toggle on 1
    fm_gen_samples(g, SAMPLES_PER_CELL);
}

// Emit a byte MSB-first
static void fm_gen_byte(fm_gen_t *g, uint8_t byte) {
    for (int i = 7; i >= 0; i--)
        fm_gen_bit(g, (byte >> i) & 1);
}

// Return word count (rounded up)
static int fm_gen_words(fm_gen_t *g) {
    return (g->bit_pos + 31) / 32;
}

// ============================================================
// Test: Synthetic INIT packet decode
// ============================================================
// SmartPort INIT packet:
//   Sync: FF FF 3F CF F3 FC FF
//   PBEGIN: C3
//   Header: 81 80 80 80 80 82 81 (DEST SRC TYPE AUX STAT ODDC GRP7)
//   Group7 data + odd bytes + checksum
//   PEND: C8

static void test_init_packet(void) {
    TEST("INIT packet decode");

    fm_gen_t gen;
    fm_gen_init(&gen);

    // Lead-in idle (low for a while, then a starting edge)
    fm_gen_samples(&gen, 40);

    // Sync sequence (these bytes are FM-encoded)
    uint8_t sync[] = { 0xFF, 0xFF, 0x3F, 0xCF, 0xF3, 0xFC, 0xFF };
    for (int i = 0; i < 7; i++)
        fm_gen_byte(&gen, sync[i]);

    // PBEGIN
    fm_gen_byte(&gen, 0xC3);

    // Header: DEST=81 SRC=80 TYPE=80 AUX=80 STAT=80 ODDC=82 GRP7=81
    uint8_t header[] = { 0x81, 0x80, 0x80, 0x80, 0x80, 0x82, 0x81 };
    for (int i = 0; i < 7; i++)
        fm_gen_byte(&gen, header[i]);

    // Group-of-7 data: 7 bytes (grp7cnt=1 means 1 group)
    // For INIT with odd_count=2, data_len=9:
    //   odd bytes first (2 bytes), then 1 group of 7
    // Odd bytes: 0x80, 0x85 (example from real capture)
    fm_gen_byte(&gen, 0x80);
    fm_gen_byte(&gen, 0x85);

    // Group of 7: all 0x80 (zeros encoded)
    for (int i = 0; i < 7; i++)
        fm_gen_byte(&gen, 0x80);

    // Checksum (doesn't matter for framing test, use dummy)
    fm_gen_byte(&gen, 0xAA);

    // PEND
    fm_gen_byte(&gen, 0xC8);

    // Trailing idle
    fm_gen_samples(&gen, 80);

    // Decode
    int words = fm_gen_words(&gen);
    memset(pkt_bytes, 0, PKT_MAX);
    int pkt_len = fm_decode(gen.buf, words, pkt_bytes, PKT_MAX);

    // Trim trailing zeros
    while (pkt_len > 0 && pkt_bytes[pkt_len - 1] == 0x00)
        pkt_len--;

    // Verify
    ASSERT_GE(pkt_len, 8, "pkt_len");

    uint8_t expected[] = { 0xC3, 0x81, 0x80, 0x80, 0x80, 0x80, 0x82, 0x81 };
    const char *names[] = { "PBEGIN", "DEST", "SRC", "TYPE", "AUX", "STAT", "ODDC", "GRP7" };
    for (int i = 0; i < 8 && i < pkt_len; i++)
        ASSERT_EQ(pkt_bytes[i], expected[i], names[i]);

    PASS();

    // Print full packet for visual inspection
    printf("  PKT (%d): ", pkt_len);
    for (int i = 0; i < pkt_len; i++)
        printf("%02X ", pkt_bytes[i]);
    printf("\n");
}

// ============================================================
// Test: All-ones byte (FF) — every cell has an edge
// ============================================================
static void test_byte_ff(void) {
    TEST("FM byte 0xFF decode");

    fm_gen_t gen;
    fm_gen_init(&gen);

    fm_gen_samples(&gen, 16);

    // 10 sync FFs (gives plenty of 1-runs for sync detect)
    for (int i = 0; i < 10; i++)
        fm_gen_byte(&gen, 0xFF);

    // C3 then FF as data
    fm_gen_byte(&gen, 0xC3);
    fm_gen_byte(&gen, 0xFF);

    // Stop with zeros so packet ends
    for (int i = 0; i < 4; i++)
        fm_gen_byte(&gen, 0x00);

    fm_gen_samples(&gen, 80);

    int words = fm_gen_words(&gen);
    memset(pkt_bytes, 0, PKT_MAX);
    int pkt_len = fm_decode(gen.buf, words, pkt_bytes, PKT_MAX);

    while (pkt_len > 0 && pkt_bytes[pkt_len - 1] == 0x00)
        pkt_len--;

    ASSERT_GE(pkt_len, 2, "pkt_len");
    ASSERT_EQ(pkt_bytes[0], 0xC3, "PBEGIN");
    ASSERT_EQ(pkt_bytes[1], 0xFF, "data");
    PASS();
}

// ============================================================
// Test: Byte with long zero runs (0x81 = 10000001)
// ============================================================
static void test_byte_81(void) {
    TEST("FM byte 0x81 decode (long gap)");

    fm_gen_t gen;
    fm_gen_init(&gen);

    fm_gen_samples(&gen, 16);

    for (int i = 0; i < 10; i++)
        fm_gen_byte(&gen, 0xFF);

    fm_gen_byte(&gen, 0xC3);
    fm_gen_byte(&gen, 0x81);

    for (int i = 0; i < 4; i++)
        fm_gen_byte(&gen, 0x00);

    fm_gen_samples(&gen, 80);

    int words = fm_gen_words(&gen);
    memset(pkt_bytes, 0, PKT_MAX);
    int pkt_len = fm_decode(gen.buf, words, pkt_bytes, PKT_MAX);

    while (pkt_len > 0 && pkt_bytes[pkt_len - 1] == 0x00)
        pkt_len--;

    ASSERT_GE(pkt_len, 2, "pkt_len");
    ASSERT_EQ(pkt_bytes[0], 0xC3, "PBEGIN");
    ASSERT_EQ(pkt_bytes[1], 0x81, "data");
    PASS();
}

// ============================================================
// Test: PIO loopback (TX → RX on hardware)
// Wire GPIO 7 (RDDATA/TX) to GPIO 6 (WRDATA/RX) for this test
// ============================================================
static void test_pio_loopback(void) {
    TEST("PIO loopback TX→RX");

    PIO pio_tx = pio0;
    PIO pio_rx = pio1;
    uint sm_tx = 0, sm_rx = 0;

    uint off_tx = pio_add_program(pio_tx, &fm_tx_program);
    uint off_rx = pio_add_program(pio_rx, &fm_rx_program);

    fm_tx_program_init(pio_tx, sm_tx, off_tx, SP_RDDATA);
    fm_rx_program_init(pio_rx, sm_rx, off_rx, SP_WRDATA);

    // Transmit sync + C3 + one test byte
    uint8_t tx_data[] = {
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF,   // sync
        0x3F, 0xCF, 0xF3, 0xFC, 0xFF,   // sync cont'd
        0xC3,                            // PBEGIN
        0x81,                            // test byte
        0xA5,                            // test byte
        0xC8,                            // PEND
        0x00, 0x00, 0x00, 0x00           // trailing zeros
    };

    for (int i = 0; i < (int)sizeof(tx_data); i++)
        pio_sm_put_blocking(pio_tx, sm_tx, (uint32_t)tx_data[i] << 24);

    // Wait for transmission to complete
    sleep_ms(10);

    // Collect RX samples
    uint count = 0;
    uint64_t deadline = time_us_64() + 5000;
    while (time_us_64() < deadline && count < SAMPLE_BUF_SIZE) {
        if (!pio_sm_is_rx_fifo_empty(pio_rx, sm_rx)) {
            sample_buf[count++] = pio_sm_get(pio_rx, sm_rx);
        }
    }

    // Cleanup PIOs
    pio_sm_set_enabled(pio_tx, sm_tx, false);
    pio_sm_set_enabled(pio_rx, sm_rx, false);
    pio_remove_program(pio_tx, &fm_tx_program, off_tx);
    pio_remove_program(pio_rx, &fm_rx_program, off_rx);

    if (count == 0) {
        FAIL("no RX samples (check loopback wire GPIO7→GPIO6)");
        return;
    }

    memset(pkt_bytes, 0, PKT_MAX);
    int pkt_len = fm_decode(sample_buf, count, pkt_bytes, PKT_MAX);

    while (pkt_len > 0 && pkt_bytes[pkt_len - 1] == 0x00)
        pkt_len--;

    printf("(%u words, %d bytes) ", count, pkt_len);

    ASSERT_GE(pkt_len, 3, "pkt_len");
    ASSERT_EQ(pkt_bytes[0], 0xC3, "PBEGIN");
    ASSERT_EQ(pkt_bytes[1], 0x81, "byte1");
    ASSERT_EQ(pkt_bytes[2], 0xA5, "byte2");
    PASS();

    if (current_test_failed) {
        printf("  PKT (%d): ", pkt_len);
        for (int i = 0; i < pkt_len; i++)
            printf("%02X ", pkt_bytes[i]);
        printf("\n");
    }
}

// ============================================================
// Main
// ============================================================
static void run_tests(void) {
    tests_run = tests_passed = asserts_run = asserts_failed = 0;

    printf("\n=== PicoPort Test Suite ===\n\n");

    // Synthetic tests (no hardware needed)
    test_byte_ff();
    test_byte_81();
    test_init_packet();

    // Hardware loopback test (needs GPIO7→GPIO6 wire)
    test_pio_loopback();

    printf("\n=== Results: %d/%d tests passed, %d/%d asserts passed ===\n",
           tests_passed, tests_run,
           asserts_run - asserts_failed, asserts_run);
    printf("=== %s ===\n\n", (tests_passed == tests_run) ? "ALL PASS" : "FAILURES");
}

int main() {
    stdio_init_all();
    sleep_ms(5000);  // extra time to connect monitor

    // Run tests, then repeat every 10s so you can see results
    while (true) {
        run_tests();
        sleep_ms(10000);
    }
}
