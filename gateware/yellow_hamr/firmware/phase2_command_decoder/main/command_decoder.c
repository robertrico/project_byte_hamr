/**
 * Phase 2: SmartPort Command Decoder
 *
 * Decodes SmartPort command packets from Yellow Hamr.
 * Detects enable pattern, captures raw bytes, identifies commands.
 *
 * SmartPort Timing:
 *   - 4µs per bit (250 kbps)
 *   - Sync pattern: FF 3F CF F3 FC FF
 *   - Packet start: C3
 *   - Commands: 00=STATUS, 01=READBLOCK, 02=WRITEBLOCK, 05=INIT
 *
 * Wiring (same as Phase 1):
 *   YH1  (phase[0]) → GPIO14
 *   YH2  (phase[1]) → GPIO27
 *   YH3  (phase[2]) → GPIO26
 *   YH4  (phase[3]) → GPIO4
 *   YH5  (wrdata)   → GPIO23
 *   YH6  (rddata)   ← GPIO21
 *   YH7  (sense)    ← GPIO13  (avoid GPIO22 - I2C SCL has board pull-ups)
 *   YH8  (_enbl1)   → GPIO35
 *   YH9  (_wrreq)   → GPIO32
 *   YH10 (_enbl2)   → GPIO33
 *   YH11 (Q7 debug) → GPIO25  *** ADD THIS WIRE ***
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "soc/gpio_reg.h"

static const char *TAG = "cmd_decoder";

// Pin definitions (same as Phase 1)
#define PIN_PHI0    GPIO_NUM_14
#define PIN_PHI1    GPIO_NUM_27
#define PIN_PHI2    GPIO_NUM_26
#define PIN_PHI3    GPIO_NUM_4
#define PIN_WRDATA  GPIO_NUM_23
#define PIN_RDDATA  GPIO_NUM_21
#define PIN_ACK     GPIO_NUM_13  // Changed from 22 (I2C SCL has board pull-ups)
#define PIN_ENBL1   GPIO_NUM_35
#define PIN_WREQ    GPIO_NUM_32
#define PIN_ENBL2   GPIO_NUM_33
#define PIN_DBG_Q7  GPIO_NUM_25  // Q7 debug from FPGA (YH11) - ADD WIRE!

// SmartPort timing (microseconds)
#define BIT_TIME_US     4
#define HALF_BIT_US     2
#define CAPTURE_TIMEOUT_US  10000  // 10ms max packet time

// Capture buffer
#define CAPTURE_SIZE    128
static uint8_t capture_buffer[CAPTURE_SIZE];
static int capture_len = 0;

// SmartPort sync pattern
static const uint8_t SYNC_PATTERN[] = {0xFF, 0x3F, 0xCF, 0xF3, 0xFC, 0xFF};

// Command names
static const char* get_command_name(uint8_t cmd) {
    switch (cmd) {
        case 0x00: return "STATUS";
        case 0x01: return "READBLOCK";
        case 0x02: return "WRITEBLOCK";
        case 0x03: return "FORMAT";
        case 0x04: return "CONTROL";
        case 0x05: return "INIT";
        case 0x06: return "OPEN";
        case 0x07: return "CLOSE";
        case 0x08: return "READ";
        case 0x09: return "WRITE";
        default:   return "UNKNOWN";
    }
}

/**
 * SmartPort ACK handling
 * ACK deasserted = HIGH (sense=1) = device ready to receive
 * ACK asserted = LOW (sense=0) = device acknowledging
 *
 * Using active drive instead of hi-z with pull-up for stronger signal
 */
static inline void ack_deassert(void)
{
    // Actively drive HIGH for strong sense=1 signal
    gpio_set_level(PIN_ACK, 1);
    gpio_set_direction(PIN_ACK, GPIO_MODE_OUTPUT);
}

static inline void ack_assert(void)
{
    // Drive low to acknowledge
    gpio_set_level(PIN_ACK, 0);
    gpio_set_direction(PIN_ACK, GPIO_MODE_OUTPUT);
}

static void configure_gpio(void)
{
    // Configure inputs (no pull-up/down)
    gpio_config_t input_conf = {
        .pin_bit_mask = (1ULL << PIN_PHI0) | (1ULL << PIN_PHI1) |
                        (1ULL << PIN_PHI2) | (1ULL << PIN_PHI3) |
                        (1ULL << PIN_WRDATA) | (1ULL << PIN_ENBL1) |
                        (1ULL << PIN_WREQ) | (1ULL << PIN_ENBL2),
                        // PIN_DBG_Q7 disabled until wire is added
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&input_conf);

    // Configure ACK as input/output so we can drive HIGH and read back actual pin state
    gpio_config_t ack_conf = {
        .pin_bit_mask = (1ULL << PIN_ACK),
        .mode = GPIO_MODE_INPUT_OUTPUT,  // Need INPUT_OUTPUT to read back pin state
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&ack_conf);
    gpio_set_level(PIN_ACK, 1);  // Actively drive HIGH = sense=1 = device ready

    // Configure rddata as output
    gpio_config_t output_conf = {
        .pin_bit_mask = (1ULL << PIN_RDDATA),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&output_conf);

    // Initial states
    gpio_set_level(PIN_RDDATA, 1);  // Idle high
    // ACK already set HIGH above (sense=1) - device ready to receive
}

static inline uint8_t read_phases(void)
{
    return (gpio_get_level(PIN_PHI3) << 3) |
           (gpio_get_level(PIN_PHI2) << 2) |
           (gpio_get_level(PIN_PHI1) << 1) |
           gpio_get_level(PIN_PHI0);
}

static inline bool read_wrdata(void)
{
    return gpio_get_level(PIN_WRDATA);
}

static inline bool drive_enabled(void)
{
    return !gpio_get_level(PIN_ENBL1) || !gpio_get_level(PIN_ENBL2);
}

/**
 * Read one byte from wrdata line (bit-banging)
 * SmartPort uses 4µs bit cells, MSB first
 * A '1' bit is indicated by a transition, '0' by no transition
 *
 * For now, we'll sample in the middle of each bit cell
 */
static uint8_t read_byte_raw(void)
{
    uint8_t byte = 0;

    for (int i = 0; i < 8; i++) {
        esp_rom_delay_us(HALF_BIT_US);
        byte = (byte << 1) | (read_wrdata() ? 1 : 0);
        esp_rom_delay_us(HALF_BIT_US);
    }

    return byte;
}

/**
 * Debug: Check for any activity on wrdata line
 * Returns number of transitions seen
 */
static int check_wrdata_activity(int timeout_us)
{
    int64_t start_time = esp_timer_get_time();
    int transitions = 0;
    bool last_state = read_wrdata();
    int high_count = last_state ? 1 : 0;
    int samples = 1;

    while ((esp_timer_get_time() - start_time) < timeout_us) {
        bool state = read_wrdata();
        samples++;
        if (state) high_count++;
        if (state != last_state) {
            transitions++;
            last_state = state;
        }
    }

    printf("[DEBUG] wrdata: %d samples, %d high (%.1f%%), %d transitions\n",
           samples, high_count, 100.0f * high_count / samples, transitions);

    return transitions;
}

/**
 * Wait for write request (Q7=1 in IWM, _wrreq goes LOW)
 * Returns true if _wrreq went low, false on timeout
 */
static bool wait_for_write_mode(int timeout_us)
{
    int64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < timeout_us) {
        if (gpio_get_level(PIN_WREQ) == 0) {
            return true;  // _wrreq is active (low)
        }
    }
    return false;
}

/**
 * Capture raw bytes from wrdata for analysis
 */
static void capture_packet(void)
{
    int64_t start_time = esp_timer_get_time();
    capture_len = 0;

    // Wait for _wrreq to go LOW (indicates Q7=1, write mode active)
    printf("[DEBUG] Waiting for write mode (_wrreq LOW)...\n");
    if (!wait_for_write_mode(50000)) {  // 50ms timeout
        printf("[DEBUG] Timeout waiting for write mode\n");
        printf("[DEBUG] wrdata = %d, _enbl1 = %d, _enbl2 = %d, _wrreq = %d\n",
               gpio_get_level(PIN_WRDATA),
               gpio_get_level(PIN_ENBL1),
               gpio_get_level(PIN_ENBL2),
               gpio_get_level(PIN_WREQ));
        return;
    }
    printf("[DEBUG] Write mode active! Capturing...\n");

    // Check for activity on wrdata
    int transitions = check_wrdata_activity(2000);  // 2ms check

    if (transitions == 0) {
        printf("[DEBUG] No transitions on wrdata\n");
        printf("[DEBUG] wrdata = %d, _enbl1 = %d, _enbl2 = %d, _wrreq = %d\n",
               gpio_get_level(PIN_WRDATA),
               gpio_get_level(PIN_ENBL1),
               gpio_get_level(PIN_ENBL2),
               gpio_get_level(PIN_WREQ));
    }

    // Capture bytes until timeout or buffer full
    start_time = esp_timer_get_time();  // Reset timer after finding write mode
    while (capture_len < CAPTURE_SIZE) {
        int64_t now = esp_timer_get_time();
        if (now - start_time > CAPTURE_TIMEOUT_US) {
            break;
        }

        capture_buffer[capture_len++] = read_byte_raw();

        // Check if drive still enabled
        if (!drive_enabled()) {
            break;
        }
    }
}

/**
 * Find sync pattern in capture buffer
 * Returns offset of sync pattern, or -1 if not found
 */
static int find_sync_pattern(void)
{
    if (capture_len < sizeof(SYNC_PATTERN)) {
        return -1;
    }

    for (int i = 0; i <= capture_len - sizeof(SYNC_PATTERN); i++) {
        bool match = true;
        for (int j = 0; j < sizeof(SYNC_PATTERN); j++) {
            if (capture_buffer[i + j] != SYNC_PATTERN[j]) {
                match = false;
                break;
            }
        }
        if (match) {
            return i;
        }
    }

    return -1;
}

/**
 * Analyze and print captured packet
 */
static void analyze_capture(void)
{
    printf("\n[CAPTURE] %d bytes: ", capture_len);

    // Print first 32 bytes in hex
    int print_len = (capture_len > 32) ? 32 : capture_len;
    for (int i = 0; i < print_len; i++) {
        printf("%02X ", capture_buffer[i]);
    }
    if (capture_len > 32) {
        printf("...");
    }
    printf("\n");

    // Look for sync pattern
    int sync_offset = find_sync_pattern();
    if (sync_offset >= 0) {
        printf("[SYNC] Found at offset %d\n", sync_offset);

        // Check for packet start (C3) after sync
        int pkt_start = sync_offset + sizeof(SYNC_PATTERN);
        if (pkt_start < capture_len && capture_buffer[pkt_start] == 0xC3) {
            printf("[PKT] Packet start (C3) at offset %d\n", pkt_start);

            // SmartPort packet structure after C3:
            // [0] = dest address
            // [1] = source address
            // [2] = type
            // [3] = aux
            // [4] = status
            // [5] = oddcnt
            // [6] = grpcnt
            // [7...] = data or command

            if (pkt_start + 8 < capture_len) {
                uint8_t dest = capture_buffer[pkt_start + 1];
                uint8_t src = capture_buffer[pkt_start + 2];
                uint8_t type = capture_buffer[pkt_start + 3];
                uint8_t cmd = capture_buffer[pkt_start + 8];

                printf("[PKT] Dest=%02X Src=%02X Type=%02X\n", dest, src, type);

                if (type == 0x80 || type == 0x85) {
                    // Command packet
                    printf("[CMD] Command: 0x%02X (%s)\n", cmd, get_command_name(cmd));
                }
            }
        }
    } else {
        // No sync found - check if data is all zeros or all ones
        bool all_zero = true;
        bool all_one = true;
        for (int i = 0; i < capture_len; i++) {
            if (capture_buffer[i] != 0x00) all_zero = false;
            if (capture_buffer[i] != 0xFF) all_one = false;
        }

        if (all_zero) {
            printf("[WARN] All zeros - wrdata line stuck low or timing issue\n");
        } else if (all_one) {
            printf("[WARN] All ones - wrdata line stuck high or timing issue\n");
        } else {
            printf("[INFO] No sync pattern found - timing may need adjustment\n");
        }
    }
}

void app_main(void)
{
    printf("\n");
    printf("========================================\n");
    printf("Phase 2: SmartPort Command Decoder\n");
    printf("========================================\n");
    printf("\n");
    printf("Waiting for SmartPort commands...\n");
    printf("Run PR#4 on Apple II to trigger INIT\n");
    printf("\n");

    configure_gpio();
    ESP_LOGI(TAG, "GPIO configured, waiting for SmartPort activity...");

    uint8_t last_phases = 0xFF;
    bool command_active = false;

    while (1) {
        uint8_t phases = read_phases();
        bool enabled = drive_enabled();

        // Detect SmartPort enable pattern
        if (enabled && (phases == 0x0A || phases == 0x0B)) {
            if (!command_active) {
                command_active = true;

                // IMMEDIATELY capture signals - no printf delays!
                // Sample all signals rapidly for 5ms to catch any activity
                #define FAST_SAMPLES 2000
                static uint8_t sig_log[FAST_SAMPLES];  // Packed: Q7|wrdata|wreq|enbl1|enbl2
                static uint8_t wrdata_log[FAST_SAMPLES];

                int64_t start = esp_timer_get_time();
                for (int i = 0; i < FAST_SAMPLES; i++) {
                    sig_log[i] = // (gpio_get_level(PIN_DBG_Q7) << 4) |  // Q7 disabled until wire added
                                 (gpio_get_level(PIN_WRDATA) << 3) |
                                 (gpio_get_level(PIN_WREQ) << 2) |
                                 (gpio_get_level(PIN_ENBL1) << 1) |
                                 gpio_get_level(PIN_ENBL2);
                    wrdata_log[i] = read_wrdata();
                    esp_rom_delay_us(2);  // ~2us per sample = 4ms total
                }
                int64_t elapsed = esp_timer_get_time() - start;

                // NOW print results (after capture)
                printf("\n[SP] SmartPort ENABLE detected (phase=0x%X)\n", phases);
                printf("[FAST] Captured %d samples in %lld us\n", FAST_SAMPLES, elapsed);

                // Analyze signal changes (Q7 disabled until wire added)
                int wreq_low = 0, wrdata_high = 0, transitions = 0;
                uint8_t last_sig = sig_log[0];
                for (int i = 0; i < FAST_SAMPLES; i++) {
                    if ((sig_log[i] & 0x04) == 0) wreq_low++;   // _wrreq LOW
                    if (sig_log[i] & 0x08) wrdata_high++;        // wrdata HIGH
                    if (sig_log[i] != last_sig) {
                        transitions++;
                        if (transitions <= 10) {
                            printf("[SIG@%d] wrdata=%d _wrreq=%d _enbl1=%d _enbl2=%d\n",
                                   i,
                                   (sig_log[i] >> 3) & 1,
                                   (sig_log[i] >> 2) & 1,
                                   (sig_log[i] >> 1) & 1,
                                   sig_log[i] & 1);
                        }
                        last_sig = sig_log[i];
                    }
                }
                printf("[STATS] _wrreq LOW: %d/%d, wrdata HIGH: %d/%d, transitions: %d\n",
                       wreq_low, FAST_SAMPLES, wrdata_high, FAST_SAMPLES, transitions);

                // Show first 32 wrdata samples
                printf("[WRDATA] ");
                for (int i = 0; i < 32 && i < FAST_SAMPLES; i++) {
                    printf("%d", wrdata_log[i] ? 1 : 0);
                }
                printf("...\n");

                // Current state (Q7 disabled until wire added)
                printf("[STATE] wrdata=%d _enbl1=%d _enbl2=%d _wrreq=%d ACK=%d\n",
                       gpio_get_level(PIN_WRDATA),
                       gpio_get_level(PIN_ENBL1),
                       gpio_get_level(PIN_ENBL2),
                       gpio_get_level(PIN_WREQ),
                       gpio_get_level(PIN_ACK));

                printf("[SP] Command complete\n");

                // Wait for drive to be disabled, or timeout after 3s
                // (Liron may leave drive enabled after "No Device Connected")
                int64_t wait_start = esp_timer_get_time();
                while (drive_enabled() && (esp_timer_get_time() - wait_start) < 3000000) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                command_active = false;
                printf("[SP] Ready for next command\n");
                vTaskDelay(pdMS_TO_TICKS(500));  // Debounce before re-arming
            }
        } else {
            if (command_active && !enabled) {
                command_active = false;
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }

        // Report phase changes (for debugging)
        if (phases != last_phases) {
            if (phases != 0x0A && phases != 0x0B) {
                // Don't spam during SmartPort enable
                ESP_LOGD(TAG, "Phase: 0x%X", phases);
            }
            last_phases = phases;
        }

        vTaskDelay(1);  // Yield to scheduler
    }
}
