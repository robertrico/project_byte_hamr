/**
 * Phase 1: Signal Monitor
 *
 * Verifies Yellow Hamr SmartPort signals are reaching ESP32-WROVER
 *
 * Wiring (Yellow Hamr GPIO → ESP32 GPIO):
 *   YH1  (phase[0]) → GPIO14
 *   YH2  (phase[1]) → GPIO27
 *   YH3  (phase[2]) → GPIO26
 *   YH4  (phase[3]) → GPIO4
 *   YH5  (wrdata)   → GPIO2   (onboard LED will flicker)
 *   YH6  (rddata)   ← GPIO21
 *   YH7  (sense)    ← GPIO22
 *   YH8  (_enbl1)   → GPIO35
 *   YH9  (_wrreq)   → GPIO32
 *   YH10 (_enbl2)   → GPIO33
 *   YH11 (rom_exp)  → GPIO25  (debug)
 *   YH13 (GND)      → GND
 */

#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "signal_monitor";

// Pin definitions - custom mapping for physical wiring convenience
#define PIN_PHI0    GPIO_NUM_14  // YH GPIO1 - phase[0]
#define PIN_PHI1    GPIO_NUM_27  // YH GPIO2 - phase[1]
#define PIN_PHI2    GPIO_NUM_26  // YH GPIO3 - phase[2]
#define PIN_PHI3    GPIO_NUM_4   // YH GPIO4 - phase[3]
#define PIN_WRDATA  GPIO_NUM_2   // YH GPIO5 - wrdata (also onboard LED)
#define PIN_RDDATA  GPIO_NUM_21  // YH GPIO6 - rddata (output to Yellow Hamr)
#define PIN_ACK     GPIO_NUM_22  // YH GPIO7 - sense/ACK (output to Yellow Hamr)
#define PIN_ENBL1   GPIO_NUM_35  // YH GPIO8 - _enbl1 (input-only pin)
#define PIN_WREQ    GPIO_NUM_32  // YH GPIO9 - _wrreq
#define PIN_ENBL2   GPIO_NUM_33  // YH GPIO10 - _enbl2
#define PIN_ROM_EXP GPIO_NUM_25  // YH GPIO11 - rom_expansion_active (debug)

// Input pin mask for bulk configuration
#define INPUT_PIN_MASK ((1ULL << PIN_PHI0) | (1ULL << PIN_PHI1) | \
                        (1ULL << PIN_PHI2) | (1ULL << PIN_PHI3) | \
                        (1ULL << PIN_WRDATA) | (1ULL << PIN_ENBL1) | \
                        (1ULL << PIN_WREQ) | (1ULL << PIN_ENBL2) | \
                        (1ULL << PIN_ROM_EXP))

// Output pin mask
#define OUTPUT_PIN_MASK ((1ULL << PIN_RDDATA) | (1ULL << PIN_ACK))

static void configure_gpio(void)
{
    // Configure inputs (signals FROM Yellow Hamr)
    gpio_config_t input_conf = {
        .pin_bit_mask = INPUT_PIN_MASK,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&input_conf);

    // Configure outputs (signals TO Yellow Hamr)
    gpio_config_t output_conf = {
        .pin_bit_mask = OUTPUT_PIN_MASK,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&output_conf);

    // Set initial states for SmartPort detection
    // ACK/sense LOW at boot signals "SmartPort device present" to Liron
    gpio_set_level(PIN_RDDATA, 1);  // Idle high
    gpio_set_level(PIN_ACK, 0);     // LOW = device present (will toggle for ACK)
}

static uint8_t read_phases(void)
{
    return (gpio_get_level(PIN_PHI3) << 3) |
           (gpio_get_level(PIN_PHI2) << 2) |
           (gpio_get_level(PIN_PHI1) << 1) |
           gpio_get_level(PIN_PHI0);
}

static const char* decode_phase(uint8_t phases)
{
    switch (phases) {
        case 0b1011: return " << SmartPort ENABLE";
        case 0b1010: return " << SmartPort ENABLE (alt)";
        case 0b0101: return " << RESET pattern";
        case 0b0000: return " << All OFF";
        case 0b1111: return " << All ON";
        default: return "";
    }
}

void app_main(void)
{
    printf("\n");
    printf("========================================\n");
    printf("Phase 1: Yellow Hamr Signal Monitor\n");
    printf("========================================\n");
    printf("\n");
    printf("Test sequence:\n");
    printf("  1. Enter Apple II monitor: CALL -151\n");
    printf("  2. Test enable: C0C9 (ON) / C0C8 (OFF)\n");
    printf("  3. Test phases: C0C1/C0C3/C0C5/C0C7\n");
    printf("  4. Run PR#4 to see SmartPort init\n");
    printf("\n");

    configure_gpio();

    ESP_LOGI(TAG, "GPIO configured, monitoring signals...");
    printf("\n");

    // State tracking
    uint8_t last_phases = 0xFF;
    bool last_enbl1 = true;
    bool last_enbl2 = true;
    bool last_rom_exp = false;
    bool last_wreq = true;
    bool last_wrdata = true;
    uint32_t last_activity_ms = 0;
    uint32_t last_heartbeat_ms = 0;

    while (1) {
        uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        bool activity = false;

        // Read signals
        bool enbl1 = gpio_get_level(PIN_ENBL1);
        bool enbl2 = gpio_get_level(PIN_ENBL2);
        bool rom_exp = gpio_get_level(PIN_ROM_EXP);
        bool wreq = gpio_get_level(PIN_WREQ);
        bool wrdata = gpio_get_level(PIN_WRDATA);
        uint8_t phases = read_phases();

        // Report enable changes
        if (enbl1 != last_enbl1) {
            printf("[ENBL1] %s  (Drive 1 %s)\n",
                   enbl1 ? "HIGH" : "LOW*",
                   enbl1 ? "disabled" : "ENABLED");
            last_enbl1 = enbl1;
            activity = true;
        }

        if (enbl2 != last_enbl2) {
            printf("[ENBL2] %s  (Drive 2 %s)\n",
                   enbl2 ? "HIGH" : "LOW*",
                   enbl2 ? "disabled" : "ENABLED");
            last_enbl2 = enbl2;
            activity = true;
        }

        if (rom_exp != last_rom_exp) {
            printf("[ROMEXP] %s  (Expansion ROM %s)\n",
                   rom_exp ? "HIGH*" : "LOW",
                   rom_exp ? "ACTIVE" : "inactive");
            last_rom_exp = rom_exp;
            activity = true;
        }

        // Report phase changes
        if (phases != last_phases) {
            printf("[PHASE] %d%d%d%d (0x%X)%s\n",
                   (phases >> 3) & 1,
                   (phases >> 2) & 1,
                   (phases >> 1) & 1,
                   phases & 1,
                   phases,
                   decode_phase(phases));
            last_phases = phases;
            activity = true;
        }

        // Report write request changes
        if (wreq != last_wreq) {
            printf("[WREQ]  %s  (Write %s)\n",
                   wreq ? "HIGH" : "LOW*",
                   wreq ? "idle" : "REQUESTED");
            last_wreq = wreq;
            activity = true;
        }

        // Report wrdata changes (debounced to avoid serial spam)
        if (wrdata != last_wrdata) {
            if (now_ms - last_activity_ms > 100) {
                printf("[WRDATA] %s\n", wrdata ? "HIGH" : "LOW");
            }
            last_wrdata = wrdata;
            activity = true;
        }

        if (activity) {
            last_activity_ms = now_ms;
        }

        // Periodic heartbeat if no activity
        if (now_ms - last_heartbeat_ms > 10000) {
            printf("... waiting (ENBL1=%s ENBL2=%s ROMEXP=%s PHASE=0x%X)\n",
                   enbl1 ? "off" : "ON",
                   enbl2 ? "off" : "ON",
                   rom_exp ? "ON" : "off",
                   phases);
            last_heartbeat_ms = now_ms;
        }

        // Small delay to prevent tight loop
        vTaskDelay(1);
    }
}
