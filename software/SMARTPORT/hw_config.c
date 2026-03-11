#include "hw_config.h"
#include "picoport.h"

/* SPI bus configuration */
static spi_t spis[] = {
    {
        .hw_inst = spi0,
        .miso_gpio = SD_PIN_MISO,
        .mosi_gpio = SD_PIN_MOSI,
        .sck_gpio  = SD_PIN_SCK,
        .baud_rate = 12500 * 1000,  // 12.5 MHz
    }
};

/* SD card configuration */
static sd_card_t sd_cards[] = {
    {
        .pcName = "0:",
        .spi = &spis[0],
        .ss_gpio = SD_PIN_CS,
        .use_card_detect = false,
    }
};

size_t sd_get_num(void)  { return count_of(sd_cards); }
sd_card_t *sd_get_by_num(size_t num) {
    if (num < sd_get_num()) return &sd_cards[num];
    return NULL;
}

size_t spi_get_num(void) { return count_of(spis); }
spi_t *spi_get_by_num(size_t num) {
    if (num < spi_get_num()) return &spis[num];
    return NULL;
}
