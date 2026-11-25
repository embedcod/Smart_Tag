#include "EPD_SPI.h"
#include "esp_log.h"

// SPI Device Handle (configured in EPD_GPIOInit)
static spi_device_handle_t spi;

void EPD_GPIOInit(void) {
	// Configure GPIOs
	esp_rom_gpio_pad_select_gpio(EPD_RES_PIN);
    gpio_set_direction(EPD_RES_PIN, GPIO_MODE_OUTPUT);
	esp_rom_gpio_pad_select_gpio(EPD_CS_PIN);
    gpio_set_direction(EPD_CS_PIN, GPIO_MODE_OUTPUT);
	esp_rom_gpio_pad_select_gpio(EPD_DC_PIN);
    gpio_set_direction(EPD_DC_PIN, GPIO_MODE_OUTPUT);
    // Configure BUSY as input
	esp_rom_gpio_pad_select_gpio(EPD_BUSY_PIN);
    gpio_set_direction(EPD_BUSY_PIN, GPIO_MODE_INPUT);

    // Initialize SPI Bus
    spi_bus_config_t buscfg = {
        .miso_io_num = -1,  // No MISO
        .mosi_io_num = 11,
        .sclk_io_num = 12,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };

    // Initialize the SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 8 * 1000 * 1000,  
        .mode = 0,                          // SPI Mode 0
        .spics_io_num = EPD_CS_PIN,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));
	ESP_LOGI("SPI_INIT", "SPI initialized successfully");
}

// Bit-bang SPI (fallback if hardware SPI fails)
void EPD_WR_Bus(uint8_t dat) {
    EPD_CS_Clr();
    for (uint8_t i = 0; i < 8; i++) {
        EPD_SCK_Clr();
        if (dat & 0x80) EPD_MOSI_Set();
        else EPD_MOSI_Clr();
        EPD_SCK_Set();
        dat <<= 1;
    }
    EPD_CS_Set();
}

// Use ESP-IDF SPI for faster transfers
void SPI_Write(uint8_t data) {
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &data,
    };
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
}

void EPD_WR_REG(uint8_t reg) {
    EPD_DC_Clr();  // Command mode
    SPI_Write(reg);
    EPD_DC_Set();  // Data mode
}

void EPD_WR_DATA8(uint8_t dat) {
    EPD_DC_Set();  // Data mode
    SPI_Write(dat);
}

/**
 * @brief Deinitialize SPI and GPIOs to prepare for sleep mode.
 *        Call this before entering deep sleep.
 */
void EPD_SPIDeinit(void) {
    // Remove the SPI device first
    if (spi) {
        ESP_ERROR_CHECK(spi_bus_remove_device(spi));
        spi = NULL; // Clear handle
    }

    // Free the SPI bus
    ESP_ERROR_CHECK(spi_bus_free(SPI2_HOST));

    // Reset GPIOs to default state (optional but recommended)
    gpio_reset_pin(EPD_RES_PIN);
    gpio_reset_pin(EPD_CS_PIN);
    gpio_reset_pin(EPD_DC_PIN);
    gpio_reset_pin(EPD_BUSY_PIN);

    ESP_LOGI("SPI_DEINIT", "SPI deinitialized successfully");
}