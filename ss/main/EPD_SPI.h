#ifndef _EPD_SPI_H_
#define _EPD_SPI_H_

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ESP-IDF GPIO Config (adjust pins as needed)
#define EPD_SCK_PIN   12
#define EPD_MOSI_PIN  11
#define EPD_RES_PIN   47
#define EPD_DC_PIN    46
#define EPD_CS_PIN    45
#define EPD_BUSY_PIN  48

// GPIO Macros (ESP-IDF style)
#define EPD_SCK_Clr()  gpio_set_level(EPD_SCK_PIN, 0)
#define EPD_SCK_Set()  gpio_set_level(EPD_SCK_PIN, 1)
#define EPD_MOSI_Clr() gpio_set_level(EPD_MOSI_PIN, 0)
#define EPD_MOSI_Set() gpio_set_level(EPD_MOSI_PIN, 1)
#define EPD_RES_Clr()  gpio_set_level(EPD_RES_PIN, 0)
#define EPD_RES_Set()  gpio_set_level(EPD_RES_PIN, 1)
#define EPD_DC_Clr()   gpio_set_level(EPD_DC_PIN, 0)
#define EPD_DC_Set()   gpio_set_level(EPD_DC_PIN, 1)
#define EPD_CS_Clr()   gpio_set_level(EPD_CS_PIN, 0)
#define EPD_CS_Set()   gpio_set_level(EPD_CS_PIN, 1)
#define EPD_ReadBUSY   gpio_get_level(EPD_BUSY_PIN)

// Function Prototypes
void EPD_GPIOInit(void);
void EPD_SPIDeinit(void);
void EPD_WR_Bus(uint8_t dat);
void EPD_WR_REG(uint8_t reg);
void EPD_WR_DATA8(uint8_t dat);

#endif // _EPD_SPI_H_