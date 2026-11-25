/*
* - Steps to set up the .bin image
* - Go to paint and resize the images to 400 X 300 and save then head to https://javl.github.io/image2cpp/
* - Code output format: plain bytes
* - Draw mode: Horizontal - 1 bit per pixel
*/


#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_log.h"
#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "freertos/FreeRTOS.h"
// EPD
#include "EPD_GUI.h"
#include "EPD_SPI.h"
#include "EPD.h"

#define MOUNT_POINT "/sdcard"

#define SD_CS_PIN 10  
#define EPD_POWER_PIN 7
#define TF_POWER_PIN 42

static uint8_t Image_BW[15000];
static uint8_t loaded_Image[15000];

static const char *TAG = "SD_CARD";

// Function Prototypes
spi_bus_config_t sdbus_cfg;
bool sd_init ();
bool load_images(const char *filename);
// epd
void displayui(void);
void clear_all(void);
void epd_power_on();
void epd_power_off();
void tf_power_on();
void tf_power_off();

//------------------------------------------------------------------------------------------------	
void app_main(void) {
	vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Main application started!");
	
	if (!sd_init()) {
		ESP_LOGI(TAG, "SD Card Init failed!");
		return;
	}
	vTaskDelay(pdMS_TO_TICKS(100));
	// Init epd
	EPD_GPIOInit();	
	vTaskDelay(pdMS_TO_TICKS(100));
	
	while(1) {
		if (!load_images("main.bin")) {
			ESP_LOGI(TAG, "Image loading failed!");
			return;
		}
		vTaskDelay(pdMS_TO_TICKS(100));		
		displayui();
		
		vTaskDelay(pdMS_TO_TICKS(2000));	
		
		if (!load_images("main_2.bin")) {
			ESP_LOGI(TAG, "Image loading failed!");
			return;
		}
		vTaskDelay(pdMS_TO_TICKS(100));		
		displayui();
		
		vTaskDelay(pdMS_TO_TICKS(2000));
		
		if (!load_images("main_3.bin")) {
			ESP_LOGI(TAG, "Image loading failed!");
			return;
		}
		vTaskDelay(pdMS_TO_TICKS(100));		
		displayui();
		
		vTaskDelay(pdMS_TO_TICKS(30000));
	}
}
//------------------------------------------------------------------------------------------------	


bool sd_init () {
	tf_power_on();
	spi_bus_config_t sdbus_cfg = {
	    .mosi_io_num = 40, 
	    .miso_io_num = 13, 
	    .sclk_io_num = 39, 
	    .quadwp_io_num = -1,
	    .quadhd_io_num = -1,
	    .max_transfer_sz = 16 * 1024,
	};
	
    esp_err_t ret;
    sdmmc_card_t *card;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
			host.slot = SPI3_HOST;

    ret = spi_bus_initialize(SPI3_HOST, &sdbus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus: %s", esp_err_to_name(ret));
        return false;
    } else {
		ESP_LOGI(TAG, "Sd card spi initialized successfuly!");
	}

    // Mount configuration settings
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS_PIN; // Configure the CS pin internally.
    slot_config.host_id = SPI3_HOST;

    // Mount the filesystem
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount filesystem");
        spi_bus_free(host.slot);
        return false;
    }
    // Print card information
    sdmmc_card_print_info(stdout, card);
    return true;
}

bool load_images(const char *filename) {

    char filepath[64];

    // Build full SD card path
    snprintf(filepath, sizeof(filepath), "%s/%s", MOUNT_POINT, filename);

    ESP_LOGI(TAG, "Opening file: %s", filepath);

    FILE *f = fopen(filepath, "rb");

    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open image file.");
        return false;
    }

    ESP_LOGI(TAG, "Loading image from SD card...");

    // Read exactly 15000 bytes
    size_t bytes_read = fread(loaded_Image, 1, sizeof(loaded_Image), f);

    fclose(f);

    ESP_LOGI(TAG, "Bytes read: %d", bytes_read);

    // Validate image size
    if (bytes_read != sizeof(loaded_Image)) {
        ESP_LOGE(TAG, "Invalid image size!");
        return false;
    }

    ESP_LOGI(TAG, "Image loaded successfully.");

    return true;
}
/*******************************************************************************************************************************
 EPD Functions
*********************************/
void clear_all(void) {
    EPD_Clear();
    Paint_NewImage(Image_BW, EPD_W, EPD_H, 0, WHITE);
    EPD_Full(WHITE);
    EPD_Display_Part(0, 0, EPD_W, EPD_H, Image_BW);
}

void epd_power_on() {
    esp_rom_gpio_pad_select_gpio(EPD_POWER_PIN);
    gpio_set_direction(EPD_POWER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(EPD_POWER_PIN, 1); 
    vTaskDelay(pdMS_TO_TICKS(1000)); // Stabilize power
}

void tf_power_on() {
    esp_rom_gpio_pad_select_gpio(TF_POWER_PIN);
    gpio_set_direction(TF_POWER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(TF_POWER_PIN, 1); 
    vTaskDelay(pdMS_TO_TICKS(1000)); // Stabilize power
}


void epd_power_off() {
    esp_rom_gpio_pad_select_gpio(EPD_POWER_PIN);
    gpio_set_direction(EPD_POWER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(EPD_POWER_PIN, 0); 
}

void tf_power_off() {
    esp_rom_gpio_pad_select_gpio(TF_POWER_PIN);
    gpio_set_direction(TF_POWER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(TF_POWER_PIN, 0); 
}

void displayui() {
	    epd_power_on();
	    vTaskDelay(pdMS_TO_TICKS(50));
	    clear_all();
	    vTaskDelay(pdMS_TO_TICKS(10));
	
        Paint_NewImage(Image_BW, EPD_W, EPD_H, 0, WHITE);
        EPD_Full(WHITE);
        EPD_Display_Part(0, 0, EPD_W, EPD_H, Image_BW);
        EPD_Init_Fast(Fast_Seconds_1_5s);
             
        EPD_Display_Fast(loaded_Image); // Image Binary data
        vTaskDelay(pdMS_TO_TICKS(100));
        epd_power_off();
  }
  