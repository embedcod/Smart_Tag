/***************************************************************************************************************************************
∎ BRIEF:
		The Aga Khan HMI Solution is an HMI Project that monitors Operating Room differential pressure 
		in all the 9 Operating theaters at THE AGA KHAN MAIN HOSPITAL.
∎ FIRMWARE: 
	⟹ The project Firmware is structured as a State Machine with 10 unique states. 
	⟹ The Differential Pressure Readings are pulled from the Aranet platform via an API.
	⟹ These readings are then displayed on an E-paper display with their corresponding last updates.
	⟹ The PWM Buzzer is used to sound alarms pertaining to Room Pressure and Battery Capacity.

∎ GUIDE: 
	⇛ The device has only one button that is configured to be Reset, Wake-up and Change page button.
	⇛ This has been achieved by Interrupt Service Routine protocol (ISR:)
	⇛ To wake up the device, short press the button for a second.
	⇛ To change the device page, short press the button for a second while it is awake.
	⇛ To Reset the device, press the button twice [in interval of 200ms]
	⇛ To set WiFi credentials type ssid: wifiname and press enter, and then type pass: wifipassword and press enter. 

∎ FEATURES: 
	→ Multi-purpose button for device control.
	→ Buzzer and Voltage Monitor
	→ Extremely low-power with a 0.00 deep sleep current draw. 
	→ HTTPS over TLS for WiFi Data GET, SMTP and SNTP
	→ Realtime sync with servers for accurate real time clock (SNTP).

∎ SOFTWARE: Engineer Eric Mulwa
∎ HARDWARE: Engineer Eric Mulwa 
	 ≈ EMAIL: mulwaericbsc@gmail.com
	 ≈ PHONE: 0796456877
	 ≈ DATE:  09/04/2025
	 ≈ UPDATED: 09/04/2025
	  
	 ⇉ OWNERSHIP: Geviton Enterprises Limited
	 ⇉ LICENSE:   Proprietary - All rights reserved by Geviton Enterprises Limited.

***************************************************************************************************************************************/

#include "EPD.h"
#include "nvs.h"
#include "time.h"
#include <math.h>
#include "cJSON.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "EPD_GUI.h"
#include "esp_log.h"
#include "EPD_SPI.h"
#include "esp_mac.h"
#include "esp_err.h"
#include "esp_sntp.h"
#include "esp_wifi.h"
#include "esp_attr.h"
#include "max17055.h"
#include <sys/time.h>
#include "wifi_ota.h"
#include "esp_event.h"
#include <esp_sleep.h>
#include "nvs_flash.h"
#include "esp_netif.h"
#include <sys/param.h>
#include "driver/i2c.h"
#include "esp_spiffs.h"
#include "mbedtls/ssl.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_vfs_fat.h"
#include <driver/ledc.h>
#include "pic_scenario.h"
#include "lwip/ip_addr.h"
#include "mbedtls/error.h"
#include "freertos/task.h"
#include "esp_netif_sntp.h"
#include <mbedtls/base64.h>
#include "mbedtls/entropy.h"
#include "esp_http_client.h"
#include "freertos/timers.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/platform.h"
#include "freertos/FreeRTOS.h"
#include "mbedtls/esp_debug.h"
#include "mbedtls/net_sockets.h"
#include <esp_adc/adc_oneshot.h>
#include "freertos/event_groups.h"

/***************************************************************************************************************************************
ENUMERATION: STATE MACHINE STATES
********************************/
typedef enum {
    STATE_POWER_ON,
    STATE_INITIALIZE,
    STATE_IDLE,
    STATE_PAGE_CONFIG,
    STATE_WiFi_CONNECT,
    STATE_OTA,
    STATE_SNTP,
    STATE_SMTP,
    STATE_UPDATE_EPD,
    STATE_WiFi_DISCONNECT,
    STATE_RESET,   
} system_state_t;
system_state_t  currentState = STATE_POWER_ON;

/*******************************************************************************************************************************
DEFINITION: DATA PARAMETERS
***************************/
RTC_DATA_ATTR float Pressure = 0.0;
RTC_DATA_ATTR float Temperature = 0.0;
RTC_DATA_ATTR float Humidity = 0.0;
RTC_DATA_ATTR char Time[20] = "00/00 00:00";
RTC_DATA_ATTR char Sensor[50] = "";
RTC_DATA_ATTR char Sensor_ID[32] = "";
RTC_DATA_ATTR int batteryCapacity = 0;
RTC_DATA_ATTR float Voltage = 0;
RTC_DATA_ATTR float Soc = 0;
uint32_t akuhId = 0;
int Pcb_Version = 1;
char deviceSerial[12];
int32_t target_number = 1;
char Access_token[50];

RTC_DATA_ATTR battery_data_t batt;
/*******************************************************************************************************************************/

/*******************************************************************************************************************************/
// Mcu Pin definitions
#define BUTTON_PIN 2
#define EPD_POWER_PIN 7
#define BUZZER_PIN 14
#define SENSE_PIN 16
#define POWER_PIN 3
#define LED_PIN 41
/*******************************************************************************************************************************/

/*******************************************************************************************************************************/
// Button Definitions
#define DEBOUNCE_TIME_MS 500      
#define MULTI_PRESS_DELAY_MS 3000  
#define MIN_PRESS_INTERVAL_MS 500 
/*******************************************************************************************************************************/

/********************************************************************************************************************************
GLOBAL: Scalling Varibles & Control Definitions
**********************************************/
#define MAX_NUMBER 15 								// Number of devices to cycle through
#define OTA_INTERVAL 3600 							// check for ota update after every 1 hour(s)
#define SNTP_MAX_RETRIES      10 					// Retry 10 times to get time from servers
#define SMTP_INTERVAL 604800 						// Send error logs once every 7 days
#define API_KEY "g58b5xesscqkh8ngzkthhy8agdecutca"  // Aranet API Key
#define DEEP_SLEEP_DURATION_US (10 * 60 * 1000000) 	// Deep sleep duration: 10 minutes
/*******************************************************************************************************************************/

/********************************************************************************************************************************
GLOBAL: Global Definitions
**************************/
volatile int buttonPressCount = 0;  
volatile bool interruptFlag = false;
volatile bool stopCurrentProcess = false;
TimerHandle_t pressTimer;      
static uint32_t storedChipId = 0;
static uint32_t storedAkuhId = 0;
RTC_DATA_ATTR int PowerOnTone = 0;
// Aranet Platform
static uint8_t Image_BW[15000]; // TODO
bool networkError = false;
bool apiError = false;
static int reconnect_attempts = 0;
RTC_DATA_ATTR int battlow = 0;
// WiFi
bool wifi_connected = false;
#define MAX_RECONNECT_ATTEMPTS 6
// OTA
bool ota_flag = false;
RTC_DATA_ATTR time_t lastOtaTime = 0; 
// SNTP
#define SNTP_RETRY_DELAY_MS   2000
#define TIMEZONE "EAT-3"
RTC_DATA_ATTR int firstBoot = 0;
struct tm timeinfo;

// SMTP
const char *path = "/spiffs/elogs.txt";
#define SERVER_USES_STARTSSL 1
int Alarmtype = 0;
int battery_low_ui = 0;

// TLS Root Certificate 
extern const uint8_t server_root_cert_pem_start[] asm("_binary_server_root_cert_pem_start");
extern const uint8_t server_root_cert_pem_end[]   asm("_binary_server_root_cert_pem_end");

// Utility Macro 
#define VALIDATE_MBEDTLS_RETURN(ret, min_valid_ret, max_valid_ret, goto_label)  \
    do {                                                                        \
        if (ret < min_valid_ret || ret > max_valid_ret) {                       \
            goto goto_label;                                                    \
        }                                                                       \
    } while (0) 
    
// Voltage Monitor
#define MONITOR_INTERVAL 3600 // Once every hour
RTC_DATA_ATTR time_t lastMonitorTime = 0; 
/*******************************************************************************************************************************/

/*******************************************************************************************************************************
KCONFIG: 
******************/
// WiFi
char WIFI_SSID[32] = "";
char WIFI_PASS[64] = "";
// SMTP
#define MAIL_SERVER         "smtp.gmail.com"
#define MAIL_PORT "587" 
#define SENDER_MAIL         "info@geviton.co.ke"
#define SENDER_PASSWORD     "wshj owac tlsj xpcu"
#define RECIPIENT_MAIL      "ericmulwa@geviton.co.ke"
// Array of recipients (NULL terminated)
const char* recipients[] = {
    //"aku-test@geviton.co.ke",
    "aku@geviton.co.ke",
    NULL // Null-terminated array
};
// smtp mail flags
int smtp_press_batt_warn_flag = 0;
int smtp_error_logs_flag = 0;

// SNTP
#define SNTP_SERVER_0 "ke.pool.ntp.org"  	 // Kenya-specific pool
#define SNTP_SERVER_1 "africa.pool.ntp.org"  // Fallback for Africa
#define SNTP_SERVER_2 "pool.ntp.org"      	 // Global fallback
//#define SNTP_SERVER_X "time.google.com" 
/******************************************************************************************************************************/


/*******************************************************************************************************************************/
// Application TAGs
static const char *TAG = "AKUH_HMI_SYSTEM";
/*******************************************************************************************************************************/

/*******************************************************************************************************************************
WATCHDOG CONSTANTS
******************/
#define WATCHDOG_TIMEOUT_MS 180000  // 3 minute(s)
TimerHandle_t watchdogTimer; 
/******************************************************************************************************************************/

/*******************************************************************************************************************************
ENUMERATION: MUSICAL TONES
********************************/
// Frequency for BeeHappy melody notes
#define NOTE_B4  2475
#define NOTE_D5  2937
#define NOTE_F5  3490
#define NOTE_B5  4950

typedef struct {
    int frequency;
    int duration;
} Note;
/*******************************************************************************************************************************/

/*******************************************************************************************************************************
INIT: Board Type Detection
********************************/
typedef enum {
    BOARD_V1_VOLTAGE_DIVIDER = 0,
    BOARD_V2_MAX17055_FG
} board_type_t;

typedef struct {
    board_type_t type;
    bool fuel_gauge_present;
} board_info_t;

static board_info_t g_board;
/******************************************************************************************************************************/


/*******************************************************************************************************************************
INIT: VOLTAGE MONITOR
********************************/
// Battery Voltage Monitor Variables and Thresholds
const float MIN_VOLTAGE = 3.2; // Min of 2.5V for the RY3420 ADH5Q 3V3
const float MAX_VOLTAGE = 4.19;                                  
const gpio_num_t analogPin = GPIO_NUM_8;      
const float R1 = 2700.0;   
const float R2 = 10000.0; 
const float Vref = 3.3;      
int batteryLevel = 0; 
adc_oneshot_unit_handle_t adc_handle;  
/******************************************************************************************************************************/

/*******************************************************************************************************************************
DEFINITIONS: I2C INITIALIZATIONS 
********************************/
// I2C CONFIG
#define I2C_PORT                   I2C_NUM_0
#define I2C_SCL_GPIO               9
#define I2C_SDA_GPIO               8
#define I2C_FREQ_HZ      		   100000
#define I2C_MASTER_TIMEOUT_MS      1000 
#define MAX17055_ADDR              0x36u
static max17055_learned_params_t g_learned = {0};
static bool                      g_learned_valid = false;
static uint16_t                  g_cycles_last_saved = 0;
static max17055_ctx_t fg;

const float MIN_SOC = 10; // 10%
const float MAX_SOC = 100; // 100%
/*******************************************************************************************************************************/


/*******************************************************************************************************************************
INIT: PWM BUZZER
********************************/
// PWM Configuration Constants
#define LEDC_TIMER            LEDC_TIMER_0
#define LEDC_MODE             LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_BUZZER   LEDC_CHANNEL_0
#define LEDC_CHANNEL_LED      LEDC_CHANNEL_1
#define LEDC_DUTY_RES         LEDC_TIMER_13_BIT 
#define LEDC_FREQUENCY        3000    

// Duty Cycle Constants for Buzzer
#define DUTY_5_PERCENT        (410)    
#define DUTY_10_PERCENT       (819)
#define DUTY_50_PERCENT       (4095)
#define DUTY_25_PERCENT       (2047) 
#define DUTY_20_PERCENT       (1638) 
#define DUTY_80_PERCENT       (6553) 
#define DUTY_100_PERCENT      (8191)
#define FADE_STEP             (50)
#define FADE_DELAY_MS         (10) 
/*******************************************************************************************************************************/

/*******************************************************************************************************************************
DEFINITIONS: UART INITIALIZATIONS
********************************/
// UART0 Initialization
#define UART_NUM       UART_NUM_0  
#define BUF_SIZE        1024

/***************************************************************************************************************************************
FUNCTIONS: PROTOTYPES
**********************/
void setupWatchdog();
void resetWatchdog();
void watchdogCallback(TimerHandle_t xTimer);
void setupButton();
void initializeTimer();
void buttonPressHandler();
void pressTimerCallback(TimerHandle_t xTimer);
void handleButtonPress();
void powerOnSequence();
void initializeSystem();
void Pages(int count);
void connectWiFi();
void SNTP();
void SMTP();
void UpdateEPD();
void DisconnectWiFi();
void resetDevice();
void shutdownDevice();
void powerdown();
// Asterisk Separator
void printAsteriskSeparator(int length);
// VOLTAGE MONITOR FUNCTIONS
void setup_pins();
void readBatteryVoltage();
void updateBatteryLevel();
void read_batt_params();
// MAX17055 Fuel Gauge
static esp_err_t i2c_master_init(void);
static esp_err_t i2c_master_deinit(void);
static void detect_board_variant(void);
void board_init(void);
static esp_err_t battery_init(void);
static esp_err_t load_learned_params_from_nvs(void);
static esp_err_t save_learned_params_to_nvs(void);
static int32_t platform_i2c_read(void *handle, uint8_t reg, uint8_t *data, uint16_t len);
static int32_t platform_i2c_write(void *handle, uint8_t reg, uint8_t *data, uint16_t len);
void max17055_save_learned(void);
void read_batt_params(void);
float battery_get_soc(void);
float battery_get_internal_resistance_mohm(void);
float battery_get_remaining_capacity_mah(void);
float battery_get_original_capacity_mah(void);
float battery_get_cycles(void);
float battery_get_temperature_c(void);
float battery_get_current_ma(void);
float battery_get_soh(void);
float battery_get_voltage_mv(void);
// PWM BUZZER & LED FUNCTIONS
void setupPWMForBuzzer();
void setupPWMForLED();
void blink_green_yellow(int blink_count);
void soundBuzzeronce(int duration_ms);
void soundBuzzertwo(int duration_ms, int delay_between_ms);
void soundBuzzerthrice(int duration_ms, int delay_between_ms);
void soundBuzzerfour(int duration_ms, int delay_between_ms);
void faultbuzzer(int duration_ms, int delay_between_ms);
void playPowerOnTone(int ledc_channel, int ledc_timer);
// CHIP MAC {Serial Number}
uint32_t getChipId();
void getChipIdString(char *deviceSerial, size_t size);
uint32_t getAkuhId();
// Aranet dynamic GET URL assignment using ESP serial numbers
static const char *deviceUrls[];
static const uint32_t deviceAkuhNumbers[];
const char *getDeviceUrl(uint32_t akuhId);
const char *getHttpPostHeader(uint32_t chipId);
// WiFi Functions
void nvs_init();
static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void wifi_connection();
void wifi_disconnect();
// OTA
void OTA();
// HTTPS CLIENT OVER SSL FUNCTIONS
void load_wifi_ssid_from_nvs();
void load_wifi_pass_from_nvs();
void save_wifi_ssid_to_nvs(const char *ssid);
void save_wifi_pass_to_nvs(const char *ssid);
void init_nvs_button_uart();
void get_aranet_data(void);
void checkpressurethreshold();
void checktempthreshold();
char* convert_utc_to_eat(const char *utc_time);
esp_err_t client_event_get_handler(esp_http_client_event_handle_t evt);
// SMTP Functions
static int write_and_get_response(mbedtls_net_context *sock_fd, unsigned char *buf, size_t len);
static int write_ssl_and_get_response(mbedtls_ssl_context *ssl, unsigned char *buf, size_t len);
static int write_ssl_data(mbedtls_ssl_context *ssl, unsigned char *buf, size_t len);
static int perform_tls_handshake(mbedtls_ssl_context *ssl);
void smtp_client_task();
void save_last_smtp_time_to_nvs(time_t lastSmtpTime);
time_t load_last_smtp_time_from_nvs();
// SNTP
bool sync_time_from_sntp();
void print_current_time();
// SPIFFS
void initialize_spiffs(void);
void log_error(const char *format, ...);
void log_data(const char *path, char *content);
char* read_file(const char *path);
bool clear_file(const char *path);
// E-PAPER DISPLAY
void displayui(void);
void clear_all(void);
void EPD_display_Battery_Low(void);
void EPD_Partial_Data_Update(void);
void epd_power_on();
void epd_power_off();
void update_display();
void CheckBatteryLevel();
void deepsleep();
// Pages
int load_target_number_from_nvs();
void save_target_number_to_nvs(int number);
void handle_next_target_selection();
void handle_previous_target_selection();
/*******************************************************************************************************************************/
// serial uart
void serial_uart_init();

// Tasks
void uart_wifi_task(void *arg);
void baremetal_task(void *param);
void interrupt_task(void *param);
/*******************************************************************************************************************************/


/*******************************************************************************************************************************
FUNCTION: BUTTON INTERRUPT HANDLER
*********************************/
static void IRAM_ATTR button_isr_handler(void* arg) {
    static unsigned long lastInterruptTime = 0;
    static unsigned long lastPressTime = 0;
    unsigned long interruptTime = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;

    if (interruptTime - lastInterruptTime > DEBOUNCE_TIME_MS) {
        if (interruptTime - lastPressTime > MIN_PRESS_INTERVAL_MS) {
            buttonPressCount++;
            if (pressTimer != NULL) {   // Guard against NULL
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                xTimerResetFromISR(pressTimer, &xHigherPriorityTaskWoken);
                if (xHigherPriorityTaskWoken) {
                    portYIELD_FROM_ISR();
                }
            }
            lastPressTime = interruptTime;
        }
    }
    lastInterruptTime = interruptTime;
}

void handleButtonPress() { 
    stopCurrentProcess = true; 
        if (interruptFlag) {
            switch (buttonPressCount) {
                case 1:
                    resetDevice();
                    break;
                case 2:
                     Pages(buttonPressCount); // FORWARD // Disabled due to devices interference at AKUH.  TODO
                    break;
                case 3:
                     Pages(buttonPressCount); // BACK // Disabled due to devices interference at AKUH.  shutdownDevice();
                    break;
                default:
                    break;
            }
            interruptFlag = false;  
            buttonPressCount = 0;  
        }
	}
		
/*******************************************************************************************************************************/

/*******************************************************************************************************************************
FUNCTION: THE MAIN APPLICATION
*****************************/
void app_main() {
 powerOnSequence();
 initializeSystem();
 resetWatchdog();
 
 xTaskCreate(uart_wifi_task, "uart_wifi_task", 4096, NULL, 1, NULL);
 xTaskCreate(interrupt_task, "interrupt_task", 2560, NULL, 3, NULL);
 xTaskCreatePinnedToCore(baremetal_task, "baremetal_task", 9216, NULL, 2, NULL, 1); // Pinned for predictable behavior

}

/*******************************************************************************************************************************
RTOS: TASKS
**************************/
/**
* Interrupt Task
*/
void interrupt_task(void *param) { 
	while (1) {
    // Check Button Press Interrupt flag
    if (interruptFlag) {
        handleButtonPress();
    }
    vTaskDelay(pdMS_TO_TICKS(50)); 
  }
}

/*******************************************************************************************************************************
CORE: CORE 0 FUNCTIONS
**************************/

// UART Wi-Fi Credentials Task (Core 0)
void uart_wifi_task(void *arg) {

    char buffer[BUF_SIZE] = {0};  
    int index = 0;

    while (1) {
    uint8_t ch;
        if (uart_read_bytes(UART_NUM, &ch, 1, pdMS_TO_TICKS(100)) > 0) {
            if (ch == '\n' || ch == '\r') {  // Enter key pressed
                if (index > 0) {
                    buffer[index] = '\0';  // Null-terminate input

                    if (strncmp(buffer, "ssid: ", 6) == 0) {
                        save_wifi_ssid_to_nvs(buffer + 6); // Save SSID
                        printf("New WiFi SSID Set: %s\n\n", buffer + 6);
                    } 
                    else if (strncmp(buffer, "pass: ", 6) == 0) {
                        save_wifi_pass_to_nvs(buffer + 6); // Save Password
                        printf("New WiFi Password Set: %s\n\n", buffer + 6);
                    } 
                    else {
                        printf("Invalid command. Use 'ssid:' or 'pass:'");
                    }

                    index = 0;  // Reset index
                    memset(buffer, 0, BUF_SIZE);  // Clear buffer
                }
            } 
            else if (index < BUF_SIZE - 1) {
                buffer[index++] = ch;  
            }
        }
       vTaskDelay(pdMS_TO_TICKS(50)); 	
    } 
} 

/**
* State Machine Task
*/
void baremetal_task(void *param) { 
	
	while (1) {
	    // State machine logic	    
	    switch (currentState) {
	        case STATE_IDLE:
                  resetWatchdog();
                  printf("HMI System entered idle mode...\n");

                  if (firstBoot == 0) {
                  printf("First Boot, you have 10s to change the WiFi Credentials!.\n"); //
                  vTaskDelay(pdMS_TO_TICKS(1000));   
                    
                  } else {
                  vTaskDelay(pdMS_TO_TICKS(50));
                  }
                  currentState = STATE_WiFi_CONNECT;	            		                
	            break;
	        case STATE_WiFi_CONNECT:
	                 resetWatchdog();
	                // connectWiFi();
	                vTaskDelay(pdMS_TO_TICKS(2000)); // small delay for wifi to stabilize	                
					currentState = STATE_SNTP;	                
	            break;
	        case STATE_SNTP:
                  resetWatchdog();
                  // if (firstBoot == 0) {
                  //   SNTP();
                  // } else {
                  // print_current_time();	
                  // }
                  vTaskDelay(pdMS_TO_TICKS(50));	
                  currentState = STATE_OTA; 	                
	            break;
	        case STATE_OTA:
	                resetWatchdog();
	               	// OTA(); // Perform ota
	                vTaskDelay(pdMS_TO_TICKS(50));	
					currentState = STATE_UPDATE_EPD;               						                
	            break;
	        case STATE_UPDATE_EPD:
		              resetWatchdog();
		              vTaskDelay(pdMS_TO_TICKS(50));
	                  UpdateEPD();
					  vTaskDelay(pdMS_TO_TICKS(50));	
					  currentState = STATE_SMTP; 
	            break;
	        case STATE_SMTP:
                  resetWatchdog(); 	
                  // time_t lastSmtpTime = load_last_smtp_time_from_nvs();  // Load from NVS at startup 
                  // //lastSmtpTime = Unix timestamp (seconds since 1970) 
                  // time_t now;
                  // time(&now);
                  // if (firstBoot == 0) {			    
                  // if (lastSmtpTime == 0) {  // Only set to current time if it wasn't loaded from NVS
                  // lastSmtpTime = now;
                  // save_last_smtp_time_to_nvs(lastSmtpTime);  // Save initial value
                  // }
                  // vTaskDelay(pdMS_TO_TICKS(50));	
                  // }    
                  // printf("Last SMTP: %s", ctime(&lastSmtpTime));	
                  // vTaskDelay(pdMS_TO_TICKS(50));					
                  // // Send the weekly error logs							             
                  // if ((now - lastSmtpTime) >= SMTP_INTERVAL) {
                  // SMTP();
                  // lastSmtpTime = now;
                  // save_last_smtp_time_to_nvs(lastSmtpTime);  // Save updated time                              
                  // }
                  vTaskDelay(pdMS_TO_TICKS(50));
                  currentState = STATE_WiFi_DISCONNECT;	                
	            break;
	        case STATE_WiFi_DISCONNECT:
                  resetWatchdog();
                  // Check for temperature sensors               
                  // if (target_number == 14 || target_number == 15) {
                  // checktempthreshold();
                  // } else {
                  // checkpressurethreshold();		

                  // }	       
                  // vTaskDelay(pdMS_TO_TICKS(50));              
                  // CheckBatteryLevel();
                  // vTaskDelay(pdMS_TO_TICKS(50));	                
                  // DisconnectWiFi();	
                  vTaskDelay(pdMS_TO_TICKS(50));              
                  deepsleep();
	            break;
	        default:
	                resetWatchdog();
	                currentState = STATE_IDLE;
	            break;
	    }
		vTaskDelay(pdMS_TO_TICKS(50));
	}
    vTaskDelay(pdMS_TO_TICKS(50));	
}

/************************************************************************************************************************************** 
FUNCTIONS: INITIALIZATION
************************/

 /**
 * @brief Timer callback for processing button presses
 */
void pressTimerCallback(TimerHandle_t xTimer) {
    interruptFlag = true;               // Set flag to process button press in main loop
}

/**
 * @brief Initialize Button with Interrupt
 */
void setupButton() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;   
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BUTTON_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ONLY;
    gpio_config(&io_conf);
    // Install ISR service and add button interrupt
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, NULL);
}

/**
 * @brief Initialize Timer for handling multi-press delay
 */
void initializeTimer() {
    pressTimer = xTimerCreate("PressTimer", MULTI_PRESS_DELAY_MS / portTICK_PERIOD_MS, pdFALSE, NULL, pressTimerCallback);
}

/**
 * @brief Get chip ID based on the MAC address
 */
uint32_t getChipId() {
    if (storedChipId == 0) {
        uint8_t mac[6];
        esp_base_mac_addr_get(mac); 

        for (int i = 0; i < 6; i++) {
            storedChipId |= ((uint32_t)mac[i] << (8 * i));
        }
    }
    return storedChipId;
}

uint32_t getAkuhId() { //
    if (storedAkuhId == 0) {
        switch (target_number) {
            case 1:storedAkuhId = 1; // OR1
                break;
            case 2:storedAkuhId = 2; // OR2
                break;
            case 3:storedAkuhId = 3; // OR3
                break;
            case 4:storedAkuhId = 4; // OR4
                break;
            case 5:storedAkuhId = 5; // OR5
                break;
            case 6:storedAkuhId = 6; // OR6
                break;
            case 7:storedAkuhId = 7; // OR7
                break;
            case 8:storedAkuhId = 8; // OR8
                break;
            case 9:storedAkuhId = 9; // OR9
                break;
            case 10:storedAkuhId = 10; // CSSD DAWC
                break;
            case 11:storedAkuhId = 11; // CSSD DIFF
                break;
            case 12:storedAkuhId = 12; // CSSD PAWD
                break;
            case 13:storedAkuhId = 13; // CSSD BUFF
                break; 
            case 14:storedAkuhId = 14; // MATERN PHAR
                break;    
            case 15:storedAkuhId = 15; // WH FRIDGE
                break;                               
            default:
                ESP_LOGW("AKUH_ID", "Invalid target number, defaulting to OR1");
                storedAkuhId = 1;
                break;
        }
    }
    return storedAkuhId;
}

/**
 * @brief Get the chip ID as a string
 */
void getChipIdString(char *deviceSerial, size_t size) {
    uint32_t chipId = getChipId();
    snprintf(deviceSerial, size, "%" PRIu32, chipId);  
    printf("Device Serial Number (Chip ID): %s\n", deviceSerial);   
}

/*-----------------------------------------------------------------------------------------*/
// Assign Aranet GET URLs based on the ESP Chip Serial Number
/*-----------------------------------------------------------------------------------------*/
/**
 * @brief Array to store device-specific URLs
 */
static const char *deviceUrls[] = { // Calibrated sensors
	"https://aranet.cloud/api/v1/measurements/last?sensor=134219589&links=false", // OR1 ID: 00745
	"https://aranet.cloud/api/v1/measurements/last?sensor=134219590&links=false", // OR2 ID: 00746
	"https://aranet.cloud/api/v1/measurements/last?sensor=134219591&links=false", // OR3 ID: 00747
	"https://aranet.cloud/api/v1/measurements/last?sensor=134219592&links=false", // OR4 ID: 00748
	"https://aranet.cloud/api/v1/measurements/last?sensor=134219593&links=false", // OR5 ID: 00749
	"https://aranet.cloud/api/v1/measurements/last?sensor=134219595&links=false", // OR6 ID: 0074B
	"https://aranet.cloud/api/v1/measurements/last?sensor=134219596&links=false", // OR7 ID: 0074C
	"https://aranet.cloud/api/v1/measurements/last?sensor=134219597&links=false", // OR8 ID: 0074D
	"https://aranet.cloud/api/v1/measurements/last?sensor=134219594&links=false", // MATERNITY ID: 0074A
	// Added 3
	"https://aranet.cloud/api/v1/measurements/last?sensor=134219686&links=false", // CSSD DAWC ID: 007A6
	"https://aranet.cloud/api/v1/measurements/last?sensor=134219571&links=false", // CSSD DIFF ID: 00733
	"https://aranet.cloud/api/v1/measurements/last?sensor=134219687&links=false", // CSSD PAWD ID: 007A7
	// Added 1 (27/04/2026)
	"https://aranet.cloud/api/v1/measurements/last?sensor=134220174&links=false", // CSSD BUFF ID: 0098E
	// Temperature sensor (10/05/2026)
	"https://aranet.cloud/api/v1/measurements/last?sensor=2111156&links=false",   // MATERN PHAR ID: 036B4
	// Temperature sensor (debug device)
	"https://aranet.cloud/api/v1/measurements/last?sensor=2110938&links=false",   // WH FRIDGE ID: 035DA
};

/**
 * @brief Array of corresponding HMI Devices Akuh numbers
 */ 
static const uint32_t deviceAkuhNumbers[] = {
	1, 	// device OR1
	2, 	// device OR2
	3, 	// device OR3
	4, 	// device OR4
	5, 	// device OR5
	6, 	// device OR6
	7, 	// device OR7
	8, 	// device OR8
	9, 	// device Maternity
	10, // device CSSD DAWC
	11, // device CSSD DIFF
	12, // device CSSD PAWD
	13, // device CSSD BUFF
	14, // device MATERN PHAR
	15, // device WH FRIDGE
};

/**
 * @brief Match Akuh Id with the right url
 */
const char *getDeviceUrl(uint32_t akuhId) {
    for (size_t i = 0; i < sizeof(deviceAkuhNumbers) / sizeof(deviceAkuhNumbers[0]); i++) {
        ESP_LOGI(TAG, "Checking index %d: akuhId=%" PRIu32 ", storedId=%" PRIu32, i, akuhId, deviceAkuhNumbers[i]);
        if (deviceAkuhNumbers[i] == akuhId) {
            ESP_LOGI(TAG, "Match found at index %d for akuhId=%" PRIu32 ". URL=%s", i, akuhId, deviceUrls[i]);
            return deviceUrls[i];
        }
    }
    ESP_LOGW(TAG, "No match found for akuhId=%" PRIu32 ". Using default URL.", akuhId);
    log_error("No match found for akuhId=%" PRIu32 ". Using default URL.", akuhId);
    return "https://aranet.cloud/api/v1/measurements/last?sensor=134219589&links=false";

}
/*******************************************************************************************************************************/


/*******************************************************************************************************************************
FUNCTION: WATCHDOG SETUP
************************/
void setupWatchdog() {
    watchdogTimer = xTimerCreate("Watchdog Timer", pdMS_TO_TICKS(WATCHDOG_TIMEOUT_MS), pdFALSE, NULL, watchdogCallback);
    if (watchdogTimer == NULL) {
        printf("Failed to create watchdog timer\n");
    }
}

void resetWatchdog() {
    if (xTimerReset(watchdogTimer, 0) != pdPASS) {
        printf("Failed to reset watchdog timer\n");
    }
}

void watchdogCallback(TimerHandle_t xTimer) {
    printf("Watchdog timer expired! Resetting device...\n");
    esp_restart();
}


/**
 * @brief Print asterisk separator in debug messages
 */
void printAsteriskSeparator(int length) {
    char separator[length + 1]; 
    for (int i = 0; i < length; i++) {
        separator[i] = '*'; 
    }
    separator[length] = '\0'; 
    printf("\n");
    printf("%s", separator);
    printf("\n");
}

/**
 * @brief Power Off
 */
void powerdown() {
    esp_rom_gpio_pad_select_gpio(POWER_PIN);
	gpio_set_direction(POWER_PIN, GPIO_MODE_OUTPUT);
	gpio_set_level(POWER_PIN, 1);
	vTaskDelay(pdMS_TO_TICKS(1000));
}
/*******************************************************************************************************************************/


/*******************************************************************************************************************************
FUNCTIONS:
********************************/

// VOLTAGE MONITOR FUNCTIONS

/**
 * @brief Initialize GPIOs and ADC
 */
void setup_pins() {
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1
    };
    adc_oneshot_new_unit(&unit_cfg, &adc_handle);

    adc_oneshot_chan_cfg_t channel_cfg = {
        .atten = ADC_ATTEN_DB_12,                             
        .bitwidth = ADC_BITWIDTH_12             
    };
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_7, &channel_cfg); 
}

/**
 * @brief Read the battery voltage
 */ 
// Read the battery voltage
void readBatteryVoltage() {
	esp_rom_gpio_pad_select_gpio(SENSE_PIN);                       
    gpio_set_direction(SENSE_PIN, GPIO_MODE_OUTPUT);   
    gpio_set_level(SENSE_PIN, 1);
	vTaskDelay(pdMS_TO_TICKS(1000));  

    // Take 10 readings and average them
    float sum = 0;
    int readings = 10;
    
    for (int i = 0; i < readings; i++) {
        int adcValue;
        adc_oneshot_read(adc_handle, ADC_CHANNEL_7, &adcValue);      
        float singleReading = (adcValue / 4095.0) * Vref;         
        singleReading = singleReading * ((R1 + R2) / R2);
        sum += singleReading;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    gpio_set_level(SENSE_PIN, 0);
    Voltage = (sum / readings); 
    
    printf("Average Voltage (10 samples): %.2f V\n", Voltage);
    log_error("Average Voltage (10 samples): %.2f V\n", Voltage);
    return;
}

// Update battery level based on voltage
void updateBatteryLevel() {
  if (g_board.type == BOARD_V2_MAX17055_FG) { 
		// Read Battery Level in intervals
		time_t now;
		time(&now);
		if (firstBoot == 0) {
		    lastMonitorTime = now;
		    vTaskDelay(pdMS_TO_TICKS(50));
		    read_batt_params();
		}	
		printf("Last Monitor: %s", ctime(&lastMonitorTime));
		vTaskDelay(pdMS_TO_TICKS(50));	              
		if ((now - lastMonitorTime) >= MONITOR_INTERVAL) {
		    read_batt_params();
		    lastMonitorTime = now;                             
		}
		
		float batteryLevel;
	    if (Soc <= MIN_SOC) {
	        batteryLevel = 0;
	    } else if (Soc >= MAX_SOC) {    
	        batteryLevel = 100;
	    } else {
	        batteryLevel = Soc;
	    }
	    batteryCapacity = batteryLevel;
	    printf("Battery Capacity: %f%%\n", batteryLevel);  
  } else {
		// Read Battery Level in intervals
		time_t now;
		time(&now);
		if (firstBoot == 0) {
		    lastMonitorTime = now;
		    vTaskDelay(pdMS_TO_TICKS(50));
		    readBatteryVoltage();
		}	
		printf("Last Monitor: %s", ctime(&lastMonitorTime));
		vTaskDelay(pdMS_TO_TICKS(50));	              
		if ((now - lastMonitorTime) >= MONITOR_INTERVAL) {
		    readBatteryVoltage();
		    lastMonitorTime = now;                             
		}
		
	    if (Voltage <= MIN_VOLTAGE) {
	        batteryLevel = 0;
	    } else if (Voltage >= MAX_VOLTAGE) {
	        batteryLevel = 100;
	    } else {
	        float voltageRange = MAX_VOLTAGE - MIN_VOLTAGE;
	        float percentagePerVolt = 100.0 / voltageRange;
	        batteryLevel = ((Voltage - MIN_VOLTAGE) * percentagePerVolt) / 5 * 5;
	    }
	    batteryCapacity = batteryLevel;
	    printf("Battery Capacity: %d%%\n", batteryLevel);	  
  }
}

/**
 * @brief I2C READ/WRITE FUNCTIONS FOR MAX17055 LIBRARY
 */ 
// Read the battery soc
void read_batt_params() {
    batt.soc       = battery_get_soc();
    batt.voltage   = battery_get_voltage_mv();
    batt.soh       = battery_get_soh();
    batt.temp_c    = battery_get_temperature_c();
    batt.cycles    = battery_get_cycles();
    batt.rint_mohm = battery_get_internal_resistance_mohm();
    batt.cap_mah   = battery_get_remaining_capacity_mah();
    batt.full_mah  = battery_get_original_capacity_mah();
    batt.curr_ma   = battery_get_current_ma();

    ESP_LOGI(TAG, "SOC          : %.1f %%", batt.soc);
    ESP_LOGI(TAG, "Voltage      : %.0f mV", batt.voltage);
    ESP_LOGI(TAG, "Current      : %+.0f mA", batt.curr_ma);
    ESP_LOGI(TAG, "SOH          : %.1f %%", batt.soh);
    ESP_LOGI(TAG, "Temperature  : %.1f oC", batt.temp_c);
    ESP_LOGI(TAG, "Cycles       : %.2f",    batt.cycles);
    ESP_LOGI(TAG, "Internal R   : %.1f mR", batt.rint_mohm);
    ESP_LOGI(TAG, "Remaining    : %.0f / %.0f mAh", batt.cap_mah, batt.full_mah);

    // Take 3 readings and average them
    float sum = 0;
    int readings = 3;
    for (int i = 0; i < readings; i++) {    
        float singleReading = battery_get_soc();      
        sum += singleReading;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
	vTaskDelay(pdMS_TO_TICKS(50));
    Soc = (sum / readings); 
    
    if (Soc <= 0) {
        Soc = 0;
    } else if (Soc >= 100) {    
        Soc = 100;
    } else {
        Soc = Soc;
    }
    
    printf("Average Soc (3 samples): %.2f %%\n", Soc);
    log_error("Average Soc (3 samples): %.2f %%\n", Soc); 
    return;
}

// ---------------------------------------------------------------------
// I2C READ/WRITE FUNCTIONS FOR MAX17055 LIBRARY
// ---------------------------------------------------------------------
static esp_err_t i2c_master_init(void)
{
	printf("Initializing the I2C master...\n");
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_SDA_GPIO,
        .scl_io_num       = I2C_SCL_GPIO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &cfg));
    return i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

static esp_err_t i2c_master_deinit(void)
{
    printf("Deinitializing the I2C master...\n");
    esp_err_t err = i2c_driver_delete(I2C_PORT);
    if (err == ESP_OK)
    {
        printf("I2C driver deleted successfully\n");
    }
    else
    {
        printf("Failed to delete I2C driver: %s\n", esp_err_to_name(err));
    }
    return err;
}

/**
*	@brief Determine board variant based on fuel gauging mechanism 
*/
static void detect_board_variant(void)
{
    printf("Detecting board variant...\n");
    if (battery_init() != ESP_OK) {
        ESP_LOGE("MAX17055", "MAX17055 initialization failed!");
        printf("No fuel gauge detected -> Board V1 (voltage divider)\n");
        g_board.type = BOARD_V1_VOLTAGE_DIVIDER;
        g_board.fuel_gauge_present = false;
    } else {
        printf("MAX17055 detected at 0x36 and initialized -> Board V2\n");
        g_board.type = BOARD_V2_MAX17055_FG;
        g_board.fuel_gauge_present = true;	
        Pcb_Version = 2;
    }
}

/**
*	@brief Detect board version at initialization
*/
void board_init(void)
{
    if (i2c_master_init() != ESP_OK) {
       ESP_LOGE(TAG, "I2C initialization failed!");
       return;
    } else {
        ESP_LOGI(TAG, "I2C initialized successfully!");
    }
    // small delay ensures bus is stable after power-up
    vTaskDelay(pdMS_TO_TICKS(100));
    detect_board_variant();
    vTaskDelay(pdMS_TO_TICKS(100));
    printf("Board type set to: %d\n", g_board.type);
}

static int32_t platform_i2c_write(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
    /* Build a single buffer: [reg, data_lsb, data_msb] */
    uint8_t buf[3];   /* reg + 2 data bytes — max17055 registers are 16-bit */
    buf[0] = reg;
    memcpy(&buf[1], data, len);
    
    esp_err_t err = i2c_master_write_to_device( I2C_PORT, MAX17055_ADDR, buf, (size_t)(len + 1u), pdMS_TO_TICKS(100));
    return (err == ESP_OK) ? 0 : -1;
}
 
static int32_t platform_i2c_read(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
    esp_err_t err = i2c_master_write_read_device( I2C_PORT, MAX17055_ADDR, &reg, 1, data, (size_t)len, pdMS_TO_TICKS(100));
    return (err == ESP_OK) ? 0 : -1;
}

static esp_err_t save_learned_params_to_nvs(void)
{
    nvs_handle_t nvs_handle;

    esp_err_t err = nvs_open("battery", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS");
        return err;
    }

    err = nvs_set_blob(
        nvs_handle,
        "learned",
        &g_learned,
        sizeof(g_learned));

    if (err == ESP_OK) {
        err = nvs_commit(nvs_handle);
    }

    nvs_close(nvs_handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Learned params saved to NVS");
    } else {
        ESP_LOGE(TAG, "Failed to save learned params");
    }

    return err;
}

static esp_err_t load_learned_params_from_nvs(void)
{
    nvs_handle_t nvs_handle;

    esp_err_t err = nvs_open("battery", NVS_READONLY, &nvs_handle);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No learned params in NVS");
        return err;
    }

    size_t required_size = sizeof(g_learned);

    err = nvs_get_blob(
        nvs_handle,
        "learned",
        &g_learned,
        &required_size);

    nvs_close(nvs_handle);

    if (err == ESP_OK) {
        g_learned_valid = true;
        ESP_LOGI(TAG, "Learned params restored from NVS");
    } else {
        ESP_LOGW(TAG, "Failed to load learned params");
    }

    return err;
}

static esp_err_t battery_init(void)
{
    fg = (max17055_ctx_t){
        .write_reg         = platform_i2c_write,
        .read_reg          = platform_i2c_read,
        .handle            = NULL,              /* unused on ESP-IDF */
        .rsense_mohm       = 10,                /* 10 mΩ sense resistor */
        .design_cap_mah    = 10000,             /* 10 000 mAh cell */
        .charge_voltage_mv = 4350,              // HV LiPo chemistry for HMI 
        .empty_voltage_mv  = 3300,              /* cut-off at 3.3 V */
        .ichg_term_ma      = 150,               /* C/20 termination current */
        .model_id          = MAX17055_MODEL_LITHIUM_COBALT,
    };

    uint16_t devname = 0;
    if (max17055_check_device(&fg, &devname) != 0) {
        ESP_LOGE(TAG, "MAX17055 not found on I2C bus!");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "MAX17055 found: DevName=0x%04X", devname);
 
    /* Read POR *before* init clears it */
    uint16_t status_raw = 0;
    max17055_reg_read(&fg, MAX17055_REG_STATUS, &status_raw);
    bool was_por = (status_raw & 0x0002u) != 0u;
 
    int32_t ret = max17055_init(&fg);
    if (ret != 0) {
        ESP_LOGE(TAG, "max17055_init() failed (err=%ld)", ret);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Fuel gauge initialized");
  
    /* Only restore learned params on a genuine power-on reset */
    if (was_por) {
        load_learned_params_from_nvs();
        if (g_learned_valid) {
            ESP_LOGI(TAG, "Restoring learned parameters from NVS...");
            ret = max17055_restore_learned_params(&fg, &g_learned);
            if (ret == 0) {
                vTaskDelay(pdMS_TO_TICKS(350));
                max17055_restore_capacity(&fg, &g_learned);
                vTaskDelay(pdMS_TO_TICKS(350));
            }
        }
    }
  return ESP_OK;
}

void max17055_save_learned(void) 
{
	/* Save learned parameters every time 64% of a cycle passes.
	 * The Cycles register has 1% LSB; bit 6 (value 64) toggling
	 * is the recommended trigger.  Here we track raw counts.     */
	uint16_t cycles_raw = 0;
	if (max17055_get_cycles_raw(&fg, &cycles_raw) == 0) {
	    if ((uint16_t)(cycles_raw - g_cycles_last_saved) >= 64u) {
	        if (max17055_save_learned_params(&fg, &g_learned) == 0) {
	            g_learned_valid     = true;
	            g_cycles_last_saved = cycles_raw;
	            ESP_LOGI(TAG, "Learned params saved (cycles_raw=%u)",
	                     cycles_raw);
	            save_learned_params_to_nvs();	           
	        }
	    }
	}	
}

/* =========================================================================
 * BATTERY READ FUNCTIONS
 * ========================================================================= */
float battery_get_soc(void)
{
    return max17055_get_soc(&fg);
}

float battery_get_voltage_mv(void)
{
    return max17055_get_voltage_mv(&fg);
}

float battery_get_soh(void)
{
    return max17055_get_age_pct(&fg);
}

float battery_get_current_ma(void)
{
    return max17055_get_current_ma(&fg);
}

float battery_get_temperature_c(void)
{
    return max17055_get_temperature_c(&fg);
}

float battery_get_cycles(void)
{
    return max17055_get_cycles(&fg);
}

float battery_get_remaining_capacity_mah(void)
{
    return max17055_get_rep_cap_mah(&fg);
}

float battery_get_original_capacity_mah(void)
{
    return max17055_get_full_cap_mah(&fg);
}

float battery_get_internal_resistance_mohm(void)
{
    return max17055_get_internal_resistance_mohm(&fg);
}

////////////////////////////////////////
// PWM Buzzer Functions              ///
////////////////////////////////////////

/**
 * @brief Configure PWM for buzzer
 */
void setupPWMForBuzzer() {
	printf("configuring PWM for buzzer...\n");
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,  
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_CHANNEL_BUZZER,
        .duty       = 0,
        .gpio_num   = BUZZER_PIN,
        .speed_mode = LEDC_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER
    };
    ledc_channel_config(&ledc_channel);
}

// Configure PWM for LED
void setupPWMForLED() {
	printf("setting led as PWM...\n");
    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_CHANNEL_LED,
        .duty       = 0,
        .gpio_num   = LED_PIN,
        .speed_mode = LEDC_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER
    };
    ledc_channel_config(&ledc_channel);
}

 /**
 * @brief blink yellow for X times.
 */
  void blink_green_yellow(int blink_count) {
    for (int i = 0; i < blink_count; i++) {        
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_LED, DUTY_100_PERCENT);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_LED);
        vTaskDelay(pdMS_TO_TICKS(200));
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_LED, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_LED);
        vTaskDelay(pdMS_TO_TICKS(200));          
    }
}

/**
 * @brief Activate the buzzer for a specific duration once
 */
void soundBuzzeronce(int duration_ms) {
	printf("sounding buzzer one time...\n");
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BUZZER, DUTY_10_PERCENT); 
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BUZZER);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));

    // Turn off buzzer
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BUZZER, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BUZZER);
}

/**
 * @brief Activate the buzzer twice with a short delay between sounds
 */
void soundBuzzertwo(int duration_ms, int delay_between_ms) {
	printf("flashing buzzer two times...\n");
    for (int i = 0; i < 2; i++) { 
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BUZZER, DUTY_10_PERCENT);  
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BUZZER);
        vTaskDelay(pdMS_TO_TICKS(duration_ms));  
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BUZZER, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BUZZER);
        vTaskDelay(pdMS_TO_TICKS(delay_between_ms));
    }
}

/**
 * @brief Activate the buzzer thrice with a short delay between sounds
 */
void soundBuzzerthrice(int duration_ms, int delay_between_ms) {
	printf("flashing buzzer three times...\n");
    for (int i = 0; i < 3; i++) { 
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BUZZER, DUTY_10_PERCENT); 
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BUZZER);
        vTaskDelay(pdMS_TO_TICKS(duration_ms));    
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BUZZER, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BUZZER);
        vTaskDelay(pdMS_TO_TICKS(delay_between_ms));     
    }
}

/**
 * @brief Activate the buzzer four times with a short delay between sounds
 */
void soundBuzzerfour(int duration_ms, int delay_between_ms) {
	printf("flashing buzzer four times...\n");
    for (int i = 0; i < 4; i++) {  
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BUZZER, DUTY_10_PERCENT); 
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BUZZER);
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BUZZER, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BUZZER);
        vTaskDelay(pdMS_TO_TICKS(delay_between_ms));  
    }
}

/**
 * @brief Activate the buzzer 10 times with a short delay between sounds
 */
void faultbuzzer(int duration_ms, int delay_between_ms) {
	printf("Sounding Alert Alarm!!...\n");
    for (int i = 0; i < 10; i++) { 
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BUZZER, DUTY_80_PERCENT); // High to cause enough nuisance for quick address
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BUZZER);
        vTaskDelay(pdMS_TO_TICKS(duration_ms)); 
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BUZZER, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BUZZER);
        vTaskDelay(pdMS_TO_TICKS(delay_between_ms));
    }
}

void playPowerOnTone(int ledc_channel, int ledc_timer) {
    printf("Playing BeeHappy power-on tone...\n");
    Note power_on_tone[] = {
        {NOTE_D5, 300}, // Bright start
        {NOTE_F5, 200}, // Strong buzz
        {NOTE_B5, 700}  // Happy finish
    };
    int note_count = sizeof(power_on_tone) / sizeof(Note);
    for (int i = 0; i < note_count; i++) {
        ledc_set_freq(LEDC_MODE, ledc_timer, power_on_tone[i].frequency);
        ledc_set_duty(LEDC_MODE, ledc_channel, DUTY_25_PERCENT);
        ledc_update_duty(LEDC_MODE, ledc_channel);
        vTaskDelay(pdMS_TO_TICKS(power_on_tone[i].duration));
        ledc_set_duty(LEDC_MODE, ledc_channel, 0);
        ledc_update_duty(LEDC_MODE, ledc_channel);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    printf("Finished playing BeeHappy power-on tone.\n");
}

/**
 * @brief Play a descending bee hum power-off tone
 */
void playPowerOffTone(int ledc_channel, int ledc_timer) {
    printf("Playing BeeHappy power-off tone...\n");
    Note power_off_tone[] = {
        {NOTE_B5, 300}, // Strong start
        {NOTE_F5, 400}, // Descend
        {NOTE_D5, 700}  // Fading buzz
    };
    int note_count = sizeof(power_off_tone) / sizeof(Note);
    for (int i = 0; i < note_count; i++) {
        ledc_set_freq(LEDC_MODE, ledc_timer, power_off_tone[i].frequency);
        ledc_set_duty(LEDC_MODE, ledc_channel, DUTY_10_PERCENT);
        ledc_update_duty(LEDC_MODE, ledc_channel);
        vTaskDelay(pdMS_TO_TICKS(power_off_tone[i].duration));
        ledc_set_duty(LEDC_MODE, ledc_channel, 0);
        ledc_update_duty(LEDC_MODE, ledc_channel);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    printf("Finished playing BeeHappy power-off tone.\n");
}

/**
 * @brief Initialize UART0
 */
void serial_uart_init() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_config);
    // Install UART driver
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
}

/**
 * @brief Initialize NVS
 */
void nvs_init() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {       
	    ESP_ERROR_CHECK(nvs_flash_erase());
	    ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    vTaskDelay(pdMS_TO_TICKS(100));
	// Initialize WiFi ssid and password
	load_wifi_ssid_from_nvs();
	load_wifi_pass_from_nvs();
	vTaskDelay(pdMS_TO_TICKS(50));    
}

////////////////////////////////////////
// WiFi Functions 				     ///
////////////////////////////////////////
static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        printf("WiFi connecting ... \n");
        break;
    case WIFI_EVENT_STA_CONNECTED:
        printf("WiFi connected ... \n");
        reconnect_attempts = 0; // Reset counter on successful connection
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        printf("WiFi disconnected ... \n");
        wifi_connected = false;
        if (reconnect_attempts < MAX_RECONNECT_ATTEMPTS)
        {
            reconnect_attempts++;
            printf("Reconnecting... Attempt %d/%d\n", reconnect_attempts, MAX_RECONNECT_ATTEMPTS);
            esp_wifi_connect();
        }
        else
        {
            printf("Max reconnection attempts reached.\n");
            //wifi_disconnect();
        }
        break;
    case IP_EVENT_STA_GOT_IP:
        printf("WiFi got IP ... \n\n");
        wifi_connected = true;
        break;
    default:
        break;
    }
}

void wifi_connection()
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);

    // Set WiFi mode to STA (critical fix)
    esp_wifi_set_mode(WIFI_MODE_STA);

	wifi_config_t wifi_configuration = { 0 }; // Initialize struct to avoid garbage values
	strcpy((char *)wifi_configuration.sta.ssid, WIFI_SSID);
	strcpy((char *)wifi_configuration.sta.password, WIFI_PASS);
	
	esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
	esp_wifi_start();
	esp_wifi_connect();

	printf("Waiting for WiFi reconnection to timeout...\n");
	while (reconnect_attempts < MAX_RECONNECT_ATTEMPTS && !wifi_connected) {
	    printf(".");
	    vTaskDelay(pdMS_TO_TICKS(100));
	} 
}

void wifi_disconnect()
{
    printf("Disconnecting WiFi...\n");
    
    // Unregister WiFi event handlers to prevent duplicate handling
    esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler);
    esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler);

    // Stop and deinit WiFi properly
    esp_wifi_disconnect();
    esp_wifi_stop();
    esp_wifi_deinit();

    // Destroy the default network interface
    esp_netif_destroy(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"));

    // Delete the event loop
    esp_event_loop_delete_default();

    wifi_connected = false;
    reconnect_attempts = 0;
    printf("WiFi disconnected and resources freed.\n");
}

// Load Wi-Fi SSID from NVS
void load_wifi_ssid_from_nvs() {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err == ESP_OK) {
        size_t required_size = 0;
        err = nvs_get_str(my_handle, "ssid", NULL, &required_size);

        if (err == ESP_OK && required_size > 0) {
            nvs_get_str(my_handle, "ssid", WIFI_SSID, &required_size);
            printf("Loaded Wi-Fi SSID: %s\n", WIFI_SSID);
        } else {
            ESP_LOGW("SSID", "No Wi-Fi SSID found in NVS, using default value: %s", WIFI_SSID);
            log_error("No Wi-Fi SSID found in NVS, using default value: %s", WIFI_SSID);
        }
        nvs_close(my_handle);
    } else {
        ESP_LOGE("SSID", "Error opening NVS handle!");
        log_error("load_wifi_ssid_from_nvs Error opening NVS handle!");
    }
}

// Load Wi-Fi password from NVS
void load_wifi_pass_from_nvs() {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err == ESP_OK) {
        size_t required_size = 0;
        err = nvs_get_str(my_handle, "pass", NULL, &required_size);

        if (err == ESP_OK && required_size > 0) {
            nvs_get_str(my_handle, "pass", WIFI_PASS, &required_size);
            printf("Loaded Wi-Fi PASS: ********\n");
        } else {
            ESP_LOGW("PASS", "No Wi-Fi PASS found in NVS, using default value: %s", WIFI_PASS);
            log_error("No Wi-Fi PASSWORD found in NVS, using default value: %s", WIFI_PASS);
        }
        nvs_close(my_handle);
    } else {
        ESP_LOGE("PASS", "Error opening NVS handle!");
        log_error("Error opening NVS handle!");
    }
}

// Save Wi-Fi SSID to NVS
void save_wifi_ssid_to_nvs(const char *ssid) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        err = nvs_set_str(my_handle, "ssid", ssid);
        if (err == ESP_OK) {
            nvs_commit(my_handle);
            printf("Saved Wi-Fi SSID: %s\n", ssid);
        } else {
            ESP_LOGE("SSID", "Failed to store Wi-Fi SSID!");
            log_error("Failed to store Wi-Fi SSID!");
        }
        nvs_close(my_handle);
    } else {
        ESP_LOGE("SSID", "Error opening NVS handle!");
        log_error("Error opening NVS handle for saving Wi-Fi SSID!");
    }
}

// Save Wi-Fi password to NVS
void save_wifi_pass_to_nvs(const char *pass) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        err = nvs_set_str(my_handle, "pass", pass);
        if (err == ESP_OK) {
            nvs_commit(my_handle);
            printf("Saved Wi-Fi PASS: %s\n", pass);
        } else {
            ESP_LOGE("PASS", "Failed to store Wi-Fi PASS!");
            log_error("Failed to store Wi-Fi PASSWORD on NVS!");
        }
        nvs_close(my_handle);
    } else {
        ESP_LOGE("PASS", "Error opening NVS handle!");
        log_error("Error opening NVS handle for saving Wi-Fi PASSWORD!");
    }
}

////////////////////////////////////////
// EPD & GET Aranet Function   		 ///
////////////////////////////////////////
// Buffer for storing HTTP response
// Accessed by two functions || All must be in the same Task else Mutex is needed
#define MAX_RESPONSE_SIZE 2048
static char http_response_buffer[MAX_RESPONSE_SIZE]; 
static size_t http_response_length = 0;

void get_aranet_data(void) {
    // Check WiFi connection status
    if (!wifi_connected) {
        ESP_LOGE(TAG, "Wi-Fi Not Connected!");
        log_error("HMI Device lost Wi-Fi Connection!");
        
	    EPD_ShowString(260, 80, "OFF", 24, WHITE); // WiFi OFF
        
        return;
    } else if (wifi_connected) {
	  EPD_ShowString(260, 80, "ON", 24, WHITE); // WiFi ON
	 } 
	    
    // Reset response buffer
    http_response_length = 0;
    memset(http_response_buffer, 0, sizeof(http_response_buffer));
     
    akuhId = getAkuhId();   
    const char *API_URL = getDeviceUrl(akuhId);
     
    // Configure HTTP client
    esp_http_client_config_t config_get = {
        .url = API_URL,
        .method = HTTP_METHOD_GET,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 10000,  // Increased timeout
        .buffer_size = 1024,  // Larger buffer size
        .event_handler = client_event_get_handler,
        .disable_auto_redirect = true,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config_get);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client!");
        log_error("Failed to initialize HTTP client!");
        return;
    }

    // Set headers
    esp_http_client_set_header(client, "accept", "application/json");
    esp_http_client_set_header(client, "ApiKey", API_KEY);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "Connection", "close");  // Ensure clean connection close
    
    // Execute GET request
    esp_err_t err = esp_http_client_perform(client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
        log_error("HTTP GET request failed: %s", esp_err_to_name(err));
        networkError = true;
        esp_http_client_cleanup(client);
		
		EPD_DrawCircle(115, 280, 7, WHITE, 0);   // Network Error (empty circle)
        return;
    } else {
        networkError = false;
		EPD_DrawCircle(225, 280, 7, WHITE, 1);   // Network Error (white filled)
    }

    // Check status code
    int status_code = esp_http_client_get_status_code(client);
    printf("HTTP Status Code: %d\n", status_code);
    
    if (status_code != 200) {
        ESP_LOGE(TAG, "API error: %d", status_code);
        log_error("API error: %d", status_code);
        apiError = true;
        
        // Print common error meanings
        if (status_code == 401) {
			ESP_LOGE(TAG, "Unauthorized - check API key"); log_error("Unauthorized - check API key");}  
        else if (status_code == 404) {ESP_LOGE(TAG, "Not Found - check URL"); log_error("Not Found - check URL");}
        else if (status_code == 500) {ESP_LOGE(TAG, "Server Error"); log_error("Server Error");}
        
        esp_http_client_cleanup(client);

		EPD_DrawCircle(385, 280, 7, WHITE, 0);  // API Error (empty circle)
		 
        return;
    } else {
        apiError = false;
		EPD_DrawCircle(385, 280, 7, WHITE, 1);  // API Error (white filled)
    }
    // Verify we received data
    if (http_response_length == 0) {
        ESP_LOGE(TAG, "No data received in event handler");
        log_error("No data received in event handler! https response length = 0");
        esp_http_client_cleanup(client);
        return;
    }

    // Log first part of response for debugging
    printf("Response (first 100 chars): %.100s\n", http_response_buffer);

    // Parse JSON response
    cJSON *root = cJSON_Parse(http_response_buffer);
    if (!root) {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL) {
            ESP_LOGE(TAG, "JSON Parsing Error before: %s", error_ptr);
            log_error("JSON Parsing Error before: %s", error_ptr);
        }
        ESP_LOGE(TAG, "JSON Parsing Failed!");
        log_error("JSON Parsing Failed!");
        esp_http_client_cleanup(client);
        return;
    }

    // Process readings
    cJSON *readings = cJSON_GetObjectItem(root, "readings");
    if (readings && cJSON_IsArray(readings)) {
        int array_size = cJSON_GetArraySize(readings);
        printf("Found %d readings\n", array_size);
        
        for (int i = 0; i < array_size; i++) {
            cJSON *reading = cJSON_GetArrayItem(readings, i);
            if (!reading) continue;

            cJSON *sensor = cJSON_GetObjectItem(reading, "sensor");
            cJSON *value = cJSON_GetObjectItem(reading, "value");
            cJSON *time = cJSON_GetObjectItem(reading, "time");

            if (sensor && value && time) {
                //char *sensor_id = cJSON_GetStringValue(sensor);
                float press_value = (float)cJSON_GetNumberValue(value);
                char *time_eat = convert_utc_to_eat(cJSON_GetStringValue(time));
					if (target_number == 14 || target_number == 15) {  
					    Temperature = roundf(press_value * 100.0f) / 100.0f;
					} else {
					    Pressure = roundf(press_value * 100.0f) / 100.0f; 
					}													
				strncpy(Time, time_eat, sizeof(Time)-1);
                Time[sizeof(Time)-1] = '\0'; 
                printf("Sensor: %.2f Pa at %s\n", press_value, time_eat);
				
                free(time_eat);
            }
        }
    } else {
        printf("No readings found in response\n");
    }

    // Cleanup
    cJSON_Delete(root);
    esp_http_client_cleanup(client);
    printf("Data fetch completed\n");
}

// Event handler for HTTP client
esp_err_t client_event_get_handler(esp_http_client_event_handle_t evt) {
    switch (evt->event_id) {
        case HTTP_EVENT_ON_DATA:
            printf("HTTP_EVENT_ON_DATA, len=%d\n", evt->data_len);
            // Check if we have space to store this chunk
            if (http_response_length + evt->data_len < MAX_RESPONSE_SIZE) {
                memcpy(http_response_buffer + http_response_length, evt->data, evt->data_len);
                http_response_length += evt->data_len;
            } else {
                ESP_LOGE(TAG, "Response too large for buffer!\n");
                log_error("Https GET response too large for buffer!\n");
            }
            break;
            
        case HTTP_EVENT_ON_FINISH:
            printf("HTTP_EVENT_ON_FINISH\n");
            // Ensure null termination
            if (http_response_length < MAX_RESPONSE_SIZE) {
                http_response_buffer[http_response_length] = '\0';
            }
            break;
            
        default:
            break;
    }
    return ESP_OK;
}

char* convert_utc_to_eat(const char *utc_time) {
    struct tm tm = {0};
    strptime(utc_time, "%Y-%m-%dT%H:%M:%SZ", &tm);
    tm.tm_hour += 3; // Convert to EAT (UTC+3)
    mktime(&tm);

    char *buffer = malloc(20);
    strftime(buffer, 20, "%d/%m %H:%M", &tm);
    return buffer;
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
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Stabilize power
}

void epd_power_off() {
    esp_rom_gpio_pad_select_gpio(EPD_POWER_PIN);
    gpio_set_direction(EPD_POWER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(EPD_POWER_PIN, 0); 
}

void EPD_Partial_Data_Update() { // TODO

    char buffer[100];
    const char* sensor;
    const char* ID;
	
	// Scan which device to use
	switch (target_number) {
	    case 1: sensor = "OR1"; ID = "ID: 00745"; break;
	    case 2: sensor = "OR2"; ID = "ID: 00746"; break;
	    case 3: sensor = "OR3"; ID = "ID: 00747"; break;
	    case 4: sensor = "OR4"; ID = "ID: 00748"; break;
	    case 5: sensor = "OR5"; ID = "ID: 00749"; break;
	    case 6: sensor = "OR6"; ID = "ID: 0074B"; break;
	    case 7: sensor = "OR7"; ID = "ID: 0074C"; break;
	    case 8: sensor = "OR8"; ID = "ID: 0074D"; break;
	    case 9: sensor = "MATERNITY"; 	 ID = "ID: 0074A"; break; 
	    case 10: sensor = "CSSD DAWC"; 	 ID = "ID: 007A6"; break; 
	    case 11: sensor = "CSSD DIFF"; 	 ID = "ID: 00733"; break; 
	    case 12: sensor = "CSSD PAWD"; 	 ID = "ID: 007A7"; break; 
	    case 13: sensor = "CSSD BUFF";   ID = "ID: 0098E"; break;
	    case 14: sensor = "MATERN PHAR"; ID = "ID: 036B4"; break;
	    case 15: sensor = "WH FRIDGE";   ID = "ID: 035DA"; break;
	    default: sensor = "Unknown";     ID = "ID: 00000"; break;
	}

	/**
	*  Display the sensor room 
	*/
	snprintf(Sensor, sizeof(Sensor), "%s", sensor);
	snprintf(Sensor_ID, sizeof(Sensor_ID), "%s", ID); 
	vTaskDelay(50 / portTICK_PERIOD_MS);
	
	if (target_number == 9  || target_number == 10 || target_number == 11 || target_number == 12 || target_number == 13 ) {
		    snprintf(buffer, sizeof(buffer), "%s", sensor);
		    EPD_ShowString(250, 180, buffer, 24, BLACK);
	} else if (target_number == 14  || target_number == 15 ) {
		    snprintf(buffer, sizeof(buffer), "%s", sensor);
		    EPD_ShowString(240, 180, buffer, 24, BLACK);				
	} else {
	    snprintf(buffer, sizeof(buffer), "%s", sensor);
	    EPD_ShowString(270, 180, buffer, 48, BLACK);				
	}
	
	/**
	*  Display the sensor ID
	*/
	snprintf(buffer, sizeof(buffer), "%s", ID);
    EPD_ShowString(270, 230, buffer, 16, BLACK);
     
    // Sensor Data Update
	if (target_number == 14 || target_number == 15) {
		snprintf(buffer, sizeof(buffer), "%.2f", Temperature);
		EPD_ShowString(55, 130, buffer, 24, BLACK);
	} else {
		snprintf(buffer, sizeof(buffer), "%.2f Pa", Pressure);
		EPD_ShowString(55, 130, buffer, 24, BLACK);
	}    
    
    strncpy(buffer, Time, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0'; // Ensure null-termination
    EPD_ShowString(35, 240, buffer, 24, BLACK); 
 
	if (battery_low_ui == 1) {
		
	} else {
	    // Update battery percentage
		snprintf(buffer, sizeof(buffer), "%d", batteryCapacity);
		EPD_ShowString(335, 79, buffer, 24, WHITE);		
	}
}

//
void displayui() {
        Paint_NewImage(Image_BW, EPD_W, EPD_H, 0, WHITE);
        EPD_Full(WHITE);
        EPD_Display_Part(0, 0, EPD_W, EPD_H, Image_BW);
        EPD_Init_Fast(Fast_Seconds_1_5s);
        
 
        if (target_number == 1) { // TODO
          EPD_ShowPicture(5, 0, 128, 296, epd_bitmap_boot_ui, BLACK); // boot ui
        } else if (target_number == 2) {
          EPD_ShowPicture(5, 0, 128, 296, epd_bitmap_apples_ui, BLACK); // apples ui
        } else if (target_number == 3) {
          EPD_ShowPicture(5, 0, 128, 296, epd_bitmap_avocados_ui, BLACK); // avocados ui
        } else if (target_number == 4) {
          EPD_ShowPicture(5, 0, 128, 296, epd_bitmap_bananas_ui, BLACK); // bananas ui
        } else if (target_number == 5) {
          EPD_ShowPicture(5, 0, 128, 296, epd_bitmap_hp_ui, BLACK); // hp ui
        } else if (target_number == 6) {
          EPD_ShowPicture(5, 0, 128, 296, epd_bitmap_iphone_ui, BLACK); // iphone ui
        } else if (target_number == 7) {
          EPD_ShowPicture(5, 0, 128, 296, epd_bitmap_oppo_ui, BLACK); // oppo ui
        } else if (target_number == 8) {
          EPD_ShowPicture(5, 0, 128, 296, epd_bitmap_sayona_ui, BLACK); // sayona ui
        } else {
          EPD_ShowPicture(5, 0, 128, 296, epd_bitmap_template_ui, BLACK); // template ui
        }
                           
        EPD_Display_Fast(Image_BW);
        vTaskDelay(50 / portTICK_PERIOD_MS);
  }

void EPD_display_Battery_Low() {
	if (g_board.type == BOARD_V2_MAX17055_FG) {
		read_batt_params();
		if (Soc < MIN_SOC) {
			battery_low_ui = 1; 
			printf("Battery voltage critically low. Sounding Alarm!!\n");
	    	vTaskDelay(50 / portTICK_PERIOD_MS);
	        EPD_Clear();
	        vTaskDelay(50 / portTICK_PERIOD_MS);
	        Paint_NewImage(Image_BW, EPD_W, EPD_H, 0, WHITE);
	        EPD_Full(WHITE);
	        EPD_Display_Part(0, 0, EPD_W, EPD_H, Image_BW);
	        EPD_Init_Fast(Fast_Seconds_1_5s);
	        
	        if (target_number == 14 || target_number == 15) {
				//EPD_ShowPicture(0, 0, 400, 300, epd_bitmap_Hmi_temp_batt_low_ui, BLACK); 
			} else {
				//EPD_ShowPicture(0, 0, 400, 300, epd_bitmap_Hmi_batt_low_ui, BLACK); 
			}	        
	        	        
	        EPD_Display_Fast(Image_BW);
		    EPD_Partial_Data_Update();
		    EPD_Display_Part(0, 0, EPD_W, EPD_H, Image_BW); // 
	    	battlow = 1; 
	    	battery_low_ui = 0; 
	        faultbuzzer(500, 500); // Battery low buzzer alert!
	        
			  if (wifi_connected && !networkError) {
				    smtp_press_batt_warn_flag = 1; 
					Alarmtype = 1;
					vTaskDelay(50 / portTICK_PERIOD_MS); 
					smtp_client_task();
					smtp_press_batt_warn_flag = 0; 
		        	vTaskDelay(50 / portTICK_PERIOD_MS);
		        	wifi_disconnect();
		        	return; 		  			  
			  } else {
				  printf("WiFi Disconnected, skipping Battery Low Mail Alert!\n");
				  log_error("WiFi Disconnected, skipping Battery Low Mail Alert!");
			  }       
		} else {
			battlow = 0;
		}
	} else {
		readBatteryVoltage();
		if (Voltage < MIN_VOLTAGE) { // 
			battery_low_ui = 1; 
			printf("Battery voltage critically low. Sounding Alarm!!\n");
	    	vTaskDelay(50 / portTICK_PERIOD_MS);
	        EPD_Clear();
	        vTaskDelay(50 / portTICK_PERIOD_MS);
	        Paint_NewImage(Image_BW, EPD_W, EPD_H, 0, WHITE);
	        EPD_Full(WHITE);
	        EPD_Display_Part(0, 0, EPD_W, EPD_H, Image_BW);
	        EPD_Init_Fast(Fast_Seconds_1_5s);
	        
	        if (target_number == 14 || target_number == 15) {
				// EPD_ShowPicture(0, 0, 400, 300, epd_bitmap_Hmi_temp_batt_low_ui, BLACK); 
			} else {
				// EPD_ShowPicture(0, 0, 400, 300, epd_bitmap_Hmi_batt_low_ui, BLACK); 
			}
	        
	        EPD_Display_Fast(Image_BW);
		    EPD_Partial_Data_Update();
		    EPD_Display_Part(0, 0, EPD_W, EPD_H, Image_BW); // 
	    	battlow = 1; 
	    	battery_low_ui = 0; 
	        faultbuzzer(500, 500); // Battery low buzzer alert!
	        
			  if (wifi_connected && !networkError) {
				    smtp_press_batt_warn_flag = 1; 
					Alarmtype = 1;
					vTaskDelay(50 / portTICK_PERIOD_MS); 
					smtp_client_task();
					smtp_press_batt_warn_flag = 0; 
		        	vTaskDelay(50 / portTICK_PERIOD_MS);
		        	wifi_disconnect();
		        	return; 		  			  
			  } else {
				  printf("WiFi Disconnected, skipping Battery Low Mail Alert!\n");
				  log_error("WiFi Disconnected, skipping Battery Low Mail Alert!");
			  }       
		} else {
			battlow = 0;
		}
	}
}

void checkpressurethreshold() 
{
    if (!wifi_connected || networkError) {
        printf("WiFi Disconnected, skipping pressure threshold check!\n");
        log_error("WiFi Disconnected, skipping pressure threshold check!\n");	
        return;
    }
    printf("Checking pressure thresholds...\n");
    // Default pressure range
    float minPressure = 2.5f;
    float maxPressure = 30.0f;
    // Special cases
    if (target_number == 10) { 
        minPressure = -7.5f;
        maxPressure = -2.5f;
    } else if (target_number == 13 ) {
        minPressure = 1.5f;
        maxPressure = 25.0f;
    }

    // Check if pressure is outside range
    bool pressureOutOfRange = (Pressure < minPressure || Pressure > maxPressure);
    if (!pressureOutOfRange) {
        return;
    }

    clear_all();
    vTaskDelay(10 / portTICK_PERIOD_MS);
    //EPD_ShowPicture(0, 0, 400, 300, epd_bitmap_hmi_warning_ui, BLACK); // Pressure out of range ui
    EPD_Display_Part(0, 0, EPD_W, EPD_H, Image_BW);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    char buffer[70];
    snprintf(buffer, sizeof(buffer), "%.2f Pa", Pressure);  
    EPD_ShowString(120, 266, buffer, 16, BLACK);

    strncpy(buffer, Time, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0'; // Ensure null-termination
    EPD_ShowString(240, 266, buffer, 16, BLACK); 

    EPD_Display_Part(0, 0, EPD_W, EPD_H, Image_BW);
    smtp_press_batt_warn_flag = 1; // 
    vTaskDelay(50 / portTICK_PERIOD_MS);
    faultbuzzer(1000, 1000); // On for 1s and off for 1s
    smtp_client_task();
    smtp_press_batt_warn_flag = 0; // 
    vTaskDelay(50 / portTICK_PERIOD_MS);   
    // Check chip ID to attach 
    getChipIdString(deviceSerial, sizeof(deviceSerial)); 
    vTaskDelay(pdMS_TO_TICKS(10));
	snprintf(Access_token, sizeof(Access_token), "%s%ld", deviceSerial, target_number);
	printf("Access Token: %s\n", Access_token);  
    vTaskDelay(50 / portTICK_PERIOD_MS);                       
    log_error("%s (%s) Pressure out of range! Current Value: %.2f", Sensor, deviceSerial, Pressure); 
}

void checktempthreshold() {
    if (!wifi_connected || networkError) {
        printf("WiFi Disconnected, skipping Temperature threshold check!\n");
        log_error("WiFi Disconnected, skipping Temperature threshold check!\n");	
        return;
    }
    printf("Checking Temperature thresholds...\n");
    // Default Temperature range
    float minTemp = 2.5f;
    float maxTemp = 7.5f;

    // Check if Temperature is outside range
    bool tempOutOfRange = (Temperature < minTemp || Temperature > maxTemp);
    if (!tempOutOfRange) {
        return;
    }

    clear_all();
    vTaskDelay(10 / portTICK_PERIOD_MS);
    //EPD_ShowPicture(0, 0, 400, 300, epd_bitmap_hmi_temp_warning_ui, BLACK); // Temp out of range ui
    EPD_Display_Part(0, 0, EPD_W, EPD_H, Image_BW);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    char buffer[70];
    snprintf(buffer, sizeof(buffer), "%.2f", Temperature);  
    EPD_ShowString(140, 266, buffer, 16, BLACK);

    strncpy(buffer, Time, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0'; // Ensure null-termination
    EPD_ShowString(240, 266, buffer, 16, BLACK); 

    EPD_Display_Part(0, 0, EPD_W, EPD_H, Image_BW);
    smtp_press_batt_warn_flag = 1; // 
    vTaskDelay(50 / portTICK_PERIOD_MS);
    faultbuzzer(1000, 1000); // On for 1s and off for 1s
    smtp_client_task();
    smtp_press_batt_warn_flag = 0; // 
    vTaskDelay(50 / portTICK_PERIOD_MS);   
    // Check chip ID to attach 
    getChipIdString(deviceSerial, sizeof(deviceSerial)); 
    vTaskDelay(pdMS_TO_TICKS(10));
	snprintf(Access_token, sizeof(Access_token), "%s%ld", deviceSerial, target_number);
	printf("Access Token: %s\n", Access_token);  
    vTaskDelay(50 / portTICK_PERIOD_MS);                       
    log_error("%s (%s) Temperature out of range! Current Value: %.2f", Sensor, deviceSerial, Temperature); 
}

/*******************************************************************************************************************************/


/*******************************************************************************************************************************
 SPIFFS Functions
*********************************/
void initialize_spiffs(void)
{
    ESP_LOGI(TAG, "Initializing SPIFFS");

	esp_vfs_spiffs_conf_t conf = {
	    .base_path = "/spiffs",
	    .partition_label = "storage",  // Match partition name in csv
	    .max_files = 5,
	    .format_if_mount_failed = true
	};

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }
    
    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d bytes, used: %d bytes", total, used);
        //log_error("Partition size: total: %d bytes, used: %d bytes", total, used); // Commented out for verbosity control
        ESP_LOGI(TAG, "Free space: %d bytes (%.1f%%)", 
                total - used, (100.0 * (total - used)) / total);
        //log_error("Free space: %d bytes (%.1f%%)", total - used, (100.0 * (total - used)) / total);
    } 
}

// Error Data Logger
void log_error(const char *format, ...) {
    if (format == NULL) return; // Safety check
    
    char file_content[300]; // Adjust size if needed
    va_list args;
    
    va_start(args, format);
    vsnprintf(file_content, sizeof(file_content), format, args); // Format the message
    va_end(args);

    log_data(path, file_content); // Log it
}

// Write function (appends logs)
void log_data(const char *path, char *content) {
    FILE *f = fopen(path, "a");  // 'a' for append mode
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open log file");
        return;
    }
    
    // Get current timestamp
    time_t now = time(NULL);
    struct tm *tm = localtime(&now);
    char timestamp[20];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", tm);
    
    fprintf(f, "[%s] %s\n", timestamp, content); // Add timestamped entry
    fclose(f);
    ESP_LOGD(TAG, "Logged: %s", content);
}

/**
 * Reads entire file into a dynamically allocated buffer
 * @param path File path to read
 * @return Pointer to allocated buffer with file contents (MUST BE FREED BY CALLER)
 *         NULL on failure
 */
char* read_file(const char *path) {
ESP_LOGI(TAG, "Reading file %s", path);
    
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        log_error("Failed to open txt file for reading");
        return NULL;
    }
    
    // Get file size
    fseek(f, 0, SEEK_END);
    size_t size = ftell(f);
    fseek(f, 0, SEEK_SET);
    
    // Allocate buffer (+1 for null terminator)
    char *buffer = malloc(size + 1); 
    if (!buffer) {
        fclose(f);
        ESP_LOGE(TAG, "Memory allocation failed");
        log_error("txt file reading Memory allocation failed");
        return NULL;
    }

    // Read entire file
    size_t bytes_read = fread(buffer, 1, size, f);
    buffer[bytes_read] = '\0'; // Null-terminate
    
    fclose(f);
    return buffer;
}

/**
 * Clears/truncates a file
 * @param path File path to clear
 * @return true if successful, false on error
 */
bool clear_file(const char *path) {
    FILE *f = fopen(path, "w"); // Open in write mode to truncate
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to clear file %s", path);
        log_error("Failed to clear txt file %s", path);
        return false;
    }
    fclose(f);
    ESP_LOGI(TAG, "Cleared file %s", path);
    return true;
}

/*******************************************************************************************************************************
 EPD Pages
*********************************/
// Load Target Number from NVS
int load_target_number_from_nvs() {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);

    if (err == ESP_OK) {
        err = nvs_get_i32(my_handle, "target_number", &target_number);
        if (err == ESP_OK) {
            printf("Loaded Target Number: %ld\n", target_number);
        } else {
            ESP_LOGW("TARGET", "No target number found in NVS, using default: %ld", target_number);
            log_error("No target number found in NVS, using default: %ld", target_number);
        }
        nvs_close(my_handle);
    } else {
        ESP_LOGE("TARGET", "Error opening NVS handle!");
        log_error("load_target_number_from_nvs Error opening NVS handle!");
    }
    return target_number;
}

// Save Target Number to NVS
void save_target_number_to_nvs(int number) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);

    if (err == ESP_OK) {
        err = nvs_set_i32(my_handle, "target_number", number);
        if (err == ESP_OK) {
            nvs_commit(my_handle);
            printf("Saved Target Number: %d\n", number);
        } else {
            ESP_LOGE("TARGET", "Failed to store target number!");
            log_error("Failed to store target number!");
        }
        nvs_close(my_handle);
    } else {
        ESP_LOGE("TARGET", "Error opening NVS handle!");
        log_error("Error opening NVS handle for saving target number!");
    }
}


// Handle button click to cycle target number forward
void handle_next_target_selection() {
    int32_t current_number = load_target_number_from_nvs();
    int32_t next_number = (current_number % MAX_NUMBER) + 1;
    save_target_number_to_nvs(next_number);
    printf("Target number changed to: %ld\n", next_number);
}

// Handle button click to cycle target number backward
void handle_previous_target_selection() {
    int32_t current_number = load_target_number_from_nvs();
    int32_t previous_number =
        ((current_number - 2 + MAX_NUMBER) % MAX_NUMBER) + 1;
    save_target_number_to_nvs(previous_number);
    printf("Target number changed to: %ld\n", previous_number);
}

/*******************************************************************************************************************************
 SNTP Functions
*********************************/
bool sync_time_from_sntp() {
    // Initialize SNTP with DHCP support
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG_MULTIPLE(3,
                           ESP_SNTP_SERVER_LIST(SNTP_SERVER_0, SNTP_SERVER_1, SNTP_SERVER_2));
    
    config.server_from_dhcp = true;       // Accept NTP servers from DHCP
    config.renew_servers_after_new_IP = true;
    config.smooth_sync = true;            // Gradual time adjustment
    esp_netif_sntp_init(&config);

    // Wait for synchronization
    int retry = 0;
    while (esp_netif_sntp_sync_wait(SNTP_RETRY_DELAY_MS / portTICK_PERIOD_MS) == ESP_ERR_TIMEOUT) {
        ESP_LOGI(TAG, "Waiting for time sync... (%d/%d)", ++retry, SNTP_MAX_RETRIES);
        if (retry >= SNTP_MAX_RETRIES) {
            ESP_LOGE(TAG, "Failed to sync time!");
            log_error("SNTP Failed to sync time!");
            esp_netif_sntp_deinit();
            return false;
        }
    }

    // Time synced successfully
    esp_netif_sntp_deinit();
    return true;
}

void print_current_time()
{
    // Ensure timezone is set (Kenya - EAT, UTC+3)
    setenv("TZ", TIMEZONE, 1);
    tzset();

    // Get current time
    time_t now;
    time(&now);
    localtime_r(&now, &timeinfo);

    // Format and print time
    char strftime_buf[64];
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    printf("Current time in Kenya: %s", strftime_buf);
    //log_error("Current time in Kenya: %s", strftime_buf);
}
/*******************************************************************************************************************************
 SMTP Functions
*********************************/

/**
 * @brief smtp init and execution
 */
void smtp_client_task() {
    char *buf = NULL;
    unsigned char base64_buffer[128];
    int ret, len;
    size_t base64_len;

    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context ctr_drbg;
    mbedtls_ssl_context ssl;
    mbedtls_x509_crt cacert;
    mbedtls_ssl_config conf;
    mbedtls_net_context server_fd;

    mbedtls_ssl_init(&ssl);
    mbedtls_x509_crt_init(&cacert);
    mbedtls_ctr_drbg_init(&ctr_drbg);
    ESP_LOGI(TAG, "Seeding the random number generator");

    mbedtls_ssl_config_init(&conf);

    mbedtls_entropy_init(&entropy);
    if ((ret = mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy,
                                     NULL, 0)) != 0) {
        ESP_LOGE(TAG, "mbedtls_ctr_drbg_seed returned -0x%x", -ret);
        log_error("mbedtls_ctr_drbg_seed returned -0x%x", -ret);
        goto exit;
    }

    ESP_LOGI(TAG, "Loading the CA root certificate...");

    ret = mbedtls_x509_crt_parse(&cacert, server_root_cert_pem_start,
                                 server_root_cert_pem_end - server_root_cert_pem_start);

    if (ret < 0) {
        ESP_LOGE(TAG, "mbedtls_x509_crt_parse returned -0x%x", -ret);
        log_error("mbedtls_x509_crt_parse returned -0x%x", -ret);
        goto exit;
    }

    ESP_LOGI(TAG, "Setting hostname for TLS session...");

    /* Hostname set here should match CN in server certificate */
    if ((ret = mbedtls_ssl_set_hostname(&ssl, MAIL_SERVER)) != 0) {
        ESP_LOGE(TAG, "mbedtls_ssl_set_hostname returned -0x%x", -ret);
        log_error("mbedtls_ssl_set_hostname returned -0x%x", -ret);
        goto exit;
    }

    ESP_LOGI(TAG, "Setting up the SSL/TLS structure...");

    if ((ret = mbedtls_ssl_config_defaults(&conf,
                                           MBEDTLS_SSL_IS_CLIENT,
                                           MBEDTLS_SSL_TRANSPORT_STREAM,
                                           MBEDTLS_SSL_PRESET_DEFAULT)) != 0) {
        ESP_LOGE(TAG, "mbedtls_ssl_config_defaults returned -0x%x", -ret);
        log_error("mbedtls_ssl_config_defaults returned -0x%x", -ret);
        goto exit;
    }

    mbedtls_ssl_conf_authmode(&conf, MBEDTLS_SSL_VERIFY_REQUIRED);
    //mbedtls_ssl_conf_authmode(&conf, MBEDTLS_SSL_VERIFY_NONE);
    mbedtls_ssl_conf_ca_chain(&conf, &cacert, NULL);
    mbedtls_ssl_conf_rng(&conf, mbedtls_ctr_drbg_random, &ctr_drbg);
#ifdef CONFIG_MBEDTLS_DEBUG
    mbedtls_esp_enable_debug_log(&conf, 4);
#endif

    if ((ret = mbedtls_ssl_setup(&ssl, &conf)) != 0) {
        ESP_LOGE(TAG, "mbedtls_ssl_setup returned -0x%x", -ret);
        log_error("mbedtls_ssl_setup returned -0x%x", -ret);
        goto exit;
    }

    mbedtls_net_init(&server_fd);

    ESP_LOGI(TAG, "Connecting to %s:%s...", MAIL_SERVER, MAIL_PORT);

    if ((ret = mbedtls_net_connect(&server_fd, MAIL_SERVER,MAIL_PORT, MBEDTLS_NET_PROTO_TCP)) != 0) {
        ESP_LOGE(TAG, "mbedtls_net_connect returned -0x%x", -ret);
        log_error("mbedtls_net_connect returned -0x%x", -ret);
        goto exit;
    }

    ESP_LOGI(TAG, "Connected.");

    mbedtls_ssl_set_bio(&ssl, &server_fd, mbedtls_net_send, mbedtls_net_recv, NULL);

    buf = (char *) calloc(1, BUF_SIZE);
    if (buf == NULL) {
        ESP_LOGE(TAG, "calloc failed for size %d", BUF_SIZE);
        log_error("calloc failed for size %d", BUF_SIZE);
        goto exit;
    }
#if SERVER_USES_STARTSSL
    /* Get response */
    ret = write_and_get_response(&server_fd, (unsigned char *) buf, 0);
    VALIDATE_MBEDTLS_RETURN(ret, 200, 299, exit);

    ESP_LOGI(TAG, "Writing HELO to server...");
    len = snprintf((char *) buf, BUF_SIZE, "HELO %s\r\n", "ESP32");
    ret = write_and_get_response(&server_fd, (unsigned char *) buf, len);
    VALIDATE_MBEDTLS_RETURN(ret, 200, 299, exit);

    ESP_LOGI(TAG, "Writing STARTTLS to server...");
    len = snprintf((char *) buf, BUF_SIZE, "STARTTLS\r\n");
    ret = write_and_get_response(&server_fd, (unsigned char *) buf, len);
    VALIDATE_MBEDTLS_RETURN(ret, 200, 299, exit);

    ret = perform_tls_handshake(&ssl);
    if (ret != 0) {
        goto exit;
    }

#else /* SERVER_USES_STARTSSL */
    ret = perform_tls_handshake(&ssl);
    if (ret != 0) {
        goto exit;
    }

    /* Get response */
    ret = write_ssl_and_get_response(&ssl, (unsigned char *) buf, 0);
    VALIDATE_MBEDTLS_RETURN(ret, 200, 299, exit);
    ESP_LOGI(TAG, "Writing HELO to server...");

    len = snprintf((char *) buf, BUF_SIZE, "HELO %s\r\n", "ESP32");
    ret = write_ssl_and_get_response(&ssl, (unsigned char *) buf, len);
    VALIDATE_MBEDTLS_RETURN(ret, 200, 299, exit);

#endif /* SERVER_USES_STARTSSL */

    /* Authentication */
    ESP_LOGI(TAG, "Authentication...");

    ESP_LOGI(TAG, "Write AUTH LOGIN");
    len = snprintf( (char *) buf, BUF_SIZE, "AUTH LOGIN\r\n" );
    ret = write_ssl_and_get_response(&ssl, (unsigned char *) buf, len);
    VALIDATE_MBEDTLS_RETURN(ret, 200, 399, exit);

    ESP_LOGI(TAG, "Write USER NAME");
    ret = mbedtls_base64_encode((unsigned char *) base64_buffer, sizeof(base64_buffer),
                                &base64_len, (unsigned char *) SENDER_MAIL, strlen(SENDER_MAIL));
    if (ret != 0) {
        ESP_LOGE(TAG, "Error in mbedtls encode! ret = -0x%x", -ret);
        log_error("Error in mbedtls encode! ret = -0x%x", -ret);
        goto exit;
    }
    len = snprintf((char *) buf, BUF_SIZE, "%s\r\n", base64_buffer);
    ret = write_ssl_and_get_response(&ssl, (unsigned char *) buf, len);
    VALIDATE_MBEDTLS_RETURN(ret, 300, 399, exit);

    ESP_LOGI(TAG, "Write PASSWORD");
    ret = mbedtls_base64_encode((unsigned char *) base64_buffer, sizeof(base64_buffer),
                                &base64_len, (unsigned char *) SENDER_PASSWORD, strlen(SENDER_PASSWORD));
    if (ret != 0) {
        ESP_LOGE(TAG, "Error in mbedtls encode! ret = -0x%x", -ret);
        log_error("Error in mbedtls encode! ret = -0x%x", -ret);
        goto exit;
    }
    len = snprintf((char *) buf, BUF_SIZE, "%s\r\n", base64_buffer);
    ret = write_ssl_and_get_response(&ssl, (unsigned char *) buf, len);
    VALIDATE_MBEDTLS_RETURN(ret, 200, 399, exit);

	/* Compose email */
    ESP_LOGI(TAG, "Write MAIL FROM");
    len = snprintf((char *) buf, BUF_SIZE, "MAIL FROM:<%s>\r\n", SENDER_MAIL);
    ret = write_ssl_and_get_response(&ssl, (unsigned char *) buf, len);
    VALIDATE_MBEDTLS_RETURN(ret, 200, 299, exit);
    
    // Check chip ID to attach 
    getChipIdString(deviceSerial, sizeof(deviceSerial));
    vTaskDelay(pdMS_TO_TICKS(10));
	snprintf(Access_token, sizeof(Access_token), "%s%ld", deviceSerial, target_number);
	printf("Access Token: %s\n", Access_token);  

    /**
     * Select which type of notification to send
     * Controlled by flags
     */
    if (smtp_press_batt_warn_flag == 1) { // Pressure out or range / Battery low alerts
			/* Send RCPT TO for each recipient */
			ESP_LOGI(TAG, "Adding recipients...");
			for (int i = 0; recipients[i] != NULL; i++) {
			    len = snprintf((char *) buf, BUF_SIZE, "RCPT TO:<%s>\r\n", recipients[i]);
			    ret = write_ssl_and_get_response(&ssl, (unsigned char *) buf, len);
			    VALIDATE_MBEDTLS_RETURN(ret, 200, 299, exit);
			    ESP_LOGI(TAG, "Added recipient: %s", recipients[i]);
			}
		
		    ESP_LOGI(TAG, "Write DATA");
		    len = snprintf((char *) buf, BUF_SIZE, "DATA\r\n");
		    ret = write_ssl_and_get_response(&ssl, (unsigned char *) buf, len);
		    VALIDATE_MBEDTLS_RETURN(ret, 300, 399, exit);
		
		    ESP_LOGI(TAG, "Write Content");
		    /* We do not take action if message sending is partly failed. */
		
			// Build recipient list for "To:" header
			char to_header[512] = "";
			for (int i = 0; recipients[i] != NULL; i++) {
			    if (i > 0) strcat(to_header, ", ");
			    strcat(to_header, recipients[i]);
			}
			  
			if (Alarmtype == 0) {
	               if (target_number == 14 || target_number == 15) {
						len = snprintf((char *) buf, BUF_SIZE,
						               "From: %s\r\n"
						               "Subject: %s Temperature Alarm Activated!\r\n"
						               "To: %s\r\n"
						               "MIME-Version: 1.0 (mime-construct 1.9)\n",
						               "AKUH HMI SYSTEM ALARM", 
						               Sensor,
						               to_header);
	               } else {
						len = snprintf((char *) buf, BUF_SIZE,
						               "From: %s\r\n"
						               "Subject: %s Pressure Alarm Activated!\r\n"
						               "To: %s\r\n"
						               "MIME-Version: 1.0 (mime-construct 1.9)\n",
						               "AKUH HMI SYSTEM ALARM", 
						               Sensor,
						               to_header);
				   }							 											
				} else if (Alarmtype == 1) {
					len = snprintf((char *) buf, BUF_SIZE,
					               "From: %s\r\n"
					               "Subject: %s Battery Low Alarm Activated!\r\n"
					               "To: %s\r\n"
					               "MIME-Version: 1.0 (mime-construct 1.9)\n",
					               "AKUH HMI SYSTEM ALARM", 
					               Sensor,
					               to_header);		
				}	  
			 
		    /**
		     * Note: We are not validating return for some ssl_writes.
		     * If by chance, it's failed; at worst email will be incomplete!
		     */
		    ret = write_ssl_data(&ssl, (unsigned char *) buf, len);
		
		    /* Multipart boundary */
		    len = snprintf((char *) buf, BUF_SIZE,
		               "Content-Type: multipart/mixed; boundary=XYZabcd1234\r\n"
		               "\r\n"
		               "--XYZabcd1234\r\n");
		    ret = write_ssl_data(&ssl, (unsigned char *) buf, len);
		
		 /* Text message */
			time_t now = time(NULL);
			struct tm *timeinfo = localtime(&now);
			char timestamp[32];
			strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", timeinfo);
			
			if (Alarmtype == 0) {
				if (target_number == 14 || target_number == 15) {
					len = snprintf((char *) buf, BUF_SIZE,
					            "Content-Type: text/plain\r\n"
				                "\r\n"
				                "Device: %s\r\n"
				                "Serial No: %s\r\n"
				                "Device Status: CRITICAL ALARM ACTIVATED!\r\n"
				                "Timestamp: %s\r\n"
				                "\r\n"
								"ALERT: Temperature Out of Range!\r\n"			                			                
				                "CURRENT VALUE: %.2f\r\n" 
				                "Potential Cause: Door left open beyond acceptable duration\r\n"
				                "\r\n"
				                "Action Required:\r\n"
				                "1. IMMEDIATELY check %s door and ensure proper closure \r\n"
				                "2. If problem persists, contact the Engineering Team\r\n"
				                "3. Document incident in maintenance log with corrective action\r\n"
				                "\r\n"
					            "--XYZabcd1234\r\n",
					           Sensor,  		// Device name from global variable
					           deviceSerial, 	// ESP serial number
					           timestamp, 		// Current time
							   Temperature, // Trigger Value		
					           Sensor); 	
				} else {
					len = snprintf((char *) buf, BUF_SIZE,
					            "Content-Type: text/plain\r\n"
				                "\r\n"
				                "Device: %s\r\n"
				                "Serial No: %s\r\n"
				                "Device Status: CRITICAL ALARM ACTIVATED!\r\n"
				                "Timestamp: %s\r\n"
				                "\r\n"
								"ALERT: Differential Pressure Out of Range!\r\n"			                			                
				                "CURRENT VALUE: %.2f\r\n" 
				                "Potential Cause: Door left open beyond acceptable duration\r\n"
				                "\r\n"
				                "Action Required:\r\n"
				                "1. IMMEDIATELY check %s door and ensure proper closure \r\n"
				                "2. If problem persists, contact the Engineering Team\r\n"
				                "3. Document incident in maintenance log with corrective action\r\n"
				                "\r\n"
					            "--XYZabcd1234\r\n",
					           Sensor,  		// Device name from global variable
					           deviceSerial, 	// ESP serial number
					           timestamp, 		// Current time
							   Pressure, 	 // Trigger Value		
					           Sensor); 	
				}																			     		
			} else if (Alarmtype == 1) {
				if (g_board.type == BOARD_V2_MAX17055_FG) { 
						len = snprintf((char *) buf, BUF_SIZE,
						            "Content-Type: text/plain\r\n"
					                "\r\n"
					                "Device: %s\r\n"
					                "Serial No: %s\r\n"
					                "Device Status: BATTERY LOW ALARM ACTIVATED!\r\n"
									"Battery Soc: %.2f\r\n" 	
					                "Timestamp: %s\r\n"
					                "\r\n"
					                "Action Required:\r\n"
					                "1. IMMEDIATELY recharge %s Battery or replace it \r\n"
					                "2. Contact the HMI maintenance Team\r\n"
					                "3. Document incident in maintenance log with corrective action\r\n"
					                "\r\n"
						            "--XYZabcd1234\r\n",
						           Sensor,  // Device name from global variable
						           deviceSerial, // ESP serial number
								   Soc, //  Updated with Battery Soc					           					            
						           timestamp,
						           Sensor);  // Current time 
						           
					Alarmtype = 0;
				} else {
						len = snprintf((char *) buf, BUF_SIZE,
						            "Content-Type: text/plain\r\n"
					                "\r\n"
					                "Device: %s\r\n"
					                "Serial No: %s\r\n"
					                "Device Status: BATTERY LOW ALARM ACTIVATED!\r\n"
									"Sampled Voltage: %.2f\r\n" 	
					                "Timestamp: %s\r\n"
					                "\r\n"
					                "Action Required:\r\n"
					                "1. IMMEDIATELY recharge %s Battery or replace it \r\n"
					                "2. Contact the HMI maintenance Team\r\n"
					                "3. Document incident in maintenance log with corrective action\r\n"
					                "\r\n"
						            "--XYZabcd1234\r\n",
						           Sensor,  // Device name from global variable
						           deviceSerial, // ESP serial number
								   Voltage, // Sampled Voltage					           					            
						           timestamp,
						           Sensor);  // Current time 
						           
					Alarmtype = 0;
				}														
			}	
		
		    ret = write_ssl_data(&ssl, (unsigned char *) buf, len);
		
		    len = snprintf((char *) buf, BUF_SIZE, "\r\n.\r\n");
		    ret = write_ssl_and_get_response(&ssl, (unsigned char *) buf, len);
		    VALIDATE_MBEDTLS_RETURN(ret, 200, 299, exit);
		    ESP_LOGI(TAG, "Email sent!");
		
		    /* Close connection */
		    mbedtls_ssl_close_notify(&ssl);
		    ret = 0; /* No errors */
		    return;
    } else if (smtp_error_logs_flag == 1) { // Weekly error logs
			/* Send RCPT TO for each recipient */
			ESP_LOGI(TAG, "Adding recipients...");
			for (int i = 0; recipients[i] != NULL; i++) {
			    len = snprintf((char *) buf, BUF_SIZE, "RCPT TO:<%s>\r\n", recipients[i]);
			    ret = write_ssl_and_get_response(&ssl, (unsigned char *) buf, len);
			    VALIDATE_MBEDTLS_RETURN(ret, 200, 299, exit);
			    ESP_LOGI(TAG, "Added recipient: %s", recipients[i]);
			}
		
		    ESP_LOGI(TAG, "Write DATA");
		    len = snprintf((char *) buf, BUF_SIZE, "DATA\r\n");
		    ret = write_ssl_and_get_response(&ssl, (unsigned char *) buf, len);
		    VALIDATE_MBEDTLS_RETURN(ret, 300, 399, exit);
		
		    ESP_LOGI(TAG, "Write Content");
		    /* We do not take action if message sending is partly failed. */
		                   
			// Build recipient list for "To:" header
			char to_header[512] = "";
			for (int i = 0; recipients[i] != NULL; i++) {
			    if (i > 0) strcat(to_header, ", ");
			    strcat(to_header, recipients[i]);
			}
			             
			len = snprintf((char *) buf, BUF_SIZE,
			               "From: %s\r\n"
			               "Subject: %s Error Logs\r\n"
			               "To: %s\r\n"
			               "MIME-Version: 1.0 (mime-construct 1.9)\n",
			               "AKUH HMI SYSTEM", 
			               Sensor,
			               to_header);
		                   
		    /**
		     * Note: We are not validating return for some ssl_writes.
		     * If by chance, it's failed; at worst email will be incomplete!
		     */
		    ret = write_ssl_data(&ssl, (unsigned char *) buf, len);
		
		    /* Multipart boundary */
		    len = snprintf((char *) buf, BUF_SIZE,
		               "Content-Type: multipart/mixed; boundary=XYZabcd1234\r\n"
		               "\r\n"
		               "--XYZabcd1234\r\n");
		    ret = write_ssl_data(&ssl, (unsigned char *) buf, len);
		
		 /* Text message */
			time_t now = time(NULL);
			struct tm *timeinfo = localtime(&now);
			char timestamp[32];
			strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", timeinfo);
			len = snprintf((char *) buf, BUF_SIZE,
			           "Content-Type: text/plain\r\n"
			           "\r\n"
			           "HMI System Alert Notification\r\n"
			           "\r\n"
			           "Device: %s\r\n"
			           "Serial No: %s\r\n"
			           "Device Status: Active\r\n"
			           "Institution: Aga Khan University Hospital\r\n"
			           "Timestamp: %s\r\n"
			           "\r\n"
			           "Error logs are attached for your review.\r\n"
			           "This is an automated message. Do not reply.\r\n"
			           "\r\n"
			           "--XYZabcd1234\r\n",
			           Sensor,  	 // Device name from global variable
			           deviceSerial, // ESP serial number
			           timestamp);   // Current time               
		   
		    ret = write_ssl_data(&ssl, (unsigned char *) buf, len);
		
		    /* Attachment */ 
		    len = snprintf((char *) buf, BUF_SIZE,
		               "Content-Type: text/plain\r\n"
		               "Content-Transfer-Encoding: base64\r\n"
		               "Content-Disposition: attachment; filename=\"elogs.txt\"\r\n"
		               "\r\n");
		    ret = write_ssl_data(&ssl, (unsigned char *) buf, len);
		
		    /* File contents in base64 */
		    // Read file contents
		    const char *path = "/spiffs/elogs.txt";
		    char *file_data = read_file(path);
		
		    //char *file_data = read_file(path);
		    if (file_data) {
		        size_t file_len = strlen(file_data);
		        size_t base64_len;
		        size_t chunk_size = 54; // Base64 chunks (72 chars is standard but some servers prefer smaller)
		        
		        for (size_t i = 0; i < file_len; i += chunk_size) {
		            size_t remaining = file_len - i;
		            size_t encode_size = (remaining > chunk_size) ? chunk_size : remaining;
		            
		            ret = mbedtls_base64_encode((unsigned char *)base64_buffer, sizeof(base64_buffer),
		                                    &base64_len, (unsigned char *)file_data + i, encode_size);
		            if (ret != 0) {
		                ESP_LOGE(TAG, "Error in mbedtls encode! ret = -0x%x", -ret);
		                log_error("Error in mbedtls encode! ret = -0x%x", -ret);
		                free(file_data);
		                goto exit;
		            }
		            
		            len = snprintf((char *)buf, BUF_SIZE, "%s\r\n", base64_buffer);
		            ret = write_ssl_data(&ssl, (unsigned char *)buf, len);
		        }
		        free(file_data);
		    } else {
		        ESP_LOGE(TAG, "Failed to read file for attachment");
		        log_error("Failed to read txt file for attachment");
		        goto exit;
		    }
		
		    len = snprintf((char *) buf, BUF_SIZE, "\r\n.\r\n");
		    ret = write_ssl_and_get_response(&ssl, (unsigned char *) buf, len);
		    VALIDATE_MBEDTLS_RETURN(ret, 200, 299, exit);
		    ESP_LOGI(TAG, "Email sent!");
		
		    // Clear txt file after transmitting info
			clear_file(path);
		    // Unmount SPIFFS (optional)
		    esp_vfs_spiffs_unregister(NULL);
		    ESP_LOGI(TAG, "SPIFFS unmounted");
		
		    /* Close connection */
		    mbedtls_ssl_close_notify(&ssl);
		    ret = 0; /* No errors */
		    return;
    }

exit:
    mbedtls_net_free(&server_fd);
    mbedtls_x509_crt_free(&cacert);
    mbedtls_ssl_free(&ssl);
    mbedtls_ssl_config_free(&conf);
    mbedtls_ctr_drbg_free(&ctr_drbg);
    mbedtls_entropy_free(&entropy);

    if (ret != 0) {
        mbedtls_strerror(ret, buf, 100);
        ESP_LOGE(TAG, "Last error was: -0x%x - %s", -ret, buf);
        log_error("Last error was: -0x%x - %s", -ret, buf);
    }

    putchar('\n'); /* Just a new line */
    if (buf) {
        free(buf);
    }
    return;
}

static int write_and_get_response(mbedtls_net_context *sock_fd, unsigned char *buf, size_t len)
{
    int ret;
    const size_t DATA_SIZE = 128;
    unsigned char data[DATA_SIZE];
    char code[4];
    size_t i, idx = 0;

    if (len) {
        ESP_LOGD(TAG, "%s", buf);
    }

    if (len && (ret = mbedtls_net_send(sock_fd, buf, len)) <= 0) {
        ESP_LOGE(TAG, "mbedtls_net_send failed with error -0x%x", -ret);
        log_error("mbedtls_net_send failed with error -0x%x", -ret);
        return ret;
    }

    do {
        len = DATA_SIZE - 1;
        memset(data, 0, DATA_SIZE);
        ret = mbedtls_net_recv(sock_fd, data, len);

        if (ret <= 0) {
            ESP_LOGE(TAG, "mbedtls_net_recv failed with error -0x%x", -ret);
            log_error("mbedtls_net_recv failed with error -0x%x", -ret);
            goto exit;
        }

        data[len] = '\0';
        printf("\n%s", data);
        len = ret;
        for (i = 0; i < len; i++) {
            if (data[i] != '\n') {
                if (idx < 4) {
                    code[idx++] = data[i];
                }
                continue;
            }

            if (idx == 4 && code[0] >= '0' && code[0] <= '9' && code[3] == ' ') {
                code[3] = '\0';
                ret = atoi(code);
                goto exit;
            }

            idx = 0;
        }
    } while (1);

exit:
    return ret;
}

static int write_ssl_and_get_response(mbedtls_ssl_context *ssl, unsigned char *buf, size_t len)
{
    int ret;
    const size_t DATA_SIZE = 128;
    unsigned char data[DATA_SIZE];
    char code[4];
    size_t i, idx = 0;

    if (len) {
        ESP_LOGD(TAG, "%s", buf);
    }

    while (len && (ret = mbedtls_ssl_write(ssl, buf, len)) <= 0) {
        if (ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE) {
            ESP_LOGE(TAG, "mbedtls_ssl_write failed with error -0x%x", -ret);
            log_error("mbedtls_ssl_write failed with error -0x%x", -ret);
            goto exit;
        }
    }

    do {
        len = DATA_SIZE - 1;
        memset(data, 0, DATA_SIZE);
        ret = mbedtls_ssl_read(ssl, data, len);

        if (ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE) {
            continue;
        }

        if (ret <= 0) {
            ESP_LOGE(TAG, "mbedtls_ssl_read failed with error -0x%x", -ret);
            log_error("mbedtls_ssl_read failed with error -0x%x", -ret);
            goto exit;
        }

        ESP_LOGD(TAG, "%s", data);

        len = ret;
        for (i = 0; i < len; i++) {
            if (data[i] != '\n') {
                if (idx < 4) {
                    code[idx++] = data[i];
                }
                continue;
            }

            if (idx == 4 && code[0] >= '0' && code[0] <= '9' && code[3] == ' ') {
                code[3] = '\0';
                ret = atoi(code);
                goto exit;
            }

            idx = 0;
        }
    } while (1);

exit:
    return ret;
}

static int write_ssl_data(mbedtls_ssl_context *ssl, unsigned char *buf, size_t len)
{
    int ret;

    if (len) {
        ESP_LOGD(TAG, "%s", buf);
    }

    while (len && (ret = mbedtls_ssl_write(ssl, buf, len)) <= 0) {
        if (ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE) {
            ESP_LOGE(TAG, "mbedtls_ssl_write failed with error -0x%x", -ret);
            log_error("mbedtls_ssl_write failed with error -0x%x", -ret);
            return ret;
        }
    }

    return 0;
}

static int perform_tls_handshake(mbedtls_ssl_context *ssl)
{
    int ret = -1;
    uint32_t flags;
    char *buf = NULL;
    buf = (char *) calloc(1, BUF_SIZE);
    if (buf == NULL) {
        ESP_LOGE(TAG, "calloc failed for size %d", BUF_SIZE);
        log_error("perform_tls_handshake calloc failed for size %d", BUF_SIZE);
        goto exit;
    }

    ESP_LOGI(TAG, "Performing the SSL/TLS handshake...");

    fflush(stdout);
    while ((ret = mbedtls_ssl_handshake(ssl)) != 0) {
        if (ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE) {
            ESP_LOGE(TAG, "mbedtls_ssl_handshake returned -0x%x", -ret);
            log_error("mbedtls_ssl_handshake returned -0x%x", -ret);
            goto exit;
        }
    }

    ESP_LOGI(TAG, "Verifying peer X.509 certificate...");

    if ((flags = mbedtls_ssl_get_verify_result(ssl)) != 0) {
        /* In real life, we probably want to close connection if ret != 0 */
        ESP_LOGW(TAG, "Failed to verify peer certificate!");
        log_error("Failed to verify peer certificate!");
        mbedtls_x509_crt_verify_info(buf, BUF_SIZE, "  ! ", flags);
        ESP_LOGW(TAG, "verification info: %s", buf);
        log_error("verification info: %s", buf);
    } else {
        ESP_LOGI(TAG, "Certificate verified.");
    }

    ESP_LOGI(TAG, "Cipher suite is %s", mbedtls_ssl_get_ciphersuite(ssl));
    ret = 0; /* No error */

exit:
    if (buf) {
        free(buf);
    }
    return ret;
}

// Load last SMTP time from NVS
time_t load_last_smtp_time_from_nvs() {
    nvs_handle_t my_handle;
    time_t lastSmtpTime = 0; // Default value if not found in NVS
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);

    if (err == ESP_OK) {
        err = nvs_get_i64(my_handle, "last_smtp_time", &lastSmtpTime);
        if (err == ESP_OK) {
            printf("Loaded last SMTP time: %s", ctime(&lastSmtpTime));
        } else {
            ESP_LOGW("SMTP", "No last SMTP time found in NVS, using default: 0");
            log_error("No Last SMTP Time found in NVS, using default: 0");
        }
        nvs_close(my_handle);
    } else {
        ESP_LOGE("SMTP", "Error opening NVS handle for reading!");
        log_error("Error opening NVS handle for reading Last SMTP Time!");
    }

    return lastSmtpTime;
}

// Save last SMTP time to NVS
void save_last_smtp_time_to_nvs(time_t lastSmtpTime) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);

    if (err == ESP_OK) {
        err = nvs_set_i64(my_handle, "last_smtp_time", lastSmtpTime);
        if (err == ESP_OK) {
            nvs_commit(my_handle);
            printf("Saved last SMTP time: %s", ctime(&lastSmtpTime));
        } else {
            ESP_LOGE("SMTP", "Failed to store last SMTP time!");
            log_error("Failed to store Last SMTP Time!");
        }
        nvs_close(my_handle);
    } else {
        ESP_LOGE("SMTP", "Error opening NVS handle for writing!");
        log_error("Error opening NVS handle for writing Last SMTP Time!");
    }
}

/**********************************************************************************************************************************/


/**********************************************************************************************************************************
FUNCTIONS: MAIN APPLICATION 
**************************/
void powerOnSequence() { 
    printAsteriskSeparator(100);
    printf("HMI System Firmware Started\n");
    setupWatchdog();
    vTaskDelay(pdMS_TO_TICKS(50));
    printAsteriskSeparator(100);
    printf("Power-On Sequence started...\n"); 
    vTaskDelay(pdMS_TO_TICKS(50));
//------------------------------------------------------------------------------------------------			  
    //Initialize PWM Buzzer & UART0
    setupPWMForBuzzer();  
    setupPWMForLED();
    vTaskDelay(pdMS_TO_TICKS(50));
    serial_uart_init();
    vTaskDelay(pdMS_TO_TICKS(50));			
 
//------------------------------------------------------------------------------------------------			 
    vTaskDelay(pdMS_TO_TICKS(50));
    printf("HMI Power-On Sequence done!...\n"); 
    printAsteriskSeparator(100);
    }

void initializeSystem() { 
    printAsteriskSeparator(100);
    printf("Initializing HMI System...\n"); 
//------------------------------------------------------------------------------------------------	
    // Initialize Interrupt Button
    initializeTimer();
    setupButton();
    initialize_spiffs();
    nvs_init();
    board_init(); 
	 if (PowerOnTone == 0) {
				  PowerOnTone = 1;
				  vTaskDelay(pdMS_TO_TICKS(10));
				  blink_green_yellow(4);
				  playPowerOnTone(LEDC_CHANNEL_BUZZER, LEDC_TIMER);		  
				  vTaskDelay(pdMS_TO_TICKS(10));
	 } else if (PowerOnTone == 1){}	  
    vTaskDelay(pdMS_TO_TICKS(50));
//------------------------------------------------------------------------------------------------				  
    // Initialize Voltage Monitor
    if (g_board.type != BOARD_V2_MAX17055_FG) {
		setup_pins();
	}    
    EPD_GPIOInit();
    load_target_number_from_nvs();
//------------------------------------------------------------------------------------------------		
    vTaskDelay(pdMS_TO_TICKS(50));
    printAsteriskSeparator(100);
    }

void connectWiFi() { 
    printAsteriskSeparator(100);
    // Check chip ID to attach 
    getChipIdString(deviceSerial, sizeof(deviceSerial)); 
    vTaskDelay(pdMS_TO_TICKS(10));
	snprintf(Access_token, sizeof(Access_token), "%s%ld", deviceSerial, target_number);
	printf("Access Token: %s\n", Access_token);  
    vTaskDelay(pdMS_TO_TICKS(10));  
    printf("HMI Initiating WiFi connection...\n"); 
    // connect to wifi function
	wifi_connection();

    vTaskDelay(pdMS_TO_TICKS(50));
    printAsteriskSeparator(100);
    }

void DisconnectWiFi() { 
    printAsteriskSeparator(100);

    printf("HMI Initiating WiFi disconnection...\n"); 

	wifi_disconnect();

    vTaskDelay(pdMS_TO_TICKS(50));
    printAsteriskSeparator(100);
    }

void UpdateEPD() { 
    printAsteriskSeparator(100);

    //printf("HMI Initiating Aranet Get and Updating EPD...\n"); 
    
    // Power on and initialize EPD
    epd_power_on();
    vTaskDelay(pdMS_TO_TICKS(50));
    // Draw main UI
    clear_all();
    vTaskDelay(pdMS_TO_TICKS(50));
    displayui();
    EPD_Display_Part(0, 0, EPD_W, EPD_H, Image_BW);
    vTaskDelay(pdMS_TO_TICKS(50));
    // partial update
	//get_aranet_data();
	//vTaskDelay(pdMS_TO_TICKS(50));
	//updateBatteryLevel();
	//vTaskDelay(pdMS_TO_TICKS(50));
  //  EPD_Partial_Data_Update(); // TODO
  //  EPD_Display_Part(0, 0, EPD_W, EPD_H, Image_BW);
	// send metadata to thingsboard
	// post_device_metadata();
	// send battery data
  //   if (g_board.type == BOARD_V2_MAX17055_FG) {  
	// 	post_battery_data();
	// }
	epd_power_off();
	vTaskDelay(pdMS_TO_TICKS(100)); 
    printAsteriskSeparator(100);
  }
  
void CheckBatteryLevel() { 
    printAsteriskSeparator(100);

	// Check Battery Level in intervals
	time_t now;
	time(&now);
	if (firstBoot == 0) {
	    lastMonitorTime = now;
	    firstBoot = 1;
	    vTaskDelay(pdMS_TO_TICKS(50));
	    EPD_display_Battery_Low();
	} else if (battlow == 1) { // If flag is 1 keep showing batt low ui
	    vTaskDelay(pdMS_TO_TICKS(50));
	    EPD_display_Battery_Low();	
	}	
	printf("Last Monitor: %s", ctime(&lastMonitorTime));
	vTaskDelay(pdMS_TO_TICKS(50));	              
	if ((now - lastMonitorTime) >= MONITOR_INTERVAL) {
	    EPD_display_Battery_Low();
	    lastMonitorTime = now;                             
	}

    vTaskDelay(50 / portTICK_PERIOD_MS);
    epd_power_off();
    
    printAsteriskSeparator(100);
	}
		
void deepsleep() {
    printAsteriskSeparator(100);
    esp_sleep_enable_ext0_wakeup(BUTTON_PIN, 0); // Enable wake-up via button press (LOW = wake)
    printf("Entering deep sleep for 10 minutes...\n");
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_DURATION_US); // Enable timer wakeup
    vTaskDelay(pdMS_TO_TICKS(50));
    // ---- Deinitialize peripherals ----
    if (watchdogTimer) { // Stop Watchdog
        xTimerStop(watchdogTimer, 0);
        xTimerDelete(watchdogTimer, 0);
        watchdogTimer = NULL;
        printf("WDT Stopped.\n");
    }
    max17055_save_learned();
    vTaskDelay(pdMS_TO_TICKS(100));
	i2c_master_deinit(); // Delete I2C
    
    printf("uart driver uninstalled.\n");
    if (adc_handle) { // Uninstall ADC
        adc_oneshot_del_unit(adc_handle);
        adc_handle = NULL;
    }
    gpio_reset_pin(SENSE_PIN);
    ledc_stop(LEDC_MODE, LEDC_CHANNEL_BUZZER, 0); // Uninstall PWM buzzer
    ledc_fade_func_uninstall();
    printf("LEDC uninstalled.\n");
    gpio_reset_pin(BUZZER_PIN);
    //EPD_SPIDeinit(); // Uninstall EPD SPI // deinitialization not smooth
    uart_driver_delete(UART_NUM); // Uninstall UART driver
    // ---- Enter Deep Sleep ----
    esp_deep_sleep_start();
}

void SNTP() { 
    printAsteriskSeparator(100);

	if (wifi_connected && !networkError) {
	    printf("HMI Initiating SNTP...\n"); 
		// Sync time SNTP
	    if (sync_time_from_sntp()) {
			print_current_time();
	    }
	  } else {
		 ESP_LOGE(TAG, "No Wifi Connection, Skipping HMI SNTP Task...\n"); 
		 log_error("No Wifi Connection, Skipping HMI SNTP Task...");
	  }

    vTaskDelay(pdMS_TO_TICKS(50));
    printAsteriskSeparator(100);
    }
    
void SMTP() { 
    printAsteriskSeparator(100);

  	if (wifi_connected && !networkError) {
	    printf("HMI Initiating SMTP...\n"); 
	    if (g_board.type == BOARD_V2_MAX17055_FG) {  
			read_batt_params();
		} else {
			readBatteryVoltage();
		}		
		printAsteriskSeparator(100);
		smtp_error_logs_flag = 1;
		vTaskDelay(pdMS_TO_TICKS(50));
	    // Send weekly error logs
		smtp_client_task(); 
		smtp_error_logs_flag = 0;
		vTaskDelay(pdMS_TO_TICKS(50));
	  } else {
		 ESP_LOGE(TAG, "No Wifi Connection, Skipping HMI SMTP Task...\n"); 
		 log_error("No Wifi Connection, Skipping HMI SMTP Task...\n");
	  }

    vTaskDelay(pdMS_TO_TICKS(50));
    printAsteriskSeparator(100);
    }
    
void OTA() { 
    printAsteriskSeparator(100);
    // Identify running partition
    running_partition();
    // Check if there is an ota update after every 1 hour
	time_t now;
	time(&now);
	if (firstBoot == 0) {
	    lastOtaTime = now; 
	    ota_flag = true;
	    vTaskDelay(pdMS_TO_TICKS(50));         
        ESP_LOGI(TAG, "OTA flag triggered, starting OTA update...");
        perform_ota();
        while (ota_flag) {
			vTaskDelay(pdMS_TO_TICKS(100));
		} 
	}
	if ((now - lastOtaTime) >= OTA_INTERVAL) {	
	    lastOtaTime = now; 
	    ota_flag = true;
	    vTaskDelay(pdMS_TO_TICKS(50));         
        ESP_LOGI(TAG, "OTA flag triggered, starting OTA update...");
        perform_ota();	 
        while (ota_flag) {
			vTaskDelay(pdMS_TO_TICKS(100));
		}                               
	}

    vTaskDelay(pdMS_TO_TICKS(50));
    printAsteriskSeparator(100);
    }
/*************************************************************************************************************************************/ 


/**************************************************************************************************************************************
FUNCTIONS: BUTTON INTERRUPT
**************************/ 
void resetDevice() { 
      printAsteriskSeparator(100);
      printf("Resetting HMI Device...\n");
      vTaskDelay(pdMS_TO_TICKS(50));
      esp_restart();
      printAsteriskSeparator(100);
    }

void Pages(int count) { 
      printAsteriskSeparator(100);
      vTaskDelay(pdMS_TO_TICKS(50));
      if (count == 2) {
		  handle_next_target_selection(); // Set next target page and restart
	  } else if (count == 3) {
		  handle_previous_target_selection(); // Set next target page and restart
	  }     
      vTaskDelay(pdMS_TO_TICKS(50));
      esp_restart();
      vTaskDelay(pdMS_TO_TICKS(50));
      printAsteriskSeparator(100);
    }
  
void shutdownDevice() { 
                printAsteriskSeparator(100);
                printf("Shutting Down HMI Device...\n");
                powerdown();
                vTaskDelay(pdMS_TO_TICKS(10));
                printAsteriskSeparator(100);
				 }
/************************************************************************************************************************************** 
PROJECT: AGA KHAM HMI DEVICE  	COMPANY: GEVITON ENTERPRISES LIMITED 	DEVELOPER: ERIC MULWA
***************************************************************************************************************************************/