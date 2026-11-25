/*
 * ============================================================================
 * GEVITON SMART TAG — Main Application Firmware  (Rev 6)
 * Target: ESP32-S3 | ESP-IDF
 * ============================================================================
 *
 * HARDWARE SUMMARY
 * ─────────────────────────────────────────────────────────────────────────────
 *  EPD Display     : 400×300 px, 1-bit monochrome, SPI
 *  SD Card         : SPI3_HOST  MOSI=40  MISO=13  SCLK=39  CS=10
 *  EPD Power       : GPIO 7   (active HIGH)
 *  TF/SD Power     : GPIO 42  (active HIGH)
 *  LED             : GPIO 41
 *  Buzzer          : GPIO 14
 *  I2C SDA         : GPIO 8   (MAX17055 fuel gauge)
 *  I2C SCL         : GPIO 9
 *
 * BUTTONS  (active LOW, internal pull-up)
 * ─────────────────────────────────────────────────────────────────────────────
 *  UP     : GPIO 4
 *  DOWN   : GPIO 6
 *  SELECT : GPIO 5
 *  BACK   : GPIO 1
 *  MENU   : GPIO 2  ← deep-sleep wake source (EXT0, active LOW)
 *                     short press  → esp_restart()
 *                     long press (≥5 s) → hardware shutdown (latch off)
 *                     wakes device from deep sleep
 *
 * POWER / SLEEP ARCHITECTURE
 * ─────────────────────────────────────────────────────────────────────────────
 *  The hardware latch (GPIO 3) is no longer used for normal shutdown.
 *  Deep sleep is used instead — the EPD retains its image with zero power.
 *
 *  Idle timeout  : 60 s of no button activity → enter deep sleep.
 *  Battery low   : SOC < 10 % → show ui_battery_low_ui → deep sleep.
 *  Long-press    : MENU held ≥ 5 s → drive GPIO 3 HIGH to cut latch power.
 *
 *  Deep-sleep wake:
 *    MENU button (GPIO 2) is configured as EXT0 wake source (LOW level).
 *    Wake is treated as a clean power-up — no state is preserved across sleep.
 *    The device always boots fresh from STATE_BOOT.
 *
 * BATTERY MONITORING  (MAX17055 over I2C)
 * ─────────────────────────────────────────────────────────────────────────────
 *  SOC is sampled at boot, on every state transition, and after every
 *  navigation action (folder enter, go back, file select).
 *
 *  Hysteresis thresholds prevent oscillation near the low-battery boundary:
 *    Sleep threshold  : SOC < 10 % → show battery_low screen + deep sleep.
 *    Resume threshold : SOC ≥ 15 % → exit battery_low, allow normal operation.
 *
 *  If the fuel gauge fails to initialise, battery_get_soc() returns a safe
 *  default (100 %) and all battery-triggered actions are suppressed.
 *
 * IMAGE BUFFER GUARD
 * ─────────────────────────────────────────────────────────────────────────────
 *  IMAGE_GUARD_BYTES bytes are appended to g_image_bw and g_loaded_image and
 *  filled with 0xAA before every image operation. If the pattern is disturbed
 *  after the operation, a buffer overrun is logged and the operation fails.
 *
 * IMAGE FORMAT
 * ─────────────────────────────────────────────────────────────────────────────
 *  Extension   : .bin
 *  Size        : 15 000 bytes exactly  (400 × 300 / 8)
 *  Resolution  : 400 × 300
 *  Bit depth   : 1-bit monochrome, plain bytes
 *  Tool        : image2cpp — Horizontal, 1-bit per pixel, Plain Bytes
 * ============================================================================
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <sys/stat.h>
#include <stdbool.h>
#include <strings.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "EPD_GUI.h"
#include "EPD_SPI.h"
#include "EPD.h"
#include "pic_scenario.h"
#include "max17055.h"

static const char *TAG = "SMART_TAG";

/* ============================================================================
 * PIN DEFINITIONS
 * ========================================================================== */
#define SD_CS_PIN           10
#define EPD_POWER_PIN        7
#define TF_POWER_PIN        42
#define POWER_OFF_PIN        3    /* strapping pin — drive HIGH only for latch-off */
#define LED_PIN             41
#define BUZZER_PIN          14

#define BTN_UP               4
#define BTN_DOWN             6
#define BTN_SELECT           5
#define BTN_BACK             1
#define BTN_MENU             2    /* EXT0 deep-sleep wake source */

/* ============================================================================
 * FILESYSTEM / BROWSER CONSTANTS
 * ========================================================================== */
#define MOUNT_POINT          "/sdcard"
#define MAX_ENTRIES           64
#define MAX_NAME_LEN          64
#define MAX_STACK_DEPTH        8
#define IMAGE_SIZE          15000

/* ============================================================================
 * TIMING
 * ========================================================================== */
#define IDLE_TIMEOUT_MS     (60 * 1000)   /* 60 s idle → deep sleep            */
#define MENU_LONGPRESS_MS   5000          /* 5 s MENU hold → hardware shutdown  */

/* ============================================================================
 * BATTERY HYSTERESIS THRESHOLDS
 *
 *   BATTERY_SLEEP_SOC  : enter battery-low sleep when SOC falls below this.
 *   BATTERY_RESUME_SOC : exit battery-low state only when SOC rises above this.
 *
 * Example:
 *   SOC =  9 % → battery-low sleep
 *   SOC = 12 % → remains in battery-low (between thresholds)
 *   SOC = 15 % → resumes normal operation
 * ========================================================================== */
#define BATTERY_SLEEP_SOC    10.0f
#define BATTERY_RESUME_SOC   15.0f

/* ============================================================================
 * DIRECTORY VIEW LAYOUT  (must match artwork)
 * ========================================================================== */
#define ITEMS_PER_PAGE    8
#define LIST_Y_ORIGIN    44
#define LIST_ROW_HEIGHT  28
#define LIST_X_ICON       8
#define LIST_X_TEXT      26
#define FONT_SIZE        16
#define DISPLAY_NAME_MAX  30

/* ============================================================================
 * APPLICATION STATES
 * ========================================================================== */
typedef enum {
    STATE_BOOT = 0,
    STATE_NO_SD,
    STATE_MAIN_MENU,
    STATE_BROWSING,
    STATE_IMAGE_VIEW,
    STATE_BATTERY_LOW,   /* shown before deep sleep on low battery */
    STATE_DEEP_SLEEP     /* triggers the deep-sleep sequence        */
} app_state_t;

/* ============================================================================
 * DATA STRUCTURES
 * ========================================================================== */
typedef enum {
    ENTRY_FILE   = 0,
    ENTRY_FOLDER = 1
} entry_type_t;

typedef struct {
    char         name[MAX_NAME_LEN];
    entry_type_t type;
} dir_entry_t;

typedef struct {
    dir_entry_t entries[MAX_ENTRIES];
    int         count;
    int         selected;
    int         scroll_offset;
} dir_listing_t;

typedef struct {
    char path[256];
    int  selected;
    int  scroll_offset;
} nav_frame_t;

/* ============================================================================
 * IMAGE BUFFER GUARD
 *
 * IMAGE_GUARD_BYTES are appended to each image buffer and filled with 0xAA
 * before image operations. After the operation, guard_check_*() verifies the
 * pattern is undisturbed. Any modification indicates a buffer overrun.
 * ========================================================================== */
#define IMAGE_GUARD_BYTES  64
#define IMAGE_GUARD_FILL   0xAAu

static uint8_t g_image_bw    [IMAGE_SIZE + IMAGE_GUARD_BYTES];
static uint8_t g_loaded_image[IMAGE_SIZE + IMAGE_GUARD_BYTES];

/* Arm the guard region before an image operation. */
static void guard_arm(uint8_t *buf)
{
    memset(buf + IMAGE_SIZE, IMAGE_GUARD_FILL, IMAGE_GUARD_BYTES);
}

/* Verify the guard region after an image operation.
 * Returns true if intact, false if corruption detected. */
static bool guard_check(const uint8_t *buf, const char *label)
{
    for (int i = 0; i < IMAGE_GUARD_BYTES; i++) {
        if (buf[IMAGE_SIZE + i] != IMAGE_GUARD_FILL) {
            ESP_LOGE(TAG, "Buffer overrun detected in %s at guard byte %d", label, i);
            return false;
        }
    }
    return true;
}

/* ============================================================================
 * GLOBAL STATE
 * ========================================================================== */
static app_state_t   g_state        = STATE_BOOT;
static bool          g_screen_dirty = true;
static bool          g_sd_mounted   = false;
static sdmmc_card_t *g_sd_card      = NULL;

/* Navigation stack */
static nav_frame_t   g_nav_stack[MAX_STACK_DEPTH];
static int           g_nav_depth   = 0;
static dir_listing_t g_dir;
static char          g_current_path[256] = MOUNT_POINT;

/* Idle timer */
static TimerHandle_t g_idle_timer = NULL;

/*
 * g_sleep_requested — set only by the idle timer callback (FreeRTOS timer
 * task). The main loop polls this flag and calls enter_deep_sleep() from the
 * application task, avoiding concurrent state mutation.
 */
static volatile bool g_sleep_requested = false;

/* ============================================================================
 * PERIPHERAL INITIALISATION TRACKING
 *
 * Prevent double-init / double-deinit which can cause runtime errors
 * (e.g. calling i2c_driver_delete on an uninstalled port).
 * ========================================================================== */
static bool g_i2c_initialized = false;
static bool g_sd_initialized  = false;

/* ============================================================================
 * I2C / FUEL GAUGE STATE
 * ========================================================================== */
#define I2C_PORT              I2C_NUM_0
#define I2C_SCL_GPIO          9
#define I2C_SDA_GPIO          8
#define I2C_FREQ_HZ           100000
#define MAX17055_ADDR         0x36u

static max17055_ctx_t            fg;
static max17055_learned_params_t g_learned          = {0};
static bool                      g_learned_valid     = false;
static uint16_t                  g_cycles_last_saved = 0;
static float                     g_soc               = 100.0f;

/*
 * g_fuel_gauge_ok — set true only after MAX17055 init succeeds.
 * If false, battery_get_soc() returns 100.0 (safe default) and
 * battery_check_and_handle() skips all battery-triggered actions.
 */
static bool g_fuel_gauge_ok = false;

/*
 * g_battery_low_active — hysteresis latch.
 * True while SOC is below BATTERY_RESUME_SOC; cleared once SOC recovers.
 */
static bool g_battery_low_active = false;

/* ============================================================================
 * FORWARD DECLARATIONS
 * ========================================================================== */

/* Hardware init */
static void        buttons_init(void);
static void        power_pins_init(void);

/* Power */
static void        epd_power_on(void);
static void        epd_power_off(void);
static void        tf_power_on(void);
static void        tf_power_off(void);

/* Deep sleep / shutdown */
static void        enter_deep_sleep(void);   /* does not return */
static void        hardware_shutdown(void);  /* does not return */

/* Battery */
static esp_err_t   i2c_master_init(void);
static esp_err_t   i2c_master_deinit(void);
static esp_err_t   battery_init(void);
static esp_err_t   load_learned_params_from_nvs(void);
static esp_err_t   save_learned_params_to_nvs(void);
static int32_t     platform_i2c_read(void *handle, uint8_t reg, uint8_t *data, uint16_t len);
static int32_t     platform_i2c_write(void *handle, uint8_t reg, uint8_t *data, uint16_t len);
static void        max17055_save_learned(void);
static float       battery_get_soc(void);
static void        battery_check_and_handle(void);

/* SD / filesystem */
static bool        sd_init(void);
static bool        sd_deinit(void);
static bool        load_images(const char *full_filepath);
static bool        scan_directory(const char *path, dir_listing_t *listing);
static bool        is_bin_file(const char *name);
static bool        is_system_entry(const char *name);
static int         entry_compare(const void *a, const void *b);

/* Navigation — all return true on success, false on failure */
static bool        nav_push(const char *path, int selected, int scroll);
static bool        nav_pop(void);
static bool        nav_enter_folder(const char *folder_name);
static bool        nav_go_back(void);
static void        nav_select_item(void);

/* Display */
static void        clear_all(void);
static void        displayui_screen(const uint8_t *ui_data);
static bool        display_bin_image(void);
static bool        render_directory_view(void);
static void        truncate_filename(const char *src, char *dst, int max_chars);

/* State machine */
static void        state_machine_tick(void);
static void        transition_to(app_state_t new_state);

/* Buttons */
typedef enum {
    BTN_EVT_NONE = 0,
    BTN_EVT_UP,
    BTN_EVT_DOWN,
    BTN_EVT_SELECT,
    BTN_EVT_BACK,
    BTN_EVT_MENU_SHORT,
    BTN_EVT_MENU_LONG
} btn_event_t;

static btn_event_t poll_buttons(void);

/* Idle timer */
static void        idle_timer_reset(void);
static void        idle_timer_callback(TimerHandle_t xTimer);

/* ============================================================================
 * app_main — Entry Point
 *
 * Wake from deep sleep is treated as a clean cold boot — no state restoration.
 * The device always starts from STATE_BOOT regardless of wake cause.
 * ========================================================================== */
void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "GEVITON SMART TAG: Firmware Rev 6 starting");

    /* NVS must be initialised before battery (learned params stored there) */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    power_pins_init();
    buttons_init();
    EPD_GPIOInit();

    /* Initialise I2C and fuel gauge */
    if (i2c_master_init() == ESP_OK) {
        if (battery_init() == ESP_OK) {
            g_fuel_gauge_ok = true;
        } else {
            ESP_LOGW(TAG, "Fuel gauge init failed: SOC monitoring disabled");
        }
    } else {
        ESP_LOGW(TAG, "I2C init failed: SOC monitoring disabled");
    }

    /* Arm image buffer guards before any display work begins */
    guard_arm(g_image_bw);
    guard_arm(g_loaded_image);

    /* One-shot idle timer — fires for all states */
    g_idle_timer = xTimerCreate(
        "idle_tmr",
        pdMS_TO_TICKS(IDLE_TIMEOUT_MS),
        pdFALSE,
        NULL,
        idle_timer_callback
    );
    xTimerStart(g_idle_timer, 0);

    /* Always start from STATE_BOOT — wake from deep sleep is a clean reboot */
    g_state        = STATE_BOOT;
    g_screen_dirty = true;

    while (1) {
        /* Sleep request flag set by idle timer callback — process here safely */
        if (g_sleep_requested) {
            g_sleep_requested = false;
            ESP_LOGI(TAG, "Processing idle sleep request");
            enter_deep_sleep();
            /* does not return */
        }

        state_machine_tick();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ============================================================================
 * BATTERY — check SOC and handle low-battery condition with hysteresis
 *
 * Called at boot, on every state transition, and after navigation actions.
 * Not called during image viewing (EPD retains image; next boot will check).
 *
 * Hysteresis prevents oscillation:
 *   Once g_battery_low_active is set (SOC < BATTERY_SLEEP_SOC), it is held
 *   until SOC recovers to BATTERY_RESUME_SOC.
 * ========================================================================== */
static void battery_check_and_handle(void)
{
    if (!g_fuel_gauge_ok) {
        ESP_LOGD(TAG, "SOC check skipped: fuel gauge not available");
        return;
    }

    g_soc = battery_get_soc();
    ESP_LOGI(TAG, "SOC check: %.1f %% (battery_low_active=%d)",
             g_soc, (int)g_battery_low_active);

    /* Exit battery-low state only once SOC recovers above resume threshold */
    if (g_battery_low_active) {
        if (g_soc >= BATTERY_RESUME_SOC) {
            ESP_LOGI(TAG, "SOC recovered to %.1f %%: resuming normal operation",
                     g_soc);
            g_battery_low_active = false;
        } else {
            /* Hold in battery-low until threshold is crossed */
            return;
        }
    }

    /* Enter battery-low sleep when SOC falls below sleep threshold */
    if (g_soc < BATTERY_SLEEP_SOC) {
        g_battery_low_active = true;
        ESP_LOGW(TAG, "Battery low (%.1f %%): showing low-battery screen", g_soc);
        epd_power_on();
        clear_all();
        displayui_screen(ui_battery_low_ui);
        epd_power_off();
        vTaskDelay(pdMS_TO_TICKS(500));
        enter_deep_sleep();
        /* does not return */
    }
}

/* ============================================================================
 * STATE MACHINE
 * ========================================================================== */

static void transition_to(app_state_t new_state)
{
    ESP_LOGI(TAG, "State %d → %d", (int)g_state, (int)new_state);
    g_state        = new_state;
    g_screen_dirty = true;
    battery_check_and_handle();
    idle_timer_reset();
}

static void state_machine_tick(void)
{
    btn_event_t evt = poll_buttons();

    if (evt != BTN_EVT_NONE) {
        idle_timer_reset();
    }

    /* MENU long-press → hardware shutdown (any state, checked first) */
    if (evt == BTN_EVT_MENU_LONG) {
        ESP_LOGI(TAG, "MENU long-press: hardware shutdown");
        hardware_shutdown();
        /* does not return */
    }

    /* MENU short-press → clean reboot (any state) */
    if (evt == BTN_EVT_MENU_SHORT) {
        ESP_LOGI(TAG, "MENU short-press: rebooting");
        sd_deinit();
        i2c_master_deinit();
        tf_power_off();
        epd_power_off();
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
        /* does not return */
    }

    switch (g_state) {

    /* ── BOOT ─────────────────────────────────────────────────────────────── */
    case STATE_BOOT:
    {
        epd_power_on();
        clear_all();
        displayui_screen(ui_boot_screen_ui);
        epd_power_off();

        vTaskDelay(pdMS_TO_TICKS(1500));

        battery_check_and_handle();

        /* sd_init() powers the SD rail internally — no tf_power_on() here */
        if (!sd_init()) {
            ESP_LOGW(TAG, "SD card not found at boot");
            transition_to(STATE_NO_SD);
            break;
        }

        strncpy(g_current_path, MOUNT_POINT, sizeof(g_current_path) - 1);
        g_current_path[sizeof(g_current_path) - 1] = '\0';
        g_nav_depth = 0;

        if (!scan_directory(g_current_path, &g_dir)) {
            ESP_LOGW(TAG, "Root scan failed at boot");
            transition_to(STATE_NO_SD);
            break;
        }

        transition_to(STATE_MAIN_MENU);
        break;
    }

    /* ── NO SD CARD ───────────────────────────────────────────────────────── */
    case STATE_NO_SD:
    {
        if (g_screen_dirty) {
            epd_power_on();
            clear_all();
            displayui_screen(ui_no_sd_card_ui);
            epd_power_off();
            g_screen_dirty = false;
        }
        /* MENU short-press (handled above) reboots for a clean SD remount */
        break;
    }

    /* ── MAIN MENU ────────────────────────────────────────────────────────── */
    case STATE_MAIN_MENU:
    {
        if (g_screen_dirty) {
            epd_power_on();
            clear_all();
            displayui_screen(ui_main_menu_ui);
            epd_power_off();
            g_screen_dirty = false;
        }

        if (evt == BTN_EVT_SELECT || evt == BTN_EVT_DOWN) {
            transition_to(STATE_BROWSING);
        }
        break;
    }

    /* ── BROWSING ─────────────────────────────────────────────────────────── */
    case STATE_BROWSING:
    {
        if (g_screen_dirty) {
            if (g_dir.count == 0) {
                epd_power_on();
                clear_all();
                displayui_screen(ui_empty_directory_ui);
                epd_power_off();
            } else {
                if (!render_directory_view()) {
                    ESP_LOGE(TAG, "render_directory_view failed: buffer overrun");
                }
            }
            g_screen_dirty = false;
        }

        if (evt == BTN_EVT_UP) {
            if (g_dir.count > 0) {
                g_dir.selected = (g_dir.selected == 0)
                                 ? g_dir.count - 1
                                 : g_dir.selected - 1;
                g_screen_dirty = true;
                battery_check_and_handle();
            }
        }
        else if (evt == BTN_EVT_DOWN) {
            if (g_dir.count > 0) {
                g_dir.selected = (g_dir.selected + 1) % g_dir.count;
                g_screen_dirty = true;
                battery_check_and_handle();
            }
        }
        else if (evt == BTN_EVT_SELECT) {
            nav_select_item();
        }
        else if (evt == BTN_EVT_BACK) {
            nav_go_back();
        }
        break;
    }

    /* ── IMAGE VIEW ───────────────────────────────────────────────────────── */
    case STATE_IMAGE_VIEW:
    {
        if (g_screen_dirty) {
            if (!display_bin_image()) {
                ESP_LOGE(TAG, "display_bin_image failed: buffer overrun");
            }
            g_screen_dirty = false;
        }

        if (evt == BTN_EVT_BACK) {
            transition_to(STATE_BROWSING);
        }
        break;
    }

    /* ── BATTERY LOW (safety net — battery_check_and_handle does the work) ── */
    case STATE_BATTERY_LOW:
        battery_check_and_handle();
        break;

    /* ── DEEP SLEEP ───────────────────────────────────────────────────────── */
    case STATE_DEEP_SLEEP:
        enter_deep_sleep();
        /* does not return */
        break;

    default:
        transition_to(STATE_BOOT);
        break;
    }
}

/* ============================================================================
 * IDLE TIMER
 * ========================================================================== */

static void idle_timer_reset(void)
{
    if (g_idle_timer == NULL) return;
    xTimerChangePeriod(g_idle_timer, pdMS_TO_TICKS(IDLE_TIMEOUT_MS), 0);
    xTimerReset(g_idle_timer, 0);
}

/*
 * Fires from the FreeRTOS timer task — must not call esp_deep_sleep_start()
 * or mutate g_state here. Set g_sleep_requested; the main loop handles it.
 */
static void idle_timer_callback(TimerHandle_t xTimer)
{
    (void)xTimer;
    ESP_LOGI(TAG, "Idle timeout: requesting deep sleep");
    g_sleep_requested = true;
}

/* ============================================================================
 * DEEP SLEEP
 *
 * Deinitialise peripherals, configure EXT0 wake source, sleep.
 * Wake from sleep is a clean cold boot — no context is saved or restored.
 * Does NOT return.
 * ========================================================================== */
static void enter_deep_sleep(void)
{
    ESP_LOGI(TAG, "Entering deep sleep");

    i2c_master_deinit();
    sd_deinit();       /* also calls tf_power_off() */
    epd_power_off();

    esp_sleep_enable_ext0_wakeup((gpio_num_t)BTN_MENU, 0);

    ESP_LOGI(TAG, "Wake source: GPIO %d LOW", BTN_MENU);
    vTaskDelay(pdMS_TO_TICKS(50));   /* flush UART */
    esp_deep_sleep_start();
}

/* ============================================================================
 * HARDWARE SHUTDOWN
 *
 * Triggered by MENU long-press (≥ 5 s).
 * Drives the latch pin HIGH to cut board power.
 * Does NOT return.
 * ========================================================================== */
static void hardware_shutdown(void)
{
    ESP_LOGI(TAG, "Hardware latch shutdown");

    epd_power_on();
    vTaskDelay(pdMS_TO_TICKS(100));
    clear_all();
    displayui_screen(ui_shutdown_ui);
    epd_power_off();

    vTaskDelay(pdMS_TO_TICKS(500));

    i2c_master_deinit();
    sd_deinit();
    tf_power_off();

    gpio_set_level(POWER_OFF_PIN, 1);

    while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
}

/* ============================================================================
 * BUTTON HANDLING
 * ========================================================================== */

static void buttons_init(void)
{
    const gpio_num_t pins[] = { BTN_UP, BTN_DOWN, BTN_SELECT, BTN_BACK, BTN_MENU };
    gpio_config_t cfg = {
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
        .pin_bit_mask = 0
    };
    for (int i = 0; i < 5; i++) {
        cfg.pin_bit_mask |= (1ULL << pins[i]);
    }
    gpio_config(&cfg);
    ESP_LOGI(TAG, "Buttons initialised");
}

static btn_event_t poll_buttons(void)
{
    static bool    nb_last[4]  = { true, true, true, true };
    static int64_t nb_ms[4]    = { 0 };
    const uint32_t DEBOUNCE_MS = 200;

    const gpio_num_t  nb_pins[4] = { BTN_UP, BTN_DOWN, BTN_SELECT, BTN_BACK };
    const btn_event_t nb_evts[4] = {
        BTN_EVT_UP, BTN_EVT_DOWN, BTN_EVT_SELECT, BTN_EVT_BACK
    };

    static bool    menu_last_pressed = false;
    static int64_t menu_press_ms     = 0;

    int64_t now_ms = (int64_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    for (int i = 0; i < 4; i++) {
        bool pressed = (gpio_get_level(nb_pins[i]) == 0);
        if (pressed && nb_last[i] && (now_ms - nb_ms[i]) > (int64_t)DEBOUNCE_MS) {
            nb_last[i] = false;
            nb_ms[i]   = now_ms;
            return nb_evts[i];
        } else if (!pressed) {
            nb_last[i] = true;
        }
    }

    bool menu_pressed = (gpio_get_level(BTN_MENU) == 0);

    if (menu_pressed && !menu_last_pressed) {
        menu_press_ms     = now_ms;
        menu_last_pressed = true;
    }
    else if (!menu_pressed && menu_last_pressed) {
        menu_last_pressed = false;
        int64_t held_ms = now_ms - menu_press_ms;
        if (held_ms >= (int64_t)MENU_LONGPRESS_MS) {
            return BTN_EVT_MENU_LONG;
        } else if (held_ms >= (int64_t)DEBOUNCE_MS) {
            return BTN_EVT_MENU_SHORT;
        }
    }

    return BTN_EVT_NONE;
}

/* ============================================================================
 * POWER PIN MANAGEMENT
 * ========================================================================== */

static void power_pins_init(void)
{
    esp_rom_gpio_pad_select_gpio(POWER_OFF_PIN);
    gpio_set_direction(POWER_OFF_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(POWER_OFF_PIN, 0);
}

static void epd_power_on(void)
{
    esp_rom_gpio_pad_select_gpio(EPD_POWER_PIN);
    gpio_set_direction(EPD_POWER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(EPD_POWER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
}

static void epd_power_off(void)
{
    esp_rom_gpio_pad_select_gpio(EPD_POWER_PIN);
    gpio_set_direction(EPD_POWER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(EPD_POWER_PIN, 0);
}

static void tf_power_on(void)
{
    esp_rom_gpio_pad_select_gpio(TF_POWER_PIN);
    gpio_set_direction(TF_POWER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(TF_POWER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
}

static void tf_power_off(void)
{
    esp_rom_gpio_pad_select_gpio(TF_POWER_PIN);
    gpio_set_direction(TF_POWER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(TF_POWER_PIN, 0);
}

/* ============================================================================
 * SD CARD INIT / DEINIT
 *
 * sd_init() manages the SD rail via tf_power_on() internally.
 * Callers must NOT call tf_power_on() separately.
 *
 * Returns true on success, false on failure.
 * sd_deinit() returns true on success (or if already deinitialised),
 * false if the unmount step failed (bus is still freed in that case).
 * ========================================================================== */

static bool sd_init(void)
{
    if (g_sd_initialized) {
        ESP_LOGW(TAG, "sd_init() called but SD already initialised: skipping");
        return true;
    }

    tf_power_on();

    spi_bus_config_t sdbus_cfg = {
        .mosi_io_num     = 40,
        .miso_io_num     = 13,
        .sclk_io_num     = 39,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 16 * 1024,
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI3_HOST;

    esp_err_t ret = spi_bus_initialize(SPI3_HOST, &sdbus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        tf_power_off();
        return false;
    }

    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed   = false,
        .max_files                = 8,
        .allocation_unit_size     = 16 * 1024,
        .use_one_fat              = false,
        .disk_status_check_enable = false,
    };

    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.gpio_cs = SD_CS_PIN;
    slot_cfg.host_id = SPI3_HOST;

    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_cfg,
                                  &mount_cfg, &g_sd_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SD mount failed: %s", esp_err_to_name(ret));
        spi_bus_free(SPI3_HOST);
        tf_power_off();
        g_sd_mounted     = false;
        g_sd_card        = NULL;
        g_sd_initialized = false;
        return false;
    }

    g_sd_mounted     = true;
    g_sd_initialized = true;
    sdmmc_card_print_info(stdout, g_sd_card);
    ESP_LOGI(TAG, "SD mounted at %s", MOUNT_POINT);
    return true;
}

static bool sd_deinit(void)
{
    if (!g_sd_initialized) {
        ESP_LOGD(TAG, "sd_deinit() called but SD not initialised: skipping");
        return true;
    }

    bool ok = true;

    if (g_sd_mounted && g_sd_card != NULL) {
        esp_err_t ret = esp_vfs_fat_sdcard_unmount(MOUNT_POINT, g_sd_card);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "SD unmount warning: %s", esp_err_to_name(ret));
            ok = false;   /* unmount failed, but continue to free the bus */
        }
        g_sd_mounted = false;
        g_sd_card    = NULL;
        ESP_LOGI(TAG, "SD unmounted");
    }

    spi_bus_free(SPI3_HOST);
    tf_power_off();
    g_sd_initialized = false;
    return ok;
}

/* ============================================================================
 * IMAGE LOADING
 *
 * Arms the guard on g_loaded_image, reads IMAGE_SIZE bytes from the file,
 * then verifies the guard is intact. Returns false on any failure.
 * ========================================================================== */

static bool load_images(const char *full_filepath)
{
    ESP_LOGI(TAG, "Opening image: %s", full_filepath);

    FILE *f = fopen(full_filepath, "rb");
    if (f == NULL) {
        ESP_LOGE(TAG, "fopen failed: SD card may have been removed");
        return false;
    }

    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    rewind(f);

    ESP_LOGI(TAG, "File size: %ld bytes (expected: %d bytes)", file_size, IMAGE_SIZE);

    if (file_size != (long)IMAGE_SIZE) {
        ESP_LOGE(TAG, "Image rejected: expected %d bytes, got %ld bytes. "
                      "Re-export from image2cpp with correct settings.",
                      IMAGE_SIZE, file_size);
        fclose(f);
        return false;
    }

    /* Arm guard before read so any overrun is detectable */
    guard_arm(g_loaded_image);

    size_t bytes_read = fread(g_loaded_image, 1, IMAGE_SIZE, f);
    fclose(f);

    if (bytes_read != IMAGE_SIZE) {
        ESP_LOGE(TAG, "Read error: only %d of %d bytes read",
                 (int)bytes_read, IMAGE_SIZE);
        return false;
    }

    /* Verify guard region was not disturbed by the read */
    if (!guard_check(g_loaded_image, "g_loaded_image")) {
        return false;
    }

    ESP_LOGI(TAG, "Image loaded OK");
    return true;
}

/* ============================================================================
 * DIRECTORY SCANNING
 * ========================================================================== */

static bool is_bin_file(const char *name)
{
    size_t len = strlen(name);
    if (len < 5) return false;
    const char *ext = name + len - 4;
    return (ext[0] == '.' &&
           (ext[1] == 'b' || ext[1] == 'B') &&
           (ext[2] == 'i' || ext[2] == 'I') &&
           (ext[3] == 'n' || ext[3] == 'N'));
}

static bool is_system_entry(const char *name)
{
    if (name == NULL || name[0] == '\0') return true;
    if (strchr(name, '~') != NULL) {
        ESP_LOGD(TAG, "Skipping system entry: %s", name);
        return true;
    }
    return false;
}

static int entry_compare(const void *a, const void *b)
{
    const dir_entry_t *ea = (const dir_entry_t *)a;
    const dir_entry_t *eb = (const dir_entry_t *)b;
    if (ea->type != eb->type) {
        return (ea->type == ENTRY_FOLDER) ? -1 : 1;
    }
    return strcasecmp(ea->name, eb->name);
}

static bool scan_directory(const char *path, dir_listing_t *listing)
{
    memset(listing, 0, sizeof(*listing));

    DIR *dir = opendir(path);
    if (dir == NULL) {
        ESP_LOGE(TAG, "opendir failed: %s", path);
        return false;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL && listing->count < MAX_ENTRIES) {

        if (entry->d_name[0] == '.') continue;
        if (is_system_entry(entry->d_name)) continue;

        char full_path[320];
        int written = snprintf(full_path, sizeof(full_path),
                               "%s/%s", path, entry->d_name);
        if (written >= (int)sizeof(full_path)) {
            ESP_LOGE(TAG, "Path too long, skipping: %s/%s", path, entry->d_name);
            continue;
        }

        struct stat st;
        if (stat(full_path, &st) != 0) {
            ESP_LOGW(TAG, "stat failed: %s", full_path);
            continue;
        }

        if (S_ISDIR(st.st_mode)) {
            strncpy(listing->entries[listing->count].name,
                    entry->d_name, MAX_NAME_LEN - 1);
            listing->entries[listing->count].name[MAX_NAME_LEN - 1] = '\0';
            listing->entries[listing->count].type = ENTRY_FOLDER;
            listing->count++;
        } else if (S_ISREG(st.st_mode) && is_bin_file(entry->d_name)) {
            strncpy(listing->entries[listing->count].name,
                    entry->d_name, MAX_NAME_LEN - 1);
            listing->entries[listing->count].name[MAX_NAME_LEN - 1] = '\0';
            listing->entries[listing->count].type = ENTRY_FILE;
            listing->count++;
        }
    }

    closedir(dir);

    if (listing->count > 1) {
        qsort(listing->entries,
              (size_t)listing->count,
              sizeof(dir_entry_t),
              entry_compare);
    }

    ESP_LOGI(TAG, "Scanned '%s': %d entries", path, listing->count);
    return true;
}

/* ============================================================================
 * NAVIGATION STACK
 *
 * nav_push() / nav_pop() own all changes to g_nav_depth.
 * No other function increments or decrements g_nav_depth directly.
 * nav_enter_folder() uses nav_push() return value to decide whether to proceed;
 * if path construction fails after a successful push, nav_pop() rolls it back.
 * ========================================================================== */

static bool nav_push(const char *path, int selected, int scroll)
{
    if (g_nav_depth >= MAX_STACK_DEPTH) {
        ESP_LOGW(TAG, "nav_push: stack full (depth=%d)", g_nav_depth);
        return false;
    }
    strncpy(g_nav_stack[g_nav_depth].path, path,
            sizeof(g_nav_stack[0].path) - 1);
    g_nav_stack[g_nav_depth].path[sizeof(g_nav_stack[0].path) - 1] = '\0';
    g_nav_stack[g_nav_depth].selected      = selected;
    g_nav_stack[g_nav_depth].scroll_offset = scroll;
    g_nav_depth++;   /* depth modified only here */
    return true;
}

static bool nav_pop(void)
{
    if (g_nav_depth <= 0) {
        ESP_LOGW(TAG, "nav_pop: stack already empty");
        return false;
    }
    g_nav_depth--;   /* depth modified only here */
    return true;
}

/*
 * nav_enter_folder — push current location, build new path, scan directory.
 *
 * Returns true on success, false on any failure (stack full, path too long,
 * or scan failure). On failure the stack is rolled back via nav_pop() so
 * g_nav_depth is never left in an inconsistent state.
 */
static bool nav_enter_folder(const char *folder_name)
{
    if (folder_name == NULL || folder_name[0] == '\0') {
        ESP_LOGE(TAG, "nav_enter_folder: empty name");
        return false;
    }

    /* Save current location before descending */
    if (!nav_push(g_current_path, g_dir.selected, g_dir.scroll_offset)) {
        return false;
    }

    /* Build the child path */
    char new_path[256];
    int written = snprintf(new_path, sizeof(new_path),
                           "%s/%s", g_current_path, folder_name);
    if (written >= (int)sizeof(new_path)) {
        ESP_LOGE(TAG, "nav_enter_folder: path too long: %s/%s",
                 g_current_path, folder_name);
        nav_pop();   /* roll back the push — no direct g_nav_depth manipulation */
        return false;
    }

    strncpy(g_current_path, new_path, sizeof(g_current_path) - 1);
    g_current_path[sizeof(g_current_path) - 1] = '\0';

    memset(&g_dir, 0, sizeof(g_dir));
    if (!scan_directory(g_current_path, &g_dir)) {
        /* Scan failed — restore the previous path and pop the stack */
        ESP_LOGE(TAG, "nav_enter_folder: scan failed for %s", g_current_path);
        nav_frame_t *frame = &g_nav_stack[g_nav_depth - 1];
        strncpy(g_current_path, frame->path, sizeof(g_current_path) - 1);
        g_current_path[sizeof(g_current_path) - 1] = '\0';
        nav_pop();
        return false;
    }

    g_screen_dirty = true;
    battery_check_and_handle();
    ESP_LOGI(TAG, "Entered: %s", g_current_path);
    return true;
}

/*
 * nav_go_back — pop the stack and return to the parent directory.
 *
 * Returns true on success, false if already at root or scan fails.
 */
static bool nav_go_back(void)
{
    if (g_nav_depth == 0) {
        ESP_LOGI(TAG, "nav_go_back: already at root: no-op");
        return false;
    }

    /* Peek at the frame before popping (pop decrements depth) */
    nav_frame_t *frame = &g_nav_stack[g_nav_depth - 1];

    if (!nav_pop()) {
        return false;
    }

    strncpy(g_current_path, frame->path, sizeof(g_current_path) - 1);
    g_current_path[sizeof(g_current_path) - 1] = '\0';

    memset(&g_dir, 0, sizeof(g_dir));
    scan_directory(g_current_path, &g_dir);

    if (g_dir.count == 0) {
        g_dir.selected      = 0;
        g_dir.scroll_offset = 0;
    } else {
        g_dir.selected      = frame->selected;
        g_dir.scroll_offset = frame->scroll_offset;
        if (g_dir.selected >= g_dir.count) {
            g_dir.selected = g_dir.count - 1;
        }
    }

    g_screen_dirty = true;
    battery_check_and_handle();
    ESP_LOGI(TAG, "Returned to: %s", g_current_path);
    return true;
}

static void nav_select_item(void)
{
    if (g_dir.count == 0) return;

    dir_entry_t *item = &g_dir.entries[g_dir.selected];

    if (item->type == ENTRY_FOLDER) {
        if (!nav_enter_folder(item->name)) {
            ESP_LOGE(TAG, "nav_select_item: enter folder failed");
        }
    }
    else if (item->type == ENTRY_FILE) {
        char full_path[320];
        int written = snprintf(full_path, sizeof(full_path),
                               "%s/%s", g_current_path, item->name);
        if (written >= (int)sizeof(full_path)) {
            ESP_LOGE(TAG, "nav_select_item: file path too long");
            return;
        }

        if (load_images(full_path)) {
            battery_check_and_handle();
            transition_to(STATE_IMAGE_VIEW);
        } else {
            ESP_LOGE(TAG, "Image load failed: %s", full_path);
            memset(&g_dir, 0, sizeof(g_dir));
            if (!scan_directory(g_current_path, &g_dir)) {
                ESP_LOGW(TAG, "SD removed: going to NO_SD");
                transition_to(STATE_NO_SD);
            } else {
                g_screen_dirty = true;
            }
        }
    }
}

/* ============================================================================
 * DISPLAY FUNCTIONS
 * ========================================================================== */

static void clear_all(void)
{
    EPD_Clear();
    Paint_NewImage(g_image_bw, EPD_W, EPD_H, 0, WHITE);
    EPD_Full(WHITE);
    EPD_Display_Part(0, 0, EPD_W, EPD_H, g_image_bw);
    vTaskDelay(pdMS_TO_TICKS(20));
}

static void displayui_screen(const uint8_t *ui_data)
{
    Paint_NewImage(g_image_bw, EPD_W, EPD_H, 0, WHITE);
    EPD_Full(WHITE);
    EPD_Display_Part(0, 0, EPD_W, EPD_H, g_image_bw);
    vTaskDelay(pdMS_TO_TICKS(20));
    EPD_Init_Fast(Fast_Seconds_1_5s);
    EPD_ShowPicture(0, 0, EPD_W, EPD_H, ui_data, BLACK);
    vTaskDelay(pdMS_TO_TICKS(20));
    EPD_Display_Fast(g_image_bw);
    vTaskDelay(pdMS_TO_TICKS(100));
}

/*
 * display_bin_image — arms the g_image_bw guard, performs the EPD render
 * sequence, then verifies guard integrity.
 * Returns false if a buffer overrun is detected.
 */
static bool display_bin_image(void)
{
    epd_power_on();
    vTaskDelay(pdMS_TO_TICKS(50));

    guard_arm(g_image_bw);
    clear_all();
    vTaskDelay(pdMS_TO_TICKS(10));

    Paint_NewImage(g_image_bw, EPD_W, EPD_H, 0, WHITE);
    EPD_Full(WHITE);
    EPD_Display_Part(0, 0, EPD_W, EPD_H, g_image_bw);
    vTaskDelay(pdMS_TO_TICKS(20));
    EPD_Init_Fast(Fast_Seconds_1_5s);
    EPD_Display_Fast(g_loaded_image);
    vTaskDelay(pdMS_TO_TICKS(20));

    vTaskDelay(pdMS_TO_TICKS(100));
    epd_power_off();

    return guard_check(g_image_bw, "g_image_bw (display_bin_image)");
}

static void truncate_filename(const char *src, char *dst, int max_chars)
{
    int src_len = (int)strlen(src);

    if (src_len <= max_chars) {
        strncpy(dst, src, (size_t)max_chars);
        dst[max_chars] = '\0';
        return;
    }

    const char *dot = strrchr(src, '.');
    char ext[6] = "";
    if (dot != NULL && (src_len - (int)(dot - src)) <= 5) {
        strncpy(ext, dot, sizeof(ext) - 1);
        ext[sizeof(ext) - 1] = '\0';
    }

    int ext_len     = (int)strlen(ext);
    int stem_budget = max_chars - 3 - ext_len;

    if (stem_budget < 1) {
        strncpy(dst, src, (size_t)max_chars);
        dst[max_chars] = '\0';
        return;
    }

    strncpy(dst, src, (size_t)stem_budget);
    dst[stem_budget] = '\0';
    strcat(dst, "...");
    strcat(dst, ext);
}

/*
 * render_directory_view — arms the g_image_bw guard, draws the directory
 * list, then verifies guard integrity.
 * Returns false if a buffer overrun is detected.
 */
static bool render_directory_view(void)
{
    epd_power_on();
    vTaskDelay(pdMS_TO_TICKS(50));

    guard_arm(g_image_bw);
    clear_all();

    Paint_NewImage(g_image_bw, EPD_W, EPD_H, 0, WHITE);
    EPD_Full(WHITE);
    vTaskDelay(pdMS_TO_TICKS(20));

    EPD_ShowPicture(0, 0, EPD_W, EPD_H, ui_directory_view_ui, BLACK);
    vTaskDelay(pdMS_TO_TICKS(20));

    /* Breadcrumb — strip mount prefix, show "/" at root */
    const char *disp_path = g_current_path + strlen(MOUNT_POINT);
    if (*disp_path == '\0') disp_path = "/";
    EPD_ShowString(8, 12, (char *)disp_path, FONT_SIZE, WHITE);
    vTaskDelay(pdMS_TO_TICKS(20));

    /* Clamp scroll to keep selection visible */
    if (g_dir.selected < g_dir.scroll_offset) {
        g_dir.scroll_offset = g_dir.selected;
    }
    if (g_dir.selected >= g_dir.scroll_offset + ITEMS_PER_PAGE) {
        g_dir.scroll_offset = g_dir.selected - ITEMS_PER_PAGE + 1;
    }

    for (int i = 0; i < ITEMS_PER_PAGE; i++) {
        int idx = g_dir.scroll_offset + i;
        if (idx >= g_dir.count) break;

        dir_entry_t *e      = &g_dir.entries[idx];
        bool         is_sel = (idx == g_dir.selected);
        uint16_t     y      = (uint16_t)(LIST_Y_ORIGIN + i * LIST_ROW_HEIGHT);

        if (is_sel) {
            EPD_DrawRectangle(4, (uint16_t)(y - 2),
                              (uint16_t)(EPD_W - 10),
                              (uint16_t)(y + LIST_ROW_HEIGHT - 4),
                              BLACK, 1);
        }

        const char *icon = (e->type == ENTRY_FOLDER) ? ">" : "-";
        EPD_ShowString(LIST_X_ICON, y, (char *)icon,
                       FONT_SIZE, is_sel ? WHITE : BLACK);

        char display_name[DISPLAY_NAME_MAX + 1];
        truncate_filename(e->name, display_name, DISPLAY_NAME_MAX);
        EPD_ShowString(LIST_X_TEXT, y, display_name,
                       FONT_SIZE, is_sel ? WHITE : BLACK);
    }

    /* Scrollbar thumb */
    if (g_dir.count > ITEMS_PER_PAGE) {
        int list_px      = ITEMS_PER_PAGE * LIST_ROW_HEIGHT;
        int thumb_h      = list_px * ITEMS_PER_PAGE / g_dir.count;
        int scroll_range = g_dir.count - ITEMS_PER_PAGE;
        int thumb_y      = LIST_Y_ORIGIN
                           + (g_dir.scroll_offset * (list_px - thumb_h))
                             / scroll_range;

        EPD_DrawRectangle((uint16_t)(EPD_W - 6), (uint16_t)thumb_y,
                          (uint16_t)(EPD_W - 2), (uint16_t)(thumb_y + thumb_h),
                          BLACK, 1);
    }

    EPD_Init_Fast(Fast_Seconds_1_5s);
    EPD_Display_Fast(g_image_bw);
    vTaskDelay(pdMS_TO_TICKS(20));

    vTaskDelay(pdMS_TO_TICKS(100));
    epd_power_off();

    return guard_check(g_image_bw, "g_image_bw (render_directory_view)");
}

/* ============================================================================
 * MAX17055 FUEL GAUGE — I2C platform bridge + init + read helpers
 *
 * NOTE: The I2C write implementation (uint8_t buf[3]) is intentional and
 * must not be modified.
 * ========================================================================== */

static esp_err_t i2c_master_init(void)
{
    if (g_i2c_initialized) {
        ESP_LOGW(TAG, "i2c_master_init() already done: skipping");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initialising I2C master");
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_SDA_GPIO,
        .scl_io_num       = I2C_SCL_GPIO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &cfg));
    esp_err_t ret = i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (ret == ESP_OK) {
        g_i2c_initialized = true;
    }
    return ret;
}

static esp_err_t i2c_master_deinit(void)
{
    if (!g_i2c_initialized) {
        ESP_LOGD(TAG, "i2c_master_deinit() not initialised: skipping");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitialising I2C master");
    esp_err_t ret = i2c_driver_delete(I2C_PORT);
    if (ret == ESP_OK) {
        g_i2c_initialized = false;
    }
    return ret;
}

static int32_t platform_i2c_write(void *handle, uint8_t reg,
                                   uint8_t *data, uint16_t len)
{
    uint8_t buf[3];
    buf[0] = reg;
    memcpy(&buf[1], data, len);
    esp_err_t err = i2c_master_write_to_device(
        I2C_PORT, MAX17055_ADDR, buf, (size_t)(len + 1u),
        pdMS_TO_TICKS(100));
    return (err == ESP_OK) ? 0 : -1;
}

static int32_t platform_i2c_read(void *handle, uint8_t reg,
                                   uint8_t *data, uint16_t len)
{
    esp_err_t err = i2c_master_write_read_device(
        I2C_PORT, MAX17055_ADDR, &reg, 1, data, (size_t)len,
        pdMS_TO_TICKS(100));
    return (err == ESP_OK) ? 0 : -1;
}

static esp_err_t save_learned_params_to_nvs(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open("battery", NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    err = nvs_set_blob(h, "learned", &g_learned, sizeof(g_learned));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);

    if (err == ESP_OK) ESP_LOGI(TAG, "Learned params saved to NVS");
    else               ESP_LOGE(TAG, "NVS save failed: %s", esp_err_to_name(err));
    return err;
}

static esp_err_t load_learned_params_from_nvs(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open("battery", NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No learned params in NVS");
        return err;
    }

    size_t sz = sizeof(g_learned);
    err = nvs_get_blob(h, "learned", &g_learned, &sz);
    nvs_close(h);

    if (err == ESP_OK) {
        g_learned_valid = true;
        ESP_LOGI(TAG, "Learned params restored from NVS");
    }
    return err;
}

static esp_err_t battery_init(void)
{
    fg = (max17055_ctx_t){
        .write_reg         = platform_i2c_write,
        .read_reg          = platform_i2c_read,
        .handle            = NULL,
        .rsense_mohm       = 10,
        .design_cap_mah    = 300,
        .charge_voltage_mv = 4200,
        .empty_voltage_mv  = 3300,
        .ichg_term_ma      = 150,
        .model_id          = MAX17055_MODEL_LITHIUM_COBALT,
    };

    uint16_t devname = 0;
    if (max17055_check_device(&fg, &devname) != 0) {
        ESP_LOGE(TAG, "MAX17055 not found on I2C bus");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "MAX17055 found: DevName=0x%04X", devname);

    uint16_t status_raw = 0;
    max17055_reg_read(&fg, MAX17055_REG_STATUS, &status_raw);
    bool was_por = (status_raw & 0x0002u) != 0u;

    if (max17055_init(&fg) != 0) {
        ESP_LOGE(TAG, "max17055_init() failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Fuel gauge initialised");

    if (was_por) {
        load_learned_params_from_nvs();
        if (g_learned_valid) {
            ESP_LOGI(TAG, "Restoring learned params...");
            if (max17055_restore_learned_params(&fg, &g_learned) == 0) {
                vTaskDelay(pdMS_TO_TICKS(350));
                max17055_restore_capacity(&fg, &g_learned);
                vTaskDelay(pdMS_TO_TICKS(350));
            }
        }
    }
    return ESP_OK;
}

static void max17055_save_learned(void)
{
    uint16_t cycles_raw = 0;
    if (max17055_get_cycles_raw(&fg, &cycles_raw) == 0) {
        if ((uint16_t)(cycles_raw - g_cycles_last_saved) >= 64u) {
            if (max17055_save_learned_params(&fg, &g_learned) == 0) {
                g_learned_valid     = true;
                g_cycles_last_saved = cycles_raw;
                ESP_LOGI(TAG, "Learned params saved (cycles_raw=%u)", cycles_raw);
                save_learned_params_to_nvs();
            }
        }
    }
}

/*
 * battery_get_soc — returns SOC % (0–100).
 * Returns 100.0 if the fuel gauge is unavailable, preventing false low-battery
 * trips. battery_check_and_handle() also guards on g_fuel_gauge_ok.
 */
static float battery_get_soc(void)
{
    if (!g_fuel_gauge_ok) {
        ESP_LOGD(TAG, "battery_get_soc: gauge unavailable, returning safe default");
        return 100.0f;
    }

    float soc = max17055_get_soc(&fg);
    if (soc < 0.0f)   soc = 0.0f;
    if (soc > 100.0f) soc = 100.0f;
    return soc;
}

/* ============================================================================
 * END OF FILE
 * ============================================================================
 */