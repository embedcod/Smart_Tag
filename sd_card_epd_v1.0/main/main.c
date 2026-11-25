/*
 * GEVITON SMART TAG — Main Application Firmware
 * Target: ESP32-S3 | ESP-IDF
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
#include "driver/gpio.h"
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

static const char *TAG = "SMART_TAG";

// PIN DEFINITIONS
#define SD_CS_PIN           10
#define EPD_POWER_PIN        7
#define TF_POWER_PIN        42
#define POWER_OFF_PIN        3

#define BTN_UP               4
#define BTN_DOWN             6
#define BTN_SELECT           5
#define BTN_BACK             1
#define BTN_MENU             2

// FILESYSTEM / BROWSER CONSTANTS
#define MOUNT_POINT          "/sdcard"
#define MAX_ENTRIES           64
#define MAX_NAME_LEN          64
#define MAX_STACK_DEPTH        8
#define IMAGE_SIZE          15000

// IDLE TIMEOUT
#define IDLE_TIMEOUT_BROWSE_MS   (60  * 1000)
#define IDLE_TIMEOUT_IMAGE_MS    (300 * 1000)

// DIRECTORY VIEW LAYOUT
#define ITEMS_PER_PAGE    8
#define LIST_Y_ORIGIN    44
#define LIST_ROW_HEIGHT  28
#define LIST_X_ICON       8
#define LIST_X_TEXT      26
#define FONT_SIZE        16
#define DISPLAY_NAME_MAX  30

// APPLICATION STATES
typedef enum {
    STATE_BOOT = 0,
    STATE_NO_SD,
    STATE_MAIN_MENU,
    STATE_BROWSING,
    STATE_IMAGE_VIEW,
    STATE_SHUTDOWN
} app_state_t;

// DATA STRUCTURES
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

#define IMAGE_GUARD_BYTES  64

/* EPD buffers — declared first so overruns cannot reach nav structures */
static uint8_t       g_image_bw[IMAGE_SIZE + IMAGE_GUARD_BYTES];
static uint8_t       g_loaded_image[IMAGE_SIZE + IMAGE_GUARD_BYTES];

/* Navigation / filesystem state */
static app_state_t   g_state              = STATE_BOOT;
static bool          g_screen_dirty       = true;
static volatile bool g_shutdown_requested = false;
static bool          g_sd_mounted         = false;
static sdmmc_card_t *g_sd_card            = NULL;
static nav_frame_t   g_nav_stack[MAX_STACK_DEPTH];
static int           g_nav_depth          = 0;
static dir_listing_t g_dir;
static char          g_current_path[256]  = MOUNT_POINT;
static TimerHandle_t g_idle_timer         = NULL;

// FORWARD DECLARATIONS
static void        buttons_init(void);
static void        power_pins_init(void);
static void        epd_power_on(void);
static void        epd_power_off(void);
static void        tf_power_on(void);
static void        tf_power_off(void);
static void        device_shutdown(void);
static bool        sd_init(void);
static void        sd_deinit(void);
static bool        load_images(const char *full_filepath);
static bool        scan_directory(const char *path, dir_listing_t *listing);
static bool        is_bin_file(const char *name);
static bool        is_system_entry(const char *name);   /* Bug B */
static int         entry_compare(const void *a, const void *b);
static bool        nav_push(const char *path, int selected, int scroll);
static bool        nav_pop(void);
static void        nav_enter_folder(const char *folder_name);
static void        nav_go_back(void);
static void        nav_select_item(void);
static void        clear_all(void);
static void        displayui_screen(const uint8_t *ui_data);
static void        display_bin_image(void);
static void        render_directory_view(void);
static void        truncate_filename(const char *src, char *dst, int max_chars);
static void        state_machine_tick(void);
static void        transition_to(app_state_t new_state);
static void        idle_timer_reset(uint32_t timeout_ms);
static void        idle_timer_callback(TimerHandle_t xTimer);

typedef enum {
    BTN_EVT_NONE = 0,
    BTN_EVT_UP,
    BTN_EVT_DOWN,
    BTN_EVT_SELECT,
    BTN_EVT_BACK,
    BTN_EVT_MENU
} btn_event_t;

static btn_event_t poll_buttons(void);

// app_main
void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "GEVITON SMART TAG: Firmware Rev 4 starting");

    power_pins_init();
    buttons_init();
    EPD_GPIOInit();

    g_idle_timer = xTimerCreate(
        "idle_tmr",
        pdMS_TO_TICKS(IDLE_TIMEOUT_BROWSE_MS),
        pdFALSE,
        NULL,
        idle_timer_callback
    );
    xTimerStart(g_idle_timer, 0);

    g_state        = STATE_BOOT;
    g_screen_dirty = true;

    while (1) {
        if (g_shutdown_requested) {
            g_shutdown_requested = false;
            transition_to(STATE_SHUTDOWN);
        }
        state_machine_tick();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// STATE MACHINE
static void transition_to(app_state_t new_state)
{
    ESP_LOGI(TAG, "State %d to %d", (int)g_state, (int)new_state);
    g_state        = new_state;
    g_screen_dirty = true;

    if (new_state == STATE_IMAGE_VIEW) {
        idle_timer_reset(IDLE_TIMEOUT_IMAGE_MS);
    } else {
        idle_timer_reset(IDLE_TIMEOUT_BROWSE_MS);
    }
}

static void state_machine_tick(void)
{
    btn_event_t evt = poll_buttons();

    if (evt != BTN_EVT_NONE) {
        uint32_t period = (g_state == STATE_IMAGE_VIEW)
                          ? IDLE_TIMEOUT_IMAGE_MS
                          : IDLE_TIMEOUT_BROWSE_MS;
        idle_timer_reset(period);
    }

    switch (g_state) {

    case STATE_BOOT:
    {
        epd_power_on();
        clear_all();
        displayui_screen(ui_boot_screen_ui);
        epd_power_off();
        tf_power_on();
        vTaskDelay(pdMS_TO_TICKS(1500));

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

    case STATE_NO_SD:
    {
        if (g_screen_dirty) {
            epd_power_on();
            clear_all();
            displayui_screen(ui_no_sd_card_ui);
            epd_power_off();
            g_screen_dirty = false;
        }

        if (evt == BTN_EVT_MENU) {
            ESP_LOGI(TAG, "Retrying SD card mount...");
            sd_deinit();
            tf_power_on();
            vTaskDelay(pdMS_TO_TICKS(500));

            if (sd_init()) {
                strncpy(g_current_path, MOUNT_POINT, sizeof(g_current_path) - 1);
                g_current_path[sizeof(g_current_path) - 1] = '\0';
                g_nav_depth = 0;

                if (!scan_directory(g_current_path, &g_dir)) {
                    ESP_LOGW(TAG, "Root scan failed after SD retry");
                    sd_deinit();
                    g_screen_dirty = true;
                } else {
                    transition_to(STATE_MAIN_MENU);
                }
            } else {
                ESP_LOGW(TAG, "SD retry failed");
                g_screen_dirty = true;
            }
        }
        break;
    }

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

    case STATE_BROWSING:
    {
        if (g_screen_dirty) {
            if (g_dir.count == 0) {
                epd_power_on();
                clear_all();
                displayui_screen(ui_empty_directory_ui);
                epd_power_off();
            } else {
                render_directory_view();
            }
            g_screen_dirty = false;
        }

        if (evt == BTN_EVT_UP) {
            if (g_dir.count > 0) {
                g_dir.selected = (g_dir.selected == 0)
                                 ? g_dir.count - 1
                                 : g_dir.selected - 1;
                g_screen_dirty = true;
            }
        }
        else if (evt == BTN_EVT_DOWN) {
            if (g_dir.count > 0) {
                g_dir.selected = (g_dir.selected + 1) % g_dir.count;
                g_screen_dirty = true;
            }
        }
        else if (evt == BTN_EVT_SELECT) {
            nav_select_item();
        }
        else if (evt == BTN_EVT_BACK) {
            nav_go_back();
        }
        else if (evt == BTN_EVT_MENU) {
            ESP_LOGI(TAG, "MENU pressed: rescanning root");
            g_nav_depth = 0;
            strncpy(g_current_path, MOUNT_POINT, sizeof(g_current_path) - 1);
            g_current_path[sizeof(g_current_path) - 1] = '\0';

            if (!scan_directory(g_current_path, &g_dir)) {
                ESP_LOGW(TAG, "Root rescan failed: SD card may be gone");
                transition_to(STATE_NO_SD);
            } else {
                transition_to(STATE_MAIN_MENU);
            }
        }
        break;
    }

    case STATE_IMAGE_VIEW:
    {
        if (g_screen_dirty) {
            display_bin_image();
            g_screen_dirty = false;
        }

        if (evt == BTN_EVT_BACK || evt == BTN_EVT_MENU) {
            transition_to(STATE_BROWSING);
        }
        break;
    }

    case STATE_SHUTDOWN:
        device_shutdown();
        //while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); } // TODO
        break;

    default:
        transition_to(STATE_BOOT);
        break;
    }
}

// IDLE TIMER
static void idle_timer_reset(uint32_t timeout_ms)
{
    if (g_idle_timer == NULL) return;
    xTimerChangePeriod(g_idle_timer, pdMS_TO_TICKS(timeout_ms), 0);
    xTimerReset(g_idle_timer, 0);
}

static void idle_timer_callback(TimerHandle_t xTimer)
{
    (void)xTimer;
    ESP_LOGI(TAG, "Idle timeout: requesting shutdown");
    g_shutdown_requested = true;
}

// BUTTON HANDLING
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
    static bool    last_state[5] = { true, true, true, true, true };
    static int64_t last_ms[5]    = { 0 };
    const uint32_t DEBOUNCE_MS   = 200;

    const gpio_num_t  pins[5] = { BTN_UP, BTN_DOWN, BTN_SELECT, BTN_BACK, BTN_MENU };
    const btn_event_t evts[5] = {
        BTN_EVT_UP, BTN_EVT_DOWN, BTN_EVT_SELECT, BTN_EVT_BACK, BTN_EVT_MENU
    };

    int64_t now_ms = (int64_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    for (int i = 0; i < 5; i++) {
        bool pressed = (gpio_get_level(pins[i]) == 0);

        if (pressed && last_state[i] && (now_ms - last_ms[i]) > (int64_t)DEBOUNCE_MS) {
            last_state[i] = false;
            last_ms[i]    = now_ms;
            return evts[i];
        } else if (!pressed) {
            last_state[i] = true;
        }
    }
    return BTN_EVT_NONE;
}

// POWER PIN MANAGEMENT
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

static void device_shutdown(void)
{
    ESP_LOGI(TAG, "Shutting down device...");

    epd_power_on();
    vTaskDelay(pdMS_TO_TICKS(100));
    clear_all();
    displayui_screen(ui_shutdown_ui);
    epd_power_off();

    vTaskDelay(pdMS_TO_TICKS(500));
    sd_deinit();
    tf_power_off();

    gpio_set_level(POWER_OFF_PIN, 1);
    //while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); } // TODO
}

// SD CARD INIT / DEINIT
static void sd_deinit(void)
{
    if (g_sd_mounted && g_sd_card != NULL) {
        esp_err_t ret = esp_vfs_fat_sdcard_unmount(MOUNT_POINT, g_sd_card);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Unmount returned: %s", esp_err_to_name(ret));
        }
        g_sd_mounted = false;
        g_sd_card    = NULL;
    }
    spi_bus_free(SPI3_HOST);
    tf_power_off();
}

static bool sd_init(void)
{
    tf_power_on();

    spi_bus_config_t sdbus_cfg = {
        .mosi_io_num     = 40,
        .miso_io_num     = 13,
        .sclk_io_num     = 39,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 16 * 1024,
    };

    esp_err_t ret;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI3_HOST;

    ret = spi_bus_initialize(SPI3_HOST, &sdbus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
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
        spi_bus_free(host.slot);
        g_sd_mounted = false;
        g_sd_card    = NULL;
        return false;
    }

    g_sd_mounted = true;
    sdmmc_card_print_info(stdout, g_sd_card);
    ESP_LOGI(TAG, "SD mounted at %s", MOUNT_POINT);
    return true;
}

// IMAGE LOADING
static bool load_images(const char *full_filepath)
{
    ESP_LOGI(TAG, "Opening image: %s", full_filepath);

    FILE *f = fopen(full_filepath, "rb");
    if (f == NULL) {
        ESP_LOGE(TAG, "fopen failed — SD card may have been removed");
        return false;
    }

    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    rewind(f);

    ESP_LOGI(TAG, "File size: %ld bytes (expected: %d bytes)", file_size, IMAGE_SIZE);

    if (file_size != (long)IMAGE_SIZE) {
        ESP_LOGE(TAG, "Image rejected — size mismatch: expected %d, got %ld bytes",
                 IMAGE_SIZE, file_size);
        fclose(f);
        return false;
    }

    size_t bytes_read = fread(g_loaded_image, 1, IMAGE_SIZE, f);
    fclose(f);

    if (bytes_read != IMAGE_SIZE) {
        ESP_LOGE(TAG, "Read error: only %d of %d bytes read",
                 (int)bytes_read, IMAGE_SIZE);
        return false;
    }

    ESP_LOGI(TAG, "Image loaded OK");
    return true;
}

// DIRECTORY SCANNING
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
    if (name == NULL || name[0] == '\0') {
        return true;
    }

    if (strchr(name, '~') != NULL) {
        ESP_LOGD(TAG, "Skipping system/unresolved entry: %s", name);
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

/*
 * scan_directory — populate listing with visible folders and .bin files.
 *
 */
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
            ESP_LOGW(TAG, "stat failed for: %s", full_path);
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

// NAVIGATION STACK
static bool nav_push(const char *path, int selected, int scroll)
{
    if (g_nav_depth >= MAX_STACK_DEPTH) {
        ESP_LOGW(TAG, "Nav stack full — cannot descend further");
        return false;
    }
    strncpy(g_nav_stack[g_nav_depth].path, path,
            sizeof(g_nav_stack[0].path) - 1);
    g_nav_stack[g_nav_depth].path[sizeof(g_nav_stack[0].path) - 1] = '\0';
    g_nav_stack[g_nav_depth].selected      = selected;
    g_nav_stack[g_nav_depth].scroll_offset = scroll;
    g_nav_depth++;
    return true;
}

static bool nav_pop(void)
{
    if (g_nav_depth <= 0) return false;
    g_nav_depth--;
    return true;
}

static void nav_enter_folder(const char *folder_name)
{
    if (folder_name == NULL || folder_name[0] == '\0') {
        ESP_LOGE(TAG, "nav_enter_folder: rejected empty folder name");
        return;
    }

    if (!nav_push(g_current_path, g_dir.selected, g_dir.scroll_offset)) {
        return;
    }

    char new_path[256];
    int written = snprintf(new_path, sizeof(new_path),
                           "%s/%s", g_current_path, folder_name);
    if (written >= (int)sizeof(new_path)) {
        ESP_LOGE(TAG, "Path too long — cannot enter folder: %s/%s",
                 g_current_path, folder_name);
        g_nav_depth--;
        return;
    }

    strncpy(g_current_path, new_path, sizeof(g_current_path) - 1);
    g_current_path[sizeof(g_current_path) - 1] = '\0';

    memset(&g_dir, 0, sizeof(g_dir));
    scan_directory(g_current_path, &g_dir);

    g_screen_dirty = true;
    ESP_LOGI(TAG, "Entered: %s", g_current_path);
}

static void nav_go_back(void)
{
    if (g_nav_depth == 0) {
        ESP_LOGI(TAG, "BACK at root no-op");
        return;
    }

    nav_pop();
    nav_frame_t *frame = &g_nav_stack[g_nav_depth];

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
    ESP_LOGI(TAG, "Returned to: %s", g_current_path);
}

static void nav_select_item(void)
{
    if (g_dir.count == 0) return;

    dir_entry_t *item = &g_dir.entries[g_dir.selected];

    if (item->type == ENTRY_FOLDER) {
        nav_enter_folder(item->name);
    }
    else if (item->type == ENTRY_FILE) {
        char full_path[320];

        int written = snprintf(full_path, sizeof(full_path),
                               "%s/%s", g_current_path, item->name);
        if (written >= (int)sizeof(full_path)) {
            ESP_LOGE(TAG, "File path too long: %s/%s",
                     g_current_path, item->name);
            return;
        }

        epd_power_on();
        clear_all();
        displayui_screen(ui_loading_ui);
        epd_power_off();

        if (load_images(full_path)) {
            transition_to(STATE_IMAGE_VIEW);
        } else {
            ESP_LOGE(TAG, "Image load failed: %s", full_path);
            memset(&g_dir, 0, sizeof(g_dir));
            if (!scan_directory(g_current_path, &g_dir)) {
                ESP_LOGW(TAG, "SD card appears removed — going to NO_SD");
                transition_to(STATE_NO_SD);
            } else {
                g_screen_dirty = true;
            }
        }
    }
}

// DISPLAY FUNCTIONS
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

static void display_bin_image(void)
{
    epd_power_on();
    vTaskDelay(pdMS_TO_TICKS(50));
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

static void render_directory_view(void)
{
    epd_power_on();
    vTaskDelay(pdMS_TO_TICKS(50));
    clear_all();

    Paint_NewImage(g_image_bw, EPD_W, EPD_H, 0, WHITE);
    EPD_Full(WHITE);
    vTaskDelay(pdMS_TO_TICKS(20));

    EPD_ShowPicture(0, 0, EPD_W, EPD_H, ui_directory_view_ui, BLACK);
    vTaskDelay(pdMS_TO_TICKS(20));

    const char *disp_path = g_current_path + strlen(MOUNT_POINT);
    if (*disp_path == '\0') disp_path = "/";
    EPD_ShowString(8, 12, (char *)disp_path, FONT_SIZE, WHITE);
    vTaskDelay(pdMS_TO_TICKS(20));

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
}
