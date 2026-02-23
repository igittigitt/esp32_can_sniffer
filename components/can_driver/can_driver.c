/**
 * CAN Driver Component - Implementation
 *
 * ESP32 TWAI controller wrapper for 3x TJA1042 transceivers.
 * STB-pin based bus selection: only one transceiver active at a time.
 */

#include "can_driver.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "CAN_DRIVER";

// ═══════════════════════════════════════════════════════════════════
// Internal State
// ═══════════════════════════════════════════════════════════════════

static can_hw_config_t  s_hw        = {0};
static can_config_t     s_cfg       = {0};
static can_bus_t        s_active    = CAN_BUS_NONE;
static bool             s_twai_up   = false;
static bool             s_initialized = false;

static can_rx_callback_t s_rx_cb    = NULL;
static void             *s_rx_ud    = NULL;

static can_stats_t      s_stats     = {0};
static SemaphoreHandle_t s_mutex    = NULL;

static TaskHandle_t     s_rx_task   = NULL;

// ═══════════════════════════════════════════════════════════════════
// Internal Helpers
// ═══════════════════════════════════════════════════════════════════

static int stb_pin_for_bus(can_bus_t bus)
{
    switch (bus) {
        case CAN_BUS_MS: return s_hw.stb_ms_gpio;
        case CAN_BUS_HS: return s_hw.stb_hs_gpio;
        case CAN_BUS_MM: return s_hw.stb_mm_gpio;
        default:         return -1;
    }
}

static uint32_t bitrate_for_bus(can_bus_t bus)
{
    switch (bus) {
        case CAN_BUS_MS: return s_cfg.bitrate_ms_kbps;
        case CAN_BUS_HS: return s_cfg.bitrate_hs_kbps;
        case CAN_BUS_MM: return s_cfg.bitrate_mm_kbps;
        default:         return 500;
    }
}

static void set_bitrate_for_bus(can_bus_t bus, uint32_t kbps)
{
    switch (bus) {
        case CAN_BUS_MS: s_cfg.bitrate_ms_kbps = kbps; break;
        case CAN_BUS_HS: s_cfg.bitrate_hs_kbps = kbps; break;
        case CAN_BUS_MM: s_cfg.bitrate_mm_kbps = kbps; break;
        default: break;
    }
}

/**
 * @brief Map kbps to TWAI timing config.
 *        Returns false for unsupported rates.
 */
static bool get_timing_config(uint32_t kbps, twai_timing_config_t *out)
{
    switch (kbps) {
        case 125:  { twai_timing_config_t t = TWAI_TIMING_CONFIG_125KBITS();  *out = t; return true; }
        case 250:  { twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS();  *out = t; return true; }
        case 500:  { twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();  *out = t; return true; }
        case 1000: { twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();    *out = t; return true; }
        default:
            ESP_LOGE(TAG, "Unsupported bitrate: %lu kbps", (unsigned long)kbps);
            return false;
    }
}

// ═══════════════════════════════════════════════════════════════════
// TWAI Start / Stop (internal, called with mutex held or at init)
// ═══════════════════════════════════════════════════════════════════

static esp_err_t twai_stop_internal(void)
{
    if (!s_twai_up) return ESP_OK;

    esp_err_t ret = twai_stop();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "twai_stop: %s", esp_err_to_name(ret));
    }
    ret = twai_driver_uninstall();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "twai_driver_uninstall: %s", esp_err_to_name(ret));
    }
    s_twai_up = false;
    ESP_LOGD(TAG, "TWAI stopped");
    return ESP_OK;
}

static esp_err_t twai_start_internal(can_bus_t bus)
{
    uint32_t kbps = bitrate_for_bus(bus);

    twai_timing_config_t t_config;
    if (!get_timing_config(kbps, &t_config)) {
        return ESP_ERR_INVALID_ARG;
    }

    twai_general_config_t g_config = {
        .mode           = s_cfg.listen_only ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL,
        .tx_io          = s_hw.tx_gpio,
        .rx_io          = s_hw.rx_gpio,
        .clkout_io      = TWAI_IO_UNUSED,
        .bus_off_io     = TWAI_IO_UNUSED,
        .tx_queue_len   = 10,
        .rx_queue_len   = 64,
        .alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_RX_QUEUE_FULL,
        .clkout_divider = 0,
        .intr_flags     = ESP_INTR_FLAG_LEVEL1,
    };

    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "twai_driver_install: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = twai_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "twai_start: %s", esp_err_to_name(ret));
        twai_driver_uninstall();
        return ret;
    }

    s_twai_up = true;
    ESP_LOGI(TAG, "TWAI started: %s bus, %lu kbps, %s",
             can_bus_name(bus), (unsigned long)kbps,
             s_cfg.listen_only ? "LISTEN-ONLY" : "NORMAL");
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════════
// RX Task
// ═══════════════════════════════════════════════════════════════════

static void can_rx_task(void *pvParameters)
{
    twai_message_t msg;

    ESP_LOGI(TAG, "RX task started");

    while (1) {
        // Block up to 100ms; loop so we can check for bus-off alerts too
        esp_err_t ret = twai_receive(&msg, pdMS_TO_TICKS(100));

        if (ret == ESP_OK) {
            // Skip remote frames (no data)
            if (msg.rtr) continue;

            s_stats.rx_frames++;

            if (s_rx_cb) {
                s_rx_cb(msg.identifier, msg.data, (uint8_t)msg.data_length_code,
                        esp_timer_get_time(), s_rx_ud);
            }
        } else if (ret == ESP_ERR_TIMEOUT) {
            // Normal - check for alerts
            uint32_t alerts = 0;
            if (twai_read_alerts(&alerts, 0) == ESP_OK) {
                if (alerts & TWAI_ALERT_BUS_OFF) {
                    s_stats.bus_off_count++;
                    ESP_LOGW(TAG, "Bus-Off detected! Recovering...");
                    twai_initiate_recovery();
                }
                if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
                    s_stats.rx_missed++;
                    ESP_LOGW(TAG, "RX queue full - frames dropped!");
                }
            }
        } else if (ret == ESP_ERR_INVALID_STATE) {
            // TWAI not running - task sleeps until bus is switched on
            vTaskDelay(pdMS_TO_TICKS(50));
        } else {
            s_stats.rx_errors++;
            ESP_LOGD(TAG, "twai_receive error: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// ═══════════════════════════════════════════════════════════════════
// STB GPIO Init
// ═══════════════════════════════════════════════════════════════════

static void stb_gpio_init(void)
{
    int stb_pins[] = { s_hw.stb_ms_gpio, s_hw.stb_hs_gpio, s_hw.stb_mm_gpio };
    for (int i = 0; i < 3; i++) {
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << stb_pins[i]),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&io);
        // All transceivers off at startup
        gpio_set_level(stb_pins[i], CAN_STB_STANDBY);
    }
    ESP_LOGD(TAG, "STB pins initialized (all STANDBY)");
}

static void stb_select_bus(can_bus_t bus)
{
    // All off first
    gpio_set_level(s_hw.stb_ms_gpio, CAN_STB_STANDBY);
    gpio_set_level(s_hw.stb_hs_gpio, CAN_STB_STANDBY);
    gpio_set_level(s_hw.stb_mm_gpio, CAN_STB_STANDBY);

    if (bus == CAN_BUS_NONE) return;

    int pin = stb_pin_for_bus(bus);
    if (pin >= 0) {
        gpio_set_level(pin, CAN_STB_ACTIVE);
        ESP_LOGD(TAG, "STB active: %s (GPIO %d)", can_bus_name(bus), pin);
    }
}

// ═══════════════════════════════════════════════════════════════════
// Public API
// ═══════════════════════════════════════════════════════════════════

esp_err_t can_driver_init(const can_hw_config_t *hw, const can_config_t *cfg)
{
    if (!hw || !cfg) return ESP_ERR_INVALID_ARG;

    s_hw  = *hw;
    s_cfg = *cfg;
    s_active = CAN_BUS_NONE;
    s_twai_up = false;

    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;

    stb_gpio_init();

    // Start RX task - it handles TWAI not running gracefully
    xTaskCreate(can_rx_task, "can_rx", 4096, NULL, 12, &s_rx_task);

    s_initialized = true;
    ESP_LOGI(TAG, "CAN driver initialized (all buses off)");
    return ESP_OK;
}

esp_err_t can_driver_deinit(void)
{
    if (!s_initialized) return ESP_OK;

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    twai_stop_internal();
    stb_select_bus(CAN_BUS_NONE);
    s_active = CAN_BUS_NONE;

    xSemaphoreGive(s_mutex);

    if (s_rx_task) {
        vTaskDelete(s_rx_task);
        s_rx_task = NULL;
    }

    s_initialized = false;
    ESP_LOGI(TAG, "CAN driver deinitialized");
    return ESP_OK;
}

esp_err_t can_set_bus(can_bus_t bus)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    ESP_LOGI(TAG, "Bus switch: %s → %s",
             can_bus_name(s_active), can_bus_name(bus));

    // 1. Stop TWAI
    twai_stop_internal();

    // 2. All STB pins off
    stb_select_bus(CAN_BUS_NONE);
    s_active = CAN_BUS_NONE;

    if (bus == CAN_BUS_NONE) {
        xSemaphoreGive(s_mutex);
        ESP_LOGI(TAG, "All buses off");
        return ESP_OK;
    }

    // 3. Activate target transceiver
    stb_select_bus(bus);
    vTaskDelay(pdMS_TO_TICKS(CAN_BUS_SWITCH_DELAY_MS));

    // 4. Start TWAI with correct bitrate
    esp_err_t ret = twai_start_internal(bus);
    if (ret == ESP_OK) {
        s_active = bus;
        s_cfg.active_bus = bus;
    } else {
        // Rollback: put transceiver back to standby
        stb_select_bus(CAN_BUS_NONE);
        ESP_LOGE(TAG, "Failed to start TWAI for %s bus", can_bus_name(bus));
    }

    xSemaphoreGive(s_mutex);
    return ret;
}

can_bus_t can_get_active_bus(void)
{
    return s_active;
}

const char *can_bus_name(can_bus_t bus)
{
    switch (bus) {
        case CAN_BUS_MS:   return "MS";
        case CAN_BUS_HS:   return "HS";
        case CAN_BUS_MM:   return "MM";
        case CAN_BUS_NONE: return "OFF";
        default:           return "???";
    }
}

esp_err_t can_set_listen_only(bool enable)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    s_cfg.listen_only = enable;

    // Restart TWAI if a bus is active
    if (s_active != CAN_BUS_NONE) {
        can_bus_t current = s_active;
        twai_stop_internal();
        vTaskDelay(pdMS_TO_TICKS(5));
        twai_start_internal(current);
    }

    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "Mode: %s", enable ? "LISTEN-ONLY" : "NORMAL");
    return ESP_OK;
}

bool can_get_listen_only(void)
{
    return s_cfg.listen_only;
}

esp_err_t can_set_bitrate(can_bus_t bus, uint32_t kbps)
{
    if (bus == CAN_BUS_NONE) return ESP_ERR_INVALID_ARG;

    // Validate
    twai_timing_config_t dummy;
    if (!get_timing_config(kbps, &dummy)) return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    set_bitrate_for_bus(bus, kbps);

    // If this bus is currently active, restart TWAI with new bitrate
    if (s_active == bus) {
        twai_stop_internal();
        vTaskDelay(pdMS_TO_TICKS(5));
        twai_start_internal(bus);
    }

    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "Bitrate %s -> %lu kbps", can_bus_name(bus), (unsigned long)kbps);
    return ESP_OK;
}

uint32_t can_get_bitrate(can_bus_t bus)
{
    return bitrate_for_bus(bus);
}

void can_set_rx_callback(can_rx_callback_t cb, void *user_data)
{
    s_rx_cb = cb;
    s_rx_ud = user_data;
}

esp_err_t can_send_frame(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    if (!s_initialized)            return ESP_ERR_INVALID_STATE;
    if (s_cfg.listen_only)         return ESP_ERR_INVALID_STATE;
    if (s_active == CAN_BUS_NONE)  return ESP_ERR_INVALID_STATE;
    if (!s_twai_up)                return ESP_ERR_INVALID_STATE;
    if (id > 0x7FF)                return ESP_ERR_INVALID_ARG;
    if (dlc > CAN_MAX_DATA_LEN)    return ESP_ERR_INVALID_ARG;

    twai_message_t msg = {
        .identifier     = id,
        .data_length_code = dlc,
        .extd           = 0,    // Standard 11-bit
        .rtr            = 0,
        .ss             = 0,
        .self            = 0,
        .dlc_non_comp   = 0,
    };
    if (data && dlc > 0) {
        memcpy(msg.data, data, dlc);
    }

    esp_err_t ret = twai_transmit(&msg, pdMS_TO_TICKS(100));
    if (ret == ESP_OK) {
        s_stats.tx_frames++;
    } else {
        ESP_LOGW(TAG, "TX failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

void can_get_stats(can_stats_t *out)
{
    if (out) *out = s_stats;
}

void can_reset_stats(void)
{
    memset(&s_stats, 0, sizeof(s_stats));
}
