/**
 * CAN Driver Component
 *
 * Wraps the ESP32 TWAI controller for CAN bus sniffing.
 * Supports 3x TJA1042 transceivers, switchable via STB pins.
 *
 * Features:
 * - 3 selectable CAN buses (MS / HS / MM) via STB pin control
 * - Configurable bitrate per bus (NVS-persistent)
 * - Listen-Only / Normal mode switchable at runtime
 * - Callback-based RX (Standard 11-bit frames only)
 * - TX support in Normal mode
 */

#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ═══════════════════════════════════════════════════════════════════
// Constants
// ═══════════════════════════════════════════════════════════════════

#define CAN_MAX_DATA_LEN        8
#define CAN_STB_ACTIVE          0       // TJA1042: STB LOW  = Normal mode
#define CAN_STB_STANDBY         1       // TJA1042: STB HIGH = Standby mode
#define CAN_BUS_SWITCH_DELAY_MS 10      // Settling time after STB switch

// ═══════════════════════════════════════════════════════════════════
// Types
// ═══════════════════════════════════════════════════════════════════

/**
 * @brief CAN bus selector
 */
typedef enum {
    CAN_BUS_NONE = 0,   ///< All transceivers off
    CAN_BUS_MS,         ///< Medium Speed  (e.g. 125 kbps, body/comfort)
    CAN_BUS_HS,         ///< High Speed    (e.g. 500 kbps, powertrain)
    CAN_BUS_MM,         ///< Multimedia    (e.g. 250 kbps, infotainment)
} can_bus_t;

/**
 * @brief Hardware GPIO configuration for CAN driver
 */
typedef struct {
    int tx_gpio;        ///< TWAI TX pin (shared by all transceivers)
    int rx_gpio;        ///< TWAI RX pin (shared by all transceivers)
    int stb_ms_gpio;    ///< STB pin for MS transceiver
    int stb_hs_gpio;    ///< STB pin for HS transceiver
    int stb_mm_gpio;    ///< STB pin for MM transceiver
} can_hw_config_t;

/**
 * @brief Per-bus configuration (bitrate, persisted in NVS)
 */
typedef struct {
    uint32_t bitrate_ms_kbps;   ///< Bitrate for MS bus in kbps
    uint32_t bitrate_hs_kbps;   ///< Bitrate for HS bus in kbps
    uint32_t bitrate_mm_kbps;   ///< Bitrate for MM bus in kbps
    can_bus_t active_bus;        ///< Bus active on last save
    bool listen_only;            ///< Listen-only mode
} can_config_t;

/**
 * @brief RX callback - called from TWAI RX task on each received frame
 *
 * @param id          CAN frame ID (11-bit standard)
 * @param data        Payload bytes
 * @param dlc         Data length (0-8)
 * @param timestamp_us esp_timer_get_time() at reception
 * @param user_data   Opaque pointer passed during registration
 */
typedef void (*can_rx_callback_t)(uint32_t id, const uint8_t *data,
                                   uint8_t dlc, uint64_t timestamp_us,
                                   void *user_data);

// ═══════════════════════════════════════════════════════════════════
// Init / Deinit
// ═══════════════════════════════════════════════════════════════════

/**
 * @brief Initialize CAN driver (GPIOs + STB pins, does NOT start TWAI yet).
 *        Call can_set_bus() afterwards to activate a bus.
 *
 * @param hw      GPIO configuration
 * @param cfg     Bus configuration (bitraten, mode)
 * @return ESP_OK on success
 */
esp_err_t can_driver_init(const can_hw_config_t *hw, const can_config_t *cfg);

/**
 * @brief Stop TWAI, put all transceivers into standby.
 */
esp_err_t can_driver_deinit(void);

// ═══════════════════════════════════════════════════════════════════
// Bus Selection
// ═══════════════════════════════════════════════════════════════════

/**
 * @brief Switch to a different CAN bus.
 *        Stops TWAI, switches STB pins, restarts TWAI with correct bitrate.
 *        CAN_BUS_NONE stops everything.
 *
 * @param bus  Target bus
 * @return ESP_OK on success
 */
esp_err_t can_set_bus(can_bus_t bus);

/**
 * @brief Get currently active bus.
 */
can_bus_t can_get_active_bus(void);

/**
 * @brief Human-readable bus name ("MS", "HS", "MM", "OFF")
 */
const char *can_bus_name(can_bus_t bus);

// ═══════════════════════════════════════════════════════════════════
// Mode
// ═══════════════════════════════════════════════════════════════════

/**
 * @brief Switch between Listen-Only and Normal mode.
 *        Restarts TWAI controller if a bus is currently active.
 *
 * @param enable  true = listen-only (no ACK), false = normal
 * @return ESP_OK on success
 */
esp_err_t can_set_listen_only(bool enable);

/**
 * @brief Query current mode.
 */
bool can_get_listen_only(void);

// ═══════════════════════════════════════════════════════════════════
// Bitrate
// ═══════════════════════════════════════════════════════════════════

/**
 * @brief Set bitrate for a specific bus (in kbps).
 *        Accepted values: 125, 250, 500, 1000.
 *        If the bus is currently active, TWAI is restarted immediately.
 *
 * @param bus       Target bus (must not be CAN_BUS_NONE)
 * @param kbps      New bitrate
 * @return ESP_OK or ESP_ERR_INVALID_ARG for unsupported rates
 */
esp_err_t can_set_bitrate(can_bus_t bus, uint32_t kbps);

/**
 * @brief Get configured bitrate for a bus (kbps).
 */
uint32_t can_get_bitrate(can_bus_t bus);

// ═══════════════════════════════════════════════════════════════════
// RX Callback
// ═══════════════════════════════════════════════════════════════════

/**
 * @brief Register RX callback. Called from TWAI RX task context.
 *        Only one callback supported; call before can_set_bus().
 */
void can_set_rx_callback(can_rx_callback_t cb, void *user_data);

// ═══════════════════════════════════════════════════════════════════
// TX
// ═══════════════════════════════════════════════════════════════════

/**
 * @brief Send a CAN frame. Only works in NORMAL mode with an active bus.
 *
 * @param id    11-bit CAN ID
 * @param data  Payload (max 8 bytes)
 * @param dlc   Data length (0-8)
 * @return ESP_OK, ESP_ERR_INVALID_STATE (listen-only/no bus), or TWAI error
 */
esp_err_t can_send_frame(uint32_t id, const uint8_t *data, uint8_t dlc);

// ═══════════════════════════════════════════════════════════════════
// Statistics
// ═══════════════════════════════════════════════════════════════════

typedef struct {
    uint32_t rx_frames;
    uint32_t tx_frames;
    uint32_t rx_errors;
    uint32_t bus_off_count;
    uint32_t rx_missed;     ///< Frames dropped due to full RX queue
} can_stats_t;

/**
 * @brief Get accumulated statistics.
 */
void can_get_stats(can_stats_t *out);

/**
 * @brief Reset statistics counters.
 */
void can_reset_stats(void);

#ifdef __cplusplus
}
#endif

#endif // CAN_DRIVER_H
