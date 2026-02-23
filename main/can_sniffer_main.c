/**
 * CAN Sniffer - ESP32-C6 with 3x TJA1042
 *
 * Features:
 * - Sniff 3 CAN buses (MS / HS / MM), switchable at runtime
 * - candump-compatible output format
 * - WiFi: NVS credentials with AP fallback
 * - Telnet: TCP command interface (port 2323)
 * - Listen-Only / Normal mode switchable
 * - Configurable bitrate per bus (NVS-persistent)
 * - TX support in Normal mode
 */

#include <stdio.h>
#include <string.h>
#include <strings.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "lwip/sockets.h"
#include "driver/gpio.h"

#include "can_driver.h"
#include "ring_buffer.h"
#include "web_server.h"
#include "led_indicator.h"

// Ausgabe-Makro: bei WS_FAKE_SOCK in WS-Puffer schreiben
#define CMD_SEND(s, buf, len) \
    do { \
        if ((s) == WS_FAKE_SOCK) ws_send_shim((s), (buf), (len)); \
        else send((s), (buf), (len), 0); \
    } while(0)

static const char *TAG = "CAN_SNIFFER";

// ═══════════════════════════════════════════════════════════════════
// Configuration
// ═══════════════════════════════════════════════════════════════════

#ifndef CONFIG_TCP_PORT
#define CONFIG_TCP_PORT 23
#endif

#ifndef CONFIG_MAX_TCP_CLIENTS
#define CONFIG_MAX_TCP_CLIENTS 3
#endif

#define TCP_PORT            CONFIG_TCP_PORT
#define MAX_CLIENTS         CONFIG_MAX_TCP_CLIENTS

// GPIO assignments (configurable via menuconfig)
#define CAN_TX_GPIO         CONFIG_CAN_TX_GPIO
#define CAN_RX_GPIO         CONFIG_CAN_RX_GPIO
#define CAN_STB_HS_GPIO     CONFIG_CAN_STB_HS_GPIO
#define CAN_STB_MS_GPIO     CONFIG_CAN_STB_MS_GPIO
#define CAN_STB_MM_GPIO     CONFIG_CAN_STB_MM_GPIO

// Default bitrates (configurable via menuconfig)
#define DEFAULT_BITRATE_HS  CONFIG_CAN_DEFAULT_BITRATE_HS
#define DEFAULT_BITRATE_MS  CONFIG_CAN_DEFAULT_BITRATE_MS
#define DEFAULT_BITRATE_MM  CONFIG_CAN_DEFAULT_BITRATE_MM

// NVS
#define NVS_NS_WIFI         "wifi_config"
#define NVS_NS_CAN          "can_config"
#define NVS_KEY_SSID        "ssid"
#define NVS_KEY_PASS        "password"
#define NVS_KEY_BR_MS       "br_ms"
#define NVS_KEY_BR_HS       "br_hs"
#define NVS_KEY_BR_MM       "br_mm"
#define NVS_KEY_ACTIVE_BUS  "active_bus"
#define NVS_KEY_LISTEN_ONLY "listen_only"

// ═══════════════════════════════════════════════════════════════════
// Global Variables
// ═══════════════════════════════════════════════════════════════════

static struct {
    uint32_t rx_frames;
    uint32_t tx_frames;
    uint32_t rx_errors;
    uint32_t bus_off_count;
} stats = {0};

static uint64_t boot_timestamp_us = 0;
static int      client_sockets[MAX_CLIENTS];
static int      client_count = 0;

// ═══════════════════════════════════════════════════════════════════
// Prototypes
// ═══════════════════════════════════════════════════════════════════

void broadcast_to_clients(const char *message, int len);
void output_candump(uint32_t id, const uint8_t *data, uint8_t dlc, uint64_t timestamp_us);
void can_rx_callback(uint32_t id, const uint8_t *data, uint8_t dlc,
                     uint64_t timestamp_us, void *user_data);

// NVS
bool wifi_get_credentials(char *ssid, size_t ssid_len, char *password, size_t pass_len);
bool wifi_set_credentials(const char *ssid, const char *password);
void can_nvs_load(can_config_t *cfg);
void can_nvs_save_bitrate(can_bus_t bus, uint32_t kbps);
void can_nvs_save_active_bus(can_bus_t bus);
void can_nvs_save_listen_only(bool enable);

// WiFi
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data);
void wifi_start_ap(void);
void wifi_connect_sta(const char *ssid, const char *password);
void wifi_init(void);

// Commands
void parse_command(char *cmd, int sock);

// Tasks
void client_handler_task(void *pvParameters);
void tcp_server_task(void *pvParameters);
void statistics_task(void *pvParameters);

// ═══════════════════════════════════════════════════════════════════
// Helper: Broadcast + candump Output
// ═══════════════════════════════════════════════════════════════════

void broadcast_to_clients(const char *message, int len)
{
    (void)len;
    ringbuf_push(message);
}

void output_candump(uint32_t id, const uint8_t *data, uint8_t dlc, uint64_t timestamp_us)
{
    char buf[64];
    double ts = (timestamp_us - boot_timestamp_us) / 1000000.0;

    // Standard candump format: (ts) can0 ID#DATA
    int pos = snprintf(buf, sizeof(buf), "(%.6f) can0 %03lX#", ts, (unsigned long)id);

    for (int i = 0; i < dlc; i++) {
        pos += snprintf(buf + pos, sizeof(buf) - pos, "%02X", data[i]);
    }
    pos += snprintf(buf + pos, sizeof(buf) - pos, "\r\n");

    broadcast_to_clients(buf, pos);
}

// ═══════════════════════════════════════════════════════════════════
// CAN RX Callback
// ═══════════════════════════════════════════════════════════════════

void can_rx_callback(uint32_t id, const uint8_t *data, uint8_t dlc,
                     uint64_t timestamp_us, void *user_data)
{
    stats.rx_frames++;
    led_indicator_send(LED_EVENT_LIN_RX);
    output_candump(id, data, dlc, timestamp_us);
    ESP_LOGD(TAG, "RX: ID 0x%03lX DLC %d", (unsigned long)id, dlc);
}

// ═══════════════════════════════════════════════════════════════════
// NVS: WiFi Credentials
// ═══════════════════════════════════════════════════════════════════

bool wifi_get_credentials(char *ssid, size_t ssid_len, char *password, size_t pass_len)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS_WIFI, NVS_READONLY, &h) != ESP_OK) return false;

    size_t sz = ssid_len;
    bool ok = (nvs_get_str(h, NVS_KEY_SSID, ssid, &sz) == ESP_OK);
    sz = pass_len;
    ok = ok && (nvs_get_str(h, NVS_KEY_PASS, password, &sz) == ESP_OK);

    nvs_close(h);
    return ok;
}

bool wifi_set_credentials(const char *ssid, const char *password)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS_WIFI, NVS_READWRITE, &h) != ESP_OK) return false;

    bool ok = (nvs_set_str(h, NVS_KEY_SSID, ssid) == ESP_OK &&
               nvs_set_str(h, NVS_KEY_PASS, password) == ESP_OK &&
               nvs_commit(h) == ESP_OK);
    nvs_close(h);
    return ok;
}

// ═══════════════════════════════════════════════════════════════════
// NVS: CAN Configuration
// ═══════════════════════════════════════════════════════════════════

void can_nvs_load(can_config_t *cfg)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS_CAN, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "No CAN NVS config found, using defaults");
        return;
    }

    uint32_t val;
    uint8_t  u8;

    if (nvs_get_u32(h, NVS_KEY_BR_MS, &val) == ESP_OK) cfg->bitrate_ms_kbps = val;
    if (nvs_get_u32(h, NVS_KEY_BR_HS, &val) == ESP_OK) cfg->bitrate_hs_kbps = val;
    if (nvs_get_u32(h, NVS_KEY_BR_MM, &val) == ESP_OK) cfg->bitrate_mm_kbps = val;
    if (nvs_get_u8 (h, NVS_KEY_ACTIVE_BUS,  &u8) == ESP_OK) cfg->active_bus   = (can_bus_t)u8;
    if (nvs_get_u8 (h, NVS_KEY_LISTEN_ONLY, &u8) == ESP_OK) cfg->listen_only  = (bool)u8;

    nvs_close(h);
    ESP_LOGI(TAG, "CAN NVS loaded: MS=%lukbps HS=%lukbps MM=%lukbps bus=%s %s",
             (unsigned long)cfg->bitrate_ms_kbps,
             (unsigned long)cfg->bitrate_hs_kbps,
             (unsigned long)cfg->bitrate_mm_kbps,
             can_bus_name(cfg->active_bus),
             cfg->listen_only ? "LISTEN-ONLY" : "NORMAL");
}

static bool can_nvs_open_rw(nvs_handle_t *h)
{
    return nvs_open(NVS_NS_CAN, NVS_READWRITE, h) == ESP_OK;
}

void can_nvs_save_bitrate(can_bus_t bus, uint32_t kbps)
{
    nvs_handle_t h;
    if (!can_nvs_open_rw(&h)) return;

    const char *key = NULL;
    switch (bus) {
        case CAN_BUS_MS: key = NVS_KEY_BR_MS; break;
        case CAN_BUS_HS: key = NVS_KEY_BR_HS; break;
        case CAN_BUS_MM: key = NVS_KEY_BR_MM; break;
        default: nvs_close(h); return;
    }
    nvs_set_u32(h, key, kbps);
    nvs_commit(h);
    nvs_close(h);
}

void can_nvs_save_active_bus(can_bus_t bus)
{
    nvs_handle_t h;
    if (!can_nvs_open_rw(&h)) return;
    nvs_set_u8(h, NVS_KEY_ACTIVE_BUS, (uint8_t)bus);
    nvs_commit(h);
    nvs_close(h);
}

void can_nvs_save_listen_only(bool enable)
{
    nvs_handle_t h;
    if (!can_nvs_open_rw(&h)) return;
    nvs_set_u8(h, NVS_KEY_LISTEN_ONLY, (uint8_t)enable);
    nvs_commit(h);
    nvs_close(h);
}

// ═══════════════════════════════════════════════════════════════════
// Command Parser
// ═══════════════════════════════════════════════════════════════════

void parse_command(char *cmd, int sock)
{
    char response[640];

    // Strip CR/LF
    for (char *p = cmd; *p; p++) {
        if (*p == '\r' || *p == '\n') *p = '\0';
    }
    if (strlen(cmd) == 0) return;

    ESP_LOGI(TAG, "CMD: '%s'", cmd);

    // ── CANBUS <MS|HS|MM|OFF> ─────────────────────────────────────
    if (strncasecmp(cmd,"CANBUS ", 7) == 0) {
        const char *arg = cmd + 7;
        can_bus_t bus;

        if      (strcasecmp(arg,"MS")  == 0) bus = CAN_BUS_MS;
        else if (strcasecmp(arg,"HS")  == 0) bus = CAN_BUS_HS;
        else if (strcasecmp(arg,"MM")  == 0) bus = CAN_BUS_MM;
        else if (strcasecmp(arg,"OFF") == 0) bus = CAN_BUS_NONE;
        else {
            snprintf(response, sizeof(response),
                     "ERROR: CANBUS <MS|HS|MM|OFF>\r\n");
            CMD_SEND(sock, response, strlen(response));
            return;
        }

        esp_err_t ret = can_set_bus(bus);
        if (ret == ESP_OK) {
            can_nvs_save_active_bus(bus);
            can_reset_stats();
            snprintf(response, sizeof(response),
                     "OK: Bus switched to %s (%lu kbps, %s)\r\n",
                     can_bus_name(bus),
                     (unsigned long)can_get_bitrate(bus),
                     can_get_listen_only() ? "LISTEN-ONLY" : "NORMAL");
        } else {
            snprintf(response, sizeof(response),
                     "ERROR: Bus switch failed (%s)\r\n", esp_err_to_name(ret));
        }
        CMD_SEND(sock, response, strlen(response));
    }

    // ── BITRATE <MS|HS|MM> <125|250|500|1000> ────────────────────
    else if (strncasecmp(cmd,"BITRATE ", 8) == 0) {
        char bus_str[8] = {0};
        unsigned long kbps_ul = 0;

        if (sscanf(cmd + 8, "%7s %lu", bus_str, &kbps_ul) != 2) {
            snprintf(response, sizeof(response),
                     "ERROR: BITRATE <MS|HS|MM> <125|250|500|1000>\r\n");
            CMD_SEND(sock, response, strlen(response));
            return;
        }
        uint32_t kbps = (uint32_t)kbps_ul;

        can_bus_t bus;
        if      (strcasecmp(bus_str,"MS") == 0) bus = CAN_BUS_MS;
        else if (strcasecmp(bus_str,"HS") == 0) bus = CAN_BUS_HS;
        else if (strcasecmp(bus_str,"MM") == 0) bus = CAN_BUS_MM;
        else {
            snprintf(response, sizeof(response),
                     "ERROR: Bus must be MS, HS or MM\r\n");
            CMD_SEND(sock, response, strlen(response));
            return;
        }

        esp_err_t ret = can_set_bitrate(bus, kbps);
        if (ret == ESP_OK) {
            can_nvs_save_bitrate(bus, kbps);
            snprintf(response, sizeof(response),
                     "OK: %s bitrate set to %lu kbps%s\r\n",
                     can_bus_name(bus), (unsigned long)kbps,
                     (can_get_active_bus() == bus) ? " (active, restarted)" : " (saved)");
        } else {
            snprintf(response, sizeof(response),
                     "ERROR: Unsupported bitrate %lu kbps\r\n", (unsigned long)kbps);
        }
        CMD_SEND(sock, response, strlen(response));
    }

    // ── MODE <LISTEN|NORMAL> ──────────────────────────────────────
    else if (strncasecmp(cmd,"MODE ", 5) == 0) {
        const char *arg = cmd + 5;
        bool listen;

        if      (strcasecmp(arg,"LISTEN") == 0) listen = true;
        else if (strcasecmp(arg,"NORMAL") == 0) listen = false;
        else {
            snprintf(response, sizeof(response),
                     "ERROR: MODE <LISTEN|NORMAL>\r\n");
            CMD_SEND(sock, response, strlen(response));
            return;
        }

        can_set_listen_only(listen);
        can_nvs_save_listen_only(listen);
        snprintf(response, sizeof(response),
                 "OK: Mode set to %s\r\n", listen ? "LISTEN-ONLY" : "NORMAL");
        CMD_SEND(sock, response, strlen(response));
    }

    // ── SEND <ID> [B0 B1 ... B7] ─────────────────────────────────
    else if (strncasecmp(cmd,"SEND ", 5) == 0) {
        if (can_get_listen_only()) {
            snprintf(response, sizeof(response),
                     "ERROR: Cannot send in LISTEN-ONLY mode\r\n");
            CMD_SEND(sock, response, strlen(response));
            return;
        }
        if (can_get_active_bus() == CAN_BUS_NONE) {
            snprintf(response, sizeof(response),
                     "ERROR: No active bus (use CANBUS MS/HS/MM first)\r\n");
            CMD_SEND(sock, response, strlen(response));
            return;
        }

        char *token = strtok(cmd + 5, " ");
        if (!token) {
            snprintf(response, sizeof(response),
                     "ERROR: SEND <ID> [B0..B7]\r\n");
            CMD_SEND(sock, response, strlen(response));
            return;
        }

        uint32_t id = (uint32_t)strtoul(token, NULL, 16);
        if (id > 0x7FF) {
            snprintf(response, sizeof(response),
                     "ERROR: ID must be 0x000-0x7FF (11-bit)\r\n");
            CMD_SEND(sock, response, strlen(response));
            return;
        }

        uint8_t data[8];
        uint8_t dlc = 0;
        while ((token = strtok(NULL, " ")) != NULL && dlc < 8) {
            data[dlc++] = (uint8_t)strtoul(token, NULL, 16);
        }

        esp_err_t ret = can_send_frame(id, data, dlc);
        if (ret == ESP_OK) {
            stats.tx_frames++;
            led_indicator_send(LED_EVENT_LIN_TX);
            output_candump(id, data, dlc, esp_timer_get_time());
            snprintf(response, sizeof(response), "OK\r\n");
        } else {
            snprintf(response, sizeof(response),
                     "ERROR: TX failed (%s)\r\n", esp_err_to_name(ret));
        }
        CMD_SEND(sock, response, strlen(response));
    }

    // ── WIFI <SSID> <PASSWORD> ────────────────────────────────────
    else if (strncasecmp(cmd,"WIFI ", 5) == 0) {
        char *ssid = strtok(cmd + 5, " ");
        char *pass = strtok(NULL, " ");

        if (!ssid || !pass) {
            snprintf(response, sizeof(response),
                     "ERROR: WIFI <SSID> <PASSWORD>\r\n");
        } else if (wifi_set_credentials(ssid, pass)) {
            snprintf(response, sizeof(response),
                     "OK: WiFi saved, type REBOOT to connect\r\n");
        } else {
            snprintf(response, sizeof(response), "ERROR: NVS write failed\r\n");
        }
        CMD_SEND(sock, response, strlen(response));
    }

    // ── STATUS ────────────────────────────────────────────────────
    else if (strcasecmp(cmd,"STATUS") == 0) {
        can_stats_t cs;
        can_get_stats(&cs);

        int pos = snprintf(response, sizeof(response),
            "+----------------------------------+\r\n"
            "|       CAN Sniffer Status         |\r\n"
            "+----------------------------------+\r\n"
            "| Active Bus : %-4s  (%lu kbps)%*s|\r\n"
            "| Mode       : %-22s|\r\n"
            "| Bitraten   : MS=%3lukbps HS=%3lukbps MM=%3lukbps |\r\n"
            "+----------------------------------+\r\n"
            "| RX Frames  : %-20lu|\r\n"
            "| TX Frames  : %-20lu|\r\n"
            "| RX Errors  : %-20lu|\r\n"
            "| Bus-Off    : %-20lu|\r\n"
            "| RX Missed  : %-20lu|\r\n"
            "+----------------------------------+\r\n",
            can_bus_name(can_get_active_bus()),
            (unsigned long)can_get_bitrate(can_get_active_bus()),
            4, "",
            can_get_listen_only() ? "LISTEN-ONLY" : "NORMAL",
            (unsigned long)can_get_bitrate(CAN_BUS_MS),
            (unsigned long)can_get_bitrate(CAN_BUS_HS),
            (unsigned long)can_get_bitrate(CAN_BUS_MM),
            (unsigned long)cs.rx_frames,
            (unsigned long)cs.tx_frames,
            (unsigned long)cs.rx_errors,
            (unsigned long)cs.bus_off_count,
            (unsigned long)cs.rx_missed);
        (void)pos;
        CMD_SEND(sock, response, strlen(response));
    }

    // ── STATS ─────────────────────────────────────────────────────
    else if (strcasecmp(cmd,"STATS") == 0) {
        can_stats_t cs;
        can_get_stats(&cs);
        snprintf(response, sizeof(response),
                 "RX:%lu TX:%lu Err:%lu BusOff:%lu Missed:%lu\r\n",
                 (unsigned long)cs.rx_frames, (unsigned long)cs.tx_frames,
                 (unsigned long)cs.rx_errors, (unsigned long)cs.bus_off_count,
                 (unsigned long)cs.rx_missed);
        CMD_SEND(sock, response, strlen(response));
    }

    // ── REBOOT ────────────────────────────────────────────────────
    else if (strcasecmp(cmd,"REBOOT") == 0) {
        snprintf(response, sizeof(response), "Rebooting...\r\n");
        CMD_SEND(sock, response, strlen(response));
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    }

    // ── HELP ──────────────────────────────────────────────────────
    else if (strcasecmp(cmd,"HELP") == 0) {
        snprintf(response, sizeof(response),
            "CAN Sniffer Commands:\r\n"
            "  CANBUS <MS|HS|MM|OFF>           - Select/deactivate bus\r\n"
            "  BITRATE <MS|HS|MM> <125|250|500|1000> - Set bitrate\r\n"
            "  MODE <LISTEN|NORMAL>            - Set RX/TX mode\r\n"
            "  SEND <ID> [B0..B7]              - Send CAN frame (NORMAL only)\r\n"
            "  WIFI <SSID> <PASSWORD>          - Set WiFi credentials\r\n"
            "  STATUS                          - Full status overview\r\n"
            "  STATS                           - Frame counters\r\n"
            "  REBOOT                          - Restart device\r\n"
            "  HELP                            - This help\r\n"
            "Output format: (timestamp) can0 ID#DATA\r\n");
        CMD_SEND(sock, response, strlen(response));
    }

    else {
        snprintf(response, sizeof(response),
                 "ERROR: Unknown command '%s' - type HELP\r\n", cmd);
        CMD_SEND(sock, response, strlen(response));
    }
}

// ═══════════════════════════════════════════════════════════════════
// TCP Server + Client Handler
// ═══════════════════════════════════════════════════════════════════


void client_handler_task(void *pvParameters)
{
    int sock = (int)(intptr_t)pvParameters;

    ringbuf_reader_t reader;
    ringbuf_reader_init_from_history(&reader, CONFIG_WS_RECONNECT_HISTORY_LINES);

    struct timeval tv = { .tv_sec = 0, .tv_usec = 50000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    char rx_buf[256];
    char tx_buf[RINGBUF_ENTRY_SIZE];

    while (1) {
        // Ring-Buffer → Telnet
        size_t len = 0;
        while (ringbuf_read(&reader, tx_buf, &len)) {
            if (send(sock, tx_buf, len, 0) < 0) goto disconnect;
        }

        // Telnet RX → parse_command
        int n = recv(sock, rx_buf, sizeof(rx_buf) - 1, 0);
        if (n > 0) {
            if ((unsigned char)rx_buf[0] == 0xFF) continue;  // Telnet IAC ignorieren
            rx_buf[n] = '\0';
            parse_command(rx_buf, sock);
        } else if (n == 0) {
            goto disconnect;
        }
        // n < 0 = SO_RCVTIMEO Timeout → weiter loopen
    }

disconnect:
    close(sock);
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (client_sockets[i] == sock) { client_sockets[i] = -1; break; }
    }
    ESP_LOGI(TAG, "Telnet client disconnected (sock=%d)", sock);
    vTaskDelete(NULL);
}

void tcp_server_task(void *pvParameters)
{
    struct sockaddr_in server_addr = {
        .sin_family      = AF_INET,
        .sin_addr.s_addr = INADDR_ANY,
        .sin_port        = htons(TCP_PORT),
    };

    for (int i = 0; i < MAX_CLIENTS; i++) client_sockets[i] = -1;

    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Socket create failed");
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
    listen(listen_sock, MAX_CLIENTS);

    ESP_LOGI(TAG, "TCP server listening on port %d", TCP_PORT);

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&client_addr, &addr_len);
        if (sock < 0) continue;

        int flag = 1;
        setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

        int slot = -1;
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (client_sockets[i] < 0) { slot = i; break; }
        }

        if (slot >= 0) {
            client_sockets[slot] = sock;
            if (slot >= client_count) client_count = slot + 1;
            ESP_LOGI(TAG, "Client %d connected", slot);

            // Welcome banner
            char welcome[256];
            snprintf(welcome, sizeof(welcome),
                     "# CAN Sniffer - Active: %s (%lu kbps, %s) - Type HELP\r\n",
                     can_bus_name(can_get_active_bus()),
                     (unsigned long)can_get_bitrate(can_get_active_bus()),
                     can_get_listen_only() ? "LISTEN-ONLY" : "NORMAL");
            send(sock, welcome, strlen(welcome), 0);

            xTaskCreate(client_handler_task, "client", 4096,
                        (void *)(intptr_t)sock, 5, NULL);
        } else {
            const char *busy = "# ERROR: Max clients reached\r\n";
            send(sock, busy, strlen(busy), 0);
            close(sock);
        }
    }
}

void statistics_task(void *pvParameters)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));
        can_stats_t cs;
        can_get_stats(&cs);
        ESP_LOGI(TAG, "Stats [%s]: RX=%lu TX=%lu Err=%lu BusOff=%lu Missed=%lu",
                 can_bus_name(can_get_active_bus()),
                 (unsigned long)cs.rx_frames, (unsigned long)cs.tx_frames,
                 (unsigned long)cs.rx_errors, (unsigned long)cs.bus_off_count,
                 (unsigned long)cs.rx_missed);
    }
}

// ═══════════════════════════════════════════════════════════════════
// WiFi
// ═══════════════════════════════════════════════════════════════════

static int wifi_retry_count = 0;
#define WIFI_MAX_RETRY 10

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        wifi_retry_count = 0;
        led_indicator_send(LED_EVENT_WIFI_STA_CONNECTING);
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (wifi_retry_count < WIFI_MAX_RETRY) {
            wifi_retry_count++;
            ESP_LOGW(TAG, "WiFi disconnected, retry %d/%d...", wifi_retry_count, WIFI_MAX_RETRY);
            led_indicator_send(LED_EVENT_WIFI_STA_CONNECTING);
            vTaskDelay(pdMS_TO_TICKS(1000 * wifi_retry_count));
            esp_wifi_connect();
        } else {
            ESP_LOGE(TAG, "WiFi failed after %d retries - restarting", WIFI_MAX_RETRY);
            led_indicator_send(LED_EVENT_WIFI_ERROR);
            vTaskDelay(pdMS_TO_TICKS(3000));
            esp_restart();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        wifi_retry_count = 0;
        led_indicator_send(LED_EVENT_WIFI_STA_CONNECTED);
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "nc " IPSTR " %d", IP2STR(&event->ip_info.ip), TCP_PORT);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        led_indicator_send(LED_EVENT_WIFI_AP_CONNECTED);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        led_indicator_send(LED_EVENT_WIFI_AP_DISCONNECTED);
    }
}

void wifi_start_ap(void)
{
    ESP_LOGW(TAG, "-----------------------------------------");
    ESP_LOGW(TAG, "No WiFi credentials - Starting AP mode");
    ESP_LOGW(TAG, "SSID:     CAN-Sniffer");
    ESP_LOGW(TAG, "Password: can12345");
    ESP_LOGW(TAG, "Then: nc 192.168.4.1 %d", TCP_PORT);
    ESP_LOGW(TAG, "Then: WIFI <SSID> <PASSWORD>  then REBOOT");
    ESP_LOGW(TAG, "-----------------------------------------");

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_event_handler, NULL));

    wifi_config_t ap_config = {
        .ap = {
            .ssid           = "CAN-Sniffer",
            .ssid_len       = strlen("CAN-Sniffer"),
            .password       = "can12345",
            .max_connection = MAX_CLIENTS,
            .authmode       = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    led_indicator_send(LED_EVENT_WIFI_AP_WAITING);
    ESP_LOGI(TAG, "AP started: 192.168.4.1");
}

void wifi_connect_sta(const char *ssid, const char *password)
{
    ESP_LOGI(TAG, "Connecting to WiFi: %s", ssid);

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid,     ssid,     sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void wifi_init(void)
{
    char ssid[32]     = {0};
    char password[64] = {0};
    bool found        = false;

    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS corrupted, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

#if defined(CONFIG_WIFI_SSID) && defined(CONFIG_WIFI_PASSWORD)
    if (strlen(CONFIG_WIFI_SSID) > 0) {
        strncpy(ssid,     CONFIG_WIFI_SSID,     sizeof(ssid) - 1);
        strncpy(password, CONFIG_WIFI_PASSWORD,  sizeof(password) - 1);
        found = true;
        ESP_LOGI(TAG, "Using Kconfig WiFi credentials");
    }
#endif

    if (!found && wifi_get_credentials(ssid, sizeof(ssid), password, sizeof(password))) {
        found = true;
        ESP_LOGI(TAG, "Using NVS WiFi credentials");
    }

    if (!found) {
        wifi_start_ap();
        return;
    }

    wifi_connect_sta(ssid, password);
}

// ═══════════════════════════════════════════════════════════════════
// app_main
// ═══════════════════════════════════════════════════════════════════

void app_main(void)
{
    boot_timestamp_us = esp_timer_get_time();
    ESP_LOGI(TAG, "CAN Sniffer starting...");

    // ── 1. Ring-Buffer + LED ──────────────────────────────────────
    ringbuf_init();
    led_indicator_init();

    // ── 2. WiFi init (NVS init happens inside) ────────────────────
    wifi_init();

    // ── 3. Load CAN configuration from NVS ───────────────────────
    can_config_t can_cfg = {
        .bitrate_ms_kbps = DEFAULT_BITRATE_MS,
        .bitrate_hs_kbps = DEFAULT_BITRATE_HS,
        .bitrate_mm_kbps = DEFAULT_BITRATE_MM,
        .active_bus      = CAN_BUS_NONE,   // always start with bus off
        .listen_only     = true,           // safe default
    };
    can_nvs_load(&can_cfg);
    can_cfg.active_bus = CAN_BUS_NONE;     // force off at boot (always safe)

    // ── 4. Init CAN driver ────────────────────────────────────────
    can_hw_config_t can_hw = {
        .tx_gpio     = CAN_TX_GPIO,
        .rx_gpio     = CAN_RX_GPIO,
        .stb_ms_gpio = CAN_STB_MS_GPIO,
        .stb_hs_gpio = CAN_STB_HS_GPIO,
        .stb_mm_gpio = CAN_STB_MM_GPIO,
    };

    can_set_rx_callback(can_rx_callback, NULL);
    ESP_ERROR_CHECK(can_driver_init(&can_hw, &can_cfg));
    ESP_LOGI(TAG, "CAN driver ready (all buses off, use CANBUS MS/HS/MM)");

    // ── 5. Start servers + tasks ──────────────────────────────────
    web_server_start();
    xTaskCreate(tcp_server_task, "tcp",   4096, NULL, 5, NULL);
    xTaskCreate(statistics_task, "stats", 2048, NULL, 3, NULL);

    ESP_LOGI(TAG, "Running! Connect: nc <IP> %d", TCP_PORT);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
