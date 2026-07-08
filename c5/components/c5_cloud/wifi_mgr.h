#pragma once

#include <stdbool.h>
#include "esp_err.h"

/**
 * @file wifi_mgr.h
 *
 * WiFi STA manager — handles connection, auto-reconnect, and NVS credential storage.
 * Falls back gracefully to offline mode when WiFi is unavailable.
 */

typedef enum {
    WIFI_MGR_DISCONNECTED,
    WIFI_MGR_CONNECTING,
    WIFI_MGR_CONNECTED,
    WIFI_MGR_ERROR,
} wifi_mgr_state_t;

/**
 * @brief One-time init: NVS, netif, event loop, wifi init. Call once at boot.
 */
void wifi_mgr_init(void);

/**
 * @brief Connect to WiFi. If ssid==NULL, loads saved credentials from NVS.
 *        Blocks until connected or timeout (~30s).
 *
 * @return ESP_OK on success, ESP_ERR_TIMEOUT or ESP_FAIL on error.
 */
esp_err_t wifi_mgr_connect(const char *ssid, const char *password);

/**
 * @brief Non-blocking state check.
 */
wifi_mgr_state_t wifi_mgr_get_state(void);

/**
 * @brief Quick check: are we connected?
 */
bool wifi_mgr_is_connected(void);

/**
 * @brief Save credentials to NVS for next boot auto-connect.
 */
void wifi_mgr_save_credentials(const char *ssid, const char *password);
