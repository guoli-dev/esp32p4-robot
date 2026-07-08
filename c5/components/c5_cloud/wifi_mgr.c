/**
 * @file wifi_mgr.c — WiFi STA manager for ESP32-C5
 *
 * Handles WiFi station mode connection with:
 *   - NVS credential storage for auto-reconnect
 *   - WiFi 6 (802.11ax) protocol support
 *   - Graceful fallback when connection fails
 */

#include "wifi_mgr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>

#define TAG "wifi_mgr"

#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
#define WIFI_CONNECT_TIMEOUT_MS  10000
#define WIFI_RETRY_MAX      3

static EventGroupHandle_t s_wifi_event_group;
static wifi_mgr_state_t   s_state = WIFI_MGR_DISCONNECTED;
static esp_netif_t       *s_netif = NULL;

/* ── NVS helpers ──────────────────────────────────── */

#define NVS_NAMESPACE  "wifi_creds"
#define NVS_KEY_SSID   "ssid"
#define NVS_KEY_PASS   "pass"

static esp_err_t nvs_load_creds(char *ssid, size_t ssid_len,
                                char *pass, size_t pass_len)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) return err;

    size_t len = ssid_len;
    err = nvs_get_str(handle, NVS_KEY_SSID, ssid, &len);
    if (err == ESP_OK) {
        len = pass_len;
        err = nvs_get_str(handle, NVS_KEY_PASS, pass, &len);
    }
    nvs_close(handle);
    return err;
}

void wifi_mgr_save_credentials(const char *ssid, const char *password)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) return;

    nvs_set_str(handle, NVS_KEY_SSID, ssid);
    if (password && password[0]) {
        nvs_set_str(handle, NVS_KEY_PASS, password);
    }
    nvs_commit(handle);
    nvs_close(handle);
    ESP_LOGI(TAG, "credentials saved for '%s'", ssid);
}

/* ── event handler ────────────────────────────────── */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case WIFI_EVENT_STA_DISCONNECTED: {
            wifi_event_sta_disconnected_t *d =
                (wifi_event_sta_disconnected_t *)event_data;
            ESP_LOGW(TAG, "disconnected, reason=%d", d->reason);
            if (s_state == WIFI_MGR_CONNECTING) {
                esp_wifi_connect();  /* retry */
            } else {
                s_state = WIFI_MGR_DISCONNECTED;
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            }
            break;
        }
        default:
            break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ip = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got IP: " IPSTR, IP2STR(&ip->ip_info.ip));
        s_state = WIFI_MGR_CONNECTED;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* ── public ──────────────────────────────────────── */

void wifi_mgr_init(void)
{
    ESP_LOGI(TAG, "init WiFi STA");

    s_wifi_event_group = xEventGroupCreate();

    /* netif + event loop */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* wifi init with minimal buffers (C5 only ~205KB RAM, BLE+WiFi tight) */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.static_rx_buf_num = 2;     /* default 10 → 2 */
    cfg.dynamic_rx_buf_num = 8;    /* default 32 → 8 */
    cfg.dynamic_tx_buf_num = 8;    /* default 32 → 8 */
    cfg.mgmt_sbuf_num = 8;         /* minimum valid, default 32 → 8 */
    cfg.cache_tx_buf_num = 8;      /* default 32 → 8 */

    esp_err_t ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi init failed (%s) — running offline (BLE only)", esp_err_to_name(ret));
        s_state = WIFI_MGR_DISCONNECTED;
        return;
    }

    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi set_mode failed — running offline");
        s_state = WIFI_MGR_DISCONNECTED;
        esp_wifi_deinit();
        return;
    }

    /* create default STA netif AFTER wifi_init */
    s_netif = esp_netif_create_default_wifi_sta();

    /* event handlers */
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &wifi_event_handler, NULL));

    /* WiFi protocol: 11ax on (C5 ECO2 supports WiFi 6) */
    wifi_protocols_t proto = {
        .ghz_2g = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX,
        .ghz_5g = WIFI_PROTOCOL_11A | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX,
    };
    esp_wifi_set_protocols(WIFI_IF_STA, &proto);

    /* no power saving during dev */
    esp_wifi_set_ps(WIFI_PS_NONE);

    /* Low TX power: 8 dBm (reduces peak current ~100mA, prevents brownout) */
    esp_wifi_set_max_tx_power(8);

    ESP_LOGI(TAG, "init OK");
}

esp_err_t wifi_mgr_connect(const char *ssid, const char *password)
{
    s_state = WIFI_MGR_CONNECTING;

    /* load credentials */
    char use_ssid[33] = {0};
    char use_pass[65] = {0};

    if (ssid && ssid[0]) {
        strncpy(use_ssid, ssid, sizeof(use_ssid) - 1);
        if (password) strncpy(use_pass, password, sizeof(use_pass) - 1);
        wifi_mgr_save_credentials(ssid, password);
        ESP_LOGI(TAG, "using SSID='%s' PASS='%s'", use_ssid, use_pass);
    } else {
        if (nvs_load_creds(use_ssid, sizeof(use_ssid),
                           use_pass, sizeof(use_pass)) != ESP_OK) {
            ESP_LOGW(TAG, "no saved credentials — staying offline");
            s_state = WIFI_MGR_DISCONNECTED;
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "loaded saved creds for '%s'", use_ssid);
    }

    /* configure and connect */
    wifi_config_t wifi_cfg = {0};
    strncpy((char *)wifi_cfg.sta.ssid, use_ssid, sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *)wifi_cfg.sta.password, use_pass,
            sizeof(wifi_cfg.sta.password) - 1);
    /* Let driver auto-detect auth and PMF */
    wifi_cfg.sta.pmf_cfg.capable = true;
    wifi_cfg.sta.pmf_cfg.required = false;
    wifi_cfg.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wifi_cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;

    /* Set CN country code for correct channel allocation */
    esp_wifi_set_country_code("CN", true);
    esp_err_t ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "wifi set_config failed (%s) — staying offline", esp_err_to_name(ret));
        s_state = WIFI_MGR_DISCONNECTED;
        return ret;
    }
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "wifi start failed (%s) — staying offline", esp_err_to_name(ret));
        s_state = WIFI_MGR_DISCONNECTED;
        return ret;
    }

    ESP_LOGI(TAG, "connecting to '%s'...", use_ssid);

    /* wait for connection, with retries */
    for (int retry = 0; retry < WIFI_RETRY_MAX; retry++) {
        EventBits_t bits = xEventGroupWaitBits(
            s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE, pdFALSE,
            pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

        if (bits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "connected!");
            return ESP_OK;
        }

        ESP_LOGW(TAG, "attempt %d/%d failed", retry + 1, WIFI_RETRY_MAX);
        if (retry < WIFI_RETRY_MAX - 1) {
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }

    ESP_LOGW(TAG, "all connect attempts failed — offline mode");
    s_state = WIFI_MGR_ERROR;
    return ESP_ERR_TIMEOUT;
}

wifi_mgr_state_t wifi_mgr_get_state(void)
{
    return s_state;
}

bool wifi_mgr_is_connected(void)
{
    return (s_state == WIFI_MGR_CONNECTED);
}
