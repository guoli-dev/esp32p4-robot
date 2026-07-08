/**
 * @file c5_ble.c — C5 BLE NUS (Nordic UART Service) 实现
 *
 * NimBLE stack, Nordic UART Service:
 *   Service:  6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 *   RX Char:  6E400002-... (Write, WriteNoResp) — 手机→C5
 *   TX Char:  6E400003-... (Notify)             — C5→手机
 */

#include "c5_ble.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include <string.h>
#include <stdio.h>

#define TAG "c5_ble"

/* ── NUS UUID ─────────────────────────────────────── */

/* 6E400001-B5A3-F393-E0A9-E50E24DCCA9E */
static const ble_uuid128_t g_nus_svc_uuid = BLE_UUID128_INIT(
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E);

/* 6E400002-... (RX: write from phone) */
static const ble_uuid128_t g_nus_rx_uuid = BLE_UUID128_INIT(
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E);

/* 6E400003-... (TX: notify to phone) */
static const ble_uuid128_t g_nus_tx_uuid = BLE_UUID128_INIT(
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E);

/* ── 内部状态 ──────────────────────────────────────── */

static c5_ble_rx_cb_t  s_rx_cb       = NULL;
static uint16_t        s_tx_handle   = 0;
static uint16_t        s_conn_handle = 0;
static bool            s_connected   = false;

/* RX 累积缓冲区 */
static char   s_cmd_buf[256];
static int    s_cmd_len = 0;

/* ── GATT 访问回调 ─────────────────────────────────── */

static int gatt_svr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)arg;

    /* TX characteristic: read → empty */
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR ||
        ctxt->op == BLE_GATT_ACCESS_OP_READ_DSC) {
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    /* RX characteristic: write / write-no-resp */
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        if (!s_rx_cb) return 0;

        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len == 0) return 0;

        /* 溢出保护 */
        if (s_cmd_len + (int)len >= (int)sizeof(s_cmd_buf) - 1) {
            s_cmd_len = 0;
        }

        int r = ble_hs_mbuf_to_flat(ctxt->om, s_cmd_buf + s_cmd_len,
                                     (uint16_t)(sizeof(s_cmd_buf) - 1 - s_cmd_len), &len);
        if (r != 0) { s_cmd_len = 0; return BLE_ATT_ERR_UNLIKELY; }
        s_cmd_len += len;
        s_cmd_buf[s_cmd_len] = '\0';

        /* 处理命令：有换行就逐行，没换行就整条 */
        bool processed = false;
        char *nl;
        while ((nl = strchr(s_cmd_buf, '\n')) != NULL) {
            *nl = '\0';
            char *line = s_cmd_buf;
            char *cr = strchr(line, '\r');
            if (cr) *cr = '\0';

            if (line[0] != '\0') {
                ESP_LOGI(TAG, "RX: '%s'", line);
                s_rx_cb(line);
                processed = true;
            }

            int remaining = s_cmd_len - (int)(nl + 1 - s_cmd_buf);
            if (remaining > 0) memmove(s_cmd_buf, nl + 1, (size_t)remaining);
            s_cmd_len = remaining;
            if (s_cmd_len >= 0 && s_cmd_len < (int)sizeof(s_cmd_buf))
                s_cmd_buf[s_cmd_len] = '\0';
        }

        /* 没换行但有内容 → 整条当命令处理（无需 \n） */
        if (!processed && s_cmd_len > 0 && strchr(s_cmd_buf, '\n') == NULL) {
            ESP_LOGI(TAG, "RX: '%s'", s_cmd_buf);
            s_rx_cb(s_cmd_buf);
            s_cmd_len = 0;
        }

        if (s_cmd_len >= (int)sizeof(s_cmd_buf) - 1) {
            ESP_LOGW(TAG, "cmd overflow, truncating");
            s_cmd_len = 0;
        }

        return 0;
    }

    return 0;
}

/* ── GAP 事件 ──────────────────────────────────────── */

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_connected = true;
            ESP_LOGI(TAG, "CONNECTED (handle=%u)", s_conn_handle);
        } else {
            s_connected = false;
            ESP_LOGW(TAG, "connect failed: %d", (int)event->connect.status);
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        s_connected = false;
        ESP_LOGI(TAG, "DISCONNECTED (reason=%d)", (int)event->disconnect.reason);

        /* Log heap for diagnostics */
        ESP_LOGI(TAG, "free heap: %u", (unsigned)esp_get_free_heap_size());

        /* Re-advertise with same params as init */
        {
            struct ble_gap_adv_params adv_params = {
                .conn_mode = BLE_GAP_CONN_MODE_UND,
                .disc_mode = BLE_GAP_DISC_MODE_GEN,
            };
            int rc2;
            for (int retry = 0; retry < 5; retry++) {
                rc2 = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                                        &adv_params, gap_event_cb, NULL);
                if (rc2 == 0 || rc2 != BLE_HS_EBUSY) break;
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            if (rc2 != 0) ESP_LOGW(TAG, "re-adv failed: %d", rc2);
        }
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "subscribe handle=%u val=%u",
                 event->subscribe.attr_handle,
                 (unsigned)event->subscribe.cur_notify);
        break;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU: %u", event->mtu.value);
        break;

    default:
        break;
    }
    return 0;
}

/* ── NimBLE host task ──────────────────────────────── */

static void nimble_host_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "nimble host task start (Core %d)", xPortGetCoreID());
    nimble_port_run();
}

/* ── Public API ────────────────────────────────────── */

void c5_ble_init(c5_ble_rx_cb_t on_rx)
{
    s_rx_cb = on_rx;

    esp_err_t r = nimble_port_init();
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init: %s", esp_err_to_name(r));
        return;
    }

    /* 设备名 */
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    char name[32];
    snprintf(name, sizeof(name), "EnvCar-%02X%02X", mac[4], mac[5]);
    ble_svc_gap_device_name_set(name);

    ble_svc_gap_init();
    ble_svc_gatt_init();

    /* 注册 NUS 服务 — 全部 static，NimBLE 存指针不拷贝 */
    static ble_uuid_any_t svc_uuid, rx_uuid, tx_uuid;
    svc_uuid.u.type = BLE_UUID_TYPE_128;
    memcpy(svc_uuid.u128.value, g_nus_svc_uuid.value, 16);
    rx_uuid.u.type = BLE_UUID_TYPE_128;
    memcpy(rx_uuid.u128.value, g_nus_rx_uuid.value, 16);
    tx_uuid.u.type = BLE_UUID_TYPE_128;
    memcpy(tx_uuid.u128.value, g_nus_tx_uuid.value, 16);

    static struct ble_gatt_chr_def nus_chrs[] = {
        {
            .uuid = (ble_uuid_t *)&rx_uuid,
            .access_cb = gatt_svr_access_cb,
            .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
        },
        {
            .uuid = (ble_uuid_t *)&tx_uuid,
            .access_cb = gatt_svr_access_cb,
            .flags = BLE_GATT_CHR_F_NOTIFY,
            .val_handle = &s_tx_handle,
        },
        { 0 }
    };

    static struct ble_gatt_svc_def svcs[] = {
        {
            .type = BLE_GATT_SVC_TYPE_PRIMARY,
            .uuid = (ble_uuid_t *)&svc_uuid,
            .characteristics = nus_chrs,
        },
        { 0 }
    };

    int rc = ble_gatts_count_cfg(svcs);
    if (rc != 0) ESP_LOGE(TAG, "ble_gatts_count_cfg: %d", rc);
    rc = ble_gatts_add_svcs(svcs);
    if (rc != 0) ESP_LOGE(TAG, "ble_gatts_add_svcs: %d", rc);

    nimble_port_freertos_init(nimble_host_task);

    /* ── BLE address: public (eFuse MAC is valid) ── */

    /* ── 构建广播数据包 (最大 31 字节) ──
     * 直接把设备名写入广播包，不依赖 Scan Response，
     * 解决部分手机扫描列表不显示名称的问题 */
    uint8_t adv_data[31];
    int pos = 0;

    /* Flags: LE General Discoverable, BR/EDR Not Supported */
    adv_data[pos++] = 0x02;   /* length */
    adv_data[pos++] = 0x01;   /* AD Type: Flags */
    adv_data[pos++] = 0x06;   /* LE General Disc | BR/EDR not supported */

    /* Complete Local Name */
    int name_len = (int)strlen(name);
    int max_name = sizeof(adv_data) - pos - 2;
    if (name_len > max_name) name_len = max_name;
    adv_data[pos++] = (uint8_t)(name_len + 1);
    adv_data[pos++] = 0x09;   /* AD Type: Complete Local Name */
    memcpy(adv_data + pos, name, name_len);
    pos += name_len;

    ble_gap_adv_set_data(adv_data, pos);

    struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
    };

    /* Retry on EBUSY (21) — NimBLE may still be stopping default
     * advertising from ble_svc_gap_init().  Wait for it to finish. */
    for (int retry = 0; retry < 10; retry++) {
        rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                               &adv_params, gap_event_cb, NULL);
        if (rc == 0) break;
        if (rc == BLE_HS_EBUSY) {
            ESP_LOGW(TAG, "adv busy, retrying (%d/10)...", retry + 1);
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            ESP_LOGE(TAG, "ble_gap_adv_start: %d", rc);
            break;
        }
    }

    ESP_LOGI(TAG, "init OK — advertising as '%s' %s",
             name, (rc == 0) ? "" : "(FAILED!)");
}

void c5_ble_send(const char *text)
{
    if (!s_connected || s_tx_handle == 0 || !text || text[0] == '\0') return;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(text, (uint16_t)strlen(text));
    if (!om) return;

    int rc = ble_gattc_notify_custom(s_conn_handle, s_tx_handle, om);
    if (rc != 0) ESP_LOGW(TAG, "notify err: %d", rc);
}

bool c5_ble_is_connected(void)
{
    return s_connected;
}
