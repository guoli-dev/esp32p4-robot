/**
 * @file main.c — ESP32-C5 AI Agent + BLE Nav + Env Monitor
 *
 * 三通道通信:
 *   - BLE NUS: 手机 ←→ C5  (蓝牙串口遥控 + 传感器数据回传)
 *   - UART:    P4  ←→ C5  (运动指令 + 环境数据)
 *   - WiFi:    C5  →  Cloud (LLM API, 语音对话)
 *
 * 完整链路:
 *   手机 → BLE → C5 → UART → P4 → 电机/巡逻
 *   传感器 ← BLE ← C5 ← UART ← P4 (环境数据回传)
 *   环境数据 → TTS 语音播报 (local_tts)
 *
 * Hardware:
 *   BLE:       ESP32-C5 内置
 *   P4 UART1:  GPIO24(TX) / GPIO25(RX)  @ 38400 baud
 *   MAX98357A  I2S TX: BCLK=GPIO4, WS=GPIO5, DOUT=GPIO23
 *   INMP441    I2S RX: BCLK=GPIO4, WS=GPIO5, DIN=GPIO6
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_err.h"

#include "wifi_mgr.h"
#include "cloud_api.h"
#include "i2s_driver.h"
#include "tts_player.h"
#include "audio_capture.h"
#include "uart_bridge.h"
#include "p4_protocol.h"
#include "voice_engine.h"
#include "local_tts.h"
#include "c5_ble.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"

#define TAG "main"

/* ── WiFi credentials (replace with your own) ───────── */
#define WIFI_SSID      "YOUR_WIFI_SSID"
#define WIFI_PASSWORD  "YOUR_WIFI_PASSWORD"

/* ── latest env data from P4 (for BLE SENSOR query) ─ */

static char s_last_env_json[160] = "{\"msg\":\"no data\"}";
static bool s_env_has_data = false;

/* ── C5-side speed tracking ── */
static int s_c5_speed = 50;

/* ── BLE 指令 → P4 UART 中继 ──────────────────────── */
/*
 * 短指令格式（大小写均可）:
 *   s / stop        停车
 *   f [cm]          前进 (无参=30cm,  f50 = 前进50cm)
 *   b [cm]          后退 (无参=30cm)
 *   l [deg]         左转 (无参=90°,   l45 = 左转45°)
 *   r [deg]         右转 (无参=90°)
 *   v <pct>         设速度 (v80 = 80%)
 *   e               环境数据
 *   p <0-3>         巡逻路径
 *   旧格式 ST/FWD:/TURN:/M: 仍然兼容
 */

static int get_speed(void) { return s_c5_speed; }

static void ble_cmd_to_p4(const char *cmd)
{
    if (!cmd || cmd[0] == '\0') return;

    printf("\n[BLE] %s\n", cmd);

    char c = cmd[0];
    const char *arg = cmd + 1;
    while (*arg == ' ') arg++;  /* skip spaces after command letter */

    switch (c) {

    /* ── stop ── */
    case 's': case 'S':
        uart_p4_send_frame(P4_CMD_NAV_STOP, NULL, 0);
        return;

    /* ── env ── */
    case 'e': case 'E':
        c5_ble_send(s_last_env_json);
        printf("[BLE] → %s\n", s_last_env_json);
        return;

    /* ── forward ── */
    case 'f': case 'F':
    case 'w': case 'W': {
        float dist = (arg[0] >= '0' && arg[0] <= '9') ? (float)atof(arg) : 30.0f;
        if (dist > 200.0f) dist = 200.0f;  /* safety cap */
        int spd = get_speed();
        uint8_t p[5]; memcpy(p, &dist, 4); p[4] = (uint8_t)spd;
        uart_p4_send_frame(P4_CMD_NAV_STRAIGHT, p, 5);
        return;
    }

    /* ── backward ── */
    case 'b': case 'B': {
        float dist = (arg[0] >= '0' && arg[0] <= '9') ? (float)atof(arg) : 30.0f;
        if (dist > 200.0f) dist = 200.0f;
        int spd = get_speed();
        uint8_t p[5]; float nd = -dist; memcpy(p, &nd, 4); p[4] = (uint8_t)spd;
        uart_p4_send_frame(P4_CMD_NAV_STRAIGHT, p, 5);
        return;
    }

    /* ── left turn ── */
    case 'l': case 'L': {
        float deg = (arg[0] >= '0' && arg[0] <= '9') ? (float)atof(arg) : 90.0f;
        if (deg > 360.0f) deg = 360.0f;
        int spd = get_speed();
        uint8_t p[5]; memcpy(p, &deg, 4); p[4] = (uint8_t)spd;
        uart_p4_send_frame(P4_CMD_NAV_TURN, p, 5);
        return;
    }

    /* ── right turn ── */
    case 'r': case 'R': {
        float deg = (arg[0] >= '0' && arg[0] <= '9') ? (float)atof(arg) : 90.0f;
        if (deg > 360.0f) deg = 360.0f;
        int spd = get_speed();
        uint8_t p[5]; float nd = -deg; memcpy(p, &nd, 4); p[4] = (uint8_t)spd;
        uart_p4_send_frame(P4_CMD_NAV_TURN, p, 5);
        return;
    }

    /* ── speed ── */
    case 'v': case 'V': {
        int spd = atoi(arg);
        if (spd < 1) spd = 1;
        if (spd > 100) spd = 100;
        s_c5_speed = spd;  /* track on C5 side too */
        uint8_t p[2] = { MOTOR_SET_SPEED, (uint8_t)spd };
        uart_p4_send_frame(P4_CMD_MOTOR, p, 2);
        return;
    }

    /* ── patrol path ── */
    case 'p': case 'P': {
        int idx = atoi(arg);
        if (idx >= 0 && idx < 4) {
            uart_p4_send_frame(P4_CMD_NAV_PATH, (const uint8_t *)&idx, 1);
        }
        return;
    }
    }

    /* ── legacy: English keyword commands ── */
    if (strcmp(cmd, "STOP") == 0 || strcmp(cmd, "stop") == 0) {
        uart_p4_send_frame(P4_CMD_NAV_STOP, NULL, 0); return;
    }
    if (strcmp(cmd, "SENSOR") == 0 || strcmp(cmd, "sensor") == 0) {
        c5_ble_send(s_last_env_json);
        printf("[BLE] → %s\n", s_last_env_json); return;
    }
    if (strncmp(cmd, "FWD:", 4) == 0 || strncmp(cmd, "fwd:", 4) == 0) {
        float dist = 30.0f; int spd = 50;
        sscanf(cmd + 4, "%f,%d", &dist, &spd);
        uint8_t p[5]; memcpy(p, &dist, 4); p[4] = (uint8_t)spd;
        uart_p4_send_frame(P4_CMD_NAV_STRAIGHT, p, 5); return;
    }
    if (strncmp(cmd, "TURN:", 5) == 0 || strncmp(cmd, "turn:", 5) == 0) {
        float deg = 90.0f; int spd = 40;
        sscanf(cmd + 5, "%f,%d", &deg, &spd);
        uint8_t p[5]; memcpy(p, &deg, 4); p[4] = (uint8_t)spd;
        uart_p4_send_frame(P4_CMD_NAV_TURN, p, 5); return;
    }
    if (cmd[0] == 'M' && cmd[1] == ':') {
        char act = cmd[2];
        uint8_t mc = MOTOR_STOP;
        if (act == 'F' || act == 'f') mc = MOTOR_FORWARD;
        else if (act == 'B' || act == 'b') mc = MOTOR_BACKWARD;
        else if (act == 'L' || act == 'l') mc = MOTOR_TURN_LEFT;
        else if (act == 'R' || act == 'r') mc = MOTOR_TURN_RIGHT;
        uint8_t p[1] = { mc };
        uart_p4_send_frame(P4_CMD_MOTOR, p, 1); return;
    }

    printf("[BLE] unknown: '%s'\n", cmd);
}

/* ── TTS PCM callback wrapper (适配 i2s_tx_write 返回类型) ── */

static void tts_pcm_callback(const int16_t *pcm, size_t count)
{
    i2s_tx_write(pcm, count);
}

/* ── P4 数据监听任务 (UART → BLE + TTS) ──────────── */

static void p4_listener_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "P4 listener start");

    uint8_t payload[P4_FRAME_MAX_PAYLOAD];
    uint16_t len;
    uint8_t type;

    while (1) {
        type = uart_p4_recv_frame(payload, &len, sizeof(payload));

        switch (type) {
        case P4_EVT_ENV_DATA:
            if (len >= 13) {
                int16_t t_raw   = (int16_t)(((uint16_t)payload[0] << 8) | payload[1]);
                uint16_t h_raw  = (uint16_t)(((uint16_t)payload[2] << 8) | payload[3]);
                uint16_t mq135_raw = (uint16_t)(((uint16_t)payload[4] << 8) | payload[5]);
                uint8_t  mq135_haz = payload[6];
                uint16_t mq136_raw = (uint16_t)(((uint16_t)payload[7] << 8) | payload[8]);
                uint8_t  mq136_haz = payload[9];
                uint8_t  aq        = payload[12];

                float temp = (float)t_raw / 10.0f;
                float hum  = (float)h_raw / 10.0f;

                /* JSON → BLE 回传手机 */
                snprintf(s_last_env_json, sizeof(s_last_env_json),
                         "{\"t\":%.1f,\"h\":%.1f,"
                         "\"mq135\":%u,\"haz135\":%u,"
                         "\"mq136\":%u,\"haz136\":%u,\"aq\":%u}",
                         (double)temp, (double)hum,
                         (unsigned)mq135_raw, (unsigned)mq135_haz,
                         (unsigned)mq136_raw, (unsigned)mq136_haz, (unsigned)aq);
                s_env_has_data = true;
                uart_env_cache_set(s_last_env_json);
                ESP_LOGI(TAG, "ENV: %s", s_last_env_json);

                if (c5_ble_is_connected()) {
                    c5_ble_send(s_last_env_json);
                }

                /* TTS 语音播报 (每 30s 一次) */
                static TickType_t last_tts = 0;
                TickType_t now = xTaskGetTickCount();
                if ((now - last_tts) > pdMS_TO_TICKS(30000) || last_tts == 0) {
                    last_tts = now;
                    char tts_text[128];
                    const char *aq_str = (aq >= 75) ? "优" : (aq >= 50) ? "良" :
                                         (aq >= 25) ? "差" : "危险";
                    const char *haz135 = (mq135_haz <= 20) ? "正常" : (mq135_haz <= 50) ? "偏高" :
                                         (mq135_haz <= 80) ? "超标" : "严重";
                    const char *haz136 = (mq136_haz <= 10) ? "安全" : (mq136_haz <= 30) ? "注意" :
                                         (mq136_haz <= 60) ? "危险" : "剧毒";
                    snprintf(tts_text, sizeof(tts_text),
                             "温度%.0f度, 湿度%.0f%%, 有害气体%s, 硫化氢%s, 空气质量%s",
                             (double)temp, (double)hum, haz135, haz136, aq_str);

                    while (i2s_tx_is_busy()) vTaskDelay(pdMS_TO_TICKS(50));
                    size_t samples = local_tts_synthesize(tts_text, tts_pcm_callback);
                    ESP_LOGI(TAG, "TTS %u samples: '%s'", (unsigned)samples, tts_text);
                }
            }
            break;

        case P4_EVT_FACE_RESULT:
            /* payload: [count:1B][faceId:2B LE][conf:1B][x:2B LE][y:2B LE][w:2B LE][h:2B LE] × N */
            if (len >= 1) {
                uint8_t count = payload[0];
                if (count > 5) count = 5;

                char face_json[256];
                int off = snprintf(face_json, sizeof(face_json),
                                   "{\"face\":{\"count\":%u,\"faces\":[", (unsigned)count);
                for (int i = 0; i < (int)count; i++) {
                    int base = 1 + i * 11;
                    if (base + 10 >= (int)len) break;
                    uint16_t fid  = (uint16_t)payload[base] | ((uint16_t)payload[base+1] << 8);
                    uint8_t  conf = payload[base+2];
                    uint16_t x    = (uint16_t)payload[base+3] | ((uint16_t)payload[base+4] << 8);
                    uint16_t y    = (uint16_t)payload[base+5] | ((uint16_t)payload[base+6] << 8);
                    uint16_t w    = (uint16_t)payload[base+7] | ((uint16_t)payload[base+8] << 8);
                    uint16_t h    = (uint16_t)payload[base+9] | ((uint16_t)payload[base+10] << 8);

                    if (i > 0 && off < (int)sizeof(face_json)) face_json[off++] = ',';
                    off += snprintf(face_json + off, (size_t)(sizeof(face_json) - off),
                                    "{\"id\":%u,\"conf\":%u,\"x\":%u,\"y\":%u,\"w\":%u,\"h\":%u}",
                                    (unsigned)fid, (unsigned)conf,
                                    (unsigned)x, (unsigned)y, (unsigned)w, (unsigned)h);
                }
                snprintf(face_json + off, sizeof(face_json) - (size_t)off, "]}}");

                ESP_LOGI(TAG, "FACE: %s", face_json);
                if (c5_ble_is_connected()) {
                    c5_ble_send(face_json);
                }

                /* TTS: 已注册人员播报 */
                if (count > 0 && payload[1] != 0) {
                    char tts_buf[64];
                    uint16_t first_id = (uint16_t)payload[1] | ((uint16_t)payload[2] << 8);
                    snprintf(tts_buf, sizeof(tts_buf), "检测到授权人员编号%d", (int)first_id);
                    while (i2s_tx_is_busy()) vTaskDelay(pdMS_TO_TICKS(50));
                    local_tts_synthesize(tts_buf, tts_pcm_callback);
                }
            }
            break;

        case P4_EVT_GESTURE_RESULT:
            /* payload: [type:1B][conf:1B][x:2B LE][y:2B LE][w:2B LE][h:2B LE] = 10 bytes */
            if (len >= 10) {
                uint8_t  gtype = payload[0];
                uint8_t  conf  = payload[1];
                uint16_t x     = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);
                uint16_t y     = (uint16_t)payload[4] | ((uint16_t)payload[5] << 8);
                uint16_t w     = (uint16_t)payload[6] | ((uint16_t)payload[7] << 8);
                uint16_t h     = (uint16_t)payload[8] | ((uint16_t)payload[9] << 8);

                const char *gname[] = {"none","palm","fist","peace","thumb","point"};
                const char *gn = (gtype <= 5) ? gname[gtype] : "?";

                char gest_json[128];
                snprintf(gest_json, sizeof(gest_json),
                         "{\"gesture\":{\"type\":\"%s\",\"conf\":%u,\"x\":%u,\"y\":%u,\"w\":%u,\"h\":%u}}",
                         gn, (unsigned)conf, (unsigned)x, (unsigned)y, (unsigned)w, (unsigned)h);

                ESP_LOGI(TAG, "GESTURE: %s", gest_json);
                if (c5_ble_is_connected()) {
                    c5_ble_send(gest_json);
                }
            }
            break;

        case P4_EVT_OBSTACLE:
            /* payload: [level:1B][reserved:1B][range_mm:2B LE] */
            if (len >= 4) {
                uint8_t  level    = payload[0];
                uint16_t range_mm = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);

                const char *lvl_str[] = {"none","warn","slow","stop"};
                const char *lvl = (level <= 3) ? lvl_str[level] : "?";

                char obst_json[96];
                snprintf(obst_json, sizeof(obst_json),
                         "{\"obstacle\":{\"level\":\"%s\",\"range_mm\":%u}}",
                         lvl, (unsigned)range_mm);

                ESP_LOGW(TAG, "OBSTACLE: %s", obst_json);
                if (c5_ble_is_connected()) {
                    c5_ble_send(obst_json);
                }

                /* TTS 语音告警 (STOP 级别播报) */
                if (level >= 3) {  /* STOP */
                    char tts_buf[64];
                    unsigned cm = (unsigned)(range_mm / 10);
                    snprintf(tts_buf, sizeof(tts_buf),
                             "前方%u厘米有障碍物，巡逻已停止", cm);
                    while (i2s_tx_is_busy()) vTaskDelay(pdMS_TO_TICKS(50));
                    local_tts_synthesize(tts_buf, tts_pcm_callback);
                }
            }
            break;

        case P4_EVT_VISION_RESULT:  /* legacy — forward as opaque blob */
            if (c5_ble_is_connected() && len > 0) {
                char vis_json[80];
                snprintf(vis_json, sizeof(vis_json),
                         "{\"vis\":\"type=%u len=%u\"}", (unsigned)payload[0], (unsigned)len);
                c5_ble_send(vis_json);
            }
            break;

        case P4_EVT_WAKE:
            /* Touchscreen voice button or P4 I2S wake — start recording */
            voice_engine_trigger_wake();
            break;

        case P4_EVT_LOCAL_CMD:
            if (len > 0 && len < 64) {
                char cmd_text[65] = {0};
                memcpy(cmd_text, payload, len);
                voice_engine_feed_text(cmd_text);
            }
            break;

        case 0:
            break;

        default:
            if (type != 0) ESP_LOGD(TAG, "P4 frame: 0x%02X len=%u", type, len);
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ── WiFi 后台连接任务 ─────────────────────────────── */

static void __attribute__((unused)) wifi_bg_task(void *arg)
{
    (void)arg;

    /* Let BLE settle before powering WiFi — avoids combined RF peak */
    vTaskDelay(pdMS_TO_TICKS(3000));

    wifi_mgr_init();

    const char *ssid = WIFI_SSID;
    const char *pass = WIFI_PASSWORD;
    if (strcmp(ssid, "YOUR_WIFI_SSID") == 0) {
        ESP_LOGI(TAG, "WiFi: no credentials configured, skipping");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "WiFi: connecting to '%s'...", ssid);
    esp_err_t r = wifi_mgr_connect(ssid, pass);
    if (r == ESP_OK) {
        ESP_LOGI(TAG, "WiFi: connected!");
        cloud_api_init();
        ESP_LOGI(TAG, "Cloud API ready — AI agent online");
    } else {
        ESP_LOGW(TAG, "WiFi: failed (%s) — staying offline", esp_err_to_name(r));
    }

    vTaskDelete(NULL);
}

/* ── main ────────────────────────────────────────── */

void app_main(void)
{
    printf("\n");
    printf("+--------------------------------------+\n");
    printf("|   C5 AI AGENT + BLE NAV + ENV MON     |\n");
    printf("+--------------------------------------+\n");
    printf("|  BLE:   EnvCar-XXXX (NUS)            |\n");
    printf("|  P4 UART1: GPIO24(TX)/GPIO25(RX)     |\n");
    printf("|  WiFi6 STA  →  Cloud LLM API         |\n");
    printf("|  I2S TX: BCLK=4 WS=5 DOUT=23         |\n");
    printf("+--------------------------------------+\n\n");

    /* ── 1. NVS ── */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS init OK");

    /* ── 2. Hardware ── */

    bool mic_ok = i2s_full_duplex_init();
    ESP_LOGI(TAG, "I2S %s", mic_ok ? "full-duplex OK" : "TX-only (no mic)");

    tts_player_init();
    audio_capture_init(4000);
    if (mic_ok) audio_capture_start();

    uart_p4_init();
    voice_engine_init();

    /* ── 3. BLE (手机蓝牙串口) ── */
    c5_ble_init(ble_cmd_to_p4);
    ESP_LOGI(TAG, "BLE NUS init OK");

    /* ── 4. P4 data listener ── */
    xTaskCreate(p4_listener_task, "p4_listen", 3072, NULL, 3, NULL);

    /* ── 5. Console — immediately available ── */
    uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0);
    esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    ESP_LOGI(TAG, "C5 ready | BLE=EnvCar | !status !key | type to chat");

    /* ── 6. WiFi ── */
    xTaskCreate(wifi_bg_task, "wifi_bg", 4096, NULL, 4, NULL);

    /* ── 7. CLI loop ── */
    char line[256];
    while (1) {
        if (fgets(line, sizeof(line), stdin)) {
            line[strcspn(line, "\r\n")] = '\0';
            if (!line[0]) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }

            if (strncmp(line, "!key ", 5) == 0) {
                char *provider = line + 5;
                char *key = strchr(provider, ' ');
                if (key) {
                    *key = '\0'; key++;
                    while (*key == ' ') key++;
                    cloud_api_set_key(provider, key);
                    printf("OK — key set for '%s'\r\n", provider);
                } else {
                    printf("Usage: !key <llm|openai> <api-key>\r\n");
                }
            } else if (strncmp(line, "!status", 7) == 0) {
                printf("WiFi:  %s\r\n", wifi_mgr_is_connected() ? "connected" : "offline");
                printf("Cloud: %s\r\n", cloud_api_is_ready() ? "ready" : "no key");
                printf("BLE:   %s\r\n", c5_ble_is_connected() ? "connected" : "waiting");
                printf("Mic:   %s\r\n", audio_capture_is_active() ? "active" : "off");
            } else if (strncmp(line, "!help", 5) == 0) {
                printf("Commands:\r\n");
                printf("  !status          — show connection info\r\n");
                printf("  !key <prov> <k>  — set API key\r\n");
                printf("  !help            — this message\r\n");
                printf("  <text>           — chat with AI\r\n");
            } else if (line[0] == '!') {
                printf("Unknown. Try !help\r\n");
            } else {
                voice_engine_feed_text(line);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
