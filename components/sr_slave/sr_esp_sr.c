/**
 * @file sr_esp_sr.c — ESP-SR WakeNet9 + MultiNet7 integration
 *
 * Uses Espressif's official speech recognition framework.
 * Architecture:
 *   - AFE (Audio Front-End) pipeline: NS (noise suppression) + VAD + WakeNet9
 *   - After wake: MultiNet7 for command recognition (3-second window)
 *   - Models loaded from SPIFFS "model" partition
 *
 * ESP-SR v2.0+ API (ESP-IDF v5.0+):
 *   srmodel_list_t → afe_config_t → esp_afe_sr_data_t
 *   feed(frames) → fetch() → wakeup_state / command_id
 *
 * Dependencies (from idf_component.yml):
 *   espressif/esp-sr: ^2.0.0
 *
 * Model files (stored in SPIFFS "model" partition):
 *   - WakeNet9:  wn9_nihaoxiaozhi  (~190 KB)
 *   - MultiNet7: mn7_cn            (~2.9 MB, PSRAM required)
 */

#include "sr_esp_sr.h"

/* ── Try to include ESP-SR headers ─────────────────────── */
/*
 * If ESP-SR component is not installed, we build with a stub
 * that always returns false.  This lets the rest of the project
 * compile while the user adds the ESP-SR dependency.
 */
#if __has_include("esp_afe_sr.h")
#define ESP_SR_AVAILABLE  1
#else
#define ESP_SR_AVAILABLE  0
#endif

#if ESP_SR_AVAILABLE

#include "esp_afe_sr.h"
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_srmodel.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

#define TAG "sr_esp"

/* ── Model storage partition label ─────────────────────── */

#define MODEL_PARTITION  "model"

/* ── MultiNet listen window after wake (ms) ────────────── */

#define MN_LISTEN_MS     3000

/* ── State ─────────────────────────────────────────────── */

typedef enum {
    SR_IDLE,            /* Waiting for wake word */
    SR_WOKE,            /* Wake word detected, listening for command */
    SR_CMD_DETECTED,    /* Command recognized */
    SR_CMD_TIMEOUT,     /* No command within listen window */
} sr_state_t;

static sr_state_t          s_sr_state = SR_IDLE;
static esp_afe_sr_data_t  *s_afe_data = NULL;
static srmodel_list_t     *s_models   = NULL;
static bool                s_active   = false;

/* MultiNet handle — initialized after wake word */
static const esp_mn_iface_t *s_mn_iface = NULL;
static esp_mn_model_t       *s_mn_model = NULL;
static int                   s_mn_listen_ms = 0;

/* ── Command ID → keyword_t mapping ────────────────────── */

static const keyword_t s_cmd_map[MN_CMD_COUNT + 1] = {
    [0]                      = KW_NONE,
    [MN_CMD_ID_FORWARD]      = KW_FORWARD,
    [MN_CMD_ID_BACKWARD]     = KW_BACKWARD,
    [MN_CMD_ID_LEFT]         = KW_LEFT,
    [MN_CMD_ID_RIGHT]        = KW_RIGHT,
    [MN_CMD_ID_STOP]         = KW_STOP,
    [MN_CMD_ID_FASTER]       = KW_FASTER,
    [MN_CMD_ID_SLOWER]       = KW_SLOWER,
    [MN_CMD_ID_SPIN]         = KW_SPIN,
    [MN_CMD_ID_PATROL]       = KW_PATROL,
    [MN_CMD_ID_FWD_ALIAS]    = KW_FORWARD,
    [MN_CMD_ID_TURN_AROUND]  = KW_TURN_AROUND,
    [MN_CMD_ID_FULL_SPEED]   = KW_FULL_SPEED,
    [MN_CMD_ID_HALF_SPEED]   = KW_HALF_SPEED,
    [MN_CMD_ID_LOW_SPEED]    = KW_LOW_SPEED,
    [MN_CMD_ID_TEMPERATURE]  = KW_TEMPERATURE,
    [MN_CMD_ID_HUMIDITY]     = KW_HUMIDITY,
    [MN_CMD_ID_AIR_QUALITY]  = KW_AIR_QUALITY,
    [MN_CMD_ID_ENV_REPORT]   = KW_ENV_REPORT,
    [MN_CMD_ID_CAM_ON]       = KW_CAM_ON,
    [MN_CMD_ID_CAM_OFF]      = KW_CAM_OFF,
    [MN_CMD_ID_COLOR_TRACK]  = KW_COLOR_TRACK,
    [MN_CMD_ID_LINE_FOLLOW]  = KW_LINE_FOLLOW,
    [MN_CMD_ID_GESTURE_CTRL] = KW_GESTURE_CTRL,
    [MN_CMD_ID_VISION_OFF]   = KW_VISION_OFF,
    [MN_CMD_ID_PATROL_SQUARE]   = KW_PATROL_SQUARE,
    [MN_CMD_ID_PATROL_ZIGZAG]   = KW_PATROL_ZIGZAG,
    [MN_CMD_ID_PATROL_STRAIGHT] = KW_PATROL_STRAIGHT,
    [MN_CMD_ID_PATROL_SAMPLE]   = KW_PATROL_SAMPLE,
};

/* ── Initialization ─────────────────────────────────────── */

bool sr_esp_init(void)
{
    if (s_active) return true;  /* already initialized */

    ESP_LOGI(TAG, "loading ESP-SR models from partition '%s'...", MODEL_PARTITION);

    /* Step 1: Load model list from SPIFFS */
    s_models = esp_srmodel_init(MODEL_PARTITION);
    if (!s_models) {
        ESP_LOGE(TAG, "esp_srmodel_init failed — is SPIFFS mounted? "
                 "Did you flash model files to '%s' partition?", MODEL_PARTITION);
        return false;
    }
    ESP_LOGI(TAG, "models loaded OK");

    /* Step 2: Create AFE config with WakeNet + MultiNet (MMNN = multi-mic, multi-net) */
    afe_config_t *afe_cfg = afe_config_init("MMNN", s_models,
                                             AFE_TYPE_SR,
                                             AFE_MODE_HIGH_PERF);
    if (!afe_cfg) {
        ESP_LOGE(TAG, "afe_config_init failed");
        esp_srmodel_deinit(s_models);
        s_models = NULL;
        return false;
    }

    /* Step 3: Allocate AFE data (contains internal buffers, model state) */
    const esp_afe_sr_iface_t *afe_iface =
        (const esp_afe_sr_iface_t *)esp_afe_handle_from_config(afe_cfg);
    if (!afe_iface) {
        ESP_LOGE(TAG, "esp_afe_handle_from_config failed");
        afe_config_free(afe_cfg);
        esp_srmodel_deinit(s_models);
        s_models = NULL;
        return false;
    }

    s_afe_data = afe_iface->create_from_config(afe_cfg);
    afe_config_free(afe_cfg);

    if (!s_afe_data) {
        ESP_LOGE(TAG, "create_from_config failed — check PSRAM availability");
        esp_srmodel_deinit(s_models);
        s_models = NULL;
        return false;
    }

    /* Step 4: Get feed chunk size */
    int chunk_size = afe_iface->get_feed_chunksize(s_afe_data);
    int channels   = afe_iface->get_feed_channel_num(s_afe_data);
    ESP_LOGI(TAG, "AFE ready: feed_chunk=%d samples, channels=%d", chunk_size, channels);
    ESP_LOGI(TAG, "WakeNet9 ('你好小智') + MultiNet7 active");

    s_sr_state = SR_IDLE;
    s_active   = true;
    return true;
}

/* ── MultiNet command registration ──────────────────────── */

static bool mn_init_commands(void)
{
    if (s_mn_model) return true;  /* already initialized */

    /* Get MultiNet7 Chinese interface */
    s_mn_iface = (const esp_mn_iface_t *)esp_mn_handle_from_name("mn7_cn");
    if (!s_mn_iface) {
        ESP_LOGW(TAG, "mn7_cn model not found — trying mn6_cn");
        s_mn_iface = (const esp_mn_iface_t *)esp_mn_handle_from_name("mn6_cn");
    }
    if (!s_mn_iface) {
        ESP_LOGW(TAG, "no Chinese MultiNet model available");
        return false;
    }

    /* Create model with 6-second buffer (covers full command utterance) */
    s_mn_model = s_mn_iface->create("mn", 6000);
    if (!s_mn_model) {
        ESP_LOGE(TAG, "MultiNet model creation failed");
        return false;
    }

    /* Clear default commands and register ours */
    esp_mn_commands_clear();

    /* Register commands as pinyin strings */
    /* ── Motion (IDs 1-10) ── */
    esp_mn_commands_add(MN_CMD_ID_FORWARD,   "qian jin");
    esp_mn_commands_add(MN_CMD_ID_BACKWARD,  "hou tui");
    esp_mn_commands_add(MN_CMD_ID_LEFT,      "zuo zhuan");
    esp_mn_commands_add(MN_CMD_ID_RIGHT,     "you zhuan");
    esp_mn_commands_add(MN_CMD_ID_STOP,      "ting zhi");
    esp_mn_commands_add(MN_CMD_ID_FASTER,    "jia su");
    esp_mn_commands_add(MN_CMD_ID_SLOWER,    "jian su");
    esp_mn_commands_add(MN_CMD_ID_SPIN,      "zhuan quan");
    esp_mn_commands_add(MN_CMD_ID_PATROL,    "xun luo");
    esp_mn_commands_add(MN_CMD_ID_FWD_ALIAS, "wang qian zou");
    /* ── Motion extended (IDs 11-14) ── */
    esp_mn_commands_add(MN_CMD_ID_TURN_AROUND, "diao tou");
    esp_mn_commands_add(MN_CMD_ID_FULL_SPEED,  "quan su");
    esp_mn_commands_add(MN_CMD_ID_HALF_SPEED,  "ban su");
    esp_mn_commands_add(MN_CMD_ID_LOW_SPEED,   "di su");
    /* ── Sensor queries (IDs 15-18) ── */
    esp_mn_commands_add(MN_CMD_ID_TEMPERATURE, "wen du");
    esp_mn_commands_add(MN_CMD_ID_HUMIDITY,    "shi du");
    esp_mn_commands_add(MN_CMD_ID_AIR_QUALITY, "kong qi zhi liang");
    esp_mn_commands_add(MN_CMD_ID_ENV_REPORT,  "huan jing bao gao");
    /* ── Vision control (IDs 19-24) ── */
    esp_mn_commands_add(MN_CMD_ID_CAM_ON,       "da kai she xiang tou");
    esp_mn_commands_add(MN_CMD_ID_CAM_OFF,      "guan bi she xiang tou");
    esp_mn_commands_add(MN_CMD_ID_COLOR_TRACK,  "yan se zhui zong");
    esp_mn_commands_add(MN_CMD_ID_LINE_FOLLOW,  "xun ji");
    esp_mn_commands_add(MN_CMD_ID_GESTURE_CTRL, "shou shi kong zhi");
    esp_mn_commands_add(MN_CMD_ID_VISION_OFF,   "guan bi shi jue");
    /* ── Navigation (IDs 25-28) ── */
    esp_mn_commands_add(MN_CMD_ID_PATROL_SQUARE,   "fang xing xun luo");
    esp_mn_commands_add(MN_CMD_ID_PATROL_ZIGZAG,   "zhe zi xun jian");
    esp_mn_commands_add(MN_CMD_ID_PATROL_STRAIGHT, "zhi xian wang fan");
    esp_mn_commands_add(MN_CMD_ID_PATROL_SAMPLE,   "ding dian cai yang");

    esp_mn_commands_update();
    ESP_LOGI(TAG, "MultiNet: 28 commands registered (motion/sensor/vision/nav)");

    s_mn_listen_ms = 0;
    return true;
}

/* ── Feed audio ─────────────────────────────────────────── */

keyword_t sr_esp_feed(const int16_t *samples, size_t count)
{
    if (!s_active || !s_afe_data || count == 0) return KW_NONE;

    /* Get AFE interface for feed/fetch */
    const esp_afe_sr_iface_t *afe_iface =
        (const esp_afe_sr_iface_t *)esp_afe_handle_from_config(NULL);

    /* Feed audio to AFE pipeline (NS → VAD → WakeNet) */
    /* Note: AFE expects its configured chunk size; we feed exactly that */
    /* Our sr_task always passes 512-sample frames (32ms @ 16kHz) */
    int16_t *feed_buf = (int16_t *)samples;  /* AFE doesn't modify input */
    afe_iface->feed(s_afe_data, feed_buf);

    /* Fetch detection results (non-blocking) */
    afe_fetch_result_t *res = afe_iface->fetch(s_afe_data);
    if (!res || res->ret_value == ESP_FAIL) {
        /* No result yet or error */
        return KW_NONE;
    }

    /* ── Wake word detection ── */
    if (res->wakeup_state == WAKENET_DETECTED) {
        ESP_LOGI(TAG, "<<< WAKE WORD '你好小智' DETECTED >>>");
        ESP_LOGI(TAG, "  model_index=%d, word_index=%d",
                 res->wakenet_model_index, res->wake_word_index);

        if (s_sr_state == SR_IDLE) {
            s_sr_state = SR_WOKE;
            /* Initialize MultiNet for command recognition */
            if (mn_init_commands()) {
                s_mn_listen_ms = 0;
            }
            return KW_WAKE;
        }
    }

    /* ── Command recognition (after wake) ── */
    if (s_sr_state == SR_WOKE && s_mn_model && s_mn_iface) {
        /* Feed the same audio to MultiNet */
        s_mn_iface->feed(s_mn_model, (int16_t *)samples);

        /* Check for command detection */
        esp_mn_state_t mn_state = s_mn_iface->detect(s_mn_model, NULL);
        if (mn_state == ESP_MN_STATE_DETECTED) {
            esp_mn_results_t *mn_result = s_mn_iface->get_results(s_mn_model);
            if (mn_result && mn_result->command_id > 0
                && mn_result->command_id <= MN_CMD_COUNT) {
                keyword_t kw = s_cmd_map[mn_result->command_id];
                ESP_LOGI(TAG, "command: %s (id=%d)",
                         sr_esp_keyword_name(kw), mn_result->command_id);
                s_sr_state = SR_IDLE;
                s_mn_listen_ms = 0;
                return kw;
            }
        }

        /* Timeout check — reset after listen window */
        s_mn_listen_ms += 32;  /* each frame is ~32ms */
        if (s_mn_listen_ms >= MN_LISTEN_MS) {
            ESP_LOGD(TAG, "command listen window expired");
            s_sr_state = SR_IDLE;
            s_mn_listen_ms = 0;
            /* Reset MultiNet for next wake cycle */
            s_mn_iface->clean(s_mn_model);
        }
    }

    return KW_NONE;
}

/* ── Reset ──────────────────────────────────────────────── */

void sr_esp_reset(void)
{
    s_sr_state = SR_IDLE;
    s_mn_listen_ms = 0;

    if (s_mn_iface && s_mn_model) {
        s_mn_iface->clean(s_mn_model);
    }
}

/* ── Keyword name ──────────────────────────────────────── */

const char *sr_esp_keyword_name(keyword_t kw)
{
    switch (kw) {
    case KW_WAKE:            return "你好小智";
    case KW_FORWARD:         return "前进";
    case KW_BACKWARD:        return "后退";
    case KW_LEFT:            return "左转";
    case KW_RIGHT:           return "右转";
    case KW_STOP:            return "停止";
    case KW_FASTER:          return "加速";
    case KW_SLOWER:          return "减速";
    case KW_SPIN:            return "转圈";
    case KW_PATROL:          return "巡逻";
    case KW_TURN_AROUND:     return "掉头";
    case KW_FULL_SPEED:      return "全速";
    case KW_HALF_SPEED:      return "半速";
    case KW_LOW_SPEED:       return "低速";
    case KW_TEMPERATURE:     return "温度";
    case KW_HUMIDITY:        return "湿度";
    case KW_AIR_QUALITY:     return "空气质量";
    case KW_ENV_REPORT:      return "环境报告";
    case KW_CAM_ON:          return "打开摄像头";
    case KW_CAM_OFF:         return "关闭摄像头";
    case KW_COLOR_TRACK:     return "颜色追踪";
    case KW_LINE_FOLLOW:     return "循迹";
    case KW_GESTURE_CTRL:    return "手势控制";
    case KW_VISION_OFF:      return "关闭视觉";
    case KW_PATROL_SQUARE:   return "方形巡逻";
    case KW_PATROL_ZIGZAG:   return "Z字巡检";
    case KW_PATROL_STRAIGHT: return "直线往返";
    case KW_PATROL_SAMPLE:   return "定点采样";
    default:                 return "?";
    }
}

/* ── Query ─────────────────────────────────────────────── */

bool sr_esp_is_active(void)
{
    return s_active;
}

/* ── Deinit ────────────────────────────────────────────── */

void sr_esp_deinit(void)
{
    s_active = false;

    if (s_mn_iface && s_mn_model) {
        s_mn_iface->destroy(s_mn_model);
        s_mn_model = NULL;
    }
    s_mn_iface = NULL;

    if (s_afe_data) {
        const esp_afe_sr_iface_t *afe_iface =
            (const esp_afe_sr_iface_t *)esp_afe_handle_from_config(NULL);
        if (afe_iface) {
            afe_iface->destroy(s_afe_data);
        }
        s_afe_data = NULL;
    }

    if (s_models) {
        esp_srmodel_deinit(s_models);
        s_models = NULL;
    }

    ESP_LOGI(TAG, "deinit OK");
}

#else  /* !ESP_SR_AVAILABLE — stub implementation */

#include "esp_log.h"
#define TAG "sr_esp"

bool sr_esp_init(void)
{
    ESP_LOGW(TAG, "ESP-SR component not installed — "
             "add 'espressif/esp-sr' to idf_component.yml and rebuild");
    return false;
}

keyword_t sr_esp_feed(const int16_t *samples, size_t count)
{
    (void)samples; (void)count;
    return KW_NONE;
}

void sr_esp_reset(void) {}

const char *sr_esp_keyword_name(keyword_t kw)
{
    switch (kw) {
    case KW_WAKE:            return "你好小智";
    case KW_FORWARD:         return "前进";
    case KW_BACKWARD:        return "后退";
    case KW_LEFT:            return "左转";
    case KW_RIGHT:           return "右转";
    case KW_STOP:            return "停止";
    case KW_FASTER:          return "加速";
    case KW_SLOWER:          return "减速";
    case KW_SPIN:            return "转圈";
    case KW_PATROL:          return "巡逻";
    case KW_TURN_AROUND:     return "掉头";
    case KW_FULL_SPEED:      return "全速";
    case KW_HALF_SPEED:      return "半速";
    case KW_LOW_SPEED:       return "低速";
    case KW_TEMPERATURE:     return "温度";
    case KW_HUMIDITY:        return "湿度";
    case KW_AIR_QUALITY:     return "空气质量";
    case KW_ENV_REPORT:      return "环境报告";
    case KW_CAM_ON:          return "打开摄像头";
    case KW_CAM_OFF:         return "关闭摄像头";
    case KW_COLOR_TRACK:     return "颜色追踪";
    case KW_LINE_FOLLOW:     return "循迹";
    case KW_GESTURE_CTRL:    return "手势控制";
    case KW_VISION_OFF:      return "关闭视觉";
    case KW_PATROL_SQUARE:   return "方形巡逻";
    case KW_PATROL_ZIGZAG:   return "Z字巡检";
    case KW_PATROL_STRAIGHT: return "直线往返";
    case KW_PATROL_SAMPLE:   return "定点采样";
    default:                 return "?";
    }
}

bool sr_esp_is_active(void) { return false; }
void sr_esp_deinit(void) {}

#endif /* ESP_SR_AVAILABLE */
