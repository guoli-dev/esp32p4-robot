/**
 * @file env_sensor.c — 环境数据统一采集任务
 *
 * 每 2 秒采集 SHT31 + MQ-135 + MQ-136, 通过 Queue 广播。
 */

#include "env_sensor.h"
#include "sht30.h"
#include "mq135.h"
#include "mq136.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <string.h>

#define TAG            "env_sensor"
#define COLLECT_MS     2000   /* 采集间隔 */
#define QUEUE_LEN      1      /* 队列深度 (xQueueOverwrite 要求长度为1) */

/* ── 内部状态 ──────────────────────────────────────── */

static QueueHandle_t s_queue  = NULL;
static env_data_t    s_latest;         /* 最新数据 */
static bool          s_has_data = false;
static portMUX_TYPE  s_spinlock = portMUX_INITIALIZER_UNLOCKED;

static bool s_sht30_ok  = false;
static bool s_mq135_ok  = false;
static bool s_mq136_ok  = false;

/* ── 空气质量综合评分 ─────────────────────────────── */

static uint8_t calc_air_quality(const env_data_t *d)
{
    int score = 100;

    /* MQ-136 H₂S: 最危险，权重最高
     *   hazard>60 = -50, >30 = -30, >10 = -15 */
    if (d->mq136_hazard > 60)      score -= 50;
    else if (d->mq136_hazard > 30) score -= 30;
    else if (d->mq136_hazard > 10) score -= 15;

    /* MQ-135 综合有害气体
     *   hazard>80 = -40, >50 = -25, >20 = -10 */
    if (d->mq135_hazard > 80)      score -= 40;
    else if (d->mq135_hazard > 50) score -= 25;
    else if (d->mq135_hazard > 20) score -= 10;

    if (score < 0)  score = 0;
    return (uint8_t)score;
}

/* ── 采集任务 ──────────────────────────────────────── */

static void env_collect_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "collect task start (Core %d), sensors: "
             "SHT31=%d MQ135=%d MQ136=%d",
             xPortGetCoreID(), (int)s_sht30_ok, (int)s_mq135_ok, (int)s_mq136_ok);

    TickType_t last = xTaskGetTickCount();

    while (1) {
        env_data_t d;
        memset(&d, 0, sizeof(d));

        /* ── SHT31 ── */
        if (s_sht30_ok) {
            float t = 0.0f, h = 0.0f;
            if (sht30_read(&t, &h)) {
                d.temp_c = t;
                d.humidity_pct = h;
            }
        }

        /* ── MQ-135 (综合有害气体) ── */
        if (s_mq135_ok) {
            uint16_t raw = 0;
            uint8_t  hazard = 0;
            mq135_read_raw(&raw);
            mq135_read_hazard(&hazard);
            d.mq135_raw = raw;
            d.mq135_hazard = hazard;
        }

        /* ── MQ-136 (H₂S) ── */
        if (s_mq136_ok) {
            uint16_t raw = 0;
            uint8_t  hazard = 0;
            mq136_read_raw(&raw);
            mq136_read_hazard(&hazard);
            d.mq136_raw = raw;
            d.mq136_hazard = hazard;
        }

        d.air_quality = calc_air_quality(&d);

        /* 更新全局最新值 */
        portENTER_CRITICAL(&s_spinlock);
        s_latest = d;
        s_has_data = true;
        portEXIT_CRITICAL(&s_spinlock);

        /* 广播到 Queue */
        if (s_queue) {
            xQueueOverwrite(s_queue, &d);  /* 覆盖旧数据，保证最新 */
        }

        ESP_LOGI(TAG, "T=%.1fC H=%.1f%% MQ135=%u(HAZ:%u) MQ136=%u(HAZ:%u) AQ=%u",
                 (double)d.temp_c, (double)d.humidity_pct,
                 (unsigned)d.mq135_raw, (unsigned)d.mq135_hazard,
                 (unsigned)d.mq136_raw, (unsigned)d.mq136_hazard,
                 (unsigned)d.air_quality);

        /* 等够 2 秒 */
        vTaskDelayUntil(&last, pdMS_TO_TICKS(COLLECT_MS));
    }
}

/* ── Public API ────────────────────────────────────── */

bool env_sensor_init(void)
{
    bool any_ok = false;

    s_queue = xQueueCreate(QUEUE_LEN, sizeof(env_data_t));
    if (!s_queue) {
        ESP_LOGE(TAG, "queue create failed");
        return false;
    }

    /* ── 创建共享 ADC2 oneshot 单元 (MQ-135 + MQ-136 共用) ── */
    adc_oneshot_unit_handle_t adc2_handle = NULL;
    adc_oneshot_unit_init_cfg_t adc_cfg = {
        .unit_id = ADC_UNIT_2,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    if (adc_oneshot_new_unit(&adc_cfg, &adc2_handle) == ESP_OK) {
        ESP_LOGI(TAG, "ADC2 oneshot unit created");
    } else {
        ESP_LOGW(TAG, "ADC2 oneshot unit failed");
    }

    /* MQ-136 H₂S (ADC2_CH1, GPIO50 J3-14) */
    if (adc2_handle && mq136_init(adc2_handle)) {
        s_mq136_ok = true;
        any_ok = true;
    } else {
        ESP_LOGW(TAG, "MQ-136 not found — H2S data unavailable");
    }

    /* MQ-135 综合有害气体 (ADC2_CH0, GPIO49 J3-16) */
    if (adc2_handle && mq135_init(adc2_handle)) {
        s_mq135_ok = true;
        any_ok = true;
    } else {
        ESP_LOGW(TAG, "MQ-135 not found — gas data unavailable");
    }

    /* SHT31 I2C */
    {
        float t, h;
        if (sht30_read(&t, &h)) {
            s_sht30_ok = true;
            any_ok = true;
            ESP_LOGI(TAG, "SHT31 OK: %.1fC %.1f%%", (double)t, (double)h);
        } else {
            ESP_LOGW(TAG, "SHT31 not found — temp/humidity unavailable");
        }
    }

    if (!any_ok) {
        ESP_LOGE(TAG, "No environmental sensors found!");
        vQueueDelete(s_queue);
        s_queue = NULL;
        return false;
    }

    xTaskCreatePinnedToCore(env_collect_task, "env_collect", 3072,
                            NULL, 2, NULL, 1);  /* Core 1, prio 2 */
    return true;
}

bool env_sensor_get(env_data_t *buf, uint32_t timeout_ms)
{
    if (!s_queue || !buf) return false;
    TickType_t t = (timeout_ms == 0) ? 0 : pdMS_TO_TICKS(timeout_ms);
    return xQueueReceive(s_queue, buf, t) == pdTRUE;
}

const env_data_t *env_sensor_peek(void)
{
    if (!s_has_data) return NULL;
    return &s_latest;
}

air_level_t env_air_level(const env_data_t *d)
{
    if (!d) return AIR_GOOD;
    uint8_t aq = d->air_quality;
    if (aq > 75) return AIR_GOOD;
    if (aq > 50) return AIR_MODERATE;
    if (aq > 25) return AIR_UNHEALTHY;
    return AIR_HAZARDOUS;
}
