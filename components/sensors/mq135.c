/**
 * @file mq135.c — MQ-135 有害气体传感器驱动 (ADC2 oneshot)
 *
 * ADC2_CH0 (GPIO49, J3-16), 12-bit, 0-3.6V 衰减
 * handle 由外部 (env_sensor) 统一管理, 与 MQ-136 共享 ADC2
 */

#include "mq135.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdatomic.h>

#define TAG           "mq135"
#define SAMPLE_COUNT  10

static adc_oneshot_unit_handle_t s_adc = NULL;
static atomic_uint s_baseline = 1000;

/* ── Public API ────────────────────────────────────── */

bool mq135_init(adc_oneshot_unit_handle_t adc_handle)
{
    if (!adc_handle) {
        ESP_LOGE(TAG, "adc_handle is NULL");
        return false;
    }
    s_adc = adc_handle;

    /* ── 配置通道 ── */
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    esp_err_t r = adc_oneshot_config_channel(s_adc, MQ135_ADC_CHANNEL, &chan_cfg);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "config_channel err: %s", esp_err_to_name(r));
        return false;
    }

    /* 读一次确认在线 */
    int raw = 0;
    r = adc_oneshot_read(s_adc, MQ135_ADC_CHANNEL, &raw);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "ADC read failed: %s", esp_err_to_name(r));
        return false;
    }

    ESP_LOGI(TAG, "init OK (ADC2 CH%d GPIO49 J3-16), raw=%d, baseline=%u",
             MQ135_ADC_CHANNEL, raw, (unsigned)atomic_load(&s_baseline));
    return true;
}

bool mq135_read_raw(uint16_t *raw)
{
    if (!raw || !s_adc) return false;

    uint32_t sum = 0;
    int val = 0;
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        if (adc_oneshot_read(s_adc, MQ135_ADC_CHANNEL, &val) == ESP_OK) {
            sum += val;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    *raw = (uint16_t)(sum / SAMPLE_COUNT);
    return true;
}

bool mq135_read_hazard(uint8_t *hazard)
{
    if (!hazard) return false;

    uint16_t raw = 0;
    if (!mq135_read_raw(&raw)) return false;

    uint16_t baseline = atomic_load(&s_baseline);

    if (raw <= baseline) { *hazard = 0; return true; }

    uint32_t range = 4095 - baseline;
    if (range == 0) range = 1;
    uint32_t val = (uint32_t)(raw - baseline) * 100 / range;
    if (val > 100) val = 100;
    *hazard = (uint8_t)val;
    return true;
}

void mq135_calibrate(void)
{
    if (!s_adc) return;
    ESP_LOGI(TAG, "calibrating in clean air (50 samples)...");

    uint32_t sum = 0;
    int val = 0;
    for (int i = 0; i < 50; i++) {
        if (adc_oneshot_read(s_adc, MQ135_ADC_CHANNEL, &val) == ESP_OK) {
            sum += val;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    uint16_t avg = (uint16_t)(sum / 50);
    atomic_store(&s_baseline, avg);
    ESP_LOGI(TAG, "calibration done — baseline = %u", (unsigned)avg);
}
