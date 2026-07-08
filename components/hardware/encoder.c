#include "encoder.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define TAG "encoder"

#define PCNT_LOW_LIMIT   -32768
#define PCNT_HIGH_LIMIT   32767

static const uint8_t s_enc_a[4] = {E1A, E2A, E3A, E4A};
static const uint8_t s_enc_b[4] = {E1B, E2B, E3B, E4B};

static pcnt_unit_handle_t s_pcnt_unit[4];
static int64_t            s_last_time[4];
static int32_t            s_last_count[4];

void encoder_init(void)
{
    ESP_LOGI(TAG, "encoder_init start...");
    for (int i = 0; i < 4; i++) {
        ESP_LOGI(TAG, "  ch%d: A=%d B=%d", i, s_enc_a[i], s_enc_b[i]);
        /* PCNT 单元 */
        pcnt_unit_config_t unit_cfg = {
            .low_limit  = PCNT_LOW_LIMIT,
            .high_limit = PCNT_HIGH_LIMIT,
            .flags = { .accum_count = 1 },
        };
        ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &s_pcnt_unit[i]));

        /* 毛刺滤波器：500 ns */
        pcnt_glitch_filter_config_t filter = {
            .max_glitch_ns = 500,
        };
        pcnt_unit_set_glitch_filter(s_pcnt_unit[i], &filter);

        /* 通道配置：A 相边沿，B 相电平，用于正交解码 */
        pcnt_chan_config_t chan_cfg = {
            .edge_gpio_num  = s_enc_a[i],
            .level_gpio_num = s_enc_b[i],
        };
        pcnt_channel_handle_t chan;
        ESP_ERROR_CHECK(pcnt_new_channel(s_pcnt_unit[i], &chan_cfg, &chan));

        /* 4 倍频正交解码 */
        pcnt_channel_set_edge_action(chan,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE,   /* 下降沿：基准 = 递减 */
            PCNT_CHANNEL_EDGE_ACTION_INCREASE);  /* 上升沿：基准 = 递增 */
        pcnt_channel_set_level_action(chan,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,       /* 低电平：保持基准 */
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE);   /* 高电平：反转基准 */

        /* 使能并启动 */
        ESP_ERROR_CHECK(pcnt_unit_enable(s_pcnt_unit[i]));
        ESP_ERROR_CHECK(pcnt_unit_clear_count(s_pcnt_unit[i]));
        ESP_ERROR_CHECK(pcnt_unit_start(s_pcnt_unit[i]));

        s_last_time[i]  = esp_timer_get_time();
        s_last_count[i] = 0;
    }
    ESP_LOGI(TAG, "4-ch quadrature encoder init done");
}

int32_t encoder_get_count(uint8_t encoder)
{
    if (encoder > 3) return 0;

    int count;
    pcnt_unit_get_count(s_pcnt_unit[encoder], &count);
    return count;
}

void encoder_reset(uint8_t encoder)
{
    if (encoder > 3) return;

    pcnt_unit_clear_count(s_pcnt_unit[encoder]);
    s_last_time[encoder]  = esp_timer_get_time();
    s_last_count[encoder] = 0;
}

float encoder_get_rpm(uint8_t encoder, uint16_t pulses_per_rev)
{
    if (encoder > 3 || pulses_per_rev == 0) return 0.0f;

    int32_t count = encoder_get_count(encoder);
    int64_t now   = esp_timer_get_time();

    int32_t delta_count = count - s_last_count[encoder];

    /* PCNT 16-bit 回绕修正 */
    if (delta_count > 32767)       delta_count -= 65536;
    else if (delta_count < -32768) delta_count += 65536;

    int64_t delta_us    = now - s_last_time[encoder];

    s_last_count[encoder] = count;
    s_last_time[encoder]  = now;

    if (delta_us == 0) return 0.0f;

    /* 4 倍频解码，每个脉冲对应 4 个边沿 */
    float revs    = (float)delta_count / (float)(pulses_per_rev * 4);
    float minutes = (float)delta_us / 60000000.0f;

    return revs / minutes;
}
