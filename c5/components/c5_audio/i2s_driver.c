/**
 * @file i2s_driver.c — I2S full-duplex driver for C5
 *
 * Phase 1: TX-only for MAX98357A speaker.
 * Phase 3: Add RX channel for INMP441 microphone.
 *
 * Uses new ESP-IDF I2S driver API (i2s_std.h).
 * Clock: BCLK = sample_rate * bits_per_sample * 2 (stereo slots) = 16k*16*2 = 512kHz
 */

#include "i2s_driver.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "i2s"

static i2s_chan_handle_t s_tx_chan = NULL;
static i2s_chan_handle_t s_rx_chan = NULL;
static TickType_t s_tx_done_at = 0;   /* FreeRTOS tick when current TX finishes */

/* ── public ──────────────────────────────────────── */

void i2s_tx_init(void)
{
    ESP_LOGI(TAG, "init TX-only mode");

    /* ── channel config ── */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &s_tx_chan));

    /* ── standard mode config ── */
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                            I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_PIN,
            .ws   = I2S_WS_PIN,
            .dout = I2S_DOUT_PIN,
            .din  = I2S_GPIO_UNUSED,   /* Phase 3: set to I2S_DIN_PIN */
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_tx_chan, &std_cfg));

    /* ── enable TX ── */
    ESP_ERROR_CHECK(i2s_channel_enable(s_tx_chan));

    ESP_LOGI(TAG, "TX ready  bclk=GPIO%d ws=GPIO%d dout=GPIO%d @%dHz",
             I2S_BCLK_PIN, I2S_WS_PIN, I2S_DOUT_PIN, I2S_SAMPLE_RATE);
}

bool i2s_full_duplex_init(void)
{
    ESP_LOGI(TAG, "init full-duplex mode");

    /* stop TX-only if already running */
    if (s_tx_chan) {
        i2s_channel_disable(s_tx_chan);
        i2s_del_channel(s_tx_chan);
        s_tx_chan = NULL;
    }
    s_tx_done_at = 0;

    /* ── create both channels ── */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);
    esp_err_t err = i2s_new_channel(&chan_cfg, &s_tx_chan, &s_rx_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to create channels: %d", err);
        /* fallback: try TX-only */
        s_rx_chan = NULL;
        i2s_tx_init();
        return false;
    }

    /* ── TX standard mode config (same as TX-only) ── */
    i2s_std_config_t tx_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                            I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_PIN,
            .ws   = I2S_WS_PIN,
            .dout = I2S_DOUT_PIN,
            .din  = I2S_DIN_PIN,           /* RX: INMP441 on GPIO6 */
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    err = i2s_channel_init_std_mode(s_tx_chan, &tx_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TX std init failed: %d", err);
        i2s_del_channel(s_tx_chan);
        i2s_del_channel(s_rx_chan);
        s_tx_chan = NULL;
        s_rx_chan = NULL;
        return false;
    }

    /* ── RX standard mode config ── */
    i2s_std_config_t rx_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                            I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_PIN,
            .ws   = I2S_WS_PIN,
            .dout = I2S_DOUT_PIN,
            .din  = I2S_DIN_PIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    err = i2s_channel_init_std_mode(s_rx_chan, &rx_cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "RX std init failed: %d — mic not connected? TX still works", err);
        i2s_del_channel(s_rx_chan);
        s_rx_chan = NULL;
        /* TX is still functional */
        ESP_ERROR_CHECK(i2s_channel_enable(s_tx_chan));
        ESP_LOGI(TAG, "TX-only fallback ready @%dHz", I2S_SAMPLE_RATE);
        return false;
    }

    /* ── enable both channels ── */
    ESP_ERROR_CHECK(i2s_channel_enable(s_tx_chan));
    ESP_ERROR_CHECK(i2s_channel_enable(s_rx_chan));

    ESP_LOGI(TAG, "full-duplex ready  bclk=GPIO%d ws=GPIO%d "
             "dout=GPIO%d din=GPIO%d @%dHz",
             I2S_BCLK_PIN, I2S_WS_PIN, I2S_DOUT_PIN, I2S_DIN_PIN,
             I2S_SAMPLE_RATE);
    return true;
}

size_t i2s_rx_read(int16_t *buf, size_t max_samples)
{
    if (!s_rx_chan || !buf || max_samples == 0) return 0;

    size_t bytes = max_samples * sizeof(int16_t);
    size_t bytes_read = 0;
    esp_err_t err = i2s_channel_read(s_rx_chan, buf, bytes, &bytes_read, 0);
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
        return 0;
    }
    return bytes_read / sizeof(int16_t);
}

bool i2s_tx_write(const int16_t *data, size_t count)
{
    if (!s_tx_chan) return false;
    size_t bytes = count * sizeof(int16_t);
    esp_err_t err = i2s_channel_write(s_tx_chan, data, bytes, NULL, 0);
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "TX write err: %d", err);
        return false;
    }
    /* Extend (not overwrite) completion timestamp so queued chunks don't
     * report "done" before earlier chunks finish playing. */
    TickType_t now = xTaskGetTickCount();
    if (s_tx_done_at < now) s_tx_done_at = now;
    uint32_t duration_ms = (uint32_t)(count * 1000ULL / I2S_SAMPLE_RATE);
    s_tx_done_at += pdMS_TO_TICKS(duration_ms);
    return true;
}

bool i2s_tx_is_busy(void)
{
    if (!s_tx_chan) return false;
    return (xTaskGetTickCount() < s_tx_done_at);
}

void i2s_tx_stop(void)
{
    /* Stop TX only — RX channel (mic) stays alive */
    if (s_tx_chan) {
        i2s_channel_disable(s_tx_chan);
        i2s_del_channel(s_tx_chan);
        s_tx_chan = NULL;
        s_tx_done_at = 0;
    }
}

void i2s_deinit(void)
{
    i2s_tx_stop();
    if (s_rx_chan) {
        i2s_channel_disable(s_rx_chan);
        i2s_del_channel(s_rx_chan);
        s_rx_chan = NULL;
    }
}
