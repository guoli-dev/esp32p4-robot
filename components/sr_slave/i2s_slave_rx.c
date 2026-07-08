/**
 * @file sr_slave.c — P4 I2S Slave RX driver
 *
 * Configures I2S_NUM_1 as a slave receiver, snooping the I2S bus
 * (BCLK/WS driven by C5, SD from INMP441).
 *
 * Uses ESP-IDF standard I2S driver (i2s_std.h).
 * Samples are buffered into a software ring buffer for consumption
 * by the wake-word detection task.
 */

#include "sr_slave.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdatomic.h>

#define TAG "sr_slave"

/* ── Ring buffer ───────────────────────────────────── */

static int16_t s_ring[SR_RING_SAMPLES];
static atomic_size_t s_ring_wr = 0;   /* DMA write position (ISR → task) */
static atomic_size_t s_ring_rd = 0;   /* consumer read position */
static i2s_chan_handle_t s_rx_chan = NULL;
static bool s_active = false;

/* ── I2S RX callback (ISR) ──────────────────────────── */

static IRAM_ATTR bool i2s_rx_cb(i2s_chan_handle_t chan, i2s_event_data_t *evt, void *user)
{
    (void)chan;
    (void)user;

    if (evt && evt->dma_buf) {
        const int16_t *src = (const int16_t *)evt->dma_buf;
        size_t count = evt->size / sizeof(int16_t);

        /* Copy into ring buffer with wrap-around */
        size_t wr = atomic_load_explicit(&s_ring_wr, memory_order_relaxed);
        for (size_t i = 0; i < count; i++) {
            s_ring[wr] = src[i];
            wr = (wr + 1) % SR_RING_SAMPLES;
        }
        /* Release: ring data is visible before wr pointer */
        atomic_store_explicit(&s_ring_wr, wr, memory_order_release);
    }
    return false;  /* don't yield from ISR */
}

/* ── Public API ────────────────────────────────────── */

bool sr_slave_init(void)
{
    ESP_LOGI(TAG, "init I2S slave RX  BCLK=GPIO%d WS=GPIO%d SD=GPIO%d",
             SR_I2S_BCLK_PIN, SR_I2S_WS_PIN, SR_I2S_SD_PIN);

    /* ── Channel config: SLAVE, RX only ── */
    i2s_chan_config_t chan_cfg = {
        .id          = SR_I2S_PORT,
        .role        = I2S_ROLE_SLAVE,
        .dma_desc_num = 4,
        .dma_frame_num = 512,
        .auto_clear   = true,
    };
    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &s_rx_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel failed: 0x%X", err);
        return false;
    }

    /* ── Standard mode: 16kHz 16-bit mono, Philips ── */
    i2s_std_config_t rx_cfg = {
        .clk_cfg = {
            .sample_rate_hz = SR_SAMPLE_RATE,
            .clk_src        = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple  = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                            SR_BITS_PER_SAMPLE,
                            I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = SR_I2S_BCLK_PIN,
            .ws   = SR_I2S_WS_PIN,
            .dout = I2S_GPIO_UNUSED,    /* RX only, no TX */
            .din  = SR_I2S_SD_PIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    err = i2s_channel_init_std_mode(s_rx_chan, &rx_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s std init failed: 0x%X — is C5 outputting BCLK/WS?", err);
        i2s_del_channel(s_rx_chan);
        s_rx_chan = NULL;
        return false;
    }

    /* ── Register RX callback ── */
    i2s_event_callbacks_t cbs = {
        .on_recv      = i2s_rx_cb,
        .on_recv_q_ovf = NULL,
        .on_sent       = NULL,
        .on_send_q_ovf = NULL,
    };
    i2s_channel_register_event_callback(s_rx_chan, &cbs, NULL);

    /* ── Enable ── */
    err = i2s_channel_enable(s_rx_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s enable failed: 0x%X", err);
        i2s_del_channel(s_rx_chan);
        s_rx_chan = NULL;
        return false;
    }

    s_ring_wr = 0;
    s_ring_rd = 0;
    s_active  = true;

    ESP_LOGI(TAG, "I2S slave ready @%dHz — listening", SR_SAMPLE_RATE);
    return true;
}

size_t sr_read_pcm(int16_t *buf, size_t max_samples)
{
    if (!s_active || !buf || max_samples == 0) return 0;

    size_t count = 0;
    /* Acquire: read wr AFTER ring data pointer */
    size_t wr = atomic_load_explicit(&s_ring_wr, memory_order_acquire);
    size_t rd = atomic_load_explicit(&s_ring_rd, memory_order_relaxed);

    while (count < max_samples && rd != wr) {
        buf[count++] = s_ring[rd];
        rd = (rd + 1) % SR_RING_SAMPLES;
    }

    atomic_store_explicit(&s_ring_rd, rd, memory_order_relaxed);
    return count;
}

size_t sr_available(void)
{
    size_t wr = atomic_load_explicit(&s_ring_wr, memory_order_acquire);
    size_t rd = atomic_load_explicit(&s_ring_rd, memory_order_relaxed);
    if (wr >= rd) {
        return wr - rd;
    } else {
        return SR_RING_SAMPLES - rd + wr;
    }
}

void sr_flush(void)
{
    atomic_store_explicit(&s_ring_rd,
        atomic_load_explicit(&s_ring_wr, memory_order_acquire),
        memory_order_relaxed);
}

bool sr_is_active(void)
{
    return s_active;
}
