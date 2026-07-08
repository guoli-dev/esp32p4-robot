/**
 * @file audio_capture.c — Microphone capture with ring buffer
 *
 * Wraps I2S RX DMA reads with a software ring buffer.
 * When no microphone is connected (s_rx_chan == NULL), all functions
 * gracefully return 0/empty.
 */

#include "audio_capture.h"
#include "i2s_driver.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>

#define TAG "capture"

static int16_t *s_ring_buf = NULL;
static size_t   s_ring_cap = 0;      /* total capacity (samples) */
static volatile size_t s_write_idx = 0;  /* ISR-safe: next write position */
static size_t   s_read_idx  = 0;
static bool     s_active = false;

/* ── public ──────────────────────────────────────── */

void audio_capture_init(size_t ring_buffer_samples)
{
    if (s_ring_buf) {
        free(s_ring_buf);
        s_ring_buf = NULL;
    }

    if (ring_buffer_samples == 0) {
        ring_buffer_samples = 8000;  /* default: 500ms @ 16kHz */
    }

    s_ring_buf = malloc(ring_buffer_samples * sizeof(int16_t));
    if (!s_ring_buf) {
        ESP_LOGE(TAG, "failed to alloc ring buffer (%u samples)",
                 (unsigned)ring_buffer_samples);
        s_ring_cap = 0;
        return;
    }

    s_ring_cap = ring_buffer_samples;
    s_write_idx = 0;
    s_read_idx = 0;
    s_active = false;

    ESP_LOGI(TAG, "init OK  ring=%u samples (%.0fms)",
             (unsigned)s_ring_cap,
             (double)s_ring_cap / I2S_SAMPLE_RATE * 1000.0);
}

void audio_capture_start(void)
{
    if (!s_ring_buf) return;
    audio_capture_flush();
    s_active = true;
    ESP_LOGI(TAG, "capture started");
}

void audio_capture_stop(void)
{
    s_active = false;
    ESP_LOGI(TAG, "capture stopped");
}

size_t audio_capture_read(int16_t *buf, size_t max_samples)
{
    if (!s_ring_buf || !buf || max_samples == 0) return 0;

    /* drain I2S RX into ring buffer first */
    while (s_active) {
        size_t free_slots;
        if (s_write_idx >= s_read_idx) {
            free_slots = s_ring_cap - (s_write_idx - s_read_idx) - 1;
        } else {
            free_slots = s_read_idx - s_write_idx - 1;
        }

        if (free_slots < 64) break;  /* ring buffer nearly full */

        /* read a small chunk from I2S — clamp to buffer boundary */
        size_t chunk = free_slots;
        if (chunk > 256) chunk = 256;
        size_t space_to_end = s_ring_cap - s_write_idx;
        if (chunk > space_to_end) chunk = space_to_end;
        size_t n = i2s_rx_read(&s_ring_buf[s_write_idx], chunk);
        if (n == 0) break;

        s_write_idx = (s_write_idx + n) % s_ring_cap;
    }

    /* now read from ring buffer into caller's buffer */
    size_t avail = audio_capture_available();
    if (avail == 0) return 0;
    if (max_samples > avail) max_samples = avail;

    size_t count = 0;
    while (count < max_samples) {
        buf[count++] = s_ring_buf[s_read_idx];
        s_read_idx = (s_read_idx + 1) % s_ring_cap;
    }
    return count;
}

void audio_capture_flush(void)
{
    /* drain any I2S data into ring buffer then reset pointers */
    if (s_ring_buf) {
        int16_t dummy[128];
        while (i2s_rx_read(dummy, 128) > 0) { /* drain */ }
    }
    s_write_idx = 0;
    s_read_idx = 0;
}

size_t audio_capture_available(void)
{
    if (!s_ring_buf) return 0;
    if (s_write_idx >= s_read_idx) {
        return s_write_idx - s_read_idx;
    }
    return s_ring_cap - s_read_idx + s_write_idx;
}

bool audio_capture_is_active(void)
{
    return s_active && (s_ring_buf != NULL);
}
