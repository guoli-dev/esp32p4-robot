/**
 * @file vision_qr.c
 * @brief QR code scanning via quirc library
 *
 * Thin wrapper around quirc (MIT-licensed embedded QR decoder).
 * quirc_data is ~9KB so we heap-allocate it to avoid stack overflow.
 */

#include "vision_core.h"
#include "quirc.h"
#include "esp_log.h"
#include <string.h>

#define TAG "vision_qr"

static struct quirc *s_qr_ctx = NULL;

bool vision_qr_begin(int w, int h)
{
    s_qr_ctx = quirc_new();
    if (!s_qr_ctx) {
        ESP_LOGE(TAG, "quirc_new failed");
        return false;
    }

    if (quirc_resize(s_qr_ctx, w, h) < 0) {
        ESP_LOGE(TAG, "quirc_resize(%d,%d) failed", w, h);
        quirc_destroy(s_qr_ctx);
        s_qr_ctx = NULL;
        return false;
    }

    ESP_LOGI(TAG, "QR scanner ready (max %dx%d)", w, h);
    return true;
}

void vision_qr_scan(const uint8_t *gray, int w, int h, qr_result_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));

    if (!s_qr_ctx) return;

    /* Copy grayscale into quirc's internal buffer */
    uint8_t *buf = quirc_begin(s_qr_ctx, NULL, NULL);
    if (!buf) return;

    for (int y = 0; y < h; y++) {
        memcpy(buf + (size_t)y * (size_t)w, gray + (size_t)y * (size_t)w, (size_t)w);
    }
    quirc_end(s_qr_ctx);

    int count = quirc_count(s_qr_ctx);
    if (count <= 0) return;

    /* Heap-allocate quirc_code and quirc_data (~4KB + ~9KB = ~13KB).
     * Stack is only ~3KB at this point in the vision task. */
    struct quirc_code *code = (struct quirc_code *)malloc(sizeof(struct quirc_code));
    struct quirc_data *data = (struct quirc_data *)malloc(sizeof(struct quirc_data));
    if (!code || !data) {
        free(code); free(data);
        return;
    }

    quirc_extract(s_qr_ctx, 0, code);
    quirc_decode_error_t err = quirc_decode(code, data);

    if (err == QUIRC_SUCCESS) {
        out->decoded = true;
        out->version = (uint8_t)data->version;
        size_t copy = (data->payload_len < (int)(sizeof(out->data) - 1))
                      ? (size_t)data->payload_len : (sizeof(out->data) - 1);
        memcpy(out->data, data->payload, copy);
        out->data[copy] = '\0';

        ESP_LOGI(TAG, "QR v%d ecc=%c len=%d \"%s\"",
                 data->version, "LMHQ"[data->ecc_level],
                 data->payload_len, out->data);
    }

    free(code);
    free(data);
}
