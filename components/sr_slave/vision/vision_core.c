/**
 * @file vision_core.c
 * @brief Image preprocessing: RAW8 Bayer demosaic, RGB↔HSV, grayscale, dispatch
 *
 * Pipeline per frame:
 *   1. Bayer 2×2 blocks → RGB565 at VIS_PROC_W×VIS_PROC_H
 *   2. Dispatch to active mode handler (color / line / gesture / QR)
 *   3. Store result → ready for UART TX to C5
 */

#include "vision_core.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>

#define TAG "vision_core"

/* ── Processing buffers ─────────────────────────────── */

static uint16_t *s_rgb565 = NULL;    /* RGB565 processing frame          */
static uint8_t  *s_gray   = NULL;    /* Grayscale (for line modes)       */
static vision_result_t s_result;     /* Latest processed result           */
static portMUX_TYPE     s_result_lock = portMUX_INITIALIZER_UNLOCKED;
static vis_mode_t s_mode  = VIS_MODE_OFF;
static vis_color_t s_target_color = VIS_COLOR_RED;

/* ── Forward declarations for mode handlers ─────────── */

extern void color_detect(const uint16_t *rgb565, int w, int h,
                          vis_color_t target, color_target_t *out);
extern void line_detect(const uint8_t *gray, int w, int h,
                         line_result_t *out);
extern void gesture_detect(const uint16_t *rgb565, int w, int h,
                            gesture_result_t *out);

/* ── Pixel helpers ──────────────────────────────────── */

/**
 * @brief Convert RGB565 pixel to 8-bit grayscale (ITU-R BT.601 luma)
 */
static inline uint8_t rgb565_to_luma(uint16_t rgb)
{
    /* R:5 G:6 B:5 → Y = 0.299R + 0.587G + 0.114B */
    uint8_t r = (rgb >> 11) & 0x1F;   /* 0-31 */
    uint8_t g = (rgb >> 5)  & 0x3F;   /* 0-63 */
    uint8_t b = (rgb)        & 0x1F;   /* 0-31 */
    /* Scale to 8-bit and apply weights */
    uint32_t y = (r * 77) + (g * 150) + (b * 29);  /* *256 ≈ 0.299*256 etc */
    return (uint8_t)(y >> 8);  /* divide by 256 */
}

/**
 * @brief RAW8 Bayer RG/GB → RGB565, downsampling 4:1
 *
 * For each 2×2 Bayer cell:
 *   [R, G1]
 *   [G2, B]
 * → single RGB565 pixel: R=avg(R), G=avg(G1,G2), B=avg(B)
 *
 * Input:  raw8[w*h]  (e.g. 800×640)
 * Output: rgb565[(w/2)*(h/2)], written to s_rgb565
 *
 * The input is scaled down to VIS_PROC_W×VIS_PROC_H using a step factor.
 */
static void bayer_downsample_rgb565(const uint8_t *raw8,
                                     uint32_t src_w, uint32_t src_h)
{
    /* How many source Bayer cells per output pixel? */
    uint32_t cell_step_x = src_w / (VIS_PROC_W * 2);   /* cells per output column */
    uint32_t cell_step_y = src_h / (VIS_PROC_H * 2);   /* cells per output row   */

    if (cell_step_x < 1) cell_step_x = 1;
    if (cell_step_y < 1) cell_step_y = 1;

    uint16_t *dst = s_rgb565;

    for (uint32_t oy = 0; oy < VIS_PROC_H; oy++) {
        for (uint32_t ox = 0; ox < VIS_PROC_W; ox++) {

            /* Source Bayer origin for this output pixel */
            uint32_t sy = oy * cell_step_y * 2;
            uint32_t sx = ox * cell_step_x * 2;

            /* Accumulate R, G, B over the source region */
            uint32_t acc_r = 0, acc_g = 0, acc_b = 0;
            uint32_t cnt_r = 0, cnt_g = 0, cnt_b = 0;

            for (uint32_t dy = 0; dy < cell_step_y; dy++) {
                for (uint32_t dx = 0; dx < cell_step_x; dx++) {
                    uint32_t row   = sy + dy * 2;
                    uint32_t row2  = row + 1;
                    uint32_t col   = sx + dx * 2;
                    uint32_t col1  = col + 1;

                    if (row  < src_h && col  < src_w) {
                        /* R at (even, even) */
                        acc_r += raw8[row * src_w + col];
                        cnt_r++;
                    }
                    if (row  < src_h && col1 < src_w) {
                        /* G1 at (even, odd) */
                        acc_g += raw8[row * src_w + col1];
                        cnt_g++;
                    }
                    if (row2 < src_h && col  < src_w) {
                        /* G2 at (odd, even) */
                        acc_g += raw8[row2 * src_w + col];
                        cnt_g++;
                    }
                    if (row2 < src_h && col1 < src_w) {
                        /* B at (odd, odd) */
                        acc_b += raw8[row2 * src_w + col1];
                        cnt_b++;
                    }
                }
            }

            uint8_t r = (cnt_r > 0) ? (uint8_t)(acc_r / cnt_r) : 0;
            uint8_t g = (cnt_g > 0) ? (uint8_t)(acc_g / cnt_g) : 0;
            uint8_t b = (cnt_b > 0) ? (uint8_t)(acc_b / cnt_b) : 0;

            /* Pack to RGB565: RRRRR GGGGGG BBBBB */
            *dst++ = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
        }
    }
}

/**
 * @brief Generate grayscale from RGB565 buffer
 */
static void rgb565_to_gray(const uint16_t *rgb, uint8_t *gray, int count)
{
    for (int i = 0; i < count; i++) {
        gray[i] = rgb565_to_luma(rgb[i]);
    }
}

/* ── Frame queue ────────────────────────────────────── */

QueueHandle_t vision_frame_queue_create(void)
{
    /* Queue holds pointers to RAW8 buffers (posted from ISR) */
    return xQueueCreate(4, sizeof(const uint8_t *));
}

/* ── Public API ─────────────────────────────────────── */

bool vision_init(void)
{
    ESP_LOGI(TAG, "vision init start (buffer=%dx%d RGB565, %u bytes)",
             VIS_PROC_W, VIS_PROC_H, (unsigned)(VIS_PROC_BUF_SIZE * 2));

    /* Allocate processing buffers */
    s_rgb565 = (uint16_t *)malloc(VIS_PROC_BUF_SIZE * sizeof(uint16_t));
    s_gray   = (uint8_t  *)malloc(VIS_PROC_GRAY_SIZE);

    if (!s_rgb565 || !s_gray) {
        ESP_LOGE(TAG, "Failed to allocate processing buffers");
        free(s_rgb565); free(s_gray);
        s_rgb565 = NULL; s_gray = NULL;
        return false;
    }

    memset(&s_result, 0, sizeof(s_result));
    s_mode = VIS_MODE_OFF;

    ESP_LOGI(TAG, "vision init OK");
    return true;
}

void vision_set_mode(vis_mode_t mode, vis_color_t color)
{
    s_mode = mode;
    s_target_color = color;
    ESP_LOGI(TAG, "mode=%d color=%d", mode, color);
}

vis_mode_t vision_get_mode(void)
{
    return s_mode;
}

void vision_process_frame(const uint8_t *raw8, uint32_t h, uint32_t w)
{
    if (s_mode == VIS_MODE_OFF) return;
    if (!raw8 || !s_rgb565 || !s_gray) return;

    uint32_t t0 = esp_timer_get_time();

    /* Step 1: Bayer → RGB565 (downsampled) */
    bayer_downsample_rgb565(raw8, w, h);

    /* Take lock, write result, release */
    portENTER_CRITICAL(&s_result_lock);
    memset(&s_result, 0, sizeof(s_result));
    s_result.type      = (s_mode == VIS_MODE_COLOR)   ? VIS_RESULT_COLOR
                       : (s_mode == VIS_MODE_LINE)    ? VIS_RESULT_LINE
                       : (s_mode == VIS_MODE_GESTURE) ? VIS_RESULT_GESTURE
                       : 0;
    s_result.timestamp_ms = (uint32_t)(t0 / 1000);

    /* Step 2: Dispatch to active mode */
    switch (s_mode) {
    case VIS_MODE_COLOR:
        color_detect(s_rgb565, VIS_PROC_W, VIS_PROC_H,
                     s_target_color, &s_result.color);
        break;

    case VIS_MODE_LINE:
        rgb565_to_gray(s_rgb565, s_gray, VIS_PROC_BUF_SIZE);
        line_detect(s_gray, VIS_PROC_W, VIS_PROC_H, &s_result.line);
        break;

    case VIS_MODE_GESTURE:
        gesture_detect(s_rgb565, VIS_PROC_W, VIS_PROC_H,
                       &s_result.gesture);
        break;

    default:
        break;
    }
    portEXIT_CRITICAL(&s_result_lock);

    uint32_t t1 = esp_timer_get_time();
    /* Uncomment for perf profiling:
    ESP_LOGD(TAG, "frame proc: %lu us", (unsigned long)(t1 - t0)); */
    (void)t1;
}

const vision_result_t *vision_get_result(vision_result_t *out)
{
    if (!out) return NULL;
    portENTER_CRITICAL(&s_result_lock);
    memcpy(out, &s_result, sizeof(s_result));
    portEXIT_CRITICAL(&s_result_lock);
    return out;
}
