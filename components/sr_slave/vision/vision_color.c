/**
 * @file vision_color.c
 * @brief Color blob tracking via HSV threshold + centroid computation
 *
 * Scans every pixel at processing resolution (160×120 = 19,200 pixels),
 * classifies by HSV range, then computes centroid of matching pixels.
 *
 * Performance: ~5 ms on ESP32-P4 @ 400 MHz for 160×120 frame.
 */

#include "vision_core.h"
#include <string.h>

/* ══════════════════════════════════════════════════════
 * HSV ranges for tracked colors (H:0..179, S/V:0..255)
 * ══════════════════════════════════════════════════════ */

typedef struct { uint8_t h_lo, h_hi, s_lo, s_hi, v_lo, v_hi; } hsv_range_t;

static const hsv_range_t s_ranges[] = {
    /* VIS_COLOR_RED    — handles hue wrap via dual check */
    [0] = {   0,  10,  60, 255,  50, 255 },
    /* VIS_COLOR_GREEN  */
    [1] = {  35,  85,  60, 255,  40, 255 },
    /* VIS_COLOR_BLUE   */
    [2] = {  90, 130,  70, 255,  40, 255 },
    /* VIS_COLOR_YELLOW */
    [3] = {  20,  35,  80, 255,  80, 255 },
};

/* ── Fast integer-only RGB565 → HSV ─────────────────── */

static void rgb565_to_hsv(uint16_t rgb, uint8_t *h, uint8_t *s, uint8_t *v)
{
    uint8_t r = (rgb >> 11) << 3;         /* R: 5→8 bits */
    uint8_t g = ((rgb >> 5) & 0x3F) << 2; /* G: 6→8 bits */
    uint8_t b = (rgb & 0x1F) << 3;        /* B: 5→8 bits */

    uint8_t mx = (r >= g) ? r : g;
    if (b > mx) mx = b;
    uint8_t mn = (r <= g) ? r : g;
    if (b < mn) mn = b;

    *v = mx;
    int delta = (int)mx - (int)mn;

    if (delta == 0) {
        *h = 0; *s = 0; return;
    }

    *s = (uint8_t)((255 * delta) / mx);

    int h360;
    if (mx == r) {
        h360 = (60 * ((int)g - (int)b)) / delta;
        if (h360 < 0) h360 += 360;
    } else if (mx == g) {
        h360 = (60 * ((int)b - (int)r)) / delta + 120;
    } else {
        h360 = (60 * ((int)r - (int)g)) / delta + 240;
    }
    *h = (uint8_t)(h360 / 2);
}

/* ── Color match ────────────────────────────────────── */

static inline bool color_hsv_match(uint8_t h, uint8_t s, uint8_t v,
                                    vis_color_t target)
{
    const hsv_range_t *r = &s_ranges[target];

    /* Saturation & value check (same for all colors) */
    if (s < r->s_lo || s > r->s_hi) return false;
    if (v < r->v_lo || v > r->v_hi) return false;

    /* Hue check */
    if (target == VIS_COLOR_RED) {
        return (h <= r->h_hi) || (h >= 170);  /* wraps at 0/180 */
    }
    return (h >= r->h_lo && h <= r->h_hi);
}

/* ── Public entry ───────────────────────────────────── */

void color_detect(const uint16_t *rgb565, int w, int h,
                   vis_color_t target, color_target_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
    out->color = (uint8_t)target;

    uint32_t sum_x = 0, sum_y = 0, count = 0;

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            uint8_t hs, ss, vs;
            rgb565_to_hsv(rgb565[(uint32_t)y * w + x], &hs, &ss, &vs);
            if (color_hsv_match(hs, ss, vs, target)) {
                sum_x += (uint32_t)x;
                sum_y += (uint32_t)y;
                count++;
            }
        }
    }

    if (count < 50) {   /* noise filter: need 50+ pixels */
        out->detected = false;
        return;
    }

    out->detected = true;
    out->cx       = (uint16_t)(sum_x / count);
    out->cy       = (uint16_t)(sum_y / count);
    out->area     = (count > 65535) ? 65535 : (uint16_t)count;
}
