/**
 * @file vision_gesture.c
 * @brief Hand gesture recognition via skin-color segmentation + contour analysis
 *
 * Pipeline:
 *   1. HSV skin-color segmentation → binary mask (heap-allocated)
 *   2. Find largest skin blob via flood-fill
 *   3. Compute convex hull (monotone chain)
 *   4. Count finger tips → classify gesture
 *
 * Memory-safe: all large buffers allocated on heap, freed after processing.
 * Works at 80×60 resolution (~4800 pixels, ~5KB per buffer).
 */

#include "vision_core.h"
#include <string.h>
#include <stdlib.h>

/* ── Skin color range (HSV: H 0..179, S/V 0..255) ──── */

#define SKIN_H_LO     0
#define SKIN_H_HI     30     /* 扩展以覆盖更多肤色 */
#define SKIN_S_LO     15     /* 降低阈值，弱光下也能检到 */
#define SKIN_S_HI     200    /* 扩展高饱和度范围 */
#define SKIN_V_LO     40     /* 降低明度阈值 */
#define SKIN_V_HI     255

/* ── Blob size limits (absolute, for 80×60 = 4800 px) ─ */

#define MIN_BLOB_AREA     50    /* smaller than this = noise   */
#define MAX_BLOB_AREA     4000  /* larger than this = not hand */

/* ── Contour limits ─────────────────────────────────── */

#define MAX_CONTOUR_PTS   512
#define MAX_HULL_PTS      64

/* ── Pixel helpers ─────────────────────────────────── */

static void rgb565_to_hsv_local(uint16_t rgb, uint8_t *h, uint8_t *s, uint8_t *v)
{
    uint8_t r = (rgb >> 11) << 3;
    uint8_t g = ((rgb >> 5) & 0x3F) << 2;
    uint8_t b = (rgb & 0x1F) << 3;

    uint8_t mx = (r >= g) ? r : g;
    if (b > mx) mx = b;
    uint8_t mn = (r <= g) ? r : g;
    if (b < mn) mn = b;
    *v = mx;

    int16_t delta = (int16_t)mx - (int16_t)mn;
    if (delta == 0) { *h = 0; *s = 0; return; }
    *s = (uint8_t)((255 * delta) / mx);

    int16_t h360;
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

static inline bool is_skin(uint16_t rgb)
{
    uint8_t h, s, v;
    rgb565_to_hsv_local(rgb, &h, &s, &v);
    return (h >= SKIN_H_LO && h <= SKIN_H_HI) &&
           (s >= SKIN_S_LO && s <= SKIN_S_HI) &&
           (v >= SKIN_V_LO && v <= SKIN_V_HI);
}

/* ── Build skin mask (heap allocated) ──────────────── */

static uint8_t *build_skin_mask(const uint16_t *rgb565, int w, int h)
{
    int total = w * h;
    uint8_t *mask = (uint8_t *)malloc((size_t)total);
    if (!mask) return NULL;
    for (int i = 0; i < total; i++) {
        mask[i] = is_skin(rgb565[i]) ? 1 : 0;
    }
    return mask;
}

/* ── Flood-fill ────────────────────────────────────── */

typedef struct { int16_t x, y; } point_t;

/**
 * @brief Scanline flood-fill with explicit heap stack.
 * @return number of points in blob, or -1 on error.
 */
static int flood_fill(const uint8_t *mask, int w, int h,
                       int sx, int sy, uint8_t *visited,
                       point_t *pts, int max_pts)
{
    /* Stack: pair of (x, y) as int32_t packed */
    int stack_cap = w * h;
    int32_t *stack = (int32_t *)malloc((size_t)stack_cap * sizeof(int32_t));
    if (!stack) return -1;

    int sp = 0, count = 0;
    stack[sp++] = ((int32_t)sx << 16) | (int32_t)sy;
    visited[sy * w + sx] = 1;

    while (sp > 0 && count < max_pts) {
        int32_t v  = stack[--sp];
        int cx = (int)(v >> 16);
        int cy = (int)(v & 0xFFFF);

        if (cx >= 0 && cx < w && cy >= 0 && cy < h) {
            pts[count].x = (int16_t)cx;
            pts[count].y = (int16_t)cy;
            count++;
        }

        /* 4-neighbor expansion */
        int neighbors[4][2] = {{-1,0},{1,0},{0,-1},{0,1}};
        for (int d = 0; d < 4 && sp < stack_cap; d++) {
            int nx = cx + neighbors[d][0];
            int ny = cy + neighbors[d][1];
            if (nx >= 0 && nx < w && ny >= 0 && ny < h &&
                mask[ny * w + nx] && !visited[ny * w + nx]) {
                visited[ny * w + nx] = 1;
                stack[sp++] = ((int32_t)nx << 16) | (int32_t)ny;
            }
        }
    }

    free(stack);
    return count;
}

/* ── Convex hull (Andrew's monotone chain) ──────────── */

static int sort_points_by_x(point_t *pts, int n)
{
    for (int i = 1; i < n; i++) {
        point_t key = pts[i];
        int j = i - 1;
        while (j >= 0 && (pts[j].x > key.x ||
                         (pts[j].x == key.x && pts[j].y > key.y))) {
            pts[j+1] = pts[j];
            j--;
        }
        pts[j+1] = key;
    }
    return n;
}

static int cross(const point_t *o, const point_t *a, const point_t *b)
{
    return (a->x - o->x) * (b->y - o->y) - (a->y - o->y) * (b->x - o->x);
}

static int convex_hull(point_t *pts, int n, point_t *hull)
{
    if (n < 3) { for (int i = 0; i < n; i++) hull[i] = pts[i]; return n; }
    sort_points_by_x(pts, n);

    int h = 0;
    for (int i = 0; i < n; i++) {
        while (h >= 2 && cross(&hull[h-2], &hull[h-1], &pts[i]) <= 0) h--;
        hull[h++] = pts[i];
    }
    int h_upper = h + 1;
    for (int i = n - 2; i >= 0; i--) {
        while (h >= h_upper && cross(&hull[h-2], &hull[h-1], &pts[i]) <= 0) h--;
        hull[h++] = pts[i];
    }
    return h - 1;
}

static int dist2(const point_t *a, const point_t *b)
{
    int dx = a->x - b->x, dy = a->y - b->y;
    return dx * dx + dy * dy;
}

/**
 * @brief Count finger tips from convex hull
 *
 * Finger tips = hull vertices significantly farther from centroid
 * than the average hull vertex (≥130% of avg distance), with
 * angular separation requirement to avoid counting neighbors.
 */
static int count_fingers(const point_t *hull, int n_hull)
{
    if (n_hull < 3) return 0;

    int cx = 0, cy = 0;
    for (int i = 0; i < n_hull; i++) { cx += hull[i].x; cy += hull[i].y; }
    cx /= n_hull; cy /= n_hull;

    int avg_d2 = 0;
    point_t c = { (int16_t)cx, (int16_t)cy };
    for (int i = 0; i < n_hull; i++) avg_d2 += dist2(&hull[i], &c);
    avg_d2 /= n_hull;

    int tips = 0, prev_idx = -999;
    for (int i = 0; i < n_hull; i++) {
        int d2 = dist2(&hull[i], &c);
        if (d2 > avg_d2 * 13 / 10) {
            if (i - prev_idx > 3 || prev_idx < 0) {
                tips++;
                prev_idx = i;
            }
        }
    }
    return tips;
}

/* ── Public entry ───────────────────────────────────── */

void gesture_detect(const uint16_t *rgb565, int w, int h,
                     gesture_result_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));

    int total = w * h;

    /* Step 1: Build skin-color mask (heap) */
    uint8_t *mask = build_skin_mask(rgb565, w, h);
    if (!mask) return;

    /* Step 2: Find a seed pixel in center region */
    int seed_x = -1, seed_y = -1;
    int y0 = h / 4, y1 = h;
    int x0 = w / 4, x1 = w * 3 / 4;
    for (int y = y0; y < y1 && seed_x < 0; y++) {
        for (int x = x0; x < x1; x++) {
            if (mask[y * w + x]) { seed_x = x; seed_y = y; break; }
        }
    }
    if (seed_x < 0) { free(mask); return; }

    /* Step 3: Flood-fill blob */
    uint8_t *visited = (uint8_t *)calloc((size_t)total, 1);
    if (!visited) { free(mask); return; }

    point_t *contour_pts = (point_t *)malloc(MAX_CONTOUR_PTS * sizeof(point_t));
    if (!contour_pts) { free(mask); free(visited); return; }

    int blob_size = flood_fill(mask, w, h, seed_x, seed_y, visited,
                                contour_pts, MAX_CONTOUR_PTS);
    free(mask);
    free(visited);

    if (blob_size < MIN_BLOB_AREA || blob_size > MAX_BLOB_AREA) {
        free(contour_pts);
        return;
    }

    /* Step 4: Convex hull */
    point_t hull[MAX_HULL_PTS];
    int n_hull = convex_hull(contour_pts, blob_size, hull);
    free(contour_pts);

    if (n_hull < 3) return;

    /* Step 5: Count fingers → classify */
    int fingers = count_fingers(hull, n_hull);

    if      (fingers >= 4) out->gesture = GEST_PALM;
    else if (fingers == 0) out->gesture = GEST_FIST;
    else if (fingers == 2) out->gesture = GEST_PEACE;
    else if (fingers == 1) out->gesture = GEST_POINT;
    else                    out->gesture = GEST_THUMB_UP;

    int expected = total / 20;
    int diff = blob_size - expected;
    if (diff < 0) diff = -diff;
    int conf = 100 - (diff * 100 / expected);
    if (conf < 0) conf = 0;
    if (conf > 100) conf = 100;
    out->confidence = (uint8_t)conf;
}
