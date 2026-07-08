/**
 * @file vision_line.c
 * @brief Black-line detection for line-following robot
 *
 * Scans the grayscale image for dark regions (line) on brighter background
 * (floor). Reports line center position and crossroad presence.
 *
 * Assumptions:
 *   - Line is darker than background (black tape on light floor)
 *   - Camera faces forward/downward
 *   - Line is roughly vertical in frame
 */

#include "vision_core.h"
#include <string.h>

/* Threshold: pixels below this value are considered "dark / on line" */
#define LINE_DARK_THRESHOLD   60

/* Crossroads: line spans > CROSSROAD_FRAC of row width */
#define CROSSROAD_FRAC        70    /* percent */

/* How many lower rows to scan (bottom portion of image) */
#define SCAN_ROWS             30

/* Minimum consecutive dark pixels to count as "line found" */
#define MIN_LINE_PIXELS       5

void line_detect(const uint8_t *gray, int w, int h, line_result_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));

    int    row_start = h - SCAN_ROWS;
    if (row_start < 0) row_start = 0;

    int    sum_pos  = 0;
    int    sum_conf = 0;
    int    valid_rows = 0;
    bool   any_crossroad = false;

    for (int y = row_start; y < h; y++) {
        const uint8_t *row = gray + (uint32_t)y * w;

        /* Scan row for dark pixels */
        int  dark_start = -1;
        int  dark_end   = -1;
        int  dark_count = 0;

        for (int x = 0; x < w; x++) {
            if (row[x] < LINE_DARK_THRESHOLD) {
                if (dark_start < 0) dark_start = x;
                dark_end = x;
                dark_count++;
            }
        }

        if (dark_count < MIN_LINE_PIXELS) continue;

        /* Line center for this row (0..w-1) → mapped to -100..+100 */
        int   center = (dark_start + dark_end) / 2;
        int   pos    = (int)((int32_t)(center - w/2) * 200 / w);

        /* Confidence: fraction of line width to expected width */
        int   line_w = dark_end - dark_start + 1;
        int   conf   = (line_w * 100) / (w / 4);  /* expect ~1/4 width */
        if (conf > 100) conf = 100;

        sum_pos  += pos;
        sum_conf += conf;
        valid_rows++;

        /* Crossroad check */
        if (dark_count > (w * CROSSROAD_FRAC / 100)) {
            any_crossroad = true;
        }
    }

    if (valid_rows > 0) {
        out->line_found = true;
        out->position   = (int16_t)(sum_pos / valid_rows);
        out->confidence = (uint8_t)(sum_conf / valid_rows);
        out->crossroads = any_crossroad;
    }
}
