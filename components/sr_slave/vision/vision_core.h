/**
 * @file vision_core.h
 * @brief Visual perception framework for ESP32-P4 robot
 *
 * Provides color tracking, line following, and gesture recognition
 * from OV5647 RAW8 Bayer frames.
 *
 * Architecture:
 *   CSI ISR → frame_queue → vision_task → mode dispatch → C5 UART
 *
 * Processing happens at reduced resolution (160x120 default) to fit in
 * SRAM alongside the 500KB CSI double-buffer (SPIRAM recommended).
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/* ══════════════════════════════════════════════════════
 * Processing resolution (configurable for memory tuning)
 * ══════════════════════════════════════════════════════ */
#define VIS_PROC_W          80      /* Processing width  (10x Bayer skip)     */
#define VIS_PROC_H          60      /* Processing height (10x Bayer skip)    */
#define VIS_PROC_BUF_SIZE   ((VIS_PROC_W) * (VIS_PROC_H))  /* RGB565 pixels  */
#define VIS_PROC_GRAY_SIZE  ((VIS_PROC_W) * (VIS_PROC_H))  /* grayscale bytes*/

/* ══════════════════════════════════════════════════════
 * Visual modes
 * ══════════════════════════════════════════════════════ */
typedef enum {
    VIS_MODE_OFF       = 0,
    VIS_MODE_COLOR     = 1,   /* Color blob tracking    */
    VIS_MODE_LINE      = 2,   /* Black line following    */
    VIS_MODE_GESTURE   = 3,   /* Hand gesture recognition */
} vis_mode_t;

/** Target colors for color tracking */
typedef enum {
    VIS_COLOR_RED    = 0,
    VIS_COLOR_GREEN  = 1,
    VIS_COLOR_BLUE   = 2,
    VIS_COLOR_YELLOW = 3,
} vis_color_t;

/* ══════════════════════════════════════════════════════
 * Mode-specific result structs
 * ══════════════════════════════════════════════════════ */

/** Color blob detection result */
typedef struct {
    bool     detected;
    uint16_t cx;            /* Centroid X (0..VIS_PROC_W-1)   */
    uint16_t cy;            /* Centroid Y (0..VIS_PROC_H-1)   */
    uint16_t area;          /* Detected pixel count           */
    uint8_t  color;         /* vis_color_t of matched target  */
} color_target_t;

/** Line following result */
typedef struct {
    bool     line_found;
    int16_t  position;      /* -100=far-left 0=center +100=far-right */
    uint8_t  confidence;    /* 0..100  */
    bool     crossroads;    /* Intersection detected */
} line_result_t;

/** Hand gesture types */
typedef enum {
    GEST_NONE     = 0,
    GEST_PALM     = 1,      /* Open hand (4+ extended fingers) */
    GEST_FIST     = 2,      /* Closed fist (0 extended)       */
    GEST_PEACE    = 3,      /* Two fingers spread             */
    GEST_THUMB_UP = 4,      /* Thumb extended upward          */
    GEST_POINT    = 5,      /* Single finger pointing         */
} gesture_t;

typedef struct {
    gesture_t gesture;
    uint8_t   confidence;   /* 0..100 */
} gesture_result_t;

/* ══════════════════════════════════════════════════════
 * Unified vision result (sent over UART to C5)
 * ══════════════════════════════════════════════════════ */

#define VIS_RESULT_COLOR    0x01
#define VIS_RESULT_LINE     0x02
#define VIS_RESULT_GESTURE  0x03

typedef struct {
    uint8_t   type;           /* One of VIS_RESULT_* above */
    uint32_t  timestamp_ms;
    union {
        color_target_t   color;
        line_result_t    line;
        gesture_result_t gesture;
    };
} vision_result_t;

/* ══════════════════════════════════════════════════════
 * Frame queue (ISR → processing task)
 * ══════════════════════════════════════════════════════ */

/** Create a frame-pointer queue for ISR-to-task handoff  */
QueueHandle_t vision_frame_queue_create(void);

/* ══════════════════════════════════════════════════════
 * Public API
 * ══════════════════════════════════════════════════════ */

/**
 * @brief Initialize vision subsystem (allocates processing buffers)
 * @return true on success
 */
bool vision_init(void);

/**
 * @brief Set active vision mode and optional target color
 * @param mode  Processing mode
 * @param color Target color (only relevant for VIS_MODE_COLOR)
 */
void vision_set_mode(vis_mode_t mode, vis_color_t color);

/**
 * @brief Get current mode
 */
vis_mode_t vision_get_mode(void);

/**
 * @brief Process a RAW8 Bayer frame (called from vision task, NOT ISR)
 *
 * Downsamples & demosaics to RGB565, then dispatches to active mode.
 *
 * @param raw8     Full-resolution RAW8 Bayer buffer (800×640)
 * @param h        Frame height (640)
 * @param w        Frame width  (800)
 */
void vision_process_frame(const uint8_t *raw8, uint32_t h, uint32_t w);

/**
 * @brief Copy latest vision result into caller-provided buffer
 *
 * Thread-safe — uses a critical section to snapshot the result.
 * @param out  Caller-allocated buffer to receive the copy
 * @return     Pointer to @p out (null if no result available yet)
 */
const vision_result_t *vision_get_result(vision_result_t *out);
