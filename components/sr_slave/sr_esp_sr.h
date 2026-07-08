/**
 * @file sr_esp_sr.h — ESP-SR wrapper for P4 voice recognition
 *
 * Replaces wake_detect.c with Espressif's official ESP-SR:
 *   - WakeNet9:  "你好小智" wake word (98% accuracy)
 *   - MultiNet7: Chinese command recognition (200+ commands, ~60ms latency)
 *
 * Fully offline — no WiFi needed for wake word + local commands.
 * Complex AI queries still go through C5 → DeepSeek.
 *
 * Architecture:
 *   I2S slave RX (i2s_slave_rx.c) → ring buffer → sr_task → sr_esp_feed()
 *                                                         ↓
 *   WakeNet9 (continuous) → wake word → UART(C5) + MultiNet7 (3s window)
 *   MultiNet7 → command ID → keyword_t → UART(C5)
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ── Keyword type ────────────────────────────────────── */

typedef enum {
    KW_NONE = 0,
    KW_WAKE,            /* "你好小智" wake word */

    /* ── 基本运动 (P4本地执行) ── */
    KW_FORWARD,         /* "前进" */
    KW_BACKWARD,        /* "后退" */
    KW_LEFT,            /* "左转" */
    KW_RIGHT,           /* "右转" */
    KW_STOP,            /* "停止" */
    KW_FASTER,          /* "加速" (speed +10) */
    KW_SLOWER,          /* "减速" (speed -10) */
    KW_SPIN,            /* "转圈" (360°) */
    KW_PATROL,          /* "巡逻" (默认路径) */
    KW_TURN_AROUND,     /* "掉头" (180°) */

    /* ── 速度预设 (P4本地执行) ── */
    KW_FULL_SPEED,      /* "全速" → 100% */
    KW_HALF_SPEED,      /* "半速" → 50% */
    KW_LOW_SPEED,       /* "低速" → 30% */

    /* ── 传感器查询 (C5 TTS播报) ── */
    KW_TEMPERATURE,     /* "温度" */
    KW_HUMIDITY,        /* "湿度" */
    KW_AIR_QUALITY,     /* "空气质量" */
    KW_ENV_REPORT,      /* "环境报告" (完整播报) */

    /* ── 视觉控制 (P4本地执行) ── */
    KW_CAM_ON,          /* "打开摄像头" */
    KW_CAM_OFF,         /* "关闭摄像头" */
    KW_COLOR_TRACK,     /* "颜色追踪" */
    KW_LINE_FOLLOW,     /* "循迹" */
    KW_GESTURE_CTRL,    /* "手势控制" */
    KW_VISION_OFF,      /* "关闭视觉" */

    /* ── 导航路径 (P4本地执行) ── */
    KW_PATROL_SQUARE,   /* "方形巡逻" (path 0) */
    KW_PATROL_ZIGZAG,   /* "Z字巡检" (path 1) */
    KW_PATROL_STRAIGHT, /* "直线往返" (path 2) */
    KW_PATROL_SAMPLE,   /* "定点采样" (path 3) */

    KW_COUNT
} keyword_t;

/* ── Public API ────────────────────────────────────────── */

/**
 * @brief Initialize ESP-SR with WakeNet9 + MultiNet7.
 *
 * Models are loaded from SPIFFS partition "model".
 * This function is idempotent — safe to call multiple times.
 *
 * @return true if ESP-SR initialized successfully.
 *         false if models are missing or PSRAM allocation failed.
 */
bool sr_esp_init(void);

/**
 * @brief Feed one audio frame (512 samples @ 16kHz = 32ms).
 *
 * Must be called from a single task (not thread-safe).
 * Detection results are returned immediately after the frame
 * that completes a keyword pattern.
 *
 * @param samples  int16_t PCM samples @ 16kHz mono
 * @param count    number of samples (must be 512 for 32ms frames)
 * @return         detected keyword (KW_NONE if no detection this frame)
 */
keyword_t sr_esp_feed(const int16_t *samples, size_t count);

/**
 * @brief Reset detection state machines.
 *
 * Call after processing a keyword to prepare for the next detection.
 */
void sr_esp_reset(void);

/**
 * @brief Get Chinese name for a keyword.
 * @return static string (do not free)
 */
const char *sr_esp_keyword_name(keyword_t kw);

/**
 * @brief True if ESP-SR is initialized and models loaded OK.
 */
bool sr_esp_is_active(void);

/**
 * @brief De-initialize ESP-SR (free models, stop tasks).
 *        Called on system shutdown or reconfigure.
 */
void sr_esp_deinit(void);

/* ── MultiNet command set ────────────────────────────────── */
/*
 * 30 Chinese commands registered in pinyin during init.
 *
 * Motion (IDs 1-10):
 *   ID 1:  "qian jin"        → KW_FORWARD
 *   ID 2:  "hou tui"         → KW_BACKWARD
 *   ID 3:  "zuo zhuan"       → KW_LEFT
 *   ID 4:  "you zhuan"       → KW_RIGHT
 *   ID 5:  "ting zhi"        → KW_STOP
 *   ID 6:  "jia su"          → KW_FASTER
 *   ID 7:  "jian su"         → KW_SLOWER
 *   ID 8:  "zhuan quan"      → KW_SPIN
 *   ID 9:  "xun luo"         → KW_PATROL
 *   ID 10: "wang qian zou"   → KW_FORWARD (alias)
 *
 * Motion extended (IDs 11-14):
 *   ID 11: "diao tou"        → KW_TURN_AROUND
 *   ID 12: "quan su"         → KW_FULL_SPEED
 *   ID 13: "ban su"          → KW_HALF_SPEED
 *   ID 14: "di su"           → KW_LOW_SPEED
 *
 * Sensor queries (IDs 15-18) — forwarded to C5 for TTS:
 *   ID 15: "wen du"          → KW_TEMPERATURE
 *   ID 16: "shi du"          → KW_HUMIDITY
 *   ID 17: "kong qi zhi liang" → KW_AIR_QUALITY
 *   ID 18: "huan jing bao gao" → KW_ENV_REPORT
 *
 * Vision control (IDs 19-24):
 *   ID 19: "da kai she xiang tou" → KW_CAM_ON
 *   ID 20: "guan bi she xiang tou" → KW_CAM_OFF
 *   ID 21: "yan se zhui zong" → KW_COLOR_TRACK
 *   ID 22: "xun ji"           → KW_LINE_FOLLOW
 *   ID 23: "shou shi kong zhi" → KW_GESTURE_CTRL
 *   ID 24: "guan bi shi jue"  → KW_VISION_OFF
 *
 * Navigation (IDs 25-28):
 *   ID 25: "fang xing xun luo" → KW_PATROL_SQUARE
 *   ID 26: "zhe zi xun jian"   → KW_PATROL_ZIGZAG
 *   ID 27: "zhi xian wang fan" → KW_PATROL_STRAIGHT
 *   ID 28: "ding dian cai yang" → KW_PATROL_SAMPLE
 */

#define MN_CMD_COUNT           28
#define MN_CMD_ID_FORWARD       1
#define MN_CMD_ID_BACKWARD      2
#define MN_CMD_ID_LEFT          3
#define MN_CMD_ID_RIGHT         4
#define MN_CMD_ID_STOP          5
#define MN_CMD_ID_FASTER        6
#define MN_CMD_ID_SLOWER        7
#define MN_CMD_ID_SPIN          8
#define MN_CMD_ID_PATROL        9
#define MN_CMD_ID_FWD_ALIAS    10
#define MN_CMD_ID_TURN_AROUND  11
#define MN_CMD_ID_FULL_SPEED   12
#define MN_CMD_ID_HALF_SPEED   13
#define MN_CMD_ID_LOW_SPEED    14
#define MN_CMD_ID_TEMPERATURE  15
#define MN_CMD_ID_HUMIDITY     16
#define MN_CMD_ID_AIR_QUALITY  17
#define MN_CMD_ID_ENV_REPORT   18
#define MN_CMD_ID_CAM_ON       19
#define MN_CMD_ID_CAM_OFF      20
#define MN_CMD_ID_COLOR_TRACK  21
#define MN_CMD_ID_LINE_FOLLOW  22
#define MN_CMD_ID_GESTURE_CTRL 23
#define MN_CMD_ID_VISION_OFF   24
#define MN_CMD_ID_PATROL_SQUARE 25
#define MN_CMD_ID_PATROL_ZIGZAG 26
#define MN_CMD_ID_PATROL_STRAIGHT 27
#define MN_CMD_ID_PATROL_SAMPLE 28
