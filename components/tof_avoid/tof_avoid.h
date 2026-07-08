/**
 * @file tof_avoid.h — VL53L1X ToF 避障组件 (单传感器)
 *
 * 1 颗 VL53L1X 激光 ToF, 正前方安装。
 * I2C1 (GPIO54/53), 与 MPU6050 (0x68) 同总线, 默认地址 0x29 不冲突。
 *
 * 避障阈值:
 *   TOF_STOP_CM    30cm — 停车
 *   TOF_SLOW_CM    60cm — 降速 30%
 *   TOF_WARN_CM   100cm — BLE 预警
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

/* ── 硬件引脚 ─────────────────────────────────────── */

#define TOF_I2C_PORT        1          /* I2C_NUM_1, 与 MPU6050 共用 */
#define TOF_SDA_GPIO        54         /* J3-2 */
#define TOF_SCL_GPIO        53         /* J3-4 */
#define TOF_XSHUT_GPIO      25         /* J3-35, XSHUT 使能 (GPIO24 固定给 CAM_XCLK) */

/* ── 避障阈值 (cm) ────────────────────────────────── */

#define TOF_STOP_CM         30         /* 停车距离 */
#define TOF_SLOW_CM         60         /* 降速距离 */
#define TOF_WARN_CM         100        /* 预警距离 */
#define TOF_MAX_RANGE_CM    400        /* 最大量程 */
#define TOF_SLOW_SPEED_PCT  30         /* 遇障减速后的速度 % */

/* ── 避障等级 ──────────────────────────────────────── */

typedef enum {
    OBSTACLE_NONE   = 0,   /* 安全, 无障碍 */
    OBSTACLE_WARN   = 1,   /* 预警: 100cm 内有物 */
    OBSTACLE_SLOW   = 2,   /* 减速: 60cm 内有物 */
    OBSTACLE_STOP   = 3,   /* 停车: 30cm 内有物 */
} obstacle_level_t;

/* ── 测距结果 ──────────────────────────────────────── */

typedef struct {
    uint16_t range_mm;         /* 距离 (mm), 0=无效测量 */
    obstacle_level_t level;
} tof_result_t;

/* ── Public API ────────────────────────────────────── */

/** @brief 初始化 I2C 总线 + VL53L1X, 启动连续测距 */
bool tof_avoid_init(void);

/** @brief 获取最近一次结果 (线程安全, copy-out) */
bool tof_avoid_peek(tof_result_t *out);

/**
 * @brief 正前方是否安全 (供 patrol 步进前调用)
 * @param distance_cm  欲行驶距离 (正=前进, 负=后退)
 * @return true=安全可走, false=前方有障碍
 */
bool tof_avoid_safe_to_move(float distance_cm);

/** @brief 根据当前障碍等级降速 */
int tof_avoid_clamp_speed(int requested_pct);
