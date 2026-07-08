#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @file waypoint.h
 * @brief 路径规划 — 一系列 Waypoint 的顺序/循环执行
 *
 * 使用示例:
 * @code
 *   waypoint_t path[] = {
 *       { WP_STRAIGHT, .param1 = 50,  .speed = 60 },
 *       { WP_TURN,     .param1 = 90,  .speed = 40 },
 *       { WP_ARC,      .param1 = 30,  .param2 = 180, .speed = 50 },
 *       { WP_WAIT,     .param1 = 500 },
 *       { WP_STOP },
 *   };
 *   waypoint_start(path, 5, false);       // 执行一次
 *   while (!waypoint_is_done()) { ... }   // 等待完成
 * @endcode
 *
 * WP_STOP 表示路径终止，之后的 waypoint 不会执行。不含 WP_STOP 且 loop=true
 * 时会无限循环。
 */

/*============================================================================
 * Waypoint 类型
 *============================================================================*/
typedef enum {
    WP_STRAIGHT = 0,  /**< 直线:           param1 = distance_cm            */
    WP_TURN,           /**< 原地转弯:       param1 = angle_deg              */
    WP_ARC,            /**< 弧线:           param1 = radius_cm, param2 = angle_deg */
    WP_WAIT,           /**< 原地等待:       param1 = wait_ms               */
    WP_STOP,           /**< 终止路径                                          */
} wp_type_t;

typedef struct {
    wp_type_t type;
    float     param1;      /**< 含义取决于 type */
    float     param2;      /**< 含义取决于 type */
    int16_t   speed;       /**< 速度百分比 1~100 (WP_WAIT / WP_STOP 忽略) */
} waypoint_t;

/*============================================================================
 * API
 *============================================================================*/

/**
 * @brief 开始执行路径
 *
 * @param wps    路径点数组（会被内部拷贝，调用后可释放/复用）
 * @param count  路径点数量 (≤ 32)
 * @param loop   true = 无限循环执行，直到 waypoint_stop()
 *
 * 调用后立即返回（非阻塞）。通过 waypoint_is_done() 或轮询 motion_is_done()
 * 来跟踪进度。
 */
void waypoint_start(const waypoint_t *wps, int count, bool loop);

/**
 * @brief 中止路径执行并停车
 */
void waypoint_stop(void);

/**
 * @brief 查询路径是否已执行完毕
 * @return true = 所有路径点已执行完毕（或未在路径模式）
 */
bool waypoint_is_done(void);

/**
 * @brief 获取当前正在执行的路径点索引 (0-based)
 * @return -1 = 未在路径模式下
 */
int waypoint_current_index(void);

/**
 * @brief 供 motion_control 内部调用：加载下一个路径点
 *
 * 当当前运动完成时由控制任务调用，自动启动下一个路径点对应的运动。
 * @return true  = 已加载新运动，false = 无更多路径点（路径结束）
 */
bool waypoint_load_next(void);
