#pragma once

#include <stdbool.h>

/**
 * @file speed_profile.h
 * @brief 梯形速度曲线规划器
 *
 * 将运动分为三段：加速 → 匀速 → 减速。
 * 若总行程太短无法达到最大速度，自动退化为三角形曲线（无匀速段）。
 *
 * 内部使用编码器边沿数 (edges) 作为距离单位，与运动控制层一致。
 * sp_compute() 返回的是 RPM 大小（始终 >= 0），方向由调用方施加。
 */

typedef struct {
    float s_accel;        /* 加速段距离 (edges) */
    float s_cruise;       /* 匀速段距离 (edges) */
    float s_decel;        /* 减速段距离 (edges) */
    float v_max;          /* 巡航速度 (RPM)，三角曲线时为实际峰值 */
    float a;              /* 加速度 (edges/s²) */
    float d;              /* 减速度 (edges/s²) */
    float edges_per_rev;  /* 每圈边沿数 = ENCODER_PPR × 4 */
    float total;          /* 总行程 (edges) */
} speed_profile_t;

/**
 * @brief 初始化梯形速度曲线
 *
 * @param sp              曲线实例
 * @param total_edges     总行程（编码器边沿数，始终 >= 0）
 * @param max_rpm         最大巡航速度 (RPM, >= 0)
 * @param accel_rpm_s     加速度 (RPM/s)
 * @param decel_rpm_s     减速度 (RPM/s)
 * @param edges_per_rev   每圈边沿数 = ENCODER_PPR × 4
 */
void sp_init(speed_profile_t *sp, float total_edges, float max_rpm,
             float accel_rpm_s, float decel_rpm_s, float edges_per_rev);

/**
 * @brief 计算当前位置对应的瞬时目标速度
 *
 * @param sp            曲线实例
 * @param dist_traveled 已走过距离（编码器边沿数，>= 0）
 * @return 目标速度 (RPM, >= 0)，到达终点返回 0
 */
float sp_compute(speed_profile_t *sp, float dist_traveled);

/**
 * @brief 检查是否已走完全程
 */
static inline bool sp_is_done(speed_profile_t *sp, float dist_traveled)
{
    return dist_traveled >= sp->total;
}
