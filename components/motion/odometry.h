#pragma once

/**
 * @file odometry.h
 * @brief 航位推算 (Dead Reckoning) — 基于编码器的位姿估计
 *
 * 使用差速运动学模型，从左右轮位移增量推算出全局 (x, y, θ)。
 * 每个控制周期调用一次 odometry_update() 累积位姿。
 */

/** @brief 2D 位姿 */
typedef struct {
    float x;       /* cm */
    float y;       /* cm */
    float theta;   /* 弧度, [-π, π] */
} pose_t;

/**
 * @brief 初始化 / 复位里程计
 */
void odometry_init(void);
void odometry_reset(void);

/**
 * @brief 更新里程计
 *
 * @param dl_cm  左侧本轮走过距离 (cm)，正=前进
 * @param dr_cm  右侧本轮走过距离 (cm)，正=前进
 *
 * 每个控制周期调用一次，传入自上次调用以来的增量。
 * 内部使用中点近似减少离散化误差。
 */
void odometry_update(float dl_cm, float dr_cm);

/**
 * @brief 获取当前位姿
 */
void odometry_get_pose(pose_t *pose);

/**
 * @brief 便捷函数：获取航向角 (度)
 */
float odometry_get_heading_deg(void);

/**
 * @brief 获取累计行驶总里程 (cm)
 */
float odometry_get_distance_cm(void);
