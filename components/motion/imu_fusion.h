#pragma once

#include <stdbool.h>

/**
 * @file imu_fusion.h
 * @brief IMU + 编码器 互补滤波融合（仅航向角 / Yaw）
 *
 * 原理:
 *   陀螺仪短时精度高但会积分漂移；编码器航向无漂移但对轮子打滑敏感。
 *   互补滤波公式:
 *     yaw = α·(yaw + gyro_z·dt) + (1−α)·encoder_yaw
 *
 *   其中 α ∈ [0,1]：
 *     α → 1   更信任陀螺仪（适合快速转弯）
 *     α → 0   更信任编码器（适合慢速、直行）
 *
 * 用法:
 *   1. imu_fusion_init(&imu, 0.98f);
 *   2. 静止时校准陀螺零偏: imu_fusion_calibrate(&imu, samples, N);
 *   3. 每个控制周期: imu_fusion_update(&imu, gyro_z_rad_s, encoder_yaw, dt);
 *   4. 获取融合航向: float yaw = imu_fusion_get_yaw(&imu);
 */

typedef struct {
    float yaw;           /* 融合后的航向角 (弧度) */
    float gyro_bias_z;   /* 陀螺 Z 轴零偏 (rad/s)
                          * 注意：若驱动层已做校准（如 MPU6050），此字段应保持为 0，
                          * 避免双重扣除。仅当驱动层未校准时使用 imu_fusion_calibrate()。 */
    float alpha;         /* 互补滤波系数, 典型 0.95~0.99 */
    bool  calibrated;    /* 是否已校准零偏 */
} imu_fusion_t;

/**
 * @brief 初始化融合器
 * @param alpha  互补滤波系数 (0~1)，建议 0.98
 */
void imu_fusion_init(imu_fusion_t *f, float alpha);

/**
 * @brief 静止状态下校准陀螺零偏
 * @param samples  静止时采集的陀螺 Z 轴原始数据 (rad/s)
 * @param n        样本数
 */
void imu_fusion_calibrate(imu_fusion_t *f, const float *samples, int n);

/**
 * @brief 每个控制周期调用一次
 * @param gyro_z       陀螺 Z 轴角速度 (rad/s)，需已扣除零偏
 * @param encoder_yaw  编码器推算的航向角 (弧度)
 * @param dt           距上次调用时间 (秒)
 */
void imu_fusion_update(imu_fusion_t *f, float gyro_z, float encoder_yaw, float dt);

/**
 * @brief 获取融合后的航向角 (弧度)
 */
float imu_fusion_get_yaw(const imu_fusion_t *f);
