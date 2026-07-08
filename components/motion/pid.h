#pragma once

#include <stdbool.h>

/**
 * @brief PID 控制器
 *
 * 增量式 PID，带积分限幅抗饱和和输出限幅。
 * 用于运动控制层的速度闭环。
 */

typedef struct {
    float kp;              /* 比例系数 */
    float ki;              /* 积分系数 */
    float kd;              /* 微分系数 */
    float integral_limit;  /* 积分限幅（抗饱和） */
    float output_limit;    /* 输出限幅（绝对值） */
    float integral;        /* 积分累加值 */
    float prev_measurement;/* 上一次测量值（用于微分项，避免微分冲击） */
    bool  first;           /* 复位后首周期标记 — 跳过微分项 */
} pid_ctrl_t;

/**
 * @brief 初始化 PID 控制器
 * @param pid        PID 实例指针
 * @param kp         比例系数
 * @param ki         积分系数
 * @param kd         微分系数
 * @param out_limit  输出限幅（绝对值），如 30.0 表示输出 ∈ [-30, 30]
 */
void pid_init(pid_ctrl_t *pid, float kp, float ki, float kd, float out_limit);

/**
 * @brief 计算 PID 输出
 * @param pid         PID 实例指针
 * @param setpoint    目标值
 * @param measurement 实际测量值
 * @param dt          距上次计算的时间间隔（秒）
 * @return PID 输出值（受 output_limit 约束）
 */
float pid_compute(pid_ctrl_t *pid, float setpoint, float measurement, float dt);

/**
 * @brief 复位 PID（清零积分和误差历史）
 */
void pid_reset(pid_ctrl_t *pid);
