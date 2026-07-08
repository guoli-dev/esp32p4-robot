#pragma once

#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 * 底盘几何参数（根据实际机器人修改）
 *============================================================================*/
#define WHEEL_RADIUS_CM       3.25f   /* 轮子半径 (cm) */
#define WHEEL_BASE_CM         15.0f   /* 左右轮间距 / 轮距 (cm) */
#define ENCODER_PPR           11      /* 编码器线数（每圈脉冲数） */

/*============================================================================
 * 运动控制参数
 *============================================================================*/
#define MOTION_CTRL_FREQ_HZ   100     /* PID 控制频率 (Hz) */
#define MOTION_PID_KP         0.5f    /* 速度环比例系数 */
#define MOTION_PID_KI         0.1f    /* 速度环积分系数 */
#define MOTION_PID_KD         0.05f   /* 速度环微分系数 */
#define MOTION_MAX_RPM        300.0f  /* 100% 速度对应的 RPM（需实测标定） */

/*============================================================================
 * 梯形速度曲线参数
 *============================================================================*/
#define MOTION_ACCEL_RPM_S     150.0f  /* 加速度 (RPM/s) — 需实测标定 */
#define MOTION_DECEL_RPM_S     150.0f  /* 减速度 (RPM/s)，通常等于加速度 */
#define MOTION_USE_TRAPEZOIDAL 1       /* 1 = 梯形速度曲线, 0 = 恒定速度 */

/*============================================================================
 * IMU 融合参数
 *============================================================================*/
#define MOTION_USE_IMU         0       /* 0 = 暂禁用 (I2C驱动冲突，待迁移) */
#define MOTION_IMU_ALPHA       0.98f   /* 互补滤波系数 (0~1, 越大越信任陀螺) */
#define MOTION_IMU_CALIB_SAMPLES 400  /* 上电校准陀螺采样数 */

/*============================================================================
 * 电机分配（左/右侧各两个电机）
 * M1=0, M2=1, M3=2, M4=3
 *============================================================================*/
#define MOTOR_LEFT_1          0       /* 左前 */
#define MOTOR_LEFT_2          1       /* 左后 */
#define MOTOR_RIGHT_1         2       /* 右前 */
#define MOTOR_RIGHT_2         3       /* 右后 */

/* 编码器与电机对应（同序号编码器对应同序号电机） */

/*============================================================================
 * 运动状态
 *============================================================================*/
typedef enum {
    MOTION_IDLE = 0,    /* 空闲，无运动指令 */
    MOTION_RUNNING,     /* 运动中 */
    MOTION_DONE,        /* 运动完成 */
    MOTION_STOPPED,     /* 被 motion_stop() 中止 */
} motion_state_t;

/*============================================================================
 * API
 *============================================================================*/

/**
 * @brief 初始化运动控制层并启动后台控制任务
 *
 * 调用前需先初始化 motor_init() 和 encoder_init()。
 */
void motion_init(void);

/**
 * @brief 直线行驶
 * @param distance_cm  距离，正=前进，负=后退 (cm)
 * @param speed        速度百分比 1~100（取绝对值，方向由 distance_cm 符号决定）
 *
 * 调用后立即返回，通过 motion_is_done() 或 motion_wait() 等待完成。
 */
void motion_straight(float distance_cm, int16_t speed);

/**
 * @brief 原地转弯（差速转向）
 * @param angle_deg    角度，正=逆时针/左转，负=顺时针/右转 (度)
 * @param speed        速度百分比 1~100
 *
 * 调用后立即返回。
 */
void motion_turn(float angle_deg, int16_t speed);

/**
 * @brief 弧线行驶
 * @param radius_cm    转弯半径，正=右转弧线，负=左转弧线 (cm)
 * @param angle_deg    弧线角度 (度)
 * @param speed        速度百分比 1~100
 *
 * 调用后立即返回。
 */
void motion_arc(float radius_cm, float angle_deg, int16_t speed);

/**
 * @brief 紧急停止并取消当前运动
 */
void motion_stop(void);

/**
 * @brief 查询当前运动是否完成
 * @return true=完成/空闲，false=仍在运动中
 */
bool motion_is_done(void);

/**
 * @brief 阻塞等待当前运动完成
 */
void motion_wait(void);

/**
 * @brief 获取当前运动状态
 */
motion_state_t motion_get_state(void);

/**
 * @brief 获取 IMU 融合后的航向角
 * @return 融合航向 (弧度)，若 IMU 未启用则返回编码器航向
 */
float motion_get_imu_yaw(void);

/**
 * @brief 查询 IMU 是否已启用
 * @return true = IMU 融合正常工作
 */
bool motion_is_imu_enabled(void);

/**
 * @brief 校准 IMU 陀螺零偏（机器人必须静止！）
 *
 * 内部调用 mpu6050_calibrate()，采集 MOTION_IMU_CALIB_SAMPLES 个样本，
 * 将零偏写入 MPU6050 驱动。校准后陀螺数据自动扣除零偏。
 *
 * 通常在 motion_init() 之后、首次运动之前调用。
 */
void motion_imu_calibrate(void);

/*============================================================================
 * 内部 API — 不等待上一运动完成，直接启动
 *
 * 仅 waypoint 引擎使用，用户代码请用 motion_straight/turn/arc。
 *============================================================================*/
void motion_straight_nowait(float distance_cm, int16_t speed);
void motion_turn_nowait(float angle_deg, int16_t speed);
void motion_arc_nowait(float radius_cm, float angle_deg, int16_t speed);
