#pragma once

#include <stdint.h>

/* M1 引脚 (D24A: PWMA/AIN1/AIN2 — AIN1 避开 BOOT GPIO0) */
#define M1_PWM  2
#define M1_IN1  45
#define M1_IN2  1

/* M2 引脚 (D24A: PWMB/BIN1/BIN2) */
#define M2_PWM  6
#define M2_IN1  3
#define M2_IN2  4

/* M3 引脚 (D24A: PWMC/CIN1/CIN2)
 * IN1/IN2 moved from GPIO7/GPIO8→22/23 to free camera SCCB I2C.
 * Re-route TB6612 CIN1→J2-36(GPIO22), CIN2→J2-38(GPIO23). */
#define M3_PWM  9
#define M3_IN1  22      /* was GPIO7  — freed for CAM_SDA */
#define M3_IN2  23      /* was GPIO8  — freed for CAM_SCL */

/* M4 引脚 (D24A: PWMD/DIN1/DIN2 — IN1/IN2 让出 GPIO49/50 给 MQ 传感器 ADC) */
#define M4_PWM  20
#define M4_IN1  28         /* J3-29, moved from GPIO49 to free ADC2_CH0 */
#define M4_IN2  29         /* J3-27, moved from GPIO50 to free ADC2_CH1 */

/* STBY 引脚 (D24A: 全局使能) */
#define MOTOR_STBY 21

/**
 * @brief 初始化四路 TB6612 电机驱动
 */
void motor_init(void);

/**
 * @brief 设置单个电机速度和方向
 * @param motor 电机编号：0(M1)、1(M2)、2(M3)、3(M4)
 * @param speed 速度值：-100~100（正=正转，负=反转，0=自由停止）
 */
void motor_set_speed(uint8_t motor, int16_t speed);

/**
 * @brief 单个电机自由滑行停止
 * @param motor 电机编号：0~3
 */
void motor_stop(uint8_t motor);

/**
 * @brief 单个电机紧急刹车停止
 * @param motor 电机编号：0~3
 */
void motor_brake(uint8_t motor);

/**
 * @brief 使能驱动板（电机可工作）
 */
void motor_standby_enable(void);

/**
 * @brief 禁用驱动板（所有电机停止，低功耗）
 */
void motor_standby_disable(void);
