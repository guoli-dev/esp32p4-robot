#include "pid.h"
#include <math.h>

void pid_init(pid_ctrl_t *pid, float kp, float ki, float kd, float out_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_limit = fabsf(out_limit);
    pid->integral_limit = pid->output_limit * 0.8f; /* 默认积分限幅为输出限幅的 80% */
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->first = true;
}

float pid_compute(pid_ctrl_t *pid, float setpoint, float measurement, float dt)
{
    if (dt <= 0.0f) return 0.0f;

    float error = setpoint - measurement;

    /* 比例项 */
    float p_out = pid->kp * error;

    /* 积分项（带限幅抗饱和） */
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    float i_out = pid->ki * pid->integral;

    /* 微分项（对测量值微分避免微分冲击）
     * 复位后首周期跳过微分 — prev_measurement 尚无有效值 */
    float d_out = 0.0f;
    if (pid->first) {
        pid->first = false;
    } else {
        d_out = -pid->kd * (measurement - pid->prev_measurement) / dt;
    }
    pid->prev_measurement = measurement;

    /* 合成输出并限幅 */
    float output = p_out + i_out + d_out;
    if (output > pid->output_limit) {
        output = pid->output_limit;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }

    return output;
}

void pid_reset(pid_ctrl_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->first = true;
}
