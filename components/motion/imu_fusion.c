#include "imu_fusion.h"
#include "motion_utils.h"
#include <math.h>
#include <string.h>

/*============================================================================
 * API
 *============================================================================*/

void imu_fusion_init(imu_fusion_t *f, float alpha)
{
    memset(f, 0, sizeof(*f));
    f->alpha = alpha;

    /* 钳制 alpha 到合理区间 */
    if (f->alpha < 0.0f)  f->alpha = 0.0f;
    if (f->alpha > 1.0f)  f->alpha = 1.0f;
}

void imu_fusion_calibrate(imu_fusion_t *f, const float *samples, int n)
{
    if (n <= 0 || !samples) return;

    float sum = 0.0f;
    for (int i = 0; i < n; i++) sum += samples[i];

    f->gyro_bias_z = sum / (float)n;
    f->calibrated  = true;
}

void imu_fusion_update(imu_fusion_t *f, float gyro_z, float encoder_yaw, float dt)
{
    if (dt <= 0.0f) return;

    /* 扣除零偏 */
    float gz = gyro_z - f->gyro_bias_z;

    /* 陀螺仪积分 */
    float gyro_yaw = f->yaw + gz * dt;

    /* 互补滤波: α·陀螺 + (1-α)·编码器 */
    f->yaw = f->alpha * gyro_yaw + (1.0f - f->alpha) * encoder_yaw;

    normalize_angle(&f->yaw);
}

float imu_fusion_get_yaw(const imu_fusion_t *f)
{
    return f->yaw;
}
