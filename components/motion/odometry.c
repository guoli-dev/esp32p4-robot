#include "odometry.h"
#include "motion_control.h"   /* WHEEL_BASE_CM, WHEEL_RADIUS_CM */
#include "motion_utils.h"
#include <math.h>
#include <string.h>

/*============================================================================
 * 差速运动学
 *
 *   中点近似（比简单欧拉精度更高）：
 *     d_center = (dl + dr) / 2
 *     d_theta  = (dr - dl) / wheel_base
 *
 *     x     += d_center · cos(θ + dθ/2)
 *     y     += d_center · sin(θ + dθ/2)
 *     theta += d_theta
 *============================================================================*/

static pose_t s_pose;

/*============================================================================
 * API
 *============================================================================*/

void odometry_init(void)
{
    memset(&s_pose, 0, sizeof(s_pose));
}

void odometry_reset(void)
{
    odometry_init();
}

void odometry_update(float dl_cm, float dr_cm)
{
    float dc = (dl_cm + dr_cm) / 2.0f;
    float dt = (dr_cm - dl_cm) / WHEEL_BASE_CM;
    float half_dt = dt / 2.0f;
    float mid_angle = s_pose.theta + half_dt;

    s_pose.x += dc * cosf(mid_angle);
    s_pose.y += dc * sinf(mid_angle);
    s_pose.theta += dt;

    normalize_angle(&s_pose.theta);
}

void odometry_get_pose(pose_t *pose)
{
    *pose = s_pose;
}

float odometry_get_heading_deg(void)
{
    return s_pose.theta * 180.0f / (float)M_PI;
}

float odometry_get_distance_cm(void)
{
    return sqrtf(s_pose.x * s_pose.x + s_pose.y * s_pose.y);
}
