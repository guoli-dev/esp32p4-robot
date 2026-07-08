#include "speed_profile.h"
#include <math.h>
#include <string.h>

/*----------------------------------------------------------------------------
 * 内部: 将死区附近的微小值归零
 *----------------------------------------------------------------------------*/
static inline float deadband(float val, float threshold)
{
    return (fabsf(val) < threshold) ? 0.0f : val;
}

/*============================================================================
 * sp_init — 规划三段式梯形曲线
 *============================================================================*/
void sp_init(speed_profile_t *sp, float total_edges, float max_rpm,
             float accel_rpm_s, float decel_rpm_s, float edges_per_rev)
{
    memset(sp, 0, sizeof(*sp));

    if (total_edges < 1.0f || max_rpm <= 0.0f) {
        sp->total = total_edges;
        sp->edges_per_rev = edges_per_rev;
        return;  /* 无效参数 → sp_compute 永远返回 0 */
    }

    sp->total         = total_edges;
    sp->edges_per_rev = edges_per_rev;

    /* 单位换算: RPM → edges/s */
    float rpm2eps = edges_per_rev / 60.0f;

    float w_max = max_rpm * rpm2eps;        /* 最大速度 (edges/s) */
    float a     = accel_rpm_s * rpm2eps;     /* 加速度 (edges/s²) */
    float d     = decel_rpm_s * rpm2eps;     /* 减速度 (edges/s²) */

    sp->a = a;
    sp->d = d;

    /* 加速/减速所需距离 */
    float s_a = (w_max * w_max) / (2.0f * a);
    float s_d = (w_max * w_max) / (2.0f * d);

    if (s_a + s_d > total_edges) {
        /*--------------------------------------------------------------
         * 三角曲线: 行程太短，无法达到 Vmax。
         * 实际峰值满足: v²/(2a) + v²/(2d) = S  ⇒  v = √(2·a·d·S / (a+d))
         *--------------------------------------------------------------*/
        float v_peak = sqrtf(2.0f * a * d * total_edges / (a + d));
        sp->v_max   = v_peak / rpm2eps;          /* 换算回 RPM */
        sp->s_accel = (v_peak * v_peak) / (2.0f * a);
        sp->s_decel = (v_peak * v_peak) / (2.0f * d);
        sp->s_cruise = 0.0f;
    } else {
        /* 完整梯形 */
        sp->v_max   = max_rpm;
        sp->s_accel = s_a;
        sp->s_decel = s_d;
        sp->s_cruise = total_edges - s_a - s_d;
    }
}

/*============================================================================
 * sp_compute — 根据已走距离推算目标速度
 *============================================================================*/
float sp_compute(speed_profile_t *sp, float n)
{
    /* 边界: 未出发 / 已到达 */
    if (n <= 0.0f)       return 0.0f;
    if (n >= sp->total)  return 0.0f;
    if (sp->v_max <= 0)  return 0.0f;

    float rpm2eps = sp->edges_per_rev / 60.0f;
    float w;  /* edges/s */

    if (n < sp->s_accel) {
        /* ── 加速段 ──  v = √(2·a·n)  */
        w = sqrtf(2.0f * sp->a * n);
    } else if (n < sp->s_accel + sp->s_cruise) {
        /* ── 匀速段 ──  */
        w = sp->v_max * rpm2eps;
    } else {
        /* ── 减速段 ──  v = √(2·d·(S−n))  */
        float remaining = sp->total - n;
        w = sqrtf(2.0f * sp->d * remaining);
    }

    return deadband(w / rpm2eps, 0.5f);   /* < 0.5 RPM 直接归零 */
}
