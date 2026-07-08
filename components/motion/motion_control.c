#include "motion_control.h"
#include "motor_control.h"
#include "encoder.h"
#include "pid.h"
#include "waypoint.h"
#include "speed_profile.h"
#include "odometry.h"
#include "imu_fusion.h"
#include "mpu6050.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

#define TAG "motion"

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

/*============================================================================
 * 内部常量
 *============================================================================*/

#define EDGES_PER_PULSE        4     /* 4x 正交解码 */
#define WHEEL_CIRCUMFERENCE    ((float)(2.0 * M_PI * WHEEL_RADIUS_CM))
#define EDGES_PER_REV          ((int32_t)(ENCODER_PPR * EDGES_PER_PULSE))
#define CM_PER_EDGE            (WHEEL_CIRCUMFERENCE / (float)EDGES_PER_REV)
#define CTRL_PERIOD_MS         (1000 / MOTION_CTRL_FREQ_HZ)
#define HALF_BASE              (WHEEL_BASE_CM / 2.0f)

/*============================================================================
 * 运动指令类型
 *============================================================================*/
typedef enum {
    CMD_NONE = 0,
    CMD_STRAIGHT,
    CMD_TURN,
    CMD_ARC,
} motion_cmd_t;

/*============================================================================
 * 运动控制内部状态
 *============================================================================*/
static motion_state_t s_state = MOTION_IDLE;
static motion_cmd_t   s_cmd   = CMD_NONE;

/* 目标编码器边沿数（正=前进方向，负=后退方向） */
static int32_t s_target_edges_left;
static int32_t s_target_edges_right;

/* 左/右两侧的目标 RPM（PID 设定值，带符号） */
static float s_target_rpm_left;
static float s_target_rpm_right;

/* 运动开始时各编码器计数值 */
static int32_t s_start_count[4];

/* PID 控制器（左/右侧各一个速度环） */
static pid_ctrl_t s_pid_left;
static pid_ctrl_t s_pid_right;

/*============================================================================
 * 梯形速度曲线状态
 *============================================================================*/
static speed_profile_t s_sp_left;     /* 左侧速度曲线 */
static speed_profile_t s_sp_right;    /* 右侧速度曲线 */
static int             s_dir_left;    /* +1 前进, -1 后退, 0 静止 */
static int             s_dir_right;
static bool            s_use_profile; /* 是否启用梯形曲线（由 MOTION_USE_TRAPEZOIDAL 决定） */

/*============================================================================
 * 里程计 / 航位推算 状态
 *============================================================================*/
static int32_t s_last_enc[4];         /* 上一周期编码器读数（用于计算增量） */

/*============================================================================
 * IMU 融合 状态
 *============================================================================*/
static imu_fusion_t s_imu;
static bool         s_imu_enabled = false;
static float        s_turn_start_yaw;     /* 转弯起始航向 (rad) */
static float        s_turn_target_rad;    /* 目标转角 (rad) */

/*============================================================================
 * 弱函数: IMU 陀螺读取（用户需根据实际 IMU 芯片实现）
 *
 * 返回 true 表示成功读取。默认返回 false（无 IMU）。
 * 例如 MPU6050 / ICM-42688 通过 I2C 读取后填入 gx/gy/gz。
 *============================================================================*/
__attribute__((weak)) bool imu_read_gyro(float *gx, float *gy, float *gz)
{
    (void)gx; (void)gy; (void)gz;
    return false;   /* 默认: 无 IMU */
}

/*============================================================================
 * 辅助函数
 *============================================================================*/

static float speed_to_rpm(float speed_percent)
{
    return (speed_percent / 100.0f) * MOTION_MAX_RPM;
}

static int16_t rpm_to_speed(float rpm)
{
    int16_t s = (int16_t)(rpm / MOTION_MAX_RPM * 100.0f);
    if (s > 100)  s = 100;
    if (s < -100) s = -100;
    return s;
}

static float get_side_rpm(uint8_t m1, uint8_t m2)
{
    float rpm1 = encoder_get_rpm(m1, ENCODER_PPR);
    float rpm2 = encoder_get_rpm(m2, ENCODER_PPR);
    return (rpm1 + rpm2) / 2.0f;
}

static int32_t get_side_edges(uint8_t m1, uint8_t m2)
{
    int32_t d1 = encoder_get_count(m1) - s_start_count[m1];
    int32_t d2 = encoder_get_count(m2) - s_start_count[m2];

    /* PCNT 16-bit wrap */
    if (d1 > 32767)       d1 -= 65536;
    else if (d1 < -32768) d1 += 65536;
    if (d2 > 32767)       d2 -= 65536;
    else if (d2 < -32768) d2 += 65536;

    return (d1 + d2) / 2;
}

/** @brief 用预读的编码器值计算侧边沿数，避免重复 PCNT 读取 */
static int32_t get_side_edges_pre(const int32_t cur[4], uint8_t m1, uint8_t m2)
{
    int32_t d1 = (int32_t)cur[m1] - s_start_count[m1];
    int32_t d2 = (int32_t)cur[m2] - s_start_count[m2];

    if (d1 > 32767)       d1 -= 65536;
    else if (d1 < -32768) d1 += 65536;
    if (d2 > 32767)       d2 -= 65536;
    else if (d2 < -32768) d2 += 65536;

    return (d1 + d2) / 2;
}

static void snapshot_start_counts(void)
{
    for (int i = 0; i < 4; i++) {
        s_start_count[i] = encoder_get_count(i);
    }
}

static void set_all_motors(int16_t speed)
{
    for (int i = 0; i < 4; i++) {
        motor_set_speed(i, speed);
    }
}

static void set_left_motors(int16_t speed)
{
    motor_set_speed(MOTOR_LEFT_1, speed);
    motor_set_speed(MOTOR_LEFT_2, speed);
}

static void set_right_motors(int16_t speed)
{
    motor_set_speed(MOTOR_RIGHT_1, speed);
    motor_set_speed(MOTOR_RIGHT_2, speed);
}

/*============================================================================
 * 里程计更新（每个控制周期调用）
 *============================================================================*/
/** @brief 里程计更新（用预读的 cur[4]，不再重复读取 PCNT） */
static void update_odometry(const int32_t cur[4])
{
    int32_t dl1 = (int32_t)cur[MOTOR_LEFT_1]  - s_last_enc[MOTOR_LEFT_1];
    int32_t dl2 = (int32_t)cur[MOTOR_LEFT_2]  - s_last_enc[MOTOR_LEFT_2];
    int32_t dr1 = (int32_t)cur[MOTOR_RIGHT_1] - s_last_enc[MOTOR_RIGHT_1];
    int32_t dr2 = (int32_t)cur[MOTOR_RIGHT_2] - s_last_enc[MOTOR_RIGHT_2];

    if (dl1 > 32767) dl1 -= 65536; else if (dl1 < -32768) dl1 += 65536;
    if (dl2 > 32767) dl2 -= 65536; else if (dl2 < -32768) dl2 += 65536;
    if (dr1 > 32767) dr1 -= 65536; else if (dr1 < -32768) dr1 += 65536;
    if (dr2 > 32767) dr2 -= 65536; else if (dr2 < -32768) dr2 += 65536;

    int32_t dl = (dl1 + dl2) / 2;
    int32_t dr = (dr1 + dr2) / 2;

    odometry_update((float)dl * CM_PER_EDGE, (float)dr * CM_PER_EDGE);

    memcpy(s_last_enc, cur, sizeof(s_last_enc));
}

/*============================================================================
 * 速度曲线初始化（两侧）
 *============================================================================*/
static void init_speed_profiles(float total_edges_left, float total_edges_right,
                                float max_rpm_left, float max_rpm_right)
{
    if (!s_use_profile) return;

    sp_init(&s_sp_left,
            fabsf(total_edges_left),
            fabsf(max_rpm_left),
            MOTION_ACCEL_RPM_S,
            MOTION_DECEL_RPM_S,
            (float)EDGES_PER_REV);

    sp_init(&s_sp_right,
            fabsf(total_edges_right),
            fabsf(max_rpm_right),
            MOTION_ACCEL_RPM_S,
            MOTION_DECEL_RPM_S,
            (float)EDGES_PER_REV);

    s_dir_left  = (max_rpm_left  >= 0) ? 1 : -1;
    s_dir_right = (max_rpm_right >= 0) ? 1 : -1;
}

/*============================================================================
 * 判断是否到达目标
 *
 * CMD_TURN + IMU 启用 → 用融合航向判定
 * 否则 → 用编码器边沿数判定
 *============================================================================*/
static bool target_reached(motion_cmd_t cmd, int32_t target_edges_l, int32_t target_edges_r,
                           float turn_start_yaw, float turn_target_rad,
                           const int32_t cur[4])
{
    /* ── IMU 增强转弯判定 ── */
    if (cmd == CMD_TURN && s_imu_enabled) {
        float yaw   = imu_fusion_get_yaw(&s_imu);
        float delta = yaw - turn_start_yaw;

        while (delta > (float)M_PI)  delta -= 2.0f * (float)M_PI;
        while (delta < -(float)M_PI) delta += 2.0f * (float)M_PI;

        float gz = 0.0f;
#if MOTION_USE_IMU
        {
            float dummy_x, dummy_y;
            if (!imu_read_gyro(&dummy_x, &dummy_y, &gz)) {
                /* I2C glitch: fall back to encoder-only check */
                gz = 0.0f;
            }
        }
#endif
        bool angle_ok  = fabsf(delta) >= fabsf(turn_target_rad) * 0.98f;

        /* Only trust gyro settling if gyro read succeeded (gz != 0 or angle was set) */
        bool settled;
        if (gz == 0.0f) {
            /* gyro failed → skip gyro settling, approve by angle alone */
            settled = true;
        } else {
            settled = (fabsf(gz) < 0.05f);
        }
        return angle_ok && settled;
    }

    /* ── 标准编码器判定（用预读 cur[]，不再重复读取）── */
    int32_t cur_l = get_side_edges_pre(cur, MOTOR_LEFT_1, MOTOR_LEFT_2);
    int32_t cur_r = get_side_edges_pre(cur, MOTOR_RIGHT_1, MOTOR_RIGHT_2);

    bool l_done = (target_edges_l >= 0) ? (cur_l >= target_edges_l)
                                        : (cur_l <= target_edges_l);
    bool r_done = (target_edges_r >= 0) ? (cur_r >= target_edges_r)
                                        : (cur_r <= target_edges_r);
    return l_done && r_done;
}

/*============================================================================
 * 控制任务 (100 Hz)
 *============================================================================*/

static void motion_ctrl_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    const float dt = (float)CTRL_PERIOD_MS / 1000.0f;

    /* 初始化里程计编码器快照 */
    for (int i = 0; i < 4; i++) {
        s_last_enc[i] = encoder_get_count(i);
    }

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CTRL_PERIOD_MS));

        /*================================================================
         * 0. 读取所有编码器（每周期仅此一次）
         *===============================================================*/
        int32_t cur[4];
        for (int i = 0; i < 4; i++) cur[i] = encoder_get_count(i);

        /*================================================================
         * 1. 里程计更新（始终运行）
         *===============================================================*/
        update_odometry(cur);

        /*================================================================
         * 2. IMU 融合更新
         *===============================================================*/
#if MOTION_USE_IMU
        float gx, gy, gz;
        if (s_imu_enabled && imu_read_gyro(&gx, &gy, &gz)) {
            float enc_yaw = odometry_get_heading_deg() * (float)M_PI / 180.0f;
            imu_fusion_update(&s_imu, gz, enc_yaw, dt);
        }
#endif

        /*================================================================
         * 3. 睡眠时跳过运动控制
         *===============================================================*/
        portENTER_CRITICAL(&s_lock);
        bool running = (s_state == MOTION_RUNNING);
        float target_rpm_l = s_target_rpm_left;
        float target_rpm_r = s_target_rpm_right;
        int32_t target_edges_l = s_target_edges_left;
        int32_t target_edges_r = s_target_edges_right;
        motion_cmd_t cmd = s_cmd;
        float turn_start_yaw = s_turn_start_yaw;
        float turn_target_rad = s_turn_target_rad;
        portEXIT_CRITICAL(&s_lock);

        if (!running) {
            continue;
        }

        /*================================================================
         * 4. 实际 RPM（encoder_get_rpm 内部读取以获取时序增量）
         *===============================================================*/
        float rpm_l = get_side_rpm(MOTOR_LEFT_1, MOTOR_LEFT_2);
        float rpm_r = get_side_rpm(MOTOR_RIGHT_1, MOTOR_RIGHT_2);

        /*================================================================
         * 5. 梯形速度曲线 → 瞬时目标 RPM（用预读 cur[] 算边沿）
         *===============================================================*/
        if (s_use_profile) {
            float traveled_l = (float)labs(get_side_edges_pre(cur,
                                          MOTOR_LEFT_1, MOTOR_LEFT_2));
            float traveled_r = (float)labs(get_side_edges_pre(cur,
                                          MOTOR_RIGHT_1, MOTOR_RIGHT_2));

            float mag_l = sp_compute(&s_sp_left,  traveled_l);
            float mag_r = sp_compute(&s_sp_right, traveled_r);

            target_rpm_l = (float)s_dir_left  * mag_l;
            target_rpm_r = (float)s_dir_right * mag_r;
        }

        /*================================================================
         * 6. PID 速度修正
         *===============================================================*/
        float corr_l = pid_compute(&s_pid_left,  target_rpm_l,  rpm_l, dt);
        float corr_r = pid_compute(&s_pid_right, target_rpm_r, rpm_r, dt);

        /*================================================================
         * 7. 驱动电机
         *===============================================================*/
        set_left_motors(rpm_to_speed(target_rpm_l  + corr_l));
        set_right_motors(rpm_to_speed(target_rpm_r + corr_r));

        /*================================================================
         * 8. 判断到达 → 自动加载下一路径点
         *===============================================================*/
        if (target_reached(cmd, target_edges_l, target_edges_r,
                           turn_start_yaw, turn_target_rad, cur)) {
            set_all_motors(0);

            if (!waypoint_load_next()) {
                portENTER_CRITICAL(&s_lock);
                s_state = MOTION_DONE;
                portEXIT_CRITICAL(&s_lock);
                ESP_LOGI(TAG, "Motion complete (L=%ld/%ld R=%ld/%ld)",
                         (long)get_side_edges_pre(cur, MOTOR_LEFT_1, MOTOR_LEFT_2),
                         (long)target_edges_l,
                         (long)get_side_edges_pre(cur, MOTOR_RIGHT_1, MOTOR_RIGHT_2),
                         (long)target_edges_r);
            }
            continue;
        }
    }
}

/*============================================================================
 * 公开 API
 *============================================================================*/

void motion_init(void)
{
    pid_init(&s_pid_left,  MOTION_PID_KP, MOTION_PID_KI, MOTION_PID_KD, 30.0f);
    pid_init(&s_pid_right, MOTION_PID_KP, MOTION_PID_KI, MOTION_PID_KD, 30.0f);

    /* 里程计 */
    odometry_init();
    for (int i = 0; i < 4; i++) {
        s_last_enc[i] = encoder_get_count(i);
    }

    /* IMU 融合 */
#if MOTION_USE_IMU
    imu_fusion_init(&s_imu, MOTION_IMU_ALPHA);

    /* 初始化 MPU6050 */
    if (mpu6050_init()) {
        /* 上电自动校准陀螺零偏（机器人必须静止！） */
        if (mpu6050_calibrate(MOTION_IMU_CALIB_SAMPLES)) {
            s_imu_enabled = true;
            ESP_LOGI(TAG, "IMU fusion enabled (MPU6050, alpha=%.2f)",
                     (double)MOTION_IMU_ALPHA);
        } else {
            ESP_LOGW(TAG, "MPU6050 calibrate failed — IMU disabled");
        }
    } else {
        ESP_LOGW(TAG, "MPU6050 not detected — falling back to encoder-only");
    }
#endif

    /* 梯形曲线 */
    s_use_profile = (bool)MOTION_USE_TRAPEZOIDAL;

    xTaskCreate(motion_ctrl_task, "motion_ctrl", 4096, NULL, 5, NULL);

    s_state = MOTION_IDLE;

    ESP_LOGI(TAG, "Motion init OK (freq=%d Hz, CM/edge=%.4f, trapezoidal=%d, IMU=%d)",
             MOTION_CTRL_FREQ_HZ, (double)CM_PER_EDGE,
             (int)s_use_profile, (int)s_imu_enabled);
}

/*============================================================================
 * IMU 校准 (运行时手动校准)
 *============================================================================*/

/**
 * @brief 运行时重新校准 IMU 陀螺零偏
 *
 * 在 robot 保持静止时调用。直接委托给 MPU6050 驱动的校准函数。
 * 校准后 s_imu.gyro_bias_z 保持为 0（零偏已在驱动层扣除，避免双重校正）。
 */
void motion_imu_calibrate(void)
{
#if MOTION_USE_IMU
    if (!s_imu_enabled) {
        ESP_LOGW(TAG, "IMU not enabled, skip calibration");
        return;
    }
    ESP_LOGI(TAG, "Starting IMU calibration — KEEP STILL!");
    if (mpu6050_calibrate(MOTION_IMU_CALIB_SAMPLES)) {
        /* 驱动层已扣除零偏，imu_fusion 不需要再扣一次 */
        s_imu.gyro_bias_z = 0.0f;
        /* 重置融合航向为当前里程计航向，避免跳变 */
        float enc_yaw = odometry_get_heading_deg() * (float)M_PI / 180.0f;
        s_imu.yaw = enc_yaw;
        ESP_LOGI(TAG, "IMU calibration done, yaw reset to %.1f°",
                 (double)(enc_yaw * 180.0f / (float)M_PI));
    }
#endif
}

/*============================================================================
 * IMU 状态查询 (供 OLED 等外部模块使用)
 *============================================================================*/

float motion_get_imu_yaw(void)
{
#if MOTION_USE_IMU
    if (s_imu_enabled) {
        return imu_fusion_get_yaw(&s_imu);
    }
#endif
    /* 回退: 编码器航向 */
    return odometry_get_heading_deg() * (float)M_PI / 180.0f;
}

bool motion_is_imu_enabled(void)
{
#if MOTION_USE_IMU
    return s_imu_enabled;
#else
    return false;
#endif
}

/*============================================================================
 * 内部：不阻塞的运动启停函数（供 waypoint 引擎调用）
 *============================================================================*/

void motion_straight_nowait(float distance_cm, int16_t speed)
{
    if (speed <= 0 || speed > 100) speed = 50;

    int32_t edges = (int32_t)(distance_cm / CM_PER_EDGE);
    float target_rpm = speed_to_rpm(speed);
    if (distance_cm < 0) target_rpm = -target_rpm;

    portENTER_CRITICAL(&s_lock);
    s_cmd                = CMD_STRAIGHT;
    s_target_edges_left  = edges;
    s_target_edges_right = edges;
    s_target_rpm_left    = target_rpm;
    s_target_rpm_right   = target_rpm;
    snapshot_start_counts();
    pid_reset(&s_pid_left);
    pid_reset(&s_pid_right);

    /* 初始化梯形速度曲线 */
    init_speed_profiles((float)edges, (float)edges, target_rpm, target_rpm);

    set_left_motors(rpm_to_speed(target_rpm));
    set_right_motors(rpm_to_speed(target_rpm));

    s_state = MOTION_RUNNING;
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGI(TAG, "Straight: %.1f cm, rpm=%.0f", (double)distance_cm, (double)target_rpm);
}

void motion_turn_nowait(float angle_deg, int16_t speed)
{
    if (speed <= 0 || speed > 100) speed = 50;

    /* 单侧轮弧长 = π × 轮距 × |角度| / 360 */
    float   arc_cm = (float)(M_PI * WHEEL_BASE_CM * fabsf(angle_deg) / 360.0);
    int32_t edges  = (int32_t)(arc_cm / CM_PER_EDGE);
    float   rpm    = speed_to_rpm(speed);

    portENTER_CRITICAL(&s_lock);
    s_cmd = CMD_TURN;
    if (angle_deg >= 0) {
        /* 左转：左后退，右前进 */
        s_target_edges_left  = -edges;
        s_target_edges_right =  edges;
        s_target_rpm_left    = -rpm;
        s_target_rpm_right   =  rpm;
    } else {
        /* 右转：左前进，右后退 */
        s_target_edges_left  =  edges;
        s_target_edges_right = -edges;
        s_target_rpm_left    =  rpm;
        s_target_rpm_right   = -rpm;
    }
    snapshot_start_counts();
    pid_reset(&s_pid_left);
    pid_reset(&s_pid_right);

    /* 梯形曲线：左右两侧距离相同 */
    init_speed_profiles((float)edges, (float)edges,
                        s_target_rpm_left, s_target_rpm_right);

    /* IMU 增强：记录起始航向，用于融合判定 */
    if (s_imu_enabled) {
        s_turn_start_yaw  = imu_fusion_get_yaw(&s_imu);
        s_turn_target_rad = angle_deg * (float)M_PI / 180.0f;
    }

    set_left_motors(rpm_to_speed(s_target_rpm_left));
    set_right_motors(rpm_to_speed(s_target_rpm_right));

    s_state = MOTION_RUNNING;
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGI(TAG, "Turn: %.1f deg, rpm=%.0f%s",
             (double)angle_deg, (double)rpm,
             s_imu_enabled ? " [IMU]" : "");
}

void motion_arc_nowait(float radius_cm, float angle_deg, int16_t speed)
{
    if (speed <= 0 || speed > 100) speed = 50;
    if (fabsf(radius_cm) < 0.01f) {
        motion_turn_nowait(angle_deg, speed);
        return;
    }

    /*--------------------------------------------------------------------
     * 差速运动学：
     *   θ = angle_deg × π/180  (rad, 带符号)
     *   v_left  = v × (1 − half_base / R)
     *   v_right = v × (1 + half_base / R)
     *
     *   arc_left  = θ × (R − half_base)   (cm, 带符号)
     *   arc_right = θ × (R + half_base)
     *
     * 若 |R| < half_base，内侧轮反转，公式自然覆盖。
     *--------------------------------------------------------------------*/
    float R = radius_cm;
    float theta_rad = angle_deg * (float)M_PI / 180.0f;

    float v        = (float)speed;
    float v_left   = v * (1.0f - HALF_BASE / R);
    float v_right  = v * (1.0f + HALF_BASE / R);

    float arc_left  = theta_rad * (R - HALF_BASE);
    float arc_right = theta_rad * (R + HALF_BASE);

    portENTER_CRITICAL(&s_lock);
    s_cmd = CMD_ARC;
    s_target_rpm_left    = speed_to_rpm(v_left);
    s_target_rpm_right   = speed_to_rpm(v_right);
    s_target_edges_left  = (int32_t)(arc_left  / CM_PER_EDGE);
    s_target_edges_right = (int32_t)(arc_right / CM_PER_EDGE);

    snapshot_start_counts();
    pid_reset(&s_pid_left);
    pid_reset(&s_pid_right);

    /* 梯形曲线：两侧行程不同 */
    init_speed_profiles((float)s_target_edges_left,
                        (float)s_target_edges_right,
                        s_target_rpm_left, s_target_rpm_right);

    set_left_motors(rpm_to_speed(s_target_rpm_left));
    set_right_motors(rpm_to_speed(s_target_rpm_right));

    s_state = MOTION_RUNNING;
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGI(TAG, "Arc: R=%.1f angle=%.1f° vL=%.1f vR=%.1f",
             (double)radius_cm, (double)angle_deg,
             (double)v_left, (double)v_right);
}

/*============================================================================
 * 公开：阻塞式 API（等待上一运动完成后启动新的）
 *============================================================================*/

void motion_straight(float distance_cm, int16_t speed)
{
    motion_wait();
    motion_straight_nowait(distance_cm, speed);
}

void motion_turn(float angle_deg, int16_t speed)
{
    motion_wait();
    motion_turn_nowait(angle_deg, speed);
}

void motion_arc(float radius_cm, float angle_deg, int16_t speed)
{
    motion_wait();
    motion_arc_nowait(radius_cm, angle_deg, speed);
}

/*============================================================================
 * 紧急停止 / 状态查询
 *============================================================================*/

void motion_stop(void)
{
    portENTER_CRITICAL(&s_lock);
    set_all_motors(0);
    s_state = MOTION_STOPPED;
    s_cmd   = CMD_NONE;
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGI(TAG, "Emergency stop");
}

bool motion_is_done(void)
{
    portENTER_CRITICAL(&s_lock);
    bool done = (s_state != MOTION_RUNNING);
    portEXIT_CRITICAL(&s_lock);
    return done;
}

void motion_wait(void)
{
    while (1) {
        portENTER_CRITICAL(&s_lock);
        bool running = (s_state == MOTION_RUNNING);
        portEXIT_CRITICAL(&s_lock);
        if (!running) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

motion_state_t motion_get_state(void)
{
    portENTER_CRITICAL(&s_lock);
    motion_state_t state = s_state;
    portEXIT_CRITICAL(&s_lock);
    return state;
}
