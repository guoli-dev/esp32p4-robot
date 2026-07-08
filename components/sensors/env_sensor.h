#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @file env_sensor.h — 环境数据统一采集
 *
 * 整合 SHT31 + MQ-135 + MQ-136, 每 2 秒采集一轮,
 * 通过 FreeRTOS Queue 广播给其他任务。
 *
 * 化工厂三件套:
 *   SHT31  → 温湿度 (高温高湿 = 化学品不稳定风险)
 *   MQ-135 → 综合有害气体 (NH3/苯/NOx/烟雾)
 *   MQ-136 → 硫化氢 H₂S (污水池/反应釜泄漏预警)
 */

/* ── 统一环境数据 ─────────────────────────────────── */

typedef struct {
    float    temp_c;          /**< 温度 °C */
    float    humidity_pct;    /**< 相对湿度 % */
    uint16_t mq135_raw;       /**< MQ-135 ADC 原始值 (0-4095) */
    uint8_t  mq135_hazard;    /**< MQ-135 综合有害气体危险指数 (0-100) */
    uint16_t mq136_raw;       /**< MQ-136 ADC 原始值 (0-4095) */
    uint8_t  mq136_hazard;    /**< MQ-136 H₂S 危险指数 (0-100) */
    uint8_t  air_quality;     /**< 综合评分 0-100 (100=最优) */
} env_data_t;

/* ── 空气质量等级 ─────────────────────────────────── */

typedef enum {
    AIR_GOOD       = 0,       /**< 优   75-100 */
    AIR_MODERATE   = 1,       /**< 良   50-74  */
    AIR_UNHEALTHY  = 2,       /**< 差   25-49  */
    AIR_HAZARDOUS  = 3,       /**< 危险  0-24  */
} air_level_t;

/* ── API ──────────────────────────────────────────── */

/**
 * @brief 启动环境传感器采集任务
 *
 * 先后初始化 MQ136, MQ135, SHT30。
 * 采集任务每 2 秒运行一次，结果通过 Queue 广播。
 *
 * @return true=至少一个传感器初始化成功
 *
 * @note 调用前需确保 I2C_NUM_0 已由 OLED 初始化
 */
bool env_sensor_init(void);

/**
 * @brief 获取最新的环境数据（非阻塞）
 *
 * @param buf 输出缓冲区
 * @param timeout_ms 等待时间(ms)，0=立即返回
 * @return true=成功获取, false=超时/未初始化
 */
bool env_sensor_get(env_data_t *buf, uint32_t timeout_ms);

/**
 * @brief 直接读取最新的环境数据（无拷贝, 线程安全）
 *
 * 适用于状态显示等对实时性要求不高、不能阻塞的场景。
 * 返回的指针在下次采集前有效。
 *
 * @return 最新数据指针, NULL=还未采集到数据
 */
const env_data_t *env_sensor_peek(void);

/**
 * @brief 根据 MQ-135 + MQ-136 综合计算空气质量等级
 */
air_level_t env_air_level(const env_data_t *d);
