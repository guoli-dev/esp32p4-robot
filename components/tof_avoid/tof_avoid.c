/**
 * @file tof_avoid.c — VL53L1X ToF 避障驱动 (单传感器)
 *
 * 1 颗 VL53L1X, I2C1 默认地址 0x29, 连续测距模式。
 */

#include "tof_avoid.h"
#include "driver/i2c_master.h"
#include "display_lcd.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

#define TAG "tof_avoid"

/* ── I2C 配置 ─────────────────────────────────────── */

#define I2C_TIMEOUT_MS       20

/* ── VL53L1X 寄存器地址 (16-bit) ──────────────────── */

#define VL53_ADDR                 0x29
#define VL53_REG_MODEL_ID         0x010F
#define VL53_REG_RESULT_RANGE_STATUS 0x0089
#define VL53_REG_RESULT_RANGE_MM  0x0096
#define VL53_REG_SYSTEM_START     0x0087

/* ── 内部状态 ─────────────────────────────────────── */

static tof_result_t s_latest;
static bool         s_has_data = false;
static i2c_master_dev_handle_t s_tof_dev = NULL;

static portMUX_TYPE s_spinlock = portMUX_INITIALIZER_UNLOCKED;

/* ── I2C 读写助手 ─────────────────────────────────── */

static int vl53_write16(uint16_t reg, uint16_t val)
{
    uint8_t buf[4] = {
        (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF),
        (uint8_t)(val >> 8), (uint8_t)(val & 0xFF)
    };
    return i2c_master_transmit(s_tof_dev, buf, 4, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static int vl53_write8(uint16_t reg, uint8_t val)
{
    uint8_t buf[3] = {
        (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), val
    };
    return i2c_master_transmit(s_tof_dev, buf, 3, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static int vl53_read16(uint16_t reg, uint16_t *val)
{
    uint8_t wbuf[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
    uint8_t rbuf[2] = {0};
    esp_err_t r = i2c_master_transmit_receive(s_tof_dev, wbuf, 2, rbuf, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (r == ESP_OK && val) *val = ((uint16_t)rbuf[0] << 8) | rbuf[1];
    return (r == ESP_OK) ? 0 : -1;
}

static int vl53_read8(uint16_t reg, uint8_t *val)
{
    uint8_t wbuf[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
    uint8_t rbuf[1] = {0};
    esp_err_t r = i2c_master_transmit_receive(s_tof_dev, wbuf, 2, rbuf, 1, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (r == ESP_OK && val) *val = rbuf[0];
    return (r == ESP_OK) ? 0 : -1;
}

/* ── 障碍等级判定 ─────────────────────────────────── */

static obstacle_level_t calc_level(uint16_t mm)
{
    if (mm == 0) return OBSTACLE_NONE;   /* 无效测量 → 安全 */
    uint16_t cm = mm / 10;
    if (cm <= TOF_STOP_CM)  return OBSTACLE_STOP;
    if (cm <= TOF_SLOW_CM)  return OBSTACLE_SLOW;
    if (cm <= TOF_WARN_CM)  return OBSTACLE_WARN;
    return OBSTACLE_NONE;
}

/* ── 采集任务 ─────────────────────────────────────── */

static void tof_collect_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "collect task start (Core %d)", xPortGetCoreID());

    while (1) {
        uint8_t  status = 0;
        uint16_t range  = 0;

        if (vl53_read8(VL53_REG_RESULT_RANGE_STATUS, &status) == 0 &&
            vl53_read16(VL53_REG_RESULT_RANGE_MM, &range) == 0) {

            tof_result_t r;
            r.range_mm = range;
            r.level    = calc_level(range);

            portENTER_CRITICAL(&s_spinlock);
            s_latest = r;
            s_has_data = true;
            portEXIT_CRITICAL(&s_spinlock);

            if (r.level >= OBSTACLE_WARN) {
                const char *lvl[] = {"NONE","WARN","SLOW","STOP"};
                ESP_LOGI(TAG, "range=%umm level=%s", (unsigned)range, lvl[r.level]);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));  /* 10Hz */
    }
}

/* ── Public API ────────────────────────────────────── */

bool tof_avoid_init(void)
{
    /* XSHUT GPIO */
    gpio_reset_pin(TOF_XSHUT_GPIO);
    gpio_set_direction(TOF_XSHUT_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(TOF_XSHUT_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    gpio_set_level(TOF_XSHUT_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(5));  /* VL53L1X boot ~4ms */

    /* I2C1 bus handle from display (shared with MPU6050) */
    i2c_master_bus_handle_t bus = display_get_i2c1_handle();
    if (!bus) {
        ESP_LOGE(TAG, "I2C1 bus not ready");
        return false;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VL53_ADDR,
        .scl_speed_hz = 400000,
    };

    esp_err_t r = i2c_master_bus_add_device(bus, &dev_cfg, &s_tof_dev);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add TOF device");
        return false;
    }

    /* 验证芯片: Model ID 应为 0xEACC */
    uint16_t model = 0;
    if (vl53_read16(VL53_REG_MODEL_ID, &model) != 0 || model != 0xEACC) {
        ESP_LOGE(TAG, "VL53L1X not found (model=0x%04X)", (unsigned)model);
        return false;
    }

    /* VHV 校准 */
    vl53_write16(0x000B, 0x0300);
    vl53_write16(0x0030, 0x0000);
    vl53_write16(0x0031, 0x0000);

    /* 长距离模式 */
    vl53_write16(0x0036, 0x0002);

    /* 测量周期 50ms (20Hz) */
    vl53_write16(0x0062, 50);

    /* 启动连续测距 */
    vl53_write8(VL53_REG_SYSTEM_START, 0x40);

    xTaskCreatePinnedToCore(tof_collect_task, "tof_collect", 2048,
                            NULL, 3, NULL, 1);

    ESP_LOGI(TAG, "init OK — VL53L1X @ 0x%02X, GPIO%d", VL53_ADDR, TOF_XSHUT_GPIO);
    return true;
}

bool tof_avoid_peek(tof_result_t *out)
{
    if (!s_has_data || !out) return false;
    portENTER_CRITICAL(&s_spinlock);
    *out = s_latest;
    portEXIT_CRITICAL(&s_spinlock);
    return true;
}

bool tof_avoid_safe_to_move(float distance_cm)
{
    if (!s_has_data) return true;

    portENTER_CRITICAL(&s_spinlock);
    uint16_t mm = s_latest.range_mm;
    portEXIT_CRITICAL(&s_spinlock);

    /* 后退不检查 */
    if (distance_cm < 0) return true;

    if (mm > 0 && mm < ((uint16_t)TOF_STOP_CM * 10)) {
        ESP_LOGW(TAG, "BLOCKED: %umm, cannot move forward %.0fcm",
                 (unsigned)mm, (double)distance_cm);
        return false;
    }
    return true;
}

int tof_avoid_clamp_speed(int requested_pct)
{
    if (!s_has_data) return requested_pct;

    portENTER_CRITICAL(&s_spinlock);
    obstacle_level_t level = s_latest.level;
    portEXIT_CRITICAL(&s_spinlock);

    switch (level) {
    case OBSTACLE_STOP: return 0;
    case OBSTACLE_SLOW: return (requested_pct > TOF_SLOW_SPEED_PCT)
                                ? TOF_SLOW_SPEED_PCT : requested_pct;
    default:            return requested_pct;
    }
}
