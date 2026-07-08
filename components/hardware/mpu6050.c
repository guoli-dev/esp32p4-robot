/**
 * @file mpu6050.c
 * @brief MPU6050 I2C 驱动实现
 *
 * 上电 → 唤醒 → 配置量程/DLPF → 校准零偏 → 持续读取。
 *
 * 陀螺数据链路:
 *   I2C 读 6 字节 → 拼 int16_t (raw) → 减零偏 → 乘 scale → rad/s
 *
 * 覆盖 motion_control.c 中的弱函数 imu_read_gyro()，使 IMU 融合模块
 * 透明地调用本驱动。
 */

#include "mpu6050.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "display_lcd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

#define TAG "mpu6050"

/*============================================================================
 * MPU6050 寄存器地址
 *============================================================================*/
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B
#define REG_TEMP_OUT_H      0x41
#define REG_GYRO_XOUT_H     0x43
#define REG_PWR_MGMT_1      0x6B
#define REG_WHO_AM_I        0x75

/* PWR_MGMT_1 寄存器位定义 (MPU6050 数据手册 §4.28) */
#define BIT_DEVICE_RESET    (1 << 7)   /* 写1复位全部寄存器，自清除 */
#define BIT_SLEEP           (1 << 6)   /* 1=睡眠模式 */
#define BIT_TEMP_DIS        (1 << 3)   /* 1=禁用温度传感器 */
#define CLKSEL_PLL_X        0x01       /* 时钟源: PLL with X-axis gyro reference */

/*============================================================================
 * 陀螺量程 → LSB/(°/s) 标度因子
 *============================================================================*/
static float s_gyro_scale = 0.0f;   /* 每 LSB 对应的 °/s */

static float gyro_fs_to_scale(mpu6050_gyro_fs_t fs)
{
    switch (fs) {
    case MPU6050_GYRO_FS_250:  return 131.0f;
    case MPU6050_GYRO_FS_500:  return 65.5f;
    case MPU6050_GYRO_FS_1000: return 32.8f;
    case MPU6050_GYRO_FS_2000: return 16.4f;
    default: return 65.5f;
    }
}

/*============================================================================
 * 陀螺零偏（由 mpu6050_calibrate 填充）
 *============================================================================*/
static float s_bias_gx = 0.0f;
static float s_bias_gy = 0.0f;
static float s_bias_gz = 0.0f;
static bool  s_calibrated = false;

static i2c_master_dev_handle_t s_mpu_dev = NULL;

/*============================================================================
 * I2C 读写辅助
 *============================================================================*/

static esp_err_t i2c_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_mpu_dev, buf, sizeof(buf), pdMS_TO_TICKS(10));
}

static esp_err_t i2c_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(s_mpu_dev, &reg, 1, data, len, pdMS_TO_TICKS(10));
}

/*============================================================================
 * 初始化
 *============================================================================*/

bool mpu6050_init(void)
{
    /* ── 1. 获取 I2C 总线句柄并添加设备 ── */
    i2c_master_bus_handle_t bus = display_get_i2c1_handle();
    if (!bus) return false;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_I2C_ADDR,
        .scl_speed_hz = 400000,
    };
    if (i2c_master_bus_add_device(bus, &dev_cfg, &s_mpu_dev) != ESP_OK)
        return false;

    /* ── 验证 MPU6050 是否存在 ── */
    uint8_t whoami = 0;
    esp_err_t ret = i2c_read_reg(REG_WHO_AM_I, &whoami, 1);
    if (ret != ESP_OK || whoami != 0x68) {
        ESP_LOGE(TAG, "WHO_AM_I check failed: ret=%s val=0x%02X (expected 0x68)",
                 esp_err_to_name(ret), (unsigned)whoami);
        return false;
    }
    ESP_LOGI(TAG, "MPU6050 detected (WHO_AM_I=0x%02X)", (unsigned)whoami);

    /* ── 3. 唤醒芯片 ──
     *    先复位 → 等待 → 关睡眠 → 选 PLL 时钟源 */
    i2c_write_reg(REG_PWR_MGMT_1, BIT_DEVICE_RESET);
    vTaskDelay(pdMS_TO_TICKS(100));

    i2c_write_reg(REG_PWR_MGMT_1, CLKSEL_PLL_X);   /* 唤醒 + PLL X 轴陀螺时钟 */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* ── 4. 配置采样率 ── */
    i2c_write_reg(REG_SMPLRT_DIV, MPU6050_SMPLRT_DIV);

    /* ── 5. 配置 DLPF ── */
    i2c_write_reg(REG_CONFIG, (uint8_t)MPU6050_DLPF_CFG);

    /* ── 6. 配置陀螺量程 ── */
    i2c_write_reg(REG_GYRO_CONFIG, (uint8_t)(MPU6050_GYRO_FS << 3));
    s_gyro_scale = gyro_fs_to_scale(MPU6050_GYRO_FS);

    /* ── 7. 配置加速度计量程 (±4g) ── */
    i2c_write_reg(REG_ACCEL_CONFIG, (uint8_t)(1 << 3));   /* ±4g */

    s_calibrated = false;
    ESP_LOGI(TAG, "Init OK: gyro FS=%d scale=%.1f LSB/°/s, DLPF=%d, I2C=%d kHz",
             (int)MPU6050_GYRO_FS, (double)s_gyro_scale,
             (int)MPU6050_DLPF_CFG, MPU6050_I2C_FREQ_HZ / 1000);

    return true;
}

/*============================================================================
 * 读取陀螺仪原始数据
 *============================================================================*/

static bool read_gyro_raw(int16_t *rx, int16_t *ry, int16_t *rz)
{
    uint8_t data[6];
    esp_err_t ret = i2c_read_reg(REG_GYRO_XOUT_H, data, 6);
    if (ret != ESP_OK) return false;

    *rx = (int16_t)((data[0] << 8) | data[1]);
    *ry = (int16_t)((data[2] << 8) | data[3]);
    *rz = (int16_t)((data[4] << 8) | data[5]);

    return true;
}

/*============================================================================
 * 校准
 *============================================================================*/

bool mpu6050_calibrate(int num_samples)
{
    if (num_samples <= 0) return false;

    ESP_LOGI(TAG, "Calibrating gyro bias (%d samples) — KEEP STILL!", num_samples);

    double sum_gx = 0.0, sum_gy = 0.0, sum_gz = 0.0;
    int valid = 0;

    for (int i = 0; i < num_samples; i++) {
        int16_t rx, ry, rz;
        if (!read_gyro_raw(&rx, &ry, &rz)) {
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }
        sum_gx += (double)rx;
        sum_gy += (double)ry;
        sum_gz += (double)rz;
        valid++;
        vTaskDelay(pdMS_TO_TICKS(2));  /* ~500 Hz 采样 */
    }

    if (valid == 0) {
        ESP_LOGE(TAG, "Calibration failed: no valid samples");
        return false;
    }

    /* 平均值 (LSB) */
    float avg_x = (float)(sum_gx / (double)valid);
    float avg_y = (float)(sum_gy / (double)valid);
    float avg_z = (float)(sum_gz / (double)valid);

    /* 转为 °/s */
    s_bias_gx = avg_x / s_gyro_scale;
    s_bias_gy = avg_y / s_gyro_scale;
    s_bias_gz = avg_z / s_gyro_scale;
    s_calibrated = true;

    ESP_LOGI(TAG, "Gyro bias (°/s): X=%.3f Y=%.3f Z=%.3f (from %d samples)",
             (double)s_bias_gx, (double)s_bias_gy, (double)s_bias_gz, valid);

    return true;
}

/*============================================================================
 * 公开读取 API
 *============================================================================*/

bool mpu6050_read_gyro(float *gx, float *gy, float *gz)
{
    int16_t rx, ry, rz;
    if (!read_gyro_raw(&rx, &ry, &rz)) return false;

    /* LSB → °/s → rad/s 转换 */
    const float deg2rad = (float)M_PI / 180.0f;

    *gx = ((float)rx / s_gyro_scale - s_bias_gx) * deg2rad;
    *gy = ((float)ry / s_gyro_scale - s_bias_gy) * deg2rad;
    *gz = ((float)rz / s_gyro_scale - s_bias_gz) * deg2rad;

    return true;
}

bool mpu6050_read_accel(float *ax, float *ay, float *az)
{
    uint8_t data[6];
    esp_err_t ret = i2c_read_reg(REG_ACCEL_XOUT_H, data, 6);
    if (ret != ESP_OK) return false;

    int16_t rx = (int16_t)((data[0] << 8) | data[1]);
    int16_t ry = (int16_t)((data[2] << 8) | data[3]);
    int16_t rz = (int16_t)((data[4] << 8) | data[5]);

    /* ±4g 量程 → 8192 LSB/g → 9.81 m/s² */
    const float scale = 8192.0f;
    const float g2ms2 = 9.80665f;

    *ax = (float)rx / scale * g2ms2;
    *ay = (float)ry / scale * g2ms2;
    *az = (float)rz / scale * g2ms2;

    return true;
}

/*============================================================================
 * 覆盖 motion_control.c 的弱函数 imu_read_gyro
 *
 * 当 MPU6050 驱动编译链接后，motion_control.c 通过 imu_read_gyro() 透明
 * 地获得陀螺数据，不再需要额外胶水代码。
 *============================================================================*/

bool imu_read_gyro(float *gx, float *gy, float *gz)
{
    return mpu6050_read_gyro(gx, gy, gz);
}
