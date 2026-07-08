/**
 * @file sht30.c — SHT30 温湿度传感器驱动 (I2C)
 *
 * I2C 地址 0x44, 单次测量模式 (repeatability: high)
 * 与 OLED(0x3C) 共用 I2C_NUM_0 (MPU6050 在 I2C_NUM_1 GPIO54/53)
 */

#include "sht30.h"
#include "driver/i2c_master.h"
#include "display_lcd.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#define TAG       "sht30"
#define SHT30_ADDR 0x44

static i2c_master_dev_handle_t s_sht30_dev = NULL;

/* ── CRC8 (多项式 0x31, 初始值 0xFF) ─────────────── */

static bool crc8_check(const uint8_t *data, int len, uint8_t crc)
{
    uint8_t calc = 0xFF;
    for (int i = 0; i < len; i++) {
        calc ^= data[i];
        for (int b = 0; b < 8; b++) {
            calc = (calc & 0x80) ? (uint8_t)((calc << 1) ^ 0x31) : (uint8_t)(calc << 1);
        }
    }
    return calc == crc;
}

/* ── Public ────────────────────────────────────────── */

bool sht30_read(float *temp_c, float *humidity_pct)
{
    /* 懒初始化：首次调用时获取 I2C 总线句柄并注册设备 */
    if (s_sht30_dev == NULL) {
        i2c_master_bus_handle_t bus = display_get_i2c0_handle();
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = SHT30_ADDR,
            .scl_speed_hz = 400000,
        };
        esp_err_t r = i2c_master_bus_add_device(bus, &dev_cfg, &s_sht30_dev);
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "add device err: %s", esp_err_to_name(r));
            return false;
        }
    }

    /* 发送单次测量命令 (高重复性, clock stretching enabled) */
    uint8_t cmd[2] = { 0x2C, 0x06 };
    esp_err_t r = i2c_master_transmit(s_sht30_dev, cmd, 2, 20);
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "I2C write err: %s", esp_err_to_name(r));
        return false;
    }

    /* SHT30 典型测量时间: high repeatability ≈ 15ms */
    vTaskDelay(pdMS_TO_TICKS(20));

    /* 读取 6 字节: temp[2] crc hum[2] crc */
    uint8_t buf[6] = {0};
    r = i2c_master_receive(s_sht30_dev, buf, 6, 30);
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "I2C read err: %s", esp_err_to_name(r));
        return false;
    }

    /* 校验 */
    if (!crc8_check(buf, 2, buf[2]) || !crc8_check(buf + 3, 2, buf[5])) {
        ESP_LOGW(TAG, "CRC fail");
        return false;
    }

    /* 计算 */
    uint16_t raw_t = ((uint16_t)buf[0] << 8) | buf[1];
    uint16_t raw_h = ((uint16_t)buf[3] << 8) | buf[4];

    if (temp_c)       *temp_c       = -45.0f + 175.0f * ((float)raw_t / 65535.0f);
    if (humidity_pct) *humidity_pct = 100.0f * ((float)raw_h / 65535.0f);

    return true;
}
