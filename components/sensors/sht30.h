#pragma once

#include <stdbool.h>

/**
 * @file sht30.h — SHT30 温湿度传感器驱动 (I2C)
 *
 * 地址 0x44, 与 OLED(0x3C) 共用 I2C_NUM_0 (GPIO41/42)
 * 注意: MPU6050(0x68) 在独立总线 I2C_NUM_1 (GPIO54/53)
 */

/** @brief 读取一次温湿度
 * @param temp_c       输出温度 (°C)
 * @param humidity_pct 输出相对湿度 (%)
 * @return true=成功, false=传感器无响应/校验失败
 */
bool sht30_read(float *temp_c, float *humidity_pct);
