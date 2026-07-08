#pragma once

/**
 * @file display_touch.h
 * @brief GT911 电容触摸驱动 (7寸屏配套)
 *
 * 硬件连接 (通过 U4 FPC 15pin):
 *   触摸 I2C   — 与摄像头 SCCB 共享 GPIO7/8 (I2C master bus)
 *   触摸 RST   — NC, 与 LCD RST 共用 (由面板复位管理)
 *   触摸 INT   — NC, 轮询模式
 *   地址: 0x14 (7-bit)
 *
 * 必须先调用 display_lcd_init() 创建 I2C master bus, 再调用此驱动.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 GT911 触摸 + 注册 LVGL 输入设备
 *
 * 从 display_lcd_init() 创建的 I2C master bus 上初始化 GT911.
 * 触摸作为 LVGL input device 注册, 之后 LVGL 所有控件自动支持触控.
 *
 * @return true=触摸初始化成功
 */
bool display_touch_init(void);

#ifdef __cplusplus
}
#endif
