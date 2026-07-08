#pragma once

/**
 * @file oled_display.h
 * @brief SSD1306 OLED 128×64 小车状态 + 手势显示
 *
 * I2C_NUM_0 (GPIO41/42), 与 SHT30(0x44) 共用, 地址 0x3C
 * 注意: MPU6050 在 I2C_NUM_1 (GPIO54/53), 为独立总线
 * 接线: VCC→3.3V  GND→GND  SDA→GPIO42  SCL→GPIO41
 */

#include <stdbool.h>

typedef enum {
    OLED_IDLE    = 0,
    OLED_RUNNING = 1,
    OLED_DONE    = 2,
    OLED_STOPPED = 3,
} oled_state_t;

/** @brief 初始化 OLED (I2C 总线须已由 MPU6050 初始化) */
bool oled_init(void);

/**
 * @brief Draw one text line on the OLED (0-3, 21 chars max)
 * @param row   0=top, 3=bottom
 * @param text  Null-terminated string (truncated to 21 chars)
 */
void oled_text_line(int row, const char *text);

/**
 * @brief Horizontal separator line at pixel Y
 */
void oled_hline(int y);

/**
 * @brief Clear framebuffer (call oled_flush() after to display)
 */
void oled_clear_buf(void);

/**
 * @brief Push framebuffer to display
 */
void oled_flush(void);

/** @brief Clear and flush */
void oled_clear(void);

/**
 * @brief Legacy: show full status page
 */
void oled_show(oled_state_t state, float rpm_l, float rpm_r,
               float heading, const char *gesture, const char *cmd);
