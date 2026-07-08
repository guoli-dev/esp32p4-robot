#pragma once

/**
 * @file display_lcd.h
 * @brief 7寸 1024×600 MIPI DSI 显示屏 + I2C master bus (GPIO7/8)
 *
 * 接线 (U4 FPC, 15P 1.0mm):
 *   MIPI DSI 2-lane   — 芯片固定引脚 (U4)
 *   LCD_BL  (GPIO26)   — J3-33, 背光 PWM (5kHz)
 *   LCD_RST (GPIO27)   — J3-31, 复位
 *   TP_SDA  (GPIO7)    — J4-21 + U4(触摸), I2C 共享总线
 *   TP_SCL  (GPIO8)    — J4-22 + U4(触摸), I2C 共享总线
 *   LDO_VO3            — 芯片内置, DSI PHY 电源 (2.5V)
 *   DC_5V              — J3-1/3, 屏幕供电
 *   GND                — J3-6/8
 *
 * I2C 总线说明:
 *   本组件初始化 MIPI DSI 显示屏的同时, 在 GPIO7/8 上创建 I2C master bus.
 *   该总线由 触摸(GT911 @0x14) 和 摄像头 SCCB (OV5647 @0x3C) 共享.
 *   外部可通过 display_get_i2c_handle() 获取总线句柄.
 */

#include <stdint.h>
#include <stdbool.h>
#include "esp_lcd_types.h"
#include "driver/i2c_master.h"
#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════
 * Pin definitions
 * ═══════════════════════════════════════════════════════════════ */

#define LCD_GPIO_BL          26      /* J3-33, 背光 PWM */
#define LCD_GPIO_RST         27      /* J3-31, 复位 */

/* 共享 I2C 总线引脚 (触摸 GT911 + 摄像头 SCCB) */
#define LCD_I2C_SDA          7       /* GPIO7  (J4-21 CAM_SDA / U4 TP_SDA) */
#define LCD_I2C_SCL          8       /* GPIO8  (J4-22 CAM_SCL / U4 TP_SCL) */

/* ═══════════════════════════════════════════════════════════════
 * Display parameters
 * ═══════════════════════════════════════════════════════════════ */

#define LCD_H_RES            1024
#define LCD_V_RES            600
#define LCD_MIPI_LANE_NUM    2
#define LCD_MIPI_LANE_BITRATE_MBPS  1000
#define LCD_DSI_PHY_LDO_CHAN        3   /* LDO_VO3 */
#define LCD_DSI_PHY_LDO_MV          2500

/* ═══════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════ */

/**
 * @brief Initialize LCD display + I2C master bus + LVGL
 *
 * 按顺序完成:
 *   1. 创建 I2C master bus (GPIO7/8) — 供触摸 + 摄像头 SCCB 共享
 *   2. DSI PHY 上电 (LDO_VO3, 2.5V)
 *   3. 创建 MIPI DSI 总线 (2-lane, 1Gbps)
 *   4. 创建 EK79007 面板
 *   5. 初始化背光 PWM (LEDC, 5kHz)
 *   6. 初始化 LVGL port
 *   7. 向 LVGL 添加显示屏
 *
 * @return Pointer to LVGL display (NULL on failure)
 */
lv_display_t *display_lcd_init(void);

/**
 * @brief 获取共享 I2C master bus 句柄 (GPIO7/8, 触摸+摄像头)
 * @return I2C master bus handle, NULL=未初始化
 */
i2c_master_bus_handle_t display_get_i2c_handle(void);

/**
 * @brief 获取 I2C_NUM_0 总线句柄 (GPIO41/42, OLED+SHT30)
 * @return I2C master bus handle, NULL=未初始化
 */
i2c_master_bus_handle_t display_get_i2c0_handle(void);

/**
 * @brief 获取 I2C_NUM_1 总线句柄 (GPIO53/54, MPU6050+VL53L1X)
 * @return I2C master bus handle, NULL=未初始化
 */
i2c_master_bus_handle_t display_get_i2c1_handle(void);

/* ═══════════════════════════════════════════════════════════════
 * Dashboard touch button callbacks
 * ═══════════════════════════════════════════════════════════════ */

typedef struct {
    void (*on_speed)(void);
    void (*on_vision)(void);
    void (*on_patrol)(void);
    void (*on_estop)(void);
    void (*on_voice)(void);
} dashboard_btn_callbacks_t;

/**
 * @brief Register touch button callbacks for the dashboard.
 * Must be called before display_lcd_init() or at least before the dashboard is created.
 * Pass NULL for callbacks that are not needed.
 */
void display_lcd_register_btn_callbacks(const dashboard_btn_callbacks_t *cbs);

/* ═══════════════════════════════════════════════════════════════
 * Dashboard data & API
 * ═══════════════════════════════════════════════════════════════ */

/** Data snapshot for dashboard update */
typedef struct {
    const char *speed_label;
    int speed_pct;
    const char *vision_mode;
    bool vision_running;
    bool patrol_active;
    int patrol_idx;
    bool c5_ok;
    bool cam_live;
    int32_t enc[4];
    float temp_c;
    float humidity;
    int air_quality;
    uint16_t mq135_raw;
    uint8_t mq135_hazard;
    uint16_t mq136_raw;
    uint8_t mq136_hazard;
} dashboard_data_t;

/**
 * @brief Create the full dashboard UI on the active screen.
 * Call once after display_lcd_init() succeeds, while holding the LVGL lock.
 */
void display_lcd_dashboard_init(void);

/**
 * @brief Update all dashboard labels with fresh data.
 * Call periodically (e.g. every 500ms) while holding the LVGL lock.
 * @param data  Snapshot of current robot state
 */
void display_lcd_dashboard_update(const dashboard_data_t *data);

/**
 * @brief Show/hide camera preview image in the dashboard.
 */
void display_lcd_dashboard_show_cam(bool show);

/**
 * @brief Set camera preview image source (call once after init).
 * @param dsc  Pointer to lv_image_dsc_t with camera frame data
 */
void display_lcd_dashboard_set_cam_src(const void *dsc);

/**
 * @brief Update camera preview (invalidate after new frame).
 * @param fb  Pointer to lv_image_dsc_t with camera frame data
 * @param w   Image width
 * @param h   Image height
 */
void display_lcd_dashboard_update_cam(const void *fb, int w, int h);

/**
 * @brief Set backlight brightness
 * @param pct 0-100 (0=off, 100=max)
 */
void display_lcd_brightness_set(int pct);

/**
 * @brief Lock LVGL (call before any LVGL API from non-LVGL task)
 * @param timeout_ms Timeout (portMAX_DELAY = block forever)
 * @return true if lock acquired
 */
bool display_lcd_lock(uint32_t timeout_ms);

/**
 * @brief Unlock LVGL
 */
void display_lcd_unlock(void);

/**
 * @brief Get LVGL display handle
 */
lv_display_t *display_lcd_get_disp(void);

#ifdef __cplusplus
}
#endif
