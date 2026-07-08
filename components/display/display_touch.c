/**
 * @file display_touch.c — GT911 电容触摸驱动 (7寸屏, MIPI DSI U4 FPC)
 *
 * 基于官方 BSP bsp_touch_new() + bsp_display_indev_init() 实现.
 * 触摸 I2C 与摄像头 SCCB 共享 GPIO7/8 上的 I2C master bus.
 *
 * 注意: RST/INT 均为 NC (通过 U4 FPC 与 LCD 共用复位),
 *       触摸工作在轮询模式, 无需中断引脚.
 */

#include "display_touch.h"
#include "display_lcd.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_lcd_touch_gt911.h"
#include "esp_lvgl_port.h"

#define TAG "touch"

/* ── Static handles ─────────────────────────────────── */

static esp_lcd_touch_handle_t    s_tp        = NULL;
static esp_lcd_panel_io_handle_t s_tp_io     = NULL;
static bool                      s_ok        = false;

/* ═══════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════ */

bool display_touch_init(void)
{
    if (s_ok) return true;

    /* 获取 I2C master bus handle (由 display_lcd_init 创建, GPIO7/8) */
    i2c_master_bus_handle_t i2c_bus = display_get_i2c_handle();
    if (!i2c_bus) {
        ESP_LOGE(TAG, "I2C bus not ready — call display_lcd_init() first");
        return false;
    }

    /* 触摸配置 — 与官方 BSP 一致 */
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = -1,         /* NC — 通过 U4 FPC 与 LCD RST 共用 */
        .int_gpio_num = -1,         /* NC — 轮询模式, 无中断 */
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy  = 0,
            .mirror_x = 0,          /* 与显示旋转一致 (mirror_x=false) */
            .mirror_y = 0,          /* 与显示旋转一致 (mirror_y=false) */
        },
    };

    /* 创建 I2C IO 接口 (GT911 地址 0x14, 400kHz) */
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    tp_io_config.scl_speed_hz = 400000;

    esp_err_t ret = esp_lcd_new_panel_io_i2c(i2c_bus, &tp_io_config, &s_tp_io);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TP I2C IO failed: %s", esp_err_to_name(ret));
        return false;
    }

    /* 创建 GT911 触摸实例 */
    ret = esp_lcd_touch_new_i2c_gt911(s_tp_io, &tp_cfg, &s_tp);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GT911 init failed: %s", esp_err_to_name(ret));
        esp_lcd_panel_io_del(s_tp_io);
        s_tp_io = NULL;
        return false;
    }

    /* 注册为 LVGL 输入设备 */
    lv_display_t *disp = display_lcd_get_disp();
    if (!disp) {
        ESP_LOGE(TAG, "LVGL display not ready");
        esp_lcd_touch_del(s_tp);
        esp_lcd_panel_io_del(s_tp_io);
        s_tp = NULL; s_tp_io = NULL;
        return false;
    }

    const lvgl_port_touch_cfg_t lvgl_touch_cfg = {
        .disp = disp,
        .handle = s_tp,
    };
    if (!lvgl_port_add_touch(&lvgl_touch_cfg)) {
        ESP_LOGE(TAG, "LVGL add touch failed");
        esp_lcd_touch_del(s_tp);
        esp_lcd_panel_io_del(s_tp_io);
        s_tp = NULL; s_tp_io = NULL;
        return false;
    }

    s_ok = true;
    ESP_LOGI(TAG, "GT911 touch OK (@0x14, I2C GPIO7/8)");
    return true;
}
