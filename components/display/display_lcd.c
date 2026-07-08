/**
 * @file display_lcd.c — 7寸 MIPI DSI 1024×600 显示屏驱动 (EK79007 + LVGL + I2C master bus)
 *
 * 基于官方 BSP wtdkp4c5_s1_board.c 简化.
 * 包含 I2C master bus 初始化, 供触摸 GT911 + 摄像头 SCCB 共享.
 * 无触摸无音频.
 */

#include "display_lcd.h"
#include <stdio.h>
#include "esp_log.h"
#include "esp_check.h"
#include "esp_ldo_regulator.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_ek79007.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"
#include "esp_lvgl_port.h"

#define TAG "display"

/* ── Static handles ─────────────────────────────────── */

static esp_lcd_dsi_bus_handle_t    s_dsi_bus   = NULL;
static esp_lcd_panel_io_handle_t   s_panel_io  = NULL;
static esp_lcd_panel_handle_t      s_panel     = NULL;
static lv_display_t               *s_disp      = NULL;
static esp_ldo_channel_handle_t    s_phy_pwr   = NULL;
static i2c_master_bus_handle_t     s_i2c_bus   = NULL;   /* GPIO7/8 — touch + camera */
static i2c_master_bus_handle_t     s_i2c0_bus  = NULL;   /* GPIO41/42 — OLED + SHT30 */
static i2c_master_bus_handle_t     s_i2c1_bus  = NULL;   /* GPIO53/54 — MPU6050 + VL53L1X */

/* ═══════════════════════════════════════════════════════════
 * I2C master bus (GPIO7/8) — 触摸 + 摄像头 SCCB 共享
 * 按照官方 BSP bsp_i2c_init() 实现
 * ═══════════════════════════════════════════════════════════ */

static esp_err_t i2c_bus_init(void)
{
    if (s_i2c_bus) return ESP_OK;  /* already created */

    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port   = -1,           /* -1 = auto-allocate */
        .sda_io_num = LCD_I2C_SDA,  /* GPIO7 (J4-21 CAM_SDA / U4 TP_SDA) */
        .scl_io_num = LCD_I2C_SCL,  /* GPIO8 (J4-22 CAM_SCL / U4 TP_SCL) */
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_i2c_bus),
                        TAG, "I2C master bus failed");

    ESP_LOGI(TAG, "I2C master bus on GPIO%d/GPIO%d (touch + camera SCCB)",
             LCD_I2C_SDA, LCD_I2C_SCL);
    return ESP_OK;
}

i2c_master_bus_handle_t display_get_i2c_handle(void)
{
    return s_i2c_bus;
}

/* ═══════════════════════════════════════════════════════════
 * I2C master bus (GPIO41/42) — I2C_NUM_0, OLED + SHT30
 * ═══════════════════════════════════════════════════════════ */

#define I2C0_SDA_GPIO  42
#define I2C0_SCL_GPIO  41

static esp_err_t i2c0_bus_init(void);
static esp_err_t i2c1_bus_init(void);

i2c_master_bus_handle_t display_get_i2c0_handle(void)
{
    if (!s_i2c0_bus) i2c0_bus_init();
    return s_i2c0_bus;
}

static esp_err_t i2c0_bus_init(void)
{
    if (s_i2c0_bus) return ESP_OK;
    i2c_master_bus_config_t cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port   = -1,
        .sda_io_num = I2C0_SDA_GPIO,
        .scl_io_num = I2C0_SCL_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&cfg, &s_i2c0_bus);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "I2C0 bus init skipped (no free controller)");
        return ESP_OK;
    }
    ESP_LOGI(TAG, "I2C0 bus on GPIO%d/GPIO%d (OLED + SHT30)",
             I2C0_SDA_GPIO, I2C0_SCL_GPIO);
    return ESP_OK;
}

/* ═══════════════════════════════════════════════════════════
 * I2C master bus (GPIO53/54) — I2C_NUM_1, MPU6050 + VL53L1X
 * ═══════════════════════════════════════════════════════════ */

#define I2C1_SDA_GPIO  54
#define I2C1_SCL_GPIO  53

i2c_master_bus_handle_t display_get_i2c1_handle(void)
{
    if (!s_i2c1_bus) i2c1_bus_init();
    return s_i2c1_bus;
}

static esp_err_t i2c1_bus_init(void)
{
    if (s_i2c1_bus) return ESP_OK;
    i2c_master_bus_config_t cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port   = -1,
        .sda_io_num = I2C1_SDA_GPIO,
        .scl_io_num = I2C1_SCL_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&cfg, &s_i2c1_bus);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "I2C1 bus init skipped (no free controller)");
        return ESP_OK;
    }
    ESP_LOGI(TAG, "I2C1 bus on GPIO%d/GPIO%d (MPU6050 + VL53L1X)",
             I2C1_SDA_GPIO, I2C1_SCL_GPIO);
    return ESP_OK;
}

/* ═══════════════════════════════════════════════════════════
 * MIPI DSI PHY power (LDO_VO3, 2.5V)
 * ═══════════════════════════════════════════════════════════ */

static esp_err_t enable_dsi_phy_power(void)
{
    if (s_phy_pwr) return ESP_OK;

    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id    = LCD_DSI_PHY_LDO_CHAN,
        .voltage_mv = LCD_DSI_PHY_LDO_MV,
    };
    ESP_RETURN_ON_ERROR(esp_ldo_acquire_channel(&ldo_cfg, &s_phy_pwr),
                        TAG, "DSI PHY LDO failed");
    ESP_LOGI(TAG, "DSI PHY power on (LDO_VO3, %dmV)", LCD_DSI_PHY_LDO_MV);
    return ESP_OK;
}

/* ═══════════════════════════════════════════════════════════
 * Backlight PWM (LEDC, 5kHz, GPIO26)
 * ═══════════════════════════════════════════════════════════ */

static esp_err_t backlight_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num       = LEDC_TIMER_1,
        .freq_hz         = 5000,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), TAG, "BL timer failed");

    ledc_channel_config_t ch = {
        .gpio_num   = LCD_GPIO_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_1,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch), TAG, "BL channel failed");
    return ESP_OK;
}

void display_lcd_brightness_set(int pct)
{
    if (pct > 100) pct = 100;
    if (pct < 0)   pct = 0;
    uint32_t duty = (uint32_t)(1023 * pct / 100);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

/* ═══════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════ */

lv_display_t *display_lcd_init(void)
{
    if (s_disp) return s_disp;

    ESP_LOGI(TAG, "init start...");

    /* 0. I2C master bus (GPIO7/8) — must be first for shared bus */
    if (i2c_bus_init() != ESP_OK) goto err;

    /* 1. DSI PHY power */
    if (enable_dsi_phy_power() != ESP_OK) goto err;

    /* 2. MIPI DSI bus: 2-lane, 1000 Mbps */
    esp_lcd_dsi_bus_config_t bus_config = {
        .bus_id             = 0,
        .num_data_lanes     = LCD_MIPI_LANE_NUM,
        .phy_clk_src        = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
        .lane_bit_rate_mbps = LCD_MIPI_LANE_BITRATE_MBPS,
    };
    if (esp_lcd_new_dsi_bus(&bus_config, &s_dsi_bus) != ESP_OK) {
        ESP_LOGE(TAG, "DSI bus failed");
        goto err;
    }

    /* 3. DBI control IO (8-bit cmd/param) */
    esp_lcd_dbi_io_config_t dbi_config = {
        .virtual_channel = 0,
        .lcd_cmd_bits    = 8,
        .lcd_param_bits  = 8,
    };
    if (esp_lcd_new_panel_io_dbi(s_dsi_bus, &dbi_config, &s_panel_io) != ESP_OK) {
        ESP_LOGE(TAG, "DBI IO failed");
        goto err;
    }

    /* 4. EK79007 panel */
    esp_lcd_dpi_panel_config_t dpi_config = EK79007_1024_600_PANEL_60HZ_CONFIG(
        LCD_COLOR_PIXEL_FORMAT_RGB565);
    dpi_config.num_fbs = 2;  /* LVGL port needs 2 frame buffers */

    ek79007_vendor_config_t vendor_config = {
        .mipi_config = {
            .dsi_bus    = s_dsi_bus,
            .dpi_config = &dpi_config,
        },
    };
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 16,
        .rgb_ele_order  = ESP_LCD_COLOR_SPACE_RGB,
        .reset_gpio_num = LCD_GPIO_RST,
        .vendor_config  = &vendor_config,
    };
    if (esp_lcd_new_panel_ek79007(s_panel_io, &panel_config, &s_panel) != ESP_OK) {
        ESP_LOGE(TAG, "EK79007 panel failed");
        goto err;
    }
    if (esp_lcd_panel_reset(s_panel) != ESP_OK ||
        esp_lcd_panel_init(s_panel) != ESP_OK) {
        ESP_LOGE(TAG, "panel reset/init failed");
        goto err;
    }
    ESP_LOGI(TAG, "panel OK (1024x600)");

    /* 6. Send DISPON via DBI after init */
    {
        esp_err_t r = esp_lcd_panel_disp_on_off(s_panel, true);
        if (r == ESP_ERR_NOT_SUPPORTED) {
            esp_lcd_panel_io_tx_param(s_panel_io, 0x29, NULL, 0);
            vTaskDelay(pdMS_TO_TICKS(50));
            ESP_LOGI(TAG, "DISPON (0x29) sent");
        }
    }

    /* 5. Backlight PWM */
    if (backlight_init() != ESP_OK) goto err;

    /* 6. LVGL port init */
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    if (lvgl_port_init(&lvgl_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "LVGL port init failed");
        goto err;
    }

    /* 7. Add display to LVGL */
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle     = s_panel_io,
        .panel_handle  = s_panel,
        .buffer_size   = LCD_H_RES * 50,
        .double_buffer = false,
        .hres          = LCD_H_RES,
        .vres          = LCD_V_RES,
        .monochrome    = false,
        .rotation = { .swap_xy = false, .mirror_x = false, .mirror_y = false },
        .flags = {
            .buff_dma   = true,
            .buff_spiram = true,
            .sw_rotate  = false,
            .direct_mode = true,
        },
    };
    const lvgl_port_display_dsi_cfg_t dpi_cfg = {
        .flags = { .avoid_tearing = true },
    };
    s_disp = lvgl_port_add_disp_dsi(&disp_cfg, &dpi_cfg);
    if (!s_disp) {
        ESP_LOGE(TAG, "LVGL add display failed");
        goto err;
    }

    /* 8. Turn on backlight (100% for max visibility) */
    display_lcd_brightness_set(100);

    /* 9. Fix display mirror via EK79007 MADCTL (0x36).
     *   Default is 0x01 (SHLR=1 = x mirrored).
     *   Call mirror(false,false) to clear both bits → normal scan. */
    esp_lcd_panel_mirror(s_panel, false, false);

    /* 10. Create full dashboard UI */
    display_lcd_lock(portMAX_DELAY);
    display_lcd_dashboard_init();
    display_lcd_unlock();

    ESP_LOGI(TAG, "init complete (I2C bus + 1024x600 LVGL)");
    return s_disp;

err:
    ESP_LOGE(TAG, "init FAILED — check LCD FPC connection and power");
    if (s_panel)    { esp_lcd_panel_del(s_panel); s_panel = NULL; }
    if (s_panel_io) { esp_lcd_panel_io_del(s_panel_io); s_panel_io = NULL; }
    if (s_dsi_bus)  { esp_lcd_del_dsi_bus(s_dsi_bus); s_dsi_bus = NULL; }
    return NULL;
}

bool display_lcd_lock(uint32_t timeout_ms)
{
    if (!s_disp) return false;  /* not initialized, don't block */
    return lvgl_port_lock(timeout_ms);
}

void display_lcd_unlock(void)
{
    if (!s_disp) return;  /* not initialized */
    lvgl_port_unlock();
}

/* ── Dashboard widget handles ─────────────────── */

static lv_obj_t *dw_title, *dw_bar, *dw_spd, *dw_vis, *dw_pat, *dw_com;
static lv_obj_t *dw_m[4], *dw_tof, *dw_env, *dw_gas, *dw_gcard;
static lv_obj_t *dw_cam_img = NULL;

static void dw_hide_all(bool hide)
{
    lv_obj_t *all[] = {dw_title, dw_bar, dw_spd, dw_vis, dw_pat, dw_com,
                       dw_tof, dw_env, dw_gas, dw_gcard,
                       dw_m[0], dw_m[1], dw_m[2], dw_m[3]};
    for (int i = 0; i < (int)(sizeof(all)/sizeof(all[0])); i++) {
        if (!all[i]) continue;
        if (hide) lv_obj_add_flag(all[i], LV_OBJ_FLAG_HIDDEN);
        else      lv_obj_clear_flag(all[i], LV_OBJ_FLAG_HIDDEN);
    }
}

void display_lcd_dashboard_show_cam(bool show)
{
    dw_hide_all(show);
    if (dw_cam_img) {
        if (show) lv_obj_clear_flag(dw_cam_img, LV_OBJ_FLAG_HIDDEN);
        else      lv_obj_add_flag(dw_cam_img, LV_OBJ_FLAG_HIDDEN);
    }
}

void display_lcd_dashboard_update_cam(const void *fb, int w, int h)
{
    if (!dw_cam_img || !fb) return;
    lv_obj_invalidate(dw_cam_img);
}

/* Auto-set source when first camera descriptor is provided */
void display_lcd_dashboard_set_cam_src(const void *dsc)
{
    if (!dw_cam_img || !dsc) return;
    lv_img_set_src(dw_cam_img, dsc);
}

/* ── Touch button event queue (defer from LVGL event handler) ── */

#define BTN_QUEUE_LEN   8
static QueueHandle_t   s_btn_queue = NULL;
static TaskHandle_t    s_btn_task  = NULL;
static dashboard_btn_callbacks_t s_btn_cbs = {0};

static void btn_task_fn(void *arg);

void display_lcd_register_btn_callbacks(const dashboard_btn_callbacks_t *cbs)
{
    if (cbs) s_btn_cbs = *cbs;
    if (!s_btn_queue) {
        s_btn_queue = xQueueCreate(BTN_QUEUE_LEN, sizeof(int));
        xTaskCreate(btn_task_fn, "dash_btn", 3072, NULL, 3, &s_btn_task);
        ESP_LOGI(TAG, "button task created");
    }
}

static void btn_task_fn(void *arg)
{
    (void)arg;
    int id;
    while (1) {
        if (xQueueReceive(s_btn_queue, &id, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "btn_task: processing %d", id);
            switch (id) {
            case 0: if (s_btn_cbs.on_speed)  s_btn_cbs.on_speed();  break;
            case 1: if (s_btn_cbs.on_vision) s_btn_cbs.on_vision(); break;
            case 2: if (s_btn_cbs.on_voice)  s_btn_cbs.on_voice();  break;
            case 3: if (s_btn_cbs.on_patrol) s_btn_cbs.on_patrol(); break;
            case 4: if (s_btn_cbs.on_estop)  s_btn_cbs.on_estop();  break;
            }
        }
    }
}

static void btn_evt_cb(lv_event_t *e)
{
    int id = (int)(intptr_t)lv_event_get_user_data(e);
    ESP_LOGI(TAG, "btn %d tapped — queued", id);
    if (s_btn_queue) {
        xQueueSend(s_btn_queue, &id, 0);
    }
}

void display_lcd_dashboard_init(void)
{
    lv_obj_t *scr = lv_disp_get_scr_act(s_disp);
    if (!scr) return;

    lv_obj_clean(scr);

    lv_obj_set_style_bg_color(scr, lv_color_hex(0x0d1117), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    /* Title */
    dw_title = lv_label_create(scr);
    lv_label_set_text(dw_title, "P4 ROBOT CONTROLLER");
    lv_obj_set_pos(dw_title, 30, 10);
    lv_obj_set_style_text_color(dw_title, lv_color_hex(0x58a6ff), 0);

    /* Status bar */
    dw_bar = lv_label_create(scr);
    lv_label_set_text(dw_bar, "");
    lv_obj_set_pos(dw_bar, 350, 14);
    lv_obj_set_style_text_color(dw_bar, lv_color_hex(0x8b949e), 0);

    /* Row 1: Status cards */
    static const struct { int x, w; const char *t; lv_obj_t **pp; } cards[] = {
        { 20, 185, "SPEED", &dw_spd },
        { 230, 225, "VISION", &dw_vis },
        { 480, 185, "PATROL", &dw_pat },
        { 690, 330, "COMMS", &dw_com },
    };
    for (int i = 0; i < 4; i++) {
        lv_obj_t *c = lv_obj_create(scr);
        lv_obj_set_size(c, cards[i].w, 55);
        lv_obj_set_pos(c, cards[i].x, 50);
        lv_obj_set_style_bg_color(c, lv_color_hex(0x161b22), 0);
        lv_obj_set_style_border_color(c, lv_color_hex(0x30363d), 0);
        lv_obj_set_style_border_width(c, 1, 0);
        lv_obj_set_style_radius(c, 6, 0);
        lv_obj_set_style_pad_all(c, 0, 0);
        *cards[i].pp = lv_label_create(c);
        lv_label_set_text(*cards[i].pp, cards[i].t);
        lv_obj_set_pos(*cards[i].pp, 10, 6);
        lv_obj_set_style_text_color(*cards[i].pp, lv_color_hex(0xc9d1d9), 0);
    }

    /* Row 2: Motor cards */
    static const int mx[] = { 20, 270, 520, 770 };
    for (int i = 0; i < 4; i++) {
        lv_obj_t *c = lv_obj_create(scr);
        lv_obj_set_size(c, 220, 75);
        lv_obj_set_pos(c, mx[i], 120);
        lv_obj_set_style_bg_color(c, lv_color_hex(0x161b22), 0);
        lv_obj_set_style_border_color(c, lv_color_hex(0x30363d), 0);
        lv_obj_set_style_border_width(c, 1, 0);
        lv_obj_set_style_radius(c, 6, 0);
        lv_obj_set_style_pad_all(c, 0, 0);
        dw_m[i] = lv_label_create(c);
        lv_label_set_text_fmt(dw_m[i], "M%d\n--", i + 1);
        lv_obj_set_pos(dw_m[i], 12, 8);
        lv_obj_set_style_text_color(dw_m[i], lv_color_hex(0xc9d1d9), 0);
    }

    /* Row 3: Sensors */
    lv_obj_t *c_tof = lv_obj_create(scr);
    lv_obj_set_size(c_tof, 490, 85);
    lv_obj_set_pos(c_tof, 20, 210);
    lv_obj_set_style_bg_color(c_tof, lv_color_hex(0x161b22), 0);
    lv_obj_set_style_border_color(c_tof, lv_color_hex(0x30363d), 0);
    lv_obj_set_style_border_width(c_tof, 1, 0);
    lv_obj_set_style_radius(c_tof, 6, 0);
    lv_obj_set_style_pad_all(c_tof, 0, 0);
    dw_tof = lv_label_create(c_tof);
    lv_label_set_text(dw_tof, "ToF: ---");
    lv_obj_set_pos(dw_tof, 10, 8);
    lv_obj_set_style_text_color(dw_tof, lv_color_hex(0xc9d1d9), 0);

    lv_obj_t *c_env = lv_obj_create(scr);
    lv_obj_set_size(c_env, 490, 85);
    lv_obj_set_pos(c_env, 530, 210);
    lv_obj_set_style_bg_color(c_env, lv_color_hex(0x161b22), 0);
    lv_obj_set_style_border_color(c_env, lv_color_hex(0x30363d), 0);
    lv_obj_set_style_border_width(c_env, 1, 0);
    lv_obj_set_style_radius(c_env, 6, 0);
    lv_obj_set_style_pad_all(c_env, 0, 0);
    dw_env = lv_label_create(c_env);
    lv_label_set_text(dw_env, "ENV: ---");
    lv_obj_set_pos(dw_env, 10, 8);
    lv_obj_set_style_text_color(dw_env, lv_color_hex(0xc9d1d9), 0);

    /* Camera preview image (hidden by default, shown on camera page) */
    dw_cam_img = lv_img_create(scr);
    lv_obj_set_pos(dw_cam_img, (1024 - 640) / 2, 55);
    lv_obj_set_size(dw_cam_img, 640, 480);
    lv_obj_add_flag(dw_cam_img, LV_OBJ_FLAG_HIDDEN);

    /* Gas card */
    dw_gcard = lv_obj_create(scr);
    lv_obj_set_size(dw_gcard, 1000, 65);
    lv_obj_set_pos(dw_gcard, 20, 310);
    lv_obj_set_style_bg_color(dw_gcard, lv_color_hex(0x161b22), 0);
    lv_obj_set_style_border_color(dw_gcard, lv_color_hex(0x30363d), 0);
    lv_obj_set_style_border_width(dw_gcard, 1, 0);
    lv_obj_set_style_radius(dw_gcard, 6, 0);
    lv_obj_set_style_pad_all(dw_gcard, 0, 0);
    dw_gas = lv_label_create(dw_gcard);
    lv_label_set_text(dw_gas, "GAS: ---");
    lv_obj_set_pos(dw_gas, 10, 8);
    lv_obj_set_style_text_color(dw_gas, lv_color_hex(0xc9d1d9), 0);

    /* ── Bottom touch buttons (always visible) ── */
    static const char *btn_labels[] = {"▲ SPEED", "◀ VISION", "🎤 VOICE", "▶ PATROL", "● STOP"};
    static const uint32_t btn_colors[] = {0x21262d, 0x21262d, 0x21262d, 0x21262d, 0x3d0000};
    int bx = 20;
    for (int i = 0; i < 5; i++) {
        int bw = (i < 4) ? 190 : 180;
        lv_obj_t *btn = lv_btn_create(scr);
        lv_obj_set_size(btn, bw, 42);
        lv_obj_set_pos(btn, bx, 545);
        lv_obj_set_style_bg_color(btn, lv_color_hex(btn_colors[i]), 0);
        lv_obj_set_style_border_color(btn, lv_color_hex(0x30363d), 0);
        lv_obj_set_style_border_width(btn, 1, 0);
        lv_obj_set_style_radius(btn, 6, 0);
        lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, btn_labels[i]);
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xc9d1d9), 0);
        lv_obj_center(lbl);
        lv_obj_add_event_cb(btn, btn_evt_cb, LV_EVENT_SHORT_CLICKED, (void *)(intptr_t)i);
        bx += bw + 15;
    }

    ESP_LOGI(TAG, "dashboard created");
}

void display_lcd_dashboard_update(const dashboard_data_t *d)
{
    const char *vm = d->vision_mode ? d->vision_mode : "OFF";
    const char *sl = d->speed_label ? d->speed_label : "--";

    /* Status bar */
    lv_label_set_text_fmt(dw_bar, "%s %d%%  |  C5:%s  CAM:%s",
                          sl, d->speed_pct,
                          d->c5_ok ? "OK" : "LOST",
                          d->cam_live ? "LIVE" : "OFF");

    /* Cards */
    lv_label_set_text_fmt(dw_spd, "SPEED: %s %d%%", sl, d->speed_pct);
    lv_label_set_text_fmt(dw_vis, "VISION: %s %s", vm, d->vision_running ? "●" : "○");
    if (d->patrol_active && d->patrol_idx >= 0)
        lv_label_set_text_fmt(dw_pat, "PATROL: P%d ●", d->patrol_idx);
    else
        lv_label_set_text(dw_pat, "PATROL: STOP ○");
    lv_label_set_text_fmt(dw_com, "C5: %s  |  CAM: %s",
                          d->c5_ok ? "OK" : "LOST",
                          d->cam_live ? "LIVE" : "OFF");

    /* Motors */
    for (int i = 0; i < 4; i++)
        lv_label_set_text_fmt(dw_m[i], "M%d\n%+ld", i + 1, (long)d->enc[i]);

    /* ToF */
    lv_label_set_text(dw_tof, "ToF: ---  [NONE]");

    /* Env */
    lv_label_set_text_fmt(dw_env, "T:%.1fC  H:%.1f%%  AQ:%d/%d",
                          (double)d->temp_c, (double)d->humidity,
                          d->air_quality, 100);

    /* Gas */
    lv_label_set_text_fmt(dw_gas, "MQ-135: %u (HAZ:%u%%)  |  MQ-136: %u (HAZ:%u%%)",
                          d->mq135_raw, d->mq135_hazard,
                          d->mq136_raw, d->mq136_hazard);
}

lv_display_t *display_lcd_get_disp(void)
{
    return s_disp;
}
