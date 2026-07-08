#pragma once

/**
 * @file cam_ov5647.h
 * @brief OV5647 MIPI CSI camera driver for ESP32-P4
 *
 * WTDKP4C5-S1 J4 — 22pin 0.5mm FPC MIPI CSI connector.
 * MIPI CSI differential lanes are fixed by P4 silicon:
 *   CSI_D0_P/N, CSI_D1_P/N, CSI_CLK_P/N → P4 package pins 42-47
 *
 * Control signals (SCCB I2C, RST, PWDN) via regular GPIO.
 * Defaults below are TENTATIVE — verify against WTDKP4C5-S1 schematic.
 *
 * Usage:
 *   cam_init(NULL);                   // use defaults
 *   cam_start(my_callback, user_data); // begin streaming
 *   cam_stop();                        // stop streaming
 *   cam_deinit();                      // release hardware
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "driver/i2c_master.h"

/* ═══════════════════════════════════════════════════════════════
 * Pin definitions — verified against WTDKP4C5-S1 J4 schematic
 * ═══════════════════════════════════════════════════════════════ */

/* SCCB (I2C) bus — verified against official WTDKP4C5-S1 Kconfig defaults.
 * J4 (22-pin FPC): CAM_SDA=GPIO7, CAM_SCL=GPIO8, XCLK=GPIO24.
 * XCLK is fixed by PCB routing through J4 — do not change.
 *
 * I2C 总线改由 display_lcd 组件初始化 (I2C master bus, GPIO7/8),
 * 与触摸 GT911 共享. 摄像头通过 cam_config_t.i2c_bus 获取外部句柄. */
#define CAM_PIN_SDA         7       /* J4 → GPIO7  (CAM_SDA, verified official) */
#define CAM_PIN_SCL         8       /* J4 → GPIO8  (CAM_SCL, verified official) */
#define CAM_PIN_RST         -1      /* Not connected (official default -1) */
#define CAM_PIN_PWDN        -1      /* Not connected on this board */
#define CAM_PIN_XCLK        24      /* J4 → GPIO24 (24 MHz XCLK, fixed by PCB) */

/* I2C bus parameters */
#define CAM_SCCB_FREQ_HZ    100000  /* 100 kHz SCCB (OV5647 max 400 kHz) */

/* OV5647 SCCB address — some modules use 0x36 when AD0 floats.
 * The probe dynamically detects the correct address at runtime. */
#define OV5647_I2C_ADDR_DEFAULT  0x3C

/* CHIP_ID register for autodetection */
#define OV5647_REG_CHIP_ID_H 0x300A
#define OV5647_REG_CHIP_ID_L 0x300B
#define OV5647_CHIP_ID       0x5647

/* ═══════════════════════════════════════════════════════════════
 * CSI controller parameters
 * ═══════════════════════════════════════════════════════════════ */

#define CAM_CSI_CTLR_ID             0
#define CAM_CSI_LANE_NUM            2
#define CAM_CSI_LANE_BITRATE_MBPS   200     /* OV5647 2-lane typical */

/* Default resolution */
#define CAM_DEFAULT_H_RES           800
#define CAM_DEFAULT_V_RES           640

/* Expected frame buffer size (RAW8: 1 byte/pixel; RGB565: 2 bytes/pixel) */
#define CAM_FB_SIZE_RAW(h, v)       ((uint32_t)(h) * (uint32_t)(v))

/* ═══════════════════════════════════════════════════════════════
 * Types
 * ═══════════════════════════════════════════════════════════════ */

/** Camera state machine */
typedef enum {
    CAM_STATE_OFF = 0,          /* Not initialized */
    CAM_STATE_READY,            /* Init OK, not streaming */
    CAM_STATE_STREAMING,        /* Actively capturing */
    CAM_STATE_ERROR,            /* Hardware error */
} cam_state_t;

/** Resolution preset */
typedef enum {
    CAM_RES_800x640 = 0,        /* Default, ~50 fps */
    CAM_RES_640x480,            /* VGA, ~60 fps */
    CAM_RES_1280x720,           /* 720p, ~30 fps */
    CAM_RES_1920x1080,          /* 1080p, ~15 fps */
    CAM_RES_2592x1944,          /* Full sensor, ~3 fps */
    CAM_RES_COUNT
} cam_resolution_t;

/**
 * @brief Frame-ready callback (runs in ISR context — keep short!)
 * @param buf       Frame buffer pointer (RGB565 / RAW8 depending on format)
 * @param len       Frame size in bytes
 * @param user_data User-provided context
 */
typedef void (*cam_frame_callback_t)(const uint8_t *buf, size_t len,
                                     void *user_data);

/** Camera configuration (pass NULL to cam_init() for defaults) */
typedef struct {
    i2c_master_bus_handle_t i2c_bus;    /* External I2C master bus (display_lcd GPIO7/8) or NULL for auto-init */
    int              sccb_sda;       /* SCCB SDA GPIO, -1 = default */
    int              sccb_scl;       /* SCCB SCL GPIO, -1 = default */
    int              rst_gpio;       /* Sensor reset, -1 = default/nc */
    int              pwdn_gpio;      /* Sensor pwdn, -1 = default/nc */
    cam_resolution_t resolution;    /* Capture resolution */
} cam_config_t;

/* ═══════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════ */

/**
 * @brief Initialize camera hardware and sensor
 *
 * Powers on sensor, detects OV5647, configures CSI bridge.
 * @param cfg Camera configuration (NULL = use defaults)
 * @return true on success
 */
bool cam_init(const cam_config_t *cfg);

/**
 * @brief Start streaming frames
 *
 * Enables CSI DMA and starts sensor output.
 * Callback runs in ISR context — queue/defer heavy work.
 *
 * @param cb        Frame-ready callback (required)
 * @param user_data User context passed to callback
 * @return true on success
 */
bool cam_start(cam_frame_callback_t cb, void *user_data);

/**
 * @brief Stop streaming
 * @return true on success
 */
bool cam_stop(void);

/**
 * @brief Get current camera state
 */
cam_state_t cam_get_state(void);

/**
 * @brief Get frame buffer size in bytes
 */
size_t cam_get_frame_size(void);

/**
 * @brief Get camera image width in pixels
 */
uint32_t cam_get_width(void);

/**
 * @brief Get camera image height in pixels
 */
uint32_t cam_get_height(void);

/**
 * @brief Deinitialize camera (release all resources)
 */
void cam_deinit(void);
