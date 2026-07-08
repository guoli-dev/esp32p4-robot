/**
 * @file cam_ov5647.c — OV5647 MIPI CSI camera via esp_video (official BSP method)
 *
 * Uses esp_video component (V4L2 interface) for sensor detection, register init,
 * CSI controller setup, and XCLK generation — exactly like the official
 * WTDKP4C5-S1 BSP.
 *
 * I2C (SCCB) on GPIO7/8 shares the master bus with GT911 touch.
 * XCLK: module has own oscillator.
 */

#include "cam_ov5647.h"

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_ldo_regulator.h"
#include "esp_video_init.h"
#include "linux/videodev2.h"

#define TAG "cam_ov5647"

static int              s_video_fd   = -1;
static cam_state_t      s_state      = CAM_STATE_OFF;
static uint8_t         *s_fb[2]      = { NULL, NULL };
static size_t            s_fb_size   = 0;
static uint32_t          s_width     = 0;
static uint32_t          s_height    = 0;
static cam_frame_callback_t s_frame_cb = NULL;
static void            *s_frame_cb_user = NULL;
static TaskHandle_t     s_stream_task = NULL;
static volatile bool    s_stream_run  = false;

static void stream_task_fn(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "stream task started");

    while (s_stream_run) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(s_video_fd, VIDIOC_DQBUF, &buf) != 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        if (s_frame_cb)
            s_frame_cb(s_fb[buf.index], buf.length, s_frame_cb_user);
        ioctl(s_video_fd, VIDIOC_QBUF, &buf);
    }
    ESP_LOGI(TAG, "stream task stopped");
    vTaskDelete(NULL);
}

bool cam_init(const cam_config_t *cfg)
{
    if (s_state != CAM_STATE_OFF) return true;

    ESP_LOGI(TAG, "── Camera init via esp_video ──");

    esp_video_init_csi_config_t csi_cfg = {
        .sccb_config = {
            .init_sccb  = false,
            .i2c_handle = cfg ? cfg->i2c_bus : NULL,
            .freq       = 100000,
        },
        .reset_pin = -1,
        .pwdn_pin  = -1,
    };

    /* Enable VDD_CAM (LDO_VO4) — camera module power on J4 */
    {
        esp_ldo_channel_handle_t ldo_vdd = NULL;
        esp_ldo_channel_config_t ldo_cfg = { .chan_id = 4, .voltage_mv = 3300 };
        if (esp_ldo_acquire_channel(&ldo_cfg, &ldo_vdd) == ESP_OK) {
            ESP_LOGI(TAG, "LDO_VO4 3.3V enabled — camera powered");
        } else {
            ESP_LOGW(TAG, "LDO_VO4 init failed");
        }
    }

    esp_video_init_config_t vid_cfg = { .csi = &csi_cfg };
    esp_err_t err = esp_video_init(&vid_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_video_init failed: 0x%x (%s)", err, esp_err_to_name(err));
        s_state = CAM_STATE_ERROR;
        return false;
    }

    s_video_fd = open("/dev/video0", O_RDONLY);
    if (s_video_fd < 0) {
        ESP_LOGE(TAG, "open /dev/video0 failed");
        s_state = CAM_STATE_ERROR;
        return false;
    }

    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(s_video_fd, VIDIOC_G_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "VIDIOC_G_FMT failed");
        close(s_video_fd);
        s_video_fd = -1;
        s_state = CAM_STATE_ERROR;
        return false;
    }

    s_width  = fmt.fmt.pix.width;
    s_height = fmt.fmt.pix.height;
    ESP_LOGI(TAG, "Camera: %lux%lu fmt=0x%08lx",
             (unsigned long)s_width, (unsigned long)s_height,
             (unsigned long)fmt.fmt.pix.pixelformat);

    /* Set to 640×480 RGB565 for better preview quality */
    {
        struct v4l2_format sf;
        memset(&sf, 0, sizeof(sf));
        sf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        sf.fmt.pix.width       = 640;
        sf.fmt.pix.height      = 480;
        sf.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
        int r = ioctl(s_video_fd, VIDIOC_S_FMT, &sf);
        if (r == 0) {
            s_width  = sf.fmt.pix.width;
            s_height = sf.fmt.pix.height;
            ESP_LOGI(TAG, "Format set: %lux%lu RGB565",
                     (unsigned long)s_width, (unsigned long)s_height);
        } else {
            ESP_LOGW(TAG, "S_FMT 640×480 not supported, keeping default");
        }
    }

    /* MMAP buffers (same as official BSP app_video.c) */
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count  = 2;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(s_video_fd, VIDIOC_REQBUFS, &req) != 0) {
        ESP_LOGE(TAG, "REQBUFS failed");
        goto err;
    }

    for (int i = 0; i < 2; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;
        if (ioctl(s_video_fd, VIDIOC_QUERYBUF, &buf) != 0) {
            ESP_LOGE(TAG, "QUERYBUF[%d] failed", i);
            goto err;
        }
        s_fb_size = buf.length;
        s_fb[i] = (uint8_t *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                  MAP_SHARED, s_video_fd, buf.m.offset);
        if (s_fb[i] == MAP_FAILED) {
            ESP_LOGE(TAG, "mmap[%d] failed", i);
            s_fb[i] = NULL;
            goto err;
        }
        if (ioctl(s_video_fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGE(TAG, "QBUF[%d] failed", i);
            goto err;
        }
    }

    s_state = CAM_STATE_READY;
    ESP_LOGI(TAG, "── Camera init OK (esp_video) ──");
    return true;

err:
    for (int i = 0; i < 2; i++) { free(s_fb[i]); s_fb[i] = NULL; }
    if (s_video_fd >= 0) { close(s_video_fd); s_video_fd = -1; }
    s_state = CAM_STATE_ERROR;
    return false;
}

bool cam_start(cam_frame_callback_t cb, void *user_data)
{
    if (s_state != CAM_STATE_READY) {
        ESP_LOGE(TAG, "cam_start: wrong state %d", s_state);
        return false;
    }
    if (!cb) return false;

    s_frame_cb = cb;
    s_frame_cb_user = user_data;
    s_stream_run = true;

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(s_video_fd, VIDIOC_STREAMON, &type) != 0) {
        ESP_LOGE(TAG, "STREAMON failed");
        s_stream_run = false;
        return false;
    }
    xTaskCreatePinnedToCore(stream_task_fn, "cam_stream", 4096, NULL, 3, &s_stream_task, 1);
    s_state = CAM_STATE_STREAMING;
    ESP_LOGI(TAG, "Streaming started");
    return true;
}

bool cam_stop(void)
{
    if (s_state != CAM_STATE_STREAMING) return true;
    s_stream_run = false;
    vTaskDelay(pdMS_TO_TICKS(50));
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(s_video_fd, VIDIOC_STREAMOFF, &type);
    if (s_stream_task) { vTaskDelete(s_stream_task); s_stream_task = NULL; }
    s_frame_cb = NULL;
    s_state = CAM_STATE_READY;
    ESP_LOGI(TAG, "Streaming stopped");
    return true;
}

cam_state_t cam_get_state(void)      { return s_state; }
size_t      cam_get_frame_size(void) { return s_fb_size; }
uint32_t    cam_get_width(void)      { return s_width; }
uint32_t    cam_get_height(void)     { return s_height; }

void cam_deinit(void)
{
    if (s_state == CAM_STATE_STREAMING) cam_stop();
    for (int i = 0; i < 2; i++) { free(s_fb[i]); s_fb[i] = NULL; }
    if (s_video_fd >= 0) { close(s_video_fd); s_video_fd = -1; }
    s_state = CAM_STATE_OFF;
    ESP_LOGI(TAG, "Camera deinit done");
}
