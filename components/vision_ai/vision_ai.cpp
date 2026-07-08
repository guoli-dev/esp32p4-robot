/**
 * @file vision_ai.cpp
 * @brief C++ 包装 — ESP-DL 人脸/行人检测，通过 C API 暴露给 C 代码
 *
 * ESP32-P4 NPU 硬件加速，几乎不占 CPU 资源。
 */

#include "vision_ai.h"
#include "esp_log.h"
#include <list>

/* ESP-DL 检测器头文件 */
#include "human_face_detect.hpp"
#include "pedestrian_detect.hpp"

using namespace dl::image;

#define TAG "vision_ai"

/* ── 静态检测器实例 (懒加载) ───────────────────────── */

static HumanFaceDetect    *s_face_det    = nullptr;
static PedestrianDetect   *s_ped_det     = nullptr;
static bool                s_face_inited = false;
static bool                s_ped_inited  = false;

/* ── 内部: 运行检测器, 填充结果 ───────────────────── */

static bool run_detect(dl::detect::DetectWrapper *detector,
                       const uint16_t *frame, int w, int h,
                       vision_ai_result_t *out)
{
    if (!detector || !frame || !out) return false;

    /* 构建 ESP-DL 图像描述 */
    dl::image::img_t img;
    img.data     = (void *)frame;
    img.width    = w;
    img.height   = h;
    img.pix_type = DL_IMAGE_PIX_TYPE_RGB565;

    /* 执行检测 (NPU 加速) */
    std::list<dl::detect::result_t> &results = detector->run(img);

    int count = 0;
    for (auto &r : results) {
        if (count >= 16) break;
        out->boxes[count].x1    = r.box[0];
        out->boxes[count].y1    = r.box[1];
        out->boxes[count].x2    = r.box[2];
        out->boxes[count].y2    = r.box[3];
        out->boxes[count].score = r.score;
        count++;
    }
    out->count = count;
    return (count > 0);
}

/* ══════════════════════════════════════════════════════════
 * Public C API
 * ══════════════════════════════════════════════════════════ */

bool vision_ai_init(vision_ai_type_t type)
{
    switch (type) {
    case VISION_AI_FACE:
        if (!s_face_inited) {
            /* HumanFaceDetect 构造函数加载模型 (从 flash rodata) */
            s_face_det = new HumanFaceDetect();
            s_face_inited = (s_face_det != nullptr);
            if (s_face_inited) {
                ESP_LOGI(TAG, "Face detector init OK (NPU)");
            } else {
                ESP_LOGE(TAG, "Face detector init FAILED");
            }
        }
        return s_face_inited;

    case VISION_AI_PEDESTRIAN:
        if (!s_ped_inited) {
            s_ped_det = new PedestrianDetect();
            s_ped_inited = (s_ped_det != nullptr);
            if (s_ped_inited) {
                ESP_LOGI(TAG, "Pedestrian detector init OK (NPU)");
            } else {
                ESP_LOGE(TAG, "Pedestrian detector init FAILED");
            }
        }
        return s_ped_inited;

    default:
        return false;
    }
}

bool vision_ai_detect(vision_ai_type_t type,
                      const uint16_t *frame, int width, int height,
                      vision_ai_result_t *out)
{
    switch (type) {
    case VISION_AI_FACE:
        if (!s_face_det) {
            if (!vision_ai_init(VISION_AI_FACE)) return false;
        }
        return run_detect(s_face_det, frame, width, height, out);

    case VISION_AI_PEDESTRIAN:
        if (!s_ped_det) {
            if (!vision_ai_init(VISION_AI_PEDESTRIAN)) return false;
        }
        return run_detect(s_ped_det, frame, width, height, out);

    default:
        return false;
    }
}

void vision_ai_draw_boxes(uint16_t *frame, int width, int height,
                          const vision_ai_result_t *result,
                          uint8_t r, uint8_t g, uint8_t b)
{
    if (!frame || !result) return;

    /* RGB565 颜色编码: R(5) G(6) B(5) */
    uint16_t color = ((r & 0x1F) << 11) | ((g & 0x3F) << 5) | (b & 0x1F);

    for (int i = 0; i < result->count; i++) {
        int x1 = result->boxes[i].x1;
        int y1 = result->boxes[i].y1;
        int x2 = result->boxes[i].x2;
        int y2 = result->boxes[i].y2;

        /* 剪裁到有效范围 */
        if (x1 < 0) x1 = 0;
        if (x2 >= width)  x2 = width - 1;
        if (y1 < 0) y1 = 0;
        if (y2 >= height) y2 = height - 1;

        /* 画四条边 (2px 粗) */
        for (int t = 0; t < 2; t++) {
            /* 上边 */
            for (int x = x1; x <= x2; x++) {
                int yy = y1 + t;
                if (yy >= 0 && yy < height)
                    frame[yy * width + x] = color;
                yy = y2 - t;
                if (yy >= 0 && yy < height)
                    frame[yy * width + x] = color;
            }
            /* 下边 */
            for (int y = y1 + t; y <= y2 - t; y++) {
                int xx = x1 + t;
                if (xx >= 0 && xx < width)
                    frame[y * width + xx] = color;
                xx = x2 - t;
                if (xx >= 0 && xx < width)
                    frame[y * width + xx] = color;
            }
        }
    }
}

void vision_ai_deinit(vision_ai_type_t type)
{
    switch (type) {
    case VISION_AI_FACE:
        if (s_face_det) {
            delete s_face_det;
            s_face_det = nullptr;
        }
        s_face_inited = false;
        break;

    case VISION_AI_PEDESTRIAN:
        if (s_ped_det) {
            delete s_ped_det;
            s_ped_det = nullptr;
        }
        s_ped_inited = false;
        break;
    }
}
