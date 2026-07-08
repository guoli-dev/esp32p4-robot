#pragma once

/**
 * @file vision_ai.h
 * @brief C 接口桥接 — 人脸检测 + 行人检测 (ESP-DL NPU 加速)
 *
 * 内部使用 C++ ESP-DL 库，通过 extern "C" 暴露给 C 代码。
 * 检测结果通过矩形框坐标列表返回。
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 检测结果：一个物体矩形框 */
typedef struct {
    int x1, y1, x2, y2;   /* 归一化坐标 (0~1023, 0~767) */
    float score;           /* 置信度 0.0~1.0 */
} vision_ai_box_t;

/** 单次检测结果 */
typedef struct {
    vision_ai_box_t boxes[16];  /* 最多 16 个检测目标 */
    int            count;       /* 实际检测数 */
} vision_ai_result_t;

/** 检测类型 */
typedef enum {
    VISION_AI_FACE      = 0,   /* 人脸检测 */
    VISION_AI_PEDESTRIAN = 1,  /* 行人检测 */
} vision_ai_type_t;

/**
 * @brief 初始化 AI 检测器 (第一次调用时懒加载)
 * @param type 检测类型
 * @return true=初始化成功
 */
bool vision_ai_init(vision_ai_type_t type);

/**
 * @brief 对 RGB565 帧执行检测
 * @param type      检测类型
 * @param frame     RGB565 帧缓冲 (width * height * 2 字节)
 * @param width     帧宽度
 * @param height    帧高度
 * @param out       检测结果输出
 * @return true=检测到目标
 */
bool vision_ai_detect(vision_ai_type_t type,
                      const uint16_t *frame, int width, int height,
                      vision_ai_result_t *out);

/**
 * @brief 在 RGB565 帧上绘制检测框 (就地修改)
 * @param frame     RGB565 帧缓冲
 * @param width     帧宽度
 * @param height    帧高度
 * @param result    检测结果
 * @param r/g/b     框颜色 (0-31 每个通道)
 */
void vision_ai_draw_boxes(uint16_t *frame, int width, int height,
                          const vision_ai_result_t *result,
                          uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 释放检测器资源
 * @param type 检测类型
 */
void vision_ai_deinit(vision_ai_type_t type);

#ifdef __cplusplus
}
#endif
