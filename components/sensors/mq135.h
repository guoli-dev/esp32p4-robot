#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_adc/adc_oneshot.h"

/**
 * @file mq135.h — MQ-135 有害气体传感器驱动 (ADC2_CH0, GPIO49 J3-16)
 *
 * 检测: NH3/苯/NOx/烟雾
 *
 * 接线 (带电路板的完整模块):
 *   VCC → J3-1/3 (5V)
 *   GND → J3-6/8 (GND)
 *   AO  → J3-16 GPIO49 ⚠️ 5V输出需分压: AO→[10k]→GPIO49→[20k]→GND
 *   DO  → 不接
 */

#define MQ135_ADC_CHANNEL   ADC_CHANNEL_0   /* GPIO49 = ADC2_CH0, J3-16 */

/* ── API (handle 由外部统一管理) ───────────────── */

bool mq135_init(adc_oneshot_unit_handle_t adc_handle);
bool mq135_read_raw(uint16_t *raw);
bool mq135_read_hazard(uint8_t *hazard);
void mq135_calibrate(void);
