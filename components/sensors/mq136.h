#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_adc/adc_oneshot.h"

/**
 * @file mq136.h — MQ-136 硫化氢(H₂S)传感器驱动 (ADC2_CH1, GPIO50 J3-14)
 *
 * 检测: H₂S 1-200ppm (化工厂污水池/反应釜泄漏预警)
 *
 * 接线 (4pin 模块):
 *   VCC → J3-1/3 (5V)
 *   GND → J3-6/8 (GND)
 *   AO  → J3-14 GPIO50 ⚠️ 5V输出需分压: AO→[10k]→GPIO50→[20k]→GND
 *   DO  → 不接
 */

#define MQ136_ADC_CHANNEL   ADC_CHANNEL_1   /* GPIO50 = ADC2_CH1, J3-14 */

/* ── API (handle 由外部统一管理) ───────────────── */

bool mq136_init(adc_oneshot_unit_handle_t adc_handle);
bool mq136_read_raw(uint16_t *raw);
bool mq136_read_hazard(uint8_t *hazard);
void mq136_calibrate(void);
