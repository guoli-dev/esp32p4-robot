#pragma once

#include <stdint.h>

/* 编码器 A/B 相信号 (避开 PSRAM GPIO24-31 及 BOOT/烧录脚 GPIO35/37，仅用 J2/J3 真实引出脚) */
#define E1A 46
#define E1B 36
#define E2A 47
#define E2B 31   /* was GPIO22 — moved to GPIO31 (J3-23) to avoid camera SCL */
#define E3A 30   /* was GPIO23 — moved to GPIO30 (J3-25) to avoid camera RST */
#define E3B 32
#define E4A 33
#define E4B 34

void encoder_init(void);
int32_t encoder_get_count(uint8_t encoder);
void encoder_reset(uint8_t encoder);
float encoder_get_rpm(uint8_t encoder, uint16_t pulses_per_rev);
