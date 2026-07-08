#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/**
 * @file wake_word.h — Lightweight "小智" wake word detector
 *
 * FFT-based spectral template matching. No neural models, no SPIFFS.
 * ~15KB code, ~2KB RAM. Runs on ESP32-C5 alongside WiFi + I2S.
 *
 * Usage:
 *   wake_word_init();
 *   while (1) {
 *       int16_t buf[512];
 *       read_mic(buf, 512);          // 32ms @ 16kHz
 *       if (wake_word_feed(buf, 512)) {
 *           // "小智" detected! → start voice pipeline
 *       }
 *   }
 */

/** One-time init. */
void wake_word_init(void);

/**
 * @brief Feed a chunk of 16kHz mono PCM. Returns true if "小智" detected.
 * @param samples  int16_t PCM audio
 * @param count    Number of samples (should be 512 per call, ~32ms)
 */
bool wake_word_feed(const int16_t *samples, size_t count);

/** Reset the detector state (after wake word handled). */
void wake_word_reset(void);
