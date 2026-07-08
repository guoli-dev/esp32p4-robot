#pragma once

#include <stddef.h>
#include <stdint.h>

/**
 * @file local_tts.h — Offline Chinese TTS engine (formant synthesis)
 *
 * Zero-dependency, pure C Chinese text-to-speech for ESP32-C5.
 * Quality is "robot voice" but clearly intelligible — suitable for
 * a small robot assistant. No model files, no downloads, no API keys.
 *
 * Usage:
 *   local_tts_synthesize("你好世界", callback);
 *   // callback receives PCM chunks at 16kHz mono int16_t
 */

typedef void (*local_tts_callback_t)(const int16_t *pcm, size_t sample_count);

/**
 * @brief Synthesize Chinese text to PCM audio.
 *
 * @param text     UTF-8 Chinese text (ASCII/mixed OK, non-Chinese skipped)
 * @param callback Called for each ~20ms PCM chunk
 * @return Total samples generated, 0 on error
 */
size_t local_tts_synthesize(const char *text, local_tts_callback_t callback);
