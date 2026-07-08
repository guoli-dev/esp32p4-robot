#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * @file audio_capture.h
 *
 * Microphone capture wrapper around I2S RX.
 * Provides a simple ring buffer for mic audio data.
 * Gracefully returns 0 when no microphone is connected.
 */

/**
 * @brief Initialize audio capture ring buffer.
 *        Must be called after i2s_full_duplex_init().
 *
 * @param ring_buffer_samples  Size of ring buffer (e.g., 8000 = 500ms @16kHz)
 */
void audio_capture_init(size_t ring_buffer_samples);

/**
 * @brief Start background capture (enables DMA→ring buffer flow).
 *        Typically called when entering LISTENING state.
 */
void audio_capture_start(void);

/**
 * @brief Stop background capture. DMA keeps running but data is discarded.
 */
void audio_capture_stop(void);

/**
 * @brief Read captured audio from ring buffer. Non-blocking.
 *
 * @param buf         Destination buffer
 * @param max_samples Max samples to read
 * @return Number of samples actually read
 */
size_t audio_capture_read(int16_t *buf, size_t max_samples);

/**
 * @brief Discard all buffered audio data.
 */
void audio_capture_flush(void);

/**
 * @brief How many samples are available in the ring buffer.
 */
size_t audio_capture_available(void);

/**
 * @brief True if the microphone is active and capturing.
 */
bool audio_capture_is_active(void);
