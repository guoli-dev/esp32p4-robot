#pragma once

/**
 * @file sr_slave.h — P4 I2S Slave RX driver
 *
 * Captures audio from INMP441 microphone by snooping the I2S bus.
 * C5 is the I2S master (drives BCLK/WS) — P4 listens as a slave.
 *
 * Wiring:
 *   P4 GPIO0  (J2-25) ← C5 GPIO4  (BCLK)
 *   P4 GPIO51 (J3-12)  ← C5 GPIO5  (WS)
 *   P4 GPIO52 (J3-10)  ← INMP441 SD
 *
 * Config: I2S_NUM_1, 16kHz, 16-bit mono, Philips standard
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ── Pin assignments ───────────────────────────────── */

#define SR_I2S_PORT        I2S_NUM_1        /* separate from I2C1 (camera) */
#define SR_I2S_BCLK_PIN    0                /* J2-25, idle GPIO */
#define SR_I2S_WS_PIN      51               /* J3-12, idle GPIO */
#define SR_I2S_SD_PIN      52               /* J3-10, INMP441 SD (shared with C5 GPIO6) */
#define SR_SAMPLE_RATE     16000
#define SR_BITS_PER_SAMPLE I2S_DATA_BIT_WIDTH_16BIT

/* ── Ring buffer size (500ms @ 16kHz) ───────────────── */

#define SR_RING_SAMPLES    8000

/* ── API ──────────────────────────────────────────── */

/**
 * @brief Initialize I2S in slave RX mode.
 *        C5 must already be outputting BCLK/WS for this to succeed.
 * @return true on success
 */
bool sr_slave_init(void);

/**
 * @brief Read PCM samples from the DMA ring buffer. Non-blocking.
 * @param buf         Destination buffer
 * @param max_samples Max samples to read
 * @return Number of samples actually read (0 if nothing available)
 */
size_t sr_read_pcm(int16_t *buf, size_t max_samples);

/**
 * @brief Number of samples available in the ring buffer.
 */
size_t sr_available(void);

/**
 * @brief Flush ring buffer (discard all buffered data).
 */
void sr_flush(void);

/**
 * @brief Start the speech recognition task (calls sr_slave_init() internally).
 *        Safe to call from app_main().
 */
void sr_task_init(void);

/**
 * @brief True if I2S slave is initialized and receiving data.
 */
bool sr_is_active(void);

/* ── Robot state API (shared with main_btn.c) ─────────── */

/**
 * @brief Activate vision mode.  Called by sr_task on voice command.
 * @param mode  0=off, 1=color_track, 2=line_follow, 3=gesture
 */
void robot_vision_activate(int mode);

/**
 * @brief Deactivate vision processing (stop robot, clear mode).
 */
void robot_vision_deactivate(void);

/**
 * @brief True if vision-driven motion is currently active.
 */
bool robot_vision_is_running(void);
