#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/* ── I2S pins (C5) ───────────────────────────────── */
#define I2S_BCLK_PIN    4    /* C5_IO4,  shared: INMP441 SCK + MAX98357A BCLK */
#define I2S_WS_PIN      5    /* C5_IO5,  shared: INMP441 WS  + MAX98357A LRCK */
#define I2S_DOUT_PIN    23   /* C5_IO23, MAX98357A DIN */
#define I2S_DIN_PIN     6    /* C5_IO6,  INMP441 SD (Phase 3 RX, not used yet) */

#define I2S_PORT        I2S_NUM_0
#define I2S_SAMPLE_RATE 16000
#define I2S_BITS_PER_SAMPLE 16

/**
 * @brief Initialize I2S in TX-only mode (speaker output)
 *
 * Phase 1: TX only — MAX98357A speaker amp.
 */
void i2s_tx_init(void);

/**
 * @brief Initialize I2S in full-duplex mode (TX speaker + RX microphone).
 *
 * Phase 3: Enables both MAX98357A and INMP441 on shared BCLK/WS.
 * Falls back to TX-only if RX channel init fails (no mic connected).
 *
 * @return true if both TX and RX are active, false if RX failed (TX still works).
 */
bool i2s_full_duplex_init(void);

/**
 * @brief Write PCM samples to I2S TX DMA buffer (non-blocking).
 *
 * @param data   int16_t mono PCM samples at 16kHz
 * @param count  number of samples
 * @return true if data accepted by DMA
 */
bool i2s_tx_write(const int16_t *data, size_t count);

/**
 * @brief Check if I2S TX is still playing (DMA not finished).
 */
bool i2s_tx_is_busy(void);

/**
 * @brief Stop I2S TX immediately.
 */
void i2s_tx_stop(void);

/**
 * @brief Stop I2S TX AND RX (full deinit).  Use i2s_tx_stop() for TX-only stop.
 */
void i2s_deinit(void);

/**
 * @brief Read captured audio from I2S RX DMA buffer (non-blocking).
 *
 * @param buf       Destination buffer for int16_t PCM samples
 * @param max_samples  Maximum number of samples to read
 * @return Number of samples actually read, 0 if RX is not active
 */
size_t i2s_rx_read(int16_t *buf, size_t max_samples);
