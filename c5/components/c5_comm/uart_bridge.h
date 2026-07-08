#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/* ── P4 ↔ C5 UART (UART1 — dedicated device link) ── */
/* C5 J2-16(GPIO24)→P4 J2-30(GPIO11), C5 J2-14(GPIO25)←P4 J2-37(GPIO10) */
#define C5_P4_TX_PIN    24   /* C5 U1TXD → P4 GPIO11 */
#define C5_P4_RX_PIN    25   /* C5 U1RXD ← P4 GPIO10 */
#define C5_P4_UART      UART_NUM_1
#define C5_P4_BAUD      38400

/**
 * @brief Initialize UART0 for P4 communication.
 */
void uart_p4_init(void);

/**
 * @brief Send raw bytes to P4. Returns true on success.
 */
bool uart_p4_send(const uint8_t *data, size_t len);

/**
 * @brief Non-blocking read from P4 UART. Returns bytes read.
 */
int uart_p4_read(uint8_t *buf, size_t max_len);

/* ── Framed protocol (p4_protocol.h) ──────────────── */

/**
 * @brief Send a framed command to P4.
 *        Automatically wraps payload with magic, type, length, checksum.
 *
 * @param cmd_type  Command type (P4_CMD_*)
 * @param payload   Payload data (NULL if len==0)
 * @param len       Payload length (max 255)
 * @return true on success
 */
bool uart_p4_send_frame(uint8_t cmd_type, const uint8_t *payload, uint16_t len);

/**
 * @brief Non-blocking framed receive. Call each tick to accumulate bytes.
 *
 * @param payload   Output buffer for payload data
 * @param len       [in/out] Input: buffer size; Output: bytes received
 * @param max_len   Size of payload buffer
 * @return Command type on success, 0 if no frame ready, 0xFF on checksum error
 */
uint8_t uart_p4_recv_frame(uint8_t *payload, uint16_t *len, uint16_t max_len);

/* ── Environment data cache (populated by main, read by voice_engine) ── */

/**
 * @brief Numeric env cache struct — lock-free, no JSON parsing needed.
 *        Each field is independently readable on RISC-V (no tearing for ≤4B).
 */
typedef struct {
    float    temp_c;
    float    humidity_pct;
    uint16_t mq135_raw;
    uint8_t  mq135_hazard;
    uint16_t mq136_raw;
    uint8_t  mq136_hazard;
    uint8_t  air_quality;
} env_cache_num_t;

/**
 * @brief Store latest environment data as JSON string + numeric cache.
 */
void uart_env_cache_set(const char *json);

/**
 * @brief Get latest environment JSON string (for BLE forwarding).
 * @return JSON string, or NULL if no data received yet.
 */
const char *uart_env_cache_get(void);

/**
 * @brief Get latest environment data as pre-parsed numbers (for voice_engine).
 * @return Pointer to cache struct, or NULL if no data yet.
 */
const env_cache_num_t *uart_env_num_get(void);

/**
 * @brief True if at least one env data frame has been received.
 */
bool uart_env_has_data(void);
