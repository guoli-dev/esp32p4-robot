#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @file c5_bridge.h — P4 ↔ C5 UART command bridge
 *
 * Listens on UART1 for TLV-framed motor commands from C5.
 * Wiring: P4 J2-37(GPIO10)→C5 J2-14(GPIO25), P4 J2-30(GPIO11)←C5 J2-16(GPIO24)
 *
 * Protocol: [0xA5][type:1B][len:2B BE][payload:N][checksum:1B]
 * Commands handled: MOTOR (0x10), HEARTBEAT (0xF0)
 */

/* ── Pin mapping ── P4 UART1: GPIO10(TX)→C5 GPIO25, GPIO11(RX)←C5 GPIO24 */
#define C5_UART_PORT      UART_NUM_1
#define C5_UART_TX        10    /* P4 J2-37 U1TXD → C5 J2-14 GPIO25 */
#define C5_UART_RX        11    /* P4 J2-30 U1RXD ← C5 J2-16 GPIO24 */
#define C5_UART_BAUD      38400

/* ── Motor sub-commands (mirrors C5's p4_protocol.h) ─ */
#define C5_MOTOR_FORWARD    0x01
#define C5_MOTOR_BACKWARD   0x02
#define C5_MOTOR_TURN_LEFT  0x03
#define C5_MOTOR_TURN_RIGHT 0x04
#define C5_MOTOR_STOP       0x05
#define C5_MOTOR_SET_SPEED  0x06

/* ── Vision / AI results (P4 → C5) ────────────────── */
#define CMD_VISION_RESULT      0x20   /* vision result (P4→C5) */
#define CMD_OBSTACLE           0x23   /* obstacle avoidance alert (P4→C5) */

/* ── Navigation commands (C5 → P4) ───────────────── */
#define CMD_NAV_PATH        0x12   /* execute preset patrol path (payload: u8 idx) */
#define CMD_NAV_STRAIGHT    0x13   /* straight: float dist(4B) + u8 speed */
#define CMD_NAV_TURN        0x14   /* turn: float angle(4B) + u8 speed */
#define CMD_NAV_STOP        0x15   /* stop nav, stop motors */

/* ── Text display command (C5 → P4) ─────────────── */
#define CMD_TEXT_DISPLAY    0x40   /* display AI speech text on LCD */

/* ── Environmental data (P4 → C5) ────────────────── */
#define CMD_ENV_DATA        0x05   /* environmental sensor data */

/* ── Speech recognition events (P4 → C5) ─────────── */
#define CMD_WAKE_EVENT      0x03   /* wake word detected by P4 */
#define CMD_LOCAL_COMMAND   0x04   /* local voice command recognized */

/* ── Public API ────────────────────────────────────── */

/** Init UART1 + start command listener task on Core 0 */
void c5_bridge_init(void);

/** True if C5 link is alive (heartbeat received within 10s) */
bool c5_bridge_link_alive(void);

/** Get current speed set by C5 (0-100) */
int  c5_bridge_get_speed(void);

/**
 * @brief Send vision result over UART to C5 (deprecated, use face/gesture)
 */
void c5_bridge_send_vision(const void *vision_result);

/**
 * @brief Send obstacle avoidance alert to C5
 *
 * Payload (4 bytes):
 *   [0] level: 0=none, 1=warn, 2=slow, 3=stop
 *   [1] reserved (0x00)
 *   [2-3] range_mm: uint16 LE
 */
void c5_bridge_send_obstacle(uint8_t level, uint16_t range_mm);

/**
 * @brief Notify C5 that wake word was detected by P4.
 *        Payload: [0x01] = wake word ID.
 */
void c5_bridge_send_wake(void);

/**
 * @brief Send a locally-recognized voice command to C5.
 * @param cmd_text  Null-terminated UTF-8 command text (e.g., "前进")
 */
void c5_bridge_send_local_cmd(const char *cmd_text);

/**
 * @brief Send environmental sensor data to C5 for TTS broadcast.
 *
 * Payload (13 bytes, binary):
 *   [0-1]  temp_c * 10   (int16 BE)   e.g. 263 = 26.3°C
 *   [2-3]  humidity * 10  (uint16 BE)  e.g. 551 = 55.1%
 *   [4-5]  mq135_raw      (uint16 BE)  ADC 0-4095
 *   [6]    mq135_hazard   (uint8)      0-100
 *   [7-8]  mq136_raw      (uint16 BE)  ADC 0-4095
 *   [9]    mq136_hazard   (uint8)      0-100
 *   [10-11] reserved      (uint16)     0x0000
 *   [12]   air_quality    (uint8)
 *
 * C5 side parses and speaks: "温度26.3度，湿度55%，有害气体正常，H2S安全，空气质量良好"
 */
void c5_bridge_send_env(const void *env_data);

/**
 * @brief Set current speed percentage (1-100).
 *        Called by sr_task when voice command changes speed preset.
 */
void c5_bridge_set_speed(int pct);

/**
 * @brief Start a preset patrol path by index (0-3).
 *        Called by sr_task on voice navigation commands.
 *        Blocks until current motion completes, then starts path.
 */
void c5_bridge_start_patrol(int idx);

/**
 * @brief Get latest AI speech text from C5 (for LCD display).
 *        Returns NULL if no new text since last call.
 *        The caller should display this text and call again.
 */
const char *c5_bridge_get_display_text(void);

/**
 * @brief Acknowledge display text — clears the "new" flag.
 */
void c5_bridge_display_ack(void);
