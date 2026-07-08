#pragma once

#include <stdint.h>

/**
 * @file p4_protocol.h
 *
 * C5 ↔ P4 UART framed protocol definitions.
 *
 * Frame format (total overhead = 5 bytes):
 *   [0xA5] [type:1B] [length:2B BE] [payload:N bytes] [checksum:1B]
 *
 * checksum = XOR of type, length_hi, length_lo, and all payload bytes.
 */

#define P4_FRAME_MAGIC         0xA5
#define P4_FRAME_HEADER_SIZE   4    /* magic + type + length_hi + length_lo */
#define P4_FRAME_FOOTER_SIZE   1    /* checksum */
#define P4_FRAME_MAX_PAYLOAD   256

/* ── Command types: C5 → P4 ──────────────────────── */

#define P4_CMD_MOTOR           0x10  /* motor control */
#define P4_CMD_HEARTBEAT       0xF0  /* heartbeat poll */

/* Navigation commands (C5 → P4) */
#define P4_CMD_NAV_PATH        0x12  /* execute preset patrol path (payload: u8 idx 0-3) */
#define P4_CMD_NAV_STRAIGHT    0x13  /* straight: float dist(4B) + u8 speed */
#define P4_CMD_NAV_TURN        0x14  /* turn: float angle(4B) + u8 speed */
#define P4_CMD_NAV_STOP        0x15  /* stop nav, stop all motors */

/* ── Command types: P4 → C5 ──────────────────────── */

#define P4_CMD_MOTOR_ACK       0x11  /* motor command executed */
#define P4_CMD_HEARTBEAT_ACK   0xF1  /* heartbeat response */
#define P4_CMD_ERROR           0xFF  /* error with error code */

/* ── Legacy compat (still used by c5_agent) ──────── */

#define P4_CMD_TEXT_INPUT      0x41  /* text from P4 (fallback input) */
#define P4_CMD_TEXT_DISPLAY    0x40  /* show text on P4's display */

/* ── P4 → C5 data / events ───────────────────────── */

#define P4_EVT_WAKE            0x03  /* wake word detected by P4 */
#define P4_EVT_LOCAL_CMD       0x04  /* local voice command from P4 */
#define P4_EVT_ENV_DATA        0x05  /* environmental sensor data (13 bytes) */
#define P4_EVT_VISION_RESULT       0x20  /* [deprecated] old vision result */
#define P4_EVT_FACE_RESULT         0x21  /* face recognition result */
#define P4_EVT_GESTURE_RESULT      0x22  /* hand gesture recognition */
#define P4_EVT_OBSTACLE            0x23  /* obstacle avoidance alert */
/*
 * P4_EVT_OBSTACLE payload (4 bytes):
 *   [0]    level:    0=none, 1=warn, 2=slow, 3=stop
 *   [1]    reserved  (0x00)
 *   [2-3]  range_mm: uint16 LE (前方距离)
 *
 * P4_EVT_FACE_RESULT payload:
 *   [count:1B] [faceId:2B LE][conf:1B][x:2B LE][y:2B LE][w:2B LE][h:2B LE] × N
 *
 * P4_EVT_GESTURE_RESULT payload:
 *   [type:1B][conf:1B][x:2B LE][y:2B LE][w:2B LE][h:2B LE]  (10 bytes)
 *   type: 0=none, 1=palm, 2=fist, 3=peace, 4=thumb_up, 5=point
 */

/* ── Env data payload layout (13 bytes, from P4) ───
 *   [0-1]  temp_c * 10   (int16 BE)    e.g. 263 = 26.3°C
 *   [2-3]  humidity * 10  (uint16 BE)   e.g. 551 = 55.1%
 *   [4-5]  mq135_raw      (uint16 BE)   ADC 0-4095
 *   [6]    mq135_hazard   (uint8)       综合有害气体指数 0-100
 *   [7-8]  mq136_raw      (uint16 BE)   ADC 0-4095
 *   [9]    mq136_hazard   (uint8)       H2S危险指数 0-100
 *   [10-11] reserved      (uint16)      0x0000
 *   [12]   air_quality    (uint8)
 */

/* ── Motor sub-commands (payload[0] of P4_CMD_MOTOR) ─ */

#define MOTOR_FORWARD          0x01
#define MOTOR_BACKWARD         0x02
#define MOTOR_TURN_LEFT        0x03
#define MOTOR_TURN_RIGHT       0x04
#define MOTOR_STOP             0x05
#define MOTOR_SET_SPEED        0x06  /* payload[1] = speed 0-100 */

/* ── Legacy compatibility ─────────────────────────── */

/* Old Route B format: [0xA5] [clip_id] — used for TTS play commands.
   Still supported alongside framed protocol. A standalone 0xA5 byte
   followed by a clip_id (< 0x10) is treated as a legacy TTS command.
   Framed commands use length > 0 in the header. */
