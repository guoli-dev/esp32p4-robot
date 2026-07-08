#pragma once

#include <stdint.h>

/**
 * @file protocol.h
 *
 * Voice command codes shared between C5 and P4 over UART.
 * P4 sends a single-byte command; C5 acknowledges and acts on it.
 */

/* ── voice command codes ─────────────────────────── */
#define VCMD_FORWARD     0x01
#define VCMD_BACKWARD    0x02
#define VCMD_TURN_LEFT   0x03
#define VCMD_TURN_RIGHT  0x04
#define VCMD_STOP        0x05
#define VCMD_SPEED_UP    0x06
#define VCMD_SPEED_DOWN  0x07
#define VCMD_GO_HOME     0x08
#define VCMD_PATROL      0x09
#define VCMD_DANCE       0x0A
