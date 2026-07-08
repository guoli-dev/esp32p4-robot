#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * @file c5_ble.h — C5 BLE NUS (Nordic UART Service)
 *
 * 手机 ←→ BLE ←→ C5 ←→ UART ←→ P4 的无线串口中继
 *
 * NUS UUID: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 *   RX (Write):     6E400002 — 手机→C5  文本指令
 *   TX (Notify):    6E400003 — C5→手机  传感器 JSON
 *
 * 设备名: "EnvCar-XXXX" (XXXX = MAC 后 4 hex)
 *
 * ── 手机指令格式 (以 \n 结尾) ────────────────────
 *   FWD:<cm>,<spd>     直行  (C5 → UART: P4_CMD_NAV_STRAIGHT)
 *   TURN:<deg>,<spd>   转弯  (C5 → UART: P4_CMD_NAV_TURN)
 *   STOP               停止  (C5 → UART: P4_CMD_NAV_STOP)
 *   PATH:<0-3>         巡逻  (C5 → UART: P4_CMD_NAV_PATH)
 *   SPEED:<pct>        速度  (C5 → UART: P4_CMD_MOTOR + MOTOR_SET_SPEED)
 *   SENSOR             请求传感器 (回应最新的环境 JSON)
 */

/* ── 回调 ──────────────────────────────────────────── */

/** BLE 收到一条完整的文本指令 */
typedef void (*c5_ble_rx_cb_t)(const char *cmd);

/* ── API ───────────────────────────────────────────── */

/** @brief 初始化 NimBLE NUS 服务并开始广播
 *  @param on_rx 手机指令回调 (在 BLE host task 中调用, 需快速返回)
 */
void c5_ble_init(c5_ble_rx_cb_t on_rx);

/** @brief 向已连接的手机发送一段文本 (通常为 JSON)
 *
 *  线程安全, 可在任何任务/回调中调用。
 *  若当前无客户端连接, 静默丢弃。
 */
void c5_ble_send(const char *text);

/** @brief 查询是否有已连接的客户端 */
bool c5_ble_is_connected(void);
