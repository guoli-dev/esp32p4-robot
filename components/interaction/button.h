#pragma once

/**
 * @file button.h
 * @brief 4 键按钮驱动（边沿触发消抖）
 *
 * button_pressed():  按下边沿（只触发一次，后续保持期间不重复触发）
 * button_released(): 释放边沿（只触发一次）
 *
 * 接线:
 *   GPIO48 → FWD  (J3-18)
 *   GPIO39 → LEFT (J3-36)
 *   GPIO43 → RIGHT(J3-28)
 *   GPIO44 → BACK (J3-26)
 *  按键另一端接地，使用芯片内部上拉，按下=低电平
 */

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    BTN_FWD   = 0,
    BTN_LEFT  = 1,
    BTN_RIGHT = 2,
    BTN_BACK  = 3,
    BTN_COUNT
} button_t;

void  button_init(void);
bool  button_pressed(button_t btn);    /* rising edge: not-pressed → pressed */
bool  button_released(button_t btn);   /* falling edge: pressed → not-pressed */
void  button_tick(void);
