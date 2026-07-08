/**
 * @file button.c
 * @brief 4 键消抖驱动 — 边沿触发
 *
 * 状态机：每个按键有 4 态
 *   IDLE → (按下足够久) → PRESSED  → 发出 press 边沿，锁存
 *    ↓                                              ↓
 *   (松手足够久) ← RELEASED ← (松手足够久)        (保持，不重复)
 *
 * button_tick() 每 ~20ms 调用一次。
 * button_pressed()  消费上升沿（调用后清除，只返回一次 true）
 * button_released() 消费下降沿
 */

#include "button.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

#define TAG "btn"
#define DEBOUNCE_MS  40
#define TICK_MS      20
#define STABLE_TICKS (DEBOUNCE_MS / TICK_MS)

static const uint8_t s_gpio[BTN_COUNT] = {
    [BTN_FWD]   = 48,
    [BTN_LEFT]  = 39,
    [BTN_RIGHT] = 43,
    [BTN_BACK]  = 44,
};

static int8_t  s_stable[BTN_COUNT];
static bool    s_state[BTN_COUNT];      /* current debounced state: true=pressed */
static bool    s_press_edge[BTN_COUNT]; /* latched rising edge */
static bool    s_release_edge[BTN_COUNT]; /* latched falling edge */

void button_init(void)
{
    for (int i = 0; i < BTN_COUNT; i++) {
        gpio_config_t cfg = {
            .pin_bit_mask = (1ULL << s_gpio[i]),
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&cfg);
    }
    memset(s_stable,       0, sizeof(s_stable));
    memset(s_state,        0, sizeof(s_state));
    memset(s_press_edge,   0, sizeof(s_press_edge));
    memset(s_release_edge, 0, sizeof(s_release_edge));
    ESP_LOGI(TAG, "4-btn edge-triggered driver OK");
}

bool button_pressed(button_t btn)
{
    bool val = s_press_edge[btn];
    s_press_edge[btn] = false;   /* consume the edge */
    return val;
}

bool button_released(button_t btn)
{
    bool val = s_release_edge[btn];
    s_release_edge[btn] = false;
    return val;
}

void button_tick(void)
{
    for (int i = 0; i < BTN_COUNT; i++) {
        bool raw = (gpio_get_level(s_gpio[i]) == 0);   /* active low */

        if (raw) {
            if (s_stable[i] < STABLE_TICKS) s_stable[i]++;
        } else {
            if (s_stable[i] > -STABLE_TICKS) s_stable[i]--;
        }

        /* Rising edge: not pressed → pressed */
        if (!s_state[i] && s_stable[i] >= STABLE_TICKS) {
            s_state[i]      = true;
            s_press_edge[i] = true;
        }
        /* Falling edge: pressed → not pressed */
        else if (s_state[i] && s_stable[i] <= -STABLE_TICKS) {
            s_state[i]        = false;
            s_release_edge[i] = true;
        }
    }
}
