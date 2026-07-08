#include "motor_control.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define TAG "motor"

#define LEDC_MODE         LEDC_LOW_SPEED_MODE
#define LEDC_FREQ_HZ      20000
#define LEDC_DUTY_RES     LEDC_TIMER_10_BIT
#define LEDC_MAX_DUTY     ((1 << LEDC_DUTY_RES) - 1)

static const uint8_t s_motor_pwm[4]  = {M1_PWM, M2_PWM, M3_PWM, M4_PWM};
static const uint8_t s_motor_in1[4]  = {M1_IN1, M2_IN1, M3_IN1, M4_IN1};
static const uint8_t s_motor_in2[4]  = {M1_IN2, M2_IN2, M3_IN2, M4_IN2};

void motor_init(void)
{
    /* LEDC 定时器配置 */
    ledc_timer_config_t timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = LEDC_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    /* 每个电机的 LEDC 通道 */
    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ch = {
            .gpio_num       = s_motor_pwm[i],
            .speed_mode     = LEDC_MODE,
            .channel        = i,          /* 通道 0~3 */
            .intr_type      = LEDC_INTR_DISABLE,
            .timer_sel      = LEDC_TIMER_0,
            .duty           = 0,
            .hpoint         = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch));

        /* 方向引脚 */
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << s_motor_in1[i]) | (1ULL << s_motor_in2[i]),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&io));
        gpio_set_level(s_motor_in1[i], 0);
        gpio_set_level(s_motor_in2[i], 0);
    }

    /* STBY 引脚 */
    gpio_config_t stby = {
        .pin_bit_mask = (1ULL << MOTOR_STBY),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&stby));
    gpio_set_level(MOTOR_STBY, 1);  /* 默认使能，电机可正常工作 */

    ESP_LOGI(TAG, "4 路电机控制初始化完成");
}

void motor_set_speed(uint8_t motor, int16_t speed)
{
    if (motor > 3) return;

    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;

    uint32_t duty = (uint32_t)(((uint32_t)abs(speed) * LEDC_MAX_DUTY) / 100);

    ledc_set_duty(LEDC_MODE, motor, duty);
    ledc_update_duty(LEDC_MODE, motor);

    if (speed > 0) {
        gpio_set_level(s_motor_in1[motor], 1);
        gpio_set_level(s_motor_in2[motor], 0);
    } else if (speed < 0) {
        gpio_set_level(s_motor_in1[motor], 0);
        gpio_set_level(s_motor_in2[motor], 1);
    } else {
        gpio_set_level(s_motor_in1[motor], 0);
        gpio_set_level(s_motor_in2[motor], 0);
    }
}

void motor_stop(uint8_t motor)
{
    motor_set_speed(motor, 0);
}

void motor_brake(uint8_t motor)
{
    if (motor > 3) return;

    ledc_set_duty(LEDC_MODE, motor, 0);
    ledc_update_duty(LEDC_MODE, motor);
    gpio_set_level(s_motor_in1[motor], 1);
    gpio_set_level(s_motor_in2[motor], 1);
}

void motor_standby_enable(void)
{
    gpio_set_level(MOTOR_STBY, 1);
}

void motor_standby_disable(void)
{
    gpio_set_level(MOTOR_STBY, 0);
}
