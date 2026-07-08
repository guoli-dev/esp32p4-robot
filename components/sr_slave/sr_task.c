/**
 * @file sr_task.c — Speech recognition task on P4
 *
 * Polls I2S slave PCM data in 32ms frames, feeds ESP-SR WakeNet9 + MultiNet7.
 * On wake word / command detection, notifies C5 via UART.
 *
 * Graceful fallback: if ESP-SR is not available (models not flashed),
 * falls back to the legacy wake_detect energy+ZCR state machine.
 *
 * C5 handles: cloud ASR → DeepSeek LLM → TTS for complex queries.
 * P4 handles:  offline wake word + local commands (instant, no WiFi).
 */

#include "sr_slave.h"
#include "sr_esp_sr.h"
#include "c5_bridge.h"
#include "motion_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define TAG "sr_task"

#define FRAME_SAMPLES   512       /* 32ms @ 16kHz */
#define POLL_DELAY_MS   10        /* poll interval when no data */

/* ── Speech recognition task ────────────────────────── */

static void sr_task(void *arg)
{
    (void)arg;

    /* Wait for C5 BCLK/WS to stabilize before init.
       C5 boots slower than P4 (WiFi + LLM init). */
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "starting I2S slave...");

    if (!sr_slave_init()) {
        ESP_LOGW(TAG, "I2S slave init FAILED — is C5 running and outputting BCLK/WS?");
        ESP_LOGW(TAG, "speech recognition disabled — retrying in 10s...");
        vTaskDelay(pdMS_TO_TICKS(10000));
        if (!sr_slave_init()) {
            ESP_LOGE(TAG, "I2S slave still failing — SR permanently disabled");
            vTaskDelete(NULL);
            return;
        }
    }

    if (sr_esp_init()) {
        ESP_LOGI(TAG, "SR backend: ESP-SR WakeNet9 + MultiNet7");
    } else {
        ESP_LOGW(TAG, "ESP-SR init failed — voice recognition disabled");
        ESP_LOGW(TAG, "Check: SPIFFS mounted? Models flashed to 'model' partition?");
    }
    ESP_LOGI(TAG, "SR task running — listening for '你好小智'");

    /* Pre-allocate frame buffer on stack (1KB) */
    int16_t frame[FRAME_SAMPLES];
    int32_t collected = 0;

    while (1) {
        /* Accumulate exactly FRAME_SAMPLES before feeding detector */
        size_t remaining = FRAME_SAMPLES - (size_t)collected;
        size_t n = sr_read_pcm(frame + collected, remaining);

        if (n > 0) {
            collected += (int32_t)n;
        } else {
            /* No data yet — C5 might not be streaming */
            vTaskDelay(pdMS_TO_TICKS(POLL_DELAY_MS));
            continue;
        }

        if (collected < FRAME_SAMPLES) {
            /* Wait for more samples */
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        /* Full frame ready — feed to ESP-SR detector */
        keyword_t result = sr_esp_feed(frame, FRAME_SAMPLES);
        collected = 0;

        switch (result) {

        case KW_WAKE:
            ESP_LOGI(TAG, "<<< WAKE WORD '你好小智' DETECTED >>>");
            c5_bridge_send_wake();
            sr_esp_reset();
            break;

        /* ── Motion: P4 local execution (instant, no UART roundtrip) ── */
        case KW_FORWARD:
            ESP_LOGI(TAG, "→ 前进 @ %d%%", c5_bridge_get_speed());
            motion_straight(30.0f, (int16_t)c5_bridge_get_speed());
            c5_bridge_send_local_cmd(sr_esp_keyword_name(result));
            sr_esp_reset();
            break;

        case KW_BACKWARD:
            ESP_LOGI(TAG, "→ 后退 @ %d%%", c5_bridge_get_speed());
            motion_straight(-30.0f, (int16_t)c5_bridge_get_speed());
            c5_bridge_send_local_cmd(sr_esp_keyword_name(result));
            sr_esp_reset();
            break;

        case KW_LEFT:
            ESP_LOGI(TAG, "→ 左转 @ %d%%", c5_bridge_get_speed());
            motion_turn(90.0f, (int16_t)c5_bridge_get_speed());
            c5_bridge_send_local_cmd(sr_esp_keyword_name(result));
            sr_esp_reset();
            break;

        case KW_RIGHT:
            ESP_LOGI(TAG, "→ 右转 @ %d%%", c5_bridge_get_speed());
            motion_turn(-90.0f, (int16_t)c5_bridge_get_speed());
            c5_bridge_send_local_cmd(sr_esp_keyword_name(result));
            sr_esp_reset();
            break;

        case KW_STOP:
            ESP_LOGI(TAG, "→ 停止");
            motion_stop();
            c5_bridge_send_local_cmd(sr_esp_keyword_name(result));
            sr_esp_reset();
            break;

        case KW_FASTER: {
            int spd = c5_bridge_get_speed() + 10;
            if (spd > 100) spd = 100;
            c5_bridge_set_speed(spd);
            ESP_LOGI(TAG, "→ 加速 → %d%%", spd);
            c5_bridge_send_local_cmd(sr_esp_keyword_name(result));
            sr_esp_reset();
            break;
        }

        case KW_SLOWER: {
            int spd = c5_bridge_get_speed() - 10;
            if (spd < 10) spd = 10;
            c5_bridge_set_speed(spd);
            ESP_LOGI(TAG, "→ 减速 → %d%%", spd);
            c5_bridge_send_local_cmd(sr_esp_keyword_name(result));
            sr_esp_reset();
            break;
        }

        case KW_SPIN:
            ESP_LOGI(TAG, "→ 转圈 @ %d%%", c5_bridge_get_speed());
            motion_turn(360.0f, (int16_t)c5_bridge_get_speed());
            c5_bridge_send_local_cmd(sr_esp_keyword_name(result));
            sr_esp_reset();
            break;

        case KW_TURN_AROUND:
            ESP_LOGI(TAG, "→ 掉头 @ %d%%", c5_bridge_get_speed());
            motion_turn(180.0f, (int16_t)c5_bridge_get_speed());
            c5_bridge_send_local_cmd(sr_esp_keyword_name(result));
            sr_esp_reset();
            break;

        /* ── Speed presets: P4 local ── */
        case KW_FULL_SPEED:
            c5_bridge_set_speed(100);
            ESP_LOGI(TAG, "→ 全速 100%%");
            c5_bridge_send_local_cmd(sr_esp_keyword_name(result));
            sr_esp_reset();
            break;

        case KW_HALF_SPEED:
            c5_bridge_set_speed(50);
            ESP_LOGI(TAG, "→ 半速 50%%");
            c5_bridge_send_local_cmd(sr_esp_keyword_name(result));
            sr_esp_reset();
            break;

        case KW_LOW_SPEED:
            c5_bridge_set_speed(30);
            ESP_LOGI(TAG, "→ 低速 30%%");
            c5_bridge_send_local_cmd(sr_esp_keyword_name(result));
            sr_esp_reset();
            break;

        /* ── Patrol navigation: P4 local ── */
        case KW_PATROL:
        case KW_PATROL_SQUARE:
            c5_bridge_start_patrol(0);
            c5_bridge_send_local_cmd(sr_esp_keyword_name(result));
            sr_esp_reset();
            break;

        case KW_PATROL_ZIGZAG:
            c5_bridge_start_patrol(1);
            c5_bridge_send_local_cmd(sr_esp_keyword_name(result));
            sr_esp_reset();
            break;

        case KW_PATROL_STRAIGHT:
            c5_bridge_start_patrol(2);
            c5_bridge_send_local_cmd(sr_esp_keyword_name(result));
            sr_esp_reset();
            break;

        case KW_PATROL_SAMPLE:
            c5_bridge_start_patrol(3);
            c5_bridge_send_local_cmd(sr_esp_keyword_name(result));
            sr_esp_reset();
            break;

        /* ── Vision control: P4 local ── */
        case KW_CAM_ON:
        case KW_COLOR_TRACK:
            robot_vision_activate(1);  /* color track */
            c5_bridge_send_local_cmd(sr_esp_keyword_name(result));
            sr_esp_reset();
            break;

        case KW_LINE_FOLLOW:
            robot_vision_activate(2);  /* line follow */
            c5_bridge_send_local_cmd(sr_esp_keyword_name(result));
            sr_esp_reset();
            break;

        case KW_GESTURE_CTRL:
            robot_vision_activate(3);  /* gesture control */
            c5_bridge_send_local_cmd(sr_esp_keyword_name(result));
            sr_esp_reset();
            break;

        case KW_CAM_OFF:
        case KW_VISION_OFF:
            robot_vision_deactivate();
            c5_bridge_send_local_cmd(sr_esp_keyword_name(result));
            sr_esp_reset();
            break;

        /* ── Sensor queries: forward to C5 for TTS ── */
        case KW_TEMPERATURE:
        case KW_HUMIDITY:
        case KW_AIR_QUALITY:
        case KW_ENV_REPORT:
            ESP_LOGI(TAG, "→ C5: %s", sr_esp_keyword_name(result));
            c5_bridge_send_local_cmd(sr_esp_keyword_name(result));
            sr_esp_reset();
            break;

        case KW_NONE:
        default:
            break;
        }

        /* Yield to other tasks (button, display, etc.) */
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/* ── Public ────────────────────────────────────────── */

void sr_task_init(void)
{
    xTaskCreatePinnedToCore(sr_task, "sr_task", 5120,
                            NULL, 4, NULL, 0);  /* Core 0, prio 4, stack 5KB */
}
