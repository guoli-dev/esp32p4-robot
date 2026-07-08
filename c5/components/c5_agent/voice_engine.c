/**
 * @file voice_engine.c — C5 AI Agent Engine (Wake Word detection)
 *
 * Pipeline:
 *   WakeWord("小智") → ASR(cloud) → DeepSeek LLM → local TTS → speaker
 *   Motor commands → UART1 → P4.
 *
 * UART: UART0=debug(PC), UART1=device(P4) — physically isolated.
 */

#include "voice_engine.h"
#include "uart_bridge.h"
#include "p4_protocol.h"
#include "tts_player.h"
#include "i2s_driver.h"
#include "audio_capture.h"
#include "cloud_api.h"
#include "local_tts.h"
#include "wake_word.h"
#include "wifi_mgr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define TAG "agent"

#define AGENT_TICK_MS     50
#define MAX_RECORD_SAMPLES (16000 * 2)  /* 2 seconds @ 16kHz (64KB) */

/* ── TTS callback — direct to I2S TX ──────────────── */

static void i2s_write_pcm(const int16_t *pcm, size_t count) { i2s_tx_write(pcm, count); }

/* ── LLM System Prompt ────────────────────────────── */

static const char *SYSTEM_PROMPT =
    "You are XiaoZhi, a robot. You MUST obey user movement orders immediately. "
    "Reply in Chinese <speech> (under 40 chars). "
    "When user says move/stop/turn, ALWAYS output the matching command: "
    "forward= MOTOR:forward, backward= MOTOR:backward, left= MOTOR:left, "
    "right= MOTOR:right, stop= MOTOR:stop. Speed: SPEED:N (1-100). "
    "Put commands in <command> tag. "
    "Example: user:'go forward' you:'<speech>OK moving</speech><command>MOTOR:forward</command>'";

/* ── agent state ──────────────────────────────────── */

static agent_state_t s_state = AGENT_WAITING;
static char   s_response_buf[1024];
static char   s_input_text[512];
static bool    s_text_pending = false;
static int16_t *s_rec_buf = NULL;
static volatile bool s_wake_requested = false;   /* touchscreen/P4 trigger */

/* ── event queue (text input from serial/P4) ──────── */
#define MSG_QUEUE_LEN  4
static QueueHandle_t s_msg_queue = NULL;
typedef struct { char text[512]; } msg_t;

/* ── motor command parser ─────────────────────────── */

static void parse_and_send_motor(const char *cmd_text)
{
    if (!cmd_text) return;

    if (strstr(cmd_text, "MOTOR:forward")) {
        uint8_t pld[] = { MOTOR_FORWARD };
        uart_p4_send_frame(P4_CMD_MOTOR, pld, sizeof(pld));
        ESP_LOGI(TAG, "P4 <- MOTOR:forward");
    } else if (strstr(cmd_text, "MOTOR:backward")) {
        uint8_t pld[] = { MOTOR_BACKWARD };
        uart_p4_send_frame(P4_CMD_MOTOR, pld, sizeof(pld));
        ESP_LOGI(TAG, "P4 <- MOTOR:backward");
    } else if (strstr(cmd_text, "MOTOR:left")) {
        uint8_t pld[] = { MOTOR_TURN_LEFT };
        uart_p4_send_frame(P4_CMD_MOTOR, pld, sizeof(pld));
        ESP_LOGI(TAG, "P4 <- MOTOR:turn_left");
    } else if (strstr(cmd_text, "MOTOR:right")) {
        uint8_t pld[] = { MOTOR_TURN_RIGHT };
        uart_p4_send_frame(P4_CMD_MOTOR, pld, sizeof(pld));
        ESP_LOGI(TAG, "P4 <- MOTOR:turn_right");
    } else if (strstr(cmd_text, "MOTOR:stop")) {
        uint8_t pld[] = { MOTOR_STOP };
        uart_p4_send_frame(P4_CMD_MOTOR, pld, sizeof(pld));
        ESP_LOGI(TAG, "P4 <- MOTOR:stop");
    }
    const char *spd = strstr(cmd_text, "SPEED:");
    if (spd) {
        int speed = atoi(spd + 6);
        if (speed < 1) speed = 1;
        if (speed > 100) speed = 100;
        uint8_t pld[] = { MOTOR_SET_SPEED, (uint8_t)speed };
        uart_p4_send_frame(P4_CMD_MOTOR, pld, sizeof(pld));
        ESP_LOGI(TAG, "P4 <- SPEED:%d", speed);
    }
}

/* ── direct motor dispatch from user input keywords ─── */

static bool input_dispatch_motor(const char *text)
{
    if (!text) return false;
    if (strstr(text, "forward") || strstr(text, "go")
        || strstr(text, "\xe5\x89\x8d")    /* 前 UTF-8 */
        || strstr(text, "\xc7\xb0")) {     /* 前 GB2312 */
        uint8_t pld[] = { MOTOR_FORWARD };
        uart_p4_send_frame(P4_CMD_MOTOR, pld, sizeof(pld));
        ESP_LOGI(TAG, "P4 <- FORWARD (direct)");
        return true;
    }
    if (strstr(text, "backward") || strstr(text, "back")
        || strstr(text, "\xe5\x90\x8e")    /* 后 UTF-8 */
        || strstr(text, "\xba\xf3")) {     /* 后 GB2312 */
        uint8_t pld[] = { MOTOR_BACKWARD };
        uart_p4_send_frame(P4_CMD_MOTOR, pld, sizeof(pld));
        ESP_LOGI(TAG, "P4 <- BACKWARD (direct)");
        return true;
    }
    if (strstr(text, "left")
        || strstr(text, "\xe5\xb7\xa6")    /* 左 UTF-8 */
        || strstr(text, "\xd7\xf3")) {     /* 左 GB2312 */
        uint8_t pld[] = { MOTOR_TURN_LEFT };
        uart_p4_send_frame(P4_CMD_MOTOR, pld, sizeof(pld));
        ESP_LOGI(TAG, "P4 <- TURN_LEFT (direct)");
        return true;
    }
    if (strstr(text, "right")
        || strstr(text, "\xe5\x8f\xb3")    /* 右 UTF-8 */
        || strstr(text, "\xd3\xd2")) {     /* 右 GB2312 */
        uint8_t pld[] = { MOTOR_TURN_RIGHT };
        uart_p4_send_frame(P4_CMD_MOTOR, pld, sizeof(pld));
        ESP_LOGI(TAG, "P4 <- TURN_RIGHT (direct)");
        return true;
    }
    if (strstr(text, "stop")
        || strstr(text, "\xe5\x81\x9c")    /* 停 UTF-8 */
        || strstr(text, "\xcd\xa3")) {     /* 停 GB2312 */
        uint8_t pld[] = { MOTOR_STOP };
        uart_p4_send_frame(P4_CMD_MOTOR, pld, sizeof(pld));
        ESP_LOGI(TAG, "P4 <- STOP (direct)");
        return true;
    }
    return false;
}

/* ── sensor query dispatcher (P4 voice → C5 TTS) ──────── */

static bool input_dispatch_sensor(const char *text)
{
    if (!text) return false;

    const env_cache_num_t *env = uart_env_num_get();
    if (!env) {
        ESP_LOGI(TAG, "sensor query but no env data yet");
        while (i2s_tx_is_busy()) vTaskDelay(pdMS_TO_TICKS(50));
        local_tts_synthesize("暂无环境数据", i2s_write_pcm);
        return true;
    }

    float t = env->temp_c;
    float h = env->humidity_pct;
    unsigned aq = env->air_quality;
    unsigned haz135 = env->mq135_hazard;
    unsigned haz136 = env->mq136_hazard;

    char tts_text[256];

    if (strstr(text, "温度")) {
        snprintf(tts_text, sizeof(tts_text), "当前温度%.1f度", (double)t);
    } else if (strstr(text, "湿度")) {
        snprintf(tts_text, sizeof(tts_text), "当前湿度百分之%.0f", (double)h);
    } else if (strstr(text, "空气质量") || strstr(text, "空气")) {
        const char *aq_str = (aq >= 75) ? "优" : (aq >= 50) ? "良" :
                             (aq >= 25) ? "差" : "危险";
        snprintf(tts_text, sizeof(tts_text), "空气质量%s", aq_str);
    } else if (strstr(text, "环境报告") || strstr(text, "环境")) {
        const char *aq_str = (aq >= 75) ? "优" : (aq >= 50) ? "良" :
                             (aq >= 25) ? "差" : "危险";
        const char *haz135_str = (haz135 <= 20) ? "正常" : (haz135 <= 50) ? "偏高" :
                                 (haz135 <= 80) ? "超标" : "严重";
        const char *haz136_str = (haz136 <= 10) ? "安全" : (haz136 <= 30) ? "注意" :
                                 (haz136 <= 60) ? "危险" : "剧毒";
        snprintf(tts_text, sizeof(tts_text),
                 "温度%.0f度, 湿度%.0f%%, 有害气体%s, 硫化氢%s, 空气质量%s",
                 (double)t, (double)h, haz135_str, haz136_str, aq_str);
    } else {
        return false;
    }

    ESP_LOGI(TAG, "sensor TTS: '%s'", tts_text);
    while (i2s_tx_is_busy()) vTaskDelay(pdMS_TO_TICKS(50));
    local_tts_synthesize(tts_text, i2s_write_pcm);
    return true;
}

/* ── extract tagged content ───────────────────────── */

static void extract_speech(const char *response, char *speech, size_t size)
{
    const char *start = strstr(response, "<speech>");
    const char *end   = strstr(response, "</speech>");
    if (start && end && start < end) {
        start += 8;
        size_t len = (size_t)(end - start);
        if (len >= size) len = size - 1;
        memcpy(speech, start, len);
        speech[len] = '\0';
    } else {
        strncpy(speech, response, size - 1);
        speech[size - 1] = '\0';
    }
}

static void extract_command(const char *response, char *cmd, size_t size)
{
    const char *start = strstr(response, "<command>");
    const char *end   = strstr(response, "</command>");
    if (start && end && start < end) {
        start += 9;
        size_t len = (size_t)(end - start);
        if (len >= size) len = size - 1;
        memcpy(cmd, start, len);
        cmd[len] = '\0';
    } else {
        cmd[0] = '\0';
    }
}

/* ── agent task ───────────────────────────────────── */

static void voice_agent_task(void *arg)
{
    ESP_LOGI(TAG, "agent task start (wake word '小智')");

    /* ── welcome ── */
    if (wifi_mgr_is_connected()) {
        ESP_LOGI(TAG, "WiFi connected — cloud AI ready");
        tts_player_play(TTS_WELCOME);
    } else {
        ESP_LOGI(TAG, "offline mode — local TTS + UART only");
    }
    while (tts_player_is_playing())
        vTaskDelay(pdMS_TO_TICKS(AGENT_TICK_MS));

    while (1) {
        /* ── Wait for event or timeout (32ms for wake word polling) ── */
        msg_t msg;
        bool got_msg = (xQueueReceive(s_msg_queue, &msg, pdMS_TO_TICKS(32)) == pdTRUE);
        if (got_msg) {
            strncpy(s_input_text, msg.text, sizeof(s_input_text) - 1);
            s_input_text[sizeof(s_input_text) - 1] = '\0';
            s_text_pending = true;
            ESP_LOGI(TAG, "msg: '%s'", s_input_text);
        }

        /* ── state machine ── */
        switch (s_state) {

        case AGENT_WAITING:
        case AGENT_IDLE:
        {
            /* ── periodic heartbeat to P4 (every ~2s) ── */
            static int hb_tick = 0;
            if (++hb_tick > 64) {
                hb_tick = 0;
                uart_p4_send_frame(P4_CMD_HEARTBEAT, NULL, 0);
            }

            if (s_text_pending) {
                if (s_rec_buf) { free(s_rec_buf); s_rec_buf = NULL; }
                s_state = AGENT_THINKING;
            } else if (s_wake_requested) {
                s_wake_requested = false;
                ESP_LOGI(TAG, "wake triggered from touchscreen");
                if (!s_rec_buf) {
                    s_rec_buf = malloc(MAX_RECORD_SAMPLES * sizeof(int16_t));
                    if (!s_rec_buf) break;
                }
                audio_capture_flush();
                s_state = AGENT_LISTENING;
            } else if (audio_capture_is_active()) {
                /* Listen for wake word "小智" via mic */
                int16_t ww_chunk[512];
                size_t n = audio_capture_read(ww_chunk, 512);
                if (n == 512 && wake_word_feed(ww_chunk, 512)) {
                    ESP_LOGI(TAG, "wake word '小智' detected!");
                    wake_word_reset();
                    if (!s_rec_buf) {
                        s_rec_buf = malloc(MAX_RECORD_SAMPLES * sizeof(int16_t));
                        if (!s_rec_buf) break;
                    }
                    audio_capture_flush();
                    s_state = AGENT_LISTENING;
                }
            }
            }
            break;

        case AGENT_LISTENING: {
            size_t rec_count = 0;
            int silence_ticks = 0;

            ESP_LOGI(TAG, "recording... (2s max, silence stops)");

            while (rec_count < MAX_RECORD_SAMPLES) {
                if (s_text_pending) {
                    if (s_rec_buf) { free(s_rec_buf); s_rec_buf = NULL; }
                    s_state = AGENT_THINKING; break;
                }

                int16_t chunk[256];
                size_t n = audio_capture_read(chunk, 256);
                float rms = 0.0f;
                if (n > 0) {
                    size_t copy = n;
                    if (rec_count + copy > MAX_RECORD_SAMPLES)
                        copy = MAX_RECORD_SAMPLES - rec_count;
                    memcpy(s_rec_buf + rec_count, chunk, copy * sizeof(int16_t));
                    rec_count += copy;
                    /* Compute RMS for silence detection */
                    for (size_t i = 0; i < n; i++) {
                        float s = (float)chunk[i] / 32768.0f;
                        rms += s * s;
                    }
                    rms = sqrtf(rms / n);
                }

                /* Silence detection: stop after ~800ms of quiet */
                if (rms < 0.01f) {
                    if (++silence_ticks > (800 / AGENT_TICK_MS)) break;
                } else {
                    silence_ticks = 0;
                }
                vTaskDelay(pdMS_TO_TICKS(AGENT_TICK_MS));
            }

            if (rec_count > 1600 && s_state == AGENT_LISTENING) {
                ESP_LOGI(TAG, "recorded %u samples (%.1fs)",
                         (unsigned)rec_count, (double)rec_count / 16000.0);

                char asr_text[512] = {0};
                int asr_ret = cloud_asr_transcribe(s_rec_buf, rec_count,
                                                   asr_text, sizeof(asr_text));
                free(s_rec_buf); s_rec_buf = NULL;

                if (asr_ret == 0 && asr_text[0]) {
                    strncpy(s_input_text, asr_text, sizeof(s_input_text) - 1);
                    s_text_pending = true;
                    s_state = AGENT_THINKING;
                } else {
                    ESP_LOGW(TAG, "ASR failed — back to idle");
                    tts_player_play(TTS_ERROR);
                    s_state = AGENT_SPEAKING;
                }
            } else if (s_state == AGENT_LISTENING) {
                ESP_LOGI(TAG, "recording too short — ignored");
                if (s_rec_buf) { free(s_rec_buf); s_rec_buf = NULL; }
                s_state = AGENT_IDLE;
            }
            break;
        }

        case AGENT_THINKING: {
            if (!cloud_api_is_ready() && !wifi_mgr_is_connected()) {
                ESP_LOGW(TAG, "no cloud");
                tts_player_play(TTS_ERROR);
                s_state = AGENT_SPEAKING;
                break;
            }
            /* Direct motor dispatch from user text keywords */
            input_dispatch_motor(s_input_text);

            ESP_LOGI(TAG, "LLM thinking...");
            size_t resp_len = sizeof(s_response_buf);
            int ret = cloud_llm_chat(s_input_text, SYSTEM_PROMPT,
                                     s_response_buf, &resp_len);
            s_text_pending = false;

            if (ret == 0) {
                char speech[256] = {0}, cmds[128] = {0};
                extract_speech(s_response_buf, speech, sizeof(speech));
                extract_command(s_response_buf, cmds, sizeof(cmds));
                ESP_LOGI(TAG, "LLM speech: '%s'", speech);
                if (cmds[0]) { ESP_LOGI(TAG, "LLM cmd: '%s'", cmds); parse_and_send_motor(cmds); }

                /* Send LLM speech text to P4 for LCD display */
                if (speech[0]) {
                    size_t tlen = strlen(speech);
                    if (tlen > 200) tlen = 200;
                    uart_p4_send_frame(P4_CMD_TEXT_DISPLAY, (const uint8_t *)speech, (uint16_t)tlen);
                }

                if (speech[0]) {
                    audio_capture_stop();
                    local_tts_synthesize(speech, i2s_write_pcm);
                    if (audio_capture_is_active()) audio_capture_start();
                }
            } else {
                ESP_LOGW(TAG, "LLM failed");
                tts_player_play(TTS_ERROR);
            }
            s_state = AGENT_SPEAKING;
            break;
        }

        case AGENT_SPEAKING:
            if (!i2s_tx_is_busy() && !tts_player_is_playing())
                s_state = AGENT_WAITING;
            break;

        case AGENT_ERROR:
            tts_player_play(TTS_ERROR);
            s_state = AGENT_SPEAKING;
            break;

        default:
            s_state = AGENT_WAITING;
            break;
        }

        tts_player_is_playing();
    }
}

/* ── public ──────────────────────────────────────── */

void voice_engine_init(void)
{
    s_rec_buf = NULL;
    s_msg_queue = xQueueCreate(MSG_QUEUE_LEN, sizeof(msg_t));

    wake_word_init();

    xTaskCreate(voice_agent_task, "voice_agent", 32768, NULL, 4, NULL);
    ESP_LOGI(TAG, "init OK (wake word '小智', event-driven)");
}

agent_state_t voice_engine_get_state(void) { return s_state; }

void voice_engine_trigger(agent_trigger_t trigger)
{
    if (trigger == TRIGGER_UART_CMD && s_state == AGENT_IDLE) { }
    ESP_LOGI(TAG, "triggered: %d", (int)trigger);
}

void voice_engine_feed_text(const char *text)
{
    if (!text || !s_msg_queue) return;
    msg_t msg = {0};
    strncpy(msg.text, text, sizeof(msg.text) - 1);
    xQueueSend(s_msg_queue, &msg, 0);
    ESP_LOGI(TAG, "queued: '%s'", text);
}

void voice_engine_trigger_wake(void)
{
    s_wake_requested = true;
    ESP_LOGI(TAG, "wake trigger queued");
}
