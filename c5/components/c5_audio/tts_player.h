#pragma once

#include <stdint.h>
#include <stdbool.h>

/* ── TTS audio clips ─────────────────────────────── */
typedef enum {
    TTS_OK = 0,             /* "好的" */
    TTS_GO_FORWARD,         /* "前进" */
    TTS_GO_BACKWARD,        /* "后退" */
    TTS_TURN_LEFT,          /* "左转" */
    TTS_TURN_RIGHT,         /* "右转" */
    TTS_STOP,               /* "已停止" */
    TTS_ERROR,              /* "请再说一次" */
    TTS_WELCOME,            /* "你好，我是小智" */
    TTS_BEEP,               /* short beep for wake acknowledgment */
    TTS_COUNT,
} tts_clip_t;

/**
 * @brief Initialize TTS player (I2S TX must be initialized first).
 */
void tts_player_init(void);

/**
 * @brief Play a pre-recorded audio clip. Non-blocking — returns immediately.
 *        DMA feeds the audio to I2S in the background.
 */
void tts_player_play(tts_clip_t clip);

/**
 * @brief Check if a clip is currently playing.
 */
bool tts_player_is_playing(void);

/**
 * @brief Stop playback immediately.
 */
void tts_player_stop(void);

/**
 * @brief Map voice_cmd_t to the corresponding TTS clip for acknowledgment.
 */
tts_clip_t tts_clip_for_command(uint8_t voice_cmd);
