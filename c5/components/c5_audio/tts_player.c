/**
 * @file tts_player.c — pre-recorded PCM audio playback engine
 *
 * Uses I2S TX DMA to play audio clips stored in flash.
 * Playback is non-blocking: the caller calls tts_player_play()
 * and polls tts_player_is_playing() to know when it's done.
 */

#include "tts_player.h"
#include "i2s_driver.h"
#include "audio_samples.h"
#include "protocol.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define TAG "tts"

static clip_desc_t s_clips[TTS_COUNT];

static bool      s_playing = false;
static tts_clip_t s_current_clip = TTS_COUNT;

/* ── public ──────────────────────────────────────── */

void tts_player_init(void)
{
    memset(s_clips, 0, sizeof(s_clips));
    audio_samples_load(s_clips);

    /* verify all clips have data */
    int missing = 0;
    for (int i = 0; i < TTS_COUNT; i++) {
        if (!s_clips[i].data || s_clips[i].len == 0) {
            ESP_LOGW(TAG, "clip %d missing", i);
            missing++;
        }
    }
    ESP_LOGI(TAG, "init OK  %d/%d clips loaded", TTS_COUNT - missing, TTS_COUNT);
}

void tts_player_play(tts_clip_t clip)
{
    if (clip >= TTS_COUNT) return;

    const clip_desc_t *desc = &s_clips[clip];
    if (!desc->data || desc->len == 0) {
        ESP_LOGW(TAG, "clip %d empty, skip", clip);
        return;
    }

    /* Wait for any previous playback to finish (don't re-init I2S —
     * the channel is already configured by i2s_full_duplex_init) */
    while (i2s_tx_is_busy()) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /* write entire clip to I2S DMA */
    bool ok = i2s_tx_write(desc->data, desc->len);
    if (ok) {
        s_playing       = true;
        s_current_clip  = clip;
        ESP_LOGI(TAG, "play clip %d (%lu samples)", clip, desc->len);
    } else {
        ESP_LOGW(TAG, "failed to start clip %d", clip);
    }
}

bool tts_player_is_playing(void)
{
    if (!s_playing) return false;
    if (i2s_tx_is_busy()) return true;

    /* DMA done */
    s_playing = false;
    s_current_clip = TTS_COUNT;
    return false;
}

void tts_player_stop(void)
{
    i2s_tx_stop();
    s_playing = false;
    s_current_clip = TTS_COUNT;
}

tts_clip_t tts_clip_for_command(uint8_t voice_cmd)
{
    switch (voice_cmd) {
    case VCMD_FORWARD:     return TTS_GO_FORWARD;
    case VCMD_BACKWARD:    return TTS_GO_BACKWARD;
    case VCMD_TURN_LEFT:   return TTS_TURN_LEFT;
    case VCMD_TURN_RIGHT:  return TTS_TURN_RIGHT;
    case VCMD_STOP:        return TTS_STOP;
    case VCMD_SPEED_UP:
    case VCMD_SPEED_DOWN:
    case VCMD_GO_HOME:
    case VCMD_PATROL:
    case VCMD_DANCE:
    default:               return TTS_OK;
    }
}
