/**
 * @file audio_samples.c — compiled-in audio clip data
 *
 * These are PLACEHOLDER samples (sine beeps).
 * Replace with real recordings via:  python tools/encode_audio.py <file>.wav
 *
 * Each clip format: 16kHz, 16-bit signed PCM, mono.
 * A ~1s clip = 16000 samples = 32KB.
 *
 * Clips are heap-allocated to exact size to avoid wasting DRAM.
 */

#include "audio_samples.h"
#include "tts_player.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"

#define TAG "samples"

/* ── helper: generate a sine-wave beep of given freq + duration ── */
static void gen_beep(int16_t *buf, int freq_hz, int duration_ms,
                     int sample_rate, int *out_len)
{
    int samples = sample_rate * duration_ms / 1000;
    for (int i = 0; i < samples; i++) {
        double t  = (double)i / sample_rate;
        double val = sin(2.0 * 3.14159265 * freq_hz * t);
        /* envelope: fade in/out 10ms */
        double env = 1.0;
        int fade   = sample_rate * 10 / 1000;  /* 10ms */
        if (i < fade)       env = (double)i / fade;
        if (i > samples - fade) env = (double)(samples - i) / fade;
        buf[i] = (int16_t)(val * env * 12000);
    }
    *out_len = samples;
}

/* ── heap-allocated clip storage ──────────────────── */

static int16_t *s_ok       = NULL;
static int16_t *s_forward  = NULL;
static int16_t *s_backward = NULL;
static int16_t *s_left     = NULL;
static int16_t *s_right    = NULL;
static int16_t *s_stop     = NULL;
static int16_t *s_error    = NULL;
static int16_t *s_welcome  = NULL;
static int16_t *s_beep     = NULL;

static int s_ok_len, s_forward_len, s_backward_len, s_left_len;
static int s_right_len, s_stop_len, s_error_len, s_welcome_len, s_beep_len;

/* ── public ──────────────────────────────────────── */

void audio_samples_load(clip_desc_t *clips)
{
    if (!clips) return;

    /* calculate required sample count for each clip */
    int ok_len, fwd_len, bwd_len, left_len, right_len;
    int stop_len, err_len, wel_len, beep_len;

    ok_len    = 16000 * 200 / 1000;   /* 200ms */
    fwd_len   = 16000 * 300 / 1000;   /* 300ms */
    bwd_len   = 16000 * 300 / 1000;   /* 300ms */
    left_len  = 16000 * 250 / 1000;   /* 250ms */
    right_len = 16000 * 250 / 1000;   /* 250ms */
    stop_len  = 16000 * 150 / 1000;   /* 150ms */
    err_len   = 16000 * 400 / 1000;   /* 400ms */
    wel_len   = 16000 * 600 / 1000;   /* 600ms */
    beep_len  = 16000 * 100 / 1000;   /* 100ms */

    /* allocate exact-size buffers */
    s_ok       = malloc(ok_len    * sizeof(int16_t));
    s_forward  = malloc(fwd_len   * sizeof(int16_t));
    s_backward = malloc(bwd_len   * sizeof(int16_t));
    s_left     = malloc(left_len  * sizeof(int16_t));
    s_right    = malloc(right_len * sizeof(int16_t));
    s_stop     = malloc(stop_len  * sizeof(int16_t));
    s_error    = malloc(err_len   * sizeof(int16_t));
    s_welcome  = malloc(wel_len   * sizeof(int16_t));
    s_beep     = malloc(beep_len  * sizeof(int16_t));

    /* ── generate placeholder beeps ──
     * Each clip uses a different frequency so you can
     * distinguish them during testing.
     * Replace with real recordings for production.
     */
    if (s_ok)       gen_beep(s_ok,       800, 200, 16000, &s_ok_len);
    if (s_forward)  gen_beep(s_forward,  600, 300, 16000, &s_forward_len);
    if (s_backward) gen_beep(s_backward, 400, 300, 16000, &s_backward_len);
    if (s_left)     gen_beep(s_left,     700, 250, 16000, &s_left_len);
    if (s_right)    gen_beep(s_right,    900, 250, 16000, &s_right_len);
    if (s_stop)     gen_beep(s_stop,    1000, 150, 16000, &s_stop_len);
    if (s_error)    gen_beep(s_error,    300, 400, 16000, &s_error_len);
    if (s_welcome)  gen_beep(s_welcome,  500, 600, 16000, &s_welcome_len);
    if (s_beep)     gen_beep(s_beep,    1200, 100, 16000, &s_beep_len);

    ESP_LOGI(TAG, "using placeholder beeps — replace with real recordings");

    /* populate descriptor table */
    clips[TTS_OK]           = (clip_desc_t){ s_ok,       s_ok_len };
    clips[TTS_GO_FORWARD]   = (clip_desc_t){ s_forward,  s_forward_len };
    clips[TTS_GO_BACKWARD]  = (clip_desc_t){ s_backward, s_backward_len };
    clips[TTS_TURN_LEFT]    = (clip_desc_t){ s_left,     s_left_len };
    clips[TTS_TURN_RIGHT]   = (clip_desc_t){ s_right,    s_right_len };
    clips[TTS_STOP]         = (clip_desc_t){ s_stop,     s_stop_len };
    clips[TTS_ERROR]        = (clip_desc_t){ s_error,    s_error_len };
    clips[TTS_WELCOME]      = (clip_desc_t){ s_welcome,  s_welcome_len };
    clips[TTS_BEEP]         = (clip_desc_t){ s_beep,     s_beep_len };
}
