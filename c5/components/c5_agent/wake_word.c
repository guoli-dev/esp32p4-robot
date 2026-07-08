/**
 * @file wake_word.c — Lightweight "小智" wake word detector
 *
 * Method: Syllable-level state machine driven by short-time energy +
 * zero-crossing rate + band energy ratio. No FFT, no models, no SPIFFS.
 *
 * "小智" acoustic pattern (xiao3 zhi4):
 *   Syllable 1 "xiao3": ~300ms, high-freq fricative onset, vowel /iao/
 *   Inter-syllable gap:   ~80-150ms
 *   Syllable 2 "zhi4":  ~200ms, retroflex affricate onset, short vowel /i/
 *   Total: ~600-800ms
 *
 * Feature extraction per 32ms frame (512 samples @ 16kHz):
 *   - Frame energy (dB)
 *   - Zero-crossing rate
 *   - High/low band energy ratio (fricatives have more HF energy)
 *
 * The detector runs a state machine over the feature stream looking for
 * the characteristic two-syllable energy + spectral pattern.
 */

#include "wake_word.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

#define TAG "wake"

/* ── Constants ─────────────────────────────────────── */
#define SAMPLE_RATE      16000
#define FRAME_SIZE       512      /* 32ms @ 16kHz */
#define HISTORY_LEN      40       /* ~1.28s of feature history */
#define SILENCE_THRESH   0.02f    /* energy threshold for speech */
#define HF_RATIO_THRESH  0.25f    /* high-freq ratio for fricatives */
#define ZCR_SPEECH_MIN   8        /* min ZCR for speech */
#define ZCR_FRICATIVE    25       /* ZCR threshold for fricative sounds */

/* ── Per-frame features ────────────────────────────── */

typedef struct {
    float energy;        /* log energy (dB-like) */
    int   zcr;           /* zero-crossing count */
    float hf_ratio;      /* high-freq / total energy ratio */
} frame_feat_t;

/* ── Detection state machine ───────────────────────── */

typedef enum {
    ST_SILENCE,          /* waiting for speech */
    ST_SYL1_ONSET,       /* first syllable rising */
    ST_SYL1_BODY,        /* first syllable sustained */
    ST_GAP,              /* inter-syllable silence */
    ST_SYL2_ONSET,       /* second syllable rising */
    ST_SYL2_BODY,        /* second syllable sustained */
    ST_POST_SILENCE,     /* after second syllable */
} det_state_t;

/* ── Detector context ──────────────────────────────── */

static frame_feat_t s_history[HISTORY_LEN];
static int  s_hist_idx = 0;
static det_state_t s_state = ST_SILENCE;
static int  s_syl1_start = -1;     /* frame index of syllable 1 start */
static int  s_syl1_end   = -1;
static int  s_syl2_start = -1;
static int  s_syl2_end   = -1;
static int  s_gap_frames = 0;
static int  s_silence_frames = 0;

/* ── Feature extraction ────────────────────────────── */

static frame_feat_t extract_features(const int16_t *samples, size_t count)
{
    frame_feat_t f = {0};
    if (count == 0) return f;

    float sum_sq = 0.0f;
    float sum_hf = 0.0f;
    int   zcr    = 0;
    int   prev   = 0;

    /* Simple 1st-order high-pass for high-freq energy estimation.
       The idea: high-freq components cause rapid sample-to-sample changes.
       We approximate HF energy as the energy of the difference signal. */
    for (size_t i = 0; i < count; i++) {
        float s = (float)samples[i] / 32768.0f;
        sum_sq += s * s;

        /* High-pass: difference between current and previous sample */
        if (i > 0) {
            float diff = s - ((float)samples[i-1] / 32768.0f);
            sum_hf += diff * diff;
        }

        /* Zero-crossing rate */
        int cur = (samples[i] >= 0) ? 1 : 0;
        if (i > 0 && cur != prev) zcr++;
        prev = cur;
    }

    /* Energy in dB-like scale (avoid log(0)) */
    float e = sum_sq / (float)count;
    f.energy = (e > 1e-10f) ? 10.0f * log10f(e) : -100.0f;
    f.zcr = zcr;
    f.hf_ratio = (sum_sq > 0.0f) ? (sum_hf / sum_sq) : 0.0f;

    return f;
}

/* ── State machine ─────────────────────────────────── */

static bool detect_pattern(void)
{
    int idx = s_hist_idx;

    /* Get current and recent frames */
    frame_feat_t *cur  = &s_history[(idx - 1 + HISTORY_LEN) % HISTORY_LEN];
    frame_feat_t *prev = &s_history[(idx - 2 + HISTORY_LEN) % HISTORY_LEN];

    bool is_speech = (cur->energy > -30.0f && cur->zcr > ZCR_SPEECH_MIN);
    bool is_silence = (cur->energy < -35.0f);
    bool is_fricative = (cur->hf_ratio > HF_RATIO_THRESH && cur->zcr > ZCR_FRICATIVE);
    (void)is_fricative; /* reserved for fricative-weighted detection */

    switch (s_state) {

    case ST_SILENCE:
        /* Looking for onset: energy rising + some HF content */
        if (cur->energy > -28.0f && cur->energy > prev->energy + 3.0f
            && cur->zcr > ZCR_SPEECH_MIN) {
            s_state = ST_SYL1_ONSET;
            s_syl1_start = (idx - 1 + HISTORY_LEN) % HISTORY_LEN;
            s_silence_frames = 0;
        }
        break;

    case ST_SYL1_ONSET:
        if (is_speech) {
            s_state = ST_SYL1_BODY;
        } else if (is_silence) {
            /* false trigger — back to silence */
            s_state = ST_SILENCE;
        }
        break;

    case ST_SYL1_BODY:
        /* Syllable 1 should last ~8-15 frames (256-480ms) */
        if (is_silence) {
            s_syl1_end = (idx - 1 + HISTORY_LEN) % HISTORY_LEN;
            s_gap_frames = 1;
            s_state = ST_GAP;
        }
        break;

    case ST_GAP:
        if (is_silence) {
            s_gap_frames++;
            if (s_gap_frames > 10) {
                /* gap too long — false trigger */
                s_state = ST_SILENCE;
            }
        } else if (is_speech) {
            /* Gap should be 2-5 frames (64-160ms) */
            if (s_gap_frames >= 2 && s_gap_frames <= 6) {
                s_state = ST_SYL2_ONSET;
                s_syl2_start = (idx - 1 + HISTORY_LEN) % HISTORY_LEN;
            } else {
                s_state = ST_SILENCE; /* gap wrong length */
            }
        }
        break;

    case ST_SYL2_ONSET:
        if (is_speech) {
            s_state = ST_SYL2_BODY;
        } else if (is_silence) {
            s_state = ST_SILENCE; /* false trigger */
        }
        break;

    case ST_SYL2_BODY:
        /* Syllable 2 should be short: ~3-8 frames (96-256ms) */
        if (is_silence) {
            s_syl2_end = (idx - 1 + HISTORY_LEN) % HISTORY_LEN;
            s_silence_frames = 1;
            s_state = ST_POST_SILENCE;
        }
        break;

    case ST_POST_SILENCE:
        if (is_silence) {
            s_silence_frames++;
            if (s_silence_frames >= 3) {
                /* Confirmed: wake word pattern matched! */
                /* Validate syllable durations */
                int syl1_len = (s_syl1_end - s_syl1_start + HISTORY_LEN) % HISTORY_LEN;
                int syl2_len = (s_syl2_end - s_syl2_start + HISTORY_LEN) % HISTORY_LEN;
                int gap_len  = s_gap_frames;

                s_state = ST_SILENCE; /* reset for next detection */
                s_silence_frames = 0;

                if (syl1_len >= 5 && syl1_len <= 18 &&
                    syl2_len >= 2 && syl2_len <= 10 &&
                    gap_len  >= 2 && gap_len  <= 8) {
                    ESP_LOGI(TAG, "小智 detected! (syl1=%d frames, gap=%d, syl2=%d)",
                             syl1_len, gap_len, syl2_len);
                    return true;
                }
                ESP_LOGD(TAG, "pattern rejected: syl1=%d gap=%d syl2=%d",
                         syl1_len, gap_len, syl2_len);
            }
        } else if (is_speech) {
            s_state = ST_SILENCE; /* unexpected speech after wake word */
        }
        break;
    }

    return false;
}

/* ── Public API ────────────────────────────────────── */

void wake_word_init(void)
{
    memset(s_history, 0, sizeof(s_history));
    s_hist_idx = 0;
    s_state = ST_SILENCE;
    s_silence_frames = 0;
    ESP_LOGI(TAG, "init OK (wake word: 小智)");
}

bool wake_word_feed(const int16_t *samples, size_t count)
{
    if (!samples || count == 0) return false;

    /* Extract features and store in circular history */
    s_history[s_hist_idx % HISTORY_LEN] = extract_features(samples, count);
    s_hist_idx++;

    /* Need at least a few frames of history before detection */
    if (s_hist_idx < 3) return false;

    return detect_pattern();
}

void wake_word_reset(void)
{
    s_state = ST_SILENCE;
    s_silence_frames = 0;
    s_syl1_start = s_syl1_end = -1;
    s_syl2_start = s_syl2_end = -1;
    s_gap_frames = 0;
}
