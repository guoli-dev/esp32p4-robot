#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

/**
 * @file cloud_api.h
 *
 * Cloud API client for LLM chat, ASR (speech-to-text), and TTS (text-to-speech).
 *
 * All functions are synchronous (blocking) and must be called from a task
 * with sufficient stack (~6KB).
 */

/* ── Init ─────────────────────────────────────────── */

/**
 * @brief One-time init. Call after WiFi is connected.
 */
void cloud_api_init(void);

/**
 * @brief Store an API key in NVS (overrides compile-time default).
 */
void cloud_api_set_key(const char *provider, const char *key);

/**
 * @brief True when ready (WiFi connected + keys present).
 */
bool cloud_api_is_ready(void);

/* ── LLM Chat ─────────────────────────────────────── */

/**
 * @brief Send a user message to the LLM, get a text response.
 *
 * @param user_message   The user's input text
 * @param system_prompt  System prompt for agent persona (NULL for default)
 * @param response_buf   Caller-provided output buffer
 * @param[in,out] buf_size  Input: buffer size; output: bytes written (0-terminated)
 * @return 0 on success, -1 on error
 */
int cloud_llm_chat(const char *user_message,
                   const char *system_prompt,
                   char *response_buf, size_t *buf_size);

/* ── ASR ──────────────────────────────────────────── */

/**
 * @brief Transcribe PCM audio to text via cloud ASR.
 *
 * @param audio        int16_t mono PCM, 16kHz
 * @param sample_count Number of samples
 * @param text_buf     Caller-provided output buffer for transcription
 * @param text_size    Size of text_buf
 * @return strlen(text_buf) on success, -1 on error
 */
int cloud_asr_transcribe(const int16_t *audio, size_t sample_count,
                         char *text_buf, size_t text_size);

/* ── TTS ──────────────────────────────────────────── */

/**
 * @brief Callback for receiving PCM audio chunks from cloud TTS.
 *        Called one or more times as the TTS audio downloads.
 */
typedef void (*cloud_tts_callback_t)(const int16_t *pcm, size_t sample_count);

/**
 * @brief Synthesize speech from text via cloud TTS.
 *
 * @param text      Text to speak
 * @param callback  Called for each PCM chunk received
 * @return Total sample count on success, -1 on error
 */
int cloud_tts_synthesize(const char *text, cloud_tts_callback_t callback);
