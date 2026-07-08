#pragma once

/**
 * @file cloud_config.h
 *
 * Cloud API configuration — edit this file to set your API keys and endpoints.
 * Compatible with OpenAI API and any OpenAI-compatible service (Qwen, etc.)
 */

/* ── LLM (chat completion) ────────────────────────── */

/* OpenAI-compatible chat completions endpoint */
#define CLOUD_LLM_URL          "https://api.deepseek.com/v1/chat/completions"

/* Your API key — override at runtime via serial: "!key llm sk-xxxxx" */
#define CLOUD_LLM_API_KEY      "sk-placeholder-llm"

/* API key for ASR/TTS — set at runtime via serial: "!key openai sk-xxxxx" */
#define CLOUD_OPENAI_API_KEY   "sk-placeholder-openai"

/* Model: deepseek-chat or deepseek-reasoner for R1 */
#define CLOUD_LLM_MODEL        "deepseek-chat"

/* ── ASR (speech-to-text / DashScope Paraformer) ──── */

#define CLOUD_ASR_URL          "https://dashscope.aliyuncs.com/api/v1/services/audio/asr/transcription"
#define CLOUD_ASR_MODEL        "paraformer-v1"
#define CLOUD_ASR_LANGUAGE     "zh"

/* ── TTS (text-to-speech / DashScope CosyVoice) ────── */

#define CLOUD_TTS_URL          "https://dashscope.aliyuncs.com/api/v1/services/audio/tts/synthesis"
#define CLOUD_TTS_MODEL        "cosyvoice-v1"
#define CLOUD_TTS_VOICE        "longxiaochun"
#define CLOUD_TTS_FORMAT       "wav"

/* ── HTTP timeouts (ms) ───────────────────────────── */
#define CLOUD_HTTP_TIMEOUT_MS      20000
#define CLOUD_CONNECT_TIMEOUT_MS   8000
