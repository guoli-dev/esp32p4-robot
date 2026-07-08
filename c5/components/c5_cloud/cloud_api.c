/**
 * @file cloud_api.c — Cloud API client (OpenAI-compatible + DashScope)
 *
 * Handles HTTPS calls to LLM Chat (DeepSeek), ASR (DashScope Paraformer),
 * and TTS (DashScope CosyVoice) endpoints.
 */

#include "cloud_api.h"
#include "cloud_config.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "mbedtls/base64.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define TAG "cloud"

/* ── internal state ───────────────────────────────── */
static bool s_ready = false;

/* ── NVS keys ─────────────────────────────────────── */
#define NVS_NS_CLOUD   "cloud_api"
#define NVS_KEY_LLM    "llm_key"
#define NVS_KEY_OPENAI "openai_key"

void cloud_api_init(void)
{
    ESP_LOGI(TAG, "cloud API init (cert bundle for TLS)");
    nvs_handle_t handle;
    char nvs_key[128] = {0};
    size_t len = sizeof(nvs_key);
    if (nvs_open(NVS_NS_CLOUD, NVS_READONLY, &handle) == ESP_OK) {
        if (nvs_get_str(handle, NVS_KEY_LLM, nvs_key, &len) == ESP_OK) {
            ESP_LOGI(TAG, "using NVS API key");
        }
        nvs_close(handle);
    }
    const char *active_key = nvs_key[0] ? nvs_key : CLOUD_LLM_API_KEY;
    if (active_key && active_key[0] && strcmp(active_key, "sk-YOUR-KEY-HERE") != 0) {
        s_ready = true;
    }
    ESP_LOGI(TAG, "init OK (ready=%d)", s_ready);
}

void cloud_api_set_key(const char *provider, const char *key)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NS_CLOUD, NVS_READWRITE, &handle) != ESP_OK) return;
    if (strcmp(provider, "llm") == 0) {
        nvs_set_str(handle, NVS_KEY_LLM, key);
    } else if (strcmp(provider, "openai") == 0) {
        nvs_set_str(handle, NVS_KEY_OPENAI, key);
    }
    nvs_commit(handle);
    nvs_close(handle);
    s_ready = true;
    ESP_LOGI(TAG, "key saved for '%s'", provider);
}

bool cloud_api_is_ready(void) { return s_ready; }

/* ── minimal JSON extractor ────────────────────────── */

static int json_extract_last_string(const char *json, const char *key,
                                     char *value, size_t max_len)
{
    char search[64];
    snprintf(search, sizeof(search), "\"%s\"", key);
    const char *last = NULL;
    const char *scan = json;
    while ((scan = strstr(scan, search)) != NULL) {
        const char *colon = scan + strlen(search);
        while (*colon == ' ') colon++;
        if (*colon == ':') {
            const char *q = colon + 1;
            while (*q == ' ') q++;
            if (*q == '"') last = scan;
        }
        scan++;
    }
    if (!last) {
        last = strstr(json, search);
        if (!last) return -1;
    }
    const char *pos = last + strlen(search);
    pos = strchr(pos, ':');
    if (!pos) return -1;
    pos = strchr(pos, '"');
    if (!pos) return -1;
    pos++;
    size_t i = 0;
    while (*pos && *pos != '"' && i < max_len - 1) {
        if (*pos == '\\' && *(pos + 1)) {
            pos++;
            switch (*pos) {
            case 'n':  value[i++] = '\n'; break;
            case 't':  value[i++] = '\t'; break;
            case '"':  value[i++] = '"';  break;
            case '\\': value[i++] = '\\'; break;
            case 'u':  value[i++] = '?';  pos += 4; break;
            default:   value[i++] = *pos; break;
            }
        } else { value[i++] = *pos; }
        pos++;
    }
    value[i] = '\0';
    return (int)i;
}

/* ── LLM Chat ─────────────────────────────────────── */

int cloud_llm_chat(const char *user_message, const char *system_prompt,
                   char *response_buf, size_t *buf_size)
{
    if (!user_message || !response_buf || !buf_size || *buf_size == 0)
        return -1;

    const char *api_key = CLOUD_LLM_API_KEY;
    nvs_handle_t handle;
    static char nvs_key[128] = {0};
    size_t klen = sizeof(nvs_key);
    if (nvs_open(NVS_NS_CLOUD, NVS_READONLY, &handle) == ESP_OK) {
        if (nvs_get_str(handle, NVS_KEY_LLM, nvs_key, &klen) == ESP_OK && nvs_key[0])
            api_key = nvs_key;
        nvs_close(handle);
    }

    const char *default_sys = "You are a small robot assistant. "
        "Respond in Chinese with short, natural speech under 50 characters. "
        "If the user wants you to move, include commands in your response "
        "using [MOTOR:action] tags where action is forward/backward/left/right/stop. "
        "Keep responses concise and friendly.";
    if (!system_prompt) system_prompt = default_sys;

    char body[2048];
    int body_len = snprintf(body, sizeof(body),
        "{\"model\":\"%s\",\"messages\":["
        "{\"role\":\"system\",\"content\":\"%s\"},"
        "{\"role\":\"user\",\"content\":\"%s\"}],"
        "\"max_tokens\":150,\"temperature\":0.7}",
        CLOUD_LLM_MODEL, system_prompt, user_message);
    if (body_len < 0 || body_len >= (int)sizeof(body)) {
        ESP_LOGE(TAG, "JSON body too long"); return -1;
    }

    esp_http_client_config_t http_cfg = {
        .url = CLOUD_LLM_URL, .method = HTTP_METHOD_POST,
        .timeout_ms = CLOUD_HTTP_TIMEOUT_MS,
        .skip_cert_common_name_check = true,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .buffer_size = 1024, .buffer_size_tx = 1024,
        .keep_alive_enable = false, .disable_auto_redirect = false,
    };
    esp_http_client_handle_t client = esp_http_client_init(&http_cfg);
    if (!client) { ESP_LOGE(TAG, "http init fail"); return -1; }

    char auth_header[256];
    snprintf(auth_header, sizeof(auth_header), "Bearer %s", api_key);
    esp_http_client_set_header(client, "Authorization", auth_header);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, body, body_len);

    ESP_LOGI(TAG, "LLM request... (%d bytes)", body_len);

    esp_err_t err = esp_http_client_open(client, body_len);
    if (err != ESP_OK) { ESP_LOGE(TAG, "open err: %d", err); goto fail; }
    int wrote = esp_http_client_write(client, body, body_len);
    if (wrote < 0) { ESP_LOGE(TAG, "write err: %d", wrote); goto fail2; }

    int content_len = esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    ESP_LOGI(TAG, "LLM: HTTP %d, len=%d", status, content_len);
    if (status != 200) {
        char err_buf[512] = {0};
        esp_http_client_read(client, err_buf, sizeof(err_buf) - 1);
        ESP_LOGE(TAG, "LLM err: '%s'", err_buf);
        goto fail2;
    }

    char resp[4096] = {0};
    int total = 0;
    while (total < (int)sizeof(resp) - 1) {
        int n = esp_http_client_read(client, resp + total, sizeof(resp) - total - 1);
        if (n <= 0) break;
        total += n;
    }
    resp[total] = '\0';
    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    char content[1024] = {0};
    if (json_extract_last_string(resp, "content", content, sizeof(content)) > 0) {
        size_t copy = strlen(content);
        if (copy >= *buf_size) copy = *buf_size - 1;
        memcpy(response_buf, content, copy);
        response_buf[copy] = '\0'; *buf_size = copy;
        ESP_LOGI(TAG, "LLM reply: '%s'", response_buf);
        return 0;
    }
    ESP_LOGW(TAG, "parse fail, raw: '%s'", resp);
    size_t raw = strlen(resp); if (raw >= *buf_size) raw = *buf_size - 1;
    memcpy(response_buf, resp, raw); response_buf[raw] = '\0'; *buf_size = raw;
    return 0;

fail2: esp_http_client_close(client);
fail:  esp_http_client_cleanup(client); return -1;
}

/* ── WAV header builder ───────────────────────────── */

static int make_wav_header(uint8_t *buf, int sample_rate,
                            int bits_per_sample, int num_samples)
{
    int byte_rate = sample_rate * (bits_per_sample / 8);
    int data_size = num_samples * (bits_per_sample / 8);
    memcpy(buf, "RIFF", 4);
    uint32_t file_size = 36 + data_size; memcpy(buf + 4, &file_size, 4);
    memcpy(buf + 8, "WAVE", 4);
    memcpy(buf + 12, "fmt ", 4);
    uint32_t fmt_size = 16; memcpy(buf + 16, &fmt_size, 4);
    uint16_t audio_fmt = 1; memcpy(buf + 20, &audio_fmt, 2);
    uint16_t channels = 1; memcpy(buf + 22, &channels, 2);
    memcpy(buf + 24, &sample_rate, 4);
    memcpy(buf + 28, &byte_rate, 4);
    uint16_t block_align = (bits_per_sample / 8) * channels;
    memcpy(buf + 32, &block_align, 2);
    memcpy(buf + 34, &bits_per_sample, 2);
    memcpy(buf + 36, "data", 4);
    memcpy(buf + 40, &data_size, 4);
    return 44;
}

/* ── helper: read NVS key for ASR/TTS ─────────────── */

static const char *load_openai_key(void)
{
    static char buf[128] = {0};
    nvs_handle_t h;
    if (nvs_open(NVS_NS_CLOUD, NVS_READONLY, &h) == ESP_OK) {
        size_t len = sizeof(buf);
        if (nvs_get_str(h, NVS_KEY_OPENAI, buf, &len) == ESP_OK && buf[0])
            { nvs_close(h); return buf; }
        nvs_close(h);
    }
    return CLOUD_OPENAI_API_KEY;
}

/* ── ASR (DashScope Paraformer) ───────────────────── */

int cloud_asr_transcribe(const int16_t *audio, size_t sample_count,
                         char *text_buf, size_t text_size)
{
    if (!audio || sample_count == 0 || !text_buf || text_size == 0) return -1;
    text_buf[0] = '\0';

    const char *api_key = load_openai_key();
    if (!api_key || !api_key[0]) { ESP_LOGW(TAG, "ASR: no key"); return -1; }

    /* Build WAV */
    size_t pcm_bytes = sample_count * sizeof(int16_t);
    size_t wav_total = 44 + pcm_bytes;
    uint8_t *wav_buf = malloc(wav_total);
    if (!wav_buf) { ESP_LOGE(TAG, "ASR: alloc fail"); return -1; }
    make_wav_header(wav_buf, 16000, 16, (int)sample_count);
    memcpy(wav_buf + 44, audio, pcm_bytes);

    /* Base64 encode WAV */
    size_t b64_len = 4 * ((wav_total + 2) / 3) + 1;
    char *b64 = malloc(b64_len);
    if (!b64) { free(wav_buf); return -1; }
    size_t b64_actual = 0;
    mbedtls_base64_encode((unsigned char *)b64, b64_len, &b64_actual, wav_buf, wav_total);
    free(wav_buf);

    /* JSON: {"model":"paraformer-v1","input":{"audio":"<b64>"}} */
    size_t json_len = 256 + b64_actual;
    char *json_body = malloc(json_len);
    if (!json_body) { free(b64); return -1; }
    int body_len = snprintf(json_body, json_len,
        "{\"model\":\"%s\",\"input\":{\"audio\":\"%s\"}}", CLOUD_ASR_MODEL, b64);
    free(b64);

    esp_http_client_config_t http_cfg = {
        .url = CLOUD_ASR_URL, .method = HTTP_METHOD_POST,
        .timeout_ms = CLOUD_HTTP_TIMEOUT_MS,
        .skip_cert_common_name_check = true,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .keep_alive_enable = false,
    };
    esp_http_client_handle_t client = esp_http_client_init(&http_cfg);
    if (!client) { free(json_body); return -1; }

    char auth[256]; snprintf(auth, sizeof(auth), "Bearer %s", api_key);
    esp_http_client_set_header(client, "Authorization", auth);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    ESP_LOGI(TAG, "ASR req... (%d bytes)", body_len);

    if (esp_http_client_open(client, body_len) != ESP_OK) {
        ESP_LOGE(TAG, "ASR open err"); free(json_body); esp_http_client_cleanup(client); return -1;
    }
    esp_http_client_write(client, json_body, body_len);
    free(json_body);
    esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);

    char resp[2048] = {0}; int total = 0;
    while (total < (int)sizeof(resp) - 1) {
        int n = esp_http_client_read(client, resp + total, sizeof(resp) - total - 1);
        if (n <= 0) break;
        total += n;
    }
    resp[total] = '\0';
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    ESP_LOGI(TAG, "ASR: HTTP %d (%d bytes)", status, total);

    if (status != 200) { ESP_LOGE(TAG, "ASR err: '%s'", resp); return -1; }

    char asr_text[1024] = {0};
    if (json_extract_last_string(resp, "text", asr_text, sizeof(asr_text)) > 0 && asr_text[0]) {
        size_t c = strlen(asr_text); if (c >= text_size) c = text_size - 1;
        memcpy(text_buf, asr_text, c); text_buf[c] = '\0';
        ESP_LOGI(TAG, "ASR: '%s'", text_buf);
        return 0;
    }
    ESP_LOGW(TAG, "ASR parse fail: '%s'", resp);
    return -1;
}

/* ── TTS (DashScope CosyVoice) ────────────────────── */

int cloud_tts_synthesize(const char *text, cloud_tts_callback_t callback)
{
    if (!text || !text[0] || !callback) return -1;

    const char *api_key = load_openai_key();
    if (!api_key || !api_key[0]) { ESP_LOGW(TAG, "TTS: no key"); return -1; }

    char body[1024];
    int body_len = snprintf(body, sizeof(body),
        "{\"model\":\"%s\",\"input\":{\"text\":\"%s\"},"
        "\"parameters\":{\"format\":\"%s\",\"sample_rate\":16000}}",
        CLOUD_TTS_MODEL, text, CLOUD_TTS_FORMAT);
    if (body_len < 0 || body_len >= (int)sizeof(body)) { ESP_LOGE(TAG, "TTS body"); return -1; }

    esp_http_client_config_t http_cfg = {
        .url = CLOUD_TTS_URL, .method = HTTP_METHOD_POST,
        .timeout_ms = CLOUD_HTTP_TIMEOUT_MS,
        .skip_cert_common_name_check = true,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .buffer_size = 4096, .keep_alive_enable = false,
    };
    esp_http_client_handle_t client = esp_http_client_init(&http_cfg);
    if (!client) return -1;

    char auth[256]; snprintf(auth, sizeof(auth), "Bearer %s", api_key);
    esp_http_client_set_header(client, "Authorization", auth);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    ESP_LOGI(TAG, "TTS req... (%d bytes)", body_len);

    if (esp_http_client_open(client, body_len) != ESP_OK) { esp_http_client_cleanup(client); return -1; }
    esp_http_client_write(client, body, body_len);
    esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    ESP_LOGI(TAG, "TTS: HTTP %d", status);

    if (status != 200) {
        char err_buf[512] = {0};
        esp_http_client_read(client, err_buf, sizeof(err_buf) - 1);
        ESP_LOGE(TAG, "TTS err: '%s'", err_buf);
        esp_http_client_close(client); esp_http_client_cleanup(client); return -1;
    }

    /* Read JSON response — heap allocated, sized for ~1s TTS (short replies).
     * 1s WAV ≈ 32KB → base64 ≈ 43KB → JSON ≈ 44KB.  64KB gives headroom. */
    size_t resp_cap = 65536;
    char *resp = malloc(resp_cap);
    if (!resp) { ESP_LOGE(TAG, "TTS: alloc resp fail");
        esp_http_client_close(client); esp_http_client_cleanup(client); return -1; }
    int total = 0;
    while (total < (int)resp_cap - 1) {
        int n = esp_http_client_read(client, resp + total, resp_cap - total - 1);
        if (n <= 0) break;
        total += n;
    }
    resp[total] = '\0';
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    ESP_LOGI(TAG, "TTS resp: %d bytes", total);

    /* Extract base64 audio — 48KB covers ~1.2s TTS */
    size_t b64_cap = 49152;
    char *b64_audio = malloc(b64_cap);
    if (!b64_audio) { ESP_LOGE(TAG, "TTS: alloc b64 fail"); free(resp); return -1; }
    if (json_extract_last_string(resp, "audio", b64_audio, b64_cap) <= 0 || !b64_audio[0]) {
        ESP_LOGW(TAG, "TTS: no audio in response (first 256 bytes): '%.256s'", resp);
        free(b64_audio); free(resp); return -1;
    }

    /* Free resp EARLY — we no longer need the HTTP response after base64 extraction.
     * This reduces peak simultaneous heap from ~150KB to ~85KB. */
    free(resp);

    /* Decode base64 → WAV bytes */
    size_t b64_len = strlen(b64_audio);
    size_t wav_max = 3 * b64_len / 4 + 4;
    uint8_t *wav = malloc(wav_max);
    if (!wav) { ESP_LOGE(TAG, "TTS: alloc wav fail"); free(b64_audio); return -1; }
    size_t wav_len = 0;
    mbedtls_base64_decode(wav, wav_max, &wav_len, (unsigned char *)b64_audio, b64_len);
    free(b64_audio);

    /* Strip WAV header (44 bytes), feed PCM to callback */
    if (wav_len > 44) {
        int16_t *pcm = (int16_t *)(wav + 44);
        size_t pcm_samples = (wav_len - 44) / sizeof(int16_t);
        callback(pcm, pcm_samples);
    }
    free(wav);
    ESP_LOGI(TAG, "TTS done (%u bytes PCM)", (unsigned)(wav_len > 44 ? wav_len - 44 : 0));
    return 0;
}
