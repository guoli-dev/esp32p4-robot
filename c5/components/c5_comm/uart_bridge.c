/**
 * @file uart_bridge.c — C5 UART0 ↔ P4
 *
 * Supports both legacy 2-byte TTS protocol and new framed TLV protocol.
 */

#include "uart_bridge.h"
#include "p4_protocol.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

#define TAG "uart"

void uart_p4_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = C5_P4_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_driver_install(C5_P4_UART, 512, 512, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(C5_P4_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(C5_P4_UART, C5_P4_TX_PIN,
                                  C5_P4_RX_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "P4 UART init OK  TX=GPIO%d RX=GPIO%d @%d",
             C5_P4_TX_PIN, C5_P4_RX_PIN, C5_P4_BAUD);
}

bool uart_p4_send(const uint8_t *data, size_t len)
{
    int sent = uart_write_bytes(C5_P4_UART, (const char *)data, len);
    return (sent > 0);
}

int uart_p4_read(uint8_t *buf, size_t max_len)
{
    int len = uart_read_bytes(C5_P4_UART, buf, max_len, 0);
    return (len > 0) ? len : 0;
}

/* ── Framed protocol ──────────────────────────────── */

bool uart_p4_send_frame(uint8_t cmd_type, const uint8_t *payload, uint16_t len)
{
    if (len > P4_FRAME_MAX_PAYLOAD) return false;

    /* build frame */
    uint8_t frame[4 + P4_FRAME_MAX_PAYLOAD + 1];  /* header + payload + checksum */
    frame[0] = P4_FRAME_MAGIC;
    frame[1] = cmd_type;
    frame[2] = (uint8_t)(len >> 8);   /* length hi */
    frame[3] = (uint8_t)(len);        /* length lo */

    /* checksum */
    uint8_t cksum = cmd_type ^ frame[2] ^ frame[3];
    for (uint16_t i = 0; i < len; i++) {
        frame[4 + i] = payload[i];
        cksum ^= payload[i];
    }
    frame[4 + len] = cksum;

    int sent = uart_write_bytes(C5_P4_UART, (const char *)frame, 5 + len);
    return (sent == (int)(5 + len));
}

uint8_t uart_p4_recv_frame(uint8_t *payload, uint16_t *len, uint16_t max_len)
{
    typedef enum {
        FS_IDLE,
        FS_GOT_MAGIC,
        FS_GOT_TYPE,
        FS_GOT_LEN_HI,
        FS_GOT_LEN_LO,   /* length fully known, reading payload next */
    } frame_state_t;

    static frame_state_t s_fs = FS_IDLE;
    static uint8_t  s_type;
    static uint16_t s_len;
    static uint8_t  s_buf[P4_FRAME_MAX_PAYLOAD];
    static uint16_t s_idx;

    uint8_t byte;
    int n;
    while ((n = uart_read_bytes(C5_P4_UART, &byte, 1, 0)) == 1) {
        switch (s_fs) {
        case FS_IDLE:
            if (byte == P4_FRAME_MAGIC) {
                s_fs = FS_GOT_MAGIC;
            }
            break;

        case FS_GOT_MAGIC:
            s_type = byte;
            s_fs = FS_GOT_TYPE;
            break;

        case FS_GOT_TYPE:
            s_len = (uint16_t)byte << 8;
            s_fs = FS_GOT_LEN_HI;
            break;

        case FS_GOT_LEN_HI:
            s_len |= byte;
            if (s_len > P4_FRAME_MAX_PAYLOAD) {
                ESP_LOGW(TAG, "frame too long: %u", s_len);
                s_fs = FS_IDLE;
                return 0xFF;
            }
            s_idx = 0;
            s_fs = FS_GOT_LEN_LO;
            /* if zero-length payload, read checksum directly */
            if (s_len == 0) goto verify_checksum;
            break;

        case FS_GOT_LEN_LO:
            /* byte is a payload byte */
            s_buf[s_idx++] = byte;
            if (s_idx >= s_len) {
verify_checksum:
            {
                /* read checksum byte */
                uint8_t ck;
                int r = uart_read_bytes(C5_P4_UART, &ck, 1, 0);
                if (r != 1) { s_fs = FS_IDLE; return 0; }

                /* verify */
                uint8_t calc = s_type ^ (uint8_t)(s_len >> 8) ^ (uint8_t)s_len;
                for (uint16_t i = 0; i < s_len; i++) calc ^= s_buf[i];
                s_fs = FS_IDLE;

                if (calc != ck) {
                    ESP_LOGW(TAG, "frame checksum err");
                    return 0xFF;
                }

                uint16_t copy = s_len;
                if (copy > max_len) copy = max_len;
                if (payload && copy > 0) {
                    for (uint16_t i = 0; i < copy; i++) payload[i] = s_buf[i];
                }
                if (len) *len = copy;
                return s_type;
            }
            }
            break;
        }
    }
    return 0;  /* no complete frame */
}

/* ── Environment data cache (JSON string + numeric) ── */

static char  s_env_cache[192] = {0};
static bool  s_env_ok = false;

/* Numeric cache — written atomically by fields, no lock needed for RISC-V */
static env_cache_num_t s_env_num = {0};

void uart_env_cache_set(const char *json)
{
    if (!json) return;
    strncpy(s_env_cache, json, sizeof(s_env_cache) - 1);
    s_env_cache[sizeof(s_env_cache) - 1] = '\0';
    s_env_ok = true;

    /* Also populate numeric cache by parsing JSON fields */
    float t = 0, h = 0;
    unsigned aq = 0, haz135 = 0, haz136 = 0;
    unsigned mq135r = 0, mq136r = 0;
    int n = sscanf(json, "{\"t\":%f,\"h\":%f,\"mq135\":%u,\"haz135\":%u,\"mq136\":%u,\"haz136\":%u,\"aq\":%u}",
                   &t, &h, &mq135r, &haz135, &mq136r, &haz136, &aq);
    if (n >= 3) {
        s_env_num.temp_c       = t;
        s_env_num.humidity_pct = h;
        s_env_num.mq135_raw    = (uint16_t)mq135r;
        s_env_num.mq135_hazard = (uint8_t)haz135;
        s_env_num.mq136_raw    = (uint16_t)mq136r;
        s_env_num.mq136_hazard = (uint8_t)haz136;
        s_env_num.air_quality  = (uint8_t)aq;
    }
}

const char *uart_env_cache_get(void)
{
    return s_env_ok ? s_env_cache : NULL;
}

const env_cache_num_t *uart_env_num_get(void)
{
    return s_env_ok ? &s_env_num : NULL;
}

bool uart_env_has_data(void)
{
    return s_env_ok;
}
