/**
 * @file c5_bridge.c — P4 C5 command receiver (motor + heartbeat)
 *
 * Listens on UART1 for TLV-framed commands from C5.
 * Protocol: [0xA5][type][len_hi][len_lo][payload:N][checksum]
 *
 * C5 motor commands are dispatched directly to P4's motion_control API.
 */

#include "c5_bridge.h"
#include "env_sensor.h"
#include "motion_control.h"
#include "waypoint.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdatomic.h>
#include <string.h>

#define TAG "c5_bridge"

#define FRAME_MAGIC   0xA5
#define CMD_MOTOR     0x10
#define CMD_HEARTBEAT 0xF0
#define MAX_PAYLOAD   256

static volatile int  s_speed = 50;     /* current speed 1-100 */
static atomic_bool    s_link_alive = false;
static volatile TickType_t s_last_hb = 0;

/* ── AI display text (from C5 voice engine) ────────── */
static char            s_display_text[256] = {0};
static volatile bool   s_display_new = false;
static portMUX_TYPE    s_display_lock = portMUX_INITIALIZER_UNLOCKED;

/* ── 预置巡逻路径 (index 0-3) ──────────────────────── */

static const waypoint_t s_patrol_0[] = {   /* 方形巡逻 */
    { WP_STRAIGHT, .param1 = 100,  .speed = 50 },
    { WP_TURN,     .param1 = 90,   .speed = 40 },
    { WP_STRAIGHT, .param1 = 100,  .speed = 50 },
    { WP_TURN,     .param1 = 90,   .speed = 40 },
    { WP_STRAIGHT, .param1 = 100,  .speed = 50 },
    { WP_TURN,     .param1 = 90,   .speed = 40 },
    { WP_STRAIGHT, .param1 = 100,  .speed = 50 },
    { WP_STOP },
};

static const waypoint_t s_patrol_1[] = {   /* Z 字形巡检 */
    { WP_STRAIGHT, .param1 = 80,   .speed = 50 },
    { WP_TURN,     .param1 = 45,   .speed = 40 },
    { WP_STRAIGHT, .param1 = 60,   .speed = 50 },
    { WP_TURN,     .param1 = -45,  .speed = 40 },
    { WP_STRAIGHT, .param1 = 80,   .speed = 50 },
    { WP_STOP },
};

static const waypoint_t s_patrol_2[] = {   /* 长直道往返 */
    { WP_STRAIGHT, .param1 = 200,  .speed = 60 },
    { WP_TURN,     .param1 = 180,  .speed = 40 },
    { WP_STRAIGHT, .param1 = 200,  .speed = 60 },
    { WP_STOP },
};

static const waypoint_t s_patrol_3[] = {   /* 定点采样 */
    { WP_STRAIGHT, .param1 = 50,   .speed = 40 },
    { WP_WAIT,     .param1 = 5000 },
    { WP_STRAIGHT, .param1 = 50,   .speed = 40 },
    { WP_WAIT,     .param1 = 5000 },
    { WP_TURN,     .param1 = 90,   .speed = 30 },
    { WP_STRAIGHT, .param1 = 50,   .speed = 40 },
    { WP_WAIT,     .param1 = 5000 },
    { WP_STOP },
};

static const struct {
    const waypoint_t *wps;
    int count;
} s_patrols[4] = {
    { s_patrol_0, sizeof(s_patrol_0) / sizeof(waypoint_t) },
    { s_patrol_1, sizeof(s_patrol_1) / sizeof(waypoint_t) },
    { s_patrol_2, sizeof(s_patrol_2) / sizeof(waypoint_t) },
    { s_patrol_3, sizeof(s_patrol_3) / sizeof(waypoint_t) },
};

/* ── TLV frame receiver (state machine) ────────────── */

static int recv_frame(uint8_t *payload, uint16_t *len_out)
{
    typedef enum { S_IDLE, S_TYPE, S_LEN_HI, S_LEN_LO, S_PAYLOAD } state_t;
    static state_t s = S_IDLE;
    static uint8_t  s_type;
    static uint16_t s_len;
    static uint8_t  s_buf[MAX_PAYLOAD];
    static uint16_t s_idx;

    uint8_t byte;
    int n, reads = 0;
    while ((n = uart_read_bytes(C5_UART_PORT, &byte, 1, 0)) == 1
           && ++reads <= 32) {   /* cap: prevent watchdog when C5 not connected */
        switch (s) {
        case S_IDLE:
            if (byte == FRAME_MAGIC) s = S_TYPE;
            break;
        case S_TYPE:
            s_type = byte; s = S_LEN_HI;
            break;
        case S_LEN_HI:
            s_len = (uint16_t)byte << 8; s = S_LEN_LO;
            break;
        case S_LEN_LO:
            s_len |= byte;
            if (s_len > MAX_PAYLOAD) { s = S_IDLE; return -1; }
            s_idx = 0;
            if (s_len == 0) goto verify;  /* zero-length payload */
            s = S_PAYLOAD;
            break;
        case S_PAYLOAD:
            s_buf[s_idx++] = byte;
            if (s_idx >= s_len) {
verify:
                /* read checksum byte with short timeout so late-arriving byte isn't lost */
                uint8_t ck; int r = uart_read_bytes(C5_UART_PORT, &ck, 1, pdMS_TO_TICKS(5));
                if (r != 1) { s = S_IDLE; return -1; }

                /* verify checksum */
                uint8_t calc = s_type ^ (uint8_t)(s_len >> 8) ^ (uint8_t)s_len;
                for (uint16_t i = 0; i < s_len; i++) calc ^= s_buf[i];
                s = S_IDLE;

                if (calc != ck) {
                    ESP_LOGW(TAG, "checksum err");
                    return -1;
                }

                uint16_t copy = s_len;
                if (payload && copy > 0)
                    memcpy(payload, s_buf, copy);
                *len_out = copy;
                return s_type;
            }
            break;
        }
    }
    return -1;  /* no complete frame */
}

/* ── ACK sender ────────────────────────────────────── */

static void send_ack(uint8_t ack_type, const uint8_t *data, uint16_t len)
{
    if (len > MAX_PAYLOAD) return;
    uint8_t frame[5 + MAX_PAYLOAD];
    frame[0] = FRAME_MAGIC;
    frame[1] = ack_type;
    frame[2] = (uint8_t)(len >> 8);
    frame[3] = (uint8_t)(len);
    uint8_t ck = ack_type ^ frame[2] ^ frame[3];
    for (uint16_t i = 0; i < len; i++) {
        frame[4 + i] = data[i];
        ck ^= data[i];
    }
    frame[4 + len] = ck;
    uart_write_bytes(C5_UART_PORT, (const char *)frame, 5 + len);
}

/* ── Motor command dispatcher ──────────────────────── */

static void dispatch_motor(const uint8_t *payload, uint16_t len)
{
    if (len < 1) return;

    uint8_t cmd = payload[0];
    int spd = s_speed;

    switch (cmd) {
    case C5_MOTOR_FORWARD:
        ESP_LOGI(TAG, "C5→P4: FORWARD @ %d%%", spd);
        motion_straight(30.0f, (int16_t)spd);
        break;
    case C5_MOTOR_BACKWARD:
        ESP_LOGI(TAG, "C5→P4: BACKWARD @ %d%%", spd);
        motion_straight(-30.0f, (int16_t)spd);
        break;
    case C5_MOTOR_TURN_LEFT:
        ESP_LOGI(TAG, "C5→P4: TURN LEFT @ %d%%", spd);
        motion_turn(90.0f, (int16_t)spd);
        break;
    case C5_MOTOR_TURN_RIGHT:
        ESP_LOGI(TAG, "C5→P4: TURN RIGHT @ %d%%", spd);
        motion_turn(-90.0f, (int16_t)spd);
        break;
    case C5_MOTOR_STOP:
        ESP_LOGI(TAG, "C5→P4: STOP");
        motion_stop();
        break;
    case C5_MOTOR_SET_SPEED:
        if (len >= 2) {
            spd = (int)payload[1];
            if (spd < 1) spd = 1;
            if (spd > 100) spd = 100;
            s_speed = spd;
            ESP_LOGI(TAG, "C5→P4: SPEED = %d%%", spd);
        }
        break;
    default:
        ESP_LOGW(TAG, "unknown motor cmd: 0x%02X", cmd);
        return;
    }

    /* send ACK */
    uint8_t ack[] = { cmd };
    send_ack(0x11 /* MOTOR_ACK */, ack, 1);
}

/* ── Navigation command dispatcher ──────────────────── */

static void dispatch_nav(const uint8_t *payload, uint16_t len)
{
    if (len < 1) return;

    uint8_t cmd = payload[0];

    switch (cmd) {
    case CMD_NAV_STOP:
        ESP_LOGI(TAG, "C5→P4: NAV_STOP");
        waypoint_stop();
        motion_stop();
        break;

    case CMD_NAV_PATH:
        if (len >= 2) {
            int idx = (int)payload[1];
            if (idx >= 0 && idx < 4) {
                ESP_LOGI(TAG, "C5→P4: NAV_PATH %d (%d waypoints)", idx, s_patrols[idx].count);
                motion_wait();  /* wait for current motion to finish */
                waypoint_start(s_patrols[idx].wps, s_patrols[idx].count, false);
            }
        }
        break;

    case CMD_NAV_STRAIGHT:
        if (len >= 6) {
            /* payload: [cmd(1)][dist(float:4)][speed(u8)] */
            float dist;
            memcpy(&dist, payload + 1, 4);
            uint8_t spd = payload[5];
            if (spd < 1) spd = (uint8_t)s_speed;
            ESP_LOGI(TAG, "C5→P4: NAV_STRAIGHT %.1fcm @ %d%%", (double)dist, (int)spd);
            motion_straight(dist, (int16_t)spd);
        }
        break;

    case CMD_NAV_TURN:
        if (len >= 6) {
            float angle;
            memcpy(&angle, payload + 1, 4);
            uint8_t spd = payload[5];
            if (spd < 1) spd = (uint8_t)s_speed;
            ESP_LOGI(TAG, "C5→P4: NAV_TURN %.1fdeg @ %d%%", (double)angle, (int)spd);
            motion_turn(angle, (int16_t)spd);
        }
        break;

    default:
        break;
    }
}

/* ── Command listener task ─────────────────────────── */

static void c5_listener_task(void *arg)
{
    ESP_LOGI(TAG, "listener task start (UART%d GPIO%d/%d)",
             C5_UART_PORT, C5_UART_TX, C5_UART_RX);

    while (1) {
        uint8_t payload[MAX_PAYLOAD];
        uint16_t len = 0;
        int type = recv_frame(payload, &len);

        if (type == CMD_MOTOR) {
            dispatch_motor(payload, len);
        } else if (type == CMD_NAV_PATH || type == CMD_NAV_STRAIGHT ||
                   type == CMD_NAV_TURN || type == CMD_NAV_STOP) {
            dispatch_nav(payload, len);
        } else if (type == CMD_TEXT_DISPLAY) {
            /* AI speech text from C5 — store for LCD display */
            if (len > 0) {
                portENTER_CRITICAL(&s_display_lock);
                size_t copy = (len < sizeof(s_display_text) - 1) ? len : (sizeof(s_display_text) - 1);
                memcpy(s_display_text, payload, copy);
                s_display_text[copy] = '\0';
                s_display_new = true;
                portEXIT_CRITICAL(&s_display_lock);
                ESP_LOGI(TAG, "AI: '%s'", s_display_text);
            }
        } else if (type == CMD_HEARTBEAT) {
            s_last_hb = xTaskGetTickCount();
            atomic_store(&s_link_alive, true);
            send_ack(0xF1 /* HEARTBEAT_ACK */, NULL, 0);
        } else if (type >= 0) {
            ESP_LOGD(TAG, "unknown frame type 0x%02X len=%u", type, len);
        }

        vTaskDelay(pdMS_TO_TICKS(50));  /* ~20Hz poll, Idle-safe when C5 offline */
    }
}

/* ── Public API ────────────────────────────────────── */

void c5_bridge_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = C5_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_driver_install(C5_UART_PORT, 512, 512, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(C5_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(C5_UART_PORT, C5_UART_TX, C5_UART_RX,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    s_last_hb = xTaskGetTickCount();

    xTaskCreatePinnedToCore(c5_listener_task, "c5_listener", 3072,
                            NULL, 4, NULL, 0);  /* Core 0, prio 4 */

    ESP_LOGI(TAG, "init OK — waiting for C5 commands");
}

bool c5_bridge_link_alive(void)
{
    TickType_t elapsed = xTaskGetTickCount() - s_last_hb;
    if ((elapsed * portTICK_PERIOD_MS) > 10000) {
        atomic_store(&s_link_alive, false);
    }
    return atomic_load(&s_link_alive);
}

int c5_bridge_get_speed(void)
{
    return s_speed;
}

/* ── Vision result sender (legacy) ──────────────────── */

void c5_bridge_send_vision(const void *vision_result)
{
    const uint8_t *data = (const uint8_t *)vision_result;
    #define VISION_PAYLOAD_SIZE  48
    send_ack(CMD_VISION_RESULT, data, VISION_PAYLOAD_SIZE);
}

/* ── Obstacle alert sender ─────────────────────────── */

void c5_bridge_send_obstacle(uint8_t level, uint16_t range_mm)
{
    uint8_t payload[4];
    payload[0] = level;
    payload[1] = 0x00;  /* reserved */
    payload[2] = (uint8_t) range_mm;
    payload[3] = (uint8_t)(range_mm >> 8);

    send_ack(CMD_OBSTACLE, payload, 4);
    ESP_LOGI(TAG, "→ C5: OBSTACLE level=%u range=%umm",
             (unsigned)level, (unsigned)range_mm);
}

/* ── Speech recognition event senders ──────────────── */

void c5_bridge_send_wake(void)
{
    uint8_t wake_id = 0x01;   /* wake word ID: 1 = "小智" */
    send_ack(CMD_WAKE_EVENT, &wake_id, 1);
    ESP_LOGI(TAG, "→ C5: WAKE_EVENT");
}

void c5_bridge_send_local_cmd(const char *cmd_text)
{
    if (!cmd_text) return;
    size_t len = strlen(cmd_text);
    if (len > 64) len = 64;   /* cap at 64 bytes */
    send_ack(CMD_LOCAL_COMMAND, (const uint8_t *)cmd_text, (uint16_t)len);
    ESP_LOGI(TAG, "→ C5: LOCAL_CMD '%s'", cmd_text);
}

/* ── Environmental data sender ─────────────────────── */

void c5_bridge_send_env(const void *env_data)
{
    if (!env_data) return;

    /* Use struct directly (safer than pointer arithmetic) */
    const env_data_t *e = (const env_data_t *)env_data;

    uint8_t payload[13];
    int16_t t_raw = (int16_t)(e->temp_c * 10.0f);
    uint16_t h_raw = (uint16_t)(e->humidity_pct * 10.0f);

    payload[0]  = (uint8_t)(t_raw >> 8);
    payload[1]  = (uint8_t)(t_raw);
    payload[2]  = (uint8_t)(h_raw >> 8);
    payload[3]  = (uint8_t)(h_raw);
    payload[4]  = (uint8_t)(e->mq135_raw >> 8);
    payload[5]  = (uint8_t)(e->mq135_raw);
    payload[6]  = e->mq135_hazard;
    payload[7]  = (uint8_t)(e->mq136_raw >> 8);
    payload[8]  = (uint8_t)(e->mq136_raw);
    payload[9]  = e->mq136_hazard;
    payload[10] = 0x00;  /* reserved */
    payload[11] = 0x00;  /* reserved */
    payload[12] = e->air_quality;

    send_ack(CMD_ENV_DATA, payload, 13);
    ESP_LOGI(TAG, "→ C5: ENV T=%.1f H=%.1f 135=%u/%u 136=%u/%u AQ=%u",
             (double)e->temp_c, (double)e->humidity_pct,
             (unsigned)e->mq135_raw, (unsigned)e->mq135_hazard,
             (unsigned)e->mq136_raw, (unsigned)e->mq136_hazard,
             (unsigned)e->air_quality);
}

/* ── Speed setter ─────────────────────────────────────── */

void c5_bridge_set_speed(int pct)
{
    if (pct < 1) pct = 1;
    if (pct > 100) pct = 100;
    s_speed = pct;
    ESP_LOGI(TAG, "speed set to %d%% (voice)", pct);
}

/* ── Patrol starter ────────────────────────────────────── */

void c5_bridge_start_patrol(int idx)
{
    if (idx < 0 || idx >= 4) {
        ESP_LOGW(TAG, "patrol index %d out of range", idx);
        return;
    }
    ESP_LOGI(TAG, "voice patrol %d (%d waypoints)", idx, s_patrols[idx].count);
    motion_wait();  /* wait for current motion to finish */
    waypoint_start(s_patrols[idx].wps, s_patrols[idx].count, false);
}

/* ── AI display text ────────────────────────────────── */

const char *c5_bridge_get_display_text(void)
{
    if (!s_display_new) return NULL;
    return s_display_text;
}

void c5_bridge_display_ack(void)
{
    s_display_new = false;
}
