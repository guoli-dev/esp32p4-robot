#include "waypoint.h"
#include "motion_control.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define TAG "waypoint"
#define MAX_WAYPOINTS 32

/*============================================================================
 * 内部状态
 *============================================================================*/
static waypoint_t s_wps[MAX_WAYPOINTS];
static int        s_count  = 0;
static int        s_index  = 0;    /* 下一个要执行的路径点索引 */
static bool       s_loop   = false;
static bool       s_active = false;

/* WP_WAIT 的非阻塞等待 */
static TickType_t s_wait_until = 0;
static bool       s_is_waiting = false;

/*============================================================================
 * API
 *============================================================================*/

void waypoint_start(const waypoint_t *wps, int count, bool loop)
{
    if (count <= 0 || !wps) {
        ESP_LOGW(TAG, "waypoint_start: empty path");
        return;
    }
    if (count > MAX_WAYPOINTS) {
        ESP_LOGW(TAG, "Truncating %d → %d waypoints", count, MAX_WAYPOINTS);
        count = MAX_WAYPOINTS;
    }

    /* 先停掉当前运动并等待完成 */
    motion_wait();

    memcpy(s_wps, wps, (size_t)count * sizeof(waypoint_t));
    s_count  = count;
    s_index  = 0;
    s_loop   = loop;
    s_active = true;
    s_is_waiting = false;

    ESP_LOGI(TAG, "Path start: %d waypoints, loop=%d", count, (int)loop);

    /* 加载第一个路径点 */
    waypoint_load_next();
}

void waypoint_stop(void)
{
    s_active     = false;
    s_is_waiting = false;
    s_count      = 0;
    s_index      = 0;
    motion_stop();
    ESP_LOGI(TAG, "Path stopped");
}

bool waypoint_is_done(void)
{
    return !s_active;
}

int waypoint_current_index(void)
{
    if (!s_active) return -1;
    /* 返回的是"刚加载的"路径点索引（已递增后的上一个） */
    int idx = s_index - 1;
    if (idx < 0 && s_loop) idx = s_count - 1;
    return idx;
}

/*============================================================================
 * waypoint_load_next — 由 motion_ctrl_task 调用
 *
 * 使用 while 循环跳过 WP_WAIT 和未知类型，避免递归。
 *============================================================================*/
bool waypoint_load_next(void)
{
    if (!s_active) return false;

    /* ── 处理 WP_WAIT（非阻塞定时检查） ── */
    if (s_is_waiting) {
        if (xTaskGetTickCount() < s_wait_until) {
            return true;  /* 仍在等待中，不算结束 */
        }
        s_is_waiting = false;
        /* 等待结束，继续加载下一个路径点 */
    }

    /* 循环查找下一个可执行路径点，跳过 WP_WAIT 和未知类型 */
    while (1) {
        /* ── 检查路径是否穷尽 ── */
        if (s_index >= s_count) {
            if (s_loop) {
                s_index = 0;
                ESP_LOGI(TAG, "Looping: restart from waypoint 0");
            } else {
                s_active = false;
                ESP_LOGI(TAG, "Path complete (%d waypoints done)", s_count);
                return false;
            }
        }

        waypoint_t *wp = &s_wps[s_index];

        /* ── WP_STOP：终止路径 ── */
        if (wp->type == WP_STOP) {
            s_active = false;
            ESP_LOGI(TAG, "WP_STOP: path ended");
            return false;
        }

        /* ── WP_WAIT：设置定时器并继续循环 ── */
        if (wp->type == WP_WAIT) {
            TickType_t ms = (TickType_t)wp->param1;
            if (ms < 1) ms = 1;
            s_wait_until = xTaskGetTickCount() + pdMS_TO_TICKS(ms);
            s_is_waiting = true;
            s_index++;
            ESP_LOGI(TAG, "WP_WAIT %lu ms", (unsigned long)ms);
            return true;  /* 等待中，路径仍活跃 */
        }

        /* ── 运动指令 ── */
        if (wp->type == WP_STRAIGHT || wp->type == WP_TURN || wp->type == WP_ARC) {
            ESP_LOGI(TAG, "[%d/%d] type=%d p1=%.1f p2=%.1f spd=%d",
                     s_index + 1, s_count,
                     (int)wp->type, (double)wp->param1, (double)wp->param2, (int)wp->speed);

            switch (wp->type) {
            case WP_STRAIGHT:
                motion_straight_nowait(wp->param1, wp->speed);
                break;
            case WP_TURN:
                motion_turn_nowait(wp->param1, wp->speed);
                break;
            case WP_ARC:
                motion_arc_nowait(wp->param1, wp->param2, wp->speed);
                break;
            default:
                break;  /* 不可达 — 外层 if 已过滤 */
            }
            s_index++;
            return true;
        }

        /* ── 未知类型：跳过并继续循环 ── */
        ESP_LOGW(TAG, "Unknown waypoint type %d, skipping", (int)wp->type);
        s_index++;
    }
}
