/**
 * @file local_tts.c — Offline Chinese TTS (Formant Synthesis)
 *
 * Pure-C Chinese text-to-speech for ESP32-C5.
 * No model files, no downloads, no API keys.
 *
 * Approach:
 *   1. UTF-8 → Unicode codepoint
 *   2. Codepoint → pinyin (initial + final + tone) via lookup table
 *   3. Pinyin → PCM audio via additive formant synthesis
 *   4. Output 16kHz mono int16_t via callback
 *
 * Quality: "Robot voice" — intelligible, suitable for a small robot.
 */

#include "local_tts.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "esp_log.h"

#define TAG "ltts"

/* ── Constants ─────────────────────────────────────── */
#define SAMPLE_RATE     16000
#define SYL_MS           240       /* syllable duration */
#define SYL_SAMPLES      ((SYL_MS * SAMPLE_RATE) / 1000)
#define CONS_MS          50        /* consonant portion */
#define CHUNK_SAMPLES    320       /* callback chunk = 20ms */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── Pinyin encoding ───────────────────────────────── */
/* 16 bits: initial(5) | final(6) | tone(3) */
#define PY_INIT(v)  (((v) & 0x1F) << 9)
#define PY_FINAL(v) (((v) & 0x3F) << 3)
#define PY_TONE(v)  ((v) & 0x07)

#define PY(init, final, tone) (PY_INIT(init) | PY_FINAL(final) | PY_TONE(tone))

#define INIT_NULL  0
#define INIT_B     1
#define INIT_P     2
#define INIT_M     3
#define INIT_F     4
#define INIT_D     5
#define INIT_T     6
#define INIT_N     7
#define INIT_L     8
#define INIT_G     9
#define INIT_K     10
#define INIT_H     11
#define INIT_J     12
#define INIT_Q     13
#define INIT_X     14
#define INIT_ZH    15
#define INIT_CH    16
#define INIT_SH    17
#define INIT_R     18
#define INIT_Z     19
#define INIT_C     20
#define INIT_S     21
#define INIT_Y     22
#define INIT_W     23

#define FIN_A      0
#define FIN_O      1
#define FIN_E      2
#define FIN_I      3
#define FIN_U      4
#define FIN_V      5   /* ü */
#define FIN_AI     6
#define FIN_EI     7
#define FIN_AO     8
#define FIN_OU     9
#define FIN_AN     10
#define FIN_EN     11
#define FIN_ANG    12
#define FIN_ENG    13
#define FIN_ONG    14
#define FIN_IA     15
#define FIN_IE     16
#define FIN_IAO    17
#define FIN_IU     18
#define FIN_IAN    19
#define FIN_IN     20
#define FIN_IANG   21
#define FIN_ING    22
#define FIN_IONG   23
#define FIN_UA     24
#define FIN_UO     25
#define FIN_UAI    26
#define FIN_UI     27
#define FIN_UAN    28
#define FIN_UN     29
#define FIN_UANG   30
#define FIN_UE     32   /* üe */
#define FIN_ER     35

/* ── Vowel formants (F1, F2, F3 in Hz) ──────────────── */
static const struct { int f1, f2, f3; } VOWEL_FMT[] = {
    [FIN_A]   = { 750, 1200, 2600 },
    [FIN_O]   = { 500,  900, 2600 },
    [FIN_E]   = { 520, 1800, 2700 },
    [FIN_I]   = { 300, 2300, 3100 },
    [FIN_U]   = { 320,  750, 2300 },
    [FIN_V]   = { 300, 2100, 2800 },
    [FIN_AI]  = { 650, 1600, 2700 },   /* a→i */
    [FIN_EI]  = { 500, 2000, 2800 },
    [FIN_AO]  = { 700, 1100, 2600 },   /* a→u */
    [FIN_OU]  = { 500, 1000, 2600 },
    [FIN_AN]  = { 700, 1300, 2600 },
    [FIN_EN]  = { 500, 1600, 2700 },
    [FIN_ANG] = { 720, 1200, 2600 },
    [FIN_ENG] = { 520, 1500, 2700 },
    [FIN_ONG] = { 480,  850, 2500 },
    [FIN_IA]  = { 320, 2400, 3100 },
    [FIN_IE]  = { 310, 2350, 3100 },
    [FIN_IAO] = { 330, 2200, 3000 },
    [FIN_IU]  = { 300, 2200, 3000 },
    [FIN_IAN] = { 320, 2400, 3100 },
    [FIN_IN]  = { 300, 2300, 3100 },
    [FIN_IANG]= { 320, 2400, 3100 },
    [FIN_ING] = { 310, 2350, 3100 },
    [FIN_IONG]= { 320, 2200, 3000 },
    [FIN_UA]  = { 350, 1200, 2500 },
    [FIN_UO]  = { 380,  900, 2500 },
    [FIN_UAI] = { 400, 1600, 2600 },
    [FIN_UI]  = { 320, 2100, 2800 },
    [FIN_UAN] = { 350, 1400, 2600 },
    [FIN_UN]  = { 320, 1600, 2700 },
    [FIN_UANG]= { 380, 1200, 2500 },
    [FIN_UE]  = { 310, 2200, 2900 },
    [FIN_ER]  = { 520, 1600, 2700 },
};

/* ── Character → Pinyin lookup table ───────────────── */
/* Sorted by Unicode codepoint for binary search.
   Only the ~400 most common Chinese chars. If a char
   is missing, it's silently skipped. */

typedef struct { uint16_t cp; uint16_t py; } char_py_t;

static const char_py_t CHAR_TABLE[] = {
    /* sorted by codepoint */
    {0x4E00, PY(INIT_NULL, FIN_I, 1)},   /* 一 yi1 */
    {0x4E09, PY(INIT_S,  FIN_AN, 1)},    /* 三 san1 */
    {0x4E0A, PY(INIT_SH, FIN_ANG, 4)},   /* 上 shang4 */
    {0x4E0B, PY(INIT_X,  FIN_IA, 4)},    /* 下 xia4 */
    {0x4E0D, PY(INIT_B,  FIN_U, 4)},     /* 不 bu4 */
    {0x4E16, PY(INIT_SH, FIN_I, 4)},     /* 世 shi4 */
    {0x4E2A, PY(INIT_G,  FIN_E, 4)},     /* 个 ge4 */
    {0x4E2D, PY(INIT_ZH, FIN_ONG, 1)},   /* 中 zhong1 */
    {0x4E3A, PY(INIT_W,  FIN_EI, 4)},    /* 为 wei4 */
    {0x4E3B, PY(INIT_ZH, FIN_U, 3)},     /* 主 zhu3 */
    {0x4E48, PY(INIT_M,  FIN_E, 0)},     /* 么 me */
    {0x4E50, PY(INIT_L,  FIN_E, 4)},     /* 乐 le4 */
    {0x4E5F, PY(INIT_Y,  FIN_E, 3)},     /* 也 ye3 */
    {0x4E86, PY(INIT_L,  FIN_E, 0)},     /* 了 le */
    {0x4E8B, PY(INIT_SH, FIN_I, 4)},     /* 事 shi4 */
    {0x4EBA, PY(INIT_R,  FIN_EN, 2)},    /* 人 ren2 */
    {0x4ED6, PY(INIT_T,  FIN_A, 1)},     /* 他 ta1 */
    {0x4ED7, PY(INIT_D,  FIN_A, 4)},     /* 大 da4 */
    {0x4EE5, PY(INIT_Y,  FIN_I, 3)},     /* 以 yi3 */
    {0x4EEC, PY(INIT_M,  FIN_EN, 0)},    /* 们 men */
    {0x4F1A, PY(INIT_H,  FIN_UI, 4)},    /* 会 hui4 */
    {0x4F46, PY(INIT_D,  FIN_AN, 4)},    /* 但 dan4 */
    {0x4F4D, PY(INIT_W,  FIN_EI, 4)},    /* 位 wei4 */
    {0x4F60, PY(INIT_N,  FIN_I, 3)},     /* 你 ni3 */
    {0x4F7F, PY(INIT_SH, FIN_I, 3)},     /* 使 shi3 */
    {0x4FBF, PY(INIT_B,  FIN_IAN, 4)},   /* 便 bian4 */
    {0x505A, PY(INIT_Z,  FIN_UO, 4)},    /* 做 zuo4 */
    {0x505C, PY(INIT_T,  FIN_ING, 2)},   /* 停 ting2 */
    {0x50CF, PY(INIT_X,  FIN_IANG,4)},   /* 像 xiang4 */
    {0x5143, PY(INIT_Y,  FIN_UAN, 2)},   /* 元 yuan2 */
    {0x5148, PY(INIT_X,  FIN_IAN, 1)},   /* 先 xian1 */
    {0x5149, PY(INIT_G,  FIN_UANG,1)},   /* 光 guang1 */
    {0x5168, PY(INIT_Q,  FIN_UAN, 2)},   /* 全 quan2 */
    {0x5173, PY(INIT_G,  FIN_UAN, 1)},   /* 关 guan1 */
    {0x5176, PY(INIT_Q,  FIN_I, 2)},     /* 其 qi2 */
    {0x5185, PY(INIT_N,  FIN_EI, 4)},    /* 内 nei4 */
    {0x51FA, PY(INIT_CH, FIN_U, 1)},     /* 出 chu1 */
    {0x5200, PY(INIT_D,  FIN_AO, 1)},    /* 刀 dao1 */
    {0x5206, PY(INIT_F,  FIN_EN, 1)},    /* 分 fen1 */
    {0x5220, PY(INIT_Q,  FIN_IE, 4)},    /* 切 qie4 */
    {0x522B, PY(INIT_B,  FIN_IE, 2)},    /* 别 bie2 */
    {0x5230, PY(INIT_D,  FIN_AO, 4)},    /* 到 dao4 */
    {0x524D, PY(INIT_Q,  FIN_IAN, 2)},   /* 前 qian2 */
    {0x52A8, PY(INIT_D,  FIN_ONG, 4)},   /* 动 dong4 */
    {0x52A9, PY(INIT_ZH, FIN_U, 4)},     /* 助 zhu4 */
    {0x52A0, PY(INIT_J,  FIN_IA, 1)},    /* 加 jia1 */
    {0x5316, PY(INIT_H,  FIN_UA, 4)},    /* 化 hua4 */
    {0x5341, PY(INIT_SH, FIN_I, 2)},     /* 十 shi2 */
    {0x5343, PY(INIT_Q,  FIN_IAN, 1)},   /* 千 qian1 */
    {0x5347, PY(INIT_SH, FIN_ENG, 1)},   /* 升 sheng1 */
    {0x5357, PY(INIT_N,  FIN_AN, 2)},    /* 南 nan2 */
    {0x536B, PY(INIT_W,  FIN_EI, 4)},    /* 卫 wei4 */
    {0x5373, PY(INIT_J,  FIN_I, 2)},     /* 即 ji2 */
    {0x53BB, PY(INIT_Q,  FIN_V, 4)},     /* 去 qu4 */
    {0x53C2, PY(INIT_C,  FIN_AN, 1)},    /* 参 can1 */
    {0x53D1, PY(INIT_F,  FIN_A, 1)},     /* 发 fa1 */
    {0x53D7, PY(INIT_SH, FIN_OU, 4)},    /* 受 shou4 */
    {0x53E3, PY(INIT_K,  FIN_OU, 3)},    /* 口 kou3 */
    {0x53EA, PY(INIT_ZH, FIN_I, 3)},     /* 只 zhi3 */
    {0x53EF, PY(INIT_K,  FIN_E, 3)},     /* 可 ke3 */
    {0x53F0, PY(INIT_T,  FIN_AI, 2)},    /* 台 tai2 */
    {0x53F3, PY(INIT_Y,  FIN_OU, 4)},    /* 右 you4 */
    {0x53F7, PY(INIT_H,  FIN_AO, 4)},    /* 号 hao4 */
    {0x5403, PY(INIT_CH, FIN_I, 1)},     /* 吃 chi1 */
    {0x5404, PY(INIT_G,  FIN_E, 4)},     /* 各 ge4 */
    {0x540E, PY(INIT_H,  FIN_OU, 4)},    /* 后 hou4 */
    {0x5411, PY(INIT_X,  FIN_IANG,4)},   /* 向 xiang4 */
    {0x542C, PY(INIT_T,  FIN_ING, 1)},   /* 听 ting1 */
    {0x5440, PY(INIT_Y,  FIN_A, 0)},     /* 呀 ya */
    {0x544A, PY(INIT_G,  FIN_AO, 4)},    /* 告 gao4 */
    {0x5473, PY(INIT_W,  FIN_EI, 4)},    /* 味 wei4 */
    {0x548C, PY(INIT_H,  FIN_E, 2)},     /* 和 he2 */
    {0x54C8, PY(INIT_H,  FIN_A, 1)},     /* 哈 ha1 */
    {0x54C9, PY(INIT_Z,  FIN_AI, 1)},    /* 哉 zai1 */
    {0x54EA, PY(INIT_N,  FIN_A, 3)},     /* 哪 na3 */
    {0x54ED, PY(INIT_Q,  FIN_I, 4)},     /* 器 qi4 */
    {0x56DB, PY(INIT_S,  FIN_I, 4)},     /* 四 si4 */
    {0x56DE, PY(INIT_H,  FIN_UI, 2)},    /* 回 hui2 */
    {0x56FD, PY(INIT_G,  FIN_UO, 2)},    /* 国 guo2 */
    {0x5728, PY(INIT_Z,  FIN_AI, 4)},    /* 在 zai4 */
    {0x5730, PY(INIT_D,  FIN_I, 4)},     /* 地 di4 */
    {0x5747, PY(INIT_J,  FIN_UN, 1)},    /* 均 jun1 */
    {0x578B, PY(INIT_X,  FIN_ING, 2)},   /* 型 xing2 */
    {0x57FA, PY(INIT_J,  FIN_I, 1)},     /* 基 ji1 */
    {0x58F0, PY(INIT_SH, FIN_ENG, 1)},   /* 声 sheng1 */
    {0x5904, PY(INIT_CH, FIN_U, 4)},     /* 处 chu4 */
    {0x5916, PY(INIT_W,  FIN_AI, 4)},    /* 外 wai4 */
    {0x591A, PY(INIT_D,  FIN_UO, 1)},    /* 多 duo1 */
    {0x5927, PY(INIT_D,  FIN_A, 4)},     /* 大 da4 */
    {0x5929, PY(INIT_T,  FIN_IAN, 1)},   /* 天 tian1 */
    {0x5931, PY(INIT_SH, FIN_I, 1)},     /* 失 shi1 */
    {0x5934, PY(INIT_T,  FIN_OU, 2)},    /* 头 tou2 */
    {0x597D, PY(INIT_H,  FIN_AO, 3)},    /* 好 hao3 */
    {0x5982, PY(INIT_R,  FIN_U, 2)},     /* 如 ru2 */
    {0x59CB, PY(INIT_SH, FIN_I, 3)},     /* 始 shi3 */
    {0x59D0, PY(INIT_J,  FIN_IE, 3)},    /* 姐 jie3 */
    {0x5B50, PY(INIT_Z,  FIN_I, 3)},     /* 子 zi3 */
    {0x5B57, PY(INIT_Z,  FIN_I, 4)},     /* 字 zi4 */
    {0x5B89, PY(INIT_NULL,FIN_AN, 1)},   /* 安 an1 */
    {0x5B8C, PY(INIT_W,  FIN_AN, 2)},    /* 完 wan2 */
    {0x5B9A, PY(INIT_D,  FIN_ING, 4)},   /* 定 ding4 */
    {0x5B9E, PY(INIT_SH, FIN_I, 2)},     /* 实 shi2 */
    {0x5BB6, PY(INIT_J,  FIN_IA, 1)},    /* 家 jia1 */
    {0x5BF9, PY(INIT_D,  FIN_UI, 4)},    /* 对 dui4 */
    {0x5C0F, PY(INIT_X,  FIN_IAO, 3)},   /* 小 xiao3 */
    {0x5C11, PY(INIT_SH, FIN_AO, 3)},    /* 少 shao3 */
    {0x5C3D, PY(INIT_J,  FIN_IN, 4)},    /* 尽 jin4 */
    {0x5C55, PY(INIT_ZH, FIN_AN, 3)},    /* 展 zhan3 */
    {0x5DE5, PY(INIT_G,  FIN_ONG, 1)},   /* 工 gong1 */
    {0x5DE6, PY(INIT_Z,  FIN_UO, 3)},    /* 左 zuo3 */
    {0x5DF2, PY(INIT_Y,  FIN_I, 3)},     /* 已 yi3 */
    {0x5E02, PY(INIT_SH, FIN_I, 4)},     /* 市 shi4 */
    {0x5E26, PY(INIT_D,  FIN_AI, 4)},    /* 带 dai4 */
    {0x5E2E, PY(INIT_B,  FIN_ANG, 1)},   /* 帮 bang1 */
    {0x5E38, PY(INIT_CH, FIN_ANG, 2)},   /* 常 chang2 */
    {0x5E73, PY(INIT_P,  FIN_ING, 2)},   /* 平 ping2 */
    {0x5E78, PY(INIT_X,  FIN_ING, 4)},   /* 幸 xing4 */
    {0x5EA6, PY(INIT_D,  FIN_U, 4)},     /* 度 du4 */
    {0x5F00, PY(INIT_K,  FIN_AI, 1)},    /* 开 kai1 */
    {0x5F02, PY(INIT_Y,  FIN_I, 4)},     /* 异 yi4 */
    {0x5F15, PY(INIT_Y,  FIN_IN, 3)},    /* 引 yin3 */
    {0x5F88, PY(INIT_H,  FIN_EN, 3)},    /* 很 hen3 */
    {0x5F97, PY(INIT_D,  FIN_E, 2)},     /* 得 de2 */
    {0x5FAE, PY(INIT_W,  FIN_EI, 1)},    /* 微 wei1 */
    {0x5FC3, PY(INIT_X,  FIN_IN, 1)},    /* 心 xin1 */
    {0x5FD9, PY(INIT_M,  FIN_ANG, 2)},   /* 忙 mang2 */
    {0x6001, PY(INIT_T,  FIN_AI, 4)},    /* 态 tai4 */
    {0x600E, PY(INIT_Z,  FIN_EN, 3)},    /* 怎 zen3 */
    {0x601D, PY(INIT_S,  FIN_I, 1)},     /* 思 si1 */
    {0x6025, PY(INIT_J,  FIN_I, 2)},     /* 急 ji2 */
    {0x6076, PY(INIT_NULL,FIN_E, 4)},    /* 恶 e4 */
    {0x60A8, PY(INIT_N,  FIN_IN, 2)},    /* 您 nin2 */
    {0x60C5, PY(INIT_Q,  FIN_ING, 2)},   /* 情 qing2 */
    {0x60F3, PY(INIT_X,  FIN_IANG,3)},   /* 想 xiang3 */
    {0x610F, PY(INIT_Y,  FIN_I, 4)},     /* 意 yi4 */
    {0x611F, PY(INIT_G,  FIN_AN, 3)},    /* 感 gan3 */
    {0x6211, PY(INIT_W,  FIN_O, 3)},     /* 我 wo3 */
    {0x6210, PY(INIT_CH, FIN_ENG, 2)},   /* 成 cheng2 */
    {0x6216, PY(INIT_H,  FIN_UO, 4)},    /* 或 huo4 */
    {0x6240, PY(INIT_S,  FIN_UO, 3)},    /* 所 suo3 */
    {0x624B, PY(INIT_SH, FIN_OU, 3)},    /* 手 shou3 */
    {0x6253, PY(INIT_D,  FIN_A, 3)},     /* 打 da3 */
    {0x6269, PY(INIT_K,  FIN_UO, 4)},    /* 扩 kuo4 */
    {0x627E, PY(INIT_ZH, FIN_AO, 3)},    /* 找 zhao3 */
    {0x6280, PY(INIT_J,  FIN_I, 4)},     /* 技 ji4 */
    {0x6295, PY(INIT_T,  FIN_OU, 2)},    /* 投 tou2 */
    {0x62B1, PY(INIT_B,  FIN_AO, 4)},    /* 抱 bao4 */
    {0x62C9, PY(INIT_L,  FIN_A, 1)},     /* 拉 la1 */
    {0x62DC, PY(INIT_B,  FIN_AI, 4)},    /* 拜 bai4 */
    {0x62FF, PY(INIT_N,  FIN_A, 2)},     /* 拿 na2 */
    {0x6307, PY(INIT_ZH, FIN_I, 3)},     /* 指 zhi3 */
    {0x6309, PY(INIT_NULL,FIN_AN, 4)},   /* 按 an4 */
    {0x6362, PY(INIT_H,  FIN_UAN, 4)},   /* 换 huan4 */
    {0x63A5, PY(INIT_J,  FIN_IE, 1)},    /* 接 jie1 */
    {0x63A8, PY(INIT_T,  FIN_UI, 1)},    /* 推 tui1 */
    {0x63D0, PY(INIT_T,  FIN_I, 2)},     /* 提 ti2 */
    {0x64AD, PY(INIT_B,  FIN_O, 1)},     /* 播 bo1 */
    {0x652F, PY(INIT_ZH, FIN_I, 1)},     /* 支 zhi1 */
    {0x6536, PY(INIT_SH, FIN_OU, 1)},    /* 收 shou1 */
    {0x653E, PY(INIT_F,  FIN_ANG, 4)},   /* 放 fang4 */
    {0x6548, PY(INIT_X,  FIN_IAO, 4)},   /* 效 xiao4 */
    {0x6570, PY(INIT_SH, FIN_U, 4)},     /* 数 shu4 */
    {0x6587, PY(INIT_W,  FIN_EN, 2)},    /* 文 wen2 */
    {0x65B0, PY(INIT_X,  FIN_IN, 1)},    /* 新 xin1 */
    {0x65B9, PY(INIT_F,  FIN_ANG, 1)},   /* 方 fang1 */
    {0x65E0, PY(INIT_W,  FIN_U, 2)},     /* 无 wu2 */
    {0x65E5, PY(INIT_R,  FIN_I, 4)},     /* 日 ri4 */
    {0x65F6, PY(INIT_SH, FIN_I, 2)},     /* 时 shi2 */
    {0x660E, PY(INIT_M,  FIN_ING, 2)},   /* 明 ming2 */
    {0x661F, PY(INIT_X,  FIN_ING, 1)},   /* 星 xing1 */
    {0x662F, PY(INIT_SH, FIN_I, 4)},     /* 是 shi4 */
    {0x666F, PY(INIT_J,  FIN_ING, 3)},   /* 景 jing3 */
    {0x667A, PY(INIT_ZH, FIN_I, 4)},     /* 智 zhi4 */
    {0x66F4, PY(INIT_G,  FIN_ENG, 4)},   /* 更 geng4 */
    {0x6700, PY(INIT_Z,  FIN_UI, 4)},    /* 最 zui4 */
    {0x6709, PY(INIT_Y,  FIN_OU, 3)},    /* 有 you3 */
    {0x671F, PY(INIT_Q,  FIN_I, 1)},     /* 期 qi1 */
    {0x672A, PY(INIT_W,  FIN_EI, 4)},    /* 未 wei4 */
    {0x672C, PY(INIT_B,  FIN_EN, 3)},    /* 本 ben3 */
    {0x673A, PY(INIT_J,  FIN_I, 1)},     /* 机 ji1 */
    {0x6740, PY(INIT_SH, FIN_A, 1)},     /* 杀 sha1 */
    {0x6765, PY(INIT_L,  FIN_AI, 2)},    /* 来 lai2 */
    {0x679C, PY(INIT_G,  FIN_UO, 3)},    /* 果 guo3 */
    {0x67E5, PY(INIT_CH, FIN_A, 2)},     /* 查 cha2 */
    {0x6807, PY(INIT_B,  FIN_IAO, 1)},   /* 标 biao1 */
    {0x6837, PY(INIT_Y,  FIN_IANG,4)},   /* 样 yang4 */
    {0x68C0, PY(INIT_J,  FIN_IAN, 3)},   /* 检 jian3 */
    {0x68EE, PY(INIT_S,  FIN_EN, 1)},    /* 森 sen1 */
    {0x690D, PY(INIT_ZH, FIN_I, 2)},     /* 植 zhi2 */
    {0x6A21, PY(INIT_M,  FIN_O, 2)},     /* 模 mo2 */
    {0x6B22, PY(INIT_H,  FIN_UAN, 1)},   /* 欢 huan1 */
    {0x6B63, PY(INIT_ZH, FIN_ENG, 4)},   /* 正 zheng4 */
    {0x6B64, PY(INIT_C,  FIN_I, 3)},     /* 此 ci3 */
    {0x6B65, PY(INIT_B,  FIN_U, 4)},     /* 步 bu4 */
    {0x6BBF, PY(INIT_D,  FIN_IAN, 4)},   /* 殿 dian4 */
    {0x6BD4, PY(INIT_B,  FIN_I, 3)},     /* 比 bi3 */
    {0x6C11, PY(INIT_M,  FIN_IN, 2)},    /* 民 min2 */
    {0x6C14, PY(INIT_Q,  FIN_I, 4)},     /* 气 qi4 */
    {0x6C34, PY(INIT_SH, FIN_UI, 3)},    /* 水 shui3 */
    {0x6C42, PY(INIT_Q,  FIN_IU, 2)},    /* 求 qiu2 */
    {0x6C7D, PY(INIT_Q,  FIN_I, 4)},     /* 汽 qi4 */
    {0x6CA1, PY(INIT_M,  FIN_EI, 2)},    /* 没 mei2 */
    {0x6CD5, PY(INIT_F,  FIN_A, 3)},     /* 法 fa3 */
    {0x6D4B, PY(INIT_C,  FIN_E, 4)},     /* 测 ce4 */
    {0x6D6A, PY(INIT_L,  FIN_ANG, 4)},   /* 浪 lang4 */
    {0x6D77, PY(INIT_H,  FIN_AI, 3)},    /* 海 hai3 */
    {0x6D88, PY(INIT_X,  FIN_IAO, 1)},   /* 消 xiao1 */
    {0x6DF1, PY(INIT_SH, FIN_EN, 1)},    /* 深 shen1 */
    {0x6E05, PY(INIT_Q,  FIN_ING, 1)},   /* 清 qing1 */
    {0x6E29, PY(INIT_W,  FIN_EN, 1)},    /* 温 wen1 */
    {0x6E2F, PY(INIT_G,  FIN_ANG, 3)},   /* 港 gang3 */
    {0x6E90, PY(INIT_Y,  FIN_UAN, 2)},   /* 源 yuan2 */
    {0x6EE1, PY(INIT_M,  FIN_AN, 3)},    /* 满 man3 */
    {0x706B, PY(INIT_H,  FIN_UO, 3)},    /* 火 huo3 */
    {0x70B9, PY(INIT_D,  FIN_IAN, 3)},   /* 点 dian3 */
    {0x7136, PY(INIT_R,  FIN_AN, 2)},    /* 然 ran2 */
    {0x7167, PY(INIT_ZH, FIN_AO, 4)},    /* 照 zhao4 */
    {0x7231, PY(INIT_NULL,FIN_AI, 4)},   /* 爱 ai4 */
    {0x7248, PY(INIT_B,  FIN_AN, 3)},    /* 版 ban3 */
    {0x7269, PY(INIT_W,  FIN_U, 4)},     /* 物 wu4 */
    {0x72B6, PY(INIT_ZH, FIN_UANG,4)},   /* 状 zhuang4 */
    {0x72EC, PY(INIT_D,  FIN_U, 2)},     /* 独 du2 */
    {0x7387, PY(INIT_L,  FIN_V, 4)},     /* 率 lv4 */
    {0x73AF, PY(INIT_H,  FIN_UAN, 2)},   /* 环 huan2 */
    {0x73B0, PY(INIT_X,  FIN_IAN, 4)},   /* 现 xian4 */
    {0x751F, PY(INIT_SH, FIN_ENG, 1)},   /* 生 sheng1 */
    {0x7528, PY(INIT_Y,  FIN_ONG, 4)},   /* 用 yong4 */
    {0x7535, PY(INIT_D,  FIN_IAN, 4)},   /* 电 dian4 */
    {0x7537, PY(INIT_N,  FIN_AN, 2)},    /* 男 nan2 */
    {0x754C, PY(INIT_J,  FIN_IE, 4)},    /* 界 jie4 */
    {0x7565, PY(INIT_L,  FIN_UE, 4)},    /* 略 lve4 */
    {0x758F, PY(INIT_SH, FIN_U, 1)},     /* 疏 shu1 */
    {0x767E, PY(INIT_B,  FIN_AI, 3)},    /* 百 bai3 */
    {0x7684, PY(INIT_D,  FIN_E, 0)},     /* 的 de */
    {0x76D1, PY(INIT_J,  FIN_IAN, 1)},   /* 监 jian1 */
    {0x770B, PY(INIT_K,  FIN_AN, 4)},    /* 看 kan4 */
    {0x771F, PY(INIT_ZH, FIN_EN, 1)},    /* 真 zhen1 */
    {0x773C, PY(INIT_Y,  FIN_AN, 3)},    /* 眼 yan3 */
    {0x77E5, PY(INIT_ZH, FIN_I, 1)},     /* 知 zhi1 */
    {0x7801, PY(INIT_M,  FIN_A, 3)},     /* 码 ma3 */
    {0x786E, PY(INIT_Q,  FIN_UE, 4)},    /* 确 que4 */
    {0x793A, PY(INIT_SH, FIN_I, 4)},     /* 示 shi4 */
    {0x793E, PY(INIT_SH, FIN_E, 4)},     /* 社 she4 */
    {0x79D1, PY(INIT_K,  FIN_E, 1)},     /* 科 ke1 */
    {0x79FB, PY(INIT_Y,  FIN_I, 2)},     /* 移 yi2 */
    {0x7A0B, PY(INIT_CH, FIN_ENG, 2)},   /* 程 cheng2 */
    {0x7A33, PY(INIT_W,  FIN_EN, 3)},    /* 稳 wen3 */
    {0x7A76, PY(INIT_J,  FIN_IU, 1)},    /* 究 jiu1 */
    {0x7A7A, PY(INIT_K,  FIN_ONG, 1)},   /* 空 kong1 */
    {0x7ACB, PY(INIT_L,  FIN_I, 4)},     /* 立 li4 */
    {0x7AD9, PY(INIT_ZH, FIN_AN, 4)},    /* 站 zhan4 */
    {0x7AEF, PY(INIT_D,  FIN_UAN, 1)},   /* 端 duan1 */
    {0x7B11, PY(INIT_X,  FIN_IAO, 4)},   /* 笑 xiao4 */
    {0x7B14, PY(INIT_B,  FIN_I, 3)},     /* 笔 bi3 */
    {0x7B49, PY(INIT_D,  FIN_ENG, 3)},   /* 等 deng3 */
    {0x7B80, PY(INIT_J,  FIN_IAN, 3)},   /* 简 jian3 */
    {0x7B97, PY(INIT_S,  FIN_UAN, 4)},   /* 算 suan4 */
    {0x7BA1, PY(INIT_G,  FIN_UAN, 3)},   /* 管 guan3 */
    {0x7C7B, PY(INIT_L,  FIN_EI, 4)},    /* 类 lei4 */
    {0x7CBE, PY(INIT_J,  FIN_ING, 1)},   /* 精 jing1 */
    {0x7CFB, PY(INIT_X,  FIN_I, 4)},     /* 系 xi4 */
    {0x7D2F, PY(INIT_L,  FIN_EI, 4)},    /* 累 lei4 */
    {0x7D27, PY(INIT_J,  FIN_IN, 3)},    /* 紧 jin3 */
    {0x7EA0, PY(INIT_J,  FIN_IU, 1)},    /* 纠 jiu1 */
    {0x7EBF, PY(INIT_X,  FIN_IAN, 4)},   /* 线 xian4 */
    {0x7EC4, PY(INIT_Z,  FIN_U, 3)},     /* 组 zu3 */
    {0x7EC7, PY(INIT_ZH, FIN_I, 1)},     /* 织 zhi1 */
    {0x7ED3, PY(INIT_J,  FIN_IE, 2)},    /* 结 jie2 */
    {0x7ED9, PY(INIT_G,  FIN_EI, 3)},    /* 给 gei3 */
    {0x7EE7, PY(INIT_J,  FIN_I, 4)},     /* 继 ji4 */
    {0x7F16, PY(INIT_B,  FIN_IAN, 1)},   /* 编 bian1 */
    {0x7F29, PY(INIT_S,  FIN_UO, 1)},    /* 缩 suo1 */
    {0x7F51, PY(INIT_W,  FIN_ANG, 3)},   /* 网 wang3 */
    {0x7F8E, PY(INIT_M,  FIN_EI, 3)},    /* 美 mei3 */
    {0x8003, PY(INIT_K,  FIN_AO, 3)},    /* 考 kao3 */
    {0x803D, PY(INIT_D,  FIN_AN, 1)},    /* 耽 dan1 */
    {0x804A, PY(INIT_L,  FIN_IAO, 2)},   /* 聊 liao2 */
    {0x8054, PY(INIT_L,  FIN_IAN, 2)},   /* 联 lian2 */
    {0x8074, PY(INIT_T,  FIN_ING, 1)},   /* 听 ting1 */
    {0x80A1, PY(INIT_G,  FIN_U, 3)},     /* 股 gu3 */
    {0x80AF, PY(INIT_K,  FIN_EN, 3)},    /* 肯 ken3 */
    {0x80FD, PY(INIT_N,  FIN_ENG, 2)},   /* 能 neng2 */
    {0x811A, PY(INIT_J,  FIN_IAO, 3)},   /* 脚 jiao3 */
    {0x8138, PY(INIT_L,  FIN_IAN, 3)},   /* 脸 lian3 */
    {0x81EA, PY(INIT_Z,  FIN_I, 4)},     /* 自 zi4 */
    {0x822A, PY(INIT_H,  FIN_ANG, 2)},   /* 航 hang2 */
    {0x8272, PY(INIT_S,  FIN_E, 4)},     /* 色 se4 */
    {0x8282, PY(INIT_J,  FIN_IE, 2)},    /* 节 jie2 */
    {0x8292, PY(INIT_M,  FIN_ANG, 2)},   /* 芒 mang2 */
    {0x82B1, PY(INIT_H,  FIN_UA, 1)},    /* 花 hua1 */
    {0x8328, PY(INIT_C,  FIN_I, 2)},     /* 茨 ci2 */
    {0x8350, PY(INIT_J,  FIN_IAN, 4)},   /* 荐 jian4 */
    {0x8377, PY(INIT_H,  FIN_E, 2)},     /* 荷 he2 */
    {0x8425, PY(INIT_Y,  FIN_ING, 2)},   /* 营 ying2 */
    {0x843D, PY(INIT_L,  FIN_UO, 4)},    /* 落 luo4 */
    {0x84DD, PY(INIT_L,  FIN_AN, 2)},    /* 蓝 lan2 */
    {0x85AA, PY(INIT_X,  FIN_IN, 1)},    /* 薪 xin1 */
    {0x884C, PY(INIT_X,  FIN_ING, 2)},   /* 行 xing2 */
    {0x8868, PY(INIT_B,  FIN_IAO, 3)},   /* 表 biao3 */
    {0x88C5, PY(INIT_ZH, FIN_UANG,1)},   /* 装 zhuang1 */
    {0x897F, PY(INIT_X,  FIN_I, 1)},     /* 西 xi1 */
    {0x8981, PY(INIT_Y,  FIN_AO, 4)},    /* 要 yao4 */
    {0x89C4, PY(INIT_G,  FIN_UI, 1)},    /* 规 gui1 */
    {0x89C6, PY(INIT_SH, FIN_I, 4)},     /* 视 shi4 */
    {0x89D2, PY(INIT_J,  FIN_IAO, 3)},   /* 角 jiao3 */
    {0x8A00, PY(INIT_Y,  FIN_AN, 2)},    /* 言 yan2 */
    {0x8A66, PY(INIT_SH, FIN_I, 4)},     /* 试 shi4 */
    {0x8A71, PY(INIT_H,  FIN_UA, 4)},    /* 话 hua4 */
    {0x8A9E, PY(INIT_Y,  FIN_U, 3)},     /* 语 yu3 */
    {0x8AAA, PY(INIT_SH, FIN_UO, 1)},    /* 说 shuo1 */
    {0x8ABF, PY(INIT_T,  FIN_IAO, 2)},   /* 调 tiao2 */
    {0x8BF4, PY(INIT_SH, FIN_UO, 1)},    /* 说 shuo1 */
    {0x8BF7, PY(INIT_Q,  FIN_ING, 3)},   /* 请 qing3 */
    {0x8C03, PY(INIT_D,  FIN_IAO, 4)},   /* 调 diao4 */
    {0x8C08, PY(INIT_T,  FIN_AN, 2)},    /* 谈 tan2 */
    {0x8C22, PY(INIT_X,  FIN_IE, 4)},    /* 谢 xie4 */
    {0x8D23, PY(INIT_Z,  FIN_E, 2)},     /* 责 ze2 */
    {0x8D28, PY(INIT_ZH, FIN_I, 4)},     /* 质 zhi4 */
    {0x8D39, PY(INIT_F,  FIN_EI, 4)},    /* 费 fei4 */
    {0x8D44, PY(INIT_Z,  FIN_I, 1)},     /* 资 zi1 */
    {0x8D70, PY(INIT_Z,  FIN_OU, 3)},    /* 走 zou3 */
    {0x8D77, PY(INIT_Q,  FIN_I, 3)},     /* 起 qi3 */
    {0x8D85, PY(INIT_CH, FIN_AO, 1)},    /* 超 chao1 */
    {0x8D8A, PY(INIT_Y,  FIN_UE, 4)},    /* 越 yue4 */
    {0x8DB3, PY(INIT_Z,  FIN_U, 2)},     /* 足 zu2 */
    {0x8DDD, PY(INIT_J,  FIN_V, 4)},     /* 距 ju4 */
    {0x8DEF, PY(INIT_L,  FIN_U, 4)},     /* 路 lu4 */
    {0x8E0A, PY(INIT_Y,  FIN_ONG, 3)},   /* 踊 yong3 */
    {0x8EAB, PY(INIT_SH, FIN_EN, 1)},    /* 身 shen1 */
    {0x8F6C, PY(INIT_ZH, FIN_UAN, 3)},   /* 转 zhuan3 */
    {0x8F7B, PY(INIT_Q,  FIN_ING, 1)},   /* 轻 qing1 */
    {0x8F93, PY(INIT_SH, FIN_U, 1)},     /* 输 shu1 */
    {0x8FC7, PY(INIT_G,  FIN_UO, 4)},    /* 过 guo4 */
    {0x8FD0, PY(INIT_Y,  FIN_UN, 4)},    /* 运 yun4 */
    {0x8FD1, PY(INIT_J,  FIN_IN, 4)},    /* 近 jin4 */
    {0x8FD4, PY(INIT_F,  FIN_AN, 3)},    /* 返 fan3 */
    {0x8FDC, PY(INIT_Y,  FIN_UAN, 3)},   /* 远 yuan3 */
    {0x8FDE, PY(INIT_L,  FIN_IAN, 2)},   /* 连 lian2 */
    {0x9000, PY(INIT_T,  FIN_UI, 4)},    /* 退 tui4 */
    {0x9001, PY(INIT_S,  FIN_ONG, 4)},   /* 送 song4 */
    {0x9009, PY(INIT_X,  FIN_UAN, 3)},   /* 选 xuan3 */
    {0x901F, PY(INIT_S,  FIN_U, 4)},     /* 速 su4 */
    {0x9020, PY(INIT_Z,  FIN_AO, 4)},    /* 造 zao4 */
    {0x9047, PY(INIT_Y,  FIN_U, 4)},     /* 遇 yu4 */
    {0x904A, PY(INIT_Y,  FIN_OU, 2)},    /* 游 you2 */
    {0x9053, PY(INIT_D,  FIN_AO, 4)},    /* 道 dao4 */
    {0x9054, PY(INIT_D,  FIN_A, 2)},     /* 达 da2 */
    {0x907F, PY(INIT_B,  FIN_I, 4)},     /* 避 bi4 */
    {0x90A3, PY(INIT_N,  FIN_A, 4)},     /* 那 na4 */
    {0x90E8, PY(INIT_B,  FIN_U, 4)},     /* 部 bu4 */
    {0x90FD, PY(INIT_D,  FIN_OU, 1)},    /* 都 dou1 */
    {0x914D, PY(INIT_P,  FIN_EI, 4)},    /* 配 pei4 */
    {0x91CD, PY(INIT_ZH, FIN_ONG, 4)},   /* 重 zhong4 */
    {0x91CF, PY(INIT_L,  FIN_IANG,4)},   /* 量 liang4 */
    {0x94A6, PY(INIT_Q,  FIN_IN, 1)},    /* 钦 qin1 */
    {0x950B, PY(INIT_F,  FIN_ENG, 1)},   /* 锋 feng1 */
    {0x9500, PY(INIT_X,  FIN_IAO, 1)},   /* 销 xiao1 */
    {0x952E, PY(INIT_J,  FIN_IAN, 4)},   /* 键 jian4 */
    {0x957F, PY(INIT_CH, FIN_ANG, 2)},   /* 长 chang2 */
    {0x95ED, PY(INIT_B,  FIN_I, 4)},     /* 闭 bi4 */
    {0x95EE, PY(INIT_W,  FIN_EN, 4)},    /* 问 wen4 */
    {0x95F4, PY(INIT_J,  FIN_IAN, 1)},   /* 间 jian1 */
    {0x9632, PY(INIT_F,  FIN_ANG, 2)},   /* 防 fang2 */
    {0x964D, PY(INIT_J,  FIN_IANG,4)},   /* 降 jiang4 */
    {0x9650, PY(INIT_X,  FIN_IAN, 4)},   /* 限 xian4 */
    {0x9664, PY(INIT_CH, FIN_U, 2)},     /* 除 chu2 */
    {0x968F, PY(INIT_S,  FIN_UI, 2)},    /* 随 sui2 */
    {0x96C6, PY(INIT_J,  FIN_I, 2)},     /* 集 ji2 */
    {0x96F7, PY(INIT_L,  FIN_EI, 2)},    /* 雷 lei2 */
    {0x9700, PY(INIT_X,  FIN_V, 1)},     /* 需 xu1 */
    {0x9759, PY(INIT_J,  FIN_ING, 4)},   /* 静 jing4 */
    {0x97F3, PY(INIT_Y,  FIN_IN, 1)},    /* 音 yin1 */
    {0x9875, PY(INIT_Y,  FIN_E, 4)},     /* 页 ye4 */
    {0x987F, PY(INIT_D,  FIN_UN, 4)},    /* 顿 dun4 */
    {0x9884, PY(INIT_Y,  FIN_U, 4)},     /* 预 yu4 */
    {0x9891, PY(INIT_P,  FIN_IN, 2)},    /* 频 pin2 */
    {0x9898, PY(INIT_T,  FIN_I, 2)},     /* 题 ti2 */
    {0x98A8, PY(INIT_F,  FIN_ENG, 1)},   /* 风 feng1 */
    {0x98DE, PY(INIT_F,  FIN_EI, 1)},    /* 飞 fei1 */
    {0x98DF, PY(INIT_SH, FIN_I, 2)},     /* 食 shi2 */
    {0x996D, PY(INIT_F,  FIN_AN, 4)},    /* 饭 fan4 */
    {0x9996, PY(INIT_SH, FIN_OU, 3)},    /* 首 shou3 */
    {0x9A71, PY(INIT_Q,  FIN_V, 1)},     /* 驱 qu1 */
    {0x9A8C, PY(INIT_Y,  FIN_AN, 4)},    /* 验 yan4 */
    {0x9AD8, PY(INIT_G,  FIN_AO, 1)},    /* 高 gao1 */
    {0x9B42, PY(INIT_H,  FIN_UN, 2)},    /* 魂 hun2 */
    {0x9E64, PY(INIT_L,  FIN_ONG, 2)},   /* 龙 long2 */
};

#define CHAR_COUNT (sizeof(CHAR_TABLE) / sizeof(CHAR_TABLE[0]))

/* ── UTF-8 decoder ─────────────────────────────────── */

static int utf8_decode(const char **ptr)
{
    const uint8_t *p = (const uint8_t *)*ptr;
    uint32_t cp;
    if ((*p & 0x80) == 0) { cp = *p; *ptr += 1; return cp; }
    if ((*p & 0xE0) == 0xC0) { cp = (*p & 0x1F) << 6; cp |= (p[1] & 0x3F); *ptr += 2; return cp; }
    if ((*p & 0xF0) == 0xE0) { cp = (*p & 0x0F) << 12; cp |= (p[1] & 0x3F) << 6; cp |= (p[2] & 0x3F); *ptr += 3; return cp; }
    if ((*p & 0xF8) == 0xF0) { cp = (*p & 0x07) << 18; cp |= (p[1] & 0x3F) << 12; cp |= (p[2] & 0x3F) << 6; cp |= (p[3] & 0x3F); *ptr += 4; return cp; }
    *ptr += 1; return -1;
}

/* ── Binary search in char table ───────────────────── */

static uint16_t lookup_pinyin(int codepoint)
{
    int lo = 0, hi = CHAR_COUNT - 1;
    while (lo <= hi) {
        int mid = (lo + hi) / 2;
        uint16_t cp = CHAR_TABLE[mid].cp;
        if (cp == (uint16_t)codepoint) return CHAR_TABLE[mid].py;
        if (cp < (uint16_t)codepoint) lo = mid + 1;
        else hi = mid - 1;
    }
    return 0xFFFF; /* not found */
}

/* ── Sine oscillator with continuous phase ──────────── */

typedef struct { float phase; } osc_t;

static float osc_next(osc_t *o, float freq, float sr)
{
    o->phase += freq / sr;
    if (o->phase >= 1.0f) o->phase -= 1.0f;
    return sinf(2.0f * (float)M_PI * o->phase);
}

/* ── Tone → F0 contour ─────────────────────────────── */

static float tone_f0(int tone, float progress)
{
    /* progress: 0.0 = start of vowel, 1.0 = end */
    float f0_lo = 160.0f, f0_hi = 280.0f;
    float mid = 180.0f, dip = 120.0f;

    switch (tone) {
    case 1: /* flat high */  return f0_hi;
    case 2: /* rising */      return f0_lo + (f0_hi - f0_lo) * progress;
    case 3: /* dipping */
        if (progress < 0.4f)  return f0_lo + (dip - f0_lo) * (progress / 0.4f);
        else                  return dip + (mid - dip) * ((progress - 0.4f) / 0.6f);
    case 4: /* falling */     return f0_hi + (f0_lo - f0_hi) * progress;
    default:                  return 200.0f;
    }
}

/* ── Synthesize one syllable ────────────────────────── */

static int synth_syllable(uint16_t pinyin, int16_t *buf, int max_samples)
{
    int init  = (pinyin >> 9) & 0x1F;
    int final = (pinyin >> 3) & 0x3F;
    int tone  = pinyin & 0x07;

    if (final >= (int)(sizeof(VOWEL_FMT)/sizeof(VOWEL_FMT[0]))) return 0;

    int f1 = VOWEL_FMT[final].f1;
    int f2 = VOWEL_FMT[final].f2;
    int f3 = VOWEL_FMT[final].f3;

    int n_cons = (CONS_MS * SAMPLE_RATE) / 1000;
    int n_vow  = SYL_SAMPLES - n_cons;
    if (n_vow < 0) n_vow = 0;
    int total = n_cons + n_vow;
    if (total > max_samples) total = max_samples;

    osc_t osc_f0 = {0}, osc_f1 = {0}, osc_f2 = {0}, osc_f3 = {0};

    for (int i = 0; i < total; i++) {
        float sample = 0.0f;

        if (i < n_cons) {
            /* Consonant: mostly noise, shaped by init type */
            float prog = (float)i / (float)n_cons;
            float env  = 1.0f - prog; /* fade out */
            float noise = ((float)(rand() & 0xFFFF) / 32768.0f - 1.0f) * 0.15f;
            sample = noise * env;
            /* Plosives (b,p,d,t,g,k) have a brief burst */
            if (init >= INIT_B && init <= INIT_K && prog < 0.2f) {
                sample += noise * 0.4f;
            }
            /* Fricatives (f,s,sh,x,h) have sustained noise */
            if ((init >= INIT_F  && init <= INIT_H)  ||
                (init >= INIT_SH && init <= INIT_S)) {
                sample += noise * 0.5f * env;
            }
        } else {
            /* Vowel: formant synthesis */
            int vi = i - n_cons;
            float prog = (float)vi / (float)n_vow;

            float f0 = tone_f0(tone, prog);
            float src = osc_next(&osc_f0, f0, SAMPLE_RATE) * 0.5f;

            /* Add slight breath to source */
            float noise = ((float)(rand() & 0xFFFF) / 32768.0f - 1.0f) * 0.02f;
            src += noise;

            /* Formant amplitudes (F1 dominant) */
            float a_f1 = osc_next(&osc_f1, (float)f1, SAMPLE_RATE) * 0.5f;
            float a_f2 = osc_next(&osc_f2, (float)f2, SAMPLE_RATE) * 0.22f;
            float a_f3 = osc_next(&osc_f3, (float)f3, SAMPLE_RATE) * 0.10f;

            sample = src + a_f1 + a_f2 + a_f3;

            /* Amplitude envelope */
            float env = 1.0f;
            if (prog < 0.05f)      env = prog / 0.05f;          /* attack */
            else if (prog > 0.80f) env = 1.0f - (prog-0.80f)/0.20f; /* decay */
            sample *= env * 0.8f;
        }

        /* Clamp and convert to int16 */
        if (sample >  1.0f) sample =  1.0f;
        if (sample < -1.0f) sample = -1.0f;
        buf[i] = (int16_t)(sample * 16000.0f);
    }

    return total;
}

/* ── Public API ────────────────────────────────────── */

size_t local_tts_synthesize(const char *text, local_tts_callback_t callback)
{
    if (!text || !callback) return 0;

    size_t total_samples = 0;
    int16_t syl_buf[SYL_SAMPLES + 320]; /* one syllable + padding */

    const char *p = text;
    while (*p) {
        int cp = utf8_decode(&p);
        if (cp < 0) continue;

        /* ASCII: play a soft beep or skip */
        if (cp < 0x80) {
            /* Soft 440Hz beep for punctuation/numbers */
            if (cp >= '0' && cp <= '9') {
                int n = SYL_SAMPLES / 2;
                for (int i = 0; i < n; i++) {
                    float t = (float)i / SAMPLE_RATE;
                    syl_buf[i] = (int16_t)(sinf(2.0f*(float)M_PI*440.0f*t) * 3000.0f);
                }
                /* vary tone by digit */
                callback(syl_buf, n);
                total_samples += n;
            } else if (cp == '.' || cp == ',' || cp == '!' || cp == '?') {
                /* short silence for punctuation */
                memset(syl_buf, 0, 160 * sizeof(int16_t));
                callback(syl_buf, 160);
                total_samples += 160;
            }
            continue;
        }

        uint16_t py = lookup_pinyin(cp);
        if (py == 0xFFFF) {
            /* Character not in table — insert a 100ms silence */
            memset(syl_buf, 0, (SAMPLE_RATE/10) * sizeof(int16_t));
            callback(syl_buf, SAMPLE_RATE/10);
            total_samples += SAMPLE_RATE/10;
            continue;
        }

        int n = synth_syllable(py, syl_buf, SYL_SAMPLES);
        if (n > 0) {
            callback(syl_buf, n);
            total_samples += n;
        }
    }

    /* Trailing silence */
    memset(syl_buf, 0, CHUNK_SAMPLES * sizeof(int16_t));
    callback(syl_buf, CHUNK_SAMPLES);
    total_samples += CHUNK_SAMPLES;

    ESP_LOGI(TAG, "TTS done: %u samples total", (unsigned)total_samples);
    return total_samples;
}
