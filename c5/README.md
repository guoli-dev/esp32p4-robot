# C5 AI Voice Co-processor

ESP32-C5 语音协处理器固件，通过 UART 与 P4 主控通信，提供 AI 语音交互、BLE 遥控和环境数据中继功能。

## 架构

```
手机 App ── BLE ──→ C5 ── UART ──→ P4 (运动控制)
                         ──→ Cloud LLM/ASR/TTS (WiFi)
传感器 ←── UART ←── P4 ──→ C5 ── BLE ──→ 手机 App
                               ── I2S ──→ 麦克风 + 扬声器
```

## 硬件连线 (C5 引脚)

| 功能 | GPIO | 连接对象 |
|------|------|----------|
| P4 UART TX | GPIO25 → P4 J2-37(GPIO10) | 38400 baud |
| P4 UART RX | GPIO24 ← P4 J2-30(GPIO11) | 38400 baud |
| I2S BCLK | GPIO4 | INMP441 + MAX98357A (共享) |
| I2S WS | GPIO5 | INMP441 + MAX98357A (共享) |
| I2S DOUT | GPIO23 → MAX98357A DIN | TTS 输出 |
| I2S DIN | GPIO6 ← INMP441 SD | 麦克风输入 |

## 组件

| 组件 | 功能 |
|------|------|
| `c5_agent` | 语音引擎、唤醒词、本地 TTS、状态管理 |
| `c5_audio` | I2S 全双工驱动、音频采集、TTS 播放、PCM 样本 |
| `c5_ble` | BLE NUS (Nordic UART Service) 蓝牙串口 |
| `c5_cloud` | WiFi 管理 + Cloud API (LLM/ASR/TTS) |
| `c5_comm` | P4 UART 桥接协议 |

## 通信协议

与 P4 使用 TLV 帧格式: `[0xA5][type:1B][len:2B BE][payload:N][checksum:1B]`

详见 [c5_comm/p4_protocol.h](c5_comm/p4_protocol.h) 和 P4 侧 [c5_bridge](https://github.com/guoli-dev/esp32p4-robot/blob/main/components/c5_bridge/c5_bridge.h)。

## 快速开始

```bash
cd c5
idf.py set-target esp32c5
idf.py build
idf.py -p COM3 flash monitor
```

### 控制台命令

| 命令 | 说明 |
|------|------|
| `!status` | 显示 WiFi/Cloud/BLE 状态 |
| `!key llm sk-xxxxx` | 设置 LLM API Key |
| `!key openai sk-xxxxx` | 设置 ASR/TTS API Key |
| `<text>` | 与 AI 对话 |
