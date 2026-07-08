# P4 Robot — ESP32-P4 智能机器人平台

基于 **ESP32-P4** 双核 MCU，配备 **C5 AI 语音协处理器**，集运动控制、机器视觉、环境感知、语音交互于一体的全功能机器人平台。

## 硬件架构

```
┌─────────────────────────────────────────────────┐
│                 P4 (主控)                        │
│  ┌─────────┐ ┌──────────┐ ┌──────────────────┐  │
│  │ 运动控制 │ │ 视觉处理   │ │ 环境传感器       │  │
│  │ · PID   │ │ · OV5647 │ │ · SHT30 温湿度   │  │
│  │ · 里程计 │ │ · 颜色   │ │ · MQ-135 有害气体│  │
│  │ · 路径  │ │ · 线条   │ │ · MQ-136 H₂S    │  │
│  │ · IMU   │ │ · 手势   │ │ · VL53L1X 避障   │  │
│  │ · 避障  │ │ · QR码   │ └──────────────────┘  │
│  └─────────┘ │ · 人脸AI │                       │
│               └──────────┘                       │
│  ┌──────────┐ ┌────────────┐                    │
│  │ 7寸 LCD  │ │ C5 UART桥  │ ←→ C5 (AI语音)     │
│  │ LVGL UI  │ │            │    ASR→LLM→TTS     │
│  │ 触控交互  │ │            │                    │
│  └──────────┘ └────────────┘                    │
└─────────────────────────────────────────────────┘
```

### 核心硬件

| 模块 | 型号 | 说明 |
|------|------|------|
| **主控** | ESP32-P4 | 双核 MCU, MIPI CSI/DSI, NPU |
| **语音协处理器** | ESP32-C5 | 离线语音识别 (ESP-SR) |
| **显示屏** | 7寸 1024×600 | MIPI DSI, EK79007, LVGL 驱动 |
| **触摸** | GT911 | I2C 电容触摸 |
| **摄像头** | OV5647 | 500万像素, MIPI CSI, 支持 RAW8/RGB565 |
| **电机驱动** | TB6612 × 2 | 四路独立驱动 |
| **编码器** | 11 PPR 霍尔编码器 × 4 |
| **IMU** | MPU6050 | 6轴姿态, 陀螺零偏校准 (暂禁用) |
| **避障** | VL53L1X | 激光 ToF, 最远 4m |
| **温湿度** | SHT30 | 精度 ±2%RH / ±0.3°C |
| **有害气体** | MQ-135 | NH₃/苯/NOx/烟雾 |
| **硫化氢** | MQ-136 | H₂S 泄漏检测 |
| **OLED** | 0.96寸 | I2C, 128×64 |

### GPIO 引脚分配

| 功能 | GPIO | 备注 |
|------|------|------|
| M1 (左前) | PWM=2, IN1=45, IN2=1 | |
| M2 (左后) | PWM=6, IN1=3, IN2=4 | |
| M3 (右前) | PWM=9, IN1=22, IN2=23 | IN1/2 移自 GPIO7/8 释放相机 SCCB |
| M4 (右后) | PWM=20, IN1=28, IN2=29 | IN1/2 移自 GPIO49/50 释放 ADC |
| STBY 使能 | 21 | |
| **I2C 共享总线** | SDA=7, SCL=8 | 触摸 + 相机 SCCB |
| **I2C0 (传感器)** | SDA=41, SCL=42 | OLED + SHT30 |
| **I2C1 (传感器)** | SDA=54, SCL=53 | MPU6050 + VL53L1X |
| UART1 (P4↔C5) | TX=10, RX=11 | 38400 baud, TLV 协议 |
| 按键 | FWD=48, LEFT=39, RIGHT=43, BACK=44 | 内部上拉, 低电平有效 |
| LCD 背光 | 26 | PWM 5kHz |
| LCD 复位 | 27 | |
| ToF XSHUT | 25 | |
| 相机 XCLK | 24 | 24MHz, PCB 固定 |

## 软件架构

### 组件结构

```
hello_world/
├── main/                    # 主程序
│   └── main_btn.c          # 初始化 + 任务调度 + 按键/触摸交互
├── components/
│   ├── motion/             # 运动控制
│   │   ├── motion_control.c   # 直线/转弯/弧线运动, 梯形速度曲线
│   │   ├── speed_profile.c    # 梯形/矩形速度规划
│   │   ├── pid.c              # 速度环 PID
│   │   ├── odometry.c         # 四轮里程计
│   │   ├── waypoint.c         # 路径点导航 (预设巡逻路径)
│   │   └── imu_fusion.c       # 互补滤波 IMU 姿态融合
│   ├── hardware/           # 硬件抽象
│   │   ├── motor_control.c    # TB6612 四路电机驱动
│   │   ├── encoder.c          # 霍尔编码器计数
│   │   └── mpu6050.c          # MPU6050 I2C 驱动
│   ├── camera/             # 摄像头
│   │   └── cam_ov5647.c       # OV5647 MIPI CSI 驱动
│   ├── vision/             # 视觉处理
│   │   ├── vision_core.c      # 视觉管线核心
│   │   ├── vision_color.c     # 颜色识别 (HSV 阈值)
│   │   ├── vision_line.c      # 线条循迹
│   │   ├── vision_gesture.c   # 手势识别
│   │   └── vision_qr/         # QR 码解码 (Quirc 库)
│   ├── vision_ai/          # AI 视觉 (ESP-DL)
│   │   └── vision_ai.cpp      # 人脸/行人检测, NPU 加速
│   ├── display/            # 显示
│   │   ├── display_lcd.c      # 7寸 MIPI DSI + LVGL + 仪表盘
│   │   └── display_touch.c    # GT911 触摸驱动
│   ├── interaction/        # 交互
│   │   ├── button.c           # 4按键消抖驱动
│   │   └── oled_display.c     # 0.96寸 OLED
│   ├── sensors/            # 环境传感器
│   │   ├── env_sensor.c       # 传感器采集任务
│   │   ├── sht30.c            # SHT30 温湿度
│   │   ├── mq135.c            # MQ-135 有害气体
│   │   └── mq136.c            # MQ-136 硫化氢
│   ├── tof_avoid/          # 避障
│   │   └── tof_avoid.c        # VL53L1X 测距 + 避障逻辑
│   ├── c5_bridge/          # P4↔C5 UART 通信
│   │   └── c5_bridge.c        # TLV 协议桥接
│   ├── sr_slave/           # 语音识别 (ESP-SR)
│   │   ├── sr_task.c          # 语音识别任务
│   │   ├── sr_esp_sr.c        # ESP-SR 多模型管理器
│   │   └── vision/            # 语音结果映射到视觉
│   └── ...
```

### UART 桥接协议 (P4 ↔ C5)

P4 通过 UART1 (38400 baud) 与 C5 AI 协处理器通信，协议格式为 TLV:

```
[0xA5][type:1B][len:2B BE][payload:N][checksum:1B XOR]
```

**P4 → C5 方向:**
- `0x03` — 唤醒事件
- `0x04` — 本地语音命令
- `0x05` — 环境传感器数据
- `0x20` — 视觉识别结果
- `0x23` — 障碍物告警

**C5 → P4 方向:**
- `0x10` — 电机控制 (前进/后退/左转/右转/停止/调速)
- `0x12` — 预设巡逻路径执行
- `0x13` — 直线指令 (浮点距离 + 速度)
- `0x14` — 转弯指令 (浮点角度 + 速度)
- `0x15` — 导航停止
- `0x40` — AI 语音文字推送到 LCD 显示
- `0xF0` — 心跳包

### 任务划分

| 任务 | 核心 | 优先级 | 说明 |
|------|------|--------|------|
| `vision` | Core 1 | 3 | 摄像头帧处理 + AI 检测 |
| `avoid` | - | 4 | 避障动作执行 |
| `button` | - | 4 | 按键扫描 |
| `status` | - | 2 | 状态更新 + 仪表盘刷新 |
| 运动控制 | Core 1 | 5 | PID 速度环 100Hz |
| C5 bridge | - | 3 | UART 命令侦听 |

### C5 AI 语音协处理器 (`c5/`)

C5 协处理器源码位于 [`c5/`](c5/) 目录，详细说明见 [c5/README.md](c5/README.md)。

```
c5/
├── main/                    # 主程序
├── components/
│   ├── c5_agent/            # 语音引擎、唤醒词、本地 TTS
│   ├── c5_audio/            # I2S 全双工驱动、音频采集/播放
│   ├── c5_ble/              # BLE NUS 蓝牙串口遥控
│   ├── c5_cloud/            # WiFi + Cloud API (LLM/ASR/TTS)
│   └── c5_comm/             # P4 UART 桥接协议
├── tools/                   # 工具脚本
└── README.md
```

## 快速开始

### 环境要求

- ESP-IDF v5.5+
- ESP32-P4 / ESP32-C5 工具链
- Python 3.8+

### 构建与烧录

```bash
# 设置目标芯片
idf.py set-target esp32p4

# 配置 (可选)
idf.py menuconfig

# 编译
idf.py build

# 烧录 + 串口监视
idf.py -p COM3 flash monitor
```

### 按键操作

| 按键 | GPIO | 功能 |
|------|------|------|
| ▲ FWD | 48 | 速度档位切换: LOW(30%) → MED(60%) → HIGH(100%) |
| ◀ LEFT | 39 | 视觉模式切换: OFF→COLOR→LINE→GESTURE |
| ▶ RIGHT | 43 | 巡逻路径切换: PATH0→1→2→3→STOP |
| ▼ BACK | 44 | 紧急停止 (最高优先级) |

## 许可证

[MIT](LICENSE) © 2026
