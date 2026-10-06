# StarBot ESP32 Motion

基于 **ESP32-S3 + micro-ROS（ROS 2 Humble）** 的移动机器人下位机固件，负责 SLAM 小车的底盘运动控制、传感器数据采集和状态显示，通过 USB 串口与电脑端 ROS 2 通信。

> 最近更新：2026-10-6，适配新电路板（DRV8701E 电机驱动 + SSD1306 OLED）

---

## 功能概览

| 功能 | 说明 |
|---|---|
| 四轮差速底盘 | 订阅 `/cmd_vel`，四轮独立 PID 闭环，DRV8701E 驱动直流减速电机 |
| 轮式里程计 | 四路霍尔编码器测速，发布 `/wheel_odom` |
| 步进电机 | 两台 Emm_V5 闭环步进电机，串口总线控制，可选上电自动回零 |
| IMU | 维特 IMU（I2C），发布 `/imu` |
| OLED 显示 | SSD1306 128×64，实时显示 ROS 连接状态、小车速度、四轮速度 |
| 断线重连 | Agent 未启动或 USB 断开时自动重连，断线时底盘自动停车 |
| WiFi 手机控制 | ESP32 开热点，手机浏览器控制板载 RGB 灯（开关 / 颜色 / 亮度），与 ROS 同时运行 |
| 机械臂 | 已改由电脑端直接控制，ESP32 中保留代码但不启用 |

---

## 系统架构

```
┌────────────────────────── 电脑（ROS 2 Humble）──────────────────────────┐
│  导航 / SLAM / 键盘遥控  ──/cmd_vel──►                                   │
│                         ◄──/wheel_odom, /imu, /stepper_motor_status─   │
│                     micro_ros_agent（串口 921600）                      │
└───────────────────────────────┬────────────────────────────────────────┘
                                │ USB 串口
┌───────────────────────────────▼─────────────────────────────────────────┐
│                         ESP32-S3-N16R8                                  │
│   CarControllerApp   StepperMotorApp   ImuApp   OledApp                 │
└──────┬──────────────────────┬──────────────┬─────────┬──────────────────┘
       │PWM+DIR / 编码器       │UART          │I2C      │I2C
   DRV8701E ×4 + 电机 ×4   Emm_V5 步进 ×2    IMU      OLED
```

---

## 硬件清单

| 部件 | 型号 / 说明 | 接口 |
|---|---|---|
| 主控 | ESP32-S3-N16R8 核心板（DevKitC 兼容） | — |
| 电机驱动 | DRV8701E ×4（CN25、CN26） | PWM + DIR |
| 底盘电机 | 带霍尔编码器的直流减速电机 ×4（H1~H4） | 编码器 A/B 相 |
| 步进电机 | Emm_V5 闭环步进驱动 ×2（CN1、CN27，地址 1、2） | UART，GPIO10/11 |
| IMU | 维特 IMU，I2C 地址 0x50 | I2C，GPIO8/9 |
| 显示屏 | SSD1306 128×64，4 针 I2C，地址 0x3C | I2C，GPIO8/9 |

- 电路原理图：[data/photo/电路图1.jpeg](data/photo/电路图1.jpeg)
- 开发板资料：[data/pdf/ESP32-S3-DevKitC.pdf](data/pdf/ESP32-S3-DevKitC.pdf)
- 完整引脚表：[src/README.md](src/README.md#三引脚分配)

---

## ROS 2 接口

节点名：`starbot_arm_controller`

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | 订阅 | 底盘速度指令（`linear.x` m/s，`angular.z` rad/s） |
| `/wheel_odom` | `nav_msgs/Odometry` | 发布 20Hz | 轮式里程计，`odom` → `base_footprint` |
| `/imu` | `sensor_msgs/Imu` | 发布 20Hz | IMU 数据，`imu_link` |
| `/stepper_motor_target` | `std_msgs/Float32MultiArray` | 订阅 | `[电机1圈数, 电机2圈数]`，相对运动，正值 CW |
| `/stepper_motor_status` | `std_msgs/Float32MultiArray` | 发布 1Hz | 当前位置估计（圈），`-2` 回零中，`-1` 等待回零 |

---

## 快速开始

### 1. 环境准备

- [PlatformIO](https://platformio.org/)（VS Code 插件或命令行）
- 电脑端 ROS 2 Humble + [micro-ROS Agent](https://github.com/micro-ROS/micro-ROS-Agent)

首次编译会自动下载依赖（micro-ROS、Adafruit SSD1306 等），需要联网。

### 2. 编译与烧录

```bash
pio run              # 编译
pio run -t upload    # 烧录
```

> USB 线请接开发板上的 **USB 转串口** 口（不是原生 USB 口），micro-ROS 使用的是 `Serial`。

### 3. 启动 Agent

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600
```

OLED 第一行显示 `ROS: CONNECTED` 即表示连接成功。一直显示 `ROS: WAIT AGENT` 时，检查波特率是否为 **921600**、USB 线是否接在 USB 转串口口上。

### 4. 测试底盘

```bash
# 查看话题
ros2 topic list

# 前进 0.1 m/s（首次测试请先把轮子架空）
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"

# 查看里程计
ros2 topic echo /wheel_odom
```

### 5. 手机控制（WiFi）

1. 手机连接 WiFi 热点 **`StarBot-ESP32`**，密码 **`starbot123`**
2. 浏览器打开 **http://192.168.4.1**
3. 在网页上控制 RGB 灯的开关、颜色和亮度

> 板载 RGB 灯（GPIO48）与右前编码器 H2 共用引脚，测试灯时请拔掉 H2。小车正式运行时在 `AppConfig.h` 中设置 `kEnableRgbLed = false`。

首次装车如果发现某个轮子转向或编码器方向相反，修改 [src/config/AppConfig.h](src/config/AppConfig.h) 中的 `kMotorReversed` / `kEncoderReversed` 即可，详见 [src/README.md](src/README.md#五使用与调试)。

---

## 目录结构

```
├── platformio.ini            # PlatformIO 配置与依赖
├── src/
│   ├── main.cpp              # 入口：硬件初始化 + micro-ROS 连接状态机
│   ├── config/AppConfig.h    # 所有引脚、话题名、底盘与 PID 参数
│   ├── apps/                 # 应用层：底盘 / 步进电机 / IMU / OLED / WiFi / RGB 灯 / 机械臂（停用）
│   ├── web/index.html        # 手机控制网页（编译时嵌入固件）
│   ├── utils/                # 工具：ROS 消息内存分配、连接状态定义
│   └── README.md             # 固件详细说明（修改记录、引脚、已知问题）
├── lib/
│   ├── Drv8701Control/       # DRV8701E 电机驱动
│   ├── OledDisplay/          # SSD1306 OLED 封装
│   ├── RgbLed/               # WS2812 RGB 灯封装
│   ├── BujinControl/         # Emm_V5 步进电机串口协议
│   ├── Kinematics/           # 差速运动学与里程计
│   ├── PidController/        # PID 控制器
│   ├── PcntQuadEncoder/      # PCNT 正交编码器（无中断、无竞态）
│   ├── IMU/                  # 维特 IMU 驱动
│   └── ...                   # 机械臂舵机等旧模块（保留）
└── data/                     # 电路原理图、开发板资料
```

---

## 常用配置

所有参数集中在 [src/config/AppConfig.h](src/config/AppConfig.h)：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `kWheelDiameterMm` | 125 | 轮子直径（mm） |
| `kWheelBaseMm` | 370 | 轮距（mm） |
| `kEncoderPulsesPerRevolution` | 14000 | 轮子转一圈的编码器脉冲数 |
| `Kp` / `Ki` / `Kd` | 1.0 / 0.3 / 0.5 | 轮速 PID 参数 |
| `kRosCmdTimeoutMs` | 0 | ROS 速度指令超时，0 = 不超时（按前进一直走，发 0 速度才停） |
| `kWifiCmdTimeoutMs` | 500 | 手机速度指令超时（网页持续发心跳，手机断开 0.5s 后停车） |
| `kMicroRosBaudrate` | 921600 | micro-ROS 串口波特率，需与 Agent 的 `-b` 参数一致 |
| `kMotorReversed` / `kEncoderReversed` | 全部 `false` | 电机 / 编码器方向反转 |
| `kEnableAutoHoming` | `false` | 步进电机上电自动回零 |
| `kOledI2cAddress` | `0x3C` | OLED 地址（部分模块为 `0x3D`） |
| `kEnableWifi` | `true` | 开启 WiFi 热点和手机控制网页 |
| `kWifiApSsid` / `kWifiApPassword` | `StarBot-ESP32` / `starbot123` | 热点名称和密码（请修改默认密码） |
| `kEnableRgbLed` | `true` | 启用板载 RGB 灯（会停用右前编码器） |

---

## 注意事项

- **编码器电平**：编码器由 5V 供电，请确认输出信号为 3.3V 电平，否则可能损坏 ESP32-S3。
- **代码尚未上板验证**：当前版本仅通过编译，电机方向和 PID 参数需实测调整。
- **ROS 速度指令默认不超时**：按前进会一直走，发 0 速度才停；与 Agent 断开时自动停车。如需超时保护，设置 `kRosCmdTimeoutMs`。
- **波特率已改为 921600**：电脑端 Agent 的启动命令、launch 文件需要同步修改。

完整的已知问题列表见 [src/README.md](src/README.md#六已知问题与漏洞)。

---

## 后续计划

- [x] WiFi 手机控制（第一阶段）：手机网页控制 RGB 灯开关、颜色、亮度
- [ ] WiFi 手机控制（第二阶段）：网页摇杆控制小车方向、显示 ROS 状态和实时速度
- [ ] `/cmd_vel` 超时保护与多指令源优先级仲裁
- [ ] 第二个扩展功能（待定）
