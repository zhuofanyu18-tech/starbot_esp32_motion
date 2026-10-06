# StarBot ESP32 Motion

![ESP32-S3](https://img.shields.io/badge/MCU-ESP32--S3-E7352C)
![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E)
![micro-ROS](https://img.shields.io/badge/micro--ROS-serial-2B9FD9)
![PlatformIO](https://img.shields.io/badge/build-PlatformIO-F5822A)
![FreeRTOS](https://img.shields.io/badge/RTOS-FreeRTOS-5CB85C)

**StarBot SLAM 小车的下位机固件。** 运行在 ESP32-S3 上，负责四轮底盘的实时闭环控制、轮式里程计、步进电机、IMU 采集和状态显示，通过 micro-ROS 接入电脑端 ROS 2，同时提供 WiFi 热点供手机直接控制。

整车分工：电脑运行 ROS 2（SLAM / 导航 / 机械臂 MoveIt2），ESP32 只做“手脚”——执行速度指令、上报传感器数据，机械臂由电脑直接控制。

> 最近更新：2026-10-6 · 适配新电路板（DRV8701E + SSD1306 OLED）· WiFi 手机控制 · 实时任务架构重构

---

## 项目亮点

- **实时控制与通信解耦**：测速、里程计、PID 合并为固定在核 1 的 10ms 控制任务（最高优先级），micro-ROS 重连、串口发布、步进电机应答等耗时操作都不会影响电机控制。
- **安全停车**：USB 断开或 Agent 关闭时底盘立即停车；速度指令按来源可配置超时（ROS 默认按停才停，手机断开 0.5s 自动停）；控制任务卡死 5s 由看门狗复位。
- **自动重连**：Agent 未启动时 ESP32 正常运行并持续探测，连上后自动创建话题，断开后自动清理重来，OLED 实时显示连接状态。
- **双控制通道**：ROS 2 走 USB 串口，手机走 WiFi 热点 + 网页（WebSocket），两者同时运行，统一汇入一个速度指令入口。
- **模块化 App 层**：每个功能一个 App（硬件初始化 / ROS 实体 / 后台任务分离），所有引脚和参数集中在一个配置文件，引脚冲突在编译期报错。

---

## 功能概览

| 功能 | 说明 | 状态 |
|---|---|---|
| 四轮差速底盘 | 订阅 `/cmd_vel`，四轮独立 PID 闭环（100Hz），DRV8701E 驱动直流减速电机 | 待上板验证 |
| 轮式里程计 | 四路霍尔编码器（PCNT 硬件计数），发布 `/wheel_odom` 20Hz | 待上板验证 |
| 步进电机 | 两台 Emm_V5 闭环步进电机，串口总线控制，可选上电自动回零 | 待上板验证 |
| IMU | 维特 IMU（I2C），发布 `/imu` 20Hz | 待上板验证 |
| OLED 显示 | SSD1306 128×64：ROS 状态、WiFi 热点、整车速度、四轮速度、运行时间 | 待上板验证 |
| 断线重连 | Agent 未启动或 USB 断开时自动重连，断线时底盘自动停车 | 待上板验证 |
| WiFi 手机控制 | ESP32 开热点，手机浏览器控制板载 RGB 灯（开关 / 颜色 / 亮度） | 待上板验证 |
| 机械臂 | 已改由电脑端直接控制，ESP32 中保留代码但不启用 | — |

> 当前版本代码均已编译通过，因暂无硬件尚未实测；电机方向、编码器方向和 PID 参数需上板整定。

---

## 系统架构

```
┌─────────────────────── 电脑（ROS 2 Humble）───────────────────────┐        ┌──────────── 手机 ────────────┐
│  SLAM / 导航 / 键盘遥控 ──/cmd_vel──►                               │        │  浏览器 http://192.168.4.1    │
│                        ◄──/wheel_odom  /imu  /stepper_motor_status │        │  RGB 灯控制（后续：摇杆遥控）  │
│                    micro_ros_agent（串口 921600）                   │        └──────────────┬───────────────┘
└─────────────────────────────┬──────────────────────────────────────┘                       │ WiFi 热点
                              │ USB 串口                                                    │ WebSocket
┌─────────────────────────────▼────────────────────────────────────────────────────────────▼───────────────┐
│ ESP32-S3-N16R8                                                                                           │
│   核 1（实时）                                       核 0（后台）                                         │
│   car_ctrl  P10  10ms  编码器→轮速→里程计→PID→PWM    WiFi / lwIP 协议栈                                    │
│   loopTask  P5         micro-ROS 连接状态机/收发     httpd     P5  网页 + WebSocket                        │
│   stepper   P3   10ms  步进电机串口收发              rgb_led   P2  RGB 灯                                  │
│                                                      oled      P1  屏幕刷新                                │
└──────┬──────────────────────────┬───────────────────────┬──────────────┬─────────────┬──────────────────────┘
       │ PWM+DIR / PCNT 编码器     │ UART                  │ I2C          │ I2C         │ RMT
   DRV8701E ×4 + 电机 ×4       Emm_V5 步进 ×2            IMU           OLED        RGB 灯
```

任务之间只通过加锁的小结构体交换数据（速度指令 `Command`、状态快照 `State`），互不阻塞。详细说明见 [src/README.md](src/README.md#7-实时任务架构)。

---

## 硬件清单

| 部件 | 型号 / 说明 | 接口 |
|---|---|---|
| 主控 | ESP32-S3-N16R8 核心板（16MB Flash + 8MB PSRAM） | — |
| 电机驱动 | DRV8701E ×4（CN25、CN26） | PWM + DIR |
| 底盘电机 | 带霍尔编码器的直流减速电机 ×4（H1~H4） | 编码器 A/B 相 |
| 步进电机 | Emm_V5 闭环步进驱动 ×2（CN1、CN27，地址 1、2） | UART，GPIO10/11 |
| IMU | 维特 IMU，I2C 地址 0x50 | I2C，GPIO8/9 |
| 显示屏 | SSD1306 128×64，4 针 I2C，地址 0x3C | I2C，GPIO8/9 |
| 状态灯 | 核心板板载 WS2812（与右前编码器 ENC_B1 共用 GPIO48） | RMT，GPIO48 |

- 电路原理图：[data/photo/电路图1.jpeg](data/photo/电路图1.jpeg)
- 开发板资料：[data/pdf/ESP32-S3-DevKitC.pdf](data/pdf/ESP32-S3-DevKitC.pdf)（官方 v1.1 原理图中 RGB 灯为 GPIO38，本项目所用核心板实测为 GPIO48）
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

# 前进 0.1 m/s（首次测试请先把轮子架空）；ROS 指令默认不超时，发 0 速度才会停
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"

# 查看里程计
ros2 topic echo /wheel_odom
```

首次装车如果发现某个轮子转向或编码器方向相反，修改 [src/config/AppConfig.h](src/config/AppConfig.h) 中的 `kMotorReversed` / `kEncoderReversed` 即可，详见 [src/README.md](src/README.md#五使用与调试)。

### 5. 手机控制（WiFi）

1. 手机连接 WiFi 热点 **`StarBot-ESP32`**，密码 **`starbot123`**
2. 浏览器打开 **http://192.168.4.1**（OLED 第 2 行也会显示地址）
3. 在网页上控制 RGB 灯的开关、颜色和亮度，多台手机同时打开会自动同步

> 板载 RGB 灯（GPIO48）与右前编码器 H2 共用引脚：启用灯时右前编码器停用、右前轮速由右后轮代替，测试灯时请拔掉 H2。小车正式运行时在 `AppConfig.h` 中设置 `kEnableRgbLed = false`。

---

## 目录结构

```
├── platformio.ini            # PlatformIO 配置与依赖
├── src/
│   ├── main.cpp              # 入口：硬件初始化、任务分配、micro-ROS 连接状态机
│   ├── config/AppConfig.h    # 所有引脚、话题名、任务优先级、底盘与 PID 参数
│   ├── apps/                 # 应用层：底盘 / 步进电机 / IMU / OLED / WiFi / RGB 灯 / 机械臂（停用）
│   ├── web/index.html        # 手机控制网页（编译时嵌入固件）
│   ├── utils/                # 工具：ROS 消息内存分配、连接状态定义
│   └── README.md             # 固件详细说明（修改记录、引脚、任务架构、已知问题）
├── lib/
│   ├── Drv8701Control/       # DRV8701E 电机驱动
│   ├── PcntQuadEncoder/      # PCNT 正交编码器（无中断、无竞态）
│   ├── Kinematics/           # 差速运动学与里程计
│   ├── PidController/        # PID 控制器
│   ├── BujinControl/         # Emm_V5 步进电机串口协议
│   ├── IMU/                  # 维特 IMU 驱动
│   ├── OledDisplay/          # SSD1306 OLED 封装
│   ├── RgbLed/               # WS2812 RGB 灯封装
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
| `kControlPeriodMs` | 10 | 控制周期（测速 + PID） |
| `kMotorReversed` / `kEncoderReversed` | 全部 `false` | 电机 / 编码器方向反转 |
| `kRosCmdTimeoutMs` | 0 | ROS 速度指令超时，0 = 不超时（按前进一直走，发 0 速度才停） |
| `kWifiCmdTimeoutMs` | 500 | 手机速度指令超时（网页持续发心跳，手机断开 0.5s 后停车） |
| `kMicroRosBaudrate` | 921600 | micro-ROS 串口波特率，需与 Agent 的 `-b` 参数一致 |
| `kEnableAutoHoming` | `false` | 步进电机上电自动回零 |
| `kOledI2cAddress` | `0x3C` | OLED 地址（部分模块为 `0x3D`） |
| `kEnableWifi` | `true` | 开启 WiFi 热点和手机控制网页 |
| `kWifiApSsid` / `kWifiApPassword` | `StarBot-ESP32` / `starbot123` | 热点名称和密码（请修改默认密码） |
| `kEnableRgbLed` | `true` | 启用板载 RGB 灯（会停用右前编码器） |

---

## 注意事项

- **编码器电平**：编码器由 5V 供电，请确认输出信号为 3.3V 电平，否则可能损坏 ESP32-S3。
- **波特率 921600**：电脑端 Agent 的启动命令、launch 文件需与固件一致。
- **ROS 速度指令默认不超时**：按前进会一直走，发 0 速度才停；与 Agent 断开时自动停车。如需超时保护，设置 `kRosCmdTimeoutMs`。
- **WiFi 无身份验证**：连上热点即可控制，请修改默认热点密码。

完整的已知问题列表见 [src/README.md](src/README.md#六已知问题与漏洞)。

---

## 更新日志

**2026-10-6**
- 适配新电路板：DRV8701E 电机驱动、新引脚分配、SSD1306 OLED 显示
- micro-ROS 断线自动重连，断线自动停车
- WiFi 手机控制第一阶段：热点 + 网页控制 RGB 灯
- 实时任务架构重构：10ms 控制任务独立运行、步进电机异步、编码器竞态修复、串口提速到 921600、看门狗

---

## 后续计划

- [x] 新电路板适配（DRV8701E / OLED / 新引脚）
- [x] 实时任务架构优化与速度指令超时（可配置）
- [x] WiFi 手机控制（第一阶段）：手机网页控制 RGB 灯开关、颜色、亮度
- [ ] 上板验证：电机 / 编码器方向校准、PID 整定
- [ ] WiFi 手机控制（第二阶段）：网页摇杆控制小车方向、显示 ROS 状态和实时速度、手机与 ROS 指令仲裁
- [ ] 下一版 PCB：ENC_B1 改到空闲引脚（GPIO1/2/12/13/14），解除与板载 RGB 灯的冲突
- [ ] 第二个扩展功能（待定）
