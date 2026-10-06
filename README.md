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
| 机械臂 | 已改由电脑端直接控制，ESP32 中保留代码但不启用 |

---

## 系统架构

```
┌────────────────────────── 电脑（ROS 2 Humble）──────────────────────────┐
│  导航 / SLAM / 键盘遥控  ──/cmd_vel──►                                   │
│                         ◄──/wheel_odom, /imu, /stepper_motor_status─   │
│                     micro_ros_agent（串口 115200）                      │
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
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

OLED 第一行显示 `ROS: CONNECTED` 即表示连接成功。

### 4. 测试底盘

```bash
# 查看话题
ros2 topic list

# 前进 0.1 m/s（首次测试请先把轮子架空）
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"

# 查看里程计
ros2 topic echo /wheel_odom
```

首次装车如果发现某个轮子转向或编码器方向相反，修改 [src/config/AppConfig.h](src/config/AppConfig.h) 中的 `kMotorReversed` / `kEncoderReversed` 即可，详见 [src/README.md](src/README.md#五使用与调试)。

---

## 目录结构

```
├── platformio.ini            # PlatformIO 配置与依赖
├── src/
│   ├── main.cpp              # 入口：硬件初始化 + micro-ROS 连接状态机
│   ├── config/AppConfig.h    # 所有引脚、话题名、底盘与 PID 参数
│   ├── apps/                 # 应用层：底盘 / 步进电机 / IMU / OLED / 机械臂（停用）
│   ├── utils/                # 工具：ROS 消息内存分配、连接状态定义
│   └── README.md             # 固件详细说明（修改记录、引脚、已知问题）
├── lib/
│   ├── Drv8701Control/       # DRV8701E 电机驱动
│   ├── OledDisplay/          # SSD1306 OLED 封装
│   ├── BujinControl/         # Emm_V5 步进电机串口协议
│   ├── Kinematics/           # 差速运动学与里程计
│   ├── PidController/        # PID 控制器
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
| `kMotorReversed` / `kEncoderReversed` | 全部 `false` | 电机 / 编码器方向反转 |
| `kEnableAutoHoming` | `false` | 步进电机上电自动回零 |
| `kOledI2cAddress` | `0x3C` | OLED 地址（部分模块为 `0x3D`） |

---

## 注意事项

- **编码器电平**：编码器由 5V 供电，请确认输出信号为 3.3V 电平，否则可能损坏 ESP32-S3。
- **代码尚未上板验证**：当前版本仅通过编译，电机方向和 PID 参数需实测调整。
- **没有 `/cmd_vel` 超时停车**：只有在与 Agent 断开连接时才会自动停车。

完整的已知问题列表见 [src/README.md](src/README.md#六已知问题与漏洞)。

---

## 后续计划

- [ ] WiFi 手机控制：手机网页摇杆控制小车方向、显示 ROS 状态（micro-ROS 仍走串口）
- [ ] `/cmd_vel` 超时保护与多指令源优先级仲裁
- [ ] 第二个扩展功能（待定）
