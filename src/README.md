# SLAM Car ESP32-S3 固件说明

## 更新日期：2026-10-6

适配新电路板（硬件参考 `data/photo/电路图1.jpeg`，开发板参考 `data/pdf/ESP32-S3-DevKitC.pdf`）。

---

## 一、本次修改内容（2026-10-6）

| 模块 | 修改 |
|---|---|
| 引脚 | 全部引脚按新电路图重新分配，集中在 `config/AppConfig.h` |
| 底盘电机 | 驱动换成 DRV8701E（PWM + DIR），新增 `lib/Drv8701Control`，旧的 `PwmControl` 保留不用 |
| 编码器 | H1~H4 改到新引脚，新增电机/编码器方向反转配置 |
| 步进电机 | 串口改为 RX1=GPIO10 / TX1=GPIO11，引脚只在 `AppConfig.h` 定义一处（修复之前两处定义不一致的问题） |
| OLED | 新增 SSD1306 128×64 显示模块：`lib/OledDisplay`（底层封装）+ `apps/OledApp`（显示内容） |
| micro-ROS | 新增断线重连状态机：Agent 未启动时不再卡死，USB 断开后自动重连，断线时底盘自动停车 |
| App 结构 | 每个 App 拆成「硬件初始化（上电一次）」和「创建/销毁 ROS 实体（每次连接/断开）」 |
| WiFi 预留 | 新增速度指令统一入口 `CarControllerApp::setTargetVelocity()`，后期手机控制直接调用 |
| 机械臂 | 改由电脑控制，ESP32 代码保留但不启用 |
| 启用状态 | 底盘、IMU、步进电机、OLED 全部启用（之前 `main.cpp` 中底盘和 IMU 是注释掉的） |

---

## 二、实现的功能

### 1. 四轮差速底盘（`CarControllerApp`）
- 订阅 `/cmd_vel`（`geometry_msgs/Twist`），逆运动学解算四个轮子目标速度
- 每个轮子独立 PID 闭环（独立 FreeRTOS 任务，10ms 周期）
- 编码器计算轮速与里程计，发布 `/wheel_odom`（`nav_msgs/Odometry`，20Hz，`odom` → `base_footprint`）
- 与 ROS 断开连接时自动停车

### 2. 步进电机（`StepperMotorApp`）
- CN1、CN27 两个 Emm_V5 驱动器并联在同一条串口总线，地址分别为 1、2
- 订阅 `/stepper_motor_target`（`std_msgs/Float32MultiArray`）：`[电机1圈数, 电机2圈数]`，相对运动，正值 CW、负值 CCW
- 发布 `/stepper_motor_status`（1Hz）：当前位置估计（圈），`-2` 表示正在回零，`-1` 表示等待回零
- 可选上电自动碰撞回零（`kEnableAutoHoming`，默认关闭）

### 3. IMU（`ImuApp`）
- 维特 IMU，I2C 接口（地址 0x50），与 OLED 共用 SDA/SCL
- 发布 `/imu`（`sensor_msgs/Imu`，20Hz，`imu_link`）
- 未检测到 IMU 时自动跳过，不影响其他功能

### 4. OLED 显示（`OledApp`）
- SSD1306 128×64，I2C 地址 0x3C，200ms 刷新一次，运行在独立任务中，ROS 断线时照常显示
- 显示内容：

```
[ROS: CONNECTED     ]   ← ROS 状态（WAIT AGENT / CREATING NODE / CONNECTED / RECONNECTING）

v  +0.25 m/s            ← 整车线速度
w  +0.50 rad/s          ← 整车角速度

FL+0.25  FR+0.25        ← 四轮速度 m/s
RL+0.25  RR+0.25
Up 00:12:34             ← 运行时间
```

- 未接屏幕时自动跳过

### 5. micro-ROS 连接管理（`main.cpp`）

```
WAITING_AGENT --ping成功--> AGENT_AVAILABLE --创建实体--> CONNECTED
     ^                                                       |
     +---------------- DISCONNECTED <------ping失败----------+
```

- 通过 `Serial`（开发板 USB 转串口那个口）与电脑通信，波特率 115200
- 节点名 `starbot_arm_controller`，话题名与之前保持一致，电脑端无需修改

---

## 三、引脚分配

| 功能 | 信号 | GPIO |
|---|---|---|
| 左前电机 A（CN25） | PWMA / DIRA | 42 / 41 |
| 右前电机 B（CN25） | PWMB / DIRB | 21 / 38 |
| 左后电机 C（CN26） | PWMC / DIRC | 4 / 5 |
| 右后电机 D（CN26） | PWMD / DIRD | 16 / 15 |
| 左前编码器 H1 | ENC_A1 / ENC_A2 | 40 / 39 |
| 右前编码器 H2 | ENC_B1 / ENC_B2 | 48 / 47 |
| 左后编码器 H3 | ENC_C1 / ENC_C2 | 6 / 7 |
| 右后编码器 H4 | ENC_D1 / ENC_D2 | 18 / 17 |
| 步进电机串口（CN1、CN27） | RX1（ESP 接收）/ TX1（ESP 发送） | 10 / 11 |
| I2C（OLED + IMU） | SDA / SCL | 8 / 9 |
| micro-ROS | USB 串口 | 43 / 44（开发板内部） |
| 未使用 | NET1 / NET2 / NET12 / NET13 / NET14 | 1 / 2 / 12 / 13 / 14 |

---

## 四、代码结构

```
src/
├── main.cpp                    # 硬件初始化 + micro-ROS 连接状态机
├── config/AppConfig.h          # 所有引脚、话题名、参数
├── utils/RosAgentState.h       # ROS 连接状态枚举
└── apps/
    ├── CarControllerApp        # 底盘：cmd_vel、PID、里程计
    ├── StepperMotorApp         # 步进电机
    ├── ImuApp                  # IMU
    ├── OledApp                 # OLED 显示
    └── MicroRosArmControllerApp # 机械臂（保留，不启用）
lib/
├── Drv8701Control/             # 新增：DRV8701E 驱动
├── OledDisplay/                # 新增：SSD1306 封装
├── BujinControl/               # Emm_V5 步进电机协议
├── Kinematics/ PidController/ IMU/ ...
```

---

## 五、使用与调试

1. 编译上传：`pio run -t upload`
2. 电脑端启动 Agent（串口号按实际情况）：
   ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
   ```
3. OLED 第一行显示 `ROS: CONNECTED` 即连接成功
4. **首次装车方向校准**（轮子先架空）：
   ```bash
   ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
   ```
   - 哪个轮子往后转 → 把 `AppConfig.h` 中 `kMotorReversed` 对应位置改为 `true`
   - OLED 上哪个轮速为负 → 把 `kEncoderReversed` 对应位置改为 `true`
   - 顺序均为：左前、右前、左后、右后

---

## 六、已知问题与漏洞

### 硬件相关
1. **编码器电平风险**：H1~H4 由 +5V 供电，如果编码器输出 5V 信号，直接接 ESP32-S3（3.3V，不耐 5V）可能损坏芯片。需确认输出电平，必要时加分压或电平转换。
2. **GPIO38 与板载 RGB 灯共用**：DevKitC 上 GPIO38 经 R17（0Ω）接 RGB 灯，电机 B 换向时 RGB 灯会乱闪，功能不受影响，介意可拆 R17。
3. **未经实物验证**：本次代码仅通过编译，电机方向、编码器方向、PID 参数均需上板测试。

### 软件相关
4. **没有 `/cmd_vel` 超时保护**：只在与 Agent 断开时停车。如果 Agent 仍在线但发布 `/cmd_vel` 的节点崩溃，小车会一直保持最后一次的速度。
5. **步进电机串口忙等阻塞主循环**：`Emm_V5_Receive_Data` 每次最多忙等 200ms，发送步进指令时主循环被阻塞，期间轮速/里程计不更新，PID 使用的是旧速度。重连时 `rmw_uros_sync_session` 也会阻塞最多 1s。
6. **步进电机位置是开环估计**：`/stepper_motor_status` 发布的是累加的指令圈数，不是电机真实位置；电机堵转或丢步时数值不准。并且代码假设电机地址 = 下标 + 1。
7. **运动学数据无锁共享**：轮速和里程计在主循环中更新，PID 任务和 OLED 任务直接读取，没有加锁。单个 float 读写是原子的，但里程计结构体可能读到“半新半旧”的数据（影响很小）。`pid_mutex_` 已创建但未使用。
8. **I2C 总线跨任务共用**：OLED（独立任务）和 IMU（主循环）依赖 Arduino `Wire` 内部的锁，没有额外的应用层互斥；OLED 整屏刷新约 25ms。
9. **时间同步失败时时间戳错误**：如果 `rmw_uros_sync_session` 失败，`/wheel_odom` 和 `/imu` 的时间戳会从 0 开始。
10. **里程计仅靠编码器**：没有与 IMU 融合，打滑时航向角会漂移（可在电脑端用 robot_localization 融合）。
11. **micro-ROS 必须接“USB 转串口”那个口**：代码使用 `Serial`（UART0，开发板上的 CP2102 口），接原生 USB 口无法通信。
12. **机械臂代码引脚冲突**：`MicroRosArmControllerApp` 使用的 GPIO16/17 在新板上已是 PWMD / ENC_D2，切勿重新启用。`lib/MLTrol` 也未适配新引脚。

---

## 七、后续计划

1. **WiFi 手机控制**：ESP32 开 WiFi，手机网页摇杆控制小车方向并显示 ROS 状态；micro-ROS 仍走串口。速度指令统一调用 `CarControllerApp::setTargetVelocity()`，届时需要增加手机与 ROS 指令的优先级仲裁和超时停车。
2. 第二个功能：待定。
