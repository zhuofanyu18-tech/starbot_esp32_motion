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
| WiFi 手机控制 | 新增 `WifiApp`：ESP32 开热点，手机浏览器打开网页通过 WebSocket 控制，第一阶段实现 RGB 灯开关/颜色/亮度 |
| RGB 灯 | 新增 `lib/RgbLed` + `apps/RgbLedApp`，板载 WS2812（GPIO48），队列 + 工作任务写灯 |
| WiFi 预留 | 新增速度指令统一入口 `CarControllerApp::setTargetVelocity()`，后期手机遥控直接调用 |
| 机械臂 | 改由电脑控制，ESP32 代码保留但不启用 |
| 启用状态 | 底盘、IMU、步进电机、OLED 全部启用（之前 `main.cpp` 中底盘和 IMU 是注释掉的） |
| 任务架构 | 测速 + PID + 里程计合并为固定在核 1 的 10ms 控制任务；步进电机串口移到独立任务；WiFi/网页/OLED/RGB 固定在核 0；控制任务和 loop 加看门狗 |
| 编码器 | 新增 `lib/PcntQuadEncoder` 取代 `Esp32PcntEncoder`：去掉溢出中断，修复读数竞态导致的 ±100 脉冲跳变 |
| 串口 | micro-ROS 波特率 115200 → **921600**（原带宽不足），收发缓冲区加大到 2KB |
| 指令超时 | 速度指令记录来源和时间戳，ROS 默认不超时（按停才停），WiFi 默认 500ms 超时 |

---

## 二、实现的功能

### 1. 四轮差速底盘（`CarControllerApp`）
- 订阅 `/cmd_vel`（`geometry_msgs/Twist`），逆运动学解算四个轮子目标速度
- 每个轮子独立 PID 闭环（控制任务固定在核 1，严格 10ms 一次：测速 → 里程计 → PID → PWM）
- 编码器计算轮速与里程计，发布 `/wheel_odom`（`nav_msgs/Odometry`，20Hz，`odom` → `base_footprint`）
- 与 ROS 断开连接时自动停车
- 速度指令超时：`kRosCmdTimeoutMs`（默认 0 = 不超时，保持最后一条指令直到发 0 速度）、`kWifiCmdTimeoutMs`（默认 500ms）

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
AP 192.168.4.1  x1      ← WiFi 热点地址 + 已连接手机数（WiFi 关闭时为空）
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

- 通过 `Serial`（开发板 USB 转串口那个口）与电脑通信，波特率 **921600**（电脑端 Agent 必须一致）
- 节点名 `starbot_arm_controller`，话题名与之前保持一致，电脑端无需修改

### 6. WiFi 手机控制（`WifiApp` + `RgbLedApp`）

参考 `esp32_idf_ws/starbot_wifi` 的架构：网络回调只做校验，通过队列交给工作任务操作硬件。

- ESP32 开热点：SSID `StarBot-ESP32`，密码 `starbot123`（`AppConfig.h` 中修改）
- 手机连上热点后，浏览器打开 **http://192.168.4.1**（OLED 第 2 行会显示地址和已连接手机数）
- 网页功能：开灯 / 关灯、8 个预设颜色、自定义取色、亮度 0~100%；多台手机同时打开时状态自动同步
- 与 micro-ROS 串口同时运行，互不影响

WebSocket 接口（`ws://192.168.4.1/ws`，文本指令）：

| 指令 | 说明 |
|---|---|
| `STATE_GET` | 查询灯状态 |
| `LED_ON` / `LED_OFF` | 开灯 / 关灯 |
| `LED_COLOR,r,g,b` | 设置颜色（0~255），同时开灯 |
| `LED_BRIGHTNESS,p` | 设置亮度（0~100） |

灯状态变化后，ESP32 向所有手机广播：`{"type":"led","enabled":true,"on":true,"r":255,"g":0,"b":0,"brightness":50}`；
出错时回复 `{"type":"error","code":"...","message":"..."}`。

> ⚠️ 板载 RGB 灯在 **GPIO48**，与右前编码器 ENC_B1（H2）是同一根线。`kEnableRgbLed=true` 时右前编码器不初始化，
> 右前轮速改用右后编码器代替（保证右前轮 PID 不会失控），**测试 WiFi 灯时请拔掉 H2**。
> 小车正式跑时把 `kEnableRgbLed` 改为 `false`，或外接一颗 WS2812 到空闲引脚并修改 `kRgbLedPin`。

---

### 7. 实时任务架构

```
核 1（实时）                                   核 0（后台）
┌──────────────────────────────────┐          ┌──────────────────────────────┐
│ car_ctrl  优先级10  10ms 周期     │          │ WiFi / lwIP 系统任务          │
│  编码器→轮速→里程计→PID→PWM      │          │ httpd     优先级5  网页/WS     │
│  写 State 快照  读 Command        │          │ rgb_led   优先级2  队列触发    │
├──────────────────────────────────┤          │ oled      优先级1  200ms 刷新  │
│ loopTask  优先级5  micro-ROS      │          └──────────────────────────────┘
│  连接状态机 / executor / 发布      │
├──────────────────────────────────┤
│ stepper   优先级3  10ms 周期      │
│  步进串口收发（等待应答时让出CPU） │
└──────────────────────────────────┘
```

- **任务之间只通过加锁的小结构体交换数据**（拷贝几十字节，微秒级），不会互相阻塞：
  - `Command`：`setTargetVelocity()` 写入（ROS 回调 / WiFi 网页），控制任务每周期读取
  - `State`：控制任务每周期写入，里程计发布和 OLED 通过 `getState()` 读取
  - 步进电机：ROS 回调写待发指令，步进任务写状态快照
- `loop()` 被阻塞（重连、时间同步、发布）**不再影响**测速和 PID
- 看门狗：控制任务和 `loop()` 卡死超过 5s 自动复位（复位后 PWM 归零，电机停转）

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
| 板载 RGB 灯（WS2812） | 与 ENC_B1 共用 | 48 |
| micro-ROS | USB 串口 | 43 / 44（开发板内部） |
| 未使用 | NET1 / NET2 / NET12 / NET13 / NET14 | 1 / 2 / 12 / 13 / 14 |

---

## 四、代码结构

```
src/
├── main.cpp                    # 硬件初始化 + micro-ROS 连接状态机
├── config/AppConfig.h          # 所有引脚、话题名、参数
├── utils/RosAgentState.h       # ROS 连接状态枚举
├── web/index.html              # 手机控制网页（编译时嵌入固件）
└── apps/
    ├── CarControllerApp        # 底盘：cmd_vel、PID、里程计
    ├── StepperMotorApp         # 步进电机
    ├── ImuApp                  # IMU
    ├── OledApp                 # OLED 显示
    ├── WifiApp                 # WiFi 热点 + 网页 + WebSocket
    ├── RgbLedApp               # RGB 灯（队列 + 工作任务）
    └── MicroRosArmControllerApp # 机械臂（保留，不启用）
lib/
├── Drv8701Control/             # 新增：DRV8701E 驱动
├── OledDisplay/                # 新增：SSD1306 封装
├── RgbLed/                     # 新增：WS2812 RGB 灯封装
├── PcntQuadEncoder/            # 新增：无中断、无竞态的 PCNT 正交编码器
├── BujinControl/               # Emm_V5 步进电机协议
├── Kinematics/ PidController/ IMU/ ...
```

---

## 五、使用与调试

1. 编译上传：`pio run -t upload`
2. 电脑端启动 Agent（串口号按实际情况）：
   ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600
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
2. **GPIO48 板载 RGB 灯与右前编码器 ENC_B1 共用**：核心板的 RGB 灯在 GPIO48（官方 DevKitC v1.1 PDF 中为 GPIO38，以实际板子为准）。启用 RGB 灯时右前编码器被停用，右前轮速用右后轮代替，里程计精度下降；两者同时接入时信号会互相干扰。建议下一版 PCB 把 ENC_B1 改到空闲引脚（GPIO1/2/12/13/14）。如果把 `kRgbLedPin` 设为 38（与 DIRB 冲突），编译会直接报错。
3. **未经实物验证**：代码仅通过编译。电机方向、编码器方向、新任务架构、WiFi 灯均需上板测试；测速窗口改为固定 10ms 后 PID 参数可能需要重新整定。

### 软件相关
4. **ROS 速度指令默认不超时**（`kRosCmdTimeoutMs = 0`，按前进一直走、按停才停）：只有与 Agent 断开时才停车。如果 Agent 仍在线但发布 `/cmd_vel` 的节点崩溃，小车会一直保持最后的速度。需要时改为 500。
5. **步进电机位置是开环估计**：`/stepper_motor_status` 发布的是累加的指令圈数，不是电机真实位置；电机堵转或丢步时数值不准。并且代码假设电机地址 = 下标 + 1。
6. **I2C 总线跨任务共用**：OLED（核 0）和 IMU（loop，核 1）依赖 Arduino `Wire` 内部的锁，没有额外的应用层互斥；OLED 整屏刷新约 25ms。
7. **时间同步失败时时间戳错误**：如果 `rmw_uros_sync_session` 失败，`/wheel_odom` 和 `/imu` 的时间戳会从 0 开始。
8. **里程计仅靠编码器**：没有与 IMU 融合，打滑时航向角会漂移（可在电脑端用 robot_localization 融合）。
9. **micro-ROS 必须接“USB 转串口”那个口，波特率 921600**：代码使用 `Serial`（UART0），接原生 USB 口或 Agent 仍用 115200 都无法连接（OLED 一直显示 `ROS: WAIT AGENT`）。
10. **WiFi 没有身份验证**：任何连上热点的人都能控制，热点密码是唯一的保护，请修改默认密码 `starbot123`。WebSocket 指令也没有“控制权”机制，多台手机可同时操作。
11. **编码器计数依赖 PCNT 到达 ±32767 自动归零的硬件行为**：已用模拟验证换算逻辑（20 万个周期误差为 0），需实测确认。
12. **机械臂代码引脚冲突**：`MicroRosArmControllerApp` 使用的 GPIO16/17 在新板上已是 PWMD / ENC_D2，切勿重新启用。`lib/MLTrol` 也未适配新引脚。

### 已修复（2026-10-6 任务架构优化）
- ~~轮速测量放在 `loop()` 中，loop 阻塞时 PID 使用过期速度~~ → 合并进 10ms 控制任务
- ~~步进电机串口忙等最长 200ms，阻塞 micro-ROS 并可能导致串口接收溢出~~ → 独立任务 + 等待时让出 CPU
- ~~编码器读数竞态，偶发 ±100 脉冲跳变~~ → `PcntQuadEncoder`，无中断
- ~~运动学数据跨任务无锁共享~~ → `Command` / `State` 加锁快照
- ~~串口 115200 带宽不足（里程计 + IMU 约 21KB/s > 11.5KB/s）~~ → 921600
- ~~没有看门狗~~ → 控制任务与 loop 加入任务看门狗

---

## 七、后续计划

1. **WiFi 手机控制**：✅ 第一阶段已完成（RGB 灯）。下一步：网页摇杆控制小车方向、显示 ROS 状态和实时速度。速度指令统一调用 `CarControllerApp::setTargetVelocity()`，需要增加手机与 ROS 指令的优先级仲裁、控制权和超时停车。
2. 第二个功能：待定。
