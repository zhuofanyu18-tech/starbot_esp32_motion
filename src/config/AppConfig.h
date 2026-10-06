#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <Arduino.h>

namespace app_config {

// micro-ROS 串口（USB 转串口）。115200 只有 11.5KB/s，而里程计 20Hz(~14.6KB/s) + IMU 20Hz(~6.4KB/s) 已超出带宽，
// 因此提高到 921600（约 92KB/s）。电脑端 Agent 需同步修改：micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600
static constexpr uint32_t kMicroRosBaudrate = 921600;
static constexpr size_t   kMicroRosRxBufferSize = 2048;  // 默认仅 256 字节，主循环稍有延迟就会丢数据
static constexpr size_t   kMicroRosTxBufferSize = 2048;  // 发送缓冲，避免发布大消息时阻塞主循环

// ---- FreeRTOS 任务分配 ----
// 核 0：WiFi 协议栈 / 网页服务器 / RGB 灯 / OLED（非实时）
// 核 1：底盘控制（最高）> micro-ROS 主循环 > 步进电机（实时相关）
static constexpr BaseType_t  kRealtimeCore = 1;
static constexpr BaseType_t  kBackgroundCore = 0;
static constexpr UBaseType_t kControlTaskPriority = 10;  // 底盘控制：测速 + PID + 里程计
static constexpr uint32_t    kControlTaskStack = 4096;
static constexpr uint32_t    kControlPeriodMs = 10;      // 100Hz
static constexpr UBaseType_t kRosTaskPriority = 5;       // Arduino loopTask（micro-ROS）
static constexpr UBaseType_t kStepperTaskPriority = 3;
static constexpr uint32_t    kStepperTaskStack = 4096;
static constexpr uint32_t    kStepperPeriodMs = 10;
static constexpr UBaseType_t kHttpdTaskPriority = 5;
static constexpr UBaseType_t kRgbLedTaskPriority = 2;
static constexpr UBaseType_t kOledTaskPriority = 1;

// ---- 速度指令超时（毫秒），0 表示不超时：保持最后一条指令，直到收到新指令（发 0 速度才停）----
// 无论是否超时，与 Agent 断开连接时底盘都会立即停车
static constexpr uint32_t kRosCmdTimeoutMs = 0;     // 键盘遥控按一下前进就一直前进，按停才停
static constexpr uint32_t kWifiCmdTimeoutMs = 500;  // 手机网页持续发送心跳，手机锁屏/断网 0.5s 后自动停车
static constexpr uint32_t kServoBaudrate = 1000000;

// 舵机控制引脚（机械臂已改由电脑控制，ESP32 不再使用，保留定义供 MicroRosArmControllerApp 编译使用）
// 注意：新电路板上 GPIO16=PWMD、GPIO17=ENC_D2，切勿再启用机械臂 App
static constexpr uint8_t kServoTxPin = 17;
static constexpr uint8_t kServoRxPin = 16;

// ROS2 舵机的节点名和话题名
static constexpr char kNodeName[] = "starbot_arm_controller";
static constexpr char kArmTrajectoryTopic[] = "/arm_controller/joint_trajectory";
static constexpr char kGripperTrajectoryTopic[] = "/gripper_controller/joint_trajectory";
static constexpr char kJointStatesTopic[] = "/joint_states";

static constexpr uint32_t kJointStatePublishPeriodMs = 50;
static constexpr uint32_t kExecutorSpinPeriodMs = 20;
static constexpr size_t kTrajectoryPointCapacity = 16;
static constexpr size_t kRosStringCapacity = 20;
// Startup policy for passive joint5: false means do not send homing command on boot.
static constexpr bool kEnableJoint5StartupMotion = false;
// Log once when an incoming arm trajectory contains passive joint5 and gets ignored.
static constexpr bool kWarnWhenIgnoringJoint5InTrajectory = true;


// 底盘话题
static constexpr char kCmdVelTopic[] = "/cmd_vel";
static constexpr char kOdomTopic[] = "/wheel_odom";
static constexpr uint32_t kOdomPublishPeriodMs = 50;    // 发布里程计的周期 20HZ
// 底盘参数
static constexpr float kWheelDiameterMm = 125.0f;       // 轮子直径，单位毫米
static constexpr float kWheelBaseMm = 370.0f;          // 轮距，单位毫米
static constexpr float kEncoderPulsesPerRevolution = 14000.0f; // 编码器每转一圈的脉冲数
static constexpr float Kp = 1.0f;                     // PID 控制器的比例增益
static constexpr float Ki = 0.3f;                     // PID 控制器的积分增益
static constexpr float Kd = 0.5f;                     // PID 控制器的微分增益
// 电机驱动 DRV8701E（PH/EN 模式：一个 PWM + 一个方向）
// 轮子对应关系：A=左前(CN25)  B=右前(CN25)  C=左后(CN26)  D=右后(CN26)
static constexpr gpio_num_t lf_motor[2] = {GPIO_NUM_42, GPIO_NUM_41};  // PWMA, DIRA
static constexpr gpio_num_t rf_motor[2] = {GPIO_NUM_21, GPIO_NUM_38};  // PWMB, DIRB（GPIO38 同时接开发板 RGB 灯）
static constexpr gpio_num_t lr_motor[2] = {GPIO_NUM_4,  GPIO_NUM_5};   // PWMC, DIRC
static constexpr gpio_num_t rr_motor[2] = {GPIO_NUM_16, GPIO_NUM_15};  // PWMD, DIRD
// 编码器：H1=左前  H2=右前  H3=左后  H4=右后
static constexpr gpio_num_t lf_encoder[2] = {GPIO_NUM_40, GPIO_NUM_39};  // ENC_A1, ENC_A2
static constexpr gpio_num_t rf_encoder[2] = {GPIO_NUM_48, GPIO_NUM_47};  // ENC_B1, ENC_B2
static constexpr gpio_num_t lr_encoder[2] = {GPIO_NUM_6,  GPIO_NUM_7};   // ENC_C1, ENC_C2
static constexpr gpio_num_t rr_encoder[2] = {GPIO_NUM_18, GPIO_NUM_17};  // ENC_D1, ENC_D2
// 装车后若某个轮子转向反了 / 编码器计数方向反了，只需改这里，顺序：左前、右前、左后、右后
static constexpr bool kMotorReversed[4]   = {false, false, false, false};
static constexpr bool kEncoderReversed[4] = {false, false, false, false};
static constexpr uint32_t kMotorPwmFrequencyHz = 20000;  // DRV8701E PWM 频率
static constexpr uint16_t kEncoderGlitchFilterCycles = 1000;  // 编码器毛刺滤波（APB 时钟周期，1000≈12.5us，最大 1023）


// IMU (Wit-Motion) 话题与参数
static constexpr char kImuTopic[] = "/imu";
static constexpr uint32_t kImuPublishPeriodMs = 50;  // 20Hz

// I2C 总线（IMU 与 OLED 共用）
static constexpr uint8_t  kI2cSdaPin = 8;
static constexpr uint8_t  kI2cSclPin = 9;
static constexpr uint32_t kI2cClockHz = 400000;
static constexpr uint8_t  kImuSdaPin = kI2cSdaPin;
static constexpr uint8_t  kImuSclPin = kI2cSclPin;

// ---- OLED SSD1306 128x64 (I2C) ----
static constexpr uint8_t  kOledI2cAddress = 0x3C;
static constexpr uint8_t  kOledWidth = 128;
static constexpr uint8_t  kOledHeight = 64;
static constexpr uint32_t kOledRefreshPeriodMs = 200;  // 5Hz

// ---- WiFi 手机控制（SoftAP：手机直连 ESP32 热点，浏览器打开 http://192.168.4.1）----
static constexpr bool     kEnableWifi = true;
static constexpr char     kWifiApSsid[] = "StarBot-ESP32";
static constexpr char     kWifiApPassword[] = "starbot123";  // 至少 8 位；留空则为开放热点
static constexpr uint8_t  kWifiApChannel = 1;
static constexpr uint8_t  kWifiApMaxConnections = 4;

// ---- 板载 RGB 灯（WS2812）----
// 注意：核心板的 RGB 灯在 GPIO48，与右前编码器 ENC_B1（H2）是同一根线！
// kEnableRgbLed=true 时不初始化右前编码器，右前轮速改用右后编码器代替，测试时请拔掉 H2。
static constexpr bool     kEnableRgbLed = true;
static constexpr uint8_t  kRgbLedPin = 48;
static constexpr uint8_t  kRgbLedMaxBrightness = 128;  // 亮度 100% 对应的通道最大值（0~255），限制电流

// ---- micro-ROS Agent 连接检测 ----
static constexpr uint32_t kAgentPingPeriodMs = 500;    // 未连接时探测 Agent 的周期
static constexpr uint32_t kAgentCheckPeriodMs = 1000;  // 已连接时检测是否断线的周期
static constexpr uint32_t kAgentPingTimeoutMs = 100;
static constexpr uint8_t  kAgentPingAttempts = 3;      // 已连接时连续 ping 失败的容忍次数

// ---- 步进电机 ----
// CN1 / CN27 两个步进驱动并联在同一条串口总线上，靠地址 1、2 区分
static constexpr uint8_t kStepperRxPin = 10;  // RX1：ESP32 接收，接驱动器 TX
static constexpr uint8_t kStepperTxPin = 11;  // TX1：ESP32 发送，接驱动器 RX
static constexpr char kStepperTargetTopic[] = "/stepper_motor_target";
static constexpr char kStepperStatusTopic[] = "/stepper_motor_status";
static constexpr uint32_t kStepperStatusPublishPeriodMs = 1000;   // 1Hz
static constexpr uint32_t kStepperPulsesPerRevolution = 3200;     // 根据驱动器微步设置调整
static constexpr uint16_t kStepperDefaultVelocityRpm = 60;
static constexpr uint8_t  kStepperDefaultAcceleration = 30;

// ---- 步进电机 回零控制 ----
    static constexpr bool    kEnableAutoHoming = false;          // 上电自动回零开关 (false=跳过回零)
    static constexpr uint16_t kHomingCollisionCurrentMa = 200;   // 碰撞检测电流阈值 mA
    static constexpr uint16_t kHomingCollisionVelRpm = 15;       // 碰撞检测阶段速度 RPM
    static constexpr uint16_t kHomingCollisionTimeMs = 100;      // 碰撞检测持续时间 ms
    static constexpr uint16_t kHomingVelocityRpm = 15;           // 回零接近速度 RPM
    static constexpr uint32_t kHomingTimeoutMs = 30000;          // 回零超时 30s
    static constexpr uint32_t kHomingPollIntervalMs = 200;       // S_ORG 状态轮询间隔

}  // namespace app_config

#endif
