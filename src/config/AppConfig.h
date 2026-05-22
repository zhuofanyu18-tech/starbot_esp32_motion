#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <Arduino.h>

namespace app_config {

static constexpr uint32_t kDebugSerialBaudrate = 115200;
static constexpr uint32_t kServoBaudrate = 1000000;

// 舵机控制引脚（机械臂已废弃，保留定义供 MicroRosArmControllerApp 编译使用）
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
static constexpr gpio_num_t lf_motor[3] = {GPIO_NUM_42, GPIO_NUM_40, GPIO_NUM_41};  // PWM, DIR1, DIR2
static constexpr gpio_num_t rf_motor[3] = {GPIO_NUM_37, GPIO_NUM_38, GPIO_NUM_39};  // PWM, DIR1, DIR2
static constexpr gpio_num_t lr_motor[3] = {GPIO_NUM_4, GPIO_NUM_6, GPIO_NUM_5};     // PWM, DIR1, DIR2
static constexpr gpio_num_t rr_motor[3] = {GPIO_NUM_16, GPIO_NUM_7, GPIO_NUM_15};   // PWM, DIR1, DIR2
static constexpr gpio_num_t lf_encoder[2] = {GPIO_NUM_21, GPIO_NUM_20};  // A, B
static constexpr gpio_num_t rf_encoder[2] = {GPIO_NUM_35, GPIO_NUM_36};  // A, B
static constexpr gpio_num_t lr_encoder[2] = {GPIO_NUM_10, GPIO_NUM_11};  // A, B
static constexpr gpio_num_t rr_encoder[2] = {GPIO_NUM_13, GPIO_NUM_12};  // A, B


// IMU (Wit-Motion) 话题与参数
static constexpr char kImuTopic[] = "/imu";
static constexpr uint32_t kImuPublishPeriodMs = 50;  // 20Hz

// IMU I2C 引脚
static constexpr uint8_t kImuSdaPin = 8;
static constexpr uint8_t kImuSclPin = 9;

// ---- 步进电机 ----
static constexpr uint8_t kStepperTxPin = 17;  // 与 BujinControl.h 中 UART_TX_PIN 一致
static constexpr uint8_t kStepperRxPin = 18;  // 与 BujinControl.h 中 UART_RX_PIN 一致
static constexpr char kStepperTargetTopic[] = "/stepper_motor_target";
static constexpr char kStepperStatusTopic[] = "/stepper_motor_status";
static constexpr uint32_t kStepperStatusPublishPeriodMs = 1000;   // 1Hz
static constexpr uint32_t kStepperPulsesPerRevolution = 3200;     // 根据驱动器微步设置调整
static constexpr uint16_t kStepperDefaultVelocityRpm = 60;
static constexpr uint8_t  kStepperDefaultAcceleration = 30;

// 手动碰撞回零参数
static constexpr uint16_t kHomingVelocityRpm = 15;          // 回零速度 RPM（很慢，防撞坏结构）
static constexpr uint32_t kHomingTimeoutMs = 30000;         // 回零超时 30s
static constexpr uint32_t kHomingPollIntervalMs = 200;      // 回零状态轮询间隔
static constexpr float    kHomingStallSpeedThreshold = 5.0f; // 低于此速度(RPM)判定为堵转

}  // namespace app_config

#endif
