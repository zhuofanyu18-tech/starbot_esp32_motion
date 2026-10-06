#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_platformio.h>
#include <rmw_microros/rmw_microros.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "config/AppConfig.h"
#include "utils/RosAgentState.h"
// #include "apps/MicroRosArmControllerApp.h"  // 机械臂已改由电脑控制，ESP32 不再使用
#include "apps/CarControllerApp.h"
#include "apps/ImuApp.h"
#include "apps/StepperMotorApp.h"
#include "apps/OledApp.h"
#include "apps/RgbLedApp.h"
#include "apps/WifiApp.h"

namespace {

rcl_allocator_t   allocator_;
rclc_support_t    support_;
rcl_node_t        node_;
rclc_executor_t   executor_;

// MicroRosArmControllerApp arm_app_;  // 机械臂已禁用
CarControllerApp         car_app_;
ImuApp                   imu_app_;
StepperMotorApp          stepper_app_;
OledApp                  oled_app_;
RgbLedApp                rgb_led_app_;
WifiApp                  wifi_app_;

RosAgentState agent_state_ = RosAgentState::kWaitingAgent;
uint32_t      last_agent_check_ms_ = 0;

void setAgentState(RosAgentState state) {
    agent_state_ = state;
    oled_app_.setRosState(state);
}

// 连上 Agent 后创建节点、executor 以及各 App 的话题
bool createRosEntities() {
    allocator_ = rcl_get_default_allocator();
    node_ = rcl_get_zero_initialized_node();
    executor_ = rclc_executor_get_zero_initialized_executor();

    if (rclc_support_init(&support_, 0, nullptr, &allocator_) != RCL_RET_OK) return false;
    if (rclc_node_init_default(&node_, app_config::kNodeName, "", &support_) != RCL_RET_OK) return false;

    constexpr size_t kHandles =
        CarControllerApp::kExecutorHandles +
        ImuApp::kExecutorHandles + StepperMotorApp::kExecutorHandles;
    if (rclc_executor_init(&executor_, &support_.context, kHandles, &allocator_) != RCL_RET_OK) return false;

    if (!car_app_.createRosEntities(support_, node_, executor_)) return false;
    if (!imu_app_.createRosEntities(support_, node_, executor_)) return false;
    if (!stepper_app_.createRosEntities(support_, node_, executor_)) return false;

    // 同步电脑时间，用于消息时间戳；失败不影响控制
    rmw_uros_sync_session(1000);
    return true;
}

// 断线后销毁所有实体，Agent 已不在，不再等待它的应答
void destroyRosEntities() {
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support_.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    car_app_.destroyRosEntities(node_);
    imu_app_.destroyRosEntities(node_);
    stepper_app_.destroyRosEntities(node_);

    rcl_ret_t ret;
    ret = rclc_executor_fini(&executor_); (void)ret;
    ret = rcl_node_fini(&node_); (void)ret;
    ret = rclc_support_fini(&support_); (void)ret;
}

/*
    micro-ROS 连接状态机（非阻塞，Agent 没启动时 ESP32 也能正常运行）
    WAITING_AGENT --ping成功--> AGENT_AVAILABLE --创建实体--> CONNECTED
         ^                                                       |
         +---------------- DISCONNECTED <------ping失败----------+
*/
void updateRosConnection() {
    const uint32_t now = millis();

    switch (agent_state_) {
    case RosAgentState::kWaitingAgent:
        if (now - last_agent_check_ms_ >= app_config::kAgentPingPeriodMs) {
            last_agent_check_ms_ = now;
            if (rmw_uros_ping_agent(app_config::kAgentPingTimeoutMs, 1) == RMW_RET_OK) {
                setAgentState(RosAgentState::kAgentAvailable);
            }
        }
        break;

    case RosAgentState::kAgentAvailable:
        if (createRosEntities()) {
            setAgentState(RosAgentState::kConnected);
        } else {
            destroyRosEntities();
            setAgentState(RosAgentState::kWaitingAgent);
        }
        last_agent_check_ms_ = now;
        break;

    case RosAgentState::kConnected:
        if (now - last_agent_check_ms_ >= app_config::kAgentCheckPeriodMs) {
            last_agent_check_ms_ = now;
            if (rmw_uros_ping_agent(app_config::kAgentPingTimeoutMs,
                                    app_config::kAgentPingAttempts) != RMW_RET_OK) {
                setAgentState(RosAgentState::kDisconnected);
                break;
            }
        }
        rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(app_config::kExecutorSpinPeriodMs));
        break;

    case RosAgentState::kDisconnected:
        // 安全保护：失去 ROS 控制后底盘立即停车
        car_app_.stop();
        destroyRosEntities();
        setAgentState(RosAgentState::kWaitingAgent);
        last_agent_check_ms_ = now;
        break;
    }
}

} // namespace

/*
    任务分配（详见 AppConfig.h）
    核 1：car_ctrl(10) 底盘控制 10ms  >  loopTask(5) micro-ROS  >  stepper(3) 步进电机串口
    核 0：WiFi 协议栈(系统)  >  httpd(5) 网页  >  rgb_led(2)  >  oled(1)
    loop() 中只保留 micro-ROS 相关逻辑，所有可能阻塞的硬件操作都在各自的任务中进行
*/
void setup() {
    // 缓冲区必须在 begin() 之前设置
    Serial.setRxBufferSize(app_config::kMicroRosRxBufferSize);
    Serial.setTxBufferSize(app_config::kMicroRosTxBufferSize);
    Serial.begin(app_config::kMicroRosBaudrate);
    set_microros_serial_transports(Serial);

    // I2C 总线只在这里初始化一次，IMU 与 OLED 共用
    Wire.begin(app_config::kI2cSdaPin, app_config::kI2cSclPin);
    Wire.setClock(app_config::kI2cClockHz);

    // 硬件初始化与 ROS 无关，上电即完成
    car_app_.initHardware();
    imu_app_.initHardware();
    stepper_app_.initHardware();
    // arm_app_.begin(support_, node_, executor_);  // 机械臂已禁用

    // WiFi 手机控制：热点 + 网页，与 micro-ROS 串口同时运行
    rgb_led_app_.begin();
    wifi_app_.begin(rgb_led_app_);

    oled_app_.begin(car_app_, wifi_app_);
    setAgentState(RosAgentState::kWaitingAgent);

    // 提高 loopTask 优先级（默认 1），保证 micro-ROS 优先于步进电机和显示
    vTaskPrioritySet(nullptr, app_config::kRosTaskPriority);
    // loopTask 加入任务看门狗：loop() 卡住超过 5s 自动复位
    enableLoopWDT();
}

void loop() {
    updateRosConnection();

    // 底盘控制、步进电机、OLED、RGB 灯都在各自的任务中运行，不依赖 loop()
    // arm_app_.update();  // 机械臂已禁用
    imu_app_.update();
    delay(1);
}
