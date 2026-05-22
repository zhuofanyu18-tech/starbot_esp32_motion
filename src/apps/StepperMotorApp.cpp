#include "apps/StepperMotorApp.h"

#include <cmath>
#include <cstdlib>
#include <rmw_microros/rmw_microros.h>

#include "BujinControl.h"

namespace {

void stopOnError(rcl_ret_t ret) {
    if (ret != RCL_RET_OK) {
        delay(2000);
        esp_restart();
    }
}

}  // namespace

StepperMotorApp *StepperMotorApp::instance_ = nullptr;

constexpr uint8_t StepperMotorApp::kMotorIds[StepperMotorApp::kMotorCount];

StepperMotorApp::StepperMotorApp() {
    instance_ = this;
}

void StepperMotorApp::allocateMessageMemory() {
    // 订阅消息：分配 3 个 float 的容量，用于接收 [turns1, turns2, turns3]
    msg_target_.layout.dim.capacity = 0;
    msg_target_.layout.dim.size = 0;
    msg_target_.layout.dim.data = nullptr;
    msg_target_.layout.data_offset = 0;

    msg_target_.data.capacity = kMotorCount;
    msg_target_.data.size = 0;
    msg_target_.data.data =
        static_cast<float *>(calloc(kMotorCount, sizeof(float)));

    // 发布消息：分配 3 个 float，用于发布当前位置
    msg_status_.layout.dim.capacity = 0;
    msg_status_.layout.dim.size = 0;
    msg_status_.layout.dim.data = nullptr;
    msg_status_.layout.data_offset = 0;

    msg_status_.data.capacity = kMotorCount;
    msg_status_.data.size = kMotorCount;
    msg_status_.data.data =
        static_cast<float *>(calloc(kMotorCount, sizeof(float)));
}

void StepperMotorApp::initMotors() {
    Emm_V5_INIT();
    Emm_V5_En_Control(0, true, false);
    motors_enabled_ = true;

    // 读走使能指令的响应，避免残留干扰回零指令
    {
        uint8_t rxCmd[128] = {0};
        uint8_t rxCount = 0;
        Emm_V5_Receive_Data(rxCmd, &rxCount);
    }

    // 上电后自动触发碰撞回零
    startHoming();
}

void StepperMotorApp::begin(
    rclc_support_t &support, rcl_node_t &node, rclc_executor_t &executor) {

    allocateMessageMemory();
    initMotors();

    stopOnError(rclc_subscription_init_default(
        &sub_target_, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        app_config::kStepperTargetTopic));

    stopOnError(rclc_publisher_init_default(
        &pub_status_, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        app_config::kStepperStatusTopic));

    stopOnError(rclc_timer_init_default(
        &timer_status_, &support,
        RCL_MS_TO_NS(app_config::kStepperStatusPublishPeriodMs),
        statusTimerCallback));

    stopOnError(rclc_executor_add_subscription(
        &executor, &sub_target_, &msg_target_, targetCallback, ON_NEW_DATA));
    stopOnError(rclc_executor_add_timer(&executor, &timer_status_));
}

void StepperMotorApp::startHoming() {
    if (xSemaphoreTake(motor_mutex, pdMS_TO_TICKS(200)) != pdTRUE) return;

    uint8_t addr = kMotorIds[homing_motor_index_];

    // 清空串口缓冲区
    while (Serial2.available()) Serial2.read();

    // 停止残留运动 + 清除堵转保护
    Emm_V5_Stop_Now(addr, false);
    uint8_t rxCmd[128] = {0};
    uint8_t rxCount = 0;
    Emm_V5_Receive_Data(rxCmd, &rxCount);
    vTaskDelay(pdMS_TO_TICKS(10));

    Emm_V5_Reset_Clog_Pro(addr);
    Emm_V5_Receive_Data(rxCmd, &rxCount);

    // 发送慢速反转指令 (dir=1=CCW)，持续反转直到堵转
    Emm_V5_Vel_Control(addr, 1, app_config::kHomingVelocityRpm, 5, false);
    Emm_V5_Receive_Data(rxCmd, &rxCount);

    xSemaphoreGive(motor_mutex);

    homing_ = true;
    homing_check_ms_ = millis();
    current_positions_[homing_motor_index_] = 0;
}

bool StepperMotorApp::pollHomingDone() {
    if (xSemaphoreTake(motor_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return false;

    uint8_t addr = kMotorIds[homing_motor_index_];
    float vel = Emm_V5_MotorVel_Get(addr);
    bool stalled = (vel >= 0.0f && vel < app_config::kHomingStallSpeedThreshold);

    if (stalled) {
        uint8_t rxCmd[128] = {0};
        uint8_t rxCount = 0;

        Emm_V5_Stop_Now(addr, false);
        Emm_V5_Receive_Data(rxCmd, &rxCount);

        Emm_V5_Reset_Clog_Pro(addr);
        Emm_V5_Receive_Data(rxCmd, &rxCount);

        Emm_V5_Reset_CurPos_To_Zero(addr);
        Emm_V5_Receive_Data(rxCmd, &rxCount);
    }

    xSemaphoreGive(motor_mutex);
    return stalled;
}

void StepperMotorApp::update() {
    // 回零期间逐一轮询，不处理运动指令
    if (homing_ && !homed_) {
        if (millis() - homing_check_ms_ >= app_config::kHomingPollIntervalMs) {
            homing_check_ms_ = millis();
            if (pollHomingDone()) {
                homing_motor_index_++;
                if (homing_motor_index_ >= kMotorCount) {
                    homed_ = true;
                    homing_ = false;
                } else {
                    startHoming();  // 开始回零下一个电机
                }
            }
        }
        return;
    }
    processPendingCommands();
}

uint32_t StepperMotorApp::turnsToPulses(float turns) const {
    return static_cast<uint32_t>(
        fabsf(turns) * static_cast<float>(app_config::kStepperPulsesPerRevolution));
}

void StepperMotorApp::processPendingCommands() {
    // 收集所有待发指令（避免持锁时间过长）
    MotorCommand batch[kMotorCount];
    size_t batch_count = 0;
    for (size_t i = 0; i < kMotorCount; ++i) {
        if (pending_cmds_[i].pending) {
            batch[batch_count++] = pending_cmds_[i];
            pending_cmds_[i].pending = false;
        }
    }

    if (batch_count == 0) return;

    // 持锁一次，批量发送：发→读响应→短延迟→下一个
    if (xSemaphoreTake(motor_mutex, pdMS_TO_TICKS(300)) != pdTRUE) return;

    if (!motors_enabled_) {
        Emm_V5_En_Control(0, true, false);
        motors_enabled_ = true;
    }

    for (size_t i = 0; i < batch_count; ++i) {
        MotorCommand &cmd = batch[i];

        Emm_V5_Pos_Control(cmd.addr, cmd.dir, cmd.vel, cmd.acc, cmd.clk, false, false);

        // 读走电机回复的响应数据，避免残留干扰下一条指令
        uint8_t rxCmd[128] = {0};
        uint8_t rxCount = 0;
        Emm_V5_Receive_Data(rxCmd, &rxCount);

        // 更新位置估计
        float turns = static_cast<float>(cmd.clk) / static_cast<float>(app_config::kStepperPulsesPerRevolution);
        if (cmd.dir != 0) turns = -turns;
        target_turns_[cmd.addr - 1] = turns;
        current_positions_[cmd.addr - 1] += turns;

        // 电机间留 5ms 间隔，确保驱动芯片处理完毕
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    xSemaphoreGive(motor_mutex);
}

void StepperMotorApp::publishStatus() {
    if (!homed_) {
        // 未回零完成时发布 -1.0 表示未就绪
        msg_status_.data.data[0] = -1.0f;
    } else {
        for (size_t i = 0; i < kMotorCount; ++i) {
            msg_status_.data.data[i] = current_positions_[i];
        }
    }
    rcl_ret_t ret = rcl_publish(&pub_status_, &msg_status_, nullptr); (void)ret;
}

void StepperMotorApp::targetCallback(const void *msgin) {
    if (!instance_ || !msgin) return;
    const auto &msg =
        *static_cast<const std_msgs__msg__Float32MultiArray *>(msgin);

    size_t count = msg.data.size < kMotorCount ? msg.data.size : kMotorCount;
    for (size_t i = 0; i < count; ++i) {
        float turns = msg.data.data[i];
        MotorCommand &cmd = instance_->pending_cmds_[i];

        cmd.addr    = kMotorIds[i];
        cmd.dir     = (turns >= 0.0f) ? 0 : 1;  // 正值 CW, 负值 CCW
        cmd.vel     = app_config::kStepperDefaultVelocityRpm;
        cmd.acc     = app_config::kStepperDefaultAcceleration;
        cmd.clk     = instance_->turnsToPulses(turns);
        cmd.pending = (cmd.clk > 0);
    }
}

void StepperMotorApp::statusTimerCallback(rcl_timer_t *timer, int64_t) {
    if (!instance_ || !timer) return;
    instance_->publishStatus();
}
