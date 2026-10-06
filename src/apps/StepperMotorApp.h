#ifndef STEPPER_MOTOR_APP_H
#define STEPPER_MOTOR_APP_H

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config/AppConfig.h"

class StepperMotorApp {
public:
    static constexpr size_t kExecutorHandles = 2;  // 1 sub + 1 timer

    // 电机数量和对应的 ID（地址）
    static constexpr size_t kMotorCount = 2;
    static constexpr uint8_t kMotorIds[kMotorCount] = {1, 2};

    StepperMotorApp();

    // 初始化串口、使能电机、按配置回零，并启动步进电机任务（上电调用一次）
    // 串口收发（每条指令等待应答 10~200ms）全部在独立任务中进行，不会阻塞 micro-ROS 主循环
    void initHardware();
    bool createRosEntities(rclc_support_t &support, rcl_node_t &node, rclc_executor_t &executor);
    void destroyRosEntities(rcl_node_t &node);

private:
    static StepperMotorApp *instance_;

    rcl_subscription_t sub_target_{};
    rcl_publisher_t    pub_status_{};
    rcl_timer_t        timer_status_{};

    std_msgs__msg__Float32MultiArray msg_target_{};
    std_msgs__msg__Float32MultiArray msg_status_{};

    struct MotorCommand {
        uint8_t  addr;
        uint8_t  dir;       // 0=CW, 1=CCW
        uint16_t vel;       // RPM
        uint8_t  acc;       // 0-255
        uint32_t clk;       // 脉冲数
        bool     pending;
    };

    // ---- 跨任务共享数据，用 lock_ 保护：ROS 回调写指令，步进任务写状态 ----
    portMUX_TYPE lock_;
    MotorCommand pending_cmds_[kMotorCount] = {};
    float        status_snapshot_[kMotorCount] = {};
    TaskHandle_t task_handle_ = nullptr;

    // ---- 以下只在步进电机任务中访问 ----
    float        current_positions_[kMotorCount] = {};
    float        target_turns_[kMotorCount] = {};
    bool         motors_enabled_ = false;
    bool         homed_ = true;
    bool         homing_ = false;
    size_t       homing_motor_index_ = 0;
    uint32_t     homing_check_ms_ = 0;
    uint32_t     homing_start_ms_[kMotorCount] = {0};
    uint8_t      homing_retry_count_ = 0;
    static constexpr uint8_t kMaxHomingRetries = 3;

    void allocateMessageMemory();
    void initMotors();
    void startHomingBuiltin(size_t motor_index);
    int8_t pollHomingBuiltinStatus();
    void advanceToNextMotor();
    void update();                  // 步进任务每个周期调用一次
    void processPendingCommands();
    void updateStatusSnapshot();
    void publishStatus();

    uint32_t turnsToPulses(float turns) const;

    static void targetCallback(const void *msgin);
    static void statusTimerCallback(rcl_timer_t *timer, int64_t last_call_time);
    static void stepperTaskFn(void *args);
};

#endif
