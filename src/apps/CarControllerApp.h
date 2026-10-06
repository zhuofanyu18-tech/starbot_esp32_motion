#ifndef CAR_CONTROLLER_APP_H
#define CAR_CONTROLLER_APP_H

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <PcntQuadEncoder.h>
#include <Drv8701Control.h>
#include <PidController.h>
#include <Kinematics.h>

#include "config/AppConfig.h"

/*
    底盘控制 App

    控制任务 car_ctrl（核 1，优先级最高，严格 10ms 一次）独占所有控制数据：
        读编码器 → 计算轮速 → 积分里程计 → 取最新速度指令（检查超时）→ 逆运动学 → PID → PWM → 写状态快照

    其他任务与控制任务之间只通过两个加锁的小结构体交换数据，互不阻塞：
        Command：setTargetVelocity() 写入（ROS 回调、WiFi 网页），控制任务读取
        State  ：控制任务写入，getState() 读取（里程计发布、OLED）
*/
class CarControllerApp {
public:
    // 本 App 向 executor 注册的 handle 数量：1 订阅速度 + 1 定时器发布里程计
    static constexpr size_t kExecutorHandles = 2;
    static constexpr uint8_t kWheelCount = 4;  // 0=左前 1=右前 2=左后 3=右后

    // 速度指令来源，不同来源使用不同的超时时间（AppConfig 中 kRosCmdTimeoutMs / kWifiCmdTimeoutMs）
    enum class CmdSource : uint8_t { kNone, kRos, kWifi };

    // 控制任务每个周期更新一次的状态快照
    struct State {
        float     linear;                    // 整车线速度 m/s
        float     angular;                   // 整车角速度 rad/s
        float     x, y, yaw;                 // 里程计 m, m, rad
        float     wheel_speed[kWheelCount];  // 四轮速度 m/s
        CmdSource source;                    // 当前生效的指令来源
        bool      cmd_timed_out;             // 指令超时，已自动停车
    };

    CarControllerApp(); // 构造函数，初始化成员变量

    // 初始化电机、编码器、PID 并启动控制任务（上电调用一次，与 ROS 是否连接无关）
    void initHardware();

    // 创建 ROS 实体并注册到外部共享 executor（每次连上 Agent 时调用）
    bool createRosEntities(rclc_support_t &support, rcl_node_t &node, rclc_executor_t &executor);

    // 销毁 ROS 实体（与 Agent 断开时调用）
    void destroyRosEntities(rcl_node_t &node);

    // 速度指令统一入口（线程安全，非阻塞）：ROS /cmd_vel 以及 WiFi 手机控制都调用这里
    void setTargetVelocity(float linear_mps, float angular_radps, CmdSource source);
    void stop();

    // 读取最新状态快照（线程安全，非阻塞）
    State getState() const;

private:
    //声明类内静态指针成员变量，用于在静态回调函数中访问类的非静态成员(CarControllerApp car_app_;)
    static CarControllerApp *instance_;

    struct Command {
        float     linear_mps;
        float     angular_radps;
        CmdSource source;
        uint32_t  stamp_ms;
    };

    // ---- 以下硬件与算法对象只在控制任务中访问 ----
    Drv8701Control    motor_;
    PcntQuadEncoder   encoders_[kWheelCount];
    PidController     pid_[kWheelCount];
    Kinematics        kinematics_;

    // ---- 跨任务共享数据，用 lock_ 保护 ----
    mutable portMUX_TYPE lock_;
    Command cmd_{};
    State   state_{};

    // {}的作用是值初始化，确保结构体中的所有成员都被初始化为零或默认值，避免未定义行为
    rcl_subscription_t sub_cmd_vel_{};  // 订阅速度话题
    rcl_publisher_t    pub_odom_{};     // 发布里程计话题
    rcl_timer_t        timer_odom_{};   // 定时器，用于定期发布里程计

    geometry_msgs__msg__Twist    msg_cmd_vel_{};    // 接收的速度消息
    nav_msgs__msg__Odometry      msg_odom_{};       // 发布的里程计消息

    TaskHandle_t control_task_handle_ = nullptr;

    void controlStep();     // 一个控制周期
    void publishOdom();     // 发布里程计消息

    static void twistCallback(const void *msgin);   // 速度的回调函数
    static void odomTimerCallback(rcl_timer_t *timer, int64_t last_call_time);  // 定时器的回调函数
    static void controlTaskFn(void *args);          // 控制任务函数
};

#endif
