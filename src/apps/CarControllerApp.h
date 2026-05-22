#ifndef CAR_CONTROLLER_APP_H
#define CAR_CONTROLLER_APP_H

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <Esp32PcntEncoder.h>
#include <PwmControl.h>
#include <PidController.h>
#include <Kinematics.h>

#include "config/AppConfig.h"

class CarControllerApp {
public:
    // 本 App 向 executor 注册的 handle 数量：1 订阅速度 + 1 定时器发布里程计
    static constexpr size_t kExecutorHandles = 2;

    CarControllerApp(); // 构造函数，初始化成员变量

    // 初始化硬件和 ROS 实体，将自身注册到外部共享 executor
    void begin(rclc_support_t &support, rcl_node_t &node, rclc_executor_t &executor);

    // 非阻塞更新：读编码器 → PID → 写电机
    void update();

private:
    //声明类内静态指针成员变量，用于在静态回调函数中访问类的非静态成员(CarControllerApp car_app_;)
    static CarControllerApp *instance_;

    PWMControl        motor_;
    Esp32PcntEncoder  encoders_[4];
    PidController     pid_[4];
    Kinematics        kinematics_;

    // {}的作用是值初始化，确保结构体中的所有成员都被初始化为零或默认值，避免未定义行为
    rcl_subscription_t sub_cmd_vel_{};  // 订阅速度话题
    rcl_publisher_t    pub_odom_{};     // 发布里程计话题
    rcl_timer_t        timer_odom_{};   // 定时器，用于定期发布里程计

    geometry_msgs__msg__Twist    msg_cmd_vel_{};    // 接收的速度消息
    nav_msgs__msg__Odometry      msg_odom_{};       // 发布的里程计消息

    SemaphoreHandle_t pid_mutex_ = nullptr;
    TaskHandle_t      pid_task_handle_ = nullptr;

    void initHardware();    // 初始化硬件接口和 PID 参数
    void publishOdom();     // 发布里程计消息

    static void twistCallback(const void *msgin);   // 速度的回调函数
    static void odomTimerCallback(rcl_timer_t *timer, int64_t last_call_time);  // 定时器的回调函数
    static void pidControlTaskFn(void *args);       // PID 控制任务函数
};

#endif
