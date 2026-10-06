#ifndef IMU_APP_H
#define IMU_APP_H

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/imu.h>

#include "config/AppConfig.h"
#include "IMU.h"

class ImuApp {
public:
    static constexpr size_t kExecutorHandles = 1;

    ImuApp();

    // 探测并初始化 IMU（上电调用一次）；I2C 总线需在外部先初始化（与 OLED 共用）
    bool initHardware();
    // IMU 未检测到时不创建任何实体，直接返回 true
    bool createRosEntities(rclc_support_t &support, rcl_node_t &node, rclc_executor_t &executor);
    void destroyRosEntities(rcl_node_t &node);
    void update();
    bool isConnected() const { return imu_.isConnected(); }

private:
    static ImuApp *instance_;

    IMU imu_;

    rcl_publisher_t pub_imu_{};
    rcl_timer_t     timer_imu_{};

    sensor_msgs__msg__Imu msg_imu_{};

    void publishImu();

    static void imuTimerCallback(rcl_timer_t *timer, int64_t last_call_time);
};

#endif
