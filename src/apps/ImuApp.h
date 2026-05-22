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

    void begin(rclc_support_t &support, rcl_node_t &node, rclc_executor_t &executor);
    void update();

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
