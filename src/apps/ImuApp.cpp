#include "apps/ImuApp.h"

#include <micro_ros_utilities/string_utilities.h>
#include <rmw_microros/rmw_microros.h>

namespace {

void stopOnError(rcl_ret_t ret) {
    if (ret != RCL_RET_OK) {
        delay(2000);
        esp_restart();
    }
}

} // namespace

ImuApp *ImuApp::instance_ = nullptr;

ImuApp::ImuApp() {
    instance_ = this;
}

void ImuApp::begin(rclc_support_t &support, rcl_node_t &node, rclc_executor_t &executor) {
    if (!imu_.begin(app_config::kImuSdaPin, app_config::kImuSclPin)) {
        // IMU not detected — continue without it, won't publish
        return;
    }

    msg_imu_.header.frame_id =
        micro_ros_string_utilities_set(msg_imu_.header.frame_id, "imu_link");

    // Set covariance to unknown (-1) per ROS REP-145
    msg_imu_.orientation_covariance[0] = -1.0;
    msg_imu_.angular_velocity_covariance[0] = -1.0;
    msg_imu_.linear_acceleration_covariance[0] = -1.0;

    stopOnError(rclc_publisher_init_default(
        &pub_imu_, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        app_config::kImuTopic));

    stopOnError(rclc_timer_init_default(
        &timer_imu_, &support,
        RCL_MS_TO_NS(app_config::kImuPublishPeriodMs),
        imuTimerCallback));

    stopOnError(rclc_executor_add_timer(&executor, &timer_imu_));
}

void ImuApp::update() {
    // Timer callback drives publishing
}

void ImuApp::publishImu() {
    if (!imu_.readAll()) return;

    const auto &d = imu_.getData();

    int64_t stamp = rmw_uros_epoch_millis();
    msg_imu_.header.stamp.sec     = (int32_t)(stamp / 1000);
    msg_imu_.header.stamp.nanosec = (uint32_t)((stamp % 1000) * 1000000);

    msg_imu_.orientation.w = d.quat_w;
    msg_imu_.orientation.x = d.quat_x;
    msg_imu_.orientation.y = d.quat_y;
    msg_imu_.orientation.z = d.quat_z;

    msg_imu_.angular_velocity.x = d.gyro_x;
    msg_imu_.angular_velocity.y = d.gyro_y;
    msg_imu_.angular_velocity.z = d.gyro_z;

    msg_imu_.linear_acceleration.x = d.accel_x;
    msg_imu_.linear_acceleration.y = d.accel_y;
    msg_imu_.linear_acceleration.z = d.accel_z;

    rcl_ret_t ret = rcl_publish(&pub_imu_, &msg_imu_, nullptr); (void)ret;
}

void ImuApp::imuTimerCallback(rcl_timer_t *timer, int64_t) {
    if (!instance_ || !timer) return;
    instance_->publishImu();
}
