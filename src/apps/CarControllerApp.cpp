#include "apps/CarControllerApp.h"

#include <cmath>
#include <micro_ros_utilities/string_utilities.h>
#include <rmw_microros/rmw_microros.h>

namespace {

void stopOnError(rcl_ret_t ret) {
    if (ret != RCL_RET_OK) {
        // Serial.printf("[CarController] RCL error %d, restarting...\n", (int)ret);
        delay(2000);
        esp_restart();
    }
}

} // namespace

// CarControllerApp * 定义instance_的类型，也就是一个对象指针，并且先把它初始化为 nullptr（空指针，表示目前没有指向任何对象）
CarControllerApp *CarControllerApp::instance_ = nullptr;


/*
    [ 内存中 car_app_ 对象 ]
    地址：0x1000
    +------------------+
    | kinematics_      |
    | pid_[4]          |
    | ...              |
    +------------------+

    instance_ 的值 = 0x1000   （存的就是上面那个对象的地址）
    在对象初始化时，CarControllerApp() 构造函数被调用，里面有 instance_ = this; 这行代码
    this 代表当前正在被构造的对象的地址，也就是 0x1000，所以 instance_ 就被赋值为 0x1000，指向了 car_app 对象
    这样，在静态回调函数 twistCallback 和 odomTimerCallback 中，通过 instance_ 就可以访问到 car_app_ 对象的成员变量和函数，
    实现对硬件的控制和数据的发布
*/

CarControllerApp::CarControllerApp() {
    instance_ = this;
    pid_mutex_ = xSemaphoreCreateMutex();
}

// 初始化硬件接口，包括电机、编码器、PID 控制器和运动学参数
void CarControllerApp::initHardware() {
    // 机械臂已废弃，GPIO 16 不再与舵机冲突
    motor_.attachMotor(0, app_config::lf_motor[0], app_config::lf_motor[1], app_config::lf_motor[2]);
    motor_.attachMotor(1, app_config::rf_motor[0], app_config::rf_motor[1], app_config::rf_motor[2]);
    motor_.attachMotor(2, app_config::lr_motor[0], app_config::lr_motor[1], app_config::lr_motor[2]);
    motor_.attachMotor(3, app_config::rr_motor[0], app_config::rr_motor[1], app_config::rr_motor[2]);

    encoders_[0].init(0, app_config::lf_encoder[0], app_config::lf_encoder[1]); encoders_[0].reset();
    encoders_[1].init(1, app_config::rf_encoder[0], app_config::rf_encoder[1]); encoders_[1].reset();
    encoders_[2].init(2, app_config::lr_encoder[0], app_config::lr_encoder[1]); encoders_[2].reset();
    encoders_[3].init(3, app_config::rr_encoder[0], app_config::rr_encoder[1]); encoders_[3].reset();

    for (int i = 0; i < 4; i++) {
        pid_[i].update_pid(app_config::Kp, app_config::Ki, app_config::Kd);
        pid_[i].out_limit(-1000, 1000);
        pid_[i].update_target(0);
    }

    kinematics_.set_wheel_distance(app_config::kWheelBaseMm);
    const float ppd = (app_config::kWheelDiameterMm * 3.1415926535f) / app_config::kEncoderPulsesPerRevolution;
    for (int i = 0; i < 4; i++) kinematics_.set_motor_param(i, ppd);
}

void CarControllerApp::begin(rclc_support_t &support, rcl_node_t &node, rclc_executor_t &executor) 
{
    initHardware();

    msg_odom_.header.frame_id = micro_ros_string_utilities_set(msg_odom_.header.frame_id, "odom");
    msg_odom_.child_frame_id = micro_ros_string_utilities_set(msg_odom_.child_frame_id, "base_footprint");

    stopOnError(rclc_subscription_init_best_effort(
        &sub_cmd_vel_, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        app_config::kCmdVelTopic));

    stopOnError(rclc_publisher_init_default(
        &pub_odom_, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        app_config::kOdomTopic));

    stopOnError(rclc_timer_init_default(
        &timer_odom_, &support,
        RCL_MS_TO_NS(app_config::kOdomPublishPeriodMs),
        odomTimerCallback));

    stopOnError(rclc_executor_add_subscription(
        &executor, &sub_cmd_vel_, &msg_cmd_vel_, twistCallback, ON_NEW_DATA));
    stopOnError(rclc_executor_add_timer(&executor, &timer_odom_));

    xTaskCreate(pidControlTaskFn, "pid_ctrl", 2048, this, 3, &pid_task_handle_);
}

void CarControllerApp::update()
{
    kinematics_.update_motor_speed(
        millis(),
        encoders_[0].getTicks(), encoders_[1].getTicks(),
        encoders_[2].getTicks(), encoders_[3].getTicks());
}

void CarControllerApp::publishOdom() {
    odom_t odom = kinematics_.get_odom();
    int64_t stamp = rmw_uros_epoch_millis();
    msg_odom_.header.stamp.sec     = (int32_t)(stamp / 1000);
    msg_odom_.header.stamp.nanosec = (uint32_t)((stamp % 1000) * 1000000);
    msg_odom_.pose.pose.position.x = odom.x;
    msg_odom_.pose.pose.position.y = odom.y;
    msg_odom_.pose.pose.orientation.w = cosf(odom.angle / 2.0f);
    msg_odom_.pose.pose.orientation.z = sinf(odom.angle / 2.0f);
    msg_odom_.twist.twist.linear.x  = odom.linear_speed;
    msg_odom_.twist.twist.angular.z = odom.angle_speed;
    rcl_ret_t ret = rcl_publish(&pub_odom_, &msg_odom_, nullptr); (void)ret;
}

void CarControllerApp::twistCallback(const void *msgin) {
    if (!instance_ || !msgin) return;
    const auto &msg = *static_cast<const geometry_msgs__msg__Twist *>(msgin);
    float linear_mm = msg.linear.x * 1000.0f;
    float angular   = msg.angular.z;
    float fl, fr, rl, rr;
    instance_->kinematics_.kinematics_inverse(linear_mm, angular, fl, fr, rl, rr);
    instance_->pid_[0].update_target(fl);
    instance_->pid_[1].update_target(fr);
    instance_->pid_[2].update_target(rl);
    instance_->pid_[3].update_target(rr);
}

void CarControllerApp::odomTimerCallback(rcl_timer_t *timer, int64_t) {
    if (!instance_ || !timer) return;
    instance_->publishOdom();
}

void CarControllerApp::pidControlTaskFn(void *args) {
    auto *self = static_cast<CarControllerApp *>(args);
    TickType_t last_wake = xTaskGetTickCount();

    while (true) 
    {
        for (int i = 0; i < 4; i++) 
        {
            float speed = self->kinematics_.get_motor_speed(i);
            float target = self->pid_[i].get_target();
            float out = self->pid_[i].update(speed);

            if (target == 0.0f && fabsf(speed) < 30.0f) 
            {
                out = 0;
                self->pid_[i].reset();
            }

            self->motor_.updateMotorSpeed(i, (int16_t)out);
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}
