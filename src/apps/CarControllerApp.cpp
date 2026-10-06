#include "apps/CarControllerApp.h"

#include <cmath>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rmw_microros/rmw_microros.h>

namespace {

// ---- 板载 RGB 灯引脚冲突检查（编译期）----
constexpr bool rgbUses(const gpio_num_t *pins) {
    return app_config::kEnableRgbLed &&
           (app_config::kRgbLedPin == (uint8_t)pins[0] || app_config::kRgbLedPin == (uint8_t)pins[1]);
}

// RGB 灯占用了电机 PWM/DIR 引脚时无法安全运行，直接编译报错（例如 GPIO38 = DIRB）
static_assert(!(rgbUses(app_config::lf_motor) || rgbUses(app_config::rf_motor) ||
                rgbUses(app_config::lr_motor) || rgbUses(app_config::rr_motor)),
              "RGB LED pin conflicts with a motor PWM/DIR pin: change kRgbLedPin or set kEnableRgbLed=false");

// RGB 灯占用的编码器下标（例如 GPIO48 = ENC_B1 → 右前 1），-1 表示没有冲突
constexpr int kRgbEncoderIndex = rgbUses(app_config::lf_encoder) ? 0 :
                                 rgbUses(app_config::rf_encoder) ? 1 :
                                 rgbUses(app_config::lr_encoder) ? 2 :
                                 rgbUses(app_config::rr_encoder) ? 3 : -1;

// 同侧的另一个轮子：左前↔左后，右前↔右后
constexpr int kSameSideWheel[CarControllerApp::kWheelCount] = {2, 3, 0, 1};

}  // namespace

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
    portMUX_INITIALIZE(&lock_);
}

// 初始化硬件接口，包括电机、编码器、PID 控制器和运动学参数
void CarControllerApp::initHardware() {
    // DRV8701E：PWM + DIR
    const gpio_num_t *motors[kWheelCount] = {
        app_config::lf_motor, app_config::rf_motor, app_config::lr_motor, app_config::rr_motor};
    for (int i = 0; i < kWheelCount; i++) {
        motor_.attachMotor(i, motors[i][0], motors[i][1],
                           app_config::kMotorReversed[i], app_config::kMotorPwmFrequencyHz);
        motor_.updateMotorSpeed(i, 0);
    }

    const gpio_num_t *encoder_pins[kWheelCount] = {
        app_config::lf_encoder, app_config::rf_encoder, app_config::lr_encoder, app_config::rr_encoder};
    for (int i = 0; i < kWheelCount; i++) {
        if (i == kRgbEncoderIndex) continue;  // 引脚被板载 RGB 灯占用，不初始化
        encoders_[i].begin(static_cast<pcnt_unit_t>(i), encoder_pins[i][0], encoder_pins[i][1],
                           app_config::kEncoderGlitchFilterCycles);
    }

    for (int i = 0; i < kWheelCount; i++) {
        pid_[i].update_pid(app_config::Kp, app_config::Ki, app_config::Kd);
        pid_[i].out_limit(-1000, 1000);
        pid_[i].update_target(0);
    }

    kinematics_.set_wheel_distance(app_config::kWheelBaseMm);
    const float ppd = (app_config::kWheelDiameterMm * 3.1415926535f) / app_config::kEncoderPulsesPerRevolution;
    for (int i = 0; i < kWheelCount; i++) kinematics_.set_motor_param(i, ppd);

    // 字符串只分配一次，重连时复用
    msg_odom_.header.frame_id = micro_ros_string_utilities_set(msg_odom_.header.frame_id, "odom");
    msg_odom_.child_frame_id = micro_ros_string_utilities_set(msg_odom_.child_frame_id, "base_footprint");

    // 控制任务与 ROS 连接无关，上电即运行（目标速度为 0 时电机保持静止）
    // 固定在核 1、优先级最高，WiFi 和网页都在核 0，不会打断控制周期
    xTaskCreatePinnedToCore(controlTaskFn, "car_ctrl", app_config::kControlTaskStack, this,
                            app_config::kControlTaskPriority, &control_task_handle_, app_config::kRealtimeCore);
}

bool CarControllerApp::createRosEntities(rclc_support_t &support, rcl_node_t &node, rclc_executor_t &executor)
{
    if (rclc_subscription_init_best_effort(
            &sub_cmd_vel_, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            app_config::kCmdVelTopic) != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &pub_odom_, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
            app_config::kOdomTopic) != RCL_RET_OK) return false;

    if (rclc_timer_init_default(
            &timer_odom_, &support,
            RCL_MS_TO_NS(app_config::kOdomPublishPeriodMs),
            odomTimerCallback) != RCL_RET_OK) return false;

    if (rclc_executor_add_subscription(
            &executor, &sub_cmd_vel_, &msg_cmd_vel_, twistCallback, ON_NEW_DATA) != RCL_RET_OK) return false;
    if (rclc_executor_add_timer(&executor, &timer_odom_) != RCL_RET_OK) return false;
    return true;
}

void CarControllerApp::destroyRosEntities(rcl_node_t &node)
{
    rcl_ret_t ret;
    ret = rcl_subscription_fini(&sub_cmd_vel_, &node); (void)ret;
    ret = rcl_publisher_fini(&pub_odom_, &node); (void)ret;
    ret = rcl_timer_fini(&timer_odom_); (void)ret;
    sub_cmd_vel_ = rcl_get_zero_initialized_subscription();
    pub_odom_    = rcl_get_zero_initialized_publisher();
    timer_odom_  = rcl_get_zero_initialized_timer();
}

void CarControllerApp::setTargetVelocity(float linear_mps, float angular_radps, CmdSource source)
{
    portENTER_CRITICAL(&lock_);
    cmd_.linear_mps = linear_mps;
    cmd_.angular_radps = angular_radps;
    cmd_.source = source;
    cmd_.stamp_ms = millis();
    portEXIT_CRITICAL(&lock_);
}

void CarControllerApp::stop()
{
    setTargetVelocity(0.0f, 0.0f, CmdSource::kNone);
}

CarControllerApp::State CarControllerApp::getState() const
{
    portENTER_CRITICAL(&lock_);
    State copy = state_;
    portEXIT_CRITICAL(&lock_);
    return copy;
}

void CarControllerApp::publishOdom() {
    const State st = getState();
    int64_t stamp = rmw_uros_epoch_millis();
    msg_odom_.header.stamp.sec     = (int32_t)(stamp / 1000);
    msg_odom_.header.stamp.nanosec = (uint32_t)((stamp % 1000) * 1000000);
    msg_odom_.pose.pose.position.x = st.x;
    msg_odom_.pose.pose.position.y = st.y;
    msg_odom_.pose.pose.orientation.w = cosf(st.yaw / 2.0f);
    msg_odom_.pose.pose.orientation.z = sinf(st.yaw / 2.0f);
    msg_odom_.twist.twist.linear.x  = st.linear;
    msg_odom_.twist.twist.angular.z = st.angular;
    rcl_ret_t ret = rcl_publish(&pub_odom_, &msg_odom_, nullptr); (void)ret;
}

void CarControllerApp::twistCallback(const void *msgin) {
    if (!instance_ || !msgin) return;
    const auto &msg = *static_cast<const geometry_msgs__msg__Twist *>(msgin);
    instance_->setTargetVelocity(msg.linear.x, msg.angular.z, CmdSource::kRos);
}

void CarControllerApp::odomTimerCallback(rcl_timer_t *timer, int64_t) {
    if (!instance_ || !timer) return;
    instance_->publishOdom();
}

void CarControllerApp::controlStep() {
    // 1. 读编码器 → 轮速 → 里程计
    int32_t ticks[kWheelCount] = {0};
    for (int i = 0; i < kWheelCount; i++) {
        if (i == kRgbEncoderIndex) continue;
        ticks[i] = encoders_[i].getTicks();
        if (app_config::kEncoderReversed[i]) ticks[i] = -ticks[i];
    }
    // 被 RGB 灯占用的编码器用同侧另一个轮子代替，否则该轮 PID 一直读到 0 速会全速输出
    if (kRgbEncoderIndex >= 0) ticks[kRgbEncoderIndex] = ticks[kSameSideWheel[kRgbEncoderIndex]];
    kinematics_.update_motor_speed(esp_timer_get_time(), ticks[0], ticks[1], ticks[2], ticks[3]);

    // 2. 取最新速度指令，超时则停车
    portENTER_CRITICAL(&lock_);
    const Command cmd = cmd_;
    portEXIT_CRITICAL(&lock_);

    uint32_t timeout_ms = 0;
    if (cmd.source == CmdSource::kRos) timeout_ms = app_config::kRosCmdTimeoutMs;
    else if (cmd.source == CmdSource::kWifi) timeout_ms = app_config::kWifiCmdTimeoutMs;
    const bool timed_out = timeout_ms > 0 && (millis() - cmd.stamp_ms) > timeout_ms;
    const float linear = timed_out ? 0.0f : cmd.linear_mps;
    const float angular = timed_out ? 0.0f : cmd.angular_radps;

    // 3. 逆运动学 → PID → 电机
    float targets[kWheelCount];
    kinematics_.kinematics_inverse(linear * 1000.0f, angular, targets[0], targets[1], targets[2], targets[3]);
    for (int i = 0; i < kWheelCount; i++) {
        pid_[i].update_target(targets[i]);
        const float speed = kinematics_.get_motor_speed(i);
        float out = pid_[i].update(speed);

        if (targets[i] == 0.0f && fabsf(speed) < 30.0f) {
            out = 0;
            pid_[i].reset();
        }
        motor_.updateMotorSpeed(i, (int16_t)out);
    }

    // 4. 写状态快照
    const odom_t &odom = kinematics_.get_odom();
    State st;
    st.linear = odom.linear_speed;
    st.angular = odom.angle_speed;
    st.x = odom.x;
    st.y = odom.y;
    st.yaw = odom.angle;
    for (int i = 0; i < kWheelCount; i++) st.wheel_speed[i] = kinematics_.get_motor_speed(i) / 1000.0f;
    st.source = cmd.source;
    st.cmd_timed_out = timed_out;

    portENTER_CRITICAL(&lock_);
    state_ = st;
    portEXIT_CRITICAL(&lock_);
}

void CarControllerApp::controlTaskFn(void *args) {
    auto *self = static_cast<CarControllerApp *>(args);

    // 加入任务看门狗：控制任务卡死超过 5s 自动复位（复位时 PWM 归零，电机停转）
    esp_task_wdt_add(nullptr);

    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        self->controlStep();
        esp_task_wdt_reset();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(app_config::kControlPeriodMs));
    }
}
