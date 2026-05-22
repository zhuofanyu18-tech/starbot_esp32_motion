#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rmw_microros/rmw_microros.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "config/AppConfig.h"
// #include "apps/MicroRosArmControllerApp.h"  // 机械臂已禁用
#include "apps/CarControllerApp.h"
#include "apps/ImuApp.h"
#include "apps/StepperMotorApp.h"

namespace {

rcl_allocator_t   allocator_;
rclc_support_t    support_;
rcl_node_t        node_;
rclc_executor_t   executor_;

// MicroRosArmControllerApp arm_app_;  // 机械臂已禁用
CarControllerApp         car_app_;
ImuApp                   imu_app_;
StepperMotorApp          stepper_app_;

} // namespace

void setup() {
    Serial.begin(app_config::kDebugSerialBaudrate);
    delay(1000);

    set_microros_serial_transports(Serial);
    delay(2000);

    allocator_ = rcl_get_default_allocator();
    rclc_support_init(&support_, 0, nullptr, &allocator_);
    rclc_node_init_default(&node_, app_config::kNodeName, "", &support_);

    constexpr size_t kHandles =
        CarControllerApp::kExecutorHandles +
        ImuApp::kExecutorHandles + StepperMotorApp::kExecutorHandles;
    rclc_executor_init(&executor_, &support_.context, kHandles, &allocator_);

    car_app_.begin(support_, node_, executor_);
    // arm_app_.begin(support_, node_, executor_);  // 机械臂已禁用
    // imu_app_.begin(support_, node_, executor_);
    stepper_app_.begin(support_, node_, executor_);

    while (!rmw_uros_epoch_synchronized()) {
        rmw_uros_sync_session(1000);
        delay(100);
    }
}

void loop() {
    rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(app_config::kExecutorSpinPeriodMs));
    // arm_app_.update();  // 机械臂已禁用
    car_app_.update();
    // imu_app_.update();
    stepper_app_.update();
    delay(1);
}
