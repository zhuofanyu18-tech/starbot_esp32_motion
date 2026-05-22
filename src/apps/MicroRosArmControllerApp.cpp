#include "apps/MicroRosArmControllerApp.h"

#include <cstring>
#include <micro_ros_platformio.h>
#include <rmw_microros/rmw_microros.h>

#include "utils/RosMessageMemory.h"

namespace {

const char *const kJointNames[ServoController::kServoCount] = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "end_joint1"};

const char *const kControlledArmJointNames[MicroRosArmControllerApp::kControlledArmJointCount] = {
    "joint1", "joint2", "joint3", "joint4"};

void stopOnError(rcl_ret_t ret) {
    if (ret != RCL_RET_OK) {
        // Serial.printf("[ArmController] RCL error %d, restarting...\n", (int)ret);
        delay(2000);
        esp_restart();
    }
}

void stopOnFalse(bool ok) {
    if (!ok) {
        // Serial.println("[ArmController] init failed, restarting...");
        delay(2000);
        esp_restart();
    }
}

uint32_t durationToMs(const builtin_interfaces__msg__Duration &d) {
    int64_t ms = (int64_t)d.sec * 1000LL + (int64_t)d.nanosec / 1000000LL;
    if (ms < 0) ms = 0;
    if (ms > (int64_t)UINT32_MAX) return UINT32_MAX;
    return (uint32_t)ms;
}

int findJoint(const trajectory_msgs__msg__JointTrajectory &msg, const char *name) {
    if (!msg.joint_names.data) return -1;
    for (size_t i = 0; i < msg.joint_names.size; ++i) {
        if (msg.joint_names.data[i].data && strcmp(msg.joint_names.data[i].data, name) == 0)
            return (int)i;
    }
    return -1;
}

} // namespace

MicroRosArmControllerApp* MicroRosArmControllerApp::instance_ = nullptr;

MicroRosArmControllerApp::MicroRosArmControllerApp() : servo_controller_(Serial1, app_config::kServoBaudrate, app_config::kServoTxPin, app_config::kServoRxPin) 
{
    instance_ = this;
}

void MicroRosArmControllerApp::begin(
    rclc_support_t &support, rcl_node_t &node, rclc_executor_t &executor) {

    servo_controller_.setJoint5StartupMotionEnabled(app_config::kEnableJoint5StartupMotion);
    servo_controller_.servo_init();

    stopOnFalse(ros_message_memory::allocateTrajectoryMessage(
        msg_arm_traj_, ServoController::kArmJointCount, app_config::kTrajectoryPointCapacity));
    stopOnFalse(ros_message_memory::allocateTrajectoryMessage(
        msg_gripper_traj_, 1, app_config::kTrajectoryPointCapacity));
    stopOnFalse(ros_message_memory::allocateJointStateMessage(
        msg_joint_states_, ServoController::kServoCount, kJointNames));

    stopOnError(rclc_subscription_init_default(
        &sub_arm_, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectory),
        app_config::kArmTrajectoryTopic));

    stopOnError(rclc_subscription_init_default(
        &sub_gripper_, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectory),
        app_config::kGripperTrajectoryTopic));

    stopOnError(rclc_publisher_init_default(
        &pub_joint_states_, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        app_config::kJointStatesTopic));

    stopOnError(rclc_timer_init_default(
        &timer_joint_states_, &support,
        RCL_MS_TO_NS(app_config::kJointStatePublishPeriodMs),
        jointStatesTimerCallback));

    stopOnError(rclc_executor_add_subscription(
        &executor, &sub_arm_, &msg_arm_traj_, armTrajectoryCallback, ON_NEW_DATA));
    stopOnError(rclc_executor_add_subscription(
        &executor, &sub_gripper_, &msg_gripper_traj_, gripperTrajectoryCallback, ON_NEW_DATA));
    stopOnError(rclc_executor_add_timer(&executor, &timer_joint_states_));
}

void MicroRosArmControllerApp::update() {
    processArmPlayback();
    processGripperPlayback();
}

void MicroRosArmControllerApp::handleArmTrajectory(
    const trajectory_msgs__msg__JointTrajectory &msg) {
    if (!msg.points.size || !msg.points.data) return;

    arm_pb_.point_count = 0;
    size_t capacity = app_config::kTrajectoryPointCapacity;
    size_t count = msg.points.size < capacity ? msg.points.size : capacity;

    for (size_t i = 0; i < count; ++i) {
        const auto &pt = msg.points.data[i];
        if (!pt.positions.data) continue;

        double pos[kControlledArmJointCount] = {};
        bool ok = true;

        if (!msg.joint_names.data || msg.joint_names.size == 0) {
            if (pt.positions.size < kControlledArmJointCount) { ok = false; }
            else {
                for (size_t j = 0; j < kControlledArmJointCount; ++j)
                    pos[j] = pt.positions.data[j];
            }
        } else {
            bool has_joint5 = findJoint(msg, "joint5") >= 0;
            if (has_joint5 && app_config::kWarnWhenIgnoringJoint5InTrajectory && !warned_joint5_) {
                warned_joint5_ = true;
            }
            for (size_t j = 0; j < kControlledArmJointCount; ++j) {
                int src = findJoint(msg, kControlledArmJointNames[j]);
                if (src < 0 || (size_t)src >= pt.positions.size) { ok = false; break; }
                pos[j] = pt.positions.data[src];
            }
        }

        if (!ok) continue;
        if (arm_pb_.point_count >= capacity) break;
        arm_pb_.offsets_ms[arm_pb_.point_count] = durationToMs(pt.time_from_start);
        for (size_t j = 0; j < kControlledArmJointCount; ++j)
            arm_pb_.positions[arm_pb_.point_count][j] = pos[j];
        arm_pb_.point_count++;
    }

    if (arm_pb_.point_count > 0) {
        arm_pb_.active = true;
        arm_pb_.start_ms = millis();
        arm_pb_.next_point = 0;
    }
}

void MicroRosArmControllerApp::handleGripperTrajectory(
    const trajectory_msgs__msg__JointTrajectory &msg) {
    if (!msg.points.size || !msg.points.data) return;

    gripper_pb_.point_count = 0;
    size_t capacity = app_config::kTrajectoryPointCapacity;
    size_t count = msg.points.size < capacity ? msg.points.size : capacity;

    for (size_t i = 0; i < count; ++i) {
        const auto &pt = msg.points.data[i];
        if (!pt.positions.data || pt.positions.size == 0) continue;
        if (gripper_pb_.point_count >= capacity) break;
        gripper_pb_.offsets_ms[gripper_pb_.point_count] = durationToMs(pt.time_from_start);
        gripper_pb_.positions[gripper_pb_.point_count]  = pt.positions.data[0];
        gripper_pb_.point_count++;
    }

    if (gripper_pb_.point_count > 0) {
        gripper_pb_.active = true;
        gripper_pb_.start_ms = millis();
        gripper_pb_.next_point = 0;
    }
}

void MicroRosArmControllerApp::processArmPlayback() {
    if (!arm_pb_.active) return;
    uint32_t elapsed = millis() - arm_pb_.start_ms;
    while (arm_pb_.next_point < arm_pb_.point_count &&
           elapsed >= arm_pb_.offsets_ms[arm_pb_.next_point]) {
        servo_controller_.moveArmJoints(
            arm_pb_.positions[arm_pb_.next_point], kControlledArmJointCount);
        arm_pb_.next_point++;
    }
    if (arm_pb_.next_point >= arm_pb_.point_count) arm_pb_.active = false;
}

void MicroRosArmControllerApp::processGripperPlayback() {
    if (!gripper_pb_.active) return;
    uint32_t elapsed = millis() - gripper_pb_.start_ms;
    while (gripper_pb_.next_point < gripper_pb_.point_count &&
           elapsed >= gripper_pb_.offsets_ms[gripper_pb_.next_point]) {
        servo_controller_.moveGripper(gripper_pb_.positions[gripper_pb_.next_point]);
        gripper_pb_.next_point++;
    }
    if (gripper_pb_.next_point >= gripper_pb_.point_count) gripper_pb_.active = false;
}

void MicroRosArmControllerApp::publishJointStates() {
    double positions[ServoController::kServoCount] = {};
    if (!servo_controller_.readJointPositions(positions, ServoController::kServoCount)) return;
    for (uint8_t i = 0; i < ServoController::kServoCount; ++i)
        msg_joint_states_.position.data[i] = positions[i];
    const int64_t now = rmw_uros_epoch_nanos();
    msg_joint_states_.header.stamp.sec     = (int32_t)(now / 1000000000LL);
    msg_joint_states_.header.stamp.nanosec = (uint32_t)(now % 1000000000LL);
    rcl_ret_t ret = rcl_publish(&pub_joint_states_, &msg_joint_states_, nullptr); (void)ret;
}

void MicroRosArmControllerApp::armTrajectoryCallback(const void *msgin) {
    if (!instance_ || !msgin) return;
    instance_->handleArmTrajectory(
        *static_cast<const trajectory_msgs__msg__JointTrajectory *>(msgin));
}

void MicroRosArmControllerApp::gripperTrajectoryCallback(const void *msgin) {
    if (!instance_ || !msgin) return;
    instance_->handleGripperTrajectory(
        *static_cast<const trajectory_msgs__msg__JointTrajectory *>(msgin));
}

void MicroRosArmControllerApp::jointStatesTimerCallback(rcl_timer_t *timer, int64_t) {
    if (!instance_ || !timer) return;
    instance_->publishJointStates();
}
