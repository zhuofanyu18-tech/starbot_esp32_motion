#ifndef MICRO_ROS_ARM_CONTROLLER_APP_H
#define MICRO_ROS_ARM_CONTROLLER_APP_H

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/joint_state.h>
#include <trajectory_msgs/msg/joint_trajectory.h>

#include "config/AppConfig.h"
#include "ServoController.h"

class MicroRosArmControllerApp {
public:
    // 用于注册 ROS 实体的句柄数量
    static constexpr size_t kExecutorHandles = 3;
    // 需要控制的关节数量
    static constexpr size_t kControlledArmJointCount = 4;

    MicroRosArmControllerApp();

    // 初始化硬件和 ROS 实体，将自身注册到外部共享 executor
    void begin(rclc_support_t &support, rcl_node_t &node, rclc_executor_t &executor);

    // 非阻塞更新：推进轨迹播放状态机
    void update();

private:
    static MicroRosArmControllerApp* instance_;

    ServoController servo_controller_;

    rcl_subscription_t sub_arm_{};
    rcl_subscription_t sub_gripper_{};
    rcl_publisher_t    pub_joint_states_{};
    rcl_timer_t        timer_joint_states_{};

    trajectory_msgs__msg__JointTrajectory msg_arm_traj_{};
    trajectory_msgs__msg__JointTrajectory msg_gripper_traj_{};
    sensor_msgs__msg__JointState          msg_joint_states_{};

    struct ArmPlayback {
        bool     active          = false;
        uint32_t start_ms        = 0;
        size_t   point_count     = 0;
        size_t   next_point      = 0;
        uint32_t offsets_ms[app_config::kTrajectoryPointCapacity]                              = {};
        double   positions[app_config::kTrajectoryPointCapacity][kControlledArmJointCount]     = {};
    };

    struct GripperPlayback {
        bool     active      = false;
        uint32_t start_ms    = 0;
        size_t   point_count = 0;
        size_t   next_point  = 0;
        uint32_t offsets_ms[app_config::kTrajectoryPointCapacity] = {};
        double   positions[app_config::kTrajectoryPointCapacity]  = {};
    };

    ArmPlayback     arm_pb_;
    GripperPlayback gripper_pb_;
    bool            warned_joint5_ = false;

    void handleArmTrajectory(const trajectory_msgs__msg__JointTrajectory &msg);
    void handleGripperTrajectory(const trajectory_msgs__msg__JointTrajectory &msg);
    void processArmPlayback();
    void processGripperPlayback();
    void publishJointStates();

    static void armTrajectoryCallback(const void *msgin);
    static void gripperTrajectoryCallback(const void *msgin);
    static void jointStatesTimerCallback(rcl_timer_t *timer, int64_t last_call_time);
};

#endif
