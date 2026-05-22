#ifndef ROS_MESSAGE_MEMORY_H
#define ROS_MESSAGE_MEMORY_H

#include <stddef.h>

#include <sensor_msgs/msg/joint_state.h>
#include <trajectory_msgs/msg/joint_trajectory.h>

namespace ros_message_memory {

bool allocateTrajectoryMessage(
    trajectory_msgs__msg__JointTrajectory &msg,
    size_t joint_count,
    size_t point_capacity);

bool allocateJointStateMessage(
    sensor_msgs__msg__JointState &msg,
    size_t joint_count,
    const char *const joint_names[]);

}  // namespace ros_message_memory

#endif
