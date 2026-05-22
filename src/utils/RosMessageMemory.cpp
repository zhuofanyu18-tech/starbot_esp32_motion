#include "utils/RosMessageMemory.h"

#include <cstdlib>

#include <micro_ros_utilities/string_utilities.h>
#include <rosidl_runtime_c/string_functions.h>

#include "config/AppConfig.h"

namespace ros_message_memory {

// 匿名命名空间：这些函数只在当前文件内可见
namespace {

//------------------------------------------------------------------------------
// allocateZeroedArray - 通用内存分配函数 ，为任意类型的数组分配内存并自动初始化为零
// T: 数组元素类型
// count: 数组长度
// 返回: 指向分配的数组的指针，失败返回 NULL
//------------------------------------------------------------------------------
template <typename T>
T *allocateZeroedArray(size_t count) {
    // calloc 会将分配的内存全部初始化为 0
    // sizeof(T) * count: 计算需要的总字节数
    // static_cast<T *>: 将 void* 转换为 T* 类型
    return static_cast<T *>(calloc(count, sizeof(T)));
}

//------------------------------------------------------------------------------
// initializeReservedString - 初始化一个 ROS 字符串，预分配指定容量
// value: 引用传递，需要初始化的 ROS 字符串
// 返回: 初始化成功返回 true，失败返回 false
//------------------------------------------------------------------------------
bool initializeReservedString(rosidl_runtime_c__String &value) {
    // micro_ros_string_utilities_init_with_size: 预分配指定大小的字符串
    // app_config::kRosStringCapacity = 20（在 AppConfig.h 中定义）
    value = micro_ros_string_utilities_init_with_size(app_config::kRosStringCapacity);
    
    // 检查字符串的 data 指针是否为 NULL（分配是否成功）
    return value.data != NULL;
}

//------------------------------------------------------------------------------
// allocateTrajectoryPointBuffers - 为轨迹点分配数据缓冲区
// point: 引用传递，需要分配缓冲区的轨迹点
// joint_count: 关节数量
// 返回: 分配成功返回 true，失败返回 false
//------------------------------------------------------------------------------
bool allocateTrajectoryPointBuffers(
    trajectory_msgs__msg__JointTrajectoryPoint &point,
    size_t joint_count) {
    // 1. 为位置数据分配内存
    point.positions.capacity = joint_count;  // 设置容量
    point.positions.size = 0;               // 当前大小为 0
    point.positions.data = allocateZeroedArray<double>(joint_count);  // 分配 double 数组

    // 2. 为速度数据分配内存
    point.velocities.capacity = joint_count;
    point.velocities.size = 0;
    point.velocities.data = allocateZeroedArray<double>(joint_count);

    // 3. 为加速度数据分配内存
    point.accelerations.capacity = joint_count;
    point.accelerations.size = 0;
    point.accelerations.data = allocateZeroedArray<double>(joint_count);

    // 4. 为力/力矩数据分配内存
    point.effort.capacity = joint_count;
    point.effort.size = 0;
    point.effort.data = allocateZeroedArray<double>(joint_count);

    // 检查所有数据分配是否成功（任何一个为 NULL 都失败）
    return point.positions.data != NULL &&
           point.velocities.data != NULL &&
           point.accelerations.data != NULL &&
           point.effort.data != NULL;
}

}  // namespace

//------------------------------------------------------------------------------
// allocateTrajectoryMessage - 为 JointTrajectory 消息分配内存
// msg: 引用传递，需要分配内存的消息
// joint_count: 关节数量
// point_capacity: 轨迹点容量上限
// 返回: 分配成功返回 true，失败返回 false
//------------------------------------------------------------------------------
bool allocateTrajectoryMessage(
    trajectory_msgs__msg__JointTrajectory &msg,
    size_t joint_count,
    size_t point_capacity) 
{
    if (point_capacity == 0) {
        return false;
    }

    // 1. 分配关节名称数组
    msg.joint_names.capacity = joint_count;  // 设置容量
    msg.joint_names.size = 0;               // 当前大小为 0
    // 分配 rosidl_runtime_c__String 类型的数组（每个元素是一个字符串）
    msg.joint_names.data = allocateZeroedArray<rosidl_runtime_c__String>(joint_count);
    if (msg.joint_names.data == NULL) {
        return false;  // 分配失败
    }

    // 2. 初始化每个关节名称字符串
    for (size_t index = 0; index < joint_count; ++index) {
        if (!initializeReservedString(msg.joint_names.data[index])) {
            return false;  // 初始化失败
        }
    }

    // 3. 分配轨迹点数组
    msg.points.capacity = point_capacity;
    msg.points.size = 0;
    // 分配 JointTrajectoryPoint 类型的数组
    msg.points.data = allocateZeroedArray<trajectory_msgs__msg__JointTrajectoryPoint>(
        point_capacity);
    if (msg.points.data == NULL) {
        return false;  // 分配失败
    }

    // 4. 为每个轨迹点分配数据缓冲区
    for (size_t index = 0; index < point_capacity; ++index) {
        if (!allocateTrajectoryPointBuffers(msg.points.data[index], joint_count)) {
            return false;  // 分配失败
        }
    }

    return true;  // 所有分配都成功
}

//------------------------------------------------------------------------------
// allocateJointStateMessage - 为 JointState 消息分配内存并填充关节名称
// msg: 引用传递，需要分配内存的消息
// joint_count: 关节数量
// joint_names: 关节名称数组（如 {"joint1", "joint2", ...}）
// 返回: 分配成功返回 true，失败返回 false
//------------------------------------------------------------------------------
bool allocateJointStateMessage(
    sensor_msgs__msg__JointState &msg,
    size_t joint_count,
    const char *const joint_names[]) {
    // 1. 分配关节名称数组
    msg.name.capacity = joint_count;  // 设置容量
    msg.name.size = joint_count;      // 当前大小等于容量（直接填充所有名称）
    // 分配 rosidl_runtime_c__String 类型的数组
    msg.name.data = allocateZeroedArray<rosidl_runtime_c__String>(joint_count);
    if (msg.name.data == NULL) {
        return false;  // 分配失败
    }

    // 2. 填充关节名称
    for (size_t index = 0; index < joint_count; ++index) {
        // 初始化一个空字符串
        msg.name.data[index] = micro_ros_string_utilities_init("");
        if (msg.name.data[index].data == NULL) {
            return false;  // 初始化失败
        }

        // 将 joint_names[index] 的内容复制到字符串中
        if (!rosidl_runtime_c__String__assign(&msg.name.data[index], joint_names[index])) {
            return false;  // 复制失败
        }
    }

    // 3. 分配关节位置数组
    msg.position.capacity = joint_count;  // 设置容量
    msg.position.size = joint_count;      // 当前大小等于容量（每个关节都有位置）
    // 分配 double 类型的数组（存储关节角度）
    msg.position.data = allocateZeroedArray<double>(joint_count);
    if (msg.position.data == NULL) {
        return false;  // 分配失败
    }

    return true;  // 所有分配都成功
}

}  // namespace ros_message_memory
