#include "ServoController.h"

#include <Arduino.h>
#include <math.h>

namespace {

// 关节标定结构体：存储关节的角度范围和对应的舵机位置
struct JointCalibration {
    float min_rad;     // 最小角度（弧度）
    int16_t min_pos;   // 最小角度对应的舵机位置
    float mid_rad;     // 中间角度（弧度）
    int16_t mid_pos;   // 中间角度对应的舵机位置
    float max_rad;     // 最大角度（弧度）
    int16_t max_pos;   // 最大角度对应的舵机位置
    bool use_mid;      // 是否使用中间点进行分段映射
};

// 关节角度 -> 舵机位置 标定表（用户实测数据）
const JointCalibration kJointCalibration[ServoController::kArmJointCount] = {
    {-1.6f, 3120, 0.0f, 2065, 1.6f, 1010, true},    // joint1
    {-1.75f, 3211, 0.0f, 2070, 1.75f, 929, true},   // joint2
    {0.0f, 2048, 0.0f, 2048, 3.0f, 92, false},         // joint3
    {-1.75f, 907, 0.0f, 2048, 1.75f, 3189, true},  // joint4
    {-2.0f, 744, 0.0f, 2048, 2.0f, 3352, true},     // joint5
};

// 机械臂初始位置（开机时的默认位置）
const int16_t kHomePosition[ServoController::kServoCount] = {
    2065, 2070, 2048, 2048, 2048, 1405};  // 前5个是关节位置，最后一个是夹爪位置

// 关节5的舵机索引
constexpr uint8_t kJoint5ServoIndex = 4;

// 判断位置是否在 min 到 mid 区间
bool isOnMinToMidSegment(int16_t position, int16_t min_pos, int16_t mid_pos) 
{
    if (min_pos <= mid_pos) 
    {
        return position <= mid_pos;
    }
    return position >= mid_pos;
}

}  // namespace

// 构造函数：初始化串口通信
ServoController::ServoController(HardwareSerial& uart, uint32_t baudrate, uint8_t tx_pin, uint8_t rx_pin)
    : servo(uart, baudrate, tx_pin, rx_pin) {
}

// 设置关节5是否在启动时移动
void ServoController::setJoint5StartupMotionEnabled(bool enabled) {
    move_joint5_on_startup_ = enabled;
}

// 舵机初始化流程
void ServoController::servo_init() {
    // 1. 测试所有舵机通信
    for (int i = 0; i < kServoCount; i++) {
        ServoStatus_t status = servo.ping(servoIDs[i]);
        // 检查舵机是否在线
        delay(100);
    }
    
    // 2. 启用所有舵机力矩
    for (int i = 0; i < kServoCount; i++) {
        ServoStatus_t status = servo.enable_torque(servoIDs[i]);
        delay(100);
    }

    // 2.1 设为最大力矩，并设置死区参数
    for (int i = 0; i < kServoCount; i++) {
        uint8_t deadband = kDeadband;
        servo.write_max_torque(servoIDs[i], kMaxTorque);
        servo.general_write(servoIDs[i], REG_CW_DEAD, &deadband, 1);
        servo.general_write(servoIDs[i], REG_CCW_DEAD, &deadband, 1);
        delay(50);
    }
    
    // 3. 设置所有舵机工作模式为位置模式
    for (int i = 0; i < kServoCount; i++) {
        ServoStatus_t status = servo.select_mode(servoIDs[i], 0);
        delay(100);
    }

    // 4. 移动到初始位置
    for (int i = 5; i >= 0; i--) 
    {
        // 跳过关节5（如果设置了不移动）
        if (i == static_cast<int>(kJoint5ServoIndex) && !move_joint5_on_startup_) {
            continue;
        }
        // 移动到初始位置
        servo.write_pos_ex(servoIDs[i], acc, speed, kHomePosition[i]);
        delay(50);
    }

    // 5. 记录初始位置，用于后续的去抖判断
    for (int i = 4; i >= 0; --i) 
    {
        last_arm_command_pos_[i] = kHomePosition[i];
    }

    last_gripper_command_pos_ = kHomePosition[kServoCount - 1];
    has_last_arm_command_ = true;
    has_last_gripper_command_ = true;
}

// 移动机械臂关节到指定位置
void ServoController::moveArmJoints(const double *joint_positions, size_t joint_count) {
    if (joint_positions == NULL || joint_count == 0) {
        return;
    }

    // 确定要控制的关节数量
    const uint8_t arm_joint_count = joint_count < kArmJointCount
        ? static_cast<uint8_t>(joint_count)
        : kArmJointCount;

    int16_t filtered_targets[kArmJointCount] = {0};
    bool should_send = !has_last_arm_command_;  // 首次命令直接发送

    // 处理每个关节的位置命令
    for (uint8_t index = 0; index < arm_joint_count; ++index) {
        // 将弧度转换为舵机位置
        const int16_t target_pos = radians_to_position(
            static_cast<float>(joint_positions[index]),
            index);

        // 首次命令，记录位置并直接发送
        if (!has_last_arm_command_) {
            last_arm_command_pos_[index] = target_pos;
            filtered_targets[index] = target_pos;
            continue;
        }

        // 计算位置变化量，应用去抖逻辑
        int32_t delta = static_cast<int32_t>(target_pos) -
            static_cast<int32_t>(last_arm_command_pos_[index]);
        if (delta < 0) {
            delta = -delta;  // 取绝对值
        }

        // 只有当变化量超过阈值时才更新位置
        if (delta >= kArmCommandDeadbandCounts) {
            last_arm_command_pos_[index] = target_pos;
            should_send = true;
        }
        filtered_targets[index] = last_arm_command_pos_[index];
    }

    has_last_arm_command_ = true;
    if (!should_send) {
        return;  // 变化量太小，不发送命令
    }

    // 准备同步控制数据
    int16_t sync_data[kArmJointCount][4] = {0};
    for (uint8_t index = 0; index < arm_joint_count; ++index) {
        sync_data[index][0] = servoIDs[index];  // 舵机ID
        sync_data[index][1] = acc;              // 加速度
        sync_data[index][2] = speed;            // 速度
        sync_data[index][3] = filtered_targets[index];  // 目标位置
    }

    // 同步发送位置命令
    servo.sync_write_pos_ex(sync_data, arm_joint_count);
}

// 控制夹爪移动
void ServoController::moveGripper(double gripper_position) {
    // 将夹爪位置转换为舵机位置
    const int16_t target_pos = gripper_width_to_position(static_cast<float>(gripper_position));
    
    // 应用去抖逻辑
    if (has_last_gripper_command_) {
        int32_t delta = static_cast<int32_t>(target_pos) -
            static_cast<int32_t>(last_gripper_command_pos_);
        if (delta < 0) {
            delta = -delta;
        }
        if (delta < kGripperCommandDeadbandCounts) {
            return;  // 变化量太小，不发送命令
        }
    }
    
    // 更新夹爪位置并发送命令
    last_gripper_command_pos_ = target_pos;
    has_last_gripper_command_ = true;

    servo.write_pos_ex(
        servoIDs[kServoCount - 1],  // 夹爪舵机ID
        acc,
        speed,
        target_pos);
}

// 读取关节位置
bool ServoController::readJointPositions(double *joint_positions, size_t joint_count) {
    if (joint_positions == NULL || joint_count == 0) {
        return false;
    }

    // 同步读取所有舵机位置
    int16_t read_data[kServoCount][5] = {0};
    ServoStatus_t status = servo.sync_read_cur_pos_ex(servoIDs, kServoCount, read_data);
    if (status.error_bits.bit_tx != 0 || status.error_bits.bit_rx != 0) {
        return false;  // 通信错误
    }

    // 确定要读取的数量
    const uint8_t read_count = joint_count < kServoCount
        ? static_cast<uint8_t>(joint_count)
        : kServoCount;

    // 转换位置数据
    for (uint8_t index = 0; index < read_count; ++index) {
        if (index == kServoCount - 1) {
            // 夹爪位置转换为开合宽度
            joint_positions[index] = position_to_gripper_width(read_data[index][0]);
        } else {
            // 关节位置转换为弧度
            joint_positions[index] = position_to_radians(read_data[index][0], index);
        }
    }

    return true;
}

// 直接控制所有舵机（5个关节 + 1个夹爪）
void ServoController::control_servos(float joint_1, float joint_2, float joint_3, float joint_4, float joint_5, float gripper) {
    // 组织角度数据
    float angles[6] = {joint_1, joint_2, joint_3, joint_4, joint_5, gripper};
    
    // 准备同步控制数据
    int16_t sync_data[6][4];
    for (int i = 0; i < kServoCount; i++) {
        sync_data[i][0] = servoIDs[i];
        sync_data[i][1] = acc;
        sync_data[i][2] = speed;
        // 转换角度为舵机位置
        sync_data[i][3] = i == (kServoCount - 1)
            ? gripper_width_to_position(angles[i])
            : radians_to_position(angles[i], i);
    }
    
    // 同步发送命令
    ServoStatus_t status = servo.sync_write_pos_ex(sync_data, kServoCount);
}

// 将弧度转换为舵机位置
int16_t ServoController::radians_to_position(float radians, uint8_t servo_index) const {
    if (servo_index >= kArmJointCount) {
        return clamp_position(2048);  // 无效索引，返回中位
    }

    const JointCalibration& calibration = kJointCalibration[servo_index];

    // 不使用中间点的关节，直接线性映射
    if (!calibration.use_mid) {
        const float clamped = clampFloat(radians, calibration.min_rad, calibration.max_rad);
        return linearMapToPosition(
            clamped,
            calibration.min_rad,
            calibration.max_rad,
            calibration.min_pos,
            calibration.max_pos);
    }

    // 使用中间点的关节，分段映射
    if (radians <= calibration.mid_rad) {
        // 映射到 min 到 mid 段
        const float clamped = clampFloat(radians, calibration.min_rad, calibration.mid_rad);
        return linearMapToPosition(
            clamped,
            calibration.min_rad,
            calibration.mid_rad,
            calibration.min_pos,
            calibration.mid_pos);
    }

    // 映射到 mid 到 max 段
    const float clamped = clampFloat(radians, calibration.mid_rad, calibration.max_rad);
    return linearMapToPosition(
        clamped,
        calibration.mid_rad,
        calibration.max_rad,
        calibration.mid_pos,
        calibration.max_pos);
}

// 将舵机位置转换为弧度
double ServoController::position_to_radians(int16_t position, uint8_t servo_index) const {
    if (servo_index >= kArmJointCount) {
        return 0.0;  // 无效索引，返回0
    }

    const JointCalibration& calibration = kJointCalibration[servo_index];

    // 确保 min_pos < max_pos
    int16_t min_pos = calibration.min_pos;
    int16_t max_pos = calibration.max_pos;
    if (min_pos > max_pos) {
        const int16_t temp = min_pos;
        min_pos = max_pos;
        max_pos = temp;
    }

    // 限制位置在有效范围内
    int16_t clamped_position = position;
    if (clamped_position < min_pos) {
        clamped_position = min_pos;
    }
    if (clamped_position > max_pos) {
        clamped_position = max_pos;
    }

    // 不使用中间点的关节，直接线性映射
    if (!calibration.use_mid) {
        return linearMapToRadian(
            clamped_position,
            calibration.min_pos,
            calibration.max_pos,
            calibration.min_rad,
            calibration.max_rad);
    }

    // 使用中间点的关节，分段映射
    if (isOnMinToMidSegment(clamped_position, calibration.min_pos, calibration.mid_pos)) {
        return linearMapToRadian(
            clamped_position,
            calibration.min_pos,
            calibration.mid_pos,
            calibration.min_rad,
            calibration.mid_rad);
    }

    return linearMapToRadian(
        clamped_position,
        calibration.mid_pos,
        calibration.max_pos,
        calibration.mid_rad,
        calibration.max_rad);
}

// 将夹爪宽度转换为舵机位置
int16_t ServoController::gripper_width_to_position(float width) const {
    // 限制宽度在有效范围内
    const float clamped_width = clampFloat(width, kGripperClosedWidth, kGripperOpenWidth);
    // 线性映射
    return linearMapToPosition(
        clamped_width,
        kGripperClosedWidth,
        kGripperOpenWidth,
        kGripperClosedPosition,
        kGripperOpenPosition);
}

// 将舵机位置转换为夹爪宽度
double ServoController::position_to_gripper_width(int16_t position) const {
    // 限制位置在有效范围内
    int16_t clamped_position = position;
    if (clamped_position < kGripperClosedPosition) {
        clamped_position = kGripperClosedPosition;
    }
    if (clamped_position > kGripperOpenPosition) {
        clamped_position = kGripperOpenPosition;
    }

    // 线性映射
    return linearMapToRadian(
        clamped_position,
        kGripperClosedPosition,
        kGripperOpenPosition,
        kGripperClosedWidth,
        kGripperOpenWidth);
}

// 线性映射函数（弧度 -> 位置）
int16_t ServoController::linearMapToPosition(
    float value,
    float in_min,
    float in_max,
    int16_t out_min,
    int16_t out_max) const {
    if (in_max == in_min) {
        return clamp_position(out_min);
    }

    const float ratio = (value - in_min) / (in_max - in_min);
    const float mapped = static_cast<float>(out_min) +
        ratio * static_cast<float>(out_max - out_min);
    return clamp_position(static_cast<int32_t>(lroundf(mapped)));
}

// 线性映射函数（位置 -> 弧度）
double ServoController::linearMapToRadian(int16_t value, int16_t in_min, int16_t in_max, float out_min, float out_max) const 
{
    if (in_max == in_min) {
        return static_cast<double>(out_min);
    }

    const float ratio = static_cast<float>(value - in_min) /
        static_cast<float>(in_max - in_min);
    return static_cast<double>(out_min + ratio * (out_max - out_min));
}

// 限制浮点数在指定范围内
float ServoController::clampFloat(float value, float min_value, float max_value) const {
    float clamped = value;
    if (clamped < min_value) {
        clamped = min_value;
    }
    if (clamped > max_value) {
        clamped = max_value;
    }
    return clamped;
}

// 限制舵机位置在有效范围内（0~4096）
int16_t ServoController::clamp_position(int32_t position) const {
    if (position < 0) {
        return 0;
    }

    if (position > 4096) {
        return 4096;
    }

    return static_cast<int16_t>(position);
}
