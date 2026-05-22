#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <stddef.h>

#include "HX_30HM.h"

/**
 * ServoController - 机械臂舵机控制器类
 *
 * 该类负责控制5自由度机械臂（5个关节舵机 + 1个夹爪舵机）的运动。
 * 包含了软件层的命令去抖算法，用于减少机械臂在目标位置附近的抖动问题。
 */
class ServoController {
public:
    /** 舵机总数：5个关节舵机 + 1个夹爪舵机 = 6个 */
    static constexpr uint8_t kServoCount = 6;

    /** 机械臂关节数量（不包括夹爪） */
    static constexpr uint8_t kArmJointCount = 5;

    /** 夹爪完全关闭时的宽度（单位：米） */
    static constexpr float kGripperClosedWidth = 0.0f;

    /** 夹爪完全打开时的宽度（单位：米） */
    static constexpr float kGripperOpenWidth = 0.02f;

    /**
     * 构造函数
     * @param uart      硬件串口引用，用于与舵机通信
     * @param baudrate  通信波特率（如115200）
     * @param tx_pin    TX引脚编号
     * @param rx_pin    RX引脚编号
     */
    ServoController(HardwareSerial& uart, uint32_t baudrate, uint8_t tx_pin, uint8_t rx_pin);

    /** 初始化舵机控制器，设置所有舵机的初始状态 */
    void servo_init();
    /** 设置开机时是否执行 joint5 的回中动作（用于 passive joint5 场景） */
    void setJoint5StartupMotionEnabled(bool enabled);

    /**
     * 移动机械臂关节到指定位置
     * @param joint_positions  关节角度数组（弧度制）
     * @param joint_count      关节数量
     */
    void moveArmJoints(const double *joint_positions, size_t joint_count);

    /**
     * 控制夹爪移动到指定位置
     * @param gripper_position 夹爪位置（0.0=完全关闭, 1.0=完全打开）
     */
    void moveGripper(double gripper_position);

    /**
     * 读取当前所有关节的位置
     * @param joint_positions  输出参数，存储读取到的关节角度（弧度制）
     * @param joint_count       要读取的关节数量
     * @return                  读取成功返回true，失败返回false
     */
    bool readJointPositions(double *joint_positions, size_t joint_count);

    /**
     * 直接控制所有舵机（5个关节 + 1个夹爪）
     * @param joint_1~5  各关节的角度值（弧度）
     * @param gripper    夹爪位置值
     */
    void control_servos(float joint_1, float joint_2, float joint_3, float joint_4, float joint_5, float gripper);

private:
    /** 串口舵机通信对象，用于与舵机进行串口通信 */
    SerialServo servo;

    /** 6个舵机的ID号，索引0~4对应5个关节，索引5对应夹爪 */
    uint8_t servoIDs[kServoCount] = {1, 2, 3, 4, 5, 6};

    /** 舵机加速度配置值（范围0~100），值越小加减速越平滑 */
    uint8_t acc = 40;

    /** 舵机速度配置值（范围0~1000），值越大移动越快 */
    int16_t speed = 300;

    /** 舵机最大扭矩限制，单位为0.1Nm。例如1000表示限制最大扭矩为100Nm */
    static constexpr uint16_t kMaxTorque = 1000;

    /**
     * 舵机硬件内部控制死区（Hardware Deadband）
     * 舵机内部会忽略小于此值的位置变化命令，用于减少舵机自身的抖动
     * 单位为位置计数，约等于0.3°/count
     */
    static constexpr uint8_t kDeadband = 5;

    /**
     * 机械臂关节命令去抖阈值（Software Debounce）
     * 软件层去抖：当新命令与上次命令的差值小于此阈值时，忽略该命令
     * 这样可以避免舵机在目标位置附近反复微调导致抖动
     * 单位为位置计数
     */
    static constexpr int16_t kArmCommandDeadbandCounts = 2;

    /** 夹爪命令去抖阈值，比关节更敏感（因为夹爪需要更精确的控制） */
    static constexpr int16_t kGripperCommandDeadbandCounts = 3;

    /** 夹爪完全关闭时对应的舵机位置计数值 */
    static constexpr int16_t kGripperClosedPosition = 1405;

    /** 夹爪完全打开时对应的舵机位置计数值 */
    static constexpr int16_t kGripperOpenPosition = 2300;

    /** 记录每个关节上次发送的命令位置，用于去抖判断 */
    int16_t last_arm_command_pos_[kArmJointCount] = {0};

    /** 记录夹爪上次发送的命令位置 */
    int16_t last_gripper_command_pos_ = 0;

    /** 标志位：是否已有上一次的关节命令记录（首次命令无需比较直接执行） */
    bool has_last_arm_command_ = false;

    /** 标志位：标志位：是否已有上一次的夹爪命令记录 */
    bool has_last_gripper_command_ = false;
    /** 开机策略：是否发送 joint5 的回中命令 */
    bool move_joint5_on_startup_ = false;

    /**
     * 线性映射函数：将一个值从输入范围映射到输出范围
     * @param value    要映射的值
     * @param in_min   输入范围最小值
     * @param in_max   输入范围最大值
     * @param out_min  输出范围最小值
     * @param out_max  输出范围最大值
     * @return         映射后的整数值
     */
    int16_t linearMapToPosition(
        float value,
        float in_min,
        float in_max,
        int16_t out_min,
        int16_t out_max) const;

    /**
     * 线性映射函数（双精度版本）：将位置计数值映射为弧度
     * @param value    位置计数值
     * @param in_min   输入范围最小值
     * @param in_max   输入范围最大值
     * @param out_min  输出弧度最小值
     * @param out_max  输出弧度最大值
     * @return         映射后的弧度值
     */
    double linearMapToRadian(
        int16_t value,
        int16_t in_min,
        int16_t in_max,
        float out_min,
        float out_max) const;

    /**
     * 将数值限制在指定范围内
     * @param value      要限制的值
     * @param min_value  最小值
     * @param max_value  最大值
     * @return           限制后的值
     */
    float clampFloat(float value, float min_value, float max_value) const;

    /**
     * 将关节角度（弧度）转换为舵机位置计数值
     * @param radians     关节角度（弧度）
     * @param servo_index 舵机索引（0~4对应5个关节舵机）
     * @return            舵机位置计数值
     */
    int16_t radians_to_position(float radians, uint8_t servo_index) const;

    /**
     * 将舵机位置计数值转换为关节角度（弧度）
     * @param position    舵机位置计数值
     * @param servo_index 舵机索引
     * @return            关节角度（弧度）
     */
    double position_to_radians(int16_t position, uint8_t servo_index) const;

    /**
     * 将夹爪开合宽度转换为舵机位置计数值
     * @param width  夹爪宽度（米）
     * @return       舵机位置计数值
     */
    int16_t gripper_width_to_position(float width) const;

    /**
     * 将舵机位置计数值转换为夹爪开合宽度
     * @param position  舵机位置计数值
     * @return         夹爪宽度（米）
     */
    double position_to_gripper_width(int16_t position) const;

    /**
     * 将位置计数值限制在有效范围内（0~4095是12位舵机的全范围）
     * @param position  要限制的位置值
     * @return          限制后的位置值
     */
    int16_t clamp_position(int32_t position) const;
};

#endif
