#ifndef _ML_TROL_H__
#define _ML_TROL_H__

#include <Arduino.h>
#include "BujinControl.h"

// 小车里程计结构体
struct OdometryData
{
    float pos_x;      // 机器人在X方向的位置 (m)
    float pos_y;      // 机器人在Y方向的位置 (m)
    float orientation; // 机器人的朝向 (rad)
    float linear_vel_x;  // 机器人的x方向线速度 (m/s)
    float linear_vel_y;  // 机器人的y方向线速度 (m/s)
    float angular_vel; // 机器人的角速度 (rad/s)

    // 构造函数
    OdometryData();
    OdometryData(float x, float y, float orient, float lin_vel_x, float lin_vel_y, float ang_vel);
};

// 轮子速度结构体
struct WheelSpeeds
{
    float front_left;  // 左前轮
    float front_right; // 右前轮
    float rear_left;   // 左后轮
    float rear_right;  // 右后轮

    // 构造函数
    WheelSpeeds();
    WheelSpeeds(float fl, float fr, float rl, float rr);
};

class MecanumKinematics
{

private:
    float car_len;      // 车子的长度 (轮子的前后距离)
    float car_wid;      // 车子的宽度 (轮子的左右距离)
    float wheel_radius; // 轮子的半径
    OdometryData odom_data;  // 里程计数据
    uint32_t last_update_time; // 上次更新时间 (ms)
    uint8_t dir[4] = {0}; // 方向数组（0正转，1反转）

public:
    // 构造函数，自动设置车子的配置
    MecanumKinematics(float length, float width, float wheel_diameter);

    // 运动学逆解：将机器人速度转换为轮子速度
    WheelSpeeds inverseKinematics(float vel_x, float vel_y, float angular_vel) const;

    // 运动学正解：将轮子速度转换为机器人速度
    void forwardKinematics(float fl, float fr, float rl, float rr, float &vel_x, float &vel_y, float &angular_vel) const;

    // 设置轮子半径
    void setWheelRadius(float radius);

    // 设置车辆尺寸
    void setCarDimensions(float length, float width);

    // 电机初始化
    void MotorInit();
    
    // 设置电机速度
    void setMotorSpeed(const WheelSpeeds& speeds);

    // 读取当前电机速度并更新里程计
    void updateOdometry(uint32_t dt_ms);

    // 获取里程计数据
    const OdometryData& getOdometryData() const;
    
    // 重置里程计
    void resetOdometry();
};

#endif