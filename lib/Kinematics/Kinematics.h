#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include <Arduino.h>

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

public:
    // 构造函数，自动设置车子的配置
    MecanumKinematics(float length, float width, float wheel_diameter);

    // 运动学逆解：将机器人速度转换为轮子速度
    WheelSpeeds inverseKinematics(float vel_x, float vel_y, float angular_vel) const;

    // 运动学正解：将轮子速度转换为机器人速度
    void forwardKinematics(const WheelSpeeds &wheel_speeds,
                           float &vel_x, float &vel_y, float &angular_vel) const;

    // 设置轮子半径
    void setWheelRadius(float radius);

    // 设置车辆尺寸
    void setCarDimensions(float length, float width);
};

#endif