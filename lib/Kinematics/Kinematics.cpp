#include "Kinematics.h"
#include <cmath>

// WheelSpeeds 结构体的构造函数实现
WheelSpeeds::WheelSpeeds()
    : front_left(0), front_right(0), rear_left(0), rear_right(0) {}

WheelSpeeds::WheelSpeeds(float fl, float fr, float rl, float rr)
    : front_left(fl), front_right(fr), rear_left(rl), rear_right(rr) {}

// MecanumKinematics 类的构造函数实现
MecanumKinematics::MecanumKinematics(float length, float width, float wheel_diameter)
    : car_len(length), car_wid(width), wheel_radius(wheel_diameter / 2.0f) {}

// 运动学逆解：将机器人速度转换为轮子速度
WheelSpeeds MecanumKinematics::inverseKinematics(float vel_x, float vel_y, float angular_vel) const
{
    // 计算机器人几何参数
    float L = car_len / 2.0f; // 前后轮距的一半
    float W = car_wid / 2.0f; // 左右轮距的一半

    // 麦克纳姆轮运动学逆解公式
    float fl = (vel_x - vel_y - angular_vel * (L + W)) / wheel_radius;
    float fr = (vel_x + vel_y + angular_vel * (L + W)) / wheel_radius;
    float rl = (vel_x + vel_y - angular_vel * (L + W)) / wheel_radius;
    float rr = (vel_x - vel_y + angular_vel * (L + W)) / wheel_radius;

    return WheelSpeeds(fl, fr, rl, rr);
}

// 运动学正解：将轮子速度转换为机器人速度
void MecanumKinematics::forwardKinematics(const WheelSpeeds &wheel_speeds,
                                          float &vel_x, float &vel_y, float &angular_vel) const
{
    // 计算机器人几何参数
    float L = car_len / 2.0f; // 前后轮距的一半
    float W = car_wid / 2.0f; // 左右轮距的一半

    // 麦克纳姆轮运动学正解公式
    vel_x = wheel_radius * (wheel_speeds.front_left + wheel_speeds.front_right + wheel_speeds.rear_left + wheel_speeds.rear_right) / 4.0f;

    vel_y = wheel_radius * (-wheel_speeds.front_left + wheel_speeds.front_right + wheel_speeds.rear_left - wheel_speeds.rear_right) / 4.0f;

    angular_vel = wheel_radius * (-wheel_speeds.front_left + wheel_speeds.front_right - wheel_speeds.rear_left + wheel_speeds.rear_right) /
                  (4.0f * (L + W));
}

// 设置轮子半径
void MecanumKinematics::setWheelRadius(float radius)
{
    wheel_radius = radius;
}

// 设置车辆尺寸
void MecanumKinematics::setCarDimensions(float length, float width)
{
    car_len = length;
    car_wid = width;
}