#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include "Arduino.h"
#include <cmath>

// 运动学正逆解：已知当前速度 --> 求小车 w 和 v
// 已知目标的 w 和 v --> 两轮子输出速度

/*
    1. 需要确定设置的变量
    一同设置：
        轮子距离  wheel_distance
        上次更新的时间 --> last_update_time
        时间的差值  -->  dt = millis() - last_update_time
    两个轮子分别设置：
        每一次脉冲对应的轮子前进距离  per_pulse_distance
        编码器的差值  -->  dtick = encoders[0].getTicks - last_encoder_tick
        对应的每个轮子的速度  motor_speed = dtick * per_pulse_distance / dt * 1000 --> mm/s
    2.确定所需构造函数
        设置电动机参数
        设置轮子间距
        运动学正逆解
*/

// 创建电机参数结构体
typedef struct
{
    float per_pulse_distance;  // 每次脉冲所对应的距离
    float motor_speed;         // 轮子的速度，通过脉冲差值计算
    int32_t last_encoder_tick; // 上一次的脉冲总量， 用来算脉冲差值
} motor_param_t;

// 创建里程计结构体
typedef struct
{
    float x;
    float y;
    float angle;
    float linear_speed;
    float angle_speed;
} odom_t;

class Kinematics
{
public:
    Kinematics() = default;
    ~Kinematics() = default;

    // 设置结构体中电动机参数-->每一次脉冲对应的轮子前进距离
    void set_motor_param(uint8_t id, float per_pulse_distance);
    // 设置轮子间距
    void set_wheel_distance(float wheel_distance);
    // 更新电动机的速度
    void update_motor_speed(uint64_t current_time, int32_t front_left_tick, int32_t front_right_tick, int32_t rear_left_tick, int32_t rear_right_tick);
    // 获取电机速度
    float get_motor_speed(uint8_t id);
    // 获取最后一次编码器的数值
    int64_t get_last_ticks(uint8_t id);
    // 正运动学计算
    void kinematics_forward(float front_left_speed, float front_right_speed, float rear_left_speed, float rear_right_speed, float &out_linear_speed, float &out_angle_speed);
    // 逆运动学计算-->由线速度和角速度推出左右轮子motor_speed
    void kinematics_inverse(float linear_speed, float angle_speed, float &out_front_left_speed, float &out_front_right_speed, float &out_rear_left_speed, float &out_rear_right_speed);

    void update_odom(uint16_t dt);
    odom_t &get_odom();
    static void TransAngleInPI(float angle, float &out_angle);

private:
    motor_param_t motor_param_[4];
    uint64_t last_update_time = 0;
    float wheel_distance_;
    odom_t odom_;
};

#endif
