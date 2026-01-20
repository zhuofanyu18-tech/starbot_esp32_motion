#include "PidController.h"
#include "Arduino.h"

PidController::PidController(float kp, float ki, float kd)
{
    reset();
    update_pid(kp, ki, kd);
}

float PidController::update(float current_speed)
{
    // error-->当前误差
    float error = target_ - current_speed;
    derror_ = error_last_ - error;
    error_last_ = error;
    error_sum_ += error;
    // intergral_up_积分上限，避免error_sum_过大
    if (error_sum_ > intergral_up_)
    {
        error_sum_ = intergral_up_;
    }
    if (error_sum_ < -1 * intergral_up_)
    {
        error_sum_ = -1 * intergral_up_;
    }
    // 获得改进后的PWM占比-->output
    float output = kp_ * error + ki_ * error_sum_ + kd_ * derror_;
    if (output > out_max_)
        output = out_max_;
    if (output < out_min_)
        output = out_min_;
        
    return output;
}

void PidController::update_pid(float kp, float ki, float kd)
{
    reset();
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PidController::update_target(float target)
{
    target_ = target;
}

void PidController::out_limit(float out_min, float out_max)
{
    out_min_ = out_min;
    out_max_ = out_max;
}

void PidController::reset(void)
{
    float target_ = 0.0f;  // 目标的PWM占空比
    float out_min_ = 0.0f; // 输出下限
    float out_max_ = 0.0f; // 输出上限
    float kp = 0.0f;       // 比例系数
    float ki = 0.0f;       // 积分系数
    float kd = 0.0f;       // 微分系数
    float error_sum_ = 0;  // 累积误差和，积分所需数据
    float derror = 0;      // 误差变化率，微分所需数据
    float error_pre_ = 0;  // 上上次误差
    float error_last_ = 0; // 上一次误差
}