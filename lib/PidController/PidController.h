#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

class PidController // 定义一个PID控制类
{
public:
    PidController() = default; // 默认构造函数
    PidController(float kp, float ki, float kd);

private:
    float target_;  // 目标的PWM占空比
    float out_min_; // 输出下限 PWM的最大值为1000
    float out_max_; // 输出上限 PWM的最小值为-1000
    float kp_;      // 比例系数
    float ki_;      // 积分系数
    float kd_;      // 微分系数
    // pid
    float error_sum_;           // 累积误差和，积分所需数据
    float derror_;              // 误差变化率，微分所需数据
    float error_pre_;           // 上上次误差
    float error_last_;          // 上一次误差
    float intergral_up_ = 8000; // 积分上限

public:
    float update(float current_speed);             // current-->当前速度  返回改进后的PWM占比-->output
    void update_target(float target);              // 设置目标速度
    void update_pid(float kp, float ki, float kd); // 更新PID系数
    void reset();                                  // 重置PID参数
    void out_limit(float out_mix, float out_max);  // 设置输出限制，PWM所输入的范围为(-1000, 1000)
};

#endif