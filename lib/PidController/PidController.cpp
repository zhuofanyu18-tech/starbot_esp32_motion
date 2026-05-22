#include "PidController.h"
#include "Arduino.h"

PidController::PidController(float kp, float ki, float kd)
{
    reset();
    update_pid(kp, ki, kd);
}

float PidController::update(float current_speed)
{
    float error = target_ - current_speed;
    derror_ = error_last_ - error;
    error_last_ = error;
    error_sum_ += error;
    if (error_sum_ > intergral_up_)  error_sum_ = intergral_up_;
    if (error_sum_ < -intergral_up_) error_sum_ = -intergral_up_;
    float output = kp_ * error + ki_ * error_sum_ + kd_ * derror_;
    if (output > out_max_) output = out_max_;
    if (output < out_min_) output = out_min_;
    return output;
}

void PidController::update_pid(float kp, float ki, float kd)
{
    reset();
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PidController::update_target(float target) { target_ = target; }
float PidController::get_target() { return target_; }
void PidController::out_limit(float out_min, float out_max) { out_min_ = out_min; out_max_ = out_max; }

void PidController::reset()
{
    target_ = 0.0f;
    error_sum_ = 0;
    derror_ = 0;
    error_pre_ = 0;
    error_last_ = 0;
}
