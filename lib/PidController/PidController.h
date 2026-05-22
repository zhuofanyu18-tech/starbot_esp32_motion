#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

class PidController
{
public:
    PidController() = default;
    PidController(float kp, float ki, float kd);

private:
    float target_;
    float out_min_;
    float out_max_;
    float kp_;
    float ki_;
    float kd_;
    float error_sum_;
    float derror_;
    float error_pre_;
    float error_last_;
    float intergral_up_ = 8000;

public:
    float update(float current_speed);
    void update_target(float target);
    float get_target();
    void update_pid(float kp, float ki, float kd);
    void reset();
    void out_limit(float out_min, float out_max);
};

#endif
