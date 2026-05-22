#include "Kinematics.h"

// 设置电机的参数
void Kinematics::set_motor_param(uint8_t id, float per_pulse_distance)
{
    motor_param_[id].per_pulse_distance = per_pulse_distance;
}

// 设置轮胎的距离
void Kinematics::set_wheel_distance(float wheel_distance)
{
    wheel_distance_ = wheel_distance;
}

// 更新电机速度，编码器数据
// 参数顺序：front_left_tick, front_right_tick, rear_left_tick, rear_right_tick
void Kinematics::update_motor_speed(uint64_t current_time, int32_t front_left_tick, int32_t front_right_tick, int32_t rear_left_tick, int32_t rear_right_tick)
{
    uint32_t dt = (uint32_t)(current_time - last_update_time);
    last_update_time = current_time;

    int32_t dtick0 = front_left_tick - motor_param_[0].last_encoder_tick;
    int32_t dtick1 = front_right_tick - motor_param_[1].last_encoder_tick;
    int32_t dtick2 = rear_left_tick - motor_param_[2].last_encoder_tick;
    int32_t dtick3 = rear_right_tick - motor_param_[3].last_encoder_tick;

    motor_param_[0].last_encoder_tick = front_left_tick;
    motor_param_[1].last_encoder_tick = front_right_tick;
    motor_param_[2].last_encoder_tick = rear_left_tick;
    motor_param_[3].last_encoder_tick = rear_right_tick;

    if (dt == 0) return;

    motor_param_[0].motor_speed = float(dtick0 * motor_param_[0].per_pulse_distance) / dt * 1000;
    motor_param_[1].motor_speed = float(dtick1 * motor_param_[1].per_pulse_distance) / dt * 1000;
    motor_param_[2].motor_speed = float(dtick2 * motor_param_[2].per_pulse_distance) / dt * 1000;
    motor_param_[3].motor_speed = float(dtick3 * motor_param_[3].per_pulse_distance) / dt * 1000;

    update_odom(dt);
}

// 获取电机速度
float Kinematics::get_motor_speed(uint8_t id)
{
    return motor_param_[id].motor_speed;
}

// 获取最后一次编码器的值
int64_t Kinematics::get_last_ticks(uint8_t id)
{
    return motor_param_[id].last_encoder_tick;
}

// 运动学逆解
void Kinematics::kinematics_inverse(float linear_speed, float angle_speed, float &out_front_left_speed, float &out_front_right_speed, float &out_rear_left_speed, float &out_rear_right_speed)
{
    // 四个轮子的速度计算，左右轮速度不同，前后轮速度相同
    // 单位：linear_speed (mm/s), angle_speed (rad/s), wheel_distance_ (mm)
    out_front_left_speed = linear_speed - (angle_speed * wheel_distance_) / 2.0;
    out_front_right_speed = linear_speed + (angle_speed * wheel_distance_) / 2.0;
    out_rear_left_speed = linear_speed - (angle_speed * wheel_distance_) / 2.0;
    out_rear_right_speed = linear_speed + (angle_speed * wheel_distance_) / 2.0;
}

// 运动学正解
void Kinematics::kinematics_forward(float front_left_speed, float front_right_speed, float rear_left_speed, float rear_right_speed, float &out_linear_speed, float &out_angle_speed)
{
    // 计算平均左右轮速度
    float left_speed = (front_left_speed + rear_left_speed) / 2.0;
    float right_speed = (front_right_speed + rear_right_speed) / 2.0;

    out_linear_speed = (right_speed + left_speed) / 2.0;
    // 调整角速度符号，使其与预期方向一致
    out_angle_speed = (right_speed - left_speed) / wheel_distance_;
}

// 获得里程计数据
odom_t &Kinematics::get_odom()
{
    return odom_;
}

void Kinematics::TransAngleInPI(float angle, float &out_angle)
{
    out_angle = angle;
    if(out_angle > PI)
    {
        out_angle -= 2 * PI;
    }
    else if(out_angle < -PI)
    {
        out_angle += 2 * PI;
    }
}

// 更新里程计数据
void Kinematics::update_odom(uint16_t dt)
{
    float dt_s = float(dt) / 1000;

    this->kinematics_forward(
        motor_param_[0].motor_speed, // front left
        motor_param_[1].motor_speed, // front right
        motor_param_[2].motor_speed, // rear left
        motor_param_[3].motor_speed, // rear right
        odom_.linear_speed,
        odom_.angle_speed
    );
    odom_.linear_speed = odom_.linear_speed / 1000; // 转换为 m/s

    odom_.angle += odom_.angle_speed * dt_s;
    Kinematics::TransAngleInPI(odom_.angle, odom_.angle);

    float delta_distance = odom_.linear_speed * dt_s;
    odom_.x += delta_distance * std::cos(odom_.angle);
    odom_.y += delta_distance * std::sin(odom_.angle);
}
