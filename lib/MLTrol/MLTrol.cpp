#include "MLTrol.h"
#include <Arduino.h>
#include "BujinControl.h"
#include <cmath>

// 里程计结构体的构造函数实现
OdometryData::OdometryData()
    : pos_x(0), pos_y(0), orientation(0), linear_vel_x(0), linear_vel_y(0), angular_vel(0) {}

OdometryData::OdometryData(float x, float y, float orient, float lin_vel_x, float lin_vel_y, float ang_vel)
    : pos_x(x), pos_y(y), orientation(orient), linear_vel_x(lin_vel_x), linear_vel_y(lin_vel_y), angular_vel(ang_vel) {}

// WheelSpeeds 结构体的构造函数实现
WheelSpeeds::WheelSpeeds()
    : front_left(0), front_right(0), rear_left(0), rear_right(0) {}

WheelSpeeds::WheelSpeeds(float fl, float fr, float rl, float rr)
    : front_left(fl), front_right(fr), rear_left(rl), rear_right(rr) {}

// SharedSpeed 结构体的构造函数实现
SharedSpeed::SharedSpeed()
    : vel_fl(0), vel_fr(0), vel_rl(0), vel_rr(0) {}
SharedSpeed::SharedSpeed(float fl, float fr, float rl, float rr)
    : vel_fl(fl), vel_fr(fr), vel_rl(rl), vel_rr(rr) {}

// MecanumKinematics 类的构造函数实现
MecanumKinematics::MecanumKinematics(float length, float width, float wheel_diameter)
    : car_len(length), car_wid(width), wheel_radius(wheel_diameter / 2.0f)
{
    resetOdometry();
}

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
void MecanumKinematics::forwardKinematics(float fl, float fr, float rl, 
    float rr, float &vel_x, float &vel_y, float &angular_vel) const
{
    // 计算机器人几何参数
    float L = car_len / 2.0f; // 前后轮距的一半
    float W = car_wid / 2.0f; // 左右轮距的一半

    // 麦克纳姆轮运动学正解公式
    vel_x = wheel_radius * (fl + fr + rl + rr) / 4.0f;

    vel_y = wheel_radius * (-fl + fr + rl - rr) / 4.0f;

    angular_vel = wheel_radius * (-fl + fr - rl + rr) / (4.0f * (L + W));
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

// 电机初始化
void MecanumKinematics::MotorInit()
{
    // 初始化步进电机-->串口通信 17 18，同时创建互斥锁和速度队列
    Emm_V5_INIT();
    // 电机使能
    Emm_V5_En_Control(0, true, true);
    
    // 创建电机控制任务
    xTaskCreatePinnedToCore(
        MotorControlTask,    // 任务函数
        "MotorCtrlTask",     // 任务名称
        10240,               // 栈大小（字节）
        NULL,                // 任务参数
        2,                   // 任务优先级（高于里程计，低于micro-ROS）
        NULL,                // 任务句柄
        1                    // 运行在核心1，和micro-ROS任务错开
    );

    delay(100);
}

// 设置速度 左前1 右前2 左后3 右后4
void MecanumKinematics::setMotorSpeed(const WheelSpeeds &speeds)
{
    // 弧度/秒 转 RPM（保留绝对值，方向通过 dir 参数控制）
    auto radToRPM = [](float rad_speed)
    {
        return static_cast<uint16_t>(abs(rad_speed) * 9.549f); // 9.549 ≈ 60/(2π)
    };

    // uint8_t dir[4] = {0}; // 方向数组（0正转，1反转)
    // 此时的dir为全局变量，可供里程计读取函数进行方向的获取
    dir[0] = (speeds.front_left < 0) ? 1 : 0;
    dir[1] = (speeds.front_right < 0) ? 1 : 0;
    dir[2] = (speeds.rear_left < 0) ? 1 : 0;
    dir[3] = (speeds.rear_right < 0) ? 1 : 0;

    // 转换速度单位并设置电机
    uint16_t vel_fl = radToRPM(speeds.front_left);
    uint16_t vel_fr = radToRPM(speeds.front_right);
    uint16_t vel_rl = radToRPM(speeds.rear_left);
    uint16_t vel_rr = radToRPM(speeds.rear_right);

    // 使用异步设置速度
    Emm_5V_Vel_Set_Async(1, dir[0], vel_fl, 0, false);
    Emm_5V_Vel_Set_Async(2, dir[1], vel_fr, 0, false);
    Emm_5V_Vel_Set_Async(3, dir[2], vel_rl, 0, false);
    Emm_5V_Vel_Set_Async(4, dir[3], vel_rr, 0, false);   
    // Emm_5V_Vel_Set(1, dir[0], vel_fl, 0, true);
    // Emm_5V_Vel_Set(2, dir[1], vel_fr, 0, true); 
    // Emm_5V_Vel_Set(3, dir[2], vel_rl, 0, true); 
    // Emm_5V_Vel_Set(4, dir[3], vel_rr, 0, true); 
    // Emm_V5_Synchronous_motion(0);
    // Serial.print("duojitongbu");
    // // 同步执行所有电机
    // if(xSemaphoreTake(motor_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
    // {
    //     Emm_V5_Synchronous_motion(0);
    //     Serial.print("duojitongbu");
    //     xSemaphoreGive(motor_mutex);
    // }

    // Serial.print("轮子速度(RPM): ");
    // Serial.print(vel_fl); Serial.print(", ");
    // Serial.print(vel_fr); Serial.print(", ");
    // Serial.print(vel_rl); Serial.print(", ");
    // Serial.println(vel_rr);
}

void MecanumKinematics::updateOdometry(uint32_t dt_ms, float vel_fl_rpm, float vel_fr_rpm, float vel_rl_rpm, float vel_rr_rpm)
{
    // 将 ms 转化为 s
    float dt_seconds = dt_ms / 1000.0f;
    
    if (dt_seconds <= 0) return;
    
    // Serial.print("测试轮子速度(RPM): "); Serial.print(vel_fl_rpm); Serial.print(", ");
    // Serial.print(vel_fr_rpm); Serial.print(", ");
    // Serial.print(vel_rl_rpm); Serial.print(", "); Serial.println(vel_rr_rpm);

    // // 读取当前电机速度（RPM）并转换为弧度/秒
    // uint16_t vel_fl_rpm = Emm_V5_MotorVel_Get_RTOS(1);
    // uint16_t vel_fr_rpm = Emm_V5_MotorVel_Get_RTOS(2);    
    // uint16_t vel_rl_rpm = Emm_V5_MotorVel_Get_RTOS(3);
    // uint16_t vel_rr_rpm = Emm_V5_MotorVel_Get_RTOS(4);

    // 第一次检测速度可能会出现65535，需要过滤
    if(vel_fl_rpm == 65535) vel_fl_rpm = 0;
    if(vel_fr_rpm == 65535) vel_fr_rpm = 0;
    if(vel_rl_rpm == 65535) vel_rl_rpm = 0;
    if(vel_rr_rpm == 65535) vel_rr_rpm = 0;
    if ((abs(vel_fl_rpm) < 1000) && (abs(vel_fr_rpm) < 1000) && (abs(vel_rl_rpm) < 1000) && (abs(vel_rr_rpm) < 1000))
    {
        // 将RPM转换为弧度/秒
        auto rpmToRad = [](float rpm) -> float
        {
            return static_cast<float>(rpm) * (2.0f * M_PI / 60.0f);
        };
        // 转换为弧度/秒
        float vel_fl = rpmToRad(vel_fl_rpm);
        float vel_fr = rpmToRad(vel_fr_rpm);
        float vel_rl = rpmToRad(vel_rl_rpm);
        float vel_rr = rpmToRad(vel_rr_rpm);
        // 此时需要根据发送的速度时的数值获取方向()，0为正转，1为转
        vel_fl = (dir[0] == 0) ? vel_fl : -vel_fl;
        vel_fr = (dir[1] == 0) ? vel_fr : -vel_fr;
        vel_rl = (dir[2] == 0) ? vel_rl : -vel_rl;
        vel_rr = (dir[3] == 0) ? vel_rr : -vel_rr;

        // Serial.print("测试轮子速度(RPM): "); Serial.print(vel_fl); Serial.print(", ");
        // Serial.print(vel_fr); Serial.print(", ");
        // Serial.print(vel_rl); Serial.print(", "); Serial.println(vel_rr);

        // 使用运动学正解计算机器人速度
        float vel_x, vel_y, angular_vel;
        this->forwardKinematics(vel_fl, vel_fr, vel_rl, vel_rr,
                                vel_x, vel_y, angular_vel);
        
        // 记录旧的朝向角度
        float old_theta = odom_data.orientation;

        // 更新里程计速度
        odom_data.linear_vel_x = vel_x;
        odom_data.linear_vel_y = vel_y;
        odom_data.angular_vel = angular_vel;

        // Serial.print("机器人速度(m/s, rad/s): ");
        // Serial.print(vel_x); Serial.print(", ");
        // Serial.print(vel_y); Serial.print(", ");
        // Serial.println(angular_vel);

        // 计算上一时刻到当前时刻的角度变化
        float delta_theta = angular_vel * dt_seconds;
        odom_data.orientation += delta_theta;
        // 保持角度在 -π 到 π 之间
        if (odom_data.orientation > M_PI) odom_data.orientation -= 2.0f * M_PI;
        else if (odom_data.orientation < -M_PI) odom_data.orientation += 2.0f * M_PI;
        
        // 使用中值角度更新位置 (x, y)
        // 中值角度即：(旧角度 + 新角度) / 2
        float mid_theta = old_theta + delta_theta / 2.0f;

        // 麦轮在全局坐标系下的位移增量计算
        odom_data.pos_x += (vel_x * cos(mid_theta) - vel_y * sin(mid_theta)) * dt_seconds;
        odom_data.pos_y += (vel_x * sin(mid_theta) + vel_y * cos(mid_theta)) * dt_seconds;

        // // 如果角速度很小，使用直线运动近似
        // if (fabs(delta_theta) < 1e-3) {
        //     // 直线运动，直接将速度积分到位置
        //     odom_data.pos_x += vel_x * dt_seconds;
        //     odom_data.pos_y += vel_y * dt_seconds;
        // } else {
        //     // 圆弧运动，使用更精确的积分
        //     // 计算圆弧半径和圆心
        //     float v_linear = sqrt(vel_x * vel_x + vel_y * vel_y);
        //     float radius = (fabs(angular_vel) > 1e-6) ? v_linear / fabs(angular_vel) : 0;
            
        //     // 计算机器人坐标系下的位移在全局坐标系的投影
        //     float avg_theta = odom_data.orientation - delta_theta / 2.0f;
        //     float cos_theta = cos(avg_theta);
        //     float sin_theta = sin(avg_theta);
            
        //     // 更新位置
        //     odom_data.pos_x += (vel_x * cos_theta - vel_y * sin_theta) * dt_seconds;
        //     odom_data.pos_y += (vel_x * sin_theta + vel_y * cos_theta) * dt_seconds;
        // }
    }
}

// 重置里程计
void MecanumKinematics::resetOdometry()
{
    odom_data.pos_x = 0.0f;
    odom_data.pos_y = 0.0f;
    odom_data.orientation = 0.0f;
    odom_data.linear_vel_x = 0.0f;
    odom_data.linear_vel_y = 0.0f;
    odom_data.angular_vel = 0.0f;
    // Serial.print("里程计已重置");
    // Serial.print(odom_data.pos_x);
    // Serial.print(", ");
    // Serial.print(odom_data.pos_y);
    // Serial.print(", ");
    // Serial.println(odom_data.orientation);

}

// 获取里程计数据
const OdometryData& MecanumKinematics::getOdometryData() const
{
    return odom_data;       
}
