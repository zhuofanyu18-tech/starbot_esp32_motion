#include "MLTrol.h"
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

// MecanumKinematics 类的构造函数实现
MecanumKinematics::MecanumKinematics(float length, float width, float wheel_diameter)
    : car_len(length), car_wid(width), wheel_radius(wheel_diameter / 2.0f), last_update_time(0) 
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
    // 初始化步进电机-->串口通信 17 18
    Emm_V5_INIT();
    Emm_V5_En_Control(0, true, true);
    // Emm_V5_En_Control(1, true, true); // 步进电机的使能
    // delay(10);
    // Emm_V5_En_Control(2, true, true); // 步进电机的使能
    // delay(10);
    // Emm_V5_En_Control(3, true, true); // 步进电机的使能
    // delay(10);
    // Emm_V5_En_Control(4, true, true); // 步进电机的使能
    // delay(10);
}

// 设置速度 左前1 右前2 左后3 右后4
void MecanumKinematics::setMotorSpeed(const WheelSpeeds &speeds)
{
    // 弧度/秒 转 RPM（保留绝对值，方向通过 dir 参数控制）
    auto radToRPM = [](float rad_speed)
    {
        return static_cast<uint16_t>(abs(rad_speed) * 9.549f); // 9.549 ≈ 60/(2π)
    };

    uint8_t dir[4] = {0}; // 方向数组（0正转，1反转）
    dir[0] = (speeds.front_left < 0) ? 1 : 0;
    dir[1] = (speeds.front_right < 0) ? 1 : 0;
    dir[2] = (speeds.rear_left < 0) ? 1 : 0;
    dir[3] = (speeds.rear_right < 0) ? 1 : 0;

    // 转换速度单位并设置电机
    uint16_t vel_fl = radToRPM(speeds.front_left);
    uint16_t vel_fr = radToRPM(speeds.front_right);
    uint16_t vel_rl = radToRPM(speeds.rear_left);
    uint16_t vel_rr = radToRPM(speeds.rear_right);
    // Emm_V5_Vel_Control(1, dir[0], vel_fl, 0, true);
    // Emm_V5_Vel_Control(2, dir[1], vel_fr, 0, true);
    // Emm_V5_Vel_Control(3, dir[2], vel_rl, 0, true);
    // Emm_V5_Vel_Control(4, dir[3], vel_rr, 0, true);
    Emm_5V_Vel_Set(1, dir[0], vel_fl, 0, true);
    delay(5);
    Emm_5V_Vel_Set(2, dir[1], vel_fr, 0, true);
    delay(5);
    Emm_5V_Vel_Set(3, dir[2], vel_rl, 0, true);
    delay(5);
    Emm_5V_Vel_Set(4, dir[3], vel_rr, 0, true);
    Emm_V5_Synchronous_motion(0);
    // Emm_V5_Synchronous_motion(2);
    // Emm_V5_Synchronous_motion(3);
    // Emm_V5_Synchronous_motion(4);
    Serial.print("轮子速度(rad/s): ");
    Serial.print("FL=");
    Serial.print(vel_fl);
    Serial.print(", FR=");
    Serial.print(vel_fr);
    Serial.print(", RL=");
    Serial.print(vel_rl);
    Serial.print(", RR=");
    Serial.println(vel_rr);
}

void MecanumKinematics::updateOdometry(uint32_t dt_ms)
{
    float dt_seconds = dt_ms / 1000.0f;
    // float dt_seconds = 0.01f; // 固定时间步长10ms
    if (dt_seconds <= 0)
        return;
    // 读取当前电机速度（RPM）并转换为弧度/秒
    uint16_t vel_fr_rpm = Emm_V5_MotorVel_Get(2);    
    uint16_t vel_rl_rpm = Emm_V5_MotorVel_Get(3);
    uint16_t vel_rr_rpm = Emm_V5_MotorVel_Get(4);
    uint16_t vel_fl_rpm = Emm_V5_MotorVel_Get(1);
    if(vel_fl_rpm == 65535) vel_fl_rpm = 0;
    if(vel_fr_rpm == 65535) vel_fr_rpm = 0;
    if(vel_rl_rpm == 65535) vel_rl_rpm = 0;
    if(vel_rr_rpm == 65535) vel_rr_rpm = 0;
    Serial.print("测试轮子速度(RPM): ");
    Serial.print(vel_fl_rpm);
    Serial.print(", ");
    Serial.print(vel_fr_rpm);
    Serial.print(", ");
    Serial.print(vel_rl_rpm);
    Serial.print(", ");
    Serial.println(vel_rr_rpm);
    if ((abs(vel_fl_rpm) < 1000) && (abs(vel_fr_rpm) < 1000) && (abs(vel_rl_rpm) < 1000) && (abs(vel_rr_rpm) < 1000))
    {
        // 将RPM转换为弧度/秒
        auto rpmToRad = [](uint16_t rpm) -> float
        {
            return static_cast<float>(rpm) * (2.0f * M_PI / 60.0f);
        };
        // 转换为弧度/秒
        float vel_fl = rpmToRad(vel_fl_rpm);
        float vel_fr = rpmToRad(vel_fr_rpm);
        float vel_rl = rpmToRad(vel_rl_rpm);
        float vel_rr = rpmToRad(vel_rr_rpm);
        // 使用运动学正解计算机器人速度
        float vel_x, vel_y, angular_vel;
        this->forwardKinematics(vel_fl, vel_fr, vel_rl, vel_rr,
                                vel_x, vel_y, angular_vel);
        // 更新里程计速度
        odom_data.linear_vel_x = vel_x;
        odom_data.linear_vel_y = vel_y;
        odom_data.angular_vel = angular_vel;

        // 计算上一时刻到当前时刻的角度变化
        float delta_theta = angular_vel * dt_seconds;
        odom_data.orientation += delta_theta;
        // 保持角度在 -π 到 π 之间
        if (odom_data.orientation > M_PI)
            odom_data.orientation -= 2.0f * M_PI;
        else if (odom_data.orientation < -M_PI)
            odom_data.orientation += 2.0f * M_PI;
        
        // 如果角速度很小，使用直线运动近似
        if (fabs(delta_theta) < 1e-3) {
            // 直线运动，直接将速度积分到位置
            odom_data.pos_x += vel_x * dt_seconds;
            odom_data.pos_y += vel_y * dt_seconds;
        } else {
            // 圆弧运动，使用更精确的积分
            // 计算圆弧半径和圆心
            float v_linear = sqrt(vel_x * vel_x + vel_y * vel_y);
            float radius = (fabs(angular_vel) > 1e-6) ? v_linear / fabs(angular_vel) : 0;
            
            // 计算机器人坐标系下的位移在全局坐标系的投影
            float avg_theta = odom_data.orientation - delta_theta / 2.0f;
            float cos_theta = cos(avg_theta);
            float sin_theta = sin(avg_theta);
            
            // 更新位置
            odom_data.pos_x += (vel_x * cos_theta - vel_y * sin_theta) * dt_seconds;
            odom_data.pos_y += (vel_x * sin_theta + vel_y * cos_theta) * dt_seconds;
        }
        // count++;
        // Serial.print("更新次数: ");
        // Serial.println(count);
    }
    // // 调试输出
    // Serial.print("Odometry - X: ");
    // Serial.print(odom_data.pos_x, 4);
    // Serial.print(" m, Y: ");
    // Serial.print(odom_data.pos_y, 4);
    // Serial.print(" m, Theta: ");
    // Serial.print(odom_data.orientation * 180.0f / M_PI, 2);
    // Serial.print("°, Vx: ");
    // Serial.print(odom_data.linear_vel_x, 4);
    // Serial.print(" m/s, Vy: ");
    // Serial.print(odom_data.linear_vel_y, 4);
    // Serial.print(" m/s, W: ");
    // Serial.print(odom_data.angular_vel, 4);
    // Serial.println(" rad/s");
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
    last_update_time = millis();
    Serial.print("里程计已重置");
    Serial.print(odom_data.pos_x);
    Serial.print(", ");
    Serial.print(odom_data.pos_y);
    Serial.print(", ");
    Serial.println(odom_data.orientation);

}

// 获取里程计数据
const OdometryData& MecanumKinematics::getOdometryData() const
{
    return odom_data;       
}
