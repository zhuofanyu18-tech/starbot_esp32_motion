#ifndef DRV8701_CONTROL_H
#define DRV8701_CONTROL_H

#include <driver/mcpwm.h>
#include <driver/gpio.h>
#include <Arduino.h>

/*
    DRV8701E 电机驱动（PH/EN 控制模式）
    每个电机只需要两根线：
        EN(PWM) --> 占空比决定转速，EN=0 时驱动器刹车（低边慢衰减）
        PH(DIR) --> 高/低电平决定转向
    最多支持 6 路：id 0~2 使用 MCPWM_UNIT_0，id 3~5 使用 MCPWM_UNIT_1
*/

typedef struct
{
    mcpwm_unit_t unit;
    mcpwm_timer_t timer;
    gpio_num_t dir;
    bool reversed;  // 安装方向相反的电机，翻转方向电平
} Drv8701Config_t;

class Drv8701Control
{
public:
    static constexpr uint8_t kMaxMotors = 6;
    static constexpr int16_t kMaxSpeed = 1023;

    Drv8701Control() = default;
    ~Drv8701Control() = default;

    void attachMotor(uint8_t id, gpio_num_t pwmPin, gpio_num_t dirPin, bool reversed = false, uint32_t frequencyHz = 20000);
    // speed 范围 -1023 ~ 1023，正负表示方向，0 表示刹车
    void updateMotorSpeed(uint8_t id, int16_t speed);
    float getCurrentDuty(uint8_t id);

private:
    Drv8701Config_t motorConfigs[kMaxMotors];
    bool mMotorAttached[kMaxMotors] = {false};
    float currentDuty[kMaxMotors] = {0.0f};
};

#endif
