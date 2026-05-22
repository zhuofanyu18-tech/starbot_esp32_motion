#ifndef PWM_CONTROL_H
#define PWM_CONTROL_H

#include <driver/mcpwm.h>
#include <driver/gpio.h>
#include <Arduino.h>

// 电机配置结构体
typedef struct
{
    mcpwm_unit_t unit;
    mcpwm_timer_t timer;
    gpio_num_t in1;
    gpio_num_t in2;
} MotorConfig_t;

class PWMControl
{
public:
    PWMControl() = default;
    ~PWMControl() = default;
    float getCurrentDuty(uint8_t id);

    void attachMotor(uint8_t id, gpio_num_t pwmIn, gpio_num_t gpioIn1, gpio_num_t gpioIn2);
    void updateMotorSpeed(uint8_t id, int16_t speed);

private:
    MotorConfig_t motorConfigs[6];
    bool mMotorAttached[6] = {false};
    float currentDuty[6] = {0.0f};
};

#endif
