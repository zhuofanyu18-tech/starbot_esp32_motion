#include "PwmControl.h"

void PWMControl::attachMotor(uint8_t id, gpio_num_t pwmIn, gpio_num_t gpioIn1, gpio_num_t gpioIn2)
{
    gpio_reset_pin(gpioIn1);
    gpio_reset_pin(gpioIn2);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpioIn1) | (1ULL << gpioIn2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(gpioIn1, HIGH);
    gpio_set_level(gpioIn2, HIGH);

    mcpwm_unit_t mcpwm_num = (id < 3) ? MCPWM_UNIT_0 : MCPWM_UNIT_1;
    mcpwm_timer_t mcpwm_timer = static_cast<mcpwm_timer_t>(id % 3);

    mcpwm_io_signals_t signal;
    switch (mcpwm_timer)
    {
    case MCPWM_TIMER_0: signal = MCPWM0A; break;
    case MCPWM_TIMER_1: signal = MCPWM1A; break;
    case MCPWM_TIMER_2: signal = MCPWM2A; break;
    default:            signal = MCPWM0A;
    }

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 20000;
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(mcpwm_num, mcpwm_timer, &pwm_config);
    mcpwm_gpio_init(mcpwm_num, signal, pwmIn);

    motorConfigs[id].unit = mcpwm_num;
    motorConfigs[id].timer = mcpwm_timer;
    motorConfigs[id].in1 = gpioIn1;
    motorConfigs[id].in2 = gpioIn2;
    mMotorAttached[id] = true;
}

void PWMControl::updateMotorSpeed(uint8_t id, int16_t speed)
{
    const MotorConfig_t &cfg = motorConfigs[id];
    speed = constrain(speed, -1023, 1023);
    float duty = (abs(speed) / 1023.0f) * 100.0f;
    currentDuty[id] = duty;
    if (speed > 0)
    {
        gpio_set_level(cfg.in1, HIGH);
        gpio_set_level(cfg.in2, LOW);
        mcpwm_set_duty(cfg.unit, cfg.timer, MCPWM_OPR_A, duty);
        mcpwm_set_duty_type(cfg.unit, cfg.timer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    }
    else if (speed < 0)
    {
        gpio_set_level(cfg.in1, LOW);
        gpio_set_level(cfg.in2, HIGH);
        mcpwm_set_duty(cfg.unit, cfg.timer, MCPWM_OPR_A, duty);
        mcpwm_set_duty_type(cfg.unit, cfg.timer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    }
    else
    {
        gpio_set_level(cfg.in1, LOW);
        gpio_set_level(cfg.in2, LOW);
        mcpwm_set_duty(cfg.unit, cfg.timer, MCPWM_OPR_A, 0);
    }
}

float PWMControl::getCurrentDuty(uint8_t id)
{
    if (id >= 6 || !mMotorAttached[id]) return 0.0f;
    return currentDuty[id];
}
