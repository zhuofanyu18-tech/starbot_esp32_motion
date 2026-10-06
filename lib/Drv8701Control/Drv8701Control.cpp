#include "Drv8701Control.h"

void Drv8701Control::attachMotor(uint8_t id, gpio_num_t pwmPin, gpio_num_t dirPin, bool reversed, uint32_t frequencyHz)
{
    if (id >= kMaxMotors) return;

    gpio_reset_pin(dirPin);
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << dirPin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(dirPin, LOW);

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

    // 先绑定引脚再初始化，上电时 EN 保持低电平（刹车），避免电机抖动
    mcpwm_gpio_init(mcpwm_num, signal, pwmPin);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = frequencyHz;
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(mcpwm_num, mcpwm_timer, &pwm_config);

    motorConfigs[id].unit = mcpwm_num;
    motorConfigs[id].timer = mcpwm_timer;
    motorConfigs[id].dir = dirPin;
    motorConfigs[id].reversed = reversed;
    mMotorAttached[id] = true;
}

void Drv8701Control::updateMotorSpeed(uint8_t id, int16_t speed)
{
    if (id >= kMaxMotors || !mMotorAttached[id]) return;

    const Drv8701Config_t &cfg = motorConfigs[id];
    speed = constrain(speed, -kMaxSpeed, kMaxSpeed);
    float duty = (abs(speed) / (float)kMaxSpeed) * 100.0f;
    currentDuty[id] = duty;

    if (speed != 0)
    {
        bool forward = (speed > 0) != cfg.reversed;
        gpio_set_level(cfg.dir, forward ? HIGH : LOW);
    }
    mcpwm_set_duty(cfg.unit, cfg.timer, MCPWM_OPR_A, duty);
    mcpwm_set_duty_type(cfg.unit, cfg.timer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
}

float Drv8701Control::getCurrentDuty(uint8_t id)
{
    if (id >= kMaxMotors || !mMotorAttached[id]) return 0.0f;
    return currentDuty[id];
}
