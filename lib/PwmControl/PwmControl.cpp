#include "PwmControl.h"

void PWMControl::attachMotor(uint8_t id, gpio_num_t pwmIn, gpio_num_t gpioIn1, gpio_num_t gpioIn2)
{

    // 1. 初始化方向控制GPIO
    gpio_reset_pin(gpioIn1); // 重置引脚状态
    gpio_reset_pin(gpioIn2);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpioIn1) | (1ULL << gpioIn2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE, //  禁用上拉/下拉避免意外电平  确保引脚电平完全由代码控制
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE // 禁用引脚中断
    };
    gpio_config(&io_conf);
    // 置为低电平
    gpio_set_level(gpioIn1, HIGH);
    gpio_set_level(gpioIn2, HIGH);

    // 2. 根据ID计算MCPWM单元和定时器
    mcpwm_unit_t mcpwm_num = (id < 3) ? MCPWM_UNIT_0 : MCPWM_UNIT_1;
    mcpwm_timer_t mcpwm_timer = static_cast<mcpwm_timer_t>(id % 3);

    // 3. 根据定时器计算信号通道-->现在只用两个电机
    mcpwm_io_signals_t signal;
    switch (mcpwm_timer)
    {
    case MCPWM_TIMER_0:
        signal = MCPWM0A; // id=1 --> MCPWM_UNIT_0 | CPWM_TIMER_0
        break;
    case MCPWM_TIMER_1:
        signal = MCPWM1A; // id=2 --> MCPWM_UNIT_0 | CPWM_TIMER_1
        break;
    case MCPWM_TIMER_2:
        signal = MCPWM2A;
        break;
    default:
        signal = MCPWM0A; // 默认值
    }

    // 4. 配置PWM参数
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 20000;
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER; // 向上计数模式
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;   // 占空比高电平有效

    // 5. 初始化MCPWM
    mcpwm_init(mcpwm_num, mcpwm_timer, &pwm_config);

    // 6. 设置PWM引脚
    mcpwm_gpio_init(mcpwm_num, signal, pwmIn);

    // 7. 保存引脚配置（用于后续控制）
    motorConfigs[id].unit = mcpwm_num;
    motorConfigs[id].timer = mcpwm_timer;
    motorConfigs[id].in1 = gpioIn1;
    motorConfigs[id].in2 = gpioIn2;

    // 8. 标记电机已连接
    this->mMotorAttached[id] = true;
}

void PWMControl::updateMotorSpeed(uint8_t id, int16_t speed)
{

    const MotorConfig_t &cfg = motorConfigs[id];
    speed = constrain(speed, -1023, 1023);

    if (speed > 0)
    {
        // 正转
        gpio_set_level(cfg.in1, HIGH);
        gpio_set_level(cfg.in2, LOW);
        mcpwm_set_duty(cfg.unit, cfg.timer, MCPWM_OPR_A, speed);
        mcpwm_set_duty_type(cfg.unit, cfg.timer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    }
    else if (speed < 0)
    {
        // 反转
        gpio_set_level(cfg.in1, 0);
        gpio_set_level(cfg.in2, 1);
        mcpwm_set_duty(cfg.unit, cfg.timer, MCPWM_OPR_A, -speed);
        mcpwm_set_duty_type(cfg.unit, cfg.timer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    }
    else
    {
        // 停止
        gpio_set_level(cfg.in1, 0);
        gpio_set_level(cfg.in2, 0);
        mcpwm_set_duty(cfg.unit, cfg.timer, MCPWM_OPR_A, 0);
    }
}