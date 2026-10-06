#include "PcntQuadEncoder.h"

bool PcntQuadEncoder::begin(pcnt_unit_t unit, gpio_num_t pinA, gpio_num_t pinB, uint16_t glitchFilterCycles)
{
    unit_ = unit;
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);

    // 通道 0：A 相边沿计数，B 相电平决定方向；通道 1：B 相边沿计数，A 相电平决定方向
    // 计数方向与原 Esp32PcntEncoder 保持一致，kEncoderReversed 配置无需修改
    pcnt_config_t config = {};
    config.pulse_gpio_num = pinA;
    config.ctrl_gpio_num = pinB;
    config.lctrl_mode = PCNT_MODE_REVERSE;
    config.hctrl_mode = PCNT_MODE_KEEP;
    config.pos_mode = PCNT_COUNT_DEC;
    config.neg_mode = PCNT_COUNT_INC;
    config.counter_h_lim = kCounterLimit;
    config.counter_l_lim = -kCounterLimit;
    config.unit = unit;
    config.channel = PCNT_CHANNEL_0;
    if (pcnt_unit_config(&config) != ESP_OK) return false;

    config.pulse_gpio_num = pinB;
    config.ctrl_gpio_num = pinA;
    config.pos_mode = PCNT_COUNT_INC;
    config.neg_mode = PCNT_COUNT_DEC;
    config.channel = PCNT_CHANNEL_1;
    if (pcnt_unit_config(&config) != ESP_OK) return false;

    if (glitchFilterCycles > 0)
    {
        pcnt_set_filter_value(unit, glitchFilterCycles > 1023 ? 1023 : glitchFilterCycles);
        pcnt_filter_enable(unit);
    }
    else
    {
        pcnt_filter_disable(unit);
    }

    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);

    lastCount_ = 0;
    ticks_ = 0;
    attached_ = true;
    return true;
}

int32_t PcntQuadEncoder::getTicks()
{
    if (!attached_) return 0;

    int16_t count = 0;
    pcnt_get_counter_value(unit_, &count);

    // 计数到 ±32767 时硬件自动归零，计数范围相当于以 32767 为模
    int32_t delta = (int32_t)count - lastCount_;
    if (delta > kCounterLimit / 2) delta -= kCounterLimit;
    else if (delta < -kCounterLimit / 2) delta += kCounterLimit;

    lastCount_ = count;
    ticks_ += delta;
    return ticks_;
}
