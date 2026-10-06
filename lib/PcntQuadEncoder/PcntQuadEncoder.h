#ifndef PCNT_QUAD_ENCODER_H
#define PCNT_QUAD_ENCODER_H

#include <Arduino.h>
#include <driver/pcnt.h>

/*
    基于 PCNT 硬件的正交编码器（4 倍频计数，A/B 相双边沿）

    与 Esp32PcntEncoder 的区别：
    - 不使用溢出中断。原库每 ±100 个脉冲进一次中断，且 getTicks() 读“硬件计数 + 中断累加值”
      两步之间没有保护，溢出恰好发生在中间时会读到 ±100 个脉冲的跳变。
    - 本类把硬件计数器上下限设为 ±32767，由调用者周期性读取，按增量在软件中累加成 int32。
      只要两次读取之间计数变化小于 16383（10ms 周期下相当于 160 万脉冲/秒），结果就完全准确。

    注意：getTicks() 不是线程安全的，只能在同一个任务中周期性调用（底盘控制任务）。
*/
class PcntQuadEncoder
{
public:
    PcntQuadEncoder() = default;

    // glitchFilterCycles：毛刺滤波，单位 APB 时钟周期（80MHz），最大 1023；0 表示关闭
    bool begin(pcnt_unit_t unit, gpio_num_t pinA, gpio_num_t pinB, uint16_t glitchFilterCycles = 1000);
    bool isAttached() const { return attached_; }

    // 返回累计脉冲数（上电后清零），需要至少每 ~100ms 调用一次
    int32_t getTicks();

private:
    static constexpr int16_t kCounterLimit = 32767;

    pcnt_unit_t unit_ = PCNT_UNIT_0;
    bool attached_ = false;
    int16_t lastCount_ = 0;
    int32_t ticks_ = 0;
};

#endif
