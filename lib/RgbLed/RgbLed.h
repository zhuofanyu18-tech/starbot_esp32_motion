#ifndef RGB_LED_H
#define RGB_LED_H

#include <Arduino.h>

/*
    单颗 WS2812 RGB 灯封装（板载灯或外接灯均可）
    - 颜色 r/g/b：0~255
    - 亮度 brightness：0~100（%），实际输出 = 颜色 × 亮度 × maxBrightness / 255 / 100
    - maxBrightness 用来限制最大电流，板载灯全亮非常刺眼
*/

class RgbLed
{
public:
    RgbLed() = default;

    void begin(uint8_t pin, uint8_t maxBrightness = 128);
    // 直接输出，不做亮度换算
    void writeRaw(uint8_t r, uint8_t g, uint8_t b);
    // 按亮度换算后输出；on=false 时熄灭
    void write(bool on, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);

private:
    uint8_t pin_ = 0;
    uint8_t maxBrightness_ = 128;
    bool initialized_ = false;
};

#endif
