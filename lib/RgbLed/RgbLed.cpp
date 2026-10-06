#include "RgbLed.h"

void RgbLed::begin(uint8_t pin, uint8_t maxBrightness)
{
    pin_ = pin;
    maxBrightness_ = maxBrightness;
    initialized_ = true;
    writeRaw(0, 0, 0);
}

void RgbLed::writeRaw(uint8_t r, uint8_t g, uint8_t b)
{
    if (!initialized_) return;
    // Arduino 内置 WS2812 驱动（RMT 外设），内部已处理 GRB 字节顺序
    neopixelWrite(pin_, r, g, b);
}

void RgbLed::write(bool on, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
    if (!on || brightness == 0)
    {
        writeRaw(0, 0, 0);
        return;
    }

    brightness = min<uint8_t>(brightness, 100);
    const uint32_t scale = (uint32_t)maxBrightness_ * brightness;  // 0 ~ 25500
    auto apply = [scale](uint8_t c) -> uint8_t {
        return (uint8_t)(((uint32_t)c * scale + 12750) / 25500);  // 四舍五入
    };
    writeRaw(apply(r), apply(g), apply(b));
}
