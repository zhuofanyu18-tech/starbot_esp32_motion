#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/*
    SSD1306 OLED（4 针 I2C，128x64）模块封装
    - I2C 总线由外部统一初始化（Wire.begin），本模块不再修改引脚，便于与 IMU 共用总线
    - 以 6x8 字体按“行”显示文本：128x64 共 8 行，每行 21 个字符
    - 所有绘制先写入显存，调用 show() 后才真正刷新到屏幕
*/

class OledDisplay
{
public:
    static constexpr uint8_t kCharWidth = 6;
    static constexpr uint8_t kLineHeight = 8;

    // i2cClockHz：刷新前后都保持该频率，避免把共用总线上 IMU 的速率降到 100kHz
    OledDisplay(uint8_t width = 128, uint8_t height = 64, TwoWire *wire = &Wire, uint32_t i2cClockHz = 400000);

    // 探测并初始化屏幕，地址一般为 0x3C（部分模块为 0x3D）
    bool begin(uint8_t i2cAddress = 0x3C);
    bool isConnected() const { return connected_; }

    void clear();
    // 在第 row 行（0 ~ 7）显示格式化文本，inverted=true 时反色显示（用作标题栏）
    void printLine(uint8_t row, bool inverted, const char *fmt, ...);
    void printLine(uint8_t row, const char *fmt, ...);
    void show();

    // 需要画图形等高级功能时直接使用底层驱动
    Adafruit_SSD1306 &raw() { return display_; }

private:
    TwoWire *wire_;
    Adafruit_SSD1306 display_;
    uint8_t width_;
    uint8_t height_;
    bool connected_ = false;

    void printLineV(uint8_t row, bool inverted, const char *fmt, va_list args);
};

#endif
