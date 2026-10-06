#include "OledDisplay.h"

#include <cstdarg>

OledDisplay::OledDisplay(uint8_t width, uint8_t height, TwoWire *wire, uint32_t i2cClockHz)
    : wire_(wire),
      display_(width, height, wire, -1, i2cClockHz, i2cClockHz),
      width_(width),
      height_(height)
{
}

bool OledDisplay::begin(uint8_t i2cAddress)
{
    // 先探测设备是否应答，没接屏幕时直接返回，不影响其他功能
    wire_->beginTransmission(i2cAddress);
    if (wire_->endTransmission() != 0)
    {
        connected_ = false;
        return false;
    }

    // periphBegin=false：总线已在外部初始化
    connected_ = display_.begin(SSD1306_SWITCHCAPVCC, i2cAddress, false, false);
    if (!connected_) return false;

    display_.setTextSize(1);
    display_.setTextWrap(false);
    display_.cp437(true);
    clear();
    show();
    return true;
}

void OledDisplay::clear()
{
    if (!connected_) return;
    display_.clearDisplay();
}

void OledDisplay::printLine(uint8_t row, bool inverted, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    printLineV(row, inverted, fmt, args);
    va_end(args);
}

void OledDisplay::printLine(uint8_t row, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    printLineV(row, false, fmt, args);
    va_end(args);
}

void OledDisplay::printLineV(uint8_t row, bool inverted, const char *fmt, va_list args)
{
    if (!connected_ || row >= height_ / kLineHeight) return;

    char buf[32];
    vsnprintf(buf, sizeof(buf), fmt, args);

    const int16_t y = row * kLineHeight;
    const uint16_t bg = inverted ? SSD1306_WHITE : SSD1306_BLACK;
    const uint16_t fg = inverted ? SSD1306_BLACK : SSD1306_WHITE;

    // 先清掉整行，避免上一帧较长的文字残留
    display_.fillRect(0, y, width_, kLineHeight, bg);
    display_.setTextColor(fg, bg);
    display_.setCursor(inverted ? 1 : 0, y);
    display_.print(buf);
}

void OledDisplay::show()
{
    if (!connected_) return;
    display_.display();
}
