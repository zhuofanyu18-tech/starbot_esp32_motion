#ifndef RGB_LED_APP_H
#define RGB_LED_APP_H

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <RgbLed.h>

#include "config/AppConfig.h"

struct RgbLedState {
    bool    on;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t brightness;  // 0~100 %
};

/*
    RGB 灯 App：网络回调只更新目标状态并放入队列（不直接操作硬件），
    由独立的工作任务负责真正写灯，避免阻塞 WiFi / HTTP 任务。
*/
class RgbLedApp {
public:
    RgbLedApp();

    // kEnableRgbLed=false 时返回 false，之后的请求都会被忽略
    bool begin();
    bool isEnabled() const { return queue_ != nullptr; }

    // 以下接口均为非阻塞，可以在任意任务中调用
    void setOn(bool on);
    void setColor(uint8_t r, uint8_t g, uint8_t b);  // 设置颜色的同时自动开灯
    void setBrightness(uint8_t percent);
    RgbLedState getState() const;

private:
    RgbLed        led_;
    QueueHandle_t queue_ = nullptr;
    mutable portMUX_TYPE lock_;
    RgbLedState   state_ = {false, 255, 255, 255, 50};

    void submit();

    static void workerTaskFn(void *args);
};

#endif
