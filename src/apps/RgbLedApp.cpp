#include "apps/RgbLedApp.h"

RgbLedApp::RgbLedApp() {
    portMUX_INITIALIZE(&lock_);
}

bool RgbLedApp::begin() {
    if (!app_config::kEnableRgbLed) return false;

    led_.begin(app_config::kRgbLedPin, app_config::kRgbLedMaxBrightness);

    // 队列长度 1：只保留最新的状态，旧的未处理请求直接被覆盖
    queue_ = xQueueCreate(1, sizeof(RgbLedState));
    if (queue_ == nullptr) return false;

    if (xTaskCreate(workerTaskFn, "rgb_led", 2048, this, 2, nullptr) != pdPASS) {
        vQueueDelete(queue_);
        queue_ = nullptr;
        return false;
    }
    return true;
}

void RgbLedApp::setOn(bool on) {
    portENTER_CRITICAL(&lock_);
    state_.on = on;
    portEXIT_CRITICAL(&lock_);
    submit();
}

void RgbLedApp::setColor(uint8_t r, uint8_t g, uint8_t b) {
    portENTER_CRITICAL(&lock_);
    state_.r = r;
    state_.g = g;
    state_.b = b;
    state_.on = true;
    portEXIT_CRITICAL(&lock_);
    submit();
}

void RgbLedApp::setBrightness(uint8_t percent) {
    portENTER_CRITICAL(&lock_);
    state_.brightness = percent > 100 ? 100 : percent;
    portEXIT_CRITICAL(&lock_);
    submit();
}

RgbLedState RgbLedApp::getState() const {
    portENTER_CRITICAL(&lock_);
    RgbLedState copy = state_;
    portEXIT_CRITICAL(&lock_);
    return copy;
}

void RgbLedApp::submit() {
    if (queue_ == nullptr) return;
    RgbLedState copy = getState();
    xQueueOverwrite(queue_, &copy);
}

void RgbLedApp::workerTaskFn(void *args) {
    auto *self = static_cast<RgbLedApp *>(args);
    RgbLedState s;

    while (true) {
        if (xQueueReceive(self->queue_, &s, portMAX_DELAY) == pdPASS) {
            self->led_.write(s.on, s.r, s.g, s.b, s.brightness);
        }
    }
}
