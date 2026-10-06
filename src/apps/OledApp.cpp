#include "apps/OledApp.h"

OledApp::OledApp()
    : oled_(app_config::kOledWidth, app_config::kOledHeight, &Wire, app_config::kI2cClockHz) {}

bool OledApp::begin(const CarControllerApp &car, const WifiApp &wifi) {
    car_ = &car;
    wifi_ = &wifi;
    if (!oled_.begin(app_config::kOledI2cAddress)) return false;

    // 放在核 0、最低优先级：刷新一帧约 25ms，不能占用核 1 的控制和 ROS 时间
    xTaskCreatePinnedToCore(refreshTaskFn, "oled", 4096, this, app_config::kOledTaskPriority,
                            &task_handle_, app_config::kBackgroundCore);
    return true;
}

void OledApp::setRosState(RosAgentState state) {
    if (state == RosAgentState::kConnected) ever_connected_ = true;
    ros_state_ = state;
}

const char *OledApp::rosStateText() const {
    switch (ros_state_) {
    case RosAgentState::kConnected:      return "ROS: CONNECTED";
    case RosAgentState::kAgentAvailable: return "ROS: CREATING NODE";
    case RosAgentState::kDisconnected:   return "ROS: LOST";
    case RosAgentState::kWaitingAgent:
    default:
        return ever_connected_ ? "ROS: RECONNECTING" : "ROS: WAIT AGENT";
    }
}

void OledApp::render() {
    oled_.clear();
    oled_.printLine(0, true, "%-20s", rosStateText());

    // 第 1 行：WiFi 热点地址和已连接手机数
    if (wifi_->isRunning()) {
        const IPAddress ip = wifi_->apIp();
        oled_.printLine(1, "AP %u.%u.%u.%u  x%u", ip[0], ip[1], ip[2], ip[3], wifi_->stationCount());
    }

    // 第 2、3 行：整车速度（来自编码器里程计）
    const CarControllerApp::State st = car_->getState();
    oled_.printLine(2, "v %+6.2f m/s", st.linear);
    oled_.printLine(3, "w %+6.2f rad/s", st.angular);

    // 第 5、6 行：四个轮子速度 m/s
    oled_.printLine(5, "FL%+5.2f  FR%+5.2f", st.wheel_speed[0], st.wheel_speed[1]);
    oled_.printLine(6, "RL%+5.2f  RR%+5.2f", st.wheel_speed[2], st.wheel_speed[3]);

    const uint32_t up_s = millis() / 1000;
    oled_.printLine(7, "Up %02lu:%02lu:%02lu",
                    (unsigned long)(up_s / 3600), (unsigned long)(up_s / 60 % 60), (unsigned long)(up_s % 60));
    oled_.show();
}

void OledApp::refreshTaskFn(void *args) {
    auto *self = static_cast<OledApp *>(args);
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        self->render();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(app_config::kOledRefreshPeriodMs));
    }
}
