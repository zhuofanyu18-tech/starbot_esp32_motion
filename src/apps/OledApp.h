#ifndef OLED_APP_H
#define OLED_APP_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <OledDisplay.h>

#include "config/AppConfig.h"
#include "apps/CarControllerApp.h"
#include "apps/WifiApp.h"
#include "utils/RosAgentState.h"

/*
    OLED 显示 App：在独立的 FreeRTOS 任务中定时刷新，ROS 断线时屏幕依旧工作
    显示内容：ROS 连接状态、WiFi 热点、小车线速度/角速度、四个轮子的实时速度、运行时间
*/
class OledApp {
public:
    OledApp();

    // 初始化屏幕并启动刷新任务；没检测到屏幕时返回 false，其余功能不受影响
    bool begin(CarControllerApp &car, const WifiApp &wifi);

    void setRosState(RosAgentState state);

private:
    OledDisplay        oled_;
    CarControllerApp  *car_ = nullptr;
    const WifiApp     *wifi_ = nullptr;
    TaskHandle_t       task_handle_ = nullptr;

    volatile RosAgentState ros_state_ = RosAgentState::kWaitingAgent;
    volatile bool          ever_connected_ = false;  // 用于区分“首次等待”和“断线重连”

    void render();
    const char *rosStateText() const;

    static void refreshTaskFn(void *args);
};

#endif
