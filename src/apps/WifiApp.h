#ifndef WIFI_APP_H
#define WIFI_APP_H

#include <IPAddress.h>
#include <esp_http_server.h>

#include "config/AppConfig.h"
#include "apps/RgbLedApp.h"

/*
    WiFi 手机控制 App
    - ESP32 开热点（SoftAP），手机连上后浏览器打开 http://192.168.4.1
    - GET /   ：返回控制网页（src/web/index.html，编译时嵌入固件）
    - /ws     ：WebSocket 文本指令，回调中只做校验并转交给各 App，不直接操作硬件

    WebSocket 指令：
        STATE_GET               查询 RGB 灯状态
        LED_ON / LED_OFF        开灯 / 关灯
        LED_COLOR,r,g,b         设置颜色（0~255），同时开灯
        LED_BRIGHTNESS,p        设置亮度（0~100）
    回复（JSON）：
        {"type":"led","enabled":true,"on":true,"r":255,"g":0,"b":0,"brightness":50}
        {"type":"error","code":"...","message":"..."}
    灯状态变化后会广播给所有已连接的手机。
*/
class WifiApp {
public:
    WifiApp();

    // 启动热点和 Web 服务器；kEnableWifi=false 时直接返回 false
    bool begin(RgbLedApp &led);

    bool isRunning() const { return server_ != nullptr; }
    uint8_t stationCount() const;  // 已连接热点的手机数量
    IPAddress apIp() const;        // 热点地址，默认 192.168.4.1

private:
    static WifiApp *instance_;

    httpd_handle_t server_ = nullptr;
    RgbLedApp     *led_ = nullptr;

    esp_err_t handleCommand(httpd_req_t *req, char *command);
    int  formatLedState(char *json, size_t size) const;
    void broadcastLedState();

    static esp_err_t sendText(httpd_req_t *req, const char *text);
    static esp_err_t sendError(httpd_req_t *req, const char *code, const char *message);

    static esp_err_t rootHandler(httpd_req_t *req);
    static esp_err_t websocketHandler(httpd_req_t *req);
};

#endif
