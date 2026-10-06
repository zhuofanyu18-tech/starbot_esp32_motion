#include "apps/WifiApp.h"

#include <WiFi.h>
#include <cerrno>
#include <cstdlib>
#include <cstring>

namespace {

constexpr size_t kMaxCommandLength = 64;
constexpr size_t kMaxWsClients = 8;

// platformio.ini 中 board_build.embed_txtfiles 嵌入的网页
extern "C" const uint8_t index_html_start[] asm("_binary_src_web_index_html_start");
extern "C" const uint8_t index_html_end[] asm("_binary_src_web_index_html_end");

// 解析 [minimum, maximum] 范围内的整数，整个字符串必须都是数字
bool parseInt(const char *token, long minimum, long maximum, long &value) {
    if (token == nullptr || token[0] == '\0') return false;
    errno = 0;
    char *end = nullptr;
    const long parsed = strtol(token, &end, 10);
    if (errno == ERANGE || end == token || *end != '\0' || parsed < minimum || parsed > maximum) {
        return false;
    }
    value = parsed;
    return true;
}

// 按逗号切分指令，返回段数；超过 capacity 段时返回 capacity + 1 表示格式错误
size_t splitCommand(char *command, char **tokens, size_t capacity) {
    size_t count = 0;
    char *save = nullptr;
    for (char *t = strtok_r(command, ",", &save); t != nullptr; t = strtok_r(nullptr, ",", &save)) {
        if (count >= capacity) return capacity + 1;
        tokens[count++] = t;
    }
    return count;
}

}  // namespace

WifiApp *WifiApp::instance_ = nullptr;

WifiApp::WifiApp() {
    instance_ = this;
}

bool WifiApp::begin(RgbLedApp &led) {
    if (!app_config::kEnableWifi) return false;
    led_ = &led;

    WiFi.mode(WIFI_AP);
    const char *password = strlen(app_config::kWifiApPassword) == 0 ? nullptr : app_config::kWifiApPassword;
    if (!WiFi.softAP(app_config::kWifiApSsid, password,
                     app_config::kWifiApChannel, 0, app_config::kWifiApMaxConnections)) {
        return false;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.max_uri_handlers = 4;
    if (httpd_start(&server_, &config) != ESP_OK) {
        server_ = nullptr;
        return false;
    }

    httpd_uri_t root_uri = {};
    root_uri.uri = "/";
    root_uri.method = HTTP_GET;
    root_uri.handler = rootHandler;

    httpd_uri_t ws_uri = {};
    ws_uri.uri = "/ws";
    ws_uri.method = HTTP_GET;
    ws_uri.handler = websocketHandler;
    ws_uri.is_websocket = true;

    if (httpd_register_uri_handler(server_, &root_uri) != ESP_OK ||
        httpd_register_uri_handler(server_, &ws_uri) != ESP_OK) {
        httpd_stop(server_);
        server_ = nullptr;
        return false;
    }
    return true;
}

uint8_t WifiApp::stationCount() const {
    return isRunning() ? WiFi.softAPgetStationNum() : 0;
}

IPAddress WifiApp::apIp() const {
    return WiFi.softAPIP();
}

esp_err_t WifiApp::rootHandler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    // 文本嵌入时末尾自动追加了 '\0'，发送时去掉
    return httpd_resp_send(req, (const char *)index_html_start, index_html_end - index_html_start - 1);
}

esp_err_t WifiApp::websocketHandler(httpd_req_t *req) {
    // GET 请求是 WebSocket 握手，握手完成即返回
    if (req->method == HTTP_GET) return ESP_OK;
    if (instance_ == nullptr) return ESP_FAIL;

    httpd_ws_frame_t frame = {};
    frame.type = HTTPD_WS_TYPE_TEXT;
    esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);  // 先读长度
    if (err != ESP_OK) return err;

    if (frame.len == 0 || frame.len > kMaxCommandLength) {
        return sendError(req, "invalid_length", "command length is invalid");
    }

    uint8_t payload[kMaxCommandLength + 1] = {0};
    frame.payload = payload;
    err = httpd_ws_recv_frame(req, &frame, kMaxCommandLength);
    if (err != ESP_OK) return err;
    if (frame.type != HTTPD_WS_TYPE_TEXT) {
        return sendError(req, "text_only", "only text commands are accepted");
    }

    return instance_->handleCommand(req, reinterpret_cast<char *>(payload));
}

esp_err_t WifiApp::handleCommand(httpd_req_t *req, char *command) {
    if (strcmp(command, "STATE_GET") == 0) {
        char json[128];
        formatLedState(json, sizeof(json));
        return sendText(req, json);
    }

    if (strncmp(command, "LED_", 4) == 0 && !led_->isEnabled()) {
        return sendError(req, "led_disabled", "RGB LED is disabled in firmware");
    }

    if (strcmp(command, "LED_ON") == 0) {
        led_->setOn(true);
        broadcastLedState();
        return ESP_OK;
    }
    if (strcmp(command, "LED_OFF") == 0) {
        led_->setOn(false);
        broadcastLedState();
        return ESP_OK;
    }

    char *tokens[4] = {nullptr};
    const size_t count = splitCommand(command, tokens, 4);

    if (count == 4 && strcmp(tokens[0], "LED_COLOR") == 0) {
        long r, g, b;
        if (!parseInt(tokens[1], 0, 255, r) || !parseInt(tokens[2], 0, 255, g) || !parseInt(tokens[3], 0, 255, b)) {
            return sendError(req, "invalid_color", "color must be 0-255");
        }
        led_->setColor((uint8_t)r, (uint8_t)g, (uint8_t)b);
        broadcastLedState();
        return ESP_OK;
    }

    if (count == 2 && strcmp(tokens[0], "LED_BRIGHTNESS") == 0) {
        long percent;
        if (!parseInt(tokens[1], 0, 100, percent)) {
            return sendError(req, "invalid_brightness", "brightness must be 0-100");
        }
        led_->setBrightness((uint8_t)percent);
        broadcastLedState();
        return ESP_OK;
    }

    return sendError(req, "unknown_command", "command is not supported");
}

int WifiApp::formatLedState(char *json, size_t size) const {
    const RgbLedState s = led_->getState();
    return snprintf(json, size,
                    "{\"type\":\"led\",\"enabled\":%s,\"on\":%s,\"r\":%u,\"g\":%u,\"b\":%u,\"brightness\":%u}",
                    led_->isEnabled() ? "true" : "false", s.on ? "true" : "false",
                    s.r, s.g, s.b, s.brightness);
}

void WifiApp::broadcastLedState() {
    char json[128];
    const int len = formatLedState(json, sizeof(json));
    if (len <= 0 || len >= (int)sizeof(json)) return;

    size_t client_count = kMaxWsClients;
    int client_fds[kMaxWsClients];
    if (httpd_get_client_list(server_, &client_count, client_fds) != ESP_OK) return;

    httpd_ws_frame_t frame = {};
    frame.type = HTTPD_WS_TYPE_TEXT;
    frame.payload = reinterpret_cast<uint8_t *>(json);
    frame.len = (size_t)len;

    for (size_t i = 0; i < client_count; ++i) {
        if (httpd_ws_get_fd_info(server_, client_fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
            httpd_ws_send_frame_async(server_, client_fds[i], &frame);
        }
    }
}

esp_err_t WifiApp::sendText(httpd_req_t *req, const char *text) {
    httpd_ws_frame_t frame = {};
    frame.type = HTTPD_WS_TYPE_TEXT;
    frame.payload = (uint8_t *)text;
    frame.len = strlen(text);
    return httpd_ws_send_frame(req, &frame);
}

esp_err_t WifiApp::sendError(httpd_req_t *req, const char *code, const char *message) {
    char json[160];
    snprintf(json, sizeof(json), "{\"type\":\"error\",\"code\":\"%s\",\"message\":\"%s\"}", code, message);
    return sendText(req, json);
}
