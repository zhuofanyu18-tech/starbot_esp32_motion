#include "ServoController.h"

// 构造函数，创建 ServoController 类时，自动将pwm_初始化
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);
ServoController::ServoController(uint8_t address)
    : _pwm(address, Wire) {}

void ServoController::begin(int sda, int scl, uint32_t freq)
{
    Wire.begin(sda, scl); // 设置iic
    _pwm.begin();
    _pwm.setPWMFreq(freq);
    delay(10); // 等待初始化完成
}

// 设置脉冲宽度 -> 500 ~ 2500
void ServoController::setServoPulse(uint8_t channel, double pulseMs)
{
    // 计算每位的微秒数 (60Hz -> 4096分辨率)
    const double usPerBit = (1000000.0 / 60.0) / 4096.0;
    uint16_t pulseValue = static_cast<uint16_t>(pulseMs * 1000 / usPerBit);
    _pwm.setPWM(channel, 0, pulseValue);
}

void ServoController::setPWM(uint8_t channel, uint16_t value)
{
    _pwm.setPWM(channel, 0, value);
}

// 设置舵机角度函数
void ServoController::setAngle(uint8_t channel, uint8_t angle,
                               double minPulse, double maxPulse)
{
    // 确保角度在0-180范围内
    if (angle > 180)
        angle = 180;

    // 计算对应的脉冲宽度（毫秒）
    // 公式：脉冲宽度 = minPulse + (maxPulse - minPulse) * (angle / 180.0)
    double pulseMs = minPulse + (maxPulse - minPulse) * (angle / 180.0);

    // 调用脉冲设置函数
    setServoPulse(channel, pulseMs);
}