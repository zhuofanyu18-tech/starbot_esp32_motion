#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

class ServoController {
public:
    // 构造函数（可选I2C地址）
    ServoController(uint8_t address = 0x40);
    
    // 初始化方法（可指定SDA/SCL引脚）
    void begin(int sda = 6, int scl = 5, uint32_t freq = 60);
    
    // 设置舵机脉冲（单位：毫秒）
    void setServoPulse(uint8_t channel, double pulseMs);
    
    // 直接设置PWM值
    void setPWM(uint8_t channel, uint16_t value);
    
    // 设置舵机角度（0-180度）
    // 默认舵机脉冲宽度为 500-2500 --> 0.5-2.5
    void setAngle(uint8_t channel, uint8_t angle, 
                 double minPulse = 0.5, double maxPulse = 2.5);

private:
    Adafruit_PWMServoDriver _pwm;
};

#endif