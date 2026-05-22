#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>

class IMU {
public:
    struct RawData {
        int16_t ax, ay, az;
        int16_t gx, gy, gz;
        int16_t hx, hy, hz;
        int16_t roll, pitch, yaw;
        int16_t temp;
        int16_t q0, q1, q2, q3;
    };

    struct Data {
        float accel_x;
        float accel_y;
        float accel_z;
        float gyro_x;
        float gyro_y;
        float gyro_z;
        float angle_roll;
        float angle_pitch;
        float angle_yaw;
        float quat_w;
        float quat_x;
        float quat_y;
        float quat_z;
        float temp;
    };

    IMU();

    bool begin(uint8_t sda_pin, uint8_t scl_pin);
    bool isConnected() const { return connected_; }

    bool readAll();
    bool readAccel();
    bool readGyro();
    bool readAngle();
    bool readQuat();
    bool readTemp();

    bool configure(uint16_t output_rate, uint16_t bandwidth,
                   uint16_t gyro_range, uint16_t accel_range);

    const Data& getData() const { return data_; }
    const RawData& getRaw() const { return raw_; }

private:
    static constexpr uint8_t kI2cAddr7Bit = 0x50;

    RawData raw_;
    Data    data_;
    bool    connected_;

    bool writeRegister(uint8_t reg, uint16_t value);
    bool readRegisters(uint8_t start_reg, uint8_t count, int16_t *dest);

    bool unlockKey();
    void convertRawToData();
};

#endif
