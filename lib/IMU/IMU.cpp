#include "IMU.h"

// Register addresses (from wit_c_sdk REG.h)
#define REG_SAVE      0x00
#define REG_CALSW     0x01
#define REG_RSW       0x02
#define REG_RRATE     0x03
#define REG_BAUD      0x04
#define REG_AXOFFSET  0x05
#define REG_AYOFFSET  0x06
#define REG_AZOFFSET  0x07
#define REG_GXOFFSET  0x08
#define REG_GYOFFSET  0x09
#define REG_GZOFFSET  0x0A
#define REG_HXOFFSET  0x0B
#define REG_HYOFFSET  0x0C
#define REG_HZOFFSET  0x0D
#define REG_IICADDR   0x1A
#define REG_BANDWIDTH 0x1F
#define REG_GYRORANGE 0x20
#define REG_ACCRANGE  0x21
#define REG_YYMM      0x30
#define REG_DDHH      0x31
#define REG_MMSS      0x32
#define REG_MS        0x33
#define REG_AX        0x34
#define REG_AY        0x35
#define REG_AZ        0x36
#define REG_GX        0x37
#define REG_GY        0x38
#define REG_GZ        0x39
#define REG_HX        0x3A
#define REG_HY        0x3B
#define REG_HZ        0x3C
#define REG_Roll      0x3D
#define REG_Pitch     0x3E
#define REG_Yaw       0x3F
#define REG_TEMP      0x40
#define REG_q0        0x51
#define REG_q1        0x52
#define REG_q2        0x53
#define REG_q3        0x54
#define REG_KEY       0x69

#define KEY_UNLOCK    0xB588

// Output rate values (RRATE)
#define RRATE_02HZ  0x01
#define RRATE_05HZ  0x02
#define RRATE_1HZ   0x03
#define RRATE_2HZ   0x04
#define RRATE_5HZ   0x05
#define RRATE_10HZ  0x06
#define RRATE_20HZ  0x07
#define RRATE_50HZ  0x08
#define RRATE_100HZ 0x09
#define RRATE_200HZ 0x0B

// Bandwidth values
#define BANDWIDTH_256HZ  0
#define BANDWIDTH_184HZ  1
#define BANDWIDTH_94HZ   2
#define BANDWIDTH_44HZ   3
#define BANDWIDTH_21HZ   4
#define BANDWIDTH_10HZ   5
#define BANDWIDTH_5HZ    6

IMU::IMU() : connected_(false) {
    memset(&raw_, 0, sizeof(raw_));
    memset(&data_, 0, sizeof(data_));
}

bool IMU::begin(uint8_t sda_pin, uint8_t scl_pin) {
    Wire.begin((int)sda_pin, (int)scl_pin);
    Wire.setClock(400000);

    // Auto-detect: try reading AX register, retry once (matching STM32 AutoScanSensor)
    for (int retry = 0; retry < 2; retry++) {
        int16_t test[3];
        if (readRegisters(REG_AX, 3, test)) {
            connected_ = true;
            return true;
        }
        delay(5);
    }
    connected_ = false;
    return false;
}

bool IMU::writeRegister(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(kI2cAddr7Bit);
    Wire.write(reg);
    Wire.write(value & 0xFF);
    Wire.write((value >> 8) & 0xFF);
    return Wire.endTransmission(true) == 0;
}

bool IMU::readRegisters(uint8_t start_reg, uint8_t count, int16_t *dest) {
    Wire.beginTransmission(kI2cAddr7Bit);
    Wire.write(start_reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    uint8_t bytes = count * 2;
    if (Wire.requestFrom(kI2cAddr7Bit, bytes) != bytes) {
        return false;
    }

    for (uint8_t i = 0; i < count; i++) {
        uint8_t lsb = Wire.read();
        uint8_t msb = Wire.read();
        dest[i] = (int16_t)(lsb | ((uint16_t)msb << 8));
    }
    return true;
}

bool IMU::unlockKey() {
    if (!writeRegister(REG_KEY, KEY_UNLOCK)) {
        return false;
    }
    delay(5);
    return true;
}

bool IMU::configure(uint16_t output_rate, uint16_t bandwidth,
                    uint16_t gyro_range, uint16_t accel_range) {
    if (!connected_) return false;
    if (!unlockKey()) return false;
    if (!writeRegister(REG_RRATE, output_rate)) return false;
    if (!writeRegister(REG_BANDWIDTH, bandwidth)) return false;
    if (!writeRegister(REG_GYRORANGE, gyro_range)) return false;
    if (!writeRegister(REG_ACCRANGE, accel_range)) return false;
    // Save parameters
    if (!writeRegister(REG_SAVE, 0x00)) return false;
    delay(100);
    return true;
}

void IMU::convertRawToData() {
    constexpr float kAccScale  = 16.0f / 32768.0f * 9.80665f;
    constexpr float kGyroScale = 2000.0f / 32768.0f * (PI / 180.0f);
    constexpr float kAngleScale = 180.0f / 32768.0f * (PI / 180.0f);
    constexpr float kQuatScale = 1.0f / 32768.0f;

    data_.accel_x = raw_.ax * kAccScale;
    data_.accel_y = raw_.ay * kAccScale;
    data_.accel_z = raw_.az * kAccScale;

    data_.gyro_x = raw_.gx * kGyroScale;
    data_.gyro_y = raw_.gy * kGyroScale;
    data_.gyro_z = raw_.gz * kGyroScale;

    data_.angle_roll  = raw_.roll * kAngleScale;
    data_.angle_pitch = raw_.pitch * kAngleScale;
    data_.angle_yaw   = raw_.yaw * kAngleScale;

    data_.quat_w = raw_.q0 * kQuatScale;
    data_.quat_x = raw_.q1 * kQuatScale;
    data_.quat_y = raw_.q2 * kQuatScale;
    data_.quat_z = raw_.q3 * kQuatScale;

    data_.temp = raw_.temp / 100.0f;
}

bool IMU::readAll() {
    if (!connected_) return false;

    int16_t buf[13];
    if (!readRegisters(REG_AX, 13, buf)) return false;

    raw_.ax   = buf[0];
    raw_.ay   = buf[1];
    raw_.az   = buf[2];
    raw_.gx   = buf[3];
    raw_.gy   = buf[4];
    raw_.gz   = buf[5];
    raw_.hx   = buf[6];
    raw_.hy   = buf[7];
    raw_.hz   = buf[8];
    raw_.roll  = buf[9];
    raw_.pitch = buf[10];
    raw_.yaw   = buf[11];
    raw_.temp  = buf[12];

    int16_t quat[4];
    if (!readRegisters(REG_q0, 4, quat)) return false;
    raw_.q0 = quat[0];
    raw_.q1 = quat[1];
    raw_.q2 = quat[2];
    raw_.q3 = quat[3];

    convertRawToData();
    return true;
}

bool IMU::readAccel() {
    if (!connected_) return false;
    int16_t buf[3];
    if (!readRegisters(REG_AX, 3, buf)) return false;
    raw_.ax = buf[0];
    raw_.ay = buf[1];
    raw_.az = buf[2];
    data_.accel_x = raw_.ax * (16.0f / 32768.0f * 9.80665f);
    data_.accel_y = raw_.ay * (16.0f / 32768.0f * 9.80665f);
    data_.accel_z = raw_.az * (16.0f / 32768.0f * 9.80665f);
    return true;
}

bool IMU::readGyro() {
    if (!connected_) return false;
    int16_t buf[3];
    if (!readRegisters(REG_GX, 3, buf)) return false;
    raw_.gx = buf[0];
    raw_.gy = buf[1];
    raw_.gz = buf[2];
    data_.gyro_x = raw_.gx * (2000.0f / 32768.0f * (PI / 180.0f));
    data_.gyro_y = raw_.gy * (2000.0f / 32768.0f * (PI / 180.0f));
    data_.gyro_z = raw_.gz * (2000.0f / 32768.0f * (PI / 180.0f));
    return true;
}

bool IMU::readAngle() {
    if (!connected_) return false;
    int16_t buf[3];
    if (!readRegisters(REG_Roll, 3, buf)) return false;
    raw_.roll  = buf[0];
    raw_.pitch = buf[1];
    raw_.yaw   = buf[2];
    data_.angle_roll  = raw_.roll * (180.0f / 32768.0f * (PI / 180.0f));
    data_.angle_pitch = raw_.pitch * (180.0f / 32768.0f * (PI / 180.0f));
    data_.angle_yaw   = raw_.yaw * (180.0f / 32768.0f * (PI / 180.0f));
    return true;
}

bool IMU::readQuat() {
    if (!connected_) return false;
    int16_t buf[4];
    if (!readRegisters(REG_q0, 4, buf)) return false;
    raw_.q0 = buf[0];
    raw_.q1 = buf[1];
    raw_.q2 = buf[2];
    raw_.q3 = buf[3];
    data_.quat_w = raw_.q0 / 32768.0f;
    data_.quat_x = raw_.q1 / 32768.0f;
    data_.quat_y = raw_.q2 / 32768.0f;
    data_.quat_z = raw_.q3 / 32768.0f;
    return true;
}

bool IMU::readTemp() {
    if (!connected_) return false;
    int16_t buf;
    if (!readRegisters(REG_TEMP, 1, &buf)) return false;
    raw_.temp = buf;
    data_.temp = raw_.temp / 100.0f;
    return true;
}
