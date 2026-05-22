#include <Arduino.h>
#include "HX_30HM.h"

#define RXD1 16
#define TXD1 17

// 创建SerialServo实例，使用Serial串口，波特率为115200，TX引脚1，RX引脚0
// 注意：根据实际连接的串口修改（可以是Serial, Serial1, Serial2等）
// 根据实际硬件连接修改TX和RX引脚号
SerialServo servo(Serial1, 1000000, TXD1, RXD1);

ServoStatus_t status;

int16_t write_pos = 4096;
int16_t read_pos = 4096;

int16_t write_pos_offset = 100;
int16_t read_pos_offset;

uint8_t write_acc = 100;

int16_t write_speed = 1000;
int16_t read_speed;

int16_t write_pwm_speed = 1000;

uint16_t write_torque = 1000;

int16_t sync_write1[2][4] = {{1, 0, 1000, 4095},
							               {2, 0, 1000, 4095}};

int16_t sync_write2[2][4] = {{1, 0, 1000, 0},
							               {2, 0, 1000, 0}};
uint8_t read_id[] = {1, 2};
int16_t sync_read_data[2][5];

uint8_t temp;
uint8_t vol;
uint16_t cur;
int16_t read_load;
uint8_t moving_status;

void setup() {
    Serial.begin(115200);

    // 1.广播搜索舵机，检查是否有舵机响应
    status = servo.ping(0xFE);    //广播搜索舵机  
    Serial.printf("ID:%d\n", status.id);  //接收到的ID号
    Serial.printf("状态：%X\n", status.error_byte);    //该舵机的工作状态(若接收到数据帧), 参考HX_30HM.h中 ServoStatus_t 结构体注释
    
    // 2.启用/禁用力矩输出，并选择工作模式
    // status = servo.enable_torque(1);         // 启用1号的舵机的力矩输出 (Enable servo torque output)
    // status = servo.disable_torque(1);         // 禁用1号的舵机的力矩输出 (Disable servo torque output)
    // status = servo.cali_pos(1);              // 校准1号的当前舵机位置 (Calibrate servo position)
    // status = servo.select_mode(1, 0);       // 选择1号的舵机工作模式 (Select servo working mode)
    
    // 3.设置和读取位置参数
    // status = servo.write_pos(1, write_pos);   // 设置1号的舵机目标位置 (Set servo target position)
    // status = servo.read_pos(1, &read_pos);    // 读取1号的舵机当前位置值 (Read servo current position value)
    // Serial.println(read_pos);

    // status = servo.write_pos_offset(1, write_pos_offset);  // 设置1号的舵机位置偏差 (Set servo position offset)
        // status = servo.read_pos_offset(1, &read_pos_offset);   // 读取1号的舵机位置偏差 (Read servo position offset)
    // Serial.println(read_pos_offset);

    // 4.设置和读取速度/加速度参数
    // status = servo.write_acc(1, write_acc);   // 设置1号的舵机加速度参数 (Set servo acceleration parameter)

    // status = servo.write_speed(1, write_speed); // 设置1号的舵机目标速度 (Set servo target speed)
        // status = servo.read_speed(1, &read_speed);  // 读取1号的舵机当前速度 (Read servo current speed)  
    // Serial.println(read_speed);

    // status = servo.read_pos_speed(1, &read_pos, &read_speed); // 同时读取1号的舵机位置和速度 (Read servo position and speed)
    // Serial.print(read_pos);
    // Serial.print(", ");
    // Serial.println(read_speed);

    // status = servo.write_pos_ex(1, write_acc, write_speed, write_pos); // 扩展位置控制：同时设置1号的舵机加速度、速度和位置 (Extended position control: set servo acceleration, speed, and position simultaneously)

    // 5.设置 PWM 和力矩参数
    // status = servo.write_pwm_speed(1, write_pwm_speed);     // 设置1号的舵机PWM速度（PWM模式下生效） (Set servo PWM speed (effective in PWM mode))
        // status = servo.write_max_torque(1, write_torque);       // 设置1号的舵机最大力矩限制 (Set servo maximum torque limit)
    // status = servo.write_reg_pos_ex(1, write_acc, write_speed, 0); // 向1号舵机指定寄存器地址写入加速度、速度和位置信息（暂不执行，需调用action触发） (Write acceleration, speed, and position information to servo register address)
    // status = servo.reg_action(1);               // 执行1号的舵机动作指令 (Execute servo action instruction)
    
    // 6.同步写入和读取多个舵机
    // status = servo.sync_write_pos_ex(sync_write1, 2);      // 向1号和2号舵机同时写入加速度、速度和位置信息 (Write acceleration, speed, and position information to servo 1 and 2 simultaneously)
    // status = servo.sync_read_cur_pos_ex(read_id, 2, sync_read_data);  // 同部读取1号和2号舵机加速度、速度和位置信息 (Read acceleration, speed, and position information from servo 1 and 2 simultaneously)
    // for(uint8_t i = 0; i < 2; i++) {
    //   for(uint8_t j = 0; j < 5; j++) {
    //     Serial.print(sync_read_data[i][j]);
    //     Serial.print(", ");
    //   }
    //   Serial.println();
    // }
    
    // 7.读取各种状态参数
        // status = servo.read_temperture(1, &temp);   // 读取1号舵机温度值 (Read servo temperature value)
    // Serial.println(temp);
        // status = servo.read_voltage(1, &vol);       //  读取1号舵机电压值 (Read servo voltage value)
    // Serial.println(vol);
        // status = servo.read_current(1, &cur);       //  读取1号舵机电流值 (Read servo current value)
    // Serial.println(cur);
        // status = servo.read_load(1, &read_load);    //  读取1号舵机负载值 (Read servo load value)
    // Serial.println(read_load);
        // status = servo.read_moving_status(1, &moving_status); //  读取1号舵机运动状态 (Read servo moving status)
    // Serial.println(moving_status);
}

void loop() {

}