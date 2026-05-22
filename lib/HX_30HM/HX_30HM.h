/**
 * @file HX_30HM.h
 * @author Min
 * @brief Servo control library header file
 * @version 1.0
 * @date 2025-10-21
 *
 * @copyright Copyright (c) 2025
 */
#ifndef PACKET_HANDLER_H
#define PACKET_HANDLER_H

#include "Arduino.h"
#include "HardwareSerial.h"
#include "HX_30HM_Def.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef HX30HM_DEBUG
#define HX30HM_DEBUG 0
#endif

#ifndef HX30HM_RX_TIMEOUT_MS
#define HX30HM_RX_TIMEOUT_MS 50
#endif

#define ACK	1
#define MAX_FRAME_SIZE 250

/**
 * @brief 舵机状态结构体 (Servo status structure)
 * @details 包含舵机ID和错误状态信息
 *          (Contains servo ID and error status information)
 */
typedef struct {
	uint8_t id;                				/**< 舵机ID (Servo ID) */
	union {
		uint8_t error_byte;      			  /**< 错误状态字节 (Error status byte) */
		struct {
			uint8_t bit_voltage : 		1;  /**< 电压错误bit标志 (Voltage error bit flag) */
			uint8_t bit_sensor : 		  1;  /**< 传感器错误bit标志 (Sensor error bit flag) */
			uint8_t bit_overheat : 		1;  /**< 过热错误bit标志 (Overheat error bit flag) */
			uint8_t bit_current : 		1;  /**< 电流错误bit标志 (Current error bit flag) */
			uint8_t bit_angle : 		  1;  /**< 角度错误bit标志 (Angle error bit flag) */
			uint8_t bit_overload : 		1;  /**< 过载错误bit标志 (Overload error bit flag) */
			uint8_t bit_tx : 			    1;  /**< 发送错误bit标志 (Transmit error bit flag) */
			uint8_t bit_rx : 			    1;  /**< 接收错误bit标志 (Receive error bit flag) */
		}error_bits;
	};
}ServoStatus_t;

typedef enum
{
	PACKET_HEADER_1 = 0,  /**< 第一个帧头字节 (First header byte) */
	PACKET_HEADER_2,      /**< 第二个帧头字节 (Second header byte) */
	PACKET_ID,            /**< ID字段 (ID field) */
	PACKET_DATA_LENGTH,   /**< 数据长度字段 (Data length field) */
	PACKET_CMD,           /**< 命令字段 (Command field) */
	PACKET_PARAMETERS,    /**< 参数字段 (Parameter field) */
	PACKET_CHECKSUM,      /**< 校验和字段 (Checksum field) */
	PACKET_FINISH         /**< 数据包解析完成 (Packet completed) */
}PacketStatus;

/**
 * @struct PacketTypeDef
 * @brief 数据包结构定义 (Packet structure definition)
 * @details 定义了舵机通信数据包的格式，包含帧头、ID、长度、命令和参数 (Defines servo communication packet format)
 */
#pragma pack(1)
typedef struct {
    uint8_t header_1;     /**< 第一个帧头字节 (First header byte) */
    uint8_t header_2;     /**< 第二个帧头字节 (Second header byte) */

    union {
        struct {
          uint8_t id;                           /**< 舵机ID (Servo ID) */
          uint8_t length;                       /**< 数据包长度 (Packet length) */
          uint8_t cmd;                          /**< 命令字节 (Command byte) */
          uint8_t args[MAX_FRAME_SIZE];         /**< 参数数据 (Parameter data) */
        } elements;                             /**< 分解的数据包元素 (Decomposed packet elements) */
        uint8_t data_raw[MAX_FRAME_SIZE + 3];   /**< 发送/接收参数数组 (Transmit/Receive parameter buffer) */
    };
} PacketTypeDef;
#pragma pack()

#ifdef __cplusplus
}
#endif

class SerialServo {
public:
  SerialServo(HardwareSerial& uart, uint32_t baudrate, uint8_t tx_pin, uint8_t rx_pin) {
    this->uart = &uart;  // 修复：获取引用的地址赋值给指针
    this->uart->setRxBufferSize(512);
    this->uart->begin(baudrate, SERIAL_8N1, rx_pin, tx_pin);  // 添加：使用tx和rx引脚参数初始化串口
    rx_skip = 1;
    rx_timeout = HX30HM_RX_TIMEOUT_MS;
    endianness = 0;
  }

  /**
  * @brief 舵机通信测试 (Servo communication test)
  * @details 检查指定ID的舵机是否在线
  *          (Check if the servo with specified ID is online)
  * @param id 舵机ID (Servo ID)
  * @return 舵机状态信息 (Servo status information)
  */
  ServoStatus_t ping(uint8_t id);

  /**
  * @brief 普通方式写入数据 (General write data)
  * @param id 舵机ID (Servo ID)
  * @param addr 寄存器首地址 (Register first address)
  * @param data 需要写入的数据指针 (Write data pointer)
  * @param data_len 写入的数据长度 (Write data length)
  * @return 舵机状态信息 (Servo status information)
  */
  ServoStatus_t general_write(uint8_t id, uint8_t addr, uint8_t *data, uint8_t data_len);

  /**
  * @brief 普通方式读取数据 (General read data)
  * @param id 舵机ID (Servo ID)
  * @param addr 寄存器首地址 (Register first address)
  * @param data 读取到的数据指针 (Read data pointer)
  * @param data_len 读取的数据长度 (Read data length)
  * @return 舵机状态信息 (Servo status information)
  */
  ServoStatus_t general_read(uint8_t id, uint8_t addr, uint8_t *data, uint8_t data_len);

  /**/
  /**
  * @brief 异步方式写入数据 (Async write data)
  * @param id 舵机ID (Servo ID)
  * @param addr 寄存器首地址 (Register first address)
  * @param data 数据指针 (Data pointer)
  * @param data_len 数据长度 (Data length)
  * @return 舵机状态信息 (Servo status information)
  */
  ServoStatus_t reg_write(uint8_t id, uint8_t addr, uint8_t *data, uint8_t data_len);

  /**
  * @brief 异步方式触发操作 (Async trigger action)
  * @param id 舵机ID (Servo ID)
  * @return 舵机状态信息 (Servo status information)
  */
  ServoStatus_t reg_action(uint8_t id);

  /**
  * @brief 同步方式写入指定ID舵机的数据 (Write data for the servo with specified ID)
  * @param addr 寄存器首地址 (Register first address)
  * @param data 写入数据 (Write data)
  * @param data_len 写入数据长度 (Write data length)
  * @param parameter_len 参数长度 (Parameter length)
  * @attention -data 该数据序列为：[id1, parameter_len, id2, parameter_len, ..., idn, parameter_len]
  * 			(Data sequence is: [id1, parameter_len, id2, parameter_len, ..., idn, parameter_len])
  * 			-parameter_len 该参数为每个舵机需要写入的参数长度，不包括id号
  *          (Parameter length for each servo, excluding ID)
  * @return 舵机状态信息 (Servo status information)
  */
  ServoStatus_t sync_write(uint8_t addr, uint8_t *data, uint8_t data_len, uint8_t parameter_len);

  /**
  * @brief 同步方式读取多个舵机的参数 (Read data for multiple servos)
  * @param addr 寄存器首地址 (Register first address)
  * @param byte_num 需要读取的数据字节数 (Number of bytes for each servo parameter)
  * @param id 需要读取的舵机ID数据指针 (Servo ID array pointer)
  * @param id_num 读取的舵机ID数量 (Number of servo IDs to read)
  * @param data 读取到的数据指针 (Read data pointer)
  * @return 舵机状态信息 (Servo status information)
  */
  ServoStatus_t sync_read(uint8_t addr, uint8_t byte_num, uint8_t *id, uint8_t id_num, uint8_t *data);

  /**
  * @brief 使能舵机力矩输出 (Enable servo torque output)
  * @param id 舵机ID (Servo ID)
  * @return 舵机状态信息 (Servo status information)
  */
  ServoStatus_t enable_torque(uint8_t id);

  /**
  * @brief 关闭舵机力矩输出 (Disable servo torque output)
  * @param id 舵机ID (Servo ID)
  * @return 舵机状态信息 (Servo status information)
  */
  ServoStatus_t disable_torque(uint8_t id);

  /**
  * @brief 校准舵机当前位置 (Calibrate servo current position)
  * @param id 舵机ID (Servo ID)
  * @return 舵机状态信息 (Servo status information)
  */
  ServoStatus_t cali_pos(uint8_t id);

  /**
  * @brief 选择舵机工作模式 (Select servo working mode)
  * @param id 舵机ID (Servo ID)
  * @param mode 工作模式 (Working mode)
  *          - 0: 位置模式 (Position mode)
  *          - 1: 闭环速度模式 (Closed-loop speed mode)
  * 			- 2: PWM模式(速度非恒定) (PWM mode (speed non-constant))
  * @return 舵机状态信息 (Servo status information)
  */
  ServoStatus_t select_mode(uint8_t id, uint8_t mode);

  /**
  * @brief 设置舵机目标位置 (Set servo target position)
  * @param id 舵机ID (Servo ID)
  * @param pos 目标位置 (Target position)
  * @attention - 位置范围: -30719 ~ 30719 (Position range: -30719 ~ 30719)
  * @return 舵机状态信息 (Servo status information)
  */
  ServoStatus_t write_pos(uint8_t id, int16_t pos);

  /**
  * @brief 设置舵机位置偏移 (Set servo position offset)
  * @param id 舵机ID (Servo ID)
  * @param offset 位置偏移量 (Position offset)
  * @attention - 偏移范围: -2047 ~ 2047 (Offset range: -2047 ~ 2047)
  * @return 舵机状态信息 (Servo status information)
  */
  ServoStatus_t write_pos_offset(uint8_t id, int16_t offset);

  /**
  * @brief 设置舵机加速度 (Set servo acceleration)
  * @param id 舵机ID (Servo ID)
  * @param acc 加速度值 (Acceleration value)
  * @attention - 加速度范围: 0 ~ 254 (Acceleration range: 0 ~ 254)
  * @return 舵机状态信息 (Servo status information)
  */
  ServoStatus_t write_acc(uint8_t id, uint8_t acc);

  /**
  * @brief 设置舵机速度 (Set servo speed)
  * @param id 舵机ID (Servo ID)
  * @param speed 速度值 (Speed value)
  * @attention - 速度范围: -3400 ~ 3400 (Speed range: -3400 ~ 3400)
  * @return 舵机状态信息 (Servo status information)
  */
  ServoStatus_t write_speed(uint8_t id, int16_t speed);

  /**
  * @brief 普通方式设置舵机加速度、速度和位置(Set servo acceleration, speed and position in normal mode)
  * @param id 舵机ID (Servo ID)
  * @param acc 加速度值 (Acceleration value)
  * @attention - 加速度范围: 0 ~ 254 (Acceleration range: 0 ~ 254)
  * @param speed 速度值 (Speed value)
  * @attention - 速度范围: -3400 ~ 3400 (Speed range: -3400 ~ 3400)
  * @param pos 目标位置 (Target position)
  * @attention - 位置范围: -30719 ~ 30719 (Position range: -30719 ~ 30719)
  * @return 舵机状态信息 (Servo status information)
  */
  ServoStatus_t write_pos_ex(uint8_t id, uint8_t acc, int16_t speed, int16_t pos);

  /**
  * @brief 设置舵机PWM速度 (Set servo PWM speed)
  * @param id 舵机ID (Servo ID)
  * @param speed PWM速度值 (PWM speed value)
  * @attention - 速度范围: -1000 ~ 1000 (Speed range: -1000 ~ 1000)
  * @return 舵机状态信息 (Servo status information)
  */
  ServoStatus_t write_pwm_speed(uint8_t id, int16_t speed);

  /**
  * @brief 设置舵机最大力矩 (Set servo maximum torque)
  * @param id 舵机ID (Servo ID)
  * @param torque 最大力矩值 (Maximum torque value)
  * @attention - 最大力矩范围: 0 ~ 1000 (Maximum torque range: 0 ~ 1000)
  * @return 舵机状态信息 (Servo status information)
  */
  ServoStatus_t write_max_torque(uint8_t id, uint16_t torque);

  /**
  * @brief 寄存器方式设置舵机位置 (Register set servo position)
  * @param id 舵机ID (Servo ID)
  * @param pos 目标位置 (Target position)
  * @attention - 位置范围: -30719 ~ 30719 (Position range: -30719 ~ 30719)
  * @param acc 加速度值 (Acceleration value)
  * @attention - 加速度范围: 0 ~ 254 (Acceleration range: 0 ~ 254)
  * @param speed 速度值 (Speed value)
  * @attention - 速度范围: -3400 ~ 3400 (Speed range: -3400 ~ 3400)
  * @return 舵机状态信息 (Servo status information)
  */
  ServoStatus_t write_reg_pos_ex(uint8_t id, uint8_t acc, int16_t speed, int16_t pos);

  /**
  * @brief 同步设置多个舵机位置、加速度和速度 (Sync set multiple servos position, acceleration and speed)
  * @param data 单个舵机需要设置的加速度、速度和位置 (Acceleration, speed and position for each servo)
  * @attention
  * - 每个舵机的数据需要遵循以下排列方式 (Each data needs to be arranged in the following ordern the following format)
  * - 数据格式为：[需要控制的舵机ID号, 加速度, 速度, 位置](format: [id, acc, speed, pos])
  * @param id_num 总共需要设置的舵机总数 (Total number of servos to be set)
  * @return 舵机状态信息 (Servo status information)
  */
  ServoStatus_t sync_write_pos_ex(int16_t (*data)[4], uint8_t id_num);

  /**
  * @brief 同步读取多个舵机当前位置、速度、负载、电压和温度 (Sync read multiple servos current position, speed, load, voltage and temperature)
  * - 数据格式为：[舵机ID号, 舵机位置](format: [id, pos])
  * @param data 存储读取到的数据 (Storage for read data)
  * @param id_num 总共需要读取的舵机总数 (Total number of servos to be read)
  * @return 舵机状态信息 (Servo status information)
  */
  ServoStatus_t sync_read_cur_pos_ex(uint8_t *id, uint8_t id_num, int16_t (*data)[5]);

  /**
  * @brief 读取舵机位置偏移 (Read servo position offset)
  * @param id 舵机ID (Servo ID)
  * @param value 位置偏移值指针 (Pointer to position offset value)
  * @return 舵机状态信息 (Servo status information)
  */
  ServoStatus_t read_pos_offset(uint8_t id, int16_t *value);

  /**
  * @brief 读取舵机当前位置 (Read servo current position)
  * @param id 舵机ID (Servo ID)
  * @param pos 位置值指针 (Pointer to position value)
  * @return 舵机状态信息 (Servo status information)
  */
  ServoStatus_t read_pos(uint8_t id, int16_t *pos);

  /**
  * @brief 读取舵机当前速度 (Read servo current speed)
  * @param id 舵机ID (Servo ID)
  * @param speed 速度值指针 (Pointer to speed value)
  * @return 舵机状态信息 (Servo status information)
  */
  ServoStatus_t read_speed(uint8_t id, int16_t *speed);

  /**
  * @brief 读取舵机位置和速度 (Read servo position and speed)
  * @param id 舵机ID (Servo ID)
  * @param pos 位置值指针 (Pointer to position value)
  * @param speed 速度值指针 (Pointer to speed value)
  * @return 舵机状态信息 (Servo status information)
  */
  ServoStatus_t read_pos_speed(uint8_t id, int16_t *pos, int16_t *speed);

  /**
  * @brief 读取舵机温度 (Read servo temperature)
  * @param id 舵机ID (Servo ID)
  * @param temp 温度值 (Temperature value)
  * @return 舵机状态信息 (Servo status information)
  */
  ServoStatus_t read_temperture(uint8_t id, uint8_t *temp);

  /**
  * @brief 读取舵机电压 (Read servo voltage)
  * @param id 舵机ID (Servo ID)
  * @param vol 电压值指针 (Pointer to voltage value)
  * @return 舵机状态信息 (Servo status information)
  */
  ServoStatus_t read_voltage(uint8_t id, uint8_t *vol);

  /**
  * @brief 读取舵机电流 (Read servo current)
  * @param id 舵机ID (Servo ID)
  * @param cur 电流值指针 (Pointer to current value)
  * @return 舵机状态信息 (Servo status information)
  */
  ServoStatus_t read_current(uint8_t id, uint16_t *cur);

  /**
  * @brief 读取舵机负载 (Read servo load)
  * @param id 舵机ID (Servo ID)
  * @param load 负载值指针 (Pointer to load value)
  * @return 舵机状态信息 (Servo status information)
  */
  ServoStatus_t read_load(uint8_t id, int16_t *load);

  /**
  * @brief 读取舵机运动状态 (Read servo moving status)
  * @param id 舵机ID (Servo ID)
  * @param status 状态值指针 (Pointer to status value)
  * @return 舵机状态信息 (Servo status information)
  */
  ServoStatus_t read_moving_status(uint8_t id, uint8_t *status);
    
private:
  uint8_t rx_skip;
  uint8_t endianness;
  uint8_t rx_frame_length;
  uint32_t rx_timeout;
  PacketStatus rx_status;
  PacketTypeDef rx_packet;

  HardwareSerial* uart; 

  uint8_t unpack(void);
  void word2bytes(uint16_t word, uint8_t *bytes_l, uint8_t *bytes_h);
  uint16_t bytes2word(uint8_t *bytes_l, uint8_t *bytes_h);
  uint8_t data_check(const uint8_t buf[], uint8_t len);
  uint8_t tx_frame_write(uint8_t id, uint8_t cmd, const uint8_t *data, uint8_t data_len);
  ServoStatus_t ack(void);
};

#endif
