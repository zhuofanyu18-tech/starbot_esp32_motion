#ifndef __BUJIN_CONTROL_H__
#define __BUJIN_CONTROL_H__

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#define ABS(x) ((x) > 0 ? (x) : -(x))
#define UART_TX_PIN 17 // ESP32发送引脚 (GPIO17)
#define UART_RX_PIN 18 // ESP32接收引脚 (GPIO18)

extern SemaphoreHandle_t motor_mutex;
extern QueueHandle_t motor_cmd_queue;

// 创建电机队列结构体
typedef struct
{
  uint8_t addr;  // ID地址
  uint8_t dir;   // 方向
  uint16_t vel;  // 转速
  uint8_t acc;   // 加速度
  bool snF;      // 多机同步
} MotorCmd_t;

typedef enum
{
  S_VER = 0,    /* 读取固件版本和对应的硬件版本 */
  S_RL = 1,     /* 读取读取相电阻和相电感 */
  S_PID = 2,    /* 读取PID参数 */
  S_VBUS = 3,   /* 读取总线电压 */
  S_CPHA = 5,   /* 读取相电流 */
  S_ENCL = 7,   /* 读取经过线性化校准后的编码器值 */
  S_TPOS = 8,   /* 读取电机目标位置角度 */
  S_VEL = 9,    /* 读取电机实时转速 */
  S_CPOS = 10,  /* 读取电机实时位置角度 */
  S_PERR = 11,  /* 读取电机位置误差角度 */
  S_FLAG = 13,  /* 读取使能/到位/堵转状态标志位 */
  S_Conf = 14,  /* 读取驱动参数 */
  S_State = 15, /* 读取系统状态参数 */
  S_ORG = 16,   /* 读取正在回零/回零失败状态标志位 */
} SysParams_t;

/**********************************************************
*** 注意：每个函数的参数的具体说明，请查阅下方的函数的注释说明
**********************************************************/
void Emm_V5_INIT(void);                                                                                                                                                             // 步进电机初始化
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr);                                                                                                                                     // 将当前位置清零
void Emm_V5_Reset_Clog_Pro(uint8_t addr);                                                                                                                                           // 解除堵转保护
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s);                                                                                                                           // 读取参数
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode);                                                                                                            // 发送命令修改开环/闭环控制模式
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF);                                                                                                                         // 电机使能控制
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);                                                                                            // 速度模式控制
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);                                                                    // 位置模式控制
void Emm_V5_Stop_Now(uint8_t addr, bool snF);                                                                                                                                       // 让电机立即停止运动
void Emm_V5_Synchronous_motion(uint8_t addr);                                                                                                                                       // 触发多机同步开始运动
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF);                                                                                                                                   // 设置单圈回零的零点位置
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF); // 修改回零参数
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);                                                                                                          // 发送命令触发回零
void Emm_V5_Origin_Interrupt(uint8_t addr);                                                                                                                                         // 强制中断并退出回零
void Emm_V5_Receive_Data(uint8_t *rxCmd, uint8_t *rxCount);                                                                                                                         // 返回数据接收函数
float Emm_V5_MotorVel_Get(uint8_t addr);
void Emm_5V_Vel_Set(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);                                                                                                                                                 // 获取电机实时(每分钟)转速
// @brief    RTOS安全的接收数据函数
bool Emm_V5_Receive_Data_NonBlocking(uint8_t *rxCmd, uint8_t *rxCount, TickType_t timeout);
// @brief    获取电机实时(每分钟)转速: (RTOS版本)
float Emm_V5_MotorVel_Get_RTOS(uint8_t addr);
void MotorControlTask(void *pvParameters);
void Emm_5V_Vel_Set_Async(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
#endif