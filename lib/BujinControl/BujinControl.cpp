#include "BujinControl.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// 创建互斥锁和队列
SemaphoreHandle_t motor_mutex = NULL;
QueueHandle_t motor_cmd_queue = NULL;


/**
 * @brief    步进电机初始化
 */
void Emm_V5_INIT(void)
{
  Serial2.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  // 创建互斥锁和队列
  motor_mutex = xSemaphoreCreateMutex();
  motor_cmd_queue = xQueueCreate(10, sizeof(MotorCmd_t));
  Serial.println("步进电机初始化完成");
}

/**
 * @brief    将当前位置清零
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr; // 地址
  cmd[1] = 0x0A; // 功能码
  cmd[2] = 0x6D; // 辅助码
  cmd[3] = 0x6B; // 校验字节

  // 发送命令
  Serial2.write(cmd, 4);
}

/**
 * @brief    解除堵转保护
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Reset_Clog_Pro(uint8_t addr)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr; // 地址
  cmd[1] = 0x0E; // 功能码
  cmd[2] = 0x52; // 辅助码
  cmd[3] = 0x6B; // 校验字节

  // 发送命令
  Serial2.write(cmd, 4);
}

/**
 * @brief    读取系统参数
 * @param    addr  ：电机地址
 * @param    s     ：系统参数类型
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0;
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[i] = addr;
  ++i; // 地址

  switch (s) // 功能码
  {
  case S_VER:
    cmd[i] = 0x1F;
    ++i;
    break;
  case S_RL:
    cmd[i] = 0x20;
    ++i;
    break;
  case S_PID:
    cmd[i] = 0x21;
    ++i;
    break;
  case S_VBUS:
    cmd[i] = 0x24;
    ++i;
    break;
  case S_CPHA:
    cmd[i] = 0x27;
    ++i;
    break;
  case S_ENCL:
    cmd[i] = 0x31;
    ++i;
    break;
  case S_TPOS:
    cmd[i] = 0x33;
    ++i;
    break;
  case S_VEL:
    cmd[i] = 0x35;
    ++i;
    break;
  case S_CPOS:
    cmd[i] = 0x36;
    ++i;
    break;
  case S_PERR:
    cmd[i] = 0x37;
    ++i;
    break;
  case S_FLAG:
    cmd[i] = 0x3A;
    ++i;
    break;
  case S_ORG:
    cmd[i] = 0x3B;
    ++i;
    break;
  case S_Conf:
    cmd[i] = 0x42;
    ++i;
    cmd[i] = 0x6C;
    ++i;
    break;
  case S_State:
    cmd[i] = 0x43;
    ++i;
    cmd[i] = 0x7A;
    ++i;
    break;
  default:
    break;
  }

  cmd[i] = 0x6B;
  ++i; // 校验字节

  // 发送命令
  Serial2.write(cmd, i);
}

/**
 * @brief    修改开环/闭环控制模式
 * @param    addr     ：电机地址
 * @param    svF      ：是否存储标志，false为不存储，true为存储
 * @param    ctrl_mode：控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr;      // 地址
  cmd[1] = 0x46;      // 功能码
  cmd[2] = 0x69;      // 辅助码
  cmd[3] = svF;       // 是否存储标志，false为不存储，true为存储
  cmd[4] = ctrl_mode; // 控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
  cmd[5] = 0x6B;      // 校验字节

  // 发送命令
  Serial2.write(cmd, 6);
}

/**
 * @brief    使能信号控制
 * @param    addr  ：电机地址
 * @param    state ：使能状态    ，true为使能电机，false为关闭电机
 * @param    snF   ：多机同步标志 ，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr;           // 地址
  cmd[1] = 0xF3;           // 功能码
  cmd[2] = 0xAB;           // 辅助码
  cmd[3] = (uint8_t)state; // 使能状态
  cmd[4] = snF;            // 多机同步运动标志
  cmd[5] = 0x6B;           // 校验字节

  // 发送命令
  Serial2.write(cmd, 6);
}

/**
 * @brief    速度模式
 * @param    addr：电机地址
 * @param    dir ：方向       ，0为CW，其余值为CCW
 * @param    vel ：速度       ，范围0 - 5000RPM
 * @param    acc ：加速度     ，范围0 - 255，注意：0是直接启动
 * @param    snF ：多机同步标志，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr;                // 地址
  cmd[1] = 0xF6;                // 功能码
  cmd[2] = dir;                 // 方向
  cmd[3] = (uint8_t)(vel >> 8); // 速度(RPM)高8位字节
  cmd[4] = (uint8_t)(vel >> 0); // 速度(RPM)低8位字节
  cmd[5] = acc;                 // 加速度，注意：0是直接启动
  cmd[6] = snF;                 // 多机同步运动标志
  cmd[7] = 0x6B;                // 校验字节

  // 发送命令
  Serial2.write(cmd, 8);
}

/**
 * @brief    位置模式
 * @param    addr：电机地址
 * @param    dir ：方向        ，0为CW，其余值为CCW
 * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
 * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
 * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
 * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
 * @param    snF ：多机同步标志 ，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr;                 // 地址
  cmd[1] = 0xFD;                 // 功能码
  cmd[2] = dir;                  // 方向
  cmd[3] = (uint8_t)(vel >> 8);  // 速度(RPM)高8位字节
  cmd[4] = (uint8_t)(vel >> 0);  // 速度(RPM)低8位字节
  cmd[5] = acc;                  // 加速度，注意：0是直接启动
  cmd[6] = (uint8_t)(clk >> 24); // 脉冲数(bit24 - bit31)
  cmd[7] = (uint8_t)(clk >> 16); // 脉冲数(bit16 - bit23)
  cmd[8] = (uint8_t)(clk >> 8);  // 脉冲数(bit8  - bit15)
  cmd[9] = (uint8_t)(clk >> 0);  // 脉冲数(bit0  - bit7 )
  cmd[10] = raF;                 // 相位/绝对标志，false为相对运动，true为绝对值运动
  cmd[11] = snF;                 // 多机同步运动标志，false为不启用，true为启用
  cmd[12] = 0x6B;                // 校验字节

  // 发送命令
  Serial2.write(cmd, 13);
}

/**
 * @brief    立即停止（所有控制模式都通用）
 * @param    addr  ：电机地址
 * @param    snF   ：多机同步标志，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 +校验字节
 */
void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr; // 地址
  cmd[1] = 0xFE; // 功能码
  cmd[2] = 0x98; // 辅助码
  cmd[3] = snF;  // 多机同步运动标志
  cmd[4] = 0x6B; // 校验字节

  // 发送命令
  Serial2.write(cmd, 5);
}

/**
 * @brief    多机同步运动
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Synchronous_motion(uint8_t addr)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr; // 地址
  cmd[1] = 0xFF; // 功能码
  cmd[2] = 0x66; // 辅助码
  cmd[3] = 0x6B; // 校验字节

  // 发送命令
  Serial2.write(cmd, 4);
}

/**
 * @brief    设置单圈回零的零点位置
 * @param    addr  ：电机地址
 * @param    svF   ：是否存储标志，false为不存储，true为存储
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr; // 地址
  cmd[1] = 0x93; // 功能码
  cmd[2] = 0x88; // 辅助码
  cmd[3] = svF;  // 是否存储标志，false为不存储，true为存储
  cmd[4] = 0x6B; // 校验字节

  // 发送命令
  Serial2.write(cmd, 5);
}

/**
 * @brief    修改回零参数
 * @param    addr  ：电机地址
 * @param    svF   ：是否存储标志，false为不存储，true为存储
 * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
 * @param    o_dir  ：回零方向，0为CW，其余值为CCW
 * @param    o_vel  ：回零速度，单位：RPM（转/分钟）
 * @param    o_tm   ：回零超时时间，单位：毫秒
 * @param    sl_vel ：无限位碰撞回零检测转速，单位：RPM（转/分钟）
 * @param    sl_ma  ：无限位碰撞回零检测电流，单位：Ma（毫安）
 * @param    sl_ms  ：无限位碰撞回零检测时间，单位：Ms（毫秒）
 * @param    potF   ：上电自动触发回零，false为不使能，true为使能
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
  uint8_t cmd[32] = {0};

  // 装载命令
  cmd[0] = addr;                    // 地址
  cmd[1] = 0x4C;                    // 功能码
  cmd[2] = 0xAE;                    // 辅助码
  cmd[3] = svF;                     // 是否存储标志，false为不存储，true为存储
  cmd[4] = o_mode;                  // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  cmd[5] = o_dir;                   // 回零方向
  cmd[6] = (uint8_t)(o_vel >> 8);   // 回零速度(RPM)高8位字节
  cmd[7] = (uint8_t)(o_vel >> 0);   // 回零速度(RPM)低8位字节
  cmd[8] = (uint8_t)(o_tm >> 24);   // 回零超时时间(bit24 - bit31)
  cmd[9] = (uint8_t)(o_tm >> 16);   // 回零超时时间(bit16 - bit23)
  cmd[10] = (uint8_t)(o_tm >> 8);   // 回零超时时间(bit8  - bit15)
  cmd[11] = (uint8_t)(o_tm >> 0);   // 回零超时时间(bit0  - bit7 )
  cmd[12] = (uint8_t)(sl_vel >> 8); // 无限位碰撞回零检测转速(RPM)高8位字节
  cmd[13] = (uint8_t)(sl_vel >> 0); // 无限位碰撞回零检测转速(RPM)低8位字节
  cmd[14] = (uint8_t)(sl_ma >> 8);  // 无限位碰撞回零检测电流(Ma)高8位字节
  cmd[15] = (uint8_t)(sl_ma >> 0);  // 无限位碰撞回零检测电流(Ma)低8位字节
  cmd[16] = (uint8_t)(sl_ms >> 8);  // 无限位碰撞回零检测时间(Ms)高8位字节
  cmd[17] = (uint8_t)(sl_ms >> 0);  // 无限位碰撞回零检测时间(Ms)低8位字节
  cmd[18] = potF;                   // 上电自动触发回零，false为不使能，true为使能
  cmd[19] = 0x6B;                   // 校验字节

  // 发送命令
  Serial2.write(cmd, 20);
}

/**
 * @brief    触发回零
 * @param    addr   ：电机地址
 * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
 * @param    snF   ：多机同步标志，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr;   // 地址
  cmd[1] = 0x9A;   // 功能码
  cmd[2] = o_mode; // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  cmd[3] = snF;    // 多机同步运动标志，false为不启用，true为启用
  cmd[4] = 0x6B;   // 校验字节

  // 发送命令
  Serial2.write(cmd, 5);
}

/**
 * @brief    强制中断并退出回零
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Origin_Interrupt(uint8_t addr)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr; // 地址
  cmd[1] = 0x9C; // 功能码
  cmd[2] = 0x48; // 辅助码
  cmd[3] = 0x6B; // 校验字节

  // 发送命令
  Serial2.write(cmd, 4);
}

/**
 * @brief    接收数据
 * @param    rxCmd   : 接收到的数据缓存在该数组
 * @param    rxCount : 接收到的数据长度
 * @retval   无
 */
void Emm_V5_Receive_Data(uint8_t *rxCmd, uint8_t *rxCount)
{
  int i = 0;
  unsigned long startTime = millis();
  unsigned long lastDataTime = startTime;
  
  // 初始化接收计数
  *rxCount = 0;
  
  // 设置最大接收时间为200ms
  while (millis() - startTime < 200)
  {
    if (Serial2.available() > 0)
    {
      if (i < 128) // 防止数组溢出
      {
        rxCmd[i++] = Serial2.read();
        lastDataTime = millis();
      }
    }
    else
    {
      // 如果10ms内没有新数据，认为一帧数据接收完成
      if (millis() - lastDataTime > 10 && i > 0)
      {
        break;
      }
    }
  }
  
  *rxCount = i;
}

/**
 * @brief    获取电机实时(每分钟)转速
 * @param    addr：电机地址
 * @retval   vel 每分钟转速
 */
float Emm_V5_MotorVel_Get(uint8_t addr)
{
  float vel = -1.0f; // 使用负数表示获取失败
  uint8_t rxCmd[128] = {0};
  uint8_t rxCount = 0;
  
  // 发送读取速度命令
  Emm_V5_Read_Sys_Params(addr, S_VEL);
  
  // 接收速度数据
  Emm_V5_Receive_Data(rxCmd, &rxCount);
  
  // 验证数据有效性
  if (rxCount == 6 && rxCmd[0] == addr && rxCmd[1] == 0x35)
  {
    // 正确解析速度数据
    vel = static_cast<float>((static_cast<uint16_t>(rxCmd[3]) << 8) | 
                             static_cast<uint16_t>(rxCmd[4]));
  }
  
  return vel;
}

/**
 * @brief    设置电机实时(每分钟)转速
 * @param    addr：电机地址
 * @param    dir ：方向       ，0为CW，其余值为CCW
 * @param    vel ：速度       ，范围0 - 5000RPM
 * @param    acc ：加速度     ，范围0 - 255，注意：0是直接启动
 * @param    snF ：多机同步标志，false为不启用，true为启用
 * @retval   无
 */
void Emm_5V_Vel_Set(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  uint8_t rxCmd[128] = {0};
  uint8_t rxCount = 0;
  
  // 发送速度控制命令
  Emm_V5_Vel_Control(addr, dir, vel, acc, snF);
  // 接收响应数据
  Emm_V5_Receive_Data(rxCmd, &rxCount);
  
  // 增加 rxCount 有效性检查，防止数组越界
  if (rxCount > 0 && rxCmd[rxCount - 1] == 0x6B)
  {
    Serial.println("电机addr速度设置成功"); // 命令执行成功
  }
  
  else
  {
    // 失败 ，打印错误信息
    Serial.print("命令执行失败，电机地址: ");
    Serial.println(addr);
    // 再次发送速度控制命令
    Emm_V5_Vel_Control(addr, dir, vel, acc, snF);
    // 接收响应数据
    Emm_V5_Receive_Data(rxCmd, &rxCount);
  }
}

/**
 * @brief    RTOS安全的接收数据函数
 * @param    timeout 获得的时间
 * @retval   无
 */
bool Emm_V5_Receive_Data_NonBlocking(uint8_t *rxCmd, uint8_t *rxCount, TickType_t timeout)
{
    int i = 0;
    *rxCount = 0;
    
    // 设置接收超时
    TickType_t startTime = xTaskGetTickCount();
    
    while ((xTaskGetTickCount() - startTime) < timeout)
    {
        if (Serial2.available() > 0)
        {
            if (i < 128)
            {
                rxCmd[i++] = Serial2.read();
                // 收到数据后重置超时
                startTime = xTaskGetTickCount();
            }
        }
        else
        {
            // 如果10ms内没有新数据，认为一帧数据接收完成
            if ((xTaskGetTickCount() - startTime) > pdMS_TO_TICKS(10) && i > 0)
            {
                break;
            }
            taskYIELD(); // 让出CPU给其他任务
        }
    }
    
    *rxCount = i;
    return (i > 0);
}

/**
 * @brief    获取电机实时(每分钟)转速: (RTOS版本)
 * @param    addr：电机地址
 * @retval   vel 每分钟转速
 */
float Emm_V5_MotorVel_Get_RTOS(uint8_t addr)
{
  float vel = -1.0f; // 使用负数表示获取失败
  uint8_t rxCmd[128] = {0};
  uint8_t rxCount = 0;
  
  // 获取互斥锁，保护串口权限
  if(xSemaphoreTake(motor_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
  {
    // 发送读取速度命令
    Emm_V5_Read_Sys_Params(addr, S_VEL);
    
    // 接收速度数据
    if(Emm_V5_Receive_Data_NonBlocking(rxCmd, &rxCount, pdMS_TO_TICKS(100)))
    {
      // 验证数据有效性
      if (rxCount == 6 && rxCmd[0] == addr && rxCmd[1] == 0x35)
      {
        // 正确解析速度数据
        vel = static_cast<float>((static_cast<uint16_t>(rxCmd[3]) << 8) | 
                                 static_cast<uint16_t>(rxCmd[4]));
      }
    }
    
    // 释放互斥锁
    xSemaphoreGive(motor_mutex);
  }
  
  return vel;
}

/**
 * @brief    电机控制任务（在独立的任务中执行）,如果队列有任务,则执行任务,否则阻塞等待
 */
void MotorControlTask(void *pvParameters)
{
    MotorCmd_t cmd;
    
    for (;;)
    {
        // 从队列中获取命令（阻塞等待）
        // 0：立刻返回pdFALSE;  portMAX_DELAY：阻塞等待直到有数据，期间不占用CPU资源
        if (xQueueReceive(motor_cmd_queue, &cmd, portMAX_DELAY) == pdTRUE)
        {
            // 获取互斥锁保护串口
            if (xSemaphoreTake(motor_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                uint8_t rxCmd[128] = {0};
                uint8_t rxCount = 0;
                
                // 发送速度控制命令
                Emm_V5_Vel_Control(cmd.addr, cmd.dir, cmd.vel, cmd.acc, cmd.snF);
                // cmd.vel 为无符号整数，使用 %u 并强制转换以避免 printf 格式错误
                Serial.printf("电机速度: %u, 多机同步：%d\n", (unsigned)cmd.vel, (int)cmd.snF);

                // 非阻塞接收响应
                if (Emm_V5_Receive_Data_NonBlocking(rxCmd, &rxCount, pdMS_TO_TICKS(50)))
                {
                    if (rxCount > 0 && rxCmd[rxCount - 1] == 0x6B)
                    {
                        // 命令执行成功
                        Serial.printf("电机地址 %d 速度设置成功\n", cmd.addr);
                    }
                }
                
                // if(cmd.addr == 4)
                // {
                //   Emm_V5_Synchronous_motion(0);
                //   Serial.println("duojitongbu");
                // }

                // 释放互斥锁
                xSemaphoreGive(motor_mutex);
            }
            
            // 让出CPU，避免任务饥饿
            taskYIELD();
        }
    }
}


/**
 * @brief    异步设置电机速度（非阻塞）
 */
void Emm_5V_Vel_Set_Async(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  MotorCmd_t cmd = {addr, dir, vel, acc, snF};
    
  // 发送命令到队列（如果队列满则等待10ms）
  if (xQueueSend(motor_cmd_queue, &cmd, pdMS_TO_TICKS(10)) != pdTRUE)
  {
    Serial.printf("警告: 电机命令队列已满，地址: %d\n", addr);
  }
}
