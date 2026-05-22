# ESP32 + MoveIt2 新手版说明

这个目录现在使用 `main.cpp` 作为唯一入口，流程是：

1. 初始化舵机串口和 micro-ROS 串口  
2. 订阅 MoveIt2 发来的轨迹话题  
3. 按标定表把 ROS 角度换算成舵机值  
4. 发布 `/joint_states`

## 话题

- 机械臂轨迹：`/arm_controller/joint_trajectory`
- 夹爪轨迹：`/gripper_controller/joint_trajectory`
- 关节状态：`/joint_states`

## ROS角度 -> 舵机值 标定


说明：

- `joint1/joint2/joint4/joint5` 使用“分段线性”映射（以 0 点为分段）。
- `joint3` 和夹爪使用“两点线性”映射。
- 超出范围会自动夹紧到最近端点，避免越界。


#include <Arduino.h>
#include <Esp32PcntEncoder.h>
#include <PwmControl.h>
#include <PidController.h>
#include <Kinematics.h>

// 引入ROS相关头文件
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <nav_msgs/msg/odometry.h>//里程计消息接口
#include <geometry_msgs/msg/twist.h>//速度控制消息接口
#include <micro_ros_utilities/string_utilities.h>
#include <cmath> // 用于数学计算
#include <freertos/semphr.h> // 用于互斥锁

// 全局互斥锁，保护共享变量
SemaphoreHandle_t g_mutex = NULL;

// 全局对象
PWMControl motor;
Esp32PcntEncoder encoders[4]; // 创建一个数组用于存储四个编码器
PidController pid_controller[4];
Kinematics kinematics;

// ROS相关对象
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;
rcl_node_t node;
rcl_publisher_t pub_odom;
nav_msgs__msg__Odometry msg_odom;
rcl_timer_t timer;
rcl_subscription_t sub_cmd_vel;
geometry_msgs__msg__Twist msg_cmd_vel;

// 编码器参数
const int ENCODER_TICKS_PER_REV = 14000; // 每转编码器脉冲数14000（2）、7441（1）
const float WHEEL_DIAMETER = 125.0; // 轮子直径（mm）
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * 3.1415926535; // 轮子周长（mm）
const float MM_PER_TICK = WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REV; // 每脉冲距离（mm）

// 全局变量
int64_t last_ticks[4] = {0, 0, 0, 0};
int64_t last_update_time = 0;
float current_speed[4] = {0.0, 0.0, 0.0, 0.0}; // 当前速度（mm/s）

float target_linear_speed = 0.0; // 目标线速度（mm/s）
float target_angular_speed = 0.0; // 目标角速度（rad/s）


// 每个轮子独立的PID参数（顺序：前左、前右、后左、后右）
// 优化后参数：增大Kp和Kd提高停止响应速度，减小Ki避免积分累积
const float kp[4] = {1.0, 1.0, 1.0, 1.0};       // 增大比例系数，提高响应速度
const float ki[4] = {0.3, 0.3, 0.3, 0.3};   // 减小积分系数，避免积分累积导致滞后
const float kd[4] = {0.5, 0.5, 0.5, 0.5};   // 增大微分系数，加强制动效果



// 编码器数据采集任务（最高优先级，仅负责速度计算）
void encoder_update_task(void* args) {
    while (true) {
        int64_t current_time = millis();
        
        // 获取互斥锁保护共享变量
        if (g_mutex != NULL) {
            xSemaphoreTake(g_mutex, portMAX_DELAY);
        }
        
        int64_t dt = current_time - last_update_time;
        
        if (dt > 0) {
            // 计算速度
            for (int i = 0; i < 4; i++) {
                int32_t current_ticks = encoders[i].getTicks();
                int32_t delta_ticks = current_ticks - last_ticks[i];
                
                // 计算速度：(脉冲数 * 每脉冲距离) / 时间
                current_speed[i] = (delta_ticks * MM_PER_TICK) / (dt / 1000.0);
                last_ticks[i] = current_ticks;
            }
        }
        
        last_update_time = current_time;
        
        // 释放互斥锁
        if (g_mutex != NULL) {
            xSemaphoreGive(g_mutex);
        }
        
        // 使用非阻塞延迟
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// PID控制任务（高优先级，仅负责电机闭环控制）
void pid_control_task(void* args) {
    while (true) {
        float local_speed[4];
        
        // 获取互斥锁读取共享变量（最小化临界区）
        if (g_mutex != NULL) {
            xSemaphoreTake(g_mutex, portMAX_DELAY);
            for (int i = 0; i < 4; i++) {
                local_speed[i] = current_speed[i];
            }
            xSemaphoreGive(g_mutex);
        } else {
            for (int i = 0; i < 4; i++) {
                local_speed[i] = current_speed[i];
            }
        }
        
        // 运行PID控制（带死区处理和积分重置）
        for (int i = 0; i < 4; i++) {
            float pid_output = pid_controller[i].update(local_speed[i]);
            
            // 死区处理：当目标速度为0且当前速度很小时，直接停止电机
            // 避免低速抖动和爬行现象，并重置PID积分项
            float target_speed = pid_controller[i].get_target();
            if (target_speed == 0.0 && fabs(local_speed[i]) < 30.0) {
                pid_output = 0;  // 直接停转
                pid_controller[i].reset();  // 重置PID积分项，避免下次启动冲击
            }
            
            // 应用PID输出到电机（PID内部已有限制）
            motor.updateMotorSpeed(i, pid_output);     
        }
        
        // 使用非阻塞延迟
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// 里程计更新任务（较低优先级，50ms周期）
void odometry_update_task(void* args) {
    while (true) {
        int64_t current_time = millis();
        int32_t ticks[4];
        
        // 获取互斥锁读取编码器数据（与encoder_update_task同步）
        if (g_mutex != NULL) {
            xSemaphoreTake(g_mutex, portMAX_DELAY);
            for (int i = 0; i < 4; i++) {
                ticks[i] = last_ticks[i]; // 使用已缓存的编码器值
            }
            xSemaphoreGive(g_mutex);
        } else {
            for (int i = 0; i < 4; i++) {
                ticks[i] = last_ticks[i];
            }
        }
        
        // 更新运动学数据（使用缓存的编码器数据，保证与速度计算同步）
        kinematics.update_motor_speed(current_time, 
                                     ticks[0], // front_left
                                     ticks[1], // front_right
                                     ticks[2], // rear_left
                                     ticks[3]); // rear_right
        
        // 使用非阻塞延迟
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}








// 定时器的回调函数
void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    // 完成里程计的发布
    odom_t odom = kinematics.get_odom();//获取当前的里程计信息
    int64_t stamp = rmw_uros_epoch_millis();//获取当前时间戳，单位为毫秒
    msg_odom.header.stamp.sec = static_cast<int32_t>(stamp / 1000);//秒部分
    msg_odom.header.stamp.nanosec = static_cast<int32_t>((stamp % 1000) * 1000000);//纳秒部分
    msg_odom.pose.pose.position.x = odom.x;
    msg_odom.pose.pose.position.y = odom.y;
    msg_odom.pose.pose.orientation.w = cos(odom.angle/2.0);
    msg_odom.pose.pose.orientation.x = 0;
    msg_odom.pose.pose.orientation.y = 0;
    msg_odom.pose.pose.orientation.z = sin(odom.angle/2.0);
    msg_odom.twist.twist.linear.x = odom.linear_speed;
    msg_odom.twist.twist.linear.y = 0.0;
    msg_odom.twist.twist.angular.z = odom.angle_speed;
    //发布里程计消息，把数据发送出去
    // 发布里程计消息
    rcl_ret_t ret = rcl_publish(&pub_odom, &msg_odom, NULL);
    (void)ret; // 消除未使用返回值的警告
}

// cmd_vel 订阅回调函数
void cmd_vel_callback(const void* msgin) {
    // 转换消息指针
    const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*) msgin;
    
    // 提取线速度和角速度（ROS的单位是m/s和rad/s，转换为mm/s）
    // 注意：target_linear_speed和target_angular_speed未使用互斥锁保护，仅用于调试/记录
    target_linear_speed = msg->linear.x * 1000.0; // 转换为mm/s
    target_angular_speed = msg->angular.z; // 角速度保持rad/s
    
    // 计算逆运动学，获取各轮目标速度
    float front_left_speed, front_right_speed, rear_left_speed, rear_right_speed;
    kinematics.kinematics_inverse(target_linear_speed, target_angular_speed, 
                                 front_left_speed, front_right_speed, 
                                 rear_left_speed, rear_right_speed);

    // 更新PID目标值（使用互斥锁保护，避免与pid_control_task竞争）
    if (g_mutex != NULL) {
        xSemaphoreTake(g_mutex, portMAX_DELAY);
    }
    pid_controller[0].update_target(front_left_speed);   // front_left
    pid_controller[1].update_target(front_right_speed);  // front_right
    pid_controller[2].update_target(rear_left_speed);    // rear_left
    pid_controller[3].update_target(rear_right_speed);   // rear_right
    if (g_mutex != NULL) {
        xSemaphoreGive(g_mutex);
    }
}

// 单独创建一个任务，单独运行micro-ros相当于一个单独的线程
void micro_ros_task(void* args ) {
    //1.设置传输协议并延迟一段时间等待设置的完成
    set_microros_serial_transports(Serial);
    delay(2000); // 等待2秒，确保设置完成

    //2.初始化内存分配器
    allocator = rcl_get_default_allocator();                                    //获取默认的内存分配器
    //3.初始化支持
    rclc_support_init(&support, 0, NULL, &allocator);                           //初始化支持，0表示没有参数，NULL表示没有参数，&allocator表示使用默认的内存分配器
    //4.初始化节点
    rclc_node_init_default(&node,"lajixiaoche","",&support);                    //初始化节点，"lajixiaoche"表示节点名称，""表示命名空间，&support表示使用默认的支持  
    //5.初始化执行器
    unsigned int num_handles = 3;                                               //订阅和计时器回调的数量，需要改的参数
    rclc_executor_init(&executor, &support.context, num_handles, &allocator);   //初始化执行器，4表示最多可以添加4个回调函数，&allocator表示使用默认的内存分配器
    
    //初始化msg
    msg_odom.header.frame_id = micro_ros_string_utilities_set(msg_odom.header.frame_id, "/wheel_odom");
    msg_odom.child_frame_id = micro_ros_string_utilities_set(msg_odom.child_frame_id, "base_footprint");
    //初始化定时器和发布者
    rclc_publisher_init_best_effort(&pub_odom, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/wheel_odom");//初始化发布者，&node表示节点对象，nav_msgs__msg__Odometry表示消息类型，"/wheel_odom"表示话题名称

    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), timer_callback);//初始化定时器，&support表示使用默认的支持，RCL_MS_TO_NS(50)表示定时器周期为50毫秒，timer_callback表示回调函数
    rclc_executor_add_timer(&executor, &timer);//将定时器添加到执行器中，&executor表示执行器对象，&timer表示定时器对象
    
    // 初始化cmd_vel订阅者
    rclc_subscription_init_default(&sub_cmd_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");
    rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel, cmd_vel_callback, ON_NEW_DATA);
    // //时间同步
    // while (!rmw_uros_epoch_synchronized()) {
    //     rmw_uros_sync_session(1000);
    //     delay(10);
    // }
    //时间同步（带超时机制和重试）
    int sync_timeout = 10000;  // 增加超时时间到10秒
    int sync_start = millis();
    int retry_count = 0;
    while (!rmw_uros_epoch_synchronized()) {
        rmw_uros_sync_session(100);
        delay(1);
        if (millis() - sync_start > sync_timeout) {
            // 超时后重新尝试初始化
            retry_count++;
            if (retry_count < 3) {
                // 重新初始化支持
                rclc_support_fini(&support);
                rclc_support_init(&support, 0, NULL, &allocator);
                sync_start = millis();  // 重置超时计时器
            } else {
                // 重试3次后继续执行
                break;
            }
        }
    }
    
    //7.循环执行器
    rclc_executor_spin(&executor);//循环执行器，等待订阅和计时器回调的触发
}



void setup()
{
    // 初始化串口（仅用于ROS通信）
    Serial.begin(115200);

    // 初始化电机

    // motor.attachMotor(0, GPIO_NUM_42, GPIO_NUM_40, GPIO_NUM_41);
    // motor.attachMotor(1, GPIO_NUM_39, GPIO_NUM_37, GPIO_NUM_38);
    // motor.attachMotor(2, GPIO_NUM_4, GPIO_NUM_6, GPIO_NUM_5);
    // motor.attachMotor(3, GPIO_NUM_16, GPIO_NUM_7, GPIO_NUM_15);
    motor.attachMotor(0, GPIO_NUM_42, GPIO_NUM_41, GPIO_NUM_40);
    motor.attachMotor(1, GPIO_NUM_37, GPIO_NUM_39, GPIO_NUM_38);
    motor.attachMotor(2, GPIO_NUM_4, GPIO_NUM_6, GPIO_NUM_5);
    motor.attachMotor(3, GPIO_NUM_16, GPIO_NUM_7, GPIO_NUM_15);

    // 设置电机速度为0（简化为循环）
    for (int i = 0; i < 4; i++) {
        motor.updateMotorSpeed(i, 0);
    }

    // 初始化编码器
    // encoders[0].init(0, 20, 21); // 前左轮编码器
    // encoders[0].reset();
    // encoders[1].init(1, 36, 35); // 前右轮编码器
    // encoders[1].reset();
    // encoders[2].init(2, 11, 10); // 后左轮编码器
    // encoders[2].reset();
    // encoders[3].init(3, 12, 13); // 后右轮编码器
    // encoders[3].reset();

    encoders[0].init(0, 21, 20); // 前左轮编码器
    encoders[0].reset();
    encoders[1].init(1, 36, 35); // 前右轮编码器
    encoders[1].reset();
    encoders[2].init(2, 11, 10); // 后左轮编码器
    encoders[2].reset();
    encoders[3].init(3, 13, 12); // 后右轮编码器
    encoders[3].reset();

    // 初始化PID控制器（每个轮子使用独立的PID参数）
    for (int i = 0; i < 4; i++) {
        pid_controller[i].update_pid(kp[i], ki[i], kd[i]);
        pid_controller[i].out_limit(-1000, 1000);
        pid_controller[i].update_target(0.0);
    }

    // 初始化运动学参数
    kinematics.set_wheel_distance(370.0); // 单位：mm
    // 使用已定义的每脉冲距离
    kinematics.set_motor_param(0, MM_PER_TICK); // front_left
    kinematics.set_motor_param(1, MM_PER_TICK); // front_right
    kinematics.set_motor_param(2, MM_PER_TICK); // rear_left
    kinematics.set_motor_param(3, MM_PER_TICK); // rear_right

    // 初始化变量
    last_update_time = millis();

    // 创建互斥锁保护共享变量
    g_mutex = xSemaphoreCreateMutex();
    
    // 创建编码器采集任务（最高优先级，保证数据采集实时性）
    xTaskCreate(encoder_update_task, "encoder_update_task", 2048, NULL, 4, NULL);
    
    // 创建PID控制任务（高优先级，保证控制实时性）
    xTaskCreate(pid_control_task, "pid_control_task", 2048, NULL, 3, NULL);
    
    // 创建里程计更新任务（较低优先级）
    xTaskCreate(odometry_update_task, "odometry_update_task", 2048, NULL, 2, NULL);
    
    // 创建micro-ros任务（最低优先级）
    xTaskCreate(micro_ros_task, "micro_ros_task", 16384, NULL, 1, NULL);
}

void loop()
{
    // 主任务可以为空，或者用于低优先级的后台任务
}

刚刚网络卡了，然后我把代码的GPIO口已经设置好了，话题名字也设置好了。请你帮我把README.md的优秀的代码给我总结一下，分析我目前的代码需要如何修正呢