# 接收ROS2发布的/cmd_vel话题，然后实现SLAM的底盘移动
## 功能描述
- 该节点订阅ROS2的/cmd_vel话题，接收速度指令
- 根据接收到的速度指令，调用底盘运动学模型计算四个轮子的速度
- 将计算得到的速度发送给底盘执行


<!-- 电机测试可用代码 -->
#include <Arduino.h>
#include "BujinControl.h"

int flag = 0;

void setup()
{
    Serial.begin(115200);
    while (!Serial) {}
    
    // 初始化串口和电机
    Emm_V5_INIT();
    Emm_V5_En_Control(0, true, false);
    
    // 等待足够的时间让电机初始化完成
    delay(2000);
    Serial.println("电机初始化完成");
    
}

void loop()
{
    float motorSpeed = 0.0f;
    if (flag == 0)
    {
        Emm_5V_Vel_Set(1, 0, 200, 0, false);
        flag = 1;
    }
    else
    {
        Emm_5V_Vel_Set(1, 0, 100, 0, false);
        flag = 0;
    }// 设置电机1的速度为100RPM
    
    delay(2000);
    // 获取电机1的实时速度
    motorSpeed = Emm_V5_MotorVel_Get(1);
    
    // 根据返回值判断是否获取成功
    if (motorSpeed >= 0)
    {
        Serial.print("dianjisudu: ");
        Serial.println(motorSpeed);
    }
    else
    {
        Serial.println("获取电机速度失败");
    }
    
    Serial.println("OK3");
    delay(3000);
}


## 最终使用版本

#include <Arduino.h>
#include "MLTrol.h"
// 引入micro-ROS和wifi相关的库
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>              // 速度消息接口
#include <nav_msgs/msg/odometry.h>                // 里程计消息接口 
#include <micro_ros_utilities/string_utilities.h> //引入字符串内存分配初始化工具

// 底盘参数
#define CAR_LENGTH 0.30f     // 车身长度(m)
#define CAR_WIDTH 0.25f      // 车身宽度(m)
#define WHEEL_DIAMETER 0.05f // 轮子直径(m)

// 创建运动学对象，用来计算里程计和速度正逆解
MecanumKinematics kinematics(CAR_LENGTH, CAR_WIDTH, WHEEL_DIAMETER);
WheelSpeeds speeds; // 结构体用来储存四个轮子的速度

// 预设速度参数（单位：m/s 和 rad/s）
#define TEST_VEL_X 0.25f  // X方向线速度
#define TEST_VEL_Y 0.2f   // Y方向线速度
#define TEST_ANGULAR 0.0f // 角速度（约0.0度/秒）

// 声明一些相关的结构体对象
rcl_allocator_t allocator;             // 内存分配器，用于动态内存分配管理
rclc_support_t support;                // 用于存储时钟，内存分配器和上下文，用于提供支持
rclc_executor_t executor;              // 执行器，用于管理订阅和计时器回调的执行
rcl_node_t node;                       // 节点
rcl_subscription_t subscriber_cmd_vel; // 创建一个速度订阅者
geometry_msgs__msg__Twist sub_msg;     // 用来储存订阅到的速度信息
rcl_publisher_t publisher_odom;        // 创建一个里程计发布者 (已注释)
nav_msgs__msg__Odometry msg_odom;      // 用来储存发布的里程计信息 (已注释)
rcl_timer_t timer_odom;                // 创建一个定时器，用来定时发布里程计信息 (已注释)

// 当收到 /cmd_vel 话题的新消息时，此函数会被自动调用
void twist_callback(const void *msg_in)
{
    // 1. 将传来的消息强制转换为 Twist 类型
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msg_in;

    // 2. 从消息中提取线速度和角速度
    float linear_x = msg->linear.x; // 前进/后退速度 (m/s) --> 前进为正
    float linear_y = msg->linear.y; // 左/右侧移速度 (m/s) --> 左边为正
    float angular_z = msg->angular.z; // 旋转速度 (rad/s) --> 逆时针为正

    // 3. 根据 linear_x 和 angular_z 计算左右轮的目标速度
    speeds = kinematics.inverseKinematics(linear_x, linear_y, angular_z);

    // 4. 将计算出的速度值发送给电机驱动器
    // Serial.print("轮子速度(rad/s): ");
    // Serial.print("FL=");
    // Serial.print(speeds.front_left);
    // Serial.print(", FR=");
    // Serial.print(speeds.front_right);
    // Serial.print(", RL=");
    // Serial.print(speeds.rear_left);
    // Serial.print(", RR=");
    // Serial.println(speeds.rear_right);

    // 5. 调用电机控制函数，将速度发送给电机
    kinematics.setMotorSpeed(speeds);
}

void micro_ros_task(void *args)
{
    //==============================================================
    // 步骤 1: 设置网络传输
    //==============================================================
    char wifi_name[] = "ROS";             // Wi-Fi SSID
    char wifi_password[] = "12345678";    // Wi-Fi 密码
    uint16_t agent_port = 8888;           // micro-ROS Agent 端口
    IPAddress agent_ip;                   // micro-ROS Agent 的 IP 地址
    agent_ip.fromString("192.168.31.50"); // 将字符串格式的IP转换为对象 192.168.31.50

    // 核心函数：设置使用WiFi与micro-ROS代理通信
    // 此函数会连接指定WiFi，并尝试连接到运行在 agent_ip:agent_port 上的Agent
    set_microros_wifi_transports(wifi_name, wifi_password, agent_ip, agent_port);

    // 等待网络连接和初始化稳定下来
    delay(2000);

    //==============================================================
    // 步骤 2: 初始化 micro-ROS 核心组件
    //==============================================================
    // 2.1 获取默认的内存分配器
    // micro-ROS 需要在设备上动态分配内存（例如创建消息时），这个分配器就是用来管理这些操作的。
    allocator = rcl_get_default_allocator();

    // 2.2 初始化支持对象 (support)
    // 这个对象包含了ROS通信所需的上下文信息（context），是初始化任何其他组件（节点、发布者等）的前提。
    rclc_support_init(&support, 0, NULL, &allocator);

    // 2.3 创建节点
    // 你的设备在ROS网络中需要一个身份，这就是节点。名字叫 "fishbot_motion_control"。
    rclc_node_init_default(&node, "fishbot_motion_control", "", &support);

    //==============================================================
    // 步骤 3: 创建订阅者 (Subscriber)
    //==============================================================
    // 3.2 初始化订阅者
    // - 关联到我们刚创建的节点 (&node)
    // - 指定消息类型：geometry_msgs::msg::Twist
    // - 指定要订阅的话题名："/cmd_vel"
    rclc_subscription_init_best_effort(
        &subscriber_cmd_vel,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel");

    //==============================================================
    // 步骤 4: 初始化执行器 (Executor) 并添加订阅
    //==============================================================
    // 4.2 初始化执行器
    // 参数 `1` 表示这个执行器只管理 1 个句柄（即我们只有一个订阅者）
    rclc_executor_init(&executor, &support.context, 1, &allocator);

    // 4.4 将订阅者添加到执行器中
    // - 告诉执行器管理哪个订阅者 (&subscriber_cmd_vel)
    // - 告诉它收到消息后把数据存到哪个变量里 (&sub_msg)
    // - 告诉它当收到新消息时，应该调用哪个函数 (twist_callback)
    // - ON_NEW_DATA 表示一有新数据就触发回调
    rclc_executor_add_subscription(&executor, &subscriber_cmd_vel, &sub_msg, &twist_callback, ON_NEW_DATA);

    //==============================================================
    // 步骤 5: 时间同步
    //==============================================================
    // micro-ROS 设备需要与ROS主机同步时间。
    // 这个循环会持续尝试同步，直到成功为止。
    while (!rmw_uros_epoch_synchronized())
    {
        rmw_uros_sync_session(1000); // 尝试同步会话，超时时间1000ms
        delay(100);                  // 等待100ms后重试
    }

    //==============================================================
    // 步骤 6: 启动主循环
    //==============================================================
    // 这是最终的核心循环。执行器会在这里持续运行：
    // 1. 检查网络连接
    // 2. 监听是否有新的 /cmd_vel 消息
    // 3. 一旦收到，立即调用 twist_callback 函数，你可以在那个函数里控制电机运动。
    // 这个函数是阻塞的，会一直运行在这里。
    rclc_executor_spin(&executor);

    // 注意：如果因为网络错误等原因退出了上面的循环，应该在这里释放所有资源（虽然通常不会执行到这里）。
}

void setup()
{
    // 初始化串口（用于调试输出）
    Serial.begin(115200);
    while (!Serial)
    {
    } // 等待串口连接
    // 初始化电机
    kinematics.MotorInit();
    Serial.println("电机初始化完成");
    xTaskCreate(micro_ros_task, "micro_ros", 10240, NULL, 1, NULL);
}

void loop()
{
    delay(10); // 等待10毫秒
}



# 测试里程计数据的功能
## 根据获取的电机速度解算出里程计数据
#include <Arduino.h>
#include "MLTrol.h"

// 底盘参数
#define CAR_LENGTH 0.30f     // 车身长度(m)
#define CAR_WIDTH 0.25f      // 车身宽度(m)
#define WHEEL_DIAMETER 0.05f // 轮子直径(m)

// 创建运动学对象，用来计算里程计和速度正逆解
MecanumKinematics kinematics(CAR_LENGTH, CAR_WIDTH, WHEEL_DIAMETER);
WheelSpeeds speeds;

// 预设速度参数（单位：m/s 和 rad/s）
#define TEST_VEL_X 0.25f  // X方向线速度
#define TEST_VEL_Y 0.1f   // Y方向线速度
#define TEST_ANGULAR 0.5f // 角速度（约0.0度/秒）


void setup()
{
    Serial.begin(115200);
    while (!Serial) {}
    kinematics.MotorInit();
    // delay(100);
    // Emm_V5_En_Control(0, true, true);
    // delay(100);
    kinematics.resetOdometry();
    speeds = kinematics.inverseKinematics(TEST_VEL_X, TEST_VEL_Y, TEST_ANGULAR);
    kinematics.setMotorSpeed(speeds);
    delay(2000);
}

void loop()
{
    delay(10);
    
    if (1) {
        kinematics.updateOdometry(10); // 自动计算时间差
        // uint16_t vel_fl_rpm = Emm_V5_MotorVel_Get(1);
        // uint16_t vel_fr_rpm = Emm_V5_MotorVel_Get(2);    
        // uint16_t vel_rl_rpm = Emm_V5_MotorVel_Get(3);
        // uint16_t vel_rr_rpm = Emm_V5_MotorVel_Get(4); // 50Hz更新
        // // Serial.print("轮子速度(RPM)理想速度: ");
        // // Serial.print(speeds.front_left);
        // // Serial.print(speeds.front_right);
        // // Serial.print(speeds.rear_left);
        // // Serial.println(speeds.rear_right);
        // Serial.print(vel_fl_rpm);
        // Serial.print(", ");
        // Serial.print(vel_fr_rpm);
        // Serial.print(", ");
        // Serial.print(vel_rl_rpm);
        // Serial.print(", ");
        // Serial.println(vel_rr_rpm);
        
        // 获取里程计数据
        // OdometryData odom = kinematics.getOdometry();
        OdometryData odom = kinematics.getOdometryData();
        Serial.println("----");
        Serial.print("Pos: (");
        Serial.print(odom.pos_x, 3);
        Serial.print(", ");
        Serial.print(odom.pos_y, 3);
        Serial.print("), Orient: ");
        Serial.print(odom.orientation, 2);
        Serial.print("vel_x: ");
        Serial.print(odom.linear_vel_x, 3);
        Serial.print(" m/s");
        Serial.print(" vel_y: ");
        Serial.print(odom.linear_vel_y, 3);
        Serial.print(" m/s");
        Serial.print(" angular_vel: ");
        Serial.print(odom.angular_vel, 3);
        Serial.println(" rad/s");
    }
}