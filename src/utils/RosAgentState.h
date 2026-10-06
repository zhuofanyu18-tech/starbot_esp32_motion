#ifndef ROS_AGENT_STATE_H
#define ROS_AGENT_STATE_H

#include <stdint.h>

// micro-ROS 与电脑端 Agent 的连接状态（main.cpp 状态机使用，OLED 显示使用）
enum class RosAgentState : uint8_t {
    kWaitingAgent,    // 正在寻找 Agent（未启动 / USB 未连接）
    kAgentAvailable,  // 找到 Agent，准备创建节点
    kConnected,       // 节点已创建，正常通信
    kDisconnected,    // 通信中断，准备销毁实体后重新寻找
};

#endif
