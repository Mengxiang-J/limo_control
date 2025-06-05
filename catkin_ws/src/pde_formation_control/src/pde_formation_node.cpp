/**
 * @file pde_formation_node.cpp
 * @brief PDE编队控制主节点 - 优化版本
 */

#include "pde_formation_control/pde_formation_controller.h"
#include <ros/ros.h>
#include <string>
#include <signal.h>

// 全局控制器指针，用于优雅关闭
PDEFormationController* g_controller = nullptr;

/**
 * @brief 信号处理函数，确保优雅关闭
 */
void signalHandler(int sig) {
    ROS_INFO("接收到关闭信号，正在安全关闭节点...");
    if (g_controller) {
        // 发送停止指令
        geometry_msgs::Twist stop_cmd;
        // 这里可以添加停止指令的发布
    }
    ros::shutdown();
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "pde_formation_node");
    ros::NodeHandle nh("~");
    
    // 注册信号处理器
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // 获取参数，添加更多默认值和验证
    int robot_id;
    int role_int;
    double alpha;
    double center_x, center_y, radius;
    
    // 参数获取，带默认值
    nh.param("robot_id", robot_id, 1);
    nh.param("role", role_int, 2);  // 默认为跟随者
    nh.param("alpha", alpha, 0.0);
    nh.param("center_x", center_x, 0.0);
    nh.param("center_y", center_y, 0.0);
    nh.param("radius", radius, 1.2);  // 使用优化后的默认半径
    
    // 参数验证
    if (robot_id < 1 || robot_id > 4) {
        ROS_ERROR("无效的机器人ID: %d，应该在1-4之间", robot_id);
        return 1;
    }
    
    if (alpha < 0.0 || alpha > 1.0) {
        ROS_ERROR("无效的alpha参数: %.2f，应该在0.0-1.0之间", alpha);
        return 1;
    }
    
    if (radius <= 0.0) {
        ROS_ERROR("无效的半径参数: %.2f，应该大于0", radius);
        return 1;
    }
    
    // 根据robot_id自动设置合理的默认alpha值（如果没有明确指定）
    if (!nh.hasParam("alpha")) {
        switch (robot_id) {
            case 1: alpha = 0.0; break;   // 0度位置
            case 2: alpha = 0.25; break;  // 90度位置
            case 3: alpha = 0.5; break;   // 180度位置
            case 4: alpha = 0.75; break;  // 270度位置
        }
        ROS_INFO("机器人%d自动设置alpha为%.2f", robot_id, alpha);
    }
    
    // 根据robot_id和alpha自动设置角色（如果没有明确指定）
    if (!nh.hasParam("role")) {
        if (robot_id == 1 && alpha == 0.0) {
            role_int = 0; // LEADER_ALPHA_0
        } else if (robot_id == 4 && alpha == 1.0) {
            role_int = 1; // LEADER_ALPHA_1
        } else {
            role_int = 2; // FOLLOWER
        }
        ROS_INFO("机器人%d自动设置角色为%d", robot_id, role_int);
    }
    
    // 转换角色整数为枚举
    PDEFormationController::AgentRole role;
    switch (role_int) {
        case 0:
            role = PDEFormationController::LEADER_ALPHA_0;
            ROS_INFO("机器人%d设置为领导者(Alpha=0)", robot_id);
            break;
        case 1:
            role = PDEFormationController::LEADER_ALPHA_1;
            ROS_INFO("机器人%d设置为领导者(Alpha=1)", robot_id);
            break;
        case 2:
            role = PDEFormationController::FOLLOWER;
            ROS_INFO("机器人%d设置为跟随者", robot_id);
            break;
        default:
            ROS_ERROR("无效的角色参数: %d，应该是0(LEADER_ALPHA_0), 1(LEADER_ALPHA_1), 或2(FOLLOWER)", role_int);
            return 1;
    }
    
    // 打印配置信息
    ROS_INFO("=== PDE编队控制节点配置 ===");
    ROS_INFO("机器人ID: %d", robot_id);
    ROS_INFO("角色: %s", (role_int == 0) ? "领导者(Alpha=0)" : 
                        (role_int == 1) ? "领导者(Alpha=1)" : "跟随者");
    ROS_INFO("Alpha参数: %.3f", alpha);
    ROS_INFO("编队中心: (%.2f, %.2f)", center_x, center_y);
    ROS_INFO("编队半径: %.2f米", radius);
    ROS_INFO("========================");
    
    // 等待其他节点启动
    ROS_INFO("等待2秒，让其他节点启动...");
    ros::Duration(2.0).sleep();
    
    try {
        // 创建控制器
        PDEFormationController controller(nh, robot_id, role, alpha);
        g_controller = &controller;
        
        // 设置期望编队
        controller.updateDesiredFormation(center_x, center_y, radius);
        
        // 初始化并启动控制器
        controller.initialize();
        
        ROS_INFO("机器人%d的PDE编队节点已启动并运行", robot_id);
        
        // 主循环
        ros::spin();
        
    } catch (const std::exception& e) {
        ROS_ERROR("控制器初始化失败: %s", e.what());
        return 1;
    }
    
    ROS_INFO("机器人%d的PDE编队节点正常关闭", robot_id);
    return 0;
}
