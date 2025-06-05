/**
 * @file pde_formation_node.cpp
 * @brief PDE-based formation control的主节点
 */

#include "pde_formation_control/pde_formation_controller.h"
#include <ros/ros.h>
#include <string>

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "pde_formation_node");
    ros::NodeHandle nh("~");
    
    // 获取参数
    int robot_id;
    int role_int;
    double alpha;
    double center_x, center_y, radius;
    
    nh.param("robot_id", robot_id, 1);
    nh.param("role", role_int, 0);
    nh.param("alpha", alpha, 0.0);
    nh.param("center_x", center_x, 0.0);
    nh.param("center_y", center_y, 0.0);
    nh.param("radius", radius, 0.8);
    
    // 将整数角色转换为枚举
    PDEFormationController::AgentRole role;
    switch (role_int) {
        case 0:
            role = PDEFormationController::LEADER_ALPHA_0;
            break;
        case 1:
            role = PDEFormationController::LEADER_ALPHA_1;
            break;
        case 2:
            role = PDEFormationController::FOLLOWER;
            break;
        default:
            ROS_ERROR("无效的角色参数: %d", role_int);
            return 1;
    }
    
    // 创建控制器
    PDEFormationController controller(nh, robot_id, role, alpha);
    
    // 设置期望编队
    controller.updateDesiredFormation(center_x, center_y, radius);
    
    // 初始化并启动控制器
    controller.initialize();
    
    ROS_INFO("机器人%d的PDE编队节点已启动", robot_id);
    
    ros::spin();
    
    return 0;
}