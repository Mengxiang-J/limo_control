/**
 * @file pde_formation_controller.h
 * @brief PDE-based formation controller for multi-agent systems
 */

#ifndef PDE_FORMATION_CONTROLLER_H
#define PDE_FORMATION_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <numeric>

class PDEFormationController {
public:
    enum AgentRole {
        LEADER_ALPHA_0,  // 领导者在α=0处
        LEADER_ALPHA_1,  // 领导者在α=1处
        FOLLOWER         // 跟随者
    };

    // 构造函数
    PDEFormationController(
        ros::NodeHandle& nh, 
        int robot_id,
        AgentRole role,
        double alpha
    );

    // 初始化函数
    void initialize();
    
    // 更新期望编队参数
    void updateDesiredFormation(double center_x, double center_y, double radius);

private:
    // ROS相关成员
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;
    ros::Publisher pose_pub_;
    ros::Timer control_timer_;
    std::vector<ros::Subscriber> neighbor_subs_;

    // 智能体参数
    int robot_id_;
    AgentRole role_;
    double alpha_;  // 空间参数 (0 <= alpha <= 1)
    
    // PDE模型参数
    int pde_order_;              // PDE模型阶数
    std::vector<double> a_coef_; // x轴PDE系数
    std::vector<double> b_coef_; // y轴PDE系数
    
    // 控制器参数
    double control_rate_;
    double K_, L_;            // Lyapunov函数参数
    double k0_, ka_;          // 领导者反馈增益
    
    // 编队参数
    double formation_center_x_;
    double formation_center_y_;
    double formation_radius_;
    
    // 当前状态
    double x_, y_, theta_;
    
    // 邻居位置
    std::map<int, std::pair<double, double>> neighbor_positions_;
    
    // 边界条件期望值
    double u0_bar_;     // x(0,t)的期望值
    double ua_bar_;     // x(1,t)的期望值
    std::vector<double> u0j_; // x^(j)(0,t)的期望值
    std::vector<double> ual_; // x^(l)(1,t)的期望值
    
    double v0_bar_;     // y(0,t)的期望值
    double va_bar_;     // y(1,t)的期望值
    std::vector<double> v0j_; // y^(j)(0,t)的期望值
    std::vector<double> val_; // y^(l)(1,t)的期望值
    
    // 回调函数
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void neighborCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int neighbor_id);
    void controlLoop(const ros::TimerEvent& event);
    
    // 控制方法
    geometry_msgs::Twist computeLeaderControl();
    geometry_msgs::Twist computeFollowerControl();
    geometry_msgs::Twist computeSimpleControl(); // 简单控制，无需邻居数据
    
    // 计算空间导数
    double calculateSpatialDerivative(int s, bool is_x);
    
    // 计算期望位置和导数
    double desiredPosition(double alpha, bool is_x);
    double desiredSpatialDerivative(double alpha, int order, bool is_x);
};

#endif // PDE_FORMATION_CONTROLLER_H
