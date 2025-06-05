/** 
 * @file pde_formation_controller.cpp
 * @brief 基于PDE的编队控制器实现 - 优化版本
 */

#include "pde_formation_control/pde_formation_controller.h"

PDEFormationController::PDEFormationController(
    ros::NodeHandle& nh, 
    int robot_id, 
    AgentRole role,
    double alpha
) : nh_(nh), robot_id_(robot_id), role_(role), alpha_(alpha) {
    // 初始化编队参数
    formation_center_x_ = 0.0;
    formation_center_y_ = 0.0;
    formation_radius_ = 1.2; // 增大半径，避免碰撞
    
    // 初始化机器人状态
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
    
    // 简化PDE参数，提高稳定性
    nh_.param("pde_order", pde_order_, 5);
    
    // 使用更保守的缩放因子
    double scale = 1e-5; // 减小缩放因子，提高稳定性
    a_coef_.resize(pde_order_ + 1);
    b_coef_.resize(pde_order_ + 1);
    
    // 简化的系数设置
    a_coef_[0] = 0.0;
    a_coef_[1] = 0.0;
    a_coef_[2] = 4*M_PI*M_PI * scale;
    a_coef_[3] = (38.0/15.0)*(4*M_PI*M_PI) * scale;
    a_coef_[4] = 1.0 * scale;
    a_coef_[5] = (38.0/15.0) * scale;
    
    // Y轴系数与X轴相同
    for (int i = 0; i <= pde_order_; i++) {
        b_coef_[i] = a_coef_[i];
    }
    
    // 更保守的控制参数
    nh_.param("control_rate", control_rate_, 5.0);  // 降低控制频率
    nh_.param("K", K_, 1.0);  // 减小增益
    nh_.param("L", L_, 0.5);  // 减小增益
    
    // 更小的反馈增益
    nh_.param("k0", k0_, -0.05);
    nh_.param("ka", ka_, -0.05);
    
    // 初始化边界条件
    u0_bar_ = 0.0;
    ua_bar_ = 0.0;
    v0_bar_ = 0.0;
    va_bar_ = 0.0;
    
    // 高阶边界条件
    u0j_.resize(std::max(0, pde_order_/2 - 1), 0.0);
    ual_.resize(std::max(0, (pde_order_ - 1)/2), 0.0);
    v0j_.resize(std::max(0, pde_order_/2 - 1), 0.0);
    val_.resize(std::max(0, (pde_order_ - 1)/2), 0.0);
    
    // 添加启动延迟，等待所有节点就绪
    startup_delay_ = 5.0; // 5秒启动延迟
    start_time_ = ros::Time::now();
    
    updateDesiredFormation(formation_center_x_, formation_center_y_, formation_radius_);
    
    ROS_INFO("机器人%d初始化完成，角色：%d，alpha：%.2f", robot_id_, (int)role_, alpha_);
}

void PDEFormationController::initialize() {
    // 创建发布者和订阅者
    std::string robot_namespace = "limo" + std::to_string(robot_id_);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/" + robot_namespace + "/cmd_vel", 1);
    odom_sub_ = nh_.subscribe("/" + robot_namespace + "/odom", 1, &PDEFormationController::odomCallback, this);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/" + robot_namespace + "/pose", 1);
    
    // 订阅所有其他机器人的位置
    int num_robots = 4;
    for (int i = 1; i <= num_robots; ++i) {
        if (i != robot_id_) {
            std::string neighbor_namespace = "limo" + std::to_string(i);
            ros::Subscriber sub = nh_.subscribe<geometry_msgs::PoseStamped>(
                "/" + neighbor_namespace + "/pose", 
                1, 
                boost::bind(&PDEFormationController::neighborCallback, this, _1, i)
            );
            neighbor_subs_.push_back(sub);
        }
    }
    
    // 启动控制定时器
    control_timer_ = nh_.createTimer(ros::Duration(1.0/control_rate_), 
                                     &PDEFormationController::controlLoop, this);
    
    ROS_INFO("机器人%d的PDE编队控制器已启动", robot_id_);
}

// ... existing code ...

void PDEFormationController::controlLoop(const ros::TimerEvent& event) {
    // 启动延迟检查
    if ((ros::Time::now() - start_time_).toSec() < startup_delay_) {
        ROS_INFO_THROTTLE(2.0, "机器人%d等待启动中... %.1f秒", 
                         robot_id_, startup_delay_ - (ros::Time::now() - start_time_).toSec());
        
        // 发布零速度指令
        geometry_msgs::Twist zero_cmd;
        cmd_vel_pub_.publish(zero_cmd);
        return;
    }
    
    // 检查邻居数据 - 降低要求
    ROS_INFO_THROTTLE(2.0, "机器人%d当前邻居数量: %zu", robot_id_, neighbor_positions_.size());
    
    // 即使没有邻居数据也能工作（使用期望位置控制）
    geometry_msgs::Twist cmd_vel;
    
    if (role_ == LEADER_ALPHA_0 || role_ == LEADER_ALPHA_1) {
        cmd_vel = computeLeaderControl();
    } else {
        cmd_vel = computeFollowerControl();
    }
    
    cmd_vel_pub_.publish(cmd_vel);
}

geometry_msgs::Twist PDEFormationController::computeLeaderControl() {
    geometry_msgs::Twist cmd_vel;
    
    // 计算期望位置
    double x_desired = desiredPosition(alpha_, true);
    double y_desired = desiredPosition(alpha_, false);
    
    // 简化的领导者控制 - 直接位置控制
    double kp = 0.3; // 保守的比例增益
    double x_error = x_desired - x_;
    double y_error = y_desired - y_;
    
    // 添加死区，减少抖动
    double deadzone = 0.05;
    if (fabs(x_error) < deadzone) x_error = 0.0;
    if (fabs(y_error) < deadzone) y_error = 0.0;
    
    cmd_vel.linear.x = kp * x_error;
    cmd_vel.linear.y = kp * y_error;
    
    // 简化的角度控制
    double desired_theta = atan2(formation_center_y_ - y_, formation_center_x_ - x_);
    double theta_error = atan2(sin(desired_theta - theta_), cos(desired_theta - theta_));
    cmd_vel.angular.z = 0.2 * theta_error; // 减小角速度增益
    
    // 严格的速度限制
    double max_linear_speed = 0.2;  // 降低最大速度
    double max_angular_speed = 0.3;
    
    // 限制线速度
    double linear_speed = sqrt(cmd_vel.linear.x * cmd_vel.linear.x + cmd_vel.linear.y * cmd_vel.linear.y);
    if (linear_speed > max_linear_speed) {
        cmd_vel.linear.x *= max_linear_speed / linear_speed;
        cmd_vel.linear.y *= max_linear_speed / linear_speed;
    }
    
    // 限制角速度
    cmd_vel.angular.z = std::max(std::min(cmd_vel.angular.z, max_angular_speed), -max_angular_speed);
    
    ROS_INFO_THROTTLE(1.0, "领导者%d: 期望(%.2f,%.2f) 当前(%.2f,%.2f) 误差(%.2f,%.2f)", 
                     robot_id_, x_desired, y_desired, x_, y_, x_error, y_error);
    
    return cmd_vel;
}

geometry_msgs::Twist PDEFormationController::computeFollowerControl() {
    geometry_msgs::Twist cmd_vel;
    
    // 计算期望位置
    double x_desired = desiredPosition(alpha_, true);
    double y_desired = desiredPosition(alpha_, false);
    
    // 改进的跟随者控制
    double kp = 0.4; // 稍高的比例增益
    double x_error = x_desired - x_;
    double y_error = y_desired - y_;
    
    // 添加死区
    double deadzone = 0.05;
    if (fabs(x_error) < deadzone) x_error = 0.0;
    if (fabs(y_error) < deadzone) y_error = 0.0;
    
    // 基本比例控制
    cmd_vel.linear.x = kp * x_error;
    cmd_vel.linear.y = kp * y_error;
    
    // 如果有邻居信息，添加协调项
    if (neighbor_positions_.size() > 0) {
        double coordination_gain = 0.1;
        
        // 计算与邻居的相对位置影响
        for (const auto& neighbor : neighbor_positions_) {
            double dx = neighbor.second.first - x_;
            double dy = neighbor.second.second - y_;
            double distance = sqrt(dx*dx + dy*dy);
            
            // 避免过近
            if (distance < 0.5 && distance > 0.01) {
                cmd_vel.linear.x -= coordination_gain * dx / distance;
                cmd_vel.linear.y -= coordination_gain * dy / distance;
            }
        }
    }
    
    // 角度控制
    double desired_theta = atan2(formation_center_y_ - y_, formation_center_x_ - x_);
    double theta_error = atan2(sin(desired_theta - theta_), cos(desired_theta - theta_));
    cmd_vel.angular.z = 0.2 * theta_error;
    
    // 速度限制
    double max_linear_speed = 0.25;
    double max_angular_speed = 0.3;
    
    double linear_speed = sqrt(cmd_vel.linear.x * cmd_vel.linear.x + cmd_vel.linear.y * cmd_vel.linear.y);
    if (linear_speed > max_linear_speed) {
        cmd_vel.linear.x *= max_linear_speed / linear_speed;
        cmd_vel.linear.y *= max_linear_speed / linear_speed;
    }
    
    cmd_vel.angular.z = std::max(std::min(cmd_vel.angular.z, max_angular_speed), -max_angular_speed);
    
    ROS_INFO_THROTTLE(1.0, "跟随者%d: 期望(%.2f,%.2f) 当前(%.2f,%.2f) 误差(%.2f,%.2f)", 
                     robot_id_, x_desired, y_desired, x_, y_, x_error, y_error);
    
    return cmd_vel;
}

// ... existing code ...

double PDEFormationController::calculateSpatialDerivative(int s, bool is_x) {
    // 简化实现，避免复杂计算
    if (s == 0) {
        return is_x ? x_ : y_;
    }
    
    // 如果没有足够的邻居信息，返回0
    if (neighbor_positions_.size() < 2) {
        return 0.0;
    }
    
    // 简化的空间导数计算
    std::vector<std::pair<double, double>> positions;
    
    // 添加邻居位置
    for (const auto& neighbor : neighbor_positions_) {
        double pos = is_x ? neighbor.second.first : neighbor.second.second;
        double alpha = getAlphaForRobot(neighbor.first);
        positions.push_back(std::make_pair(alpha, pos));
    }
    
    // 添加自己
    positions.push_back(std::make_pair(alpha_, is_x ? x_ : y_));
    
    // 按alpha排序
    std::sort(positions.begin(), positions.end());
    
    // 简单的一阶导数近似
    if (s == 1 && positions.size() >= 2) {
        // 找到自己的位置
        for (size_t i = 0; i < positions.size(); ++i) {
            if (fabs(positions[i].first - alpha_) < 1e-6) {
                if (i > 0 && i < positions.size() - 1) {
                    double h = (positions[i+1].first - positions[i-1].first) / 2.0;
                    if (h > 1e-6) {
                        return (positions[i+1].second - positions[i-1].second) / h;
                    }
                }
                break;
            }
        }
    }
    
    return 0.0; // 默认返回0
}

// 添加辅助函数
double PDEFormationController::getAlphaForRobot(int robot_id) {
    // 更均匀的alpha分配
    switch (robot_id) {
        case 1: return 0.0;   // 0度
        case 2: return 0.25;  // 90度
        case 3: return 0.5;   // 180度
        case 4: return 0.75;  // 270度
        default: return 0.5;
    }
}

// ... existing code ...
