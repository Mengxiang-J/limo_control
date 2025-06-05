/**
 * @file pde_formation_controller.cpp
 * @brief PDE-based formation controller implementation
 */

#include "pde_formation_control/pde_formation_controller.h"
#include <numeric>

PDEFormationController::PDEFormationController(
    ros::NodeHandle& nh, 
    int robot_id, 
    AgentRole role,
    double alpha
) : nh_(nh), robot_id_(robot_id), role_(role), alpha_(alpha) {
    // 初始化编队参数
    formation_center_x_ = 0.0;
    formation_center_y_ = 0.0;
    formation_radius_ = 0.8; // 默认半径0.8m
    
    // 初始化机器人状态
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
    
    // 从参数服务器加载PDE模型参数
    nh_.param("pde_order", pde_order_, 5);  // 默认5阶PDE
    
    // 初始化PDE系数(根据论文中的Case 1)
    double scale = 1e-4; // 缩放因子，避免数值问题
    a_coef_.resize(pde_order_ + 1);
    b_coef_.resize(pde_order_ + 1);
    
    // 默认系数(论文Case 1)
    a_coef_[0] = 0.0 * scale;
    a_coef_[1] = 0.0 * scale;
    a_coef_[2] = 4*M_PI*M_PI * scale;
    a_coef_[3] = (38.0/15.0)*(4*M_PI*M_PI) * scale;
    a_coef_[4] = 1.0 * scale;
    a_coef_[5] = (38.0/15.0) * scale;
    
    b_coef_[0] = 0.0 * scale;
    b_coef_[1] = 0.0 * scale;
    b_coef_[2] = 4*M_PI*M_PI * scale;
    b_coef_[3] = (38.0/15.0)*(4*M_PI*M_PI) * scale;
    b_coef_[4] = 1.0 * scale;
    b_coef_[5] = (38.0/15.0) * scale;
    
    // 加载控制器参数
    nh_.param("control_rate", control_rate_, 10.0);  // 默认10Hz
    nh_.param("K", K_, 2.0);  // Lyapunov函数参数
    nh_.param("L", L_, 1.0);  // Lyapunov函数参数
    
    // 领导者反馈增益
    nh_.param("k0", k0_, -0.1);
    nh_.param("ka", ka_, -0.1);
    
    // 初始化边界条件期望值
    u0_bar_ = 0.0;
    ua_bar_ = 0.0;
    v0_bar_ = 0.0;
    va_bar_ = 0.0;
    
    // 高阶边界条件
    u0j_.resize(std::max(0, pde_order_/2 - 1), 0.0);
    ual_.resize(std::max(0, (pde_order_ - 1)/2), 0.0);
    v0j_.resize(std::max(0, pde_order_/2 - 1), 0.0);
    val_.resize(std::max(0, (pde_order_ - 1)/2), 0.0);
    
    // 更新期望编队参数
    updateDesiredFormation(formation_center_x_, formation_center_y_, formation_radius_);
}

void PDEFormationController::initialize() {
    // 创建发布器和订阅器
    std::string robot_namespace = "limo" + std::to_string(robot_id_);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/" + robot_namespace + "/cmd_vel", 1);
    odom_sub_ = nh_.subscribe("/" + robot_namespace + "/odom", 1, &PDEFormationController::odomCallback, this);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/" + robot_namespace + "/pose", 10);
    
    // 订阅邻居位置
    int num_robots = 4; // 总机器人数量
    for (int i = 1; i <= num_robots; ++i) {
        if (i != robot_id_) {
            std::string neighbor_namespace = "limo" + std::to_string(i);
            ros::Subscriber sub = nh_.subscribe<geometry_msgs::PoseStamped>(
                "/" + neighbor_namespace + "/pose", 
                10, 
                boost::bind(&PDEFormationController::neighborCallback, this, _1, i)
            );
            neighbor_subs_.push_back(sub);
        }
    }
    
    // 启动控制定时器
    control_timer_ = nh_.createTimer(ros::Duration(1.0/control_rate_), 
                                     &PDEFormationController::controlLoop, this);
    
    ROS_INFO("机器人%d的PDE编队控制器已初始化，角色%d，alpha=%.2f", 
              robot_id_, (int)role_, alpha_);
}

void PDEFormationController::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // 提取位置
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    
    // 提取方向
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, theta_);
    
    // 发布当前位姿供其他机器人使用
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map";
    pose_msg.pose.position.x = x_;
    pose_msg.pose.position.y = y_;
    pose_msg.pose.position.z = 0.0;
    pose_msg.pose.orientation = msg->pose.pose.orientation;
    pose_pub_.publish(pose_msg);
    
    // 调试信息
    ROS_INFO_THROTTLE(1.0, "机器人%d当前位置: (%.2f, %.2f), 方向: %.2f", 
                     robot_id_, x_, y_, theta_);
}

void PDEFormationController::neighborCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int neighbor_id) {
    // 调试输出
    ROS_INFO("收到机器人%d的位置数据: (%.2f, %.2f)", 
             neighbor_id, msg->pose.position.x, msg->pose.position.y);
    
    // 存储邻居位置
    neighbor_positions_[neighbor_id] = std::make_pair(msg->pose.position.x, msg->pose.position.y);
}

void PDEFormationController::controlLoop(const ros::TimerEvent& event) {
    // 打印当前邻居数量
    ROS_INFO_THROTTLE(1.0, "当前邻居数量: %zu", neighbor_positions_.size());
    
    // 计算控制命令
    geometry_msgs::Twist cmd_vel;
    
    // 即使没有邻居数据也能运行的修改
    if (neighbor_positions_.size() < 1) {
        // 使用简单控制方法（不依赖邻居数据）
        cmd_vel = computeSimpleControl();
        ROS_WARN_THROTTLE(1.0, "邻居数据不足，使用简单控制模式");
    } else {
        // 使用正常PDE控制
        if (role_ == LEADER_ALPHA_0 || role_ == LEADER_ALPHA_1) {
            cmd_vel = computeLeaderControl();
        } else {
            cmd_vel = computeFollowerControl();
        }
    }
    
    // 打印控制命令
    ROS_INFO_THROTTLE(1.0, "发送速度命令: linear=(%.2f, %.2f), angular=%.2f", 
                     cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
    
    // 发布控制命令
    cmd_vel_pub_.publish(cmd_vel);
}

// 简单控制器，不依赖邻居数据
geometry_msgs::Twist PDEFormationController::computeSimpleControl() {
    geometry_msgs::Twist cmd_vel;
    
    // 计算期望位置
    double x_desired = desiredPosition(alpha_, true);
    double y_desired = desiredPosition(alpha_, false);
    
    // 计算位置误差
    double x_error = x_desired - x_;
    double y_error = y_desired - y_;
    
    // 打印目标位置和误差
    ROS_INFO_THROTTLE(1.0, "目标位置: (%.2f, %.2f), 误差: (%.2f, %.2f)", 
                     x_desired, y_desired, x_error, y_error);
    
    // 简单比例控制
    double kp = 0.3; // 比例增益
    cmd_vel.linear.x = kp * x_error;
    cmd_vel.linear.y = kp * y_error;
    
    // 转换到机器人局部坐标系
    double linear_x = cmd_vel.linear.x * cos(theta_) + cmd_vel.linear.y * sin(theta_);
    double linear_y = -cmd_vel.linear.x * sin(theta_) + cmd_vel.linear.y * cos(theta_);
    
    cmd_vel.linear.x = linear_x;
    cmd_vel.linear.y = linear_y;
    
    // 朝向控制，使机器人面向编队中心
    double desired_theta = atan2(formation_center_y_ - y_, formation_center_x_ - x_);
    double theta_error = atan2(sin(desired_theta - theta_), cos(desired_theta - theta_));
    cmd_vel.angular.z = 0.5 * theta_error;
    
    // 速度限制
    double max_linear_speed = 0.2; // 降低速度提高安全性
    double max_angular_speed = 0.5;
    
    double linear_speed = sqrt(cmd_vel.linear.x * cmd_vel.linear.x + cmd_vel.linear.y * cmd_vel.linear.y);
    if (linear_speed > max_linear_speed) {
        cmd_vel.linear.x *= max_linear_speed / linear_speed;
        cmd_vel.linear.y *= max_linear_speed / linear_speed;
    }
    
    cmd_vel.angular.z = std::max(std::min(cmd_vel.angular.z, max_angular_speed), -max_angular_speed);
    
    return cmd_vel;
}

geometry_msgs::Twist PDEFormationController::computeLeaderControl() {
    geometry_msgs::Twist cmd_vel;
    
    // 计算期望位置和当前误差
    double x_desired = desiredPosition(alpha_, true);
    double y_desired = desiredPosition(alpha_, false);
    
    double x_error = x_desired - x_;
    double y_error = y_desired - y_;
    
    // 打印目标位置和误差
    ROS_INFO_THROTTLE(1.0, "领导者目标位置: (%.2f, %.2f), 误差: (%.2f, %.2f)", 
                     x_desired, y_desired, x_error, y_error);
    
    // 计算边界控制项
    double u0 = 0.0;
    double ua = 0.0;
    double v0 = 0.0;
    double va = 0.0;
    
    if (role_ == LEADER_ALPHA_0) {
        // α=0处的领导者(u0控制)
        if (pde_order_ >= 3) {
            // 计算高阶PDE的空间导数
            double sum_term1_x = 0.0;
            double sum_term2_x = 0.0;
            double sum_term1_y = 0.0;
            double sum_term2_y = 0.0;
            
            for (int j = 3; j <= pde_order_; j++) {
                sum_term1_x += a_coef_[j] * calculateSpatialDerivative(j-2, true);
                sum_term1_y += b_coef_[j] * calculateSpatialDerivative(j-2, false);
            }
            
            for (int j = 2; j <= pde_order_; j++) {
                sum_term2_x += a_coef_[j] * calculateSpatialDerivative(j-1, true);
                sum_term2_y += b_coef_[j] * calculateSpatialDerivative(j-1, false);
            }
            
            u0 = k0_ * (L_ * sum_term1_x - K_ * sum_term2_x) + u0_bar_;
            v0 = k0_ * (L_ * sum_term1_y - K_ * sum_term2_y) + v0_bar_;
        } else {
            // m=2情况(简化)
            u0 = -k0_ * K_ * a_coef_[2] * calculateSpatialDerivative(1, true) + u0_bar_;
            v0 = -k0_ * K_ * b_coef_[2] * calculateSpatialDerivative(1, false) + v0_bar_;
        }
        
        // 使用直接控制
        cmd_vel.linear.x = 0.3 * (u0 - x_);
        cmd_vel.linear.y = 0.3 * (v0 - y_);
    } else if (role_ == LEADER_ALPHA_1) {
        // α=1处的领导者(ua控制)
        if (pde_order_ >= 3) {
            // 计算高阶PDE的空间导数
            double sum_term1_x = 0.0;
            double sum_term2_x = 0.0;
            double sum_term1_y = 0.0;
            double sum_term2_y = 0.0;
            
            for (int j = 2; j <= pde_order_; j++) {
                sum_term1_x += a_coef_[j] * calculateSpatialDerivative(j-1, true);
                sum_term1_y += b_coef_[j] * calculateSpatialDerivative(j-1, false);
            }
            
            for (int j = 3; j <= pde_order_; j++) {
                sum_term2_x += a_coef_[j] * calculateSpatialDerivative(j-2, true);
                sum_term2_y += b_coef_[j] * calculateSpatialDerivative(j-2, false);
            }
            
            ua = ka_ * ((K_ + L_) * sum_term1_x - L_ * sum_term2_x) + ua_bar_;
            va = ka_ * ((K_ + L_) * sum_term1_y - L_ * sum_term2_y) + va_bar_;
        } else {
            // m=2情况(简化)
            ua = ka_ * (K_ + L_) * a_coef_[2] * calculateSpatialDerivative(1, true) + ua_bar_;
            va = ka_ * (K_ + L_) * b_coef_[2] * calculateSpatialDerivative(1, false) + va_bar_;
        }
        
        // 使用直接控制
        cmd_vel.linear.x = 0.3 * (ua - x_);
        cmd_vel.linear.y = 0.3 * (va - y_);
    }
    
    // 转换到机器人局部坐标系
    double linear_x = cmd_vel.linear.x * cos(theta_) + cmd_vel.linear.y * sin(theta_);
    double linear_y = -cmd_vel.linear.x * sin(theta_) + cmd_vel.linear.y * cos(theta_);
    
    cmd_vel.linear.x = linear_x;
    cmd_vel.linear.y = linear_y;
    
    // 朝向控制，使机器人面向编队中心
    double desired_theta = atan2(formation_center_y_ - y_, formation_center_x_ - x_);
    double theta_error = atan2(sin(desired_theta - theta_), cos(desired_theta - theta_));
    cmd_vel.angular.z = 0.5 * theta_error;
    
    // 速度限制
    double max_linear_speed = 0.2; // 降低速度提高安全性
    double max_angular_speed = 0.5;
    
    double linear_speed = sqrt(cmd_vel.linear.x * cmd_vel.linear.x + cmd_vel.linear.y * cmd_vel.linear.y);
    if (linear_speed > max_linear_speed) {
        cmd_vel.linear.x *= max_linear_speed / linear_speed;
        cmd_vel.linear.y *= max_linear_speed / linear_speed;
    }
    
    cmd_vel.angular.z = std::max(std::min(cmd_vel.angular.z, max_angular_speed), -max_angular_speed);
    
    return cmd_vel;
}

geometry_msgs::Twist PDEFormationController::computeFollowerControl() {
    geometry_msgs::Twist cmd_vel;
    
    // 处理邻居位置和对应的alpha值
    std::vector<std::pair<double, double>> positions;
    std::vector<double> alphas;
    
    // 处理邻居位置
    for (const auto& neighbor : neighbor_positions_) {
        // 获取邻居alpha值
        double neighbor_alpha;
        switch (neighbor.first) {
            case 1: neighbor_alpha = 0.0; break;
            case 4: neighbor_alpha = 1.0; break;
            case 2: neighbor_alpha = 0.33; break;
            case 3: neighbor_alpha = 0.67; break;
            default: neighbor_alpha = 0.5; // 默认值
        }
        
        ROS_INFO("机器人%d: alpha=%.2f, 位置=(%.2f, %.2f)", 
                 neighbor.first, neighbor_alpha, 
                 neighbor.second.first, neighbor.second.second);
        
        positions.push_back(neighbor.second);
        alphas.push_back(neighbor_alpha);
    }
    
    // 计算期望位置
    double x_desired = desiredPosition(alpha_, true);
    double y_desired = desiredPosition(alpha_, false);
    
    // 打印目标位置
    ROS_INFO_THROTTLE(1.0, "跟随者目标位置: (%.2f, %.2f), 当前位置: (%.2f, %.2f)", 
                     x_desired, y_desired, x_, y_);
    
    // 简单比例控制
    double kp = 0.3; // 比例增益
    cmd_vel.linear.x = kp * (x_desired - x_);
    cmd_vel.linear.y = kp * (y_desired - y_);
    
    // 转换到机器人局部坐标系
    double linear_x = cmd_vel.linear.x * cos(theta_) + cmd_vel.linear.y * sin(theta_);
    double linear_y = -cmd_vel.linear.x * sin(theta_) + cmd_vel.linear.y * cos(theta_);
    
    cmd_vel.linear.x = linear_x;
    cmd_vel.linear.y = linear_y;
    
    // 朝向控制
    double desired_theta = atan2(formation_center_y_ - y_, formation_center_x_ - x_);
    double theta_error = atan2(sin(desired_theta - theta_), cos(desired_theta - theta_));
    cmd_vel.angular.z = 0.5 * theta_error;
    
    // 速度限制
    double max_linear_speed = 0.2; // 降低速度提高安全性
    double max_angular_speed = 0.5;
    
    double linear_speed = sqrt(cmd_vel.linear.x * cmd_vel.linear.x + cmd_vel.linear.y * cmd_vel.linear.y);
    if (linear_speed > max_linear_speed) {
        cmd_vel.linear.x *= max_linear_speed / linear_speed;
        cmd_vel.linear.y *= max_linear_speed / linear_speed;
    }
    
    cmd_vel.angular.z = std::max(std::min(cmd_vel.angular.z, max_angular_speed), -max_angular_speed);
    
    return cmd_vel;
}

double PDEFormationController::calculateSpatialDerivative(int s, bool is_x) {
    // 使用有限差分近似
    // 这是一个简化实现，用于演示
    
    if (s == 0) {
        return is_x ? x_ : y_;
    }
    
    // 获取所有机器人位置
    std::vector<std::pair<double, double>> positions;
    for (const auto& neighbor : neighbor_positions_) {
        double pos = is_x ? neighbor.second.first : neighbor.second.second;
        // 根据机器人ID分配alpha
        double alpha = (neighbor.first == 1) ? 0.0 : 
                      (neighbor.first == 4) ? 1.0 : 
                      (neighbor.first == 2) ? 0.33 : 0.67;
        positions.push_back(std::make_pair(alpha, pos));
    }
    
    // 添加自己
    positions.push_back(std::make_pair(alpha_, is_x ? x_ : y_));
    
    // 按alpha排序
    std::sort(positions.begin(), positions.end());
    
    // 在排序列表中找到自己的索引
    int my_idx = -1;
    for (size_t i = 0; i < positions.size(); ++i) {
        if (std::abs(positions[i].first - alpha_) < 1e-6) {
            my_idx = i;
            break;
        }
    }
    
    // 计算s阶导数至少需要s+1个点
    if (positions.size() < s + 1) {
        ROS_WARN("计算%d阶导数的点不足", s);
        return 0.0;
    }
    
    // 使用中心差分计算空间导数
    // 这是一个简化实现，更准确的方法应使用论文中方程(35)
    double h = 1.0 / (positions.size() - 1);
    double result = 0.0;
    
    // 一阶导数，使用中心差分
    if (s == 1) {
        if (my_idx > 0 && my_idx < positions.size() - 1) {
            result = (positions[my_idx + 1].second - positions[my_idx - 1].second) / (2.0 * h);
        } else if (my_idx == 0 && positions.size() > 1) {
            // 前向差分
            result = (positions[1].second - positions[0].second) / h;
        } else if (my_idx == positions.size() - 1 && positions.size() > 1) {
            // 后向差分
            result = (positions[positions.size() - 1].second - positions[positions.size() - 2].second) / h;
        }
    }
    // 二阶导数，使用中心差分
    else if (s == 2) {
        if (my_idx > 0 && my_idx < positions.size() - 1) {
            result = (positions[my_idx + 1].second - 2.0 * positions[my_idx].second + positions[my_idx - 1].second) / (h * h);
        }
    }
    // 高阶导数，回退到解析方法
    else {
        // 使用期望位置的导数作为近似
        result = desiredSpatialDerivative(alpha_, s, is_x);
    }
    
    return result;
}

void PDEFormationController::updateDesiredFormation(double center_x, double center_y, double radius) {
    formation_center_x_ = center_x;
    formation_center_y_ = center_y;
    formation_radius_ = radius;
    
    // 更新圆形编队的边界条件
    // 对于中心在(cx, cy)、半径为r的圆：
    // x(α) = cx + r * cos(2πα)
    // y(α) = cy + r * sin(2πα)
    
    // 设置x轴边界条件
    u0_bar_ = center_x + radius * cos(0);           // x(0) = cx + r
    ua_bar_ = center_x + radius * cos(2 * M_PI);    // x(1) = cx + r
    
    // 设置y轴边界条件
    v0_bar_ = center_y + radius * sin(0);           // y(0) = cy + 0
    va_bar_ = center_y + radius * sin(2 * M_PI);    // y(1) = cy + 0
    
    // 设置高阶边界条件
    // 对于圆，α=0和α=1处的导数取决于阶数
    for (int j = 1; j < u0j_.size(); ++j) {
        u0j_[j-1] = radius * (-pow(2*M_PI, j)) * sin(j * 2 * M_PI * 0);  // x^(j)(0)
        v0j_[j-1] = radius * pow(2*M_PI, j) * cos(j * 2 * M_PI * 0);     // y^(j)(0)
    }
    
    for (int l = 1; l < ual_.size(); ++l) {
        ual_[l-1] = radius * (-pow(2*M_PI, l)) * sin(l * 2 * M_PI * 1);  // x^(l)(1)
        val_[l-1] = radius * pow(2*M_PI, l) * cos(l * 2 * M_PI * 1);     // y^(l)(1)
    }
    
    ROS_INFO("更新编队参数: 中心=(%.2f, %.2f), 半径=%.2f", 
             formation_center_x_, formation_center_y_, formation_radius_);
}

double PDEFormationController::desiredPosition(double alpha, bool is_x) {
    // 对于中心在(cx, cy)、半径为r的圆：
    // x(α) = cx + r * cos(2πα)
    // y(α) = cy + r * sin(2πα)
    if (is_x) {
        return formation_center_x_ + formation_radius_ * cos(2 * M_PI * alpha);
    } else {
        return formation_center_y_ + formation_radius_ * sin(2 * M_PI * alpha);
    }
}

double PDEFormationController::desiredSpatialDerivative(double alpha, int order, bool is_x) {
    // 对于中心在(cx, cy)、半径为r的圆：
    // x(α) = cx + r * cos(2πα)
    // y(α) = cy + r * sin(2πα)
    
    // 对于x轴：
    // x^(1)(α) = -r * 2π * sin(2πα)
    // x^(2)(α) = -r * (2π)^2 * cos(2πα)
    // x^(3)(α) = r * (2π)^3 * sin(2πα)
    // 等等
    
    // 对于y轴：
    // y^(1)(α) = r * 2π * cos(2πα)
    // y^(2)(α) = -r * (2π)^2 * sin(2πα)
    // y^(3)(α) = -r * (2π)^3 * cos(2πα)
    // 等等
    
    double result = 0.0;
    double factor = pow(2 * M_PI, order) * formation_radius_;
    
    if (is_x) {
        switch (order % 4) {
            case 0: result = factor * cos(2 * M_PI * alpha); break;
            case 1: result = -factor * sin(2 * M_PI * alpha); break;
            case 2: result = -factor * cos(2 * M_PI * alpha); break;
            case 3: result = factor * sin(2 * M_PI * alpha); break;
        }
    } else {
        switch (order % 4) {
            case 0: result = factor * sin(2 * M_PI * alpha); break;
            case 1: result = factor * cos(2 * M_PI * alpha); break;
            case 2: result = -factor * sin(2 * M_PI * alpha); break;
            case 3: result = -factor * cos(2 * M_PI * alpha); break;
        }
    }
    
    return result;
}