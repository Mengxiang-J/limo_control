#include "pde_formation_control/pde_formation_controller.h"
#include <iostream>
#include <iomanip>
#include <algorithm>

PDEFormationController::PDEFormationController() : nh_(), pnh_("~") {
    // 从参数服务器读取参数
    pnh_.param("circle_radius", circle_traj_.radius, 0.8);
    pnh_.param("angular_frequency", circle_traj_.angular_freq, 0.3);
    pnh_.param("center_x", circle_traj_.center_x, 0.0);
    pnh_.param("center_y", circle_traj_.center_y, 0.0);

    // 🔧 修复：确保所有容器都正确初始化
    pde_params_.N = 4;  // 设定四个机器人
    pde_params_.m = 3;  // 设定PDE的阶数
    pde_params_.k0 = -0.5;
    pde_params_.ka = -0.5;
    pde_params_.h = 1.0 / std::max(1.0, static_cast<double>(pde_params_.N - 1));  // 防止除零

    // 🔧 修复：安全的向量初始化
    pde_params_.a_coeffs.clear();
    pde_params_.b_coeffs.clear();
    pde_params_.a_coeffs.resize(pde_params_.m + 1, 0.0);
    pde_params_.b_coeffs.resize(pde_params_.m + 1, 0.0);
    
    // 确保最高阶系数不为零（避免数值不稳定）
    if (pde_params_.m >= 0 && pde_params_.m < pde_params_.a_coeffs.size()) {
        pde_params_.a_coeffs[pde_params_.m] = -1.0;
    }

    initialize();
}

void PDEFormationController::initialize() {
    initializeRobotStates();
    initializePublishersSubscribers();

    if (!checkStabilityConditions()) {
        ROS_WARN("PDE stability conditions not satisfied! Adjusting parameters...");
    }

    circle_traj_.start_time = ros::Time::now().toSec();

    ROS_INFO("=== PDE Formation Controller Initialized ===");
    ROS_INFO("PDE Order: %d", pde_params_.m);
    ROS_INFO("Number of robots: %d", pde_params_.N);
    ROS_INFO("Circle radius: %.2f m", circle_traj_.radius);
    ROS_INFO("Angular frequency: %.2f rad/s", circle_traj_.angular_freq);
    ROS_INFO("Boundary gains: k0=%.2f, ka=%.2f", pde_params_.k0, pde_params_.ka);
    ROS_INFO("============================================");
}

void PDEFormationController::initializeRobotStates() {
    // 🔧 修复：清除并重新初始化所有容器
    robot_states_.clear();
    desired_states_.clear();
    alpha_values_.clear();
    
    robot_states_.resize(pde_params_.N);
    desired_states_.resize(pde_params_.N);
    alpha_values_.resize(pde_params_.N);

    // 🔧 修复：安全的alpha值计算，避免除零
    for (int i = 0; i < pde_params_.N; i++) {
        if (pde_params_.N > 1) {
            alpha_values_[i] = static_cast<double>(i) / static_cast<double>(pde_params_.N);
        } else {
            alpha_values_[i] = 0.0;
        }

        // 🔧 修复：确保derivatives向量正确初始化
        desired_states_[i].derivatives_x.clear();
        desired_states_[i].derivatives_y.clear();
        desired_states_[i].derivatives_x.resize(pde_params_.m + 1, 0.0);
        desired_states_[i].derivatives_y.resize(pde_params_.m + 1, 0.0);
    }

    ROS_INFO("Robot alpha values (均匀分布):");
    for (int i = 0; i < pde_params_.N; i++) {
        double angle_deg = alpha_values_[i] * 360.0;
        ROS_INFO("  Robot %d (limo%d): alpha = %.3f (角度: %.1f°)", 
                 i+1, i+1, alpha_values_[i], angle_deg);
    }
}

void PDEFormationController::initializePublishersSubscribers() {
    cmd_vel_pubs_.clear();
    odom_subs_.clear();
    
    cmd_vel_pubs_.resize(pde_params_.N);
    odom_subs_.resize(pde_params_.N);

    // 使用绝对话题路径（带命名空间）
    std::vector<std::string> robot_namespaces = {
        "limo1", "limo2", "limo3", "limo4"
    };

    ROS_INFO("Setting up topic connections for %d robots", pde_params_.N);

    for (int i = 0; i < pde_params_.N && i < robot_namespaces.size(); i++) {
        // 构造带命名空间的完整话题名
        std::string cmd_vel_topic = "/" + robot_namespaces[i] + "/cmd_vel";
        std::string odom_topic = "/" + robot_namespaces[i] + "/odom";

        // 初始化发布器（使用全局节点句柄）
        cmd_vel_pubs_[i] = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic, 10);

        // 初始化订阅器（使用boost::bind传递robot_id）
        odom_subs_[i] = nh_.subscribe<nav_msgs::Odometry>(
            odom_topic, 
            10, 
            boost::bind(&PDEFormationController::odomCallback, this, _1, i)
        );

        ROS_INFO("Robot %d configured: cmd_vel=%s, odom=%s", 
                i+1, cmd_vel_topic.c_str(), odom_topic.c_str());

        // 短暂延迟避免瞬时高负载
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }

    // 验证连接状态
    ros::Time start = ros::Time::now();
    while ((ros::Time::now() - start).toSec() < 5.0) {
        int ready_count = 0;
        for (int i = 0; i < pde_params_.N; i++) {
            if (cmd_vel_pubs_[i].getNumSubscribers() > 0) {
                ready_count++;
            }
        }
        
        if (ready_count == pde_params_.N) {
            ROS_INFO("All %d robots connected successfully!", pde_params_.N);
            break;
        }
        
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
}
void PDEFormationController::odomCallback(const nav_msgs::Odometry::ConstPtr& msg, int robot_id) {
    // 🔧 修复：添加更严格的边界检查
    if (!msg) {
        ROS_ERROR("Received null odometry message for robot %d", robot_id);
        return;
    }
    
    if (robot_id < 0 || robot_id >= static_cast<int>(robot_states_.size()) || robot_id >= pde_params_.N) {
        ROS_ERROR("Invalid robot_id: %d (valid range: 0-%d)", robot_id, pde_params_.N-1);
        return;
    }

    // 添加调试输出
    ROS_INFO_ONCE("Successfully received odometry data for robot %d (limo%d)", robot_id+1, robot_id+1);
    ROS_DEBUG("Robot %d position: x=%.3f, y=%.3f", robot_id+1, 
              msg->pose.pose.position.x, msg->pose.pose.position.y);

    auto& state = robot_states_[robot_id];

    // 🔧 修复：添加数值有效性检查
    if (std::isnan(msg->pose.pose.position.x) || std::isnan(msg->pose.pose.position.y) ||
        std::isinf(msg->pose.pose.position.x) || std::isinf(msg->pose.pose.position.y)) {
        ROS_WARN("Invalid position data for robot %d: x=%.3f, y=%.3f", 
                 robot_id+1, msg->pose.pose.position.x, msg->pose.pose.position.y);
        return;
    }

    state.x = msg->pose.pose.position.x;
    state.y = msg->pose.pose.position.y;

    // 🔧 修复：安全的四元数处理
    try {
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        if (!std::isnan(yaw) && !std::isinf(yaw)) {
            state.theta = yaw;
        }
    } catch (const std::exception& e) {
        ROS_WARN("Error processing orientation for robot %d: %s", robot_id+1, e.what());
    }

    state.vx = msg->twist.twist.linear.x;
    state.vy = msg->twist.twist.linear.y;
    state.vtheta = msg->twist.twist.angular.z;

    state.last_update = ros::Time::now();

    if (!state.initialized) {
        ROS_INFO("✅ Robot %d (limo%d) initialized at position (%.2f, %.2f)", 
                 robot_id+1, robot_id+1, state.x, state.y);
        state.initialized = true;
    }
}

PDEFormationController::DesiredState PDEFormationController::computeDesiredTrajectory(double alpha, double t) {
    DesiredState desired;

    // 🔧 修复：添加输入参数有效性检查
    if (std::isnan(alpha) || std::isinf(alpha) || std::isnan(t) || std::isinf(t)) {
        ROS_WARN("Invalid trajectory parameters: alpha=%.3f, t=%.3f", alpha, t);
        return desired;  // 返回零初始化的状态
    }

    // 修正的圆形运动公式
    double phase = 2.0 * M_PI * alpha;
    double time_phase = circle_traj_.angular_freq * (t - circle_traj_.start_time);
    double total_phase = phase + time_phase;

    // 标准圆形运动位置
    desired.x = circle_traj_.center_x + circle_traj_.radius * cos(total_phase);
    desired.y = circle_traj_.center_y + circle_traj_.radius * sin(total_phase);

    // 速度（位置对时间的导数）
    desired.vx = -circle_traj_.radius * circle_traj_.angular_freq * sin(total_phase);
    desired.vy = circle_traj_.radius * circle_traj_.angular_freq * cos(total_phase);

    // 🔧 修复：确保derivatives向量有正确的大小
    if (desired.derivatives_x.size() != static_cast<size_t>(pde_params_.m + 1)) {
        desired.derivatives_x.resize(pde_params_.m + 1, 0.0);
    }
    if (desired.derivatives_y.size() != static_cast<size_t>(pde_params_.m + 1)) {
        desired.derivatives_y.resize(pde_params_.m + 1, 0.0);
    }

    // 计算高阶导数用于PDE
    for (int order = 0; order <= pde_params_.m; order++) {
        if (order == 0) {
            desired.derivatives_x[order] = desired.x;
            desired.derivatives_y[order] = desired.y;
        } else {
            double coeff = pow(circle_traj_.angular_freq, order) * circle_traj_.radius;
            if (order % 4 == 0) {
                desired.derivatives_x[order] = coeff * cos(total_phase);
                desired.derivatives_y[order] = coeff * sin(total_phase);
            } else if (order % 4 == 1) {
                desired.derivatives_x[order] = -coeff * sin(total_phase);
                desired.derivatives_y[order] = coeff * cos(total_phase);
            } else if (order % 4 == 2) {
                desired.derivatives_x[order] = -coeff * cos(total_phase);
                desired.derivatives_y[order] = -coeff * sin(total_phase);
            } else {
                desired.derivatives_x[order] = coeff * sin(total_phase);
                desired.derivatives_y[order] = -coeff * cos(total_phase);
            }
        }
    }

    return desired;
}

// 改进的边界控制（适用于Limo差分驱动）
geometry_msgs::Twist PDEFormationController::computeBoundaryControlMultiRobot(int robot_id, double t) {
    geometry_msgs::Twist cmd;

    // 🔧 修复：添加边界检查
    if (robot_id < 0 || robot_id >= static_cast<int>(robot_states_.size()) || 
        robot_id >= static_cast<int>(alpha_values_.size())) {
        ROS_ERROR("Invalid robot_id for boundary control: %d", robot_id);
        return cmd;
    }

    if (!robot_states_[robot_id].initialized) {
        return cmd;
    }

    // 计算期望状态
    double alpha = alpha_values_[robot_id];
    DesiredState desired = computeDesiredTrajectory(alpha, t);

    // 当前状态
    double current_x = robot_states_[robot_id].x;
    double current_y = robot_states_[robot_id].y;
    double current_theta = robot_states_[robot_id].theta;

    // 位置误差
    double error_x = desired.x - current_x;
    double error_y = desired.y - current_y;
    double distance_error = sqrt(error_x*error_x + error_y*error_y);

    // Limo差分驱动控制策略
    const double POSITION_TOLERANCE = 0.08;
    const double ANGLE_TOLERANCE = 0.15;

    if (distance_error < POSITION_TOLERANCE) {
        // 已接近目标，执行圆形轨迹跟踪
        double tangent_theta = atan2(desired.vy, desired.vx);
        double angle_error = wrapAngle(tangent_theta - current_theta);

        // 圆形运动：恒定前进速度 + 角度修正
        cmd.linear.x = 0.15;
        cmd.angular.z = circle_traj_.angular_freq + 1.5 * angle_error;
    } else {
        // 向目标点移动
        double desired_theta = atan2(error_y, error_x);
        double angle_error = wrapAngle(desired_theta - current_theta);

        if (fabs(angle_error) > ANGLE_TOLERANCE) {
            // 大角度偏差：原地转向
            cmd.linear.x = 0.0;
            cmd.angular.z = (angle_error > 0) ? 0.4 : -0.4;
        } else {
            // 小角度偏差：前进并微调
            cmd.linear.x = std::min(0.6 * distance_error, 0.2);
            cmd.angular.z = std::max(-0.4, std::min(0.4, 2.0 * angle_error));
        }
    }

    // 🔧 修复：添加数值有效性检查
    if (std::isnan(cmd.linear.x) || std::isinf(cmd.linear.x)) cmd.linear.x = 0.0;
    if (std::isnan(cmd.angular.z) || std::isinf(cmd.angular.z)) cmd.angular.z = 0.0;

    // 速度限制（符合Limo性能）
    limitVelocity(cmd, 0.25, 0.5);

    return cmd;
}

// 改进的跟随者控制
geometry_msgs::Twist PDEFormationController::computeFollowerControlMultiRobot(int robot_id, double t) {
    geometry_msgs::Twist cmd;

    // 🔧 修复：添加边界检查
    if (robot_id < 0 || robot_id >= static_cast<int>(robot_states_.size())) {
        ROS_ERROR("Invalid robot_id for follower control: %d", robot_id);
        return cmd;
    }

    if (!robot_states_[robot_id].initialized) {
        return cmd;
    }

    // 邻居检查
    int left_neighbor = (robot_id - 1 + pde_params_.N) % pde_params_.N;
    int right_neighbor = (robot_id + 1) % pde_params_.N;

    // 🔧 修复：确保邻居索引有效
    if (left_neighbor < 0 || left_neighbor >= static_cast<int>(robot_states_.size()) ||
        right_neighbor < 0 || right_neighbor >= static_cast<int>(robot_states_.size())) {
        ROS_WARN("Invalid neighbor indices for robot %d", robot_id);
        return cmd;
    }

    if (!robot_states_[left_neighbor].initialized || !robot_states_[right_neighbor].initialized) {
        ROS_WARN_THROTTLE(2.0, "Robot %d: neighbors not ready", robot_id + 1);
        return cmd;
    }

    // 计算期望状态（基于理想轨迹）
    double alpha = alpha_values_[robot_id];
    DesiredState desired = computeDesiredTrajectory(alpha, t);

    // 邻居平均位置（PDE邻域影响）
    double neighbor_avg_x = (robot_states_[left_neighbor].x + robot_states_[right_neighbor].x) / 2.0;
    double neighbor_avg_y = (robot_states_[left_neighbor].y + robot_states_[right_neighbor].y) / 2.0;

    // 当前状态
    double current_x = robot_states_[robot_id].x;
    double current_y = robot_states_[robot_id].y;
    double current_theta = robot_states_[robot_id].theta;

    // 组合控制：理想轨迹 + 邻居影响
    double target_x = 0.7 * desired.x + 0.3 * neighbor_avg_x;
    double target_y = 0.7 * desired.y + 0.3 * neighbor_avg_y;

    double error_x = target_x - current_x;
    double error_y = target_y - current_y;
    double distance_error = sqrt(error_x*error_x + error_y*error_y);

    // 差分驱动控制
    double desired_theta = atan2(error_y, error_x);
    double angle_error = wrapAngle(desired_theta - current_theta);

    const double ANGLE_THRESHOLD = 0.2;

    if (fabs(angle_error) > ANGLE_THRESHOLD) {
        cmd.linear.x = 0.0;
        cmd.angular.z = (angle_error > 0) ? 0.3 : -0.3;
    } else {
        cmd.linear.x = std::min(0.5 * distance_error, 0.18);
        cmd.angular.z = std::max(-0.3, std::min(0.3, 1.5 * angle_error));
    }

    // 🔧 修复：添加数值有效性检查
    if (std::isnan(cmd.linear.x) || std::isinf(cmd.linear.x)) cmd.linear.x = 0.0;
    if (std::isnan(cmd.angular.z) || std::isinf(cmd.angular.z)) cmd.angular.z = 0.0;

    // 速度限制
    limitVelocity(cmd, 0.2, 0.4);

    return cmd;
}

// 保留原有的PDE计算函数（添加安全检查）
geometry_msgs::Twist PDEFormationController::computeBoundaryControl(int robot_id, double t) {
    geometry_msgs::Twist cmd;

    if (robot_id != 0) return cmd;

    double linear_speed = 0.2;
    double angular_speed = 0.3;

    cmd.linear.x = linear_speed;
    cmd.angular.z = angular_speed;

    return cmd;
}

geometry_msgs::Twist PDEFormationController::computeFollowerControl(int robot_id, double t) {
    geometry_msgs::Twist cmd;

    if (robot_id <= 0 || robot_id >= pde_params_.N - 1) {
        return cmd;
    }

    if (robot_id >= static_cast<int>(robot_states_.size()) ||
        !robot_states_[robot_id-1].initialized || 
        !robot_states_[robot_id+1].initialized) {
        return cmd;
    }

    cmd.linear.x = computePDEDiscretization(robot_id, true);
    cmd.linear.y = computePDEDiscretization(robot_id, false);

    return cmd;
}

double PDEFormationController::computePDEDiscretization(int robot_id, bool is_x_axis) {
    // 🔧 修复：添加边界检查
    if (robot_id < 0 || robot_id >= static_cast<int>(robot_states_.size())) {
        ROS_ERROR("Invalid robot_id for PDE discretization: %d", robot_id);
        return 0.0;
    }

    double result = 0.0;

    const std::vector<double>& coeffs = is_x_axis ? pde_params_.a_coeffs : pde_params_.b_coeffs;

    // 🔧 修复：确保系数向量有效
    if (coeffs.empty() || coeffs.size() <= static_cast<size_t>(pde_params_.m)) {
        ROS_WARN("Invalid PDE coefficients");
        return 0.0;
    }

    std::vector<double> positions(pde_params_.N);
    for (int i = 0; i < pde_params_.N && i < static_cast<int>(robot_states_.size()); i++) {
        positions[i] = is_x_axis ? robot_states_[i].x : robot_states_[i].y;
    }

    // 零阶项
    if (!coeffs.empty()) {
        result += coeffs[0] * positions[robot_id];
    }

    // 一阶项（中心差分）
    if (robot_id > 0 && robot_id < pde_params_.N - 1 && coeffs.size() > 1) {
        if (pde_params_.h != 0.0) {  // 避免除零
            result += coeffs[1] * (positions[robot_id + 1] - positions[robot_id - 1]) / (2 * pde_params_.h);
        }
    }

    // 高阶项
    for (int s = 2; s <= pde_params_.m && s < static_cast<int>(coeffs.size()); s++) {
        if (robot_id >= s/2 && robot_id < pde_params_.N - s/2) {
            std::vector<double> local_positions;
            for (int i = robot_id - s/2; i <= robot_id + s/2 && i < static_cast<int>(positions.size()); i++) {
                if (i >= 0) {
                    local_positions.push_back(positions[i]);
                }
            }

            if (local_positions.size() >= static_cast<size_t>(s + 1)) {
                double derivative_approx = computeFiniteDifference(local_positions, s, pde_params_.h);
                result += coeffs[s] * derivative_approx;
            }
        }
    }

    // 🔧 修复：检查结果有效性
    if (std::isnan(result) || std::isinf(result)) {
        result = 0.0;
    }

    return result;
}

double PDEFormationController::computeFiniteDifference(const std::vector<double>& values, int order, double h) {
    // 🔧 修复：添加输入有效性检查
    if (values.empty() || order < 0 || h == 0.0) {
        return 0.0;
    }
    
    if (values.size() < static_cast<size_t>(order + 1)) {
        return 0.0;
    }

    double result = 0.0;
    double h_power = pow(h, order);

    // 🔧 修复：检查h_power有效性
    if (h_power == 0.0 || std::isnan(h_power) || std::isinf(h_power)) {
        return 0.0;
    }

    for (int k = 0; k <= order && k < static_cast<int>(values.size()); k++) {
        double binomial_coeff = 1.0;
        for (int i = 0; i < k; i++) {
            binomial_coeff *= (order - i) / (i + 1.0);
        }

        double sign = (k % 2 == 0) ? 1.0 : -1.0;
        result += sign * binomial_coeff * values[k];
    }

    result /= h_power;
    
    // 🔧 修复：检查最终结果
    if (std::isnan(result) || std::isinf(result)) {
        result = 0.0;
    }

    return result;
}

bool PDEFormationController::checkStabilityConditions() {
    // 🔧 修复：添加向量有效性检查
    if (pde_params_.a_coeffs.empty() || 
        pde_params_.m < 0 || 
        static_cast<size_t>(pde_params_.m) >= pde_params_.a_coeffs.size()) {
        return false;
    }

    bool stable = true;

    if (pde_params_.m % 2 == 0) {
        double condition = pow(-1, pde_params_.m/2) * pde_params_.a_coeffs[pde_params_.m];
        if (condition > 0) {
            ROS_WARN("Stability condition violated: %f > 0", condition);
            stable = false;
        }
    } else {
        double condition = pow(-1, (pde_params_.m-1)/2) * pde_params_.a_coeffs[pde_params_.m];
        if (condition < 0) {
            ROS_WARN("Stability condition violated: %f < 0", condition);
            stable = false;
        }
    }

    return stable;
}

void PDEFormationController::printFormationStatus() {
    static int status_count = 0;
    status_count++;

    ROS_INFO("=== Formation Status #%d ===", status_count);

    double current_time = ros::Time::now().toSec();
    double elapsed_time = current_time - circle_traj_.start_time;
    ROS_INFO("Time: %.1fs", elapsed_time);

    double total_error = 0.0;
    int ready_count = 0;

    for (int i = 0; i < pde_params_.N && i < static_cast<int>(robot_states_.size()) && i < static_cast<int>(alpha_values_.size()); i++) {
        if (robot_states_[i].initialized) {
            double alpha = alpha_values_[i];
            DesiredState desired = computeDesiredTrajectory(alpha, current_time);

            double error_x = desired.x - robot_states_[i].x;
            double error_y = desired.y - robot_states_[i].y;
            double error = sqrt(error_x*error_x + error_y*error_y);

            total_error += error;
            ready_count++;

            std::string role = (i == 0 || i == pde_params_.N-1) ? "Leader" : "Follower";
            double angle_deg = alpha * 360.0;

            ROS_INFO("Robot %d (%s, α=%.2f, %.0f°): pos(%.2f,%.2f) → target(%.2f,%.2f) error=%.3f", 
                     i+1, role.c_str(), alpha, angle_deg,
                     robot_states_[i].x, robot_states_[i].y, 
                     desired.x, desired.y, error);
        } else {
            ROS_INFO("Robot %d: Not ready", i+1);
        }
    }

    if (ready_count > 0) {
        double avg_error = total_error / ready_count;
        ROS_INFO("Formation Quality: Average error = %.3fm", avg_error);

        if (avg_error < 0.05) {
            ROS_INFO("*** EXCELLENT FORMATION ***");
        } else if (avg_error < 0.15) {
            ROS_INFO("*** GOOD FORMATION ***");
        } else if (avg_error < 0.3) {
            ROS_INFO("*** ACCEPTABLE FORMATION ***");
        } else {
            ROS_INFO("*** POOR FORMATION - ADJUSTING ***");
        }
    }

    ROS_INFO("=====================================");
}

void PDEFormationController::controlLoop() {
    ros::Rate rate(10);

    ROS_INFO("Starting 4-robot PDE formation control...");

    // 详细的连接检查
    ROS_INFO("Performing detailed connection check...");
    for (int i = 0; i < 10; i++) {
        ros::spinOnce();

        int ready_count = 0;
        for (int j = 0; j < pde_params_.N && j < static_cast<int>(robot_states_.size()); j++) {
            if (robot_states_[j].initialized) {
                ready_count++;
            }
        }

        ROS_INFO("Connection check %d/10: %d/%d robots ready", 
                 i+1, ready_count, pde_params_.N);

        if (ready_count == pde_params_.N) {
            ROS_INFO("All robots connected successfully!");
            break;
        }

        ros::Duration(1.0).sleep();
    }

    // 🔧 修复：添加try-catch保护控制循环
    while (ros::ok()) {
        try {
            double current_time = ros::Time::now().toSec();

            if (allRobotsReady()) {
                static bool first_success = true;
                if (first_success) {
                    ROS_INFO("🎉 All 4 robots ready! Starting PDE formation control...");
                    first_success = false;
                }

                // 🔧 修复：确保循环边界安全
                int max_robots = std::min(pde_params_.N, static_cast<int>(cmd_vel_pubs_.size()));
                for (int i = 0; i < max_robots; i++) {
                    geometry_msgs::Twist cmd;

                    // 使用改进的多机器人控制函数
                    if (i == 0 || i == pde_params_.N - 1) {
                        cmd = computeBoundaryControlMultiRobot(i, current_time);
                    } else {
                        cmd = computeFollowerControlMultiRobot(i, current_time);
                    }

                    // 🔧 修复：检查发布器有效性
                    if (i < static_cast<int>(cmd_vel_pubs_.size())) {
                        cmd_vel_pubs_[i].publish(cmd);
                    }
                }

                static int count = 0;
                if (++count % 50 == 0) {
                    printFormationStatus();
                }
            } else {
                // 发送停止命令
                geometry_msgs::Twist stop_cmd;
                int max_robots = std::min(pde_params_.N, static_cast<int>(cmd_vel_pubs_.size()));
                for (int i = 0; i < max_robots; i++) {
                    cmd_vel_pubs_[i].publish(stop_cmd);
                }

                static int warn_count = 0;
                if (++warn_count % 20 == 0) {
                    ROS_WARN("Robots not ready:");
                    int max_check = std::min(pde_params_.N, static_cast<int>(robot_states_.size()));
                    for (int i = 0; i < max_check; i++) {
                        if (!robot_states_[i].initialized) {
                            ROS_WARN("  Robot %d (limo%d): No odometry data received", i+1, i+1);
                        } else {
                            double time_diff = (ros::Time::now() - robot_states_[i].last_update).toSec();
                            if (time_diff > 0.5) {
                                ROS_WARN("  Robot %d (limo%d): Data too old (%.1fs ago)", i+1, i+1, time_diff);
                            }
                        }
                    }
                }
            }

            ros::spinOnce();
            rate.sleep();
        } catch (const std::exception& e) {
            ROS_ERROR("Error in control loop: %s", e.what());
            // 发送停止命令
            geometry_msgs::Twist stop_cmd;
            for (size_t i = 0; i < cmd_vel_pubs_.size(); i++) {
                cmd_vel_pubs_[i].publish(stop_cmd);
            }
            ros::Duration(0.1).sleep();
        }
    }
}

void PDEFormationController::printSystemStatus() {
    static int status_count = 0;
    status_count++;

    std::cout << "\n=== Formation Status #" << status_count << " ===" << std::endl;
    std::cout << std::fixed << std::setprecision(2);

    double current_time = ros::Time::now().toSec() - circle_traj_.start_time;
    std::cout << "Time: " << current_time << "s" << std::endl;

    int max_robots = std::min(pde_params_.N, static_cast<int>(robot_states_.size()));
    for (int i = 0; i < max_robots; i++) {
        if (robot_states_[i].initialized && i < static_cast<int>(alpha_values_.size())) {
            DesiredState desired = computeDesiredTrajectory(alpha_values_[i], ros::Time::now().toSec());

            double error_x = desired.x - robot_states_[i].x;
            double error_y = desired.y - robot_states_[i].y;
            double error_dist = sqrt(error_x*error_x + error_y*error_y);

            std::string role = (i == 0 || i == pde_params_.N-1) ? "Leader" : "Follower";

            std::cout << "Robot " << i+1 << " (" << role << "): "
                      << "Pos(" << robot_states_[i].x << ", " << robot_states_[i].y << ") "
                      << "Desired(" << desired.x << ", " << desired.y << ") "
                      << "Error: " << error_dist << "m" << std::endl;
        } else {
            std::cout << "Robot " << i+1 << ": Not ready" << std::endl;
        }
    }

    if (allRobotsReady()) {
        double total_error = 0.0;
        int ready_count = 0;

        for (int i = 0; i < max_robots; i++) {
            if (i < static_cast<int>(alpha_values_.size())) {
                DesiredState desired = computeDesiredTrajectory(alpha_values_[i], ros::Time::now().toSec());
                double error_x = desired.x - robot_states_[i].x;
                double error_y = desired.y - robot_states_[i].y;
                total_error += sqrt(error_x*error_x + error_y*error_y);
                ready_count++;
            }
        }

        if (ready_count > 0) {
            double avg_error = total_error / ready_count;
            std::cout << "Formation Quality: Average error = " << avg_error << "m" << std::endl;

            if (avg_error < 0.1) {
                std::cout << "*** EXCELLENT FORMATION ***" << std::endl;
            } else if (avg_error < 0.3) {
                std::cout << "*** GOOD FORMATION ***" << std::endl;
            } else if (avg_error < 0.5) {
                std::cout << "*** ACCEPTABLE FORMATION ***" << std::endl;
            } else {
                std::cout << "*** POOR FORMATION - CHECK PARAMETERS ***" << std::endl;
            }
        }
    }

    std::cout << "==============================" << std::endl;
}

bool PDEFormationController::allRobotsReady() {
    int max_robots = std::min(pde_params_.N, static_cast<int>(robot_states_.size()));
    for (int i = 0; i < max_robots; i++) {
        if (!robot_states_[i].initialized) {
            return false;
        }

        double time_diff = (ros::Time::now() - robot_states_[i].last_update).toSec();
        if (time_diff > 0.5) {
            return false;
        }
    }
    return true;
}

void PDEFormationController::limitVelocity(geometry_msgs::Twist& cmd, double max_linear, double max_angular) {
    // 🔧 修复：添加输入有效性检查
    if (std::isnan(cmd.linear.x) || std::isinf(cmd.linear.x)) cmd.linear.x = 0.0;
    if (std::isnan(cmd.linear.y) || std::isinf(cmd.linear.y)) cmd.linear.y = 0.0;
    if (std::isnan(cmd.angular.z) || std::isinf(cmd.angular.z)) cmd.angular.z = 0.0;

    double linear_magnitude = sqrt(cmd.linear.x * cmd.linear.x + cmd.linear.y * cmd.linear.y);
    if (linear_magnitude > max_linear && linear_magnitude > 0.0) {
        double scale = max_linear / linear_magnitude;
        cmd.linear.x *= scale;
        cmd.linear.y *= scale;
    }

    cmd.angular.z = std::max(-max_angular, std::min(max_angular, cmd.angular.z));
}

double PDEFormationController::wrapAngle(double angle) {
    // 🔧 修复：添加输入有效性检查
    if (std::isnan(angle) || std::isinf(angle)) {
        return 0.0;
    }
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pde_formation_controller");

    try {
        PDEFormationController controller;
        controller.controlLoop();
    } catch (const std::exception& e) {
        ROS_ERROR("PDE Formation Controller error: %s", e.what());
        return -1;
    } catch (...) {
        ROS_ERROR("Unknown error in PDE Formation Controller");
        return -1;
    }

    return 0;
}
