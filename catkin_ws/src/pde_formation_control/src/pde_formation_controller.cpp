#include "pde_formation_control/pde_formation_controller.h"
#include <iostream>
#include <iomanip>
#include <algorithm>  // 添加这个，用于std::min, std::max
PDEFormationController::PDEFormationController() : nh_(), pnh_("~") {
    // 从参数服务器读取参数
    pnh_.param("circle_radius", circle_traj_.radius, 0.8);
    pnh_.param("angular_frequency", circle_traj_.angular_freq, 0.3);
    pnh_.param("center_x", circle_traj_.center_x, 0.0);
    pnh_.param("center_y", circle_traj_.center_y, 0.0);
    
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
    robot_states_.resize(pde_params_.N);
    desired_states_.resize(pde_params_.N);
    alpha_values_.resize(pde_params_.N);
    
    // 修正alpha值计算
    for (int i = 0; i < pde_params_.N; i++) {
        if (pde_params_.N == 1) {
            alpha_values_[i] = 0.0;  // 单机器人情况
        } else {
            alpha_values_[i] = static_cast<double>(i) / (pde_params_.N - 1);
        }
        
        // 初始化期望状态的导数存储
        desired_states_[i].derivatives_x.resize(pde_params_.m + 1);
        desired_states_[i].derivatives_y.resize(pde_params_.m + 1);
    }
    
    ROS_INFO("Robot alpha values:");
    for (int i = 0; i < pde_params_.N; i++) {
        ROS_INFO("  Robot %d (limo%d): alpha = %.3f", i+1, i+1, alpha_values_[i]);
    }
}
// 修改initializePublishersSubscribers函数
void PDEFormationController::initializePublishersSubscribers() {
    cmd_vel_pubs_.resize(pde_params_.N);
    odom_subs_.resize(pde_params_.N);
    
    // 强制使用统一的话题命名，不再自动检测
    std::vector<std::string> cmd_topics = {
        "/limo1/cmd_vel", "/limo2/cmd_vel", "/limo3/cmd_vel", "/limo4/cmd_vel"
    };
    std::vector<std::string> odom_topics = {
        "/limo1/odom", "/limo2/odom", "/limo3/odom", "/limo4/odom"
    };
    std::vector<std::string> robot_names = {"limo1", "limo2", "limo3", "limo4"};
    
    ROS_INFO("Using unified topic structure for all 4 robots");
    
    for (int i = 0; i < pde_params_.N; i++) {
        cmd_vel_pubs_[i] = nh_.advertise<geometry_msgs::Twist>(cmd_topics[i], 10);
        odom_subs_[i] = nh_.subscribe<nav_msgs::Odometry>(
            odom_topics[i], 10, 
            boost::bind(&PDEFormationController::odomCallback, this, _1, i)
        );
        
        ROS_INFO("Setup robot %d (%s): cmd_vel->%s, odom->%s", 
                 i+1, robot_names[i].c_str(), cmd_topics[i].c_str(), odom_topics[i].c_str());
        
        // 等待一下让连接建立
        ros::Duration(0.2).sleep();
        ros::spinOnce();
    }
    
    // 等待所有连接建立
    ROS_INFO("Waiting for all robot connections to establish...");
    ros::Duration(3.0).sleep();
    
    // 检查连接状态
    for (int i = 0; i < pde_params_.N; i++) {
        int num_subscribers = cmd_vel_pubs_[i].getNumSubscribers();
        ROS_INFO("Robot %d cmd_vel has %d subscribers", i+1, num_subscribers);
    }
}
void PDEFormationController::odomCallback(const nav_msgs::Odometry::ConstPtr& msg, int robot_id) {
    if (robot_id < 0 || robot_id >= pde_params_.N) {
        ROS_ERROR("Invalid robot_id: %d", robot_id);
        return;
    }
    
    // 添加调试信息
    ROS_INFO_ONCE("Received odometry data for robot %d (limo%d)", robot_id+1, robot_id+1);
    
    auto& state = robot_states_[robot_id];
    
    state.x = msg->pose.pose.position.x;
    state.y = msg->pose.pose.position.y;
    
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    state.theta = yaw;
    
    state.vx = msg->twist.twist.linear.x;
    state.vy = msg->twist.twist.linear.y;
    state.vtheta = msg->twist.twist.angular.z;
    
    state.last_update = ros::Time::now();
    
    if (!state.initialized) {
        ROS_INFO("Robot %d (limo%d) initialized at position (%.2f, %.2f)", 
                 robot_id+1, robot_id+1, state.x, state.y);
        state.initialized = true;
    }
    
    // 修正的调试信息部分
    static std::vector<int> callback_count;
    if (callback_count.size() != pde_params_.N) {
        callback_count.resize(pde_params_.N, 0);
    }
    
    if (robot_id >= 0 && robot_id < callback_count.size()) {
        callback_count[robot_id]++;
        if (callback_count[robot_id] % 50 == 0) {
            ROS_INFO("Robot %d callback count: %d, pos: (%.2f, %.2f)", 
                     robot_id+1, callback_count[robot_id], state.x, state.y);
        }
    }
}

PDEFormationController::DesiredState PDEFormationController::computeDesiredTrajectory(double alpha, double t) {
    DesiredState desired;
    
    double phase = 2.0 * M_PI * alpha;
    double time_phase = circle_traj_.angular_freq * (t - circle_traj_.start_time);
    
    // 位置
    desired.x = circle_traj_.center_x + circle_traj_.radius * sin(phase + time_phase);
    desired.y = circle_traj_.center_y + circle_traj_.radius * cos(phase + time_phase);
    
    // 速度
    desired.vx = circle_traj_.radius * circle_traj_.angular_freq * cos(phase + time_phase);
    desired.vy = -circle_traj_.radius * circle_traj_.angular_freq * sin(phase + time_phase);
    
    // 计算高阶导数
    for (int order = 0; order <= pde_params_.m; order++) {
        if (order == 0) {
            desired.derivatives_x[order] = desired.x;
            desired.derivatives_y[order] = desired.y;
        } else {
            double coeff = pow(2 * M_PI, order) * circle_traj_.radius;
            if (order % 4 == 0) {
                desired.derivatives_x[order] = coeff * sin(phase + time_phase);
                desired.derivatives_y[order] = coeff * cos(phase + time_phase);
            } else if (order % 4 == 1) {
                desired.derivatives_x[order] = coeff * cos(phase + time_phase);
                desired.derivatives_y[order] = -coeff * sin(phase + time_phase);
            } else if (order % 4 == 2) {
                desired.derivatives_x[order] = -coeff * sin(phase + time_phase);
                desired.derivatives_y[order] = -coeff * cos(phase + time_phase);
            } else {
                desired.derivatives_x[order] = -coeff * cos(phase + time_phase);
                desired.derivatives_y[order] = coeff * sin(phase + time_phase);
            }
        }
    }
    
    return desired;
}

geometry_msgs::Twist PDEFormationController::computeBoundaryControl(int robot_id, double t) {
    geometry_msgs::Twist cmd;
    
    if (robot_id != 0) return cmd;
    
    // 简单的圆形运动：恒定前进速度 + 恒定角速度
    double linear_speed = 0.2;    // 前进速度 m/s
    double angular_speed = 0.3;   // 角速度 rad/s
    
    cmd.linear.x = linear_speed;
    cmd.angular.z = angular_speed;
    
    // 当前位置
    double current_x = robot_states_[robot_id].x;
    double current_y = robot_states_[robot_id].y;
    
    ROS_INFO_THROTTLE(1.0, "Simple circle: pos(%.2f,%.2f) cmd(lin=%.2f, ang=%.2f)", 
                     current_x, current_y, cmd.linear.x, cmd.angular.z);
    
    return cmd;
}

// 新增多机器人边界控制函数
geometry_msgs::Twist PDEFormationController::computeBoundaryControlMultiRobot(int robot_id, double t) {
    geometry_msgs::Twist cmd;
    
    // 计算该机器人在圆形编队中的期望位置
    double alpha = alpha_values_[robot_id];
    double time_elapsed = t - circle_traj_.start_time;
    double phase = 2.0 * M_PI * alpha + circle_traj_.angular_freq * time_elapsed;
    
    double desired_x = circle_traj_.center_x + circle_traj_.radius * cos(phase);
    double desired_y = circle_traj_.center_y + circle_traj_.radius * sin(phase);
    
    // 当前位置
    double current_x = robot_states_[robot_id].x;
    double current_y = robot_states_[robot_id].y;
    double current_theta = robot_states_[robot_id].theta;
    
    // 位置误差
    double error_x = desired_x - current_x;
    double error_y = desired_y - current_y;
    double distance_error = sqrt(error_x*error_x + error_y*error_y);
    
    // 计算期望朝向
    double desired_theta = atan2(error_y, error_x);
    double angle_error = desired_theta - current_theta;
    while (angle_error > M_PI) angle_error -= 2*M_PI;
    while (angle_error < -M_PI) angle_error += 2*M_PI;
    
    // 差分驱动控制
    if (fabs(angle_error) > 0.2) {
        cmd.linear.x = 0.0;
        cmd.angular.z = (angle_error > 0) ? 0.5 : -0.5;
    } else {
        cmd.linear.x = std::min(0.8 * distance_error, 0.25);
        cmd.angular.z = std::max(-0.5, std::min(0.5, 1.5 * angle_error));
    }
    
    return cmd;
}
geometry_msgs::Twist PDEFormationController::computeFollowerControl(int robot_id, double t) {
    geometry_msgs::Twist cmd;
    
    if (robot_id <= 0 || robot_id >= pde_params_.N - 1) {
        return cmd;
    }
    
    if (!robot_states_[robot_id-1].initialized || !robot_states_[robot_id+1].initialized) {
        return cmd;
    }
    
    cmd.linear.x = computePDEDiscretization(robot_id, true);
    cmd.linear.y = computePDEDiscretization(robot_id, false);
    
    return cmd;
}
// 新增跟随者控制函数
geometry_msgs::Twist PDEFormationController::computeFollowerControlMultiRobot(int robot_id, double t) {
    geometry_msgs::Twist cmd;
    
    // 检查邻居是否可用
    if (!robot_states_[robot_id-1].initialized || !robot_states_[robot_id+1].initialized) {
        return cmd;
    }
    
    // 简化的PDE跟随控制：平均邻居位置
    double neighbor_avg_x = (robot_states_[robot_id-1].x + robot_states_[robot_id+1].x) / 2.0;
    double neighbor_avg_y = (robot_states_[robot_id-1].y + robot_states_[robot_id+1].y) / 2.0;
    
    double current_x = robot_states_[robot_id].x;
    double current_y = robot_states_[robot_id].y;
    double current_theta = robot_states_[robot_id].theta;
    
    // 跟随邻居的平均位置
    double error_x = neighbor_avg_x - current_x;
    double error_y = neighbor_avg_y - current_y;
    double distance_error = sqrt(error_x*error_x + error_y*error_y);
    
    double desired_theta = atan2(error_y, error_x);
    double angle_error = desired_theta - current_theta;
    while (angle_error > M_PI) angle_error -= 2*M_PI;
    while (angle_error < -M_PI) angle_error += 2*M_PI;
    
    // 控制律
    if (fabs(angle_error) > 0.2) {
        cmd.linear.x = 0.0;
        cmd.angular.z = (angle_error > 0) ? 0.3 : -0.3;
    } else {
        cmd.linear.x = std::min(0.6 * distance_error, 0.2);
        cmd.angular.z = std::max(-0.3, std::min(0.3, 1.0 * angle_error));
    }
    
    return cmd;
}


double PDEFormationController::computePDEDiscretization(int robot_id, bool is_x_axis) {
    double result = 0.0;
    
    const std::vector<double>& coeffs = is_x_axis ? pde_params_.a_coeffs : pde_params_.b_coeffs;
    
    std::vector<double> positions(pde_params_.N);
    for (int i = 0; i < pde_params_.N; i++) {
        positions[i] = is_x_axis ? robot_states_[i].x : robot_states_[i].y;
    }
    
    // 零阶项
    result += coeffs[0] * positions[robot_id];
    
    // 一阶项（中心差分）
    if (robot_id > 0 && robot_id < pde_params_.N - 1) {
        result += coeffs[1] * (positions[robot_id + 1] - positions[robot_id - 1]) / (2 * pde_params_.h);
    }
    
    // 高阶项
    for (int s = 2; s <= pde_params_.m; s++) {
        if (robot_id >= s/2 && robot_id < pde_params_.N - s/2) {
            std::vector<double> local_positions;
            for (int i = robot_id - s/2; i <= robot_id + s/2; i++) {
                local_positions.push_back(positions[i]);
            }
            
            if (local_positions.size() >= s + 1) {
                double derivative_approx = computeFiniteDifference(local_positions, s, pde_params_.h);
                result += coeffs[s] * derivative_approx;
            }
        }
    }
    
    return result;
}

double PDEFormationController::computeFiniteDifference(const std::vector<double>& values, int order, double h) {
    if (values.size() < order + 1) return 0.0;
    
    double result = 0.0;
    double h_power = pow(h, order);
    
    for (int k = 0; k <= order; k++) {
        double binomial_coeff = 1.0;
        for (int i = 0; i < k; i++) {
            binomial_coeff *= (order - i) / (i + 1.0);
        }
        
        double sign = (k % 2 == 0) ? 1.0 : -1.0;
        
        if (k < values.size()) {
            result += sign * binomial_coeff * values[k];
        }
    }
    
    return result / h_power;
}

bool PDEFormationController::checkStabilityConditions() {
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
    ROS_INFO("=== Formation Status ===");
    for (int i = 0; i < pde_params_.N; i++) {
        if (robot_states_[i].initialized) {
            // 计算期望位置
            double alpha = alpha_values_[i];
            double time_elapsed = ros::Time::now().toSec() - circle_traj_.start_time;
            double phase = 2.0 * M_PI * alpha + circle_traj_.angular_freq * time_elapsed;
            
            double desired_x = circle_traj_.center_x + circle_traj_.radius * cos(phase);
            double desired_y = circle_traj_.center_y + circle_traj_.radius * sin(phase);
            
            double error_x = desired_x - robot_states_[i].x;
            double error_y = desired_y - robot_states_[i].y;
            double error = sqrt(error_x*error_x + error_y*error_y);
            
            std::string role = (i == 0 || i == pde_params_.N-1) ? "Leader" : "Follower";
            ROS_INFO("Robot %d (%s): pos(%.2f,%.2f) target(%.2f,%.2f) error=%.2f", 
                     i+1, role.c_str(), robot_states_[i].x, robot_states_[i].y, 
                     desired_x, desired_y, error);
        } else {
            ROS_INFO("Robot %d: Not ready", i+1);
        }
    }
}

void PDEFormationController::controlLoop() {
    ros::Rate rate(10);
    
    ROS_INFO("Starting 4-robot PDE formation control...");
    
    // 详细的连接检查
    ROS_INFO("Performing detailed connection check...");
    for (int i = 0; i < 10; i++) {  // 检查10次
        ros::spinOnce();
        
        int ready_count = 0;
        for (int j = 0; j < pde_params_.N; j++) {
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
    
    while (ros::ok()) {
        double current_time = ros::Time::now().toSec();
        
        if (allRobotsReady()) {
            static bool first_success = true;
            if (first_success) {
                ROS_INFO("All 4 robots ready! Starting PDE formation control...");
                first_success = false;
            }
            
            for (int i = 0; i < pde_params_.N; i++) {
                geometry_msgs::Twist cmd;
                
                if (i == 0 || i == pde_params_.N - 1) {
                    cmd = computeBoundaryControlMultiRobot(i, current_time);
                } else {
                    cmd = computeFollowerControlMultiRobot(i, current_time);
                }
                
                cmd_vel_pubs_[i].publish(cmd);
            }
            
            static int count = 0;
            if (++count % 50 == 0) {
                printFormationStatus();
            }
        } else {
            // 发送停止命令并详细报告哪些机器人没准备好
            geometry_msgs::Twist stop_cmd;
            for (int i = 0; i < pde_params_.N; i++) {
                cmd_vel_pubs_[i].publish(stop_cmd);
            }
            
            static int warn_count = 0;
            if (++warn_count % 20 == 0) {  // 每2秒详细报告一次
                ROS_WARN("Robots not ready:");
                for (int i = 0; i < pde_params_.N; i++) {
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
    }
}
void PDEFormationController::printSystemStatus() {
    static int status_count = 0;
    status_count++;
    
    std::cout << "\n=== Formation Status #" << status_count << " ===" << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    
    // 打印当前时间
    double current_time = ros::Time::now().toSec() - circle_traj_.start_time;
    std::cout << "Time: " << current_time << "s" << std::endl;
    
    // 打印每个机器人的状态
    for (int i = 0; i < pde_params_.N; i++) {
        if (robot_states_[i].initialized) {
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
    
    // 计算编队质量指标
    if (allRobotsReady()) {
        double total_error = 0.0;
        int ready_count = 0;
        
        for (int i = 0; i < pde_params_.N; i++) {
            DesiredState desired = computeDesiredTrajectory(alpha_values_[i], ros::Time::now().toSec());
            double error_x = desired.x - robot_states_[i].x;
            double error_y = desired.y - robot_states_[i].y;
            total_error += sqrt(error_x*error_x + error_y*error_y);
            ready_count++;
        }
        
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
    
    std::cout << "==============================" << std::endl;
}

bool PDEFormationController::allRobotsReady() {
    for (int i = 0; i < pde_params_.N; i++) {
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
    double linear_magnitude = sqrt(cmd.linear.x * cmd.linear.x + cmd.linear.y * cmd.linear.y);
    if (linear_magnitude > max_linear) {
        double scale = max_linear / linear_magnitude;
        cmd.linear.x *= scale;
        cmd.linear.y *= scale;
    }
    
    cmd.angular.z = std::max(-max_angular, std::min(max_angular, cmd.angular.z));
}

double PDEFormationController::wrapAngle(double angle) {
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
    }
    
    return 0;
}