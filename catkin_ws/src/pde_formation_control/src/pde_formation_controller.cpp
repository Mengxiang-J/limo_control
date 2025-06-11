#include "pde_formation_control/pde_formation_controller.h"
#include <iostream>
#include <iomanip>

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
    
    for (int i = 0; i < pde_params_.N; i++) {
        alpha_values_[i] = static_cast<double>(i) / (pde_params_.N - 1);
        desired_states_[i].derivatives_x.resize(pde_params_.m + 1);
        desired_states_[i].derivatives_y.resize(pde_params_.m + 1);
    }
    
    ROS_INFO("Robot alpha values:");
    for (int i = 0; i < pde_params_.N; i++) {
        ROS_INFO("  Robot %d (limo%d): alpha = %.3f", i+1, i+1, alpha_values_[i]);
    }
}

void PDEFormationController::initializePublishersSubscribers() {
    std::vector<std::string> robot_names = {"limo1", "limo2", "limo3", "limo4"};
    
    cmd_vel_pubs_.resize(pde_params_.N);
    odom_subs_.resize(pde_params_.N);
    
    for (int i = 0; i < pde_params_.N; i++) {
        std::string cmd_topic = "/" + robot_names[i] + "/cmd_vel";
        cmd_vel_pubs_[i] = nh_.advertise<geometry_msgs::Twist>(cmd_topic, 10);
        
        std::string odom_topic = "/" + robot_names[i] + "/odom";
        odom_subs_[i] = nh_.subscribe<nav_msgs::Odometry>(
            odom_topic, 10, 
            boost::bind(&PDEFormationController::odomCallback, this, _1, i)
        );
        
        ROS_INFO("Setup robot %d: cmd_vel->%s, odom->%s", 
                 i+1, cmd_topic.c_str(), odom_topic.c_str());
    }
}

void PDEFormationController::odomCallback(const nav_msgs::Odometry::ConstPtr& msg, int robot_id) {
    if (robot_id < 0 || robot_id >= pde_params_.N) return;
    
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
    
    if (robot_id != 0 && robot_id != pde_params_.N - 1) {
        return cmd;
    }
    
    DesiredState desired = computeDesiredTrajectory(alpha_values_[robot_id], t);
    desired_states_[robot_id] = desired;
    
    double error_x = desired.x - robot_states_[robot_id].x;
    double error_y = desired.y - robot_states_[robot_id].y;
    
    // 边界控制律实现
    if (pde_params_.m >= 3) {
        double boundary_term_x = 0.0;
        double boundary_term_y = 0.0;
        
        for (int j = 3; j <= pde_params_.m; j++) {
            if (j-2 < desired.derivatives_x.size()) {
                boundary_term_x += pde_params_.a_coeffs[j] * desired.derivatives_x[j-2];
                boundary_term_y += pde_params_.b_coeffs[j] * desired.derivatives_y[j-2];
            }
        }
        
        double k_term_x = 0.0;
        double k_term_y = 0.0;
        for (int j = 2; j <= pde_params_.m; j++) {
            k_term_x += pde_params_.a_coeffs[j] * error_x;
            k_term_y += pde_params_.b_coeffs[j] * error_y;
        }
        
        if (robot_id == 0) {
            cmd.linear.x = desired.vx + pde_params_.k0 * 
                          (pde_params_.L * boundary_term_x - pde_params_.K * k_term_x);
            cmd.linear.y = desired.vy + pde_params_.k0 * 
                          (pde_params_.L * boundary_term_y - pde_params_.K * k_term_y);
        } else {
            cmd.linear.x = desired.vx + pde_params_.ka * 
                          ((pde_params_.K + pde_params_.L) * k_term_x - pde_params_.L * boundary_term_x);
            cmd.linear.y = desired.vy + pde_params_.ka * 
                          ((pde_params_.K + pde_params_.L) * k_term_y - pde_params_.L * boundary_term_y);
        }
    } else {
        cmd.linear.x = desired.vx + pde_params_.k0 * pde_params_.K * pde_params_.a_coeffs[2] * error_x;
        cmd.linear.y = desired.vy + pde_params_.ka * (pde_params_.K + pde_params_.L) * pde_params_.a_coeffs[2] * error_y;
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

void PDEFormationController::controlLoop() {
    ros::Rate rate(50);
    int loop_count = 0;
    double start_time = ros::Time::now().toSec();
    
    ROS_INFO("Starting PDE formation control loop at 50Hz...");
    
    while (ros::ok()) {
        double current_time = ros::Time::now().toSec();
        
        if (allRobotsReady()) {
            for (int i = 0; i < pde_params_.N; i++) {
                geometry_msgs::Twist cmd;
                
                if (i == 0 || i == pde_params_.N - 1) {
                    cmd = computeBoundaryControl(i, current_time);
                } else {
                    cmd = computeFollowerControl(i, current_time);
                }
                
                limitVelocity(cmd, 0.5, 1.0);
                cmd_vel_pubs_[i].publish(cmd);
            }
            
            // 每隔5秒打印一次状态
            if (loop_count % 250 == 0) {
                printSystemStatus();
            }
        } else {
            geometry_msgs::Twist stop_cmd;
            for (int i = 0; i < pde_params_.N; i++) {
                cmd_vel_pubs_[i].publish(stop_cmd);
            }
            
            if (loop_count % 100 == 0) {  // 每2秒打印一次
                ROS_WARN("Waiting for all robots to be ready...");
                for (int i = 0; i < pde_params_.N; i++) {
                    if (!robot_states_[i].initialized) {
                        ROS_WARN("  Robot %d (limo%d): not initialized", i+1, i+1);
                    } else {
                        double time_diff = (ros::Time::now() - robot_states_[i].last_update).toSec();
                        if (time_diff > 0.5) {
                            ROS_WARN("  Robot %d (limo%d): data too old (%.1fs)", i+1, i+1, time_diff);
                        }
                    }
                }
            }
        }
        
        loop_count++;
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