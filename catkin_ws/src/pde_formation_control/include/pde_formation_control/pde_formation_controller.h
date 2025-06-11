#ifndef PDE_FORMATION_CONTROLLER_H
#define PDE_FORMATION_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <cmath>
#include <string>

class PDEFormationController {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    // 发布器和订阅器
    std::vector<ros::Publisher> cmd_vel_pubs_;
    std::vector<ros::Subscriber> odom_subs_;
    
    // 机器人状态结构
    struct RobotState {
        double x, y, theta;
        double vx, vy, vtheta;
        bool initialized;
        ros::Time last_update;
        
        RobotState() : x(0), y(0), theta(0), vx(0), vy(0), vtheta(0), 
                      initialized(false) {}
    };
    
 // 修改PDEParameters
// PDE模型参数（基于论文）
struct PDEParameters {
    std::vector<double> a_coeffs;  // x轴PDE系数
    std::vector<double> b_coeffs;  // y轴PDE系数
    int m;  // PDE阶数
    double k0, ka;  // 边界控制增益
    double K, L;    // Lyapunov函数参数
    double p;       // 稳定性参数
    double h;       // 空间步长
    int N;          // 智能体数量
    
    PDEParameters() {
        m = 3;  // 保持3阶
        N = 4;  // 恢复4台机器人
        h = 1.0 / (N - 1);
        
        // PDE系数
        a_coeffs.resize(m + 1);
        b_coeffs.resize(m + 1);
        
        a_coeffs[0] = 0.0;
        a_coeffs[1] = 0.0;
        a_coeffs[2] = 1.0;
        a_coeffs[3] = -0.1;
        
        b_coeffs = a_coeffs;
        
        k0 = -0.5;
        ka = -0.5;
        K = 1.0;
        L = 1.0;
        p = 0.5;
    }
} pde_params_;
    // 圆形轨迹参数
    struct CircleTrajectory {
        double center_x, center_y;
        double radius;
        double angular_freq;
        double start_time;
        
        CircleTrajectory() : center_x(0.0), center_y(0.0), radius(0.8), 
                           angular_freq(0.3), start_time(0.0) {}
    } circle_traj_;
    
    std::vector<RobotState> robot_states_;
    std::vector<double> alpha_values_;
    
    // 期望轨迹结构
    struct DesiredState {
        double x, y;
        double vx, vy;
        std::vector<double> derivatives_x;
        std::vector<double> derivatives_y;
    };
    std::vector<DesiredState> desired_states_;
    
public:
    PDEFormationController();
    ~PDEFormationController() = default;
    
    void initialize();
    void controlLoop();
    
private:
    void initializePublishersSubscribers();
    void initializeRobotStates();
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, int robot_id);
    
    DesiredState computeDesiredTrajectory(double alpha, double t);
    geometry_msgs::Twist computeBoundaryControl(int robot_id, double t);
    geometry_msgs::Twist computeFollowerControl(int robot_id, double t);
    double computePDEDiscretization(int robot_id, bool is_x_axis);
    double computeFiniteDifference(const std::vector<double>& values, int order, double h);
    
    bool checkStabilityConditions();
    bool allRobotsReady();
    void limitVelocity(geometry_msgs::Twist& cmd, double max_linear, double max_angular);
    void printSystemStatus();  // 用于控制台输出状态
    double wrapAngle(double angle);
     // 新增的多机器人控制函数
    geometry_msgs::Twist computeBoundaryControlMultiRobot(int robot_id, double t);
    geometry_msgs::Twist computeFollowerControlMultiRobot(int robot_id, double t);
    void printFormationStatus();  // 也需要添加这个
};

#endif