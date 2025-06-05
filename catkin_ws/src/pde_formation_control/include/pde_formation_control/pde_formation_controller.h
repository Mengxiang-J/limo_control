/**
 * @file pde_formation_controller.h
 * @brief PDE-based formation controller for multi-agent systems
 * 
 * This controller implements the higher-order PDE-based formation control
 * method described in "Formation Control of Multiagent System Based on 
 * Higher Order Partial Differential Equations" by Yamaguchi et al.
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
        LEADER_ALPHA_0,  // Leader at α = 0
        LEADER_ALPHA_1,  // Leader at α = 1
        FOLLOWER         // Follower agents
    };

    /**
     * @brief Constructor for PDEFormationController
     * 
     * @param nh ROS node handle
     * @param robot_id ID of this robot
     * @param role Role of this agent (leader or follower)
     * @param alpha Spatial parameter for this agent (0 <= alpha <= 1)
     */
    PDEFormationController(
        ros::NodeHandle& nh, 
        int robot_id,
        AgentRole role,
        double alpha
    );

    /**
     * @brief Initialize the controller
     */
    void initialize();

    /**
     * @brief Main control loop
     */
    void controlLoop(const ros::TimerEvent& event);

    /**
     * @brief Update the desired formation
     * 
     * @param center_x X-coordinate of the formation center
     * @param center_y Y-coordinate of the formation center
     * @param radius Radius of the circular formation
     */
    void updateDesiredFormation(double center_x, double center_y, double radius);

private:
    // ROS-related members
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;
    std::vector<ros::Subscriber> neighbor_subs_;
    ros::Publisher pose_pub_;
    ros::Timer control_timer_;

    // Agent parameters
    int robot_id_;
    AgentRole role_;
    double alpha_;  // Spatial parameter (0 <= alpha <= 1)
    
    // PDE model parameters
    int pde_order_;              // Order of the PDE model (m)
    std::vector<double> a_coef_; // Coefficients for x-axis PDE
    std::vector<double> b_coef_; // Coefficients for y-axis PDE
    
    // Controller parameters
    double control_rate_;
    double K_, L_;            // Lyapunov function parameters
    double k0_, ka_;          // Feedback gains for leaders
    
    // Formation parameters
    double formation_center_x_;
    double formation_center_y_;
    double formation_radius_;
    
    // Current state
    double x_, y_, theta_;
    
    // Neighbor positions
    std::map<int, std::pair<double, double>> neighbor_positions_;
    
    // Desired values for boundary conditions
    double u0_bar_;     // Desired value for x(0,t)
    double ua_bar_;     // Desired value for x(1,t)
    std::vector<double> u0j_; // Desired values for x^(j)(0,t)
    std::vector<double> ual_; // Desired values for x^(l)(1,t)
    
    double v0_bar_;     // Desired value for y(0,t)
    double va_bar_;     // Desired value for y(1,t)
    std::vector<double> v0j_; // Desired values for y^(j)(0,t)
    std::vector<double> val_; // Desired values for y^(l)(1,t)
    
    /**
     * @brief Callback for odometry data
     */
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    
    /**
     * @brief Callback for neighbor position data
     */
    void neighborCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int neighbor_id);
    
    /**
     * @brief Compute the control input for a leader agent
     */
    geometry_msgs::Twist computeLeaderControl();
    
    /**
     * @brief Compute the control input for a follower agent
     */
    geometry_msgs::Twist computeFollowerControl();
    
    /**
     * @brief Calculate the spatial derivative of order 's' at position 'alpha'
     * 
     * @param s Order of the derivative
     * @param is_x True for x-axis, false for y-axis
     * @return Value of the spatial derivative
     */
    double calculateSpatialDerivative(int s, bool is_x);
    
    /**
     * @brief Calculate the desired position based on alpha
     * 
     * @param alpha Spatial parameter
     * @param is_x True for x-coordinate, false for y-coordinate
     * @return Desired position
     */
    double desiredPosition(double alpha, bool is_x);
    
    /**
     * @brief Calculate the desired spatial derivative
     * 
     * @param alpha Spatial parameter
     * @param order Derivative order
     * @param is_x True for x-axis, false for y-axis
     * @return Desired spatial derivative
     */
    double desiredSpatialDerivative(double alpha, int order, bool is_x);
};

#endif // PDE_FORMATION_CONTROLLER_H