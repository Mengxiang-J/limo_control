/**
 * @file limo_pde_circle_controller.cpp
 * @brief 基于PDE模型的Limo机器人圆周轨迹运动控制ROS节点
 *
 * 参考论文："Formation Control of Multiagent System Based on Higher Order Partial Differential Equations"
 * 作者：Kaiyo Yamaguchi, Takahiro Endo, Fumitoshi Matsuno
 *
 * 单车应用说明：
 * 单车可视为一多智能体系统中的一员，其 α（空间参数）随时间变化模拟在目标圆形轨迹上均匀移动，
 * 机器人控制自身跟踪圆上的目标点即为"PDE目标平衡解"。
 * 本代码为单车版本，若要多车协作（真正的PDE分布式控制），需将所有car的α分配在[0,1]进行同步更新。
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <cmath>

class LimoPDECircleController {
private:
    ros::NodeHandle nh_;                   // ROS节点句柄
    ros::Subscriber odom_sub_;             // 里程计订阅器
    ros::Publisher cmd_vel_pub_;           // 速度指令发布器
    ros::Timer timer_;                     // 控制循环定时器

    // 圆轨迹参数
    double circle_radius_;                 // 圆半径，单位m，建议设为0.6
    double linear_speed_;                  // 期望“圆周线速度”，例如0.2m/s
    double alpha_;                         // 空间参数[0,1]，决定圆上的目标点
    double update_rate_;                   // 控制频率（Hz）

    // 机器人状态
    double x_, y_, theta_;                 // 坐标x,y，航向theta（弧度）

    // 圆心(可自定义在地图上其他位置)
    double center_x_, center_y_;

public:
    LimoPDECircleController() : nh_("~")
     {
        // 1. 初始化参数
        nh_.param("circle_radius", circle_radius_, 0.6);      // 圆半径0.6m
        nh_.param("linear_speed", linear_speed_, 0.2);        // 线速度0.2m/s
        nh_.param("update_rate", update_rate_, 10.0);         // 控制频率10Hz
        
        alpha_ = 0.0;
        x_ = 0.0; y_ = 0.0; theta_ = 0.0;
        center_x_ = 0.0; center_y_ = 0.0;
        // 2. 订阅odom，发布速度
        odom_sub_ = nh_.subscribe("/odom", 1, &LimoPDECircleController::odomCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        // 3. 启动控制定时器
        timer_ = nh_.createTimer(ros::Duration(1.0/update_rate_), &LimoPDECircleController::controlLoop, this);

        ROS_INFO("Limo PDE圆形控制器已初始化，圆半径: %.2f m", circle_radius_);
    }

    ~LimoPDECircleController() {
        // 析构函数，停止机器人
        geometry_msgs::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(stop_cmd);
        ROS_INFO("Limo PDE Circle Controller 关闭");
    }

    // 里程计回调，获取机器人当前位姿
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, theta_);
    }

    // 控制环函数
    void controlLoop(const ros::TimerEvent& event) {
        // 若odom无数据则跳过
        if (odom_sub_.getNumPublishers() == 0) {
            ROS_WARN_THROTTLE(1, "未接收到里程计数据");
            return;
        }

        // 1. 根据线速度递推alpha，实现目标点在圆周上均匀移动
        double d_alpha = linear_speed_ / (2 * M_PI * circle_radius_) * (1.0 / update_rate_);
        alpha_ += d_alpha;
        if (alpha_ >= 1.0) alpha_ -= 1.0;

        // 2. 目标点为圆上的平衡解，即 PDE equilibrium curve:
        double target_x = center_x_ + circle_radius_ * cos(2 * M_PI * alpha_);
        double target_y = center_y_ + circle_radius_ * sin(2 * M_PI * alpha_);

        // 3. 与目标点误差
        double error_x = target_x - x_;
        double error_y = target_y - y_;

        // 目标朝向为指向目标点的切线方向
        double target_theta = atan2(target_y - y_, target_x - x_);
        double error_theta = atan2(sin(target_theta - theta_), cos(target_theta - theta_));

        // 4. 控制器设计（等效于 PDE模型分布控制时的反馈调节）
        double Kp_linear = 1.5;                // 线速度增益
        double Kp_angular = 2.0;               // 角速度增益
        double dist_error = sqrt(error_x*error_x + error_y*error_y);

        double linear_vel = Kp_linear * dist_error;
        double angular_vel = Kp_angular * error_theta;

        // 5. 限幅与安全制动
        linear_vel = std::min(linear_vel, linear_speed_);
        if (dist_error < 0.05) linear_vel = 0;                         // 足够接近则只调朝向
        angular_vel = std::max(std::min(angular_vel, 1.5), -1.5);

        // 6. 发布速度指令
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = linear_vel;
        cmd_vel.angular.z = angular_vel;
        cmd_vel_pub_.publish(cmd_vel);

        // 7. 打印调试信息
        ROS_INFO_THROTTLE(1, "alpha=%.2f, 机器人(%.2f,%.2f) 到目标(%.2f,%.2f), dist=%.2f, v=%.2f, w=%.2f",
                          alpha_, x_, y_, target_x, target_y, dist_error, linear_vel, angular_vel);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "limo_pde_circle_controller");
    LimoPDECircleController controller;
    ros::spin();
    return 0;
}
