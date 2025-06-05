#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <string>

class BasicController {
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;
    
    double current_x_;
    double current_y_;
    double current_yaw_;
    
    std::string robot_name_;

public:
    BasicController(const std::string& robot_name) : robot_name_(robot_name) {
        // 发布速度命令
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(robot_name + "/cmd_vel", 10);
        
        // 订阅里程计信息
        odom_sub_ = nh_.subscribe(robot_name + "/odom", 10, 
                                &BasicController::odomCallback, this);
        
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_yaw_ = 0.0;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // 获取当前位置
        current_x_ = msg->pose.pose.position.x;  
        current_y_ = msg->pose.pose.position.y;
        
        // 获取当前朝向
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw_);
        
        // 打印当前位置信息进行调试
        ROS_INFO("%s current pose: x=%.2f, y=%.2f, yaw=%.2f", 
                 robot_name_.c_str(), current_x_, current_y_, current_yaw_);
    }

    void moveForward() {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.1;  // 0.1 m/s向前
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd_vel);
    }

    void stop() {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd_vel);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "basic_controller");
    
    if (argc != 2) {
        ROS_ERROR("Usage: %s <robot_name>", argv[0]);
        return 1;
    }
    
    std::string robot_name = argv[1];
    BasicController controller(robot_name);
    
    ros::Rate rate(10);  // 10Hz
    bool moving = true;
    
    while(ros::ok()) {
        // 简单测试:每5秒切换一次运动状态
        ros::Time now = ros::Time::now();
        if ((now.sec % 10) < 5) {
            if (moving) {
                ROS_INFO("Moving forward...");
                controller.moveForward();
            }
        } else {
            if (moving) {
                ROS_INFO("Stopping...");
                controller.stop();
            }
        }
        
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}