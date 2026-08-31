#ifndef SIMPLE_ROBOT_TRAJECTORY_HPP
#define SIMPLE_ROBOT_TRAJECTORY_HPP
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

class RobotTrajectory : public rclcpp::Node {
    public : 
        RobotTrajectory(const std::string &name);
    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

        void odomCallback(const nav_msgs::msg::Odometry &msg);
        nav_msgs::msg::Path path_;
};

#endif