#ifndef NOISY_CONTROLLER_HPP
#define NOISY_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_ros/transform_broadcaster.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

class NoisyController : public rclcpp::Node{
    public : 
        NoisyController(const std::string &name);
    private:
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

        void jointCallback(const sensor_msgs::msg::JointState &msg);
        
        double wheel_radius_;
        double wheel_separation_;

        // wheel linear and angular velocity calculation variables
        double left_wheel_prev_pos_;
        double right_wheel_prev_pos_;
        rclcpp::Time prev_time_;

        // robot position and orientation
        double x_;
        double y_;
        double theta_;

        // odom publisher variables
        nav_msgs::msg::Odometry odom_msg_;


        // transform broadcaster from odom to base_footprint
        std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
        geometry_msgs::msg::TransformStamped transform_stamped_;
};

#endif