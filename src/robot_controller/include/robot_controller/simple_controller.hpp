#ifndef SIMPLE_CONTROLLER_HPP
#define SIMPLE_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "Eigen/Core"
#include <tf2_ros/transform_broadcaster.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>


class SimpleController : public rclcpp::Node{
    public : 
        SimpleController(const std::string &name);
    private:
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_pub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

        void velCallback(const geometry_msgs::msg::TwistStamped &msg);
        void jointCallback(const sensor_msgs::msg::JointState &msg);
        
        double wheel_radius_;
        double wheel_separation_;
        Eigen::Matrix2d speed_conversion_;

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